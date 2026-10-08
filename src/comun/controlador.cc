#include "controlador.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/diagram_builder.h"

#include "modelo_panda.h"
#include "referencia.h"

namespace ipre2026 {

using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

namespace {

// PD simple para los dedos (joints 8 y 9), fuera del paper
constexpr double kKpDedos = 50.0;
constexpr double kKdDedos = 2.0;

// Saturacion simetrica: ecs. (8) y (11)
double Saturar(double x, double limite) {
    return std::clamp(x, -limite, limite);
}

}  // namespace

ModoControl ModoDesdeTexto(const std::string& texto) {
    if (texto == "vsc") return ModoControl::kVSC;
    if (texto == "fl")  return ModoControl::kFL;
    if (texto == "ff")  return ModoControl::kFF;
    throw std::runtime_error("modo desconocido: " + texto + " (vsc|fl|ff)");
}

std::string TextoDesdeModo(ModoControl modo) {
    switch (modo) {
        case ModoControl::kVSC: return "vsc";
        case ModoControl::kFL:  return "fl";
        case ModoControl::kFF:  return "ff";
    }
    return "?";
}

// =============================================================================
// PARAMETROS DE DISENO (decisiones tuyas, ver Tabla I del paper)
// =============================================================================
ParametrosDiseno ParametrosPorDefecto() {
    ParametrosDiseno p;
    // Amplitud A_i: define e_max = Delta*A y omega_max (ec. 6). La ETSO necesita
    // A_i > 0 en todos los joints, aunque la referencia actual solo mueva el 1.
    p.A = Eigen::VectorXd::Constant(kNumJointsBrazo, kAmplitudRef);
    // Precision Delta_i [% de A_i]. El paper usa 0.1. k crece como 1/Delta.
    p.Delta = Eigen::VectorXd::Constant(kNumJointsBrazo, 1.0);
    p.omega_ref = 2.0 * M_PI * kFrecuenciaRefHz;
    p.q_centro = PoseHome();
    return p;
}

// =============================================================================
// ControladorETSO
// =============================================================================
ControladorETSO::ControladorETSO(std::unique_ptr<MultibodyPlant<double>> plant,
                                 ModoControl modo, const ParametrosDiseno& parametros)
    : plant_(std::move(plant)), modo_(modo), par_(parametros) {
    plant_context_ = plant_->CreateDefaultContext();
    n_ = plant_->num_positions();   // 9
    n_etso_ = kNumJointsBrazo;      // 7

    // -------------------------------------------------------------------------
    // PASO 0: diseno de los parametros ETSO (una sola vez)
    // -------------------------------------------------------------------------
    CalcularParametrosETSO();

    estado_idx_  = DeclareVectorInputPort("estimated_state", 2 * n_).get_index();
    deseado_idx_ = DeclareVectorInputPort("desired_state",   2 * n_).get_index();
    aceleracion_idx_ =
        DeclareVectorInputPort("desired_acceleration", n_).get_index();
    control_idx_ = DeclareVectorOutputPort("control", n_,
                                           &ControladorETSO::CalcTorque).get_index();
    tau0_idx_ = DeclareVectorOutputPort("tau0", n_,
                                        &ControladorETSO::CalcTau0).get_index();
    delta_tau_idx_ = DeclareVectorOutputPort("delta_tau", n_,
                                             &ControladorETSO::CalcDeltaTau).get_index();
}

void ControladorETSO::MuestrearRegion(Eigen::VectorXd* j_max,
                                      Eigen::VectorXd* tau_L) const {
    // Region de trabajo: caja alrededor de q_centro, recortada a los limites.
    const Eigen::VectorXd lim_lo = plant_->GetPositionLowerLimits();
    const Eigen::VectorXd lim_hi = plant_->GetPositionUpperLimits();
    Eigen::VectorXd lo = par_.q_centro, hi = par_.q_centro;  // dedos fijos
    Eigen::VectorXd dq_max = Eigen::VectorXd::Zero(n_);
    Eigen::VectorXd ddq_max = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < n_etso_; ++i) {
        const double r = par_.A[i] + par_.margen_q;
        lo[i] = std::max(lim_lo[i], par_.q_centro[i] - r);
        hi[i] = std::min(lim_hi[i], par_.q_centro[i] + r);
        dq_max[i]  = par_.A[i] * par_.omega_ref;                  // fila q̇_max de Tabla I
        ddq_max[i] = par_.A[i] * par_.omega_ref * par_.omega_ref; // fila q̈_max de Tabla I
    }

    std::mt19937 rng(0);  // semilla fija: diseno reproducible
    std::uniform_real_distribution<double> U(0.0, 1.0);

    *j_max = Eigen::VectorXd::Zero(n_);
    *tau_L = Eigen::VectorXd::Zero(n_);
    Eigen::MatrixXd M(n_, n_);
    Eigen::VectorXd Cv(n_), q(n_), dq(n_);

    for (int s = 0; s < par_.muestras; ++s) {
        for (int i = 0; i < n_; ++i) {
            q[i] = lo[i] + U(rng) * (hi[i] - lo[i]);
            // Mitad de las veces en un vertice de la caja de velocidades
            // (donde suele estar el maximo de los terminos cuadraticos).
            const double u = 2.0 * U(rng) - 1.0;
            dq[i] = dq_max[i] * (U(rng) < 0.5 ? (u < 0 ? -1.0 : 1.0) : u);
        }
        plant_->SetPositions(plant_context_.get(), q);
        plant_->SetVelocities(plant_context_.get(), dq);
        plant_->CalcMassMatrix(*plant_context_, &M);
        plant_->CalcBiasTerm(*plant_context_, &Cv);  // c(q,dq)
        const Eigen::VectorXd g =
            -plant_->CalcGravityGeneralizedForces(*plant_context_);  // g(q) del paper

        for (int i = 0; i < n_etso_; ++i) {
            j_max->coeffRef(i) = std::max((*j_max)[i], M(i, i));
            // ec. (4): tau_L = c + g + sum_{j != i} j_ij q̈_j.
            // El termino de acoplamiento se acota analiticamente: sum |j_ij| q̈_max,j
            double acople = 0.0;
            for (int j = 0; j < n_etso_; ++j) {
                if (j != i) acople += std::abs(M(i, j)) * ddq_max[j];
            }
            tau_L->coeffRef(i) =
                std::max((*tau_L)[i], std::abs(Cv[i] + g[i]) + acople);
        }
    }
}

void ControladorETSO::CalcularParametrosETSO() {
    // --- PASO 0a: limite de torque tau_max,i (del URDF) ----------------------
    // Es el limite FISICO del actuador del Panda (87 / 12 Nm). No se aumenta:
    // si se agranda aqui, el diseno deja de ser valido para el robot real.
    tau_max_total_ = plant_->GetEffortUpperLimits();
    tau_max_ = tau_max_total_.head(n_etso_);

    // --- PASO 0b: friccion viscosa B_i (damping del modelo del controlador) ---
    B_.resize(n_etso_);
    for (int i = 0; i < n_etso_; ++i) {
        const std::string nombre = "panda_joint" + std::to_string(i + 1);
        B_[i] = plant_->GetJointByName<RevoluteJoint>(nombre).default_damping();
        if (B_[i] <= 0.0) {
            throw std::runtime_error(
                "[ETSO] " + nombre + " sin damping en el modelo del controlador. "
                "Construye el modelo con AgregarPanda(plant, B).");
        }
    }

    // --- PASO 0c y 0d: j_ii,max y tau_Lmax (ec. 4) en la region de trabajo -----
    // tau_Lmax es la carga FISICA que ve cada joint (gravedad + Coriolis +
    // acoplamiento). Es la misma para VSC, FL y FF (como en la Tabla I): la
    // saturacion (11) actua sobre el torque total, asi que la capacidad de
    // frenado/aceleracion que suponen (6) y (7) depende de tau_max - tau_L real,
    // no de cuanto de esa carga compense tau0. Lo que si cambia con FL/FF es la
    // perturbacion residual que ve la ETSO en la zona lineal -> mejor precision.
    Eigen::VectorXd j_max, tau_L;
    MuestrearRegion(&j_max, &tau_L);
    J_max_ = j_max.head(n_etso_);
    tau_L_max_ = par_.factor_seguridad * tau_L.head(n_etso_);

    // --- PASO 0e: parametros derivados, ecs. (6), (7), (10) --------------------
    T_.resize(n_etso_); K_.resize(n_etso_); w_max_.resize(n_etso_);
    alpha_.resize(n_etso_); k_.resize(n_etso_); e_max_.resize(n_etso_);

    for (int i = 0; i < n_etso_; ++i) {
        T_[i] = J_max_[i] / B_[i];
        K_[i] = 1.0 / B_[i];

        const double dtau = tau_max_[i] - tau_L_max_[i];
        const double stau = tau_max_[i] + tau_L_max_[i];
        if (dtau <= 0.0) {
            std::ostringstream msg;
            msg << "[ETSO] joint " << i + 1 << ": tau_Lmax = " << tau_L_max_[i]
                << " Nm >= tau_max = " << tau_max_[i] << " Nm. La garantia del "
                << "diseno no se cumple: reduce la region de trabajo (A, margen_q), "
                << "las velocidades/aceleraciones (omega_ref) o el factor de seguridad.";
            throw std::runtime_error(msg.str());
        }

        // ec. (6): frecuencia maxima que el joint puede seguir
        w_max_[i] = (-1.0 + std::sqrt(1.0 + 4.0 * K_[i] * dtau * T_[i] / par_.A[i]))
                    / (2.0 * T_[i]);

        // ec. (7): pendiente de la linea de conmutacion
        const double a = dtau / stau;
        alpha_[i] = 1.0 / (T_[i] * (1.0 - a * std::log(1.0 + 1.0 / a)));

        // ec. (10): parametro del algoritmo
        const double w2 = w_max_[i] * w_max_[i];
        const double invT2 = 1.0 / (T_[i] * T_[i]);
        const double a2 = alpha_[i] * alpha_[i];
        k_[i] = 100.0 * tau_max_[i] * w_max_[i] / (par_.Delta[i] * dtau)
                * std::sqrt((w2 + invT2) / (w2 + a2));

        e_max_[i] = par_.Delta[i] / 100.0 * par_.A[i];

        if (par_.omega_ref > w_max_[i]) {
            std::cerr << "[ETSO] AVISO joint " << i + 1 << ": omega_ref = "
                      << par_.omega_ref << " > omega_max = " << w_max_[i]
                      << " rad/s; el joint no puede seguir esa referencia.\n";
        }
    }
}

std::string ControladorETSO::TablaParametrosCsv() const {
    std::ostringstream out;
    out << std::setprecision(10)
        << "joint,tau_max,tau_L_max,B,J_max,T_max,K,A,Delta,w_max,alpha,k,e_max\n";
    for (int i = 0; i < n_etso_; ++i) {
        out << i + 1 << "," << tau_max_[i] << "," << tau_L_max_[i] << "," << B_[i]
            << "," << J_max_[i] << "," << T_[i] << "," << K_[i] << "," << par_.A[i]
            << "," << par_.Delta[i] << "," << w_max_[i] << "," << alpha_[i] << ","
            << k_[i] << "," << e_max_[i] << "\n";
    }
    return out.str();
}

void ControladorETSO::ImprimirParametros() const {
    std::cout << "\n[ETSO] modo " << TextoDesdeModo(modo_)
              << " - parametros de diseno (formato Tabla I)\n"
              << " joint   tau_max  tau_Lmax        B    J_max    T_max"
              << "    w_max    alpha          k    e_max\n";
    std::cout << std::fixed;
    for (int i = 0; i < n_etso_; ++i) {
        std::cout << std::setw(6) << i + 1
                  << std::setprecision(2)
                  << std::setw(10) << tau_max_[i]
                  << std::setw(10) << tau_L_max_[i]
                  << std::setw(9)  << B_[i]
                  << std::setprecision(4)
                  << std::setw(9)  << J_max_[i]
                  << std::setw(9)  << T_[i]
                  << std::setw(9)  << w_max_[i]
                  << std::setw(9)  << alpha_[i]
                  << std::setprecision(1)
                  << std::setw(11) << k_[i]
                  << std::setprecision(5)
                  << std::setw(9)  << e_max_[i] << "\n";
    }
    std::cout << std::defaultfloat << std::endl;
}

void ControladorETSO::Calcular(const Context<double>& context, Eigen::VectorXd* tau,
                               Eigen::VectorXd* tau0, Eigen::VectorXd* delta_tau) const {
    // -------------------------------------------------------------------------
    // PASO 1: leer entradas
    // -------------------------------------------------------------------------
    const Eigen::VectorXd& x  = get_input_port(estado_idx_).Eval(context);   // [q; dq]
    const Eigen::VectorXd& xd = get_input_port(deseado_idx_).Eval(context);  // [q_d; dq_d]

    const Eigen::VectorXd q    = x.head(n_);
    const Eigen::VectorXd dq   = x.tail(n_);
    const Eigen::VectorXd q_d  = xd.head(n_);
    const Eigen::VectorXd dq_d = xd.tail(n_);
    // q̈_d solo lo usa FF. Si el puerto no esta conectado (p.ej. LCM) vale 0.
    const auto& puerto_acc = get_input_port(aceleracion_idx_);
    const Eigen::VectorXd ddq_d = puerto_acc.HasValue(context)
                                      ? Eigen::VectorXd(puerto_acc.Eval(context))
                                      : Eigen::VectorXd::Zero(n_);

    // -------------------------------------------------------------------------
    // PASO 2: errores (seccion III)
    // -------------------------------------------------------------------------
    const Eigen::VectorXd e  = q_d - q;     // e_i  = q_di  - q_i
    const Eigen::VectorXd de = dq_d - dq;   // ė_i  = dq_di - dq_i

    // -------------------------------------------------------------------------
    // PASO 3: torque robusto ETSO, delta_tau, ecs. (9) y (8). No usa el modelo.
    // -------------------------------------------------------------------------
    *delta_tau = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < n_etso_; ++i) {
        // ec. (9)
        const double tau_pd =
            (1.0 / K_[i]) * ((T_[i] * (alpha_[i] + k_[i]) - 1.0) * de[i]
                             + T_[i] * alpha_[i] * k_[i] * e[i]);
        // ec. (8)
        (*delta_tau)[i] = Saturar(tau_pd, tau_max_[i]);
    }

    // -------------------------------------------------------------------------
    // PASO 4: torque del modelo tau0 segun el modo (usa la copia de la planta)
    //   Convencion Drake:  M(q) q̈ + C(q,dq) dq = tau_g(q) + tau
    //   => c(q,dq) del paper = C dq,   g(q) del paper = -tau_g
    // -------------------------------------------------------------------------
    *tau0 = Eigen::VectorXd::Zero(n_);
    switch (modo_) {
        case ModoControl::kVSC: {
            break;  // ETSO puro: tau0 = 0
        }
        case ModoControl::kFL: {
            // ec. (12): tau0 = c(q,dq) + g(q), en el estado MEDIDO
            plant_->SetPositions(plant_context_.get(), q);
            plant_->SetVelocities(plant_context_.get(), dq);
            Eigen::VectorXd Cv(n_);
            plant_->CalcBiasTerm(*plant_context_, &Cv);
            *tau0 = Cv - plant_->CalcGravityGeneralizedForces(*plant_context_);
            break;
        }
        case ModoControl::kFF: {
            // ec. (13): tau0 = B dq_d + c(q_d,dq_d) + g(q_d) + J(q_d) q̈_d, en la REFERENCIA
            plant_->SetPositions(plant_context_.get(), q_d);
            plant_->SetVelocities(plant_context_.get(), dq_d);
            Eigen::MatrixXd M(n_, n_);
            plant_->CalcMassMatrix(*plant_context_, &M);
            Eigen::VectorXd Cv(n_);
            plant_->CalcBiasTerm(*plant_context_, &Cv);
            *tau0 = M * ddq_d + Cv - plant_->CalcGravityGeneralizedForces(*plant_context_);
            tau0->head(n_etso_) += B_.cwiseProduct(dq_d.head(n_etso_));
            break;
        }
    }

    // -------------------------------------------------------------------------
    // PASO 5: superposicion y saturacion, ec. (11); dedos con PD simple
    // -------------------------------------------------------------------------
    *tau = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < n_etso_; ++i) {
        (*tau)[i] = Saturar((*tau0)[i] + (*delta_tau)[i], tau_max_[i]);
    }
    for (int i = n_etso_; i < n_; ++i) {
        (*tau)[i] = Saturar((*tau0)[i] + kKpDedos * e[i] + kKdDedos * de[i],
                            tau_max_total_[i]);
    }
}

void ControladorETSO::CalcTorque(const Context<double>& context,
                                 BasicVector<double>* output) const {
    Eigen::VectorXd tau, tau0, delta_tau;
    Calcular(context, &tau, &tau0, &delta_tau);
    output->SetFromVector(tau);
}

void ControladorETSO::CalcTau0(const Context<double>& context,
                               BasicVector<double>* output) const {
    Eigen::VectorXd tau, tau0, delta_tau;
    Calcular(context, &tau, &tau0, &delta_tau);
    output->SetFromVector(tau0);
}

void ControladorETSO::CalcDeltaTau(const Context<double>& context,
                                   BasicVector<double>* output) const {
    Eigen::VectorXd tau, tau0, delta_tau;
    Calcular(context, &tau, &tau0, &delta_tau);
    output->SetFromVector(delta_tau);
}

// =============================================================================
// Modelo del controlador y envoltorio para el proceso LCM
// =============================================================================
std::unique_ptr<MultibodyPlant<double>> CrearModeloControlador(
    const Eigen::VectorXd& damping) {
    // Solo se usa para calcular M, c, g -> time_step = 0 (continua) basta.
    auto modelo = std::make_unique<MultibodyPlant<double>>(0.0);
    AgregarPanda(modelo.get(), damping);
    // -------------------------------------------------------------------------
    // FASE 2 (incertidumbre): aqui modificar masas/inercias/damping de la copia
    // para que el modelo del controlador deje de coincidir con la planta "real".
    // -------------------------------------------------------------------------
    modelo->Finalize();
    return modelo;
}

ControladorPanda::ControladorPanda(ModoControl modo) {
    DiagramBuilder<double> builder;

    auto etso = builder.AddSystem<ControladorETSO>(
        CrearModeloControlador(DampingPandaPorDefecto()), modo);
    etso->set_name("controlador_etso");
    etso->ImprimirParametros();

    estimated_state_input_port_index_ =
        builder.ExportInput(etso->get_estimated_state_input_port(), "estimated_state");
    desired_state_input_port_index_ =
        builder.ExportInput(etso->get_desired_state_input_port(), "desired_state");
    control_output_port_index_ =
        builder.ExportOutput(etso->get_control_output_port(), "control");

    builder.BuildInto(this);
}

}  // namespace ipre2026
