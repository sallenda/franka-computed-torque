#include "controlador.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/diagram_builder.h"

namespace ipre2026 {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

namespace {

// =============================================================================
// PARAMETROS DE DISENO (decisiones tuyas, ver Tabla I del paper)
// =============================================================================
constexpr int kNumJointsBrazo = 7;

// TODO: amplitud A_i de la referencia por joint. Debe coincidir con trayectoria.cc.
//       Ojo: la ETSO necesita A_i > 0 en todos los joints (define e_max y omega_max).
constexpr double kAmplitudRad = 0.4;

// TODO: precision deseada Delta_i [% de A_i]. El paper usa 0.1; aqui 1.0 para
//       empezar, porque k crece como 1/Delta y con LCM a 5 ms un k grande chatterea.
constexpr double kPrecisionPorc = 1.0;

// TODO: friccion viscosa de respaldo si el URDF no trae damping [Nms].
//       Si se usa, agrega el MISMO damping a la planta en franka_simulation.cc.
constexpr double kBRespaldo = 10.0;

// Muestras para estimar J_ii,max
constexpr int kMuestrasJmax = 5000;

// PD simple para los dedos (joints 8 y 9)
constexpr double kKpDedos = 50.0;
constexpr double kKdDedos = 2.0;

// Saturacion simetrica: ecs. (8) y (11)
double Saturar(double x, double limite) {
    return std::clamp(x, -limite, limite);
}

}  // namespace

// =============================================================================
// ControladorETSO
// =============================================================================
ControladorETSO::ControladorETSO(std::unique_ptr<MultibodyPlant<double>> plant,
                                 ModoControl modo)
    : plant_(std::move(plant)), modo_(modo) {
    plant_context_ = plant_->CreateDefaultContext();
    n_ = plant_->num_positions();   // 9
    n_etso_ = kNumJointsBrazo;      // 7

    // -------------------------------------------------------------------------
    // PASO 0: diseno de los parametros ETSO (una sola vez)
    // -------------------------------------------------------------------------
    CalcularParametrosETSO();
    ImprimirParametros();

    // Puertos (mismos nombres que el controlador anterior)
    estado_idx_  = DeclareVectorInputPort("estimated_state", 2 * n_).get_index();
    deseado_idx_ = DeclareVectorInputPort("desired_state",   2 * n_).get_index();
    // TODO (modo FF): cuando trayectoria.cc publique q̈_d, cambiar a 3 * n_
    control_idx_ = DeclareVectorOutputPort("control", n_,
                                           &ControladorETSO::CalcTorque).get_index();
}

void ControladorETSO::CalcularParametrosETSO() {
    // --- PASO 0a: limite de torque tau_max,i (del URDF) ----------------------
    // Asume que el orden de actuadores coincide con el de los joints (asi es en el Panda).
    tau_max_total_ = plant_->GetEffortUpperLimits();
    tau_max_ = tau_max_total_.head(n_etso_);

    // --- PASO 0b: friccion viscosa B_i (damping del URDF) --------------------
    B_.resize(n_etso_);
    for (int i = 0; i < n_etso_; ++i) {
        const std::string nombre = "panda_joint" + std::to_string(i + 1);
        const auto& joint = plant_->GetJointByName<RevoluteJoint>(nombre);
        B_[i] = joint.default_damping();
        if (B_[i] <= 0.0) {
            std::cerr << "[ETSO] " << nombre << " sin damping en el URDF; usando B = "
                      << kBRespaldo << " Nms (TODO: revisar)" << std::endl;
            B_[i] = kBRespaldo;
        }
    }

    // --- PASO 0c: inercia diagonal maxima j_ii,max -----------------------------
    // TODO: idealmente muestrear solo el rango que recorre TU trayectoria.
    J_max_ = CalcularJmax(kMuestrasJmax).head(n_etso_);

    // --- PASO 0d: cota del torque de carga tau_Lmax,i, ec. (4) ------------------
    // TODO: calcular evaluando c + g + sum_{j!=i} J_ij * q̈_j a lo largo de la
    //       trayectoria (con el modelo peor caso) y agregar margen.
    //       Placeholder: misma proporcion que el paper (tau_Lmax = tau_max / 2).
    tau_L_max_ = 0.5 * tau_max_;

    // --- PASO 0e: referencia y precision (decisiones de diseno) ---------------
    A_     = Eigen::VectorXd::Constant(n_etso_, kAmplitudRad);
    Delta_ = Eigen::VectorXd::Constant(n_etso_, kPrecisionPorc);

    // --- PASO 0f: parametros derivados, ecs. (6), (7), (10) --------------------
    T_.resize(n_etso_); K_.resize(n_etso_); w_max_.resize(n_etso_);
    alpha_.resize(n_etso_); k_.resize(n_etso_);

    for (int i = 0; i < n_etso_; ++i) {
        T_[i] = J_max_[i] / B_[i];
        K_[i] = 1.0 / B_[i];

        const double dtau = tau_max_[i] - tau_L_max_[i];
        const double stau = tau_max_[i] + tau_L_max_[i];
        if (dtau <= 0.0) {
            throw std::runtime_error("[ETSO] tau_max <= tau_Lmax en joint " +
                                     std::to_string(i + 1));
        }

        // ec. (6): frecuencia maxima que el joint puede seguir
        w_max_[i] = (-1.0 + std::sqrt(1.0 + 4.0 * K_[i] * dtau * T_[i] / A_[i]))
                    / (2.0 * T_[i]);

        // ec. (7): pendiente de la linea de conmutacion
        const double a = dtau / stau;
        alpha_[i] = 1.0 / (T_[i] * (1.0 - a * std::log(1.0 + 1.0 / a)));

        // ec. (10): parametro del algoritmo
        const double w2 = w_max_[i] * w_max_[i];
        const double invT2 = 1.0 / (T_[i] * T_[i]);
        const double a2 = alpha_[i] * alpha_[i];
        k_[i] = 100.0 * tau_max_[i] * w_max_[i] / (Delta_[i] * dtau)
                * std::sqrt((w2 + invT2) / (w2 + a2));
    }
}

Eigen::VectorXd ControladorETSO::CalcularJmax(int num_muestras) const {
    // Muestreo aleatorio uniforme dentro de los limites articulares del URDF.
    const Eigen::VectorXd lo = plant_->GetPositionLowerLimits();
    const Eigen::VectorXd hi = plant_->GetPositionUpperLimits();
    Eigen::VectorXd j_max = Eigen::VectorXd::Zero(n_);
    Eigen::MatrixXd M(n_, n_);

    plant_->SetVelocities(plant_context_.get(), Eigen::VectorXd::Zero(n_));
    for (int s = 0; s < num_muestras; ++s) {
        const Eigen::VectorXd r = 0.5 * (Eigen::VectorXd::Random(n_).array() + 1.0);
        const Eigen::VectorXd q = lo + r.cwiseProduct(hi - lo);
        plant_->SetPositions(plant_context_.get(), q);
        plant_->CalcMassMatrix(*plant_context_, &M);
        j_max = j_max.cwiseMax(M.diagonal());
    }
    return j_max;
}

void ControladorETSO::ImprimirParametros() const {
    std::cout << "\n[ETSO] Parametros de diseno (formato Tabla I)\n"
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
                  << std::setw(9)  << Delta_[i] / 100.0 * A_[i] << "\n";
    }
    std::cout << std::endl;
}

void ControladorETSO::CalcTorque(const Context<double>& context,
                                 BasicVector<double>* output) const {
    // -------------------------------------------------------------------------
    // PASO 1: leer entradas
    // -------------------------------------------------------------------------
    const Eigen::VectorXd& x  = get_input_port(estado_idx_).Eval(context);   // [q; dq]
    const Eigen::VectorXd& xd = get_input_port(deseado_idx_).Eval(context);  // [q_d; dq_d]

    const Eigen::VectorXd q    = x.head(n_);
    const Eigen::VectorXd dq   = x.tail(n_);
    const Eigen::VectorXd q_d  = xd.head(n_);
    const Eigen::VectorXd dq_d = xd.tail(n_);
    // TODO (modo FF): leer q̈_d de la entrada cuando sea de tamano 3n.
    const Eigen::VectorXd ddq_d = Eigen::VectorXd::Zero(n_);

    // -------------------------------------------------------------------------
    // PASO 2: errores (seccion III)
    // -------------------------------------------------------------------------
    const Eigen::VectorXd e  = q_d - q;     // e_i  = q_di  - q_i
    const Eigen::VectorXd de = dq_d - dq;   // ė_i  = dq_di - dq_i

    // -------------------------------------------------------------------------
    // PASO 3: torque robusto ETSO, delta_tau, ecs. (9) y (8). No usa el modelo.
    // -------------------------------------------------------------------------
    Eigen::VectorXd delta_tau = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < n_etso_; ++i) {
        // ec. (9)
        const double tau_pd =
            (1.0 / K_[i]) * ((T_[i] * (alpha_[i] + k_[i]) - 1.0) * de[i]
                             + T_[i] * alpha_[i] * k_[i] * e[i]);
        // ec. (8)
        delta_tau[i] = Saturar(tau_pd, tau_max_[i]);
    }

    // -------------------------------------------------------------------------
    // PASO 4: torque del modelo tau0 segun el modo (usa la copia de la planta)
    //   Convencion Drake:  M(q) q̈ + C(q,dq) dq = tau_g(q) + tau
    //   => g(q) del paper = -tau_g
    // -------------------------------------------------------------------------
    Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(n_);
    switch (modo_) {
        case ModoControl::kVSC: {
            // ETSO puro: tau0 = 0
            break;
        }
        case ModoControl::kFL: {
            // ec. (12): tau0 = c(q,dq) + g(q), en el estado MEDIDO
            plant_->SetPositions(plant_context_.get(), q);
            plant_->SetVelocities(plant_context_.get(), dq);
            Eigen::VectorXd Cv(n_);
            plant_->CalcBiasTerm(*plant_context_, &Cv);
            const Eigen::VectorXd tau_g =
                plant_->CalcGravityGeneralizedForces(*plant_context_);
            tau0 = Cv - tau_g;
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
            const Eigen::VectorXd tau_g =
                plant_->CalcGravityGeneralizedForces(*plant_context_);
            tau0 = M * ddq_d + Cv - tau_g;
            tau0.head(n_etso_) += B_.cwiseProduct(dq_d.head(n_etso_));
            break;
        }
    }

    // -------------------------------------------------------------------------
    // PASO 5: superposicion y saturacion, ec. (11)
    // -------------------------------------------------------------------------
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n_);
    for (int i = 0; i < n_etso_; ++i) {
        tau[i] = Saturar(tau0[i] + delta_tau[i], tau_max_[i]);
    }

    // -------------------------------------------------------------------------
    // PASO 6: dedos (fuera del paper): PD simple + tau0 del modo
    // -------------------------------------------------------------------------
    for (int i = n_etso_; i < n_; ++i) {
        tau[i] = Saturar(tau0[i] + kKpDedos * e[i] + kKdDedos * de[i],
                         tau_max_total_[i]);
    }

    // -------------------------------------------------------------------------
    // PASO 7: salida
    // -------------------------------------------------------------------------
    output->SetFromVector(tau);
}

// =============================================================================
// ControladorPanda (envoltorio con los mismos puertos que antes)
// =============================================================================
ControladorPanda::ControladorPanda(ModoControl modo) {
    DiagramBuilder<double> builder;

    // Copia de la planta para el controlador (modelo del robot sin el entorno).
    // Solo se usa para calcular M, c, g -> time_step = 0 (continua) basta.
    auto controller_plant = std::make_unique<MultibodyPlant<double>>(0.0);
    Parser controller_parser(controller_plant.get());
    controller_parser.AddModelsFromUrl(
        "package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
    controller_plant->WeldFrames(controller_plant->world_frame(),
                                 controller_plant->GetFrameByName("panda_link0"),
                                 RigidTransformd{});

    // -------------------------------------------------------------------------
    // FASE 2 (incertidumbre): aqui modificar masas/inercias de la copia
    // (o agregar una carga en franka_simulation.cc) para que el modelo del
    // controlador deje de coincidir con la planta "real".
    // -------------------------------------------------------------------------

    controller_plant->Finalize();

    auto etso = builder.AddSystem<ControladorETSO>(std::move(controller_plant), modo);
    etso->set_name("controlador_etso");

    estimated_state_input_port_index_ =
        builder.ExportInput(etso->get_estimated_state_input_port(), "estimated_state");
    desired_state_input_port_index_ =
        builder.ExportInput(etso->get_desired_state_input_port(), "desired_state");
    control_output_port_index_ =
        builder.ExportOutput(etso->get_control_output_port(), "control");

    builder.BuildInto(this);
}

}  // namespace ipre2026