#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"

namespace ipre2026 {

// Estructura de control (secciones III y IV del paper).
enum class ModoControl {
    kVSC,  // ETSO puro, tau0 = 0                                        (Fig. 1)
    kFL,   // ETSO + c(q,dq) + g(q) en el estado MEDIDO, ec. (12)        (Fig. 2)
    kFF,   // ETSO + dinamica inversa completa en la REFERENCIA, ec. (13) (Fig. 3)
};

ModoControl ModoDesdeTexto(const std::string& texto);  // "vsc" | "fl" | "ff"
std::string TextoDesdeModo(ModoControl modo);

// -----------------------------------------------------------------------------
// Parametros de diseno de la ETSO (las "decisiones" de la Tabla I).
// Todos los vectores son de tamano 7 (joints del brazo), salvo q_centro.
// -----------------------------------------------------------------------------
struct ParametrosDiseno {
    Eigen::VectorXd A;        // A_i      amplitud de la referencia [rad]
    Eigen::VectorXd Delta;    // Delta_i  precision deseada [% de A_i]
    double omega_ref{0.0};    // omega_i  frecuencia de la referencia [rad/s]

    // Region de trabajo donde se evaluan j_ii,max y tau_Lmax (ec. 4):
    //   q   en [q_centro - A - margen_q, q_centro + A + margen_q] (recortado a limites)
    //   |dq| <= A * omega_ref,   |ddq| <= A * omega_ref^2   (convencion de la Tabla I)
    Eigen::VectorXd q_centro;  // tamano 9
    double margen_q{0.1};      // [rad]

    double factor_seguridad{1.2};  // tau_Lmax = factor * max|tau_L| muestreado
    int muestras{20000};
};

// Valores por defecto, coherentes con la referencia actual (joint 1: 0.4 rad, 0.1 Hz).
ParametrosDiseno ParametrosPorDefecto();

// -----------------------------------------------------------------------------
// ControladorETSO: LeafSystem con la ley de control del paper.
//   Entradas:  estimated_state       [q; dq]      (2n)
//              desired_state         [q_d; dq_d]  (2n)
//              desired_acceleration  q̈_d          (n)  OPCIONAL: si no se
//                                                      conecta se usa 0
//   Salidas:   control    tau      (n)   torque aplicado, ec. (11)
//              tau0       tau0     (n)   parte del modelo (computed torque)
//              delta_tau  Δtau     (n)   parte robusta ETSO, ec. (8)-(9)
// -----------------------------------------------------------------------------
class ControladorETSO : public drake::systems::LeafSystem<double> {
public:
    // `plant` es el modelo que el controlador "cree" que tiene el robot (IDM).
    // Debe tener el damping B que el controlador asume (se lee del modelo).
    ControladorETSO(std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
                    ModoControl modo,
                    const ParametrosDiseno& parametros = ParametrosPorDefecto());

    const drake::systems::InputPort<double>& get_estimated_state_input_port() const {
        return this->get_input_port(estado_idx_);
    }
    const drake::systems::InputPort<double>& get_desired_state_input_port() const {
        return this->get_input_port(deseado_idx_);
    }
    const drake::systems::InputPort<double>& get_desired_acceleration_input_port() const {
        return this->get_input_port(aceleracion_idx_);
    }
    const drake::systems::OutputPort<double>& get_control_output_port() const {
        return this->get_output_port(control_idx_);
    }
    const drake::systems::OutputPort<double>& get_tau0_output_port() const {
        return this->get_output_port(tau0_idx_);
    }
    const drake::systems::OutputPort<double>& get_delta_tau_output_port() const {
        return this->get_output_port(delta_tau_idx_);
    }

    // Tabla de parametros en formato CSV (una fila por joint), para graficar.
    std::string TablaParametrosCsv() const;
    void ImprimirParametros() const;

    const Eigen::VectorXd& e_max() const { return e_max_; }
    const Eigen::VectorXd& tau_max() const { return tau_max_; }

private:
    // PASO 0: diseno de parametros ETSO (una vez, en el constructor)
    void CalcularParametrosETSO();
    // Muestrea la region de trabajo: j_ii,max y cota de la ec. (4).
    void MuestrearRegion(Eigen::VectorXd* j_max, Eigen::VectorXd* tau_L) const;

    // PASOS 1-5: ley de control
    void Calcular(const drake::systems::Context<double>& context,
                  Eigen::VectorXd* tau, Eigen::VectorXd* tau0,
                  Eigen::VectorXd* delta_tau) const;
    void CalcTorque(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;
    void CalcTau0(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const;
    void CalcDeltaTau(const drake::systems::Context<double>& context,
                      drake::systems::BasicVector<double>* output) const;

    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    // Contexto propio de la copia para evaluar M, c, g. Mutable porque las
    // Calc* son const (no es thread-safe; suficiente para este uso).
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;

    ModoControl modo_;
    ParametrosDiseno par_;
    int n_{0};       // total de joints (7 brazo + 2 dedos = 9)
    int n_etso_{0};  // joints controlados con ETSO (brazo = 7)

    drake::systems::InputPortIndex estado_idx_;
    drake::systems::InputPortIndex deseado_idx_;
    drake::systems::InputPortIndex aceleracion_idx_;
    drake::systems::OutputPortIndex control_idx_;
    drake::systems::OutputPortIndex tau0_idx_;
    drake::systems::OutputPortIndex delta_tau_idx_;

    // Parametros por articulacion del brazo (tamano n_etso_), nombres como en Tabla I
    Eigen::VectorXd tau_max_;     // tau_max,i   limite de torque (URDF)
    Eigen::VectorXd tau_L_max_;   // tau_Lmax,i  cota del torque de carga, ec. (4)
    Eigen::VectorXd B_;           // B_i         friccion viscosa (del modelo)
    Eigen::VectorXd J_max_;       // j_ii,max    inercia diagonal maxima
    Eigen::VectorXd T_;           // T_max,i = J_max / B
    Eigen::VectorXd K_;           // K_i = 1 / B
    Eigen::VectorXd w_max_;       // omega_max,i  ec. (6)
    Eigen::VectorXd alpha_;       // alpha_i      ec. (7)
    Eigen::VectorXd k_;           // k_i          ec. (10)
    Eigen::VectorXd e_max_;       // Delta_i/100 * A_i

    Eigen::VectorXd tau_max_total_;  // limites de torque de los n joints (incluye dedos)
};

// -----------------------------------------------------------------------------
// ControladorPanda: Diagram envoltorio usado por el proceso LCM. Construye la
// copia del modelo (con el mismo damping por defecto que la planta) y expone
// los mismos puertos que antes.
// -----------------------------------------------------------------------------
class ControladorPanda : public drake::systems::Diagram<double> {
public:
    explicit ControladorPanda(ModoControl modo = ModoControl::kFL);

    const drake::systems::InputPort<double>& get_estimated_state_input_port() const {
        return this->get_input_port(estimated_state_input_port_index_);
    }
    const drake::systems::InputPort<double>& get_desired_state_input_port() const {
        return this->get_input_port(desired_state_input_port_index_);
    }
    const drake::systems::OutputPort<double>& get_control_output_port() const {
        return this->get_output_port(control_output_port_index_);
    }

private:
    drake::systems::InputPortIndex estimated_state_input_port_index_;
    drake::systems::InputPortIndex desired_state_input_port_index_;
    drake::systems::OutputPortIndex control_output_port_index_;
};

// Construye la copia del modelo del Panda para el controlador (sin entorno).
std::unique_ptr<drake::multibody::MultibodyPlant<double>> CrearModeloControlador(
    const Eigen::VectorXd& damping);

}  // namespace ipre2026
