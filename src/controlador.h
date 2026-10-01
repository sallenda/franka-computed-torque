#pragma once

#include <memory>

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

// -----------------------------------------------------------------------------
// ControladorETSO: LeafSystem con la ley de control del paper.
//   Entradas:  estimated_state [q; dq]      (2n)
//              desired_state   [q_d; dq_d]  (2n)
//   Salida:    control         tau          (n)
// -----------------------------------------------------------------------------
class ControladorETSO : public drake::systems::LeafSystem<double> {
public:
    ControladorETSO(std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
                    ModoControl modo);

    const drake::systems::InputPort<double>& get_estimated_state_input_port() const {
        return this->get_input_port(estado_idx_);
    }
    const drake::systems::InputPort<double>& get_desired_state_input_port() const {
        return this->get_input_port(deseado_idx_);
    }
    const drake::systems::OutputPort<double>& get_control_output_port() const {
        return this->get_output_port(control_idx_);
    }

private:
    // PASO 0: diseno de parametros ETSO (se ejecuta una vez, en el constructor)
    void CalcularParametrosETSO();
    Eigen::VectorXd CalcularJmax(int num_muestras) const;
    void ImprimirParametros() const;

    // PASOS 1-7: ley de control (se ejecuta en cada paso)
    void CalcTorque(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;

    // Copia de la planta = modelo que el controlador "cree" que tiene el robot
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    // Contexto propio de la copia para evaluar M, c, g. Es mutable porque
    // CalcTorque es const (no es thread-safe, suficiente para este uso).
    mutable std::unique_ptr<drake::systems::Context<double>> plant_context_;

    ModoControl modo_;
    int n_{0};       // total de joints (7 brazo + 2 dedos = 9)
    int n_etso_{0};  // joints controlados con ETSO (brazo = 7)

    drake::systems::InputPortIndex estado_idx_;
    drake::systems::InputPortIndex deseado_idx_;
    drake::systems::OutputPortIndex control_idx_;

    // Parametros por articulacion del brazo (tamano n_etso_), nombres como en Tabla I
    Eigen::VectorXd tau_max_;     // tau_max,i   limite de torque
    Eigen::VectorXd tau_L_max_;   // tau_Lmax,i  cota del torque de carga (4)
    Eigen::VectorXd B_;           // B_i         friccion viscosa
    Eigen::VectorXd J_max_;       // j_ii,max    inercia diagonal maxima
    Eigen::VectorXd A_;           // A_i         amplitud de referencia
    Eigen::VectorXd Delta_;       // Delta_i     precision deseada [% de A_i]
    Eigen::VectorXd T_;           // T_max,i = J_max / B
    Eigen::VectorXd K_;           // K_i = 1 / B
    Eigen::VectorXd w_max_;       // omega_max,i  ec. (6)
    Eigen::VectorXd alpha_;       // alpha_i      ec. (7)
    Eigen::VectorXd k_;           // k_i          ec. (10)

    Eigen::VectorXd tau_max_total_;  // limites de torque de los n joints (incluye dedos)
};

// -----------------------------------------------------------------------------
// ControladorPanda: Diagram envoltorio con los mismos puertos que antes,
// para que franka_task_control.cc funcione sin cambios.
// -----------------------------------------------------------------------------
class ControladorPanda : public drake::systems::Diagram<double> {
public:
    explicit ControladorPanda(ModoControl modo = ModoControl::kVSC);

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

}  // namespace ipre2026