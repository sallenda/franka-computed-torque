#pragma once

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/model_instance.h"
#include "drake/systems/framework/leaf_system.h"


namespace ipre2026 {

// =============================================================================
// Friccion articular del Franka Emika Panda (modelo de Gaz et al., 2019)
//
//   C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca,
//   "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of
//   Feasible Parameters Using Penalty-Based Optimization", IEEE RA-L, 2019.
//
// Modelo sigmoidal por joint (ec. 25 del paper):
//
//   tau_f(dq) = phi1 / (1 + exp(-phi2 (dq + phi3))) - phi1 / (1 + exp(-phi2 phi3))
//
// Es un "Coulomb suave": vale 0 en dq = 0 y se satura en ~ +-phi1/2 para
// velocidades grandes. Gaz et al. reportan que la parte viscosa del Panda es
// despreciable. Los phi son los identificados en SU robot (material
// suplementario, Tabla I); el Franka real de la tesis puede tener otros.
// =============================================================================

constexpr int kNumJointsFriccion = 7;

// Tabla 7x3: fila j = (phi1 [Nm], phi2 [s/rad], phi3 [rad/s]) del joint j+1.
Eigen::Matrix<double, kNumJointsFriccion, 3> ParametrosFriccionGaz();

// Torque de friccion del joint j (0..6) a velocidad dq [rad/s].
// Convencion: es el torque que la friccion "consume", es decir, aparece en
// la ecuacion del robot como  J q̈ + c + g + tau_f(dq) = tau.
double TorqueFriccion(int j, double dq);

// Friccion viscosa equivalente B_j [Nms/rad]: ajuste por minimos cuadrados de
// tau_f(dq) ≈ B_j dq sobre |dq| <= v_ajuste.
double BEquivalente(int j, double v_ajuste);

// Cota del error del modelo lineal: max |tau_f(dq) - B dq| sobre |dq| <= v_cota.
// Es la parte de la friccion que el modelo del paper (J q̈ + B q̇) no explica,
// asi que se suma a la cota del torque de carga tau_Lmax (ec. 4).
double ResiduoFriccionMax(int j, double B, double v_cota);

// -----------------------------------------------------------------------------
// FriccionFranka: sistema que aplica la friccion a la planta simulada.
//   Entrada: estado completo de la planta [q; v] (plant.get_state_output_port())
//   Salida:  fuerzas generalizadas de TODA la planta (num_velocities), con
//            -tau_f(dq_j) en panda_joint1..7 y 0 en el resto (dedos, cuerpos
//            libres como el cubo). Se conecta a
//            plant.get_applied_generalized_force_input_port().
// -----------------------------------------------------------------------------
class FriccionFranka : public drake::systems::LeafSystem<double> {
public:
    // `plant` debe estar finalizada y vivir mas que este sistema.
    FriccionFranka(const drake::multibody::MultibodyPlant<double>& plant,
                   drake::multibody::ModelInstanceIndex panda);

private:
    void CalcFuerza(const drake::systems::Context<double>& context,
                    drake::systems::BasicVector<double>* output) const;

    int nq_{0}, nv_{0};
    // Indice de la velocidad de panda_joint(j+1) dentro de v (toda la planta)
    Eigen::Matrix<int, kNumJointsFriccion, 1> indice_v_;
};

}  // namespace ipre2026
