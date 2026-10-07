#pragma once

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/model_instance.h"

namespace ipre2026 {

// Numero de joints del brazo (sin dedos) y total del modelo panda_arm_hand.
constexpr int kNumJointsBrazo = 7;
constexpr int kNumJointsTotal = 9;

// -----------------------------------------------------------------------------
// Friccion viscosa B_i [Nms/rad] de los 7 joints del brazo.
//
// El URDF de drake_models NO trae <dynamics damping>, asi que el robot simulado
// no tiene friccion a menos que la agreguemos. Este es el valor "real" de la
// planta simulada. En el paper B_i = 50 Nms con tau_max = 1100/500 Nm; aqui se
// usa una proporcion parecida respecto a los 87/12 Nm del Panda.
//
// Restriccion fisica a tener en cuenta: la velocidad maxima en regimen con
// torque maximo es tau_max / B. Con B = 10 y tau_max = 12 Nm (joints 5-7) da
// 1.2 rad/s, suficiente para trayectorias lentas.
// -----------------------------------------------------------------------------
inline Eigen::VectorXd DampingPandaPorDefecto() {
    Eigen::VectorXd B(kNumJointsBrazo);
    B << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
    return B;
}

// Pose home (7 joints del brazo + 2 dedos).
inline Eigen::VectorXd PoseHome() {
    Eigen::VectorXd q(kNumJointsTotal);
    q << 0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0, 0.04, 0.04;
    return q;
}

// Agrega el Panda (panda_arm_hand.urdf) a `plant`, lo suelda al mundo y fija la
// friccion viscosa de los joints del brazo a `damping` (tamano 7).
// Debe llamarse antes de plant->Finalize().
//
// Se usa tanto para la planta "real" como para la copia del controlador, asi
// que para introducir incertidumbre basta con pasar un B distinto a cada una.
drake::multibody::ModelInstanceIndex AgregarPanda(
    drake::multibody::MultibodyPlant<double>* plant,
    const Eigen::VectorXd& damping);

}  // namespace ipre2026
