#pragma once

/// @file
/// Protocolo LCM sincrono entre los tres procesos (planta, controlador,
/// trayectoria). Imita como funciona libfranka con el Franka real: el robot
/// entrega el estado en el instante t y el controlador devuelve el torque de
/// ESE MISMO instante, cada 1 ms.
///
/// Un ciclo k (t = k * kPeriodoControl):
///
///   mi_sim_lcm      publica PANDA_STATUS   con utime = t   (q, dq medidos)
///   mi_trayectoria  responde DESIRED_STATE con timestamp = t (q_d(t), dq_d(t))
///   mi_controlador  con estado(t) y referencia(t) publica
///                   PANDA_COMMAND con utime = t (torque)
///   mi_sim_lcm      recibe el comando con utime = t, lo aplica y avanza 1 ms
///
/// Como la planta espera el comando del instante actual antes de avanzar, no
/// hay retardo variable entre procesos: el lazo es el mismo que en una
/// simulacion de un solo proceso con el controlador muestreado a 1 ms.

#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Dense>

#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_constants.h"

namespace ipre2026 {

// Periodo del lazo de control [s]: 1 kHz, el que exige el Franka real (FCI).
constexpr double kPeriodoControl = 0.001;

const char kCanalEstado[] = "PANDA_STATUS";
const char kCanalComando[] = "PANDA_COMMAND";
const char kCanalReferencia[] = "DESIRED_STATE";

// Tiempo <-> utime (microsegundos enteros, para comparar sin error de redondeo)
inline int64_t UtimeDesdeTiempo(double t) {
    return static_cast<int64_t>(std::llround(t * 1e6));
}
inline double TiempoDesdeUtime(int64_t utime) { return 1e-6 * static_cast<double>(utime); }

// Pose home del Panda (7 brazo + 2 dedos), compartida por los tres procesos:
// pose inicial de la planta, referencia por defecto del controlador y centro
// de la trayectoria y de la region de trabajo del diseno ETSO.
inline Eigen::VectorXd PoseHome() {
    Eigen::VectorXd q(9);
    q << 0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0, 0.04, 0.04;
    return q;
}

inline drake::lcmt_panda_status ArmarEstado(int64_t utime, const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dq,
                                            const Eigen::VectorXd& tau_aplicado) {
    const int n = static_cast<int>(q.size());
    drake::lcmt_panda_status msg{};
    msg.utime = utime;
    msg.robot_utime = utime;
    msg.num_joints = n;
    msg.joint_position.assign(q.data(), q.data() + n);
    msg.joint_velocity.assign(dq.data(), dq.data() + n);
    msg.joint_torque.assign(tau_aplicado.data(), tau_aplicado.data() + n);
    msg.joint_torque_desired = msg.joint_torque;
    msg.joint_position_desired.assign(n, 0.0);
    msg.joint_velocity_desired.assign(n, 0.0);
    msg.joint_acceleration_desired.assign(n, 0.0);
    msg.joint_torque_external.assign(n, 0.0);
    msg.control_command_success_rate = 1.0;
    msg.control_mode = static_cast<int8_t>(
        drake::manipulation::franka_panda::PandaControlModes::kTorque);
    return msg;
}

inline drake::lcmt_panda_command ArmarComando(int64_t utime, const Eigen::VectorXd& tau) {
    const int n = static_cast<int>(tau.size());
    drake::lcmt_panda_command msg{};
    msg.utime = utime;
    msg.num_joint_position = 0;
    msg.num_joint_velocity = 0;
    msg.num_joint_torque = n;
    msg.joint_torque.assign(tau.data(), tau.data() + n);
    msg.control_mode_expected = static_cast<int8_t>(
        drake::manipulation::franka_panda::PandaControlModes::kTorque);
    return msg;
}

}  // namespace ipre2026
