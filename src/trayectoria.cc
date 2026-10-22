/// @file
///
/// Publica una trayectoria sinusoidal como estado deseado en el canal LCM
/// "DESIRED_STATE". El controlador (franka_task_control.cc) se suscribe a
/// este canal para mover el robot.
///
/// Es sincrono con la planta (ver protocolo_lcm.h): por cada PANDA_STATUS con
/// tiempo t publica q_d(t), dq_d(t) con timestamp = t, asi el controlador usa
/// la referencia del mismo instante que el estado medido. El tiempo de la
/// trayectoria parte en 0 cuando arranca este proceso.
///
/// Parametros facilmente ajustables:
///   kFreqHz    - frecuencia de la oscilacion (Hz)
///   kAmpRad    - amplitud en radianes para el joint 0
///   kNumJoints - debe coincidir con el controlador (9)

#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcmt_panda_status.hpp"

#include "protocolo_lcm.h"

namespace ipre2026 {
namespace {

const int kNumJoints = 9;
const double kFreqHz = 0.1;   // periodo de 10 segundos
const double kAmpRad = 0.4;   // amplitud de oscilacion del joint 0

// Estado deseado [q; dq] en el tiempo t (desde que parte la trayectoria).
// El joint 0 (rotacion de la base) oscila sinusoidalmente alrededor de
// la posicion home. El resto permanece fijo en home.
Eigen::VectorXd SetpointSinusoidal(double t) {
    const double w = 2 * std::numbers::pi * kFreqHz;

    // Posicion home
    Eigen::VectorXd q = PoseHome(), dq = Eigen::VectorXd::Zero(kNumJoints);

    // Oscila el joint 0 alrededor del home
    q[0]  += kAmpRad * std::sin(w * t);
    dq[0]  = kAmpRad * w * std::cos(w * t);

    Eigen::VectorXd x(2 * kNumJoints);
    x << q, dq;
    return x;
}

int DoMain() {
    drake::lcm::DrakeLcm lcm;
    int64_t utime_estado = -1;
    int64_t estados_recibidos = 0, estados_atendidos = 0;  // incluye repetidos
    auto sub = lcm.Subscribe(kCanalEstado,
        [&utime_estado, &estados_recibidos](const void* b, int n) {
            drake::lcmt_panda_status msg;
            msg.decode(b, 0, n);
            utime_estado = msg.utime;
            ++estados_recibidos;
        });
    sub->set_queue_capacity(4);

    std::cout << "Trayectoria lista: responde a cada " << kCanalEstado << " con "
              << kCanalReferencia << "." << std::endl;

    int64_t utime_inicio = -1;
    while (true) {
        lcm.HandleSubscriptions(100 /* ms */);
        // Tambien se responde a un estado repetido (la planta lo reenvia si
        // el lazo se detuvo, p. ej. por un paquete perdido).
        if (estados_recibidos == estados_atendidos) continue;
        estados_atendidos = estados_recibidos;
        if (utime_inicio < 0) utime_inicio = utime_estado;

        const double t = TiempoDesdeUtime(utime_estado - utime_inicio);
        const Eigen::VectorXd x = SetpointSinusoidal(t);

        drake::lcmt_drake_signal msg;
        msg.dim = 2 * kNumJoints;
        msg.val.assign(x.data(), x.data() + x.size());
        msg.coord.assign(msg.dim, "");
        msg.timestamp = utime_estado;  // mismo instante que el estado
        drake::lcm::Publish(&lcm, kCanalReferencia, msg);
    }
    return 0;
}

}  // namespace
}  // namespace ipre2026

int main() {
    return ipre2026::DoMain();
}
