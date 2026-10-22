/// @file
///
/// Proceso de control: por cada PANDA_STATUS (estado en el instante t) calcula
/// el torque con ControladorPanda (ETSO) y lo publica de inmediato en
/// PANDA_COMMAND con el mismo utime. Es la misma logica que un callback de
/// libfranka en el Franka real: estado(t) -> torque(t), una vez por milisegundo.
/// Ver protocolo_lcm.h.
///
/// La referencia se toma de DESIRED_STATE (publicada por mi_trayectoria con el
/// mismo timestamp t). Si mi_trayectoria no esta corriendo, se usa la pose home.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>

#include <Eigen/Dense>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/lcmt_drake_signal.hpp"

#include "controlador.h"
#include "protocolo_lcm.h"

namespace ipre2026 {
namespace {

// 7 joints del brazo + 2 de los dedos de la mano.
const int kNumJoints = 9;

// Cuanto esperar (reloj) la referencia del instante t una vez que la
// trayectoria ya esta corriendo. Normalmente llega en microsegundos.
constexpr auto kEsperaReferencia = std::chrono::milliseconds(20);

int DoMain() {
    // Controlador ETSO (el mismo Diagram de antes) y su contexto
    ControladorPanda controlador;
    auto ctx = controlador.CreateDefaultContext();

    Eigen::VectorXd home = Eigen::VectorXd::Zero(2 * kNumJoints);
    home.head(kNumJoints) = PoseHome();

    drake::lcm::DrakeLcm lcm;
    std::optional<drake::lcmt_panda_status> estado;
    std::optional<drake::lcmt_drake_signal> referencia;
    int64_t estados_recibidos = 0;  // cuenta tambien los estados repetidos
    auto sub_estado = lcm.Subscribe(kCanalEstado,
        [&estado, &estados_recibidos](const void* b, int n) {
            drake::lcmt_panda_status msg;
            msg.decode(b, 0, n);
            estado = std::move(msg);
            ++estados_recibidos;
        });
    auto sub_ref = lcm.Subscribe(kCanalReferencia, [&referencia](const void* b, int n) {
        drake::lcmt_drake_signal msg;
        msg.decode(b, 0, n);
        if (msg.dim == 2 * kNumJoints) referencia = std::move(msg);
    });
    sub_estado->set_queue_capacity(4);
    sub_ref->set_queue_capacity(4);

    std::cout << "Controlador listo: responde a cada " << kCanalEstado
              << " con " << kCanalComando << "." << std::endl;

    int64_t ultimo_utime = -1, estados_atendidos = 0;
    std::optional<drake::lcmt_panda_command> ultimo_comando;
    bool trayectoria_activa = false;
    int64_t referencias_tardias = 0;
    while (true) {
        lcm.HandleSubscriptions(100 /* ms */);
        if (!estado || estados_recibidos == estados_atendidos) continue;
        estados_atendidos = estados_recibidos;
        const int64_t utime = estado->utime;

        // La planta reenvia el mismo estado si no le llego el comando (paquete
        // UDP perdido): se reenvia el comando ya calculado para ese instante.
        if (utime == ultimo_utime) {
            drake::lcm::Publish(&lcm, kCanalComando, *ultimo_comando);
            continue;
        }
        if (estado->num_joints != kNumJoints) {
            std::cerr << "[ctrl] estado con " << estado->num_joints << " joints" << std::endl;
            return 1;
        }

        // Referencia del mismo instante. Antes de la primera referencia: home.
        // En el primer estado se espera un poco por si la trayectoria ya esta
        // corriendo (si no, el controlador usaria home 1 ms y daria un golpe).
        if (ultimo_utime < 0 && !referencia) {
            const auto inicio = std::chrono::steady_clock::now();
            while (!referencia &&
                   std::chrono::steady_clock::now() - inicio < kEsperaReferencia) {
                lcm.HandleSubscriptions(1);
            }
        }
        if (referencia) trayectoria_activa = true;
        if (trayectoria_activa && referencia->timestamp != utime) {
            const auto inicio = std::chrono::steady_clock::now();
            while (referencia->timestamp != utime &&
                   std::chrono::steady_clock::now() - inicio < kEsperaReferencia) {
                lcm.HandleSubscriptions(1);
            }
            if (referencia->timestamp != utime && ++referencias_tardias % 1000 == 1) {
                std::cerr << "[ctrl] referencia no llego a tiempo (" << referencias_tardias
                          << " veces); usando la ultima recibida" << std::endl;
            }
        }
        const Eigen::VectorXd xd = trayectoria_activa
            ? Eigen::Map<const Eigen::VectorXd>(referencia->val.data(), 2 * kNumJoints)
            : home;

        Eigen::VectorXd x(2 * kNumJoints);
        x << Eigen::Map<const Eigen::VectorXd>(estado->joint_position.data(), kNumJoints),
             Eigen::Map<const Eigen::VectorXd>(estado->joint_velocity.data(), kNumJoints);

        controlador.get_estimated_state_input_port().FixValue(ctx.get(), x);
        controlador.get_desired_state_input_port().FixValue(ctx.get(), xd);
        const Eigen::VectorXd tau = controlador.get_control_output_port().Eval(*ctx);

        ultimo_comando = ArmarComando(utime, tau);
        drake::lcm::Publish(&lcm, kCanalComando, *ultimo_comando);
        ultimo_utime = utime;
    }
    return 0;
}

}  // namespace
}  // namespace ipre2026

int main() {
    return ipre2026::DoMain();
}
