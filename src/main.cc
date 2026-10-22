/// @file
///
/// Proceso de la planta simulada (Panda + mesa + cubo + friccion) con Meshcat.
///
/// Ciclo SINCRONO con el controlador (ver protocolo_lcm.h). En cada paso k:
///   1. publica PANDA_STATUS con el estado en t = k * 1 ms
///   2. espera el PANDA_COMMAND con ese mismo utime
///   3. aplica ese torque y avanza la simulacion 1 ms
/// Si el controlador no esta corriendo, la simulacion queda en pausa (el robot
/// no se cae) y se reanuda sola cuando llega el primer comando.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>

#include <Eigen/Dense>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/lcm_messages.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include "planta_comun.h"
#include "protocolo_lcm.h"

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::math::RigidTransformd;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

namespace {

// Si no llega el comando en este tiempo (de reloj), se vuelve a publicar el
// estado: asi un controlador que parte tarde recibe el estado actual.
constexpr auto kReintento = std::chrono::milliseconds(50);
// Espera tras la cual se avisa que el controlador no responde.
constexpr auto kAvisoEspera = std::chrono::seconds(2);

}  // namespace

int main() {
    auto meshcat = std::make_shared<Meshcat>();
    meshcat->DeleteAddedControls();

    /// INICIALIZANDO BLOQUES ///
    DiagramBuilder<double> builder;
    auto planta = builder.AddSystem<ipre2026::PlantaComun>();
    MeshcatVisualizer<double>::AddToBuilder(&builder, planta->get_query_output_port(), meshcat);
    auto diagram = builder.Build();

    /// SIMULACION ///
    Simulator<double> simulator(*diagram);
    auto& context = simulator.get_mutable_context();
    const auto& plant = planta->get_plant();
    auto& m_plant_context = plant.GetMyMutableContextFromRoot(&context);
    auto& planta_context = planta->GetMyMutableContextFromRoot(&context);

    // Posición Z inicial: Mesa superior (0.0) + mitad del cubo (0.025) + margen (0.01)
    RigidTransformd initial_pose(Eigen::Vector3d(0.2, 0.0, 0.035));
    plant.SetFreeBodyPose(&m_plant_context, plant.GetBodyByName("cubo"), initial_pose);

    // El Panda parte en home: el diseno ETSO (J_max, j_min) supone que el robot
    // esta en la region de trabajo alrededor de home. Por defecto el URDF
    // dejaria q = 0, fuera incluso del limite del joint 4.
    const auto panda = plant.GetModelInstanceByName("panda");
    plant.SetPositions(&m_plant_context, panda, ipre2026::PoseHome());

    const int n = plant.num_actuated_dofs();  // 9 (el cubo no tiene actuadores)
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(n);
    planta->get_torque_input_port().FixValue(&planta_context, tau);

    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();

    // --- LCM: el ultimo comando recibido ---------------------------------------
    drake::lcm::DrakeLcm lcm;
    std::optional<drake::lcmt_panda_command> comando;
    auto sub = lcm.Subscribe(ipre2026::kCanalComando,
        [&comando](const void* buffer, int size) {
            drake::lcmt_panda_command msg;
            msg.decode(buffer, 0, size);
            comando = std::move(msg);
        });
    sub->set_queue_capacity(4);

    std::cout << "Meshcat disponible en: " << meshcat->web_url() << std::endl;
    std::cout << "Esperando al controlador (canal " << ipre2026::kCanalComando
              << "); la simulacion parte cuando responda..." << std::endl;

    const auto& puerto_estado = planta->get_state_output_port();
    bool controlador_activo = false;
    for (int64_t k = 0;; ++k) {
        const int64_t utime = k * ipre2026::UtimeDesdeTiempo(ipre2026::kPeriodoControl);

        // 1. Publicar el estado del instante actual
        const Eigen::VectorXd x = puerto_estado.Eval(planta_context);
        const auto estado = ipre2026::ArmarEstado(utime, x.head(n), x.tail(n), tau);
        drake::lcm::Publish(&lcm, ipre2026::kCanalEstado, estado);

        // 2. Esperar el comando de ESTE instante
        const auto inicio = std::chrono::steady_clock::now();
        auto ultimo_envio = inicio;
        bool avisado = false;
        while (!(comando && comando->utime == utime)) {
            lcm.HandleSubscriptions(1 /* ms */);
            const auto ahora = std::chrono::steady_clock::now();
            if (ahora - ultimo_envio > kReintento) {
                drake::lcm::Publish(&lcm, ipre2026::kCanalEstado, estado);
                ultimo_envio = ahora;
            }
            if (!avisado && ahora - inicio > kAvisoEspera) {
                std::cout << "[sim] t = " << ipre2026::TiempoDesdeUtime(utime)
                          << " s: en pausa, esperando al controlador..." << std::endl;
                avisado = true;
            }
        }
        if (comando->num_joint_torque != n) {
            std::cerr << "[sim] comando con " << comando->num_joint_torque
                      << " torques, se esperaban " << n << std::endl;
            return 1;
        }
        // Tras una pausa larga, que el reloj de tiempo real no intente "recuperar"
        if (avisado || !controlador_activo) {
            simulator.ResetStatistics();
            std::cout << (controlador_activo ? "[sim] controlador de vuelta; se reanuda."
                                             : "[sim] controlador conectado.") << std::endl;
            controlador_activo = true;
        }

        // 3. Aplicar el torque y avanzar un periodo de control
        tau = Eigen::Map<const Eigen::VectorXd>(comando->joint_torque.data(), n);
        planta->get_torque_input_port().FixValue(&planta_context, tau);
        simulator.AdvanceTo(static_cast<double>(k + 1) * ipre2026::kPeriodoControl);
    }
    return 0;
}
