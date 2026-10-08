/// @file
///
/// Proceso de control: lee el estado del Franka Panda simulado por
/// franka_simulation.cc desde el canal LCM "PANDA_STATUS", calcula el
/// torque con el controlador existente (ControladorPanda) y publica
/// el comando de torque en el canal LCM "PANDA_COMMAND".

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_command_sender.h"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/manipulation/franka_panda/panda_status_receiver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/multiplexer.h"

#include "controlador.h"

namespace ipre2026 {
namespace {

using drake::manipulation::franka_panda::PandaCommandSender;
using drake::manipulation::franka_panda::PandaStatusReceiver;
namespace PandaControlModes = drake::manipulation::franka_panda::PandaControlModes;
using drake::systems::DiagramBuilder;
using drake::systems::Multiplexer;
using drake::systems::Simulator;

// 7 joints del brazo + 2 de los dedos de la mano.
const int kNumJoints = 9;

// Convierte un lcmt_drake_signal recibido por LCM en un vector de estado deseado.
// Mientras no llega el primer mensaje, usa el estado home como default seguro.
class DesiredStateFromLcm : public drake::systems::LeafSystem<double> {
 public:
  DesiredStateFromLcm(int size, const Eigen::VectorXd& default_state)
      : size_(size), default_(default_state) {
    DeclareAbstractInputPort("lcmt_drake_signal",
                             drake::Value<drake::lcmt_drake_signal>());
    DeclareVectorOutputPort("desired_state", size,
                            &DesiredStateFromLcm::Calc);
  }
 private:
  void Calc(const drake::systems::Context<double>& ctx,
            drake::systems::BasicVector<double>* out) const {
    const auto& msg = get_input_port().Eval<drake::lcmt_drake_signal>(ctx);
    if (msg.dim == size_) {
      for (int i = 0; i < size_; ++i) out->SetAtIndex(i, msg.val[i]);
    } else {
      out->SetFromVector(default_);  // antes del primer mensaje
    }
  }
  int size_;
  Eigen::VectorXd default_;
};

int DoMain() {
    DiagramBuilder<double> builder;

    // Conexion LCM compartida
    auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

    // Recibe el estado del robot publicado por franka_simulation.cc
    auto status_sub = builder.AddSystem(
        drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_panda_status>(
            "PANDA_STATUS", lcm));
    status_sub->set_name("status_subscriber");
    auto status_receiver = builder.AddSystem<PandaStatusReceiver>(kNumJoints);
    status_receiver->set_name("status_receiver");

    // Arma el vector de estado [posicion; velocidad] que espera el controlador
    auto state_mux = builder.AddSystem<Multiplexer<double>>(
        std::vector<int>{kNumJoints, kNumJoints});
    state_mux->set_name("state_mux");

    // Recibe el estado deseado desde el canal LCM "DESIRED_STATE"
    // (publicado por trayectoria.cc). Usa pose home como default hasta que llegue
    // el primer mensaje.
    Eigen::VectorXd home = Eigen::VectorXd::Zero(2 * kNumJoints);
    home.head(kNumJoints) << 0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0, 0.04, 0.04;
    auto desired_sub = builder.AddSystem(
        drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_drake_signal>(
            "DESIRED_STATE", lcm));
    desired_sub->set_name("desired_subscriber");
    auto desired_receiver =
        builder.AddSystem<DesiredStateFromLcm>(2 * kNumJoints, home);
    desired_receiver->set_name("desired_receiver");

    // Controlador existente (InverseDynamicsController)
    auto controlador = builder.AddSystem<ControladorPanda>();
    controlador->set_name("controlador");

    // Empaqueta el torque calculado en un mensaje lcmt_panda_command
    auto command_sender = builder.AddSystem<PandaCommandSender>(
        kNumJoints, PandaControlModes::kTorque);
    command_sender->set_name("command_sender");
    auto command_pub = builder.AddSystem(
        drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_panda_command>(
            "PANDA_COMMAND", lcm, 0.005 /* periodo de publicacion */));
    command_pub->set_name("command_publisher");

    // Conexiones
    builder.Connect(status_sub->get_output_port(),
                     status_receiver->get_input_port());
    builder.Connect(status_receiver->get_position_measured_output_port(),
                     state_mux->get_input_port(0));
    builder.Connect(status_receiver->get_velocity_measured_output_port(),
                     state_mux->get_input_port(1));
    builder.Connect(state_mux->get_output_port(),
                     controlador->get_estimated_state_input_port());
    builder.Connect(desired_sub->get_output_port(),
                     desired_receiver->get_input_port());
    builder.Connect(desired_receiver->get_output_port(),
                     controlador->get_desired_state_input_port());
    builder.Connect(controlador->get_control_output_port(),
                     command_sender->get_torque_input_port());
    builder.Connect(command_sender->get_output_port(),
                     command_pub->get_input_port());

    auto diagram = builder.Build();

    Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);
    simulator.AdvanceTo(std::numeric_limits<double>::infinity());

    return 0;
}

}  // namespace
}  // namespace ipre2026

int main() {
    return ipre2026::DoMain();
}
