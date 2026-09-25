/// @file
///
/// Publica una trayectoria sinusoidal como estado deseado en el canal LCM
/// "DESIRED_STATE". El controlador (franka_task_control.cc) se suscribe a
/// este canal para mover el robot.
///
/// Parametros facilmente ajustables:
///   kFreqHz    - frecuencia de la oscilacion (Hz)
///   kAmpRad    - amplitud en radianes para el joint 0
///   kNumJoints - debe coincidir con el controlador (9)

#include <cmath>
#include <limits>
#include <numbers>

#include <Eigen/Dense>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace ipre2026 {
namespace {

const int kNumJoints = 9;
const double kFreqHz = 0.1;   // periodo de 10 segundos
const double kAmpRad = 0.4;   // amplitud de oscilacion del joint 0

// Genera el vector de estado deseado [q; dq] variando en el tiempo.
// El joint 0 (rotacion de la base) oscila sinusoidalmente alrededor de
// la posicion home. El resto permanece fijo en home.
class SetpointSinusoidal : public drake::systems::LeafSystem<double> {
 public:
  SetpointSinusoidal() {
    DeclareVectorOutputPort("desired_state", 2 * kNumJoints,
                            &SetpointSinusoidal::Calc);
  }
 private:
  void Calc(const drake::systems::Context<double>& ctx,
            drake::systems::BasicVector<double>* out) const {
    const double t = ctx.get_time();
    const double w = 2 * std::numbers::pi * kFreqHz;

    // Posicion home
    Eigen::VectorXd q(kNumJoints), dq(kNumJoints);
    q  << 0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0, 0.04, 0.04;
    dq.setZero();

    // Oscila el joint 0 alrededor del home
    q[0]  += kAmpRad * std::sin(w * t);
    dq[0]  = kAmpRad * w * std::cos(w * t);

    for (int i = 0; i < kNumJoints; ++i) {
      out->SetAtIndex(i, q[i]);
      out->SetAtIndex(kNumJoints + i, dq[i]);
    }
  }
};

// Convierte un vector de doubles en un lcmt_drake_signal para publicar por LCM.
class VectorToLcmSignal : public drake::systems::LeafSystem<double> {
 public:
  explicit VectorToLcmSignal(int size) : size_(size) {
    DeclareInputPort("input", drake::systems::kVectorValued, size);
    DeclareAbstractOutputPort("lcmt_drake_signal", &VectorToLcmSignal::Calc);
  }
 private:
  void Calc(const drake::systems::Context<double>& ctx,
            drake::lcmt_drake_signal* msg) const {
    const auto& vec = get_input_port().Eval(ctx);
    msg->dim = size_;
    msg->val.assign(vec.data(), vec.data() + size_);
    msg->coord.assign(size_, "");
    msg->timestamp = static_cast<int64_t>(ctx.get_time() * 1e6);
  }
  int size_;
};

int DoMain() {
  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto setpoint = builder.AddSystem<SetpointSinusoidal>();
  setpoint->set_name("setpoint");

  auto converter = builder.AddSystem<VectorToLcmSignal>(2 * kNumJoints);
  converter->set_name("converter");

  auto desired_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_drake_signal>(
          "DESIRED_STATE", lcm, 0.005 /* periodo de publicacion */));
  desired_pub->set_name("desired_publisher");

  builder.Connect(setpoint->get_output_port(), converter->get_input_port());
  builder.Connect(converter->get_output_port(), desired_pub->get_input_port());

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace
}  // namespace ipre2026

int main() {
  return ipre2026::DoMain();
}
