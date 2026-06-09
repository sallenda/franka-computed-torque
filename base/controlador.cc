#include "controlador.h"

#include <memory>
#include <Eigen/Dense>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

namespace ipre2026 {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::controllers::InverseDynamicsController;

ControladorPanda::ControladorPanda() {
    DiagramBuilder<double> builder;

    // Planta interna para el controlador (modelo del robot sin el mundo entorno)
    auto controller_plant = std::make_unique<MultibodyPlant<double>>(0.001);
    Parser controller_parser(controller_plant.get());
    controller_parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
    controller_plant->WeldFrames(controller_plant->world_frame(), controller_plant->GetFrameByName("panda_link0"), RigidTransformd{});
    controller_plant->Finalize();

    const int num_pos = controller_plant->num_positions();
    Eigen::VectorXd kp = Eigen::VectorXd::Constant(num_pos, 1500.0);
    Eigen::VectorXd ki = Eigen::VectorXd::Constant(num_pos, 0.0);
    Eigen::VectorXd kd = Eigen::VectorXd::Constant(num_pos, 150.0);

    auto controller = builder.AddSystem<InverseDynamicsController<double>>(
        std::move(controller_plant), kp, ki, kd, false);

    estimated_state_input_port_index_ = builder.ExportInput(controller->get_input_port_estimated_state(), "estimated_state");
    desired_state_input_port_index_ = builder.ExportInput(controller->get_input_port_desired_state(), "desired_state");
    control_output_port_index_ = builder.ExportOutput(controller->get_output_port_control(), "control");

    builder.BuildInto(this);
}

} // namespace ipre2026