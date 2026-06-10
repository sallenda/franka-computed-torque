#include "planta_comun.h"

#include <memory>
#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"
#include "drake/manipulation/franka_panda/panda_command_receiver.h"
#include "drake/manipulation/franka_panda/panda_constants.h"
#include "drake/manipulation/franka_panda/panda_status_sender.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"



namespace ipre2026 {

using drake::geometry::Box;
using drake::manipulation::franka_panda::PandaCommandReceiver;
using drake::manipulation::franka_panda::PandaStatusSender;
namespace PandaControlModes = drake::manipulation::franka_panda::PandaControlModes;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::systems::Demultiplexer;
using drake::systems::DiagramBuilder;

PlantaComun::PlantaComun() {
    DiagramBuilder<double> builder;

    // Agrega la planta y el scene graph
    auto [plant_ref, scene_graph_ref] = drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    plant_ = &plant_ref;
    scene_graph_ = &scene_graph_ref;
    
    Parser parser(plant_);

    // suelo
    RigidTransformd suelo(Eigen::Vector3d(0.0, 0.0, -0.05));
    plant_->RegisterVisualGeometry(plant_->world_body(), suelo, Box(5, 5, 0.02), "mesa_vis", drake::Vector4<double>(0.5, 0.5, 0.5, 1.0));
    plant_->RegisterCollisionGeometry(plant_->world_body(), suelo, Box(5, 5, 0.02), "mesa_col", CoulombFriction<double>(1.0, 1.0));

    // modelo del brazo
    auto panda_models = parser.AddModelsFromUrl("package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
    const auto& panda = panda_models[0];
    plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("panda_link0", panda), RigidTransformd{});

    // instancia de cubo
    auto instancia_cubo = plant_->AddModelInstance("cubo");
    // Cubo más grande (ej: 0.1 x 0.1 x 0.1)
    const auto& cuerpo_cubo = plant_->AddRigidBody("cubo", instancia_cubo, SpatialInertia<double>::SolidBoxWithMass(0.1, 0.1, 0.1, 0.1));
    plant_->RegisterCollisionGeometry(cuerpo_cubo, RigidTransformd{}, Box(0.1, 0.1, 0.1), "cubo_col", CoulombFriction<double>(2.0, 1.5));
    plant_->RegisterVisualGeometry(cuerpo_cubo, RigidTransformd{}, Box(0.1, 0.1, 0.1), "cubo_vis", drake::Vector4<double>(0.9, 0.2, 0.2, 1.0));
    

    // gravedad
    plant_->mutable_gravity_field().set_gravity_vector(Eigen::Vector3d(0.0, 0.0, -9.81));


    plant_->Finalize();

    // Numero de joints controlados por LCM (7 del brazo + 2 de los dedos)
    const int num_joints = plant_->num_actuated_dofs(panda);

    // Conexion LCM compartida
    auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

    // Recibe los torques deseados desde el canal PANDA_COMMAND
    auto command_sub = builder.AddSystem(
        drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_panda_command>(
            "PANDA_COMMAND", lcm));
    command_sub->set_name("command_subscriber");
    auto command_receiver = builder.AddSystem<PandaCommandReceiver>(
        num_joints, PandaControlModes::kTorque);
    command_receiver->set_name("command_receiver");

    // Publica el estado del robot por el canal PANDA_STATUS
    auto status_sender = builder.AddSystem<PandaStatusSender>(num_joints);
    status_sender->set_name("status_sender");
    auto status_pub = builder.AddSystem(
        drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_panda_status>(
            "PANDA_STATUS", lcm, 0.005 /* periodo de publicacion */));
    status_pub->set_name("status_publisher");

    // Separa el estado de la planta [q; v] en posicion y velocidad
    auto state_demux = builder.AddSystem<Demultiplexer>(2 * num_joints, num_joints);
    state_demux->set_name("state_demux");

    // Conexiones del comando: PANDA_COMMAND -> torque -> actuacion de la planta
    builder.Connect(command_sub->get_output_port(),
                     command_receiver->get_message_input_port());
    builder.Connect(state_demux->get_output_port(0),
                     command_receiver->get_position_measured_input_port());
    builder.Connect(command_receiver->get_commanded_torque_output_port(),
                     plant_->get_actuation_input_port(panda));

    // Conexiones del estado: planta -> PANDA_STATUS
    builder.Connect(plant_->get_state_output_port(panda),
                     state_demux->get_input_port());
    builder.Connect(state_demux->get_output_port(0),
                     status_sender->get_position_measured_input_port());
    builder.Connect(state_demux->get_output_port(1),
                     status_sender->get_velocity_measured_input_port());
    builder.Connect(command_receiver->get_commanded_torque_output_port(),
                     status_sender->get_torque_commanded_input_port());
    builder.Connect(status_sender->get_output_port(),
                     status_pub->get_input_port());

    // Exportar puertos
    state_output_port_index_ = builder.ExportOutput(plant_->get_state_output_port(panda), "state_output");
    query_output_port_index_ = builder.ExportOutput(scene_graph_->get_query_output_port(), "query_object");

    builder.BuildInto(this);
}

} // namespace ipre2026