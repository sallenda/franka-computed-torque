#include "planta_comun.h"

#include <memory>
#include <Eigen/Dense>

#include "drake/common/eigen_types.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/systems/framework/diagram_builder.h"

namespace ipre2026 {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::SpatialInertia;
using drake::systems::DiagramBuilder;

PlantaComun::PlantaComun() {
    DiagramBuilder<double> builder;
    
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

    

    // Exportar puertos
    actuation_input_port_index_ = builder.ExportInput(plant_->get_actuation_input_port(panda), "actuation_input");
    state_output_port_index_ = builder.ExportOutput(plant_->get_state_output_port(panda), "state_output");
    query_output_port_index_ = builder.ExportOutput(scene_graph_->get_query_output_port(), "query_object");

    builder.BuildInto(this);
}

} // namespace ipre2026