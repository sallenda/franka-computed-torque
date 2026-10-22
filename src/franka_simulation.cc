#include "planta_comun.h"

#include "friccion_franka.h"
#include "protocolo_lcm.h"

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

    // Agrega la planta y el scene graph. Paso de la planta = periodo de control
    // (1 ms): en cada paso se aplica el torque recibido para ese instante.
    auto [plant_ref, scene_graph_ref] = drake::multibody::AddMultibodyPlantSceneGraph(&builder, kPeriodoControl);
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

    // Friccion articular realista del Panda (Gaz et al. 2019, ver friccion_franka.h).
    // El URDF no trae friccion; sin esto el robot simulado no tendria ninguna.
    auto friccion = builder.AddSystem<FriccionFranka>(*plant_, panda);
    friccion->set_name("friccion_franka");
    builder.Connect(plant_->get_state_output_port(), friccion->get_input_port());
    builder.Connect(friccion->get_output_port(),
                    plant_->get_applied_generalized_force_input_port());

    // Entrada de torque del Panda (7 brazo + 2 dedos). La comunicacion LCM
    // (PANDA_STATUS / PANDA_COMMAND) se hace en main.cc, en un ciclo sincrono
    // que espera el comando de cada instante antes de avanzar (protocolo_lcm.h).
    torque_input_port_index_ = builder.ExportInput(plant_->get_actuation_input_port(panda), "torque");

    // Exportar puertos
    state_output_port_index_ = builder.ExportOutput(plant_->get_state_output_port(panda), "state_output");
    query_output_port_index_ = builder.ExportOutput(scene_graph_->get_query_output_port(), "query_object");

    builder.BuildInto(this);
}

} // namespace ipre2026