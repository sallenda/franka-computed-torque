#include <iostream>
#include <limits>
#include <memory>
#include <Eigen/Dense>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include "planta_comun.h"

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::math::RigidTransformd;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;

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
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();

    auto& context = simulator.get_mutable_context();
    auto& m_plant_context = planta->get_plant().GetMyMutableContextFromRoot(&context);

    // Posición Z inicial: Mesa superior (0.0) + mitad del cubo (0.025) + margen (0.01)
    RigidTransformd initial_pose(Eigen::Vector3d(0.2, 0.0, 0.035));
    planta->get_plant().SetFreeBodyPose(&m_plant_context, planta->get_plant().GetBodyByName("cubo"), initial_pose);

    std::cout << "Meshcat disponible en: " << meshcat->web_url() << std::endl;
    std::cout << "Esperando comandos de torque en el canal LCM PANDA_COMMAND..." << std::endl;

    // El movimiento del robot ahora depende de los comandos recibidos por LCM
    // (publicados por franka_task_control), y el estado se publica en PANDA_STATUS.
    simulator.AdvanceTo(std::numeric_limits<double>::infinity());

    return 0;
}
