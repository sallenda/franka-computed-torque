#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

#include "planta_comun.h"
#include "controlador.h"

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
    auto controlador = builder.AddSystem<ipre2026::ControladorPanda>();

    /// CONEXIONES ///
    builder.Connect(planta->get_state_output_port(), controlador->get_estimated_state_input_port());
    builder.Connect(controlador->get_control_output_port(), planta->get_actuation_input_port());
    
    // Exportamos el input de estado deseado del controlador
    builder.ExportInput(controlador->get_desired_state_input_port(), "estado_deseado");

    MeshcatVisualizer<double>::AddToBuilder(&builder, planta->get_query_output_port(), meshcat);
    
    auto diagram = builder.Build();

    /// INICIALIZACIÓN SLIDERS ///
    const int num_pos = 9; // Posiciones del robot con los dedos
    meshcat->AddSlider("j1", -2.89, 2.89, 0.01, 0.0);
    meshcat->AddSlider("j2", -1.76, 1.76, 0.01, -0.5);
    meshcat->AddSlider("j3", -2.89, 2.89, 0.01, 0.0);
    meshcat->AddSlider("j4", -3.07, -0.07, 0.01, -2.0);
    meshcat->AddSlider("j5", -2.89, 2.89, 0.01, 0.0);
    meshcat->AddSlider("j6", -0.01, 3.75, 0.01, 1.5);
    meshcat->AddSlider("j7", -2.89, 2.89, 0.01, 0.0);
    meshcat->AddSlider("dedo", 0.0, 0.08, 0.001, 0.04);

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

    while (true) {
        Eigen::VectorXd q_target = Eigen::VectorXd::Zero(num_pos);
        q_target[0] = meshcat->GetSliderValue("j1");
        q_target[1] = meshcat->GetSliderValue("j2");
        q_target[2] = meshcat->GetSliderValue("j3");
        q_target[3] = meshcat->GetSliderValue("j4");
        q_target[4] = meshcat->GetSliderValue("j5");
        q_target[5] = meshcat->GetSliderValue("j6");
        q_target[6] = meshcat->GetSliderValue("j7");
        q_target[7] = meshcat->GetSliderValue("dedo");
        q_target[8] = q_target[7]; // dedos simétricos

        Eigen::VectorXd x_deseado = Eigen::VectorXd::Zero(num_pos * 2);
        x_deseado.head(num_pos) = q_target;

        diagram->get_input_port(0).FixValue(&context, x_deseado);

        double t = context.get_time();
        simulator.AdvanceTo(t + 0.02);
    }

    return 0;
}