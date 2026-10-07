/// @file
///
/// Replica del paper en UN solo proceso (sin LCM): planta + referencia +
/// controlador ETSO + registro de datos. Al no haber procesos asincronos ni
/// retardo de comunicacion, el lazo es el del paper (Figs. 1-3) y los
/// resultados son deterministas y reproducibles.
///
/// Uso:
///   ./build/src/sin_lcm/experimento [opciones]
///     --modo=fl            vsc | fl | ff                         (defecto fl)
///     --referencia=seno    seno | excitacion                     (defecto seno)
///     --t_final=20         duracion [s]
///     --dt=1e-4            paso de la planta discreta [s] (0 = continua)
///     --delta=1.0          precision Delta_i [% de A_i] para los 7 joints
///     --B_planta=10        friccion viscosa de la planta [Nms] (por defecto
///                          la misma que asume el controlador)
///     --dt_log=1e-3        periodo de registro [s]
///     --offset=0           desplazamiento inicial del robot respecto de la
///                          referencia en los 7 joints [rad] (fase de alcance,
///                          como los experimentos 1 y 2 del paper)
///     --salida=resultados/fl   carpeta donde se escriben log.csv y parametros.csv
///     --visualizar         abre Meshcat y simula en tiempo real
///
/// Graficos: python3 scripts/graficar.py resultados/fl

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/vector_log_sink.h"

#include "controlador.h"
#include "modelo_panda.h"
#include "referencia.h"

namespace ipre2026 {
namespace {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::VectorLogSink;

// -----------------------------------------------------------------------------
// Opciones de linea de comando (--clave=valor o --bandera)
// -----------------------------------------------------------------------------
struct Opciones {
    std::string modo{"fl"};
    std::string referencia{"seno"};
    double t_final{20.0};
    double dt{1e-4};
    double delta{1.0};
    double B_planta{-1.0};  // < 0 => DampingPandaPorDefecto()
    double dt_log{1e-3};
    double offset{0.0};
    std::string salida;
    bool visualizar{false};
};

Opciones LeerOpciones(int argc, char** argv) {
    std::map<std::string, std::string> kv;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--", 0) != 0) throw std::runtime_error("argumento invalido: " + a);
        a = a.substr(2);
        const auto eq = a.find('=');
        kv[a.substr(0, eq)] = (eq == std::string::npos) ? "1" : a.substr(eq + 1);
    }
    Opciones o;
    for (const auto& [k, v] : kv) {
        if (k == "modo") o.modo = v;
        else if (k == "referencia") o.referencia = v;
        else if (k == "t_final") o.t_final = std::stod(v);
        else if (k == "dt") o.dt = std::stod(v);
        else if (k == "delta") o.delta = std::stod(v);
        else if (k == "B_planta") o.B_planta = std::stod(v);
        else if (k == "dt_log") o.dt_log = std::stod(v);
        else if (k == "offset") o.offset = std::stod(v);
        else if (k == "salida") o.salida = v;
        else if (k == "visualizar") o.visualizar = true;
        else throw std::runtime_error("opcion desconocida: --" + k);
    }
    if (o.salida.empty()) o.salida = "resultados/" + o.modo;
    return o;
}

// -----------------------------------------------------------------------------
// Fuente de referencia analitica: [q_d; dq_d] y q̈_d (ver comun/referencia.h)
// -----------------------------------------------------------------------------
class FuenteReferencia : public drake::systems::LeafSystem<double> {
public:
    explicit FuenteReferencia(TipoReferencia tipo) : tipo_(tipo) {
        DeclareVectorOutputPort("desired_state", 2 * kNumJointsTotal,
                                &FuenteReferencia::CalcEstado);
        DeclareVectorOutputPort("desired_acceleration", kNumJointsTotal,
                                &FuenteReferencia::CalcAceleracion);
    }

private:
    void CalcEstado(const Context<double>& ctx, BasicVector<double>* out) const {
        const EstadoDeseado r = EvaluarReferencia(tipo_, ctx.get_time());
        out->get_mutable_value() << r.q, r.dq;
    }
    void CalcAceleracion(const Context<double>& ctx, BasicVector<double>* out) const {
        out->SetFromVector(EvaluarReferencia(tipo_, ctx.get_time()).ddq);
    }
    TipoReferencia tipo_;
};

// Escribe una columna por componente: prefijo1..prefijoN
void Encabezado(std::ofstream& f, const std::string& prefijo, int n) {
    for (int i = 0; i < n; ++i) f << "," << prefijo << i + 1;
}

int DoMain(int argc, char** argv) {
    const Opciones op = LeerOpciones(argc, argv);
    const ModoControl modo = ModoDesdeTexto(op.modo);
    const TipoReferencia tipo_ref = ReferenciaDesdeTexto(op.referencia);
    const int n = kNumJointsTotal;

    const Eigen::VectorXd B_controlador = DampingPandaPorDefecto();
    const Eigen::VectorXd B_planta =
        op.B_planta < 0 ? B_controlador
                        : Eigen::VectorXd::Constant(kNumJointsBrazo, op.B_planta);

    DiagramBuilder<double> builder;

    // --- Planta "real": solo el Panda (sin mesa ni cubo), con damping B -------
    auto [plant, scene_graph] =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder, op.dt);
    const auto panda = AgregarPanda(&plant, B_planta);
    plant.Finalize();

    // --- Referencia ----------------------------------------------------------
    auto referencia = builder.AddSystem<FuenteReferencia>(tipo_ref);
    referencia->set_name("referencia");

    // --- Controlador ETSO (modelo propio = IDM del controlador) ---------------
    ParametrosDiseno par = ParametrosPorDefecto();
    par.Delta.setConstant(op.delta);
    auto etso = builder.AddSystem<ControladorETSO>(
        CrearModeloControlador(B_controlador), modo, par);
    etso->set_name("controlador_etso");
    etso->ImprimirParametros();

    builder.Connect(plant.get_state_output_port(panda),
                    etso->get_estimated_state_input_port());
    builder.Connect(referencia->GetOutputPort("desired_state"),
                    etso->get_desired_state_input_port());
    builder.Connect(referencia->GetOutputPort("desired_acceleration"),
                    etso->get_desired_acceleration_input_port());
    builder.Connect(etso->get_control_output_port(),
                    plant.get_actuation_input_port(panda));

    // --- Registro -------------------------------------------------------------
    auto log_x = LogVectorOutput(plant.get_state_output_port(panda), &builder, op.dt_log);
    auto log_xd = LogVectorOutput(referencia->GetOutputPort("desired_state"), &builder,
                                  op.dt_log);
    auto log_tau = LogVectorOutput(etso->get_control_output_port(), &builder, op.dt_log);
    auto log_tau0 = LogVectorOutput(etso->get_tau0_output_port(), &builder, op.dt_log);
    auto log_dtau = LogVectorOutput(etso->get_delta_tau_output_port(), &builder,
                                    op.dt_log);

    std::shared_ptr<drake::geometry::Meshcat> meshcat;
    if (op.visualizar) {
        meshcat = std::make_shared<drake::geometry::Meshcat>();
        drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph,
                                                                 meshcat);
    }

    auto diagram = builder.Build();

    // --- Condicion inicial: robot en el estado deseado en t = 0, menos offset --
    Simulator<double> simulator(*diagram);
    auto& plant_ctx = plant.GetMyMutableContextFromRoot(&simulator.get_mutable_context());
    const EstadoDeseado r0 = EvaluarReferencia(tipo_ref, 0.0);
    Eigen::VectorXd q0 = r0.q;
    q0.head(kNumJointsBrazo).array() -= op.offset;
    plant.SetPositions(&plant_ctx, panda, q0);
    plant.SetVelocities(&plant_ctx, panda, r0.dq);

    if (op.visualizar) {
        std::cout << "Meshcat: " << meshcat->web_url() << std::endl;
        simulator.set_target_realtime_rate(1.0);
    }
    simulator.Initialize();

    std::cout << "Simulando modo " << op.modo << ", referencia " << op.referencia
              << ", " << op.t_final << " s, dt = " << op.dt << " s..." << std::endl;
    const auto t0 = std::chrono::steady_clock::now();
    simulator.AdvanceTo(op.t_final);
    const double t_cpu =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // --- Guardar datos ---------------------------------------------------------
    std::filesystem::create_directories(op.salida);
    {
        std::ofstream f(op.salida + "/parametros.csv");
        f << etso->TablaParametrosCsv();
    }
    const auto& ctx = simulator.get_context();
    const auto& X = log_x->FindLog(ctx);
    const auto& XD = log_xd->FindLog(ctx);
    const auto& TAU = log_tau->FindLog(ctx);
    const auto& TAU0 = log_tau0->FindLog(ctx);
    const auto& DTAU = log_dtau->FindLog(ctx);
    const int m = std::min({X.num_samples(), XD.num_samples(), TAU.num_samples(),
                            TAU0.num_samples(), DTAU.num_samples()});
    {
        std::ofstream f(op.salida + "/log.csv");
        f << "t";
        Encabezado(f, "q", n);  Encabezado(f, "dq", n);
        Encabezado(f, "qd", n); Encabezado(f, "dqd", n);
        Encabezado(f, "tau", n); Encabezado(f, "tau0_", n); Encabezado(f, "dtau", n);
        f << "\n" << std::setprecision(10);
        for (int s = 0; s < m; ++s) {
            f << X.sample_times()[s];
            for (int i = 0; i < 2 * n; ++i) f << "," << X.data()(i, s);
            for (int i = 0; i < 2 * n; ++i) f << "," << XD.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << TAU.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << TAU0.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << DTAU.data()(i, s);
            f << "\n";
        }
    }

    // --- Resumen: error maximo vs e_max y fraccion de tiempo saturado ----------
    // t_alcance: desde cuando |e| queda dentro de la banda e_max para siempre.
    // max|e| se mide en la fase de seguimiento: desde t_alcance + 1 s (o desde
    // t = 0 si el robot parte sobre la referencia, offset = 0).
    std::cout << "\nResumen (" << m << " muestras, " << std::fixed
              << std::setprecision(1) << t_cpu << " s de CPU)\n"
              << " joint   t_alcance [s]   max|e| segu. [rad]   e_max [rad]   cumple"
              << "   % saturado\n";
    const auto& ts = X.sample_times();
    for (int i = 0; i < kNumJointsBrazo; ++i) {
        auto err = [&](int s) { return std::abs(XD.data()(i, s) - X.data()(i, s)); };
        int s_alcance = 0;
        for (int s = 0; s < m; ++s) {
            if (err(s) > etso->e_max()[i]) s_alcance = s + 1;
        }
        const bool alcanza = s_alcance < m;
        const double t_alcance = alcanza ? ts[s_alcance] : ts[m - 1];
        const double t_inicio = (op.offset == 0.0) ? 0.0 : t_alcance + 1.0;
        double e_max_obs = 0.0;
        int saturado = 0;
        for (int s = 0; s < m; ++s) {
            if (ts[s] >= t_inicio) e_max_obs = std::max(e_max_obs, err(s));
            if (std::abs(TAU.data()(i, s)) >= etso->tau_max()[i] - 1e-9) ++saturado;
        }
        if (!alcanza) e_max_obs = err(m - 1);
        std::cout << std::setw(6) << i + 1 << std::fixed << std::setprecision(3)
                  << std::setw(16) << (alcanza ? t_alcance : NAN)
                  << std::scientific << std::setw(21) << e_max_obs
                  << std::setw(14) << etso->e_max()[i]
                  << std::setw(9) << (alcanza && e_max_obs <= etso->e_max()[i] ? "si" : "NO")
                  << std::fixed << std::setprecision(1) << std::setw(13)
                  << 100.0 * saturado / std::max(m, 1) << "\n";
    }
    std::cout << "\nDatos en " << op.salida << "/ (log.csv, parametros.csv)" << std::endl;
    return 0;
}

}  // namespace
}  // namespace ipre2026

int main(int argc, char** argv) {
    try {
        return ipre2026::DoMain(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
