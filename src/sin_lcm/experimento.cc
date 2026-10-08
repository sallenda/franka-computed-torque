/// @file
///
/// Replica del paper en UN solo proceso (sin LCM): planta + referencia +
/// controlador ETSO + registro de datos. Al no haber procesos asincronos ni
/// retardo de comunicacion, el lazo es el del paper (Figs. 1-3) y los
/// resultados son deterministas y reproducibles.
///
/// Dos formas de uso:
///
///   * Por lotes (defecto): simula --t_log segundos lo mas rapido posible,
///     escribe log.csv + parametros.csv, imprime el resumen y termina.
///
///   * En vivo (--visualizar): abre Meshcat y simula en tiempo real sin fin.
///     Parte EN PAUSA (para que el navegador alcance a cargar el robot antes
///     de la fase de alcance, que dura < 1 s). Solo los primeros --t_log
///     segundos se guardan en el CSV; despues la simulacion sigue corriendo
///     para mirar el robot. Se detiene con Ctrl+C o con "Detener".
///
/// Uso:
///   ./build/src/sin_lcm/experimento [opciones]
///     --modo=fl            vsc | fl | ff                         (defecto fl)
///     --referencia=seno    seno | excitacion                     (defecto seno)
///     --t_log=10           segundos iniciales que se guardan en el CSV [s]
///     --t_final=...        duracion total [s]. Por defecto = t_log sin
///                          --visualizar, e infinita con --visualizar
///     --dt=...             paso de la planta discreta [s] (0 = continua).
///                          Por defecto 1e-4, o 1e-3 con --visualizar para
///                          poder correr en tiempo real (a 1e-4 la simulacion
///                          va ~3 veces mas lenta que el tiempo real)
///     --delta=1.0          precision Delta_i [% de A_i] para los 7 joints
///     --B_planta=10        friccion viscosa de la planta [Nms] (por defecto
///                          la misma que asume el controlador)
///     --dt_log=1e-3        periodo de registro [s]
///     --offset=0           desplazamiento inicial del robot respecto de la
///                          referencia en los 7 joints [rad] (fase de alcance,
///                          como los experimentos 1 y 2 del paper)
///     --salida=resultados/fl   carpeta donde se escriben log.csv y parametros.csv
///     --visualizar         modo en vivo con Meshcat (ver arriba)
///
/// En Meshcat (modo en vivo; los controles estan en "Open Controls"):
///   - la silueta azul translucida es la REFERENCIA q_d (robot "fantasma");
///     el robot blanco es la planta simulada
///   - Barra espaciadora / "Pausa / Continuar": inicia o pausa la simulacion
///   - Tecla R / "Reiniciar (t = 0)": vuelve al estado inicial, en pausa
///     (para volver a ver la fase de alcance)
///   - "Velocidad": factor de tiempo real (0.05 = camara lenta)
///   - "Detener": termina el programa
///   (las teclas funcionan con el foco en la ventana de Meshcat)
///
/// Graficos: python3 scripts/graficar.py resultados/fl

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "controlador.h"
#include "modelo_panda.h"
#include "referencia.h"

namespace ipre2026 {
namespace {

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::VectorLogSink;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

constexpr double kInfinito = std::numeric_limits<double>::infinity();

// Nombres de los controles de Meshcat
const char kBotonPausa[] = "Pausa / Continuar";
const char kBotonReiniciar[] = "Reiniciar (t = 0)";
const char kBotonDetener[] = "Detener";
const char kSliderVelocidad[] = "Velocidad";
const char kPrefijoFantasma[] = "referencia";

// -----------------------------------------------------------------------------
// Opciones de linea de comando (--clave=valor o --bandera)
// -----------------------------------------------------------------------------
struct Opciones {
    std::string modo{"fl"};
    std::string referencia{"seno"};
    double t_log{10.0};
    double t_final{-1.0};  // < 0 => segun el modo (ver LeerOpciones)
    double dt{-1.0};       // < 0 => segun el modo
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
        else if (k == "t_log") o.t_log = std::stod(v);
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
    if (o.dt < 0) o.dt = o.visualizar ? 1e-3 : 1e-4;
    if (o.t_final < 0) o.t_final = o.visualizar ? kInfinito : o.t_log;
    o.t_log = std::min(o.t_log, o.t_final);
    if (o.t_log <= 0) throw std::runtime_error("--t_log debe ser > 0");
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

// -----------------------------------------------------------------------------
// Robot "fantasma": dibuja la referencia q_d como una silueta azul translucida.
//
// Es un segundo Panda con su propia MultibodyPlant (fuera de la dinamica),
// registrado en el mismo SceneGraph y movido por MultibodyPositionToGeometryPose.
// Se dibujan sus geometrias de COLISION (66 esferas del URDF): son primitivas,
// asi que la transparencia va en su color. (Las mallas glTF del modelo visual
// cargan asincronamente en el navegador y no aceptan bien la transparencia.)
//   - Se le quita el rol de ilustracion -> el visualizador del robot real no lo ve.
//   - Sus geometrias de proximidad se marcan con (meshcat, accepting) =
//     "referencia" -> solo las dibuja el visualizador con ese prefijo.
//   - Se filtran sus colisiones con todo -> no afecta la dinamica.
// -----------------------------------------------------------------------------
void AgregarFantasmaReferencia(DiagramBuilder<double>* builder,
                               SceneGraph<double>* scene_graph,
                               const MultibodyPlant<double>& planta_real,
                               const drake::systems::OutputPort<double>& estado_deseado,
                               const drake::geometry::Rgba& color) {
    auto fantasma = std::make_unique<MultibodyPlant<double>>(0.0);
    fantasma->RegisterAsSourceForSceneGraph(scene_graph);
    AgregarPanda(fantasma.get(), DampingPandaPorDefecto(), kPrefijoFantasma);
    fantasma->Finalize();
    const auto source_id = *fantasma->get_source_id();
    const auto& inspector = scene_graph->model_inspector();

    drake::geometry::GeometrySet geometrias_fantasma, geometrias_reales;
    for (drake::multibody::BodyIndex b(0); b < fantasma->num_bodies(); ++b) {
        const auto frame = fantasma->GetBodyFrameIdIfExists(b);
        if (!frame) continue;
        geometrias_fantasma.Add(*frame);
        if (!inspector.GetGeometries(*frame, Role::kIllustration).empty()) {
            scene_graph->RemoveRole(source_id, *frame, Role::kIllustration);
        }
        for (const auto g : inspector.GetGeometries(*frame, Role::kProximity)) {
            drake::geometry::ProximityProperties props(*inspector.GetProximityProperties(g));
            props.AddProperty("meshcat", "accepting", std::string(kPrefijoFantasma));
            props.AddProperty("phong", "diffuse", color);
            scene_graph->AssignRole(source_id, g, props,
                                    drake::geometry::RoleAssign::kReplace);
        }
    }
    for (drake::multibody::BodyIndex b(0); b < planta_real.num_bodies(); ++b) {
        if (const auto frame = planta_real.GetBodyFrameIdIfExists(b)) {
            geometrias_reales.Add(*frame);
        }
    }
    scene_graph->collision_filter_manager().Apply(
        drake::geometry::CollisionFilterDeclaration()
            .ExcludeWithin(geometrias_fantasma)
            .ExcludeBetween(geometrias_fantasma, geometrias_reales));

    // input_multibody_state = true: la entrada es [q; dq], igual que desired_state
    auto a_pose = builder->AddSystem<MultibodyPositionToGeometryPose<double>>(
        std::move(fantasma), true);
    a_pose->set_name("fantasma_referencia");
    builder->Connect(estado_deseado, a_pose->get_input_port());
    builder->Connect(a_pose->get_output_port(),
                     scene_graph->get_source_pose_port(source_id));
}

// Escribe una columna por componente: prefijo1..prefijoN
void Encabezado(std::ofstream& f, const std::string& prefijo, int n) {
    for (int i = 0; i < n; ++i) f << "," << prefijo << i + 1;
}

// Loggers del experimento (todos con el mismo periodo)
struct Registros {
    VectorLogSink<double>* x{};
    VectorLogSink<double>* xd{};
    VectorLogSink<double>* tau{};
    VectorLogSink<double>* tau0{};
    VectorLogSink<double>* dtau{};

    std::vector<VectorLogSink<double>*> todos() const { return {x, xd, tau, tau0, dtau}; }

    // Vacia los registros (para no acumular memoria despues de t_log).
    void Vaciar(Context<double>* raiz) const {
        for (auto* log : todos()) log->FindMutableLog(raiz).Clear();
    }
};

// -----------------------------------------------------------------------------
// Escribe log.csv y parametros.csv con las muestras t <= t_log e imprime el
// resumen: tiempo de alcance, error de seguimiento vs e_max y % de saturacion.
// -----------------------------------------------------------------------------
void GuardarYResumir(const Opciones& op, const ControladorETSO& etso,
                     const Registros& reg, const Context<double>& ctx, double t_cpu) {
    const int n = kNumJointsTotal;
    std::filesystem::create_directories(op.salida);
    {
        std::ofstream f(op.salida + "/parametros.csv");
        f << etso.TablaParametrosCsv();
    }
    const auto& X = reg.x->FindLog(ctx);
    const auto& XD = reg.xd->FindLog(ctx);
    const auto& TAU = reg.tau->FindLog(ctx);
    const auto& TAU0 = reg.tau0->FindLog(ctx);
    const auto& DTAU = reg.dtau->FindLog(ctx);
    const auto& ts = X.sample_times();
    int m = std::min({X.num_samples(), XD.num_samples(), TAU.num_samples(),
                      TAU0.num_samples(), DTAU.num_samples()});
    while (m > 0 && ts[m - 1] > op.t_log + 1e-9) --m;  // solo la ventana [0, t_log]
    if (m == 0) throw std::runtime_error("no hay muestras registradas");
    {
        std::ofstream f(op.salida + "/log.csv");
        f << "t";
        Encabezado(f, "q", n);  Encabezado(f, "dq", n);
        Encabezado(f, "qd", n); Encabezado(f, "dqd", n);
        Encabezado(f, "tau", n); Encabezado(f, "tau0_", n); Encabezado(f, "dtau", n);
        f << "\n" << std::setprecision(10);
        for (int s = 0; s < m; ++s) {
            f << ts[s];
            for (int i = 0; i < 2 * n; ++i) f << "," << X.data()(i, s);
            for (int i = 0; i < 2 * n; ++i) f << "," << XD.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << TAU.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << TAU0.data()(i, s);
            for (int i = 0; i < n; ++i) f << "," << DTAU.data()(i, s);
            f << "\n";
        }
    }

    // t_alcance: desde cuando |e| queda dentro de la banda e_max para siempre.
    // max|e| se mide en la fase de seguimiento: desde t_alcance + 1 s (o desde
    // t = 0 si el robot parte sobre la referencia, offset = 0).
    std::cout << "\nResumen de los primeros " << op.t_log << " s (" << m
              << " muestras, " << std::fixed << std::setprecision(1) << t_cpu
              << " s de reloj)\n"
              << " joint   t_alcance [s]   max|e| segu. [rad]   e_max [rad]   cumple"
              << "   % saturado\n";
    for (int i = 0; i < kNumJointsBrazo; ++i) {
        auto err = [&](int s) { return std::abs(XD.data()(i, s) - X.data()(i, s)); };
        int s_alcance = 0;
        for (int s = 0; s < m; ++s) {
            if (err(s) > etso.e_max()[i]) s_alcance = s + 1;
        }
        const bool alcanza = s_alcance < m;
        const double t_alcance = alcanza ? ts[s_alcance] : ts[m - 1];
        const double t_inicio = (op.offset == 0.0) ? 0.0 : t_alcance + 1.0;
        double e_max_obs = 0.0;
        int saturado = 0;
        for (int s = 0; s < m; ++s) {
            if (ts[s] >= t_inicio) e_max_obs = std::max(e_max_obs, err(s));
            if (std::abs(TAU.data()(i, s)) >= etso.tau_max()[i] - 1e-9) ++saturado;
        }
        if (!alcanza) e_max_obs = err(m - 1);
        std::cout << std::setw(6) << i + 1 << std::fixed << std::setprecision(3)
                  << std::setw(16) << (alcanza ? t_alcance : NAN)
                  << std::scientific << std::setw(21) << e_max_obs
                  << std::setw(14) << etso.e_max()[i]
                  << std::setw(9) << (alcanza && e_max_obs <= etso.e_max()[i] ? "si" : "NO")
                  << std::fixed << std::setprecision(1) << std::setw(13)
                  << 100.0 * saturado / std::max(m, 1) << "\n";
    }
    std::cout << "\nDatos en " << op.salida << "/ (log.csv, parametros.csv)" << std::endl;
}

int DoMain(int argc, char** argv) {
    const Opciones op = LeerOpciones(argc, argv);
    const ModoControl modo = ModoDesdeTexto(op.modo);
    const TipoReferencia tipo_ref = ReferenciaDesdeTexto(op.referencia);

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
    Registros reg;
    reg.x = LogVectorOutput(plant.get_state_output_port(panda), &builder, op.dt_log);
    reg.xd = LogVectorOutput(referencia->GetOutputPort("desired_state"), &builder,
                             op.dt_log);
    reg.tau = LogVectorOutput(etso->get_control_output_port(), &builder, op.dt_log);
    reg.tau0 = LogVectorOutput(etso->get_tau0_output_port(), &builder, op.dt_log);
    reg.dtau = LogVectorOutput(etso->get_delta_tau_output_port(), &builder, op.dt_log);

    // --- Visualizacion (solo en vivo) -----------------------------------------
    std::shared_ptr<Meshcat> meshcat;
    if (op.visualizar) {
        meshcat = std::make_shared<Meshcat>();
        // Referencia: silueta azul translucida (solo visual, ver arriba)
        const drake::geometry::Rgba azul(0.16, 0.47, 0.84, 0.35);
        AgregarFantasmaReferencia(&builder, &scene_graph, plant,
                                  referencia->GetOutputPort("desired_state"), azul);
        // Robot real
        MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);
        // Robot fantasma: geometria de proximidad marcada con "accepting"
        drake::geometry::MeshcatVisualizerParams params_fantasma;
        params_fantasma.prefix = kPrefijoFantasma;
        params_fantasma.role = Role::kProximity;
        params_fantasma.default_color = azul;
        params_fantasma.include_unspecified_accepting = false;
        MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat,
                                                params_fantasma);
    }

    auto diagram = builder.Build();

    // --- Condicion inicial: robot en el estado deseado en t = 0, menos offset --
    Simulator<double> simulator(*diagram);
    auto& raiz = simulator.get_mutable_context();
    auto& plant_ctx = plant.GetMyMutableContextFromRoot(&raiz);
    const EstadoDeseado r0 = EvaluarReferencia(tipo_ref, 0.0);
    Eigen::VectorXd q0 = r0.q;
    q0.head(kNumJointsBrazo).array() -= op.offset;
    plant.SetPositions(&plant_ctx, panda, q0);
    plant.SetVelocities(&plant_ctx, panda, r0.dq);
    // Copia del estado inicial, para el boton "Reiniciar"
    const std::unique_ptr<Context<double>> ctx_inicial = raiz.Clone();

    std::cout << "Modo " << op.modo << ", referencia " << op.referencia
              << ", dt = " << op.dt << " s, se registran los primeros " << op.t_log
              << " s" << std::endl;

    simulator.Initialize();  // registra t = 0 y dibuja la pose inicial
    if (op.visualizar) {
        meshcat->AddButton(kBotonPausa, "Space");
        meshcat->AddButton(kBotonReiniciar, "KeyR");
        meshcat->AddButton(kBotonDetener);
        meshcat->AddSlider(kSliderVelocidad, 0.05, 2.0, 0.05, 1.0);
        simulator.set_target_realtime_rate(1.0);
        // Camara encuadrando el brazo
        meshcat->SetCameraPose(Eigen::Vector3d(1.6, -1.6, 1.1),
                               Eigen::Vector3d(0.2, 0.0, 0.45));

        std::cout << "\nAbre Meshcat en el navegador: " << meshcat->web_url()
                  << "\nLa simulacion esta EN PAUSA. Para empezar, presiona la barra "
                  << "espaciadora en Meshcat\n(o \"Open Controls\" -> \"" << kBotonPausa
                  << "\"). R reinicia, Ctrl+C o \"Detener\" termina." << std::endl;
        if (op.dt < 1e-3) {
            std::cout << "AVISO: con dt = " << op.dt << " s la simulacion probablemente "
                      << "no alcance el tiempo real; el robot se vera en camara lenta."
                      << std::endl;
        }
    } else {
        std::cout << "Simulando..." << std::endl;
    }

    // --- Bucle de simulacion ---------------------------------------------------
    // Se avanza por tramos (cortos en vivo, para atender los controles de
    // Meshcat). Al llegar a t_log se escribe el CSV; desde ahi se vacian los
    // registros en cada tramo, para que la memoria no crezca mientras la
    // simulacion sigue.
    const double tramo = op.visualizar ? 0.02 : 1.0;
    bool guardado = false;
    bool en_pausa = op.visualizar;  // en vivo parte en pausa
    int clicks_pausa = 0, clicks_reiniciar = 0, clicks_detener = 0;
    double velocidad = 1.0;
    auto t_cpu0 = std::chrono::steady_clock::now();
    auto t_estado = t_cpu0;

    while (raiz.get_time() < op.t_final) {
        if (op.visualizar) {
            if (meshcat->GetButtonClicks(kBotonDetener) > clicks_detener) break;
            int c = meshcat->GetButtonClicks(kBotonReiniciar);
            if (c > clicks_reiniciar) {
                clicks_reiniciar = c;
                raiz.SetTimeStateAndParametersFrom(*ctx_inicial);
                if (!guardado) reg.Vaciar(&raiz);
                simulator.Initialize();
                en_pausa = true;
                std::cout << "\nReiniciado en t = 0 (en pausa)" << std::endl;
            }
            c = meshcat->GetButtonClicks(kBotonPausa);
            if (c > clicks_pausa) {
                clicks_pausa = c;
                en_pausa = !en_pausa;
                // Sin esto el simulador intentaria "recuperar" el tiempo pausado
                if (!en_pausa) simulator.ResetStatistics();
            }
            const double v = meshcat->GetSliderValue(kSliderVelocidad);
            if (v != velocidad) {
                velocidad = v;
                simulator.set_target_realtime_rate(velocidad);
                simulator.ResetStatistics();
            }
            if (en_pausa) {
                const auto ahora = std::chrono::steady_clock::now();
                if (ahora - t_estado > std::chrono::seconds(1)) {
                    t_estado = ahora;
                    std::cout << "\r  t = " << std::fixed << std::setprecision(1)
                              << raiz.get_time() << " s   [en pausa]                "
                              << std::flush;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                continue;
            }
        }

        double t_sig = std::min(raiz.get_time() + tramo, op.t_final);
        if (!guardado) t_sig = std::min(t_sig, op.t_log);
        simulator.AdvanceTo(t_sig);

        if (!guardado && raiz.get_time() >= op.t_log) {
            const double t_cpu = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_cpu0).count();
            GuardarYResumir(op, *etso, reg, raiz, t_cpu);
            guardado = true;
            if (op.visualizar) {
                std::cout << "\nLa simulacion sigue en Meshcat (ya no se registra). "
                          << "Ctrl+C o \"Detener\" para terminar." << std::endl;
            }
        }
        if (guardado) reg.Vaciar(&raiz);

        // Linea de estado cada ~1 s de reloj (solo en vivo)
        const auto ahora = std::chrono::steady_clock::now();
        if (op.visualizar && ahora - t_estado > std::chrono::seconds(1)) {
            t_estado = ahora;
            std::cout << "\r  t = " << std::fixed << std::setprecision(1)
                      << raiz.get_time() << " s   velocidad real x" << std::setprecision(2)
                      << simulator.get_actual_realtime_rate() << "   " << std::flush;
        }
    }
    if (op.visualizar) std::cout << "\nFin." << std::endl;
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
