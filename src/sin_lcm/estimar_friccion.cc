/// @file
///
/// Estima la friccion viscosa B_i de cada joint del brazo a partir de un log
/// con columnas t, q1..q9, dq1..dq9, tau1..tau9 (el formato de log.csv que
/// escribe `experimento`; un log del Franka real con esos nombres sirve igual).
///
/// Modelo (ec. 1 del paper):  J(q) q̈ + B q̇ + c(q,q̇) + g(q) = tau
/// Con el modelo rigido SIN friccion se calcula el residuo
///     y = tau - [J(q) q̈ + c(q,q̇) + g(q)]
/// y por minimos cuadrados, joint por joint,  y_i ≈ B_i q̇_i  (+ Fc_i sign(q̇_i)
/// con --coulomb). q̈ se obtiene por diferencias centrales de q̇.
///
/// Importante: el residuo absorbe TODO lo que el modelo no explica (errores
/// de masa/inercia, ruido, friccion no lineal). En simulacion recupera el B de
/// la planta; en el robot real el resultado es tan bueno como el modelo rigido.
/// Necesita excitacion: el joint debe moverse (usa --referencia=excitacion).
///
/// Uso:
///   ./build/src/sin_lcm/estimar_friccion resultados/excitacion/log.csv [--coulomb]

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"

#include "controlador.h"
#include "modelo_panda.h"

namespace ipre2026 {
namespace {

struct Log {
    std::vector<double> t;
    Eigen::MatrixXd q, dq, tau;  // (muestras x 9)
};

Log LeerLog(const std::string& ruta) {
    std::ifstream f(ruta);
    if (!f) throw std::runtime_error("no se pudo abrir " + ruta);
    std::string linea;
    std::getline(f, linea);
    std::unordered_map<std::string, int> col;
    {
        std::stringstream ss(linea);
        std::string nombre;
        for (int c = 0; std::getline(ss, nombre, ','); ++c) col[nombre] = c;
    }
    auto indice = [&](const std::string& nombre) {
        auto it = col.find(nombre);
        if (it == col.end()) throw std::runtime_error("falta la columna " + nombre);
        return it->second;
    };
    const int n = kNumJointsTotal;
    std::vector<int> iq(n), idq(n), itau(n);
    for (int i = 0; i < n; ++i) {
        iq[i] = indice("q" + std::to_string(i + 1));
        idq[i] = indice("dq" + std::to_string(i + 1));
        itau[i] = indice("tau" + std::to_string(i + 1));
    }
    const int it = indice("t");

    std::vector<std::vector<double>> filas;
    while (std::getline(f, linea)) {
        if (linea.empty()) continue;
        std::vector<double> fila;
        std::stringstream ss(linea);
        std::string celda;
        while (std::getline(ss, celda, ',')) fila.push_back(std::stod(celda));
        filas.push_back(std::move(fila));
    }
    Log log;
    const int m = static_cast<int>(filas.size());
    log.q.resize(m, n); log.dq.resize(m, n); log.tau.resize(m, n);
    for (int s = 0; s < m; ++s) {
        log.t.push_back(filas[s][it]);
        for (int i = 0; i < n; ++i) {
            log.q(s, i) = filas[s][iq[i]];
            log.dq(s, i) = filas[s][idq[i]];
            log.tau(s, i) = filas[s][itau[i]];
        }
    }
    return log;
}

int DoMain(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "uso: estimar_friccion <log.csv> [--coulomb]\n";
        return 1;
    }
    const std::string ruta = argv[1];
    const bool coulomb = (argc > 2 && std::string(argv[2]) == "--coulomb");

    const Log log = LeerLog(ruta);
    const int m = static_cast<int>(log.t.size());
    const int n = kNumJointsTotal;
    if (m < 3) throw std::runtime_error("log demasiado corto");

    // Modelo rigido sin friccion (B = 0): la friccion es lo que se quiere estimar.
    auto modelo = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.0);
    AgregarPanda(modelo.get(), Eigen::VectorXd::Zero(kNumJointsBrazo));
    modelo->Finalize();
    auto ctx = modelo->CreateDefaultContext();

    // Acumuladores de las ecuaciones normales por joint (1 o 2 parametros)
    const int p = coulomb ? 2 : 1;
    std::vector<Eigen::MatrixXd> AtA(kNumJointsBrazo, Eigen::MatrixXd::Zero(p, p));
    std::vector<Eigen::VectorXd> Aty(kNumJointsBrazo, Eigen::VectorXd::Zero(p));
    std::vector<double> yy(kNumJointsBrazo, 0.0), sum_dq2(kNumJointsBrazo, 0.0);
    int usadas = 0;

    Eigen::MatrixXd M(n, n);
    Eigen::VectorXd Cv(n);
    for (int s = 1; s + 1 < m; ++s) {
        const double h = log.t[s + 1] - log.t[s - 1];
        if (h <= 0) continue;
        const Eigen::VectorXd q = log.q.row(s).transpose();
        const Eigen::VectorXd dq = log.dq.row(s).transpose();
        const Eigen::VectorXd ddq = (log.dq.row(s + 1) - log.dq.row(s - 1)).transpose() / h;
        const Eigen::VectorXd tau = log.tau.row(s).transpose();

        modelo->SetPositions(ctx.get(), q);
        modelo->SetVelocities(ctx.get(), dq);
        modelo->CalcMassMatrix(*ctx, &M);
        modelo->CalcBiasTerm(*ctx, &Cv);
        const Eigen::VectorXd g = -modelo->CalcGravityGeneralizedForces(*ctx);
        const Eigen::VectorXd y = tau - (M * ddq + Cv + g);

        for (int i = 0; i < kNumJointsBrazo; ++i) {
            Eigen::VectorXd a(p);
            a[0] = dq[i];
            if (coulomb) a[1] = (dq[i] > 0) - (dq[i] < 0);
            AtA[i] += a * a.transpose();
            Aty[i] += a * y[i];
            yy[i] += y[i] * y[i];
            sum_dq2[i] += dq[i] * dq[i];
        }
        ++usadas;
    }

    std::cout << "Estimacion de friccion con " << usadas << " muestras de " << ruta
              << "\n joint   B [Nms/rad]" << (coulomb ? "   Fc [Nm]" : "")
              << "   R^2     rms(dq) [rad/s]\n";
    for (int i = 0; i < kNumJointsBrazo; ++i) {
        const double rms = std::sqrt(sum_dq2[i] / std::max(usadas, 1));
        std::cout << std::setw(6) << i + 1;
        if (rms < 0.02) {
            std::cout << "   sin excitacion suficiente (el joint casi no se mueve)\n";
            continue;
        }
        const Eigen::VectorXd theta = AtA[i].ldlt().solve(Aty[i]);
        // R^2 = 1 - SSres/SStot (sin centrar, el modelo no tiene termino constante)
        const double ss_res = yy[i] - 2 * theta.dot(Aty[i]) + theta.dot(AtA[i] * theta);
        const double r2 = 1.0 - ss_res / std::max(yy[i], 1e-12);
        std::cout << std::fixed << std::setprecision(3) << std::setw(14) << theta[0];
        if (coulomb) std::cout << std::setw(10) << theta[1];
        std::cout << std::setw(8) << r2 << std::setw(14) << rms << "\n";
    }
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
