#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include "modelo_panda.h"

namespace ipre2026 {

// -----------------------------------------------------------------------------
// Referencias analiticas (q_d, dq_d, ddq_d). Unica fuente de verdad para el
// experimento sin LCM y para el publicador de trayectoria con LCM, de modo que
// la amplitud que usa el diseno ETSO coincide con la que se publica.
// -----------------------------------------------------------------------------

// Referencia actual (en standby, sin cambios respecto de trayectoria.cc):
// el joint 1 oscila +-kAmplitudRef con frecuencia kFrecuenciaRefHz alrededor del home.
constexpr double kAmplitudRef = 0.4;      // [rad]
constexpr double kFrecuenciaRefHz = 0.1;  // periodo de 10 s

enum class TipoReferencia {
    kSeno,        // la de arriba
    kExcitacion,  // todos los joints, para identificar friccion (ver estimar_friccion)
};

inline TipoReferencia ReferenciaDesdeTexto(const std::string& s) {
    if (s == "seno") return TipoReferencia::kSeno;
    if (s == "excitacion") return TipoReferencia::kExcitacion;
    throw std::runtime_error("referencia desconocida: " + s + " (seno|excitacion)");
}

struct EstadoDeseado {
    Eigen::VectorXd q, dq, ddq;  // tamano kNumJointsTotal
};

inline EstadoDeseado EvaluarReferencia(TipoReferencia tipo, double t) {
    EstadoDeseado r;
    r.q = PoseHome();
    r.dq = Eigen::VectorXd::Zero(kNumJointsTotal);
    r.ddq = Eigen::VectorXd::Zero(kNumJointsTotal);

    switch (tipo) {
        case TipoReferencia::kSeno: {
            const double w = 2 * std::numbers::pi * kFrecuenciaRefHz;
            r.q[0] += kAmplitudRef * std::sin(w * t);
            r.dq[0] = kAmplitudRef * w * std::cos(w * t);
            r.ddq[0] = -kAmplitudRef * w * w * std::sin(w * t);
            break;
        }
        case TipoReferencia::kExcitacion: {
            // Forma del paper, q_d = A(1 - cos(w t)): parte con velocidad cero.
            // Frecuencias distintas por joint para que las velocidades no esten
            // correlacionadas (mejor condicionamiento de los minimos cuadrados).
            const double A = 0.3;
            for (int i = 0; i < kNumJointsBrazo; ++i) {
                const double w = 0.6 + 0.15 * i;  // [rad/s]
                // Joint 4 esta cerca de su limite superior (-0.07): se mueve hacia abajo.
                const double s = (i == 3) ? -1.0 : 1.0;
                r.q[i] += s * A * (1.0 - std::cos(w * t));
                r.dq[i] = s * A * w * std::sin(w * t);
                r.ddq[i] = s * A * w * w * std::cos(w * t);
            }
            break;
        }
    }
    return r;
}

}  // namespace ipre2026
