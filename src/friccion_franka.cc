#include "friccion_franka.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "drake/multibody/tree/revolute_joint.h"

namespace ipre2026 {

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::systems::BasicVector;
using drake::systems::Context;

Eigen::Matrix<double, kNumJointsFriccion, 3> ParametrosFriccionGaz() {
    Eigen::Matrix<double, kNumJointsFriccion, 3> phi;
    //        phi1 [Nm]    phi2 [s/rad]  phi3 [rad/s]
    phi << 5.4615e-01,  5.1181e+00,  3.9533e-02,   // joint 1
           8.7224e-01,  9.0657e+00,  2.5882e-02,   // joint 2
           6.4068e-01,  1.0136e+01, -4.6070e-02,   // joint 3
           1.2794e+00,  5.5903e+00,  3.6194e-02,   // joint 4
           8.3904e-01,  8.3469e+00,  2.6226e-02,   // joint 5
           3.0301e-01,  1.7133e+01, -2.1047e-02,   // joint 6
           5.6489e-01,  1.0336e+01,  3.5526e-03;   // joint 7
    return phi;
}

namespace {
double Sigmoide(double x) { return 1.0 / (1.0 + std::exp(-x)); }
constexpr int kMuestrasVelocidad = 2001;  // discretizacion para ajuste y cota
}  // namespace

double TorqueFriccion(int j, double dq) {
    static const auto phi = ParametrosFriccionGaz();
    const double p1 = phi(j, 0), p2 = phi(j, 1), p3 = phi(j, 2);
    return p1 * Sigmoide(p2 * (dq + p3)) - p1 * Sigmoide(p2 * p3);
}

double BEquivalente(int j, double v_ajuste) {
    // B = sum(v tau_f(v)) / sum(v^2) sobre una grilla simetrica en [-v, v]
    double num = 0.0, den = 0.0;
    for (int k = 0; k < kMuestrasVelocidad; ++k) {
        const double v = -v_ajuste + 2.0 * v_ajuste * k / (kMuestrasVelocidad - 1);
        num += v * TorqueFriccion(j, v);
        den += v * v;
    }
    return num / den;
}

double ResiduoFriccionMax(int j, double B, double v_cota) {
    double r = 0.0;
    for (int k = 0; k < kMuestrasVelocidad; ++k) {
        const double v = -v_cota + 2.0 * v_cota * k / (kMuestrasVelocidad - 1);
        r = std::max(r, std::abs(TorqueFriccion(j, v) - B * v));
    }
    return r;
}

FriccionFranka::FriccionFranka(const MultibodyPlant<double>& plant,
                               ModelInstanceIndex panda) {
    if (!plant.is_finalized()) {
        throw std::runtime_error("[FriccionFranka] la planta debe estar finalizada");
    }
    nq_ = plant.num_positions();
    nv_ = plant.num_velocities();
    for (int j = 0; j < kNumJointsFriccion; ++j) {
        const auto& joint = plant.GetJointByName<RevoluteJoint>(
            "panda_joint" + std::to_string(j + 1), panda);
        indice_v_[j] = joint.velocity_start();
    }
    DeclareVectorInputPort("plant_state", nq_ + nv_);
    DeclareVectorOutputPort("generalized_force", nv_, &FriccionFranka::CalcFuerza);
}

void FriccionFranka::CalcFuerza(const Context<double>& context,
                                BasicVector<double>* output) const {
    const auto& x = get_input_port(0).Eval(context);
    const auto v = x.tail(nv_);
    // Fuerza aplicada = -tau_f: la friccion se opone al movimiento.
    Eigen::VectorXd f = Eigen::VectorXd::Zero(nv_);
    for (int j = 0; j < kNumJointsFriccion; ++j) {
        f[indice_v_[j]] = -TorqueFriccion(j, v[indice_v_[j]]);
    }
    output->SetFromVector(f);
}

}  // namespace ipre2026
