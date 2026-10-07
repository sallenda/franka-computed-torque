#include "modelo_panda.h"

#include <stdexcept>
#include <string>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace ipre2026 {

using drake::math::RigidTransformd;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;

ModelInstanceIndex AgregarPanda(MultibodyPlant<double>* plant,
                                const Eigen::VectorXd& damping) {
    if (damping.size() != kNumJointsBrazo) {
        throw std::runtime_error("[AgregarPanda] damping debe tener tamano 7");
    }
    Parser parser(plant);
    const auto modelos = parser.AddModelsFromUrl(
        "package://drake_models/franka_description/urdf/panda_arm_hand.urdf");
    const ModelInstanceIndex panda = modelos.at(0);
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("panda_link0", panda),
                      RigidTransformd{});

    for (int i = 0; i < kNumJointsBrazo; ++i) {
        auto& joint = plant->GetMutableJointByName<RevoluteJoint>(
            "panda_joint" + std::to_string(i + 1), panda);
        joint.set_default_damping(damping[i]);
    }
    return panda;
}

}  // namespace ipre2026
