#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"

namespace ipre2026 {

class PlantaComun : public drake::systems::Diagram<double> {
public:
    PlantaComun();

    const drake::multibody::MultibodyPlant<double>& get_plant() const { return *plant_; }
    const drake::geometry::SceneGraph<double>& get_scene_graph() const { return *scene_graph_; }

    const drake::systems::InputPort<double>& get_actuation_input_port() const {
        return this->get_input_port(actuation_input_port_index_);
    }

    const drake::systems::OutputPort<double>& get_state_output_port() const {
        return this->get_output_port(state_output_port_index_);
    }

    const drake::systems::OutputPort<double>& get_query_output_port() const {
        return this->get_output_port(query_output_port_index_);
    }

private:
    drake::multibody::MultibodyPlant<double>* plant_{nullptr};
    drake::geometry::SceneGraph<double>* scene_graph_{nullptr};

    drake::systems::InputPortIndex actuation_input_port_index_;
    drake::systems::OutputPortIndex state_output_port_index_;
    drake::systems::OutputPortIndex query_output_port_index_;
};

} // namespace ipre2026
