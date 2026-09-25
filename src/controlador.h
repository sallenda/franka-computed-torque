#pragma once

#include "drake/systems/framework/diagram.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"

namespace ipre2026 {

class ControladorPanda : public drake::systems::Diagram<double> {
public:
    ControladorPanda();

    const drake::systems::InputPort<double>& get_estimated_state_input_port() const {
        return this->get_input_port(estimated_state_input_port_index_);
    }

    const drake::systems::InputPort<double>& get_desired_state_input_port() const {
        return this->get_input_port(desired_state_input_port_index_);
    }

    const drake::systems::OutputPort<double>& get_control_output_port() const {
        return this->get_output_port(control_output_port_index_);
    }

private:
    drake::systems::InputPortIndex estimated_state_input_port_index_;
    drake::systems::InputPortIndex desired_state_input_port_index_;
    drake::systems::OutputPortIndex control_output_port_index_;
};

} // namespace ipre2026