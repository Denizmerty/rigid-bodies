#pragma once

#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/ui/experiment_content.hpp>

namespace rigidbodies::app
{
    [[nodiscard]] ui::ExperimentContent parse_experiment_content(const physics::ScenarioDocument& document);
}
