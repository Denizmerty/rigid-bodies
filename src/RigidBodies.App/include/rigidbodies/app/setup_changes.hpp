#pragma once

#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/ui/run_types.hpp>

#include <vector>

namespace rigidbodies::app
{
    [[nodiscard]] std::vector<ui::SetupChange> compute_setup_changes(const physics::World& setup, const physics::World& original);
}
