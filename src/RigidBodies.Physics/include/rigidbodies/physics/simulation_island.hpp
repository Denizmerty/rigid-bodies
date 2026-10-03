#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <cstddef>
#include <vector>

namespace rigidbodies::physics
{
    // Dynamic bodies connected by solid contacts, active joints, or enabled springs.
    // Static/kinematic endpoints are boundaries and can be shared by independent islands.
    // Members and work indices retain canonical world order. Rebuilt each collision step.
    struct SimulationIsland
    {
        std::vector<BodyId> bodies;
        std::vector<std::size_t> manifold_indices;
        std::vector<std::size_t> constraint_indices;
        bool awake { false };
        // Custom world-mutating solvers/constraints retain their original whole-world ordering.
        bool serial_fallback { false };
    };
}
