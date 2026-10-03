#pragma once

#include <rigidbodies/physics/constraint.hpp>

namespace rigidbodies::physics
{
    struct ConstraintGraphStatistics
    {
        std::size_t island_count {};
        std::size_t row_count {};
        Real velocity_residual {};
    };

    // Solves connected dynamic islands as bounded dense Jacobian systems. Fixed endpoints do
    // not merge unrelated islands. Intended for small demonstrative mechanisms; redundant rows
    // use relative diagonal regularization (1e-10), preserving deterministic finite solutions.
    // Rows and body IDs are borrowed only during this call; no world pointers survive it.
    ConstraintGraphStatistics solve_constraint_graph(World& world, const std::vector<ConstraintPtr>& constraints);
    ConstraintGraphStatistics measure_constraint_graph(World& world, const std::vector<ConstraintPtr>& constraints);
}
