#pragma once

#include <cstddef>
#include <string_view>

namespace rigidbodies::fuzz
{
    constexpr std::size_t maximum_input_bytes = 16384;

    struct DocumentCoverage
    {
        bool json_accepted {};
        bool semantic_budget_exceeded {};
        bool scenario_accepted {};
        bool shape_accepted {};
        bool world_rejection_checked {};
    };

    // Shared by the deterministic CTest mutations and the coverage-guided LLVM entry point.
    // Invariant violations escape as exceptions, including allocation failures: the harness
    // must not turn crashes or resource failures into ordinary document rejections.
    [[nodiscard]] DocumentCoverage exercise_document(std::string_view input);
}
