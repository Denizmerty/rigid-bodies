#pragma once

#include <rigidbodies/physics/convex_distance.hpp>

#include <functional>

namespace rigidbodies::physics
{
    // A convex shape's prescribed motion over one step. Translation is separated from the
    // remaining point-travel bound so conservative advancement can reject parallel motion.
    // nonlinear_travel_m bounds rotation and any curved translation, per unit step fraction.
    struct ShapeSweep
    {
        const Shape* shape { nullptr };
        std::function<math::Transform2(Real)> transform_at_fraction;
        math::Vec2 linear_displacement_m {};
        Real nonlinear_travel_m { 0.0 };
        // A finite broad-phase envelope exists, but the motion cannot be safely advanced.
        // Return an unresolved impact at the start so callers can stop and report the motion.
        bool conservative_stop { false };
    };

    struct SweepResult
    {
        bool hit { false };
        bool converged { true };
        Real fraction { 1.0 };
        ConvexDistanceResult distance;
        int iterations { 0 };
    };

    // Conservative advancement with a bounded amount of work. On exhaustion a conservative
    // candidate is returned rather than silently allowing a body to cross unresolved geometry.
    // The caller can form a speculative constraint and report the exhausted iteration budget.
    [[nodiscard]] SweepResult sweep_shapes(const ShapeSweep& first, const ShapeSweep& second,
        Real tolerance_m = 1.0e-6, int maximum_iterations = 64);
}
