#include <rigidbodies/physics/continuous_collision.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::physics
{
    SweepResult sweep_shapes(const ShapeSweep& first, const ShapeSweep& second, Real tolerance_m, int maximum_iterations)
    {
        if (!first.shape || !second.shape || !first.transform_at_fraction || !second.transform_at_fraction ||
            !math::is_finite(tolerance_m) || tolerance_m <= 0.0 || maximum_iterations < 1 || maximum_iterations > 1024 ||
            !math::is_finite(first.linear_displacement_m) || !math::is_finite(second.linear_displacement_m) ||
            !math::is_finite(first.nonlinear_travel_m) || !math::is_finite(second.nonlinear_travel_m) ||
            first.nonlinear_travel_m < 0.0 || second.nonlinear_travel_m < 0.0)
        {
            throw std::invalid_argument("A shape sweep requires finite motion bounds, positive tolerance and 1..1024 iterations");
        }
        SweepResult result;
        result.fraction = 0.0;
        const auto relative_translation = second.linear_displacement_m - first.linear_displacement_m;
        for (int iteration = 0; iteration < maximum_iterations; ++iteration)
        {
            result.iterations = iteration + 1;
            result.distance = convex_distance(*first.shape, first.transform_at_fraction(result.fraction), *second.shape, second.transform_at_fraction(result.fraction));
            if (!result.distance.valid)
            {
                result.converged = false;
                return result;
            }
            if (first.conservative_stop || second.conservative_stop)
            {
                result.hit = true;
                result.converged = false;
                return result;
            }
            if (result.distance.intersecting || result.distance.distance_m <= tolerance_m)
            {
                result.hit = true;
                return result;
            }
            const auto closing_bound = std::max(0.0, -math::dot(relative_translation, result.distance.normal)) +
                first.nonlinear_travel_m + second.nonlinear_travel_m;
            if (closing_bound <= 0.0)
                return result;
            const auto advance = (result.distance.distance_m - tolerance_m * 0.5) / closing_bound;
            if (advance > 1.0 - result.fraction)
                return result;
            if (advance <= 1.0e-12)
            {
                result.hit = true;
                result.converged = false;
                return result;
            }
            result.fraction = std::min(1.0, result.fraction + advance);
        }
        result.hit = true;
        result.converged = false;
        return result;
    }
}
