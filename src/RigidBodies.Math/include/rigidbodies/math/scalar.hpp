#pragma once

#include <cmath>

namespace rigidbodies::math
{

    // The simulation works in SI units at household scale, where double precision keeps
    // accumulated integration error far below anything a viewer can perceive. Rendering
    // narrows to float only at the device boundary.
    using Real = double;

    inline constexpr Real pi = 3.1415926535897932384626433832795;
    inline constexpr Real two_pi = 2.0 * pi;
    inline constexpr Real half_pi = 0.5 * pi;

    // Tolerance used for geometric predicates expressed in metres. Contacts closer than this
    // are treated as coincident rather than as separate features.
    inline constexpr Real geometric_epsilon = 1.0e-9;

    [[nodiscard]] inline constexpr Real degrees_to_radians(Real degrees)
    {
        return degrees * (pi / 180.0);
    }

    [[nodiscard]] inline constexpr Real radians_to_degrees(Real radians)
    {
        return radians * (180.0 / pi);
    }

    [[nodiscard]] inline constexpr Real clamp(Real value, Real minimum, Real maximum)
    {
        if (value < minimum)
        {
            return minimum;
        }
        if (value > maximum)
        {
            return maximum;
        }
        return value;
    }

    [[nodiscard]] inline constexpr Real lerp(Real start, Real end, Real fraction)
    {
        return start + (end - start) * fraction;
    }

    [[nodiscard]] inline bool nearly_equal(Real left, Real right, Real tolerance = geometric_epsilon)
    {
        return std::abs(left - right) <= tolerance;
    }

    [[nodiscard]] inline bool is_finite(Real value)
    {
        return std::isfinite(value);
    }

    // Wraps an angle into (-pi, pi]. Orientation is stored unwrapped in the body state, so this
    // is applied when an angle is reported or compared rather than during integration.
    [[nodiscard]] inline Real wrap_angle(Real radians)
    {
        const auto wrapped = std::remainder(radians, two_pi);
        return wrapped <= -pi ? wrapped + two_pi : wrapped;
    }

} // namespace rigidbodies::math
