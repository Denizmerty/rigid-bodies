#pragma once

#include <string>
#include <string_view>
#include <optional>

namespace rigidbodies::core
{
    // Display conversion only: every simulation input and stored quantity remains SI.
    enum class DisplayUnits
    {
        si,
        centimetre_gram
    };

    enum class DisplayQuantity
    {
        time,
        length,
        mass,
        velocity,
        acceleration,
        angle,
        angular_velocity,
        force,
        momentum,
        energy,
        torque,
        inertia,
        density,
        air_density,
        viscosity,
        stiffness,
        damping,
        twist_stiffness,
        twist_damping,
        power,
        coefficient,
        multiplier,
        percentage,
        count,
        // A ratio stored as a fraction (1.0) and presented as a percentage (100 %).
        scale,
        // A small length such as a drawing tolerance, always in one unit (mm, or cm in CGS) so a
        // group of tolerances reads alike and a value never changes unit while it is dragged.
        fine_length,
        // v/c with unit "c", shown with its leading nines and truncated towards rest, so it never reads as c.
        speed_fraction,
        // The value is γ − 1, so tiny excesses keep their digits; the text is γ, with no unit.
        lorentz_factor_excess,
        // A speed up to c: whole m/s (cm/s) from 1 000, truncated, so a speed below c never reads as c.
        relativistic_speed,
        // How far a speed is below c: whole m/s (cm/s) from 1 000, rounded up, so the gap is never understated.
        speed_gap,
        // Energy that may reach 10²¹ J: one base unit, "d.dd × 10ⁿ" outside [0.001, 10 000).
        relativistic_energy,
        // Momentum likewise, in kg·m/s (g·cm/s).
        relativistic_momentum,
        // A clock reading, always in nanoseconds so lab and probe clocks compare digit for digit.
        fine_time,
        // A short interval: ns from 1 ns, then ps, fs, and "d.dd × 10ⁿ s" below 1 fs.
        duration,
        // A small length in the unit that suits it: m, mm, µm, nm (cm in CGS).
        small_length
    };

    // Mirrors physics::speed_of_light_m_s (Core cannot include Physics); relativity_stage.cpp asserts equality.
    inline constexpr double speed_of_light_m_s = 299792458.0;

    [[nodiscard]] double display_factor(DisplayQuantity quantity, DisplayUnits units);
    [[nodiscard]] std::string_view display_unit(DisplayQuantity quantity, DisplayUnits units);
    [[nodiscard]] double display_value(double value_si, DisplayQuantity quantity, DisplayUnits units);

    // Returns only the figure. A negative precision requests the quantity's read-out default;
    // angle fields pass 1 to retain their editable tenth-degree precision.
    [[nodiscard]] std::string format_value(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision = -1);
    // The unit format_value chose for this value (mm below a centimetre, g below ten grams and so
    // on). An editable field shows it, and reads unitless input in it.
    [[nodiscard]] std::string format_unit(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision = -1);
    [[nodiscard]] std::string format_quantity(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision = -1);
    // Three (or `digits`) significant figures in fixed notation for magnitudes in [0.001, 1 000 000),
    // "d.dd × 10ⁿ" outside; U+2212 minus; no negative zero; "0" for zero.
    [[nodiscard]] std::string format_significant(double value, int digits = 3);
    // The decimals a speed fraction's text is worked to before its trailing zeros are trimmed:
    // three significant figures below ½ c, then at least two figures of the gap 1 − β, so 0.99999
    // is worked to six and its last digit is a millionth. 3 outside (0, 1).
    [[nodiscard]] int speed_fraction_decimals(double fraction);

    // Parses learner-entered text into SI. A bare number uses the unit currently displayed.
    // Units from either display system are accepted; a mismatched or malformed unit is rejected.
    [[nodiscard]] std::optional<double> parse_quantity(std::string_view text, DisplayQuantity quantity, DisplayUnits displayed_units);
}
