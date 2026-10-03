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
        fine_length
    };

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

    // Parses learner-entered text into SI. A bare number uses the unit currently displayed.
    // Units from either display system are accepted; a mismatched or malformed unit is rejected.
    [[nodiscard]] std::optional<double> parse_quantity(std::string_view text, DisplayQuantity quantity, DisplayUnits displayed_units);
}
