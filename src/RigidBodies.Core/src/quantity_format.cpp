#include <rigidbodies/core/display_units.hpp>

#include <rigidbodies/core/text_format.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <cstdlib>
#include <cctype>

namespace rigidbodies::core
{
    namespace
    {
        constexpr std::string_view minus_sign = "\xE2\x88\x92";
        constexpr std::string_view thin_space = "\xE2\x80\x89";
        constexpr std::string_view no_break_space = "\xC2\xA0";
        constexpr std::string_view em_dash = "\xE2\x80\x94";

        std::string fixed_ascii(double value, int decimals)
        {
            const auto threshold = 0.5 * std::pow(10.0, -decimals);
            if (std::abs(value) < threshold)
                value = 0.0;
            std::ostringstream stream;
            stream.imbue(std::locale::classic());
            stream << std::fixed << std::setprecision(decimals) << value;
            return stream.str();
        }

        std::string trim_fraction(std::string value)
        {
            const auto dot = value.find('.');
            if (dot == std::string::npos)
                return value;
            while (!value.empty() && value.back() == '0')
                value.pop_back();
            if (!value.empty() && value.back() == '.')
                value.pop_back();
            return value;
        }

        // Numbers group from five integer digits (7850, 12 400); `grouped_from` lowers that for
        // readings that must line up with longer grouped numbers.
        std::string group_and_sign(std::string value, std::size_t grouped_from = 5)
        {
            const auto negative = !value.empty() && value.front() == '-';
            if (negative)
                value.erase(value.begin());
            const auto dot = value.find('.');
            const auto integer_digits = dot == std::string::npos ? value.size() : dot;
            if (integer_digits >= grouped_from)
            {
                for (std::size_t index = integer_digits - 3; index > 0; index -= std::min<std::size_t>(index, 3))
                    value.insert(index, thin_space);
            }
            if (negative && value != "0")
                value.insert(0, minus_sign);
            return value;
        }

        std::string fixed_number(double value, int decimals, bool trim = false)
        {
            auto result = fixed_ascii(value, decimals);
            if (trim)
                result = trim_fraction(std::move(result));
            return group_and_sign(std::move(result));
        }

        int significant_decimals(double value, int digits)
        {
            const auto magnitude = std::abs(value);
            if (magnitude == 0.0)
                return std::max(digits - 1, 0);
            return std::max(0, digits - 1 - static_cast<int>(std::floor(std::log10(magnitude))));
        }

        std::string significant(double value, int digits = 3, bool trim = false)
        {
            return fixed_number(value, significant_decimals(value, digits), trim);
        }

        // As significant_decimals, but rounding that carries into the next power of ten keeps the
        // count of figures (0.9996 reads 1.00, not 1.000).
        int carried_decimals(double value, int digits)
        {
            auto decimals = significant_decimals(value, digits);
            if (decimals > 0 && std::round(std::abs(value) * std::pow(10.0, decimals)) >= std::pow(10.0, digits))
                --decimals;
            return decimals;
        }

        std::string carried_significant(double value, int digits = 3)
        {
            return fixed_number(value, carried_decimals(value, digits));
        }

        std::string superscript(int exponent)
        {
            static constexpr std::string_view digits[] = { "\xE2\x81\xB0", "\xC2\xB9", "\xC2\xB2", "\xC2\xB3", "\xE2\x81\xB4", "\xE2\x81\xB5", "\xE2\x81\xB6", "\xE2\x81\xB7", "\xE2\x81\xB8", "\xE2\x81\xB9" };
            std::string result;
            if (exponent < 0)
            {
                result += "\xE2\x81\xBB";
                exponent = -exponent;
            }
            const auto text = std::to_string(exponent);
            for (const auto character : text)
                result += digits[static_cast<std::size_t>(character - '0')];
            return result;
        }

        std::string mantissa(double value, int digits = 3)
        {
            if (value == 0.0)
                return "0";
            auto exponent = static_cast<int>(std::floor(std::log10(std::abs(value))));
            // Below about 10⁻³⁰⁸ the power of ten itself underflows to zero, so scale in two factors.
            auto scaled = exponent < -300 ? value * 1.0e300 / std::pow(10.0, exponent + 300) : value / std::pow(10.0, exponent);
            const auto decimals = std::max(digits - 1, 0);
            // Rounding can carry into a new digit (9.9996 to 10.00), which belongs to the next power.
            const auto scale = std::pow(10.0, decimals);
            if (std::round(std::abs(scaled) * scale) / scale >= 10.0)
            {
                ++exponent;
                scaled /= 10.0;
            }
            return fixed_number(scaled, decimals) + " \xC3\x97 10" + superscript(exponent);
        }

        // `digits` significant figures in fixed notation from `lower` up to where rounding reaches
        // `upper` (9999.7 J is not "10 000 J"), "d.dd × 10ⁿ" outside.
        std::string significant_within(double value, double lower, double upper, int digits = 3)
        {
            if (value == 0.0)
                return "0";
            const auto magnitude = std::abs(value);
            const auto decimals = carried_decimals(value, digits);
            const auto scale = std::pow(10.0, decimals);
            if (magnitude < lower || std::round(magnitude * scale) / scale >= upper)
                return mantissa(value, digits);
            return fixed_number(value, decimals);
        }

        // v/c cut towards rest. From ½ c the digits come from the exact gap 1 − β with at least
        // two figures of it, so the text stops at 0.99…990 and can never read as c.
        std::string speed_fraction_text(double fraction)
        {
            if (fraction < 0.0)
                return std::string { minus_sign } + speed_fraction_text(-fraction);
            if (fraction == 0.0)
                return "0";
            const auto decimals = speed_fraction_decimals(fraction);
            std::string digits;
            if (fraction < 0.5)
            {
                // Two factors keep the power of ten finite for the smallest fractions.
                const auto scaled = decimals > 300 ? fraction * 1.0e300 * std::pow(10.0, decimals - 300) : fraction * std::pow(10.0, decimals);
                digits = std::to_string(static_cast<long long>(std::floor(scaled + 1.0e-6)));
            }
            else
            {
                const auto gap = 1.0 - fraction; // exact for fraction >= 0.5 (Sterbenz)
                long long power = 1;
                for (int index = 0; index < decimals; ++index)
                    power *= 10;
                const auto shortfall = static_cast<long long>(std::ceil(gap * static_cast<double>(power) - 1.0e-6));
                digits = std::to_string(power - shortfall);
            }
            if (digits.size() < static_cast<std::size_t>(decimals))
                digits.insert(0, static_cast<std::size_t>(decimals) - digits.size(), '0');
            while (!digits.empty() && digits.back() == '0')
                digits.pop_back();
            return digits.empty() ? std::string("0") : "0." + digits;
        }

        std::string lorentz_factor_text(double excess)
        {
            if (excess < 0.0)
                return std::string { em_dash };
            if (excess == 0.0)
                return "1";
            if (excess < 1.0e-6)
                return "1 + " + mantissa(excess);
            if (excess < 0.1)
                return fixed_number(1.0 + excess, 1 - static_cast<int>(std::floor(std::log10(excess))));
            return significant(1.0 + excess, 4, true);
        }

        struct Figure
        {
            std::string value;
            std::string_view unit;
            bool tight_unit {};
        };

        Figure prefixed_si(double value, std::string_view base, std::string_view milli, std::string_view kilo)
        {
            const auto magnitude = std::abs(value);
            if (magnitude < 1.0e-5)
                return { "0", base };
            if (magnitude >= 10000.0)
                return { significant(value / 1000.0), kilo };
            if (magnitude < 0.01)
                return { significant(value * 1000.0), milli };
            return { significant(value), base };
        }

        Figure cgs_large(double value, std::string_view unit, double zero_threshold)
        {
            if (std::abs(value) < zero_threshold)
                return { "0", unit };
            if (std::abs(value) >= 1000000.0)
                return { mantissa(value), unit };
            return { significant(value), unit };
        }

        struct UnitStep
        {
            double size;
            std::string_view unit;
        };

        // Three figures in the largest unit the value reaches once rounded (0.9996 mm reads 1.00 mm,
        // not 1000 µm); below the smallest unit, "d.dd × 10ⁿ" in the base unit.
        Figure stepped_unit(double value, std::initializer_list<UnitStep> steps, std::string_view base, double base_size)
        {
            const auto magnitude = std::abs(value);
            for (const auto& step : steps)
            {
                if (magnitude >= 0.9995 * step.size)
                    return { carried_significant(value / step.size), step.unit };
            }
            return { mantissa(value / base_size), base };
        }

        // Whole units from 1 000, every thousand grouped so a speed and its gap below c line up with
        // c digit for digit; three figures below.
        Figure whole_speed(double value_si, bool cgs, bool round_up)
        {
            const auto speed = value_si * (cgs ? 100.0 : 1.0);
            const std::string_view unit = cgs ? "cm/s" : "m/s";
            if (speed == 0.0)
                return { "0", unit };
            const auto magnitude = std::abs(speed);
            // A whole number that arrives a rounding error away (the gap below 0.99 c computes as
            // 299 792 458.0000003 cm/s) counts as that number, so the speed and the gap add up to c.
            // The error comes from storing β, so it scales with c even when the gap is small.
            const auto light = speed_of_light_m_s * (cgs ? 100.0 : 1.0);
            const auto noise = std::max(magnitude, light) * 1.0e-12;
            // Three figures that would round to 1000 (999.7) go whole, so they are grouped and a
            // speed is still cut.
            if (std::round(magnitude) < 1000.0)
            {
                if (!round_up)
                    return { carried_significant(speed), unit };
                // A gap is rounded up in its last figure too, so it is never understated: beside a
                // speed cut to whole units, 869.4 m/s reads 870 and the two add up to c. One that
                // rounds up to 1000 is grouped as a whole number below.
                const auto scale = std::pow(10.0, significant_decimals(magnitude, 3));
                const auto raised = std::ceil(magnitude * scale - noise * scale) / scale;
                if (raised < 1000.0)
                    return { carried_significant(std::copysign(raised, speed)), unit };
            }
            auto whole = round_up ? std::ceil(magnitude - noise) : std::floor(magnitude + noise);
            // That allowance never lifts a speed below c to c's own digits.
            if (!round_up && magnitude < light)
                whole = std::min(whole, light - 1.0);
            return { group_and_sign(fixed_ascii(std::copysign(whole, speed), 0), 4), unit };
        }

        Figure duration_figure(double seconds)
        {
            if (seconds == 0.0)
                return { "0", "ns" };
            // From the point where picoseconds would round to 1000, two decimals of a nanosecond,
            // grouped from four digits as the clock readings beside it are (3 599.46 ns under
            // 3 600.00 ns).
            if (std::abs(seconds) >= 0.9995e-9)
                return { group_and_sign(fixed_ascii(seconds * 1.0e9, 2), 4), "ns" };
            return stepped_unit(seconds, { { 1.0e-12, "ps" }, { 1.0e-15, "fs" } }, "s", 1.0);
        }

        Figure formatted(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision)
        {
            const auto cgs = units == DisplayUnits::centimetre_gram;
            switch (quantity)
            {
            case DisplayQuantity::time:
                if (std::abs(value_si) >= 3600.0)
                {
                    const auto hours = static_cast<long long>(std::abs(value_si) / 3600.0);
                    const auto minutes = static_cast<long long>(std::fmod(std::abs(value_si), 3600.0) / 60.0);
                    return { (value_si < 0.0 ? std::string { minus_sign } : std::string {}) + std::to_string(hours) + " h " + (minutes < 10 ? "0" : "") + std::to_string(minutes), "min" };
                }
                if (std::abs(value_si) >= 1000.0)
                {
                    const auto minutes = static_cast<long long>(std::abs(value_si) / 60.0);
                    const auto seconds = static_cast<long long>(std::fmod(std::abs(value_si), 60.0));
                    return { (value_si < 0.0 ? std::string { minus_sign } : std::string {}) + std::to_string(minutes) + " min " + std::to_string(seconds), "s" };
                }
                return { fixed_number(value_si, 2), "s" };
            case DisplayQuantity::mass:
                if (cgs)
                    return { significant(value_si * 1000.0), "g" };
                if (std::abs(value_si) < 0.01)
                    return { fixed_number(value_si * 1000.0, 1), "g" };
                return { significant(value_si), "kg" };
            case DisplayQuantity::length:
                if (cgs)
                    return { std::abs(value_si) >= 0.01 ? fixed_number(value_si * 100.0, 1) : significant(value_si * 100.0), "cm" };
                // Zero reads in the scene's metres; only a small length that is there needs millimetres.
                if (value_si != 0.0 && std::abs(value_si) < 0.01)
                    return { fixed_number(value_si * 1000.0, 1), "mm" };
                return { std::abs(value_si) >= 0.1 || value_si == 0.0 ? fixed_number(value_si, 2) : significant(value_si), "m" };
            case DisplayQuantity::fine_length:
                return cgs ? Figure { fixed_number(value_si * 100.0, 2), "cm" } : Figure { fixed_number(value_si * 1000.0, 1), "mm" };
            case DisplayQuantity::velocity:
                return { fixed_number(value_si * (cgs ? 100.0 : 1.0), cgs ? 1 : 2), cgs ? "cm/s" : "m/s" };
            case DisplayQuantity::acceleration:
                return { fixed_number(value_si * (cgs ? 100.0 : 1.0), cgs ? 0 : 2), cgs ? "cm/s\xC2\xB2" : "m/s\xC2\xB2" };
            case DisplayQuantity::angle:
                return { fixed_number(value_si, precision < 0 ? 0 : precision), "\xC2\xB0", true };
            case DisplayQuantity::angular_velocity:
                return { fixed_number(value_si, 2), "rad/s" };
            case DisplayQuantity::energy:
                return cgs ? cgs_large(value_si * 1.0e7, "erg", 100.0) : prefixed_si(value_si, "J", "mJ", "kJ");
            case DisplayQuantity::force:
                return cgs ? cgs_large(value_si * 1.0e5, "dyn", 1.0) : prefixed_si(value_si, "N", "mN", "kN");
            case DisplayQuantity::torque:
                return cgs ? cgs_large(value_si * 1.0e7, "dyn\xC2\xB7"
                                                         "cm",
                                 100.0)
                           : prefixed_si(value_si, "N\xC2\xB7m", "mN\xC2\xB7m", "kN\xC2\xB7m");
            case DisplayQuantity::momentum:
                return cgs ? cgs_large(value_si * 1.0e5, "g\xC2\xB7"
                                                         "cm/s",
                                 1.0)
                           : prefixed_si(value_si, "kg\xC2\xB7m/s", "g\xC2\xB7m/s", "kN\xC2\xB7s");
            case DisplayQuantity::inertia:
                if (cgs)
                    return { significant(value_si * 1.0e7), "g\xC2\xB7"
                                                            "cm\xC2\xB2" };
                return { std::abs(value_si) < 0.001 && value_si != 0.0 ? mantissa(value_si) : significant(value_si), "kg\xC2\xB7m\xC2\xB2" };
            case DisplayQuantity::density:
                return { cgs ? significant(value_si * 0.001) : fixed_number(value_si, 0), cgs ? "g/cm\xC2\xB3" : "kg/m\xC2\xB3" };
            case DisplayQuantity::air_density:
                return { cgs ? mantissa(value_si * 0.001) : fixed_number(value_si, 3), cgs ? "g/cm\xC2\xB3" : "kg/m\xC2\xB3" };
            case DisplayQuantity::viscosity:
                return { mantissa(value_si * (cgs ? 10.0 : 1.0)), cgs ? "g/(cm\xC2\xB7s)" : "Pa\xC2\xB7s" };
            case DisplayQuantity::stiffness:
                return { significant(value_si * (cgs ? 1000.0 : 1.0)), cgs ? "dyn/cm" : "N/m" };
            case DisplayQuantity::damping:
                return { significant(value_si * (cgs ? 1000.0 : 1.0)), cgs ? "dyn\xC2\xB7s/cm" : "N\xC2\xB7s/m" };
            case DisplayQuantity::twist_stiffness:
                return { cgs && std::abs(value_si * 1.0e7) >= 1.0e6 ? mantissa(value_si * 1.0e7) : significant(value_si * (cgs ? 1.0e7 : 1.0)), cgs ? "dyn\xC2\xB7"
                                                                                                                                                      "cm/rad"
                                                                                                                                                    : "N\xC2\xB7m/rad" };
            case DisplayQuantity::twist_damping:
                return { cgs && std::abs(value_si * 1.0e7) >= 1.0e6 ? mantissa(value_si * 1.0e7) : significant(value_si * (cgs ? 1.0e7 : 1.0)), cgs ? "dyn\xC2\xB7"
                                                                                                                                                      "cm\xC2\xB7s/rad"
                                                                                                                                                    : "N\xC2\xB7m\xC2\xB7s/rad" };
            case DisplayQuantity::power:
                return cgs ? cgs_large(value_si * 1.0e7, "erg/s", 100.0) : prefixed_si(value_si, "W", "mW", "kW");
            case DisplayQuantity::coefficient:
                return { fixed_number(value_si, 2), {} };
            case DisplayQuantity::multiplier:
                return { fixed_number(value_si, precision < 0 ? 2 : precision, true), "\xC3\x97", true };
            case DisplayQuantity::percentage:
                return { std::abs(value_si - std::round(value_si)) < 0.0005 ? fixed_number(value_si, 0) : significant(value_si, 2), "%" };
            case DisplayQuantity::count:
                return { fixed_number(value_si, 0), {} };
            case DisplayQuantity::scale:
            {
                const auto percent = value_si * 100.0;
                return { std::abs(percent - std::round(percent)) < 0.0005 ? fixed_number(percent, 0) : significant(percent, 3), "%" };
            }
            case DisplayQuantity::speed_fraction:
                // c itself, or beyond, is never a speed fraction; the dash carries no unit.
                if (!(std::abs(value_si) < 1.0))
                    return { std::string { em_dash }, {} };
                return { speed_fraction_text(value_si), "c" };
            case DisplayQuantity::lorentz_factor_excess:
                return { lorentz_factor_text(value_si), {} };
            case DisplayQuantity::relativistic_speed:
                return whole_speed(value_si, cgs, false);
            case DisplayQuantity::speed_gap:
                return whole_speed(value_si, cgs, true);
            case DisplayQuantity::relativistic_energy:
                return { significant_within(value_si * (cgs ? 1.0e7 : 1.0), 1.0e-3, 1.0e4), cgs ? "erg" : "J" };
            case DisplayQuantity::relativistic_momentum:
                return { significant_within(value_si * (cgs ? 1.0e5 : 1.0), 1.0e-3, 1.0e4), cgs ? "g\xC2\xB7"
                                                                                                  "cm/s"
                                                                                                : "kg\xC2\xB7m/s" };
            case DisplayQuantity::fine_time:
            {
                const auto nanoseconds = value_si * 1.0e9;
                // Grouped from four digits, so 3 600.00 ns lines up with 10 800.00 ns on the other clock.
                if (std::abs(value_si) >= 1.0e-9 || value_si == 0.0)
                    return { group_and_sign(fixed_ascii(nanoseconds, 2), 4), "ns" };
                return { fixed_number(nanoseconds, std::min(6, carried_decimals(nanoseconds, 3))), "ns" };
            }
            case DisplayQuantity::duration:
                return duration_figure(value_si);
            case DisplayQuantity::small_length:
                if (cgs)
                    return stepped_unit(value_si, { { 1.0e-2, "cm" } }, "cm", 1.0e-2);
                return stepped_unit(value_si, { { 1.0, "m" }, { 1.0e-3, "mm" }, { 1.0e-6, "\xC2\xB5m" }, { 1.0e-9, "nm" } }, "m", 1.0);
            }
            return {};
        }
    }

    int speed_fraction_decimals(double fraction)
    {
        // A speed or gap at a power of ten counts as that power, whichever side of it the nearest
        // double falls, so 0.9999 is worked to five decimals like 0.9999 typed.
        constexpr auto snap = 1.0e-9;
        if (!(fraction > 0.0) || !(fraction < 1.0))
            return 3;
        if (fraction < 0.5)
            return 2 - static_cast<int>(std::floor(std::log10(fraction) + snap));
        return std::max(3, 1 - static_cast<int>(std::floor(std::log10(1.0 - fraction) + snap)));
    }

    double display_factor(DisplayQuantity quantity, DisplayUnits units)
    {
        if (quantity == DisplayQuantity::scale)
            return 100.0;
        if (quantity == DisplayQuantity::fine_length)
            return units == DisplayUnits::si ? 1000.0 : 100.0;
        // Clock readings and short intervals read in nanoseconds in both systems, so a bare typed
        // number means nanoseconds.
        if (quantity == DisplayQuantity::fine_time || quantity == DisplayQuantity::duration)
            return 1.0e9;
        if (units == DisplayUnits::si)
            return 1.0;
        switch (quantity)
        {
        case DisplayQuantity::length:
        case DisplayQuantity::fine_length:
        case DisplayQuantity::velocity:
        case DisplayQuantity::acceleration:
        case DisplayQuantity::relativistic_speed:
        case DisplayQuantity::speed_gap:
        case DisplayQuantity::small_length:
            return 100.0;
        case DisplayQuantity::mass:
            return 1000.0;
        case DisplayQuantity::force:
        case DisplayQuantity::momentum:
        case DisplayQuantity::relativistic_momentum:
            return 1.0e5;
        case DisplayQuantity::energy:
        case DisplayQuantity::torque:
        case DisplayQuantity::inertia:
        case DisplayQuantity::twist_stiffness:
        case DisplayQuantity::twist_damping:
        case DisplayQuantity::power:
        case DisplayQuantity::relativistic_energy:
            return 1.0e7;
        case DisplayQuantity::density:
        case DisplayQuantity::air_density:
            return 0.001;
        case DisplayQuantity::viscosity:
            return 10.0;
        case DisplayQuantity::stiffness:
        case DisplayQuantity::damping:
            return 1000.0;
        case DisplayQuantity::time:
        case DisplayQuantity::angle:
        case DisplayQuantity::angular_velocity:
        case DisplayQuantity::coefficient:
        case DisplayQuantity::multiplier:
        case DisplayQuantity::percentage:
        case DisplayQuantity::count:
        case DisplayQuantity::speed_fraction:
        case DisplayQuantity::lorentz_factor_excess:
            return 1.0;
        case DisplayQuantity::scale:
            return 100.0;
        case DisplayQuantity::fine_time:
        case DisplayQuantity::duration:
            return 1.0e9;
        }
        return 1.0;
    }

    std::string_view display_unit(DisplayQuantity quantity, DisplayUnits units)
    {
        // A speed fraction of one would be c itself, which reads as a bare dash, so ask at ½ c.
        const auto result = formatted(quantity == DisplayQuantity::speed_fraction ? 0.5 : 1.0, quantity, units, -1);
        return result.unit;
    }

    double display_value(double value_si, DisplayQuantity quantity, DisplayUnits units)
    {
        return value_si * display_factor(quantity, units);
    }

    std::string format_value(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision)
    {
        if (!std::isfinite(value_si))
            return "\xE2\x80\x94";
        return formatted(value_si == 0.0 ? 0.0 : value_si, quantity, units, precision).value;
    }

    std::string format_unit(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision)
    {
        if (!std::isfinite(value_si))
            return std::string(display_unit(quantity, units));
        return std::string(formatted(value_si == 0.0 ? 0.0 : value_si, quantity, units, precision).unit);
    }

    std::string format_quantity(double value_si, DisplayQuantity quantity, DisplayUnits units, int precision)
    {
        if (!std::isfinite(value_si))
            return "\xE2\x80\x94";
        const auto result = formatted(value_si == 0.0 ? 0.0 : value_si, quantity, units, precision);
        if (result.unit.empty())
            return result.value;
        return result.value + (result.tight_unit ? std::string {} : std::string { no_break_space }) + std::string { result.unit };
    }

    std::string format_significant(double value, int digits)
    {
        if (!std::isfinite(value))
            return std::string { em_dash };
        return significant_within(value == 0.0 ? 0.0 : value, 1.0e-3, 1.0e6, std::max(digits, 1));
    }

    std::optional<double> parse_quantity(std::string_view text, DisplayQuantity quantity, DisplayUnits displayed_units)
    {
        std::string normalized;
        normalized.reserve(text.size());
        for (std::size_t i = 0; i < text.size();)
        {
            const auto remaining = text.substr(i);
            if (remaining.rfind("\xE2\x88\x92", 0) == 0) // U+2212
            {
                normalized.push_back('-');
                i += 3;
            }
            else if (remaining.rfind("\xE2\x80\x89", 0) == 0 || remaining.rfind("\xC2\xA0", 0) == 0)
            {
                i += remaining.front() == '\xE2' ? 3 : 2;
            }
            else
                normalized.push_back(text[i++]);
        }
        const auto has_point = normalized.find('.') != std::string::npos;
        if (!has_point)
            std::replace(normalized.begin(), normalized.end(), ',', '.');
        normalized.erase(std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c)
                             {
                                 return c == ' ' || c == '\t';
                             }),
            normalized.end());
        if (normalized.empty())
            return std::nullopt;

        char* end = nullptr;
        const auto value = std::strtod(normalized.c_str(), &end);
        if (end == normalized.c_str() || !std::isfinite(value))
            return std::nullopt;
        std::string unit(end);
        std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c)
            {
                return static_cast<char>(std::tolower(c));
            });

        if (unit.empty())
            return value / display_factor(quantity, displayed_units);

        const auto one_of = [&unit](std::initializer_list<std::string_view> values)
        {
            return std::find(values.begin(), values.end(), unit) != values.end();
        };
        switch (quantity)
        {
        case DisplayQuantity::mass:
            if (unit == "kg")
                return value;
            if (unit == "g")
                return value / 1000.0;
            break;
        case DisplayQuantity::length:
        case DisplayQuantity::fine_length:
            if (unit == "m")
                return value;
            if (unit == "cm")
                return value / 100.0;
            if (unit == "mm")
                return value / 1000.0;
            break;
        case DisplayQuantity::velocity:
            if (unit == "m/s")
                return value;
            if (unit == "cm/s")
                return value / 100.0;
            break;
        case DisplayQuantity::acceleration:
            if (one_of({ "m/s2", "m/s^2", "m/s\xC2\xB2" }))
                return value;
            if (one_of({ "cm/s2", "cm/s^2", "cm/s\xC2\xB2" }))
                return value / 100.0;
            break;
        case DisplayQuantity::angle:
            if (one_of({ "deg", "\xC2\xB0" }))
                return value;
            break;
        case DisplayQuantity::angular_velocity:
            if (unit == "rad/s")
                return value;
            break;
        case DisplayQuantity::time:
            if (unit == "s")
                return value;
            if (unit == "ms")
                return value / 1000.0;
            break;
        case DisplayQuantity::force:
            if (unit == "n")
                return value;
            if (unit == "mn")
                return value / 1000.0;
            if (unit == "kn")
                return value * 1000.0;
            if (unit == "dyn")
                return value / 1.0e5;
            break;
        case DisplayQuantity::energy:
            if (unit == "j")
                return value;
            if (unit == "mj")
                return value / 1000.0;
            if (unit == "kj")
                return value * 1000.0;
            if (unit == "erg")
                return value / 1.0e7;
            break;
        case DisplayQuantity::density:
        case DisplayQuantity::air_density:
            if (one_of({ "kg/m3", "kg/m^3", "kg/m\xC2\xB3" }))
                return value;
            if (one_of({ "g/cm3", "g/cm^3", "g/cm\xC2\xB3" }))
                return value * 1000.0;
            break;
        case DisplayQuantity::viscosity:
            if (one_of({ "pa\xC2\xB7s", "pas" }))
                return value;
            if (unit == "g/(cm\xC2\xB7s)")
                return value / 10.0;
            break;
        case DisplayQuantity::stiffness:
            if (unit == "n/m")
                return value;
            if (unit == "dyn/cm")
                return value / 1000.0;
            break;
        case DisplayQuantity::damping:
            if (one_of({ "n\xC2\xB7s/m", "ns/m" }))
                return value;
            if (unit == "dyn\xC2\xB7s/cm")
                return value / 1000.0;
            break;
        case DisplayQuantity::twist_stiffness:
            if (unit == "n\xC2\xB7m/rad")
                return value;
            if (unit == "dyn\xC2\xB7"
                        "cm/rad")
                return value / 1.0e7;
            break;
        case DisplayQuantity::twist_damping:
            if (unit == "n\xC2\xB7m\xC2\xB7s/rad")
                return value;
            if (unit == "dyn\xC2\xB7"
                        "cm\xC2\xB7s/rad")
                return value / 1.0e7;
            break;
        case DisplayQuantity::multiplier:
            if (one_of({ "x", "\xC3\x97" }))
                return value;
            break;
        case DisplayQuantity::percentage:
            if (unit == "%")
                return value;
            break;
        case DisplayQuantity::scale:
            if (unit == "%")
                return value / 100.0;
            break;
        case DisplayQuantity::speed_fraction:
            // A fraction of c, a percentage of it, or a speed. The parser never range-checks, so
            // "1 c" reaches the control's own limit and its message.
            if (unit == "c")
                return value;
            if (one_of({ "%", "%c" }))
                return value / 100.0;
            if (unit == "m/s")
                return value / speed_of_light_m_s;
            if (unit == "km/s")
                return value * 1000.0 / speed_of_light_m_s;
            if (unit == "cm/s")
                return value / 100.0 / speed_of_light_m_s;
            break;
        case DisplayQuantity::fine_time:
        case DisplayQuantity::duration:
            if (unit == "s")
                return value;
            if (unit == "ms")
                return value / 1.0e3;
            if (one_of({ "\xC2\xB5s", "us" }))
                return value / 1.0e6;
            if (unit == "ns")
                return value / 1.0e9;
            if (unit == "ps")
                return value / 1.0e12;
            if (unit == "fs")
                return value / 1.0e15;
            break;
        case DisplayQuantity::small_length:
            if (unit == "m")
                return value;
            if (unit == "cm")
                return value / 100.0;
            if (unit == "mm")
                return value / 1.0e3;
            if (one_of({ "\xC2\xB5m", "um" }))
                return value / 1.0e6;
            if (unit == "nm")
                return value / 1.0e9;
            break;
        case DisplayQuantity::coefficient:
        case DisplayQuantity::count:
            break;
        default:
            // The remaining specialist unit families accept their exact displayed unit in either
            // system. This keeps input strict without duplicating formatting policy here.
            for (const auto system : { DisplayUnits::si, DisplayUnits::centimetre_gram })
            {
                auto expected = std::string(display_unit(quantity, system));
                std::transform(expected.begin(), expected.end(), expected.begin(), [](unsigned char c)
                    {
                        return static_cast<char>(std::tolower(c));
                    });
                if (unit == expected)
                    return value / display_factor(quantity, system);
            }
            break;
        }
        return std::nullopt;
    }
}
