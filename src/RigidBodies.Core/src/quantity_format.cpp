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

        std::string group_and_sign(std::string value)
        {
            const auto negative = !value.empty() && value.front() == '-';
            if (negative)
                value.erase(value.begin());
            const auto dot = value.find('.');
            const auto integer_digits = dot == std::string::npos ? value.size() : dot;
            if (integer_digits >= 5)
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
            const auto exponent = static_cast<int>(std::floor(std::log10(std::abs(value))));
            const auto scaled = value / std::pow(10.0, exponent);
            return significant(scaled, digits) + " \xC3\x97 10" + superscript(exponent);
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
                return { fixed_number(value_si, 2, true), "\xC3\x97", true };
            case DisplayQuantity::percentage:
                return { std::abs(value_si - std::round(value_si)) < 0.0005 ? fixed_number(value_si, 0) : significant(value_si, 2), "%" };
            case DisplayQuantity::count:
                return { fixed_number(value_si, 0), {} };
            case DisplayQuantity::scale:
            {
                const auto percent = value_si * 100.0;
                return { std::abs(percent - std::round(percent)) < 0.0005 ? fixed_number(percent, 0) : significant(percent, 3), "%" };
            }
            }
            return {};
        }
    }

    double display_factor(DisplayQuantity quantity, DisplayUnits units)
    {
        if (quantity == DisplayQuantity::scale)
            return 100.0;
        if (quantity == DisplayQuantity::fine_length)
            return units == DisplayUnits::si ? 1000.0 : 100.0;
        if (units == DisplayUnits::si)
            return 1.0;
        switch (quantity)
        {
        case DisplayQuantity::length:
        case DisplayQuantity::fine_length:
        case DisplayQuantity::velocity:
        case DisplayQuantity::acceleration:
            return 100.0;
        case DisplayQuantity::mass:
            return 1000.0;
        case DisplayQuantity::force:
        case DisplayQuantity::momentum:
            return 1.0e5;
        case DisplayQuantity::energy:
        case DisplayQuantity::torque:
        case DisplayQuantity::inertia:
        case DisplayQuantity::twist_stiffness:
        case DisplayQuantity::twist_damping:
        case DisplayQuantity::power:
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
            return 1.0;
        case DisplayQuantity::scale:
            return 100.0;
        }
        return 1.0;
    }

    std::string_view display_unit(DisplayQuantity quantity, DisplayUnits units)
    {
        const auto result = formatted(1.0, quantity, units, -1);
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
