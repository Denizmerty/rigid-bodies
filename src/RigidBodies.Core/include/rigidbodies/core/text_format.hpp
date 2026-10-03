#pragma once

#include <array>
#include <cstddef>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>

namespace rigidbodies::core
{

    // Text formatting for messages and read-outs: positional `{}` substitution and two numeric
    // conversions. More complex formatting belongs at the call site.
    //
    // Conversions use the classic locale to keep number formatting consistent across machines.
    namespace detail
    {

        template <typename Value>
        std::string to_text(const Value& value)
        {
            std::ostringstream stream;
            stream.imbue(std::locale::classic());
            stream << value;
            return stream.str();
        }

        inline std::string to_text(const std::string& value)
        {
            return value;
        }

        inline std::string to_text(std::string_view value)
        {
            return std::string { value };
        }

        inline std::string to_text(const char* value)
        {
            return value == nullptr ? std::string {} : std::string { value };
        }

        std::string substitute_values(std::string_view pattern, const std::string* values, std::size_t count);

    } // namespace detail

    // Replaces each `{}` with the next argument, converted to text. Extra placeholders remain in
    // the output and extra arguments are ignored. A count mismatch does not throw an exception.
    template <typename... Arguments>
    [[nodiscard]] std::string substitute(std::string_view pattern, const Arguments&... arguments)
    {
        const std::array<std::string, sizeof...(Arguments)> values { detail::to_text(arguments)... };
        return detail::substitute_values(pattern, values.data(), values.size());
    }

    // Formats a value to the requested number of significant digits, without trailing decimal zeros.
    // Useful for quantities that span several orders of magnitude.
    [[nodiscard]] std::string number(double value, int significant_digits = 6);

    // Formats a value with a fixed number of decimal places for aligned read-outs.
    [[nodiscard]] std::string fixed(double value, int decimals);

    // Converts a content identifier into learner-facing fallback text.
    [[nodiscard]] std::string humanise_identifier(std::string_view identifier);

} // namespace rigidbodies::core
