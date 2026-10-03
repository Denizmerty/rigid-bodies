#include <rigidbodies/core/text_format.hpp>

#include <iomanip>
#include <cctype>

namespace rigidbodies::core
{
    namespace detail
    {

        std::string substitute_values(std::string_view pattern, const std::string* values, std::size_t count)
        {
            std::string result;
            result.reserve(pattern.size() + 16 * count);

            std::size_t next = 0;
            for (std::size_t index = 0; index < pattern.size(); ++index)
            {
                const auto character = pattern[index];

                if (character == '{' && index + 1 < pattern.size() && pattern[index + 1] == '}')
                {
                    if (next < count)
                    {
                        result.append(values[next]);
                        ++next;
                    }
                    else
                    {
                        // More placeholders than arguments. The placeholder is kept so that the
                        // mismatch is visible in the message rather than silently swallowed.
                        result.append("{}");
                    }
                    ++index;
                    continue;
                }

                result.push_back(character);
            }

            return result;
        }

    } // namespace detail

    std::string number(double value, int significant_digits)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::defaultfloat << std::setprecision(significant_digits) << value;
        return stream.str();
    }

    std::string fixed(double value, int decimals)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::fixed << std::setprecision(decimals) << value;
        return stream.str();
    }

    std::string humanise_identifier(std::string_view identifier)
    {
        std::string result { identifier };
        for (auto& character : result)
            if (character == '_')
                character = ' ';
        if (!result.empty())
            result.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(result.front())));
        return result;
    }

} // namespace rigidbodies::core
