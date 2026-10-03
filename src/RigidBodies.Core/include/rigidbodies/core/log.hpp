#pragma once

#include <rigidbodies/core/text_format.hpp>

#include <string>
#include <string_view>

namespace rigidbodies::core
{

    enum class LogLevel
    {
        trace,
        info,
        warning,
        error
    };

    [[nodiscard]] std::string_view to_string(LogLevel level);

    // Messages below the active level are discarded before they are formatted.
    [[nodiscard]] LogLevel active_log_level();
    void set_active_log_level(LogLevel level);

    // Writes one already-formatted message. Diagnostics go to the standard error stream so that
    // they stay readable when the application is launched from a terminal and so that they do not
    // interleave with anything the program writes to standard output.
    void write_log(LogLevel level, std::string_view message);

    template <typename... Arguments>
    void log(LogLevel level, std::string_view pattern, const Arguments&... arguments)
    {
        if (level < active_log_level())
        {
            return;
        }
        write_log(level, substitute(pattern, arguments...));
    }

    template <typename... Arguments>
    void log_trace(std::string_view pattern, const Arguments&... arguments)
    {
        log(LogLevel::trace, pattern, arguments...);
    }

    template <typename... Arguments>
    void log_info(std::string_view pattern, const Arguments&... arguments)
    {
        log(LogLevel::info, pattern, arguments...);
    }

    template <typename... Arguments>
    void log_warning(std::string_view pattern, const Arguments&... arguments)
    {
        log(LogLevel::warning, pattern, arguments...);
    }

    template <typename... Arguments>
    void log_error(std::string_view pattern, const Arguments&... arguments)
    {
        log(LogLevel::error, pattern, arguments...);
    }

} // namespace rigidbodies::core
