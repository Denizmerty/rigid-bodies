#include <rigidbodies/core/log.hpp>

#include <iostream>

namespace rigidbodies::core
{
    namespace
    {

        LogLevel active_level = LogLevel::info;

    } // namespace

    std::string_view to_string(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::trace:
            return "trace";
        case LogLevel::info:
            return "info";
        case LogLevel::warning:
            return "warning";
        case LogLevel::error:
            return "error";
        }
        return "info";
    }

    LogLevel active_log_level()
    {
        return active_level;
    }

    void set_active_log_level(LogLevel level)
    {
        active_level = level;
    }

    void write_log(LogLevel level, std::string_view message)
    {
        std::cerr << '[' << to_string(level) << "] " << message << '\n';
    }

} // namespace rigidbodies::core
