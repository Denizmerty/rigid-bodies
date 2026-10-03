#pragma once

#include <rigidbodies/ui/ui_command.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rigidbodies::ui
{
    enum class Severity : std::uint8_t
    {
        info,
        success,
        warning,
        error
    };

    struct NotificationAction
    {
        std::string label;
        std::optional<UiCommand> command;
        std::string reveal_key;
        std::string reveal_instance;
    };

    struct Notification
    {
        std::uint64_t serial {};
        Severity severity { Severity::info };
        std::string text;
        std::optional<NotificationAction> action;
        std::string source;
        bool persistent {};
    };

    struct Banner
    {
        std::uint64_t serial {};
        std::string text;
        std::vector<NotificationAction> actions;
    };

    struct StatusChip
    {
        std::string key;
        Severity severity { Severity::info };
        std::string text;
        std::string why_key;
        std::optional<UiCommand> action;
    };

    struct InlineNotice
    {
        std::string key;
        Severity severity { Severity::info };
        std::string text;
    };
}
