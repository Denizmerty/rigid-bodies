#pragma once

#include <rigidbodies/ui/notifications.hpp>

#include <string_view>
#include <vector>

namespace rigidbodies::app
{
    class Notifier
    {
    public:
        std::uint64_t post(ui::Severity severity, std::string text, std::string source = {},
            std::optional<ui::NotificationAction> action = {}, bool persistent = false);
        void dismiss(std::uint64_t serial);
        void clear_source(std::string_view source);
        [[nodiscard]] const std::vector<ui::Notification>& notifications() const;

    private:
        std::vector<ui::Notification> notifications_;
        std::uint64_t next_serial_ { 1 };
    };
}
