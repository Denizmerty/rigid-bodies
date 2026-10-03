#include <rigidbodies/app/notifier.hpp>

#include <algorithm>

namespace rigidbodies::app
{
    std::uint64_t Notifier::post(ui::Severity severity, std::string text, std::string source,
        std::optional<ui::NotificationAction> action, bool persistent)
    {
        if (text.empty())
        {
            clear_source(source);
            return 0;
        }
        if (!source.empty())
            clear_source(source);
        ui::Notification notification;
        notification.serial = next_serial_++;
        notification.severity = severity;
        notification.text = std::move(text);
        notification.action = std::move(action);
        notification.source = std::move(source);
        notification.persistent = persistent || severity == ui::Severity::error;
        notifications_.push_back(std::move(notification));
        if (notifications_.size() > 16)
            notifications_.erase(notifications_.begin(), notifications_.begin() + static_cast<std::ptrdiff_t>(notifications_.size() - 16));
        return notifications_.back().serial;
    }

    void Notifier::dismiss(std::uint64_t serial)
    {
        notifications_.erase(std::remove_if(notifications_.begin(), notifications_.end(), [&](const ui::Notification& item)
                                 {
                                     return item.serial == serial;
                                 }),
            notifications_.end());
    }

    void Notifier::clear_source(std::string_view source)
    {
        if (source.empty())
            return;
        notifications_.erase(std::remove_if(notifications_.begin(), notifications_.end(), [&](const ui::Notification& item)
                                 {
                                     return item.source == source;
                                 }),
            notifications_.end());
    }

    const std::vector<ui::Notification>& Notifier::notifications() const
    {
        return notifications_;
    }
}
