#include <rigidbodies/ui/toast_presenter.hpp>

#include <algorithm>

namespace rigidbodies::ui
{
    std::vector<Notification> ToastPresenter::update(const std::vector<Notification>& source, double wall_time_s, bool present)
    {
        std::vector<Notification> visible;
        for (const auto& notification : source)
        {
            auto [entry, inserted] = timing_.try_emplace(notification.serial);
            if (inserted)
                entry->second.first_seen = wall_time_s;
            const auto& timing = entry->second;
            const auto active_age = wall_time_s - timing.first_seen - timing.paused_total -
                (timing.hovered ? wall_time_s - timing.paused_since : 0.0);
            if (dismissed_.count(notification.serial) || (present && notification.severity != Severity::error) ||
                (!notification.persistent && notification.severity != Severity::error && active_age >= 4.0))
                continue;
            visible.push_back(notification);
        }
        while (visible.size() > 3)
        {
            const auto removable = std::find_if(visible.begin(), visible.end(), [](const Notification& item)
                {
                    return item.severity != Severity::error && !item.persistent;
                });
            if (removable == visible.end())
                visible.erase(visible.begin());
            else
            {
                dismissed_.insert(removable->serial);
                visible.erase(removable);
            }
        }
        return visible;
    }

    void ToastPresenter::set_hovered(std::uint64_t serial, bool hovered, double wall_time_s)
    {
        auto& timing = timing_[serial];
        if (hovered && !timing.hovered)
        {
            timing.hovered = true;
            timing.paused_since = wall_time_s;
        }
        else if (!hovered && timing.hovered)
        {
            timing.paused_total += wall_time_s - timing.paused_since;
            timing.hovered = false;
        }
    }

    void ToastPresenter::dismiss(std::uint64_t serial)
    {
        dismissed_.insert(serial);
    }
}
