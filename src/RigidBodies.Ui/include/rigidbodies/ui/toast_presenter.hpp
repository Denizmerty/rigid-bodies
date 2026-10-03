#pragma once

#include <rigidbodies/ui/notifications.hpp>

#include <map>
#include <set>
#include <vector>

namespace rigidbodies::ui
{
    class ToastPresenter
    {
    public:
        [[nodiscard]] std::vector<Notification> update(const std::vector<Notification>& source, double wall_time_s, bool present = false);
        void set_hovered(std::uint64_t serial, bool hovered, double wall_time_s);
        void dismiss(std::uint64_t serial);

    private:
        struct Timing
        {
            double first_seen {}, paused_since {}, paused_total {};
            bool hovered {};
        };
        std::map<std::uint64_t, Timing> timing_;
        std::set<std::uint64_t> dismissed_;
    };
}
