#include <rigidbodies/app/notifier.hpp>
#include <rigidbodies/ui/toast_presenter.hpp>
#include <rigidbodies/ui/ui_context.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("notification sources replace in place and producer history is bounded")
    {
        app::Notifier notifier;
        notifier.post(ui::Severity::info, "first", "property");
        notifier.post(ui::Severity::success, "second", "property");
        RIGIDBODIES_EXPECT(notifier.notifications().size() == 1 && notifier.notifications().front().text == "second", "new feedback from one source replaces stale feedback");
        for (int i = 0; i < 20; ++i)
            notifier.post(ui::Severity::info, std::to_string(i), "source-" + std::to_string(i));
        RIGIDBODIES_EXPECT(notifier.notifications().size() == 16, "producer keeps its last sixteen notifications");
    }

    RIGIDBODIES_TEST("toast lifetime pauses on hover while errors persist")
    {
        ui::ToastPresenter presenter;
        std::vector<ui::Notification> source {
            { 1, ui::Severity::info, "temporary", {}, "one", false },
            { 2, ui::Severity::error, "persistent", {}, "two", true }
        };
        RIGIDBODIES_EXPECT(presenter.update(source, 10.0).size() == 2, "both toasts appear initially");
        presenter.set_hovered(1, true, 11.0);
        RIGIDBODIES_EXPECT(presenter.update(source, 20.0).size() == 2, "hover pauses the four second expiry");
        presenter.set_hovered(1, false, 20.0);
        auto visible = presenter.update(source, 23.1);
        RIGIDBODIES_EXPECT(visible.size() == 1 && visible.front().serial == 2, "temporary toast expires after four active seconds");
        RIGIDBODIES_EXPECT(presenter.update(source, 100.0).size() == 1, "error remains until dismissed");
        presenter.dismiss(2);
        RIGIDBODIES_EXPECT(presenter.update(source, 101.0).empty(), "explicit dismissal removes the error");
    }

    RIGIDBODIES_TEST("toast stack is limited to three and protects errors from ordinary eviction")
    {
        ui::ToastPresenter presenter;
        std::vector<ui::Notification> source {
            { 1, ui::Severity::error, "error", {}, "e", true },
            { 2, ui::Severity::info, "old", {}, "a", false },
            { 3, ui::Severity::warning, "middle", {}, "b", false },
            { 4, ui::Severity::success, "new", {}, "c", false }
        };
        auto visible = presenter.update(source, 0.0);
        RIGIDBODIES_EXPECT(visible.size() == 3 && visible.front().serial == 1, "a fourth toast evicts the oldest non-error");
        RIGIDBODIES_EXPECT(presenter.update(source, 0.0, true).size() == 1, "Present mode shows only errors");
        presenter.dismiss(4);
        visible = presenter.update(source, 1.0);
        RIGIDBODIES_EXPECT(visible.size() == 2 && visible.front().serial == 1 && visible.back().serial == 3, "an evicted toast stays dismissed when stack space opens");
    }

    RIGIDBODIES_TEST("a notification reveal action opens the section that owns its control")
    {
        ui::UiContext context;
        const ui::NotificationAction action { "Inspect", {}, "world.advanced.integration_method", {} };
        context.reveal(action.reveal_key, action.reveal_instance);
        RIGIDBODIES_EXPECT(context.view_state().section_open("world.advanced", false), "the reveal key opens its owning disclosure before backend scrolling and focus");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
