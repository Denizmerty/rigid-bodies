#include <rigidbodies/app/input_router.hpp>
#include <rigidbodies/ui/layout.hpp>
#include <rigidbodies/ui/toast_presenter.hpp>
#include <rigidbodies/ui/view_state.hpp>
#include "test_framework.hpp"
using namespace rigidbodies;
RIGIDBODIES_TEST("Present owns chrome focus lock and final Escape")
{
    ui::ViewState state;
    RIGIDBODIES_EXPECT(state.present().lock && !state.present().mode && !state.present().spotlight, "safe defaults");
    state.present().mode = true;
    ui::LayoutInput input;
    input.viewport = { 1600, 900 };
    input.present = true;
    const auto layout = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(layout.find(ui::RegionId::present_strip) && !layout.find(ui::RegionId::command_bar), "lesson strip replaces command bar");
    RIGIDBODIES_EXPECT(layout.stage.minimum.y == 56 && layout.stage.maximum.y == 900, "stage excludes only lesson strip");
    app::InputContext context;
    context.present = true;
    context.select_tool = true;
    ui::UiEvent escape;
    escape.kind = ui::UiEventKind::key_down;
    escape.key = ui::UiKey::escape;
    RIGIDBODIES_EXPECT(app::route_event(escape, {}, context).escape == app::EscapeStep::leave_present, "last Escape leaves Present");
}
RIGIDBODIES_TEST("Present suppresses non-error toasts")
{
    ui::ToastPresenter presenter;
    std::vector<ui::Notification> source { { 1, ui::Severity::info, "info" }, { 2, ui::Severity::error, "error" } };
    const auto visible = presenter.update(source, 0, true);
    RIGIDBODIES_EXPECT(visible.size() == 1 && visible.front().severity == ui::Severity::error, "only errors survive");
}
int main()
{
    return rigidbodies::testing::run_all();
}
