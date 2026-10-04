#include <rigidbodies/app/input_router.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    ui::UiEvent key(ui::UiKey value, ui::KeyModifiers modifiers = {})
    {
        ui::UiEvent event;
        event.kind = ui::UiEventKind::key_down;
        event.key = value;
        event.modifiers = modifiers;
        return event;
    }

    RIGIDBODIES_TEST("input routing priority keeps F12 above developer capture")
    {
        app::InputContext context;
        context.developer_captured = true;
        const auto f12 = app::route_event(key(ui::UiKey::f12), app::AppAction::toggle_developer_overlay, context);
        RIGIDBODIES_EXPECT(f12.action == app::AppAction::toggle_developer_overlay && !f12.to_developer, "F12 always dismisses the developer inspector");
        auto held_f12 = key(ui::UiKey::f12);
        held_f12.repeat = true;
        for (const auto captured : { false, true })
        {
            context.developer_captured = captured;
            const auto repeated = app::route_event(held_f12, {}, context);
            RIGIDBODIES_EXPECT(!repeated.action && !repeated.to_developer, "holding F12 never toggles the inspector again, whether it is open or closed");
        }
        const auto ordinary = app::route_event(key(ui::UiKey::r), app::AppAction::reset_scenario, context);
        RIGIDBODIES_EXPECT(ordinary.to_developer && !ordinary.action, "developer capture precedes ordinary bindings");
    }

    RIGIDBODIES_TEST("text fields capture every destructive single key and editing chord")
    {
        app::InputContext context;
        context.focus = ui::FocusOwner::text_field;
        for (const auto& binding : app::key_bindings)
        {
            if (binding.key == ui::UiKey::escape || binding.key == ui::UiKey::f1 || binding.key == ui::UiKey::f12 || binding.modifiers.alt)
                continue;
            const auto decision = app::route_event(key(binding.key, binding.modifiers), binding.action, context);
            const auto expected_field = binding.key != ui::UiKey::f10 && binding.key != ui::UiKey::f5 && binding.key != ui::UiKey::menu &&
                (!binding.modifiers.control || binding.key == ui::UiKey::a || binding.key == ui::UiKey::z);
            if (expected_field)
                RIGIDBODIES_EXPECT(decision.to_interface && !decision.action, "a field owns printable and editing keys");
        }
        const auto backspace = app::route_event(key(ui::UiKey::backspace), {}, context);
        RIGIDBODIES_EXPECT(backspace.to_interface && !backspace.action, "Backspace remains a field edit rather than an application action");
        ui::UiEvent text;
        text.kind = ui::UiEventKind::text_input;
        text.text = "2 kg";
        RIGIDBODIES_EXPECT(app::route_event(text, {}, context).to_interface, "typed text reaches the field");
    }

    RIGIDBODIES_TEST("focused fields own word editing and both redo chords")
    {
        app::InputContext context;
        context.focus = ui::FocusOwner::text_field;
        for (const auto value : { ui::UiKey::arrow_left, ui::UiKey::arrow_right, ui::UiKey::home, ui::UiKey::end, ui::UiKey::backspace, ui::UiKey::delete_key, ui::UiKey::y, ui::UiKey::z })
            for (const auto shift : { false, true })
            {
                const auto event = key(value, { shift, true, false });
                const auto decision = app::route_event(event, app::action_for_key(value, event.modifiers), context);
                RIGIDBODIES_EXPECT(decision.to_interface && !decision.action, "field editing chords neither disappear nor redo scene edits");
            }
        const auto search = app::route_event(key(ui::UiKey::k, { false, true, false }), app::AppAction::open_command_search, context);
        RIGIDBODIES_EXPECT(search.action == app::AppAction::open_command_search, "global command search remains available while editing");
    }

    RIGIDBODIES_TEST("Alt vertical arrows adjust focused number fields instead of nudging scene objects")
    {
        app::InputContext context;
        for (const auto value : { ui::UiKey::arrow_up, ui::UiKey::arrow_down })
            for (const auto shift : { false, true })
            {
                const auto event = key(value, { shift, false, true });
                const auto action = app::action_for_key(value, event.modifiers);
                context.focus = ui::FocusOwner::text_field;
                const auto field = app::route_event(event, action, context);
                RIGIDBODIES_EXPECT(field.to_interface && !field.action, "a focused numeric field receives its fine or coarse adjustment");
                context.focus = ui::FocusOwner::scene;
                const auto scene = app::route_event(event, action, context);
                RIGIDBODIES_EXPECT(scene.action == action && !scene.to_interface, "the same chord still nudges objects when the scene owns focus");
            }
    }

    RIGIDBODIES_TEST("focused buttons own Space and modal surfaces block background scene edits")
    {
        app::InputContext context;
        context.focus = ui::FocusOwner::keyboard_control;
        auto decision = app::route_event(key(ui::UiKey::space), app::AppAction::toggle_pause, context);
        RIGIDBODIES_EXPECT(decision.to_interface && !decision.action, "Space activates the focused control instead of playing the scene");
        context.interface_modal = true;
        for (const auto action : { app::AppAction::delete_selection, app::AppAction::undo, app::AppAction::reset_scenario, app::AppAction::next_object })
        {
            decision = app::route_event(key(ui::UiKey::delete_key), action, context);
            RIGIDBODIES_EXPECT(!decision.action && !decision.to_scene, "a modal surface protects the scene beneath it");
        }
        decision = app::route_event(key(ui::UiKey::k, { false, true, false }), app::AppAction::open_command_search, context);
        RIGIDBODIES_EXPECT(decision.action == app::AppAction::open_command_search, "global command search can still open above a sheet");
        context.has_selection = context.draw_active = true;
        decision = app::route_event(key(ui::UiKey::escape), {}, context);
        RIGIDBODIES_EXPECT(decision.to_interface && !decision.to_scene, "Escape is owned by the modal instead of clearing the scene or draft selection");
    }

    RIGIDBODIES_TEST("auto repeat supports editing and control navigation without repeating activations")
    {
        auto event = key(ui::UiKey::r);
        event.repeat = true;
        app::InputContext scene;
        RIGIDBODIES_EXPECT(!app::route_event(event, app::AppAction::reset_scenario, scene).action, "repeat cannot reset the experiment");
        scene.focus = ui::FocusOwner::text_field;
        RIGIDBODIES_EXPECT(app::route_event(event, app::AppAction::reset_scenario, scene).to_interface, "repeat supports text editing");
        scene.focus = ui::FocusOwner::keyboard_control;
        event.key = ui::UiKey::arrow_down;
        RIGIDBODIES_EXPECT(app::route_event(event, {}, scene).to_interface, "holding an arrow traverses a control list");
        event.key = ui::UiKey::space;
        RIGIDBODIES_EXPECT(!app::route_event(event, app::AppAction::toggle_pause, scene).to_interface, "holding Space does not activate a button repeatedly");
    }

    RIGIDBODIES_TEST("escape ladder resolves exactly one highest priority step")
    {
        app::InputContext context;
        context.gesture_active = context.draw_active = context.draft_node_selected = context.has_selection = true;
        context.draft_nodes = 3;
        context.select_tool = false;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::escape), {}, context).escape == app::EscapeStep::cancel_gesture, "gesture is first");
        context.gesture_active = false;
        context.interface_escape = ui::EscapeTarget::text_field;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::escape), {}, context).escape == app::EscapeStep::revert_field, "field is second");
        context.interface_escape = ui::EscapeTarget::none;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::escape), {}, context).escape == app::EscapeStep::draw_deselect_node, "selected draft node precedes discard");
        context.draft_node_selected = false;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::escape), {}, context).escape == app::EscapeStep::draw_confirm_discard, "a substantial draft asks first");
        context.draft_nodes = 1;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::escape), {}, context).escape == app::EscapeStep::draw_discard, "a one-point draft discards directly");
    }

    RIGIDBODIES_TEST("Tab and F6 belong to interface traversal and pointer capture is exclusive")
    {
        app::InputContext context;
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::tab), {}, context).to_interface, "Tab traverses controls");
        RIGIDBODIES_EXPECT(app::route_event(key(ui::UiKey::f6), {}, context).to_interface, "F6 traverses surfaces");
        ui::UiEvent pointer;
        pointer.kind = ui::UiEventKind::pointer_down;
        context.pointer_over_surface = true;
        auto decision = app::route_event(pointer, {}, context);
        RIGIDBODIES_EXPECT(decision.to_interface && !decision.to_scene, "surface clicks do not leak to scene");
        context.pointer_over_surface = false;
        context.gesture_active = true;
        decision = app::route_event(pointer, {}, context);
        RIGIDBODIES_EXPECT(decision.to_scene && !decision.to_interface, "captured scene gesture keeps pointer delivery");
    }

    RIGIDBODIES_TEST("a pointer leaving for the title bar clears hover on both the interface and the stage")
    {
        app::InputContext context;
        ui::UiEvent leave;
        leave.kind = ui::UiEventKind::pointer_leave;
        auto decision = app::route_event(leave, {}, context);
        RIGIDBODIES_EXPECT(decision.to_interface && decision.to_scene, "both sides hear that the pointer left");
        context.pointer_over_surface = true;
        decision = app::route_event(leave, {}, context);
        RIGIDBODIES_EXPECT(decision.to_interface && decision.to_scene, "the leave reaches the stage even from over the interface");
        context.developer_captured = true;
        decision = app::route_event(leave, {}, context);
        RIGIDBODIES_EXPECT(decision.to_developer && !decision.to_interface && !decision.to_scene, "developer capture still takes it");
    }

    RIGIDBODIES_TEST("the window frame resizes from its top edge and otherwise follows the interface")
    {
        app::FrameGeometry frame;
        frame.width = 1600.0;
        frame.height = 900.0;
        frame.edge = 6.0;
        using P = ui::WindowPart;
        using H = app::FrameHit;
        RIGIDBODIES_EXPECT(app::frame_hit({ 800.0, 2.0 }, P::title_bar, frame) == H::top, "the top edge resizes over the title bar");
        RIGIDBODIES_EXPECT(app::frame_hit({ 1595.0, 2.0 }, P::close, frame) == H::top_right, "and over the close button near the corner, diagonally");
        RIGIDBODIES_EXPECT(app::frame_hit({ 5.0, 3.0 }, P::title_bar, frame) == H::top_left, "the top-left corner resizes diagonally");
        RIGIDBODIES_EXPECT(app::frame_hit({ 1580.0, 20.0 }, P::close, frame) == H::close, "below the edge the close button is the close button");
        RIGIDBODIES_EXPECT(app::frame_hit({ 800.0, 20.0 }, P::title_bar, frame) == H::title_bar, "an empty stretch of the strip moves the window");
        RIGIDBODIES_EXPECT(app::frame_hit({ 800.0, 20.0 }, P::client, frame) == H::client, "a toolbar control stays a control");
        RIGIDBODIES_EXPECT(app::frame_hit({ 1500.0, 20.0 }, P::minimize, frame) == H::minimize && app::frame_hit({ 1540.0, 20.0 }, P::maximize, frame) == H::maximize, "minimize and maximize are caption buttons");
        frame.maximized = true;
        RIGIDBODIES_EXPECT(app::frame_hit({ 1599.0, 0.0 }, P::close, frame) == H::close, "a maximized window's close button reaches the screen corner");
        RIGIDBODIES_EXPECT(app::frame_hit({ 800.0, 0.0 }, P::title_bar, frame) == H::title_bar, "a maximized window has no edge to resize");
        frame.maximized = false;
        frame.resizable = false;
        RIGIDBODIES_EXPECT(app::frame_hit({ 800.0, 2.0 }, P::title_bar, frame) == H::title_bar, "a fixed-size window has no edge to resize");
        RIGIDBODIES_EXPECT(app::frame_hit({ 1540.0, 20.0 }, P::maximize, frame) == H::title_bar, "nor a maximize button to press");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
