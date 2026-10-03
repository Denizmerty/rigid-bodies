#include <rigidbodies/app/simulation_session.hpp>
#include "test_framework.hpp"

#include <algorithm>

using namespace rigidbodies;

namespace
{
    physics::BodyId circle(app::SimulationSession& session, math::Vec2 position, double radius, physics::BodyType type = physics::BodyType::dynamic_body)
    {
        physics::BodyDefinition definition;
        definition.position_m = position;
        definition.type = type;
        definition.name = "target";
        physics::Collider collider;
        collider.shape = physics::make_circle(radius);
        definition.colliders.push_back(collider);
        return session.world().create_body(definition);
    }
    ui::UiEvent pointer(ui::UiEventKind kind, math::Vec2 point, double scale = 1.0, bool shift = false)
    {
        ui::UiEvent event;
        event.kind = kind;
        event.pointer_px = point;
        event.logical_pixel_scale = scale;
        event.modifiers.shift = shift;
        return event;
    }
    ui::UiEvent secondary(ui::UiEventKind kind, math::Vec2 point, double scale = 1.0)
    {
        auto event = pointer(kind, point, scale);
        event.button = ui::PointerButton::secondary;
        return event;
    }
}

RIGIDBODIES_TEST("picking tolerance scales and smallest overlapping body wins")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    const auto large = circle(session, {}, .5);
    const auto small = circle(session, {}, .2);
    const auto edge = session.camera().world_to_screen({ .2, 0 });
    for (const auto scale : { .75, 1.0, 1.5, 2.0 })
    {
        auto press = pointer(ui::UiEventKind::pointer_down, edge + math::Vec2 { 5.9 * scale, 0 }, scale);
        session.handle_scene_event(press);
        session.handle_scene_event(pointer(ui::UiEventKind::pointer_up, press.pointer_px, scale));
        RIGIDBODIES_EXPECT(session.selection() == small, "smallest body wins inside scaled tolerance");
    }
    RIGIDBODIES_EXPECT(!(session.selection() == large), "iteration order does not override smallest-area priority");
}

RIGIDBODIES_TEST("three pixels is a click and four pixels starts pull")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    const auto id = circle(session, {}, .2);
    ui::UiCommand mode;
    mode.kind = ui::UiCommandKind::set_interaction_mode;
    mode.id = "pull";
    session.apply(mode);
    const auto start = session.camera().world_to_screen({});
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_down, start));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, start + math::Vec2 { 3, 0 }));
    RIGIDBODIES_EXPECT(session.world().force_generators(id).empty(), "3 px leaves pull unattached");
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, start + math::Vec2 { 4, 0 }));
    RIGIDBODIES_EXPECT(!session.world().force_generators(id).empty(), "4 px attaches pull");
}

RIGIDBODIES_TEST("hover timing publishes silhouette then card without selecting")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    const auto id = circle(session, {}, .2);
    session.set_wall_time(10.0);
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, session.camera().world_to_screen({})));
    RIGIDBODIES_EXPECT(!session.build_model().hover.has_value(), "hover stays quiet before 150 ms");
    session.set_wall_time(10.15);
    const auto silhouette = session.build_model();
    RIGIDBODIES_EXPECT(silhouette.hover && silhouette.hover->body == id && silhouette.hover->silhouette && silhouette.hover->card_lines.empty(), "150 ms publishes only the silhouette");
    session.set_wall_time(10.61);
    const auto card = session.build_model();
    RIGIDBODIES_EXPECT(card.hover && card.hover->card_lines.size() >= 3, "600 ms publishes the read-only object card");
    RIGIDBODIES_EXPECT(!session.selection().is_valid(), "hover never becomes selection");
}

RIGIDBODIES_TEST("shift marquee includes free and driven objects but excludes fixed objects")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    const auto moving = circle(session, { -.5, 0 }, .15);
    const auto driven = circle(session, { .5, 0 }, .15, physics::BodyType::kinematic_body);
    const auto fixed = circle(session, { 0, .5 }, .15, physics::BodyType::static_body);
    const auto first = session.camera().world_to_screen({ -1, -1 });
    const auto second = session.camera().world_to_screen({ 1, 1 });
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_down, first, 1.0, true));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, second, 1.0, true));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_up, second, 1.0, true));
    const auto selected = session.selections();
    RIGIDBODIES_EXPECT(std::find(selected.begin(), selected.end(), moving) != selected.end(), "marquee selects free objects");
    RIGIDBODIES_EXPECT(std::find(selected.begin(), selected.end(), driven) != selected.end(), "marquee selects driven objects");
    RIGIDBODIES_EXPECT(std::find(selected.begin(), selected.end(), fixed) == selected.end(), "marquee excludes fixed objects");
}

RIGIDBODIES_TEST("velocity handle scales and commits one undoable edit at four pixels")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    session.stepper().set_paused(true);
    const auto id = circle(session, {}, .2);
    session.set_selection(id);
    ui::UiCommand scale;
    scale.kind = ui::UiCommandKind::set_ui_scale;
    scale.value = 1.5;
    session.apply(scale);
    auto model = session.build_model();
    const auto handle = std::find_if(model.handles.begin(), model.handles.end(), [](const auto& item)
        {
            return item.id == "velocity";
        });
    RIGIDBODIES_EXPECT(handle != model.handles.end() && handle->screen_bounds.width == 36.0 && handle->screen_bounds.height == 36.0, "handle target follows interface scale");
    const auto start = handle->screen_bounds.center();
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_down, start, 1.5));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, start + math::Vec2 { 4.4, 0 }, 1.5));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_up, start + math::Vec2 { 4.4, 0 }, 1.5));
    RIGIDBODIES_EXPECT_NEAR(math::length(session.world().find_body(id)->linear_velocity_m_s()), 0.0, 0.0, "under four logical pixels changes nothing");
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_down, start, 1.5));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_move, start + math::Vec2 { 6.0, 0 }, 1.5));
    session.handle_scene_event(pointer(ui::UiEventKind::pointer_up, start + math::Vec2 { 6.0, 0 }, 1.5));
    RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s().x > 0.0 && session.build_model().can_undo, "four logical pixels commits the handle edit");
    ui::UiCommand undo;
    undo.kind = ui::UiCommandKind::undo;
    session.apply(undo);
    RIGIDBODIES_EXPECT_NEAR(math::length(session.world().find_body(id)->linear_velocity_m_s()), 0.0, 0.0, "one undo restores velocity");
    RIGIDBODIES_EXPECT(!session.build_model().can_undo, "one release creates exactly one history entry");
}

RIGIDBODIES_TEST("Add Ball pauses a running lab names the object and is one undo")
{
    app::SimulationSession session;
    session.world().clear();
    session.set_viewport({ 1000, 800 });
    session.camera().set_view_height(4.0);
    session.stepper().set_paused(false);
    ui::UiCommand add;
    add.kind = ui::UiCommandKind::add_object;
    add.id = "ball";
    session.apply(add);
    const auto added = session.build_model();
    RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && session.selection().is_valid(), "Add Ball creates and selects one object");
    RIGIDBODIES_EXPECT(added.pause_reason.reason == ui::PauseReason::new_object && added.paused, "running lab pauses with the new-object reason");
    RIGIDBODIES_EXPECT(!added.notifications.empty() && added.notifications.back().text == "Added **Ball 1**" && added.notifications.back().action && added.notifications.back().action->label == "Undo", "toast names the object and exposes Undo");
    ui::UiCommand undo;
    undo.kind = ui::UiCommandKind::undo;
    session.apply(undo);
    RIGIDBODIES_EXPECT(session.world().body_ids().empty() && !session.selection().is_valid() && !session.build_model().can_undo, "one undo removes the object and restores selection/history");
}

RIGIDBODIES_TEST("visible joint and spring glyphs publish connection hit areas")
{
    app::SimulationSession session;
    session.configure({});
    session.set_viewport({ 1000, 800 });
    RIGIDBODIES_EXPECT(session.load_scenario("distance_chain"), "joint scenario loads");
    ui::UiCommand show;
    show.kind = ui::UiCommandKind::set_layer;
    show.id = "constraints";
    show.flag = true;
    session.apply(show);
    const auto model = session.build_model();
    RIGIDBODIES_EXPECT(std::any_of(model.handles.begin(), model.handles.end(), [](const auto& handle)
                           {
                               return handle.kind == ui::StageTargetKind::connection && handle.screen_bounds.width > 0.0;
                           }),
        "connection glyphs expose model hit areas");
}

RIGIDBODIES_TEST("secondary clicks target objects connections and empty space while four pixels pans")
{
    {
        app::SimulationSession session;
        session.world().clear();
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
        const auto id = circle(session, {}, .2);
        const auto point = session.camera().world_to_screen({});
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, point));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, point + math::Vec2 { 3, 0 }));
        const auto request = session.build_model().context_menu_request;
        RIGIDBODIES_EXPECT(request && request->kind == ui::StageTargetKind::object && request->body == id, "short secondary click targets the object");
        RIGIDBODIES_EXPECT(session.selection() == id && session.selections().size() == 1, "the menu's object becomes the selection its commands act on");
        session.set_selection(id);
        session.request_context_menu_for_selection();
        const auto keyboard = session.build_model().context_menu_request;
        RIGIDBODIES_EXPECT(keyboard && keyboard->kind == ui::StageTargetKind::object && keyboard->body == id, "keyboard menu targets the current selection");
    }
    {
        app::SimulationSession session;
        session.world().clear();
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
        const auto first = circle(session, { -1.0, 0.0 }, .2);
        const auto second = circle(session, { 1.0, 0.0 }, .2);
        session.set_selections({ first, second });
        const auto inside = session.camera().world_to_screen({ 1.0, 0.0 });
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, inside));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, inside));
        RIGIDBODIES_EXPECT(session.selections().size() == 2, "right-clicking inside a multi-selection keeps the group");
        const auto third = circle(session, { 0.0, 1.0 }, .2);
        const auto outside = session.camera().world_to_screen({ 0.0, 1.0 });
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, outside));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, outside));
        RIGIDBODIES_EXPECT(session.selections().size() == 1 && session.selection() == third, "right-clicking another object selects it alone");
    }
    {
        app::SimulationSession session;
        session.world().clear();
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
        const auto point = session.camera().world_to_screen({ 1.5, 1.0 });
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, point));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, point));
        const auto request = session.build_model().context_menu_request;
        RIGIDBODIES_EXPECT(request && request->kind == ui::StageTargetKind::empty_space, "empty-space click targets the stage");
    }
    {
        app::SimulationSession session;
        session.world().clear();
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
        const auto point = session.camera().world_to_screen({});
        const auto before = session.camera().center_m();
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, point));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_move, point + math::Vec2 { 4, 0 }));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, point + math::Vec2 { 4, 0 }));
        RIGIDBODIES_EXPECT(math::length_squared(session.camera().center_m() - before) > 0.0 && !session.build_model().context_menu_request, "four pixels becomes a pan and opens no menu");
    }
    {
        app::SimulationSession session;
        session.configure({});
        session.set_viewport({ 1000, 800 });
        session.load_scenario("distance_chain");
        ui::UiCommand show;
        show.kind = ui::UiCommandKind::set_layer;
        show.id = "constraints";
        show.flag = true;
        session.apply(show);
        const auto model = session.build_model();
        const auto handle = std::find_if(model.handles.begin(), model.handles.end(), [](const auto& item)
            {
                return item.kind == ui::StageTargetKind::connection;
            });
        RIGIDBODIES_EXPECT(handle != model.handles.end(), "connection handle is available");
        const auto point = handle->screen_bounds.center();
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_down, point));
        session.handle_scene_event(secondary(ui::UiEventKind::pointer_up, point));
        const auto request = session.build_model().context_menu_request;
        RIGIDBODIES_EXPECT(request && request->kind == ui::StageTargetKind::connection && !request->id.empty(), "connection glyph gets its own menu target");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
