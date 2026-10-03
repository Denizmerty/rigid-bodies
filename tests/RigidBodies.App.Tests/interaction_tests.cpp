#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <limits>

namespace
{
    using namespace rigidbodies;

    void command(app::SimulationSession& session, ui::UiCommandKind kind, std::string id = {})
    {
        ui::UiCommand request;
        request.kind = kind;
        request.id = std::move(id);
        session.apply(request);
    }

    void isolate(app::SimulationSession& session)
    {
        session.world().clear();
        auto settings = session.world().settings();
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        session.world().set_settings(settings);
        session.stepper().set_fixed_step(0.01);
        session.stepper().set_substep_count(1);
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
    }

    physics::BodyId disc(app::SimulationSession& session, math::Vec2 position = {}, physics::BodyType type = physics::BodyType::dynamic_body)
    {
        physics::BodyDefinition definition;
        definition.name = "Drag target";
        definition.type = type;
        definition.position_m = position;
        definition.sleep_enabled = false;
        physics::Collider collider;
        collider.shape = physics::make_circle(0.15);
        definition.colliders.push_back(collider);
        return session.world().create_body(definition);
    }

    bool pointer(app::SimulationSession& session, ui::UiEventKind kind, const math::Vec2& world, double time = 0.0, bool shift = false, bool consumed = false)
    {
        ui::UiEvent event;
        event.kind = kind;
        event.pointer_px = session.camera().world_to_screen(world);
        event.timestamp_s = time;
        event.modifiers.shift = shift;
        return session.handle_scene_event(event, consumed);
    }

    RIGIDBODIES_TEST("shift selection toggles bodies and an empty shifted click preserves the group")
    {
        app::SimulationSession session;
        isolate(session);
        const auto a = disc(session, { -1.0, 0.0 });
        const auto b = disc(session, { 1.0, 0.0 });
        pointer(session, ui::UiEventKind::pointer_down, { -1.0, 0.0 });
        pointer(session, ui::UiEventKind::pointer_up, { -1.0, 0.0 });
        pointer(session, ui::UiEventKind::pointer_down, { 1.0, 0.0 }, 0.0, true);
        pointer(session, ui::UiEventKind::pointer_up, { 1.0, 0.0 }, 0.0, true);
        RIGIDBODIES_EXPECT(session.selections().size() == 2 && session.selection() == b, "Shift appends a primary selection");
        pointer(session, ui::UiEventKind::pointer_down, { 2.0, 1.0 }, 0.0, true);
        RIGIDBODIES_EXPECT(session.selections().size() == 2, "Shift empty-space does not discard a group");
        pointer(session, ui::UiEventKind::pointer_down, { 1.0, 0.0 }, 0.0, true);
        pointer(session, ui::UiEventKind::pointer_up, { 1.0, 0.0 }, 0.0, true);
        RIGIDBODIES_EXPECT(session.selections().size() == 1 && session.selection() == a, "Shift selected-body removes it");
        pointer(session, ui::UiEventKind::pointer_down, { 2.0, 1.0 });
        RIGIDBODIES_EXPECT(session.selections().empty(), "plain empty-space clears selection");
    }

    RIGIDBODIES_TEST("group movement is camera-scaled, bounded, undoable and releases over a panel")
    {
        app::SimulationSession session;
        isolate(session);
        const auto a = disc(session, { -1.0, 0.0 });
        const auto b = disc(session, { 1.0, 0.0 });
        auto settings = session.world().settings();
        settings.limits.maximum_position_m = 3.0;
        session.world().set_settings(settings);
        session.camera().set_view_height(8.0);
        session.set_selections({ a, b });
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        pointer(session, ui::UiEventKind::pointer_down, { -1.0, 0.0 });
        pointer(session, ui::UiEventKind::pointer_move, { 0.0, 0.5 });
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(a)->position_m().y, 0.5, 1.0e-12, "device coordinates map back through the zoomed camera");
        pointer(session, ui::UiEventKind::pointer_move, { 20.0, 0.5 });
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(b)->position_m().x, 3.0, 1.0e-12, "furthest member constrains the entire group");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(a)->position_m().x, 1.0, 1.0e-12, "relative placement survives world limits");
        RIGIDBODIES_EXPECT(pointer(session, ui::UiEventKind::pointer_up, { 20.0, 0.5 }, 0.0, false, true), "a consumed release still reaches the gesture");
        RIGIDBODIES_EXPECT(!session.interaction_active() && session.build_model().can_undo, "one completed gesture is one history entry");
        command(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(a)->position_m().x, -1.0, 0.0, "undo restores all group members");
        command(session, ui::UiCommandKind::redo);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(b)->position_m().x, 3.0, 0.0, "redo restores the committed placement");
    }

    RIGIDBODIES_TEST("static and prescribed bodies remain selectable but cannot be dragged")
    {
        app::SimulationSession session;
        isolate(session);
        const auto a = disc(session, {}, physics::BodyType::static_body);
        const auto b = disc(session, { 1.0, 0.0 }, physics::BodyType::kinematic_body);
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        for (const auto id : { a, b })
        {
            const auto position = session.world().find_body(id)->position_m();
            pointer(session, ui::UiEventKind::pointer_down, position);
            pointer(session, ui::UiEventKind::pointer_move, position + math::Vec2 { 0.5, 0.5 });
            pointer(session, ui::UiEventKind::pointer_up, position);
            RIGIDBODIES_EXPECT(!session.interaction_active() && session.selection() == id, "fixed and prescribed bodies can be inspected without a capture");
            RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == position, "the prescribed placement does not change");
        }
        RIGIDBODIES_EXPECT(!session.build_model().can_undo, "selection alone creates no scenario history");
    }

    RIGIDBODIES_TEST("move captures freeze simulation and Escape restores the pre-gesture world")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        session.world().find_body(id)->set_linear_velocity({ 2.0, 0.0 });
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        pointer(session, ui::UiEventKind::pointer_down, {});
        pointer(session, ui::UiEventKind::pointer_move, { 0.5, 0.0 });
        session.advance(0.1);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "moving a body is an edit with no hidden physics drift");
        ui::UiEvent cancel;
        cancel.kind = ui::UiEventKind::key_down;
        cancel.key = ui::UiKey::escape;
        session.handle_scene_event(cancel, true);
        RIGIDBODIES_EXPECT(!session.interaction_active() && !session.build_model().can_undo, "cancelling discards the pending edit");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, 0.0, 0.0, "cancelling restores the original position");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 2.0, 0.0, "cancelling restores the original velocity");
    }

    RIGIDBODIES_TEST("focus loss cancels manipulation even when the interface consumes the event")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        pointer(session, ui::UiEventKind::pointer_down, {});
        pointer(session, ui::UiEventKind::pointer_move, { 0.5, 0.0 });
        ui::UiEvent event;
        event.kind = ui::UiEventKind::focus_lost;
        session.handle_scene_event(event, true);
        RIGIDBODIES_EXPECT(!session.interaction_active(), "focus loss never leaves a captured pointer behind");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, 0.0, 0.0, "unintentional focus changes restore the world");
    }

    RIGIDBODIES_TEST("throwing uses timed world motion and clamps the release velocity")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        command(session, ui::UiCommandKind::set_interaction_mode, "throw");
        pointer(session, ui::UiEventKind::pointer_down, {}, 1.0);
        pointer(session, ui::UiEventKind::pointer_move, { 0.4, 0.0 }, 1.1);
        pointer(session, ui::UiEventKind::pointer_up, { 0.4, 0.0 }, 1.1);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 4.0, 1.0e-10, "throw speed is measured in metres per second");
        pointer(session, ui::UiEventKind::pointer_down, { 0.4, 0.0 }, 2.0);
        pointer(session, ui::UiEventKind::pointer_move, { 10.4, 0.0 }, 2.01);
        pointer(session, ui::UiEventKind::pointer_up, { 10.4, 0.0 }, 2.01);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 25.0, 1.0e-10, "throw speed has an explicit finite cap");
    }

    RIGIDBODIES_TEST("holding a throw still before release removes stale motion and clicks do not stop bodies")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        command(session, ui::UiCommandKind::set_interaction_mode, "throw");
        session.world().find_body(id)->set_linear_velocity({ 3.0, 0.0 });
        pointer(session, ui::UiEventKind::pointer_down, {}, 1.0);
        pointer(session, ui::UiEventKind::pointer_up, {}, 1.1);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 3.0, 0.0, "a plain click in throw mode leaves motion intact");
        pointer(session, ui::UiEventKind::pointer_down, {}, 2.0);
        pointer(session, ui::UiEventKind::pointer_move, { 0.4, 0.0 }, 2.1);
        pointer(session, ui::UiEventKind::pointer_up, { 0.4, 0.0 }, 2.5);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 0.0, 0.0, "holding the target still removes the old throwing impulse");
    }

    RIGIDBODIES_TEST("pulling applies a named force at the grabbed material point on every substep")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        session.stepper().set_substep_count(4);
        command(session, ui::UiCommandKind::set_interaction_mode, "pull");
        pointer(session, ui::UiEventKind::pointer_down, { 0.0, 0.1 });
        pointer(session, ui::UiEventKind::pointer_move, { 0.5, 0.1 });
        session.advance(0.01);
        const auto* body = session.world().find_body(id);
        RIGIDBODIES_EXPECT(body->position_m().x > 0.0 && body->linear_velocity_m_s().x > 0.2, "a running pull accelerates instead of teleporting");
        RIGIDBODIES_EXPECT(body->angular_velocity_rad_s() < 0.0, "the off-centre grab produces clockwise torque");
        RIGIDBODIES_EXPECT(std::any_of(body->applied_force_channels().begin(), body->applied_force_channels().end(), [](const auto& channel)
                               {
                                   return channel.name == "Pointer spring";
                               }),
            "the final substep contains the explicit teaching force channel");
        pointer(session, ui::UiEventKind::pointer_up, { 0.5, 0.1 });
        session.advance(0.01);
        const auto& channels = session.world().find_body(id)->applied_force_channels();
        RIGIDBODIES_EXPECT(std::none_of(channels.begin(), channels.end(), [](const auto& channel)
                               {
                                   return channel.name == "Pointer spring";
                               }),
            "the pointer spring stops immediately after release");
        RIGIDBODIES_EXPECT(session.world().force_generators(id).empty(), "the temporary generator never enters the committed world");
        command(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "undo restores the complete pre-pull experiment");
    }

    RIGIDBODIES_TEST("panel presses and nonfinite coordinates cannot move a scene body")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session);
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        pointer(session, ui::UiEventKind::pointer_down, {}, 0.0, false, true);
        RIGIDBODIES_EXPECT(!session.interaction_active(), "a panel press does not reach a body beneath it");
        pointer(session, ui::UiEventKind::pointer_down, {});
        ui::UiEvent event;
        event.kind = ui::UiEventKind::pointer_move;
        event.pointer_px = { std::numeric_limits<double>::infinity(), 10.0 };
        session.handle_scene_event(event);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, 0.0, 0.0, "invalid coordinates never reach the physics state");
        pointer(session, ui::UiEventKind::pointer_up, {});
    }

    RIGIDBODIES_TEST("pull springs are sampled at staged integrator poses and never survive cancellation")
    {
        for (int integrator = 0; integrator < 3; ++integrator)
        {
            app::SimulationSession session;
            isolate(session);
            const auto id = disc(session);
            ui::UiCommand integrator_command;
            integrator_command.kind = ui::UiCommandKind::set_integrator;
            integrator_command.id = integrator == 0 ? "semi_implicit_euler" : integrator == 1 ? "velocity_verlet"
                                                                                              : "runge_kutta_4";
            session.apply(integrator_command);
            command(session, ui::UiCommandKind::set_interaction_mode, "pull");
            pointer(session, ui::UiEventKind::pointer_down, {});
            pointer(session, ui::UiEventKind::pointer_move, { 0.5, 0.0 });
            for (int index = 0; index < 20; ++index)
                session.advance(0.01);
            const double frequency = std::sqrt(11.0);
            const double expected = 0.5 * (1.0 - std::exp(-1.4) * (std::cos(frequency * 0.2) + 7.0 / frequency * std::sin(frequency * 0.2)));
            RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, expected, integrator == 2 ? 2.0e-6 : 0.015, "the mass-normalized damped spring follows its analytic step response");
            ui::UiEvent cancel;
            cancel.kind = ui::UiEventKind::focus_lost;
            session.handle_scene_event(cancel);
            RIGIDBODIES_EXPECT(session.world().force_generators(id).empty(), "cancellation restores a world without transient force registrations");
            RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, 0.0, 0.0, "cancellation restores the captured experiment for every integrator");
        }
    }

    RIGIDBODIES_TEST("the keyboard reference drives exact modifier matching and one command mapping")
    {
        for (const auto& binding : app::key_bindings)
        {
            RIGIDBODIES_EXPECT(!binding.chord.empty() && !binding.description.empty(), "every binding has a visible explanation");
            RIGIDBODIES_EXPECT(app::action_for_key(binding.key, binding.modifiers) == binding.action, "displayed bindings dispatch the documented action");
        }
        RIGIDBODIES_EXPECT(app::action_for_key(ui::UiKey::r, { false, true, false }) == app::AppAction::none, "Ctrl+R cannot silently reset a scenario");
        RIGIDBODIES_EXPECT(app::action_for_key(ui::UiKey::z, { true, true, false }) == app::AppAction::redo, "redo accepts the conventional shifted chord");
        RIGIDBODIES_EXPECT(app::command_for_action(app::AppAction::pull_mode).id == "pull", "mode shortcuts and visible controls use the same command");
        RIGIDBODIES_EXPECT(app::command_for_action(app::AppAction::undo).kind == ui::UiCommandKind::undo, "history uses one shared command path");
        RIGIDBODIES_EXPECT(app::action_for_key(ui::UiKey::h, {}) == app::AppAction::none, "H has no application binding");
        RIGIDBODIES_EXPECT(app::key_bindings.size() >= 28, "the shell registry contains the complete shortcut set");
        RIGIDBODIES_EXPECT(app::command_for_action(app::action_for_key(ui::UiKey::f, {})).kind == ui::UiCommandKind::frame_subject, "F dispatches frame subject");
        const auto frame = std::find_if(app::key_bindings.begin(), app::key_bindings.end(), [](const auto& binding)
            {
                return binding.key == ui::UiKey::f && !binding.modifiers.shift;
            });
        RIGIDBODIES_EXPECT(frame != app::key_bindings.end() && frame->description == "Frame subject", "keyboard reference names Frame subject");
    }

    RIGIDBODIES_TEST("a coalesced button release includes its final position and Ctrl+A selects free objects only")
    {
        app::SimulationSession session;
        isolate(session);
        const auto moving = disc(session);
        const auto fixed = disc(session, { -1.0, 0.0 }, physics::BodyType::static_body);
        command(session, ui::UiCommandKind::select_all);
        RIGIDBODIES_EXPECT(session.selections() == std::vector<physics::BodyId> { moving }, "select all includes free objects and excludes fixed objects");
        command(session, ui::UiCommandKind::set_interaction_mode, "move");
        pointer(session, ui::UiEventKind::pointer_down, {});
        pointer(session, ui::UiEventKind::pointer_up, { 0.4, 0.2 }, 0.1, false, true);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(moving)->position_m().x, 0.4, 1.0e-12, "the final device position survives a missing motion event");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(fixed)->position_m().x, -1.0, 0.0, "fixed members stay in place during a group edit");
    }

    RIGIDBODIES_TEST("Free fall selection and deletion keep fixed objects unless one is selected alone")
    {
        app::SimulationSession session;
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "Free fall loads for the selection gate");
        std::vector<physics::BodyId> free_objects;
        physics::BodyId floor;
        for (const auto id : session.world().body_ids())
        {
            const auto* body = session.world().find_body(id);
            if (body->type() == physics::BodyType::dynamic_body)
                free_objects.push_back(id);
            else if (body->type() == physics::BodyType::static_body)
                floor = id;
        }
        RIGIDBODIES_EXPECT(free_objects.size() == 3 && floor.is_valid(), "Free fall has three free objects and one fixed floor");

        command(session, ui::UiCommandKind::select_all);
        RIGIDBODIES_EXPECT(session.selections() == free_objects, "Ctrl+A selects exactly Free fall's free objects");

        const auto ball = free_objects.front();
        session.set_selections({ floor, ball });
        command(session, ui::UiCommandKind::delete_selected_body);
        RIGIDBODIES_EXPECT(session.world().find_body(ball) == nullptr && session.world().find_body(floor) != nullptr, "Delete removes a selected ball and keeps the selected floor");
        RIGIDBODIES_EXPECT(session.selections() == std::vector<physics::BodyId> { floor }, "the fixed object kept by Delete remains selected");

        session.set_selection(floor);
        command(session, ui::UiCommandKind::delete_selected_body);
        RIGIDBODIES_EXPECT(session.world().find_body(floor) == nullptr && session.selections().empty(), "Delete removes a fixed floor selected on its own");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
