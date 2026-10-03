#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    void send(app::SimulationSession& session, ui::UiCommandKind kind, double value = 0.0, std::string id = {}, physics::BodyId body = {})
    {
        ui::UiCommand command;
        command.kind = kind;
        command.value = value;
        command.id = std::move(id);
        command.body = body;
        session.apply(command);
    }

    std::vector<physics::BodyId> dynamic_bodies(const app::SimulationSession& session)
    {
        std::vector<physics::BodyId> result;
        for (const auto id : session.world().body_ids())
        {
            const auto* body = session.world().find_body(id);
            if (body != nullptr && body->type() == physics::BodyType::dynamic_body)
                result.push_back(id);
        }
        return result;
    }

    RIGIDBODIES_TEST("parameter undo changes neither time nor play state and restores the starting setup")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture loads");
        const auto body = dynamic_bodies(session).front();
        session.set_selection(body);
        session.stepper().set_paused(false);
        session.advance(0.5);
        const auto time = session.world().statistics().elapsed_time_s;
        send(session, ui::UiCommandKind::set_selected_mass, 2.0, {}, body);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(body)->mass_properties().mass_kg, 2.0, 0.0, "mass changes live");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, time, 0.0, "parameter undo never rewinds");
        RIGIDBODIES_EXPECT(!session.stepper().is_paused(), "parameter undo preserves play state");
        const auto restored_mass = session.world().find_body(body)->mass_properties().mass_kg;
        session.reset_scenario();
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(body)->mass_properties().mass_kg, restored_mass, 0.0, "undo also restores the starting setup property");
    }

    RIGIDBODIES_TEST("state undo returns to its moment pauses and never moves the camera")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        const auto body = dynamic_bodies(session).front();
        session.set_selection(body);
        session.stepper().set_paused(false);
        session.advance(0.25);
        const auto time = session.world().statistics().elapsed_time_s;
        session.camera().set_center({ 8.0, -3.0 });
        send(session, ui::UiCommandKind::set_selected_velocity_x, 4.0, {}, body);
        session.advance(0.25);
        const auto camera = session.camera().center_m();
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "state undo pauses for inspection");
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, time, 1.0e-12, "state undo returns to the edit moment");
        RIGIDBODIES_EXPECT(session.camera().center_m() == camera, "undo never moves the view");
        RIGIDBODIES_EXPECT(session.build_model().pause_reason.reason == ui::PauseReason::after_undo, "the badge explains the pause");
    }

    RIGIDBODIES_TEST("mixed-selection mass undo restores each distinct original value")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        const auto bodies = dynamic_bodies(session);
        RIGIDBODIES_EXPECT(bodies.size() >= 2, "fixture has two editable bodies");
        const auto first = bodies[0];
        const auto second = bodies[1];
        session.world().find_body(first)->override_mass(0.5);
        session.world().find_body(second)->override_mass(3.0);
        session.set_selections({ first, second });
        send(session, ui::UiCommandKind::set_selected_mass, 1.0);
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(first)->mass_properties().mass_kg, 0.5, 0.0, "first original is exact");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(second)->mass_properties().mass_kg, 3.0, 0.0, "second original is exact");
    }

    RIGIDBODIES_TEST("playback speed is never recorded in undo history")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        const auto before = session.build_model().undo_history.size();
        send(session, ui::UiCommandKind::set_time_scale, 0.5);
        RIGIDBODIES_EXPECT(session.build_model().undo_history.size() == before, "speed is interface transport state, not an edit");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
