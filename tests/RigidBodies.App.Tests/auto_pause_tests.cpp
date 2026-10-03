#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    void send(app::SimulationSession& session, ui::UiCommandKind kind, bool flag = false)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.flag = flag;
        session.apply(command);
    }

    RIGIDBODIES_TEST("Draw pauses on entry but Space controls real time under the draft")
    {
        app::SimulationSession session;
        session.load_scenario("empty_lab");
        session.stepper().set_paused(false);
        send(session, ui::UiCommandKind::start_new_shape);
        auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.paused && model.pause_reason.reason == ui::PauseReason::drawing, "Draw entry has an explicit pause reason");
        send(session, ui::UiCommandKind::toggle_pause);
        const auto before = session.world().statistics().elapsed_time_s;
        session.advance(0.1);
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s > before && session.shape_editor().active(), "time advances while the draft remains open");
        send(session, ui::UiCommandKind::toggle_pause);
        send(session, ui::UiCommandKind::cancel_shape_outline);
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "Discard preserves the current pause state");
    }

    RIGIDBODIES_TEST("pause at next impact arms plays stops once and disarms")
    {
        app::SimulationSession session;
        session.world().clear();
        auto settings = session.world().settings();
        settings.gravity_m_s2 = {};
        session.world().set_settings(settings);
        for (const auto [x, vx] : { std::pair { -0.12, 2.0 }, std::pair { 0.12, -2.0 } })
        {
            physics::BodyDefinition definition;
            definition.position_m = { x, 0.0 };
            definition.linear_velocity_m_s = { vx, 0.0 };
            physics::Collider collider;
            collider.shape = physics::make_circle(0.1);
            collider.material.restitution = 1.0;
            collider.material.static_friction = collider.material.kinetic_friction = 0.0;
            definition.colliders.push_back(collider);
            session.world().create_body(definition);
        }
        session.stepper().set_fixed_step(0.01);
        send(session, ui::UiCommandKind::pause_at_next_impact, true);
        RIGIDBODIES_EXPECT(!session.stepper().is_paused() && session.build_model().next_impact_armed, "arming from Paused starts the run");
        session.advance(0.2);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.paused && !model.next_impact_armed && model.pause_reason.reason == ui::PauseReason::impact, "the first moving-pair impact pauses and consumes the one-shot");
        RIGIDBODIES_EXPECT(model.banner.has_value() && !model.impacts.empty(), "the pause provides evidence without opening the drawer");
    }

    RIGIDBODIES_TEST("background pause publishes its reason without auto-resuming")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.simulation.start_paused = false;
        config.controls.pause_in_background = true;
        session.configure(config);
        session.set_window_backgrounded(true);
        RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.build_model().pause_reason.reason == ui::PauseReason::in_background, "focus loss pauses with an explanatory badge");
        session.set_window_backgrounded(false);
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "returning to the app does not unexpectedly start time");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
