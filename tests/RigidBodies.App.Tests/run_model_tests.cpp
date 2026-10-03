#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

#include <algorithm>

namespace
{
    using namespace rigidbodies;

    physics::BodyId first_dynamic(app::SimulationSession& session)
    {
        physics::BodyId result;
        session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (!result.is_valid() && body.type() == physics::BodyType::dynamic_body)
                    result = id;
            });
        return result;
    }

    void value(app::SimulationSession& session, ui::UiCommandKind kind, double amount)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.value = amount;
        session.apply(command);
    }

    void invoke(app::SimulationSession& session, ui::UiCommandKind kind, bool flag = false)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.flag = flag;
        session.apply(command);
    }
}

RIGIDBODIES_TEST("parameter edits at Ready and during a run survive Back to start")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    core::ApplicationConfig config;
    config.simulation.start_paused = false;
    session.configure(config);
    const auto id = first_dynamic(session);
    session.set_selection(id);
    value(session, ui::UiCommandKind::set_selected_mass, 2.0);
    session.advance(0.1);
    invoke(session, ui::UiCommandKind::reset_scenario);
    RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->mass_properties().mass_kg, 2.0, 1.0e-12, "Ready parameter edit is in the setup");
    invoke(session, ui::UiCommandKind::toggle_pause);
    session.advance(0.1);
    value(session, ui::UiCommandKind::set_selected_mass, 3.0);
    invoke(session, ui::UiCommandKind::reset_scenario);
    RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "Back to start lands at t zero");
    RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->mass_properties().mass_kg, 3.0, 1.0e-12, "mid-run parameter edit is also applied to the setup");
}

RIGIDBODIES_TEST("mid-run state edit is live only and offers Keep once")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    core::ApplicationConfig config;
    config.simulation.start_paused = false;
    session.configure(config);
    const auto id = first_dynamic(session);
    session.set_selection(id);
    session.advance(0.1);
    value(session, ui::UiCommandKind::set_selected_velocity_x, 1.0);
    const auto keep_count = [](const ui::UiModel& model)
    {
        return std::count_if(model.notifications.begin(), model.notifications.end(), [](const auto& notification)
            {
                return notification.action && notification.action->label == "Keep as starting state";
            });
    };
    const auto after_first = keep_count(session.build_model());
    value(session, ui::UiCommandKind::set_selected_velocity_y, 2.0);
    RIGIDBODIES_EXPECT(after_first == 1 && keep_count(session.build_model()) == 1, "one Keep as starting state toast is posted per run");
    invoke(session, ui::UiCommandKind::reset_scenario);
    RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s() == math::Vec2 {}, "Back to start discards mid-run state edits");
}

RIGIDBODIES_TEST("Keep as starting state rebases the current world at t zero")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    core::ApplicationConfig config;
    config.simulation.start_paused = false;
    session.configure(config);
    const auto id = first_dynamic(session);
    session.advance(0.1);
    const auto position = session.world().find_body(id)->position_m();
    const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
    invoke(session, ui::UiCommandKind::keep_state_as_setup);
    RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.world().statistics().elapsed_time_s == 0.0, "Keep lands Ready at t zero");
    RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == position && session.world().find_body(id)->linear_velocity_m_s() == velocity, "the pre-command world becomes the setup");
    invoke(session, ui::UiCommandKind::toggle_pause);
    session.advance(0.05);
    invoke(session, ui::UiCommandKind::reset_scenario);
    RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == position && session.world().find_body(id)->linear_velocity_m_s() == velocity, "the rebased setup is restorable");
}

RIGIDBODIES_TEST("Restore original removes setup changes, keeps lab settings, and is undoable")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    const auto id = first_dynamic(session);
    session.set_selection(id);
    const auto original_mass = session.world().find_body(id)->mass_properties().mass_kg;
    value(session, ui::UiCommandKind::set_selected_mass, 2.0);
    physics::BodyDefinition extra;
    extra.name = "Added";
    extra.type = physics::BodyType::dynamic_body;
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    extra.colliders.push_back(collider);
    const auto added = session.world().create_body(extra);
    invoke(session, ui::UiCommandKind::keep_state_as_setup);
    ui::UiCommand integrator;
    integrator.kind = ui::UiCommandKind::set_integrator;
    integrator.id = "runge_kutta_4";
    session.apply(integrator);
    invoke(session, ui::UiCommandKind::restore_original);
    RIGIDBODIES_EXPECT(!session.world().is_valid(added) && session.world().find_body(id)->mass_properties().mass_kg == original_mass, "original parameters and structure are restored");
    RIGIDBODIES_EXPECT(session.world().integrator().name() == "runge_kutta_4", "lab settings survive Restore original");
    invoke(session, ui::UiCommandKind::undo);
    RIGIDBODIES_EXPECT(session.world().is_valid(added) && session.world().find_body(id)->mass_properties().mass_kg == 2.0, "Undo reverses Restore original");
}

RIGIDBODIES_TEST("Replay runs from t zero while Back to start remains Ready")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    core::ApplicationConfig config;
    config.simulation.start_paused = false;
    session.configure(config);
    session.advance(0.1);
    invoke(session, ui::UiCommandKind::reset_scenario, true);
    RIGIDBODIES_EXPECT(!session.stepper().is_paused() && session.world().statistics().elapsed_time_s == 0.0, "Replay is running at t zero");
    session.advance(0.05);
    invoke(session, ui::UiCommandKind::reset_scenario);
    RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.world().statistics().elapsed_time_s == 0.0, "Back to start is Ready at t zero");
}

int main()
{
    return rigidbodies::testing::run_all();
}
