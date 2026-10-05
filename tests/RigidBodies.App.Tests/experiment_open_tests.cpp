#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;
    void command(app::SimulationSession& session, ui::UiCommandKind kind, double number = 0.0, bool flag = false, std::string id = {})
    {
        ui::UiCommand request;
        request.kind = kind;
        request.value = number;
        request.flag = flag;
        request.id = std::move(id);
        session.apply(request);
    }
}

RIGIDBODIES_TEST("opening an experiment applies the complete scoping table")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    command(session, ui::UiCommandKind::set_gravity_magnitude, 1.62);
    command(session, ui::UiCommandKind::set_integrator, 0.0, false, "runge_kutta_4");
    command(session, ui::UiCommandKind::set_time_scale, 0.5);
    command(session, ui::UiCommandKind::set_pause_on_impact, 0.0, true);
    session.set_selection(session.world().body_ids().front());
    command(session, ui::UiCommandKind::set_interaction_mode, 0.0, false, "throw");
    RIGIDBODIES_EXPECT(session.load_scenario("ramp"), "Ramp and friction exists");
    const auto confirmation = session.build_model().confirmation;
    RIGIDBODIES_EXPECT(confirmation && session.scenario_id() == "free_fall", "opening another experiment waits for confirmation of the unsaved edits");
    session.apply(confirmation->confirm);
    RIGIDBODIES_EXPECT(session.scenario_id() == "ramp" && !session.build_model().confirmation, "confirming discard opens Ramp and friction");
    const auto model = session.build_model();
    RIGIDBODIES_EXPECT_NEAR(math::length(session.world().settings().gravity_m_s2), physics::standard_gravity_m_s2, 1.0e-12, "the new experiment supplies gravity");
    RIGIDBODIES_EXPECT(session.world().integrator().name() == "runge_kutta_4" && model.lab_changes.size() == 1, "lab setting is kept and reported by the chip model");
    RIGIDBODIES_EXPECT_NEAR(session.stepper().time_scale(), 0.5, 0.0, "playback speed is kept");
    RIGIDBODIES_EXPECT(!model.pause_on_impact && !model.selection.is_valid() && model.interaction_mode == "select", "pause-on-impact, selection, and tool are reset");
    RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == 0.0, "opening lands at t zero");
    command(session, ui::UiCommandKind::undo);
    RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall", "opening is undoable");
}

RIGIDBODIES_TEST("opening Chasing light applies the scoping table and undo returns to the Newtonian experiment")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    command(session, ui::UiCommandKind::set_time_scale, 0.5);
    command(session, ui::UiCommandKind::set_pause_on_impact, 0.0, true);
    session.set_selection(session.world().body_ids().front());
    command(session, ui::UiCommandKind::toggle_pause);
    session.advance(0.25);
    RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light exists");
    if (const auto confirmation = session.build_model().confirmation)
        session.apply(confirmation->confirm);
    const auto model = session.build_model();
    RIGIDBODIES_EXPECT(session.scenario_id() == "chasing_light" && session.relativity_active() && model.relativity, "the relativity experiment opens with its probe");
    RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == 0.0 && session.relativity_probe()->race().lab_time_s == 0.0 && session.stepper().is_paused(), "it opens paused at t zero with both clocks at zero");
    RIGIDBODIES_EXPECT_NEAR(session.stepper().time_scale(), 0.5, 0.0, "playback speed is kept");
    RIGIDBODIES_EXPECT(!model.pause_on_impact && !model.selection.is_valid() && model.interaction_mode == "select", "pause-on-impact, selection and tool are reset");
    RIGIDBODIES_EXPECT(math::length(session.world().settings().gravity_m_s2) == 0.0, "the probe's track has no gravity");
    command(session, ui::UiCommandKind::undo);
    RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall" && !session.relativity_active() && !session.build_model().relativity, "undo returns to free fall without a probe");
    command(session, ui::UiCommandKind::redo);
    RIGIDBODIES_EXPECT(session.scenario_id() == "chasing_light" && session.relativity_active() && session.relativity_probe()->setup().speed_fraction == 0.0, "redo reopens Chasing light at rest");
}

RIGIDBODIES_TEST("Keep lab settings preference can reset lab settings on open")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    command(session, ui::UiCommandKind::set_integrator, 0.0, false, "runge_kutta_4");
    ui::UiCommand preference;
    preference.kind = ui::UiCommandKind::set_preference;
    preference.id = preference.detail = "prefs.experiments.keep_lab_settings";
    preference.flag = false;
    session.apply(preference);
    RIGIDBODIES_EXPECT(session.load_scenario("ramp"), "another experiment exists");
    const auto confirmation = session.build_model().confirmation;
    RIGIDBODIES_EXPECT(confirmation && session.scenario_id() == "free_fall", "the edited lab settings are protected until departure is confirmed");
    session.apply(confirmation->confirm);
    RIGIDBODIES_EXPECT(session.scenario_id() == "ramp" && !session.build_model().confirmation, "confirming discard opens the next experiment");
    RIGIDBODIES_EXPECT(session.world().integrator().name() == "semi_implicit_euler" && session.build_model().lab_changes.empty(), "the new experiment's lab defaults apply when the preference is off");
}

int main()
{
    return rigidbodies::testing::run_all();
}
