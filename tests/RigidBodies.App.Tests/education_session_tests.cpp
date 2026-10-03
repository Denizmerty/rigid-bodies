#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <limits>

namespace
{
    using namespace rigidbodies;

    void command(app::SimulationSession& session, ui::UiCommandKind kind, double value = 0.0, bool flag = false, std::string id = {}, physics::BodyId body = {})
    {
        ui::UiCommand request;
        request.kind = kind;
        request.value = value;
        request.flag = flag;
        request.id = std::move(id);
        request.body = body;
        session.apply(request);
    }

    void isolate(app::SimulationSession& session)
    {
        (void)session.load_scenario("empty_lab");
        session.world().clear();
        auto settings = session.world().settings();
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        session.world().set_settings(settings);
        session.stepper().set_fixed_step(0.01);
        session.stepper().set_substep_count(1);
        session.stepper().set_paused(false);
        session.set_viewport({ 1000, 800 });
    }

    physics::BodyId disc(app::SimulationSession& session, const math::Vec2& position, const math::Vec2& velocity, double mass = 1.0, bool sensor = false)
    {
        physics::BodyDefinition body;
        body.name = position.x < 0.0 ? "Left disc" : "Right disc";
        body.position_m = position;
        body.linear_velocity_m_s = velocity;
        body.sleep_enabled = false;
        physics::Collider collider;
        collider.shape = physics::make_circle(0.1);
        collider.material = physics::materials::oak_wood();
        collider.material.restitution = 1.0;
        collider.material.static_friction = collider.material.kinetic_friction = 0.0;
        collider.material.rolling_friction_m = collider.material.spinning_friction_m = 0.0;
        collider.is_sensor = sensor;
        body.colliders.push_back(collider);
        const auto id = session.world().create_body(body);
        session.world().find_body(id)->override_mass(mass);
        return id;
    }

    physics::BodyId named_body(const app::SimulationSession& session, std::string_view name)
    {
        for (const auto id : session.world().body_ids())
            if (session.world().find_body(id)->name() == name)
                return id;
        RIGIDBODIES_FAIL("the scenario includes the requested body");
    }

    const ui::RunRecord& current_run(const app::SimulationSession& session)
    {
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.current_run != nullptr, "advancing the experiment opens a recorded run");
        return *model.current_run;
    }

    RIGIDBODIES_TEST("run sampling follows simulated time and paused frames never create measurements")
    {
        app::SimulationSession session;
        isolate(session);
        disc(session, {}, { 0.1, 0.0 });
        command(session, ui::UiCommandKind::clear_energy_history);
        for (int frame = 0; frame < 210; ++frame)
            session.advance(0.01);
        const auto samples = current_run(session).series.time_s;
        RIGIDBODIES_EXPECT(samples.size() == 85, "the recorder samples the initial instant and forty times per simulated second");
        for (std::size_t index = 1; index < samples.size(); ++index)
            RIGIDBODIES_EXPECT(samples[index] - samples[index - 1] >= 0.025f - 1.0e-6f, "normal sampling is limited to forty measurements per simulated second");
        const auto elapsed = session.world().statistics().elapsed_time_s;
        command(session, ui::UiCommandKind::toggle_pause);
        for (int frame = 0; frame < 40; ++frame)
            session.advance(0.2);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, elapsed, 0.0, "paused frame time does not advance the world");
        RIGIDBODIES_EXPECT(current_run(session).series.time_s.size() == samples.size(), "paused frames append no samples");
        RIGIDBODIES_EXPECT_NEAR(current_run(session).series.time_s.back(), samples.back(), 0.0, "the last measurement remains the last simulated sample");
        command(session, ui::UiCommandKind::clear_energy_history);
        RIGIDBODIES_EXPECT(current_run(session).series.time_s.size() == samples.size(), "Clear graph preserves the recorded run");
        RIGIDBODIES_EXPECT_NEAR(current_run(session).plot_start_s, elapsed, 0.0, "Clear graph moves only the visible plot start");
    }

    RIGIDBODIES_TEST("property commands preserve identity update physical quantities and reset the measurement baseline")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session, {}, { 1.0, 2.0 });
        session.set_selection(id);
        session.advance(0.1);
        const auto samples_before_edit = current_run(session).series.time_s.size();
        command(session, ui::UiCommandKind::set_selected_mass, 2.0, false, {}, id);
        RIGIDBODIES_EXPECT(session.selection() == id && session.world().is_valid(id), "property changes preserve the selected body identity");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->mass_properties().mass_kg, 2.0, 0.0, "mass control applies kilograms to the body");
        RIGIDBODIES_EXPECT(current_run(session).series.time_s.size() == samples_before_edit, "an edit preserves the recorded graph");
        RIGIDBODIES_EXPECT(!current_run(session).markers.empty(), "an external edit is explained by an intervention marker");
        command(session, ui::UiCommandKind::set_selected_velocity_x, 4.0);
        command(session, ui::UiCommandKind::set_selected_velocity_y, -3.0);
        command(session, ui::UiCommandKind::set_selected_angular_velocity, 1.25);
        RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s() == math::Vec2(4.0, -3.0), "component controls edit the requested velocity without losing its other component");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->angular_velocity_rad_s(), 1.25, 0.0, "spin control applies radians per second");
        command(session, ui::UiCommandKind::set_selected_material, 0.0, false, "steel");
        const auto* body = session.world().find_body(id);
        RIGIDBODIES_EXPECT(body->colliders().front().material.name == "steel", "known material applies to the selected shape");
        RIGIDBODIES_EXPECT(!body->has_mass_override(), "material change derives mass from density again");
        const auto density_mass = body->mass_properties().mass_kg;
        command(session, ui::UiCommandKind::set_selected_mass, 3.0);
        command(session, ui::UiCommandKind::use_selected_density_mass);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->mass_properties().mass_kg, density_mass, 1.0e-12, "density control removes a later manual mass override");
        RIGIDBODIES_EXPECT(session.build_model().notifications.back().text.find("updated") != std::string::npos, "successful edit is acknowledged in the model");
    }

    RIGIDBODIES_TEST("stale invalid and draft-conflicting property requests are rejected without partial edits")
    {
        app::SimulationSession session;
        isolate(session);
        const auto first = disc(session, { -1.0, 0.0 }, { 1.0, 0.0 });
        const auto second = disc(session, { 1.0, 0.0 }, {});
        session.set_selection(first);
        command(session, ui::UiCommandKind::clear_energy_history);
        session.advance(0.1);
        const auto sample_count = current_run(session).series.time_s.size();
        command(session, ui::UiCommandKind::set_selected_mass, 9.0, false, {}, second);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(first)->mass_properties().mass_kg, 1.0, 0.0, "stale panel request cannot edit the current selection");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(second)->mass_properties().mass_kg, 9.0, 0.0, "an explicit body target supports Guide controls without changing selection");
        for (const auto kind : { ui::UiCommandKind::set_selected_mass, ui::UiCommandKind::set_selected_velocity_x, ui::UiCommandKind::set_selected_velocity_y, ui::UiCommandKind::set_selected_angular_velocity })
            command(session, kind, std::numeric_limits<double>::quiet_NaN());
        command(session, ui::UiCommandKind::set_selected_material, 0.0, false, "unknown material");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(first)->mass_properties().mass_kg, 1.0, 0.0, "invalid property values preserve mass");
        RIGIDBODIES_EXPECT(session.world().find_body(first)->linear_velocity_m_s() == math::Vec2(1.0, 0.0), "invalid velocity does not reach simulation state");
        RIGIDBODIES_EXPECT(current_run(session).series.time_s.size() == sample_count, "rejected edits leave the valid history intact");
        command(session, ui::UiCommandKind::start_new_shape);
        RIGIDBODIES_EXPECT(session.shape_editor().active(), "fixture has an uncommitted outline draft");
        command(session, ui::UiCommandKind::set_selected_mass, 5.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(first)->mass_properties().mass_kg, 1.0, 0.0, "property edit cannot race with the outline transaction");
        RIGIDBODIES_EXPECT(session.build_model().notifications.back().text.find("Apply or discard the outline") != std::string::npos, "the message asks the viewer to apply or discard the outline");
        command(session, ui::UiCommandKind::cancel_shape_outline);
        session.world().destroy_body(first);
        command(session, ui::UiCommandKind::set_selected_mass, 5.0, false, {}, first);
        RIGIDBODIES_EXPECT(session.build_model().notifications.back().text.find("still in the setup") != std::string::npos, "a removed body is rejected even when its old selection handle remains");
        RIGIDBODIES_EXPECT(session.world().is_valid(second), "unrelated bodies remain untouched");
    }

    RIGIDBODIES_TEST("Back to start preserves Ready edits and display preferences")
    {
        app::SimulationSession session;
        session.configure({});
        const auto id = named_body(session, "steel_ball");
        const auto original_position = session.world().find_body(id)->position_m();
        session.set_selection(id);
        command(session, ui::UiCommandKind::set_selected_mass, 2.5);
        command(session, ui::UiCommandKind::set_selected_velocity_x, 4.0);
        command(session, ui::UiCommandKind::set_selected_material, 0.0, false, "rubber");
        const auto edited_mass = session.world().find_body(id)->mass_properties().mass_kg;
        command(session, ui::UiCommandKind::set_display_units, 0.0, false, "centimetre_gram");
        command(session, ui::UiCommandKind::set_vector_scale, 17.0, false, "velocity");
        command(session, ui::UiCommandKind::set_vector_scale, 3.5, false, "force");
        command(session, ui::UiCommandKind::set_vector_auto_scale, 0.0, true);
        command(session, ui::UiCommandKind::set_vector_components, 0.0, false, "chosen");
        command(session, ui::UiCommandKind::set_component_angle_degrees, 30.0);
        session.advance(0.05);
        session.reset_scenario();
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->mass_properties().mass_kg, edited_mass, 1.0e-12, "the final mass after Ready parameter edits survives Back to start");
        RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == original_position && session.world().find_body(id)->linear_velocity_m_s() == math::Vec2 { 4.0, 0.0 }, "state edited at Ready defines the starting motion");
        RIGIDBODIES_EXPECT(session.world().find_body(id)->colliders().front().material.name == "rubber", "material edited at Ready survives Back to start");
        RIGIDBODIES_EXPECT(session.build_model().current_run == nullptr && session.world().statistics().elapsed_time_s == 0.0, "reset closes even a short run and returns to Ready");
        RIGIDBODIES_EXPECT(session.selection() == id, "Back to start keeps a valid selection");
        for (int pass = 0; pass < 2; ++pass)
        {
            if (pass == 1)
                RIGIDBODIES_EXPECT(session.load_scenario("fast_projectile"), "a different scenario is available");
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(model.display_units == core::DisplayUnits::centimetre_gram, "unit preference survives world replacement");
            RIGIDBODIES_EXPECT(model.vector_scales.automatic, "automatic scaling is a persistent viewing preference");
            RIGIDBODIES_EXPECT_NEAR(model.vector_scales.velocity, 17.0, 0.0, "manual velocity setting is retained behind automatic mode");
            RIGIDBODIES_EXPECT_NEAR(model.vector_scales.force, 3.5, 0.0, "quantity factors remain independent");
            RIGIDBODIES_EXPECT(model.vector_components == render::VectorComponents::custom_axes, "chosen basis is retained");
            RIGIDBODIES_EXPECT_NEAR(model.component_angle_rad, math::pi / 6.0, 1.0e-12, "angle remains a view preference in radians");
        }
    }

    RIGIDBODIES_TEST("collision comparison runs independently of the interactive world and history")
    {
        app::SimulationSession session;
        isolate(session);
        const auto id = disc(session, { 0.3, 1.0 }, { 1.0, 2.0 }, 0.5);
        session.set_selection(id);
        command(session, ui::UiCommandKind::clear_energy_history);
        session.advance(0.1);
        const auto position = session.world().find_body(id)->position_m();
        const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
        const auto elapsed = session.world().statistics().elapsed_time_s;
        const auto steps = session.world().statistics().step_index;
        const auto sample_count = current_run(session).series.time_s.size();
        command(session, ui::UiCommandKind::compare_collisions);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.inline_notice("measure.collision.comparison") == nullptr && model.collision_comparison.size() == 3, "all three material responses are measured");
        RIGIDBODIES_EXPECT(!model.notifications.empty(), "comparison publishes a reveal action for its result");
        RIGIDBODIES_EXPECT(model.collision_comparison[0].restitution == 1.0 && model.collision_comparison[1].restitution == 0.0 && model.collision_comparison[2].restitution == 0.5, "side-by-side reports retain elastic inelastic partial order");
        for (const auto& report : model.collision_comparison)
        {
            RIGIDBODIES_EXPECT_NEAR(report.incoming_momentum_kg_m_s, report.outgoing_momentum_kg_m_s, 1.0e-8, "each isolated collision conserves momentum");
            RIGIDBODIES_EXPECT_NEAR(report.measured_first_velocity_m_s, report.predicted_first_velocity_m_s, 1.0e-6, "predicted and measured first velocity agree");
            RIGIDBODIES_EXPECT_NEAR(report.measured_second_velocity_m_s, report.predicted_second_velocity_m_s, 1.0e-6, "predicted and measured second velocity agree");
        }
        RIGIDBODIES_EXPECT(session.selection() == id, "offline comparison does not replace selection");
        RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == position && session.world().find_body(id)->linear_velocity_m_s() == velocity, "reference experiments do not touch interactive body state");
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == elapsed && session.world().statistics().step_index == steps, "offline measurements do not advance interactive time");
        RIGIDBODIES_EXPECT(current_run(session).series.time_s.size() == sample_count, "reference runs never pollute scene history");
    }

    RIGIDBODIES_TEST("pause on impact stops at the solved substep and retains before and after evidence")
    {
        app::SimulationSession session;
        isolate(session);
        const auto first = disc(session, { -0.12, 0.0 }, { 2.0, 0.0 });
        const auto second = disc(session, { 0.12, 0.0 }, { -2.0, 0.0 });
        session.stepper().set_fixed_step(0.02);
        session.stepper().set_substep_count(8);
        command(session, ui::UiCommandKind::set_pause_on_impact, 0.0, true);
        command(session, ui::UiCommandKind::clear_energy_history);
        session.advance(0.24);
        auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.paused && model.impacts.size() == 1, "impact stops the running session and captures a report");
        const auto impact = model.impacts.front();
        RIGIDBODIES_EXPECT(session.world().statistics().step_index > 0 && session.world().statistics().step_index < 8, "remaining substeps of the fixed step are not executed after impact");
        RIGIDBODIES_EXPECT(session.stepper().completed_steps() == 0, "a partially executed fixed step and the cancelled frame tail are not reported as completed work");
        RIGIDBODIES_EXPECT_NEAR(impact.time_s, session.world().statistics().elapsed_time_s, 1.0e-12, "report timestamp is the actual stopped substep");
        RIGIDBODIES_EXPECT(impact.first == first && impact.second == second, "captured pair preserves body identity");
        RIGIDBODIES_EXPECT_NEAR(impact.first_velocity_before_m_s.x, 2.0, 1.0e-9, "incoming first velocity precedes response");
        RIGIDBODIES_EXPECT_NEAR(impact.second_velocity_before_m_s.x, -2.0, 1.0e-9, "incoming second velocity precedes response");
        RIGIDBODIES_EXPECT_NEAR(impact.first_velocity_after_m_s.x, -2.0, 1.0e-7, "elastic first velocity includes completed response");
        RIGIDBODIES_EXPECT_NEAR(impact.second_velocity_after_m_s.x, 2.0, 1.0e-7, "elastic second velocity includes completed response");
        RIGIDBODIES_EXPECT_NEAR(impact.impulse_on_second_n_s.x, 4.0, 1.0e-7, "inspector captures the net exchanged impulse");
        RIGIDBODIES_EXPECT(current_run(session).impact_count == 1, "the open run counts the impact evidence");
        session.advance(0.2);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, impact.time_s, 0.0, "subsequent paused frames do not advance past impact");
        command(session, ui::UiCommandKind::toggle_pause);
        session.advance(session.stepper().fixed_step_s());
        model = session.build_model();
        RIGIDBODIES_EXPECT(!model.paused && model.impacts.size() == 1, "resuming separation does not repeatedly pause on the same episode");
        RIGIDBODIES_EXPECT(session.stepper().completed_steps() == 1, "resuming counts only the newly completed fixed step");
        RIGIDBODIES_EXPECT_NEAR(model.impacts.front().time_s, impact.time_s, 0.0, "saved evidence retains its original timestamp after the world advances");
        RIGIDBODIES_EXPECT_NEAR(model.impacts.front().first_velocity_before_m_s.x, impact.first_velocity_before_m_s.x, 0.0, "incoming state is a saved value rather than a live body reference");
    }

    RIGIDBODIES_TEST("sensor overlaps and stationary solid contacts never trigger impact pause")
    {
        for (const auto sensor : { false, true })
        {
            app::SimulationSession session;
            isolate(session);
            const auto speed = sensor ? 1.0 : 0.0;
            const auto offset = sensor ? 0.15 : 0.095;
            disc(session, { -offset, 0.0 }, { speed, 0.0 }, 1.0, sensor);
            disc(session, { offset, 0.0 }, { -speed, 0.0 }, 1.0, sensor);
            command(session, ui::UiCommandKind::set_pause_on_impact, 0.0, true);
            session.advance(0.2);
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(!model.paused && model.impacts.empty(), "impact inspection requires exchanged collision impulse, not mere overlap");
            RIGIDBODIES_EXPECT_NEAR(model.elapsed_time_s, 0.2, 1.0e-12, "non-impact contact does not interrupt the requested frame");
        }
    }

    RIGIDBODIES_TEST("Back to start keeps pause-on-impact while opening another experiment turns it off")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.simulation.start_paused = false;
        session.configure(config);
        isolate(session);
        command(session, ui::UiCommandKind::set_pause_on_impact, 0.0, true);
        disc(session, { -0.12, 0.0 }, { 2.0, 0.0 });
        disc(session, { 0.12, 0.0 }, { -2.0, 0.0 });
        session.advance(0.1);
        RIGIDBODIES_EXPECT(!session.build_model().impacts.empty(), "fixture captures collision evidence");
        session.reset_scenario();
        auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.impacts.empty(), "reset clears evidence from the prior simulation epoch");
        RIGIDBODIES_EXPECT(model.pause_on_impact, "pause preference remains explicitly selected");
        RIGIDBODIES_EXPECT(session.load_scenario("shape_workshop"), "another scenario loads normally");
        model = session.build_model();
        RIGIDBODIES_EXPECT(model.impacts.empty() && !model.pause_on_impact, "opening clears collision records and turns pause-at-impact off");
        RIGIDBODIES_EXPECT(model.current_run == nullptr, "new scenario is Ready with no run until time advances");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
