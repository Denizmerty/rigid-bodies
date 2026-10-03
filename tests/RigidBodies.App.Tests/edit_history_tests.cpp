#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include "test_framework.hpp"
#include <limits>

namespace
{
    using namespace rigidbodies;
    using K = ui::UiCommandKind;
    void command(app::SimulationSession& session, K kind, double value = 0.0, std::string id = {}, bool flag = false)
    {
        ui::UiCommand c;
        c.kind = kind;
        c.value = value;
        c.id = std::move(id);
        c.flag = flag;
        session.apply(c);
    }
    physics::BodyId disc(app::SimulationSession& s, physics::BodyType type = physics::BodyType::dynamic_body)
    {
        physics::BodyDefinition d;
        d.type = type;
        physics::Collider c;
        c.shape = physics::make_circle(0.2);
        d.colliders.push_back(c);
        return s.world().create_body(d);
    }
    RIGIDBODIES_TEST("batch mass edit undo and redo preserve every body handle")
    {
        app::SimulationSession s;
        const auto a = disc(s), b = disc(s);
        const auto mass = s.world().find_body(a)->mass_properties().mass_kg;
        s.set_selections({ a, b });
        command(s, K::set_selected_mass, 7.0);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(a)->mass_properties().mass_kg, 7, 1e-12, "first edited");
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(b)->mass_properties().mass_kg, 7, 1e-12, "second edited");
        const auto paused_before_undo = s.stepper().is_paused();
        command(s, K::undo);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(a)->mass_properties().mass_kg, mass, 1e-12, "first restored");
        RIGIDBODIES_EXPECT(s.selections().size() == 2 && s.stepper().is_paused() == paused_before_undo, "parameter undo retains selection and play state");
        command(s, K::redo);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(b)->mass_properties().mass_kg, 7, 1e-12, "second redone");
    }
    RIGIDBODIES_TEST("mass previews hold time and commit as one undo entry")
    {
        app::SimulationSession s;
        RIGIDBODIES_EXPECT(s.load_scenario("empty_lab"), "recording fixture opens");
        const auto body = disc(s);
        const auto original = s.world().find_body(body)->mass_properties().mass_kg;
        s.set_selection(body);
        s.stepper().set_paused(false);
        command(s, K::clear_energy_history);
        for (int frame = 0; frame < 4; ++frame)
            s.advance(0.03);
        const auto samples_before_preview = s.build_model().current_run->series.time_s.size();
        RIGIDBODIES_EXPECT(samples_before_preview > 1, "the transaction starts with recorded run samples");
        const auto elapsed = s.world().statistics().elapsed_time_s;
        ui::UiCommand edit;
        edit.kind = K::set_selected_mass;
        edit.phase = ui::UiEditPhase::preview;
        for (int index = 0; index < 30; ++index)
        {
            edit.value = 2.0 + index * 0.1;
            s.apply(edit);
            s.advance(1.0 / 60.0);
        }
        RIGIDBODIES_EXPECT(s.build_model().held_reason == "adjusting", "parameter preview exposes the adjusting hold");
        RIGIDBODIES_EXPECT_NEAR(s.world().statistics().elapsed_time_s, elapsed, 1e-12, "simulation time does not advance during previews");
        RIGIDBODIES_EXPECT(s.build_model().current_run->series.time_s.size() == samples_before_preview, "previews do not alter the run graph");
        edit.phase = ui::UiEditPhase::commit;
        edit.value = 5.0;
        s.apply(edit);
        RIGIDBODIES_EXPECT(s.build_model().held_reason.empty() && s.build_model().can_undo, "commit releases the hold and creates history");
        RIGIDBODIES_EXPECT(s.build_model().current_run->series.time_s.size() == samples_before_preview, "committing an edit preserves the graph samples");
        RIGIDBODIES_EXPECT(!s.build_model().current_run->markers.empty(), "committing an edit records an intervention marker");
        command(s, K::undo);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(body)->mass_properties().mass_kg, original, 1e-12, "one undo restores the state before the first preview");
        RIGIDBODIES_EXPECT(s.build_model().undo_label != "Set mass of object to 5.00 kg", "thirty previews and one commit create exactly one property entry");
    }
    RIGIDBODIES_TEST("cancel restores the state before a preview transaction")
    {
        app::SimulationSession s;
        const auto body = disc(s);
        const auto original = s.world().find_body(body)->mass_properties().mass_kg;
        s.set_selection(body);
        ui::UiCommand edit;
        edit.kind = K::set_selected_mass;
        edit.value = 7.0;
        edit.phase = ui::UiEditPhase::preview;
        s.apply(edit);
        edit.phase = ui::UiEditPhase::cancel;
        s.apply(edit);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(body)->mass_properties().mass_kg, original, 1e-12, "cancel restores the exact before value");
        RIGIDBODIES_EXPECT(!s.build_model().can_undo && s.build_model().held_reason.empty(), "cancel creates no undo entry and releases the hold");
    }
    RIGIDBODIES_TEST("field commits coalesce for one second of wall time")
    {
        app::SimulationSession merged;
        const auto a = disc(merged);
        const auto original = merged.world().find_body(a)->mass_properties().mass_kg;
        merged.set_selection(a);
        merged.set_wall_time(10.0);
        command(merged, K::set_selected_mass, 2.0);
        merged.set_wall_time(10.9);
        command(merged, K::set_selected_mass, 3.0);
        command(merged, K::undo);
        RIGIDBODIES_EXPECT_NEAR(merged.world().find_body(a)->mass_properties().mass_kg, original, 1e-12, "commits 0.9 seconds apart merge");
        RIGIDBODIES_EXPECT(!merged.build_model().can_undo, "merged commits occupy one history entry");

        app::SimulationSession separate;
        const auto b = disc(separate);
        separate.set_selection(b);
        separate.set_wall_time(20.0);
        command(separate, K::set_selected_mass, 2.0);
        separate.set_wall_time(21.1);
        command(separate, K::set_selected_mass, 3.0);
        command(separate, K::undo);
        RIGIDBODIES_EXPECT_NEAR(separate.world().find_body(b)->mass_properties().mass_kg, 2.0, 1e-12, "commits 1.1 seconds apart stay separate");
        RIGIDBODIES_EXPECT(separate.build_model().can_undo, "older entry remains after undoing the later commit");
    }
    RIGIDBODIES_TEST("unsupported batch member rejects entire property transaction")
    {
        app::SimulationSession s;
        const auto a = disc(s), b = disc(s, physics::BodyType::static_body);
        const auto mass = s.world().find_body(a)->mass_properties().mass_kg;
        s.set_selections({ a, b });
        command(s, K::set_selected_mass, 4.0);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(a)->mass_properties().mass_kg, mass, 1e-12, "no partial edit");
        RIGIDBODIES_EXPECT(!s.build_model().can_undo && !s.build_model().notifications.empty(), "rejection does not consume history");
    }
    RIGIDBODIES_TEST("batch velocity edit preserves each orthogonal component")
    {
        app::SimulationSession s;
        const auto a = disc(s), b = disc(s);
        s.world().find_body(a)->set_linear_velocity({ 1, 2 });
        s.world().find_body(b)->set_linear_velocity({ 3, 4 });
        s.set_selections({ a, b });
        command(s, K::set_selected_velocity_x, 6);
        RIGIDBODIES_EXPECT(s.world().find_body(a)->linear_velocity_m_s() == math::Vec2 { 6, 2 }, "A orthogonal velocity");
        RIGIDBODIES_EXPECT(s.world().find_body(b)->linear_velocity_m_s() == math::Vec2 { 6, 4 }, "B orthogonal velocity");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.world().find_body(b)->linear_velocity_m_s() == math::Vec2 { 3, 4 }, "undo exact vector");
    }
    RIGIDBODIES_TEST("delete selection keeps fixed objects and restores the connected scene with original handles")
    {
        app::SimulationSession s;
        s.configure({});
        s.load_scenario("shape_workshop");
        const auto ids = s.world().body_ids();
        std::vector<physics::BodyId> fixed;
        for (const auto id : ids)
            if (s.world().find_body(id)->type() == physics::BodyType::static_body)
                fixed.push_back(id);
        s.set_selections(ids);
        command(s, K::delete_selected_body);
        RIGIDBODIES_EXPECT(s.world().body_ids() == fixed && s.selections() == fixed, "mixed deletion keeps fixed objects selected");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.world().body_ids() == ids && s.selections() == ids, "identities and selection restored");
        command(s, K::redo);
        RIGIDBODIES_EXPECT(s.world().body_ids() == fixed && s.selections() == fixed, "eligible deletion redone without removing fixed objects");
    }
    RIGIDBODIES_TEST("undo scene change restores reset baseline and preferences")
    {
        app::SimulationSession s;
        s.configure({});
        const auto first = s.scenario_id();
        command(s, K::set_gravity_magnitude, 2.0);
        const auto first_count = s.world().body_ids().size();
        s.load_scenario("collision_comparison");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.scenario_id() == first && s.world().body_ids().size() == first_count, "original scene restored");
        s.reset_scenario();
        RIGIDBODIES_EXPECT(s.scenario_id() == first, "reset still belongs to original scene");
        RIGIDBODIES_EXPECT_NEAR(math::length(s.world().settings().gravity_m_s2), 2, 1e-12, "selected environment preserved");
    }
    RIGIDBODIES_TEST("invalid command and display preferences preserve redo branch")
    {
        app::SimulationSession s;
        const auto a = disc(s);
        s.set_selection(a);
        command(s, K::set_selected_mass, 3);
        command(s, K::undo);
        command(s, K::set_selected_mass, -1);
        ui::UiCommand theme;
        theme.kind = K::set_theme;
        theme.id = "workbench_light";
        s.apply(theme);
        command(s, K::set_ui_scale, 1.5);
        RIGIDBODIES_EXPECT(s.build_model().can_redo, "failed edit and appearance leave redo available");
        command(s, K::redo);
        RIGIDBODIES_EXPECT(s.build_model().theme_id == "workbench_light" && s.build_model().ui_scale == 1.5, "redo preserves appearance");
        command(s, K::undo);
        command(s, K::set_selected_mass, 8);
        RIGIDBODIES_EXPECT(!s.build_model().can_redo, "new valid edit cuts obsolete branch");
    }
    RIGIDBODIES_TEST("history capacity is bounded and recent edits remain undoable")
    {
        app::SimulationSession s;
        const auto a = disc(s);
        s.set_selection(a);
        for (int i = 0; i < 80; ++i)
            command(s, K::set_selected_mass, i + 1.0);
        std::size_t count = 0;
        while (s.build_model().can_undo)
        {
            command(s, K::undo);
            ++count;
        }
        RIGIDBODIES_EXPECT(count == app::maximum_session_edits, "bounded history");
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(a)->mass_properties().mass_kg, 16, 1e-12, "oldest retained before state");
    }
    RIGIDBODIES_TEST("undo restores elapsed simulation evidence rather than editing a later body")
    {
        app::SimulationSession s;
        const auto a = disc(s);
        s.set_selection(a);
        s.world().find_body(a)->set_linear_velocity({ 1, 0 });
        s.advance(.05);
        const auto before = s.world().statistics().elapsed_time_s;
        const auto position = s.world().find_body(a)->position_m();
        command(s, K::set_selected_mass, 3);
        s.advance(.05);
        command(s, K::undo);
        RIGIDBODIES_EXPECT_NEAR(s.world().statistics().elapsed_time_s, before + .05, 1e-12, "parameter undo preserves elapsed simulation time");
        RIGIDBODIES_EXPECT(!(s.world().find_body(a)->position_m() == position), "parameter undo preserves the later pose");
    }
    RIGIDBODIES_TEST("solver environment and force generator changes are undoable")
    {
        app::SimulationSession s;
        s.configure({});
        const auto iterations = s.world().settings().solver.velocity_iterations;
        command(s, K::set_solver_parameter, 17, "velocity_iterations");
        RIGIDBODIES_EXPECT(s.world().settings().solver.velocity_iterations == 17, "solver tuning");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.world().settings().solver.velocity_iterations == iterations, "solver undo");
        const auto density = s.world().settings().air_density_kg_m3;
        command(s, K::set_environment_parameter, 5, "air_density_kg_m3");
        command(s, K::undo);
        RIGIDBODIES_EXPECT_NEAR(s.world().settings().air_density_kg_m3, density, 1e-12, "environment undo");
        const bool enabled = s.world().force_generators().front()->is_enabled();
        command(s, K::set_force_generator_enabled, 0, "0", !enabled);
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.world().force_generators().front()->is_enabled() == enabled, "force clone restored");
    }
    RIGIDBODIES_TEST("no-op or malformed settings cannot destroy a redo branch")
    {
        app::SimulationSession s;
        const auto a = disc(s);
        s.set_selection(a);
        command(s, K::set_selected_mass, 3);
        command(s, K::undo);
        const auto iterations = s.world().settings().solver.velocity_iterations;
        command(s, K::set_solver_parameter, iterations + 0.5, "velocity_iterations");
        command(s, K::set_solver_parameter, iterations, "velocity_iterations");
        command(s, K::set_environment_parameter, s.world().settings().air_density_kg_m3, "air_density_kg_m3");
        command(s, K::set_fixed_step, std::numeric_limits<double>::quiet_NaN());
        command(s, K::set_time_scale, s.stepper().time_scale());
        RIGIDBODIES_EXPECT(s.build_model().can_redo && !s.build_model().can_undo, "no hidden modifications");
        RIGIDBODIES_EXPECT(s.world().settings().solver.velocity_iterations == iterations, "fractional iteration count rejected");
    }
    RIGIDBODIES_TEST("restoring an edit cancels queued single steps without inventing completed work")
    {
        app::SimulationSession s;
        const auto a = disc(s);
        s.set_selection(a);
        s.advance(0.03);
        s.stepper().set_paused(true);
        const auto count = s.stepper().completed_steps();
        const auto elapsed = s.world().statistics().elapsed_time_s;
        command(s, K::single_step);
        command(s, K::set_selected_mass, 3);
        command(s, K::undo);
        s.advance(0.0);
        RIGIDBODIES_EXPECT(s.stepper().completed_steps() == count, "undo cannot count an unexecuted request");
        RIGIDBODIES_EXPECT_NEAR(s.world().statistics().elapsed_time_s, elapsed, 1e-12, "undo discards pending input");
        command(s, K::single_step);
        command(s, K::start_new_shape);
        RIGIDBODIES_EXPECT(s.stepper().completed_steps() == count, "draft activation also clears input without a phantom step");
    }
    RIGIDBODIES_TEST("outline creation pointer edits and commit have separate undo steps")
    {
        app::SimulationSession s;
        s.set_viewport({ 1000, 800 });
        command(s, K::start_new_shape);
        const auto before = s.world().body_ids().size();
        for (const auto p : { math::Vec2 { -0.5, -0.5 }, math::Vec2 { 0.5, -0.5 }, math::Vec2 { 0, 0.5 } })
        {
            ui::UiEvent e;
            e.kind = ui::UiEventKind::pointer_down;
            e.pointer_px = s.camera().world_to_screen(p);
            s.handle_scene_event(e);
            e.kind = ui::UiEventKind::pointer_up;
            s.handle_scene_event(e);
        }
        RIGIDBODIES_EXPECT(s.shape_editor().outline().nodes.size() == 3, "three drawn nodes");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.shape_editor().outline().nodes.size() == 2, "undo one node");
        command(s, K::redo);
        command(s, K::close_shape_outline);
        command(s, K::commit_shape_outline);
        RIGIDBODIES_EXPECT(s.world().body_ids().size() == before + 1 && !s.shape_editor().active(), "shape committed");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.world().body_ids().size() == before && s.shape_editor().active(), "commit restores draft");
        command(s, K::redo);
        RIGIDBODIES_EXPECT(s.world().body_ids().size() == before + 1, "commit redone");
    }

    RIGIDBODIES_TEST("undoing Back to start reopens the run it closed")
    {
        app::SimulationSession s;
        RIGIDBODIES_EXPECT(s.load_scenario("free_fall"), "run-history fixture opens");
        s.stepper().set_paused(false);
        for (int frame = 0; frame < 40; ++frame)
            s.advance(0.025);
        RIGIDBODIES_EXPECT(s.build_model().current_run != nullptr, "playback owns an open run");
        s.reset_scenario();
        RIGIDBODIES_EXPECT(s.build_model().current_run == nullptr && s.build_model().runs.size() == 1, "Back to start closes and keeps the run");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.build_model().current_run != nullptr && s.build_model().runs.empty(), "structural history restores the open run instead of duplicating it");
        command(s, K::redo);
        RIGIDBODIES_EXPECT(s.build_model().current_run == nullptr && s.build_model().runs.size() == 1, "redo closes the same run again");
    }

    RIGIDBODIES_TEST("run stars pins and clearing create no undo entry or redo branch")
    {
        app::SimulationSession s;
        RIGIDBODIES_EXPECT(s.load_scenario("free_fall"), "non-edit run fixture opens");
        const auto body = s.world().body_ids().back();
        s.set_selection(body);
        command(s, K::set_selected_mass, 2.0);
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.build_model().can_redo, "fixture starts with a redo branch");
        const auto undo_label = s.build_model().undo_label;
        command(s, K::star_run, 1.0, {}, true);
        ui::UiCommand pin;
        pin.kind = K::pin_run_value;
        pin.id = "speed";
        pin.body = body;
        pin.detail = "maximum";
        s.apply(pin);
        command(s, K::unpin_run_value, 0.0, "speed:1");
        command(s, K::clear_runs);
        RIGIDBODIES_EXPECT(s.build_model().can_redo && s.build_model().undo_label == undo_label, "run-list operations leave edit history untouched");
        command(s, K::redo);
        RIGIDBODIES_EXPECT_NEAR(s.world().find_body(body)->mass_properties().mass_kg, 2.0, 1.0e-12, "redo still applies after run-list operations");
    }

    RIGIDBODIES_TEST("structural undo restores experiment pinned values")
    {
        app::SimulationSession s;
        RIGIDBODIES_EXPECT(s.load_scenario("shape_workshop"), "pinned structural fixture opens");
        const auto body = s.world().body_ids().back();
        s.set_selection(body);
        ui::UiCommand pin;
        pin.kind = K::pin_run_value;
        pin.id = "speed";
        pin.body = body;
        pin.detail = "at_end";
        s.apply(pin);
        RIGIDBODIES_EXPECT(s.build_model().pinned_values.size() == 1, "fixture pins one value");
        command(s, K::delete_selected_body);
        command(s, K::unpin_run_value, 0.0, "speed:1");
        RIGIDBODIES_EXPECT(s.build_model().pinned_values.empty(), "non-edit removal changes the current runs view");
        command(s, K::undo);
        RIGIDBODIES_EXPECT(s.build_model().pinned_values.size() == 1 && s.world().is_valid(body), "undo restores the shared run and pin snapshot with the body");
    }
}
int main()
{
    return rigidbodies::testing::run_all();
}
