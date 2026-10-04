#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/aerodynamic.hpp>
#include <rigidbodies/physics/joint.hpp>

#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <limits>

RIGIDBODIES_TEST("background pausing never resumes automatically and the disabled preference does nothing")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    ui::UiCommand preference;
    preference.kind = ui::UiCommandKind::set_preference;
    preference.id = "true";
    preference.detail = "prefs.playback.pause_in_background";
    preference.flag = true;
    session.apply(preference);
    ui::UiCommand play;
    play.kind = ui::UiCommandKind::toggle_pause;
    if (session.build_model().paused)
        session.apply(play);
    RIGIDBODIES_EXPECT(!session.build_model().paused, "fixture is running");
    session.set_window_backgrounded(true);
    RIGIDBODIES_EXPECT(session.build_model().paused, "losing focus pauses when enabled");
    session.set_window_backgrounded(false);
    RIGIDBODIES_EXPECT(session.build_model().paused, "regaining focus never resumes automatically");

    app::SimulationSession disabled;
    if (disabled.build_model().paused)
        disabled.apply(play);
    disabled.set_window_backgrounded(true);
    RIGIDBODIES_EXPECT(!disabled.build_model().paused, "losing focus does nothing when disabled");
}

namespace
{
    using namespace rigidbodies;

    physics::BodyId add_moving_circle(app::SimulationSession& session, double speed = 1.0)
    {
        session.world().clear();
        physics::BodyDefinition definition;
        definition.name = "moving_circle";
        definition.type = physics::BodyType::kinematic_body;
        definition.linear_velocity_m_s = { speed, 0.0 };
        physics::Collider collider;
        collider.shape = physics::make_circle(0.005);
        definition.colliders.push_back(collider);
        session.set_viewport({ 800, 600 });
        return session.world().create_body(definition);
    }

    math::Vec2 drawn_circle_center(app::SimulationSession& session)
    {
        session.scene_settings().layers = {};
        session.scene_settings().layers.set(render::VisualizationLayer::bodies, true);
        session.scene_settings().material_shading = false;
        session.scene_settings().depth_background = false;
        render::DrawList list;
        session.render(list);
        for (const auto& command : list.commands())
        {
            if (command.kind == render::DrawCommandKind::circle_fill)
            {
                return session.camera().screen_to_world(list.vertices()[command.vertex_offset]);
            }
        }
        RIGIDBODIES_FAIL("a circle should be drawn");
    }

    void apply_flag(app::SimulationSession& session, ui::UiCommandKind kind, bool flag)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.flag = flag;
        session.apply(command);
    }

    void apply_value(app::SimulationSession& session, ui::UiCommandKind kind, double value)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.value = value;
        session.apply(command);
    }

    physics::BodyId add_dynamic_circle(app::SimulationSession& session, math::Vec2 position = {})
    {
        physics::BodyDefinition definition;
        definition.position_m = position;
        physics::Collider collider;
        collider.shape = physics::make_circle(0.1);
        definition.colliders.push_back(collider);
        return session.world().create_body(definition);
    }

    physics::ForceGeneratorPtr find_force(app::SimulationSession& session, std::string_view name)
    {
        for (const auto& generator : session.world().force_generators())
        {
            if (generator->name() == name)
            {
                return generator;
            }
        }
        RIGIDBODIES_FAIL("the expected generator should exist");
    }

    RIGIDBODIES_TEST("change serial advances for state-changing commands but not simulation motion")
    {
        app::SimulationSession session;
        core::ApplicationConfig running_config;
        running_config.simulation.start_paused = false;
        session.configure(running_config);
        const auto initial = session.build_model().change_serial;
        apply_value(session, ui::UiCommandKind::set_time_scale, 0.5);
        const auto after_command = session.build_model().change_serial;
        RIGIDBODIES_EXPECT(after_command == initial + 1, "one state-changing command advances the serial exactly once");
        session.advance(0.25);
        RIGIDBODIES_EXPECT(session.build_model().change_serial == after_command, "simulation motion alone does not advance the change serial");

        ui::UiCommand none;
        session.apply(none);
        RIGIDBODIES_EXPECT(session.build_model().change_serial == after_command, "an empty command does not claim a state change");
    }

    class StatefulIntegrator final : public physics::Integrator
    {
    public:
        mutable int calls { 0 };
        std::shared_ptr<physics::Integrator> clone() const override
        {
            return std::make_shared<StatefulIntegrator>(*this);
        }
        std::string_view name() const override
        {
            return "stateful_test_method";
        }
        void integrate_velocity(physics::RigidBody& body, double dt) const override
        {
            ++calls;
            physics::SemiImplicitEulerIntegrator {}.integrate_velocity(body, dt);
        }
        void integrate_position(physics::RigidBody& body, double dt) const override
        {
            ++calls;
            physics::SemiImplicitEulerIntegrator {}.integrate_position(body, dt);
        }
    };

    RIGIDBODIES_TEST("Back to start restores the setup, preserves a still-valid selection, and keeps the camera")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.simulation.start_paused = false;
        session.configure(config);
        const auto ids = session.world().body_ids();
        const auto initial_position = session.world().find_body(ids.front())->position_m();
        auto changed_settings = session.world().settings();
        changed_settings.gravity_m_s2 = { 3.0, -2.0 };
        session.world().set_settings(changed_settings);
        find_force(session, "uniform_gravity")->set_enabled(false);
        find_force(session, "aerodynamic_drag")->set_enabled(true);
        session.set_selection(ids.front());
        session.world().destroy_body(ids.front());
        session.advance(0.1);
        session.camera().set_center({ 7.0, -3.0 });
        session.camera().set_view_height(12.0);
        const auto camera_before_reset = session.camera();
        ui::UiCommand queued_step;
        queued_step.kind = ui::UiCommandKind::single_step;
        session.apply(queued_step);
        session.reset_scenario();

        RIGIDBODIES_EXPECT(session.world().body_ids() == ids, "reset restores original handles");
        RIGIDBODIES_EXPECT(session.selection() == ids.front(), "Back to start keeps a selection whose stable body id exists in the setup");
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "time resets");
        RIGIDBODIES_EXPECT_NEAR(session.world().settings().gravity_m_s2.x, 0.0, 0.0, "environment restored");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(ids.front())->position_m().y, initial_position.y, 0.0, "deleted body restored");
        RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.world().statistics().elapsed_time_s == 0.0, "reset returns paused at t = 0");
        RIGIDBODIES_EXPECT(session.camera().center_m() == camera_before_reset.center_m() && session.camera().view_height_m() == camera_before_reset.view_height_m(), "reset leaves the camera bit-identical");
        session.advance(0.1);
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == 0.0, "reset cancels a queued single step");
        RIGIDBODIES_EXPECT(find_force(session, "uniform_gravity")->is_enabled(), "gravity restored");
        RIGIDBODIES_EXPECT(!find_force(session, "aerodynamic_drag")->is_enabled(), "drag restored");
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, true);
        RIGIDBODIES_EXPECT(find_force(session, "aerodynamic_drag")->is_enabled(), "control points to restored generator");
        session.reset_scenario();
        RIGIDBODIES_EXPECT(find_force(session, "aerodynamic_drag")->is_enabled(), "an explicit interface choice overrides the independent snapshot after reuse");
    }

    RIGIDBODIES_TEST("session interpolates across the full fixed step with multiple substeps")
    {
        app::SimulationSession session;
        const auto id = add_moving_circle(session);
        session.stepper().set_fixed_step(0.01);
        session.stepper().set_substep_count(4);
        session.advance(0.015);
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == 4, "four physics steps ran");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->position_m().x, 0.01, 1.0e-12, "full step integrated");
        RIGIDBODIES_EXPECT_NEAR(drawn_circle_center(session).x, 0.005, 1.0e-12, "render blends full step endpoints");
    }

    RIGIDBODIES_TEST("picking follows the displayed body between simulation steps")
    {
        app::SimulationSession session;
        const auto id = add_moving_circle(session, 10.0);
        session.stepper().set_fixed_step(0.02);
        session.advance(0.03);
        const auto drawn = drawn_circle_center(session);
        RIGIDBODIES_EXPECT_NEAR(drawn.x, 0.1, 1.0e-12, "body shown halfway");
        session.select_at(session.camera().world_to_screen(drawn));
        RIGIDBODIES_EXPECT(session.selection() == id, "displayed position is selectable");
        session.select_at(session.camera().world_to_screen(session.world().find_body(id)->position_m()));
        RIGIDBODIES_EXPECT(!session.selection().is_valid(), "undrawn current position is not picked");
    }

    RIGIDBODIES_TEST("pause single step and resume never rewind the displayed body")
    {
        app::SimulationSession session;
        add_moving_circle(session);
        session.stepper().set_fixed_step(0.01);
        session.stepper().set_substep_count(4);
        session.advance(0.015);
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::toggle_pause;
        session.apply(command);
        RIGIDBODIES_EXPECT_NEAR(drawn_circle_center(session).x, 0.01, 1.0e-12, "pause shows current pose");
        command.kind = ui::UiCommandKind::single_step;
        session.apply(command);
        session.advance(1.0);
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == 8, "single step includes exactly four substeps");
        RIGIDBODIES_EXPECT_NEAR(drawn_circle_center(session).x, 0.02, 1.0e-12, "single step visible immediately");
        command.kind = ui::UiCommandKind::toggle_pause;
        session.apply(command);
        session.advance(0.002);
        RIGIDBODIES_EXPECT_NEAR(drawn_circle_center(session).x, 0.02, 1.0e-12, "resume holds until next step");
    }

    RIGIDBODIES_TEST("session respects substep budget and surfaces time skipped")
    {
        app::SimulationSession session;
        add_moving_circle(session);
        session.stepper().set_maximum_substeps_per_frame(10);
        session.stepper().set_substep_count(3);
        session.stepper().set_time_scale(std::numeric_limits<double>::max());
        session.advance(1.0);
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == 9, "actual physics work is bounded");
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.substeps == 3, "model reports substep count");
        RIGIDBODIES_EXPECT(std::isfinite(model.discarded_time_s) && model.discarded_time_s > 0.0, "model reports dropped time");
    }

    RIGIDBODIES_TEST("targeted force demonstration survives substeps and scenario reset")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.startup_scenario = "targeted_force";
        config.simulation.start_paused = true;
        config.simulation.substeps = 3;
        session.configure(config);
        physics::BodyId attracted;
        physics::BodyId free;
        session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.name() == "attracted_ball")
                    attracted = id;
                if (body.name() == "free_ball")
                    free = id;
            });
        RIGIDBODIES_EXPECT(attracted.is_valid() && free.is_valid(), "comparison objects exist");
        RIGIDBODIES_EXPECT(session.world().force_generators(attracted).size() == 1, "attraction attached only to chosen ball");
        RIGIDBODIES_EXPECT(session.world().force_generators(free).empty(), "neighbour has no attraction");
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::single_step;
        session.apply(command);
        session.advance(1.0);
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == 3, "all substeps executed");
        RIGIDBODIES_EXPECT(session.world().find_body(attracted)->applied_force_n().x > 3.0, "attraction acts in every substep");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(free)->applied_force_n().x, 0.0, 0.0, "neighbour falls freely");
        RIGIDBODIES_EXPECT(session.world().find_body(attracted)->applied_force_channels().size() == 2, "gravity and attraction are separate");
        const auto old_generator = session.world().force_generators(attracted).front();
        old_generator->set_enabled(false);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.world().force_generators(attracted).front()->is_enabled(), "snapshot restores scoped force");
        RIGIDBODIES_EXPECT(session.world().force_generators(attracted).front() != old_generator, "restored force detached from old pointer");
        RIGIDBODIES_EXPECT(session.world().find_body(attracted)->applied_force_channels().empty(), "reset clears prior measurements");
        session.set_selection(attracted);
        session.set_selection(free);
    }

    RIGIDBODIES_TEST("substep configuration round trips and rejects invalid counts")
    {
        const auto path = std::filesystem::current_path() / "session_test_config.cfg";
        struct Cleanup
        {
            std::filesystem::path path;
            ~Cleanup()
            {
                std::error_code error;
                std::filesystem::remove(path, error);
            }
        } cleanup { path };
        core::ApplicationConfig config;
        config.simulation.substeps = 4;
        config.simulation.maximum_substeps_per_frame = 16;
        RIGIDBODIES_EXPECT(core::save_application_config(config, path), "config saved");
        auto loaded = core::load_application_config(path);
        RIGIDBODIES_EXPECT(loaded.issues.empty(), "saved config accepted");
        app::SimulationSession session;
        session.configure(loaded.config);
        RIGIDBODIES_EXPECT(session.stepper().substep_count() == 4, "config reaches stepper");
        RIGIDBODIES_EXPECT(session.stepper().maximum_substeps_per_frame() == 16, "budget reaches stepper");
        {
            std::ofstream file { path };
            file << "simulation.substeps = 0\nsimulation.maximum_substeps_per_frame = 4097\n";
        }
        loaded = core::load_application_config(path);
        RIGIDBODIES_EXPECT(loaded.issues.size() == 2, "invalid counts reported");
        RIGIDBODIES_EXPECT(loaded.config.simulation.substeps == 1, "default retained");
    }

    RIGIDBODIES_TEST("integration method controls cycle and survive scenario resets without retaining simulated state")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.simulation.start_paused = true;
        session.configure(config);
        const auto initial_ids = session.world().body_ids();
        const auto initial_position = session.world().find_body(initial_ids.front())->position_m();
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_integrator;
        for (const auto* expected : { "velocity_verlet", "runge_kutta_4", "semi_implicit_euler" })
        {
            command.id = expected;
            session.apply(command);
            RIGIDBODIES_EXPECT(session.world().integrator().name() == expected, "method control cycles supported algorithms");
            session.world().find_body(initial_ids.front())->set_position({ 3.0, 4.0 });
            apply_flag(session, ui::UiCommandKind::toggle_pause, false);
            session.advance(0.02);
            session.reset_scenario();
            RIGIDBODIES_EXPECT(session.world().integrator().name() == expected, "reset keeps chosen comparison method");
            RIGIDBODIES_EXPECT(session.world().body_ids() == initial_ids, "reset retains snapshot identifier allocation");
            RIGIDBODIES_EXPECT(session.world().find_body(initial_ids.front())->position_m() == initial_position, "reset discards evolved body state");
        }
        command.id = "velocity_verlet";
        session.apply(command);
        RIGIDBODIES_EXPECT(session.load_scenario("targeted_force"), "another scenario loads");
        RIGIDBODIES_EXPECT(session.world().integrator().name() == "velocity_verlet", "scenario changes keep chosen method");
        command.kind = ui::UiCommandKind::single_step;
        session.apply(command);
        session.advance(1.0);
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == 1, "selected method runs from ordinary UI commands");
    }

    RIGIDBODIES_TEST("energy comparison is readable through the model and leaves the live session untouched")
    {
        app::SimulationSession session;
        session.configure({});
        session.advance(0.025);
        const auto id = session.world().body_ids().front();
        session.set_selection(id);
        const auto position = session.world().find_body(id)->position_m();
        const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
        const auto steps = session.world().statistics().step_index;
        const auto time = session.world().statistics().elapsed_time_s;
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::compare_integrators;
        session.apply(command);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.energy_comparison.size() == 3, "all benchmark methods available to the measure drawer");
        RIGIDBODIES_EXPECT(model.inline_notice("measure.energy.comparison") == nullptr, "default comparison completes successfully");
        RIGIDBODIES_EXPECT_NEAR(model.energy_comparison_settings.duration_s, 10.0, 0.0, "comparison duration explicit");
        RIGIDBODIES_EXPECT_NEAR(model.energy_comparison_settings.time_step_s, 1.0 / 120.0, 0.0, "comparison timestep explicit");
        RIGIDBODIES_EXPECT(session.selection() == id && session.world().find_body(id)->position_m() == position && session.world().find_body(id)->linear_velocity_m_s() == velocity,
            "benchmark leaves current object and selection unchanged");
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == steps && session.world().statistics().elapsed_time_s == time, "benchmark does not advance session clock");
        command.kind = ui::UiCommandKind::compare_integrators;
        session.apply(command);
        RIGIDBODIES_EXPECT_NEAR(session.build_model().energy_comparison[2].maximum_relative_drift, model.energy_comparison[2].maximum_relative_drift, 0.0, "cached report remains available");
    }

    RIGIDBODIES_TEST("opening scopes integration to the explicit lab settings")
    {
        app::SimulationSession session;
        core::ApplicationConfig config;
        config.simulation.start_paused = false;
        session.configure(config);
        const auto custom = std::make_shared<StatefulIntegrator>();
        session.world().set_integrator(custom);
        session.load_scenario("free_fall");
        RIGIDBODIES_EXPECT(session.world().integrator().name() == "semi_implicit_euler", "untracked integrator objects do not leak across an experiment open");
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_integrator;
        command.id = "semi_implicit_euler";
        session.apply(command);
        RIGIDBODIES_EXPECT(session.world().integrator().name() == "semi_implicit_euler", "cycling custom method enters the built in sequence");
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.world().integrator().name() == "semi_implicit_euler", "explicit user choice takes precedence over checkpoint method");
    }

    RIGIDBODIES_TEST("the complete scenario catalogue loads without paging")
    {
        app::SimulationSession session;
        session.configure({});
        const auto scenarios = physics::available_scenarios();
        RIGIDBODIES_EXPECT(!scenarios.empty() && physics::find_scenario("empty_lab") != nullptr, "the complete bundled catalogue is available");
        for (const auto& scenario : scenarios)
            RIGIDBODIES_EXPECT(session.load_scenario(scenario.id), "every catalogue entry loads directly");
    }

    RIGIDBODIES_TEST("collision controls persist in the setup but another experiment restores its own collision rules")
    {
        app::SimulationSession session;
        session.configure({});
        const auto initial_ids = session.world().body_ids();
        const auto initial_position = session.world().find_body(initial_ids.front())->position_m();
        auto settings = session.world().settings();
        settings.collision.restitution_mixing = physics::MaterialMixing::minimum;
        settings.collision.continuous = false;
        settings.solver.warm_starting = false;
        session.world().set_settings(settings);
        apply_flag(session, ui::UiCommandKind::toggle_pause, false);
        session.advance(0.02);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.world().settings().collision.restitution_mixing == physics::MaterialMixing::maximum, "unselected setting restores from the checkpoint");
        RIGIDBODIES_EXPECT(session.world().settings().collision.continuous && session.world().settings().solver.warm_starting, "unselected switches restore from checkpoint");

        apply_flag(session, ui::UiCommandKind::set_continuous_collision, false);
        apply_flag(session, ui::UiCommandKind::set_warm_starting, false);
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_restitution_mixing;
        const std::pair<const char*, physics::MaterialMixing> choices[] = { { "geometric_mean", physics::MaterialMixing::geometric_mean }, { "arithmetic_mean", physics::MaterialMixing::arithmetic_mean }, { "minimum", physics::MaterialMixing::minimum }, { "maximum", physics::MaterialMixing::maximum } };
        for (const auto& [choice, expected] : choices)
        {
            command.id = choice;
            session.apply(command);
            RIGIDBODIES_EXPECT(session.world().settings().collision.restitution_mixing == expected, "mixing control cycles all four policies");
            session.world().find_body(initial_ids.front())->set_position({ 3.0, 4.0 });
            apply_flag(session, ui::UiCommandKind::toggle_pause, false);
            session.advance(0.02);
            session.reset_scenario();
            RIGIDBODIES_EXPECT(session.world().settings().collision.restitution_mixing == expected, "reset keeps the explicitly selected policy");
            RIGIDBODIES_EXPECT(!session.world().settings().collision.continuous && !session.world().settings().solver.warm_starting, "reset keeps disabled comparison switches");
            RIGIDBODIES_EXPECT(session.world().body_ids() == initial_ids && session.world().find_body(initial_ids.front())->position_m() == initial_position, "body checkpoint still restores exactly");
        }
        session.apply(command);
        RIGIDBODIES_EXPECT(session.load_scenario("restitution_drop"), "drop experiment exists");
        const auto confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation && session.scenario_id() == "free_fall", "unsaved collision settings require confirmation before replacing the experiment");
        session.apply(confirmation->confirm);
        RIGIDBODIES_EXPECT(session.scenario_id() == "restitution_drop" && !session.build_model().confirmation, "confirming discard opens the requested drop experiment");
        RIGIDBODIES_EXPECT(session.world().settings().collision.restitution_mixing == physics::MaterialMixing::maximum, "opening uses the new experiment's bounce rule");
        RIGIDBODIES_EXPECT(session.world().settings().collision.continuous && !session.world().settings().solver.warm_starting, "opening uses the experiment's catch-fast setting but keeps the lab warm-start setting");
        apply_flag(session, ui::UiCommandKind::set_continuous_collision, true);
        apply_flag(session, ui::UiCommandKind::set_warm_starting, true);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.world().settings().collision.continuous && session.world().settings().solver.warm_starting, "later enabled choices replace earlier disabled choices");
    }

    RIGIDBODIES_TEST("gravity presets change the actual field without altering the atmosphere")
    {
        app::SimulationSession session;
        session.configure({});
        auto environment = session.world().settings();
        environment.air_velocity_m_s = { 3.0, -1.0 };
        environment.air_density_kg_m3 = 2.5;
        environment.air_dynamic_viscosity_pa_s = 0.002;
        session.world().set_settings(environment);
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_gravity_preset;
        for (const auto& preset : { std::pair<const char*, double> { "earth", physics::standard_gravity_m_s2 }, { "moon", 1.62 }, { "mars", 3.73 } })
        {
            command.id = preset.first;
            session.apply(command);
            const auto& actual = session.world().settings();
            RIGIDBODIES_EXPECT_NEAR(actual.gravity_m_s2.y, -preset.second, 0.0, "the selected celestial field is downward with the stated strength");
            RIGIDBODIES_EXPECT(actual.gravity_m_s2.x == 0.0 && actual.air_velocity_m_s == environment.air_velocity_m_s &&
                    actual.air_density_kg_m3 == environment.air_density_kg_m3 && actual.air_dynamic_viscosity_pa_s == environment.air_dynamic_viscosity_pa_s,
                "a gravity preset does not silently change atmospheric properties");
        }
        command.id = "unknown";
        session.apply(command);
        RIGIDBODIES_EXPECT_NEAR(session.world().settings().gravity_m_s2.y, -3.73, 0.0, "unknown presets leave the field unchanged");
        session.world().clear();
        const auto id = add_dynamic_circle(session);
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, 0.0);
        apply_value(session, ui::UiCommandKind::set_gravity_magnitude, 4.0);
        session.world().step(0.01);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 0.04, 1.0e-12, "direction and magnitude controls affect integrated body motion");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().y, 0.0, 1.0e-12, "a horizontal field produces no vertical acceleration");
    }

    RIGIDBODIES_TEST("zero strength retains the chosen direction and invalid gravity edits are ignored")
    {
        app::SimulationSession session;
        session.configure({});
        apply_value(session, ui::UiCommandKind::set_gravity_magnitude, 0.0);
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, 180.0);
        RIGIDBODIES_EXPECT_NEAR(session.build_model().gravity_direction_degrees, 180.0, 0.0, "zero gravity retains a direction for the next strength change");
        apply_value(session, ui::UiCommandKind::set_gravity_magnitude, 2.0);
        const auto expected = session.world().settings().gravity_m_s2;
        RIGIDBODIES_EXPECT_NEAR(expected.x, -2.0, 1.0e-12, "restoring strength uses the remembered direction");
        for (const auto invalid : { -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
            apply_value(session, ui::UiCommandKind::set_gravity_magnitude, invalid);
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, std::numeric_limits<double>::infinity());
        RIGIDBODIES_EXPECT(session.world().settings().gravity_m_s2 == expected, "invalid controls neither mutate the field nor introduce nonfinite values");
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, 360000090.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().settings().gravity_m_s2.y, 2.0, 1.0e-12, "large finite angles wrap before conversion");
    }

    RIGIDBODIES_TEST("environment belongs to each experiment while Back to start keeps its edits")
    {
        app::SimulationSession session;
        session.configure({});
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, 0.0);
        apply_value(session, ui::UiCommandKind::set_gravity_magnitude, 2.0);
        apply_flag(session, ui::UiCommandKind::set_gravity_enabled, false);
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, false);
        apply_flag(session, ui::UiCommandKind::set_angular_drag_enabled, false);
        apply_flag(session, ui::UiCommandKind::set_magnus_enabled, false);
        RIGIDBODIES_EXPECT(session.load_scenario("aerodynamic_profiles"), "the comparison scenario exists");
        const auto confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation && session.scenario_id() == "free_fall", "unsaved environment edits remain until leaving is confirmed");
        session.apply(confirmation->confirm);
        RIGIDBODIES_EXPECT(session.scenario_id() == "aerodynamic_profiles" && !session.build_model().confirmation, "confirming discard opens the comparison experiment");
        RIGIDBODIES_EXPECT_NEAR(session.world().settings().gravity_m_s2.x, 0.0, 1.0e-12, "opening uses the new experiment's gravity rather than an override");
        const auto opened = session.build_model();
        RIGIDBODIES_EXPECT(find_force(session, "uniform_gravity")->is_enabled() && opened.drag_enabled,
            "opening uses the new experiment's own force switches");
        apply_value(session, ui::UiCommandKind::set_gravity_angle_degrees, 0.0);
        apply_value(session, ui::UiCommandKind::set_gravity_magnitude, 2.0);
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, false);
        apply_flag(session, ui::UiCommandKind::set_magnus_enabled, false);
        session.reset_scenario();
        RIGIDBODIES_EXPECT_NEAR(session.world().settings().gravity_m_s2.x, 2.0, 1.0e-12, "environment edits are part of the starting setup");
        RIGIDBODIES_EXPECT(!session.build_model().drag_enabled && !session.build_model().magnus_enabled,
            "Back to start preserves the setup's force switches");
    }

    RIGIDBODIES_TEST("body gravity scale controls affect only a live selected dynamic body")
    {
        app::SimulationSession session;
        session.configure({});
        session.world().clear();
        const auto chosen = add_dynamic_circle(session);
        const auto neighbor = add_dynamic_circle(session, { 2.0, 0.0 });
        session.set_selection(chosen);
        apply_value(session, ui::UiCommandKind::set_selected_gravity_scale, 0.25);
        session.world().step(0.01);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(chosen)->linear_velocity_m_s().y,
            session.world().find_body(neighbor)->linear_velocity_m_s().y * 0.25,
            1.0e-12,
            "the selected scale changes only that body's gravitational acceleration");
        for (const auto invalid : { -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
            apply_value(session, ui::UiCommandKind::set_selected_gravity_scale, invalid);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(chosen)->gravity_scale(), 0.25, 0.0, "invalid UI scales are ignored");
        session.world().destroy_body(chosen);
        const auto replacement = add_dynamic_circle(session);
        apply_value(session, ui::UiCommandKind::set_selected_gravity_scale, 2.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(replacement)->gravity_scale(), 1.0, 0.0, "a stale selected handle cannot edit a replacement slot");
        session.set_selection(neighbor);
        session.world().find_body(neighbor)->set_type(physics::BodyType::kinematic_body);
        apply_value(session, ui::UiCommandKind::set_selected_gravity_scale, 2.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(neighbor)->gravity_scale(), 1.0, 0.0, "prescribed bodies do not acquire irrelevant UI gravity edits");
    }

    RIGIDBODIES_TEST("terminal speed reports the actual air model with explicit unavailable cases")
    {
        app::SimulationSession session;
        session.configure({});
        session.world().clear();
        const auto id = add_dynamic_circle(session);
        session.set_selection(id);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Air effects are off.", "disabled air does not display a fictitious equilibrium");
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, true);
        physics::ForceContext context;
        context.gravity_m_s2 = session.world().settings().gravity_m_s2;
        const auto expected = physics::estimate_terminal_speed(*session.world().find_body(id), context);
        RIGIDBODIES_EXPECT(expected && session.build_model().terminal_speed_m_s, "enabled gravity and drag have a finite equilibrium estimate");
        RIGIDBODIES_EXPECT_NEAR(*session.build_model().terminal_speed_m_s, expected->speed_m_s, 1.0e-10, "the view uses the same aerodynamic API as the simulation");
        apply_flag(session, ui::UiCommandKind::set_gravity_enabled, false);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Gravity is off.", "disabled gravity is reported explicitly");
        apply_flag(session, ui::UiCommandKind::set_gravity_enabled, true);
        auto environment = session.world().settings();
        environment.air_density_kg_m3 = 0.0;
        session.world().set_settings(environment);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s, "no air has no positive-weight drag balance");
        environment.air_density_kg_m3 = physics::default_air_density_kg_m3;
        environment.gravity_m_s2 = {};
        environment.air_velocity_m_s = { 2.0, 0.5 };
        environment.air_dynamic_viscosity_pa_s = 0.003;
        session.world().set_settings(environment);
        const auto zero = session.build_model();
        RIGIDBODIES_EXPECT(zero.terminal_speed_m_s && *zero.terminal_speed_m_s == 0.0, "zero gravity has zero terminal speed relative to air");
        RIGIDBODIES_EXPECT(zero.air_velocity_m_s == environment.air_velocity_m_s && zero.air_dynamic_viscosity_pa_s == environment.air_dynamic_viscosity_pa_s,
            "wind and viscosity readouts reflect the live environment");
        session.world().destroy_body(id);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s, "a destroyed selection cannot retain an old estimate");
    }

    RIGIDBODIES_TEST("terminal readouts reject ambiguous force registrations and recognize local uniform gravity")
    {
        app::SimulationSession session;
        session.configure({});
        session.world().clear();
        const auto id = add_dynamic_circle(session);
        session.set_selection(id);
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, true);
        auto local_air = std::make_shared<physics::AerodynamicDrag>();
        session.world().add_force_generator(id, local_air);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Multiple air models.",
            "the report never estimates one model when two air loads actually act");
        session.world().remove_force_generator(id, local_air);
        session.world().add_force_generator(local_air);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Multiple air models.",
            "multiple global air registrations are also explicit");
        session.world().remove_force_generator(local_air);
        auto local_gravity = std::make_shared<physics::UniformGravity>();
        session.world().add_force_generator(id, local_gravity);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Multiple gravity fields.",
            "double gravity cannot silently reuse a single-field estimate");
        apply_flag(session, ui::UiCommandKind::set_gravity_enabled, false);
        RIGIDBODIES_EXPECT(session.build_model().terminal_speed_m_s.has_value(), "a sole enabled local uniform field supports the same terminal model");
        apply_flag(session, ui::UiCommandKind::set_drag_enabled, false);
        session.world().add_force_generator(id, local_air);
        RIGIDBODIES_EXPECT(!session.build_model().terminal_speed_m_s && session.build_model().terminal_speed_note == "Body-specific air model.",
            "the global air control does not misrepresent a separately attached model");
    }

    RIGIDBODIES_TEST("body gravity edits survive Back to start")
    {
        app::SimulationSession session;
        session.configure({});
        physics::BodyId dynamic;
        session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.type() == physics::BodyType::dynamic_body)
                    dynamic = id;
            });
        session.set_selection(dynamic);
        apply_value(session, ui::UiCommandKind::set_selected_gravity_scale, 0.25);
        session.set_selection({});
        session.reset_scenario();
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(dynamic)->gravity_scale(), 0.25, 0.0, "body parameter edits remain in the starting setup");
    }

    RIGIDBODIES_TEST("constraint graph choice persists through resets and scenario changes")
    {
        app::SimulationSession session;
        session.configure({});
        RIGIDBODIES_EXPECT(session.world().settings().constraint_graph_enabled, "coupled graph solving is the default");
        apply_flag(session, ui::UiCommandKind::set_constraint_graph, false);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(!session.world().settings().constraint_graph_enabled, "reset keeps the explicit graph comparison choice");
        RIGIDBODIES_EXPECT(session.load_scenario("distance_chain"), "the chain demonstration exists");
        const auto confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation && session.scenario_id() == "free_fall", "the edited setup requires confirmation before opening another experiment");
        session.apply(confirmation->confirm);
        RIGIDBODIES_EXPECT(session.scenario_id() == "distance_chain" && !session.build_model().confirmation, "confirming discard opens the chain demonstration");
        RIGIDBODIES_EXPECT(!session.world().settings().constraint_graph_enabled, "new scene honors the solver preference");
        apply_flag(session, ui::UiCommandKind::set_constraint_graph, true);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.world().settings().constraint_graph_enabled && !session.world().constraints().empty(), "reenabling affects the actual connected scene");
    }

    RIGIDBODIES_TEST("selected joint motor and limit commands edit only their live keyed connection")
    {
        app::SimulationSession session;
        session.configure({});
        session.world().clear();
        const auto first = add_dynamic_circle(session);
        const auto second = add_dynamic_circle(session, { 1.0, 0.0 });
        const auto unrelated = add_dynamic_circle(session, { 3.0, 0.0 });
        physics::RevoluteJointDefinition hinge;
        hinge.first = first;
        hinge.second = second;
        hinge.motor_speed_rad_s = 2.0;
        hinge.maximum_motor_torque_n_m = 3.0;
        hinge.lower_angle_rad = -0.5;
        hinge.upper_angle_rad = 0.5;
        const auto joint = std::make_shared<physics::JointConstraint>(hinge);
        session.world().add_constraint(joint, "hinge");
        session.set_selection(second);
        ui::UiCommand edit;
        edit.body = second;
        edit.id = "hinge";
        edit.kind = ui::UiCommandKind::set_joint_motor_enabled;
        edit.flag = true;
        session.apply(edit);
        edit.kind = ui::UiCommandKind::set_joint_limits_enabled;
        session.apply(edit);
        edit.kind = ui::UiCommandKind::reverse_joint_motor;
        session.apply(edit);
        const auto actual = std::get<physics::RevoluteJointDefinition>(joint->definition());
        RIGIDBODIES_EXPECT(actual.motor_enabled && actual.limits_enabled && actual.motor_speed_rad_s == -2.0, "commands reach the joint's physical definition");
        session.set_selection(unrelated);
        session.apply(edit);
        RIGIDBODIES_EXPECT(std::get<physics::RevoluteJointDefinition>(joint->definition()).motor_speed_rad_s == 2.0, "an explicit annotated endpoint can edit its joint without changing selection");
        edit.body = unrelated;
        session.apply(edit);
        session.set_selection(second);
        edit.body = second;
        edit.id = "missing";
        session.apply(edit);
        RIGIDBODIES_EXPECT(std::get<physics::RevoluteJointDefinition>(joint->definition()).motor_speed_rad_s == 2.0, "unrelated endpoint and missing key cannot change the motor");
        session.world().destroy_body(second);
        edit.id = "hinge";
        session.apply(edit);
        RIGIDBODIES_EXPECT(session.world().constraints().empty(), "a destroyed endpoint removes the editable connection");
    }

    RIGIDBODIES_TEST("slider motor edits survive Back to start")
    {
        app::SimulationSession session;
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("prismatic_drive"), "slider demonstration exists");
        const auto joint = std::dynamic_pointer_cast<physics::JointConstraint>(session.world().constraint_by_key("driven_slider"));
        RIGIDBODIES_EXPECT(joint != nullptr, "slider has a stable editable key");
        const auto initial = std::get<physics::PrismaticJointDefinition>(joint->definition());
        session.set_selection(joint->second_body());
        ui::UiCommand edit;
        edit.body = joint->second_body();
        edit.id = "driven_slider";
        edit.kind = ui::UiCommandKind::reverse_joint_motor;
        session.apply(edit);
        edit.kind = ui::UiCommandKind::set_joint_motor_enabled;
        edit.flag = !initial.motor_enabled;
        session.apply(edit);
        RIGIDBODIES_EXPECT(std::get<physics::PrismaticJointDefinition>(joint->definition()).motor_speed_m_s == -initial.motor_speed_m_s, "slider target changes sign");
        apply_flag(session, ui::UiCommandKind::toggle_pause, false);
        session.advance(0.02);
        session.reset_scenario();
        const auto restored = std::dynamic_pointer_cast<physics::JointConstraint>(session.world().constraint_by_key("driven_slider"));
        const auto definition = std::get<physics::PrismaticJointDefinition>(restored->definition());
        RIGIDBODIES_EXPECT(restored != joint && definition.motor_speed_m_s == -initial.motor_speed_m_s && definition.motor_enabled == !initial.motor_enabled, "Back to start restores an independent joint with the edited setup settings");
        session.set_selection(restored->second_body());
        session.set_selection({});
    }

    RIGIDBODIES_TEST("broken joints cannot be silently repaired by motor controls")
    {
        app::SimulationSession session;
        session.configure({});
        session.world().clear();
        const auto first = add_dynamic_circle(session);
        const auto second = add_dynamic_circle(session);
        session.world().find_body(first)->set_type(physics::BodyType::static_body);
        physics::RevoluteJointDefinition hinge;
        hinge.first = first;
        hinge.second = second;
        hinge.break_force_n = 0.01;
        hinge.maximum_motor_torque_n_m = 1.0;
        const auto joint = std::make_shared<physics::JointConstraint>(hinge);
        session.world().add_constraint(joint, "weak_hinge");
        session.world().find_body(second)->apply_force_at_center({ 100.0, 0.0 });
        session.world().step(0.01);
        RIGIDBODIES_EXPECT(joint->is_broken(), "the weak connection has failed under load");
        session.set_selection(second);
        ui::UiCommand edit;
        edit.kind = ui::UiCommandKind::set_joint_motor_enabled;
        edit.body = second;
        edit.id = "weak_hinge";
        edit.flag = true;
        session.apply(edit);
        RIGIDBODIES_EXPECT(joint->is_broken() && !std::get<physics::RevoluteJointDefinition>(joint->definition()).motor_enabled, "control cannot reset a fracture");
    }

    RIGIDBODIES_TEST("cached bounce comparison reports all four drops without modifying the live session")
    {
        app::SimulationSession session;
        session.configure({});
        session.load_scenario("restitution_drop");
        session.advance(0.025);
        const auto ids = session.world().body_ids();
        const auto id = *std::find_if(ids.begin(), ids.end(), [&](const auto candidate)
            {
                const auto* body = session.world().find_body(candidate);
                return body && body->type() == physics::BodyType::dynamic_body;
            });
        session.set_selection(id);
        apply_flag(session, ui::UiCommandKind::set_continuous_collision, false);
        apply_flag(session, ui::UiCommandKind::set_warm_starting, false);
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_integrator;
        command.id = "velocity_verlet";
        session.apply(command);
        command.kind = ui::UiCommandKind::set_restitution_mixing;
        command.id = "geometric_mean";
        session.apply(command);
        const auto position = session.world().find_body(id)->position_m();
        const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
        const auto before = session.build_model();
        command.kind = ui::UiCommandKind::compare_restitution;
        session.apply(command);
        const auto compared = session.build_model();
        RIGIDBODIES_EXPECT(compared.restitution_comparison.size() == 4 && compared.inline_notice("measure.restitution.comparison") == nullptr, "all four policy measurements complete");
        RIGIDBODIES_EXPECT(compared.restitution_comparison_settings.continuous, "isolated measurements use CCD regardless of the live switch");
        RIGIDBODIES_EXPECT_NEAR(compared.restitution_comparison[0].theoretical_rebound_height_m, 0.16, 1.0e-14, "geometric policy predicts e squared times drop height");
        RIGIDBODIES_EXPECT_NEAR(compared.restitution_comparison[0].measured_rebound_height_m, 0.16, 0.025, "model exposes simulated rebound height");
        RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == position && session.world().find_body(id)->linear_velocity_m_s() == velocity, "live body motion unchanged");
        RIGIDBODIES_EXPECT(session.selection() == id && compared.elapsed_time_s == before.elapsed_time_s && compared.paused == before.paused, "selection time and pacing unchanged");
        RIGIDBODIES_EXPECT(session.world().integrator().name() == "velocity_verlet" && session.world().settings().collision.restitution_mixing == physics::MaterialMixing::geometric_mean, "live algorithms unchanged");
        RIGIDBODIES_EXPECT(!session.world().settings().collision.continuous && !session.world().settings().solver.warm_starting, "live collision switches unchanged");

        command.kind = ui::UiCommandKind::compare_integrators;
        session.apply(command);
        RIGIDBODIES_EXPECT(session.build_model().energy_comparison.size() == 3, "energy comparison remains available beside the bounce report");
        command.kind = ui::UiCommandKind::compare_restitution;
        session.apply(command);
        RIGIDBODIES_EXPECT_NEAR(session.build_model().restitution_comparison[3].measured_rebound_height_m, compared.restitution_comparison[3].measured_rebound_height_m, 0.0, "cached measurements remain available");
    }
}

RIGIDBODIES_TEST("visual controls preserve physics state and undo history")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    const auto time = session.world().statistics().elapsed_time_s;
    const auto count = session.world().statistics().body_count;
    ui::UiCommand command;
    command.kind = ui::UiCommandKind::set_visual_effect;
    command.id = "impact_sparks";
    command.flag = false;
    session.apply(command);
    RIGIDBODIES_EXPECT(!session.build_model().visual_settings.impact_sparks, "individual effect setting reaches the UI model");
    command.kind = ui::UiCommandKind::set_visual_budget;
    command.id = "spark_budget";
    command.value = 1e9;
    session.apply(command);
    RIGIDBODIES_EXPECT(session.scene_settings().spark_budget == 256, "effect budget has a validated ceiling");
    command.value = std::numeric_limits<double>::quiet_NaN();
    session.apply(command);
    RIGIDBODIES_EXPECT(session.scene_settings().spark_budget == 256, "invalid budget cannot poison storage limits");
    RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == time && session.world().statistics().body_count == count && !session.build_model().can_undo, "presentation controls leave world and edit history unchanged");
}

RIGIDBODIES_TEST("successful scenario edits reset trails while rejected property edits preserve them")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    core::ApplicationConfig config;
    config.simulation.start_paused = false;
    session.configure(config);
    session.set_viewport({ 800, 600 });
    auto& settings = session.scene_settings();
    settings.layers = render::LayerMask::none();
    settings.layers.set(render::VisualizationLayer::trajectories, true);
    settings.depth_background = false;
    settings.motion_trails = false;
    settings.directional_blur = false;
    session.advance(.08);
    physics::BodyId selection;
    session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
        {
            if (body.type() == physics::BodyType::dynamic_body)
                selection = id;
        });
    RIGIDBODIES_EXPECT(selection.is_valid(), "the configured demonstration contains a dynamic body");
    session.set_selection(selection);
    render::DrawList before, after;
    session.render(before);
    RIGIDBODIES_EXPECT(!before.is_empty(), "running demonstration records trajectories");
    ui::UiCommand edit;
    edit.kind = ui::UiCommandKind::set_selected_mass;
    edit.body = selection;
    edit.value = -1;
    session.apply(edit);
    session.render(after);
    RIGIDBODIES_EXPECT(after.vertices() == before.vertices(), "a rejected edit leaves the existing display history visible");
    edit.value = 2;
    session.apply(edit);
    session.render(after);
    RIGIDBODIES_EXPECT(after.vertices() != before.vertices(), "successful intervention clears pre-edit trails while retaining the always-visible selection outline");
}

RIGIDBODIES_TEST("every experiment opens Ready at t zero and clears queued stepping")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    for (const auto& scenario : physics::available_scenarios())
    {
        ui::UiCommand step;
        step.kind = ui::UiCommandKind::single_step;
        session.apply(step);
        RIGIDBODIES_EXPECT(session.load_scenario(scenario.id), "catalogue experiment loads");
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "experiment opens paused");
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "experiment opens at t zero");
        session.advance(1.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, 0.0, 0.0, "load cancels queued step");
    }
}

RIGIDBODIES_TEST("runtime profiling and workers survive scene edits without entering undo history")
{
    using namespace rigidbodies;
    app::SimulationSession session;
    session.configure({});
    ui::UiCommand command;
    command.kind = ui::UiCommandKind::set_physics_profiling;
    command.flag = true;
    session.apply(command);
    command.kind = ui::UiCommandKind::set_physics_workers;
    command.value = 2;
    session.apply(command);
    RIGIDBODIES_EXPECT(session.world().profiling_enabled() && session.world().parallel_settings().worker_count == 2, "runtime controls apply");
    RIGIDBODIES_EXPECT(!session.build_model().can_undo, "diagnostics do not create an edit");
    for (const auto invalid : { -1.0, 0.5, 33.0, std::numeric_limits<double>::infinity() })
    {
        command.value = invalid;
        session.apply(command);
        RIGIDBODIES_EXPECT(session.world().parallel_settings().worker_count == 2, "invalid worker counts are ignored");
    }
    command.kind = ui::UiCommandKind::set_solver_parameter;
    command.id = "velocity_iterations";
    command.value = 12;
    session.apply(command);
    command.kind = ui::UiCommandKind::set_physics_workers;
    command.value = 1;
    session.apply(command);
    command.kind = ui::UiCommandKind::undo;
    session.apply(command);
    RIGIDBODIES_EXPECT(session.world().profiling_enabled() && session.world().parallel_settings().worker_count == 1, "undo preserves current runtime policy");
    command.kind = ui::UiCommandKind::reset_scenario;
    session.apply(command);
    RIGIDBODIES_EXPECT(session.world().profiling_enabled() && session.world().parallel_settings().worker_count == 1, "reset preserves runtime policy");
    command.kind = ui::UiCommandKind::load_scenario;
    command.id = "stable_stack";
    session.apply(command);
    RIGIDBODIES_EXPECT(session.world().profiling_enabled() && session.world().parallel_settings().worker_count == 1, "document loading preserves runtime policy");
    session.world().step(1.0 / 120.0);
    RIGIDBODIES_EXPECT(session.world().profile().total_s > 0, "runtime timing remains usable");
}
int main()
{
    return rigidbodies::testing::run_all();
}
