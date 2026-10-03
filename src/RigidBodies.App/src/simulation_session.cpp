#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/setup_changes.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/aerodynamic.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace rigidbodies::app
{
    namespace
    {

        // One wheel notch changes the visible height by this factor. A value near one keeps the
        // zoom controllable; a larger one skips past what the viewer was looking at.

        physics::IntegratorPtr integrator_from_id(std::string_view id)
        {
            if (id == "velocity_verlet")
            {
                return std::make_shared<physics::VelocityVerletIntegrator>();
            }
            if (id == "runge_kutta_4")
            {
                return std::make_shared<physics::RungeKutta4Integrator>();
            }
            return std::make_shared<physics::SemiImplicitEulerIntegrator>();
        }

        std::string integrator_label(std::string_view id)
        {
            if (id == "semi_implicit_euler")
                return "Semi-implicit Euler";
            if (id == "velocity_verlet")
                return "Velocity Verlet";
            if (id == "runge_kutta_4")
                return "Runge–Kutta 4";
            return std::string(id);
        }

        render::VisualizationLayer layer_from_id(std::string_view id, bool& found)
        {
            if (const auto* description = render::find_layer(id); description != nullptr)
            {
                found = true;
                return description->layer;
            }
            found = false;
            return render::VisualizationLayer::grid;
        }

        // A body's hover card replaces its stage label once the pointer has rested this long.
        constexpr double hover_card_delay_s = 0.6;

        // The switches and limits behind each Effects quality choice. Standard is the first-run
        // configuration; Low keeps the effects that explain the physics (shading, trails, flashes,
        // soft impact cues) and drops the decorative ones; High raises every limit.
        struct EffectsPreset
        {
            std::string_view id;
            bool material_shading, contact_shadows, depth_background, motion_trails, directional_blur, impact_flashes, impact_sparks, impact_dust, soft_deformation, transitions;
            std::size_t shading_body_budget, contact_shadow_budget, motion_body_budget, directional_blur_budget, impact_flash_budget, spark_budget, dust_budget, deformation_budget;
        };
        constexpr EffectsPreset effects_presets[] {
            { "low", true, false, false, true, false, true, false, false, true, true, 64, 32, 16, 16, 16, 32, 16, 16 },
            { "standard", true, true, true, true, true, true, true, true, true, true, 128, 96, 64, 64, 32, 128, 64, 32 },
            { "high", true, true, true, true, true, true, true, true, true, true, 512, 192, 128, 128, 48, 256, 128, 64 },
        };

        const EffectsPreset* find_effects_preset(std::string_view id)
        {
            for (const auto& preset : effects_presets)
                if (preset.id == id)
                    return &preset;
            return nullptr;
        }

        void apply_effects_preset(render::SceneRenderSettings& settings, const EffectsPreset& preset)
        {
            settings.material_shading = preset.material_shading;
            settings.contact_shadows = preset.contact_shadows;
            settings.depth_background = preset.depth_background;
            settings.motion_trails = preset.motion_trails;
            settings.directional_blur = preset.directional_blur;
            settings.impact_flashes = preset.impact_flashes;
            settings.impact_sparks = preset.impact_sparks;
            settings.impact_dust = preset.impact_dust;
            settings.soft_deformation = preset.soft_deformation;
            settings.transitions = preset.transitions;
            settings.shading_body_budget = preset.shading_body_budget;
            settings.contact_shadow_budget = preset.contact_shadow_budget;
            settings.motion_body_budget = preset.motion_body_budget;
            settings.directional_blur_budget = preset.directional_blur_budget;
            settings.impact_flash_budget = preset.impact_flash_budget;
            settings.spark_budget = preset.spark_budget;
            settings.dust_budget = preset.dust_budget;
            settings.deformation_budget = preset.deformation_budget;
        }

        // Reduce Motion holds the decorative motion switches off, so they say nothing about which
        // preset the viewer chose and are left out of the comparison while it is on.
        bool matches_effects_preset(const render::SceneRenderSettings& settings, const EffectsPreset& preset, bool motion_reduced)
        {
            const auto motion_matches = motion_reduced || (settings.directional_blur == preset.directional_blur && settings.impact_flashes == preset.impact_flashes && settings.impact_sparks == preset.impact_sparks && settings.impact_dust == preset.impact_dust && settings.transitions == preset.transitions);
            return motion_matches && settings.material_shading == preset.material_shading && settings.contact_shadows == preset.contact_shadows && settings.depth_background == preset.depth_background && settings.motion_trails == preset.motion_trails && settings.soft_deformation == preset.soft_deformation && settings.shading_body_budget == preset.shading_body_budget && settings.contact_shadow_budget == preset.contact_shadow_budget && settings.motion_body_budget == preset.motion_body_budget && settings.directional_blur_budget == preset.directional_blur_budget && settings.impact_flash_budget == preset.impact_flash_budget && settings.spark_budget == preset.spark_budget && settings.dust_budget == preset.dust_budget && settings.deformation_budget == preset.deformation_budget;
        }

        bool point_in_triangle(const math::Vec2& point, const std::array<math::Vec2, 3>& triangle)
        {
            const auto first = math::cross(triangle[1] - triangle[0], point - triangle[0]);
            const auto second = math::cross(triangle[2] - triangle[1], point - triangle[1]);
            const auto third = math::cross(triangle[0] - triangle[2], point - triangle[2]);
            return (first >= 0.0 && second >= 0.0 && third >= 0.0) || (first <= 0.0 && second <= 0.0 && third <= 0.0);
        }

    } // namespace

    void SimulationSession::configure(const core::ApplicationConfig& config)
    {
        start_paused_ = config.simulation.start_paused;
        reduce_motion_ = config.interface_settings.reduce_motion;
        pause_in_background_ = config.controls.pause_in_background;
        keep_lab_settings_ = config.experiments.keep_lab_settings;
        recommended_view_ = config.experiments.recommended_view;
        ask_predictions_ = config.experiments.ask_predictions;
        experiments_on_start_ = config.experiments.on_start;
        capture_area_ = config.capture.area;
        camera_zoom_sensitivity_ = config.controls.wheel_zoom_factor;
        stepper_.set_fixed_step(config.simulation.steps_per_second > 0.0 ? 1.0 / config.simulation.steps_per_second : 1.0 / 120.0);
        stepper_.set_maximum_substeps_per_frame(config.simulation.maximum_substeps_per_frame);
        stepper_.set_substep_count(config.simulation.substeps);
        stepper_.set_time_scale(config.simulation.time_scale);
        stepper_.set_paused(config.simulation.start_paused);

        default_view_height_m_ = config.simulation.default_view_height_m;
        camera_.set_view_height(default_view_height_m_);

        scene_settings_.theme = render::theme_by_name(config.interface_settings.theme.c_str());
        scene_settings_.display_units = config.interface_settings.units == "centimetre_gram" ? core::DisplayUnits::centimetre_gram : core::DisplayUnits::si;
        theme_id_ = config.interface_settings.theme == "workbench_light" || config.interface_settings.theme == "workbench_projector" ? config.interface_settings.theme : "workbench_dark";
        ui_scale_ = std::clamp(config.interface_settings.interface_scale, 0.75, 2.0);

        auto layers = render::LayerMask::defaults();
        layers.set(render::VisualizationLayer::velocity_vectors, config.visualization.show_velocity_vectors);
        layers.set(render::VisualizationLayer::force_vectors, config.visualization.show_force_vectors);
        layers.set(render::VisualizationLayer::contact_points, config.visualization.show_contact_points);
        layers.set(render::VisualizationLayer::center_of_mass, config.visualization.show_center_of_mass);
        layers.set(render::VisualizationLayer::trajectories, config.visualization.show_trajectories);
        layers.set(render::VisualizationLayer::bounding_boxes, config.visualization.show_bounding_boxes);
        layers.set(render::VisualizationLayer::grid, config.visualization.show_grid);
        scene_settings_.layers = layers;
        base_layers_ = layers;
        scene_settings_.vectors_selected_only = config.visualization.arrow_scope == "selected";
        scene_settings_.vector_scales.automatic = config.visualization.arrow_length_automatic;
        scene_settings_.vector_scales.velocity = config.visualization.velocity_scale;
        scene_settings_.vector_scales.acceleration = config.visualization.acceleration_scale;
        scene_settings_.vector_scales.force = config.visualization.force_scale;
        scene_settings_.vector_scales.momentum = config.visualization.momentum_scale;
        scene_settings_.vector_components = config.visualization.split_arrows == "world" ? render::VectorComponents::world_axes : config.visualization.split_arrows == "chosen" ? render::VectorComponents::custom_axes
            : config.visualization.split_arrows == "contact"                                                                                                                    ? render::VectorComponents::contact_axes
                                                                                                                                                                                : render::VectorComponents::none;
        scene_settings_.component_angle_rad = math::degrees_to_radians(config.visualization.direction_degrees);
        scene_settings_.material_shading = config.effects.material_shading;
        scene_settings_.contact_shadows = config.effects.contact_shadows;
        scene_settings_.depth_background = config.effects.depth_background;
        scene_settings_.motion_trails = config.effects.motion_trails;
        scene_settings_.directional_blur = config.effects.directional_blur;
        scene_settings_.impact_flashes = config.effects.impact_flashes;
        scene_settings_.impact_sparks = config.effects.impact_sparks;
        scene_settings_.impact_dust = config.effects.impact_dust;
        scene_settings_.soft_deformation = config.effects.soft_deformation;
        scene_settings_.transitions = config.effects.transitions;
        scene_settings_.shading_body_budget = config.effects.shading_body_budget;
        scene_settings_.contact_shadow_budget = config.effects.contact_shadow_budget;
        scene_settings_.motion_body_budget = config.effects.motion_body_budget;
        scene_settings_.directional_blur_budget = config.effects.directional_blur_budget;
        scene_settings_.impact_flash_budget = config.effects.impact_flash_budget;
        scene_settings_.spark_budget = config.effects.spark_budget;
        scene_settings_.dust_budget = config.effects.dust_budget;
        scene_settings_.deformation_budget = config.effects.deformation_budget;
        // The accessibility clamp runs after the effect assignments above so a configured Reduce
        // Motion wins over the stored effect toggles from the very first frame.
        if (reduce_motion_)
            apply_reduce_motion();
        // The stored quality name is only a label; the switches and limits decide what it reads.
        refresh_effects_quality();

        // The world starts with gravity only. Drag is created here so that the interface can turn
        // it on without the session having to know how to build one later.
        refresh_force_generators();
        if (!gravity_)
        {
            gravity_ = std::make_shared<physics::UniformGravity>();
            world_.add_force_generator(gravity_);
        }

        if (!drag_)
        {
            drag_ = std::make_shared<physics::AerodynamicDrag>();
            drag_->set_enabled(false);
            world_.add_force_generator(drag_);
        }

        if (!load_scenario(config.startup_scenario))
        {
            core::log_warning("the configured startup scenario \"{}\" is not known; loading the default instead", config.startup_scenario);
            load_scenario(physics::default_scenario_id());
        }
        undo_edits_.clear();
        redo_edits_.clear();
    }

    physics::World& SimulationSession::world()
    {
        return world_;
    }

    const physics::World& SimulationSession::world() const
    {
        return world_;
    }

    render::Camera2D& SimulationSession::camera()
    {
        return camera_;
    }

    const render::Camera2D& SimulationSession::camera() const
    {
        return camera_;
    }

    render::SceneRenderSettings& SimulationSession::scene_settings()
    {
        return scene_settings_;
    }

    const render::SceneRenderSettings& SimulationSession::scene_settings() const
    {
        return scene_settings_;
    }

    physics::TimeStepper& SimulationSession::stepper()
    {
        return stepper_;
    }

    const std::string& SimulationSession::scenario_id() const
    {
        return scenario_id_;
    }

    LabSettings SimulationSession::capture_lab_settings() const
    {
        const auto& settings = world_.settings();
        return {
            std::string(world_.integrator().name()),
            stepper_.fixed_step_s(),
            settings.solver.velocity_iterations,
            settings.solver.position_iterations,
            settings.solver.linear_slop_m,
            settings.solver.position_correction_fraction,
            settings.solver.restitution_threshold_m_s,
            settings.solver.maximum_position_correction_m,
            settings.solver.warm_starting,
            settings.constraint_graph_enabled
        };
    }

    void SimulationSession::apply_lab_settings(const LabSettings& settings)
    {
        world_.set_integrator(integrator_from_id(settings.integrator_id));
        stepper_.set_fixed_step(settings.fixed_step_s);
        auto world_settings = world_.settings();
        world_settings.solver.velocity_iterations = settings.velocity_iterations;
        world_settings.solver.position_iterations = settings.position_iterations;
        world_settings.solver.linear_slop_m = settings.linear_slop_m;
        world_settings.solver.position_correction_fraction = settings.position_correction_fraction;
        world_settings.solver.restitution_threshold_m_s = settings.restitution_threshold_m_s;
        world_settings.solver.maximum_position_correction_m = settings.maximum_position_correction_m;
        world_settings.solver.warm_starting = settings.warm_starting;
        world_settings.constraint_graph_enabled = settings.constraint_graph;
        world_.set_settings(world_settings);
        lab_ = settings;
    }

    void SimulationSession::rebuild_snapshot_views()
    {
        if (setup_.world.is_valid())
            setup_view_.restore(setup_.world);
        if (original_.world.is_valid())
            original_view_.restore(original_.world);
    }

    void SimulationSession::merge_live_structure_into_setup()
    {
        physics::World baseline;
        baseline.restore(setup_.world);
        physics::World merged;
        merged.restore(world_.snapshot());
        merged.restart_timeline();
        merged.set_settings(baseline.settings());
        merged.set_potential_energy_reference_height(baseline.potential_energy_reference_height_m());
        for (const auto id : merged.body_ids())
        {
            auto* body = merged.find_body(id);
            if (const auto* original = baseline.find_body(id))
                *body = *original;
            else if (body)
            {
                body->set_linear_velocity({});
                body->set_angular_velocity(0.0);
                body->clear_accumulators();
            }
        }
        setup_.world = merged.snapshot();
        rebuild_snapshot_views();
    }

    void SimulationSession::sync_setup_after_command(const ui::UiCommand& command)
    {
        if (updating_setup_ || !setup_.world.is_valid())
            return;
        using K = ui::UiCommandKind;
        const auto parameter = command.kind == K::set_selected_mass || command.kind == K::use_selected_density_mass || command.kind == K::set_selected_material || command.kind == K::set_selected_gravity_scale || command.kind == K::set_joint_motor_enabled || command.kind == K::set_joint_limits_enabled || command.kind == K::reverse_joint_motor || command.kind == K::set_joint_motor_speed || command.kind == K::set_spring_parameter || command.kind == K::set_gravity_enabled || command.kind == K::set_gravity_magnitude || command.kind == K::set_gravity_angle_degrees || command.kind == K::set_gravity_preset || command.kind == K::set_drag_enabled || command.kind == K::set_angular_drag_enabled || command.kind == K::set_magnus_enabled || command.kind == K::set_environment_parameter || command.kind == K::set_restitution_mixing || command.kind == K::set_continuous_collision || command.kind == K::set_energy_reference_height;
        const auto state = command.kind == K::set_selected_velocity_x || command.kind == K::set_selected_velocity_y || command.kind == K::set_selected_velocity || command.kind == K::set_selected_angular_velocity || command.kind == K::set_selected_position || command.kind == K::set_selected_orientation || command.kind == K::stop_selected_motion;
        const auto lab = command.kind == K::set_integrator || command.kind == K::set_fixed_step || command.kind == K::set_warm_starting || command.kind == K::set_constraint_graph || command.kind == K::set_solver_parameter;
        const auto structure_add_remove = command.kind == K::delete_selected_body || command.kind == K::commit_shape_outline || command.kind == K::add_object;
        const auto structure_pose_dependent = command.kind == K::assemble_selected_bodies || command.kind == K::split_selected_body;
        if (!parameter && !state && !lab && !structure_add_remove && !structure_pose_dependent)
            return;
        if (lab)
            lab_ = capture_lab_settings();
        const auto ready = world_.statistics().elapsed_time_s <= 0.0;
        if (ready || lab)
        {
            setup_.world = world_.snapshot();
            setup_.gravity_direction_degrees = gravity_direction_degrees_;
            rebuild_snapshot_views();
            return;
        }
        const auto keep_toast = [&](std::string_view message)
        {
            if (state_setup_toast_shown_)
                return;
            ui::NotificationAction keep;
            keep.label = "Keep as starting state";
            keep.command = ui::UiCommand { K::keep_state_as_setup };
            notify(ui::Severity::info, std::string(message), "setup-state", keep);
            state_setup_toast_shown_ = true;
        };
        if (structure_pose_dependent)
        {
            keep_toast("This change to the setup applies only to the current run.");
            return;
        }
        if (structure_add_remove)
        {
            if (command.kind == K::delete_selected_body)
            {
                const auto live = world_.snapshot();
                const auto live_direction = gravity_direction_degrees_;
                updating_setup_ = true;
                world_.restore(setup_.world);
                gravity_direction_degrees_ = setup_.gravity_direction_degrees;
                refresh_force_generators();
                auto setup_command = command;
                setup_command.body = command_target_body_;
                apply_untracked(setup_command);
                setup_.world = world_.snapshot();
                rebuild_snapshot_views();
                world_.restore(live);
                gravity_direction_degrees_ = live_direction;
                refresh_force_generators();
                updating_setup_ = false;
            }
            else
                merge_live_structure_into_setup();
            return;
        }
        if (state)
        {
            keep_toast("This motion change applies to the current run.");
            return;
        }
        const auto target = command_target_body_;
        if (target.is_valid() && world_.is_valid(target) && !setup_view_.is_valid(target))
        {
            const auto* body = world_.find_body(target);
            keep_toast(core::substitute("{} was created during this run, so the change applies only to this run.", body ? body->name() : std::string { "This object" }));
            return;
        }
        const auto live = world_.snapshot();
        const auto live_direction = gravity_direction_degrees_;
        updating_setup_ = true;
        world_.restore(setup_.world);
        gravity_direction_degrees_ = setup_.gravity_direction_degrees;
        refresh_force_generators();
        apply_untracked(command);
        setup_.world = world_.snapshot();
        setup_.gravity_direction_degrees = gravity_direction_degrees_;
        rebuild_snapshot_views();
        world_.restore(live);
        gravity_direction_degrees_ = live_direction;
        refresh_force_generators();
        updating_setup_ = false;
    }

    void SimulationSession::set_window_backgrounded(bool backgrounded)
    {
        if (!pause_in_background_)
            return;
        if (backgrounded)
        {
            paused_by_background_ = !stepper_.is_paused();
            if (paused_by_background_)
            {
                stepper_.set_paused(true);
                pause_reason_ = { ui::PauseReason::in_background, 0 };
            }
        }
        else if (paused_by_background_)
        {
            paused_by_background_ = false;
        }
    }

    bool SimulationSession::load_scenario_untracked(std::string_view id)
    {
        if (!physics::find_scenario(id))
            return false;
        const auto opening_first = scenario_id_.empty();
        close_current_run(false);
        const auto carried_lab = capture_lab_settings();
        const auto carried_speed = stepper_.time_scale();
        finish_shape_editor();
        notifier_.clear_source("shape");
        if (!physics::load_scenario(world_, id))
        {
            return false;
        }

        scenario_id_ = std::string { id };
        current_setup_path_.clear();
        if (const auto* document = physics::scenario_document_for_id(id))
        {
            scenario_document_ = std::make_shared<physics::ScenarioDocument>(*document);
            experiment_content_ = parse_experiment_content(*scenario_document_);
        }
        else
        {
            scenario_document_.reset();
            experiment_content_.reset();
        }
        run_recorder_.set_experiment(id);
        pending_prediction_.reset();
        next_impact_armed_ = false;
        pause_reason_ = {};
        notifier_.clear_source("content");
        const auto scenarios = physics::available_scenarios();
        for (std::size_t index = 0; index < scenarios.size(); ++index)
        {
            if (scenarios[index].id == id)
            {
                break;
            }
        }
        scene_settings_.selection = {};
        selected_bodies_.clear();
        interaction_.mode = InteractionMode::select;
        pause_on_impact_ = false;
        scene_renderer_.clear_trajectories();
        stepper_.reset();
        stepper_.set_time_scale(carried_speed);
        stepper_.set_paused(start_paused_);
        stepper_.cancel_single_step_request();
        single_step_pending_ = false;
        refresh_force_generators();
        if (!drag_)
        {
            drag_ = std::make_shared<physics::AerodynamicDrag>();
            drag_->set_enabled(false);
            world_.add_force_generator(drag_);
        }
        const auto loaded_lab = capture_lab_settings();
        default_lab_ = loaded_lab;
        original_.world = world_.snapshot();
        const auto gravity = world_.settings().gravity_m_s2;
        original_.gravity_direction_degrees = math::length_squared(gravity) > 0.0 ? math::radians_to_degrees(std::atan2(gravity.y, gravity.x)) : -90.0;
        apply_lab_settings(!opening_first && keep_lab_settings_ ? carried_lab : loaded_lab);
        setup_.world = world_.snapshot();
        setup_.gravity_direction_degrees = original_.gravity_direction_degrees;
        rebuild_snapshot_views();
        gravity_direction_degrees_ = setup_.gravity_direction_degrees;
        state_setup_toast_shown_ = false;
        reset_measurements();

        // Each experiment opens on its own recommended view, so the Show presets name what is on
        // screen rather than a mixture carried over from the previous experiment.
        if (recommended_view_)
            scene_settings_.layers = recommended_layers();

        // Scenarios may install or configure physical effects. Explicit interface preferences
        // take precedence, while an untouched session uses the demonstration's own environment.
        frame_subject();
        core::log_info("scenario \"{}\" loaded with {} objects", scenario_id_, world_.body_ids().size());
        mark_edit_changed();
        return true;
    }

    void SimulationSession::reset_scenario_untracked()
    {
        const auto previous_selection = selected_bodies_;
        finish_shape_editor();
        notifier_.clear_source("shape");
        if (setup_.world.is_valid())
        {
            const auto runtime_parallel = world_.parallel_settings();
            const auto runtime_profile = world_.profiling_enabled();
            world_.restore(setup_.world);
            world_.set_parallel_settings(runtime_parallel);
            world_.set_profiling_enabled(runtime_profile);
            apply_lab_settings(lab_);
            refresh_force_generators();
            synchronize_render_history();
            selected_bodies_.clear();
            for (const auto id : previous_selection)
                if (world_.is_valid(id))
                    selected_bodies_.push_back(id);
            scene_settings_.selection = selected_bodies_.empty() ? physics::BodyId {} : selected_bodies_.back();
            scene_renderer_.clear_trajectories();
            stepper_.reset();
            stepper_.set_paused(true);
            stepper_.cancel_single_step_request();
            single_step_pending_ = false;
            reset_measurements();
            state_setup_toast_shown_ = false;
            mark_edit_changed();
            pause_reason_ = {};
        }
    }

    void SimulationSession::close_current_run(bool make_previous)
    {
        run_recorder_.close_run(true, make_previous);
    }

    void SimulationSession::begin_run_if_needed()
    {
        if (run_recorder_.current() || !setup_.world.is_valid() || !original_.world.is_valid())
            return;
        auto original_changes = compute_setup_changes(setup_view_, original_view_);
        std::vector<ui::SetupChange> previous_changes;
        if (const auto* previous = run_recorder_.previous())
        {
            for (const auto& current : original_changes)
            {
                const auto old = std::find_if(previous->changes_from_original.begin(), previous->changes_from_original.end(), [&](const auto& value)
                    {
                        return value.key == current.key;
                    });
                if (old == previous->changes_from_original.end() || old->current_text != current.current_text)
                    previous_changes.push_back(current);
            }
            for (const auto& old : previous->changes_from_original)
                if (std::none_of(original_changes.begin(), original_changes.end(), [&](const auto& value)
                        {
                            return value.key == old.key;
                        }))
                    previous_changes.push_back(old);
        }
        run_recorder_.begin_run(world_, std::move(original_changes), std::move(previous_changes), pending_prediction_);
        pending_prediction_.reset();
        run_recorder_.record_sample(world_, world_.statistics().elapsed_time_s);
    }

    void SimulationSession::refresh_force_generators()
    {
        gravity_.reset();
        drag_.reset();
        for (const auto& generator : world_.force_generators())
        {
            if (generator && generator->name() == "uniform_gravity")
            {
                gravity_ = generator;
            }
            else if (generator && generator->name() == "aerodynamic_drag")
            {
                drag_ = generator;
            }
        }
    }

    void SimulationSession::synchronize_render_history()
    {
        world_.for_each_body([](physics::BodyId, physics::RigidBody& body)
            {
                body.capture_previous_transform();
            });
    }

    void SimulationSession::advance(double frame_time_s)
    {
        if (!held_reason_.empty() || (interaction_.active && interaction_.mode != InteractionMode::pull))
            return;
        const auto steps = stepper_.advance(frame_time_s);
        if (steps > 0)
            begin_run_if_needed();
        for (int index = 0; index < steps; ++index)
        {
            for (int substep = 0; substep < stepper_.substep_count(); ++substep)
            {
                apply_interaction_forces();
                world_.step(stepper_.substep_s(), substep == 0);
                scene_renderer_.record_visual_sample(world_, stepper_.substep_s(), scene_settings_);
                run_recorder_.record_sample(world_, world_.statistics().elapsed_time_s);
                if (collect_impacts())
                {
                    const auto unrun = (steps - index - 1) * stepper_.substep_count() + stepper_.substep_count() - substep - 1;
                    stepper_.cancel_scheduled_substeps(unrun);
                    stepper_.set_paused(true);
                    synchronize_render_history();
                    single_step_pending_ = false;
                    return;
                }
            }

            // Trails are sampled per simulation step rather than per frame, so a trail records the
            // motion rather than how fast the machine happened to be drawing.
            if (scene_settings_.layers.is_enabled(render::VisualizationLayer::trajectories))
            {
                scene_renderer_.record_trajectory_sample(world_, scene_settings_.trajectory_length);
            }
        }
        if (single_step_pending_ && steps > 0)
        {
            synchronize_render_history();
        }
        if (follow_selection_ && !stepper_.is_paused())
            if (const auto* body = world_.find_body(selection()))
            {
                const auto bounds = body->compute_bounds();
                const auto a = camera_.world_to_screen(bounds.minimum), b = camera_.world_to_screen(bounds.maximum);
                const auto minimum = math::min_components(a, b), maximum = math::max_components(a, b);
                const auto focus = camera_.focus_rect();
                const auto inner_minimum = math::Vec2 { focus.left + focus.width * 0.15, focus.top + focus.height * 0.15 };
                const auto inner_maximum = math::Vec2 { focus.left + focus.width * 0.85, focus.top + focus.height * 0.85 };
                math::Vec2 shift;
                if (minimum.x < inner_minimum.x)
                    shift.x = inner_minimum.x - minimum.x;
                else if (maximum.x > inner_maximum.x)
                    shift.x = inner_maximum.x - maximum.x;
                if (minimum.y < inner_minimum.y)
                    shift.y = inner_minimum.y - minimum.y;
                else if (maximum.y > inner_maximum.y)
                    shift.y = inner_maximum.y - maximum.y;
                if (math::length_squared(shift) > 0.0)
                    camera_.pan_by_screen_delta(shift);
            }
        single_step_pending_ = false;
    }

    double SimulationSession::stage_text_scale() const
    {
        // The renderer caps stage text at twice the preference's default, so the enlargement
        // stops there and chips keep matching the labels.
        return presenting_ ? std::min(ui_scale_ * overlay::present_scale, std::max(ui_scale_, 2.0)) : ui_scale_;
    }

    double SimulationSession::velocity_handle_scale() const
    {
        // The knob sits on the arrow the stage drew. A drag keeps the factor it began with, so an
        // automatic arrow length cannot rescale the arrow under the pointer.
        if (handle_drag_.active && handle_drag_.velocity_scale > 0.0)
            return handle_drag_.velocity_scale;
        const auto drawn = scene_renderer_.drawn_velocity_scale();
        if (std::isfinite(drawn) && drawn > 0.0)
            return drawn;
        return scene_settings_.vector_scales.velocity * std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
    }

    double SimulationSession::overlay_scale() const
    {
        return stage_text_scale() * std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
    }

    double SimulationSession::stage_view_height_m() const
    {
        // The readout counts the whole stage left in view, from the toolbar down to the status
        // line or a sheet, not the inset area that framing fills.
        const auto& stage = visible_stage_rect_.empty() ? camera_.focus_rect() : visible_stage_rect_;
        const auto viewport_height = static_cast<double>(camera_.viewport().height);
        if (stage.empty() || !(viewport_height > 0.0))
            return camera_.view_height_m();
        return camera_.view_height_m() * stage.height / viewport_height;
    }

    physics::BodyId SimulationSession::hover_card_body() const
    {
        if (!hover_cards_allowed_ || !interaction_.hover_connection_key.empty() || !world_.is_valid(interaction_.hover_body))
            return {};
        const auto elapsed = (has_wall_time_ ? wall_time_s_ : 0.0) - interaction_.hover_started_s;
        return elapsed >= hover_card_delay_s ? interaction_.hover_body : physics::BodyId {};
    }

    std::optional<render::ScreenRect> SimulationSession::hover_target_bounds() const
    {
        if (!interaction_.hover_connection_key.empty())
        {
            // A connection's card stands beside the drawn line, padded to clear its end pins and
            // a joint's symbol.
            const auto anchors = connection_anchors_m(interaction_.hover_connection_key, interaction_.hover_connection_kind);
            if (!anchors)
                return std::nullopt;
            const auto first = camera_.world_to_screen(anchors->first), second = camera_.world_to_screen(anchors->second);
            const auto pad = 8.0 * std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
            const auto minimum = math::min_components(first, second) - math::Vec2 { pad, pad };
            const auto maximum = math::max_components(first, second) + math::Vec2 { pad, pad };
            return render::ScreenRect { minimum.x, minimum.y, maximum.x - minimum.x, maximum.y - minimum.y };
        }
        const auto* body = world_.find_body(interaction_.hover_body);
        if (body == nullptr)
            return std::nullopt;
        const auto bounds = body->compute_bounds(body->interpolated_transform(scene_settings_.interpolation_alpha));
        const auto first = camera_.world_to_screen(bounds.minimum), second = camera_.world_to_screen(bounds.maximum);
        return render::ScreenRect { std::min(first.x, second.x), std::min(first.y, second.y), std::abs(second.x - first.x), std::abs(second.y - first.y) };
    }

    const physics::RigidBody* SimulationSession::handle_body() const
    {
        if (shape_editor_.active() || selected_bodies_.size() > 1)
            return nullptr;
        const auto* body = world_.find_body(selection());
        return body && body->type() == physics::BodyType::dynamic_body ? body : nullptr;
    }

    double SimulationSession::handle_ring_radius(const physics::RigidBody& body) const
    {
        // The ring widens past a neighbour's centre-of-mass symbol rather than run through it,
        // so the grip, which travels the whole ring as the body turns, never covers one.
        const auto scale = overlay_scale();
        const auto base = rotation_ring_radius(camera_, body, scale);
        const auto centre = camera_.world_to_screen(body.world_center_of_mass_m());
        const auto clearance = (overlay::knob_radius + overlay::rotation_grip_radius + 3.0) * scale;
        auto radius = base;
        for (int pass = 0; pass < 8; ++pass)
        {
            bool widened = false;
            world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& other)
                {
                    if (&other == &body || other.type() == physics::BodyType::static_body || is_marker(id))
                        return;
                    const auto distance = math::length(camera_.world_to_screen(other.world_center_of_mass_m()) - centre);
                    if (std::abs(distance - radius) < clearance && distance + clearance <= base * 2.5)
                    {
                        radius = distance + clearance;
                        widened = true;
                    }
                });
            if (!widened)
                break;
        }
        return radius;
    }

    void SimulationSession::reserve_overlay_areas()
    {
        overlay_areas_.clear();
        scene_renderer_.set_overlay_rings({});
        if (stepper_.is_paused())
        {
            const auto scale = overlay_scale();
            const auto square = [&](const math::Vec2& centre, double half)
            {
                overlay_areas_.emplace_back(centre - math::Vec2 { half, half }, centre + math::Vec2 { half, half });
            };
            // Plates keep clear of the grips and prefer places off the rotation ring.
            if (const auto* body = interaction_.active ? nullptr : handle_body())
            {
                const auto centre_m = body->world_center_of_mass_m();
                const auto centre = camera_.world_to_screen(centre_m);
                const auto ring = handle_ring_radius(*body);
                thread_local std::vector<std::pair<math::Vec2, double>> rings;
                rings.assign(1, { centre, ring });
                scene_renderer_.set_overlay_rings(rings);
                square(rotation_knob_position(centre, ring, body->orientation_rad()), (overlay::rotation_grip_radius + 4.0) * scale);
                square(velocity_knob_position(camera_, centre_m, body->linear_velocity_m_s(), velocity_handle_scale(), scale), (overlay::knob_radius + 4.0) * scale);
                // A knob beyond the stage's capped velocity arrow hangs on a tether past the
                // arrow's head, which plates keep clear of too. The tether is covered in pieces
                // whose boxes stay thin across it, as far as the viewport.
                const auto tip = velocity_handle_tip(camera_, centre_m, body->linear_velocity_m_s(), velocity_handle_scale());
                const auto reach = math::length(tip - centre);
                const auto capped = static_cast<double>(scene_settings_.vector_scales.maximum_drawn_length_px) * std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
                if (scene_settings_.layers.is_enabled(render::VisualizationLayer::velocity_vectors) && std::isfinite(capped) && capped > 0.0 && std::isfinite(reach) && reach > capped)
                {
                    const auto direction = (tip - centre) / reach;
                    const auto half = 3.0 * scale;
                    const auto piece = std::min(reach, 2.0 * half / std::max(std::min(std::abs(direction.x), std::abs(direction.y)), 1.0e-6));
                    const math::Vec2 viewport { static_cast<double>(camera_.viewport().width), static_cast<double>(camera_.viewport().height) };
                    int pieces = 0;
                    for (auto along = capped; along < reach && pieces < 96; along += piece, ++pieces)
                    {
                        const auto from = centre + direction * along, to = centre + direction * std::min(along + piece, reach);
                        if (std::min(from.x, to.x) > viewport.x || std::max(from.x, to.x) < 0.0 || std::min(from.y, to.y) > viewport.y || std::max(from.y, to.y) < 0.0)
                            break;
                        overlay_areas_.emplace_back(math::min_components(from, to) - math::Vec2 { half, half }, math::max_components(from, to) + math::Vec2 { half, half });
                    }
                }
            }
            square(gravity_compass_centre(camera_, scale), (overlay::compass_radius + 4.0) * scale);
        }
        // A plate the hover card would cover moves aside or is left out whole rather than
        // showing a fragment past the card's edge. The card's own outline is reported: plates keep
        // their usual spacing from it and may come up to its edge before giving way. The card was
        // placed beside its target last frame, so it is carried along with the target to where it
        // is drawn now.
        const auto target = hover_target_bounds();
        if (hover_card_area_ && target)
        {
            const auto shift = hover_card_target_ ? math::Vec2 { target->left - hover_card_target_->left, target->top - hover_card_target_->top } : math::Vec2 {};
            const auto& card = *hover_card_area_;
            const math::Vec2 minimum { card.left + shift.x, card.top + shift.y };
            overlay_areas_.emplace_back(minimum, minimum + math::Vec2 { card.width, card.height });
        }
        hover_card_target_ = target;
        scene_renderer_.set_overlay_areas(overlay_areas_);
    }

    void SimulationSession::apply_reduce_motion()
    {
        // Transitions, flashes, blur, sparks, and dust are decorative motion; trails and soft
        // deformation stay on because they communicate the physics rather than embellish it.
        scene_settings_.transitions = false;
        scene_settings_.impact_flashes = false;
        scene_settings_.directional_blur = false;
        scene_settings_.impact_sparks = false;
        scene_settings_.impact_dust = false;
    }

    void SimulationSession::refresh_effects_quality()
    {
        effects_quality_ = "custom";
        for (const auto& preset : effects_presets)
            if (matches_effects_preset(scene_settings_, preset, reduce_motion_))
            {
                effects_quality_ = std::string(preset.id);
                return;
            }
    }

    bool SimulationSession::starting_motion_edit(const ui::UiCommand& command) const
    {
        using K = ui::UiCommandKind;
        if (command.detail != "setup" || !(command.kind == K::set_selected_velocity_x || command.kind == K::set_selected_velocity_y || command.kind == K::set_selected_angular_velocity))
            return false;
        const auto target = command.body.is_valid() ? command.body : selection();
        return !updating_setup_ && setup_.world.is_valid() && world_.statistics().elapsed_time_s > 0.0 && setup_view_.is_valid(target);
    }

    void SimulationSession::apply_starting_motion_edit(const ui::UiCommand& command)
    {
        const auto live = world_.snapshot();
        const auto live_direction = gravity_direction_degrees_;
        auto setup_command = command;
        setup_command.detail.clear();
        setup_command.body = command.body.is_valid() ? command.body : selection();
        // The live run is untouched, so its trails stay and the property toast stays quiet; the
        // notice below says where the value went instead.
        const auto previewing = applying_preview_;
        applying_preview_ = true;
        updating_setup_ = true;
        world_.restore(setup_.world);
        gravity_direction_degrees_ = setup_.gravity_direction_degrees;
        refresh_force_generators();
        apply_untracked(setup_command);
        setup_.world = world_.snapshot();
        setup_.gravity_direction_degrees = gravity_direction_degrees_;
        rebuild_snapshot_views();
        world_.restore(live);
        gravity_direction_degrees_ = live_direction;
        refresh_force_generators();
        updating_setup_ = false;
        applying_preview_ = previewing;
        if (previewing)
            return;
        ui::NotificationAction back;
        back.label = "Back to start";
        back.command = ui::UiCommand { ui::UiCommandKind::reset_scenario };
        notify(ui::Severity::info, "The new starting value applies from the next run.", "setup-start", back);
    }

    void SimulationSession::render(render::DrawList& list)
    {
        scene_settings_.interpolation_alpha = stepper_.is_paused() || stepper_.time_scale() == 0.0
            ? 1.0
            : stepper_.interpolation_fraction();
        scene_settings_.text_scale = static_cast<float>(stage_text_scale());
        scene_settings_.label_replaced = hover_card_body();
        if (scene_settings_.layers.is_enabled(render::VisualizationLayer::labels))
            scene_renderer_.set_body_names(object_names());
        reserve_overlay_areas();
        scene_renderer_.set_selected_bodies(selected_bodies_);
        scene_renderer_.render(world_, camera_, scene_settings_, list);
        const auto scene_layer = list.layer();
        shape_editor_.set_view_scale(overlay_scale());
        if (shape_editor_.active())
        {
            list.set_layer(overlay::veil_layer);
            list.add_rectangle_fill({ 0.0, 0.0 }, { static_cast<double>(camera_.viewport().width), static_cast<double>(camera_.viewport().height) }, scene_settings_.theme.background.with_alpha(0.60f));
        }
        list.set_layer(overlay::overlay_layer);
        shape_editor_.draw(camera_, scene_settings_.theme, list, scene_settings_.display_units);
        render_interaction(list);
        if (present_spotlight_)
        {
            // A neutral dark shade reads as a spotlight on every theme; shading with a light
            // background colour would wash the surroundings out and make the lit circle look dim.
            list.set_layer(overlay::spotlight_layer);
            const auto scale = overlay_scale();
            const auto dark = overlay_palette(scene_settings_.theme).dark;
            add_spotlight(list, camera_.viewport(), camera_.world_to_screen(pointer_world_m_), 80.0 * scale, 48.0 * scale, render::Color { 0.0f, 0.0f, 0.0f, dark ? 0.55f : 0.32f });
        }
        list.set_layer(scene_layer);
    }

    void SimulationSession::set_viewport(const render::ViewportSize& viewport)
    {
        camera_.set_viewport(viewport);
    }

    void SimulationSession::set_focus_rect(const render::ScreenRect& rect)
    {
        const auto previous = camera_.focus_rect();
        if (previous.left == rect.left && previous.top == rect.top && previous.width == rect.width && previous.height == rect.height)
            return;
        const auto preserved = camera_.screen_to_world(previous.empty() ? math::Vec2 { camera_.viewport().width * 0.5, camera_.viewport().height * 0.5 } : previous.center());
        // The selection's screen box, if the reader could see any of it before the change.
        std::optional<math::Aabb> selection_m;
        for (const auto id : selected_bodies_)
            if (const auto* body = world_.find_body(id))
            {
                if (!selection_m)
                    selection_m.emplace();
                selection_m->expand(body->compute_bounds(body->interpolated_transform(scene_settings_.interpolation_alpha)));
            }
        const auto screen_box = [&](const math::Aabb& bounds)
        {
            math::Aabb box;
            box.expand(camera_.world_to_screen(bounds.minimum));
            box.expand(camera_.world_to_screen(bounds.maximum));
            return box;
        };
        if (selection_m && !previous.empty())
        {
            const auto box = screen_box(*selection_m);
            if (selection_m->is_empty() || box.maximum.x < previous.left || box.minimum.x > previous.left + previous.width || box.maximum.y < previous.top || box.minimum.y > previous.top + previous.height)
                selection_m.reset();
        }
        camera_.set_focus_rect(rect);
        if (camera_user_moved_)
        {
            camera_.set_focus_center_world(preserved);
            // A panel or sheet that opens over the selection moves the view just enough to keep the
            // selection in the stage left uncovered; a view the reader moved stays theirs otherwise.
            if (selection_m && !rect.empty())
            {
                const auto box = screen_box(*selection_m);
                // Something wider than the stage, such as the ground, is left where it is.
                const auto keep_inside = [](double minimum, double maximum, double low, double high)
                {
                    if (maximum - minimum > high - low)
                        return 0.0;
                    return minimum < low ? low - minimum : maximum > high ? high - maximum
                                                                          : 0.0;
                };
                const math::Vec2 shift { keep_inside(box.minimum.x, box.maximum.x, rect.left, rect.left + rect.width), keep_inside(box.minimum.y, box.maximum.y, rect.top, rect.top + rect.height) };
                if (math::length_squared(shift) > 0.0)
                    camera_.pan_by_screen_delta(shift);
            }
        }
        else
            frame_subject();
    }

    void SimulationSession::pan_view(const math::Vec2& screen_delta_px)
    {
        camera_.pan_by_screen_delta(screen_delta_px);
        camera_user_moved_ = true;
        follow_selection_ = false;
    }

    void SimulationSession::zoom_view(double wheel_delta, const math::Vec2& anchor_px)
    {
        if (std::abs(wheel_delta) <= 0.0)
        {
            return;
        }
        // Scrolling away from the viewer reduces the visible height, which reads as zooming in.
        camera_.zoom_about_screen_point(std::pow(camera_zoom_sensitivity_, -wheel_delta), anchor_px);
        camera_user_moved_ = true;
    }

    void SimulationSession::frame_everything()
    {
        const auto bounds = world_.compute_bounds();
        if (!bounds.is_empty())
        {
            camera_.frame_bounds(bounds);
            camera_user_moved_ = true;
        }
    }

    std::optional<math::Aabb> SimulationSession::authored_view_bounds() const
    {
        if (!scenario_document_)
            return std::nullopt;
        const auto* view = scenario_document_->root.find("view");
        if (view == nullptr || !view->is_object())
            return std::nullopt;
        const auto number = [&](std::string_view key) -> std::optional<double>
        {
            if (const auto* value = view->find(key); value != nullptr && value->is_number() && std::isfinite(value->as_number()))
                return value->as_number();
            return std::nullopt;
        };
        const auto x = number("center_x_m"), y = number("center_y_m"), height = number("height_m");
        if (!x || !y || !height || !(*height > 0.0))
            return std::nullopt;
        // A width makes the authored region fit both ways, so a narrow stage still shows all of it.
        const auto half_height = std::clamp(*height, 1.0, 500.0) * 0.5;
        const auto width = number("width_m");
        const auto half_width = width && *width > 0.0 ? std::min(*width, 5000.0) * 0.5 : 0.0;
        math::Aabb authored;
        authored.expand({ *x - half_width, *y - half_height });
        authored.expand({ *x + half_width, *y + half_height });
        return authored;
    }

    math::Aabb SimulationSession::predicted_motion_bounds(const math::Aabb& current_subject) const
    {
        // A short look-ahead on a copy of the world shows where the subject is about to go: the
        // wall a projectile strikes, a pendulum's swing, a slider's travel. The live world is
        // never stepped, and a very large scene is framed as it stands.
        constexpr double horizon_s = 1.5;
        constexpr std::size_t maximum_bodies = 256;
        // Fixed bodies up to this size are framed whole once something strikes them; larger
        // ones, such as the ground, are framed only around the subject.
        constexpr double whole_fixture_m = 3.0;
        const auto ids = world_.body_ids();
        if (ids.size() > maximum_bodies || current_subject.is_empty())
        {
            motion_prediction_ = {};
            return {};
        }

        std::uint64_t fingerprint = 1469598103934665603ull;
        const auto mix = [&](double value)
        {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &value, sizeof bits);
            fingerprint = (fingerprint ^ bits) * 1099511628211ull;
        };
        for (const auto id : ids)
            if (const auto* body = world_.find_body(id))
            {
                mix(static_cast<double>(id.index));
                mix(static_cast<double>(id.generation));
                mix(static_cast<double>(body->type()));
                mix(body->position_m().x);
                mix(body->position_m().y);
                mix(body->orientation_rad());
                mix(body->linear_velocity_m_s().x);
                mix(body->linear_velocity_m_s().y);
                mix(body->angular_velocity_rad_s());
                mix(body->mass_properties().mass_kg);
                mix(body->gravity_scale());
                mix(static_cast<double>(world_.force_generators(id).size()));
            }
        mix(world_.statistics().elapsed_time_s);
        mix(static_cast<double>(world_.constraints().size()));
        mix(static_cast<double>(world_.spring_ids().size()));
        mix(static_cast<double>(world_.force_generators().size()));
        mix(world_.settings().gravity_m_s2.x);
        mix(world_.settings().gravity_m_s2.y);
        mix(stepper_.substep_s());
        mix(static_cast<double>(stepper_.substep_count()));
        if (motion_prediction_.valid && motion_prediction_.fingerprint == fingerprint)
            return motion_prediction_.bounds;

        math::Aabb predicted;
        std::vector<physics::BodyId> struck;
        try
        {
            physics::World preview;
            preview.restore(world_.snapshot());
            preview.set_parallel_settings({ 0, preview.parallel_settings().minimum_batch_size });
            struct Track
            {
                physics::BodyId id;
                math::Vec2 previous_m;
                double travelled_m {};
                bool done {};
            };
            std::vector<Track> tracks;
            for (const auto id : ids)
                if (const auto* body = world_.find_body(id); body && body->type() != physics::BodyType::static_body && !is_marker(id))
                    tracks.push_back({ id, body->world_center_of_mass_m() });
            // A body that leaves at speed is followed only so far, so one runaway object cannot
            // shrink everything else to specks.
            const auto travel_limit_m = std::max(3.0, 4.0 * math::length(current_subject.extents()));
            std::vector<std::pair<physics::BodyId, physics::BodyId>> resting;
            const auto substeps = std::max(1, stepper_.substep_count());
            const auto step_s = stepper_.substep_s();
            const auto steps = step_s > 0.0 ? static_cast<int>(std::ceil(horizon_s / (step_s * substeps))) : 0;
            for (int step = 0; step < steps && !tracks.empty(); ++step)
            {
                for (int substep = 0; substep < substeps; ++substep)
                    preview.step(step_s, substep == 0);
                // Contact with a fixed body that is already there after the first step is resting
                // contact. A new one is an impact: the fixed body joins the subject and the
                // striking body's track ends, since a rebound is not worth zooming out for.
                for (const auto& manifold : preview.manifolds())
                {
                    if (manifold.point_count == 0)
                        continue;
                    const auto* first = preview.find_body(manifold.first);
                    const auto* second = preview.find_body(manifold.second);
                    if (first == nullptr || second == nullptr || (first->type() == physics::BodyType::static_body) == (second->type() == physics::BodyType::static_body))
                        continue;
                    const auto fixed = first->type() == physics::BodyType::static_body ? manifold.first : manifold.second;
                    const auto moving = first->type() == physics::BodyType::static_body ? manifold.second : manifold.first;
                    if (is_marker(fixed))
                        continue;
                    const auto pair = std::make_pair(moving, fixed);
                    if (step == 0)
                    {
                        resting.push_back(pair);
                        continue;
                    }
                    if (std::find(resting.begin(), resting.end(), pair) != resting.end())
                        continue;
                    if (std::find(struck.begin(), struck.end(), fixed) == struck.end())
                        struck.push_back(fixed);
                    for (auto& track : tracks)
                        if (track.id == moving && !track.done)
                        {
                            predicted.expand((first->type() == physics::BodyType::static_body ? second : first)->compute_bounds());
                            track.done = true;
                        }
                }
                bool following = false;
                for (auto& track : tracks)
                {
                    if (track.done)
                        continue;
                    const auto* body = preview.find_body(track.id);
                    if (body == nullptr)
                    {
                        track.done = true;
                        continue;
                    }
                    const auto centre = body->world_center_of_mass_m();
                    track.travelled_m += math::length(centre - track.previous_m);
                    track.previous_m = centre;
                    if (!(track.travelled_m <= travel_limit_m))
                    {
                        track.done = true;
                        continue;
                    }
                    predicted.expand(body->compute_bounds());
                    following = true;
                }
                if (!following)
                    break;
            }
            // The predicted path may widen the frame by about the subject's own size, so an
            // object falling or flying out of the experiment cannot zoom everything else away.
            if (!predicted.is_empty())
            {
                const auto reach = math::Vec2 { std::max(1.5, current_subject.extents().x), std::max(1.5, current_subject.extents().y) };
                predicted.minimum = math::max_components(predicted.minimum, current_subject.minimum - reach);
                predicted.maximum = math::min_components(predicted.maximum, current_subject.maximum + reach);
                if (predicted.is_empty())
                    predicted = {};
            }
            // A fixture the subject already rests on, such as a ramp, is the apparatus too.
            for (const auto& pair : resting)
                if (std::find(struck.begin(), struck.end(), pair.second) == struck.end())
                    struck.push_back(pair.second);
            for (const auto id : struck)
                if (const auto* body = world_.find_body(id))
                    if (const auto bounds = body->compute_bounds(); !bounds.is_empty() && std::max(bounds.extents().x, bounds.extents().y) <= whole_fixture_m)
                        predicted.expand(bounds);
        }
        catch (const std::exception& exception)
        {
            core::log_info("framing look-ahead skipped: {}", exception.what());
            predicted = {};
            struck.clear();
        }
        motion_prediction_ = { fingerprint, predicted, std::move(struck), true };
        return predicted;
    }

    math::Aabb SimulationSession::subject_bounds() const
    {
        if (const auto authored = authored_view_bounds())
            return *authored;
        const auto is_marker = [&](physics::BodyId id, const physics::RigidBody& body)
        {
            return body.type() == physics::BodyType::static_body && !body.colliders().empty() &&
                std::all_of(body.colliders().begin(), body.colliders().end(), [](const auto& collider)
                    {
                        return collider.is_sensor;
                    }) &&
                world_.force_generators(id).empty();
        };
        const auto body_bounds = [&](const physics::RigidBody& body)
        {
            return body.compute_bounds(body.interpolated_transform(scene_settings_.interpolation_alpha));
        };
        const auto anchor = [&](physics::BodyId id, const math::Vec2& local)
        {
            if (const auto* body = world_.find_body(id))
                return math::transform_point(body->interpolated_transform(scene_settings_.interpolation_alpha), local);
            return local;
        };

        math::Aabb subject;
        for (const auto id : world_.body_ids())
            if (const auto* body = world_.find_body(id); body != nullptr && body->type() != physics::BodyType::static_body && !is_marker(id, *body))
                subject.expand(body_bounds(*body));

        for (const auto& constraint : world_.constraints())
            if (const auto joint = std::dynamic_pointer_cast<const physics::JointConstraint>(constraint))
            {
                std::visit([&](const auto& endpoints)
                    {
                        subject.expand(anchor(endpoints.first, endpoints.local_anchor_first_m));
                        subject.expand(anchor(endpoints.second, endpoints.local_anchor_second_m));
                    },
                    joint->definition());
            }
        for (const auto spring : world_.spring_ids())
            if (const auto* definition = world_.spring_definition(spring))
                std::visit([&](const auto& value)
                    {
                        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, physics::LinearSpringDefinition>)
                        {
                            subject.expand(anchor(value.first, value.local_anchor_first_m));
                            subject.expand(anchor(value.second, value.local_anchor_second_m));
                        }
                        else
                        {
                            if (const auto* first = world_.find_body(value.first))
                                subject.expand(anchor(value.first, first->mass_properties().center_of_mass_m));
                            if (const auto* second = world_.find_body(value.second))
                                subject.expand(anchor(value.second, second->mass_properties().center_of_mass_m));
                        }
                    },
                    *definition);

        if (!subject.is_empty())
        {
            const auto current = subject;
            // A limited joint's whole travel belongs to the subject: the slider's carriage at
            // both stops, the hinged body swung through its permitted range.
            for (const auto& constraint : world_.constraints())
                if (const auto joint = std::dynamic_pointer_cast<const physics::JointConstraint>(constraint); joint && !joint->is_broken())
                {
                    const auto report = joint->report(world_);
                    const auto* moved = world_.find_body(report.second);
                    if (!report.limits_enabled || moved == nullptr || moved->type() == physics::BodyType::static_body)
                        continue;
                    const auto placement = moved->interpolated_transform(scene_settings_.interpolation_alpha);
                    if (report.kind == physics::JointKind::prismatic && math::length_squared(report.axis) > 0.0)
                        for (const auto limit : { report.lower_limit, report.upper_limit })
                        {
                            auto shifted = placement;
                            shifted.translation += math::normalized(report.axis) * (limit - report.translation_m);
                            subject.expand(moved->compute_bounds(shifted));
                        }
                    else if (report.kind == physics::JointKind::revolute)
                    {
                        constexpr int samples = 6;
                        for (int index = 0; index <= samples; ++index)
                        {
                            const auto limit = report.lower_limit + (report.upper_limit - report.lower_limit) * index / samples;
                            const math::Rotation2 turn { limit - report.angle_rad };
                            const math::Transform2 about_pivot { report.first_anchor_m - math::rotate(turn, report.first_anchor_m), turn };
                            subject.expand(moved->compute_bounds(math::concatenate(about_pivot, placement)));
                        }
                    }
                }
            subject.expand(predicted_motion_bounds(current));
            // A point a force pulls towards, and anything the guide asks about, is part of the
            // lesson even when it is a reference marker.
            for (const auto id : world_.body_ids())
                for (const auto& generator : world_.force_generators(id))
                    if (const auto* attractor = dynamic_cast<const physics::PointAttractor*>(generator.get()); attractor && attractor->is_enabled())
                    {
                        const auto target = attractor->world_position_m();
                        subject.expand(target);
                        for (const auto other : world_.body_ids())
                            if (const auto* marker = world_.find_body(other); marker && is_marker(other, *marker) && body_bounds(*marker).contains(target))
                                subject.expand(body_bounds(*marker));
                    }
            if (experiment_content_)
            {
                const auto include_document_body = [&](const std::string& document_id)
                {
                    if (document_id.empty())
                        return;
                    for (const auto id : world_.body_ids())
                        if (world_.body_document_id(id) == document_id)
                            if (const auto* body = world_.find_body(id))
                                subject.expand(body_bounds(*body));
                };
                for (const auto& step : experiment_content_->guide.steps)
                    include_document_body(step.body);
                for (const auto& variable : experiment_content_->guide.variables)
                    include_document_body(variable.body);
            }
        }

        bool used_fallback = false;
        if (subject.is_empty())
        {
            used_fallback = true;
            for (const auto id : world_.body_ids())
                if (const auto* body = world_.find_body(id); body != nullptr && !is_marker(id, *body))
                    subject.expand(body_bounds(*body));
            if (subject.is_empty())
                for (const auto id : world_.body_ids())
                    if (const auto* body = world_.find_body(id))
                        subject.expand(body_bounds(*body));
        }

        if (!subject.is_empty() && !used_fallback)
        {
            const auto width = subject.extents().x;
            const auto horizontal_minimum = subject.minimum.x - width * 0.25;
            const auto horizontal_maximum = subject.maximum.x + width * 0.25;
            // A support counts when the subject rests on it, strikes it during the look-ahead,
            // passes just above it, or will fall onto it under gravity however long that takes.
            // A floor far below a floating subject would only shrink the subject.
            const auto reach_m = std::max(0.3, subject.extents().y * 0.25);
            const auto reach_minimum_y = subject.minimum.y - reach_m, reach_maximum_y = subject.maximum.y + reach_m;
            const auto& touched = motion_prediction_.touched_fixtures;
            math::Vec2 gravity_m_s2 {};
            for (const auto& generator : world_.force_generators())
                if (generator && generator->is_enabled() && dynamic_cast<const physics::UniformGravity*>(generator.get()))
                    gravity_m_s2 = world_.settings().gravity_m_s2;
            const auto falls_onto = [&](const math::Aabb& support)
            {
                if (!(math::length_squared(gravity_m_s2) > 0.0))
                    return false;
                bool falls = false;
                world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                    {
                        if (falls || body.type() != physics::BodyType::dynamic_body || !(body.gravity_scale() > 0.0) || is_marker(id, body))
                            return;
                        const auto bounds = body_bounds(body);
                        falls = bounds.maximum.x >= support.minimum.x && bounds.minimum.x <= support.maximum.x && math::dot(support.center() - bounds.center(), gravity_m_s2) > 0.0;
                    });
                return falls;
            };
            for (const auto id : world_.body_ids())
                if (const auto* body = world_.find_body(id); body != nullptr && body->type() == physics::BodyType::static_body && !is_marker(id, *body))
                {
                    const auto support = body_bounds(*body);
                    if (support.is_empty() || support.maximum.x < horizontal_minimum || support.minimum.x > horizontal_maximum)
                        continue;
                    const auto near = support.maximum.y >= reach_minimum_y && support.minimum.y <= reach_maximum_y;
                    if (!near && std::find(touched.begin(), touched.end(), id) == touched.end() && !falls_onto(support))
                        continue;
                    math::Aabb clipped = support;
                    clipped.minimum.x = std::max(clipped.minimum.x, horizontal_minimum);
                    clipped.maximum.x = std::min(clipped.maximum.x, horizontal_maximum);
                    subject.expand(clipped);
                }
            const auto vertical_margin = subject.extents().y * 0.125;
            subject.minimum.y -= vertical_margin;
            subject.maximum.y += vertical_margin;
        }
        return subject;
    }

    void SimulationSession::frame_subject()
    {
        camera_user_moved_ = false;
        if (const auto authored = authored_view_bounds())
        {
            camera_.frame_bounds(*authored, 0.0);
            return;
        }

        const auto bounds = subject_bounds();
        if (bounds.is_empty())
        {
            camera_.set_center({});
            camera_.set_view_height(default_view_height_m_);
            return;
        }
        camera_.frame_bounds(bounds, 0.08);
        const auto& focus = camera_.focus_rect();
        const auto focus_height_m = focus.empty() ? camera_.view_height_m() : camera_.view_height_m() * focus.height / static_cast<double>(camera_.viewport().height);
        if (focus_height_m < 1.0)
        {
            auto minimum_bounds = bounds;
            const auto center = bounds.center();
            const auto minimum_extent = 1.0 / 1.16;
            minimum_bounds.minimum.y = std::min(minimum_bounds.minimum.y, center.y - minimum_extent * 0.5);
            minimum_bounds.maximum.y = std::max(minimum_bounds.maximum.y, center.y + minimum_extent * 0.5);
            camera_.frame_bounds(minimum_bounds, 0.08);
        }
    }

    void SimulationSession::frame_selection()
    {
        const auto* body = world_.find_body(scene_settings_.selection);
        if (body == nullptr)
        {
            notify(ui::Severity::info, "Nothing selected", "frame_selection");
            return;
        }

        auto bounds = body->compute_bounds(body->interpolated_transform(scene_settings_.interpolation_alpha));
        for (const auto id : selected_bodies_)
            if (const auto* selected = world_.find_body(id))
                bounds.expand(selected->compute_bounds(selected->interpolated_transform(scene_settings_.interpolation_alpha)));
        if (bounds.is_empty())
        {
            return;
        }
        // A tight frame on a small object leaves nothing around it for context, so the box is
        // widened before it is framed.
        bounds.grow(std::max(math::length(bounds.extents()), 0.2));
        camera_.frame_bounds(bounds, 0.1);
        camera_user_moved_ = true;
    }

    void SimulationSession::select_at(const math::Vec2& screen_point_px)
    {
        const auto world_point = camera_.screen_to_world(screen_point_px);
        physics::BodyId selected;
        world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (!is_marker(id) && body.contains_world_point(world_point, body.interpolated_transform(scene_settings_.interpolation_alpha)))
                {
                    selected = id;
                }
            });
        set_selection(selected);
    }

    physics::BodyId SimulationSession::selection() const
    {
        return scene_settings_.selection;
    }

    void SimulationSession::set_selection(physics::BodyId id)
    {
        if (shape_editor_.active())
            return;
        if (!world_.is_valid(id) || is_marker(id))
            id = {};
        const auto changed = !(scene_settings_.selection == id);
        if (changed)
        {
            authored_part_index_ = 0;
        }
        scene_settings_.selection = id;
        selected_bodies_.clear();
        if (id.is_valid())
            selected_bodies_.push_back(id);
    }

    math::Vec2 SimulationSession::pointer_world_position(const math::Vec2& screen_point_px) const
    {
        return camera_.screen_to_world(screen_point_px);
    }

    void SimulationSession::set_pointer_position(const math::Vec2& screen_point_px)
    {
        pointer_world_m_ = camera_.screen_to_world(screen_point_px);
    }

    bool SimulationSession::pick_surface_at(const math::Vec2& screen_point_px)
    {
        const auto id = pick_body_at(screen_point_px, ui_scale_);
        const auto* body = world_.find_body(id);
        if (!body)
            return false;
        const auto bounds = body->compute_bounds(body->interpolated_transform(scene_settings_.interpolation_alpha));
        const auto world = camera_.screen_to_world(screen_point_px);
        const auto horizontal_distance = std::min(std::abs(world.y - bounds.minimum.y), std::abs(world.y - bounds.maximum.y));
        const auto vertical_distance = std::min(std::abs(world.x - bounds.minimum.x), std::abs(world.x - bounds.maximum.x));
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_component_angle_degrees;
        command.value = horizontal_distance <= vertical_distance ? math::radians_to_degrees(body->orientation_rad()) : math::radians_to_degrees(body->orientation_rad()) + 90.0;
        apply(command);
        return true;
    }

    void SimulationSession::set_frame_time(double seconds)
    {
        frame_time_s_ = seconds;
        scene_renderer_.advance_presentation(seconds);
    }

    void SimulationSession::set_wall_time(double seconds)
    {
        if (std::isfinite(seconds))
        {
            wall_time_s_ = seconds;
            has_wall_time_ = true;
        }
    }

    bool SimulationSession::should_quit() const
    {
        return quit_requested_;
    }

    void SimulationSession::set_gravity_enabled(bool enabled)
    {
        if (!gravity_)
        {
            gravity_ = std::make_shared<physics::UniformGravity>();
            world_.add_force_generator(gravity_);
        }
        gravity_->set_enabled(enabled);
    }

    void SimulationSession::set_drag_enabled(bool enabled)
    {
        if (!drag_)
        {
            drag_ = std::make_shared<physics::AerodynamicDrag>();
            world_.add_force_generator(drag_);
        }
        drag_->set_enabled(enabled);
    }

    const ShapeEditor& SimulationSession::shape_editor() const
    {
        return shape_editor_;
    }

    std::vector<math::Vec2> SimulationSession::shape_snap_vertices() const
    {
        std::vector<math::Vec2> vertices;
        world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                const auto parts = physics::authored_parts(body);
                for (std::size_t index = 0; index < parts.size(); ++index)
                {
                    if (id == edited_shape_body_ && index == edited_part_index_)
                        continue;
                    const auto& part = parts[index];
                    const auto placement = math::concatenate(body.transform(), part->local_transform);
                    for (const auto& node : part->shape->source.nodes)
                        vertices.push_back(math::transform_point(placement, node.position_m));
                }
                for (const auto& collider : body.colliders())
                {
                    if (collider.authored_part || !collider.shape)
                        continue;
                    const auto placement = math::concatenate(body.transform(), collider.local_transform);
                    if (const auto* polygon = dynamic_cast<const physics::ConvexPolygonShape*>(collider.shape.get()))
                        for (const auto& vertex : polygon->vertices())
                            vertices.push_back(math::transform_point(placement, vertex));
                    else if (const auto* segment = dynamic_cast<const physics::SegmentShape*>(collider.shape.get()))
                    {
                        vertices.push_back(math::transform_point(placement, segment->start_m()));
                        vertices.push_back(math::transform_point(placement, segment->end_m()));
                    }
                }
            });
        return vertices;
    }

    bool SimulationSession::handle_scene_event(const ui::UiEvent& event, bool interface_consumed)
    {
        if ((event.kind == ui::UiEventKind::pointer_move || event.kind == ui::UiEventKind::pointer_down || event.kind == ui::UiEventKind::pointer_up) &&
            std::isfinite(event.pointer_px.x) && std::isfinite(event.pointer_px.y))
            set_pointer_position(event.pointer_px);
        if (!interface_consumed && !shape_editor_.active() && event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::primary && event.click_count >= 2)
        {
            const auto hit = pick_body_at(event.pointer_px, event.logical_pixel_scale);
            if (hit.is_valid())
            {
                set_selection(hit);
                if (const auto* body = world_.find_body(hit))
                {
                    const auto parts = physics::authored_parts(*body);
                    const auto world_point = camera_.screen_to_world(event.pointer_px);
                    double smallest_area = std::numeric_limits<double>::infinity();
                    std::optional<std::size_t> part_under_pointer;
                    for (std::size_t index = 0; index < parts.size(); ++index)
                    {
                        const auto& part = parts[index];
                        if (!part || !part->shape)
                            continue;
                        const auto placement = math::concatenate(body->transform(), part->local_transform);
                        const auto local_point = math::inverse_transform_point(placement, world_point);
                        double area = 0.0;
                        bool contains = false;
                        for (const auto& triangle : part->shape->render_triangles)
                        {
                            area += std::abs(math::cross(triangle[1] - triangle[0], triangle[2] - triangle[0])) * 0.5;
                            contains = contains || point_in_triangle(local_point, triangle);
                        }
                        if (contains && area < smallest_area)
                        {
                            smallest_area = area;
                            part_under_pointer = index;
                        }
                    }
                    if (part_under_pointer)
                    {
                        ui::UiCommand select_part;
                        select_part.kind = ui::UiCommandKind::select_authored_part;
                        select_part.value = static_cast<double>(*part_under_pointer);
                        apply(select_part);
                    }
                }
                ui::UiCommand edit;
                edit.kind = ui::UiCommandKind::edit_selected_shape;
                apply(edit);
                return true;
            }
        }
        if (!interface_consumed && event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::primary)
        {
            for (std::size_t index = 0; index < inspected_impacts_.size(); ++index)
            {
                const auto screen = camera_.world_to_screen(inspected_impacts_[index].point_m);
                if (math::length(screen - event.pointer_px) <= 6.0 * std::max(0.01, event.logical_pixel_scale))
                {
                    ui::UiCommand select;
                    select.kind = ui::UiCommandKind::select_impact;
                    select.value = static_cast<double>(index);
                    apply(select);
                    reveal_request_ = ui::RevealRequest { ++reveal_request_serial_, "measure.collisions.list", std::to_string(index) };
                    return true;
                }
            }
        }
        if (handle_stage_handle_event(event, interface_consumed))
            return true;
        if (event.kind == ui::UiEventKind::focus_lost && pending_edit_command_)
        {
            cancel_edit();
            held_reason_.clear();
        }
        if (event.kind == ui::UiEventKind::focus_lost && shape_editor_.active())
        {
            if (pending_edit_)
                cancel_edit();
            shape_editor_.release_pointer();
            dragging_view_ = false;
            secondary_pan_pending_ = false;
            return true;
        }
        if (handle_interaction_event(event, interface_consumed))
            return true;
        if (event.kind == ui::UiEventKind::pointer_up)
        {
            const auto was_panning = dragging_view_ || secondary_pan_pending_;
            if (event.button == ui::PointerButton::secondary && secondary_pan_pending_ && !dragging_view_ && !interface_consumed)
            {
                ui::ContextMenuRequest request;
                request.serial = ++context_menu_serial_;
                request.screen_position_px = event.pointer_px;
                bool impact_hit = false;
                for (std::size_t index = 0; index < inspected_impacts_.size(); ++index)
                {
                    if (math::length(camera_.world_to_screen(inspected_impacts_[index].point_m) - event.pointer_px) <= 6.0 * std::max(0.01, event.logical_pixel_scale))
                    {
                        request.kind = ui::StageTargetKind::impact;
                        request.id = std::to_string(index);
                        impact_hit = true;
                        break;
                    }
                }
                if (impact_hit)
                {
                }
                else if (const auto connection = pick_connection_at(event.pointer_px, event.logical_pixel_scale))
                {
                    request.kind = ui::StageTargetKind::connection;
                    request.id = connection->key;
                    request.detail = connection->kind;
                }
                else
                {
                    request.body = pick_body_at(event.pointer_px, event.logical_pixel_scale);
                    request.kind = request.body.is_valid() ? ui::StageTargetKind::object : ui::StageTargetKind::empty_space;
                    // The menu's commands act on the selection, so an object outside it becomes the
                    // selection first; right-clicking inside a multi-selection keeps the group.
                    if (request.body.is_valid() && !is_marker(request.body) && std::find(selected_bodies_.begin(), selected_bodies_.end(), request.body) == selected_bodies_.end())
                        set_selections({ request.body });
                }
                context_menu_anchor_m_ = camera_.screen_to_world(request.screen_position_px);
                context_menu_request_ = std::move(request);
            }
            dragging_view_ = false;
            secondary_pan_pending_ = false;
            const auto was_editing = shape_editor_.active() && handle_shape_event(event, interface_consumed);
            return was_panning || was_editing;
        }
        if (secondary_pan_pending_ && event.kind == ui::UiEventKind::pointer_move)
        {
            const auto scale = std::max(event.logical_pixel_scale, 1.0e-9);
            const auto travelled_logical_px = math::length(event.pointer_px - secondary_pan_start_px_) / scale;
            if (travelled_logical_px >= 4.0)
            {
                dragging_view_ = true;
                secondary_pan_pending_ = false;
                pan_view(event.pointer_px - secondary_pan_start_px_);
            }
            return true;
        }
        if (dragging_view_ && event.kind == ui::UiEventKind::pointer_move)
        {
            pan_view(event.pointer_delta_px);
            return true;
        }
        if (!interface_consumed && event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::middle)
        {
            dragging_view_ = true;
            return true;
        }
        if (!interface_consumed && event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::secondary)
        {
            secondary_pan_pending_ = true;
            secondary_pan_start_px_ = event.pointer_px;
            return true;
        }
        if (shape_editor_.active())
        {
            if (handle_shape_event(event, interface_consumed))
                return true;
        }
        if (interface_consumed)
            return false;
        if (event.kind == ui::UiEventKind::wheel)
        {
            zoom_view(event.wheel_delta, event.pointer_px);
            return true;
        }
        if (event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::primary)
        {
            select_at(event.pointer_px);
            return true;
        }
        return false;
    }

    void SimulationSession::begin_shape_editor(bool edit_selected)
    {
        if (shape_editor_.active())
        {
            notify(ui::Severity::warning, "Apply or discard the current outline first.", "shape");
            return;
        }
        physics::Outline outline;
        auto placement = math::Transform2 { camera_.center_m(), {} };
        std::optional<physics::ShapeAuthoringOptions> options;
        edited_shape_body_ = {};
        if (edit_selected)
        {
            const auto* body = world_.find_body(selection());
            const auto parts = body ? physics::authored_parts(*body) : std::vector<physics::AuthoredPartPtr> {};
            if (parts.empty())
            {
                notify(ui::Severity::warning, "Select an authored shape to edit its outline.", "shape");
                return;
            }
            authored_part_index_ %= parts.size();
            const auto& part = parts[authored_part_index_];
            edited_shape_body_ = selection();
            edited_part_index_ = authored_part_index_;
            outline = part->shape->source;
            options = part->shape->options;
            placement = math::concatenate(body->transform(), part->local_transform);
            shape_material_ = part->material;
            shape_edit_target_ = part->name.empty() ? body->name() : part->name;
        }
        else
        {
            const auto names = physics::materials::catalogue_names();
            shape_material_ = physics::materials::by_name(names[new_shape_material_index_ % names.size()]);
            shape_edit_target_ = "New shape";
        }
        stepper_.set_paused(true);
        pause_reason_ = { ui::PauseReason::drawing, 0 };
        stepper_.cancel_single_step_request();
        single_step_pending_ = false;
        synchronize_render_history();
        dragging_view_ = false;
        notifier_.clear_source("shape");
        shape_editor_.begin(std::move(outline), placement, options);
    }

    void SimulationSession::finish_shape_editor()
    {
        if (!shape_editor_.active())
            return;
        shape_editor_.end();
        edited_shape_body_ = {};
        dragging_view_ = false;
        synchronize_render_history();
    }

    void SimulationSession::request_context_menu_for_selection()
    {
        ui::ContextMenuRequest request;
        request.serial = ++context_menu_serial_;
        if (selected_connection_)
        {
            request.kind = ui::StageTargetKind::connection;
            request.id = selected_connection_->key;
            request.detail = selected_connection_->kind;
            if (const auto anchors = connection_anchors_m(selected_connection_->key, selected_connection_->kind))
                request.screen_position_px = camera_.world_to_screen((anchors->first + anchors->second) * 0.5);
            else
                request.screen_position_px = { camera_.viewport().width * 0.5, camera_.viewport().height * 0.5 };
        }
        else if (const auto* body = world_.find_body(selection()))
        {
            request.kind = ui::StageTargetKind::object;
            request.body = selection();
            request.screen_position_px = camera_.world_to_screen(body->world_center_of_mass_m());
        }
        else
        {
            request.kind = ui::StageTargetKind::empty_space;
            request.screen_position_px = camera_.focus_rect().empty() ? math::Vec2 { camera_.viewport().width * 0.5, camera_.viewport().height * 0.5 }
                                                                      : camera_.focus_rect().center();
        }
        context_menu_anchor_m_ = camera_.screen_to_world(request.screen_position_px);
        context_menu_request_ = std::move(request);
    }

    void SimulationSession::commit_shape_editor()
    {
        if (!shape_editor_.active())
            return;
        const auto result = shape_editor_.build();
        if (!result.succeeded())
        {
            notify(ui::Severity::error, shape_editor_.diagnostic(), "shape");
            return;
        }
        try
        {
            auto committed = edited_shape_body_;
            const auto editing = world_.is_valid(edited_shape_body_);
            const auto was_running = !stepper_.is_paused();
            if (editing)
                physics::edit_authored_part(world_, edited_shape_body_, edited_part_index_, result.shape);
            else if (!(edited_shape_body_ == physics::BodyId {}))
                throw std::invalid_argument("The edited body no longer exists. Cancel this draft and start a new outline.");
            else
            {
                physics::BodyDefinition body;
                body.name = "Drawn shape";
                body.position_m = shape_editor_.placement().translation;
                physics::AuthoredPartDefinition part;
                part.shape = result.shape;
                part.material = shape_material_;
                part.name = "Drawn outline";
                committed = physics::create_authored_body(world_, std::move(body), { part });
            }
            finish_shape_editor();
            if (!editing && was_running)
            {
                stepper_.set_paused(true);
                pause_reason_ = { ui::PauseReason::new_object, 0 };
            }
            set_selection(committed);
            scene_renderer_.clear_trajectories();
            notify(ui::Severity::success, editing ? "Part updated. Its material, position and orientation are unchanged." : "Shape created.", "shape");
            mark_edit_changed();
        }
        catch (const std::exception& error)
        {
            stepper_.set_paused(true);
            pause_reason_ = { ui::PauseReason::error, 0 };
            notify(ui::Severity::error, error.what(), "shape");
        }
    }

    bool SimulationSession::apply_shape_command(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        switch (command.kind)
        {
        case K::start_new_shape:
            begin_shape_editor(false);
            return true;
        case K::edit_selected_shape:
            begin_shape_editor(true);
            return true;
        case K::commit_shape_outline:
            commit_shape_editor();
            return true;
        case K::cancel_shape_outline:
            finish_shape_editor();
            notify(ui::Severity::info, "Draft cancelled.", "shape");
            return true;
        case K::set_shape_material:
            if (shape_editor_.active() && edited_shape_body_ == physics::BodyId {})
            {
                const auto names = physics::materials::catalogue_names();
                const auto found = std::find_if(names.begin(), names.end(), [&](std::string_view name)
                    {
                        std::string lowered(name);
                        std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c)
                            {
                                return static_cast<char>(std::tolower(c));
                            });
                        return name == command.id || lowered == command.id;
                    });
                if (found != names.end())
                {
                    new_shape_material_index_ = static_cast<std::size_t>(found - names.begin());
                    shape_material_ = physics::materials::by_name(*found);
                }
            }
            return true;
        case K::select_authored_part:
            if (!shape_editor_.active())
                if (const auto* body = world_.find_body(selection()))
                {
                    const auto parts = physics::authored_parts(*body);
                    const auto requested = command.value < 0.0 ? std::size_t { 0 } : static_cast<std::size_t>(command.value);
                    if (!parts.empty())
                        authored_part_index_ = std::min(requested, parts.size() - 1);
                }
            return true;
        case K::assemble_selected_bodies:
        case K::split_selected_body:
            if (shape_editor_.active())
                return true;
            try
            {
                const auto* body = world_.find_body(selection());
                if (command.kind == K::split_selected_body && (!body || physics::authored_parts(*body).empty()))
                    throw std::invalid_argument("Select an authored shape first.");
                if (command.kind == K::assemble_selected_bodies)
                {
                    const auto assembly = selected_bodies_;
                    if (assembly.size() < 2 || std::any_of(assembly.begin(), assembly.end(), [&](const auto id)
                                                   {
                                                       const auto* candidate = world_.find_body(id);
                                                       return !candidate || physics::authored_parts(*candidate).empty();
                                                   }))
                        throw std::invalid_argument("Select at least two authored shapes to combine.");
                    const auto combined = physics::assemble_authored_bodies(world_, assembly);
                    set_selection(combined);
                    notify(ui::Severity::success, "Objects combined. You can still edit each part.", "shape");
                    mark_edit_changed();
                    scene_renderer_.clear_trajectories();
                }
                else
                {
                    if (physics::authored_parts(*body).size() < 2)
                        throw std::invalid_argument("Select a combined object with at least two parts.");
                    const auto separated = physics::separate_authored_body(world_, selection());
                    set_selection(separated.empty() ? physics::BodyId {} : separated.front());
                    notify(ui::Severity::success, "Object separated into its original parts.", "shape");
                    mark_edit_changed();
                    scene_renderer_.clear_trajectories();
                }
                synchronize_render_history();
            }
            catch (const std::exception& error)
            {
                notify(ui::Severity::error, error.what(), "shape");
            }
            return true;
        default:
            if (shape_editor_.apply(command))
            {
                notifier_.clear_source("shape");
                return true;
            }
            return false;
        }
    }

    void SimulationSession::populate_shape_model(ui::UiModel& model) const
    {
        model.shape_editor_active = shape_editor_.active();
        model.shape_edit_target = shape_edit_target_;
        model.shape_material_name = shape_material_.name;
        model.shape_can_change_material = edited_shape_body_ == physics::BodyId {};
        model.shape_snap_grid = shape_editor_.snap_grid();
        model.shape_snap_vertices = shape_editor_.snap_vertices();
        model.shape_snap_angles = shape_editor_.snap_angles();
        model.shape_grid_spacing_m = shape_editor_.grid_spacing_m();
        const auto& options = shape_editor_.options();
        model.shape_vertex_budget = options.max_collision_vertices;
        model.shape_render_tolerance_m = options.render_tolerance_m;
        model.shape_collision_tolerance_m = options.collision_tolerance_m;
        model.shape_simplification_tolerance_m = options.simplification_tolerance_m;
        model.shape_concavity_tolerance_m = options.concavity_tolerance_m;
        model.draft.active = shape_editor_.active();
        model.draft.editing_existing = edited_shape_body_.is_valid();
        model.draft.target_name = shape_edit_target_;
        model.draft.material_name = shape_material_.name;
        if (shape_editor_.active())
        {
            const auto& outline = shape_editor_.outline();
            model.shape_outline_closed = outline.closed;
            model.shape_node_count = outline.nodes.size();
            model.shape_node_selected = shape_editor_.selected_node().has_value();
            if (const auto index = shape_editor_.selected_node())
            {
                model.shape_selected_node = *index;
                model.shape_selected_edge_cubic = outline.nodes[*index].outgoing_edge == physics::OutlineEdgeKind::cubic;
                const auto continuity = outline.nodes[*index].continuity;
                model.shape_continuity = continuity == physics::OutlineContinuity::corner ? "corner" : continuity == physics::OutlineContinuity::aligned ? "aligned"
                                                                                                                                                         : "mirrored";
            }
            const auto result = shape_editor_.build();
            model.shape_can_commit = result.succeeded();
            model.shape_diagnostic = shape_editor_.diagnostic();
            model.draft.closed = outline.closed;
            model.draft.valid = result.succeeded();
            model.draft.diagnostic = model.shape_diagnostic;
            model.draft.node_count = outline.nodes.size();
            if (result.shape)
            {
                model.shape_render_vertex_count = result.shape->render_outline.size();
                model.shape_collision_vertex_count = result.shape->collision_outline.size();
                model.shape_convex_part_count = result.shape->convex_parts.size();
                model.draft.drawing_vertex_count = result.shape->render_outline.size();
                model.draft.collision_vertex_count = result.shape->collision_outline.size();
                model.draft.piece_count = result.shape->convex_parts.size();
            }
        }
        if (const auto* body = world_.find_body(selection()))
        {
            const auto parts = physics::authored_parts(*body);
            model.authored_part_count = parts.size();
            model.selected_shape_authored = !parts.empty();
            model.can_split_shape = parts.size() > 1;
            if (!parts.empty())
            {
                model.authored_part_index = authored_part_index_ % parts.size();
                const auto& part = parts[model.authored_part_index];
                model.authored_part_name = part->name.empty() ? "Outline" : part->name;
                model.authored_material_name = part->material.name;
            }
        }
        model.can_assemble_shapes = selected_bodies_.size() >= 2 && std::all_of(selected_bodies_.begin(), selected_bodies_.end(), [&](const auto id)
                                                                        {
                                                                            const auto* candidate = world_.find_body(id);
                                                                            return candidate && !physics::authored_parts(*candidate).empty();
                                                                        });
    }

    void SimulationSession::apply_untracked(const ui::UiCommand& command)
    {
        if (starting_motion_edit(command))
        {
            apply_starting_motion_edit(command);
            return;
        }
        if (apply_education_command(command))
        {
            sync_setup_after_command(command);
            return;
        }
        if (apply_shape_command(command))
        {
            sync_setup_after_command(command);
            return;
        }
        switch (command.kind)
        {
        case ui::UiCommandKind::none:
            break;

        case ui::UiCommandKind::add_object:
        {
            const auto was_running = !stepper_.is_paused();
            if (!stepper_.is_paused())
            {
                stepper_.set_paused(true);
                pause_reason_ = { ui::PauseReason::new_object, 0 };
            }
            physics::BodyDefinition definition;
            const auto base_name = command.id == "plank" ? std::string { "Plank" } : command.id == "box" ? std::string { "Box" }
                                                                                                         : std::string { "Ball" };
            std::size_t number = 1;
            world_.for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    if (body.name().rfind(base_name, 0) == 0)
                        ++number;
                });
            definition.name = base_name + " " + std::to_string(number);
            definition.type = physics::BodyType::dynamic_body;
            physics::Collider collider;
            collider.shape = command.id == "plank" ? physics::make_box(1.2, 0.18) : command.id == "box" ? physics::make_box(0.45, 0.45)
                                                                                                        : physics::make_circle(0.23);
            definition.colliders.push_back(std::move(collider));
            auto position = camera_.focus_rect().empty() ? camera_.center_m() : camera_.screen_to_world(camera_.focus_rect().center());
            position.x = std::round(position.x * 10.0) / 10.0;
            position.y = std::round(position.y * 10.0) / 10.0;
            definition.position_m = position;
            for (int attempt = 0; attempt < 32; ++attempt)
            {
                physics::RigidBody candidate(definition);
                const auto bounds = candidate.compute_bounds();
                bool occupied = false;
                world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                    {
                        if (!is_marker(id) && math::overlaps(bounds, body.compute_bounds()))
                            occupied = true;
                    });
                if (!occupied)
                    break;
                definition.position_m.x += 0.15;
                if (attempt % 6 == 5)
                {
                    definition.position_m.x = position.x;
                    definition.position_m.y += 0.15;
                }
            }
            const auto id = world_.create_body(definition);
            set_selection(id);
            mark_edit_changed();
            synchronize_render_history();
            ui::NotificationAction undo;
            undo.label = "Undo";
            ui::UiCommand undo_command;
            undo_command.kind = ui::UiCommandKind::undo;
            undo.command = undo_command;
            notify(ui::Severity::success, "Added **" + definition.name + "**", "add-object", undo);
            if (was_running)
                pause_reason_ = { ui::PauseReason::new_object, 0 };
            break;
        }

        case ui::UiCommandKind::set_visual_effect:
        {
            using Settings = render::SceneRenderSettings;
            const std::pair<const char*, bool Settings::*> options[] = {
                { "material_shading", &Settings::material_shading }, { "contact_shadows", &Settings::contact_shadows }, { "depth_background", &Settings::depth_background }, { "motion_trails", &Settings::motion_trails }, { "directional_blur", &Settings::directional_blur }, { "impact_flashes", &Settings::impact_flashes }, { "impact_sparks", &Settings::impact_sparks }, { "impact_dust", &Settings::impact_dust }, { "soft_deformation", &Settings::soft_deformation }, { "transitions", &Settings::transitions }
            };
            for (const auto& option : options)
                if (command.id == option.first)
                    scene_settings_.*option.second = command.flag;
            if (reduce_motion_)
                apply_reduce_motion();
            refresh_effects_quality();
            break;
        }

        case ui::UiCommandKind::set_visual_budget:
        {
            if (!std::isfinite(command.value) || command.value < 0 || std::floor(command.value) != command.value)
                break;
            using Settings = render::SceneRenderSettings;
            struct Budget
            {
                const char* id;
                std::size_t Settings::* field;
                double maximum;
            };
            const Budget budgets[] = {
                { "motion_body_budget", &Settings::motion_body_budget, 128 },
                { "directional_blur_budget", &Settings::directional_blur_budget, 128 },
                { "shading_body_budget", &Settings::shading_body_budget, 4096 },
                { "contact_shadow_budget", &Settings::contact_shadow_budget, 192 },
                { "impact_flash_budget", &Settings::impact_flash_budget, 48 },
                { "spark_budget", &Settings::spark_budget, 256 },
                { "dust_budget", &Settings::dust_budget, 128 },
                { "deformation_budget", &Settings::deformation_budget, 64 }
            };
            for (const auto& budget : budgets)
                if (command.id == budget.id)
                    scene_settings_.*budget.field = static_cast<std::size_t>(std::min(command.value, budget.maximum));
            refresh_effects_quality();
            break;
        }

        case ui::UiCommandKind::toggle_pause:
            stepper_.set_paused(!stepper_.is_paused());
            if (stepper_.is_paused())
            {
                synchronize_render_history();
            }
            else
                pause_reason_ = {};
            break;

        case ui::UiCommandKind::single_step:
            stepper_.request_single_step();
            single_step_pending_ = true;
            break;

        case ui::UiCommandKind::step_many:
        {
            const auto count = std::clamp(static_cast<int>(std::llround(command.value)), 1, 100);
            for (int index = 0; index < count; ++index)
            {
                stepper_.request_single_step();
                single_step_pending_ = true;
                advance(0.0);
            }
            break;
        }

        case ui::UiCommandKind::reset_scenario:
            if (command.flag || world_.statistics().elapsed_time_s > 0.0)
            {
                close_current_run(true);
                reset_scenario_untracked();
                if (command.flag)
                    stepper_.set_paused(false);
            }
            break;

        case ui::UiCommandKind::load_scenario:
            load_scenario_untracked(command.id);
            break;

        case ui::UiCommandKind::set_time_scale:
        {
            const auto previous = stepper_.time_scale();
            stepper_.set_time_scale(command.value);
            if (previous != stepper_.time_scale())
                mark_edit_changed();
            if (stepper_.time_scale() == 0.0)
            {
                synchronize_render_history();
            }
            break;
        }

        case ui::UiCommandKind::set_fixed_step:
        {
            // UI commands are validated at the session boundary. TimeStepper's internal clamp is
            // useful for configuration loading, but an out-of-range edit must not become a
            // different, undoable value than the learner entered.
            if (!std::isfinite(command.value) || command.value < 1.0e-4 || command.value > 0.02)
                break;
            const auto previous = stepper_.fixed_step_s();
            stepper_.set_fixed_step(command.value);
            if (previous != stepper_.fixed_step_s())
            {
                lab_.fixed_step_s = stepper_.fixed_step_s();
                mark_edit_changed();
            }
            synchronize_render_history();
            break;
        }

        case ui::UiCommandKind::set_integrator:
            if (command.id == "semi_implicit_euler" || command.id == "velocity_verlet" || command.id == "runge_kutta_4")
            {
                lab_.integrator_id = command.id;
                world_.set_integrator(integrator_from_id(lab_.integrator_id));
                mark_edit_changed();
                synchronize_render_history();
            }
            break;

        case ui::UiCommandKind::compare_integrators:
            if (energy_comparison_.empty())
            {
                energy_comparison_settings_ = {};
                energy_comparison_settings_.duration_s = 10.0;
                energy_comparison_error_.clear();
                try
                {
                    energy_comparison_ = physics::compare_harmonic_energy_drift(energy_comparison_settings_);
                }
                catch (const std::exception& error)
                {
                    energy_comparison_error_ = error.what();
                    core::log_warning("energy comparison failed: {}", energy_comparison_error_);
                }
            }
            if (energy_comparison_error_.empty())
            {
                ui::NotificationAction reveal;
                reveal.label = "View results";
                reveal.reveal_key = "measure.theory.integration_run";
                notify(ui::Severity::success, "Energy-drift comparison complete.", "comparison", reveal);
            }
            break;

        case ui::UiCommandKind::set_restitution_mixing:
        {
            auto settings = world_.settings();
            if (command.id == "maximum")
                settings.collision.restitution_mixing = physics::MaterialMixing::maximum;
            else if (command.id == "minimum")
                settings.collision.restitution_mixing = physics::MaterialMixing::minimum;
            else if (command.id == "arithmetic_mean")
                settings.collision.restitution_mixing = physics::MaterialMixing::arithmetic_mean;
            else if (command.id == "geometric_mean")
                settings.collision.restitution_mixing = physics::MaterialMixing::geometric_mean;
            else
                break;
            world_.set_settings(settings);
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::set_continuous_collision:
        {
            auto settings = world_.settings();
            settings.collision.continuous = command.flag;
            world_.set_settings(settings);
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::set_warm_starting:
        {
            auto settings = world_.settings();
            settings.solver.warm_starting = command.flag;
            world_.set_settings(settings);
            lab_.warm_starting = command.flag;
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::set_constraint_graph:
        {
            auto settings = world_.settings();
            settings.constraint_graph_enabled = command.flag;
            world_.set_settings(settings);
            lab_.constraint_graph = command.flag;
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::compare_restitution:
            if (restitution_comparison_.empty())
            {
                restitution_comparison_settings_ = {};
                restitution_comparison_error_.clear();
                try
                {
                    restitution_comparison_ = physics::compare_restitution_drops(restitution_comparison_settings_);
                }
                catch (const std::exception& error)
                {
                    restitution_comparison_error_ = error.what();
                    core::log_warning("bounce comparison failed: {}", restitution_comparison_error_);
                }
            }
            if (restitution_comparison_error_.empty())
            {
                ui::NotificationAction reveal;
                reveal.label = "View results";
                reveal.reveal_key = "measure.theory.bounce_run";
                notify(ui::Severity::success, "Bounce comparison complete.", "comparison", reveal);
            }
            break;

        case ui::UiCommandKind::set_layer:
        {
            bool found = false;
            const auto layer = layer_from_id(command.id, found);
            if (found)
            {
                scene_settings_.layers.set(layer, command.flag);
                if (layer == render::VisualizationLayer::trajectories && !command.flag)
                    scene_renderer_.clear_trajectories();
            }
            break;
        }

        case ui::UiCommandKind::set_layer_mask:
            if (command.id == "recommended")
                scene_settings_.layers = recommended_layers();
            else if (command.id == "all")
                scene_settings_.layers = render::LayerMask::all();
            else if (command.id == "none")
                scene_settings_.layers = render::LayerMask::none();
            else
                scene_settings_.layers = render::LayerMask { static_cast<std::uint32_t>(command.value) };
            break;

        case ui::UiCommandKind::select_body:
            selected_connection_.reset();
            set_selection(command.body);
            break;

        case ui::UiCommandKind::select_bodies:
            selected_connection_.reset();
            set_selections(command.bodies);
            break;

        case ui::UiCommandKind::select_connection:
        {
            const auto valid_joint = command.detail == "joint" && world_.constraint_by_key(command.id);
            const auto valid_spring = command.detail == "spring" && world_.spring_by_key(command.id).is_valid();
            if (valid_joint || valid_spring)
                selected_connection_ = ui::SelectedConnection { command.id, command.detail };
            break;
        }

        case ui::UiCommandKind::clear_selection:
            set_selection({});
            break;

        case ui::UiCommandKind::set_selected_position:
        case ui::UiCommandKind::set_selected_orientation:
        case ui::UiCommandKind::stop_selected_motion:
        {
            const auto targets = command.body.is_valid() ? std::vector<physics::BodyId> { command.body } : selected_bodies_;
            for (const auto id : targets)
                if (auto* body = world_.find_body(id); body && body->type() != physics::BodyType::static_body)
                {
                    bool changed = false;
                    if (command.kind == ui::UiCommandKind::set_selected_position)
                    {
                        const auto position = command.detail == "offset" ? body->position_m() + math::Vec2 { command.value, command.value_y }
                            : command.detail == "x"                      ? math::Vec2 { command.value, body->position_m().y }
                            : command.detail == "y"                      ? math::Vec2 { body->position_m().x, command.value }
                                                                         : math::Vec2 { command.value, command.value_y };
                        if (math::is_finite(position) && std::abs(position.x) <= 100.0 && std::abs(position.y) <= 100.0)
                        {
                            body->set_position(position);
                            changed = true;
                        }
                    }
                    else if (command.kind == ui::UiCommandKind::set_selected_orientation && std::isfinite(command.value))
                    {
                        body->set_orientation(math::degrees_to_radians(command.value));
                        changed = true;
                    }
                    else if (command.kind == ui::UiCommandKind::stop_selected_motion)
                    {
                        body->set_linear_velocity({});
                        body->set_angular_velocity(0.0);
                        changed = true;
                    }
                    if (!changed)
                        continue;
                    body->capture_previous_transform();
                    (void)world_.notify_body_properties_changed(id);
                    mark_edit_changed();
                }
            break;
        }

        case ui::UiCommandKind::set_joint_motor_enabled:
        case ui::UiCommandKind::set_joint_limits_enabled:
        case ui::UiCommandKind::reverse_joint_motor:
        case ui::UiCommandKind::set_joint_motor_speed:
        {
            const auto key = !command.id.empty() ? command.id : selected_connection_ && selected_connection_->kind == "joint" ? selected_connection_->key
                                                                                                                              : std::string {};
            if (key.empty())
                break;
            const auto joint = std::dynamic_pointer_cast<physics::JointConstraint>(world_.constraint_by_key(key));
            if (!joint || joint->is_broken() || (command.body.is_valid() && !(joint->first_body() == command.body) && !(joint->second_body() == command.body)))
                break;
            auto definition = joint->definition();
            bool edited = false;
            std::visit([&](auto& value)
                {
                    using Definition = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition> ||
                        std::is_same_v<Definition, physics::PrismaticJointDefinition>)
                    {
                        if (command.kind == ui::UiCommandKind::set_joint_motor_enabled)
                            value.motor_enabled = command.flag;
                        else if (command.kind == ui::UiCommandKind::set_joint_limits_enabled)
                            value.limits_enabled = command.flag;
                        else if (command.kind == ui::UiCommandKind::set_joint_motor_speed)
                        {
                            if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition>)
                            {
                                if (std::isfinite(command.value) && std::abs(command.value) <= 50.0)
                                    value.motor_speed_rad_s = command.value;
                                else
                                    return;
                            }
                            else
                            {
                                if (std::isfinite(command.value) && std::abs(command.value) <= 10.0)
                                    value.motor_speed_m_s = command.value;
                                else
                                    return;
                            }
                        }
                        else if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition>)
                            value.motor_speed_rad_s = -value.motor_speed_rad_s;
                        else
                            value.motor_speed_m_s = -value.motor_speed_m_s;
                        edited = true;
                    }
                },
                definition);
            if (edited)
            {
                joint->set_definition(definition);
                mark_edit_changed();
            }
            break;
        }

        case ui::UiCommandKind::set_spring_parameter:
        {
            const auto key = !command.id.empty() ? command.id : selected_connection_ && selected_connection_->kind == "spring" ? selected_connection_->key
                                                                                                                               : std::string {};
            const auto spring = world_.spring_by_key(key);
            const auto* current = world_.spring_definition(spring);
            if (!current || !std::isfinite(command.value) || command.value < 0.0)
                break;
            auto definition = *current;
            bool changed = false;
            std::visit([&](auto& value)
                {
                    using Definition = std::decay_t<decltype(value)>;
                    if (command.detail == "stiffness")
                    {
                        if constexpr (std::is_same_v<Definition, physics::LinearSpringDefinition>)
                            value.stiffness_n_m = command.value;
                        else
                            value.stiffness_n_m_rad = command.value;
                        changed = true;
                    }
                    else if (command.detail == "damping")
                    {
                        if constexpr (std::is_same_v<Definition, physics::LinearSpringDefinition>)
                            value.damping_n_s_m = command.value;
                        else
                            value.damping_n_m_s_rad = command.value;
                        changed = true;
                    }
                },
                definition);
            if (changed && world_.set_spring(spring, definition))
                mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::delete_selected_body:
        {
            if (shape_editor_.active())
            {
                ui::UiCommand remove;
                remove.kind = ui::UiCommandKind::remove_shape_node;
                shape_editor_.apply(remove);
                break;
            }
            const bool delete_lone_fixed_object = selected_bodies_.size() == 1;
            bool removed = false;
            std::vector<physics::BodyId> kept;
            for (const auto id : selected_bodies_)
            {
                const auto* body = world_.find_body(id);
                if (body != nullptr && body->type() == physics::BodyType::static_body && !delete_lone_fixed_object)
                {
                    kept.push_back(id);
                    continue;
                }
                removed = world_.destroy_body(id) || removed;
            }
            if (removed)
            {
                set_selections(kept);
                mark_edit_changed();
            }
            break;
        }

        case ui::UiCommandKind::set_gravity_enabled:
            refresh_force_generators();
            set_gravity_enabled(command.flag);
            mark_edit_changed();
            break;

        case ui::UiCommandKind::set_gravity_magnitude:
        case ui::UiCommandKind::set_gravity_angle_degrees:
        {
            if (!std::isfinite(command.value) || (command.kind == ui::UiCommandKind::set_gravity_magnitude && command.value < 0.0))
                break;
            const auto gravity = world_.settings().gravity_m_s2;
            auto magnitude = std::hypot(gravity.x, gravity.y);
            if (command.kind == ui::UiCommandKind::set_gravity_angle_degrees && !std::isfinite(magnitude))
                break;
            if (magnitude > 0.0)
                gravity_direction_degrees_ = math::radians_to_degrees(std::atan2(gravity.y, gravity.x));
            if (command.kind == ui::UiCommandKind::set_gravity_angle_degrees)
                gravity_direction_degrees_ = std::remainder(command.value, 360.0);
            else
                magnitude = command.value;
            const auto angle = math::degrees_to_radians(gravity_direction_degrees_);
            auto settings = world_.settings();
            settings.gravity_m_s2 = math::Vec2 { std::cos(angle), std::sin(angle) } * magnitude;
            world_.set_settings(settings);
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::set_gravity_preset:
        {
            const auto magnitude = command.id == "earth" ? physics::standard_gravity_m_s2
                : command.id == "moon"                   ? 1.62
                : command.id == "mars"                   ? 3.73
                                                         : -1.0;
            if (magnitude < 0.0)
                break;
            gravity_direction_degrees_ = -90.0;
            auto settings = world_.settings();
            settings.gravity_m_s2 = math::Vec2 { 0.0, -magnitude };
            world_.set_settings(settings);
            mark_edit_changed();
            break;
        }

        case ui::UiCommandKind::set_selected_gravity_scale:
        {
            const auto targets = command.body.is_valid() ? std::vector<physics::BodyId> { command.body } : selected_bodies_;
            const bool supported = !targets.empty() && std::all_of(targets.begin(), targets.end(), [&](physics::BodyId id)
                                                           {
                                                               const auto* body = world_.find_body(id);
                                                               return body && body->type() == physics::BodyType::dynamic_body;
                                                           });
            if (shape_editor_.active() || !supported)
            {
                notify(ui::Severity::error, "Select only dynamic bodies and finish the outline before changing gravity scale.", "property");
                break;
            }
            if (std::isfinite(command.value) && command.value >= 0.0)
                for (const auto id : targets)
                    if (auto* body = world_.find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                    {
                        body->set_gravity_scale(command.value);
                        (void)world_.notify_body_properties_changed(id);
                        mark_edit_changed();
                    }
            break;
        }

        case ui::UiCommandKind::set_energy_reference_height:
            if (std::isfinite(command.value) && command.value >= -100.0 && command.value <= 100.0)
            {
                world_.set_potential_energy_reference_height(command.value);
                mark_edit_changed();
            }
            break;

        case ui::UiCommandKind::star_run:
            if (!run_recorder_.star(static_cast<int>(std::llround(command.value)), command.flag))
                notify(ui::Severity::warning, "At most 8 runs can be starred. Unstar one first.", "runs");
            break;
        case ui::UiCommandKind::clear_runs:
            run_recorder_.clear_unstarred();
            break;
        case ui::UiCommandKind::pin_run_value:
        {
            ui::RunAggregator aggregator = ui::RunAggregator::at_end;
            if (command.detail == "maximum")
                aggregator = ui::RunAggregator::maximum;
            else if (command.detail == "minimum")
                aggregator = ui::RunAggregator::minimum;
            else if (command.detail == "at_first_impact")
                aggregator = ui::RunAggregator::at_first_impact;
            else if (command.detail == "at_time")
                aggregator = ui::RunAggregator::at_time;
            if (!run_recorder_.pin({ {}, command.id, command.body, aggregator, command.value }))
                notify(ui::Severity::warning, "At most 12 values can be pinned. Remove one first.", "runs");
            break;
        }
        case ui::UiCommandKind::unpin_run_value:
            (void)run_recorder_.unpin(command.id);
            break;

        case ui::UiCommandKind::set_drag_enabled:
            refresh_force_generators();
            set_drag_enabled(command.flag);
            mark_edit_changed();
            break;

        case ui::UiCommandKind::set_angular_drag_enabled:
            refresh_force_generators();
            if (!drag_)
                set_drag_enabled(false);
            if (auto* drag = dynamic_cast<physics::AerodynamicDrag*>(drag_.get()))
            {
                auto settings = drag->settings();
                settings.angular_drag = command.flag;
                drag->set_settings(settings);
            }
            mark_edit_changed();
            break;

        case ui::UiCommandKind::set_magnus_enabled:
            refresh_force_generators();
            if (!drag_)
                set_drag_enabled(false);
            if (auto* drag = dynamic_cast<physics::AerodynamicDrag*>(drag_.get()))
            {
                auto settings = drag->settings();
                settings.magnus_lift = command.flag;
                drag->set_settings(settings);
            }
            mark_edit_changed();
            break;

        case ui::UiCommandKind::set_environment_parameter:
        {
            if (!std::isfinite(command.value))
                break;
            auto settings = world_.settings();
            bool changed = false;
            const auto assign = [&](double& target, double value)
            {
                changed = changed || target != value;
                target = value;
            };
            if (command.id == "air_density_kg_m3" && command.value >= 0.0 && command.value <= 10.0)
                assign(settings.air_density_kg_m3, command.value);
            else if (command.id == "air_velocity_x_m_s" && std::abs(command.value) <= 30.0)
                assign(settings.air_velocity_m_s.x, command.value);
            else if (command.id == "air_velocity_y_m_s" && std::abs(command.value) <= 30.0)
                assign(settings.air_velocity_m_s.y, command.value);
            else if (command.id == "air_dynamic_viscosity_pa_s" && command.value >= 1.0e-6 && command.value <= 1.0e-3)
                assign(settings.air_dynamic_viscosity_pa_s, command.value);
            if (changed)
            {
                world_.set_settings(settings);
                mark_edit_changed();
            }
            break;
        }

        case ui::UiCommandKind::frame_all:
            frame_everything();
            break;

        case ui::UiCommandKind::frame_selection:
            frame_selection();
            break;

        case ui::UiCommandKind::frame_subject:
            frame_subject();
            break;

        case ui::UiCommandKind::set_view_height:
        {
            // The value is the height the status line reports, so the visible stage shows exactly
            // that much while the focus area keeps its centre.
            const auto& focus = camera_.focus_rect();
            const auto& stage = visible_stage_rect_.empty() ? focus : visible_stage_rect_;
            const auto viewport_height = static_cast<double>(camera_.viewport().height);
            if (stage.empty() || !(viewport_height > 0.0))
                camera_.set_view_height(command.value);
            else
            {
                const auto centre = camera_.screen_to_world(focus.empty() ? stage.center() : focus.center());
                camera_.set_view_height(command.value * viewport_height / stage.height);
                camera_.set_focus_center_world(centre);
            }
            camera_user_moved_ = true;
            break;
        }

        case ui::UiCommandKind::set_vector_scope:
            scene_settings_.vectors_selected_only = command.id == "selected";
            break;

        case ui::UiCommandKind::set_preference:
        {
            const auto key = command.detail.empty() ? std::string_view(command.id) : std::string_view(command.detail);
            if (key == "prefs.accessibility.reduce_motion")
            {
                const auto* chosen = find_effects_preset(effects_quality_);
                reduce_motion_ = command.flag;
                if (reduce_motion_)
                    apply_reduce_motion();
                else
                {
                    // Turning the preference off restores the suppressed effects to the chosen
                    // quality, or to their factory state for a custom mix, rather than leaving the
                    // viewer wondering why effects stay disabled.
                    const render::SceneRenderSettings defaults;
                    scene_settings_.transitions = chosen ? chosen->transitions : defaults.transitions;
                    scene_settings_.impact_flashes = chosen ? chosen->impact_flashes : defaults.impact_flashes;
                    scene_settings_.directional_blur = chosen ? chosen->directional_blur : defaults.directional_blur;
                    scene_settings_.impact_sparks = chosen ? chosen->impact_sparks : defaults.impact_sparks;
                    scene_settings_.impact_dust = chosen ? chosen->impact_dust : defaults.impact_dust;
                }
                refresh_effects_quality();
            }
            else if (key == "view.follow_selection")
                follow_selection_ = command.flag;
            else if (key == "prefs.playback.pause_in_background")
                pause_in_background_ = command.flag;
            else if (key == "prefs.experiments.keep_lab_settings")
                keep_lab_settings_ = command.flag;
            else if (key == "prefs.experiments.open_running")
                start_paused_ = !command.flag;
            else if (key == "prefs.experiments.recommended_view")
                recommended_view_ = command.flag;
            else if (key == "prefs.experiments.ask_predictions")
                ask_predictions_ = command.flag;
            else if (key == "prefs.experiments.on_start")
                experiments_on_start_ = command.id;
            else if (key == "prefs.effects.quality" || key == "prefs.effects.restore_defaults")
            {
                // Custom names the current mix, so choosing it keeps the switches as they are.
                if (const auto* preset = find_effects_preset(key == "prefs.effects.quality" ? std::string_view(command.id) : std::string_view("standard")))
                    apply_effects_preset(scene_settings_, *preset);
                if (reduce_motion_)
                    apply_reduce_motion();
                refresh_effects_quality();
            }
            else if (key == "prefs.capture.area")
                capture_area_ = command.id;
            break;
        }

        case ui::UiCommandKind::restore_original:
            if (original_.world.is_valid())
            {
                close_current_run(false);
                world_.restore(original_.world);
                gravity_direction_degrees_ = original_.gravity_direction_degrees;
                apply_lab_settings(lab_);
                refresh_force_generators();
                setup_.world = world_.snapshot();
                setup_.gravity_direction_degrees = gravity_direction_degrees_;
                rebuild_snapshot_views();
                stepper_.reset();
                stepper_.set_paused(true);
                state_setup_toast_shown_ = false;
                synchronize_render_history();
                frame_subject();
                mark_edit_changed();
                notify(ui::Severity::success, "Restored the original experiment.", "restore-original");
            }
            break;

        case ui::UiCommandKind::keep_state_as_setup:
            close_current_run(false);
            world_.restart_timeline();
            setup_.world = world_.snapshot();
            setup_.gravity_direction_degrees = gravity_direction_degrees_;
            rebuild_snapshot_views();
            stepper_.reset();
            stepper_.set_paused(true);
            state_setup_toast_shown_ = false;
            mark_edit_changed();
            break;

        case ui::UiCommandKind::revert_lab_settings:
            apply_lab_settings(default_lab_);
            setup_.world = world_.snapshot();
            setup_.gravity_direction_degrees = gravity_direction_degrees_;
            rebuild_snapshot_views();
            synchronize_render_history();
            mark_edit_changed();
            break;

        case ui::UiCommandKind::revert_change:
            if (command.id == "setup" || command.id.rfind("setup:", 0) == 0)
                reset_scenario_untracked();
            break;

        case ui::UiCommandKind::quit:
            quit_requested_ = true;
            break;
        default:
            break;
        }
        sync_setup_after_command(command);
    }

    std::vector<std::pair<physics::BodyId, std::string>> SimulationSession::object_names() const
    {
        std::vector<std::pair<physics::BodyId, std::string>> names;
        for (const auto id : world_.body_ids())
        {
            const auto* body = world_.find_body(id);
            if (!body)
                continue;
            auto name = core::humanise_identifier(body->name());
            if (experiment_content_)
            {
                const auto document_id = world_.body_document_id(id);
                const auto annotation = std::find_if(experiment_content_->bodies.begin(), experiment_content_->bodies.end(), [&](const auto& value)
                    {
                        return value.document_id == document_id;
                    });
                if (annotation != experiment_content_->bodies.end() && !annotation->label.empty())
                    name = annotation->label;
            }
            names.emplace_back(id, std::move(name));
        }
        // Unnamed objects are numbered free first, then driven, then fixed; markers stay unnumbered.
        std::size_t unnamed_index = 1;
        for (const auto type : { physics::BodyType::dynamic_body, physics::BodyType::kinematic_body, physics::BodyType::static_body })
            for (auto& [id, name] : names)
                if (name.empty() && world_.find_body(id)->type() == type && !is_marker(id))
                    name = "Object " + std::to_string(unnamed_index++);
        for (auto& entry : names)
            if (entry.second.empty())
                entry.second = "Object";
        return names;
    }

    bool SimulationSession::is_marker(physics::BodyId id) const
    {
        const auto* body = world_.find_body(id);
        if (!body)
            return false;
        if (experiment_content_)
        {
            const auto document_id = world_.body_document_id(id);
            const auto annotation = std::find_if(experiment_content_->bodies.begin(), experiment_content_->bodies.end(), [&](const auto& value)
                {
                    return value.document_id == document_id;
                });
            if (annotation != experiment_content_->bodies.end() && annotation->role == "marker")
                return true;
        }
        return body->type() == physics::BodyType::static_body && !body->colliders().empty() &&
            std::all_of(body->colliders().begin(), body->colliders().end(), [](const auto& collider)
                {
                    return collider.is_sensor;
                }) &&
            world_.force_generators(id).empty();
    }

    std::size_t SimulationSession::user_object_count() const
    {
        std::size_t count = 0;
        for (const auto id : world_.body_ids())
            if (!is_marker(id) && !original_view_.is_valid(id))
                ++count;
        return count;
    }

    bool SimulationSession::has_setup_changes() const
    {
        return user_object_count() > 0 || !compute_setup_changes(setup_view_, original_view_).empty();
    }

    void SimulationSession::populate_run_model(ui::UiModel& model) const
    {
        model.current_run = run_recorder_.current();
        model.previous_run = run_recorder_.previous();
        model.runs = run_recorder_.kept(run_recorder_.experiment_id());
        model.pinned_values = run_recorder_.pinned(run_recorder_.experiment_id());
        model.starred_run_count = run_recorder_.starred_count();
        if (model.current_run)
            model.graph_markers = model.current_run->markers;
        model.changes = compute_setup_changes(setup_view_, original_view_);
        model.scenario_content = experiment_content_;
        model.selected_connection = selected_connection_;
        if (!pending_leave_scenario_.empty())
        {
            ui::UiCommand confirm { ui::UiCommandKind::load_scenario };
            confirm.id = pending_leave_scenario_;
            confirm.flag = true;
            ui::UiCommand cancel { ui::UiCommandKind::load_scenario };
            cancel.detail = "cancel";
            ui::UiCommand save { ui::UiCommandKind::save_arrangement };
            const auto count = user_object_count();
            const auto title = scenario_document_ ? scenario_document_->metadata.title : scenario_id_;
            model.confirmation = ui::ConfirmationModel { "Leave " + title + "?",
                core::substitute("{}{}{} will be lost.", count, count == 1 ? " drawn or imported object" : " drawn or imported objects", shape_editor_.active() ? " and an open draft" : ""),
                "Leave",
                std::move(confirm),
                std::move(cancel),
                std::move(save) };
        }
        else if (pending_quit_confirmation_)
        {
            ui::UiCommand confirm { ui::UiCommandKind::quit };
            confirm.flag = true;
            ui::UiCommand cancel { ui::UiCommandKind::quit };
            cancel.detail = "cancel";
            ui::UiCommand save { ui::UiCommandKind::save_arrangement };
            const auto count = compute_setup_changes(setup_view_, original_view_).size();
            model.confirmation = ui::ConfirmationModel { "Quit?", core::substitute("Quit and lose {} change{}?", count, count == 1 ? "" : "s"), "Quit", std::move(confirm), std::move(cancel), std::move(save) };
        }
        const auto first = undo_edits_.size() > 20 ? undo_edits_.size() - 20 : 0;
        for (std::size_t index = undo_edits_.size(); index > first; --index)
        {
            const auto& edit = undo_edits_[index - 1];
            physics::World before;
            before.restore(edit.before.world);
            model.undo_history.push_back({ edit.label, edit.category, before.statistics().elapsed_time_s });
        }
        auto names = object_names();
        for (auto& [id, name] : names)
        {
            const auto* body = world_.find_body(id);
            ui::ObjectItem item;
            item.id = id;
            item.document_id = std::string(world_.body_document_id(id));
            item.display_name = std::move(name);
            item.kind = body->type() == physics::BodyType::dynamic_body ? "free" : body->type() == physics::BodyType::kinematic_body ? "driven"
                                                                                                                                     : "fixed";
            item.role = is_marker(id) ? "marker" : "object";
            item.moving = ui::body_moving(*body);
            item.selected = std::find(selected_bodies_.begin(), selected_bodies_.end(), id) != selected_bodies_.end();
            if (experiment_content_)
            {
                const auto annotation = std::find_if(experiment_content_->bodies.begin(), experiment_content_->bodies.end(), [&](const auto& value)
                    {
                        return value.document_id == item.document_id;
                    });
                if (annotation != experiment_content_->bodies.end())
                {
                    item.role = is_marker(id) ? "marker" : annotation->role;
                    item.caption = annotation->caption;
                }
            }
            if (item.role != "marker" && !original_view_.is_valid(id))
                ++model.user_object_count;
            model.objects.push_back(std::move(item));
        }
    }

    render::LayerMask SimulationSession::recommended_layers() const
    {
        auto layers = base_layers_;
        if (experiment_content_)
            for (const auto& layer_id : experiment_content_->presentation.layers)
            {
                bool found = false;
                const auto layer = layer_from_id(layer_id, found);
                if (found)
                    layers.set(layer, true);
            }
        return layers;
    }

    ui::UiModel SimulationSession::build_model() const
    {
        ui::UiModel model;
        model.world = &world_;
        model.setup_world = setup_.world.is_valid() ? &setup_view_ : nullptr;
        model.original_world = original_.world.is_valid() ? &original_view_ : nullptr;
        model.change_serial = change_serial_;
        model.visual_settings = scene_settings_;
        model.selection = scene_settings_.selection;
        model.selected_bodies = selected_bodies_;
        model.can_undo = !undo_edits_.empty();
        model.can_redo = !redo_edits_.empty();
        model.undo_label = undo_edits_.empty() ? "" : undo_edits_.back().label;
        model.redo_label = redo_edits_.empty() ? "" : redo_edits_.back().label;
        // While the draft editor owns the stage none of the pointer tools is in use, so the tool
        // switch shows no active segment; the underlying mode returns with Apply or Discard.
        model.interaction_mode = shape_editor_.active()        ? ""
            : interaction_.mode == InteractionMode::move       ? "select"
            : interaction_.mode == InteractionMode::throw_body ? "throw"
            : interaction_.mode == InteractionMode::pull       ? "pull"
                                                               : "select";
        model.held_reason = held_reason_;
        model.theme_id = theme_id_;
        model.ui_scale = ui_scale_;
        model.camera_zoom_sensitivity = camera_zoom_sensitivity_;
        for (const auto& binding : key_bindings)
            if (binding.action != AppAction::toggle_developer_overlay)
            {
                // A command with two keys is one entry naming both, as the menus show it once.
                const auto same_command = std::find_if(model.keyboard_reference.begin(), model.keyboard_reference.end(), [&](const ui::KeyReference& entry)
                    {
                        return entry.description == binding.description;
                    });
                if (same_command != model.keyboard_reference.end())
                {
                    same_command->chord += " / " + std::string(binding.chord);
                    continue;
                }
                const auto category = binding.action == AppAction::toggle_pause || binding.action == AppAction::pause_at_next_impact || binding.action == AppAction::single_step || binding.action == AppAction::step_many || binding.action == AppAction::reset_scenario || binding.action == AppAction::replay                                                                                                                                      ? ui::KeyCategory::playback
                    : binding.action == AppAction::select_mode || binding.action == AppAction::throw_mode || binding.action == AppAction::pull_mode                                                                                                                                                                                                                                                                                                   ? ui::KeyCategory::tools
                    : binding.action == AppAction::frame_subject || binding.action == AppAction::frame_selection                                                                                                                                                                                                                                                                                                                                      ? ui::KeyCategory::view
                    : binding.action == AppAction::open_library || binding.action == AppAction::toggle_show || binding.action == AppAction::toggle_measure || binding.action == AppAction::open_world || binding.action == AppAction::toggle_guide || binding.action == AppAction::toggle_inspector_pin || binding.action == AppAction::open_main_menu || binding.action == AppAction::open_preferences || binding.action == AppAction::open_add_menu ? ui::KeyCategory::surfaces
                    : binding.action == AppAction::quit                                                                                                                                                                                                                                                                                                                                                                                               ? ui::KeyCategory::files
                    : binding.action == AppAction::toggle_help                                                                                                                                                                                                                                                                                                                                                                                        ? ui::KeyCategory::help
                                                                                                                                                                                                                                                                                                                                                                                                                                                      : ui::KeyCategory::editing;
                model.keyboard_reference.push_back({ std::string(binding.chord), std::string(binding.description), category, ui::KeyContext::global });
            }
        populate_shape_model(model);
        populate_education_model(model);
        populate_run_model(model);
        const auto gravity = world_.settings().gravity_m_s2;
        model.gravity_direction_degrees = std::hypot(gravity.x, gravity.y) > 0.0
            ? math::radians_to_degrees(std::atan2(gravity.y, gravity.x))
            : gravity_direction_degrees_;
        model.air_velocity_m_s = world_.settings().air_velocity_m_s;
        model.air_dynamic_viscosity_pa_s = world_.settings().air_dynamic_viscosity_pa_s;
        const physics::AerodynamicDrag* drag = nullptr;
        const physics::AerodynamicDrag* terminal_drag = nullptr;
        std::size_t air_model_count = 0;
        std::size_t gravity_model_count = 0;
        const auto count_enabled_effects = [&](const physics::ForceGeneratorPtr& generator)
        {
            if (!generator || !generator->is_enabled())
                return;
            if (dynamic_cast<const physics::UniformGravity*>(generator.get()))
                ++gravity_model_count;
            if (const auto* candidate = dynamic_cast<const physics::AerodynamicDrag*>(generator.get()))
            {
                ++air_model_count;
                terminal_drag = candidate;
            }
        };
        for (const auto& generator : world_.force_generators())
        {
            if (!generator)
                continue;
            count_enabled_effects(generator);
            if (const auto* candidate = dynamic_cast<const physics::AerodynamicDrag*>(generator.get()))
                drag = candidate;
        }
        if (drag)
        {
            model.drag_enabled = drag->is_enabled();
            model.angular_drag_enabled = drag->settings().angular_drag;
            model.magnus_enabled = drag->settings().magnus_lift;
        }
        const auto* selected = world_.find_body(model.selection);
        if (selected)
            for (const auto& generator : world_.force_generators(model.selection))
                count_enabled_effects(generator);
        if (!selected || selected->type() != physics::BodyType::dynamic_body)
            model.terminal_speed_note = "Select a dynamic body.";
        else if (air_model_count == 0)
            model.terminal_speed_note = "Air effects are off.";
        else if (air_model_count != 1)
            model.terminal_speed_note = "Multiple air models.";
        else if (!model.drag_enabled || terminal_drag != drag)
            model.terminal_speed_note = "Body-specific air model.";
        else if (gravity_model_count == 0)
            model.terminal_speed_note = "Gravity is off.";
        else if (gravity_model_count != 1)
            model.terminal_speed_note = "Multiple gravity fields.";
        else
        {
            physics::ForceContext context;
            context.gravity_m_s2 = gravity;
            context.air_density_kg_m3 = world_.settings().air_density_kg_m3;
            context.air_velocity_m_s = model.air_velocity_m_s;
            context.air_dynamic_viscosity_pa_s = model.air_dynamic_viscosity_pa_s;
            try
            {
                if (const auto estimate = physics::estimate_terminal_speed(*selected, context, drag->settings()))
                    model.terminal_speed_m_s = estimate->speed_m_s;
                else
                    model.terminal_speed_note = "No finite drag balance.";
            }
            catch (const std::exception&)
            {
                model.terminal_speed_note = "Estimate unavailable.";
            }
        }
        model.energy_comparison_settings = energy_comparison_settings_;
        model.energy_comparison = energy_comparison_;
        if (!energy_comparison_error_.empty())
            model.inline_notices.push_back({ "measure.energy.comparison", ui::Severity::error, energy_comparison_error_ });
        model.restitution_comparison_settings = restitution_comparison_settings_;
        model.restitution_comparison = restitution_comparison_;
        if (!restitution_comparison_error_.empty())
            model.inline_notices.push_back({ "measure.restitution.comparison", ui::Severity::error, restitution_comparison_error_ });
        model.paused = stepper_.is_paused();
        model.time_scale = stepper_.time_scale();
        model.fixed_step_s = stepper_.fixed_step_s();
        model.substeps = stepper_.substep_count();
        model.discarded_time_s = stepper_.discarded_time_s();
        model.elapsed_time_s = world_.statistics().elapsed_time_s;
        model.run_state = model.elapsed_time_s <= 0.0 ? ui::RunState::ready : stepper_.is_paused() ? ui::RunState::paused
                                                                                                   : ui::RunState::running;
        model.frame_time_s = frame_time_s_;
        model.performance = { frame_time_s_, frame_time_s_ > 0.0 ? 1.0 / frame_time_s_ : 0.0, stepper_.substep_count(), render_backend_name_, interface_backend_name_, pointer_world_m_, stage_view_height_m() };
        model.context_menu_request = context_menu_request_;
        if (model.context_menu_request)
            model.context_menu_request->screen_position_px = camera_.world_to_screen(context_menu_anchor_m_);
        model.reveal_request = reveal_request_;
        // The objects a hover card should leave in view: every free or driven object beside the
        // one it describes, as screen boxes.
        const auto stage_objects = [&](physics::BodyId described)
        {
            std::vector<render::ScreenRect> boxes;
            world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (id == described || body.type() == physics::BodyType::static_body || is_marker(id) || boxes.size() >= 64)
                        return;
                    const auto bounds = body.compute_bounds(body.interpolated_transform(scene_settings_.interpolation_alpha));
                    if (bounds.is_empty())
                        return;
                    const auto a = camera_.world_to_screen(bounds.minimum), b = camera_.world_to_screen(bounds.maximum);
                    boxes.push_back({ std::min(a.x, b.x), std::min(a.y, b.y), std::abs(b.x - a.x), std::abs(b.y - a.y) });
                });
            return boxes;
        };
        if (!interaction_.hover_connection_key.empty())
        {
            const auto elapsed = (has_wall_time_ ? wall_time_s_ : 0.0) - interaction_.hover_started_s;
            if (elapsed >= 0.15)
            {
                ui::HoverModel hover;
                hover.kind = ui::StageTargetKind::connection;
                hover.id = interaction_.hover_connection_key;
                hover.silhouette = true;
                if (const auto target = hover_target_bounds())
                    hover.screen_bounds = *target;
                if (elapsed >= hover_card_delay_s)
                {
                    hover.card_lines.push_back(interaction_.hover_connection_kind == "spring" ? "Spring" : "Joint");
                    hover.card_lines.push_back(core::humanise_identifier(interaction_.hover_connection_key));
                    hover.avoid = stage_objects({});
                }
                model.hover = std::move(hover);
            }
        }
        else if (world_.is_valid(interaction_.hover_body))
        {
            const auto* body = world_.find_body(interaction_.hover_body);
            const auto elapsed = (has_wall_time_ ? wall_time_s_ : 0.0) - interaction_.hover_started_s;
            if (body && elapsed >= 0.15)
            {
                ui::HoverModel hover;
                hover.kind = ui::StageTargetKind::object;
                hover.body = interaction_.hover_body;
                hover.silhouette = true;
                if (const auto target = hover_target_bounds())
                    hover.screen_bounds = *target;
                if (elapsed >= hover_card_delay_s)
                {
                    hover.avoid = stage_objects(interaction_.hover_body);
                    const auto item = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& candidate)
                        {
                            return candidate.id == interaction_.hover_body;
                        });
                    if (item != model.objects.end() && item->role == "marker")
                    {
                        hover.card_lines.push_back(item->display_name);
                        if (!item->caption.empty())
                            hover.card_lines.push_back(item->caption);
                    }
                    else
                    {
                        // The card stands in for the stage label it hides, naming the object
                        // exactly as the panels do, and adds what the label cannot: what kind of
                        // object it is and how it is moving right now.
                        hover.card_lines.push_back(item != model.objects.end() ? item->display_name : body->name().empty() ? "Object"
                                                                                                                           : core::humanise_identifier(body->name()));
                        if (body->type() == physics::BodyType::static_body)
                            hover.card_lines.push_back("Fixed object");
                        else
                        {
                            const auto speed = math::length(body->linear_velocity_m_s());
                            const auto kind = body->type() == physics::BodyType::kinematic_body ? std::string_view { "Driven object" } : std::string_view { "Free object" };
                            // The Inspector's word and test for the same state.
                            hover.card_lines.push_back(!ui::body_moving(*body) ? core::substitute("{} \xC2\xB7 resting", kind)
                                    : speed > 1.0e-3                           ? core::substitute("{} \xC2\xB7 moving at {}", kind, core::format_quantity(speed, core::DisplayQuantity::velocity, model.display_units))
                                                                               : core::substitute("{} \xC2\xB7 moving", kind));
                            hover.card_lines.push_back("Mass " + core::format_quantity(body->mass_properties().mass_kg, core::DisplayQuantity::mass, model.display_units));
                        }
                    }
                }
                model.hover = std::move(hover);
            }
        }
        model.scenario_id = scenario_id_;
        model.notifications = notifier_.notifications();
        if (shape_editor_.active())
            model.can_export_shape = shape_editor_.build().succeeded();
        else if (const auto* body = world_.find_body(selection()))
            model.can_export_shape = !physics::authored_parts(*body).empty();
        model.layers = scene_settings_.layers;
        model.recommended_layers = recommended_layers();
        model.vector_scales = scene_settings_.vector_scales;
        model.drawn_force_scale = scene_renderer_.drawn_force_scale();
        model.pointer_world_m = pointer_world_m_;
        model.view_height_m = stage_view_height_m();
        model.render_backend = render_backend_name_;
        model.interface_backend = interface_backend_name_;
        model.reduce_motion = reduce_motion_;
        model.pause_in_background = pause_in_background_;
        model.keep_lab_settings = keep_lab_settings_;
        model.open_experiments_running = !start_paused_;
        model.recommended_view = recommended_view_;
        model.ask_predictions = ask_predictions_;
        model.follow_selection = follow_selection_;
        model.experiments_on_start = experiments_on_start_;
        model.effects_quality = effects_quality_;
        model.capture_area = capture_area_;
        model.last_setup_file = last_setup_file_;
        const auto hs = overlay_scale();
        if ((model.run_state == ui::RunState::ready || model.run_state == ui::RunState::paused) && selected && selected_bodies_.size() <= 1)
        {
            const auto centre = camera_.world_to_screen(selected->world_center_of_mass_m());
            const auto velocity_tip = velocity_knob_position(camera_, selected->world_center_of_mass_m(), selected->linear_velocity_m_s(), velocity_handle_scale(), hs);
            model.handles.push_back({ "velocity", ui::StageTargetKind::handle, { velocity_tip.x - 12.0 * hs, velocity_tip.y - 12.0 * hs, 24.0 * hs, 24.0 * hs }, selection(), selected->type() == physics::BodyType::dynamic_body, "Only free objects have editable velocity." });
            const auto knob = rotation_knob_position(centre, handle_ring_radius(*selected), selected->orientation_rad());
            model.handles.push_back({ "rotation", ui::StageTargetKind::handle, { knob.x - 12.0 * hs, knob.y - 12.0 * hs, 24.0 * hs, 24.0 * hs }, selection(), selected->type() != physics::BodyType::static_body, "Fixed objects cannot rotate." });
        }
        const auto compass = gravity_compass_centre(camera_, hs);
        const auto compass_extent = std::max(overlay::handle_hit_radius, overlay::compass_radius + 2.0) * hs;
        model.handles.push_back({ "gravity", ui::StageTargetKind::handle, { compass.x - compass_extent, compass.y - compass_extent, 2.0 * compass_extent, 2.0 * compass_extent }, {}, true, {} });
        if (scene_settings_.layers.is_enabled(render::VisualizationLayer::constraints))
        {
            for (const auto& constraint : world_.constraints())
                if (const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get()))
                {
                    const auto report = joint->report(world_);
                    const auto position = camera_.world_to_screen((report.first_anchor_m + report.second_anchor_m) * 0.5);
                    model.handles.push_back({ "connection:" + std::string(world_.constraint_key(constraint)), ui::StageTargetKind::connection, { position.x - 6.0 * hs, position.y - 6.0 * hs, 12.0 * hs, 12.0 * hs }, {}, true, {} });
                }
            for (const auto id : world_.spring_ids())
                if (const auto report = world_.spring_report(id))
                {
                    const auto position = camera_.world_to_screen((report->first_anchor_m + report->second_anchor_m) * 0.5);
                    model.handles.push_back({ "connection:" + std::string(world_.spring_key(id)), ui::StageTargetKind::connection, { position.x - 6.0 * hs, position.y - 6.0 * hs, 12.0 * hs, 12.0 * hs }, {}, true, {} });
                }
        }
        for (const auto& impact : model.impacts)
        {
            const auto position = camera_.world_to_screen(impact.contact_point_m);
            model.handles.push_back({ "impact:" + std::to_string(impact.index), ui::StageTargetKind::impact, { position.x - 6.0 * hs, position.y - 6.0 * hs, 12.0 * hs, 12.0 * hs }, {}, true, {} });
        }
        const auto current_lab = capture_lab_settings();
        if (current_lab.integrator_id != default_lab_.integrator_id)
            model.lab_changes.push_back({ "integrator", "Integration method", integrator_label(default_lab_.integrator_id), integrator_label(current_lab.integrator_id) });
        if (current_lab.fixed_step_s != default_lab_.fixed_step_s)
            model.lab_changes.push_back({ "fixed_step", "Time step", core::format_quantity(default_lab_.fixed_step_s, core::DisplayQuantity::time, model.display_units), core::format_quantity(current_lab.fixed_step_s, core::DisplayQuantity::time, model.display_units) });
        if (current_lab.warm_starting != default_lab_.warm_starting)
            model.lab_changes.push_back({ "warm_starting", "Reuse contact impulses", default_lab_.warm_starting ? "On" : "Off", current_lab.warm_starting ? "On" : "Off" });
        if (current_lab.constraint_graph != default_lab_.constraint_graph)
            model.lab_changes.push_back({ "constraint_graph", "Solve joints together", default_lab_.constraint_graph ? "On" : "Off", current_lab.constraint_graph ? "On" : "Off" });
        for (const auto& scenario : physics::available_scenarios())
        {
            ui::ExperimentCard card;
            card.id = std::string(scenario.id);
            card.title = std::string(scenario.title);
            card.summary = std::string(scenario.summary);
            if (const auto* document = physics::scenario_document_for_id(scenario.id))
            {
                const auto& metadata = document->metadata;
                const auto content = parse_experiment_content(*document);
                card.title = metadata.title;
                card.summary = metadata.summary;
                card.collection = content.collection;
                card.level = content.level;
                card.hook = metadata.hook;
                card.collection_order = metadata.collection_order;
                card.suggested_order = metadata.suggested_order;
                card.concepts = metadata.concepts;
                card.builds_on = metadata.prerequisites;
                card.tags = metadata.tags;
            }
            model.catalogue.push_back(std::move(card));
        }

        if (scenario_document_)
        {
            const auto& metadata = scenario_document_->metadata;
            model.scenario_title = metadata.title;
            model.scenario_summary = metadata.summary;
            model.scenario_concepts = metadata.concepts;
            model.scenario_prerequisites = metadata.prerequisites;
            model.scenario_suggested_order = metadata.suggested_order;
        }
        else if (const auto* description = physics::find_scenario(scenario_id_); description != nullptr)
        {
            model.scenario_title = std::string { description->title };
            model.scenario_summary = std::string { description->summary };
        }

        return model;
    }

} // namespace rigidbodies::app
