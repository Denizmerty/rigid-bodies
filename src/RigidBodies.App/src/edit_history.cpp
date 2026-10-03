#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::app
{
    namespace
    {
        bool same_outline(const ShapeEditor& a, const ShapeEditor& b)
        {
            if (a.active() != b.active())
                return false;
            const auto& x = a.outline();
            const auto& y = b.outline();
            if (x.closed != y.closed || x.nodes.size() != y.nodes.size())
                return false;
            for (std::size_t i = 0; i < x.nodes.size(); ++i)
            {
                const auto& p = x.nodes[i];
                const auto& q = y.nodes[i];
                if (!(p.position_m == q.position_m) || !(p.incoming_handle_m == q.incoming_handle_m) ||
                    !(p.outgoing_handle_m == q.outgoing_handle_m) || p.outgoing_edge != q.outgoing_edge || p.continuity != q.continuity)
                    return false;
            }
            const auto& p = a.options();
            const auto& q = b.options();
            return p.render_tolerance_m == q.render_tolerance_m && p.collision_tolerance_m == q.collision_tolerance_m &&
                p.simplification_tolerance_m == q.simplification_tolerance_m && p.concavity_tolerance_m == q.concavity_tolerance_m &&
                p.max_collision_vertices == q.max_collision_vertices;
        }

        std::string edit_label(ui::UiCommandKind kind)
        {
            using K = ui::UiCommandKind;
            switch (kind)
            {
            case K::load_scenario:
                return "Change scenario";
            case K::reset_scenario:
                return "Back to start";
            case K::restore_original:
                return "Restore original";
            case K::keep_state_as_setup:
                return "Keep as starting state";
            case K::revert_change:
                return "Revert change";
            case K::revert_lab_settings:
                return "Revert lab settings";
            case K::delete_selected_body:
                return "Delete selection";
            case K::add_object:
                return "Add object";
            case K::set_selected_mass:
            case K::use_selected_density_mass:
            case K::set_selected_material:
            case K::set_selected_velocity_x:
            case K::set_selected_velocity_y:
            case K::set_selected_velocity:
            case K::set_selected_angular_velocity:
            case K::set_selected_position:
            case K::set_selected_orientation:
            case K::stop_selected_motion:
            case K::set_selected_gravity_scale:
                return "Edit selection properties";
            case K::start_new_shape:
            case K::edit_selected_shape:
            case K::commit_shape_outline:
            case K::cancel_shape_outline:
            case K::close_shape_outline:
            case K::insert_shape_node:
            case K::remove_shape_node:
            case K::set_shape_edge:
            case K::set_shape_continuity:
            case K::set_shape_material:
            case K::set_shape_vertex_budget:
            case K::set_shape_render_tolerance:
            case K::set_shape_collision_tolerance:
            case K::set_shape_simplification_tolerance:
            case K::set_shape_concavity_tolerance:
                return "Edit shape";
            case K::assemble_selected_bodies:
            case K::split_selected_body:
                return "Change assembly";
            case K::set_fixed_step:
            case K::set_integrator:
            case K::set_restitution_mixing:
            case K::set_continuous_collision:
            case K::set_warm_starting:
            case K::set_constraint_graph:
            case K::set_solver_parameter:
                return "Edit simulation settings";
            case K::set_gravity_enabled:
            case K::set_gravity_magnitude:
            case K::set_gravity_angle_degrees:
            case K::set_gravity_preset:
            case K::set_drag_enabled:
            case K::set_angular_drag_enabled:
            case K::set_magnus_enabled:
            case K::set_force_generator_enabled:
            case K::set_environment_parameter:
                return "Edit forces";
            case K::set_joint_motor_enabled:
            case K::set_joint_limits_enabled:
            case K::reverse_joint_motor:
            case K::set_joint_motor_speed:
                return "Edit joint";
            case K::set_spring_parameter:
                return "Edit spring";
            case K::set_energy_reference_height:
                return "Set zero height";
            default:
                return {};
            }
        }

        bool same_control(const ui::UiCommand& left, const ui::UiCommand& right)
        {
            return left.kind == right.kind && left.id == right.id && left.body == right.body &&
                left.bodies == right.bodies && left.detail == right.detail;
        }

        bool command_can_coalesce(const ui::UiCommandKind kind)
        {
            using K = ui::UiCommandKind;
            switch (kind)
            {
            case K::set_physics_workers:
            case K::set_solver_parameter:
            case K::set_environment_parameter:
            case K::set_time_scale:
            case K::set_fixed_step:
            case K::set_vector_scale:
            case K::set_component_angle_degrees:
            case K::set_selected_mass:
            case K::set_selected_velocity_x:
            case K::set_selected_velocity_y:
            case K::set_selected_angular_velocity:
            case K::set_selected_position:
            case K::set_shape_grid_spacing:
            case K::set_shape_vertex_budget:
            case K::set_shape_render_tolerance:
            case K::set_shape_collision_tolerance:
            case K::set_shape_simplification_tolerance:
            case K::set_shape_concavity_tolerance:
            case K::set_gravity_magnitude:
            case K::set_gravity_angle_degrees:
            case K::set_selected_gravity_scale:
                return true;
            default:
                return false;
            }
        }

        ui::EditCategory edit_category(ui::UiCommandKind kind)
        {
            using K = ui::UiCommandKind;
            switch (kind)
            {
            case K::set_integrator:
            case K::set_fixed_step:
            case K::set_warm_starting:
            case K::set_constraint_graph:
            case K::set_solver_parameter:
            case K::revert_lab_settings:
                return ui::EditCategory::lab;
            case K::set_selected_mass:
            case K::use_selected_density_mass:
            case K::set_selected_material:
            case K::set_selected_gravity_scale:
            case K::set_joint_motor_enabled:
            case K::set_joint_limits_enabled:
            case K::reverse_joint_motor:
            case K::set_joint_motor_speed:
            case K::set_spring_parameter:
            case K::set_gravity_enabled:
            case K::set_gravity_magnitude:
            case K::set_gravity_angle_degrees:
            case K::set_gravity_preset:
            case K::set_drag_enabled:
            case K::set_angular_drag_enabled:
            case K::set_magnus_enabled:
            case K::set_force_generator_enabled:
            case K::set_environment_parameter:
            case K::set_restitution_mixing:
            case K::set_continuous_collision:
            case K::set_energy_reference_height:
            case K::revert_change:
                return ui::EditCategory::parameter;
            case K::delete_selected_body:
            case K::commit_shape_outline:
            case K::import_shape:
            case K::add_object:
                return ui::EditCategory::structure;
            case K::assemble_selected_bodies:
            case K::split_selected_body:
                return ui::EditCategory::structure_pose;
            default:
                return ui::EditCategory::state;
            }
        }
    }

    SessionEditState SimulationSession::capture_edit_state() const
    {
        SessionEditState state;
        state.world = world_.snapshot();
        state.setup = setup_;
        state.original = original_;
        state.lab = lab_;
        state.default_lab = default_lab_;
        state.stepper = stepper_;
        state.runs = run_recorder_;
        state.shape_editor = shape_editor_;
        state.selected_bodies = selected_bodies_;
        state.selection = selection();
        state.edited_shape_body = edited_shape_body_;
        state.authored_part_index = authored_part_index_;
        state.edited_part_index = edited_part_index_;
        state.new_shape_material_index = new_shape_material_index_;
        state.shape_material = shape_material_;
        state.shape_edit_target = shape_edit_target_;
        state.pause_reason = pause_reason_;
        state.next_impact_armed = next_impact_armed_;
        state.pending_prediction = pending_prediction_;
        state.selected_connection = selected_connection_;
        state.scenario_id = scenario_id_;
        state.scenario_document = scenario_document_;
        state.gravity_direction_degrees = gravity_direction_degrees_;
        state.state_setup_toast_shown = state_setup_toast_shown_;
        return state;
    }

    void SimulationSession::restore_edit_state(const SessionEditState& state, std::optional<ui::EditCategory> category)
    {
        restoring_edit_ = true;
        const auto runtime_parallel = world_.parallel_settings();
        const auto runtime_profile = world_.profiling_enabled();
        world_.restore(state.world);
        world_.set_parallel_settings(runtime_parallel);
        world_.set_profiling_enabled(runtime_profile);
        setup_ = state.setup;
        original_ = state.original;
        lab_ = state.lab;
        default_lab_ = state.default_lab;
        stepper_ = state.stepper;
        run_recorder_ = state.runs;
        shape_editor_ = state.shape_editor;
        selected_bodies_ = state.selected_bodies;
        edited_shape_body_ = state.edited_shape_body;
        authored_part_index_ = state.authored_part_index;
        edited_part_index_ = state.edited_part_index;
        new_shape_material_index_ = state.new_shape_material_index;
        shape_material_ = state.shape_material;
        shape_edit_target_ = state.shape_edit_target;
        pause_reason_ = state.pause_reason;
        next_impact_armed_ = state.next_impact_armed;
        pending_prediction_ = state.pending_prediction;
        selected_connection_ = state.selected_connection;
        scenario_id_ = state.scenario_id;
        scenario_document_ = state.scenario_document;
        gravity_direction_degrees_ = state.gravity_direction_degrees;
        state_setup_toast_shown_ = state.state_setup_toast_shown;
        scene_settings_.selection = state.selection;
        // An undo result must remain inspectable and must not inherit a captured pointer.
        interaction_.active = false;
        dragging_view_ = false;
        shape_editor_.release_pointer();
        if (category && (*category == ui::EditCategory::state || *category == ui::EditCategory::structure || *category == ui::EditCategory::structure_pose))
        {
            stepper_.set_paused(true);
            pause_reason_ = { ui::PauseReason::after_undo, 0 };
        }
        single_step_pending_ = false;
        stepper_.cancel_single_step_request();
        refresh_force_generators();
        rebuild_snapshot_views();
        synchronize_render_history();
        if (!category || (*category != ui::EditCategory::parameter && *category != ui::EditCategory::lab))
            scene_renderer_.clear_trajectories();
        notifier_.clear_source("shape");
        restoring_edit_ = false;
    }

    void SimulationSession::restore_property_state(const SessionEditState& state, ui::EditCategory category)
    {
        // Parameter and lab undo is deliberately not a snapshot restore. A snapshot contains
        // transient pose, velocity, contact and clock state; restoring it would make a harmless
        // mass or gravity undo jump backwards in time. Instead, read the durable properties from
        // the target snapshot and transplant only those into the live world.
        restoring_edit_ = true;
        single_step_pending_ = false;
        stepper_.cancel_single_step_request();
        if (category == ui::EditCategory::lab)
        {
            lab_ = state.lab;
            apply_lab_settings(lab_);
            restoring_edit_ = false;
            return;
        }

        physics::World source;
        source.restore(state.world);

        for (const auto id : world_.body_ids())
        {
            auto* destination = world_.find_body(id);
            const auto* original = source.find_body(id);
            if (!destination || !original)
                continue;
            destination->clear_colliders();
            for (const auto& collider : original->colliders())
                destination->add_collider(collider);
            if (original->has_mass_override())
                destination->override_mass(original->mass_properties().mass_kg);
            destination->set_gravity_scale(original->gravity_scale());
            destination->set_linear_damping(original->linear_damping());
            destination->set_angular_damping(original->angular_damping());
            destination->set_fixed_rotation(original->has_fixed_rotation());
            (void)world_.notify_body_properties_changed(id);
        }

        auto settings = source.settings();
        // Lab settings are session-wide and are independent of an environment-property undo.
        settings.solver.warm_starting = lab_.warm_starting;
        settings.constraint_graph_enabled = lab_.constraint_graph;
        world_.set_settings(settings);
        world_.set_potential_energy_reference_height(source.potential_energy_reference_height_m());
        gravity_direction_degrees_ = state.gravity_direction_degrees;

        for (const auto& constraint : world_.constraints())
        {
            const auto key = world_.constraint_key(constraint);
            const auto source_constraint = source.constraint_by_key(key);
            const auto destination_joint = std::dynamic_pointer_cast<physics::JointConstraint>(constraint);
            const auto source_joint = std::dynamic_pointer_cast<physics::JointConstraint>(source_constraint);
            if (destination_joint && source_joint)
                destination_joint->set_definition(source_joint->definition());
        }
        for (const auto spring : world_.spring_ids())
            if (const auto* definition = source.spring_definition(spring))
                (void)world_.set_spring(spring, *definition);

        const auto generator_enabled = [](const physics::World& value, std::string_view name)
        {
            return std::any_of(value.force_generators().begin(), value.force_generators().end(), [&](const auto& generator)
                {
                    return generator && generator->name() == name && generator->is_enabled();
                });
        };
        set_gravity_enabled(generator_enabled(source, "uniform_gravity"));
        set_drag_enabled(generator_enabled(source, "aerodynamic_drag"));
        if (drag_)
        {
            const auto source_drag = std::find_if(source.force_generators().begin(), source.force_generators().end(), [](const auto& generator)
                {
                    return generator && generator->name() == "aerodynamic_drag";
                });
            const auto* from = source_drag == source.force_generators().end() ? nullptr : dynamic_cast<const physics::AerodynamicDrag*>(source_drag->get());
            auto* to = dynamic_cast<physics::AerodynamicDrag*>(drag_.get());
            if (from && to)
                to->set_settings(from->settings());
        }

        setup_ = state.setup;
        apply_lab_settings(lab_);
        rebuild_snapshot_views();
        synchronize_render_history();
        restoring_edit_ = false;
    }

    void SimulationSession::begin_edit(std::string label)
    {
        if (restoring_edit_ || pending_edit_)
            return;
        pending_edit_ = capture_edit_state();
        pending_edit_label_ = std::move(label);
        pending_edit_command_.reset();
        edit_changed_ = false;
    }
    bool SimulationSession::edit_in_progress() const
    {
        return pending_edit_.has_value();
    }
    void SimulationSession::mark_edit_changed()
    {
        if (pending_edit_)
        {
            // A scenario edit can teleport a body or replace its physical meaning. Start its
            // display history afresh on the first successful change, preserving invalid edits
            // and allowing subsequent substeps of a continuous pull to retain their own trail.
            if (!edit_changed_ && !applying_preview_)
                scene_renderer_.clear_trajectories();
            edit_changed_ = true;
        }
    }
    void SimulationSession::commit_edit(const ui::UiCommand* command, bool allow_coalescing)
    {
        if (!pending_edit_)
            return;
        const bool draft_changed = !same_outline(pending_edit_->shape_editor, shape_editor_) ||
            pending_edit_->shape_material.name != shape_material_.name;
        if (edit_changed_ || draft_changed)
        {
            SessionEdit next;
            next.before = std::move(*pending_edit_);
            next.after = capture_edit_state();
            next.label = std::move(pending_edit_label_);
            // A starting value changed mid-run leaves the run alone, so undoing it must not rewind
            // the run as a motion undo would, and the run's graph gets no marker for it.
            const auto starting_value = command && starting_motion_edit(*command);
            next.category = starting_value ? ui::EditCategory::parameter : command ? edit_category(command->kind)
                                                                                   : ui::EditCategory::state;
            next.coalescing_command = command ? *command : ui::UiCommand {};
            next.committed_wall_time_s = wall_time_s_;
            if (command && !starting_value && run_recorder_.current() && next.category != ui::EditCategory::lab)
            {
                auto marker = next.label;
                if (command->kind == ui::UiCommandKind::set_selected_mass)
                {
                    physics::World before;
                    before.restore(next.before.world);
                    const auto id = command->body.is_valid() ? command->body : next.before.selection;
                    const auto* old_body = before.find_body(id);
                    const auto* new_body = world_.find_body(id);
                    if (old_body && new_body)
                        marker = core::substitute("Mass {} → {} kg", core::fixed(old_body->mass_properties().mass_kg, 2), core::fixed(new_body->mass_properties().mass_kg, 2));
                }
                run_recorder_.add_marker(world_.statistics().elapsed_time_s, std::move(marker));
                if (next.category != ui::EditCategory::parameter)
                    run_recorder_.mark_changed_during_run();
            }
            if (allow_coalescing && has_wall_time_ && command && redo_edits_.empty() && !undo_edits_.empty() &&
                same_control(undo_edits_.back().coalescing_command, *command) &&
                wall_time_s_ >= undo_edits_.back().committed_wall_time_s &&
                wall_time_s_ - undo_edits_.back().committed_wall_time_s <= 1.0)
            {
                undo_edits_.back().after = std::move(next.after);
                undo_edits_.back().label = std::move(next.label);
                undo_edits_.back().committed_wall_time_s = wall_time_s_;
            }
            else
            {
                undo_edits_.push_back(std::move(next));
                if (undo_edits_.size() > maximum_session_edits)
                    undo_edits_.erase(undo_edits_.begin());
            }
            redo_edits_.clear();
        }
        pending_edit_.reset();
        pending_edit_command_.reset();
        edit_changed_ = false;
    }
    void SimulationSession::cancel_edit()
    {
        if (!pending_edit_)
            return;
        auto state = std::move(*pending_edit_);
        pending_edit_.reset();
        pending_edit_command_.reset();
        restore_edit_state(state);
        edit_changed_ = false;
    }

    bool SimulationSession::load_scenario(std::string_view id)
    {
        if (!physics::find_scenario(id))
            return false;
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::load_scenario;
        command.id = std::string { id };
        apply(command);
        return true;
    }
    void SimulationSession::reset_scenario()
    {
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::reset_scenario;
        apply(command);
    }

    void SimulationSession::apply(const ui::UiCommand& command)
    {
        command_target_body_ = command.body.is_valid() ? command.body : selection();
        using K = ui::UiCommandKind;
        const auto changed = [&]()
        {
            if (command.kind == K::load_scenario)
            {
                if (command.detail == "cancel")
                {
                    pending_leave_scenario_.clear();
                    return true;
                }
                if (!command.flag && (shape_editor_.active() || user_object_count() > 0))
                {
                    pending_leave_scenario_ = command.id;
                    return true;
                }
                pending_leave_scenario_.clear();
            }
            if (command.kind == K::quit)
            {
                if (command.detail == "cancel")
                {
                    pending_quit_confirmation_ = false;
                    return true;
                }
                if (!command.flag && (shape_editor_.active() || has_setup_changes()))
                {
                    pending_quit_confirmation_ = true;
                    return true;
                }
                pending_quit_confirmation_ = false;
            }
            if (command.kind == K::undo || command.kind == K::redo)
            {
                if (interaction_.active)
                {
                    cancel_interaction();
                    return true;
                }
                if (pending_edit_)
                {
                    shape_editor_.release_pointer();
                    commit_edit(pending_edit_command_ ? &*pending_edit_command_ : nullptr, false);
                    held_reason_.clear();
                }
                auto& source = command.kind == K::undo ? undo_edits_ : redo_edits_;
                auto& destination = command.kind == K::undo ? redo_edits_ : undo_edits_;
                if (source.empty())
                    return false;
                auto entry = std::move(source.back());
                source.pop_back();
                const auto& target = command.kind == K::undo ? entry.before : entry.after;
                if (entry.category == ui::EditCategory::parameter || entry.category == ui::EditCategory::lab)
                    restore_property_state(target, entry.category);
                else
                    restore_edit_state(target, entry.category);
                if (run_recorder_.current() && entry.category != ui::EditCategory::parameter && entry.category != ui::EditCategory::lab)
                {
                    run_recorder_.cut_at(world_.statistics().elapsed_time_s);
                    run_recorder_.mark_changed_during_run();
                }
                destination.push_back(std::move(entry));
                return true;
            }
            if (command.phase == ui::UiEditPhase::cancel)
            {
                if (pending_edit_ && pending_edit_command_ && same_control(*pending_edit_command_, command))
                {
                    cancel_edit();
                    held_reason_.clear();
                    return true;
                }
                return false;
            }
            auto label = edit_label(command.kind);
            const auto* target_body = world_.find_body(command_target_body_);
            const auto target_name = target_body && !target_body->name().empty() ? target_body->name() : std::string { "object" };
            switch (command.kind)
            {
            case K::set_selected_mass:
                label = core::substitute("Set mass of {} to {} kg", target_name, core::fixed(command.value, 2));
                break;
            case K::set_selected_material:
                label = core::substitute("Set material of {} to {}", target_name, command.id);
                break;
            case K::set_selected_gravity_scale:
                label = core::substitute("Set gravity scale of {} to {}", target_name, core::fixed(command.value, 2));
                break;
            case K::set_selected_velocity_x:
                label = core::substitute("Set velocity x of {} to {} m/s", target_name, core::fixed(command.value, 2));
                break;
            case K::set_selected_velocity_y:
                label = core::substitute("Set velocity y of {} to {} m/s", target_name, core::fixed(command.value, 2));
                break;
            case K::set_selected_angular_velocity:
                label = core::substitute("Set spin of {} to {} rad/s", target_name, core::fixed(command.value, 2));
                break;
            case K::set_gravity_magnitude:
                label = core::substitute("Set gravity strength to {} m/s²", core::fixed(command.value, 2));
                break;
            case K::set_time_scale:
                label.clear();
                break;
            default:
                break;
            }
            if (command.phase == ui::UiEditPhase::preview && !label.empty())
            {
                if (pending_edit_ && (!pending_edit_command_ || !same_control(*pending_edit_command_, command)))
                    commit_edit(pending_edit_command_ ? &*pending_edit_command_ : nullptr, false);
                if (!pending_edit_)
                {
                    begin_edit(label);
                    pending_edit_command_ = command;
                }
                applying_preview_ = true;
                try
                {
                    if (!apply_developer_command(command))
                        apply_untracked(command);
                }
                catch (...)
                {
                    applying_preview_ = false;
                    cancel_edit();
                    held_reason_.clear();
                    stepper_.set_paused(true);
                    pause_reason_ = { ui::PauseReason::error, 0 };
                    notify(ui::Severity::error, "The change could not be applied. Open Details to see why.", "edit-error");
                    throw;
                }
                applying_preview_ = false;
                if (edit_changed_)
                    held_reason_ = "adjusting";
                return command.kind != K::none;
            }
            if (pending_edit_ && pending_edit_command_ && same_control(*pending_edit_command_, command) && command.phase == ui::UiEditPhase::commit)
            {
                applying_preview_ = false;
                if (!apply_developer_command(command))
                    apply_untracked(command);
                commit_edit(&command, false);
                held_reason_.clear();
                return true;
            }
            if (pending_edit_ && !interaction_.active)
            {
                shape_editor_.release_pointer();
                commit_edit(pending_edit_command_ ? &*pending_edit_command_ : nullptr, false);
                held_reason_.clear();
            }
            if (apply_interaction_command(command))
                return true;
            if (command.kind == K::set_theme)
            {
                if (command.id != "workbench_dark" && command.id != "workbench_light" && command.id != "workbench_projector")
                    return false;
                const auto changed = command.id != theme_id_;
                theme_id_ = command.id;
                scene_settings_.theme = render::theme_by_name(command.id.c_str());
                return changed;
            }
            if (command.kind == K::set_ui_scale)
            {
                if (!std::isfinite(command.value))
                    return false;
                const auto next = std::clamp(command.value, 0.75, 2.0);
                const auto result = next != ui_scale_;
                ui_scale_ = next;
                return result;
            }
            if (command.kind == K::set_camera_zoom_sensitivity)
            {
                if (!std::isfinite(command.value))
                    return false;
                const auto next = std::clamp(command.value, 1.01, 1.5);
                const auto result = next != camera_zoom_sensitivity_;
                camera_zoom_sensitivity_ = next;
                return result;
            }
            if (command.kind == K::set_view_height)
            {
                const auto before = camera_.view_height_m();
                apply_untracked(command);
                return camera_.view_height_m() != before;
            }
            if (!label.empty() && interaction_.active)
                cancel_interaction();
            const auto own_transaction = !label.empty() && !pending_edit_;
            if (own_transaction)
                begin_edit(label);
            try
            {
                if (!apply_developer_command(command))
                    apply_untracked(command);
                if (own_transaction)
                    commit_edit(&command, command_can_coalesce(command.kind));
            }
            catch (...)
            {
                if (own_transaction)
                    cancel_edit();
                stepper_.set_paused(true);
                pause_reason_ = { ui::PauseReason::error, 0 };
                notify(ui::Severity::error, "The change could not be applied. Open Details to see why.", "edit-error");
                throw;
            }
            return command.kind != K::none;
        }();
        if (changed)
            ++change_serial_;
    }

    const std::vector<physics::BodyId>& SimulationSession::selections() const
    {
        return selected_bodies_;
    }
    void SimulationSession::set_selections(const std::vector<physics::BodyId>& ids)
    {
        if (shape_editor_.active())
            return;
        std::vector<physics::BodyId> valid;
        for (const auto id : ids)
            if (world_.is_valid(id) && !is_marker(id) && std::find(valid.begin(), valid.end(), id) == valid.end())
                valid.push_back(id);
        set_selection(valid.empty() ? physics::BodyId {} : valid.back());
        selected_bodies_ = std::move(valid);
    }

    bool SimulationSession::handle_shape_event(const ui::UiEvent& event, bool consumed)
    {
        const bool starting = !pending_edit_ && !consumed &&
            ((event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::primary) ||
                event.kind == ui::UiEventKind::key_down);
        if (starting)
            begin_edit("Edit outline");
        const auto handled = shape_editor_.handle_event(event, camera_, shape_snap_vertices(), consumed);
        if (pending_edit_ && (!shape_editor_.has_pointer_capture() || event.kind == ui::UiEventKind::pointer_up))
            commit_edit();
        return handled;
    }
}
