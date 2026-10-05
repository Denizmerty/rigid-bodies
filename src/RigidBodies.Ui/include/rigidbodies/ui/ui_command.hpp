#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <string>
#include <vector>
#include <cstdint>

namespace rigidbodies::ui
{

    // Actions the interface can request from the application.
    //
    // Panels emit these commands for the application to handle. Add a case and a handler when
    // introducing a new action.
    enum class UiCommandKind
    {
        none,
        undo,
        redo,
        select_all,
        set_interaction_mode,
        set_ui_scale,
        toggle_developer_overlay,
        set_physics_profiling,
        set_physics_workers,
        set_solver_parameter,
        set_force_generator_enabled,
        set_environment_parameter,
        set_camera_zoom_sensitivity,
        set_visual_effect,
        set_visual_budget,
        export_still,
        toggle_frame_capture,
        toggle_pause,
        single_step,
        reset_scenario,
        load_scenario,
        save_arrangement,
        open_arrangement,
        export_shape,
        import_shape,
        set_time_scale,
        set_fixed_step,
        compare_integrators,
        compare_restitution,
        set_continuous_collision,
        set_warm_starting,
        set_constraint_graph,
        set_layer_mask,
        select_body,
        clear_selection,
        set_vector_auto_scale,
        set_vector_scale,
        set_component_angle_degrees,
        clear_energy_history,
        set_pause_on_impact,
        compare_collisions,
        set_selected_mass,
        use_selected_density_mass,
        set_selected_material,
        set_selected_velocity_x,
        set_selected_velocity_y,
        set_selected_angular_velocity,
        set_joint_motor_enabled,
        set_joint_limits_enabled,
        reverse_joint_motor,
        start_new_shape,
        edit_selected_shape,
        split_selected_body,
        close_shape_outline,
        commit_shape_outline,
        cancel_shape_outline,
        insert_shape_node,
        remove_shape_node,
        set_shape_snap_grid,
        set_shape_snap_vertices,
        set_shape_snap_angles,
        set_shape_grid_spacing,
        set_shape_vertex_budget,
        set_shape_render_tolerance,
        set_shape_collision_tolerance,
        set_shape_simplification_tolerance,
        set_shape_concavity_tolerance,
        delete_selected_body,
        set_gravity_enabled,
        set_gravity_magnitude,
        set_gravity_angle_degrees,
        set_gravity_preset,
        set_selected_gravity_scale,
        set_drag_enabled,
        set_angular_drag_enabled,
        set_magnus_enabled,
        frame_all,
        frame_selection,
        set_view_height,
        quit,
        frame_subject,

        // Direct setters are appended to preserve the values of existing commands.
        set_integrator,
        set_restitution_mixing,
        set_vector_components,
        set_display_units,
        set_theme,
        set_layer,
        set_shape_edge,
        set_shape_continuity,
        set_shape_material,
        select_impact,
        select_authored_part,

        // Shell commands.
        step_many,
        set_vector_scope,
        open_captures_folder,
        set_preference,

        // Setup-layer commands.
        restore_original,
        keep_state_as_setup,
        revert_change,
        revert_lab_settings,

        // Inspector and connections.
        select_bodies,
        select_connection,
        assemble_selected_bodies,
        set_selected_position,
        set_selected_orientation,
        stop_selected_motion,
        set_joint_motor_speed,
        set_spring_parameter,
        set_energy_reference_height,

        // Measure and Guide.
        pause_at_next_impact,
        set_prediction,

        // Runs and Compare.
        star_run,
        clear_runs,
        pin_run_value,
        unpin_run_value,
        set_selected_velocity,
        add_object,

        // The window's own title bar; its close button sends quit, which confirms unsaved changes.
        minimize_window,
        toggle_maximize_window,

        set_shape_node_position,

        // Special relativity: value is the probe's speed fraction v/c, or ±1 with detail "nudge" (one
        // rapidity step) or "preset" (the next or previous speed on the ladder).
        set_relativity_speed
    };

    enum class UiEditPhase : std::uint8_t
    {
        commit,
        preview,
        cancel
    };

    struct UiCommand
    {
        UiCommandKind kind { UiCommandKind::none };

        // Identifier the command refers to, such as a scenario or a visualisation layer.
        std::string id;

        // Numeric payload, such as a time scale or a view height.
        double value { 0.0 };

        double value_y { 0.0 };

        bool flag { false };

        physics::BodyId body;

        std::vector<physics::BodyId> bodies;
        std::string detail;
        UiEditPhase phase { UiEditPhase::commit };
    };

} // namespace rigidbodies::ui
