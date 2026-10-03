#pragma once

#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/education.hpp>
#include <rigidbodies/ui/education_model.hpp>
#include <rigidbodies/physics/energy_drift.hpp>
#include <rigidbodies/physics/restitution_comparison.hpp>
#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/render/visualization_layers.hpp>
#include <rigidbodies/ui/notifications.hpp>
#include <rigidbodies/ui/experiment_content.hpp>
#include <rigidbodies/ui/run_types.hpp>
#include <rigidbodies/ui/ui_command.hpp>

#include <string>
#include <string_view>
#include <optional>
#include <cstdint>

namespace rigidbodies::ui
{
    enum class RunState : std::uint8_t
    {
        ready,
        running,
        paused,
        held
    };
    enum class KeyCategory : std::uint8_t
    {
        playback,
        tools,
        view,
        surfaces,
        editing,
        draw,
        files,
        help
    };
    enum class KeyContext : std::uint8_t
    {
        global,
        scene,
        draw,
        keyboard_control,
        list,
        present,
        developer
    };

    struct KeyReference
    {
        std::string chord, description;
        KeyCategory category { KeyCategory::help };
        KeyContext context { KeyContext::global };
    };

    struct ExperimentCard
    {
        std::string id, title, summary, collection, level, hook;
        int collection_order { 0 }, suggested_order { 0 };
        std::vector<std::string> concepts, builds_on, tags;
        bool lab { false };
    };

    struct LabChange
    {
        std::string key, label, default_text, current_text;
    };

    struct SetupFileInfo
    {
        std::uint64_t serial {};
        std::string title, based_on, date, path;
    };

    struct ConfirmationModel
    {
        std::string title, text, confirm_label;
        UiCommand confirm, cancel, save;
    };

    enum class StageTargetKind : std::uint8_t
    {
        none,
        object,
        connection,
        impact,
        empty_space,
        handle
    };

    struct HoverModel
    {
        StageTargetKind kind { StageTargetKind::none };
        physics::BodyId body;
        std::string id, detail;
        render::ScreenRect screen_bounds;
        std::vector<std::string> card_lines;
        // The other objects on the stage, which the card leaves visible where it can.
        std::vector<render::ScreenRect> avoid;
        bool silhouette { false };
    };

    struct StageHandle
    {
        std::string id;
        StageTargetKind kind { StageTargetKind::handle };
        render::ScreenRect screen_bounds;
        physics::BodyId body;
        bool enabled { true };
        std::string disabled_reason;
    };

    struct ContextMenuRequest
    {
        std::uint64_t serial {};
        math::Vec2 screen_position_px {};
        StageTargetKind kind { StageTargetKind::empty_space };
        physics::BodyId body;
        std::string id, detail;
    };

    struct RevealRequest
    {
        std::uint64_t serial {};
        std::string key, instance;
    };

    struct PerformanceModel
    {
        double frame_time_s {};
        double frames_per_second {};
        int substeps { 1 };
        std::string renderer, interface_backend;
        math::Vec2 pointer_world_m {};
        double view_height_m { 4.0 };
    };
    // Read-only simulation view supplied to panels each frame.
    //
    // Panels request changes through commands that the application validates and applies. They
    // cannot modify the world directly.
    struct UiModel
    {
        const physics::World* world { nullptr };
        std::uint64_t change_serial { 0 };

        std::vector<physics::BodyId> selected_bodies;
        std::string undo_label, redo_label;
        std::string interaction_mode { "select" };
        double ui_scale { 1.0 }, camera_zoom_sensitivity { 1.12 };
        std::vector<KeyReference> keyboard_reference;
        std::vector<ExperimentCard> catalogue;
        std::vector<LabChange> lab_changes;
        std::vector<SetupChange> changes;
        std::vector<UndoHistoryItem> undo_history;
        std::vector<ObjectItem> objects;
        std::vector<ImpactItem> impacts;
        const RunRecord* current_run { nullptr };
        const RunRecord* previous_run { nullptr };
        math::Span<const RunRecord> runs;
        math::Span<const PinnedValue> pinned_values;
        std::size_t starred_run_count {};
        std::vector<InterventionMarker> graph_markers;
        std::optional<Prediction> prediction;
        std::optional<ExperimentContent> scenario_content;
        std::optional<SelectedConnection> selected_connection;
        std::optional<ConfirmationModel> confirmation;
        std::optional<HoverModel> hover;
        std::optional<ContextMenuRequest> context_menu_request;
        std::optional<RevealRequest> reveal_request;
        std::vector<StageHandle> handles;
        PerformanceModel performance;
        std::optional<SetupFileInfo> last_setup_file;
        DraftModel draft;
        PauseState pause_reason;
        std::size_t user_object_count {};
        std::size_t shape_node_count { 0 };
        std::size_t shape_selected_node { 0 };
        std::size_t shape_render_vertex_count { 0 };
        std::size_t shape_collision_vertex_count { 0 };
        std::size_t shape_convex_part_count { 0 };
        std::size_t shape_vertex_budget { 128 };
        std::string shape_continuity;
        std::string shape_diagnostic;
        std::string shape_edit_target;
        std::string shape_material_name;
        double shape_grid_spacing_m { 0.1 };
        double shape_render_tolerance_m { 0.002 };
        double shape_collision_tolerance_m { 0.01 };
        double shape_simplification_tolerance_m { 0.002 };
        double shape_concavity_tolerance_m { 0.0 };
        std::size_t authored_part_index { 0 };
        std::size_t authored_part_count { 0 };
        std::string authored_part_name;
        std::string authored_material_name;
        double gravity_direction_degrees { -90.0 };
        std::optional<double> terminal_speed_m_s;
        std::string terminal_speed_note;
        math::Vec2 air_velocity_m_s {};
        double air_dynamic_viscosity_pa_s { 1.81e-5 };
        physics::EnergyDriftSettings energy_comparison_settings;
        std::vector<physics::EnergyDriftReport> energy_comparison;
        physics::RestitutionComparisonSettings restitution_comparison_settings;
        std::vector<physics::RestitutionComparisonReport> restitution_comparison;
        physics::CollisionComparisonSettings collision_comparison_settings;
        std::vector<physics::CollisionComparisonReport> collision_comparison;
        std::vector<InlineNotice> inline_notices;
        std::vector<Notification> notifications;
        std::optional<Banner> banner;
        std::vector<StatusChip> status_chips;
        std::string tool_hint;
        std::optional<double> oscillation_period_s;
        std::string oscillation_period_note;
        std::optional<double> critical_ramp_angle_rad;
        std::string ramp_angle_note;
        double component_angle_rad { 0.0 };

        double time_scale { 1.0 };
        double fixed_step_s { 1.0 / 120.0 };
        double discarded_time_s { 0.0 };
        double elapsed_time_s { 0.0 };
        RunState run_state { RunState::ready };

        // Measured frame interval, for the performance read-out.
        double frame_time_s { 0.0 };

        std::string scenario_id;
        std::string scenario_title;
        std::string scenario_summary;
        std::vector<std::string> scenario_concepts, scenario_prerequisites;

        render::VectorScales vector_scales;
        // The force factor the stage actually drew with (pixels per newton), 0 before a frame.
        double drawn_force_scale { 0.0 };

        // Pointer position in world coordinates, shown as a read-out and used by the panels that
        // describe what is under the cursor.
        math::Vec2 pointer_world_m {};

        double view_height_m { 4.0 };
        std::string render_backend;
        std::string interface_backend;

        // Group the small flags and counters to avoid padding in this per-frame view.
        physics::BodyId selection;
        render::SceneRenderSettings visual_settings;
        core::DisplayUnits display_units { core::DisplayUnits::si };
        render::VectorComponents vector_components { render::VectorComponents::none };
        int substeps { 1 };
        int scenario_suggested_order { 0 };
        render::LayerMask layers;
        // The layers the experiment opens with; the Show presets offer them as Recommended.
        render::LayerMask recommended_layers { render::LayerMask::defaults() };
        bool can_undo { false }, can_redo { false };
        std::string theme_id { "workbench_dark" };
        std::string held_reason;
        bool capturing_frames { false };
        std::size_t capture_frame_count { 0 }, capture_frame_limit { 120 };
        bool shape_editor_active { false };
        bool shape_outline_closed { false };
        bool shape_can_commit { false };
        bool shape_node_selected { false };
        bool shape_selected_edge_cubic { false };
        bool shape_can_change_material { true };
        bool shape_snap_grid { true };
        bool shape_snap_vertices { true };
        bool shape_snap_angles { false };
        bool selected_shape_authored { false };
        bool can_assemble_shapes { false };
        bool can_split_shape { false };
        bool drag_enabled { false };
        bool angular_drag_enabled { true };
        bool magnus_enabled { true };
        bool pause_on_impact { false };
        bool next_impact_armed { false };
        bool paused { false };
        bool can_export_shape { false };
        bool reduce_motion { false }, pause_in_background { false };
        bool keep_lab_settings { true }, open_experiments_running { false };
        bool recommended_view { true }, ask_predictions { true };
        bool follow_selection { false };
        std::string experiments_on_start { "last" }, effects_quality { "standard" }, capture_area { "stage" };
        const physics::World* setup_world { nullptr };
        const physics::World* original_world { nullptr };

        [[nodiscard]] const InlineNotice* inline_notice(std::string_view key) const
        {
            for (const auto& notice : inline_notices)
                if (notice.key == key)
                    return &notice;
            return nullptr;
        }
    };

} // namespace rigidbodies::ui
