#pragma once

#include <rigidbodies/app/shape_editor.hpp>
#include <rigidbodies/app/notifier.hpp>
#include <rigidbodies/app/edit_history.hpp>
#include <rigidbodies/app/interaction_state.hpp>
#include <rigidbodies/app/experiment_content.hpp>
#include <rigidbodies/app/run_recorder.hpp>
#include <rigidbodies/core/application_config.hpp>
#include <rigidbodies/physics/time_stepper.hpp>
#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/ui/ui_command.hpp>
#include <rigidbodies/ui/ui_model.hpp>
#include <rigidbodies/ui/ui_backend.hpp>

#include <string>
#include <optional>
#include <cstdint>
#include <utility>

namespace rigidbodies::app
{
    struct SetupSaveSnapshot
    {
        std::shared_ptr<SetupFileAssociation> document;
        SetupFileAssociation file;
        std::string text;
        std::uint64_t serial {};
    };

    struct PendingSetupOpen
    {
        std::string text, path, title;
    };

    struct SetupDeparture
    {
        ui::UiCommand command;
        std::shared_ptr<const PendingSetupOpen> opening;
        std::shared_ptr<SetupFileAssociation> document;
        std::uint64_t serial {};
    };

    // Everything that makes up one session with the playground: the world being simulated, the
    // pacing, the view onto it, and what is currently selected.
    //
    // The session sits between the interface and the simulation. It is the only place that turns
    // an interface command into a change, and the only place that reads the simulation to build
    // the view the interface sees. Keeping both directions here is what allows the interface and
    // the simulation to remain unaware of each other.
    // State remains grouped by subsystem ownership instead of interleaving unrelated fields to
    // minimise padding in this orchestration object.
    // NOLINTNEXTLINE(clang-analyzer-optin.performance.Padding)
    class SimulationSession
    {
    public:
        void configure(const core::ApplicationConfig& config);

        [[nodiscard]] physics::World& world();
        [[nodiscard]] const physics::World& world() const;

        [[nodiscard]] render::Camera2D& camera();
        [[nodiscard]] const render::Camera2D& camera() const;

        [[nodiscard]] render::SceneRenderSettings& scene_settings();
        [[nodiscard]] const render::SceneRenderSettings& scene_settings() const;

        [[nodiscard]] physics::TimeStepper& stepper();

        bool load_scenario(std::string_view id);

        // Document operations are also available without a window or filesystem. Failed reads
        // leave the entire session intact; successful imports are one undoable edit.
        bool save_arrangement(std::string& text, std::string& error) const;
        bool save_arrangement(std::string& text, std::string& error, std::string_view title,
            bool current_moment, bool include_guide) const;
        bool open_arrangement(std::string_view text, std::string& error, std::string path = {});
        // User-facing Open validates the file before protecting unsaved edits with a prompt.
        bool request_open_arrangement(std::string text, std::string& error, std::string path = {});
        bool save_current_arrangement(std::string& text, std::string& error) const;
        [[nodiscard]] std::optional<SetupSaveSnapshot> capture_setup_save(std::string& error,
            std::string_view title, bool current_moment, bool include_guide) const;
        [[nodiscard]] std::optional<SetupSaveSnapshot> capture_current_setup_save(std::string& error) const;
        [[nodiscard]] bool can_write_setup_save(const SetupSaveSnapshot& snapshot, std::string_view path, std::string& error) const;
        // Records only the document and editable state represented by the bytes that were written.
        // Returns true when that saved document is still current and has no newer setup edits.
        bool complete_setup_save(const SetupSaveSnapshot& snapshot, std::string path);
        void note_setup_file(std::string path, std::string title, std::string based_on,
            bool current_moment = false, bool include_guide = true);
        // A confirmation's Save resumes its departure only after a successful write.
        [[nodiscard]] std::optional<SetupDeparture> take_pending_departure_for_save();
        void complete_saved_departure(const SetupDeparture& departure);
        [[nodiscard]] const std::string& current_setup_path() const
        {
            return setup_file_->path;
        }
        [[nodiscard]] const SetupFileAssociation& current_setup_file() const
        {
            return *setup_file_;
        }
        bool export_shape(std::string& text, std::string& error) const;
        bool import_shape(std::string_view text, std::string& error);
        void set_content_message(std::string message);
        void notify(ui::Severity severity, std::string message, std::string source = {},
            std::optional<ui::NotificationAction> action = {}, bool persistent = false);

        // Restores the captured starting world, including its environment and force settings,
        // while keeping explicitly selected integration, collision and environmental options.
        void reset_scenario();

        // Runs however many fixed steps the elapsed frame time has earned.
        void advance(double frame_time_s);

        void render(render::DrawList& list);

        void apply(const ui::UiCommand& command);

        // The same routing entry point serves SDL input and window-free interaction tests.
        // Panel consumption blocks scene presses, while captured drags always get their release.
        bool handle_scene_event(const ui::UiEvent& event, bool interface_consumed = false);
        [[nodiscard]] const ShapeEditor& shape_editor() const;

        [[nodiscard]] ui::UiModel build_model() const;

        void set_viewport(const render::ViewportSize& viewport);
        void set_focus_rect(const render::ScreenRect& rect);
        // The part of the stage the reader can see, between the toolbar and the status line or a
        // sheet; the view height readout describes it, and stage arrows and plates may use all of
        // it. Without one the focus area stands in.
        void set_visible_stage_rect(const render::ScreenRect& rect)
        {
            visible_stage_rect_ = rect;
            scene_renderer_.set_visible_stage(rect);
        }

        // Camera interaction, expressed in the pixels the platform reports.
        void pan_view(const math::Vec2& screen_delta_px);
        void zoom_view(double wheel_delta, const math::Vec2& anchor_px);
        void frame_everything();
        void frame_selection();
        [[nodiscard]] math::Aabb subject_bounds() const;
        void frame_subject();
        [[nodiscard]] bool camera_user_moved() const
        {
            return camera_user_moved_;
        }

        // Selects the topmost object under a point on screen, or clears the selection when the
        // point is empty.
        void select_at(const math::Vec2& screen_point_px);

        [[nodiscard]] physics::BodyId selection() const;
        void set_selection(physics::BodyId id);
        [[nodiscard]] const std::vector<physics::BodyId>& selections() const;
        void set_selections(const std::vector<physics::BodyId>& ids);
        [[nodiscard]] InteractionMode interaction_mode() const;
        [[nodiscard]] bool interaction_active() const;
        [[nodiscard]] ui::CursorShape scene_cursor() const;
        void set_present_spotlight(bool enabled)
        {
            present_spotlight_ = enabled;
        }
        // Present mode enlarges stage text, value chips and handles for a room.
        void set_presenting(bool presenting)
        {
            presenting_ = presenting;
        }
        // Whether the interface currently has room for the hover card, which stands in for the
        // hovered body's stage label while it is shown.
        void set_hover_cards_allowed(bool allowed)
        {
            hover_cards_allowed_ = allowed;
        }
        // Where the interface placed the hover card last frame; stage labels keep clear of it.
        void set_hover_card_area(const std::optional<render::ScreenRect>& area)
        {
            hover_card_area_ = area;
        }
        // Metres of world visible from top to bottom of the visible stage, as the status line
        // reports it.
        [[nodiscard]] double stage_view_height_m() const;
        void cancel_gesture();
        void deselect_draft_node();
        void request_context_menu_for_selection();

        [[nodiscard]] math::Vec2 pointer_world_position(const math::Vec2& screen_point_px) const;

        void set_pointer_position(const math::Vec2& screen_point_px);
        bool pick_surface_at(const math::Vec2& screen_point_px);

        [[nodiscard]] bool should_quit() const;

        void set_frame_time(double seconds);
        void set_wall_time(double seconds);
        void set_window_backgrounded(bool backgrounded);

        [[nodiscard]] const std::string& scenario_id() const;

    private:
        [[nodiscard]] std::string setup_fingerprint() const;
        void publish_setup_file(const SetupFileAssociation& file);
        void apply_untracked(const ui::UiCommand& command);
        bool load_scenario_untracked(std::string_view id);
        void reset_scenario_untracked();
        [[nodiscard]] SessionEditState capture_edit_state() const;
        void restore_edit_state(const SessionEditState& state, std::optional<ui::EditCategory> category = {});
        void restore_property_state(const SessionEditState& state, ui::EditCategory category);
        void begin_edit(std::string label);
        void mark_edit_changed();
        void commit_edit(const ui::UiCommand* command = nullptr, bool allow_coalescing = false);
        void cancel_edit();
        [[nodiscard]] bool edit_in_progress() const;
        bool handle_interaction_event(const ui::UiEvent&, bool);
        bool apply_interaction_command(const ui::UiCommand&);
        void apply_interaction_forces();
        void cancel_interaction();
        void render_interaction(render::DrawList&) const;

        // Stage overlays follow the monitor's pixel density as well as the text-size preference,
        // matching the logical pixel scale that the pointer hit tests already use. Present mode
        // enlarges both.
        [[nodiscard]] double overlay_scale() const;
        // Device pixels per metre per second of the selection's velocity arrow, as last drawn.
        [[nodiscard]] double velocity_handle_scale() const;
        [[nodiscard]] double stage_text_scale() const;
        // The body whose hover card is shown, if any; its stage label steps aside meanwhile.
        [[nodiscard]] physics::BodyId hover_card_body() const;
        // The screen box a hover card stands beside: the hovered body or connection.
        [[nodiscard]] std::optional<render::ScreenRect> hover_target_bounds() const;
        // Only a single selected free body gets the velocity and rotation handles.
        [[nodiscard]] const physics::RigidBody* handle_body() const;
        [[nodiscard]] double handle_ring_radius(const physics::RigidBody& body) const;
        void reserve_overlay_areas();
        [[nodiscard]] math::Aabb predicted_motion_bounds(const math::Aabb& current_subject) const;
        [[nodiscard]] std::optional<math::Aabb> authored_view_bounds() const;
        void apply_reduce_motion();
        // Names the Effects quality preset the current switches and limits match, or "custom".
        void refresh_effects_quality();
        // A Guide or Present field for an experiment's starting motion edits the setup only, so a
        // run in progress keeps going and the next run starts with the new value. Before the run
        // starts the setup and the live world are one state and the edit applies to both.
        [[nodiscard]] bool starting_motion_edit(const ui::UiCommand& command) const;
        void apply_starting_motion_edit(const ui::UiCommand& command);
        bool apply_developer_command(const ui::UiCommand&);
        bool handle_shape_event(const ui::UiEvent&, bool interface_consumed);
        void set_gravity_enabled(bool enabled);
        void set_drag_enabled(bool enabled);
        void refresh_force_generators();
        void synchronize_render_history();
        bool apply_shape_command(const ui::UiCommand& command);
        void begin_shape_editor(bool edit_selected);
        void finish_shape_editor();
        void commit_shape_editor();
        void populate_shape_model(ui::UiModel& model) const;
        // The view an experiment opens with: the configured layers and the overlays it teaches with.
        [[nodiscard]] render::LayerMask recommended_layers() const;
        [[nodiscard]] std::vector<math::Vec2> shape_snap_vertices() const;
        bool apply_education_command(const ui::UiCommand& command);
        void reset_measurements();
        bool collect_impacts();
        void populate_education_model(ui::UiModel& model) const;
        [[nodiscard]] LabSettings capture_lab_settings() const;
        void apply_lab_settings(const LabSettings& settings);
        void sync_setup_after_command(const ui::UiCommand& command);
        void merge_live_structure_into_setup();
        void rebuild_snapshot_views();
        void close_current_run(bool make_previous);
        void begin_run_if_needed();
        void populate_run_model(ui::UiModel& model) const;
        [[nodiscard]] bool is_marker(physics::BodyId id) const;
        // One name per body in world order, shared by the stage labels and every panel.
        [[nodiscard]] std::vector<std::pair<physics::BodyId, std::string>> object_names() const;
        [[nodiscard]] std::size_t user_object_count() const;
        [[nodiscard]] bool has_setup_changes() const;
        [[nodiscard]] physics::BodyId pick_body_at(const math::Vec2& screen_point_px, double logical_scale = 1.0) const;
        [[nodiscard]] std::optional<ui::SelectedConnection> pick_connection_at(const math::Vec2& screen_point_px, double logical_scale = 1.0) const;
        // The two world points the stage draws a joint or spring between.
        [[nodiscard]] std::optional<std::pair<math::Vec2, math::Vec2>> connection_anchors_m(std::string_view key, std::string_view kind) const;
        [[nodiscard]] std::string handle_at(const math::Vec2& screen_point_px, double logical_scale = 1.0) const;
        bool handle_stage_handle_event(const ui::UiEvent& event, bool interface_consumed);

        physics::World world_;
        std::vector<physics::BodyId> selected_bodies_;
        InteractionState interaction_;
        HandleDragState handle_drag_;
        std::vector<SessionEdit> undo_edits_, redo_edits_;
        std::optional<SessionEditState> pending_edit_;
        std::string pending_edit_label_;
        std::optional<ui::UiCommand> pending_edit_command_;
        bool edit_changed_ { false }, restoring_edit_ { false }, applying_preview_ { false };
        double wall_time_s_ { 0.0 };
        bool has_wall_time_ { false };
        std::string held_reason_;
        std::uint64_t change_serial_ { 0 };
        std::string theme_id_ { "workbench_dark" };
        double ui_scale_ { 1.0 }, camera_zoom_sensitivity_ { 1.12 };
        bool start_paused_ { true };
        bool reduce_motion_ { false }, pause_in_background_ { false }, keep_lab_settings_ { true };
        bool recommended_view_ { true }, ask_predictions_ { true };
        // The layers the configuration asks for before any experiment adds its own overlays.
        render::LayerMask base_layers_ { render::LayerMask::defaults() };
        bool follow_selection_ { false };
        bool present_spotlight_ { false };
        bool presenting_ { false };
        bool hover_cards_allowed_ { true };
        std::optional<render::ScreenRect> hover_card_area_;
        std::optional<render::ScreenRect> hover_card_target_;
        bool paused_by_background_ { false };
        std::string experiments_on_start_ { "last" }, effects_quality_ { "standard" }, capture_area_ { "stage" };
        SetupSnapshot setup_, original_;
        physics::World setup_view_, original_view_;
        LabSettings lab_, default_lab_;
        RunRecorder run_recorder_;
        std::optional<ui::ExperimentContent> experiment_content_;
        std::optional<ui::Prediction> pending_prediction_;
        std::optional<ui::SelectedConnection> selected_connection_;
        ui::PauseState pause_reason_;
        bool next_impact_armed_ { false };
        bool updating_setup_ { false }, state_setup_toast_shown_ { false };
        physics::BodyId command_target_body_;
        physics::TimeStepper stepper_;
        render::Camera2D camera_;
        render::ScreenRect visible_stage_rect_;
        bool camera_user_moved_ { false };
        render::SceneRenderer scene_renderer_;
        render::SceneRenderSettings scene_settings_;
        std::vector<std::pair<math::Vec2, math::Vec2>> overlay_areas_;
        // The look-ahead behind subject framing, reused while the world it was run from is
        // unchanged so that resizing or opening a panel does not simulate again.
        struct MotionPrediction
        {
            std::uint64_t fingerprint {};
            math::Aabb bounds;
            // Fixed bodies the subject rests on or strikes during the look-ahead.
            std::vector<physics::BodyId> touched_fixtures;
            bool valid { false };
        };
        mutable MotionPrediction motion_prediction_;
        ShapeEditor shape_editor_;
        physics::BodyId edited_shape_body_;
        std::size_t authored_part_index_ { 0 };
        std::size_t edited_part_index_ { 0 };
        std::size_t new_shape_material_index_ { 0 };
        physics::Material shape_material_ { physics::materials::oak_wood() };
        std::string shape_edit_target_;
        bool dragging_view_ { false };
        bool secondary_pan_pending_ { false };
        math::Vec2 secondary_pan_start_px_ {};
        std::optional<ui::ContextMenuRequest> context_menu_request_;
        // Where on the stage the context menu was asked for. The menu is placed from it every
        // frame, so it stays beside its object when the stage is reframed under the open menu.
        math::Vec2 context_menu_anchor_m_ {};
        std::uint64_t context_menu_serial_ {};
        std::optional<ui::RevealRequest> reveal_request_;
        std::uint64_t reveal_request_serial_ {};

        physics::ForceGeneratorPtr gravity_;
        physics::ForceGeneratorPtr drag_;

        std::string scenario_id_;
        std::shared_ptr<SetupFileAssociation> setup_file_ { std::make_shared<SetupFileAssociation>() };
        std::optional<ui::SetupFileInfo> last_setup_file_;
        std::uint64_t setup_file_serial_ {};
        std::shared_ptr<const physics::ScenarioDocument> scenario_document_;
        Notifier notifier_;
        double default_view_height_m_ { 4.0 };
        double frame_time_s_ { 0.0 };
        math::Vec2 pointer_world_m_ {};
        std::string render_backend_name_;
        std::string interface_backend_name_;
        bool quit_requested_ { false };
        std::string pending_leave_scenario_;
        bool pending_quit_confirmation_ { false };
        std::shared_ptr<const PendingSetupOpen> pending_setup_open_;
        std::uint64_t departure_serial_ {};
        bool single_step_pending_ { false };
        // Playback speed previews remain live, but cancellation restores their starting value
        // without making a physics edit or holding the simulation clock.
        std::optional<double> speed_preview_start_;
        double gravity_direction_degrees_ { -90.0 };
        physics::EnergyDriftSettings energy_comparison_settings_;
        std::vector<physics::EnergyDriftReport> energy_comparison_;
        std::string energy_comparison_error_;
        physics::RestitutionComparisonSettings restitution_comparison_settings_;
        std::vector<physics::RestitutionComparisonReport> restitution_comparison_;
        std::string restitution_comparison_error_;
        bool pause_on_impact_ { false };
        std::vector<physics::CollisionImpactReport> inspected_impacts_;
        std::size_t impact_report_index_ { 0 };
        physics::CollisionComparisonSettings collision_comparison_settings_;
        std::vector<physics::CollisionComparisonReport> collision_comparison_;
        std::string collision_comparison_error_;

        friend class Application;
    };

} // namespace rigidbodies::app
