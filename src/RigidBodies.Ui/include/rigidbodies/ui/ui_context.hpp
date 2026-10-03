#pragma once

#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/ui_backend.hpp>

#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rigidbodies::ui
{

    // Owns the interface backend, panels and pending commands.
    //
    // The application supplies a simulation view and input, requests a frame, then drains the
    // commands. This class handles panel and backend details.
    class UiContext
    {
    public:
        UiContext();
        ~UiContext();

        UiContext(const UiContext&) = delete;
        UiContext(UiContext&&) = delete;
        UiContext& operator=(const UiContext&) = delete;
        UiContext& operator=(UiContext&&) = delete;

        // Starts the requested backend, falling back to the overlay when it is unavailable. The
        // backend actually in use is reported by backend_name.
        [[nodiscard]] bool initialize(UiBackendKind requested, const std::filesystem::path& asset_root);
        void shutdown();

        [[nodiscard]] std::string_view backend_name() const;
        // Read-only access for integration tests and diagnostics that inspect the live document.
        [[nodiscard]] const UiBackend* backend() const;

        [[nodiscard]] const Theme& theme() const;
        void set_theme(const Theme& value);

        [[nodiscard]] float scale() const;
        void set_scale(float value);

        // The window around the interface, reported by the application each frame. Panels never
        // see it; it decides only whether the top strip carries the window's own title bar.
        void set_window_frame(const WindowFrameState& value);
        // Whether the backend in use can draw the window's title bar controls.
        [[nodiscard]] bool draws_window_controls() const;
        // What the point (in pixels) is part of while the interface draws the title bar.
        [[nodiscard]] WindowPart window_part(const Vec2& point) const;

        void add_panel(std::unique_ptr<Panel> panel);
        [[nodiscard]] Panel* find_panel(std::string_view id);
        [[nodiscard]] const std::vector<std::unique_ptr<Panel>>& panels() const;

        // Returns true when the interface consumed the event.
        bool handle_event(const UiEvent& event);
        // Called when a press goes to the scene instead of the interface.
        void release_focus();

        void build(const UiModel& model, const render::RenderDevice& device, DrawList& list, double wall_time_s = 0.0);

        // Hands over the commands collected since the previous call and clears the queue.
        [[nodiscard]] std::vector<UiCommand> take_commands();

        [[nodiscard]] bool surface_at(const Vec2& point) const;
        // A press outside an open menu or popover closes it and is not forwarded to the stage,
        // the way desktop menus behave. Returns true when the press was used to dismiss.
        [[nodiscard]] bool menu_open() const;
        bool dismiss_menus_at(const Vec2& point);
        void close_menus();
        [[nodiscard]] const LayoutResult& layout() const;
        [[nodiscard]] FocusOwner focus_owner() const;
        [[nodiscard]] EscapeTarget escape_target() const;
        [[nodiscard]] bool wants_text_input() const;
        [[nodiscard]] Rect text_input_area() const;
        // Where the hover card was placed in the latest build, if one is shown.
        [[nodiscard]] std::optional<Rect> hover_card_bounds() const;
        [[nodiscard]] CursorShape cursor() const;
        [[nodiscard]] bool pointer_captured() const;

        void set_clipboard_hooks(ClipboardReader reader, ClipboardWriter writer);

        [[nodiscard]] ViewState& view_state();
        [[nodiscard]] const ViewState& view_state() const;
        void request(ViewRequest request, std::string_view key = {});
        void reveal(std::string_view key, std::string_view instance = {});

    private:
        // Opens one menu, closing any other; requesting an open menu again closes it.
        void open_menu(std::string_view id);
        // Whether the window can show the Inspector only as a sheet over the stage: always when
        // narrow, and when too narrow to dock it even once the Guide yields.
        [[nodiscard]] bool inspector_as_sheet() const;

        UiBackendPtr backend_;
        std::vector<std::unique_ptr<Panel>> panels_;
        std::vector<UiCommand> commands_;
        Theme theme_;
        float scale_ { 1.0f };
        WindowFrameState window_frame_;
        bool pointer_over_interface_ { false };
        ViewportSize last_viewport_ {};
        ViewState view_state_;
        std::optional<std::uint64_t> hovered_notification_;
        std::uint64_t context_menu_serial_ {};
        // The selection that last opened the inspector; a hidden inspector stays hidden until the
        // learner selects something else.
        physics::BodyId inspected_selection_ {};
        // The experiment the side surfaces were last arranged for; opening another one hands the
        // shared dock back to its Guide.
        std::string arranged_experiment_;
        double caption_height_logical_ {};
        bool context_menu_requested_ { false };
        std::uint64_t reveal_request_serial_ {};
        // A reveal or field focus waits for the build that creates its row: the Inspector builds
        // only its current page, and a sheet's field exists only once the sheet is shown.
        std::optional<std::pair<std::string, std::string>> pending_reveal_;
        std::optional<std::string> pending_focus_;
        double wall_time_s_ { 0.0 };
        LayoutResult layout_;
        LayoutInput layout_input_;
        LayoutMode previous_layout_mode_ { LayoutMode::medium };
        ClipboardReader clipboard_reader_;
        ClipboardWriter clipboard_writer_;
    };

} // namespace rigidbodies::ui
