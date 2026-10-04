#pragma once

#include <rigidbodies/app/input.hpp>
#include <rigidbodies/app/input_router.hpp>
#include <rigidbodies/app/frame_capture.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/window.hpp>
#include <rigidbodies/app/window_chrome.hpp>
#include <rigidbodies/core/resource_paths.hpp>
#include <rigidbodies/render/render_device.hpp>
#include <rigidbodies/ui/ui_context.hpp>

#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
#include <rigidbodies/app/developer_overlay.hpp>
#endif

#include <filesystem>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct SDL_Cursor;
union SDL_Event;

namespace rigidbodies::app
{
    struct ContentFileDialog;

    // A hidden, deterministic render of the whole window to a PNG, for visual review. It reads only
    // the shipped configuration, never reads or writes user preferences, and advances the
    // simulation in fixed frames so the same arguments produce the same image.
    struct ScreenshotOptions
    {
        std::filesystem::path output;
        int width { 1600 }, height { 900 };
        // Overrides the monitor's pixel density, so 100-200 % displays can be reviewed on any
        // machine. Zero keeps the monitor's own scale.
        float display_scale { 0.0f };
        // The text-size preference; zero keeps the configured value.
        double ui_scale { 0.0 };
        std::string theme, scenario;
        int run_frames { 0 };
        // Interface states to open before capturing, such as guide, measure, menu or present.
        std::vector<std::string> states;
    };

    // The lifetime of the program.
    //
    // Startup is ordered so that each stage only needs what the stages before it produced:
    // configuration, then the platform, then the window, then the device, then the interface, then
    // the session. Shutdown runs the same order in reverse. Every stage can fail and says so, and
    // a failure at any point leaves the stages already started to be closed down normally.
    class Application
    {
    public:
        Application();
        ~Application();

        Application(const Application&) = delete;
        Application(Application&&) = delete;
        Application& operator=(const Application&) = delete;
        Application& operator=(Application&&) = delete;

        // Reads the configuration and brings up everything the run loop needs. The path is the
        // directory holding the executable, which is what the resource locator searches from.
        [[nodiscard]] bool initialize(const std::filesystem::path& executable_directory, bool hidden_smoke = false,
            bool render_benchmark = false, bool wide_benchmark = false, const ScreenshotOptions* screenshot = nullptr);

        // Runs until the window is closed. Returns the process exit status.
        [[nodiscard]] int run();

        void shutdown();

    private:
        void process_events();
        void handle_action(AppAction action);
        void apply_command(const ui::UiCommand& command);
        void apply_pending_commands();
        bool begin_content_dialog(const ui::UiCommand& command);
        void finish_content_dialog();
        void update(double frame_time_s);
        void present();
        void sync_viewport();
        void sync_text_input_and_cursor();
        void persist_preferences();
        [[nodiscard]] std::string preference_signature() const;
        [[nodiscard]] float display_scale() const;
        void render_frame(double frame_time_s);
        void apply_screenshot_states();
        [[nodiscard]] int run_screenshot();

        // The window's frame while the interface draws its title bar.
        void install_window_chrome();
        void update_window_frame(const ui::UiModel& model);
        void apply_window_control_clicks();
        void request_close();
        // The platform's move, size and menu loops hold the run loop inside event polling. A frame
        // drawn from an event watch on each of their live exposures keeps the window, title bar
        // and all, drawn at its live size.
        static bool watch_events(void* application, SDL_Event* event);
        void draw_live_frame();

        core::ApplicationConfig config_;
        core::ResourcePaths paths_;

        std::unique_ptr<Window> window_;
        std::unique_ptr<WindowChrome> window_chrome_;
        ui::WindowFrameState window_frame_;
        // A window whose title bar the interface draws waits for its first frame to be shown.
        bool show_window_pending_ { false };
        bool watching_events_ { false };
        bool drawing_live_frame_ { false };
        // The title bar went back to the platform because a high-contrast theme came on; it is
        // taken over again once the theme goes off.
        bool title_bar_yielded_to_contrast_ { false };
        double next_contrast_check_s_ { 0.0 };
        // The interface scale and display the window's minimum size was last set for.
        float minimum_size_scale_ { 0.0f };
        std::uint32_t minimum_size_display_ { 0 };
        render::RenderDevicePtr device_;
        ui::UiContext interface_;
        SimulationSession session_;
        InputTranslator input_;
        FrameCapture frame_capture_;
        std::shared_ptr<ContentFileDialog> content_dialog_;
        std::optional<SetupDeparture> save_continuation_;
        std::filesystem::path last_content_folder_;
        math::Vec2 input_pixel_scale_ { 1.0, 1.0 };

#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        std::unique_ptr<DeveloperOverlay> developer_overlay_;
        FramePhaseTimings frame_timings_, previous_frame_timings_;
#endif
        double previous_interface_s_ {};
        double previous_simulation_s_ {}, previous_scene_s_ {}, previous_submit_s_ {}, previous_present_s_ {};

        // One list for the scene and one for the interface, submitted in that order so that the
        // panels are drawn over the simulation. Keeping them separate lets the scene be rebuilt
        // without touching the interface and the other way round.
        render::DrawList scene_list_;
        render::DrawList interface_list_;

        bool running_ { false };
        bool hidden_smoke_ { false };
        bool render_benchmark_ { false };
        std::optional<ScreenshotOptions> screenshot_;
        float display_scale_override_ { 0.0f };
        double last_frame_time_s_ { 0.0 };
        // When the frame being drawn began; frames drawn during a platform loop share the clock.
        double frame_start_s_ { 0.0 };
        bool text_input_active_ { false };
        ui::CursorShape cursor_shape_ { ui::CursorShape::arrow };
        SDL_Cursor* cursor_ { nullptr };
        std::array<SDL_Cursor*, 11> cursor_cache_ {};
        std::string last_capture_status_;
        std::string observed_preference_signature_;
        double preference_save_due_s_ { -1.0 };
    };

} // namespace rigidbodies::app
