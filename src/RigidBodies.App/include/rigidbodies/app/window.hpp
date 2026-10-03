#pragma once

#include <rigidbodies/core/application_config.hpp>
#include <rigidbodies/render/camera2d.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>

struct SDL_Window;

namespace rigidbodies::app
{

    // The platform window and the platform subsystem behind it.
    //
    // Window creation and the platform lifetime are kept together because they share it: the
    // subsystem has to be running before a window can exist and has to outlive it. Keeping both in
    // one class is what makes the ordering impossible to get wrong from outside.
    class Window
    {
    public:
        // Starts the platform subsystems the application needs. Returns false when they are
        // unavailable, which is the normal outcome on a machine with no display.
        [[nodiscard]] static bool initialize_platform();
        static void shutdown_platform();

        // A window whose title bar the interface draws is created without the platform's and
        // stays hidden until show(), so no frameless placeholder appears while the interface loads.
        [[nodiscard]] static std::unique_ptr<Window> create(const core::WindowConfig& config, bool open_gl = false, bool hidden = false,
            bool custom_title_bar = false);

        ~Window();

        Window(const Window&) = delete;
        Window(Window&&) = delete;
        Window& operator=(const Window&) = delete;
        Window& operator=(Window&&) = delete;

        [[nodiscard]] SDL_Window* native() const;

        // Size of the drawable surface in pixels, which differs from the window size on a display
        // with a scale factor applied.
        [[nodiscard]] render::ViewportSize drawable_size() const;

        [[nodiscard]] float display_scale() const;

        void set_title(std::string_view title);

        void show();
        void hide();
        // Gives the window back the platform's title bar, or takes it away.
        void set_bordered(bool bordered);

        [[nodiscard]] bool maximized() const;
        [[nodiscard]] bool minimized() const;
        [[nodiscard]] bool resizable() const;
        [[nodiscard]] bool active() const;
        // The display the window is on, and the part of it the window may use, in the window's
        // own coordinates; nothing when the platform cannot say.
        [[nodiscard]] std::uint32_t display_id() const;
        [[nodiscard]] std::optional<render::ViewportSize> usable_display_size() const;
        void minimize();
        void toggle_maximized();
        // Brings a minimized or covered window forward, so a question it asks can be answered.
        void restore_and_raise();
        [[nodiscard]] std::uint32_t id() const;

        // Milliseconds elapsed since the platform started, read from the platform timer rather
        // than from a wall clock so that it cannot move backwards.
        [[nodiscard]] static double elapsed_seconds();

    private:
        explicit Window(SDL_Window* window);

        SDL_Window* window_ { nullptr };
    };

} // namespace rigidbodies::app
