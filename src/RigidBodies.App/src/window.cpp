#include <rigidbodies/app/window.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/project_identity.hpp>

#include <SDL3/SDL.h>

#include <algorithm>

namespace rigidbodies::app
{

    bool Window::initialize_platform()
    {
        // Video brings in the event loop and the timer along with the display, which is everything
        // the application needs. Audio and the game controller subsystems are left out because
        // nothing here uses them and starting them would only add ways to fail.
        if (!SDL_Init(SDL_INIT_VIDEO))
        {
            core::log_error("the platform video subsystem could not be started: {}", SDL_GetError());
            return false;
        }

        SDL_SetAppMetadata(rigidbodies::display_name.data(), rigidbodies::project_version.data(), "org.rigidbodies.playground");
        // Closing the window asks the application to quit, so unsaved changes are confirmed
        // first, whichever way the window was closed.
        SDL_SetHint(SDL_HINT_QUIT_ON_LAST_WINDOW_CLOSE, "0");
        return true;
    }

    void Window::shutdown_platform()
    {
        SDL_Quit();
    }

    std::unique_ptr<Window> Window::create(const core::WindowConfig& config, bool open_gl, bool hidden, bool custom_title_bar)
    {
        SDL_WindowFlags flags = SDL_WINDOW_HIGH_PIXEL_DENSITY;
        if (hidden || custom_title_bar)
            flags |= SDL_WINDOW_HIDDEN;
        if (custom_title_bar)
        {
            // A borderless window keeps the platform's caption, system menu and resizing styles
            // under these hints, so it still snaps, maximizes, animates and resizes like any other.
            SDL_SetHint("SDL_BORDERLESS_WINDOWED_STYLE", "1");
            SDL_SetHint("SDL_BORDERLESS_RESIZABLE_STYLE", "1");
            flags |= SDL_WINDOW_BORDERLESS;
        }
        if (open_gl)
        {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
            SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
            // MSAA lives in a render target, so unavailable window pixel formats never prevent
            // startup. The device negotiates its sample count and reports any downgrade.
            SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
            SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
            flags |= SDL_WINDOW_OPENGL;
        }
        if (config.resizable)
        {
            flags |= SDL_WINDOW_RESIZABLE;
        }
        if (config.start_maximised)
        {
            flags |= SDL_WINDOW_MAXIMIZED;
        }

        auto* handle = SDL_CreateWindow(config.title.c_str(), std::max(config.width, 320), std::max(config.height, 240), flags);
        if (handle == nullptr)
        {
            core::log_error("the window could not be created: {}", SDL_GetError());
            return nullptr;
        }

        core::log_info("window {} at {} by {}", hidden ? "created hidden" : "opened", config.width, config.height);
        return std::unique_ptr<Window> { new Window { handle } };
    }

    Window::Window(SDL_Window* window) : window_(window)
    {
    }

    Window::~Window()
    {
        if (window_ != nullptr)
        {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
        }
    }

    SDL_Window* Window::native() const
    {
        return window_;
    }

    render::ViewportSize Window::drawable_size() const
    {
        int width = 0;
        int height = 0;
        SDL_GetWindowSizeInPixels(window_, &width, &height);
        return { std::max(width, 1), std::max(height, 1) };
    }

    float Window::display_scale() const
    {
        const auto scale = SDL_GetWindowDisplayScale(window_);
        return scale > 0.0f ? scale : 1.0f;
    }

    void Window::set_title(std::string_view title)
    {
        // The platform expects a terminated string, and a view carries no guarantee of one.
        const std::string terminated { title };
        SDL_SetWindowTitle(window_, terminated.c_str());
    }

    void Window::show()
    {
        SDL_ShowWindow(window_);
    }

    void Window::hide()
    {
        SDL_HideWindow(window_);
    }

    void Window::set_bordered(bool bordered)
    {
        SDL_SetWindowBordered(window_, bordered);
    }

    bool Window::maximized() const
    {
        return (SDL_GetWindowFlags(window_) & SDL_WINDOW_MAXIMIZED) != 0;
    }

    bool Window::minimized() const
    {
        return (SDL_GetWindowFlags(window_) & SDL_WINDOW_MINIMIZED) != 0;
    }

    bool Window::resizable() const
    {
        return (SDL_GetWindowFlags(window_) & SDL_WINDOW_RESIZABLE) != 0;
    }

    bool Window::active() const
    {
        return (SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) != 0;
    }

    std::uint32_t Window::display_id() const
    {
        return SDL_GetDisplayForWindow(window_);
    }

    std::optional<render::ViewportSize> Window::usable_display_size() const
    {
        SDL_Rect usable {};
        if (!SDL_GetDisplayUsableBounds(SDL_GetDisplayForWindow(window_), &usable) || usable.w <= 0 || usable.h <= 0)
            return std::nullopt;
        return render::ViewportSize { usable.w, usable.h };
    }

    void Window::minimize()
    {
        SDL_MinimizeWindow(window_);
    }

    void Window::toggle_maximized()
    {
        if (maximized())
            SDL_RestoreWindow(window_);
        else
            SDL_MaximizeWindow(window_);
    }

    void Window::restore_and_raise()
    {
        if (minimized())
            SDL_RestoreWindow(window_);
        SDL_RaiseWindow(window_);
    }

    std::uint32_t Window::id() const
    {
        return SDL_GetWindowID(window_);
    }

    double Window::elapsed_seconds()
    {
        return static_cast<double>(SDL_GetTicksNS()) * 1.0e-9;
    }

} // namespace rigidbodies::app
