#pragma once

#include <rigidbodies/ui/ui_backend.hpp>

#include <functional>
#include <memory>

struct SDL_Window;

namespace rigidbodies::app
{

    // The window's frame while the interface draws its title bar.
    //
    // The platform still moves, resizes, snaps, maximizes and minimizes the window, opens its
    // window menu, and offers its snap layouts on the maximize button; the interface only says
    // what each point of the window is. The window controls are the platform's caption buttons in
    // all but appearance, so the first click on an inactive window works them, as it does native
    // ones. Windows is the platform this exists for; elsewhere the window keeps its own title bar.
    class WindowChrome
    {
    public:
        // What a point, in the window's client pixels, is part of.
        using PartAt = std::function<ui::WindowPart(const ui::Vec2&)>;

        // Whether the interface may draw this platform's title bar: not under a high-contrast
        // theme, whose system colours a drawn title bar would not follow.
        [[nodiscard]] static bool supported();

        // Takes over the frame of a borderless window. Returns nothing when that is not possible,
        // in which case the window should be given the platform's title bar back.
        [[nodiscard]] static std::unique_ptr<WindowChrome> install(SDL_Window* window, const PartAt& part_at);

        ~WindowChrome();

        WindowChrome(const WindowChrome&) = delete;
        WindowChrome(WindowChrome&&) = delete;
        WindowChrome& operator=(const WindowChrome&) = delete;
        WindowChrome& operator=(WindowChrome&&) = delete;

        // The window control under the pointer and the one held pressed. The platform, not the
        // interface, receives the pointer over them.
        [[nodiscard]] ui::WindowPart hot() const;
        [[nodiscard]] ui::WindowPart pressed() const;

        // The window control clicked since the previous call, or client when none was.
        [[nodiscard]] ui::WindowPart take_click();

        // Matches the platform's border around the window to a dark or a light interface.
        void set_dark(bool dark);

        // The smallest client area, in pixels, the window may be resized to. The frame keeps the
        // platform's resizing borders outside the window, where SDL does not expect them, so it
        // sizes the window itself.
        void set_minimum_client_size(int width, int height);

    private:
        struct Impl;
        explicit WindowChrome(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> impl_;
    };

} // namespace rigidbodies::app
