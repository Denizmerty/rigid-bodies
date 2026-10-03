#include <rigidbodies/app/window_chrome.hpp>

#include <rigidbodies/app/input_router.hpp>
#include <rigidbodies/core/log.hpp>

#include <SDL3/SDL.h>

#include <utility>

#if defined(_WIN32)
#if !defined(NOMINMAX)
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>
#include <windowsx.h>

#include <algorithm>
#include <cmath>
#endif

namespace rigidbodies::app
{

#if defined(_WIN32)

    namespace
    {
        constexpr wchar_t instance_property[] = L"RigidBodies.WindowChrome";

        // Desktop Window Manager attributes, named here so an older SDK still builds.
        constexpr DWORD dwm_immersive_dark_mode = 20;
        constexpr DWORD dwm_window_corner_preference = 33;
        constexpr DWORD dwm_corner_round = 2;

        // The resizing top edge's depth in logical pixels; the other edges are the platform's own
        // invisible borders outside the window.
        constexpr double top_edge_logical = 4.0;

        // The undocumented message the platform uses to open a window's menu at a point.
        constexpr UINT popup_window_menu = 0x0313;

        using SetWindowAttribute = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);

        // The window manager's library is loaded on first use, as SDL loads it, so neither build
        // has to link it.
        SetWindowAttribute set_window_attribute()
        {
            static const auto function = []() -> SetWindowAttribute
            {
                if (auto* module = LoadLibraryExW(L"dwmapi.dll", nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32))
                    return reinterpret_cast<SetWindowAttribute>(GetProcAddress(module, "DwmSetWindowAttribute"));
                return nullptr;
            }();
            return function;
        }

        bool is_window_control(WPARAM hit)
        {
            return hit == HTMINBUTTON || hit == HTMAXBUTTON || hit == HTCLOSE;
        }

        ui::WindowPart part_for(WPARAM hit)
        {
            switch (hit)
            {
            case HTMINBUTTON:
                return ui::WindowPart::minimize;
            case HTMAXBUTTON:
                return ui::WindowPart::maximize;
            case HTCLOSE:
                return ui::WindowPart::close;
            default:
                return ui::WindowPart::client;
            }
        }

        LRESULT hit_code(FrameHit hit)
        {
            switch (hit)
            {
            case FrameHit::title_bar:
                return HTCAPTION;
            case FrameHit::minimize:
                return HTMINBUTTON;
            case FrameHit::maximize:
                return HTMAXBUTTON;
            case FrameHit::close:
                return HTCLOSE;
            case FrameHit::top:
                return HTTOP;
            case FrameHit::top_left:
                return HTTOPLEFT;
            case FrameHit::top_right:
                return HTTOPRIGHT;
            case FrameHit::client:
                break;
            }
            return HTCLIENT;
        }
    }

    struct WindowChrome::Impl
    {
        HWND window { nullptr };
        SDL_Window* sdl_window { nullptr };
        WNDPROC platform_procedure { nullptr };
        PartAt part_at;
        ui::WindowPart hot { ui::WindowPart::client }, pressed { ui::WindowPart::client }, clicked { ui::WindowPart::client };
        bool tracking_leave { false };
        bool dark { true };
        SIZE minimum_client {};

        static LRESULT CALLBACK procedure(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
        {
            auto* self = static_cast<Impl*>(GetPropW(window, instance_property));
            return self ? self->handle(message, wparam, lparam) : DefWindowProcW(window, message, wparam, lparam);
        }

        LRESULT forward(UINT message, WPARAM wparam, LPARAM lparam) const
        {
            return CallWindowProcW(platform_procedure, window, message, wparam, lparam);
        }

        LRESULT handle(UINT message, WPARAM wparam, LPARAM lparam)
        {
            switch (message)
            {
            case WM_NCCALCSIZE:
                if (wparam == TRUE)
                {
                    // The platform passes its parameters through the integer argument.
                    // NOLINTNEXTLINE(performance-no-int-to-ptr)
                    return calculate_client_area(*reinterpret_cast<NCCALCSIZE_PARAMS*>(lparam), lparam);
                }
                break;
            case WM_GETMINMAXINFO:
            {
                // SDL limits the window's outer size; the minimum is the client area's.
                const auto result = forward(message, wparam, lparam);
                const auto insets = frame_insets();
                // NOLINTNEXTLINE(performance-no-int-to-ptr)
                auto& limits = *reinterpret_cast<MINMAXINFO*>(lparam);
                limits.ptMinTrackSize.x = std::max(limits.ptMinTrackSize.x, minimum_client.cx + insets.cx);
                limits.ptMinTrackSize.y = std::max(limits.ptMinTrackSize.y, minimum_client.cy + insets.cy);
                return result;
            }
            case WM_DPICHANGED:
            {
                // SDL gives a borderless window the client size it wants as its outer size; the
                // borders outside the window are added back at the new scale.
                const auto result = forward(message, wparam, lparam);
                if (!IsZoomed(window) && !IsIconic(window))
                    grow_by_frame();
                return result;
            }
            case WM_NCHITTEST:
                return hit_test(lparam);
            case WM_NCMOUSEMOVE:
                // Moving between non-client parts sends no leave, so every move sets the control
                // under the pointer. The platform still sees the move: over maximize, for its snap
                // layouts; over minimize and close as over the title bar, since there it would only
                // add its own tooltip to the interface's.
                hot = part_for(wparam);
                track_leave();
                if (wparam == HTMINBUTTON || wparam == HTCLOSE)
                    return forward(message, HTCAPTION, lparam);
                break;
            case WM_NCMOUSEHOVER:
                if (wparam == HTMINBUTTON || wparam == HTCLOSE)
                    return 0;
                break;
            case WM_NCMOUSELEAVE:
                tracking_leave = false;
                hot = pressed = ui::WindowPart::client;
                break;
            case WM_NCLBUTTONDOWN:
            case WM_NCLBUTTONDBLCLK:
                // The platform would draw its own caption button and track it; the interface
                // draws this one, so the press is only noted, and acts on release over it.
                if (is_window_control(wparam))
                {
                    hot = pressed = part_for(wparam);
                    return 0;
                }
                break;
            case WM_NCLBUTTONUP:
                if (is_window_control(wparam))
                {
                    if (pressed == part_for(wparam))
                        clicked = pressed;
                    pressed = ui::WindowPart::client;
                    return 0;
                }
                break;
            case WM_NCRBUTTONDOWN:
            case WM_NCRBUTTONDBLCLK:
                if (is_window_control(wparam) || wparam == HTCAPTION)
                    return 0;
                break;
            case WM_NCRBUTTONUP:
                // A right click anywhere on the title bar opens the window menu, as on a native one.
                if (is_window_control(wparam) || wparam == HTCAPTION)
                {
                    SendMessageW(window, popup_window_menu, 0, lparam);
                    return 0;
                }
                break;
            case WM_MOUSEMOVE:
                hot = pressed = ui::WindowPart::client;
                break;
            case WM_LBUTTONUP:
            case WM_CAPTURECHANGED:
            case WM_CANCELMODE:
            case WM_ENTERSIZEMOVE:
                pressed = ui::WindowPart::client;
                break;
            case WM_ACTIVATE:
                if (LOWORD(wparam) == WA_INACTIVE)
                    hot = pressed = ui::WindowPart::client;
                break;
            case WM_SYSCOMMAND:
                // Alt+Space opens the window menu, which SDL otherwise keeps from the platform
                // along with every other menu key.
                if ((wparam & 0xFFF0) == SC_KEYMENU && lparam == VK_SPACE)
                    return DefWindowProcW(window, message, wparam, lparam);
                break;
            case WM_SETTINGCHANGE:
            {
                // SDL matches the window to the system's colours when they change; the frame
                // follows the interface's theme instead.
                const auto result = forward(message, wparam, lparam);
                apply_dark();
                return result;
            }
            case WM_NCDESTROY:
            {
                const auto previous = platform_procedure;
                SetWindowLongPtrW(window, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(previous));
                RemovePropW(window, instance_property);
                const auto destroyed = window;
                window = nullptr;
                return CallWindowProcW(previous, destroyed, message, wparam, lparam);
            }
            default:
                break;
            }
            return forward(message, wparam, lparam);
        }

        LRESULT calculate_client_area(NCCALCSIZE_PARAMS& parameters, LPARAM lparam)
        {
            if (IsZoomed(window))
            {
                // SDL fits a maximized borderless window to the monitor's work area.
                const auto result = forward(WM_NCCALCSIZE, TRUE, lparam);
                leave_auto_hide_taskbar_reachable(parameters.rgrc[0]);
                return result;
            }
            // The platform's invisible borders stay outside the window on the left, right and
            // bottom, where the window resizes natively; its title bar gives way to the client area.
            const auto top = parameters.rgrc[0].top;
            const auto result = DefWindowProcW(window, WM_NCCALCSIZE, TRUE, lparam);
            parameters.rgrc[0].top = top;
            return result;
        }

        // A maximized window covering a monitor whose taskbar hides itself would keep the taskbar
        // hidden for good; stopping two pixels short of its edge lets the pointer reveal it.
        void leave_auto_hide_taskbar_reachable(RECT& client) const
        {
            MONITORINFO monitor {};
            monitor.cbSize = sizeof(monitor);
            if (!GetMonitorInfoW(MonitorFromWindow(window, MONITOR_DEFAULTTONEAREST), &monitor))
                return;
            const auto hides_on = [&](UINT edge)
            {
                APPBARDATA taskbar {};
                taskbar.cbSize = sizeof(taskbar);
                taskbar.uEdge = edge;
                taskbar.rc = monitor.rcMonitor;
                return SHAppBarMessage(ABM_GETAUTOHIDEBAREX, &taskbar) != 0;
            };
            if (hides_on(ABE_TOP))
                client.top += 2;
            if (hides_on(ABE_LEFT))
                client.left += 2;
            if (hides_on(ABE_BOTTOM))
                client.bottom -= 2;
            if (hides_on(ABE_RIGHT))
                client.right -= 2;
        }

        LRESULT hit_test(LPARAM lparam) const
        {
            POINT point { GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam) };
            ScreenToClient(window, &point);
            RECT client {};
            GetClientRect(window, &client);
            // Outside the client area are the platform's own borders.
            if (point.x < client.left || point.x >= client.right || point.y < client.top || point.y >= client.bottom)
                return DefWindowProcW(window, WM_NCHITTEST, 0, lparam);
            FrameGeometry frame;
            frame.width = static_cast<double>(client.right);
            frame.height = static_cast<double>(client.bottom);
            frame.edge = std::max(1.0, std::round(top_edge_logical * static_cast<double>(SDL_GetWindowDisplayScale(sdl_window))));
            frame.maximized = IsZoomed(window) != FALSE;
            frame.resizable = (GetWindowLongPtrW(window, GWL_STYLE) & WS_THICKFRAME) != 0;
            const ui::Vec2 at { static_cast<double>(point.x), static_cast<double>(point.y) };
            return hit_code(frame_hit(at, part_at(at), frame));
        }

        // How much larger the window is than its client area: the platform's borders outside it.
        SIZE frame_insets() const
        {
            RECT outer {}, client {};
            GetWindowRect(window, &outer);
            GetClientRect(window, &client);
            return { (outer.right - outer.left) - client.right, (outer.bottom - outer.top) - client.bottom };
        }

        // Makes room for the borders around a client area that SDL sized as the whole window.
        void grow_by_frame() const
        {
            RECT outer {};
            GetWindowRect(window, &outer);
            const auto insets = frame_insets();
            SetWindowPos(window, nullptr, 0, 0, (outer.right - outer.left) + insets.cx, (outer.bottom - outer.top) + insets.cy, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
        }

        void track_leave()
        {
            if (tracking_leave)
                return;
            TRACKMOUSEEVENT track {};
            track.cbSize = sizeof(track);
            track.dwFlags = TME_LEAVE | TME_NONCLIENT;
            track.hwndTrack = window;
            tracking_leave = TrackMouseEvent(&track) != FALSE;
        }

        void apply_dark() const
        {
            if (const auto set = set_window_attribute())
            {
                const BOOL value = dark ? TRUE : FALSE;
                set(window, dwm_immersive_dark_mode, &value, sizeof(value));
            }
        }
    };

    bool WindowChrome::supported()
    {
        HIGHCONTRASTW contrast {};
        contrast.cbSize = sizeof(contrast);
        return !(SystemParametersInfoW(SPI_GETHIGHCONTRAST, sizeof(contrast), &contrast, 0) && (contrast.dwFlags & HCF_HIGHCONTRASTON) != 0);
    }

    std::unique_ptr<WindowChrome> WindowChrome::install(SDL_Window* window, const PartAt& part_at)
    {
        if (!window || !part_at || !supported())
            return nullptr;
        auto* handle = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(window), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
        if (!handle)
            return nullptr;
        // Without these styles the platform could not move, snap or minimize a window it shows
        // no title bar for.
        constexpr LONG_PTR required = WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
        if ((GetWindowLongPtrW(handle, GWL_STYLE) & required) != required)
        {
            core::log_warning("the window lacks the platform's title bar styles; keeping the platform's title bar");
            return nullptr;
        }
        auto impl = std::make_unique<Impl>();
        impl->window = handle;
        impl->sdl_window = window;
        impl->part_at = part_at;
        if (!SetPropW(handle, instance_property, impl.get()))
            return nullptr;
        // The platform keeps window procedures as integers.
        // NOLINTNEXTLINE(performance-no-int-to-ptr)
        impl->platform_procedure = reinterpret_cast<WNDPROC>(SetWindowLongPtrW(handle, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(&Impl::procedure)));
        if (!impl->platform_procedure)
        {
            RemovePropW(handle, instance_property);
            return nullptr;
        }
        if (const auto set = set_window_attribute())
        {
            const DWORD corners = dwm_corner_round;
            set(handle, dwm_window_corner_preference, &corners, sizeof(corners));
        }
        impl->apply_dark();
        // The client area now reaches the window's top and its borders lie outside it; the
        // platform recomputes it at once, and the window grows by its borders so the client
        // area keeps the size SDL gave the whole window.
        SetWindowPos(handle, nullptr, 0, 0, 0, 0, SWP_FRAMECHANGED | SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
        if (!IsZoomed(handle) && !IsIconic(handle))
            impl->grow_by_frame();
        return std::unique_ptr<WindowChrome>(new WindowChrome(std::move(impl)));
    }

    WindowChrome::WindowChrome(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
    {
    }

    WindowChrome::~WindowChrome()
    {
        auto* handle = impl_->window;
        if (!handle)
            return;
        // The window goes back to SDL only if nothing has replaced this procedure since.
        // NOLINTNEXTLINE(performance-no-int-to-ptr)
        if (reinterpret_cast<WNDPROC>(GetWindowLongPtrW(handle, GWLP_WNDPROC)) == &Impl::procedure)
        {
            SetWindowLongPtrW(handle, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(impl_->platform_procedure));
            SetWindowPos(handle, nullptr, 0, 0, 0, 0, SWP_FRAMECHANGED | SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
        }
        RemovePropW(handle, instance_property);
    }

    ui::WindowPart WindowChrome::hot() const
    {
        return impl_->hot;
    }

    ui::WindowPart WindowChrome::pressed() const
    {
        return impl_->pressed;
    }

    ui::WindowPart WindowChrome::take_click()
    {
        return std::exchange(impl_->clicked, ui::WindowPart::client);
    }

    void WindowChrome::set_dark(bool dark)
    {
        if (impl_->dark == dark)
            return;
        impl_->dark = dark;
        if (impl_->window)
            impl_->apply_dark();
    }

    void WindowChrome::set_minimum_client_size(int width, int height)
    {
        auto* handle = impl_->window;
        impl_->minimum_client = { std::max(width, 1), std::max(height, 1) };
        if (!handle || IsZoomed(handle) || IsIconic(handle))
            return;
        RECT client {};
        GetClientRect(handle, &client);
        if (client.right >= impl_->minimum_client.cx && client.bottom >= impl_->minimum_client.cy)
            return;
        const auto insets = impl_->frame_insets();
        SetWindowPos(handle, nullptr, 0, 0, std::max(client.right, impl_->minimum_client.cx) + insets.cx, std::max(client.bottom, impl_->minimum_client.cy) + insets.cy, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
    }

#else

    struct WindowChrome::Impl
    {
    };

    bool WindowChrome::supported()
    {
        return false;
    }

    std::unique_ptr<WindowChrome> WindowChrome::install(SDL_Window*, const PartAt&)
    {
        return nullptr;
    }

    WindowChrome::WindowChrome(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
    {
    }

    WindowChrome::~WindowChrome() = default;

    ui::WindowPart WindowChrome::hot() const
    {
        return ui::WindowPart::client;
    }

    ui::WindowPart WindowChrome::pressed() const
    {
        return ui::WindowPart::client;
    }

    ui::WindowPart WindowChrome::take_click()
    {
        return ui::WindowPart::client;
    }

    void WindowChrome::set_dark(bool)
    {
    }

    void WindowChrome::set_minimum_client_size(int, int)
    {
    }

#endif

} // namespace rigidbodies::app
