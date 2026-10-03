#pragma once

#include <rigidbodies/math/vec2.hpp>

#include <cstdint>
#include <string>

namespace rigidbodies::ui
{

    using math::Vec2;

    // Input as the interface sees it, translated from whatever the platform layer reported.
    //
    // The interface never sees a platform event type. Keeping a neutral event here is what lets the
    // panels be exercised by a test that synthesises clicks, and what keeps the interface from
    // acquiring a dependency on the windowing library.
    enum class PointerButton
    {
        primary,
        secondary,
        middle
    };

    // Neutral keys shared by focused interface controls and the application's visible binding
    // registry. A focused control can consume a chord before the application dispatches it.
    enum class UiKey
    {
        unknown,
        escape,
        space,
        enter,
        tab,
        backspace,
        delete_key,
        arrow_left,
        arrow_right,
        arrow_up,
        arrow_down,
        period,
        a,
        c,
        d,
        f,
        g,
        h,
        i,
        j,
        k,
        l,
        m,
        n,
        o,
        p,
        q,
        r,
        s,
        t,
        v,
        w,
        x,
        y,
        z,
        digit_0,
        comma,
        slash,
        left_bracket,
        right_bracket,
        minus,
        equals,
        keypad_plus,
        keypad_minus,
        home,
        end,
        page_up,
        page_down,
        f1,
        f5,
        f6,
        f9,
        f10,
        menu,
        f12
    };

    struct KeyModifiers
    {
        bool shift { false };
        bool control { false };
        bool alt { false };
    };

    enum class UiEventKind
    {
        pointer_move,
        pointer_down,
        pointer_up,
        wheel,
        key_down,
        key_up,
        text_input,
        viewport_resized,
        focus_lost,
        focus_gained,
        // The pointer left the interface's client area: out of the window, or onto the window's
        // own title bar, resize edges or caption buttons, which the platform handles itself.
        pointer_leave
    };

    struct UiEvent
    {
        UiEventKind kind { UiEventKind::pointer_move };

        // Pointer position in device pixels, with the origin at the top-left corner.
        Vec2 pointer_px {};

        // Movement since the previous pointer event, in device pixels.
        Vec2 pointer_delta_px {};

        PointerButton button { PointerButton::primary };
        int click_count { 1 };

        // Positive away from the viewer, the direction that conventionally zooms in.
        double wheel_delta { 0.0 };

        UiKey key { UiKey::unknown };
        KeyModifiers modifiers {};
        bool repeat { false };

        // Device pixels per logical interface pixel. Pointer gesture thresholds divide by this
        // value so they remain stable across display scale and the interface text-size setting.
        double logical_pixel_scale { 1.0 };

        // Monotonic platform time, used only for estimating a released pointer's velocity.
        // Synthetic events may leave this at zero; a zero-duration gesture produces no throw.
        double timestamp_s { 0.0 };

        // Text for a text_input event, already decoded.
        std::string text;

        int viewport_width { 0 };
        int viewport_height { 0 };
    };

} // namespace rigidbodies::ui
