#pragma once

#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/ui/ui_backend.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace rigidbodies::app
{
    struct InputContext
    {
        ui::FocusOwner focus { ui::FocusOwner::scene };
        ui::EscapeTarget interface_escape { ui::EscapeTarget::none };
        bool developer_captured {};
        bool pointer_over_surface {}, pointer_captured_by_interface {};
        bool gesture_active {}, draw_active {}, draft_node_selected {};
        std::size_t draft_nodes {};
        bool select_tool {}, has_selection {}, keyboard_mode {}, present {}, present_locked {}, pick_surface_armed {};
        bool open_list {};
        bool interface_modal {};
    };

    enum class EscapeStep : std::uint8_t
    {
        none,
        cancel_gesture,
        revert_field,
        close_control,
        close_transient,
        close_sheet,
        draw_deselect_node,
        draw_confirm_discard,
        draw_discard,
        tool_to_select,
        clear_selection,
        leave_keyboard_mode,
        leave_present
    };

    struct RouteDecision
    {
        bool to_developer {}, to_interface {}, to_scene {};
        std::optional<AppAction> action;
        EscapeStep escape { EscapeStep::none };
    };

    [[nodiscard]] RouteDecision route_event(const ui::UiEvent& event, std::optional<AppAction> bound_action,
        const InputContext& context);

    // Where a point lies in a window whose title bar the interface draws: the interface, the
    // title bar the window moves by, a window control, or the window's top edge, which resizes
    // it. The platform's own borders outside the window take the other three edges.
    enum class FrameHit : std::uint8_t
    {
        client,
        title_bar,
        minimize,
        maximize,
        close,
        top,
        top_left,
        top_right
    };

    struct FrameGeometry
    {
        // The window's client area in pixels.
        double width {}, height {};
        // The depth of the resizing top edge, in pixels.
        double edge {};
        bool maximized {}, resizable { true };
    };

    [[nodiscard]] FrameHit frame_hit(const ui::Vec2& point, ui::WindowPart part, const FrameGeometry& frame);
}
