#include <rigidbodies/app/input_router.hpp>

namespace rigidbodies::app
{
    namespace
    {
        bool is_key(const ui::UiEvent& event)
        {
            return event.kind == ui::UiEventKind::key_down || event.kind == ui::UiEventKind::key_up;
        }

        bool is_pointer(const ui::UiEvent& event)
        {
            return event.kind == ui::UiEventKind::pointer_move || event.kind == ui::UiEventKind::pointer_down ||
                event.kind == ui::UiEventKind::pointer_up || event.kind == ui::UiEventKind::wheel;
        }

        bool field_key(const ui::UiEvent& event)
        {
            if (event.kind == ui::UiEventKind::text_input)
                return true;
            if (!is_key(event))
                return false;
            if (event.modifiers.alt)
                return false;
            if (event.modifiers.control)
                return event.key == ui::UiKey::a || event.key == ui::UiKey::c || event.key == ui::UiKey::v ||
                    event.key == ui::UiKey::x || event.key == ui::UiKey::z;
            switch (event.key)
            {
            case ui::UiKey::backspace:
            case ui::UiKey::delete_key:
            case ui::UiKey::arrow_left:
            case ui::UiKey::arrow_right:
            case ui::UiKey::arrow_up:
            case ui::UiKey::arrow_down:
            case ui::UiKey::home:
            case ui::UiKey::end:
            case ui::UiKey::enter:
            case ui::UiKey::tab:
            case ui::UiKey::space:
            case ui::UiKey::period:
            case ui::UiKey::comma:
            case ui::UiKey::slash:
            case ui::UiKey::left_bracket:
            case ui::UiKey::right_bracket:
            case ui::UiKey::minus:
            case ui::UiKey::equals:
            case ui::UiKey::keypad_plus:
            case ui::UiKey::keypad_minus:
            case ui::UiKey::digit_0:
            case ui::UiKey::a:
            case ui::UiKey::c:
            case ui::UiKey::d:
            case ui::UiKey::f:
            case ui::UiKey::g:
            case ui::UiKey::h:
            case ui::UiKey::i:
            case ui::UiKey::j:
            case ui::UiKey::k:
            case ui::UiKey::l:
            case ui::UiKey::m:
            case ui::UiKey::n:
            case ui::UiKey::o:
            case ui::UiKey::p:
            case ui::UiKey::q:
            case ui::UiKey::r:
            case ui::UiKey::s:
            case ui::UiKey::t:
            case ui::UiKey::v:
            case ui::UiKey::w:
            case ui::UiKey::x:
            case ui::UiKey::y:
            case ui::UiKey::z:
                return true;
            default:
                return false;
            }
        }

        EscapeStep escape_step(const InputContext& context)
        {
            if (context.gesture_active || context.pick_surface_armed)
                return EscapeStep::cancel_gesture;
            if (context.interface_escape == ui::EscapeTarget::text_field)
                return EscapeStep::revert_field;
            if (context.interface_escape == ui::EscapeTarget::transient)
                return EscapeStep::close_transient;
            if (context.interface_escape == ui::EscapeTarget::sheet)
                return EscapeStep::close_sheet;
            if (context.draw_active)
            {
                if (context.draft_node_selected)
                    return EscapeStep::draw_deselect_node;
                return context.draft_nodes >= 2 ? EscapeStep::draw_confirm_discard : EscapeStep::draw_discard;
            }
            if (!context.select_tool)
                return EscapeStep::tool_to_select;
            if (context.has_selection)
                return EscapeStep::clear_selection;
            if (context.keyboard_mode)
                return EscapeStep::leave_keyboard_mode;
            if (context.present)
                return EscapeStep::leave_present;
            return EscapeStep::none;
        }
    }

    RouteDecision route_event(const ui::UiEvent& event, std::optional<AppAction> bound_action, const InputContext& context)
    {
        RouteDecision result;
        if (event.kind == ui::UiEventKind::key_down && event.key == ui::UiKey::f12)
        {
            result.action = AppAction::toggle_developer_overlay;
            return result;
        }
        if (context.developer_captured)
        {
            result.to_developer = true;
            return result;
        }
        if (event.repeat && is_key(event))
        {
            result.to_interface = context.focus == ui::FocusOwner::text_field;
            return result;
        }
        if (event.kind == ui::UiEventKind::key_down && event.key == ui::UiKey::escape)
        {
            result.escape = escape_step(context);
            result.to_interface = result.escape == EscapeStep::revert_field || result.escape == EscapeStep::close_transient ||
                result.escape == EscapeStep::close_sheet || result.escape == EscapeStep::leave_keyboard_mode || result.escape == EscapeStep::leave_present;
            result.to_scene = result.escape == EscapeStep::cancel_gesture || result.escape == EscapeStep::draw_deselect_node ||
                result.escape == EscapeStep::draw_confirm_discard || result.escape == EscapeStep::draw_discard ||
                result.escape == EscapeStep::tool_to_select || result.escape == EscapeStep::clear_selection;
            return result;
        }
        if (context.focus == ui::FocusOwner::text_field && field_key(event) &&
            !(event.kind == ui::UiEventKind::key_down && event.modifiers.control && event.key == ui::UiKey::k))
        {
            result.to_interface = true;
            return result;
        }
        if (context.focus == ui::FocusOwner::transient && is_key(event))
        {
            const auto transient_key = event.key == ui::UiKey::arrow_left || event.key == ui::UiKey::arrow_right ||
                event.key == ui::UiKey::arrow_up || event.key == ui::UiKey::arrow_down || event.key == ui::UiKey::enter ||
                event.key == ui::UiKey::home || event.key == ui::UiKey::end || event.key == ui::UiKey::tab ||
                (context.open_list && event.key == ui::UiKey::space);
            if (transient_key)
            {
                result.to_interface = true;
                return result;
            }
        }
        if (context.gesture_active && is_pointer(event))
        {
            result.to_scene = true;
            return result;
        }
        if (context.focus == ui::FocusOwner::keyboard_control && is_key(event) &&
            (event.key == ui::UiKey::enter || event.key == ui::UiKey::arrow_left || event.key == ui::UiKey::arrow_right ||
                event.key == ui::UiKey::arrow_up || event.key == ui::UiKey::arrow_down))
        {
            result.to_interface = true;
            return result;
        }
        if (event.kind == ui::UiEventKind::key_down && (event.key == ui::UiKey::tab || event.key == ui::UiKey::f6))
        {
            result.to_interface = true;
            return result;
        }
        if (bound_action && *bound_action != AppAction::none && event.kind == ui::UiEventKind::key_down)
        {
            if (context.present && context.present_locked)
            {
                const auto allowed = *bound_action == AppAction::toggle_present || *bound_action == AppAction::toggle_pause || *bound_action == AppAction::pause_at_next_impact ||
                    *bound_action == AppAction::reset_scenario || *bound_action == AppAction::replay || *bound_action == AppAction::frame_subject ||
                    *bound_action == AppAction::select_mode || *bound_action == AppAction::throw_mode || *bound_action == AppAction::pull_mode ||
                    *bound_action == AppAction::previous_guide_step || *bound_action == AppAction::next_guide_step;
                if (!allowed)
                    return result;
            }
            result.action = bound_action;
            return result;
        }
        if (is_pointer(event))
        {
            result.to_interface = context.pointer_over_surface || context.pointer_captured_by_interface;
            result.to_scene = !result.to_interface;
            return result;
        }
        if (event.kind == ui::UiEventKind::focus_lost || event.kind == ui::UiEventKind::pointer_leave)
            result.to_interface = result.to_scene = true;
        return result;
    }

    FrameHit frame_hit(const ui::Vec2& point, ui::WindowPart part, const FrameGeometry& frame)
    {
        // The top edge resizes across the whole width, over the controls too, as a native title
        // bar's does; within two edge depths of a corner it resizes diagonally. A maximized
        // window has no edge, so its controls reach the screen's top and corner.
        if (frame.resizable && !frame.maximized && point.y < frame.edge)
        {
            const auto corner = 2.0 * frame.edge;
            if (point.x < corner)
                return FrameHit::top_left;
            if (point.x >= frame.width - corner)
                return FrameHit::top_right;
            return FrameHit::top;
        }
        switch (part)
        {
        case ui::WindowPart::title_bar:
            return FrameHit::title_bar;
        case ui::WindowPart::minimize:
            return FrameHit::minimize;
        case ui::WindowPart::maximize:
            return frame.resizable ? FrameHit::maximize : FrameHit::title_bar;
        case ui::WindowPart::close:
            return FrameHit::close;
        case ui::WindowPart::client:
            break;
        }
        return FrameHit::client;
    }
}
