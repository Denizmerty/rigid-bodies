#include <rigidbodies/app/input.hpp>

#include <SDL3/SDL.h>

namespace rigidbodies::app
{
    namespace
    {

        std::optional<ui::PointerButton> to_pointer_button(Uint8 button)
        {
            switch (button)
            {
            case SDL_BUTTON_LEFT:
                return ui::PointerButton::primary;
            case SDL_BUTTON_RIGHT:
                return ui::PointerButton::secondary;
            case SDL_BUTTON_MIDDLE:
                return ui::PointerButton::middle;
            default:
                return std::nullopt;
            }
        }

        ui::UiKey to_ui_key(SDL_Keycode key)
        {
            switch (key)
            {
            case SDLK_ESCAPE:
                return ui::UiKey::escape;
            case SDLK_SPACE:
                return ui::UiKey::space;
            case SDLK_RETURN:
            case SDLK_KP_ENTER:
                return ui::UiKey::enter;
            case SDLK_TAB:
                return ui::UiKey::tab;
            case SDLK_BACKSPACE:
                return ui::UiKey::backspace;
            case SDLK_DELETE:
                return ui::UiKey::delete_key;
            case SDLK_LEFT:
                return ui::UiKey::arrow_left;
            case SDLK_RIGHT:
                return ui::UiKey::arrow_right;
            case SDLK_UP:
                return ui::UiKey::arrow_up;
            case SDLK_DOWN:
                return ui::UiKey::arrow_down;
            case SDLK_PERIOD:
                return ui::UiKey::period;
            case SDLK_A:
                return ui::UiKey::a;
            case SDLK_C:
                return ui::UiKey::c;
            case SDLK_D:
                return ui::UiKey::d;
            case SDLK_F:
                return ui::UiKey::f;
            case SDLK_G:
                return ui::UiKey::g;
            case SDLK_H:
                return ui::UiKey::h;
            case SDLK_I:
                return ui::UiKey::i;
            case SDLK_J:
                return ui::UiKey::j;
            case SDLK_K:
                return ui::UiKey::k;
            case SDLK_L:
                return ui::UiKey::l;
            case SDLK_M:
                return ui::UiKey::m;
            case SDLK_N:
                return ui::UiKey::n;
            case SDLK_O:
                return ui::UiKey::o;
            case SDLK_P:
                return ui::UiKey::p;
            case SDLK_Q:
                return ui::UiKey::q;
            case SDLK_R:
                return ui::UiKey::r;
            case SDLK_S:
                return ui::UiKey::s;
            case SDLK_T:
                return ui::UiKey::t;
            case SDLK_V:
                return ui::UiKey::v;
            case SDLK_W:
                return ui::UiKey::w;
            case SDLK_X:
                return ui::UiKey::x;
            case SDLK_Y:
                return ui::UiKey::y;
            case SDLK_Z:
                return ui::UiKey::z;
            case SDLK_0:
                return ui::UiKey::digit_0;
            case SDLK_COMMA:
                return ui::UiKey::comma;
            case SDLK_SLASH:
                return ui::UiKey::slash;
            case SDLK_LEFTBRACKET:
                return ui::UiKey::left_bracket;
            case SDLK_RIGHTBRACKET:
                return ui::UiKey::right_bracket;
            case SDLK_MINUS:
                return ui::UiKey::minus;
            case SDLK_EQUALS:
                return ui::UiKey::equals;
            case SDLK_KP_PLUS:
                return ui::UiKey::keypad_plus;
            case SDLK_KP_MINUS:
                return ui::UiKey::keypad_minus;
            case SDLK_HOME:
                return ui::UiKey::home;
            case SDLK_END:
                return ui::UiKey::end;
            case SDLK_PAGEUP:
                return ui::UiKey::page_up;
            case SDLK_PAGEDOWN:
                return ui::UiKey::page_down;
            case SDLK_F1:
                return ui::UiKey::f1;
            case SDLK_F5:
                return ui::UiKey::f5;
            case SDLK_F6:
                return ui::UiKey::f6;
            case SDLK_F9:
                return ui::UiKey::f9;
            case SDLK_F10:
                return ui::UiKey::f10;
            case SDLK_MENU:
                return ui::UiKey::menu;
            case SDLK_F12:
                return ui::UiKey::f12;
            default:
                return ui::UiKey::unknown;
            }
        }

        ui::KeyModifiers to_modifiers(SDL_Keymod modifiers)
        {
            ui::KeyModifiers result;
            result.shift = (modifiers & SDL_KMOD_SHIFT) != 0;
            result.control = (modifiers & SDL_KMOD_CTRL) != 0;
            result.alt = (modifiers & SDL_KMOD_ALT) != 0;
            return result;
        }

    } // namespace

    const ui::Vec2& InputTranslator::pointer_px() const
    {
        return pointer_px_;
    }

    bool InputTranslator::is_primary_button_down() const
    {
        return primary_button_down_;
    }

    bool InputTranslator::is_middle_button_down() const
    {
        return middle_button_down_;
    }

    TranslatedEvent InputTranslator::translate(const SDL_Event& event)
    {
        TranslatedEvent result;

        switch (event.type)
        {
        case SDL_EVENT_QUIT:
        {
            result.quit_requested = true;
            break;
        }
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
        {
            result.close_requested = true;
            break;
        }
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
        {
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::pointer_leave;
            translated.pointer_px = pointer_px_;
            translated.modifiers = modifiers_;
            result.ui_event = translated;
            break;
        }
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
        {
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::viewport_resized;
            translated.viewport_width = event.window.data1;
            translated.viewport_height = event.window.data2;
            result.ui_event = translated;
            result.viewport_changed = true;
            break;
        }
        case SDL_EVENT_WINDOW_FOCUS_LOST:
        {
            primary_button_down_ = middle_button_down_ = false;
            modifiers_ = {};
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::focus_lost;
            translated.pointer_px = pointer_px_;
            result.ui_event = translated;
            break;
        }
        case SDL_EVENT_WINDOW_FOCUS_GAINED:
        {
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::focus_gained;
            translated.pointer_px = pointer_px_;
            result.ui_event = translated;
            break;
        }
        case SDL_EVENT_MOUSE_MOTION:
        {
            const ui::Vec2 position { static_cast<double>(event.motion.x), static_cast<double>(event.motion.y) };

            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::pointer_move;
            translated.pointer_px = position;
            translated.pointer_delta_px = { static_cast<double>(event.motion.xrel), static_cast<double>(event.motion.yrel) };
            result.ui_event = translated;

            pointer_px_ = position;
            break;
        }
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
        {
            const auto pressed = event.type == SDL_EVENT_MOUSE_BUTTON_DOWN;
            const auto button = to_pointer_button(event.button.button);
            if (!button)
                break;

            ui::UiEvent translated;
            translated.kind = pressed ? ui::UiEventKind::pointer_down : ui::UiEventKind::pointer_up;
            translated.pointer_px = { static_cast<double>(event.button.x), static_cast<double>(event.button.y) };
            translated.click_count = static_cast<int>(event.button.clicks);
            translated.button = *button;
            result.ui_event = translated;

            pointer_px_ = translated.pointer_px;
            if (*button == ui::PointerButton::primary)
            {
                primary_button_down_ = pressed;
            }
            else if (*button == ui::PointerButton::middle)
            {
                middle_button_down_ = pressed;
            }
            break;
        }
        case SDL_EVENT_MOUSE_WHEEL:
        {
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::wheel;
            translated.pointer_px = pointer_px_;
            translated.wheel_delta = static_cast<double>(event.wheel.y);
            result.ui_event = translated;
            break;
        }
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP:
        {
            const auto pressed = event.type == SDL_EVENT_KEY_DOWN;

            ui::UiEvent translated;
            translated.kind = pressed ? ui::UiEventKind::key_down : ui::UiEventKind::key_up;
            translated.key = to_ui_key(event.key.key);
            translated.modifiers = to_modifiers(event.key.mod);
            translated.repeat = pressed && event.key.repeat;
            modifiers_ = translated.modifiers;
            translated.pointer_px = pointer_px_;
            result.ui_event = translated;

            // Repeats are ignored so that holding a key does not fire its action every frame.
            if (pressed && !event.key.repeat)
            {
                result.action = action_for_key(translated.key, translated.modifiers);
            }
            break;
        }
        case SDL_EVENT_TEXT_INPUT:
        {
            ui::UiEvent translated;
            translated.kind = ui::UiEventKind::text_input;
            translated.text = event.text.text != nullptr ? event.text.text : "";
            result.ui_event = translated;
            break;
        }
        default:
            break;
        }

        if (result.ui_event)
        {
            result.ui_event->timestamp_s = static_cast<double>(event.common.timestamp) * 1.0e-9;
            result.ui_event->modifiers = modifiers_;
        }
        return result;
    }

} // namespace rigidbodies::app
