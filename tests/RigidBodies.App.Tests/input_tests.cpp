#include <rigidbodies/app/input.hpp>
#include <SDL3/SDL.h>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("SDL keyboard chords and repeats translate without opening a window")
    {
        app::InputTranslator translator;
        SDL_Event event {};
        event.type = SDL_EVENT_KEY_DOWN;
        event.key.key = SDLK_Z;
        event.key.mod = SDL_KMOD_CTRL;
        auto translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.action == app::AppAction::undo, "Ctrl+Z invokes undo");
        RIGIDBODIES_EXPECT(translated.ui_event && !translated.ui_event->repeat, "an initial key press is not marked as a repeat");
        event.key.repeat = true;
        translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->repeat, "an SDL auto-repeat is preserved in the neutral event");
        RIGIDBODIES_EXPECT(translated.action == app::AppAction::none, "held shortcuts do not repeat");
        event.key.repeat = false;
        event.key.mod = static_cast<SDL_Keymod>(SDL_KMOD_CTRL | SDL_KMOD_SHIFT);
        RIGIDBODIES_EXPECT(translator.translate(event).action == app::AppAction::redo, "Ctrl+Shift+Z invokes redo");
        event.key.key = SDLK_F1;
        event.key.mod = SDL_KMOD_NONE;
        RIGIDBODIES_EXPECT(translator.translate(event).action == app::AppAction::toggle_help, "F1 shows the teaching keyboard reference");
        event.key.key = SDLK_F12;
        RIGIDBODIES_EXPECT(translator.translate(event).action == app::AppAction::toggle_developer_overlay, "F12 is reserved for development inspection");

        event.key.key = SDLK_SPACE;
        event.key.repeat = false;
        RIGIDBODIES_EXPECT(translator.translate(event).action == app::AppAction::toggle_pause, "the first Space press dispatches Play or Pause");
        event.key.repeat = true;
        translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->repeat && translated.action == app::AppAction::none, "holding Space dispatches no second Play or Pause action");

        event.type = SDL_EVENT_KEY_UP;
        translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && !translated.ui_event->repeat, "key releases are never marked as auto-repeated presses");
    }

    RIGIDBODIES_TEST("SDL pointer events retain modifiers and nanosecond timestamps")
    {
        app::InputTranslator translator;
        SDL_Event event {};
        event.type = SDL_EVENT_KEY_DOWN;
        event.key.key = SDLK_LSHIFT;
        event.key.mod = SDL_KMOD_SHIFT;
        (void)translator.translate(event);
        event = {};
        event.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        event.button.button = SDL_BUTTON_LEFT;
        event.button.x = 25.0f;
        event.button.y = 40.0f;
        event.common.timestamp = 1250000000;
        const auto translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->modifiers.shift, "Shift reaches multi-selection on a pointer event");
        RIGIDBODIES_EXPECT_NEAR(translated.ui_event->timestamp_s, 1.25, 1.0e-12, "throw timing uses monotonic seconds");
        RIGIDBODIES_EXPECT(translator.is_primary_button_down(), "a primary press starts platform capture state");

        event.button.button = SDL_BUTTON_X1;
        RIGIDBODIES_EXPECT(!translator.translate(event).ui_event, "mouse X1 produces no neutral pointer event");
        event.button.button = SDL_BUTTON_X2;
        RIGIDBODIES_EXPECT(!translator.translate(event).ui_event, "mouse X2 produces no neutral pointer event");
    }

    RIGIDBODIES_TEST("SDL focus loss clears held buttons and modifiers and cancels scene capture")
    {
        app::InputTranslator translator;
        SDL_Event event {};
        event.type = SDL_EVENT_KEY_DOWN;
        event.key.key = SDLK_SPACE;
        event.key.mod = SDL_KMOD_SHIFT;
        (void)translator.translate(event);
        event = {};
        event.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        event.button.button = SDL_BUTTON_LEFT;
        (void)translator.translate(event);
        event.button.button = SDL_BUTTON_MIDDLE;
        (void)translator.translate(event);
        event = {};
        event.type = SDL_EVENT_WINDOW_FOCUS_LOST;
        const auto translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->kind == ui::UiEventKind::focus_lost, "scene capture receives an explicit cancellation");
        RIGIDBODIES_EXPECT(!translator.is_primary_button_down() && !translator.is_middle_button_down(), "no held pointer flags leak into the next focus session");
        RIGIDBODIES_EXPECT(!translated.ui_event->modifiers.shift, "selection modifiers reset with focus");
    }

    RIGIDBODIES_TEST("SDL window close requests ask to quit and a pointer leaving the window is reported")
    {
        app::InputTranslator translator;
        SDL_Event event {};
        event.type = SDL_EVENT_MOUSE_MOTION;
        event.motion.x = 120.0f;
        event.motion.y = 16.0f;
        (void)translator.translate(event);
        event = {};
        event.type = SDL_EVENT_WINDOW_MOUSE_LEAVE;
        auto translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->kind == ui::UiEventKind::pointer_leave, "the leave becomes an interface event");
        RIGIDBODIES_EXPECT(translated.ui_event && translated.ui_event->pointer_px.x == 120.0 && translated.ui_event->pointer_px.y == 16.0, "it carries where the pointer was last seen");
        RIGIDBODIES_EXPECT(!translated.quit_requested && !translated.close_requested, "leaving closes nothing");
        event = {};
        event.type = SDL_EVENT_WINDOW_CLOSE_REQUESTED;
        translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.close_requested && !translated.quit_requested && !translated.ui_event, "closing the window asks to quit rather than quitting");
        event = {};
        event.type = SDL_EVENT_QUIT;
        translated = translator.translate(event);
        RIGIDBODIES_EXPECT(translated.quit_requested && !translated.close_requested, "the platform ending the session still quits at once");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
