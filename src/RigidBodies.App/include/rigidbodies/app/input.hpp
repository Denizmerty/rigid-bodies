#pragma once

#include <rigidbodies/app/key_bindings.hpp>

#include <optional>
#include <vector>

union SDL_Event;

namespace rigidbodies::app
{

    // What one platform event turned into. A single event may produce an interface event, an
    // application action, or nothing at all.
    struct TranslatedEvent
    {
        std::optional<ui::UiEvent> ui_event;
        AppAction action { AppAction::none };
        bool quit_requested { false };
        bool viewport_changed { false };
        // The window was asked to close (its close button, Alt+F4, the taskbar), which quits the
        // way the Quit command does, confirming unsaved changes first.
        bool close_requested { false };
    };

    // Converts platform events into interface events and application actions.
    //
    // Panels and camera interaction use these platform-independent values, so tests can drive
    // them without opening a window.
    class InputTranslator
    {
    public:
        [[nodiscard]] TranslatedEvent translate(const SDL_Event& event);

        // Pointer position in pixels as of the most recent event.
        [[nodiscard]] const ui::Vec2& pointer_px() const;

        [[nodiscard]] bool is_primary_button_down() const;
        [[nodiscard]] bool is_middle_button_down() const;

    private:
        ui::Vec2 pointer_px_ {};
        ui::KeyModifiers modifiers_ {};
        bool primary_button_down_ { false };
        bool middle_button_down_ { false };
    };

} // namespace rigidbodies::app
