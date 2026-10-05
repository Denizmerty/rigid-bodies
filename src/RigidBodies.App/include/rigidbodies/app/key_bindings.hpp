#pragma once

#include <rigidbodies/ui/ui_command.hpp>
#include <rigidbodies/ui/ui_event.hpp>

#include <array>
#include <string_view>

namespace rigidbodies::app
{
    enum class AppAction
    {
        none,
        quit,
        toggle_pause,
        pause_at_next_impact,
        single_step,
        step_many,
        reset_scenario,
        replay,
        frame_subject,
        frame_selection,
        toggle_developer_overlay,
        delete_selection,
        undo,
        redo,
        select_all,
        select_mode,
        throw_mode,
        pull_mode,
        toggle_help,
        open_library,
        toggle_show,
        toggle_measure,
        open_world,
        toggle_guide,
        toggle_inspector_pin,
        open_main_menu,
        open_preferences,
        open_add_menu,
        draw_shape,
        combine_selection,
        separate_selection,
        previous_object,
        next_object,
        save_setup,
        save_setup_as,
        open_setup,
        import_shape,
        nudge_left,
        nudge_right,
        nudge_up,
        nudge_down,
        nudge_left_large,
        nudge_right_large,
        nudge_up_large,
        nudge_down_large,
        toggle_present,
        open_command_search,
        open_context_menu,
        previous_guide_step,
        next_guide_step,
        raise_probe_speed,
        lower_probe_speed,
        next_speed_preset,
        previous_speed_preset
    };

    struct KeyBinding
    {
        ui::UiKey key;
        ui::KeyModifiers modifiers;
        AppAction action;
        std::string_view chord;
        std::string_view description;
    };

    // This is the shared source for dispatch and the visible keyboard reference. Matching all
    // modifiers prevents a text-editing chord such as Ctrl+R from silently resetting a world.
    inline constexpr std::array<KeyBinding, 56> key_bindings { { { ui::UiKey::space, {}, AppAction::toggle_pause, "Space", "Play / pause" },
        { ui::UiKey::space, { true, false, false }, AppAction::pause_at_next_impact, "Shift+Space", "Play until next impact" },
        { ui::UiKey::period, {}, AppAction::single_step, ".", "Single step" },
        { ui::UiKey::period, { true, false, false }, AppAction::step_many, "Shift+.", "Step 10 frames" },
        { ui::UiKey::r, {}, AppAction::reset_scenario, "R", "Back to start" },
        { ui::UiKey::r, { true, false, false }, AppAction::replay, "Shift+R", "Replay from start" },
        { ui::UiKey::f, {}, AppAction::frame_subject, "F", "Frame subject" },
        { ui::UiKey::f, { true, false, false }, AppAction::frame_selection, "Shift+F", "Frame selection" },
        { ui::UiKey::delete_key, {}, AppAction::delete_selection, "Del", "Delete selection" },
        { ui::UiKey::z, { false, true, false }, AppAction::undo, "Ctrl+Z", "Undo" },
        { ui::UiKey::z, { true, true, false }, AppAction::redo, "Ctrl+Shift+Z", "Redo" },
        { ui::UiKey::y, { false, true, false }, AppAction::redo, "Ctrl+Y", "Redo" },
        { ui::UiKey::a, { false, true, false }, AppAction::select_all, "Ctrl+A", "Select all free objects" },
        { ui::UiKey::v, {}, AppAction::select_mode, "V", "Select & move" },
        { ui::UiKey::t, {}, AppAction::throw_mode, "T", "Throw" },
        { ui::UiKey::p, {}, AppAction::pull_mode, "P", "Pull" },
        { ui::UiKey::f1, {}, AppAction::toggle_help, "F1", "Keyboard shortcuts" },
        { ui::UiKey::slash, { true, false, false }, AppAction::toggle_help, "?", "Keyboard shortcuts" },
        { ui::UiKey::l, {}, AppAction::open_library, "L", "Open Library" },
        { ui::UiKey::s, {}, AppAction::toggle_show, "S", "Toggle Show" },
        { ui::UiKey::m, {}, AppAction::toggle_measure, "M", "Toggle Measure" },
        { ui::UiKey::w, {}, AppAction::open_world, "W", "Open World" },
        { ui::UiKey::g, {}, AppAction::toggle_guide, "G", "Toggle Guide" },
        { ui::UiKey::i, {}, AppAction::toggle_inspector_pin, "I", "Toggle Inspector" },
        { ui::UiKey::f10, {}, AppAction::open_main_menu, "F10", "Open main menu" },
        { ui::UiKey::f10, { true, false, false }, AppAction::open_context_menu, "Shift+F10", "Open context menu" },
        { ui::UiKey::menu, {}, AppAction::open_context_menu, "Menu", "Open context menu" },
        { ui::UiKey::f5, {}, AppAction::toggle_present, "F5", "Toggle Present" },
        { ui::UiKey::k, { false, true, false }, AppAction::open_command_search, "Ctrl+K", "Command search" },
        { ui::UiKey::comma, { false, true, false }, AppAction::open_preferences, "Ctrl+,", "Open Preferences" },
        { ui::UiKey::a, { true, false, false }, AppAction::open_add_menu, "Shift+A", "Open Add menu" },
        { ui::UiKey::f12, {}, AppAction::toggle_developer_overlay, "F12", "Developer inspection (Debug)" },
        { ui::UiKey::q, { false, true, false }, AppAction::quit, "Ctrl+Q", "Quit" },
        { ui::UiKey::d, {}, AppAction::draw_shape, "D", "Draw shape" },
        { ui::UiKey::g, { false, true, false }, AppAction::combine_selection, "Ctrl+G", "Combine selected shapes" },
        { ui::UiKey::g, { true, true, false }, AppAction::separate_selection, "Ctrl+Shift+G", "Separate selected shape" },
        { ui::UiKey::left_bracket, {}, AppAction::previous_object, "[", "Previous object" },
        { ui::UiKey::right_bracket, {}, AppAction::next_object, "]", "Next object" },
        { ui::UiKey::s, { false, true, false }, AppAction::save_setup, "Ctrl+S", "Save setup" },
        { ui::UiKey::s, { true, true, false }, AppAction::save_setup_as, "Ctrl+Shift+S", "Save setup as" },
        { ui::UiKey::o, { false, true, false }, AppAction::open_setup, "Ctrl+O", "Open setup" },
        { ui::UiKey::i, { false, true, false }, AppAction::import_shape, "Ctrl+I", "Import shape" },
        { ui::UiKey::arrow_left, { false, false, true }, AppAction::nudge_left, "Alt+Left", "Nudge selection left 1 cm" },
        { ui::UiKey::arrow_right, { false, false, true }, AppAction::nudge_right, "Alt+Right", "Nudge selection right 1 cm" },
        { ui::UiKey::arrow_up, { false, false, true }, AppAction::nudge_up, "Alt+Up", "Nudge selection up 1 cm" },
        { ui::UiKey::arrow_down, { false, false, true }, AppAction::nudge_down, "Alt+Down", "Nudge selection down 1 cm" },
        { ui::UiKey::arrow_left, { true, false, true }, AppAction::nudge_left_large, "Shift+Alt+Left", "Nudge selection left 10 cm" },
        { ui::UiKey::arrow_right, { true, false, true }, AppAction::nudge_right_large, "Shift+Alt+Right", "Nudge selection right 10 cm" },
        { ui::UiKey::arrow_up, { true, false, true }, AppAction::nudge_up_large, "Shift+Alt+Up", "Nudge selection up 10 cm" },
        { ui::UiKey::arrow_down, { true, false, true }, AppAction::nudge_down_large, "Shift+Alt+Down", "Nudge selection down 10 cm" },
        { ui::UiKey::arrow_left, {}, AppAction::previous_guide_step, "Left", "Previous guide step (Present)" },
        { ui::UiKey::arrow_right, {}, AppAction::next_guide_step, "Right", "Next guide step (Present)" },
        { ui::UiKey::arrow_up, {}, AppAction::raise_probe_speed, "Up", "Raise probe speed a little" },
        { ui::UiKey::arrow_down, {}, AppAction::lower_probe_speed, "Down", "Lower probe speed a little" },
        { ui::UiKey::arrow_up, { true, false, false }, AppAction::next_speed_preset, "Shift+Up", "Next speed preset" },
        { ui::UiKey::arrow_down, { true, false, false }, AppAction::previous_speed_preset, "Shift+Down", "Previous speed preset" } } };

    [[nodiscard]] inline AppAction action_for_key(ui::UiKey key, const ui::KeyModifiers& modifiers)
    {
        for (const auto& binding : key_bindings)
            if (binding.key == key && binding.modifiers.shift == modifiers.shift && binding.modifiers.control == modifiers.control && binding.modifiers.alt == modifiers.alt)
                return binding.action;
        return AppAction::none;
    }

    [[nodiscard]] inline ui::UiCommand command_for_action(AppAction action)
    {
        ui::UiCommand command;
        switch (action)
        {
        case AppAction::quit:
            command.kind = ui::UiCommandKind::quit;
            break;
        case AppAction::toggle_pause:
            command.kind = ui::UiCommandKind::toggle_pause;
            break;
        case AppAction::pause_at_next_impact:
            command.kind = ui::UiCommandKind::pause_at_next_impact;
            command.flag = true;
            break;
        case AppAction::single_step:
            command.kind = ui::UiCommandKind::single_step;
            break;
        case AppAction::step_many:
            command.kind = ui::UiCommandKind::step_many;
            command.value = 10.0;
            break;
        case AppAction::reset_scenario:
            command.kind = ui::UiCommandKind::reset_scenario;
            break;
        case AppAction::replay:
            command.kind = ui::UiCommandKind::reset_scenario;
            command.flag = true;
            break;
        case AppAction::frame_subject:
            command.kind = ui::UiCommandKind::frame_subject;
            break;
        case AppAction::frame_selection:
            command.kind = ui::UiCommandKind::frame_selection;
            break;
        case AppAction::delete_selection:
            command.kind = ui::UiCommandKind::delete_selected_body;
            break;
        case AppAction::undo:
            command.kind = ui::UiCommandKind::undo;
            break;
        case AppAction::redo:
            command.kind = ui::UiCommandKind::redo;
            break;
        case AppAction::select_all:
            command.kind = ui::UiCommandKind::select_all;
            break;
        case AppAction::toggle_help:
        case AppAction::none:
        case AppAction::toggle_developer_overlay:
        case AppAction::open_library:
        case AppAction::toggle_show:
        case AppAction::toggle_measure:
        case AppAction::open_world:
        case AppAction::toggle_guide:
        case AppAction::toggle_inspector_pin:
        case AppAction::open_main_menu:
        case AppAction::open_preferences:
        case AppAction::open_add_menu:
        case AppAction::toggle_present:
        case AppAction::open_command_search:
        case AppAction::open_context_menu:
        case AppAction::previous_guide_step:
        case AppAction::next_guide_step:
            break;
        case AppAction::draw_shape:
            command.kind = ui::UiCommandKind::start_new_shape;
            break;
        case AppAction::combine_selection:
            command.kind = ui::UiCommandKind::assemble_selected_bodies;
            break;
        case AppAction::separate_selection:
            command.kind = ui::UiCommandKind::split_selected_body;
            break;
        case AppAction::previous_object:
        case AppAction::next_object:
            break;
        case AppAction::save_setup:
        case AppAction::save_setup_as:
            command.kind = ui::UiCommandKind::save_arrangement;
            command.flag = action == AppAction::save_setup_as;
            break;
        case AppAction::open_setup:
            command.kind = ui::UiCommandKind::open_arrangement;
            break;
        case AppAction::import_shape:
            command.kind = ui::UiCommandKind::import_shape;
            break;
        case AppAction::nudge_left:
        case AppAction::nudge_right:
        case AppAction::nudge_up:
        case AppAction::nudge_down:
        case AppAction::nudge_left_large:
        case AppAction::nudge_right_large:
        case AppAction::nudge_up_large:
        case AppAction::nudge_down_large:
        {
            command.kind = ui::UiCommandKind::set_selected_position;
            command.detail = "offset";
            const auto large = action == AppAction::nudge_left_large || action == AppAction::nudge_right_large ||
                action == AppAction::nudge_up_large || action == AppAction::nudge_down_large;
            const auto distance = large ? 0.10 : 0.01;
            if (action == AppAction::nudge_left || action == AppAction::nudge_left_large)
                command.value = -distance;
            if (action == AppAction::nudge_right || action == AppAction::nudge_right_large)
                command.value = distance;
            if (action == AppAction::nudge_up || action == AppAction::nudge_up_large)
                command.value_y = distance;
            if (action == AppAction::nudge_down || action == AppAction::nudge_down_large)
                command.value_y = -distance;
            break;
        }
        case AppAction::select_mode:
        case AppAction::throw_mode:
        case AppAction::pull_mode:
            command.kind = ui::UiCommandKind::set_interaction_mode;
            command.id = action == AppAction::throw_mode ? "throw" : action == AppAction::pull_mode ? "pull"
                                                                                                    : "select";
            break;
        // The probe's speed: one rapidity step, or the next rung of the speed ladder. A Newtonian
        // experiment consumes the command with no effect.
        case AppAction::raise_probe_speed:
        case AppAction::lower_probe_speed:
            command.kind = ui::UiCommandKind::set_relativity_speed;
            command.detail = "nudge";
            command.value = action == AppAction::raise_probe_speed ? 1.0 : -1.0;
            break;
        case AppAction::next_speed_preset:
        case AppAction::previous_speed_preset:
            command.kind = ui::UiCommandKind::set_relativity_speed;
            command.detail = "preset";
            command.value = action == AppAction::next_speed_preset ? 1.0 : -1.0;
            break;
        }
        return command;
    }
}
