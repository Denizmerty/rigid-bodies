#pragma once

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/math/span.hpp>
#include <rigidbodies/ui/ui_command.hpp>

#include <cstdint>
#include <string>
#include <string_view>

namespace rigidbodies::ui
{
    enum class NumberScale : std::uint8_t
    {
        linear,
        logarithmic
    };
    enum class ControlKind : std::uint8_t
    {
        action,
        number,
        stepper,
        select,
        radio_list,
        checklist,
        segmented,
        switch_control,
        checkbox,
        tabs,
        section,
        list,
        readout
    };
    enum class EditCategory : std::uint8_t
    {
        not_an_edit,
        parameter,
        state,
        structure,
        structure_pose,
        lab,
        draft
    };

    struct NumberSpec
    {
        double minimum {}, maximum {};
        double soft_minimum {}, soft_maximum {};
        double step {};
        double coarse { 10.0 }, fine { 0.1 };
        NumberScale scale { NumberScale::linear };
        core::DisplayQuantity quantity {};
        int decimals { -1 };
        math::Span<const double> detents;
        bool wraps { false };
        bool slider { false }, dial { false };
        bool session_minimum { false }, session_maximum { false };
    };

    struct OptionSpec
    {
        std::string_view id, label, secondary, icon;
    };

    struct ControlSpec
    {
        std::string_view key, label, location;
        ControlKind kind {};
        UiCommandKind command {};
        std::string_view command_id;
        NumberSpec number;
        math::Span<const OptionSpec> options;
        std::string_view shortcut;
        EditCategory category {};
        std::string_view expert_term;
        math::Span<const std::string_view> synonyms;
        bool developer_only { false };
    };

    [[nodiscard]] const ControlSpec* find_control_spec(std::string_view key);
    [[nodiscard]] math::Span<const ControlSpec> control_specs();

    [[nodiscard]] bool valid_control_key(std::string_view key);
    [[nodiscard]] std::string element_id(std::string_view host, std::string_view key, std::string_view instance = {});
    [[nodiscard]] std::string control_key(const UiCommand& command);
}
