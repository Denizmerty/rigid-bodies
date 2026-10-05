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
        logarithmic,
        // Speed fractions in [0, 1): the slider and the normal step move in rapidity, so each nine
        // gets equal room and no position or step reaches c.
        rapidity
    };
    // Which step a keyboard press or a label scrub takes: Shift asks for the coarse step, Alt for the
    // fine one.
    enum class StepSize : std::uint8_t
    {
        normal,
        coarse,
        fine
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
        // Multipliers of the step on a linear scale and factors on a logarithmic one. A rapidity
        // scale ignores both: its coarse step is the speed ladder and its fine step one unit in the
        // last digit of the value's own text.
        double coarse { 10.0 }, fine { 0.1 };
        NumberScale scale { NumberScale::linear };
        core::DisplayQuantity quantity {};
        int decimals { -1 };
        math::Span<const double> detents;
        bool wraps { false };
        bool slider { false }, dial { false };
        bool session_minimum { false }, session_maximum { false };
        // When positive, the smallest value above zero that can be set: a speed is rest or at least
        // this fast.
        double smallest_nonzero {};
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

    // Slider position in [0, 1] for a value (clamped to the soft range), and back.
    [[nodiscard]] double slider_position(const NumberSpec& spec, double value);
    [[nodiscard]] double slider_value(const NumberSpec& spec, double position);
    // One keyboard step (a focused field's arrows, the global Up/Down keys), clamped to [minimum, maximum].
    [[nodiscard]] double keyboard_step(const NumberSpec& spec, double value, int direction, StepSize size);
    // A label scrub `steps` steps from its start value, clamped to [minimum, maximum].
    [[nodiscard]] double scrub_value(const NumberSpec& spec, double start, int steps, StepSize size);
    // Rounds to `decimals` when the spec has them, then clamps to [minimum, maximum] again.
    [[nodiscard]] double rounded_value(const NumberSpec& spec, double value);
    // Whether a typed value can be set: within [minimum, maximum], and zero or at least the
    // smallest value above zero.
    [[nodiscard]] bool accepts_value(const NumberSpec& spec, double value);
    // Whether a typed value differs from the one shown by more than rounding: by a millionth of a
    // millionth, relative to the values where the smallest value above zero is itself that small.
    [[nodiscard]] bool differs_from_shown(const NumberSpec& spec, double typed, double shown);

    [[nodiscard]] bool valid_control_key(std::string_view key);
    [[nodiscard]] std::string element_id(std::string_view host, std::string_view key, std::string_view instance = {});
    [[nodiscard]] std::string control_key(const UiCommand& command);
}
