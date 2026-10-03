#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/ui_model.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::ui
{
    struct CommandSearchResult
    {
        const ControlSpec* control {};
        std::string label, location, key;
        UiCommand command;
        std::string shortcut;
        int rank {};
    };

    [[nodiscard]] std::vector<CommandSearchResult> search_commands(std::string_view query,
        math::Span<const KeyReference> keys = {});
}
