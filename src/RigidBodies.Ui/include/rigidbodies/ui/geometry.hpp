#pragma once

#include <rigidbodies/math/vec2.hpp>

namespace rigidbodies::ui
{
    struct Rect
    {
        math::Vec2 minimum {};
        math::Vec2 maximum {};

        [[nodiscard]] double width() const;
        [[nodiscard]] double height() const;
        [[nodiscard]] bool contains(const math::Vec2& point) const;
        [[nodiscard]] Rect inset(double amount) const;
    };
}
