#pragma once

#include <rigidbodies/math/vec2.hpp>

#include <limits>

namespace rigidbodies::math
{

    // Axis-aligned bounds used by the broad phase, by camera framing, and by picking. An empty
    // box is represented by inverted extents so that expansion from empty behaves correctly.
    struct Aabb
    {
        Vec2 minimum { std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max() };
        Vec2 maximum { std::numeric_limits<Real>::lowest(), std::numeric_limits<Real>::lowest() };

        [[nodiscard]] constexpr bool is_empty() const
        {
            return minimum.x > maximum.x || minimum.y > maximum.y;
        }

        [[nodiscard]] constexpr Vec2 center() const
        {
            return (minimum + maximum) * 0.5;
        }

        [[nodiscard]] constexpr Vec2 extents() const
        {
            return maximum - minimum;
        }

        [[nodiscard]] constexpr Real area() const
        {
            if (is_empty())
            {
                return 0.0;
            }
            const auto size = extents();
            return size.x * size.y;
        }

        constexpr void expand(const Vec2& point)
        {
            minimum = min_components(minimum, point);
            maximum = max_components(maximum, point);
        }

        constexpr void expand(const Aabb& other)
        {
            if (other.is_empty())
            {
                return;
            }
            expand(other.minimum);
            expand(other.maximum);
        }

        // Grows the box uniformly. The broad phase inflates proxies by a small margin so that a
        // slowly moving body does not force a rebuild on every step.
        constexpr void grow(Real margin)
        {
            if (is_empty())
            {
                return;
            }
            minimum -= Vec2 { margin, margin };
            maximum += Vec2 { margin, margin };
        }

        [[nodiscard]] constexpr bool contains(const Vec2& point) const
        {
            return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y;
        }
    };

    [[nodiscard]] inline constexpr bool overlaps(const Aabb& left, const Aabb& right)
    {
        if (left.is_empty() || right.is_empty())
        {
            return false;
        }
        return left.minimum.x <= right.maximum.x && left.maximum.x >= right.minimum.x && left.minimum.y <= right.maximum.y && left.maximum.y >= right.minimum.y;
    }

    [[nodiscard]] inline constexpr Aabb combined(const Aabb& left, const Aabb& right)
    {
        Aabb result = left;
        result.expand(right);
        return result;
    }

} // namespace rigidbodies::math
