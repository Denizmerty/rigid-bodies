#pragma once

#include <rigidbodies/math/scalar.hpp>

namespace rigidbodies::math
{

    struct Vec2
    {
        Real x {};
        Real y {};

        constexpr Vec2() = default;

        constexpr Vec2(Real x_value, Real y_value) : x(x_value), y(y_value)
        {
        }

        [[nodiscard]] constexpr Vec2 operator-() const
        {
            return { -x, -y };
        }

        constexpr Vec2& operator+=(const Vec2& other)
        {
            x += other.x;
            y += other.y;
            return *this;
        }

        constexpr Vec2& operator-=(const Vec2& other)
        {
            x -= other.x;
            y -= other.y;
            return *this;
        }

        constexpr Vec2& operator*=(Real scale)
        {
            x *= scale;
            y *= scale;
            return *this;
        }

        constexpr Vec2& operator/=(Real divisor)
        {
            x /= divisor;
            y /= divisor;
            return *this;
        }
    };

    [[nodiscard]] inline constexpr Vec2 operator+(const Vec2& left, const Vec2& right)
    {
        return { left.x + right.x, left.y + right.y };
    }

    [[nodiscard]] inline constexpr Vec2 operator-(const Vec2& left, const Vec2& right)
    {
        return { left.x - right.x, left.y - right.y };
    }

    [[nodiscard]] inline constexpr Vec2 operator*(const Vec2& vector, Real scale)
    {
        return { vector.x * scale, vector.y * scale };
    }

    [[nodiscard]] inline constexpr Vec2 operator*(Real scale, const Vec2& vector)
    {
        return { vector.x * scale, vector.y * scale };
    }

    [[nodiscard]] inline constexpr Vec2 operator/(const Vec2& vector, Real divisor)
    {
        return { vector.x / divisor, vector.y / divisor };
    }

    [[nodiscard]] inline constexpr bool operator==(const Vec2& left, const Vec2& right)
    {
        return left.x == right.x && left.y == right.y;
    }

    [[nodiscard]] inline constexpr Real dot(const Vec2& left, const Vec2& right)
    {
        return left.x * right.x + left.y * right.y;
    }

    // Scalar cross product. In two dimensions the cross of two vectors is the signed area of the
    // parallelogram they span, which is also the out-of-plane component used for torque.
    [[nodiscard]] inline constexpr Real cross(const Vec2& left, const Vec2& right)
    {
        return left.x * right.y - left.y * right.x;
    }

    // Cross product of an in-plane vector with an out-of-plane scalar, used to turn an angular
    // velocity into the linear velocity it induces at an offset.
    [[nodiscard]] inline constexpr Vec2 cross(const Vec2& vector, Real scalar)
    {
        return { scalar * vector.y, -scalar * vector.x };
    }

    [[nodiscard]] inline constexpr Vec2 cross(Real scalar, const Vec2& vector)
    {
        return { -scalar * vector.y, scalar * vector.x };
    }

    [[nodiscard]] inline constexpr Real length_squared(const Vec2& vector)
    {
        return dot(vector, vector);
    }

    [[nodiscard]] inline Real length(const Vec2& vector)
    {
        return std::sqrt(length_squared(vector));
    }

    [[nodiscard]] inline Real distance(const Vec2& left, const Vec2& right)
    {
        return length(right - left);
    }

    // Returns the zero vector for inputs shorter than the geometric tolerance so that callers
    // never propagate a division by a degenerate length.
    [[nodiscard]] inline Vec2 normalized(const Vec2& vector)
    {
        const auto magnitude = length(vector);
        if (magnitude <= geometric_epsilon)
        {
            return {};
        }
        return vector / magnitude;
    }

    // Rotates by a quarter turn counter-clockwise.
    [[nodiscard]] inline constexpr Vec2 perpendicular(const Vec2& vector)
    {
        return { -vector.y, vector.x };
    }

    [[nodiscard]] inline constexpr Vec2 min_components(const Vec2& left, const Vec2& right)
    {
        return { left.x < right.x ? left.x : right.x, left.y < right.y ? left.y : right.y };
    }

    [[nodiscard]] inline constexpr Vec2 max_components(const Vec2& left, const Vec2& right)
    {
        return { left.x > right.x ? left.x : right.x, left.y > right.y ? left.y : right.y };
    }

    [[nodiscard]] inline constexpr Vec2 lerp(const Vec2& start, const Vec2& end, Real fraction)
    {
        return start + (end - start) * fraction;
    }

    [[nodiscard]] inline bool is_finite(const Vec2& vector)
    {
        return is_finite(vector.x) && is_finite(vector.y);
    }

} // namespace rigidbodies::math
