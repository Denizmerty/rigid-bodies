#pragma once

#include <rigidbodies/math/vec2.hpp>

namespace rigidbodies::math
{

    // A planar rotation held as a sine/cosine pair. Bodies store an unwrapped angle so that
    // accumulated revolutions stay observable for educational read-outs; this type is the cached
    // trigonometric form used whenever points are transformed.
    struct Rotation2
    {
        Real sine {};
        Real cosine { 1.0 };

        constexpr Rotation2() = default;

        explicit Rotation2(Real radians) : sine(std::sin(radians)), cosine(std::cos(radians))
        {
        }

        constexpr Rotation2(Real sine_value, Real cosine_value) : sine(sine_value), cosine(cosine_value)
        {
        }

        [[nodiscard]] Real angle() const
        {
            return std::atan2(sine, cosine);
        }

        [[nodiscard]] constexpr Vec2 x_axis() const
        {
            return { cosine, sine };
        }

        [[nodiscard]] constexpr Vec2 y_axis() const
        {
            return { -sine, cosine };
        }
    };

    [[nodiscard]] inline constexpr Rotation2 inverse(const Rotation2& rotation)
    {
        return { -rotation.sine, rotation.cosine };
    }

    [[nodiscard]] inline constexpr Vec2 rotate(const Rotation2& rotation, const Vec2& vector)
    {
        return { rotation.cosine * vector.x - rotation.sine * vector.y, rotation.sine * vector.x + rotation.cosine * vector.y };
    }

    [[nodiscard]] inline constexpr Vec2 rotate_inverse(const Rotation2& rotation, const Vec2& vector)
    {
        return { rotation.cosine * vector.x + rotation.sine * vector.y, -rotation.sine * vector.x + rotation.cosine * vector.y };
    }

    [[nodiscard]] inline constexpr Rotation2 concatenate(const Rotation2& first, const Rotation2& second)
    {
        return { first.sine * second.cosine + first.cosine * second.sine, first.cosine * second.cosine - first.sine * second.sine };
    }

} // namespace rigidbodies::math
