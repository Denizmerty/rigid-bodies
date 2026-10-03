#pragma once

#include <rigidbodies/math/rotation2.hpp>

namespace rigidbodies::math
{

    // Rigid placement of a local frame within the world frame: a rotation followed by a
    // translation. Every shape stores its geometry in a local frame and is placed through one of
    // these, which keeps compound bodies and shape authoring straightforward.
    struct Transform2
    {
        Vec2 translation {};
        Rotation2 rotation {};

        constexpr Transform2() = default;

        constexpr Transform2(const Vec2& translation_value, const Rotation2& rotation_value) : translation(translation_value), rotation(rotation_value)
        {
        }

        [[nodiscard]] static Transform2 from_angle(const Vec2& translation_value, Real radians)
        {
            return { translation_value, Rotation2 { radians } };
        }
    };

    [[nodiscard]] inline constexpr Vec2 transform_point(const Transform2& transform, const Vec2& local_point)
    {
        return rotate(transform.rotation, local_point) + transform.translation;
    }

    [[nodiscard]] inline constexpr Vec2 inverse_transform_point(const Transform2& transform, const Vec2& world_point)
    {
        return rotate_inverse(transform.rotation, world_point - transform.translation);
    }

    // Directions carry no translation, so only the rotation applies.
    [[nodiscard]] inline constexpr Vec2 transform_direction(const Transform2& transform, const Vec2& local_direction)
    {
        return rotate(transform.rotation, local_direction);
    }

    [[nodiscard]] inline constexpr Vec2 inverse_transform_direction(const Transform2& transform, const Vec2& world_direction)
    {
        return rotate_inverse(transform.rotation, world_direction);
    }

    [[nodiscard]] inline constexpr Transform2 concatenate(const Transform2& outer, const Transform2& inner)
    {
        return { transform_point(outer, inner.translation), concatenate(outer.rotation, inner.rotation) };
    }

    [[nodiscard]] inline constexpr Transform2 inverse(const Transform2& transform)
    {
        const auto inverse_rotation = inverse(transform.rotation);
        return { rotate(inverse_rotation, -transform.translation), inverse_rotation };
    }

} // namespace rigidbodies::math
