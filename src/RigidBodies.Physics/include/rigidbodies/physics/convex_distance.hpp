#pragma once

#include <rigidbodies/physics/shape.hpp>

namespace rigidbodies::physics
{
    struct ConvexDistanceResult
    {
        bool valid { false };
        bool intersecting { false };
        Real distance_m { 0.0 };
        Real penetration_depth_m { 0.0 };
        math::Vec2 normal { 1.0, 0.0 };
        math::Vec2 first_point_m {};
        math::Vec2 second_point_m {};
    };

    // Bounded GJK distance and EPA penetration for convex support mappings. The normal points
    // first -> second. Separated witnesses satisfy second-first = normal*distance; penetrating
    // witnesses satisfy first-second = normal*depth, to numerical tolerance. Touching counts as
    // intersection with zero depth. Invalid transforms/support outputs return valid=false.
    [[nodiscard]] ConvexDistanceResult convex_distance(const Shape& first, const math::Transform2& first_transform,
        const Shape& second, const math::Transform2& second_transform);
}
