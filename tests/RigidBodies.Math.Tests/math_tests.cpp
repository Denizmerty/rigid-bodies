#include <rigidbodies/math/aabb.hpp>
#include <rigidbodies/math/polygon.hpp>
#include <rigidbodies/math/transform2.hpp>

#include "test_framework.hpp"

#include <vector>

namespace
{

    using namespace rigidbodies::math;

    std::vector<Vec2> unit_square()
    {
        return { { 0.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 1.0 }, { 0.0, 1.0 } };
    }

    RIGIDBODIES_TEST("vector cross product measures signed area")
    {
        RIGIDBODIES_EXPECT_NEAR(cross(Vec2 { 1.0, 0.0 }, Vec2 { 0.0, 1.0 }), 1.0, 1.0e-12, "counter-clockwise pair is positive");
        RIGIDBODIES_EXPECT_NEAR(cross(Vec2 { 0.0, 1.0 }, Vec2 { 1.0, 0.0 }), -1.0, 1.0e-12, "clockwise pair is negative");
        RIGIDBODIES_EXPECT_NEAR(cross(Vec2 { 2.0, 2.0 }, Vec2 { 4.0, 4.0 }), 0.0, 1.0e-12, "parallel vectors span nothing");
    }

    RIGIDBODIES_TEST("normalizing a degenerate vector yields zero rather than a division by zero")
    {
        const auto result = normalized(Vec2 { 0.0, 0.0 });
        RIGIDBODIES_EXPECT(result == Vec2 { 0.0, 0.0 }, "degenerate input returns the zero vector");
    }

    RIGIDBODIES_TEST("a rotation and its inverse return a point unchanged")
    {
        const Rotation2 rotation { degrees_to_radians(37.0) };
        const Vec2 point { 0.4, -1.3 };
        const auto round_trip = rotate_inverse(rotation, rotate(rotation, point));
        RIGIDBODIES_EXPECT_NEAR(round_trip.x, point.x, 1.0e-12, "x component survives the round trip");
        RIGIDBODIES_EXPECT_NEAR(round_trip.y, point.y, 1.0e-12, "y component survives the round trip");
    }

    RIGIDBODIES_TEST("a transform and its inverse compose to the identity")
    {
        const auto transform = Transform2::from_angle({ 2.5, -0.75 }, degrees_to_radians(115.0));
        const Vec2 point { -0.3, 1.9 };
        const auto round_trip = inverse_transform_point(transform, transform_point(transform, point));
        RIGIDBODIES_EXPECT_NEAR(round_trip.x, point.x, 1.0e-12, "x component survives the round trip");
        RIGIDBODIES_EXPECT_NEAR(round_trip.y, point.y, 1.0e-12, "y component survives the round trip");
    }

    RIGIDBODIES_TEST("polygon area and centroid match the closed form for a unit square")
    {
        const auto square = unit_square();
        RIGIDBODIES_EXPECT_NEAR(signed_area(square), 1.0, 1.0e-12, "unit square has unit area");

        const auto middle = centroid(square);
        RIGIDBODIES_EXPECT_NEAR(middle.x, 0.5, 1.0e-12, "centroid sits at the middle in x");
        RIGIDBODIES_EXPECT_NEAR(middle.y, 0.5, 1.0e-12, "centroid sits at the middle in y");
    }

    RIGIDBODIES_TEST("second moment of area matches the closed form for a square")
    {
        // A square of side a has a polar second moment of a^4 / 6 about its centre.
        const auto square = unit_square();
        RIGIDBODIES_EXPECT_NEAR(second_moment_of_area(square), 1.0 / 6.0, 1.0e-12, "unit square polar second moment");
    }

    RIGIDBODIES_TEST("winding is normalised without changing the outline")
    {
        std::vector<Vec2> clockwise { { 0.0, 0.0 }, { 0.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 0.0 } };
        RIGIDBODIES_EXPECT(signed_area(clockwise) < 0.0, "input is wound clockwise");

        const auto corrected = ensure_counter_clockwise(clockwise);
        RIGIDBODIES_EXPECT_NEAR(signed_area(corrected), 1.0, 1.0e-12, "corrected outline encloses the same area");
    }

    RIGIDBODIES_TEST("convexity is reported for convex and concave outlines")
    {
        RIGIDBODIES_EXPECT(is_convex(unit_square()), "a square is convex");

        const std::vector<Vec2> arrow { { 0.0, 0.0 }, { 2.0, 0.0 }, { 1.0, 1.0 }, { 2.0, 2.0 }, { 0.0, 2.0 } };
        RIGIDBODIES_EXPECT(!is_convex(arrow), "an outline with a reflex corner is not convex");
    }

    RIGIDBODIES_TEST("support point returns the extreme vertex along a direction")
    {
        const auto square = unit_square();
        const auto corner = support_point(square, { 1.0, 1.0 });
        RIGIDBODIES_EXPECT(corner == Vec2 { 1.0, 1.0 }, "the far corner is the support point along the diagonal");
    }

    RIGIDBODIES_TEST("edge normals of a counter-clockwise outline point outward")
    {
        const auto square = unit_square();
        const auto normals = edge_normals(square);
        RIGIDBODIES_EXPECT(normals.size() == square.size(), "one normal per edge");

        const auto middle = centroid(square);
        for (std::size_t index = 0; index < normals.size(); ++index)
        {
            const auto edge_midpoint = (square[index] + square[(index + 1) % square.size()]) * 0.5;
            RIGIDBODIES_EXPECT(dot(normals[index], edge_midpoint - middle) > 0.0, "normal points away from the interior");
        }
    }

    RIGIDBODIES_TEST("an empty bounding box reports as empty and absorbs its first point")
    {
        Aabb bounds;
        RIGIDBODIES_EXPECT(bounds.is_empty(), "a default box is empty");
        RIGIDBODIES_EXPECT(!overlaps(bounds, bounds), "an empty box overlaps nothing");

        bounds.expand(Vec2 { 1.0, 2.0 });
        RIGIDBODIES_EXPECT(!bounds.is_empty(), "one point makes the box non-empty");
        RIGIDBODIES_EXPECT(bounds.contains(Vec2 { 1.0, 2.0 }), "the box contains the point it absorbed");
        RIGIDBODIES_EXPECT_NEAR(bounds.area(), 0.0, 1.0e-12, "a single point encloses no area");
    }

    RIGIDBODIES_TEST("bounding boxes overlap only when they intersect")
    {
        Aabb left;
        left.expand(Vec2 { 0.0, 0.0 });
        left.expand(Vec2 { 1.0, 1.0 });

        Aabb right;
        right.expand(Vec2 { 0.5, 0.5 });
        right.expand(Vec2 { 2.0, 2.0 });

        Aabb far_away;
        far_away.expand(Vec2 { 5.0, 5.0 });
        far_away.expand(Vec2 { 6.0, 6.0 });

        RIGIDBODIES_EXPECT(overlaps(left, right), "adjacent boxes overlap");
        RIGIDBODIES_EXPECT(!overlaps(left, far_away), "separated boxes do not overlap");
    }

    RIGIDBODIES_TEST("angles wrap into a half-open turn")
    {
        RIGIDBODIES_EXPECT_NEAR(wrap_angle(0.0), 0.0, 1.0e-12, "zero is unchanged");
        RIGIDBODIES_EXPECT_NEAR(wrap_angle(two_pi), 0.0, 1.0e-12, "a full turn wraps to zero");
        RIGIDBODIES_EXPECT_NEAR(wrap_angle(pi + 0.25), -pi + 0.25, 1.0e-12, "just past half a turn wraps negative");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
