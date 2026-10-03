#include <rigidbodies/physics/convex_distance.hpp>
#include "test_framework.hpp"
#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;
    using rigidbodies::math::Transform2;

    class WrappedShape final : public Shape
    {
    public:
        explicit WrappedShape(ShapePtr shape) : shape_(std::move(shape))
        {
        }
        ShapeKind kind() const override
        {
            return ShapeKind::circle;
        }
        ShapePtr clone() const override
        {
            return std::make_shared<WrappedShape>(shape_->clone());
        }
        rigidbodies::math::Aabb compute_bounds(const Transform2& t) const override
        {
            return shape_->compute_bounds(t);
        }
        MassProperties compute_mass_properties(Real density) const override
        {
            return shape_->compute_mass_properties(density);
        }
        Vec2 support_point(const Vec2& direction) const override
        {
            return shape_->support_point(direction);
        }
        bool contains_local_point(const Vec2& point) const override
        {
            return shape_->contains_local_point(point);
        }
        Real bounding_radius() const override
        {
            return shape_->bounding_radius();
        }

    private:
        ShapePtr shape_;
    };

    void check_witnesses(const ConvexDistanceResult& result, Real tolerance = 2.0e-7)
    {
        RIGIDBODIES_EXPECT(result.valid, "the query converges to valid geometry");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(result.normal), 1.0, 1.0e-12, "normal has unit length");
        const auto delta = result.second_point_m - result.first_point_m;
        const auto signed_distance = result.intersecting ? -result.penetration_depth_m : result.distance_m;
        RIGIDBODIES_EXPECT_NEAR(delta.x, result.normal.x * signed_distance, tolerance, "horizontal witnesses agree with signed distance");
        RIGIDBODIES_EXPECT_NEAR(delta.y, result.normal.y * signed_distance, tolerance, "vertical witnesses agree with signed distance");
    }

    RIGIDBODIES_TEST("GJK circle distance and EPA overlap match the analytical radii")
    {
        const CircleShape first { 0.7 };
        const CircleShape second { 0.4 };
        for (const auto x : { 3.0, 1.1, 0.8, 0.2 })
        {
            const auto result = convex_distance(first, {}, second, Transform2::from_angle({ x, 0.0 }, 0.0));
            check_witnesses(result);
            if (x > 1.1)
                RIGIDBODIES_EXPECT_NEAR(result.distance_m, x - 1.1, 2.0e-8, "separated circles use the exact surface gap");
            else
                RIGIDBODIES_EXPECT_NEAR(result.penetration_depth_m, 1.1 - x, 2.0e-7, "overlapping circles use the minimum separating translation");
        }
    }

    RIGIDBODIES_TEST("distance supports custom geometry without trusting the shape-kind tag")
    {
        const WrappedShape first { make_box(2.0, 2.0) };
        const WrappedShape second { make_box(2.0, 2.0) };
        const auto separated = convex_distance(first, {}, second, Transform2::from_angle({ 3.0, 3.0 }, 0.0));
        check_witnesses(separated);
        RIGIDBODIES_EXPECT_NEAR(separated.distance_m, std::sqrt(2.0), 1.0e-10, "diagonal box distance measures the corner gap");
        const auto overlap = convex_distance(first, {}, second, Transform2::from_angle({ 1.5, 0.0 }, 0.0));
        check_witnesses(overlap);
        RIGIDBODIES_EXPECT(overlap.intersecting, "custom convex support maps overlap");
        RIGIDBODIES_EXPECT_NEAR(overlap.penetration_depth_m, 0.5, 1.0e-10, "EPA finds the box overlap depth");
    }

    RIGIDBODIES_TEST("distance and witnesses are symmetric and covariant under rigid placement")
    {
        const auto first = make_box(1.2, 0.7);
        const CircleShape second { 0.25 };
        const auto a = Transform2::from_angle({ -0.4, 0.2 }, 0.3);
        const auto b = Transform2::from_angle({ 1.4, 0.8 }, -0.2);
        const auto result = convex_distance(*first, a, second, b);
        const auto reversed = convex_distance(second, b, *first, a);
        check_witnesses(result);
        check_witnesses(reversed);
        RIGIDBODIES_EXPECT_NEAR(result.distance_m, reversed.distance_m, 1.0e-8, "swapping shapes preserves distance");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(result.normal + reversed.normal), 0.0, 1.0e-7, "swapping shapes reverses the normal");
        const auto placement = Transform2::from_angle({ 7.0, -4.0 }, 0.71);
        const auto moved = convex_distance(*first, rigidbodies::math::concatenate(placement, a), second, rigidbodies::math::concatenate(placement, b));
        check_witnesses(moved);
        RIGIDBODIES_EXPECT_NEAR(moved.distance_m, result.distance_m, 1.0e-8, "rigid placement preserves distance");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::distance(moved.first_point_m, rigidbodies::math::transform_point(placement, result.first_point_m)), 0.0, 2.0e-7, "the first witness follows the common transform");
    }

    RIGIDBODIES_TEST("coincident circles and zero-area segment cases stay finite")
    {
        const WrappedShape circle { make_circle(1.0) };
        const auto coincident = convex_distance(circle, {}, circle, {});
        check_witnesses(coincident);
        RIGIDBODIES_EXPECT(coincident.intersecting, "coincident smooth shapes intersect");
        RIGIDBODIES_EXPECT_NEAR(coincident.penetration_depth_m, 2.0, 2.0e-3, "bounded EPA conservatively estimates the symmetric smooth penetration");
        const SegmentShape segment { { -1.0, 0.0 }, { 1.0, 0.0 } };
        const auto gap = convex_distance(segment, {}, segment, Transform2::from_angle({ 3.0, 0.0 }, 0.0));
        check_witnesses(gap);
        RIGIDBODIES_EXPECT_NEAR(gap.distance_m, 1.0, 1.0e-12, "parallel collinear segments retain their endpoint gap");
        const auto touch = convex_distance(segment, {}, segment, Transform2::from_angle({ 2.0, 0.0 }, 0.0));
        check_witnesses(touch);
        RIGIDBODIES_EXPECT(touch.intersecting && touch.penetration_depth_m == 0.0, "endpoint touching is a zero-depth intersection");
        const auto crossing = convex_distance(segment, {}, segment, Transform2::from_angle({}, rigidbodies::math::half_pi));
        check_witnesses(crossing);
        RIGIDBODIES_EXPECT(crossing.intersecting, "crossing segments intersect");
    }

    RIGIDBODIES_TEST("invalid transforms return an explicit invalid distance result")
    {
        const CircleShape circle { 1.0 };
        const auto invalid = Transform2::from_angle({ std::numeric_limits<Real>::infinity(), 0.0 }, 0.0);
        RIGIDBODIES_EXPECT(!convex_distance(circle, {}, circle, invalid).valid, "infinite placements never enter iterative geometry");
        RIGIDBODIES_EXPECT(!convex_distance(circle, {}, circle, Transform2::from_angle({}, std::numeric_limits<Real>::quiet_NaN())).valid,
            "invalid rotations are rejected");
    }

    RIGIDBODIES_TEST("off-axis support distances reproduce analytic circles over a deterministic sweep")
    {
        const WrappedShape first { make_circle(0.7, { 0.15, -0.1 }) };
        const WrappedShape second { make_circle(0.3, { -0.1, 0.2 }) };
        const auto a = Transform2::from_angle({ 0.2, 0.1 }, 0.3);
        for (int index = 1; index <= 180; ++index)
        {
            const auto angle = static_cast<Real>(index) * 0.173;
            const auto radius = 0.2 + static_cast<Real>(index % 17) * 0.11;
            const auto b = Transform2::from_angle({ radius * std::cos(angle), radius * std::sin(angle) }, angle);
            const auto center_a = rigidbodies::math::transform_point(a, { 0.15, -0.1 });
            const auto center_b = rigidbodies::math::transform_point(b, { -0.1, 0.2 });
            const auto signed_distance = rigidbodies::math::distance(center_a, center_b) - 1.0;
            const auto result = convex_distance(first, a, second, b);
            check_witnesses(result, 1.0e-6);
            RIGIDBODIES_EXPECT_NEAR(result.intersecting ? -result.penetration_depth_m : result.distance_m, signed_distance, 1.0e-6, "support-only circles reproduce the analytical signed distance at every orientation");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
