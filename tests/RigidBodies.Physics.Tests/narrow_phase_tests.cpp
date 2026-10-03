#include <rigidbodies/physics/narrow_phase.hpp>
#include <rigidbodies/physics/convex_distance.hpp>
#include "test_framework.hpp"
#include <algorithm>
#include <cmath>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;
    using rigidbodies::math::Transform2;

    class CustomBox final : public Shape
    {
    public:
        ShapeKind kind() const override
        {
            return ShapeKind::circle;
        }
        ShapePtr clone() const override
        {
            return std::make_shared<CustomBox>(*this);
        }
        rigidbodies::math::Aabb compute_bounds(const Transform2& transform) const override
        {
            return box_.compute_bounds(transform);
        }
        MassProperties compute_mass_properties(Real density) const override
        {
            return box_.compute_mass_properties(density);
        }
        Vec2 support_point(const Vec2& direction) const override
        {
            return box_.support_point(direction);
        }
        bool contains_local_point(const Vec2& point) const override
        {
            return box_.contains_local_point(point);
        }
        Real bounding_radius() const override
        {
            return box_.bounding_radius();
        }

    private:
        ConvexPolygonShape box_ { ConvexPolygonShape::box(2.0, 2.0) };
    };

    NarrowPhaseQuery query(const Shape& a, const Shape& b, Vec2 offset = {})
    {
        NarrowPhaseQuery result;
        result.first_shape = &a;
        result.second_shape = &b;
        result.second_transform = Transform2::from_angle(offset, 0.0);
        return result;
    }

    void check_anchors(const NarrowPhaseQuery& input, const ContactManifold& manifold)
    {
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(manifold.normal), 1.0, 1.0e-12, "the manifold normal has unit length");
        for (std::size_t index = 0; index < manifold.point_count; ++index)
        {
            const auto& point = manifold.points[index];
            const auto a = rigidbodies::math::transform_point(input.first_transform, point.local_anchor_first_m);
            const auto b = rigidbodies::math::transform_point(input.second_transform, point.local_anchor_second_m);
            RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::dot(b - a, manifold.normal), point.separation_m, 1.0e-10, "local anchors preserve the signed separation");
            RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::distance((a + b) * 0.5, point.world_position_m), 0.0, 1.0e-10, "the displayed contact is the witness midpoint");
        }
    }

    RIGIDBODIES_TEST("circle manifolds distinguish exact overlap touching separation and speculative margin")
    {
        const CircleShape a { 0.5 };
        const CircleShape b { 0.25 };
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        auto input = query(a, b, { 0.6, 0.0 });
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold) && manifold.point_count == 1, "overlapping circles produce one contact");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.15, 1.0e-12, "circle penetration is exact");
        check_anchors(input, manifold);
        input.second_transform.translation.x = 0.75;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "tangent circles count as touching");
        input.second_transform.translation.x = 0.8;
        RIGIDBODIES_EXPECT(!narrow.collide(input, manifold) && manifold.is_empty(), "a real gap clears the old manifold");
        input.contact_margin_m = 0.06;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold) && manifold.is_speculative, "margin produces an explicitly speculative contact");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, 0.05, 1.0e-12, "speculation retains positive separation");
    }

    RIGIDBODIES_TEST("polygon clipping produces two stable contacts on a shared face")
    {
        const auto a = make_box(2.0, 2.0);
        const auto b = make_box(1.0, 1.0);
        auto input = query(*a, *b, { 0.1, 1.4 });
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold) && manifold.point_count == 2, "a box face clips to two contacts");
        check_anchors(input, manifold);
        RIGIDBODIES_EXPECT_NEAR(manifold.normal.y, 1.0, 1.0e-12, "normal points from the lower box to the upper box");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.1, 1.0e-12, "face depth is exact");
        const auto first_key = manifold.points[0].feature.key();
        const auto second_key = manifold.points[1].feature.key();
        RIGIDBODIES_EXPECT(first_key != second_key, "the two contacts have distinct persistent features");
        input.second_transform.translation.x += 0.01;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold) && manifold.point_count == 2, "sliding along a face retains two contacts");
        RIGIDBODIES_EXPECT(manifold.points[0].feature.key() == first_key && manifold.points[1].feature.key() == second_key,
            "small tangential motion preserves feature identities");
    }

    RIGIDBODIES_TEST("circle polygon dispatch handles edges vertices and full containment")
    {
        const CircleShape circle { 0.25 };
        const auto box = make_box(2.0, 2.0);
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        auto input = query(circle, *box);
        input.first_transform.translation = { 1.2, 0.0 };
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "a circle overlaps the box edge");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.05, 1.0e-12, "edge penetration is exact");
        input.first_transform.translation = { 1.1, 1.1 };
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "a circle overlaps a corner Voronoi region");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, std::sqrt(0.02) - 0.25, 1.0e-12, "corner penetration uses Euclidean distance");
        input.first_transform.translation = { 0.2, 0.0 };
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "a fully contained circle has a separating contact");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -1.05, 1.0e-12, "containment includes radius and distance to the enclosing face");
        RIGIDBODIES_EXPECT_NEAR(manifold.normal.x, -1.0, 1.0e-12, "the contained circle is pushed toward its nearest exit");
        check_anchors(input, manifold);
    }

    RIGIDBODIES_TEST("two-sided segments collide with circle faces endpoints and polygon faces")
    {
        const SegmentShape segment { { -1.0, 0.0 }, { 1.0, 0.0 } };
        const CircleShape circle { 0.2 };
        const auto box = make_box(0.6, 0.6);
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        for (const auto y : { -0.1, 0.1 })
        {
            auto input = query(segment, circle, { 0.0, y });
            RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "both sides of a segment generate circle contacts");
            RIGIDBODIES_EXPECT(manifold.normal.y * y > 0.0, "the segment normal faces the circle on either side");
            RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.1, 1.0e-12, "segment-circle depth is exact");
            check_anchors(input, manifold);
        }
        auto endpoint = query(segment, circle, { 1.1, 0.0 });
        RIGIDBODIES_EXPECT(narrow.collide(endpoint, manifold), "segment endpoints are included");
        RIGIDBODIES_EXPECT_NEAR(manifold.normal.x, 1.0, 1.0e-12, "endpoint contact normal is radial");
        auto face = query(segment, *box, { 0.0, 0.2 });
        RIGIDBODIES_EXPECT(narrow.collide(face, manifold) && manifold.point_count == 2, "a box resting across a segment gets two support points");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.1, 1.0e-12, "segment-polygon penetration is exact");
    }

    RIGIDBODIES_TEST("segment intersections and collinear gaps do not depend on endpoint ordering")
    {
        const SegmentShape horizontal { { -1.0, 0.0 }, { 1.0, 0.0 } };
        const SegmentShape reversed { { 1.0, 0.0 }, { -1.0, 0.0 } };
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        auto input = query(horizontal, reversed, { 3.0, 0.0 });
        RIGIDBODIES_EXPECT(!narrow.collide(input, manifold), "collinear but disjoint segments do not collide");
        input.second_transform = Transform2::from_angle({}, rigidbodies::math::half_pi);
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "crossing zero-width segments collide");
        check_anchors(input, manifold);
        input.second_transform = {};
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "coincident reversed segments stay well-defined");
    }

    RIGIDBODIES_TEST("manifolds respect swapping and a common rigid transform")
    {
        const CircleShape circle { 0.4 };
        const auto box = make_box(1.0, 1.0);
        auto input = query(circle, *box, { 0.7, 0.1 });
        CollisionNarrowPhase narrow;
        ContactManifold original;
        RIGIDBODIES_EXPECT(narrow.collide(input, original), "the original geometry overlaps");
        auto reverse = input;
        std::swap(reverse.first_shape, reverse.second_shape);
        std::swap(reverse.first_transform, reverse.second_transform);
        ContactManifold swapped;
        RIGIDBODIES_EXPECT(narrow.collide(reverse, swapped), "reversing the query retains overlap");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(original.normal + swapped.normal), 0.0, 1.0e-12, "normal reverses under swapping");
        RIGIDBODIES_EXPECT_NEAR(original.points[0].separation_m, swapped.points[0].separation_m, 1.0e-12, "depth remains unchanged under swapping");
        const auto placement = Transform2::from_angle({ 2.0, -3.0 }, 0.6);
        input.first_transform = rigidbodies::math::concatenate(placement, input.first_transform);
        input.second_transform = rigidbodies::math::concatenate(placement, input.second_transform);
        ContactManifold moved;
        RIGIDBODIES_EXPECT(narrow.collide(input, moved), "a rigid transform retains overlap");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::distance(moved.points[0].world_position_m, rigidbodies::math::transform_point(placement, original.points[0].world_position_m)),
            0.0,
            1.0e-12,
            "contact point follows the common placement");
        check_anchors(input, moved);
    }

    RIGIDBODIES_TEST("GJK and EPA agree with specialized overlap depths away from ambiguous ties")
    {
        const auto box = make_box(2.0, 2.0);
        const CircleShape circle { 0.5 };
        CollisionNarrowPhase narrow;
        for (const auto center : { Vec2 { 1.2, 0.2 }, Vec2 { 1.2, 1.2 }, Vec2 { 0.3, 0.1 } })
        {
            auto input = query(*box, circle, center);
            ContactManifold manifold;
            RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "the specialized pair overlaps");
            const auto distance = convex_distance(*box, {}, circle, input.second_transform);
            RIGIDBODIES_EXPECT(distance.valid && distance.intersecting, "the support-mapping pair also overlaps");
            RIGIDBODIES_EXPECT_NEAR(distance.penetration_depth_m, -manifold.points[0].separation_m, 1.0e-6, "specialized and support-mapping depths agree");
        }
    }

    RIGIDBODIES_TEST("rotated polygon SAT and support mapping agree across a deterministic placement sweep")
    {
        const auto first = make_box(1.4, 0.8);
        const auto second = make_regular_polygon(5, 0.6);
        CollisionNarrowPhase narrow;
        for (int x = -7; x <= 7; ++x)
        {
            for (int y = -7; y <= 7; ++y)
            {
                auto input = query(*first, *second);
                input.first_transform = Transform2::from_angle({ 0.1, -0.05 }, 0.31);
                input.second_transform = Transform2::from_angle({ static_cast<Real>(x) * 0.2, static_cast<Real>(y) * 0.2 }, 0.07 * x - 0.13 * y);
                ContactManifold manifold;
                const auto colliding = narrow.collide(input, manifold);
                const auto distance = convex_distance(*first, input.first_transform, *second, input.second_transform);
                RIGIDBODIES_EXPECT(distance.valid, "GJK returns valid geometry for ordinary rotated polygons");
                RIGIDBODIES_EXPECT(colliding == (distance.intersecting || distance.distance_m < 1.0e-8),
                    "SAT clipping and GJK agree whether the polygons touch");
                if (colliding)
                    check_anchors(input, manifold);
            }
        }
    }

    RIGIDBODIES_TEST("speculative corner contacts use Euclidean gap rather than independent face margins")
    {
        const auto box = make_box(2.0, 2.0);
        auto input = query(*box, *box, { 2.08, 2.08 });
        input.contact_margin_m = 0.1;
        ContactManifold manifold;
        CollisionNarrowPhase narrow;
        RIGIDBODIES_EXPECT(!narrow.collide(input, manifold), "diagonal Euclidean gap exceeds the margin despite both face gaps fitting");
        input.contact_margin_m = 0.12;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold) && manifold.point_count == 1, "a sufficiently large margin includes the unique nearest corner pair");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, std::sqrt(2.0) * 0.08, 1.0e-10, "corner separation is the true Euclidean distance");
    }

    RIGIDBODIES_TEST("custom shape subclasses and repeated polygon vertices safely use support mapping")
    {
        const CustomBox custom;
        const CircleShape circle { 0.5 };
        auto input = query(custom, circle, { 1.3, 0.1 });
        CollisionNarrowPhase narrow;
        ContactManifold manifold;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "a custom box with a circle kind tag uses virtual support mapping");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.2, 1.0e-7, "custom-box penetration matches its actual geometry");
        check_anchors(input, manifold);
        const ConvexPolygonShape repeated { { { -1.0, -1.0 }, { 1.0, -1.0 }, { 1.0, -1.0 }, { 1.0, 1.0 }, { -1.0, 1.0 } } };
        input.first_shape = &repeated;
        RIGIDBODIES_EXPECT(narrow.collide(input, manifold), "a duplicate vertex does not create an invalid zero face normal");
        RIGIDBODIES_EXPECT_NEAR(manifold.points[0].separation_m, -0.2, 1.0e-7, "duplicate vertices do not alter the convex geometry");
        check_anchors(input, manifold);
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
