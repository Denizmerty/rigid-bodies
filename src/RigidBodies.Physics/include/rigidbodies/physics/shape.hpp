#pragma once

#include <rigidbodies/math/aabb.hpp>
#include <rigidbodies/math/polygon.hpp>
#include <rigidbodies/math/transform2.hpp>
#include <rigidbodies/physics/mass_properties.hpp>

#include <memory>
#include <vector>

namespace rigidbodies::physics
{

    enum class ShapeKind
    {
        circle,
        convex_polygon,
        segment
    };

    // Collision geometry in a local frame, independent of any body that uses it. Several bodies
    // can share one shape.
    //
    // The interface provides bounds, mass distribution, a support function and containment tests.
    // Generic narrow-phase routines use the support function to work across shape types.
    class Shape
    {
    public:
        Shape() = default;
        Shape(const Shape&) = default;
        Shape(Shape&&) = default;
        Shape& operator=(const Shape&) = default;
        Shape& operator=(Shape&&) = default;
        virtual ~Shape() = default;

        // A geometric category, not a concrete-type tag. Custom implementations may share a kind;
        // specialized code must check the actual type before using built-in-only accessors.
        [[nodiscard]] virtual ShapeKind kind() const = 0;

        [[nodiscard]] virtual std::shared_ptr<Shape> clone() const = 0;

        // World-space bounds for the shape placed by the given transform.
        [[nodiscard]] virtual math::Aabb compute_bounds(const math::Transform2& transform) const = 0;

        // Mass properties in the local frame for a uniform areal density, in kilograms per square
        // metre. Bodies convert a material density and a nominal thickness into that value.
        [[nodiscard]] virtual MassProperties compute_mass_properties(Real areal_density_kg_m2) const = 0;

        // Farthest point of the shape along a local-frame direction.
        [[nodiscard]] virtual math::Vec2 support_point(const math::Vec2& local_direction) const = 0;

        [[nodiscard]] virtual bool contains_local_point(const math::Vec2& local_point) const = 0;

        // Radius of the smallest circle centred on the local origin that encloses the shape. Used
        // for conservative early-outs and for camera framing.
        [[nodiscard]] virtual Real bounding_radius() const = 0;
    };

    using ShapePtr = std::shared_ptr<Shape>;

    class CircleShape final : public Shape
    {
    public:
        explicit CircleShape(Real radius_m, const math::Vec2& local_center_m = {});

        [[nodiscard]] ShapeKind kind() const override;
        [[nodiscard]] std::shared_ptr<Shape> clone() const override;
        [[nodiscard]] math::Aabb compute_bounds(const math::Transform2& transform) const override;
        [[nodiscard]] MassProperties compute_mass_properties(Real areal_density_kg_m2) const override;
        [[nodiscard]] math::Vec2 support_point(const math::Vec2& local_direction) const override;
        [[nodiscard]] bool contains_local_point(const math::Vec2& local_point) const override;
        [[nodiscard]] Real bounding_radius() const override;

        [[nodiscard]] Real radius_m() const;
        [[nodiscard]] const math::Vec2& local_center_m() const;

    private:
        Real radius_m_ {};
        math::Vec2 local_center_m_ {};
    };

    // A convex outline wound counter-clockwise. Concave authored outlines are decomposed into a
    // set of these before they reach a body.
    class ConvexPolygonShape final : public Shape
    {
    public:
        explicit ConvexPolygonShape(std::vector<math::Vec2> vertices);

        // Axis-aligned box centred on the local origin, the most common authored primitive.
        [[nodiscard]] static ConvexPolygonShape box(Real width_m, Real height_m);

        // Regular polygon inscribed in a circle of the given radius.
        [[nodiscard]] static ConvexPolygonShape regular(int side_count, Real circumradius_m);

        [[nodiscard]] ShapeKind kind() const override;
        [[nodiscard]] std::shared_ptr<Shape> clone() const override;
        [[nodiscard]] math::Aabb compute_bounds(const math::Transform2& transform) const override;
        [[nodiscard]] MassProperties compute_mass_properties(Real areal_density_kg_m2) const override;
        [[nodiscard]] math::Vec2 support_point(const math::Vec2& local_direction) const override;
        [[nodiscard]] bool contains_local_point(const math::Vec2& local_point) const override;
        [[nodiscard]] Real bounding_radius() const override;

        [[nodiscard]] const std::vector<math::Vec2>& vertices() const;
        [[nodiscard]] const std::vector<math::Vec2>& normals() const;

    private:
        std::vector<math::Vec2> vertices_;
        std::vector<math::Vec2> normals_;
        Real bounding_radius_ {};
    };

    // A thin two-point segment. Segments carry no area, so they are only meaningful as static
    // geometry such as a floor, a ramp, or a wall.
    class SegmentShape final : public Shape
    {
    public:
        SegmentShape(const math::Vec2& start_m, const math::Vec2& end_m);

        [[nodiscard]] ShapeKind kind() const override;
        [[nodiscard]] std::shared_ptr<Shape> clone() const override;
        [[nodiscard]] math::Aabb compute_bounds(const math::Transform2& transform) const override;
        [[nodiscard]] MassProperties compute_mass_properties(Real areal_density_kg_m2) const override;
        [[nodiscard]] math::Vec2 support_point(const math::Vec2& local_direction) const override;
        [[nodiscard]] bool contains_local_point(const math::Vec2& local_point) const override;
        [[nodiscard]] Real bounding_radius() const override;

        [[nodiscard]] const math::Vec2& start_m() const;
        [[nodiscard]] const math::Vec2& end_m() const;

    private:
        math::Vec2 start_m_ {};
        math::Vec2 end_m_ {};
    };

    [[nodiscard]] ShapePtr make_circle(Real radius_m, const math::Vec2& local_center_m = {});
    [[nodiscard]] ShapePtr make_box(Real width_m, Real height_m);
    [[nodiscard]] ShapePtr make_regular_polygon(int side_count, Real circumradius_m);
    [[nodiscard]] ShapePtr make_segment(const math::Vec2& start_m, const math::Vec2& end_m);

} // namespace rigidbodies::physics
