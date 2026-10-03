#pragma once

#include <rigidbodies/physics/material.hpp>
#include <rigidbodies/physics/shape.hpp>

#include <cstdint>
#include <optional>

namespace rigidbodies::physics
{

    struct AuthoredPartDefinition;

    // Bit masks deciding which colliders are eligible to collide. A pair is tested only when each
    // side belongs to a category the other side accepts.
    struct CollisionFilter
    {
        std::uint32_t category { 0x0001 };
        std::uint32_t mask { 0xFFFF };

        // Pairs sharing a non-zero group always collide when the group is positive and never
        // collide when it is negative, overriding the category test. This is how the parts of one
        // authored object are kept from colliding with each other.
        std::int32_t group { 0 };
    };

    [[nodiscard]] bool should_collide(const CollisionFilter& left, const CollisionFilter& right);

    // One piece of collision geometry attached to a body. Compound bodies carry several colliders,
    // each with its own shape, placement and material. The body combines their mass contributions.
    struct Collider
    {
        ShapePtr shape;

        // Placement of the shape within the body frame.
        math::Transform2 local_transform;

        Material material { materials::oak_wood() };

        CollisionFilter filter {};

        // Out-of-plane thickness in metres. Mass is shape area times thickness times bulk density,
        // so planar objects can use familiar material densities.
        Real depth_m { 0.05 };

        // Overrides the drag coefficient of the material when present. Drag depends far more on
        // profile than on what an object is made of.
        std::optional<Real> drag_coefficient_override;

        // A sensor reports overlaps without generating contact impulses.
        bool is_sensor { false };

        // Per-part bulk density, independent of the material's contact and drag properties.
        // Zero means no material; negative or non-finite mass parameters are rejected.
        std::optional<Real> density_override_kg_m3;

        // Inward wall thickness of a hollow cross-section. Absent means solid; zero means no
        // material; a thickness filling the profile means solid. Circles produce concentric
        // tubes and convex polygons use parallel inward-offset edges. This changes mass only:
        // collision, picking, and rendering continue to use the original convex outer envelope.
        std::optional<Real> shell_thickness_m;

        // Immutable logical source shared by every convex cell of one authored part. Null marks
        // ordinary primitive geometry. Keeping this identity separates logical parts from their
        // collision decomposition and lets rendering reuse the cached smooth exterior.
        std::shared_ptr<const AuthoredPartDefinition> authored_part;

        [[nodiscard]] Real effective_density_kg_m3() const;

        [[nodiscard]] Real areal_density_kg_m2() const;

        [[nodiscard]] Real effective_drag_coefficient() const;

        // Mass contribution in the body frame, already placed by the local transform.
        [[nodiscard]] MassProperties compute_mass_properties() const;

        [[nodiscard]] math::Aabb compute_bounds(const math::Transform2& body_transform) const;
    };

} // namespace rigidbodies::physics
