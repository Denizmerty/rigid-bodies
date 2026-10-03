#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/shape_authoring.hpp>

namespace rigidbodies::physics
{
    class World;
    struct ShapeDocument;

    struct AuthoredPartDefinition
    {
        std::shared_ptr<const AuthoredShape> shape;
        math::Transform2 local_transform;
        Material material { materials::oak_wood() };
        Real depth_m { 0.05 };
        CollisionFilter filter;
        std::optional<Real> density_override_kg_m3;
        std::optional<Real> drag_coefficient_override;
        bool is_sensor { false };
        std::string name;
        // Imported document metadata and additive fields belong to the logical part, so editing,
        // assembly, separation, and snapshots retain them alongside the authoritative geometry.
        std::shared_ptr<const ShapeDocument> source_document;
    };

    using AuthoredPartPtr = std::shared_ptr<const AuthoredPartDefinition>;

    // Unique logical parts in collider order, regardless of each part's convex cell count.
    [[nodiscard]] std::vector<AuthoredPartPtr> authored_parts(const RigidBody& body);

    // All mutations validate and stage their complete result before publishing it. Successful
    // edits invalidate body pointers, but surviving IDs and force-generator instances remain.
    // The source/options are authoritative: committing rebuilds a detached immutable cache once;
    // stepping and rendering never tessellate. placement.colliders must be empty.
    BodyId create_authored_body(World& world, BodyDefinition placement, const std::vector<AuthoredPartDefinition>& parts, std::string stable_key = {});

    // Replaces only one source shape, preserving its placement/material and the live BodyId.
    // The rigid velocity field is preserved when the new geometry moves the centre of mass.
    void edit_authored_part(World& world, BodyId body, std::size_t part_index, std::shared_ptr<const AuthoredShape> shape);

    // Assembly preserves current world geometry, each source material, and total linear/angular
    // momentum with the resulting geometry-defined mass. Existing ordered overlap ownership
    // still applies. Bodies must be authored dynamic bodies with compatible motion settings,
    // without mass overrides, pending loads, local generators, springs, or joints.
    BodyId assemble_authored_bodies(World& world, const std::vector<BodyId>& bodies, std::string name = "Assembly", std::string stable_key = {});

    // Each complete logical part becomes a body with the parent's rigid velocity at its own COM.
    // Overlapping parts regain their independent full volume, so splitting overlap can change
    // total material mass. Convex collision cells never become separate physical objects.
    [[nodiscard]] std::vector<BodyId> separate_authored_body(World& world, BodyId body);
}
