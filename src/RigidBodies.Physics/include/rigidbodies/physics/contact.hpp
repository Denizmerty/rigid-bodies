#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/education_accounting.hpp>

#include <array>
#include <cstddef>

namespace rigidbodies::physics
{

    // Identifies the feature pair that produced a contact point. The solver uses it to match points
    // across steps and reuse accumulated impulses, helping stacks settle with less jitter.
    struct ContactFeatureId
    {
        std::uint16_t incoming_edge { 0 };
        std::uint16_t outgoing_edge { 0 };
        std::uint16_t incoming_vertex { 0 };
        std::uint16_t outgoing_vertex { 0 };

        [[nodiscard]] std::uint64_t key() const;
    };

    struct ContactPoint
    {
        math::Vec2 world_position_m {};

        // Negative when the shapes overlap. The solver pushes the pair apart only for the portion
        // of the overlap beyond an allowed slop, so that resting contact does not oscillate.
        Real separation_m {};

        ContactFeatureId feature {};

        // Impulse magnitudes carried over between steps for warm starting.
        Real normal_impulse_n_s {};
        Real tangent_impulse_n_s {};
        Real rolling_impulse_n_m_s {};
        Real spinning_impulse_n_m_s {};

        // World converts the narrow-phase witnesses to body-local anchors before cache matching.
        // The solver retains them while the bodies move so position correction uses fresh poses.
        math::Vec2 local_anchor_first_m {};
        math::Vec2 local_anchor_second_m {};
        bool body_anchors_valid { false };
        Real pre_solve_normal_velocity_m_s {};
        Real speculative_velocity_bias_m_s {};
        // A speculative speed cap can remove incoming velocity before geometric touch. Carry
        // that impact speed to the first touching solve so restitution is not lost to the cap.
        Real pending_impact_speed_m_s {};
    };

    // A contact manifold is the small set of points through which two touching shapes exchange
    // impulse, together with the single normal shared by those points. Two points are enough to
    // describe any contact between convex planar shapes.
    inline constexpr std::size_t maximum_manifold_points = 2;

    struct ContactManifold
    {
        BodyId first;
        BodyId second;

        // Index of the collider within each body, so that compound bodies resolve per part.
        std::size_t first_collider { 0 };
        std::size_t second_collider { 0 };

        // Points from the first body towards the second.
        math::Vec2 normal {};

        std::array<ContactPoint, maximum_manifold_points> points {};
        std::size_t point_count { 0 };

        ContactMaterial material {};

        // Sensor overlaps are reported but never solved.
        bool is_sensor { false };
        bool is_speculative { false };

        // Reset by prepare; sums actual applied impulse increments, including warm starting.
        CollisionEnergyAccounting energy;
        math::Vec2 applied_impulse_on_second_n_s {};
        Real applied_angular_impulse_first_n_m_s {}, applied_angular_impulse_second_n_m_s {};
        Real applied_normal_impulse_n_s {}, applied_tangent_impulse_n_s {};
        // The angular part of the normal impulses alone; the rest of the angular impulse came
        // from friction. World::step uses the split to measure each kind's work.
        Real applied_normal_angular_impulse_first_n_m_s {}, applied_normal_angular_impulse_second_n_m_s {};

        [[nodiscard]] bool is_empty() const
        {
            return point_count == 0;
        }
    };

    // A pair of colliders whose bounds overlap, produced by the broad phase and consumed by the
    // narrow phase.
    struct BroadPhasePair
    {
        BodyId first;
        BodyId second;
        std::size_t first_collider { 0 };
        std::size_t second_collider { 0 };
    };

} // namespace rigidbodies::physics
