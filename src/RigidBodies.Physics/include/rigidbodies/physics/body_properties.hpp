#pragma once

#include <rigidbodies/physics/world.hpp>

#include <optional>

namespace rigidbodies::physics
{
    struct BodyPropertyEdit
    {
        std::optional<Real> mass_kg;
        bool use_density_mass { false };
        // Applies to all logical parts; resets per-collider density/drag overrides and derives mass.
        std::optional<Material> material;
        std::optional<math::Vec2> linear_velocity_m_s;
        std::optional<Real> angular_velocity_rad_s;
    };

    // Validate on a detached candidate before changing the live body. The identifier and attached
    // forces/joints survive. Editing is an external intervention, not a conservation experiment.
    // Mass and velocity edits require a dynamic body; material edits also support static supports.
    void edit_body_properties(World& world, BodyId id, const BodyPropertyEdit& edit);
}
