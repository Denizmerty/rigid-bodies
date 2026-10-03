#include <rigidbodies/physics/body_properties.hpp>
#include <rigidbodies/physics/authored_body.hpp>

#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace rigidbodies::physics
{
    namespace
    {
        void require(bool condition, const char* message)
        {
            if (!condition)
                throw std::invalid_argument(message);
        }
        void validate_material(const Material& material)
        {
            const auto nonnegative = [](Real value)
            {
                return math::is_finite(value) && value >= 0.0;
            };
            require(nonnegative(material.density_kg_m3) && nonnegative(material.restitution) && material.restitution <= 1.0 &&
                    nonnegative(material.static_friction) && nonnegative(material.kinetic_friction) && nonnegative(material.drag_coefficient) &&
                    nonnegative(material.rolling_friction_m) && nonnegative(material.spinning_friction_m) &&
                    math::is_finite(material.friction_axis_local) && nonnegative(material.friction_anisotropy_ratio),
                "Material properties must be finite and physically valid.");
        }
    }

    void edit_body_properties(World& world, BodyId id, const BodyPropertyEdit& edit)
    {
        auto* body = world.find_body(id);
        require(body != nullptr, "The selected body no longer exists.");
        const bool motion_or_mass = edit.mass_kg || edit.use_density_mass || edit.linear_velocity_m_s || edit.angular_velocity_rad_s;
        require(!motion_or_mass || body->type() == BodyType::dynamic_body, "Mass and velocity editing require a dynamic body.");
        require(body->type() != BodyType::kinematic_body, "Stop prescribed motion before editing physical properties.");
        require(!edit.mass_kg || (math::is_finite(*edit.mass_kg) && *edit.mass_kg > 0.0), "Mass must be finite and greater than zero.");
        const auto& limits = world.settings().limits;
        if (edit.linear_velocity_m_s)
            require(math::is_finite(*edit.linear_velocity_m_s) && std::hypot(edit.linear_velocity_m_s->x, edit.linear_velocity_m_s->y) <= limits.maximum_linear_speed_m_s,
                "Velocity exceeds the world's finite speed limit.");
        if (edit.angular_velocity_rad_s)
            require(math::is_finite(*edit.angular_velocity_rad_s) && std::abs(*edit.angular_velocity_rad_s) <= limits.maximum_angular_speed_rad_s &&
                    (!body->has_fixed_rotation() || *edit.angular_velocity_rad_s == 0.0),
                "Spin exceeds the speed limit or conflicts with fixed rotation.");

        auto candidate = *body;
        if (edit.material)
        {
            validate_material(*edit.material);
            auto colliders = candidate.colliders();
            std::unordered_map<const AuthoredPartDefinition*, AuthoredPartPtr> changed_parts;
            for (auto& collider : colliders)
            {
                collider.material = *edit.material;
                collider.density_override_kg_m3.reset();
                collider.drag_coefficient_override.reset();
                if (collider.authored_part)
                {
                    auto& changed = changed_parts[collider.authored_part.get()];
                    if (!changed)
                    {
                        auto part = *collider.authored_part;
                        part.material = *edit.material;
                        part.density_override_kg_m3.reset();
                        part.drag_coefficient_override.reset();
                        changed = std::make_shared<const AuthoredPartDefinition>(std::move(part));
                    }
                    collider.authored_part = changed;
                }
            }
            candidate.clear_colliders();
            for (auto& collider : colliders)
                candidate.add_collider(std::move(collider));
        }
        else if (edit.use_density_mass)
            candidate.rebuild_mass_properties();
        if (edit.mass_kg)
            candidate.override_mass(*edit.mass_kg);
        const auto& mass = candidate.mass_properties();
        require(math::is_finite(mass.mass_kg) && math::is_finite(mass.inertia_kg_m2) && math::is_finite(mass.center_of_mass_m), "The edited mass distribution cannot be represented.");
        require(candidate.type() != BodyType::dynamic_body || (mass.mass_kg > 0.0 && mass.inertia_kg_m2 > 0.0), "Dynamic bodies require positive mass and inertia.");
        require(math::is_finite(candidate.inverse_mass()) && math::is_finite(candidate.inverse_inertia()), "Mass and inertia are too small for a finite response.");
        candidate.set_linear_velocity(edit.linear_velocity_m_s.value_or(body->velocity_at_world_point(candidate.world_center_of_mass_m())));
        if (edit.angular_velocity_rad_s)
            candidate.set_angular_velocity(*edit.angular_velocity_rad_s);
        require(std::hypot(candidate.linear_velocity_m_s().x, candidate.linear_velocity_m_s().y) <= limits.maximum_linear_speed_m_s,
            "The changed mass centre would exceed the speed limit.");
        candidate.capture_previous_transform();
        *body = std::move(candidate);
        world.notify_body_properties_changed(id);
    }
}
