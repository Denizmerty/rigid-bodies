#include <rigidbodies/physics/collider.hpp>
#include <rigidbodies/physics/compound_mass.hpp>

#include <stdexcept>

namespace rigidbodies::physics
{

    bool should_collide(const CollisionFilter& left, const CollisionFilter& right)
    {
        if (left.group != 0 && left.group == right.group)
        {
            return left.group > 0;
        }
        return (left.category & right.mask) != 0 && (right.category & left.mask) != 0;
    }

    Real Collider::effective_density_kg_m3() const
    {
        const auto density = density_override_kg_m3.value_or(material.density_kg_m3);
        if (!math::is_finite(density) || density < 0.0)
        {
            throw std::invalid_argument("collider density must be finite and non-negative");
        }
        return density;
    }

    Real Collider::areal_density_kg_m2() const
    {
        if (!math::is_finite(depth_m) || depth_m < 0.0)
        {
            throw std::invalid_argument("collider depth must be finite and non-negative");
        }
        return effective_density_kg_m3() * depth_m;
    }

    Real Collider::effective_drag_coefficient() const
    {
        return drag_coefficient_override.value_or(material.drag_coefficient);
    }

    MassProperties Collider::compute_mass_properties() const
    {
        auto properties = compute_compound_mass_properties({ *this });
        if (properties.mass_kg == 0.0 && shape)
        {
            // Preserve the shape's conventional centre even when there is no material. A segment
            // therefore still reports its midpoint, and a zero-density disc reports its centre.
            properties.center_of_mass_m = transformed(shape->compute_mass_properties(0.0), local_transform).center_of_mass_m;
        }
        return properties;
    }

    math::Aabb Collider::compute_bounds(const math::Transform2& body_transform) const
    {
        if (!shape)
        {
            return {};
        }
        return shape->compute_bounds(math::concatenate(body_transform, local_transform));
    }

} // namespace rigidbodies::physics
