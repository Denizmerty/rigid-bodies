#include <rigidbodies/physics/mass_properties.hpp>

namespace rigidbodies::physics
{

    bool MassProperties::is_valid() const
    {
        return math::is_finite(mass_kg) && math::is_finite(inertia_kg_m2) && math::is_finite(center_of_mass_m) && mass_kg >= 0.0 && inertia_kg_m2 >= 0.0;
    }

    Real shift_inertia(Real inertia_about_center_kg_m2, Real mass_kg, Real offset_distance_m)
    {
        return inertia_about_center_kg_m2 + mass_kg * offset_distance_m * offset_distance_m;
    }

    MassProperties combine(const MassProperties& left, const MassProperties& right)
    {
        const auto total_mass = left.mass_kg + right.mass_kg;
        if (total_mass <= 0.0)
        {
            // Two massless parts stay massless; the combined centre is their midpoint so that a
            // caller inspecting the result still sees a sensible position.
            MassProperties massless;
            massless.center_of_mass_m = (left.center_of_mass_m + right.center_of_mass_m) * 0.5;
            return massless;
        }

        MassProperties combined;
        combined.mass_kg = total_mass;
        combined.center_of_mass_m = (left.center_of_mass_m * left.mass_kg + right.center_of_mass_m * right.mass_kg) / total_mass;
        combined.inertia_kg_m2 = shift_inertia(left.inertia_kg_m2, left.mass_kg, math::distance(left.center_of_mass_m, combined.center_of_mass_m)) +
            shift_inertia(right.inertia_kg_m2, right.mass_kg, math::distance(right.center_of_mass_m, combined.center_of_mass_m));
        return combined;
    }

    MassProperties transformed(const MassProperties& properties, const math::Transform2& transform)
    {
        MassProperties result = properties;
        // Rotational inertia about the centre of mass is invariant under a rigid transform in the
        // plane, so only the centre of mass moves.
        result.center_of_mass_m = math::transform_point(transform, properties.center_of_mass_m);
        return result;
    }

} // namespace rigidbodies::physics
