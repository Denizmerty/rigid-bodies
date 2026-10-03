#pragma once

#include <rigidbodies/math/transform2.hpp>
#include <rigidbodies/physics/units.hpp>

namespace rigidbodies::physics
{

    // Mass, centre of mass, and rotational inertia for a piece of geometry. Inertia is always
    // stated about the centre of mass; the parallel-axis shift to another reference point is a
    // separate, explicit step so that no caller has to guess which convention a value follows.
    struct MassProperties
    {
        Real mass_kg {};
        math::Vec2 center_of_mass_m {};
        Real inertia_kg_m2 {};

        [[nodiscard]] bool is_valid() const;
    };

    // Combines two sets expressed in the same frame, recomputing the shared centre of mass and
    // shifting each inertia onto it. This is what makes compound bodies work.
    [[nodiscard]] MassProperties combine(const MassProperties& left, const MassProperties& right);

    // Moves the reference point of an inertia by the parallel-axis theorem.
    [[nodiscard]] Real shift_inertia(Real inertia_about_center_kg_m2, Real mass_kg, Real offset_distance_m);

    // Re-expresses a set of mass properties in a parent frame.
    [[nodiscard]] MassProperties transformed(const MassProperties& properties, const math::Transform2& transform);

} // namespace rigidbodies::physics
