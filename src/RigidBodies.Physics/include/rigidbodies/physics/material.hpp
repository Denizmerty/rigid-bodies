#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/math/vec2.hpp>
#include <rigidbodies/physics/units.hpp>

#include <string>
#include <string_view>

namespace rigidbodies::physics
{

    // Surface and bulk properties of a body. Density determines mass and rotational inertia;
    // the contact properties control bounce and friction.
    struct Material
    {
        std::string name;

        Real density_kg_m3 { 1000.0 };

        // Coefficient of restitution in [0, 1]: the fraction of separating speed retained after
        // an impact. Zero is perfectly inelastic, one is perfectly elastic.
        Real restitution { 0.2 };

        // Coulomb friction coefficients. Static friction bounds the tangential force a resting
        // contact can carry; kinetic friction applies once sliding begins and is normally the
        // smaller of the two.
        Real static_friction { 0.6 };
        Real kinetic_friction { 0.5 };

        // Dimensionless drag coefficient used by the aerodynamic force generators. The value is a
        // property of shape as much as of material, so a collider may override it.
        Real drag_coefficient { 1.0 };

        // Angular resistance lengths: each bounds a contact torque impulse by its length times
        // the normal impulse. Rolling describes deformation losses; spinning describes contact
        // patch torsion. In this planar model both dissipate relative out-of-plane rotation and
        // their capacities add. Both default to zero, disabling these sources of resistance.
        Real rolling_friction_m { 0.0 };
        Real spinning_friction_m { 0.0 };

        // The friction ellipse's preferred axis, expressed in collider-local coordinates. The
        // parallel coefficient is ratio times the ordinary perpendicular coefficient. A ratio
        // of one is isotropic; zero permits free slip exactly along this axis.
        math::Vec2 friction_axis_local { 1.0, 0.0 };
        Real friction_anisotropy_ratio { 1.0 };
    };

    // Rules for combining two materials' properties at a contact. A named policy lets experiments
    // compare the available mixing conventions.
    enum class MaterialMixing
    {
        geometric_mean,
        arithmetic_mean,
        minimum,
        maximum
    };

    [[nodiscard]] Real mix_material_values(Real left, Real right, MaterialMixing mixing);

    struct DirectionalFriction
    {
        math::Vec2 axis_local { 1.0, 0.0 };
        Real ratio { 1.0 };
        Real static_friction {};
        Real kinetic_friction {};
    };

    struct ContactMaterial
    {
        Real restitution {};
        Real static_friction {};
        Real kinetic_friction {};
        Real rolling_friction_m {};
        Real spinning_friction_m {};
        DirectionalFriction first_direction {};
        DirectionalFriction second_direction {};
        MaterialMixing friction_mixing { MaterialMixing::geometric_mean };
    };

    [[nodiscard]] ContactMaterial combine_materials(const Material& left, const Material& right,
        MaterialMixing friction_mixing = MaterialMixing::geometric_mean, MaterialMixing restitution_mixing = MaterialMixing::maximum);

    // A small set of household references so that scenarios can start from recognisable objects
    // rather than from bare numbers. The catalogue is data, not policy: scenarios are free to
    // supply their own materials.
    namespace materials
    {

        [[nodiscard]] Material oak_wood();
        [[nodiscard]] Material steel();
        [[nodiscard]] Material aluminium();
        [[nodiscard]] Material rubber();
        [[nodiscard]] Material glass();
        [[nodiscard]] Material expanded_polystyrene();

        [[nodiscard]] math::Span<const std::string_view> catalogue_names();

        // Returns the named entry, or the oak reference when the name is not in the catalogue.
        [[nodiscard]] Material by_name(std::string_view name);

    } // namespace materials

} // namespace rigidbodies::physics
