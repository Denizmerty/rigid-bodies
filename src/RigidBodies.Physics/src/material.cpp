#include <rigidbodies/physics/material.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace rigidbodies::physics
{

    Real mix_material_values(Real left, Real right, MaterialMixing mixing)
    {
        left = math::is_finite(left) ? std::max(left, 0.0) : 0.0;
        right = math::is_finite(right) ? std::max(right, 0.0) : 0.0;
        switch (mixing)
        {
        case MaterialMixing::geometric_mean:
            return std::sqrt(left) * std::sqrt(right);
        case MaterialMixing::arithmetic_mean:
            return 0.5 * left + 0.5 * right;
        case MaterialMixing::minimum:
            return std::min(left, right);
        case MaterialMixing::maximum:
            return std::max(left, right);
        }
        return 0.5 * left + 0.5 * right;
    }

    ContactMaterial combine_materials(const Material& left, const Material& right, MaterialMixing friction_mixing, MaterialMixing restitution_mixing)
    {
        ContactMaterial combined;
        // Restitution takes the larger of the pair: a rubber ball bounces off a wooden floor, and
        // averaging would hide that. Friction mixes under the selected policy.
        combined.restitution = math::clamp(mix_material_values(left.restitution, right.restitution, restitution_mixing), 0.0, 1.0);
        combined.static_friction = mix_material_values(left.static_friction, right.static_friction, friction_mixing);
        combined.kinetic_friction = mix_material_values(left.kinetic_friction, right.kinetic_friction, friction_mixing);
        // Kinetic friction above static friction would let a sliding contact resist more than a
        // resting one, so the pair is kept ordered.
        combined.kinetic_friction = std::min(combined.kinetic_friction, combined.static_friction);
        combined.rolling_friction_m = mix_material_values(left.rolling_friction_m, right.rolling_friction_m, friction_mixing);
        combined.spinning_friction_m = mix_material_values(left.spinning_friction_m, right.spinning_friction_m, friction_mixing);
        combined.first_direction = { left.friction_axis_local, left.friction_anisotropy_ratio, left.static_friction, left.kinetic_friction };
        combined.second_direction = { right.friction_axis_local, right.friction_anisotropy_ratio, right.static_friction, right.kinetic_friction };
        combined.friction_mixing = friction_mixing;
        return combined;
    }

    namespace materials
    {
        namespace
        {

            constexpr std::array<std::string_view, 6> catalogue_entries {
                "oak_wood",
                "steel",
                "aluminium",
                "rubber",
                "glass",
                "expanded_polystyrene"
            };

        } // namespace

        Material oak_wood()
        {
            return { "oak_wood", 750.0, 0.30, 0.55, 0.45, 1.05 };
        }

        Material steel()
        {
            return { "steel", 7850.0, 0.55, 0.70, 0.55, 1.00 };
        }

        Material aluminium()
        {
            return { "aluminium", 2700.0, 0.45, 0.60, 0.47, 1.00 };
        }

        Material rubber()
        {
            return { "rubber", 1100.0, 0.80, 1.10, 0.95, 1.05 };
        }

        Material glass()
        {
            return { "glass", 2500.0, 0.60, 0.50, 0.40, 0.95 };
        }

        Material expanded_polystyrene()
        {
            return { "expanded_polystyrene", 25.0, 0.20, 0.55, 0.50, 1.20 };
        }

        math::Span<const std::string_view> catalogue_names()
        {
            return catalogue_entries;
        }

        Material by_name(std::string_view name)
        {
            if (name == "steel")
            {
                return steel();
            }
            if (name == "aluminium")
            {
                return aluminium();
            }
            if (name == "rubber")
            {
                return rubber();
            }
            if (name == "glass")
            {
                return glass();
            }
            if (name == "expanded_polystyrene")
            {
                return expanded_polystyrene();
            }
            return oak_wood();
        }

    } // namespace materials

} // namespace rigidbodies::physics
