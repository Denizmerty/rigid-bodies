#pragma once

#include <rigidbodies/math/scalar.hpp>

namespace rigidbodies::physics
{

    using math::Real;

    // Every quantity crossing a physics interface is SI: metres, kilograms, seconds, radians, and
    // the units derived from them. Presentation layers convert for display; the simulation never
    // stores a display unit. The suffix on each field name states the unit so that a reader never
    // has to trace a value back to its source to know what it holds.
    inline constexpr Real standard_gravity_m_s2 = 9.80665;

    // Sea-level dry air at 15 degrees Celsius, the default the playground starts from.
    inline constexpr Real default_air_density_kg_m3 = 1.225;

    // The playground targets household-scale objects. These bounds are advisory: they inform
    // default camera framing, the ranges offered by the interface, and the warnings shown when a
    // scenario drifts outside the range the presentation was designed around.
    inline constexpr Real minimum_practical_length_m = 1.0e-3;
    inline constexpr Real maximum_practical_length_m = 1.0e2;
    inline constexpr Real minimum_practical_mass_kg = 1.0e-4;
    inline constexpr Real maximum_practical_mass_kg = 1.0e4;

    [[nodiscard]] inline constexpr Real grams_to_kilograms(Real grams)
    {
        return grams * 1.0e-3;
    }

    [[nodiscard]] inline constexpr Real centimetres_to_metres(Real centimetres)
    {
        return centimetres * 1.0e-2;
    }

    [[nodiscard]] inline constexpr Real metres_to_centimetres(Real metres)
    {
        return metres * 1.0e2;
    }

    [[nodiscard]] inline constexpr Real joules_from_newton_metres(Real newton_metres)
    {
        return newton_metres;
    }

} // namespace rigidbodies::physics
