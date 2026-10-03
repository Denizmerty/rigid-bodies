#pragma once

#include <rigidbodies/math/vec2.hpp>
#include <rigidbodies/physics/units.hpp>

#include <variant>

namespace rigidbodies::physics
{

    struct KinematicState
    {
        // Position and velocity refer to the world-space centre of mass, not the body-frame origin.
        math::Vec2 position_m {};
        Real orientation_rad { 0.0 };
        math::Vec2 linear_velocity_m_s {};
        Real angular_velocity_rad_s { 0.0 };
    };

    struct LinearMotion
    {
        math::Vec2 origin_m {};
        Real initial_orientation_rad { 0.0 };
        math::Vec2 velocity_m_s {};
        Real angular_velocity_rad_s { 0.0 };
    };

    struct HarmonicMotion
    {
        math::Vec2 origin_m {};
        Real initial_orientation_rad { 0.0 };
        math::Vec2 translation_amplitude_m {};
        Real rotation_amplitude_rad { 0.0 };
        Real frequency_hz { 1.0 };
        Real phase_rad { 0.0 };
    };

    struct CircularMotion
    {
        math::Vec2 center_m {};
        Real radius_m { 1.0 };
        Real angular_speed_rad_s { 1.0 };
        Real phase_rad { 0.0 };
        Real orientation_offset_rad { 0.0 };

        // Follow the tangent in the direction of travel, with the offset added. Negative speeds
        // point clockwise; at zero speed the radial direction is used because no tangent is
        // preferred. When false, orientation stays equal to orientation_offset_rad.
        bool orient_to_path { true };
    };

    // An immutable value describing motion against elapsed time since attachment. Repeated or
    // out-of-order samples do not accumulate integration error or mutate the path. Copying the
    // value is sufficient to include it in a world snapshot.
    class KinematicMotion
    {
    public:
        using Definition = std::variant<LinearMotion, HarmonicMotion, CircularMotion>;

        // The default path stays at the origin with no rotation.
        KinematicMotion() = default;
        explicit KinematicMotion(LinearMotion motion);
        explicit KinematicMotion(HarmonicMotion motion);
        explicit KinematicMotion(CircularMotion motion);

        [[nodiscard]] const Definition& definition() const;

        // Orientations are unwrapped and velocities are analytic time derivatives. Parameters
        // must be finite, frequency and radius nonnegative, and elapsed time finite/nonnegative.
        // Invalid parameters or elapsed times throw invalid_argument; an evaluated pose outside
        // the finite Real range throws overflow_error rather than returning a corrupt state.
        [[nodiscard]] KinematicState sample(Real elapsed_time_s) const;

    private:
        Definition definition_ { LinearMotion {} };
    };

} // namespace rigidbodies::physics
