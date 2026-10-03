#include <rigidbodies/physics/kinematic_motion.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {

        void require(bool condition)
        {
            if (!condition)
            {
                throw std::invalid_argument("Kinematic motion parameters must be finite and within their valid ranges");
            }
        }

        void validate(const LinearMotion& motion)
        {
            require(math::is_finite(motion.origin_m) && math::is_finite(motion.initial_orientation_rad) &&
                math::is_finite(motion.velocity_m_s) && math::is_finite(motion.angular_velocity_rad_s));
        }

        void validate(const HarmonicMotion& motion)
        {
            require(math::is_finite(motion.origin_m) && math::is_finite(motion.initial_orientation_rad) &&
                math::is_finite(motion.translation_amplitude_m) && math::is_finite(motion.rotation_amplitude_rad) &&
                math::is_finite(motion.frequency_hz) && motion.frequency_hz >= 0.0 && math::is_finite(motion.phase_rad));
            const auto angular_frequency = math::two_pi * motion.frequency_hz;
            require(math::is_finite(angular_frequency) &&
                math::is_finite(motion.translation_amplitude_m * angular_frequency) &&
                math::is_finite(motion.rotation_amplitude_rad * angular_frequency));
        }

        void validate(const CircularMotion& motion)
        {
            require(math::is_finite(motion.center_m) && math::is_finite(motion.radius_m) && motion.radius_m >= 0.0 &&
                math::is_finite(motion.angular_speed_rad_s) && math::is_finite(motion.phase_rad) &&
                math::is_finite(motion.orientation_offset_rad) &&
                math::is_finite(motion.radius_m * motion.angular_speed_rad_s));
        }

        // Reduce time before multiplication so a periodic path can still be sampled at a very
        // large finite time. Tiny angular speeds may have an infinite period, but their product
        // with any finite time remains finite, so they need no reduction.
        Real periodic_phase(Real elapsed_time_s, Real angular_speed_rad_s, Real phase_rad)
        {
            const auto period_s = angular_speed_rad_s == 0.0
                ? 0.0
                : math::two_pi / std::abs(angular_speed_rad_s);
            const auto reduced_time_s = period_s > 0.0 && math::is_finite(period_s)
                ? std::remainder(elapsed_time_s, period_s)
                : elapsed_time_s;
            return std::remainder(phase_rad, math::two_pi) + reduced_time_s * angular_speed_rad_s;
        }

        KinematicState evaluate(const LinearMotion& motion, Real elapsed_time_s)
        {
            KinematicState result;
            result.position_m = {
                std::fma(motion.velocity_m_s.x, elapsed_time_s, motion.origin_m.x),
                std::fma(motion.velocity_m_s.y, elapsed_time_s, motion.origin_m.y)
            };
            result.orientation_rad = std::fma(motion.angular_velocity_rad_s, elapsed_time_s, motion.initial_orientation_rad);
            result.linear_velocity_m_s = motion.velocity_m_s;
            result.angular_velocity_rad_s = motion.angular_velocity_rad_s;
            return result;
        }

        KinematicState evaluate(const HarmonicMotion& motion, Real elapsed_time_s)
        {
            const auto angular_frequency = math::two_pi * motion.frequency_hz;
            const auto phase = periodic_phase(elapsed_time_s, angular_frequency, motion.phase_rad);
            const auto displacement = std::sin(phase);
            const auto speed = angular_frequency * std::cos(phase);
            KinematicState result;
            result.position_m = motion.origin_m + motion.translation_amplitude_m * displacement;
            result.orientation_rad = motion.initial_orientation_rad + motion.rotation_amplitude_rad * displacement;
            result.linear_velocity_m_s = motion.translation_amplitude_m * speed;
            result.angular_velocity_rad_s = motion.rotation_amplitude_rad * speed;
            return result;
        }

        KinematicState evaluate(const CircularMotion& motion, Real elapsed_time_s)
        {
            const auto phase = periodic_phase(elapsed_time_s, motion.angular_speed_rad_s, motion.phase_rad);
            const auto cosine = std::cos(phase);
            const auto sine = std::sin(phase);
            const auto speed = motion.radius_m * motion.angular_speed_rad_s;
            KinematicState result;
            result.position_m = motion.center_m + math::Vec2 { cosine, sine } * motion.radius_m;
            result.linear_velocity_m_s = math::Vec2 { -sine, cosine } * speed;
            result.orientation_rad = motion.orientation_offset_rad;
            if (motion.orient_to_path)
            {
                const auto tangent_offset = motion.angular_speed_rad_s > 0.0 ? math::half_pi
                    : motion.angular_speed_rad_s < 0.0                       ? -math::half_pi
                                                                             : 0.0;
                result.orientation_rad = std::fma(motion.angular_speed_rad_s, elapsed_time_s, motion.phase_rad) + tangent_offset + motion.orientation_offset_rad;
                result.angular_velocity_rad_s = motion.angular_speed_rad_s;
            }
            return result;
        }

    } // namespace

    KinematicMotion::KinematicMotion(LinearMotion motion) : definition_(motion)
    {
        validate(std::get<LinearMotion>(definition_));
    }

    KinematicMotion::KinematicMotion(HarmonicMotion motion) : definition_(motion)
    {
        validate(std::get<HarmonicMotion>(definition_));
    }

    KinematicMotion::KinematicMotion(CircularMotion motion) : definition_(motion)
    {
        validate(std::get<CircularMotion>(definition_));
    }

    const KinematicMotion::Definition& KinematicMotion::definition() const
    {
        return definition_;
    }

    KinematicState KinematicMotion::sample(Real elapsed_time_s) const
    {
        if (!math::is_finite(elapsed_time_s) || elapsed_time_s < 0.0)
        {
            throw std::invalid_argument("Kinematic motion requires finite, nonnegative elapsed time");
        }
        const auto result = std::visit([elapsed_time_s](const auto& motion)
            {
                return evaluate(motion, elapsed_time_s);
            },
            definition_);
        if (!math::is_finite(result.position_m) || !math::is_finite(result.orientation_rad) ||
            !math::is_finite(result.linear_velocity_m_s) || !math::is_finite(result.angular_velocity_rad_s))
        {
            throw std::overflow_error("Kinematic motion exceeds the finite simulation range");
        }
        return result;
    }

} // namespace rigidbodies::physics
