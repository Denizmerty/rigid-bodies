#include <rigidbodies/physics/kinematic_motion.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    template <typename Error, typename Function>
    bool rejects(Function&& function)
    {
        try
        {
            function();
        }
        catch (const Error&)
        {
            return true;
        }
        return false;
    }

    void expect_derivative(const KinematicMotion& motion, Real time_s)
    {
        const Real difference_s = 1.0e-6;
        const auto before = motion.sample(time_s - difference_s);
        const auto after = motion.sample(time_s + difference_s);
        const auto current = motion.sample(time_s);
        const auto derivative = (after.position_m - before.position_m) / (2.0 * difference_s);
        RIGIDBODIES_EXPECT_NEAR(current.linear_velocity_m_s.x, derivative.x, 1.0e-7, "horizontal velocity is the path derivative");
        RIGIDBODIES_EXPECT_NEAR(current.linear_velocity_m_s.y, derivative.y, 1.0e-7, "vertical velocity is the path derivative");
        RIGIDBODIES_EXPECT_NEAR(current.angular_velocity_rad_s,
            (after.orientation_rad - before.orientation_rad) / (2.0 * difference_s),
            1.0e-7,
            "angular velocity is the unwrapped orientation derivative");
    }

    RIGIDBODIES_TEST("the default kinematic path remains stationary")
    {
        const KinematicMotion motion;
        const auto state = motion.sample(123.0);
        RIGIDBODIES_EXPECT(state.position_m == Vec2 {} && state.linear_velocity_m_s == Vec2 {}, "the default centre of mass stays at the origin");
        RIGIDBODIES_EXPECT(state.orientation_rad == 0.0 && state.angular_velocity_rad_s == 0.0, "the default path does not turn");
    }

    RIGIDBODIES_TEST("linear motion prescribes translation and unwrapped rotation from an explicit origin")
    {
        LinearMotion definition;
        definition.origin_m = { 3.0, -2.0 };
        definition.initial_orientation_rad = 0.25;
        definition.velocity_m_s = { -1.5, 2.0 };
        definition.angular_velocity_rad_s = 4.0;
        const KinematicMotion motion { definition };
        const auto state = motion.sample(2.0);
        RIGIDBODIES_EXPECT(state.position_m == Vec2 { 0.0, 2.0 }, "position is origin plus elapsed time times velocity");
        RIGIDBODIES_EXPECT_NEAR(state.orientation_rad, 8.25, 0.0, "rotations are not wrapped at a revolution");
        RIGIDBODIES_EXPECT(state.linear_velocity_m_s == definition.velocity_m_s, "linear velocity is prescribed exactly");
        RIGIDBODIES_EXPECT_NEAR(state.angular_velocity_rad_s, 4.0, 0.0, "angular velocity is prescribed exactly");
        expect_derivative(motion, 2.0);
    }

    RIGIDBODIES_TEST("a harmonic piston reaches its endpoints with zero speed")
    {
        HarmonicMotion definition;
        definition.origin_m = { 2.0, -1.0 };
        definition.translation_amplitude_m = { 3.0, 4.0 };
        definition.initial_orientation_rad = 0.2;
        definition.rotation_amplitude_rad = 0.5;
        definition.frequency_hz = 2.0;
        const KinematicMotion motion { definition };
        const auto peak = motion.sample(0.125);
        RIGIDBODIES_EXPECT_NEAR(peak.position_m.x, 5.0, 1.0e-14, "horizontal peak includes the origin");
        RIGIDBODIES_EXPECT_NEAR(peak.position_m.y, 3.0, 1.0e-14, "vertical peak includes the origin");
        RIGIDBODIES_EXPECT_NEAR(peak.orientation_rad, 0.7, 1.0e-14, "angular oscillation reaches its amplitude");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(peak.linear_velocity_m_s), 0.0, 1.0e-13, "translation reverses at rest");
        RIGIDBODIES_EXPECT_NEAR(peak.angular_velocity_rad_s, 0.0, 1.0e-13, "rotation reverses at rest");
        expect_derivative(motion, 0.19);
    }

    RIGIDBODIES_TEST("harmonic phase defines initial displacement and a zero frequency holds it")
    {
        HarmonicMotion definition;
        definition.origin_m = { 1.0, 2.0 };
        definition.translation_amplitude_m = { -2.0, 3.0 };
        definition.rotation_amplitude_rad = 0.25;
        definition.frequency_hz = 0.0;
        definition.phase_rad = rigidbodies::math::half_pi;
        const KinematicMotion motion { definition };
        const auto state = motion.sample(1000.0);
        RIGIDBODIES_EXPECT_NEAR(state.position_m.x, -1.0, 1.0e-14, "phase selects the initial horizontal displacement");
        RIGIDBODIES_EXPECT_NEAR(state.position_m.y, 5.0, 1.0e-14, "phase selects the initial vertical displacement");
        RIGIDBODIES_EXPECT_NEAR(state.orientation_rad, 0.25, 1.0e-14, "phase also controls the angular displacement");
        RIGIDBODIES_EXPECT(state.linear_velocity_m_s == Vec2 {} && state.angular_velocity_rad_s == 0.0, "zero frequency produces no movement");
    }

    RIGIDBODIES_TEST("circular motion follows a tangent in the direction of travel")
    {
        CircularMotion definition;
        definition.center_m = { 3.0, -1.0 };
        definition.radius_m = 2.0;
        definition.angular_speed_rad_s = -2.0;
        definition.phase_rad = rigidbodies::math::half_pi;
        definition.orientation_offset_rad = 0.2;
        const KinematicMotion motion { definition };
        const auto initial = motion.sample(0.0);
        RIGIDBODIES_EXPECT_NEAR(initial.position_m.x, 3.0, 1.0e-14, "phase sets the circle's starting point");
        RIGIDBODIES_EXPECT_NEAR(initial.position_m.y, 1.0, 1.0e-14, "radius is measured from the explicit centre");
        RIGIDBODIES_EXPECT_NEAR(initial.linear_velocity_m_s.x, 4.0, 1.0e-14, "clockwise motion at the top travels right");
        RIGIDBODIES_EXPECT_NEAR(initial.linear_velocity_m_s.y, 0.0, 1.0e-14, "the initial velocity is tangent to the circle");
        RIGIDBODIES_EXPECT_NEAR(initial.orientation_rad, 0.2, 1.0e-14, "clockwise orientation faces the actual direction of travel");
        RIGIDBODIES_EXPECT_NEAR(initial.angular_velocity_rad_s, -2.0, 0.0, "turning follows the signed orbital speed");
        const auto later = motion.sample(10.0);
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::distance(later.position_m, definition.center_m), 2.0, 1.0e-13, "orbital radius stays fixed");
        RIGIDBODIES_EXPECT_NEAR(later.orientation_rad, -19.8, 1.0e-13, "multiple clockwise rotations remain unwrapped");
        expect_derivative(motion, 0.7);
    }

    RIGIDBODIES_TEST("a circular platform can travel without rotating and zero speed has a defined orientation")
    {
        CircularMotion definition;
        definition.orientation_offset_rad = 0.7;
        definition.orient_to_path = false;
        const KinematicMotion platform { definition };
        const auto moving = platform.sample(3.0);
        RIGIDBODIES_EXPECT_NEAR(moving.orientation_rad, 0.7, 0.0, "a nonturning platform keeps its explicit orientation");
        RIGIDBODIES_EXPECT_NEAR(moving.angular_velocity_rad_s, 0.0, 0.0, "a nonturning platform has no angular velocity");
        expect_derivative(platform, 0.7);

        definition.orient_to_path = true;
        definition.angular_speed_rad_s = 0.0;
        definition.phase_rad = 0.5;
        const auto stationary = KinematicMotion { definition }.sample(100.0);
        RIGIDBODIES_EXPECT_NEAR(stationary.orientation_rad, 1.2, 1.0e-14, "zero-speed orientation is the radial phase plus offset");
        RIGIDBODIES_EXPECT(stationary.linear_velocity_m_s == Vec2 {} && stationary.angular_velocity_rad_s == 0.0, "zero orbital speed is stationary");
    }

    RIGIDBODIES_TEST("kinematic samples are deterministic values independent of call order and later definition edits")
    {
        HarmonicMotion definition;
        definition.translation_amplitude_m = { 1.0, 2.0 };
        definition.rotation_amplitude_rad = 0.3;
        definition.frequency_hz = 0.75;
        definition.phase_rad = 0.4;
        const KinematicMotion original { definition };
        const auto copied = original;
        const auto expected = original.sample(3.75);
        definition.translation_amplitude_m = { 100.0, 100.0 };
        (void)original.sample(1000.0);
        (void)original.sample(0.0);
        const auto actual = copied.sample(3.75);
        RIGIDBODIES_EXPECT(actual.position_m == expected.position_m && actual.linear_velocity_m_s == expected.linear_velocity_m_s,
            "copied paths reproduce position and velocity exactly");
        RIGIDBODIES_EXPECT(actual.orientation_rad == expected.orientation_rad && actual.angular_velocity_rad_s == expected.angular_velocity_rad_s,
            "copied paths reproduce angular state exactly");
        RIGIDBODIES_EXPECT(original.sample(3.75).position_m == expected.position_m, "out-of-order sampling changes no hidden state");
    }

    RIGIDBODIES_TEST("invalid motion parameters and sample times are rejected")
    {
        const auto infinity = std::numeric_limits<Real>::infinity();
        const auto nan = std::numeric_limits<Real>::quiet_NaN();
        LinearMotion linear;
        linear.velocity_m_s.x = infinity;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { linear };
                               }),
            "infinite linear velocity is rejected");
        HarmonicMotion harmonic;
        harmonic.frequency_hz = -1.0;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { harmonic };
                               }),
            "negative harmonic frequency is rejected");
        harmonic.frequency_hz = std::numeric_limits<Real>::max();
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { harmonic };
                               }),
            "overflowing angular frequency is rejected");
        harmonic.frequency_hz = 1.0;
        harmonic.phase_rad = nan;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { harmonic };
                               }),
            "NaN phase is rejected");
        CircularMotion circular;
        circular.radius_m = -1.0;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { circular };
                               }),
            "negative radius is rejected");
        circular.radius_m = std::numeric_limits<Real>::max();
        circular.angular_speed_rad_s = 2.0;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   KinematicMotion motion { circular };
                               }),
            "unrepresentable orbital speed is rejected");

        const KinematicMotion motion;
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   (void)motion.sample(-1.0);
                               }),
            "negative elapsed time is rejected");
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   (void)motion.sample(infinity);
                               }),
            "infinite elapsed time is rejected");
        RIGIDBODIES_EXPECT(rejects<std::invalid_argument>([&]
                               {
                                   (void)motion.sample(nan);
                               }),
            "NaN elapsed time is rejected");
    }

    RIGIDBODIES_TEST("periodic translation stays finite at extreme elapsed times and linear overflow fails explicitly")
    {
        const auto maximum = std::numeric_limits<Real>::max();
        HarmonicMotion harmonic;
        harmonic.translation_amplitude_m = { 1.0, 2.0 };
        harmonic.frequency_hz = 10.0;
        const auto periodic = KinematicMotion { harmonic }.sample(maximum);
        RIGIDBODIES_EXPECT(std::isfinite(periodic.position_m.x) && std::isfinite(periodic.linear_velocity_m_s.x),
            "period reduction avoids multiplying an extreme time by frequency");

        LinearMotion linear;
        linear.velocity_m_s = { 2.0, 0.0 };
        const KinematicMotion motion { linear };
        RIGIDBODIES_EXPECT(rejects<std::overflow_error>([&]
                               {
                                   (void)motion.sample(maximum);
                               }),
            "an unrepresentable position cannot escape as infinity");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
