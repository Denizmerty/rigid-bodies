#include <rigidbodies/physics/education.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <set>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    template <typename Exception, typename Function>
    void expect_exception(Function function, std::string_view reason)
    {
        bool caught = false;
        try
        {
            function();
        }
        catch (const Exception&)
        {
            caught = true;
        }
        RIGIDBODIES_EXPECT(caught, reason);
    }

    const RigidBody& named_body(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == name)
                return *world.find_body(id);
        RIGIDBODIES_FAIL("scenario body must exist");
    }

    RIGIDBODIES_TEST("derived periods follow inertia stiffness gravity and damping with missing nonoscillatory states")
    {
        RIGIDBODIES_EXPECT_NEAR(*oscillation_period_s(1.0, 4.0), math::pi, 1.0e-14, "undamped angular frequency is sqrt(k/m)");
        RIGIDBODIES_EXPECT_NEAR(*oscillation_period_s(4.0, 4.0), math::two_pi, 1.0e-14, "quadrupling mass doubles period");
        RIGIDBODIES_EXPECT_NEAR(*oscillation_period_s(1.0, 4.0, 2.0), math::two_pi / std::sqrt(3.0), 1.0e-14, "underdamped frequency includes the damping correction");
        RIGIDBODIES_EXPECT(!oscillation_period_s(1.0, 4.0, 4.0) && !oscillation_period_s(1.0, 4.0, 5.0),
            "critical and overdamped return no oscillation period");
        RIGIDBODIES_EXPECT_NEAR(*pendulum_period_s(1.0, standard_gravity_m_s2), math::two_pi / std::sqrt(standard_gravity_m_s2), 1.0e-14, "small-angle period depends on pendulum length and gravity");
        RIGIDBODIES_EXPECT_NEAR(*pendulum_period_s(4.0, standard_gravity_m_s2), 2.0 * *pendulum_period_s(1.0, standard_gravity_m_s2), 1.0e-14, "pendulum period doubles when length quadruples");
        RIGIDBODIES_EXPECT_NEAR(*critical_ramp_angle_rad(0.0), 0.0, 0.0, "frictionless ramp has a zero threshold");
        RIGIDBODIES_EXPECT_NEAR(*critical_ramp_angle_rad(1.0), math::pi / 4.0, 1.0e-14, "unit coefficient gives a 45 degree threshold");
        RIGIDBODIES_EXPECT(*critical_ramp_angle_rad(2.0) > math::pi / 4.0, "physical friction coefficients are not incorrectly capped at one");
        const auto infinity = std::numeric_limits<Real>::infinity();
        const auto nan = std::numeric_limits<Real>::quiet_NaN();
        for (const auto invalid : { 0.0, -1.0, infinity, nan })
        {
            RIGIDBODIES_EXPECT(!oscillation_period_s(invalid, 1.0) && !oscillation_period_s(1.0, invalid), "invalid mass or stiffness has no period");
            RIGIDBODIES_EXPECT(!pendulum_period_s(invalid, 1.0) && !pendulum_period_s(1.0, invalid), "zero gravity and invalid pendulum inputs have no period");
        }
        for (const auto invalid : { -1.0, infinity, nan })
            RIGIDBODIES_EXPECT(!oscillation_period_s(1.0, 1.0, invalid) && !critical_ramp_angle_rad(invalid), "invalid damping and friction are unavailable");
    }

    RIGIDBODIES_TEST("predicted oscillator period agrees with an independently measured full cycle")
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        World world { settings };
        world.set_integrator(std::make_shared<RungeKutta4Integrator>());
        BodyDefinition definition;
        definition.position_m = { 0.3, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.05);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, "oscillator");
        auto* body = world.find_body(id);
        body->override_mass(0.8);
        world.add_force_generator(id, std::make_shared<PointAttractor>(math::Vec2 {}, 5.0));
        constexpr Real dt = 1.0 / 480.0;
        std::array<Real, 2> downward_crossings {};
        std::size_t crossing_count = 0;
        for (int index = 0; index < 3000 && crossing_count < downward_crossings.size(); ++index)
        {
            const auto before_x = body->position_m().x;
            const auto before_time = world.statistics().elapsed_time_s;
            world.step(dt);
            const auto after_x = body->position_m().x;
            if (before_x > 0.0 && after_x <= 0.0)
                downward_crossings[crossing_count++] = before_time + dt * before_x / (before_x - after_x);
        }
        RIGIDBODIES_EXPECT(crossing_count == 2, "two same-direction equilibrium crossings measure a full period");
        RIGIDBODIES_EXPECT_NEAR(downward_crossings[1] - downward_crossings[0], *oscillation_period_s(0.8, 5.0), 1.0e-6, "measured period uses simulated positions rather than a theoretical substitution");
    }

    RIGIDBODIES_TEST("head on comparison measures isolated collision velocities momentum and energy loss")
    {
        const auto reports = compare_head_on_collisions();
        const std::array<Real, 3> restitutions { 1.0, 0.0, 0.5 };
        const std::array<Real, 3> first_velocities { -2.0, 0.0, -1.0 };
        const std::array<Real, 3> second_velocities { 1.0, 0.0, 0.5 };
        RIGIDBODIES_EXPECT(reports.size() == 3, "all three responses share the same initial setup");
        for (std::size_t index = 0; index < reports.size(); ++index)
        {
            const auto& report = reports[index];
            RIGIDBODIES_EXPECT_NEAR(report.restitution, restitutions[index], 0.0, "comparison order is stable");
            RIGIDBODIES_EXPECT(!report.label.empty() && report.step_count > 0, "measurement has a visible identity and advances an actual world");
            RIGIDBODIES_EXPECT_NEAR(report.predicted_first_velocity_m_s, first_velocities[index], 1.0e-12, "theory accounts for unequal masses");
            RIGIDBODIES_EXPECT_NEAR(report.predicted_second_velocity_m_s, second_velocities[index], 1.0e-12, "theory predicts the second velocity");
            RIGIDBODIES_EXPECT_NEAR(report.measured_first_velocity_m_s, first_velocities[index], 1.0e-8, "actual first impact velocity agrees with theory");
            RIGIDBODIES_EXPECT_NEAR(report.measured_second_velocity_m_s, second_velocities[index], 1.0e-8, "actual second impact velocity agrees with theory");
            RIGIDBODIES_EXPECT_NEAR(report.incoming_momentum_kg_m_s, 0.0, 0.0, "initial momentum matches both incoming bodies");
            RIGIDBODIES_EXPECT_NEAR(report.outgoing_momentum_kg_m_s, report.incoming_momentum_kg_m_s, 1.0e-8, "all three collision types conserve momentum");
            RIGIDBODIES_EXPECT_NEAR(report.incoming_kinetic_energy_j, 3.0, 1.0e-12, "same initial energy in every trial");
            RIGIDBODIES_EXPECT_NEAR(report.incoming_kinetic_energy_j - report.outgoing_kinetic_energy_j, report.predicted_energy_loss_j, 1.0e-8, "measured energy reduction equals reduced-mass restitution prediction");
            RIGIDBODIES_EXPECT_NEAR(report.impact_time_s, 1.0 / 3.0, 1.0 / 240.0 + 1.0e-6, "impact time comes from the shared initial separation");
        }
        RIGIDBODIES_EXPECT(reports[0].outgoing_kinetic_energy_j > reports[2].outgoing_kinetic_energy_j &&
                reports[2].outgoing_kinetic_energy_j > reports[1].outgoing_kinetic_energy_j,
            "side by side observations distinguish elastic partial and fully inelastic energy");
    }

    RIGIDBODIES_TEST("collision prediction and measurement are Galilean invariant for nonzero total momentum")
    {
        CollisionComparisonSettings settings;
        settings.first_mass_kg = 0.7;
        settings.second_mass_kg = 1.3;
        settings.first_velocity_m_s = 1.7;
        settings.second_velocity_m_s = -0.2;
        const auto base = measure_head_on_collision(0.35, settings);
        settings.first_velocity_m_s += 4.0;
        settings.second_velocity_m_s += 4.0;
        const auto shifted = measure_head_on_collision(0.35, settings);
        RIGIDBODIES_EXPECT_NEAR(base.outgoing_momentum_kg_m_s, 0.93, 1.0e-8, "nonzero isolated momentum is conserved");
        RIGIDBODIES_EXPECT_NEAR(shifted.outgoing_momentum_kg_m_s, 8.93, 1.0e-8, "common velocity changes momentum by total mass times velocity");
        RIGIDBODIES_EXPECT_NEAR(shifted.measured_first_velocity_m_s - base.measured_first_velocity_m_s, 4.0, 1.0e-8, "first outgoing speed transforms with the frame");
        RIGIDBODIES_EXPECT_NEAR(shifted.measured_second_velocity_m_s - base.measured_second_velocity_m_s, 4.0, 1.0e-8, "second outgoing speed transforms with the frame");
        RIGIDBODIES_EXPECT_NEAR(shifted.predicted_energy_loss_j, base.predicted_energy_loss_j, 1.0e-12, "dissipated energy depends only on relative incoming speed");
        RIGIDBODIES_EXPECT_NEAR(base.measured_first_velocity_m_s, base.predicted_first_velocity_m_s, 1.0e-8, "custom mass and restitution use the same simulation contract");
        const auto repeat = measure_head_on_collision(0.35, settings);
        RIGIDBODIES_EXPECT(repeat.measured_first_velocity_m_s == shifted.measured_first_velocity_m_s && repeat.step_count == shifted.step_count,
            "reference comparison is reproducible within the same build");
    }

    RIGIDBODIES_TEST("collision comparison rejects invalid parameters missing impacts and numerical clamps")
    {
        for (const auto invalid : { -0.1, 1.1, std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
            expect_exception<std::invalid_argument>([&]()
                {
                    (void)measure_head_on_collision(invalid);
                },
                "restitution stays finite in [0,1]");
        for (int problem = 0; problem < 8; ++problem)
        {
            CollisionComparisonSettings settings;
            if (problem == 0)
                settings.first_mass_kg = 0.0;
            if (problem == 1)
                settings.second_mass_kg = -1.0;
            if (problem == 2)
                settings.radius_m = 0.0;
            if (problem == 3)
                settings.initial_gap_m = -1.0;
            if (problem == 4)
                settings.first_velocity_m_s = settings.second_velocity_m_s;
            if (problem == 5)
                settings.time_step_s = std::numeric_limits<Real>::quiet_NaN();
            if (problem == 6)
                settings.maximum_steps = 1000001;
            if (problem == 7)
                settings.maximum_duration_s = 0.0;
            expect_exception<std::invalid_argument>([&]()
                {
                    (void)compare_head_on_collisions(settings);
                },
                "invalid comparison cannot enter a measurement loop");
        }
        CollisionComparisonSettings settings;
        settings.maximum_steps = 2;
        expect_exception<std::runtime_error>([&]()
            {
                (void)measure_head_on_collision(0.5, settings);
            },
            "step exhaustion is not reported as an impact");
        settings = {};
        settings.maximum_duration_s = 0.01;
        expect_exception<std::runtime_error>([&]()
            {
                (void)measure_head_on_collision(0.5, settings);
            },
            "duration exhaustion is not reported as an impact");
        settings = {};
        settings.first_velocity_m_s = 150.0;
        expect_exception<std::overflow_error>([&]()
            {
                (void)measure_head_on_collision(0.5, settings);
            },
            "a clamped initial speed is not presented as the requested experiment");
    }

    RIGIDBODIES_TEST("collision demonstration keeps three equal setups isolated and matches reference measurements")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "collision_comparison"), "new comparison scene is loadable");
        RIGIDBODIES_EXPECT(world.body_ids().size() == 6 && !world.compute_bounds().is_empty(), "three two-body rows supply useful camera bounds");
        const auto reports = compare_head_on_collisions();
        const std::array<std::string, 3> names { "Elastic", "Inelastic", "Partial" };
        for (std::size_t index = 0; index < names.size(); ++index)
        {
            const auto& left = named_body(world, names[index] + " left");
            const auto& right = named_body(world, names[index] + " right");
            RIGIDBODIES_EXPECT_NEAR(left.mass_properties().mass_kg, 1.0, 1.0e-12, "all first masses match");
            RIGIDBODIES_EXPECT_NEAR(right.mass_properties().mass_kg, 2.0, 1.0e-12, "all second masses match");
            RIGIDBODIES_EXPECT(left.linear_velocity_m_s() == math::Vec2 { 2.0, 0.0 } && right.linear_velocity_m_s() == math::Vec2 { -1.0, 0.0 }, "all rows start with identical velocities");
            if (index + 1 < names.size())
                RIGIDBODIES_EXPECT(!should_collide(left.colliders().front().filter, named_body(world, names[index + 1] + " left").colliders().front().filter),
                    "collision filters keep the reference rows independent even after dragging");
        }
        for (int index = 0; index < 80; ++index)
            world.step(1.0 / 120.0);
        for (std::size_t index = 0; index < names.size(); ++index)
        {
            RIGIDBODIES_EXPECT_NEAR(named_body(world, names[index] + " left").linear_velocity_m_s().x, reports[index].measured_first_velocity_m_s, 1.0e-8, "visible scene and independent comparison measure the same first response");
            RIGIDBODIES_EXPECT_NEAR(named_body(world, names[index] + " right").linear_velocity_m_s().x, reports[index].measured_second_velocity_m_s, 1.0e-8, "visible scene and independent comparison measure the same second response");
            RIGIDBODIES_EXPECT_NEAR(named_body(world, names[index] + " left").linear_velocity_m_s().y, 0.0, 0.0, "ordinary scene gravity cannot contaminate the horizontal isolated comparison");
        }
    }

    RIGIDBODIES_TEST("ballistic reference compares measured trajectories and known Euler position error")
    {
        const auto reports = compare_projectile_trajectories();
        RIGIDBODIES_EXPECT(reports.size() == 3, "all existing integrators run the same projectile experiment");
        for (const auto& report : reports)
        {
            RIGIDBODIES_EXPECT(!report.integrator_name.empty() && report.step_count == 30, "method and actual step count identify the run");
            RIGIDBODIES_EXPECT_NEAR(report.elapsed_time_s, 0.5, 1.0e-14, "fractional endpoint reaches requested time");
            RIGIDBODIES_EXPECT_NEAR(report.predicted_position_m.x, 1.0, 1.0e-14, "horizontal prediction is constant speed times time");
            RIGIDBODIES_EXPECT_NEAR(report.predicted_position_m.y, 1.5 - standard_gravity_m_s2 / 8.0, 1.0e-14, "vertical prediction includes constant acceleration");
            RIGIDBODIES_EXPECT_NEAR(report.measured_position_m.x, 1.0, 1.0e-14, "horizontal simulated position matches theory");
            RIGIDBODIES_EXPECT_NEAR(report.velocity_error_m_s, 0.0, 1.0e-13, "all methods integrate constant acceleration velocity exactly up to roundoff");
            RIGIDBODIES_EXPECT_NEAR(report.position_error_m, math::length(report.measured_position_m - report.predicted_position_m), 0.0, "error is derived from measured output rather than estimated order");
        }
        RIGIDBODIES_EXPECT_NEAR(reports[0].position_error_m, standard_gravity_m_s2 * 0.5 / 120.0, 1.0e-13, "semi-implicit Euler has the expected g*T*dt/2 position error");
        RIGIDBODIES_EXPECT_NEAR(reports[1].position_error_m, 0.0, 1.0e-13, "Verlet reproduces constant-acceleration position");
        RIGIDBODIES_EXPECT_NEAR(reports[2].position_error_m, 0.0, 1.0e-13, "RK4 reproduces constant-acceleration position");
    }

    RIGIDBODIES_TEST("ballistic comparison converges with step size and handles a partial last step")
    {
        ProjectileComparisonSettings settings;
        settings.gravity_m_s2 = { 2.0, -4.0 };
        settings.initial_position_m = { -0.4, 0.2 };
        settings.initial_velocity_m_s = { -1.0, 0.5 };
        settings.duration_s = 0.51;
        settings.time_step_s = 0.03;
        const auto coarse = measure_projectile_trajectory(SemiImplicitEulerIntegrator {}, settings);
        settings.time_step_s *= 0.5;
        const auto fine = measure_projectile_trajectory(SemiImplicitEulerIntegrator {}, settings);
        RIGIDBODIES_EXPECT_NEAR(coarse.position_error_m, 2.0 * fine.position_error_m, 1.0e-13, "halving Euler step halves ballistic position error");
        settings.time_step_s = 0.2;
        const auto partial = measure_projectile_trajectory(VelocityVerletIntegrator {}, settings);
        RIGIDBODIES_EXPECT(partial.step_count == 3, "nonintegral duration takes a shortened final step");
        RIGIDBODIES_EXPECT_NEAR(partial.elapsed_time_s, 0.51, 1.0e-14, "reported comparison time includes only requested duration");
        RIGIDBODIES_EXPECT_NEAR(partial.position_error_m, 0.0, 1.0e-13, "partial final step preserves Verlet's constant-gravity result");
    }

    RIGIDBODIES_TEST("ballistic measurement rejects invalid or impractical comparisons before returning results")
    {
        for (int problem = 0; problem < 5; ++problem)
        {
            ProjectileComparisonSettings settings;
            if (problem == 0)
                settings.time_step_s = 0.0;
            if (problem == 1)
                settings.duration_s = -1.0;
            if (problem == 2)
                settings.gravity_m_s2.x = std::numeric_limits<Real>::quiet_NaN();
            if (problem == 3)
                settings.initial_velocity_m_s.y = std::numeric_limits<Real>::infinity();
            if (problem == 4)
                settings.time_step_s = 1.0e-12;
            expect_exception<std::invalid_argument>([&]()
                {
                    (void)compare_projectile_trajectories(settings);
                },
                "invalid and excessive work requests cannot produce a misleading measurement");
        }
        ProjectileComparisonSettings settings;
        settings.initial_position_m.x = 150.0;
        expect_exception<std::overflow_error>([&]()
            {
                (void)compare_projectile_trajectories(settings);
            },
            "clamped initial position invalidates reference comparison");
        settings = {};
        settings.gravity_m_s2.y = -1000.0;
        expect_exception<std::overflow_error>([&]()
            {
                (void)compare_projectile_trajectories(settings);
            },
            "a motion clamp during the run is reported explicitly");
    }
} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
