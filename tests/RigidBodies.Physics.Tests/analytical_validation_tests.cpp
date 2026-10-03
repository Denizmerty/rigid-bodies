#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    std::string context(std::string_view label, std::initializer_list<std::pair<std::string_view, Real>> values)
    {
        std::ostringstream out;
        out << label << std::setprecision(17);
        for (const auto& value : values)
            out << ' ' << value.first << '=' << value.second;
        return out.str();
    }

    World laboratory(math::Vec2 gravity = {})
    {
        WorldSettings settings;
        settings.gravity_m_s2 = gravity;
        settings.sleep.enabled = false;
        settings.solver.restitution_threshold_m_s = 0.0;
        World world { settings };
        world.set_parallel_settings({ 1, 64 });
        return world;
    }

    BodyDefinition circle(math::Vec2 position, math::Vec2 velocity = {})
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        Collider collider;
        collider.shape = make_circle(0.25);
        collider.material.restitution = 1.0;
        collider.material.static_friction = 0.0;
        collider.material.kinetic_friction = 0.0;
        definition.colliders.push_back(collider);
        return definition;
    }

    void advance(World& world, int steps, Real dt, const std::string& description)
    {
        for (int step = 0; step < steps; ++step)
            world.step(dt);
        RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, description + " stays inside numerical limits");
    }

    RIGIDBODIES_TEST("projectile trajectories expose the exact Euler bias and staged constant-acceleration solution")
    {
        const std::array<IntegratorPtr, 3> methods { std::make_shared<SemiImplicitEulerIntegrator>(),
            std::make_shared<VelocityVerletIntegrator>(),
            std::make_shared<RungeKutta4Integrator>() };
        for (std::size_t method = 0; method < methods.size(); ++method)
            for (const Real mass : { 0.2, 1.0, 9.0 })
                for (const int steps : { 15, 30, 60 })
                {
                    const Real duration = 0.5;
                    const Real dt = duration / steps;
                    const math::Vec2 gravity { 1.25, -9.81 };
                    const math::Vec2 velocity { 2.3, 4.1 };
                    auto world = laboratory(gravity);
                    world.set_integrator(methods[method]);
                    auto definition = circle({ -1.0, 2.0 }, velocity);
                    definition.colliders.front().local_transform.translation = { 0.1, -0.07 };
                    const auto id = world.create_body(definition);
                    world.find_body(id)->override_mass(mass);
                    const auto initial = world.find_body(id)->world_center_of_mass_m();
                    const auto initial_energy = world.find_body(id)->kinetic_energy_j() - mass * math::dot(gravity, initial);
                    const auto description = context(methods[method]->name(), { { "mass", mass }, { "dt", dt } });
                    advance(world, steps, dt, description);
                    const auto* body = world.find_body(id);
                    const auto continuum = initial + velocity * duration + gravity * (0.5 * duration * duration);
                    const auto bias = method == 0 ? gravity * (0.5 * duration * dt) : math::Vec2 {};
                    const auto expected_velocity = velocity + gravity * duration;
                    RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, continuum.x + bias.x, 2.0e-12, description + " center x");
                    RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().y, continuum.y + bias.y, 2.0e-12, description + " center y");
                    RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, expected_velocity.x, 2.0e-12, description + " velocity x");
                    RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().y, expected_velocity.y, 2.0e-12, description + " velocity y");
                    const auto energy = body->kinetic_energy_j() - mass * math::dot(gravity, body->world_center_of_mass_m());
                    RIGIDBODIES_EXPECT_NEAR(energy - initial_energy, -mass * math::dot(gravity, bias), 2.0e-11 * mass, description + " mechanical energy defect");
                }
    }

    RIGIDBODIES_TEST("world elastic impacts match closed-form unequal-mass velocities in rotated moving frames")
    {
        for (const Real first_mass : { 0.25, 1.0, 4.0 })
            for (const Real second_mass : { 0.25, 1.0, 4.0 })
                for (const Real angle : { 0.0, 0.37, 1.2 })
                {
                    auto world = laboratory();
                    const math::Vec2 normal { std::cos(angle), std::sin(angle) };
                    const auto tangent = math::perpendicular(normal);
                    const auto drift = normal * 0.7 + tangent * 0.3;
                    const auto first = world.create_body(circle(-normal * 0.25, normal * 3.0 + drift));
                    const auto second = world.create_body(circle(normal * 0.25, normal * -1.0 + drift));
                    world.find_body(first)->override_mass(first_mass);
                    world.find_body(second)->override_mass(second_mass);
                    const auto initial_momentum = world.find_body(first)->linear_momentum_kg_m_s() + world.find_body(second)->linear_momentum_kg_m_s();
                    const auto initial_energy = world.find_body(first)->kinetic_energy_j() + world.find_body(second)->kinetic_energy_j();
                    const auto first_final = ((first_mass - second_mass) * 3.0 - 2.0 * second_mass) / (first_mass + second_mass);
                    const auto second_final = (6.0 * first_mass - (second_mass - first_mass)) / (first_mass + second_mass);
                    const auto description = context("elastic", { { "m1", first_mass }, { "m2", second_mass }, { "angle", angle } });
                    advance(world, 1, 1.0 / 240.0, description);
                    const auto* a = world.find_body(first);
                    const auto* b = world.find_body(second);
                    RIGIDBODIES_EXPECT(world.statistics().manifold_count == 1, description + " passes through collision detection");
                    RIGIDBODIES_EXPECT_NEAR(math::dot(a->linear_velocity_m_s() - drift, normal), first_final, 2.0e-10, description + " first normal velocity");
                    RIGIDBODIES_EXPECT_NEAR(math::dot(b->linear_velocity_m_s() - drift, normal), second_final, 2.0e-10, description + " second normal velocity");
                    RIGIDBODIES_EXPECT_NEAR(math::dot(a->linear_velocity_m_s() - drift, tangent), 0.0, 2.0e-10, description + " first tangent velocity");
                    RIGIDBODIES_EXPECT_NEAR(math::dot(b->linear_velocity_m_s() - drift, tangent), 0.0, 2.0e-10, description + " second tangent velocity");
                    const auto momentum = a->linear_momentum_kg_m_s() + b->linear_momentum_kg_m_s();
                    RIGIDBODIES_EXPECT_NEAR(momentum.x, initial_momentum.x, 2.0e-10, description + " momentum x");
                    RIGIDBODIES_EXPECT_NEAR(momentum.y, initial_momentum.y, 2.0e-10, description + " momentum y");
                    RIGIDBODIES_EXPECT_NEAR(a->kinetic_energy_j() + b->kinetic_energy_j(), initial_energy, 2.0e-9, description + " kinetic energy");
                    RIGIDBODIES_EXPECT_NEAR(a->angular_velocity_rad_s(), 0.0, 2.0e-10, description + " no first-body spin");
                    RIGIDBODIES_EXPECT_NEAR(b->angular_velocity_rad_s(), 0.0, 2.0e-10, description + " no second-body spin");
                }
    }

    void verify_ramp(Real angle, Real static_friction, Real kinetic_friction, Real initial_speed, bool sticks)
    {
        constexpr Real gravity = 9.81;
        constexpr Real duration = 0.5;
        constexpr int steps = 120;
        constexpr Real dt = duration / steps;
        auto world = laboratory({ 0.0, -gravity });
        const math::Vec2 normal { -std::sin(angle), std::cos(angle) };
        const math::Vec2 downhill { -std::cos(angle), -std::sin(angle) };
        BodyDefinition ramp;
        ramp.type = BodyType::static_body;
        ramp.position_m = normal * -0.25;
        ramp.orientation_rad = angle;
        Collider surface;
        surface.shape = make_box(30.0, 0.5);
        surface.material.static_friction = static_friction;
        surface.material.kinetic_friction = kinetic_friction;
        surface.material.restitution = 0.0;
        ramp.colliders.push_back(surface);
        world.create_body(ramp);
        auto block = ramp;
        block.type = BodyType::dynamic_body;
        block.position_m = normal * 0.25;
        block.linear_velocity_m_s = downhill * initial_speed;
        block.fixed_rotation = true;
        block.colliders.front().shape = make_box(1.0, 0.5);
        const auto id = world.create_body(block);
        world.find_body(id)->override_mass(2.0);
        const auto initial = world.find_body(id)->world_center_of_mass_m();
        const auto description = context("ramp", { { "angle", angle }, { "mu_s", static_friction }, { "mu_k", kinetic_friction }, { "v0", initial_speed } });
        advance(world, steps, dt, description);
        const auto* body = world.find_body(id);
        const auto acceleration = sticks ? 0.0 : gravity * (std::sin(angle) - kinetic_friction * std::cos(angle));
        const auto expected_speed = initial_speed + acceleration * duration;
        // Semi-implicit Euler's exact constant-acceleration quadrature includes T*dt/2.
        const auto expected_travel = initial_speed * duration + 0.5 * acceleration * duration * (duration + dt);
        RIGIDBODIES_EXPECT_NEAR(math::dot(body->linear_velocity_m_s(), downhill), expected_speed, 2.0e-8, description + " downhill speed");
        RIGIDBODIES_EXPECT_NEAR(math::dot(body->world_center_of_mass_m() - initial, downhill), expected_travel, 2.0e-8, description + " downhill displacement");
        RIGIDBODIES_EXPECT_NEAR(math::dot(body->linear_velocity_m_s(), normal), 0.0, 2.0e-8, description + " supported normal velocity");
        RIGIDBODIES_EXPECT_NEAR(math::dot(body->world_center_of_mass_m() - initial, normal), 0.0, 2.0e-8, description + " surface height");
        RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), angle, 1.0e-12, description + " fixed-rotation block");
    }

    RIGIDBODIES_TEST("frictionless inclined blocks reproduce g sin theta")
    {
        for (const Real angle : { 0.15, 0.5, 0.85 })
            verify_ramp(angle, 0.0, 0.0, 0.0, false);
    }

    RIGIDBODIES_TEST("inclined blocks remain at rest below the static friction angle")
    {
        for (const Real angle : { 0.1, 0.3, 0.45 })
            verify_ramp(angle, 0.5, 0.3, 0.0, true);
    }

    RIGIDBODIES_TEST("inclined blocks slide above static capacity with kinetic Coulomb acceleration")
    {
        for (const Real angle : { 0.45, 0.6 })
            for (const Real initial_speed : { 0.0, 0.7 })
                verify_ramp(angle, 0.3, 0.2, initial_speed, false);
    }

    RIGIDBODIES_TEST("small-angle pendulum follows the analytical phase within separate model and time-step bounds")
    {
        constexpr Real gravity = 9.81;
        constexpr int steps = 4096;
        for (const Real length : { 0.7, 1.3 })
            for (const Real amplitude : { 0.02, 0.04 })
            {
                auto world = laboratory({ 0.0, -gravity });
                auto anchor = circle({});
                anchor.type = BodyType::static_body;
                const auto fixed = world.create_body(anchor);
                auto bob = circle({ length * std::sin(amplitude), -length * std::cos(amplitude) });
                bob.fixed_rotation = true;
                const auto moving = world.create_body(bob);
                world.find_body(moving)->override_mass(2.0);
                DistanceJointDefinition definition;
                definition.first = fixed;
                definition.second = moving;
                definition.length_m = length;
                world.add_constraint(std::make_shared<JointConstraint>(definition));
                const auto frequency = std::sqrt(gravity / length);
                const auto period = 2.0 * math::pi / frequency;
                const auto dt = period / steps;
                // Duhamel's formula and |sin(theta)-theta| <= |theta|^3/6 give
                // this conservative small-angle model bound for an undamped libration.
                const auto numerical_bound = 2.0 * amplitude * frequency * dt;
                for (int step = 1; step <= steps; ++step)
                {
                    world.step(dt);
                    if (step % 64 != 0)
                        continue;
                    const auto time = step * dt;
                    const auto model_bound = frequency * time * amplitude * amplitude * amplitude / 6.0;
                    const auto position = world.find_body(moving)->world_center_of_mass_m();
                    const auto angle = std::atan2(position.x, -position.y);
                    const auto description = context("pendulum", { { "length", length }, { "amplitude", amplitude }, { "time", time }, { "dt", dt } });
                    RIGIDBODIES_EXPECT_NEAR(angle, amplitude * std::cos(frequency * time), model_bound + numerical_bound, description + " small-angle phase");
                    RIGIDBODIES_EXPECT_NEAR(math::length(position), length, 2.0e-6, description + " rod length");
                }
                RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "pendulum never reaches a numerical cap");
            }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
