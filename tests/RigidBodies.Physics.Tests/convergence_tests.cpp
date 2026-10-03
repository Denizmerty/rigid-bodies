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

    struct Method
    {
        IntegratorPtr integrator;
        int order;
    };

    std::array<Method, 3> methods()
    {
        return { Method { std::make_shared<SemiImplicitEulerIntegrator>(), 1 },
            Method { std::make_shared<VelocityVerletIntegrator>(), 2 },
            Method { std::make_shared<RungeKutta4Integrator>(), 4 } };
    }

    World laboratory(const IntegratorPtr& method, math::Vec2 gravity = {})
    {
        WorldSettings settings;
        settings.gravity_m_s2 = gravity;
        settings.sleep.enabled = false;
        settings.solver.restitution_threshold_m_s = 0.0;
        World world { settings };
        world.set_integrator(method);
        world.set_parallel_settings({ 1, 64 });
        return world;
    }

    BodyId particle(World& world, Real position, Real velocity)
    {
        BodyDefinition definition;
        definition.position_m = { position, 0.0 };
        definition.linear_velocity_m_s = { velocity, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.25);
        collider.material.restitution = 1.0;
        collider.material.static_friction = 0.0;
        collider.material.kinetic_friction = 0.0;
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    void verify_order(const Method& method, const std::array<Real, 3>& errors, const std::string& description)
    {
        std::array<Real, 2> orders {};
        for (std::size_t level = 0; level + 1 < errors.size(); ++level)
        {
            RIGIDBODIES_EXPECT(std::isfinite(errors[level]) && errors[level + 1] > 1.0e-13, description + " errors stay above floating-point noise");
            const auto observed = std::log2(errors[level] / errors[level + 1]);
            orders[level] = observed;
            RIGIDBODIES_EXPECT_NEAR(observed, static_cast<Real>(method.order), 0.15, context(description, { { "refinement", static_cast<Real>(level) }, { "coarse_error", errors[level] }, { "fine_error", errors[level + 1] } }));
        }
        std::cout << context("convergence " + description, { { "E0", errors[0] }, { "E1", errors[1] }, { "E2", errors[2] }, { "p01", orders[0] }, { "p12", orders[1] } }) << '\n';
    }

    RIGIDBODIES_TEST("three-grid oscillator convergence follows first second and fourth order across frequency scales")
    {
        for (const auto& method : methods())
            for (const Real frequency : { 0.8, 2.0, 4.0 })
            {
                std::array<Real, 3> errors {};
                const Real amplitude = 0.7;
                const Real duration = 1.3 / frequency;
                for (std::size_t level = 0; level < errors.size(); ++level)
                {
                    auto world = laboratory(method.integrator);
                    const auto id = particle(world, amplitude, 0.0);
                    world.add_force_generator(id, std::make_shared<PointAttractor>(math::Vec2 {}, frequency * frequency));
                    const int steps = 24 << level;
                    for (int step = 0; step < steps; ++step)
                        world.step(duration / steps);
                    const auto* body = world.find_body(id);
                    const auto position_error = body->world_center_of_mass_m().x / amplitude - std::cos(frequency * duration);
                    const auto velocity_error = body->linear_velocity_m_s().x / (amplitude * frequency) + std::sin(frequency * duration);
                    errors[level] = std::hypot(position_error, velocity_error);
                    RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "oscillator convergence excludes numerical clamping");
                }
                verify_order(method, errors, context(method.integrator->name(), { { "frequency", frequency } }));
            }
    }

    class ViscousForce final : public ForceGenerator
    {
    public:
        explicit ViscousForce(Real rate) : rate_(rate)
        {
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<ViscousForce>(*this);
        }
        std::string_view name() const override
        {
            return "validation_viscous_force";
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center(body.linear_velocity_m_s() * (-rate_ * body.mass_properties().mass_kg));
        }

    private:
        Real rate_;
    };

    RIGIDBODIES_TEST("velocity-dependent force convergence agrees with exact terminal-velocity transients")
    {
        for (const auto& method : methods())
            for (const Real decay_rate : { 0.5, 1.5, 3.0 })
            {
                constexpr Real initial_velocity = -0.8;
                constexpr Real terminal_velocity = 1.1;
                const Real duration = 1.1 / decay_rate;
                const auto exact_velocity = terminal_velocity + (initial_velocity - terminal_velocity) * std::exp(-decay_rate * duration);
                const auto exact_position = terminal_velocity * duration + (initial_velocity - terminal_velocity) * (-std::expm1(-decay_rate * duration)) / decay_rate;
                std::array<Real, 3> errors {};
                for (std::size_t level = 0; level < errors.size(); ++level)
                {
                    auto world = laboratory(method.integrator, { terminal_velocity * decay_rate, 0.0 });
                    const auto id = particle(world, 0.0, initial_velocity);
                    world.add_force_generator(id, std::make_shared<ViscousForce>(decay_rate));
                    const int steps = 12 << level;
                    for (int step = 0; step < steps; ++step)
                        world.step(duration / steps);
                    const auto* body = world.find_body(id);
                    // The common velocity scale makes position and velocity errors dimensionless.
                    const auto scale = terminal_velocity - initial_velocity;
                    errors[level] = std::hypot((body->world_center_of_mass_m().x - exact_position) * decay_rate / scale,
                        (body->linear_velocity_m_s().x - exact_velocity) / scale);
                    RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "viscous transient excludes numerical clamping");
                }
                verify_order(method, errors, context(method.integrator->name(), { { "decay_rate", decay_rate } }));
            }
    }

    RIGIDBODIES_TEST("constrained pendulum refinement separates solver discretization from small-angle model error")
    {
        constexpr Real gravity = 9.81;
        constexpr Real amplitude = 0.002;
        const Method method { std::make_shared<SemiImplicitEulerIntegrator>(), 1 };
        for (const Real length : { 0.7, 1.3 })
        {
            const auto frequency = std::sqrt(gravity / length);
            const auto duration = 0.5 * math::pi / frequency;
            const auto model_bound = frequency * duration * amplitude * amplitude * amplitude / 6.0;
            std::array<Real, 3> errors {};
            for (std::size_t level = 0; level < errors.size(); ++level)
            {
                auto world = laboratory(method.integrator, { 0.0, -gravity });
                BodyDefinition anchor;
                anchor.type = BodyType::static_body;
                Collider collider;
                collider.shape = make_circle(0.02);
                anchor.colliders.push_back(collider);
                const auto fixed = world.create_body(anchor);
                auto bob = anchor;
                bob.type = BodyType::dynamic_body;
                bob.position_m = { length * std::sin(amplitude), -length * std::cos(amplitude) };
                bob.fixed_rotation = true;
                const auto moving = world.create_body(bob);
                world.find_body(moving)->override_mass(1.0);
                DistanceJointDefinition joint;
                joint.first = fixed;
                joint.second = moving;
                joint.length_m = length;
                world.add_constraint(std::make_shared<JointConstraint>(joint));
                const int steps = 128 << level;
                for (int step = 0; step < steps; ++step)
                    world.step(duration / steps);
                const auto position = world.find_body(moving)->world_center_of_mass_m();
                errors[level] = std::abs(std::atan2(position.x, -position.y) - amplitude * std::cos(frequency * duration));
                RIGIDBODIES_EXPECT(errors[level] > 100.0 * model_bound, "pendulum discretization signal exceeds the bounded model mismatch");
                RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "pendulum refinement excludes numerical clamping");
            }
            verify_order(method, errors, context("constrained pendulum", { { "length", length }, { "amplitude", amplitude } }));
        }
    }

    RIGIDBODIES_TEST("an off-grid elastic impact obeys a shrinking first-order event-location envelope")
    {
        for (const auto& method : methods())
            for (const int steps : { 32, 64, 128, 256 })
            {
                constexpr Real duration = 0.8;
                constexpr Real initial_position = -1.337;
                constexpr Real speed = 3.0;
                constexpr Real impact_position = -0.25;
                constexpr Real impact_time = (impact_position - initial_position) / speed;
                constexpr Real exact_final_position = impact_position - speed * (duration - impact_time);
                const Real dt = duration / steps;
                auto world = laboratory(method.integrator);
                const auto id = particle(world, initial_position, speed);
                BodyDefinition wall;
                wall.type = BodyType::static_body;
                wall.position_m = { 0.25, 0.0 };
                Collider collider;
                collider.shape = make_box(0.5, 10.0);
                collider.material.restitution = 1.0;
                collider.material.static_friction = 0.0;
                collider.material.kinetic_friction = 0.0;
                wall.colliders.push_back(collider);
                world.create_body(wall);
                int beginnings = 0;
                for (int step = 0; step < steps; ++step)
                {
                    world.step(dt);
                    for (const auto& event : world.contact_events())
                        if (event.kind == CollisionEventKind::begin)
                            ++beginnings;
                }
                const auto description = context(method.integrator->name(), { { "dt", dt }, { "impact_time", impact_time } });
                const auto* body = world.find_body(id);
                // Moving the velocity discontinuity by one step displaces the reflected path
                // by at most |v_before-v_after|*dt. Contact margin contributes both ways.
                const auto event_envelope = 2.0 * speed * dt + 2.0 * world.settings().collision.contact_margin_m;
                RIGIDBODIES_EXPECT_NEAR(body->position_m().x, exact_final_position, event_envelope, description + " reflected trajectory");
                RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, -speed, 2.0e-10, description + " post-impact velocity");
                RIGIDBODIES_EXPECT_NEAR(body->kinetic_energy_j(), 0.5 * speed * speed, 2.0e-9, description + " elastic energy");
                RIGIDBODIES_EXPECT(beginnings == 1, description + " exactly one impact begins");
                RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, description + " excludes numerical clamping");
            }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
