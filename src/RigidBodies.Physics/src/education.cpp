#include <rigidbodies/physics/education.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <typeinfo>

namespace rigidbodies::physics
{
    namespace
    {
        bool positive(Real value)
        {
            return math::is_finite(value) && value > 0.0;
        }

        void check_limits(const World& world)
        {
            if (world.statistics().limit_event_count > 0)
                throw std::overflow_error("Reference experiment exceeded the simulation motion limits");
            if (world.statistics().sweep_iteration_limit_count > 0)
                throw std::runtime_error("Reference experiment exhausted collision sweep iterations");
        }

        void validate(Real restitution, const CollisionComparisonSettings& settings)
        {
            if (!math::is_finite(restitution) || restitution < 0.0 || restitution > 1.0 ||
                !positive(settings.first_mass_kg) || !positive(settings.second_mass_kg) ||
                !math::is_finite(settings.first_velocity_m_s) || !math::is_finite(settings.second_velocity_m_s) ||
                settings.first_velocity_m_s <= settings.second_velocity_m_s ||
                !positive(settings.radius_m) || !positive(settings.initial_gap_m) ||
                !positive(settings.time_step_s) || !positive(settings.maximum_duration_s) ||
                settings.maximum_steps == 0 || settings.maximum_steps > 1000000)
                throw std::invalid_argument("Collision comparison needs positive finite masses, dimensions and time bounds, approaching velocities, restitution in [0,1], and 1..1000000 steps");
        }

        BodyId add_disc(World& world, std::string name, Real position, Real velocity, Real mass, Real radius, Real restitution)
        {
            BodyDefinition definition;
            definition.name = name;
            definition.position_m = { position, 0.0 };
            definition.linear_velocity_m_s = { velocity, 0.0 };
            definition.sleep_enabled = false;
            Collider collider;
            collider.shape = make_circle(radius);
            collider.material = materials::rubber();
            collider.material.restitution = restitution;
            collider.material.static_friction = collider.material.kinetic_friction = 0.0;
            collider.material.rolling_friction_m = collider.material.spinning_friction_m = 0.0;
            definition.colliders.push_back(collider);
            const auto id = world.create_body(definition, std::move(name));
            world.find_body(id)->override_mass(mass);
            return id;
        }
    } // namespace

    std::optional<Real> oscillation_period_s(Real mass_kg, Real stiffness_n_m, Real damping_n_s_m)
    {
        if (!positive(mass_kg) || !positive(stiffness_n_m) || !math::is_finite(damping_n_s_m) || damping_n_s_m < 0.0)
            return std::nullopt;
        const auto decay = (damping_n_s_m / mass_kg) * 0.5;
        const auto frequency_squared = stiffness_n_m / mass_kg - decay * decay;
        if (!positive(frequency_squared))
            return std::nullopt;
        const auto period = math::two_pi / std::sqrt(frequency_squared);
        return positive(period) ? std::optional<Real> { period } : std::nullopt;
    }

    std::optional<Real> pendulum_period_s(Real length_m, Real gravity_m_s2)
    {
        if (!positive(length_m) || !positive(gravity_m_s2))
            return std::nullopt;
        const auto period = math::two_pi * std::sqrt(length_m / gravity_m_s2);
        return positive(period) ? std::optional<Real> { period } : std::nullopt;
    }

    std::optional<Real> critical_ramp_angle_rad(Real static_friction)
    {
        if (!math::is_finite(static_friction) || static_friction < 0.0)
            return std::nullopt;
        return std::atan(static_friction);
    }

    CollisionComparisonReport measure_head_on_collision(Real restitution, const CollisionComparisonSettings& settings)
    {
        validate(restitution, settings);
        CollisionComparisonReport report;
        report.restitution = restitution;
        report.label = restitution == 1.0 ? "Elastic" : restitution == 0.0 ? "Inelastic"
                                                                           : "Partially elastic";
        const auto first_mass = settings.first_mass_kg;
        const auto second_mass = settings.second_mass_kg;
        const auto total_mass = first_mass + second_mass;
        const auto relative_speed = settings.first_velocity_m_s - settings.second_velocity_m_s;
        report.incoming_momentum_kg_m_s = first_mass * settings.first_velocity_m_s + second_mass * settings.second_velocity_m_s;
        report.incoming_kinetic_energy_j = 0.5 * first_mass * settings.first_velocity_m_s * settings.first_velocity_m_s +
            0.5 * second_mass * settings.second_velocity_m_s * settings.second_velocity_m_s;
        const auto centre_velocity = report.incoming_momentum_kg_m_s / total_mass;
        report.predicted_first_velocity_m_s = centre_velocity - restitution * (second_mass / total_mass) * relative_speed;
        report.predicted_second_velocity_m_s = centre_velocity + restitution * (first_mass / total_mass) * relative_speed;
        report.predicted_energy_loss_j = 0.5 * (first_mass / total_mass) * second_mass * (1.0 - restitution * restitution) * relative_speed * relative_speed;
        if (!positive(total_mass) || !positive(report.incoming_kinetic_energy_j) ||
            !math::is_finite(report.incoming_momentum_kg_m_s) || !math::is_finite(report.predicted_energy_loss_j) ||
            !math::is_finite(report.predicted_first_velocity_m_s) || !math::is_finite(report.predicted_second_velocity_m_s))
            throw std::invalid_argument("Collision comparison exceeds the finite numerical range");

        WorldSettings environment;
        environment.gravity_m_s2 = {};
        environment.sleep.enabled = false;
        environment.collision.contact_margin_m = 1.0e-6;
        environment.solver.linear_slop_m = 1.0e-6;
        environment.solver.restitution_threshold_m_s = 0.0;
        World world { environment };
        const auto offset = settings.radius_m + 0.5 * settings.initial_gap_m;
        const auto first_id = add_disc(world, "first", -offset, settings.first_velocity_m_s, first_mass, settings.radius_m, restitution);
        const auto second_id = add_disc(world, "second", offset, settings.second_velocity_m_s, second_mass, settings.radius_m, restitution);
        const auto* first = world.find_body(first_id);
        const auto* second = world.find_body(second_id);
        check_limits(world);
        for (std::size_t index = 0; index < settings.maximum_steps; ++index)
        {
            const auto remaining = settings.maximum_duration_s - world.statistics().elapsed_time_s;
            const auto dt = std::min(settings.time_step_s, remaining);
            if (dt <= 0.0)
                break;
            world.step(dt);
            ++report.step_count;
            check_limits(world);
            const auto first_velocity = first->linear_velocity_m_s().x;
            const auto second_velocity = second->linear_velocity_m_s().x;
            // A speculative approach cap can change velocity before the actual restitution
            // response. Wait for the completed impact episode instead of mistaking that cap for
            // the collision. The world retains swept impacts even when their manifold ends.
            if (!world.impact_reports().empty())
            {
                report.measured_first_velocity_m_s = first_velocity;
                report.measured_second_velocity_m_s = second_velocity;
                report.outgoing_momentum_kg_m_s = first_mass * first_velocity + second_mass * second_velocity;
                report.outgoing_kinetic_energy_j = first->kinetic_energy_j() + second->kinetic_energy_j();
                report.impact_time_s = world.statistics().elapsed_time_s;
                if (!math::is_finite(report.outgoing_momentum_kg_m_s) || !math::is_finite(report.outgoing_kinetic_energy_j))
                    throw std::overflow_error("Collision comparison produced non-finite measured quantities");
                return report;
            }
        }
        throw std::runtime_error("Collision comparison did not reach impact within its duration or step budget");
    }

    std::vector<CollisionComparisonReport> compare_head_on_collisions(const CollisionComparisonSettings& settings)
    {
        return { measure_head_on_collision(1.0, settings), measure_head_on_collision(0.0, settings), measure_head_on_collision(0.5, settings) };
    }

    ProjectileComparisonReport measure_projectile_trajectory(const Integrator& integrator, const ProjectileComparisonSettings& settings)
    {
        if (!math::is_finite(settings.initial_position_m) || !math::is_finite(settings.initial_velocity_m_s) ||
            !math::is_finite(settings.gravity_m_s2) || !positive(settings.duration_s) || !positive(settings.time_step_s))
            throw std::invalid_argument("Projectile comparison needs finite vectors and positive finite duration and step");
        auto ratio = settings.duration_s / settings.time_step_s;
        const auto nearest_integer = std::round(ratio);
        if (std::abs(ratio - nearest_integer) <= 8.0 * std::numeric_limits<Real>::epsilon() * std::max(1.0, ratio))
            ratio = nearest_integer;
        const auto requested_steps = std::max(1.0, std::ceil(ratio));
        if (!math::is_finite(requested_steps) || requested_steps > 1000000.0)
            throw std::invalid_argument("Projectile comparison is limited to 1000000 steps per integrator");
        auto method = integrator.clone();
        const auto* cloned_method = method.get();
        if (!cloned_method || cloned_method == &integrator || typeid(*cloned_method) != typeid(integrator))
            throw std::logic_error("Projectile comparison requires an independent integrator clone of the original type");

        ProjectileComparisonReport report;
        report.integrator_name = std::string { integrator.name() };
        const auto duration = settings.duration_s;
        report.predicted_position_m = settings.initial_position_m + settings.initial_velocity_m_s * duration + settings.gravity_m_s2 * (0.5 * duration * duration);
        report.predicted_velocity_m_s = settings.initial_velocity_m_s + settings.gravity_m_s2 * duration;
        if (!math::is_finite(report.predicted_position_m) || !math::is_finite(report.predicted_velocity_m_s))
            throw std::invalid_argument("Projectile prediction exceeds the finite numerical range");
        WorldSettings environment;
        environment.gravity_m_s2 = settings.gravity_m_s2;
        environment.sleep.enabled = false;
        World world { environment };
        world.set_integrator(std::move(method));
        BodyDefinition body;
        body.name = "reference_projectile";
        body.position_m = settings.initial_position_m;
        body.linear_velocity_m_s = settings.initial_velocity_m_s;
        body.sleep_enabled = false;
        Collider collider;
        collider.shape = make_circle(0.05);
        body.colliders.push_back(collider);
        const auto id = world.create_body(body, body.name);
        check_limits(world);
        const auto steps = static_cast<std::size_t>(requested_steps);
        for (std::size_t index = 0; index < steps; ++index)
        {
            const auto dt = index + 1 == steps ? duration - static_cast<Real>(index) * settings.time_step_s : settings.time_step_s;
            if (dt <= 0.0)
                break;
            world.step(dt);
            ++report.step_count;
            check_limits(world);
        }
        report.measured_position_m = world.find_body(id)->world_center_of_mass_m();
        report.measured_velocity_m_s = world.find_body(id)->linear_velocity_m_s();
        report.position_error_m = math::length(report.measured_position_m - report.predicted_position_m);
        report.velocity_error_m_s = math::length(report.measured_velocity_m_s - report.predicted_velocity_m_s);
        report.elapsed_time_s = world.statistics().elapsed_time_s;
        return report;
    }

    std::vector<ProjectileComparisonReport> compare_projectile_trajectories(const ProjectileComparisonSettings& settings)
    {
        return { measure_projectile_trajectory(SemiImplicitEulerIntegrator {}, settings),
            measure_projectile_trajectory(VelocityVerletIntegrator {}, settings),
            measure_projectile_trajectory(RungeKutta4Integrator {}, settings) };
    }

} // namespace rigidbodies::physics
