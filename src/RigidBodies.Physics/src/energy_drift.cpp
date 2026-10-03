#include <rigidbodies/physics/energy_drift.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <typeinfo>

namespace rigidbodies::physics
{
    EnergyDriftReport measure_harmonic_energy_drift(const Integrator& integrator, const EnergyDriftSettings& settings)
    {
        if (!math::is_finite(settings.duration_s) || settings.duration_s <= 0.0 ||
            !math::is_finite(settings.time_step_s) || settings.time_step_s <= 0.0 ||
            !math::is_finite(settings.mass_kg) || settings.mass_kg <= 0.0 ||
            !math::is_finite(settings.stiffness_n_m) || settings.stiffness_n_m <= 0.0 ||
            !math::is_finite(settings.initial_displacement_m) || !math::is_finite(settings.initial_velocity_m_s))
        {
            throw std::invalid_argument("Energy comparison requires finite positive duration, step, mass and stiffness, and finite initial state");
        }
        auto step_ratio = settings.duration_s / settings.time_step_s;
        const auto nearest_integer = std::round(step_ratio);
        const auto ratio_tolerance = 8.0 * std::numeric_limits<Real>::epsilon() * std::max(1.0, std::abs(step_ratio));
        if (std::abs(step_ratio - nearest_integer) <= ratio_tolerance)
        {
            step_ratio = nearest_integer;
        }
        const auto requested_steps = std::max(1.0, std::ceil(step_ratio));
        constexpr Real maximum_comparison_steps = 10000000.0;
        if (!math::is_finite(requested_steps) || requested_steps > maximum_comparison_steps)
        {
            throw std::invalid_argument("Energy comparison is limited to 10000000 steps per integrator; increase the step or reduce the duration");
        }
        auto method = integrator.clone();
        const auto* cloned_method = method.get();
        if (!cloned_method || cloned_method == &integrator || typeid(*cloned_method) != typeid(integrator))
        {
            throw std::logic_error("Energy comparison requires an independent integrator clone of the original type");
        }

        WorldSettings environment;
        environment.gravity_m_s2 = {};
        World world { environment };
        world.set_integrator(std::move(method));
        BodyDefinition definition;
        definition.position_m = { settings.initial_displacement_m, 0.0 };
        definition.linear_velocity_m_s = { settings.initial_velocity_m_s, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.5);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, "energy_subject");
        if (world.statistics().limit_event_count > 0)
        {
            throw std::overflow_error("Energy comparison initial state exceeds the simulation motion limits");
        }
        auto* body = world.find_body(id);
        body->override_mass(settings.mass_kg);
        body->set_sleep_enabled(false);
        world.add_force_generator(id, std::make_shared<PointAttractor>(math::Vec2 {}, settings.stiffness_n_m));

        const auto energy = [&]()
        {
            return body->kinetic_energy_j() + 0.5 * settings.stiffness_n_m * math::length_squared(body->world_center_of_mass_m());
        };
        EnergyDriftReport report;
        report.integrator_name = std::string { integrator.name() };
        report.initial_energy_j = energy();
        if (!math::is_finite(report.initial_energy_j))
        {
            throw std::invalid_argument("Energy comparison initial energy exceeds the finite numeric range");
        }
        report.final_energy_j = report.initial_energy_j;
        const auto steps = static_cast<std::size_t>(requested_steps);
        for (std::size_t index = 0; index < steps; ++index)
        {
            // Compute the remaining duration from the index, avoiding accumulated rounding in
            // the loop condition and an accidental extra near-zero step at the end.
            const auto elapsed = static_cast<Real>(index) * settings.time_step_s;
            const auto dt = index + 1 == steps ? settings.duration_s - elapsed : settings.time_step_s;
            if (dt <= 0.0)
            {
                break;
            }
            world.step(dt);
            if (world.statistics().limit_event_count > 0)
            {
                throw std::overflow_error("Energy comparison exceeded the simulation motion limits; reduce the step or initial energy");
            }
            ++report.step_count;
            report.final_energy_j = energy();
            if (!math::is_finite(report.final_energy_j))
            {
                throw std::overflow_error("Energy comparison became non-finite; reduce the time step or initial energy");
            }
            report.final_drift_j = report.final_energy_j - report.initial_energy_j;
            report.maximum_absolute_drift_j = std::max(report.maximum_absolute_drift_j, std::abs(report.final_drift_j));
        }
        report.elapsed_time_s = world.statistics().elapsed_time_s;
        report.maximum_relative_drift = report.initial_energy_j > 0.0 ? report.maximum_absolute_drift_j / report.initial_energy_j : 0.0;
        return report;
    }

    std::vector<EnergyDriftReport> compare_harmonic_energy_drift(const EnergyDriftSettings& settings)
    {
        return { measure_harmonic_energy_drift(SemiImplicitEulerIntegrator {}, settings),
            measure_harmonic_energy_drift(VelocityVerletIntegrator {}, settings),
            measure_harmonic_energy_drift(RungeKutta4Integrator {}, settings) };
    }
} // namespace rigidbodies::physics
