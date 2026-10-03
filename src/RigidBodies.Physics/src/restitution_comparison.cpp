#include <rigidbodies/physics/restitution_comparison.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {

        std::string_view policy_name(MaterialMixing mixing)
        {
            switch (mixing)
            {
            case MaterialMixing::geometric_mean:
                return "geometric_mean";
            case MaterialMixing::arithmetic_mean:
                return "arithmetic_mean";
            case MaterialMixing::minimum:
                return "minimum";
            case MaterialMixing::maximum:
                return "maximum";
            }
            throw std::invalid_argument("Unknown restitution mixing policy");
        }

        void validate(const RestitutionComparisonSettings& settings)
        {
            const auto positive = [](Real value)
            {
                return math::is_finite(value) && value > 0.0;
            };
            const auto coefficient = [](Real value)
            {
                return math::is_finite(value) && value >= 0.0 && value <= 1.0;
            };
            if (!positive(settings.drop_height_m) || !positive(settings.ball_radius_m) || !positive(settings.mass_kg) ||
                !positive(settings.time_step_s) || !positive(settings.maximum_duration_s) ||
                !coefficient(settings.ball_restitution) || !coefficient(settings.floor_restitution) ||
                settings.maximum_steps == 0 || settings.maximum_steps > 1000000)
            {
                throw std::invalid_argument("Drop comparison needs finite positive dimensions, mass, duration and step, restitution in [0,1], and 1..1000000 maximum steps");
            }
        }

        Material frictionless_material(Real restitution)
        {
            auto material = materials::rubber();
            material.restitution = restitution;
            material.static_friction = 0.0;
            material.kinetic_friction = 0.0;
            material.rolling_friction_m = 0.0;
            material.spinning_friction_m = 0.0;
            return material;
        }

        void check_limits(const World& world)
        {
            if (world.statistics().limit_event_count > 0)
            {
                throw std::overflow_error("Drop comparison reached a motion limit; reduce the drop height or time step");
            }
            if (world.statistics().sweep_iteration_limit_count > 0)
            {
                throw std::runtime_error("Drop comparison exhausted collision sweep iterations");
            }
        }

    } // namespace

    RestitutionComparisonReport measure_restitution_drop(MaterialMixing mixing, const RestitutionComparisonSettings& settings)
    {
        validate(settings);
        RestitutionComparisonReport report;
        report.mixing = mixing;
        report.mixing_name = policy_name(mixing);
        report.mixed_restitution = mix_material_values(settings.ball_restitution, settings.floor_restitution, mixing);
        report.drop_height_m = settings.drop_height_m;
        report.theoretical_rebound_height_m = report.mixed_restitution * report.mixed_restitution * settings.drop_height_m;

        WorldSettings environment;
        environment.sleep.enabled = false;
        environment.collision.continuous = settings.continuous;
        environment.collision.restitution_mixing = mixing;
        environment.collision.contact_margin_m = 1.0e-6;
        environment.solver.linear_slop_m = 1.0e-6;
        environment.solver.restitution_threshold_m_s = 0.0;
        environment.solver.velocity_iterations = 12;
        environment.solver.position_iterations = 6;
        World world { environment };

        BodyDefinition ground;
        ground.name = "drop_floor";
        ground.type = BodyType::static_body;
        ground.position_m = { 0.0, -0.05 };
        Collider floor_collider;
        floor_collider.shape = make_box(4.0, 0.1);
        floor_collider.material = frictionless_material(settings.floor_restitution);
        ground.colliders.push_back(floor_collider);
        const auto floor_id = world.create_body(ground, "floor");

        BodyDefinition ball;
        ball.name = "drop_ball";
        ball.position_m = { 0.0, settings.drop_height_m + settings.ball_radius_m };
        Collider ball_collider;
        ball_collider.shape = make_circle(settings.ball_radius_m);
        ball_collider.material = frictionless_material(settings.ball_restitution);
        ball.colliders.push_back(ball_collider);
        const auto ball_id = world.create_body(ball, "ball");
        auto* body = world.find_body(ball_id);
        body->override_mass(settings.mass_kg);
        body->set_sleep_enabled(false);
        check_limits(world);

        bool impacted = false;
        bool completed = false;
        Real largest_approach_speed = 0.0;
        for (std::size_t index = 0; index < settings.maximum_steps; ++index)
        {
            const auto remaining = settings.maximum_duration_s - world.statistics().elapsed_time_s;
            const auto dt = std::min(settings.time_step_s, remaining);
            if (dt <= 0.0)
            {
                break;
            }
            const auto previous_velocity = body->linear_velocity_m_s().y;
            largest_approach_speed = std::max(largest_approach_speed, -previous_velocity);
            world.step(dt);
            ++report.step_count;
            check_limits(world);
            const auto height = body->world_center_of_mass_m().y - settings.ball_radius_m;
            const auto velocity = body->linear_velocity_m_s().y;
            if (!math::is_finite(height) || !math::is_finite(velocity))
            {
                throw std::overflow_error("Drop comparison produced a non-finite trajectory");
            }
            bool contact_impulse = false;
            Real contact_approach_speed = 0.0;
            for (const auto& manifold : world.manifolds())
            {
                if (manifold.is_sensor || !((manifold.first == ball_id && manifold.second == floor_id) || (manifold.first == floor_id && manifold.second == ball_id)))
                {
                    continue;
                }
                for (std::size_t point = 0; point < manifold.point_count; ++point)
                {
                    contact_impulse = contact_impulse || manifold.points[point].normal_impulse_n_s > 0.0;
                    contact_approach_speed = std::max(contact_approach_speed, -manifold.points[point].pre_solve_normal_velocity_m_s);
                }
            }
            if (!impacted && contact_impulse && largest_approach_speed > 0.0 &&
                (velocity > 1.0e-10 || (report.mixed_restitution == 0.0 && velocity >= -1.0e-10)))
            {
                impacted = true;
                report.impact_time_s = world.statistics().elapsed_time_s;
                report.impact_speed_m_s = std::max(largest_approach_speed, contact_approach_speed);
                report.rebound_speed_m_s = std::max(0.0, velocity);
                report.apex_time_s = report.impact_time_s;
                report.measured_rebound_height_m = std::max(0.0, height);
                if (velocity <= 1.0e-10)
                {
                    completed = true;
                    break;
                }
            }
            else if (impacted)
            {
                if (height > report.measured_rebound_height_m)
                {
                    report.measured_rebound_height_m = height;
                    report.apex_time_s = world.statistics().elapsed_time_s;
                }
                if (velocity <= 0.0)
                {
                    completed = true;
                    break;
                }
            }
        }
        if (!completed)
        {
            throw std::runtime_error(impacted ? "Drop comparison did not reach the first rebound apex within its duration or step budget"
                                              : "Drop comparison did not reach the first impact within its duration or step budget");
        }
        report.measured_restitution = std::sqrt(report.measured_rebound_height_m / settings.drop_height_m);
        report.height_error_m = report.measured_rebound_height_m - report.theoretical_rebound_height_m;
        return report;
    }

    std::vector<RestitutionComparisonReport> compare_restitution_drops(const RestitutionComparisonSettings& settings)
    {
        return { measure_restitution_drop(MaterialMixing::geometric_mean, settings),
            measure_restitution_drop(MaterialMixing::arithmetic_mean, settings),
            measure_restitution_drop(MaterialMixing::minimum, settings),
            measure_restitution_drop(MaterialMixing::maximum, settings) };
    }

} // namespace rigidbodies::physics
