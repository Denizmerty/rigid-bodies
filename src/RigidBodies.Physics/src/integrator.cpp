#include <rigidbodies/physics/integrator.hpp>

#include <algorithm>

namespace rigidbodies::physics
{
    namespace
    {
        IntegratedMotion motion_of(const RigidBody& body)
        {
            return { body.world_center_of_mass_m(), body.orientation_rad(), body.linear_velocity_m_s(), body.angular_velocity_rad_s() };
        }

        struct Derivative
        {
            math::Vec2 position_rate;
            Real orientation_rate;
            math::Vec2 velocity_rate;
            Real angular_velocity_rate;
        };

        Derivative derivative(const RigidBody& body, const ForceSample& force)
        {
            return { body.linear_velocity_m_s(), body.angular_velocity_rad_s(), force.force_n * body.inverse_mass() - body.linear_velocity_m_s() * body.linear_damping(), force.torque_n_m * body.inverse_inertia() - body.angular_velocity_rad_s() * body.angular_damping() };
        }

        IntegratedMotion advance(const IntegratedMotion& initial, const Derivative& rate, Real dt)
        {
            return { initial.center_position_m + rate.position_rate * dt,
                initial.orientation_rad + rate.orientation_rate * dt,
                initial.linear_velocity_m_s + rate.velocity_rate * dt,
                initial.angular_velocity_rad_s + rate.angular_velocity_rate * dt };
        }

        RigidBody trial_body(const RigidBody& source, const IntegratedMotion& motion)
        {
            auto body = source;
            body.clear_accumulators();
            body.set_simulated_velocity(motion.linear_velocity_m_s, motion.angular_velocity_rad_s);
            const math::Rotation2 rotation { motion.orientation_rad };
            body.set_simulated_pose(motion.center_position_m - math::rotate(rotation, body.mass_properties().center_of_mass_m), motion.orientation_rad);
            return body;
        }

        bool advances_dynamically(const RigidBody& body, Real dt)
        {
            return math::is_finite(dt) && dt > 0.0 && body.type() == BodyType::dynamic_body && body.is_awake();
        }

        IntegratedMotion non_dynamic_motion(const RigidBody& body, Real dt)
        {
            auto motion = motion_of(body);
            if (body.type() == BodyType::kinematic_body && math::is_finite(dt) && dt > 0.0)
            {
                motion.center_position_m += motion.linear_velocity_m_s * dt;
                motion.orientation_rad += motion.angular_velocity_rad_s * dt;
            }
            return motion;
        }

        [[noreturn]] void reject_split_stage()
        {
            throw std::logic_error("Staged integrators require World::step or predict_motion with a force evaluator");
        }
    }

    IntegratedMotion Integrator::predict_motion(const RigidBody& body, Real time_step_s, const ForceEvaluator&) const
    {
        if (!math::is_finite(time_step_s) || time_step_s <= 0.0 || (body.type() == BodyType::dynamic_body && !body.is_awake()))
        {
            return motion_of(body);
        }
        auto predicted = body;
        integrate_velocity(predicted, time_step_s);
        integrate_position(predicted, time_step_s);
        return motion_of(predicted);
    }

    std::vector<IntegratedMotion> Integrator::predict_coupled(const std::vector<RigidBody>&, Real, const CoupledForceEvaluator&) const
    {
        throw std::logic_error("This staged integrator does not support coupled spring forces");
    }

    std::vector<IntegratedMotion> VelocityVerletIntegrator::predict_coupled(const std::vector<RigidBody>& bodies, Real dt, const CoupledForceEvaluator& evaluate) const
    {
        if (!evaluate)
            throw std::invalid_argument("Coupled Verlet requires a force evaluator");
        std::vector<IntegratedMotion> result;
        std::vector<Derivative> initial_rates;
        std::vector<RigidBody> endpoints;
        result.reserve(bodies.size());
        initial_rates.reserve(bodies.size());
        endpoints.reserve(bodies.size());
        for (const auto& body : bodies)
        {
            auto motion = non_dynamic_motion(body, dt);
            const auto rate = derivative(body, { body.accumulated_force_n(), body.accumulated_torque_n_m() });
            if (advances_dynamically(body, dt))
            {
                motion = advance(motion_of(body), rate, dt);
                motion.center_position_m += rate.velocity_rate * (0.5 * dt * dt);
                motion.orientation_rad += rate.angular_velocity_rate * (0.5 * dt * dt);
            }
            result.push_back(motion);
            initial_rates.push_back(rate);
            endpoints.push_back(trial_body(body, motion));
        }
        const auto loads = evaluate(endpoints, dt);
        if (loads.size() != bodies.size())
            throw std::invalid_argument("Coupled force sample size must match body count");
        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            if (!advances_dynamically(bodies[index], dt))
                continue;
            const auto last = derivative(endpoints[index], loads[index]);
            result[index].linear_velocity_m_s = bodies[index].linear_velocity_m_s() + (initial_rates[index].velocity_rate + last.velocity_rate) * (0.5 * dt);
            result[index].angular_velocity_rad_s = bodies[index].angular_velocity_rad_s() + (initial_rates[index].angular_velocity_rate + last.angular_velocity_rate) * (0.5 * dt);
        }
        return result;
    }

    std::vector<IntegratedMotion> RungeKutta4Integrator::predict_coupled(const std::vector<RigidBody>& bodies, Real dt, const CoupledForceEvaluator& evaluate) const
    {
        if (!evaluate)
            throw std::invalid_argument("Coupled RK4 requires a force evaluator");
        std::vector<Derivative> first;
        first.reserve(bodies.size());
        for (const auto& body : bodies)
            first.push_back(derivative(body, { body.accumulated_force_n(), body.accumulated_torque_n_m() }));
        const auto stage = [&](const std::vector<Derivative>& previous, Real offset)
        {
            std::vector<RigidBody> probes;
            probes.reserve(bodies.size());
            for (std::size_t index = 0; index < bodies.size(); ++index)
            {
                const auto& body = bodies[index];
                const auto motion = advances_dynamically(body, dt) ? advance(motion_of(body), previous[index], offset) : non_dynamic_motion(body, offset);
                probes.push_back(trial_body(body, motion));
            }
            const auto loads = evaluate(probes, offset);
            if (loads.size() != bodies.size())
                throw std::invalid_argument("Coupled force sample size must match body count");
            std::vector<Derivative> rates;
            rates.reserve(bodies.size());
            for (std::size_t index = 0; index < bodies.size(); ++index)
                rates.push_back(derivative(probes[index], loads[index]));
            return rates;
        };
        const auto second = stage(first, dt * 0.5);
        const auto third = stage(second, dt * 0.5);
        const auto fourth = stage(third, dt);
        std::vector<IntegratedMotion> result;
        result.reserve(bodies.size());
        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            const auto& body = bodies[index];
            if (!advances_dynamically(body, dt))
            {
                result.push_back(non_dynamic_motion(body, dt));
                continue;
            }
            const auto& a = first[index];
            const auto& b = second[index];
            const auto& c = third[index];
            const auto& d = fourth[index];
            const Derivative average {
                (a.position_rate + b.position_rate * 2.0 + c.position_rate * 2.0 + d.position_rate) / 6.0,
                (a.orientation_rate + b.orientation_rate * 2.0 + c.orientation_rate * 2.0 + d.orientation_rate) / 6.0,
                (a.velocity_rate + b.velocity_rate * 2.0 + c.velocity_rate * 2.0 + d.velocity_rate) / 6.0,
                (a.angular_velocity_rate + b.angular_velocity_rate * 2.0 + c.angular_velocity_rate * 2.0 + d.angular_velocity_rate) / 6.0
            };
            result.push_back(advance(motion_of(body), average, dt));
        }
        return result;
    }

    std::string_view SemiImplicitEulerIntegrator::name() const
    {
        return "semi_implicit_euler";
    }

    void SemiImplicitEulerIntegrator::integrate_velocity(RigidBody& body, Real time_step_s) const
    {
        if (body.type() != BodyType::dynamic_body || !body.is_awake())
        {
            return;
        }
        auto linear_velocity = body.linear_velocity_m_s() + body.accumulated_force_n() * body.inverse_mass() * time_step_s;
        auto angular_velocity = body.angular_velocity_rad_s() + body.accumulated_torque_n_m() * body.inverse_inertia() * time_step_s;
        // This rational damping factor is stable for any positive step and cannot reverse motion.
        // Staged integrators instead sample the continuous damping acceleration at every stage.
        linear_velocity *= 1.0 / (1.0 + time_step_s * body.linear_damping());
        angular_velocity *= 1.0 / (1.0 + time_step_s * body.angular_damping());
        body.set_simulated_velocity(linear_velocity, angular_velocity);
    }

    void SemiImplicitEulerIntegrator::integrate_position(RigidBody& body, Real time_step_s) const
    {
        if (body.type() == BodyType::static_body || (body.type() == BodyType::dynamic_body && !body.is_awake()))
        {
            return;
        }
        // A free body rotates about its centre of mass, which can differ from the body origin.
        const auto center_of_mass = body.world_center_of_mass_m() + body.linear_velocity_m_s() * time_step_s;
        const auto orientation = body.orientation_rad() + body.angular_velocity_rad_s() * time_step_s;
        const math::Rotation2 rotation { orientation };
        body.set_simulated_pose(center_of_mass - math::rotate(rotation, body.mass_properties().center_of_mass_m), orientation);
    }

    std::string_view VelocityVerletIntegrator::name() const
    {
        return "velocity_verlet";
    }

    IntegratedMotion VelocityVerletIntegrator::predict_motion(const RigidBody& body, Real dt, const ForceEvaluator& evaluate) const
    {
        if (!advances_dynamically(body, dt))
        {
            return non_dynamic_motion(body, dt);
        }
        if (!evaluate)
        {
            throw std::invalid_argument("Velocity Verlet requires a force evaluator");
        }
        const auto initial = motion_of(body);
        const auto first = derivative(body, { body.accumulated_force_n(), body.accumulated_torque_n_m() });
        auto result = advance(initial, first, dt);
        result.center_position_m += first.velocity_rate * (0.5 * dt * dt);
        result.orientation_rad += first.angular_velocity_rate * (0.5 * dt * dt);
        const auto endpoint = trial_body(body, result);
        const auto last = derivative(endpoint, evaluate(endpoint, dt));
        result.linear_velocity_m_s = initial.linear_velocity_m_s + (first.velocity_rate + last.velocity_rate) * (0.5 * dt);
        result.angular_velocity_rad_s = initial.angular_velocity_rad_s + (first.angular_velocity_rate + last.angular_velocity_rate) * (0.5 * dt);
        return result;
    }

    void VelocityVerletIntegrator::integrate_velocity(RigidBody&, Real) const
    {
        reject_split_stage();
    }

    void VelocityVerletIntegrator::integrate_position(RigidBody&, Real) const
    {
        reject_split_stage();
    }

    std::string_view RungeKutta4Integrator::name() const
    {
        return "runge_kutta_4";
    }

    IntegratedMotion RungeKutta4Integrator::predict_motion(const RigidBody& body, Real dt, const ForceEvaluator& evaluate) const
    {
        if (!advances_dynamically(body, dt))
        {
            return non_dynamic_motion(body, dt);
        }
        if (!evaluate)
        {
            throw std::invalid_argument("Runge-Kutta 4 requires a force evaluator");
        }
        const auto initial = motion_of(body);
        const auto first = derivative(body, { body.accumulated_force_n(), body.accumulated_torque_n_m() });
        const auto second_body = trial_body(body, advance(initial, first, dt * 0.5));
        const auto second = derivative(second_body, evaluate(second_body, dt * 0.5));
        const auto third_body = trial_body(body, advance(initial, second, dt * 0.5));
        const auto third = derivative(third_body, evaluate(third_body, dt * 0.5));
        const auto fourth_body = trial_body(body, advance(initial, third, dt));
        const auto fourth = derivative(fourth_body, evaluate(fourth_body, dt));
        const Derivative average {
            (first.position_rate + second.position_rate * 2.0 + third.position_rate * 2.0 + fourth.position_rate) / 6.0,
            (first.orientation_rate + second.orientation_rate * 2.0 + third.orientation_rate * 2.0 + fourth.orientation_rate) / 6.0,
            (first.velocity_rate + second.velocity_rate * 2.0 + third.velocity_rate * 2.0 + fourth.velocity_rate) / 6.0,
            (first.angular_velocity_rate + second.angular_velocity_rate * 2.0 + third.angular_velocity_rate * 2.0 + fourth.angular_velocity_rate) / 6.0
        };
        return advance(initial, average, dt);
    }

    void RungeKutta4Integrator::integrate_velocity(RigidBody&, Real) const
    {
        reject_split_stage();
    }

    void RungeKutta4Integrator::integrate_position(RigidBody&, Real) const
    {
        reject_split_stage();
    }

} // namespace rigidbodies::physics
