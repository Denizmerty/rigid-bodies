#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <memory>
#include <functional>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    struct ForceSample
    {
        math::Vec2 force_n {};
        Real torque_n_m {};
    };

    // The callback samples the same step's force sources at a trial state and a time offset from
    // the start of the step. It includes pending direct forces, but excludes numerical damping.
    using ForceEvaluator = std::function<ForceSample(const RigidBody&, Real elapsed_offset_s)>;
    // Every body is at the same stage time. The evaluator may replace prescribed kinematic
    // states before sampling and returns one load per body in the same canonical order.
    using CoupledForceEvaluator = std::function<std::vector<ForceSample>(std::vector<RigidBody>&, Real elapsed_offset_s)>;

    struct IntegratedMotion
    {
        math::Vec2 center_position_m {};
        Real orientation_rad {};
        math::Vec2 linear_velocity_m_s {};
        Real angular_velocity_rad_s {};
    };

    // Advances one body over one step. Velocity and position updates are separate so the contact
    // solver can correct force-updated velocities before the bodies move.
    //
    // The shared interface lets an experiment compare integration methods.
    class Integrator
    {
    public:
        Integrator() = default;
        Integrator(const Integrator&) = default;
        Integrator(Integrator&&) = default;
        Integrator& operator=(const Integrator&) = default;
        Integrator& operator=(Integrator&&) = default;
        virtual ~Integrator() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<Integrator> clone() const
        {
            throw std::logic_error("Integrator does not support snapshots");
        }

        [[nodiscard]] virtual std::string_view name() const = 0;

        // Predict unconstrained motion. The default preserves existing custom integrators by
        // invoking their two split methods on a body copy. Stage-based methods override this and
        // request force samples without advancing live force-generator state.
        [[nodiscard]] virtual IntegratedMotion predict_motion(const RigidBody& body, Real time_step_s, const ForceEvaluator& evaluate) const;
        [[nodiscard]] virtual bool requires_force_evaluation() const
        {
            return false;
        }
        [[nodiscard]] virtual bool supports_coupled_prediction() const
        {
            return false;
        }
        [[nodiscard]] virtual std::vector<IntegratedMotion> predict_coupled(const std::vector<RigidBody>& bodies, Real time_step_s, const CoupledForceEvaluator& evaluate) const;

        // Applies the accumulated force and torque to the velocities.
        virtual void integrate_velocity(RigidBody& body, Real time_step_s) const = 0;

        // Applies the resulting velocities to the placement.
        virtual void integrate_position(RigidBody& body, Real time_step_s) const = 0;
    };

    using IntegratorPtr = std::shared_ptr<Integrator>;

    // Semi-implicit Euler: velocity is advanced first, then position is advanced using the new
    // velocity. It is only first-order accurate but does not inject energy the way explicit Euler
    // does, which is what makes it the usual choice for interactive rigid-body simulation.
    class SemiImplicitEulerIntegrator final : public Integrator
    {
    public:
        [[nodiscard]] std::shared_ptr<Integrator> clone() const override
        {
            return std::make_shared<SemiImplicitEulerIntegrator>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void integrate_velocity(RigidBody& body, Real time_step_s) const override;
        void integrate_position(RigidBody& body, Real time_step_s) const override;
    };

    // Velocity Verlet is second order and symplectic for position-dependent conservative forces.
    // With velocity-dependent forces or damping, the endpoint velocity is an explicit predictor;
    // the resulting second-order predictor-corrector does not have the symplectic guarantee.
    // Both staged methods must be advanced through World::step or predict_motion. The legacy
    // split methods reject calls because independent calls cannot reevaluate endpoint forces.
    class VelocityVerletIntegrator final : public Integrator
    {
    public:
        [[nodiscard]] std::shared_ptr<Integrator> clone() const override
        {
            return std::make_shared<VelocityVerletIntegrator>(*this);
        }
        [[nodiscard]] std::string_view name() const override;
        [[nodiscard]] bool requires_force_evaluation() const override
        {
            return true;
        }
        [[nodiscard]] IntegratedMotion predict_motion(const RigidBody& body, Real time_step_s, const ForceEvaluator& evaluate) const override;
        [[nodiscard]] bool supports_coupled_prediction() const override
        {
            return true;
        }
        [[nodiscard]] std::vector<IntegratedMotion> predict_coupled(const std::vector<RigidBody>& bodies, Real time_step_s, const CoupledForceEvaluator& evaluate) const override;
        void integrate_velocity(RigidBody& body, Real time_step_s) const override;
        void integrate_position(RigidBody& body, Real time_step_s) const override;
    };

    // Classical four-stage Runge-Kutta integrates placement and velocity together, with forces
    // reevaluated at each stage. Fourth-order accuracy assumes smooth force laws; it does not
    // imply fourth-order accuracy through discontinuous collision or constraint corrections.
    class RungeKutta4Integrator final : public Integrator
    {
    public:
        [[nodiscard]] std::shared_ptr<Integrator> clone() const override
        {
            return std::make_shared<RungeKutta4Integrator>(*this);
        }
        [[nodiscard]] std::string_view name() const override;
        [[nodiscard]] bool requires_force_evaluation() const override
        {
            return true;
        }
        [[nodiscard]] IntegratedMotion predict_motion(const RigidBody& body, Real time_step_s, const ForceEvaluator& evaluate) const override;
        [[nodiscard]] bool supports_coupled_prediction() const override
        {
            return true;
        }
        [[nodiscard]] std::vector<IntegratedMotion> predict_coupled(const std::vector<RigidBody>& bodies, Real time_step_s, const CoupledForceEvaluator& evaluate) const override;
        void integrate_velocity(RigidBody& body, Real time_step_s) const override;
        void integrate_position(RigidBody& body, Real time_step_s) const override;
    };

} // namespace rigidbodies::physics
