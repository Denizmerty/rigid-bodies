#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <memory>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    class World;

    // A scalar impulse equation J*v = target with bounds on the total substep impulse.
    // Linear Jacobians multiply centre-of-mass velocity; angular Jacobians multiply spin.
    struct ConstraintRow
    {
        BodyId first, second;
        math::Vec2 linear_first, linear_second;
        Real angular_first {}, angular_second {};
        Real target_velocity {};
        Real lower_impulse { -std::numeric_limits<Real>::infinity() };
        Real upper_impulse { std::numeric_limits<Real>::infinity() };
        Real impulse {};
    };

    // A relationship between two bodies that the solver enforces by applying impulses, such as a
    // pin joint, a distance rod, a spring, or a motor.
    //
    // The three-call shape is the one every impulse-based solver needs: a preparation pass that
    // caches per-step quantities, a velocity pass that may run several times, and an optional
    // position pass that removes the drift a velocity-only solution leaves behind.
    class Constraint
    {
    public:
        Constraint() = default;
        Constraint(const Constraint&) = default;
        Constraint(Constraint&&) = default;
        Constraint& operator=(const Constraint&) = default;
        Constraint& operator=(Constraint&&) = default;
        virtual ~Constraint() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<Constraint> clone() const
        {
            throw std::logic_error("Constraint does not support snapshots");
        }

        [[nodiscard]] virtual std::string_view name() const = 0;

        [[nodiscard]] virtual BodyId first_body() const = 0;
        [[nodiscard]] virtual BodyId second_body() const = 0;

        virtual void prepare(World& world, Real time_step_s) = 0;
        virtual void solve_velocity(World& world, Real time_step_s) = 0;

        // Returns true when the remaining positional error is within tolerance.
        virtual bool solve_position(World& world, Real time_step_s);

        // Optional coupled-solver seam. prepare builds and warm-starts these rows. The graph
        // solver writes their accumulated impulses directly. Older custom constraints retain
        // their sequential callbacks by returning nullptr.
        [[nodiscard]] virtual std::vector<ConstraintRow>* velocity_rows()
        {
            return nullptr;
        }
        virtual void finalize_velocity(World&, Real)
        {
        }
        // An endpoint property edit invalidates warm-start impulses without changing the
        // relationship or repairing a broken link. Stateful custom constraints should override.
        virtual void invalidate_cached_impulses() noexcept
        {
        }
        [[nodiscard]] virtual bool is_broken() const
        {
            return false;
        }
        [[nodiscard]] virtual bool collide_connected() const
        {
            return true;
        }
        [[nodiscard]] virtual bool has_active_drive() const
        {
            return false;
        }
        [[nodiscard]] virtual bool supports_sleeping_load() const
        {
            return false;
        }
        [[nodiscard]] virtual Real broken_force_n() const
        {
            return 0.0;
        }
        [[nodiscard]] virtual Real broken_torque_n_m() const
        {
            return 0.0;
        }
        [[nodiscard]] virtual std::uint64_t revision() const
        {
            return revision_;
        }

        [[nodiscard]] bool is_enabled() const
        {
            return enabled_;
        }

        void set_enabled(bool value)
        {
            if (enabled_ != value)
                ++revision_;
            enabled_ = value;
        }

    private:
        bool enabled_ { true };
        std::uint64_t revision_ {};
    };

    using ConstraintPtr = std::shared_ptr<Constraint>;

} // namespace rigidbodies::physics
