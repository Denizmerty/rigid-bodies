#pragma once

#include <rigidbodies/physics/contact.hpp>

#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    class World;

    // Tuning shared by every solver implementation. The values are exposed because they are part
    // of what the playground explains: raising the iteration count visibly steadies a stack, and
    // the slop and correction settings are the difference between objects that rest and objects
    // that tremble.
    struct SolverSettings
    {
        // Velocity passes per step. More passes propagate impulses further through a stack.
        int velocity_iterations { 8 };

        // Position passes per step, used to remove residual overlap.
        int position_iterations { 3 };

        // Overlap tolerated without correction, in metres. A small allowance keeps resting
        // contacts from alternating between pushing apart and falling back together.
        Real linear_slop_m { 0.005 };

        // Fraction of the excess overlap removed per position pass.
        Real position_correction_fraction { 0.2 };

        // Separating speed below which an impact is treated as resting contact and restitution is
        // suppressed, in metres per second. Without it a resting object never stops bouncing.
        Real restitution_threshold_m_s { 1.0 };

        // Reuses the impulses accumulated on matching contact features from the previous step.
        bool warm_starting { true };

        // Maximum pseudo-position correction per pass. These impulses alter placement only.
        Real maximum_position_correction_m { 0.2 };
    };

    // Applies the impulses that keep touching bodies from interpenetrating and that produce
    // bouncing and friction.
    //
    // The world calls it between velocity and position integration, then again for placement-only
    // overlap correction. Force integration and the response to impacts stay independent.
    class ContactSolver
    {
    public:
        ContactSolver() = default;
        ContactSolver(const ContactSolver&) = default;
        ContactSolver(ContactSolver&&) = default;
        ContactSolver& operator=(const ContactSolver&) = default;
        ContactSolver& operator=(ContactSolver&&) = default;
        virtual ~ContactSolver() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<ContactSolver> clone() const
        {
            throw std::logic_error("ContactSolver does not support snapshots");
        }

        [[nodiscard]] virtual std::string_view name() const = 0;

        // Caches per-contact quantities and applies warm-start impulses.
        virtual void prepare(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) = 0;

        // Corrects velocities so that touching bodies separate rather than interpenetrate.
        virtual void solve_velocity(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) = 0;

        // Removes the residual overlap that a velocity-only solution leaves behind.
        virtual void solve_position(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) = 0;
    };

    using ContactSolverPtr = std::shared_ptr<ContactSolver>;

    // Accumulated unilateral normal impulses, a two-point normal block solve, Coulomb friction,
    // and separate nonlinear position correction. Prepared data contains identifiers and values,
    // never pointers into a world, so snapshots can clone a solver independently.
    class SequentialImpulseContactSolver final : public ContactSolver
    {
    public:
        [[nodiscard]] std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<SequentialImpulseContactSolver>(*this);
        }
        [[nodiscard]] std::string_view name() const override;
        void prepare(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;
        void solve_velocity(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;
        void solve_position(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;

    private:
        struct PreparedPoint
        {
            math::Vec2 radius_first_m;
            math::Vec2 radius_second_m;
            Real normal_mass {};
            Real tangent_mass {};
            Real target_normal_velocity_m_s {};
            Real incoming_impact_speed_m_s {};
            bool friction_enabled { false };
        };
        struct PreparedManifold
        {
            BodyId first;
            BodyId second;
            math::Vec2 normal;
            math::Vec2 tangent;
            std::array<PreparedPoint, maximum_manifold_points> points;
            std::size_t point_count {};
            Real static_friction {};
            Real kinetic_friction {};
            Real rolling_friction_m {};
            Real spinning_friction_m {};
            Real k11 {}, k12 {}, k22 {};
            bool block_normal { false };
            bool active { false };
        };
        std::vector<PreparedManifold> prepared_;
    };

    // Null implementation that leaves every velocity untouched. It pairs with the null narrow
    // phase for worlds that simulate free motion without collision response.
    class NullContactSolver final : public ContactSolver
    {
    public:
        [[nodiscard]] std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<NullContactSolver>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void prepare(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;
        void solve_velocity(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;
        void solve_position(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real time_step_s) override;
    };

} // namespace rigidbodies::physics
