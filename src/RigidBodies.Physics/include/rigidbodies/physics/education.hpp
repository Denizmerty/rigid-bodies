#pragma once

#include <rigidbodies/physics/integrator.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rigidbodies::physics
{

    // Linear single-degree-of-freedom oscillator: T = 2*pi/sqrt(k/m - (c/(2*m))^2).
    // The rotational analogue uses inertia, torsional stiffness, and angular damping instead.
    // Invalid inputs and critically/overdamped systems have no finite oscillation period.
    [[nodiscard]] std::optional<Real> oscillation_period_s(Real mass_kg, Real stiffness_n_m, Real damping_n_s_m = 0.0);
    // Small-angle point-mass pendulum, with a massless rigid support and uniform gravity.
    [[nodiscard]] std::optional<Real> pendulum_period_s(Real length_m, Real gravity_m_s2);
    // Isotropic Coulomb contact on one straight ramp, with no other applied force: theta=atan(mu_s).
    // Supply the mixed coefficient of the pair, not just one surface's material coefficient.
    [[nodiscard]] std::optional<Real> critical_ramp_angle_rad(Real static_friction);

    struct CollisionComparisonSettings
    {
        Real first_mass_kg { 1.0 }, second_mass_kg { 2.0 };
        Real first_velocity_m_s { 2.0 }, second_velocity_m_s { -1.0 };
        Real radius_m { 0.12 }, initial_gap_m { 1.0 };
        Real time_step_s { 1.0 / 240.0 }, maximum_duration_s { 5.0 };
        std::size_t maximum_steps { 120000 };
    };

    struct CollisionComparisonReport
    {
        Real restitution {};
        std::string label;
        Real predicted_first_velocity_m_s {}, predicted_second_velocity_m_s {};
        Real measured_first_velocity_m_s {}, measured_second_velocity_m_s {};
        Real incoming_momentum_kg_m_s {}, outgoing_momentum_kg_m_s {};
        Real incoming_kinetic_energy_j {}, outgoing_kinetic_energy_j {};
        Real predicted_energy_loss_j {};
        Real impact_time_s {};
        std::size_t step_count {};
    };

    // Two isolated frictionless discs approach along their line of centres. Each run uses the
    // ordinary collision pipeline with no gravity, damping, or sleep. The report captures their
    // first actual contact response, after any speculative approach cap. e=0 means equal outgoing
    // velocity, not a permanent weld.
    // Missing impact, exhausted work limits, or numerical clamps throw instead of fabricating a
    // measurement. The vector order is elastic (1), inelastic (0), partially elastic (0.5).
    [[nodiscard]] CollisionComparisonReport measure_head_on_collision(Real restitution, const CollisionComparisonSettings& settings = {});
    [[nodiscard]] std::vector<CollisionComparisonReport> compare_head_on_collisions(const CollisionComparisonSettings& settings = {});

    struct ProjectileComparisonSettings
    {
        math::Vec2 initial_position_m {}, initial_velocity_m_s { 2.0, 3.0 };
        math::Vec2 gravity_m_s2 { 0.0, -standard_gravity_m_s2 };
        Real duration_s { 0.5 }, time_step_s { 1.0 / 60.0 };
    };

    struct ProjectileComparisonReport
    {
        std::string integrator_name;
        math::Vec2 predicted_position_m {}, measured_position_m {};
        math::Vec2 predicted_velocity_m_s {}, measured_velocity_m_s {};
        Real position_error_m {}, velocity_error_m_s {}, elapsed_time_s {};
        std::size_t step_count {};
    };

    // Isolated ballistic trajectory under constant gravity: x=x0+v0*t+g*t^2/2, v=v0+g*t.
    // This is a fresh reference experiment, independent of edits to the interactive scene.
    [[nodiscard]] ProjectileComparisonReport measure_projectile_trajectory(const Integrator& integrator, const ProjectileComparisonSettings& settings = {});
    [[nodiscard]] std::vector<ProjectileComparisonReport> compare_projectile_trajectories(const ProjectileComparisonSettings& settings = {});

} // namespace rigidbodies::physics
