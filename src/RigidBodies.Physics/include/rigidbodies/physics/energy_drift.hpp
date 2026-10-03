#pragma once

#include <rigidbodies/physics/integrator.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rigidbodies::physics
{
    struct EnergyDriftSettings
    {
        Real duration_s { 60.0 };
        Real time_step_s { 1.0 / 120.0 };
        Real mass_kg { 1.0 };
        Real stiffness_n_m { 4.0 };
        Real initial_displacement_m { 1.0 };
        Real initial_velocity_m_s { 0.0 };
    };

    struct EnergyDriftReport
    {
        std::string integrator_name;
        std::size_t step_count {};
        Real elapsed_time_s {};
        Real initial_energy_j {};
        Real final_energy_j {};
        Real final_drift_j {};
        Real maximum_absolute_drift_j {};
        // Relative drift is zero for the exactly stationary, zero-energy initial condition.
        Real maximum_relative_drift {};
    };

    // Run one horizontal harmonic oscillator with no gravity, damping, contacts, or sleeping.
    // Every run starts from identical state. The final step is shortened when needed to reach
    // the requested duration. Energy includes the PointAttractor spring potential explicitly.
    // The supplied integrator is cloned so mutable implementation state cannot affect another run.
    // Runs that reach motion limits fail explicitly rather than reporting clamped trajectories as
    // integration error. Each run is limited to ten million steps.
    [[nodiscard]] EnergyDriftReport measure_harmonic_energy_drift(const Integrator& integrator, const EnergyDriftSettings& settings = {});
    [[nodiscard]] std::vector<EnergyDriftReport> compare_harmonic_energy_drift(const EnergyDriftSettings& settings = {});
} // namespace rigidbodies::physics
