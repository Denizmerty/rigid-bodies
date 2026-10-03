#pragma once

#include <rigidbodies/physics/material.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace rigidbodies::physics
{

    struct RestitutionComparisonSettings
    {
        // Height of the ball's bottom above the flat floor at release, with zero initial speed.
        Real drop_height_m { 1.0 };
        Real ball_radius_m { 0.05 };
        Real mass_kg { 0.5 };
        Real ball_restitution { 0.8 };
        Real floor_restitution { 0.2 };
        Real time_step_s { 1.0 / 240.0 };
        Real maximum_duration_s { 5.0 };
        std::size_t maximum_steps { 120000 };
        bool continuous { true };
    };

    struct RestitutionComparisonReport
    {
        MaterialMixing mixing { MaterialMixing::geometric_mean };
        std::string mixing_name;
        Real mixed_restitution {};
        Real drop_height_m {};
        Real theoretical_rebound_height_m {};
        Real measured_rebound_height_m {};
        Real measured_restitution {};
        Real height_error_m {};
        Real impact_speed_m_s {};
        Real rebound_speed_m_s {};
        Real impact_time_s {};
        Real apex_time_s {};
        std::size_t step_count {};
    };

    // Drop an isolated ball onto a fixed floor with zero friction, damping, and sleeping. The
    // experiment uses the normal simulation pipeline and semi-implicit Euler, with continuous
    // collision enabled by default. All policies start from the same dimensions and release pose.
    //
    // The analytical first rebound height is e^2 * drop_height. The measured height is the largest
    // sampled height of the ball's bottom between its first upward response and first apex; the
    // measured coefficient is sqrt(measured_height / drop_height). Finite-step integration and
    // contact tolerances account for their difference. A non-bouncing response has its apex at
    // impact. Motion caps, sweep iteration exhaustion, or failure to reach an impact/apex within
    // the supplied duration/step budget fail explicitly rather than returning a misleading result.
    [[nodiscard]] RestitutionComparisonReport measure_restitution_drop(MaterialMixing mixing, const RestitutionComparisonSettings& settings = {});
    [[nodiscard]] std::vector<RestitutionComparisonReport> compare_restitution_drops(const RestitutionComparisonSettings& settings = {});

} // namespace rigidbodies::physics
