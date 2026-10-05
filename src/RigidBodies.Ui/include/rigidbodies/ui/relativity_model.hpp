#pragma once

#include <rigidbodies/physics/special_relativity.hpp>

#include <cstdint>
#include <optional>

namespace rigidbodies::ui
{
    // The relativity experiment as the panels see it, filled each build by the session from the
    // physics helpers so every formula exists once. Clock and race values are the interpolated sample
    // the stage draws this frame.
    struct RelativityModel
    {
        double rest_mass_kg { 1.0 };
        double speed_fraction {}, one_minus_speed_fraction { 1.0 }, rapidity {};
        double original_speed_fraction {}, maximum_speed_fraction { physics::maximum_speed_fraction };
        double speed_m_s {}, below_light_m_s { physics::speed_of_light_m_s };
        double lorentz_factor { 1.0 }, lorentz_factor_minus_one {}, inverse_lorentz_factor { 1.0 }, clock_lag_rate {};
        double rest_energy_j {}, kinetic_energy_j {}, newtonian_kinetic_energy_j {};
        double momentum_kg_m_s {}, newtonian_momentum_kg_m_s {};
        double lab_time_s {}, proper_time_s {}, clock_lag_s {};
        double light_lead_m {};
        bool light_finished {};
        std::int64_t completed_laps {};
        std::optional<double> last_lap_margin_s;
        bool last_lap_speed_changed {};
        double lab_seconds_per_world_second { physics::relativity_lab_seconds_per_world_second };
    };
}
