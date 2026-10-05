#pragma once

#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/ui/relativity_model.hpp>

namespace rigidbodies::testing
{
    // The relativity model a session would publish for a 1 kg probe set to this speed at Ready and
    // run for this much lab time, built with the same physics helpers, for tests that build a
    // UiModel by hand.
    inline ui::RelativityModel relativity_model_at(double speed_fraction, double lab_time_s)
    {
        physics::RelativisticProbe probe({ 1.0, speed_fraction });
        probe.advance(lab_time_s);
        const auto& factors = probe.factors();
        const auto quantities = physics::relativistic_quantities(probe.setup().rest_mass_kg, factors);
        const auto sample = probe.sample(1.0);
        ui::RelativityModel model;
        model.rest_mass_kg = probe.setup().rest_mass_kg;
        model.speed_fraction = factors.speed_fraction;
        model.one_minus_speed_fraction = factors.one_minus_speed_fraction;
        model.rapidity = factors.rapidity;
        model.original_speed_fraction = 0.0;
        model.maximum_speed_fraction = physics::maximum_speed_fraction;
        model.speed_m_s = quantities.speed_m_s;
        model.below_light_m_s = quantities.below_light_m_s;
        model.lorentz_factor = factors.lorentz_factor;
        model.lorentz_factor_minus_one = factors.lorentz_factor_minus_one;
        model.inverse_lorentz_factor = factors.inverse_lorentz_factor;
        model.clock_lag_rate = factors.clock_lag_rate;
        model.rest_energy_j = quantities.rest_energy_j;
        model.kinetic_energy_j = quantities.kinetic_energy_j;
        model.newtonian_kinetic_energy_j = quantities.newtonian_kinetic_energy_j;
        model.momentum_kg_m_s = quantities.momentum_kg_m_s;
        model.newtonian_momentum_kg_m_s = quantities.newtonian_momentum_kg_m_s;
        model.lab_time_s = sample.lab_time_s;
        model.proper_time_s = sample.proper_time_s;
        model.clock_lag_s = sample.clock_lag_s;
        model.light_lead_m = sample.light_lead_m;
        model.light_finished = sample.light_finished;
        model.completed_laps = sample.completed_laps;
        if (sample.last_lap)
        {
            model.last_lap_margin_s = sample.last_lap->margin_s;
            model.last_lap_speed_changed = sample.last_lap->speed_changed;
        }
        model.lab_seconds_per_world_second = physics::relativity_lab_seconds_per_world_second;
        return model;
    }
}
