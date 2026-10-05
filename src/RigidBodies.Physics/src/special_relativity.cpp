#include <rigidbodies/physics/special_relativity.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        // Above this rapidity the speed is formed from its gap, so 1 − β keeps its digits.
        constexpr Real gap_form_rapidity = 0.5;
        // Far above the maximum speed's 8.41 and below where tanh rounds to 1 (about 19.06).
        constexpr Real maximum_factor_rapidity = 18.0;
        // A rung this close to β, relative to β's distance from the nearer end of [0, 1], is β's own rung.
        constexpr Real ladder_rung_tolerance = 1.0e-9;
        // Well inside the range of std::int64_t, so a lap count never overflows.
        constexpr Real maximum_lap_count = 4.0e18;

        // Every factor from β and its gap, given (1 − β)(1 + β) = 1 − β² formed without cancellation.
        LorentzFactors factors_from(Real speed_fraction, Real gap, Real one_minus_square)
        {
            LorentzFactors result;
            result.speed_fraction = speed_fraction;
            result.one_minus_speed_fraction = gap;
            result.inverse_lorentz_factor = std::sqrt(one_minus_square);
            result.lorentz_factor = 1.0 / result.inverse_lorentz_factor;
            result.speed_fraction_times_lorentz_factor = speed_fraction * result.lorentz_factor;
            // (βγ)²/(γ + 1) rather than γ − 1, which would round to 0 near rest.
            result.lorentz_factor_minus_one = result.speed_fraction_times_lorentz_factor * result.speed_fraction_times_lorentz_factor / (result.lorentz_factor + 1.0);
            result.clock_lag_rate = result.lorentz_factor_minus_one / result.lorentz_factor;
            result.rapidity = 0.5 * std::log1p(2.0 * speed_fraction / gap);
            return result;
        }

        bool valid_speed_fraction(Real speed_fraction)
        {
            return std::isfinite(speed_fraction) && speed_fraction >= 0.0 && speed_fraction <= maximum_speed_fraction;
        }

        // A negative zero is rest, so a setup never writes or fingerprints "-0".
        Real without_negative_zero(Real speed_fraction)
        {
            return speed_fraction == 0.0 ? 0.0 : speed_fraction;
        }

        // Near c the tolerance scales with the gap, so 0.99999 typed and 0.99999 from the ladder are one
        // rung; near rest it scales with β, so even 1e-9 c is a rung above rest. A few units in the
        // last place are added for the top rung, where a billionth of the gap is less than one: 99.99999 %
        // parses one below 0.9999999. Rest gets none, so it is never a moving speed's rung. Near c both
        // values are within a factor of two of each other, so their difference is exact (Sterbenz).
        bool same_rung(Real preset, Real speed_fraction)
        {
            const auto rounding = 4.0 * std::numeric_limits<Real>::epsilon() * preset;
            return std::abs(preset - speed_fraction) <= ladder_rung_tolerance * std::min(speed_fraction, 1.0 - speed_fraction) + rounding;
        }

        // Advances one race by Δt > 0 at the constant speed of `factors`. Exact for the segment: the
        // probe's distance and the light's lead are linear in lab time, and whole laps at constant
        // speed are counted in one division, so the cost never depends on Δt.
        void advance_race(RelativityRace& race, const LorentzFactors& factors, Real track_length_m, Real step_s)
        {
            race.lab_time_s += step_s;
            race.proper_time_s += step_s * factors.inverse_lorentz_factor;
            // Accumulated from its own rate, never formed as t − τ, so a tiny lag keeps its digits.
            race.clock_lag_s += step_s * factors.clock_lag_rate;
            const auto speed_m_s = factors.speed_fraction * speed_of_light_m_s;
            const auto gap = factors.one_minus_speed_fraction;
            if (speed_m_s == 0.0)
            {
                race.lap_lead_s += step_s;
                race.lap_elapsed_s += step_s;
                return;
            }
            const auto to_finish_s = (track_length_m - race.lap_distance_m) / speed_m_s;
            if (step_s < to_finish_s)
            {
                race.lap_distance_m += speed_m_s * step_s;
                race.lap_lead_s += step_s * gap;
                race.lap_elapsed_s += step_s;
                return;
            }
            // The probe crosses the finish inside this step. The light's lead at that moment is the
            // lap's margin, accumulated rather than formed as a difference of two lap times.
            race.lap_lead_s += to_finish_s * gap;
            race.last_lap = RelativityLapResult { race.lap_lead_s, race.lap_speed_changed };
            ++race.completed_laps;
            auto remaining_s = step_s - to_finish_s;
            const auto lap_time_s = track_length_m / speed_m_s;
            const auto whole_laps = std::floor(remaining_s / lap_time_s);
            if (whole_laps >= 1.0)
            {
                if (static_cast<Real>(race.completed_laps) + whole_laps > maximum_lap_count)
                    throw std::overflow_error("A relativistic probe cannot count that many laps");
                race.completed_laps += static_cast<std::int64_t>(whole_laps);
                remaining_s = std::max(0.0, remaining_s - whole_laps * lap_time_s);
                race.last_lap = RelativityLapResult { lap_time_s * gap, false };
            }
            // A new pulse leaves the start line with the probe.
            race.lap_distance_m = std::min(speed_m_s * remaining_s, std::nextafter(track_length_m, 0.0));
            race.lap_lead_s = remaining_s * gap;
            race.lap_elapsed_s = remaining_s;
            race.lap_speed_changed = false;
        }

        bool finite_race(const RelativityRace& race)
        {
            return std::isfinite(race.lab_time_s) && std::isfinite(race.proper_time_s) && std::isfinite(race.clock_lag_s) &&
                std::isfinite(race.lap_distance_m) && std::isfinite(race.lap_lead_s) && std::isfinite(race.lap_elapsed_s);
        }
    }

    std::optional<LorentzFactors> lorentz_factors(Real speed_fraction)
    {
        if (!std::isfinite(speed_fraction) || speed_fraction < 0.0 || speed_fraction >= 1.0)
            return std::nullopt;
        const auto beta = without_negative_zero(speed_fraction);
        const auto gap = 1.0 - beta;
        return factors_from(beta, gap, gap * (1.0 + beta));
    }

    std::optional<LorentzFactors> lorentz_factors_from_gap(Real one_minus_speed_fraction)
    {
        const auto gap = one_minus_speed_fraction;
        // A gap of 2⁻⁵⁴ or less leaves 1 − gap rounded to 1, a β that would read as c.
        if (!std::isfinite(gap) || gap <= 0.0 || gap > 1.0 || 1.0 - gap == 1.0)
            return std::nullopt;
        return factors_from(1.0 - gap, gap, gap * (2.0 - gap));
    }

    std::optional<LorentzFactors> lorentz_factors_from_rapidity(Real rapidity)
    {
        if (!std::isfinite(rapidity) || rapidity < 0.0 || rapidity > maximum_factor_rapidity)
            return std::nullopt;
        const auto phi = rapidity == 0.0 ? 0.0 : rapidity;
        LorentzFactors result;
        result.rapidity = phi;
        result.lorentz_factor = std::cosh(phi);
        result.speed_fraction_times_lorentz_factor = std::sinh(phi);
        const auto half = std::sinh(0.5 * phi);
        result.lorentz_factor_minus_one = 2.0 * half * half;
        result.inverse_lorentz_factor = 1.0 / result.lorentz_factor;
        result.one_minus_speed_fraction = 2.0 / (1.0 + std::exp(2.0 * phi));
        result.speed_fraction = phi > gap_form_rapidity ? 1.0 - result.one_minus_speed_fraction : std::tanh(phi);
        result.clock_lag_rate = result.lorentz_factor_minus_one / result.lorentz_factor;
        return result;
    }

    Real rapidity_from_speed_fraction(Real speed_fraction)
    {
        if (!std::isfinite(speed_fraction))
            return 0.0;
        const auto beta = std::clamp(speed_fraction, 0.0, maximum_speed_fraction);
        return 0.5 * std::log1p(2.0 * beta / (1.0 - beta));
    }

    Real speed_fraction_from_rapidity(Real rapidity)
    {
        if (std::isnan(rapidity) || rapidity <= 0.0)
            return 0.0;
        if (rapidity >= maximum_rapidity())
            return maximum_speed_fraction;
        if (rapidity <= gap_form_rapidity)
            return std::tanh(rapidity);
        return std::min(1.0 - 2.0 / (1.0 + std::exp(2.0 * rapidity)), maximum_speed_fraction);
    }

    Real maximum_rapidity()
    {
        return rapidity_from_speed_fraction(maximum_speed_fraction);
    }

    Real adjacent_speed_preset(Real speed_fraction, int direction)
    {
        if (direction > 0)
        {
            for (const auto preset : speed_fraction_ladder)
                if (preset > speed_fraction && !same_rung(preset, speed_fraction))
                    return preset;
        }
        else if (direction < 0)
        {
            for (auto preset = speed_fraction_ladder.rbegin(); preset != speed_fraction_ladder.rend(); ++preset)
                if (*preset < speed_fraction && !same_rung(*preset, speed_fraction))
                    return *preset;
        }
        return speed_fraction;
    }

    Real speed_fraction_on_ladder(Real speed_fraction)
    {
        for (const auto preset : speed_fraction_ladder)
            if (same_rung(preset, speed_fraction))
                return preset;
        return speed_fraction;
    }

    bool settable_speed_fraction(Real speed_fraction)
    {
        return speed_fraction == 0.0 || (valid_speed_fraction(speed_fraction) && speed_fraction >= minimum_moving_speed_fraction);
    }

    RelativisticQuantities relativistic_quantities(Real rest_mass_kg, const LorentzFactors& factors)
    {
        if (!std::isfinite(rest_mass_kg) || rest_mass_kg < minimum_relativity_mass_kg || rest_mass_kg > maximum_relativity_mass_kg)
            throw std::invalid_argument("Relativistic rest mass must be finite and within the relativity mass bounds");
        const auto rest_momentum_kg_m_s = rest_mass_kg * speed_of_light_m_s;
        RelativisticQuantities result;
        result.speed_m_s = factors.speed_fraction * speed_of_light_m_s;
        result.below_light_m_s = factors.one_minus_speed_fraction * speed_of_light_m_s;
        result.rest_energy_j = rest_momentum_kg_m_s * speed_of_light_m_s;
        result.kinetic_energy_j = factors.lorentz_factor_minus_one * result.rest_energy_j;
        result.total_energy_j = factors.lorentz_factor * result.rest_energy_j;
        result.momentum_kg_m_s = factors.speed_fraction_times_lorentz_factor * rest_momentum_kg_m_s;
        result.newtonian_kinetic_energy_j = 0.5 * rest_mass_kg * result.speed_m_s * result.speed_m_s;
        result.newtonian_momentum_kg_m_s = rest_mass_kg * result.speed_m_s;
        return result;
    }

    bool operator==(const RelativitySetup& a, const RelativitySetup& b)
    {
        return a.rest_mass_kg == b.rest_mass_kg && a.speed_fraction == b.speed_fraction;
    }

    bool operator!=(const RelativitySetup& a, const RelativitySetup& b)
    {
        return !(a == b);
    }

    bool valid_relativity_setup(const RelativitySetup& setup)
    {
        return std::isfinite(setup.rest_mass_kg) && setup.rest_mass_kg >= minimum_relativity_mass_kg && setup.rest_mass_kg <= maximum_relativity_mass_kg &&
            valid_speed_fraction(setup.speed_fraction);
    }

    RelativisticProbe::RelativisticProbe(const RelativitySetup& setup, Real track_length_m) : setup_(setup), track_length_m_(track_length_m)
    {
        if (!valid_relativity_setup(setup) || !std::isfinite(track_length_m) || track_length_m <= 0.0)
            throw std::invalid_argument("A relativistic probe needs a valid setup and a positive track length");
        setup_.speed_fraction = without_negative_zero(setup_.speed_fraction);
        factors_ = *lorentz_factors(setup_.speed_fraction);
    }

    const RelativitySetup& RelativisticProbe::setup() const
    {
        return setup_;
    }

    const LorentzFactors& RelativisticProbe::factors() const
    {
        return factors_;
    }

    Real RelativisticProbe::track_length_m() const
    {
        return track_length_m_;
    }

    void RelativisticProbe::set_speed_fraction(Real speed_fraction)
    {
        if (!valid_speed_fraction(speed_fraction))
            throw std::invalid_argument("Probe speed must be finite and from 0 c to the maximum speed fraction");
        if (speed_fraction == setup_.speed_fraction)
            return;
        setup_.speed_fraction = without_negative_zero(speed_fraction);
        factors_ = *lorentz_factors(setup_.speed_fraction);
        if (current_.lap_elapsed_s > 0.0)
            current_.lap_speed_changed = true;
        previous_ = current_;
    }

    void RelativisticProbe::advance(Real lab_time_step_s)
    {
        if (!std::isfinite(lab_time_step_s) || lab_time_step_s < 0.0)
            throw std::invalid_argument("A relativistic probe advances by a finite, nonnegative lab time");
        if (lab_time_step_s == 0.0)
            return;
        auto next = current_;
        advance_race(next, factors_, track_length_m_, lab_time_step_s);
        if (!finite_race(next))
            throw std::overflow_error("A relativistic probe step would leave the representable range");
        current_ = next;
    }

    void RelativisticProbe::capture_previous()
    {
        previous_ = current_;
    }

    void RelativisticProbe::restart()
    {
        current_ = {};
        previous_ = {};
    }

    const RelativityRace& RelativisticProbe::race() const
    {
        return current_;
    }

    RelativityRaceSample RelativisticProbe::sample(Real interpolation_alpha) const
    {
        const auto alpha = std::isnan(interpolation_alpha) ? 1.0 : std::clamp(interpolation_alpha, 0.0, 1.0);
        auto race = current_;
        // The speed is constant between the two states, because a speed change resets the history,
        // so advancing the earlier state with the same algorithm reproduces the step at any fraction.
        if (alpha < 1.0 && previous_.lab_time_s != current_.lab_time_s)
        {
            race = previous_;
            const auto step_s = alpha * (current_.lab_time_s - previous_.lab_time_s);
            if (step_s > 0.0)
                advance_race(race, factors_, track_length_m_, step_s);
        }
        RelativityRaceSample result;
        result.lab_time_s = race.lab_time_s;
        result.proper_time_s = race.proper_time_s;
        result.clock_lag_s = race.clock_lag_s;
        result.probe_position_m = race.lap_distance_m;
        const auto light_m = race.lap_distance_m + speed_of_light_m_s * race.lap_lead_s;
        result.light_finished = light_m >= track_length_m_;
        result.light_position_m = std::max(std::min(light_m, track_length_m_), result.probe_position_m);
        result.light_lead_m = result.light_finished ? 0.0 : speed_of_light_m_s * race.lap_lead_s;
        result.completed_laps = race.completed_laps;
        result.last_lap = race.last_lap;
        return result;
    }
}
