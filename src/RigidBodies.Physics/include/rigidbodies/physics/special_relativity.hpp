#pragma once

#include <rigidbodies/physics/units.hpp>

#include <array>
#include <cstdint>
#include <optional>

namespace rigidbodies::physics
{
    // The fastest speed a relativity experiment accepts, as a fraction of c. Seven nines keep 1 − β
    // exact in double, keep the rapidity (8.41) far below where tanh rounds to 1 (about 19.06) and
    // leave c − v at 29.98 m/s, so a speed shown to the metre per second never reads as c.
    inline constexpr Real maximum_speed_fraction = 0.9999999;
    // The slowest speed above rest a learner or a document may set, about 0.3 mm/s. Its v/c text
    // stays short, (βγ)² stays far above underflow, and the field's digit steps stay finite.
    inline constexpr Real minimum_moving_speed_fraction = 1.0e-12;
    // The speeds Shift+Up and Shift+Down step between. Ascending; the last is the maximum.
    inline constexpr std::array<Real, 9> speed_fraction_ladder { 0.0, 0.5, 0.9, 0.99, 0.999, 0.9999, 0.99999, 0.999999, 0.9999999 };
    // One rapidity step of the speed control, ln(10)/20. Near c a nine is ln(10)/2, so ten steps add one nine.
    inline constexpr Real speed_rapidity_step = 0.11512925464970229;
    // The distance light travels in one nanosecond, about 30 cm.
    inline constexpr Real light_nanosecond_m = speed_of_light_m_s * 1.0e-9;
    // The repeating race track: ten light-nanoseconds, about 3 m.
    inline constexpr Real relativity_track_length_m = 10.0 * light_nanosecond_m;
    // Slow motion: one second of the world clock, one second on screen at 1×, is one nanosecond in the lab.
    inline constexpr Real relativity_lab_seconds_per_world_second = 1.0e-9;
    inline constexpr Real minimum_relativity_mass_kg = 1.0e-6;
    inline constexpr Real maximum_relativity_mass_kg = 1.0e6;

    // Every factor of one speed, computed without cancellation at either end of [0, 1).
    struct LorentzFactors
    {
        Real speed_fraction {};                      // β
        Real one_minus_speed_fraction { 1.0 };       // 1 − β, exact for β ≥ ½ (Sterbenz)
        Real lorentz_factor { 1.0 };                 // γ = 1/√((1 − β)(1 + β))
        Real lorentz_factor_minus_one {};            // γ − 1 = (βγ)²/(γ + 1)
        Real speed_fraction_times_lorentz_factor {}; // βγ = p/(mc)
        Real inverse_lorentz_factor { 1.0 };         // 1/γ = √((1 − β)(1 + β)) = dτ/dt
        Real clock_lag_rate {};                      // 1 − 1/γ = (γ − 1)/γ = d(t − τ)/dt
        Real rapidity {};                            // φ = ½·log1p(2β/(1 − β))
    };
    // std::nullopt unless β is finite and 0 ≤ β < 1.
    [[nodiscard]] std::optional<LorentzFactors> lorentz_factors(Real speed_fraction);
    // From the gap 1 − β in (0, 1], for curve samples near c. std::nullopt as well for a gap so small
    // (2⁻⁵⁴ or less) that 1 − gap rounds to 1, so β is always below c.
    [[nodiscard]] std::optional<LorentzFactors> lorentz_factors_from_gap(Real one_minus_speed_fraction);
    // From the rapidity in [0, 18]: γ = cosh φ, βγ = sinh φ, γ − 1 = 2 sinh²(φ/2), 1 − β = 2/(1 + e^{2φ}).
    [[nodiscard]] std::optional<LorentzFactors> lorentz_factors_from_rapidity(Real rapidity);
    // artanh β with β clamped to [0, maximum_speed_fraction]; 0 for a non-finite input.
    [[nodiscard]] Real rapidity_from_speed_fraction(Real speed_fraction);
    // tanh of the rapidity clamped to [0, maximum_rapidity()], computed as 1 − 2/(1 + e^{2φ}) above
    // φ = ½ so the gap keeps its digits; never above maximum_speed_fraction.
    [[nodiscard]] Real speed_fraction_from_rapidity(Real rapidity);
    [[nodiscard]] Real maximum_rapidity(); // 8.405621391022310
    // The next ladder speed strictly faster (direction > 0) or slower (direction < 0) than β; β itself at
    // either end. A moving rung within a billionth of β's distance from the nearer end of [0, 1], or a
    // few units in the last place, is β's own rung, so 0.99999 typed and 0.99999 from the ladder are
    // the same rung while 1e-9 c steps down to rest.
    [[nodiscard]] Real adjacent_speed_preset(Real speed_fraction, int direction);
    // The ladder speed that is β's own rung, by the rule above, or β itself: a typed 99.999 %, which
    // parses a rounding error below 0.99999, is that rung.
    [[nodiscard]] Real speed_fraction_on_ladder(Real speed_fraction);
    // Rest, or a finite speed from minimum_moving_speed_fraction to maximum_speed_fraction.
    [[nodiscard]] bool settable_speed_fraction(Real speed_fraction);

    struct RelativisticQuantities
    {
        Real speed_m_s {};                  // βc
        Real below_light_m_s {};            // (1 − β)c, from the exact gap; never 0
        Real rest_energy_j {};              // mc²
        Real kinetic_energy_j {};           // (γ − 1)mc²
        Real total_energy_j {};             // γmc²
        Real momentum_kg_m_s {};            // βγ·mc
        Real newtonian_kinetic_energy_j {}; // ½m(βc)²
        Real newtonian_momentum_kg_m_s {};  // mβc
    };
    // Throws std::invalid_argument for a mass outside [minimum, maximum]_relativity_mass_kg.
    [[nodiscard]] RelativisticQuantities relativistic_quantities(Real rest_mass_kg, const LorentzFactors& factors);

    // The authored and saved starting conditions. Clocks are never part of it.
    struct RelativitySetup
    {
        Real rest_mass_kg { 1.0 };
        Real speed_fraction {};
    };
    [[nodiscard]] bool operator==(const RelativitySetup& a, const RelativitySetup& b);
    [[nodiscard]] bool operator!=(const RelativitySetup& a, const RelativitySetup& b);
    // Finite, mass within the relativity bounds, 0 ≤ β ≤ maximum_speed_fraction.
    [[nodiscard]] bool valid_relativity_setup(const RelativitySetup& setup);

    // The race of one finished lap.
    struct RelativityLapResult
    {
        Real margin_s {};      // how long before the probe the light finished; > 0 for every β < 1
        bool speed_changed {}; // the speed changed while the lap was run
    };

    // The probe, its clock and the light pulse of the current lap, at one instant.
    struct RelativityRace
    {
        Real lab_time_s {}, proper_time_s {}, clock_lag_s {};
        Real lap_distance_m {}; // the probe's distance from the start line, [0, L)
        Real lap_lead_s {};     // Σ(1 − β)Δt since the lap began: the light's lead in lab seconds
        Real lap_elapsed_s {};  // lab time since the lap began
        bool lap_speed_changed {};
        std::int64_t completed_laps {};
        std::optional<RelativityLapResult> last_lap;
    };

    // One frame's view, between the last two fixed steps.
    struct RelativityRaceSample
    {
        Real lab_time_s {}, proper_time_s {}, clock_lag_s {};
        Real probe_position_m {}; // [0, L)
        Real light_position_m {}; // [probe_position_m, L]
        Real light_lead_m {};     // c × lap_lead_s while the pulse is on the track, else 0
        bool light_finished {};
        std::int64_t completed_laps {};
        std::optional<RelativityLapResult> last_lap;
    };

    // A point mass at a constant, learner-set speed on a repeating straight track, the clock it
    // carries and the light pulse that leaves the start line with it on every lap. Exact for each
    // segment of constant speed: neither step size nor step count changes a result beyond rounding.
    // Copyable, so undo snapshots and Back to start are plain copies.
    class RelativisticProbe
    {
    public:
        // Throws std::invalid_argument unless valid_relativity_setup(setup) and track_length_m > 0. A
        // negative zero speed is stored as +0, here and in set_speed_fraction.
        explicit RelativisticProbe(const RelativitySetup& setup, Real track_length_m = relativity_track_length_m);
        [[nodiscard]] const RelativitySetup& setup() const;
        [[nodiscard]] const LorentzFactors& factors() const;
        [[nodiscard]] Real track_length_m() const;
        // Throws std::invalid_argument unless finite and 0 ≤ β ≤ maximum_speed_fraction. An equal
        // value changes nothing. A new value marks a lap already under way as changed and resets the
        // interpolation history, so a sample never mixes two speeds.
        void set_speed_fraction(Real speed_fraction);
        // Advances by Δt ≥ 0 of lab time; throws std::invalid_argument for a negative or non-finite Δt.
        // Constant cost for any Δt. A step whose result would not be representable throws
        // std::overflow_error and leaves the probe unchanged.
        void advance(Real lab_time_step_s);
        // Interpolation history: the state at the start of the current fixed step.
        void capture_previous();
        // Clocks, lap, pulse and history back to zero; speed and mass kept.
        void restart();
        [[nodiscard]] const RelativityRace& race() const;
        // α clamped to [0, 1]; α = 1 is the current state.
        [[nodiscard]] RelativityRaceSample sample(Real interpolation_alpha) const;

    private:
        RelativitySetup setup_;
        LorentzFactors factors_;
        Real track_length_m_ {};
        RelativityRace current_, previous_;
    };
}
