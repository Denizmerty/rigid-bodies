#include <rigidbodies/physics/special_relativity.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using namespace rigidbodies::physics;

    constexpr Real nanosecond_s = 1.0e-9;
    constexpr Real infinity = std::numeric_limits<Real>::infinity();
    constexpr Real not_a_number = std::numeric_limits<Real>::quiet_NaN();

    bool speed_refused(RelativisticProbe& probe, Real speed_fraction)
    {
        try
        {
            probe.set_speed_fraction(speed_fraction);
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }

    template <typename Error>
    bool step_refused(RelativisticProbe& probe, Real lab_time_step_s)
    {
        try
        {
            probe.advance(lab_time_step_s);
        }
        catch (const Error&)
        {
            return true;
        }
        return false;
    }

    bool probe_refused(const RelativitySetup& setup, Real track_length_m = relativity_track_length_m)
    {
        try
        {
            const RelativisticProbe probe { setup, track_length_m };
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }

    bool mass_refused(Real rest_mass_kg)
    {
        try
        {
            (void)relativistic_quantities(rest_mass_kg, *lorentz_factors(0.5));
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }

    void expect_relative(Real actual, Real expected, Real relative, std::string_view message)
    {
        RIGIDBODIES_EXPECT_NEAR(actual, expected, std::abs(expected) * relative, message);
    }

    LorentzFactors factors_at(Real speed_fraction)
    {
        const auto factors = lorentz_factors(speed_fraction);
        RIGIDBODIES_EXPECT(factors.has_value(), "a speed below c has Lorentz factors");
        return *factors;
    }

    RelativisticProbe probe_at(Real speed_fraction, Real rest_mass_kg = 1.0)
    {
        return RelativisticProbe { RelativitySetup { rest_mass_kg, speed_fraction } };
    }

    // The constant-speed lap margin in closed form: lap time L/v less the light's time L/c.
    Real closed_form_margin_s(Real speed_fraction)
    {
        return relativity_track_length_m / speed_of_light_m_s * (1.0 - speed_fraction) / speed_fraction;
    }

    // Speeds evenly spaced in rapidity from rest to the maximum, so both ends are sampled densely.
    std::vector<Real> rapidity_spaced_speeds(int count)
    {
        std::vector<Real> result;
        for (int index = 0; index < count; ++index)
            result.push_back(speed_fraction_from_rapidity(index + 1 == count ? maximum_rapidity() : maximum_rapidity() * index / (count - 1)));
        return result;
    }

    // Uneven but deterministic step sizes that add up to `total_s`: the last step takes the rest.
    std::vector<Real> uneven_steps(Real total_s, int count)
    {
        std::vector<Real> weights;
        Real weight_sum = 0.0;
        std::uint32_t state = 2463534242U;
        for (int index = 0; index < count; ++index)
        {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            weights.push_back(1.0 + static_cast<Real>(state % 1000U));
            weight_sum += weights.back();
        }
        std::vector<Real> steps;
        Real elapsed_s = 0.0;
        for (int index = 0; index + 1 < count; ++index)
        {
            steps.push_back(total_s * weights[static_cast<std::size_t>(index)] / weight_sum);
            elapsed_s += steps.back();
        }
        steps.push_back(total_s - elapsed_s);
        return steps;
    }

    // Advances in small fixed steps, as the session does, until the probe has finished `laps` laps.
    void run_until_laps(RelativisticProbe& probe, std::int64_t laps, Real step_s)
    {
        for (int guard = 0; probe.race().completed_laps < laps && guard < 1000000; ++guard)
            probe.advance(step_s);
        RIGIDBODIES_EXPECT(probe.race().completed_laps >= laps, "the probe finishes the laps it was run for");
    }

    bool same_race(const RelativityRace& a, const RelativityRace& b)
    {
        const auto same_lap = a.last_lap.has_value() == b.last_lap.has_value() &&
            (!a.last_lap || (a.last_lap->margin_s == b.last_lap->margin_s && a.last_lap->speed_changed == b.last_lap->speed_changed));
        return a.lab_time_s == b.lab_time_s && a.proper_time_s == b.proper_time_s && a.clock_lag_s == b.clock_lag_s &&
            a.lap_distance_m == b.lap_distance_m && a.lap_lead_s == b.lap_lead_s && a.lap_elapsed_s == b.lap_elapsed_s &&
            a.lap_speed_changed == b.lap_speed_changed && a.completed_laps == b.completed_laps && same_lap;
    }

    // Fixed algorithm rather than standard-library distributions, so the sequence is the same everywhere.
    class SequenceRandom
    {
    public:
        Real unit()
        {
            state_ ^= state_ << 13;
            state_ ^= state_ >> 7;
            state_ ^= state_ << 17;
            return static_cast<Real>(state_ >> 11) / static_cast<Real>(1ULL << 53);
        }

    private:
        std::uint64_t state_ { 0x2545f4914f6cdd1dULL };
    };

    RIGIDBODIES_TEST("the track, the ladder and the rapidity step have their stated values")
    {
        RIGIDBODIES_EXPECT(speed_of_light_m_s == 299792458.0, "c is exact");
        RIGIDBODIES_EXPECT_NEAR(relativity_track_length_m, 2.99792458, 1.0e-15, "the track is ten light-nanoseconds");
        RIGIDBODIES_EXPECT_NEAR(light_nanosecond_m, 0.299792458, 1.0e-16, "a mark is one light-nanosecond apart");
        RIGIDBODIES_EXPECT_NEAR(speed_rapidity_step, std::log(10.0) / 20.0, 1.0e-17, "ten rapidity steps add one nine near c");
        RIGIDBODIES_EXPECT(speed_fraction_ladder.front() == 0.0 && speed_fraction_ladder.back() == maximum_speed_fraction, "the ladder runs from rest to the maximum");
        for (std::size_t index = 1; index < speed_fraction_ladder.size(); ++index)
            RIGIDBODIES_EXPECT(speed_fraction_ladder[index] > speed_fraction_ladder[index - 1], "the ladder ascends");
        RIGIDBODIES_EXPECT(maximum_speed_fraction < 1.0 && 1.0 - maximum_speed_fraction > 0.0, "the maximum is below c in double precision");
    }

    RIGIDBODIES_TEST("Lorentz factors match exact values at 0.6 c and 0.8 c")
    {
        const auto slow = factors_at(0.6);
        RIGIDBODIES_EXPECT_NEAR(slow.lorentz_factor, 1.25, 1.0e-15, "γ at 0.6 c is 5/4");
        RIGIDBODIES_EXPECT_NEAR(slow.speed_fraction_times_lorentz_factor, 0.75, 1.0e-15, "βγ at 0.6 c is 3/4");
        RIGIDBODIES_EXPECT_NEAR(slow.lorentz_factor_minus_one, 0.25, 1.0e-15, "γ − 1 at 0.6 c is 1/4");
        RIGIDBODIES_EXPECT_NEAR(slow.inverse_lorentz_factor, 0.8, 1.0e-15, "the moving clock runs at 4/5 of the lab rate");
        RIGIDBODIES_EXPECT_NEAR(slow.clock_lag_rate, 0.2, 1.0e-15, "the moving clock loses 1/5 of each lab second");
        RIGIDBODIES_EXPECT_NEAR(slow.rapidity, std::log(2.0), 1.0e-15, "the rapidity of 0.6 c is ln 2");
        RIGIDBODIES_EXPECT_NEAR(slow.one_minus_speed_fraction, 0.4, 1.0e-16, "the gap is kept");
        const auto fast = factors_at(0.8);
        RIGIDBODIES_EXPECT_NEAR(fast.lorentz_factor, 5.0 / 3.0, 1.0e-15, "γ at 0.8 c is 5/3");
        RIGIDBODIES_EXPECT_NEAR(fast.speed_fraction_times_lorentz_factor, 4.0 / 3.0, 1.0e-15, "βγ at 0.8 c is 4/3");
        RIGIDBODIES_EXPECT_NEAR(fast.lorentz_factor_minus_one, 2.0 / 3.0, 1.0e-15, "γ − 1 at 0.8 c is 2/3");
        RIGIDBODIES_EXPECT_NEAR(fast.inverse_lorentz_factor, 0.6, 1.0e-15, "the moving clock runs at 3/5 of the lab rate");
        RIGIDBODIES_EXPECT_NEAR(fast.clock_lag_rate, 0.4, 1.0e-15, "the moving clock loses 2/5 of each lab second");
        RIGIDBODIES_EXPECT_NEAR(fast.rapidity, std::log(3.0), 1.0e-15, "the rapidity of 0.8 c is ln 3");
        const auto rest = factors_at(0.0);
        RIGIDBODIES_EXPECT(rest.lorentz_factor == 1.0 && rest.inverse_lorentz_factor == 1.0 && rest.one_minus_speed_fraction == 1.0, "rest has unit factors exactly");
        RIGIDBODIES_EXPECT(rest.lorentz_factor_minus_one == 0.0 && rest.speed_fraction_times_lorentz_factor == 0.0 && rest.clock_lag_rate == 0.0 && rest.rapidity == 0.0, "rest has zero excess, momentum, lag and rapidity exactly");
        RIGIDBODIES_EXPECT(!std::signbit(factors_at(-0.0).speed_fraction), "a negative zero reads as rest");
        const auto resting = probe_at(-0.0);
        RIGIDBODIES_EXPECT(!std::signbit(resting.setup().speed_fraction) && resting.setup() == RelativitySetup { 1.0, 0.0 }, "a probe built at negative zero is at rest");
        auto stopped = probe_at(0.5);
        stopped.set_speed_fraction(-0.0);
        RIGIDBODIES_EXPECT(!std::signbit(stopped.setup().speed_fraction) && stopped.factors().lorentz_factor == 1.0, "a negative zero speed is stored as rest");
    }

    RIGIDBODIES_TEST("Lorentz factors near c match high-precision references")
    {
        const auto near = factors_at(0.99999);
        expect_relative(near.lorentz_factor, 223.6073567695785, 1.0e-12, "γ at 0.99999 c");
        expect_relative(near.speed_fraction_times_lorentz_factor, 223.6051206960108, 1.0e-12, "βγ at 0.99999 c");
        expect_relative(near.lorentz_factor_minus_one, 222.6073567695785, 1.0e-12, "γ − 1 at 0.99999 c");
        expect_relative(near.rapidity, 6.1030338227611, 1.0e-12, "rapidity at 0.99999 c");
        const auto quantities = relativistic_quantities(1.0, near);
        expect_relative(quantities.kinetic_energy_j, 2.000695e19, 1.0e-6, "kinetic energy of 1 kg at 0.99999 c");
        expect_relative(quantities.momentum_kg_m_s, 6.703513e10, 1.0e-6, "momentum of 1 kg at 0.99999 c");
        expect_relative(quantities.below_light_m_s, 2997.9246, 1.0e-7, "0.99999 c is 2 998 m/s below c");
        expect_relative(quantities.kinetic_energy_j / quantities.newtonian_kinetic_energy_j, 445.22, 1.0e-4, "Newton's ½mv² is 445 times too small");
        expect_relative(quantities.rest_energy_j, 8.987551787368176e16, 1.0e-15, "rest energy is mc²");
        expect_relative(quantities.total_energy_j, quantities.rest_energy_j + quantities.kinetic_energy_j, 1.0e-15, "total energy is rest plus kinetic");
        const auto maximum = factors_at(maximum_speed_fraction);
        expect_relative(maximum.lorentz_factor, 2236.068033989975, 1.0e-12, "γ at the maximum speed");
        expect_relative(maximum.rapidity, 8.40562139102231, 1.0e-12, "rapidity at the maximum speed");
        const auto fastest = relativistic_quantities(1.0, maximum);
        expect_relative(fastest.kinetic_energy_j, 2.008779e20, 1.0e-6, "kinetic energy of 1 kg at the maximum speed");
        expect_relative(fastest.momentum_kg_m_s, 6.703563e11, 1.0e-6, "momentum of 1 kg at the maximum speed");
        expect_relative(fastest.below_light_m_s, 29.9792, 1.0e-5, "the maximum speed stays 30 m/s below c");
        RIGIDBODIES_EXPECT(fastest.speed_m_s < speed_of_light_m_s && fastest.below_light_m_s > 0.0, "the fastest probe is still slower than light");
        const auto half = relativistic_quantities(2.0, factors_at(0.5));
        expect_relative(half.kinetic_energy_j, 2.0 * 1.390379e16, 1.0e-6, "kinetic energy scales with rest mass");
        expect_relative(half.newtonian_momentum_kg_m_s, 2.0 * 0.5 * speed_of_light_m_s, 1.0e-15, "Newton's momentum is mv");
        expect_relative(half.momentum_kg_m_s / half.newtonian_momentum_kg_m_s, 1.154700538379251, 1.0e-12, "momentum exceeds mv by γ");
    }

    RIGIDBODIES_TEST("kinetic energy keeps full precision near rest")
    {
        const auto crawl = factors_at(1.0e-9);
        expect_relative(crawl.lorentz_factor_minus_one, 5.0e-19, 1.0e-12, "γ − 1 is ½β² rather than zero");
        RIGIDBODIES_EXPECT(crawl.clock_lag_rate > 0.0, "the moving clock still loses time");
        const auto quantities = relativistic_quantities(1.0, crawl);
        expect_relative(quantities.kinetic_energy_j, quantities.newtonian_kinetic_energy_j, 1.0e-12, "kinetic energy is ½mv² near rest");
        expect_relative(quantities.kinetic_energy_j, 4.4937758936840882e-2, 1.0e-12, "1 kg at 0.3 m/s has 0.0449 J");
        const auto slow = relativistic_quantities(3.0, factors_at(1.0e-4));
        expect_relative(slow.kinetic_energy_j, slow.newtonian_kinetic_energy_j * (1.0 + 0.75e-8), 1.0e-12, "the first correction is ¾β²");
    }

    RIGIDBODIES_TEST("speeds at or above light are rejected")
    {
        for (const auto speed : { 1.0, 1.0 + 1.0e-12, infinity, -infinity, not_a_number, -1.0e-12 })
            RIGIDBODIES_EXPECT(!lorentz_factors(speed).has_value(), "no factors at, above or below the physical range");
        RIGIDBODIES_EXPECT(lorentz_factors(std::nextafter(1.0, 0.0)).has_value(), "the last double below c still has finite factors");
        for (const auto gap : { 0.0, -1.0e-12, 1.0 + 1.0e-12, not_a_number, infinity })
            RIGIDBODIES_EXPECT(!lorentz_factors_from_gap(gap).has_value(), "a gap must be in (0, 1]");
        for (const auto gap : { 1.0e-17, std::ldexp(1.0, -54), 1.0e-300, std::numeric_limits<Real>::denorm_min() })
            RIGIDBODIES_EXPECT(!lorentz_factors_from_gap(gap).has_value(), "a gap too small to leave β below 1 is refused");
        for (const auto gap : { std::ldexp(1.0, -53), 6.0e-17 })
        {
            const auto edge = lorentz_factors_from_gap(gap);
            RIGIDBODIES_EXPECT(edge && edge->speed_fraction < 1.0 && edge->one_minus_speed_fraction == gap && relativistic_quantities(1.0, *edge).speed_m_s < speed_of_light_m_s, "the smallest accepted gaps still leave β below 1");
        }
        for (const auto rapidity : { -1.0e-12, 18.0 + 1.0e-9, not_a_number, infinity })
            RIGIDBODIES_EXPECT(!lorentz_factors_from_rapidity(rapidity).has_value(), "a rapidity must be in [0, 18]");
        auto probe = probe_at(0.5);
        probe.advance(3.0 * nanosecond_s);
        const auto before = probe.race();
        for (const auto speed : { 1.0, not_a_number, -0.1, maximum_speed_fraction + 1.0e-9, infinity })
            RIGIDBODIES_EXPECT(speed_refused(probe, speed), "the probe refuses speeds outside zero to the maximum");
        RIGIDBODIES_EXPECT(probe.setup().speed_fraction == 0.5 && same_race(probe.race(), before) && !probe.race().lap_speed_changed, "a refused speed changes nothing");
        probe.set_speed_fraction(maximum_speed_fraction);
        RIGIDBODIES_EXPECT(probe.setup().speed_fraction == maximum_speed_fraction && probe.factors().speed_fraction == maximum_speed_fraction, "the maximum is accepted");
        for (const auto setup : { RelativitySetup { 1.0, 1.0 }, RelativitySetup { 0.0, 0.5 }, RelativitySetup { 1.0e9, 0.5 }, RelativitySetup { not_a_number, 0.5 }, RelativitySetup { 1.0, -0.5 } })
        {
            RIGIDBODIES_EXPECT(!valid_relativity_setup(setup), "invalid setups are recognised");
            RIGIDBODIES_EXPECT(probe_refused(setup), "a probe cannot be built from an invalid setup");
        }
        RIGIDBODIES_EXPECT(probe_refused({}, 0.0) && probe_refused({}, -1.0) && probe_refused({}, infinity), "a probe needs a track");
        RIGIDBODIES_EXPECT(mass_refused(0.0) && mass_refused(not_a_number), "a massless probe is not modelled");
        RIGIDBODIES_EXPECT(mass_refused(2.0e6) && !mass_refused(maximum_relativity_mass_kg) && !mass_refused(minimum_relativity_mass_kg), "the rest mass has bounds, and they are valid");
        RIGIDBODIES_EXPECT(valid_relativity_setup({ minimum_relativity_mass_kg, 0.0 }) && valid_relativity_setup({ maximum_relativity_mass_kg, maximum_speed_fraction }), "the bounds themselves are valid");
        for (const auto step_s : { -1.0e-12, not_a_number, infinity })
            RIGIDBODIES_EXPECT(step_refused<std::invalid_argument>(probe, step_s), "time only runs forward by finite steps");
    }

    RIGIDBODIES_TEST("gap and rapidity forms agree with the speed form")
    {
        const auto check = [](const LorentzFactors& expected, const LorentzFactors& actual, Real relative)
        {
            expect_relative(actual.lorentz_factor, expected.lorentz_factor, relative, "γ agrees");
            expect_relative(actual.lorentz_factor_minus_one, expected.lorentz_factor_minus_one, relative, "γ − 1 agrees");
            expect_relative(actual.speed_fraction_times_lorentz_factor, expected.speed_fraction_times_lorentz_factor, relative, "βγ agrees");
            expect_relative(actual.inverse_lorentz_factor, expected.inverse_lorentz_factor, relative, "1/γ agrees");
            expect_relative(actual.clock_lag_rate, expected.clock_lag_rate, relative, "the lag rate agrees");
            expect_relative(actual.rapidity, expected.rapidity, relative, "the rapidity agrees");
            expect_relative(actual.one_minus_speed_fraction, expected.one_minus_speed_fraction, relative, "the gap agrees");
            expect_relative(actual.speed_fraction, expected.speed_fraction, relative, "β agrees");
        };
        const auto lowest = rapidity_from_speed_fraction(0.5);
        for (int index = 0; index <= 2000; ++index)
        {
            const auto speed = speed_fraction_from_rapidity(lowest + (maximum_rapidity() - lowest) * index / 2000.0);
            RIGIDBODIES_EXPECT(speed > 0.4999999 && speed <= maximum_speed_fraction, "the samples cover half c to the maximum");
            const auto expected = factors_at(speed);
            const auto from_gap = lorentz_factors_from_gap(1.0 - speed);
            const auto from_rapidity = lorentz_factors_from_rapidity(rapidity_from_speed_fraction(speed));
            RIGIDBODIES_EXPECT(from_gap && from_rapidity, "both other forms accept the sample");
            check(expected, *from_gap, 1.0e-12);
            check(expected, *from_rapidity, 1.0e-12);
        }
        for (const auto speed : { 1.0e-6, 1.0e-3, 0.1, 0.3, 0.49 })
        {
            const auto from_rapidity = lorentz_factors_from_rapidity(rapidity_from_speed_fraction(speed));
            RIGIDBODIES_EXPECT(from_rapidity.has_value(), "slow rapidities are accepted");
            check(factors_at(speed), *from_rapidity, 1.0e-12);
        }
        const auto deep = lorentz_factors_from_gap(1.0e-12);
        RIGIDBODIES_EXPECT(deep && std::isfinite(deep->lorentz_factor) && deep->one_minus_speed_fraction == 1.0e-12, "a curve sample beyond the maximum keeps its gap");
        const auto edge = lorentz_factors_from_rapidity(18.0);
        RIGIDBODIES_EXPECT(edge && edge->speed_fraction < 1.0 && edge->one_minus_speed_fraction > 0.0, "the largest rapidity is still below c");
    }

    RIGIDBODIES_TEST("factors rise strictly and stay finite up to the maximum")
    {
        const auto speeds = rapidity_spaced_speeds(10000);
        RIGIDBODIES_EXPECT(speeds.front() == 0.0 && speeds.back() == maximum_speed_fraction, "the samples run from rest to the maximum");
        auto previous = factors_at(speeds.front());
        for (std::size_t index = 1; index < speeds.size(); ++index)
        {
            const auto current = factors_at(speeds[index]);
            RIGIDBODIES_EXPECT(std::isfinite(current.lorentz_factor) && std::isfinite(current.lorentz_factor_minus_one) && std::isfinite(current.speed_fraction_times_lorentz_factor) &&
                    std::isfinite(current.inverse_lorentz_factor) && std::isfinite(current.clock_lag_rate) && std::isfinite(current.rapidity),
                "every factor is finite");
            RIGIDBODIES_EXPECT(current.speed_fraction > previous.speed_fraction, "β rises");
            RIGIDBODIES_EXPECT(current.lorentz_factor > previous.lorentz_factor, "γ rises");
            RIGIDBODIES_EXPECT(current.lorentz_factor_minus_one > previous.lorentz_factor_minus_one, "γ − 1 rises");
            RIGIDBODIES_EXPECT(current.speed_fraction_times_lorentz_factor > previous.speed_fraction_times_lorentz_factor, "βγ rises");
            RIGIDBODIES_EXPECT(current.rapidity > previous.rapidity, "the rapidity rises");
            RIGIDBODIES_EXPECT(current.clock_lag_rate > previous.clock_lag_rate, "the lag rate rises");
            RIGIDBODIES_EXPECT(current.inverse_lorentz_factor < previous.inverse_lorentz_factor, "the clock rate falls");
            RIGIDBODIES_EXPECT(current.one_minus_speed_fraction < previous.one_minus_speed_fraction && current.one_minus_speed_fraction > 0.0, "the gap shrinks and never closes");
            previous = current;
        }
    }

    RIGIDBODIES_TEST("clock rate and lag rate sum to one")
    {
        auto speeds = rapidity_spaced_speeds(2000);
        for (const auto speed : { 1.0e-12, 1.0e-9, 1.0e-6, 0.6, 0.8, 0.99999 })
            speeds.push_back(speed);
        for (const auto speed : speeds)
        {
            const auto factors = factors_at(speed);
            RIGIDBODIES_EXPECT_NEAR(factors.inverse_lorentz_factor + factors.clock_lag_rate, 1.0, 1.0e-15, "τ and t − τ share every lab second");
        }
    }

    RIGIDBODIES_TEST("rapidity round-trips and is capped below c")
    {
        auto speeds = rapidity_spaced_speeds(1000);
        for (const auto speed : speed_fraction_ladder)
            speeds.push_back(speed);
        for (const auto speed : { 1.0e-9, 0.25, 0.75 })
            speeds.push_back(speed);
        for (const auto speed : speeds)
        {
            const auto round_trip = speed_fraction_from_rapidity(rapidity_from_speed_fraction(speed));
            expect_relative(1.0 - round_trip, 1.0 - speed, 1.0e-9, "the gap survives a round trip through rapidity");
            expect_relative(round_trip, speed, 1.0e-9, "β survives a round trip through rapidity");
            RIGIDBODIES_EXPECT(round_trip <= maximum_speed_fraction, "a round trip never passes the maximum");
        }
        RIGIDBODIES_EXPECT(speed_fraction_from_rapidity(30.0) == maximum_speed_fraction, "a rapidity past the cap gives the maximum exactly");
        RIGIDBODIES_EXPECT(speed_fraction_from_rapidity(maximum_rapidity()) == maximum_speed_fraction, "the cap itself gives the maximum exactly");
        RIGIDBODIES_EXPECT(speed_fraction_from_rapidity(infinity) == maximum_speed_fraction, "an infinite rapidity is capped");
        RIGIDBODIES_EXPECT(speed_fraction_from_rapidity(-1.0) == 0.0 && speed_fraction_from_rapidity(not_a_number) == 0.0, "negative and missing rapidities are rest");
        RIGIDBODIES_EXPECT_NEAR(maximum_rapidity(), 8.405621391022310, 1.0e-12, "the rapidity cap is artanh of the maximum speed");
        RIGIDBODIES_EXPECT(rapidity_from_speed_fraction(1.0) == maximum_rapidity() && rapidity_from_speed_fraction(2.0) == maximum_rapidity(), "speeds at or past c are clamped to the maximum");
        RIGIDBODIES_EXPECT(rapidity_from_speed_fraction(-0.5) == 0.0 && rapidity_from_speed_fraction(not_a_number) == 0.0 && rapidity_from_speed_fraction(infinity) == 0.0, "negative and non-finite speeds have no rapidity");
        for (int step = 0; step <= 200; ++step)
        {
            const auto speed = speed_fraction_from_rapidity(step * 0.1);
            RIGIDBODIES_EXPECT(speed >= 0.0 && speed <= maximum_speed_fraction && speed < 1.0, "every rapidity maps below c");
        }
    }

    RIGIDBODIES_TEST("the speed ladder compares gaps")
    {
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.0, +1) == 0.5, "rest climbs to half c");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.95, +1) == 0.99, "between rungs climbs to the next rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.95, -1) == 0.9, "between rungs descends to the previous rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.99999, +1) == 0.999999, "a typed rung is the same rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(maximum_speed_fraction, +1) == maximum_speed_fraction, "the top stays at the top");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.0, -1) == 0.0, "rest stays at rest");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.99999, -1) == 0.9999, "a typed rung descends one rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.7, 0) == 0.7, "no direction keeps the speed");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(std::nextafter(0.99999, 0.0), +1) == 0.999999 && adjacent_speed_preset(0.99999 - 1.0e-15, +1) == 0.999999, "a rung reached through rounding from below is still that rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(std::nextafter(0.99999, 1.0), -1) == 0.9999 && adjacent_speed_preset(0.99999 + 1.0e-15, -1) == 0.9999, "a rung reached through rounding from above is still that rung");
        RIGIDBODIES_EXPECT(adjacent_speed_preset(std::nextafter(0.5, 1.0), -1) == 0.0 && adjacent_speed_preset(std::nextafter(0.5, 0.0), +1) == 0.9, "half c reached through rounding is still half c");
        for (const auto crawl : { 1.0e-9, 1.0e-12, 5.0e-10, 1.0e-300, std::numeric_limits<Real>::denorm_min() })
        {
            RIGIDBODIES_EXPECT(adjacent_speed_preset(crawl, -1) == 0.0, "any speed above rest steps down to rest");
            RIGIDBODIES_EXPECT(adjacent_speed_preset(crawl, +1) == 0.5, "any speed below half c steps up to half c");
        }
        RIGIDBODIES_EXPECT(adjacent_speed_preset(0.5 - 1.0e-6, +1) == 0.5 && adjacent_speed_preset(0.99999 - 1.0e-12, +1) == 0.99999, "a speed just short of a rung steps up to it");
        // At the top a billionth of the gap is less than one unit in the last place; 99.99999 %
        // parses one unit below the maximum and is still the top rung.
        const auto typed_top = std::nextafter(maximum_speed_fraction, 0.0);
        RIGIDBODIES_EXPECT(adjacent_speed_preset(typed_top, +1) == typed_top && adjacent_speed_preset(typed_top, -1) == 0.999999, "a top reached through rounding is still the top");
        Real speed = 0.0;
        for (std::size_t rung = 1; rung < speed_fraction_ladder.size(); ++rung)
        {
            speed = adjacent_speed_preset(speed, +1);
            RIGIDBODIES_EXPECT(speed == speed_fraction_ladder[rung], "climbing visits every rung in order");
        }
        for (std::size_t rung = speed_fraction_ladder.size() - 1; rung-- > 0;)
        {
            speed = adjacent_speed_preset(speed, -1);
            RIGIDBODIES_EXPECT(speed == speed_fraction_ladder[rung], "descending visits every rung in order");
        }
    }

    RIGIDBODIES_TEST("a speed a rounding error from a rung is that rung, and a moving speed has a floor")
    {
        for (const auto rung : speed_fraction_ladder)
        {
            RIGIDBODIES_EXPECT(speed_fraction_on_ladder(rung) == rung, "a rung is its own rung");
            if (rung > 0.0)
                for (const auto near : { std::nextafter(rung, 0.0), std::nextafter(rung, 1.0) })
                    if (near <= maximum_speed_fraction)
                        RIGIDBODIES_EXPECT(speed_fraction_on_ladder(near) == rung, "a neighbouring double is the rung");
        }
        RIGIDBODIES_EXPECT(speed_fraction_on_ladder(99.999 / 100.0) == 0.99999 && speed_fraction_on_ladder(99.99999 / 100.0) == maximum_speed_fraction && speed_fraction_on_ladder(99.9 / 100.0) == 0.999, "typed percentages land on their rungs");
        for (const auto between : { 0.95, 0.50034614, 0.99998, 0.9999901, 1.0e-9, 1.0e-12, 0.5 - 1.0e-6 })
            RIGIDBODIES_EXPECT(speed_fraction_on_ladder(between) == between, "a speed merely close to a rung keeps its own value");
        RIGIDBODIES_EXPECT(settable_speed_fraction(0.0) && settable_speed_fraction(-0.0) && settable_speed_fraction(minimum_moving_speed_fraction) && settable_speed_fraction(maximum_speed_fraction), "rest, the slowest moving speed and the maximum can be set");
        for (const auto refused : { 1.0e-13, 9.99e-13, 1.0e-300, std::numeric_limits<Real>::denorm_min(), -1.0e-12, std::nextafter(maximum_speed_fraction, 1.0), 1.0, std::numeric_limits<Real>::quiet_NaN(), std::numeric_limits<Real>::infinity() })
            RIGIDBODIES_EXPECT(!settable_speed_fraction(refused), "a speed slower than the floor, faster than the maximum or not finite is refused");
        // At the floor every reading keeps its digits: K is far from underflow and agrees with
        // Newton's ½mv², from which it differs only by about β², here far below rounding.
        const auto slowest = lorentz_factors(minimum_moving_speed_fraction);
        const auto quantities = relativistic_quantities(minimum_relativity_mass_kg, *slowest);
        RIGIDBODIES_EXPECT(slowest && quantities.kinetic_energy_j > 0.0 && std::abs(quantities.kinetic_energy_j - quantities.newtonian_kinetic_energy_j) <= 1.0e-12 * quantities.newtonian_kinetic_energy_j, "the slowest moving probe has Newton's kinetic energy, not zero");
    }

    RIGIDBODIES_TEST("proper time is exact for each constant-speed segment")
    {
        for (const auto count : { 1, 997, 10000 })
        {
            auto probe = probe_at(0.6);
            for (const auto step_s : uneven_steps(10.0 * nanosecond_s, count))
                probe.advance(step_s);
            RIGIDBODIES_EXPECT_NEAR(probe.race().lab_time_s, 10.0 * nanosecond_s, 1.0e-21, "the lab clock reads 10 ns");
            RIGIDBODIES_EXPECT_NEAR(probe.race().proper_time_s, 8.0 * nanosecond_s, 1.0e-21, "at 0.6 c the probe clock reads 8 ns");
            RIGIDBODIES_EXPECT_NEAR(probe.race().clock_lag_s, 2.0 * nanosecond_s, 1.0e-21, "and is 2 ns behind");
            probe.set_speed_fraction(0.8);
            for (const auto step_s : uneven_steps(5.0 * nanosecond_s, count))
                probe.advance(step_s);
            RIGIDBODIES_EXPECT_NEAR(probe.race().proper_time_s, 11.0 * nanosecond_s, 2.0e-21, "5 ns at 0.8 c adds 3 ns to the probe clock");
            RIGIDBODIES_EXPECT_NEAR(probe.race().clock_lag_s, 4.0 * nanosecond_s, 2.0e-21, "and 2 ns to its lag");
        }
    }

    RIGIDBODIES_TEST("clock lag is accumulated, never subtracted")
    {
        auto probe = probe_at(1.0e-6);
        for (int step = 0; step < 1000; ++step)
            probe.advance(1.0e-3);
        const auto& race = probe.race();
        expect_relative(race.clock_lag_s, 5.0e-13, 1.0e-10, "a probe at 300 m/s loses half a picosecond each second");
        expect_relative(race.proper_time_s + race.clock_lag_s, race.lab_time_s, 1.0e-12, "probe time and lag add up to lab time");
    }

    RIGIDBODIES_TEST("light wins every lap by a margin that shrinks with each nine")
    {
        Real previous_margin_s = 0.0;
        for (const auto speed : speed_fraction_ladder)
        {
            if (speed == 0.0)
                continue;
            auto probe = probe_at(speed);
            const auto lap_time_s = relativity_track_length_m / (speed * speed_of_light_m_s);
            run_until_laps(probe, 2, lap_time_s / 7.3);
            const auto& lap = probe.race().last_lap;
            RIGIDBODIES_EXPECT(lap.has_value(), "a finished lap reports its race");
            expect_relative(lap->margin_s, closed_form_margin_s(speed), 1.0e-6, "the margin is the lap time less the light's time");
            RIGIDBODIES_EXPECT(lap->margin_s > 0.0 && !lap->speed_changed, "light wins a steady lap");
            // 0.5 c to 0.9 c divides the margin by exactly 9; every later rung divides it by about 10.
            if (previous_margin_s > 0.0)
                RIGIDBODIES_EXPECT(lap->margin_s <= previous_margin_s / 9.0 * (1.0 + 1.0e-9), "each nine shrinks the margin ninefold or more");
            previous_margin_s = lap->margin_s;
        }
        expect_relative(previous_margin_s, 1.0e-15, 1.0e-3, "at the maximum speed light still wins by a femtosecond");
    }

    RIGIDBODIES_TEST("light leads by c(1 − β)t within a lap")
    {
        for (const auto speed : { 0.9, 0.99999 })
        {
            auto probe = probe_at(speed);
            for (int step = 0; step < 50; ++step)
                probe.advance(0.1 * nanosecond_s);
            const auto sample = probe.sample(1.0);
            const auto expected_m = speed_of_light_m_s * (1.0 - speed) * probe.race().lab_time_s;
            expect_relative(sample.light_lead_m, expected_m, 1.0e-9, "the light's lead grows at c − v");
            RIGIDBODIES_EXPECT(!sample.light_finished && sample.completed_laps == 0, "both are still on their first lap");
            expect_relative(sample.light_position_m, sample.probe_position_m + sample.light_lead_m, 1.0e-15, "the pulse is ahead of the probe by the lead");
        }
        auto steady = probe_at(0.9);
        steady.advance(5.0 * nanosecond_s);
        expect_relative(steady.sample(1.0).light_lead_m, 0.149896229, 1.0e-9, "at 0.9 c light leads by 15 cm after 5 ns");
        auto close = probe_at(0.99999);
        close.advance(5.0 * nanosecond_s);
        expect_relative(close.sample(1.0).light_lead_m, 1.4989623e-5, 1.0e-7, "at 0.99999 c light leads by 15 µm after 5 ns");
    }

    RIGIDBODIES_TEST("a probe at rest never finishes a lap")
    {
        auto probe = probe_at(0.0);
        probe.advance(9.99 * nanosecond_s);
        auto sample = probe.sample(1.0);
        RIGIDBODIES_EXPECT(!sample.light_finished && sample.probe_position_m == 0.0 && sample.completed_laps == 0, "light is still racing a probe that stays put");
        expect_relative(sample.light_position_m, speed_of_light_m_s * 9.99 * nanosecond_s, 1.0e-12, "the pulse has run 9.99 light-nanoseconds");
        probe.advance(0.02 * nanosecond_s);
        sample = probe.sample(1.0);
        RIGIDBODIES_EXPECT(sample.light_finished && sample.light_position_m == relativity_track_length_m && sample.light_lead_m == 0.0, "the pulse finishes and waits at the line");
        probe.advance(1.0);
        RIGIDBODIES_EXPECT(probe.race().completed_laps == 0 && probe.race().lap_distance_m == 0.0 && !probe.race().last_lap, "a probe at rest never finishes a lap");
        RIGIDBODIES_EXPECT(probe.race().proper_time_s == probe.race().lab_time_s && probe.race().clock_lag_s == 0.0, "its clock keeps lab time");
    }

    RIGIDBODIES_TEST("a speed change mid-lap is reported with that lap")
    {
        auto probe = probe_at(0.0);
        for (int step = 0; step < 30; ++step)
            probe.advance(nanosecond_s);
        probe.set_speed_fraction(0.5);
        RIGIDBODIES_EXPECT(probe.race().lap_speed_changed, "the lap under way is flagged");
        run_until_laps(probe, 1, 0.5 * nanosecond_s);
        auto lap = probe.race().last_lap;
        RIGIDBODIES_EXPECT(lap.has_value() && lap->speed_changed, "the mixed lap says the speed changed during it");
        expect_relative(lap->margin_s, 40.0 * nanosecond_s, 1.0e-9, "light finished 40 ns before a probe that waited 30 ns");
        RIGIDBODIES_EXPECT(!probe.race().lap_speed_changed, "the next lap starts clean");
        run_until_laps(probe, 2, 0.5 * nanosecond_s);
        lap = probe.race().last_lap;
        RIGIDBODIES_EXPECT(lap.has_value() && !lap->speed_changed, "the next lap is steady");
        expect_relative(lap->margin_s, 10.0 * nanosecond_s, 1.0e-9, "and reports the steady margin");
        auto fresh = probe_at(0.0);
        fresh.set_speed_fraction(0.9);
        RIGIDBODIES_EXPECT(!fresh.race().lap_speed_changed, "a speed chosen before the lap begins does not flag it");
        auto same = probe_at(0.5);
        same.advance(nanosecond_s);
        same.set_speed_fraction(0.5);
        RIGIDBODIES_EXPECT(!same.race().lap_speed_changed, "setting the same speed changes nothing");
    }

    RIGIDBODIES_TEST("large steps run whole laps in constant time")
    {
        auto probe = probe_at(0.5);
        probe.advance(1.0);
        const auto& race = probe.race();
        RIGIDBODIES_EXPECT(race.completed_laps >= 49999999 && race.completed_laps <= 50000001, "one second at 0.5 c is fifty million laps");
        RIGIDBODIES_EXPECT(race.last_lap && !race.last_lap->speed_changed, "the last of them was steady");
        expect_relative(race.last_lap->margin_s, 10.0 * nanosecond_s, 1.0e-6, "each lap is won by 10 ns");
        RIGIDBODIES_EXPECT(race.lap_distance_m >= 0.0 && race.lap_distance_m < relativity_track_length_m, "the probe is on the track");
        expect_relative(race.proper_time_s, 1.0 / 1.154700538379251, 1.0e-12, "the probe clock reads t/γ");
        auto far = probe_at(0.99999);
        far.advance(1.0e6);
        expect_relative(static_cast<Real>(far.race().completed_laps), 1.0e14 * 0.99999, 1.0e-9, "a million seconds near c is a hundred million million laps");
        RIGIDBODIES_EXPECT(far.race().lap_distance_m < relativity_track_length_m && far.sample(1.0).light_position_m >= far.sample(1.0).probe_position_m, "the race stays on the track");
        RIGIDBODIES_EXPECT(step_refused<std::overflow_error>(far, 1.0e300) && far.race().lab_time_s == 1.0e6, "an unrepresentable lap count is refused and changes nothing");
    }

    RIGIDBODIES_TEST("interpolated samples stay continuous across a lap boundary")
    {
        for (const int substeps : { 1, 4 })
        {
            auto probe = probe_at(0.9);
            probe.advance(10.5 * nanosecond_s);
            probe.capture_previous();
            for (int step = 0; step < substeps; ++step)
                probe.advance(nanosecond_s / substeps);
            RIGIDBODIES_EXPECT(probe.race().completed_laps == 1, "the step crosses the finish");
            const auto start = probe.sample(0.0), end = probe.sample(1.0);
            RIGIDBODIES_EXPECT_NEAR(start.lab_time_s, 10.5 * nanosecond_s, 1.0e-24, "α = 0 is the start of the step");
            RIGIDBODIES_EXPECT(end.lab_time_s == probe.race().lab_time_s && end.probe_position_m == probe.race().lap_distance_m, "α = 1 is the current state");
            Real previous_unwrapped_m = -1.0;
            for (const auto alpha : { 0.0, 0.25, 0.5, 0.75, 1.0 })
            {
                const auto sample = probe.sample(alpha);
                const auto unwrapped_m = static_cast<Real>(sample.completed_laps) * relativity_track_length_m + sample.probe_position_m;
                RIGIDBODIES_EXPECT(unwrapped_m > previous_unwrapped_m, "the probe only moves forward");
                previous_unwrapped_m = unwrapped_m;
                RIGIDBODIES_EXPECT(sample.light_position_m >= sample.probe_position_m, "light is never behind the probe");
                RIGIDBODIES_EXPECT_NEAR(sample.proper_time_s, start.proper_time_s + alpha * (end.proper_time_s - start.proper_time_s), 1.0e-23, "the probe clock is linear in α");
                RIGIDBODIES_EXPECT_NEAR(sample.lab_time_s, start.lab_time_s + alpha * (end.lab_time_s - start.lab_time_s), 1.0e-23, "the lab clock is linear in α");
            }
            RIGIDBODIES_EXPECT(probe.sample(-1.0).lab_time_s == start.lab_time_s && probe.sample(2.0).lab_time_s == end.lab_time_s && probe.sample(not_a_number).lab_time_s == end.lab_time_s, "α is clamped");
        }
        auto changed = probe_at(0.5);
        changed.advance(nanosecond_s);
        changed.capture_previous();
        changed.advance(nanosecond_s);
        changed.set_speed_fraction(0.9);
        RIGIDBODIES_EXPECT(changed.sample(0.0).lab_time_s == changed.race().lab_time_s, "a speed change resets the history so no sample mixes speeds");
    }

    RIGIDBODIES_TEST("the probe is never ahead of the light")
    {
        SequenceRandom random;
        auto probe = probe_at(0.0);
        std::int64_t laps = 0;
        for (int iteration = 0; iteration < 20000; ++iteration)
        {
            const auto choice = random.unit();
            if (choice < 0.05)
                probe.set_speed_fraction(speed_fraction_ladder[static_cast<std::size_t>(random.unit() * speed_fraction_ladder.size()) % speed_fraction_ladder.size()]);
            else if (choice < 0.1)
                probe.set_speed_fraction(random.unit() * maximum_speed_fraction);
            else if (choice < 0.12)
                probe.set_speed_fraction(speed_fraction_from_rapidity(random.unit() * maximum_rapidity()));
            else if (choice < 0.4)
                probe.capture_previous();
            else
                probe.advance(random.unit() * random.unit() * 30.0 * nanosecond_s);
            const auto sample = probe.sample(random.unit());
            RIGIDBODIES_EXPECT(sample.light_position_m >= sample.probe_position_m, "the probe is never ahead of the light");
            RIGIDBODIES_EXPECT(sample.probe_position_m >= 0.0 && sample.probe_position_m < relativity_track_length_m && sample.light_position_m <= relativity_track_length_m, "both stay on the track");
            RIGIDBODIES_EXPECT(sample.light_lead_m >= 0.0 && std::isfinite(sample.light_lead_m), "the lead is never negative");
            RIGIDBODIES_EXPECT(!probe.race().last_lap || probe.race().last_lap->margin_s > 0.0, "light wins every lap");
            RIGIDBODIES_EXPECT(probe.race().completed_laps >= laps, "laps only accumulate");
            laps = probe.race().completed_laps;
        }
        RIGIDBODIES_EXPECT(laps > 10, "the sequence runs many laps");
    }

    RIGIDBODIES_TEST("restart zeroes clocks and race but keeps speed and mass")
    {
        auto probe = probe_at(0.9, 2.5);
        for (int step = 0; step < 40; ++step)
            probe.advance(0.7 * nanosecond_s);
        probe.capture_previous();
        probe.advance(0.5 * nanosecond_s);
        RIGIDBODIES_EXPECT(probe.race().completed_laps > 0, "the probe has lapped before the restart");
        probe.restart();
        RIGIDBODIES_EXPECT(same_race(probe.race(), RelativityRace {}), "clocks, lap and pulse are back to zero");
        RIGIDBODIES_EXPECT(probe.setup() == RelativitySetup { 2.5, 0.9 } && probe.factors().speed_fraction == 0.9, "speed and mass are kept");
        const auto sample = probe.sample(0.5);
        RIGIDBODIES_EXPECT(sample.lab_time_s == 0.0 && sample.proper_time_s == 0.0 && sample.probe_position_m == 0.0 && !sample.last_lap, "the interpolation history is cleared");
        probe.advance(nanosecond_s);
        expect_relative(probe.race().proper_time_s, nanosecond_s / 2.294157338705618, 1.0e-12, "the restarted clocks run at the kept speed");
    }

    RIGIDBODIES_TEST("copies are independent snapshots")
    {
        auto probe = probe_at(0.99);
        probe.advance(3.0 * nanosecond_s);
        const auto snapshot = probe;
        probe.advance(20.0 * nanosecond_s);
        probe.set_speed_fraction(0.5);
        RIGIDBODIES_EXPECT(snapshot.race().lab_time_s == 3.0 * nanosecond_s && snapshot.setup().speed_fraction == 0.99, "a copy keeps the moment it was taken");
        auto restored = snapshot;
        RIGIDBODIES_EXPECT(same_race(restored.race(), snapshot.race()) && restored.setup() == snapshot.setup(), "restoring a copy restores the race");
        restored.restart();
        RIGIDBODIES_EXPECT(snapshot.race().lab_time_s == 3.0 * nanosecond_s && restored.race().lab_time_s == 0.0, "changing one copy leaves the others alone");
        RIGIDBODIES_EXPECT_NEAR(probe.race().lab_time_s, 23.0 * nanosecond_s, 1.0e-24, "the original keeps running");
        RIGIDBODIES_EXPECT(probe.setup() != snapshot.setup(), "setups compare by value");
    }

    RIGIDBODIES_TEST("identical command sequences give bit-identical probes")
    {
        const auto run = []
        {
            SequenceRandom random;
            auto probe = probe_at(0.0, 0.75);
            std::vector<RelativityRaceSample> samples;
            for (int iteration = 0; iteration < 5000; ++iteration)
            {
                const auto choice = random.unit();
                if (choice < 0.05)
                    probe.set_speed_fraction(speed_fraction_from_rapidity(random.unit() * maximum_rapidity()));
                else if (choice < 0.3)
                    probe.capture_previous();
                else if (choice < 0.31)
                    probe.restart();
                else
                    probe.advance(random.unit() * 4.0 * nanosecond_s);
                samples.push_back(probe.sample(random.unit()));
            }
            return std::make_pair(probe, samples);
        };
        const auto first = run(), second = run();
        RIGIDBODIES_EXPECT(same_race(first.first.race(), second.first.race()) && first.first.setup() == second.first.setup(), "the same commands give the same probe bit for bit");
        RIGIDBODIES_EXPECT(first.second.size() == second.second.size(), "the same number of samples");
        for (std::size_t index = 0; index < first.second.size(); ++index)
        {
            const auto& a = first.second[index];
            const auto& b = second.second[index];
            RIGIDBODIES_EXPECT(a.lab_time_s == b.lab_time_s && a.proper_time_s == b.proper_time_s && a.clock_lag_s == b.clock_lag_s && a.probe_position_m == b.probe_position_m &&
                    a.light_position_m == b.light_position_m && a.light_lead_m == b.light_lead_m && a.light_finished == b.light_finished && a.completed_laps == b.completed_laps,
                "every sample is identical");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
