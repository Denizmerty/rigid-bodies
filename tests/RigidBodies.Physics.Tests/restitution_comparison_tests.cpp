#include <rigidbodies/physics/restitution_comparison.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace
{

    using namespace rigidbodies::physics;

    BodyId scenario_body(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
        {
            if (world.find_body(id)->name() == name)
                return id;
        }
        RIGIDBODIES_FAIL("expected named scenario body exists");
    }

    void step_without_limits(World& world, int steps, Real dt = 1.0 / 120.0)
    {
        for (int step = 0; step < steps; ++step)
        {
            world.step(dt);
            RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "scenario remains within numerical motion bounds");
            RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count == 0, "continuous collision converges within its iteration budget");
        }
    }

    RIGIDBODIES_TEST("drop experiment compares four mixing laws against actual first rebound heights")
    {
        const auto reports = compare_restitution_drops();
        const std::array<Real, 4> coefficients { 0.4, 0.5, 0.2, 0.8 };
        const std::array<MaterialMixing, 4> policies { MaterialMixing::geometric_mean, MaterialMixing::arithmetic_mean, MaterialMixing::minimum, MaterialMixing::maximum };
        RIGIDBODIES_EXPECT(reports.size() == policies.size(), "all four policies produce independent reports");
        for (std::size_t index = 0; index < reports.size(); ++index)
        {
            const auto& report = reports[index];
            const auto coefficient = coefficients[index];
            RIGIDBODIES_EXPECT(report.mixing == policies[index] && !report.mixing_name.empty(), "report identifies mixing policy");
            RIGIDBODIES_EXPECT_NEAR(report.mixed_restitution, coefficient, 1.0e-12, "mixed coefficient matches closed form");
            RIGIDBODIES_EXPECT_NEAR(report.drop_height_m, 1.0, 0.0, "every run starts at the same release height");
            RIGIDBODIES_EXPECT_NEAR(report.theoretical_rebound_height_m, coefficient * coefficient, 1.0e-12, "analytical rebound uses e squared times drop height");
            RIGIDBODIES_EXPECT_NEAR(report.measured_rebound_height_m, report.theoretical_rebound_height_m, 0.025, "measured first apex agrees within finite-step contact error");
            RIGIDBODIES_EXPECT_NEAR(report.measured_restitution, coefficient, 0.04, "height-derived coefficient measures the bounce");
            RIGIDBODIES_EXPECT_NEAR(report.measured_restitution, std::sqrt(report.measured_rebound_height_m / report.drop_height_m), 1.0e-14, "reported coefficient comes from measured height");
            RIGIDBODIES_EXPECT_NEAR(report.height_error_m, report.measured_rebound_height_m - report.theoretical_rebound_height_m, 0.0, "signed height error is reported");
            RIGIDBODIES_EXPECT(report.impact_speed_m_s > 4.0 && report.rebound_speed_m_s > 0.0, "impact and rebound are observed rather than fabricated from theory");
            RIGIDBODIES_EXPECT(report.impact_time_s > 0.4 && report.apex_time_s >= report.impact_time_s && report.step_count > 0, "report captures first impact and later apex");
        }
        RIGIDBODIES_EXPECT(reports[2].measured_rebound_height_m < reports[0].measured_rebound_height_m &&
                reports[0].measured_rebound_height_m < reports[1].measured_rebound_height_m &&
                reports[1].measured_rebound_height_m < reports[3].measured_rebound_height_m,
            "measured heights reveal the actual difference between the mixing policies");
    }

    RIGIDBODIES_TEST("swapping ball and floor coefficients preserves symmetric mixing results")
    {
        RestitutionComparisonSettings settings;
        settings.ball_restitution = 0.2;
        settings.floor_restitution = 0.8;
        const auto swapped = compare_restitution_drops(settings);
        const auto original = compare_restitution_drops();
        for (std::size_t index = 0; index < original.size(); ++index)
        {
            RIGIDBODIES_EXPECT_NEAR(swapped[index].measured_rebound_height_m, original[index].measured_rebound_height_m, 1.0e-12, "surface coefficient order does not alter the measured bounce");
            RIGIDBODIES_EXPECT(swapped[index].step_count == original[index].step_count, "symmetric runs reach apex on the same step");
        }
    }

    RIGIDBODIES_TEST("perfectly elastic and zero restitution drops have their expected limiting rebounds")
    {
        RestitutionComparisonSettings settings;
        settings.ball_restitution = 1.0;
        settings.floor_restitution = 1.0;
        settings.time_step_s = 1.0 / 480.0;
        const auto elastic = measure_restitution_drop(MaterialMixing::geometric_mean, settings);
        RIGIDBODIES_EXPECT_NEAR(elastic.measured_rebound_height_m, 1.0, 0.015, "elastic drop recovers release height within integration error");
        settings.ball_restitution = 0.0;
        settings.floor_restitution = 0.0;
        const auto inelastic = measure_restitution_drop(MaterialMixing::maximum, settings);
        RIGIDBODIES_EXPECT(inelastic.measured_rebound_height_m < 0.002 && inelastic.rebound_speed_m_s < 1.0e-8, "zero restitution rests instead of inventing a bounce");
        RIGIDBODIES_EXPECT_NEAR(inelastic.apex_time_s, inelastic.impact_time_s, 0.0, "non-bouncing impact is its own apex");
    }

    RIGIDBODIES_TEST("drop experiment rejects invalid inputs and bounded runs without a first impact")
    {
        for (const auto value : { 0.0, -1.0, std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
        {
            RestitutionComparisonSettings settings;
            settings.time_step_s = value;
            bool rejected = false;
            try
            {
                (void)measure_restitution_drop(MaterialMixing::maximum, settings);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "invalid step cannot enter a simulation loop");
        }
        for (int problem = 0; problem < 4; ++problem)
        {
            RestitutionComparisonSettings settings;
            if (problem == 0)
            {
                settings.ball_restitution = 1.1;
            }
            else if (problem == 1)
            {
                settings.mass_kg = 0.0;
            }
            else if (problem == 2)
            {
                settings.maximum_steps = 0;
            }
            else
            {
                settings.maximum_steps = 1000001;
            }
            bool rejected = false;
            try
            {
                (void)compare_restitution_drops(settings);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "invalid physical parameters and excessive work limits are rejected");
        }
        for (const bool limited_steps : { false, true })
        {
            RestitutionComparisonSettings settings;
            if (limited_steps)
            {
                settings.maximum_steps = 2;
            }
            else
            {
                settings.maximum_duration_s = 0.01;
            }
            bool rejected = false;
            try
            {
                (void)measure_restitution_drop(MaterialMixing::maximum, settings);
            }
            catch (const std::runtime_error&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "budget exhaustion without impact fails explicitly");
        }
    }

    RIGIDBODIES_TEST("numerical motion bounds cannot silently change restitution measurement conditions")
    {
        RestitutionComparisonSettings settings;
        settings.drop_height_m = 150.0;
        bool rejected = false;
        try
        {
            (void)measure_restitution_drop(MaterialMixing::maximum, settings);
        }
        catch (const std::overflow_error&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "clamped release pose cannot be reported as the requested drop height");
    }

    RIGIDBODIES_TEST("contact demonstrations expose stable keyed bodies and relevant material choices")
    {
        World world;
        for (const auto* scenario : { "stable_stack", "friction_comparison", "fast_projectile", "restitution_drop" })
        {
            RIGIDBODIES_EXPECT(load_scenario(world, scenario), "contact demonstration is registered");
            RIGIDBODIES_EXPECT(!world.compute_bounds().is_empty(), "contact demonstration has camera framing bounds");
        }
        RIGIDBODIES_EXPECT(world.body_ids().size() == 6, "restitution scene contains ball floor and four theoretical height markers");
        RIGIDBODIES_EXPECT(load_scenario(world, "friction_comparison"), "friction scenario loads");
        bool along = false;
        bool across = false;
        bool rolling = false;
        for (const auto id : world.body_ids())
        {
            const auto* body = world.find_body(id);
            const auto& material = body->colliders().front().material;
            if (body->name() == "slide_along_grain")
            {
                along = material.friction_anisotropy_ratio == 0.15 && material.friction_axis_local == rigidbodies::math::Vec2 { 1.0, 0.0 };
            }
            if (body->name() == "slide_across_grain")
            {
                across = material.friction_axis_local == rigidbodies::math::Vec2 { 0.0, 1.0 };
            }
            if (body->name() == "resisted_rolling_ball")
            {
                rolling = material.rolling_friction_m > 0.0 && material.spinning_friction_m > 0.0;
            }
        }
        RIGIDBODIES_EXPECT(along && across && rolling, "sliding and angular resistance examples use actual solver material parameters");
    }

    RIGIDBODIES_TEST("six box stack settles in order on a continuous chain of support and sleeps")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "stable_stack"), "stack scenario loads");
        std::array<BodyId, 7> supports;
        supports[0] = scenario_body(world, "stack_floor");
        for (std::size_t index = 1; index < supports.size(); ++index)
            supports[index] = scenario_body(world, "stack_box_" + std::to_string(index));

        step_without_limits(world, 600);
        RIGIDBODIES_EXPECT(world.statistics().sleeping_body_count == 6, "all six boxes settle and sleep within five seconds");
        std::array<rigidbodies::math::Vec2, 6> sleeping_positions;
        for (std::size_t index = 1; index < supports.size(); ++index)
        {
            const auto* box = world.find_body(supports[index]);
            const auto* below = world.find_body(supports[index - 1]);
            sleeping_positions[index - 1] = box->position_m();
            RIGIDBODIES_EXPECT(!box->is_awake(), "supported box remains asleep");
            RIGIDBODIES_EXPECT_NEAR(box->position_m().x, 0.0, 0.05, "stack keeps its horizontal alignment");
            RIGIDBODIES_EXPECT_NEAR(box->orientation_rad(), 0.0, 0.025, "boxes stay level rather than tipping");
            RIGIDBODIES_EXPECT_NEAR(box->position_m().y, -1.05 + static_cast<Real>(index - 1) * 0.3, 0.035, "box retains its original vertical order and support height");
            const auto lower_top = below->compute_bounds(below->transform()).maximum.y;
            const auto box_bottom = box->compute_bounds(box->transform()).minimum.y;
            RIGIDBODIES_EXPECT_NEAR(box_bottom, lower_top, 0.015, "each adjacent support remains in geometric contact");
            bool supported = false;
            for (const auto& manifold : world.manifolds())
            {
                const auto pair_matches = (manifold.first == supports[index] && manifold.second == supports[index - 1]) ||
                    (manifold.second == supports[index] && manifold.first == supports[index - 1]);
                if (!pair_matches || manifold.is_sensor)
                    continue;
                for (std::size_t point = 0; point < manifold.point_count; ++point)
                    supported = supported || (manifold.points[point].normal_impulse_n_s > 0.0 && manifold.points[point].separation_m <= 0.01);
            }
            RIGIDBODIES_EXPECT(supported, "each box retains an actual load-bearing contact to the box or floor below");
        }
        step_without_limits(world, 120);
        for (std::size_t index = 1; index < supports.size(); ++index)
        {
            const auto* box = world.find_body(supports[index]);
            RIGIDBODIES_EXPECT(box->position_m() == sleeping_positions[index - 1] && !box->is_awake(), "settled stack remains stationary for another second");
        }
    }

    RIGIDBODIES_TEST("friction demonstration distinguishes grain direction and rolling resistance in motion")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "friction_comparison"), "friction scenario loads");
        const auto along_id = scenario_body(world, "slide_along_grain");
        const auto across_id = scenario_body(world, "slide_across_grain");
        const auto ideal_id = scenario_body(world, "ideal_rolling_ball");
        const auto resisted_id = scenario_body(world, "resisted_rolling_ball");
        const auto start_x = world.find_body(along_id)->position_m().x;
        step_without_limits(world, 240);
        const auto& along = *world.find_body(along_id);
        const auto& across = *world.find_body(across_id);
        const auto& ideal = *world.find_body(ideal_id);
        const auto& resisted = *world.find_body(resisted_id);
        const auto along_distance = along.position_m().x - start_x;
        const auto across_distance = across.position_m().x - start_x;
        RIGIDBODIES_EXPECT(across_distance > 0.15 && along_distance > across_distance + 0.3, "preferred grain direction produces a visibly longer stopping distance");
        RIGIDBODIES_EXPECT_NEAR(along.linear_velocity_m_s().x, 0.0, 0.02, "along-grain block eventually stops under friction");
        RIGIDBODIES_EXPECT_NEAR(across.linear_velocity_m_s().x, 0.0, 0.02, "across-grain block eventually stops under friction");
        RIGIDBODIES_EXPECT_NEAR(ideal.linear_velocity_m_s().x, 1.5, 0.02, "ideal rolling preserves translation speed");
        RIGIDBODIES_EXPECT_NEAR(ideal.angular_velocity_rad_s(), -12.5, 0.1, "ideal rolling preserves spin and the no-slip relationship");
        RIGIDBODIES_EXPECT_NEAR(ideal.position_m().x - start_x, 3.0, 0.02, "ideal roller covers speed times elapsed time");
        RIGIDBODIES_EXPECT(std::abs(resisted.linear_velocity_m_s().x) < 0.15 && std::abs(resisted.angular_velocity_rad_s()) < 1.0, "rolling and spinning resistance slow both translation and rotation");
        RIGIDBODIES_EXPECT(ideal.position_m().x > resisted.position_m().x + 1.0, "rolling resistance changes the observed travel distance");
        RIGIDBODIES_EXPECT_NEAR(ideal.position_m().y, -0.38, 0.01, "ideal roller remains supported within its own lane");
        RIGIDBODIES_EXPECT_NEAR(resisted.position_m().y, -1.33, 0.01, "resisted roller remains supported within its own lane");
    }

    RIGIDBODIES_TEST("fast projectile rebounds with CCD but crosses the thin target when CCD is disabled")
    {
        for (const bool continuous : { true, false })
        {
            WorldSettings settings;
            settings.collision.continuous = continuous;
            World world { settings };
            RIGIDBODIES_EXPECT(load_scenario(world, "fast_projectile"), "projectile scenario loads");
            // A data document restores its full environment; the experiment then changes CCD.
            auto experiment_settings = world.settings();
            experiment_settings.collision.continuous = continuous;
            world.set_settings(experiment_settings);
            const auto projectile = scenario_body(world, "fast_ball");
            step_without_limits(world, 30);
            const auto* body = world.find_body(projectile);
            if (continuous)
            {
                RIGIDBODIES_EXPECT(body->position_m().x < 1.2, "CCD keeps the projectile on the approach side after rebound");
                RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, -24.0, 0.02, "thin target returns the restitution fraction of the incoming speed");
            }
            else
            {
                RIGIDBODIES_EXPECT(body->position_m().x > 1.225, "discrete samples miss the thin target between endpoints");
                RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 40.0, 1.0e-12, "missed target leaves incoming velocity unchanged");
            }
        }
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
