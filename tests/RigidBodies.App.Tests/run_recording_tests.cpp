#include <rigidbodies/app/run_recorder.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/shape.hpp>
#include <rigidbodies/ui/run_compare.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>

namespace
{
    using namespace rigidbodies;

    physics::World world_with_objects(std::size_t count)
    {
        physics::World world;
        auto settings = world.settings();
        settings.gravity_m_s2 = {};
        world.set_settings(settings);
        for (std::size_t index = 0; index < count; ++index)
        {
            physics::BodyDefinition definition;
            definition.name = "Object " + std::to_string(index + 1);
            definition.position_m = { static_cast<double>(index % 8), static_cast<double>(index / 8) };
            definition.linear_velocity_m_s = { static_cast<double>(index + 1) * 0.01, 0.0 };
            physics::Collider collider;
            collider.shape = physics::make_circle(0.1);
            definition.colliders.push_back(collider);
            world.create_body(definition);
        }
        return world;
    }

    void complete_run(app::RunRecorder& recorder, physics::World& world, double duration = 0.5)
    {
        recorder.begin_run(world, {}, {});
        const auto sample_count = static_cast<int>(duration / 0.025 + 0.5);
        for (int sample = 1; sample <= sample_count; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
    }

    void complete_full_run(app::RunRecorder& recorder, physics::World& world)
    {
        recorder.begin_run(world, {}, {});
        for (int sample = 1; sample <= 2400; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
    }

    class TemporaryCurrentDirectory
    {
    public:
        TemporaryCurrentDirectory()
            : original_(std::filesystem::current_path()),
              directory_(std::filesystem::temp_directory_path() /
                  ("rigid-bodies-run-data-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
        {
            std::filesystem::create_directory(directory_);
            std::filesystem::current_path(directory_);
        }

        ~TemporaryCurrentDirectory()
        {
            std::error_code error;
            std::filesystem::current_path(original_, error);
            std::filesystem::remove_all(directory_, error);
        }

        [[nodiscard]] const std::filesystem::path& path() const
        {
            return directory_;
        }

    private:
        std::filesystem::path original_, directory_;
    };

    RIGIDBODIES_TEST("run series use forty hertz floats and the documented full-run memory")
    {
        auto world = world_with_objects(32);
        app::RunRecorder recorder;
        recorder.set_experiment("memory");
        recorder.begin_run(world, {}, {});
        for (int sample = 1; sample <= 2400; ++sample)
            recorder.record_sample(world, sample * 0.025);
        const auto* run = recorder.current();
        RIGIDBODIES_EXPECT(run && run->series.time_s.size() == 2400, "a sixty-second run contains exactly 2,400 samples");
        RIGIDBODIES_EXPECT(sizeof(run->series.time_s.front()) == 4 && sizeof(run->series.scene.front()) == 4 && sizeof(run->series.objects.front()) == 4, "all recorded samples are four-byte floats");
        RIGIDBODIES_EXPECT(recorder.memory_bytes() == 4444800, "a full 32-object run occupies the prescribed 4,444,800 bytes");
    }

    RIGIDBODIES_TEST("the first sample reads momentum from the moving bodies and every row of the budget")
    {
        auto world = world_with_objects(3);
        app::RunRecorder recorder;
        recorder.set_experiment("first sample");
        recorder.begin_run(world, {}, {});
        recorder.record_sample(world, 0.0);
        const auto& series = recorder.current()->series;
        const auto expected = physics::measure_world_momentum_kg_m_s(world);
        RIGIDBODIES_EXPECT(expected.x > 0.0, "the objects start moving");
        RIGIDBODIES_EXPECT(series.scene.size() == 8 && std::abs(series.scene[6] - static_cast<float>(expected.x)) < 1.0e-6f, "momentum at t = 0 is the objects' m v, not the stale step statistics");
        RIGIDBODIES_EXPECT(series.ledger.size() == 6, "each sample records the six other rows of the energy budget");
        recorder.record_sample(world, 0.025);
        recorder.cut_at(0.0);
        RIGIDBODIES_EXPECT(series.ledger.size() == 6 && series.time_s.size() == 1, "cutting a run trims its budget rows with its samples");
    }

    RIGIDBODIES_TEST("retention keeps ten runs per experiment and refuses a ninth star")
    {
        auto world = world_with_objects(1);
        app::RunRecorder recorder;
        recorder.set_experiment("retention");
        for (int index = 0; index < 11; ++index)
            complete_run(recorder, world);
        const auto kept = recorder.kept("retention");
        RIGIDBODIES_EXPECT(kept.size() == 10 && kept.front().number == 2 && kept.back().number == 11, "the eleventh run drops the oldest unstarred run");
        for (int number = 2; number <= 9; ++number)
            RIGIDBODIES_EXPECT(recorder.star(number, true), "the first eight stars are accepted");
        RIGIDBODIES_EXPECT(!recorder.star(10, true), "a ninth star is refused");
        recorder.clear_unstarred();
        RIGIDBODIES_EXPECT(recorder.kept("retention").size() == 8, "Clear runs preserves every starred run");
        complete_run(recorder, world);
        RIGIDBODIES_EXPECT(recorder.kept("retention").back().number == 12, "clearing does not reset run numbering");
    }

    RIGIDBODIES_TEST("the shared memory budget drops the oldest unstarred runs across experiments")
    {
        auto world = world_with_objects(32);
        app::RunRecorder recorder;
        recorder.set_experiment("first");
        for (int index = 0; index < 10; ++index)
            complete_full_run(recorder, world);
        RIGIDBODIES_EXPECT(recorder.memory_bytes() + 4444800 <= 48u * 1024u * 1024u,
            "kept runs leave one full recording buffer inside the 48 MiB budget");
        recorder.set_experiment("second");
        for (int index = 0; index < 10; ++index)
        {
            complete_full_run(recorder, world);
            RIGIDBODIES_EXPECT(recorder.memory_bytes() + 4444800 <= 48u * 1024u * 1024u,
                "every keep enforces the shared session budget");
        }
        RIGIDBODIES_EXPECT(recorder.kept("first").empty(), "the globally oldest unstarred runs are dropped first");
        RIGIDBODIES_EXPECT(recorder.kept("second").size() == 10, "the ten newest full runs remain available");
    }

    RIGIDBODIES_TEST("a pinned free-fall maximum matches the analytic speed")
    {
        physics::World world;
        auto settings = world.settings();
        settings.sleep.enabled = false;
        world.set_settings(settings);
        physics::BodyDefinition definition;
        definition.position_m = { 0.0, 100.0 };
        definition.sleep_enabled = false;
        physics::Collider collider;
        collider.shape = physics::make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto body = world.create_body(definition);
        app::RunRecorder recorder;
        recorder.set_experiment("free-fall-analytic");
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:max", "speed", body, ui::RunAggregator::maximum, 0.0 }), "maximum speed can be pinned");
        recorder.begin_run(world, {}, {});
        for (int sample = 1; sample <= 20; ++sample)
        {
            world.step(0.025, true);
            recorder.record_sample(world, sample * 0.025);
        }
        recorder.close_run(true, true);
        const auto result = recorder.kept("free-fall-analytic").front().pinned_results.front();
        const auto expected = physics::standard_gravity_m_s2 * 0.5;
        RIGIDBODIES_EXPECT(result.has_value(), "the kept run receives its maximum-speed result");
        RIGIDBODIES_EXPECT_NEAR(*result, expected, expected * 0.01, "recorded maximum speed agrees with g times t within one percent");
    }

    RIGIDBODIES_TEST("a bouncing ball's recorded mechanical energy is level in flight and steps down at impacts")
    {
        physics::World world;
        RIGIDBODIES_EXPECT(physics::load_scenario(world, "restitution_drop"), "bundled scenario loads");
        app::RunRecorder recorder;
        recorder.set_experiment("restitution-drop-steps");
        recorder.begin_run(world, {}, {});
        recorder.record_sample(world, 0.0);
        bool rebound_pending = false;
        for (int step = 1; step <= 480; ++step)
        {
            world.step(1.0 / 120.0, true);
            rebound_pending = rebound_pending || physics::impact_in_progress(world);
            recorder.record_sample(world, world.statistics().elapsed_time_s);
        }
        RIGIDBODIES_EXPECT(rebound_pending, "fast impacts pass through a step that stops the ball before its rebound");
        const auto& series = recorder.current()->series;
        const auto mechanical = [&](std::size_t sample)
        {
            const auto* scene = series.scene.data() + sample * 8;
            return static_cast<double>(scene[0]) + scene[1] + scene[2] + scene[3];
        };
        double largest_rise = 0.0;
        for (std::size_t sample = 1; sample < series.time_s.size(); ++sample)
            largest_rise = std::max(largest_rise, mechanical(sample) - mechanical(sample - 1));
        RIGIDBODIES_EXPECT(series.time_s.size() > 150, "the run is sampled at forty hertz");
        RIGIDBODIES_EXPECT(largest_rise < 1.0e-3, "no sample catches an impact between its loss and the energy it returns");
    }

    RIGIDBODIES_TEST("short runs predictions and pinned values follow their lifecycle")
    {
        auto world = world_with_objects(1);
        app::RunRecorder recorder;
        recorder.set_experiment("lifecycle");
        recorder.begin_run(world, {}, {}, ui::Prediction { "faster", "" });
        recorder.record_sample(world, 0.25);
        recorder.close_run(true, true);
        RIGIDBODIES_EXPECT(recorder.kept("lifecycle").empty(), "runs shorter than half a second are not kept");
        ui::PinnedValue speed { "speed:max", "speed", world.body_ids().front(), ui::RunAggregator::maximum, 0.0 };
        RIGIDBODIES_EXPECT(recorder.pin(speed), "a graph quantity can be pinned");
        recorder.begin_run(world, {}, {}, ui::Prediction { "faster", "" });
        for (int sample = 1; sample <= 20; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
        const auto kept = recorder.kept("lifecycle");
        RIGIDBODIES_EXPECT(kept.size() == 1 && kept.front().prediction && kept.front().prediction->option == "faster", "the next kept run owns the prediction");
        RIGIDBODIES_EXPECT(kept.front().pinned_results.size() == 1 && kept.front().pinned_results.front().has_value(), "a kept run receives its pinned aggregate");
        for (int index = 1; index < 12; ++index)
        {
            auto value = speed;
            value.key = "speed:" + std::to_string(index);
            RIGIDBODIES_EXPECT(recorder.pin(value), "the first twelve pinned values are accepted");
        }
        auto thirteenth = speed;
        thirteenth.key = "speed:13";
        RIGIDBODIES_EXPECT(!recorder.pin(thirteenth), "a thirteenth pinned value is refused");
    }

    RIGIDBODIES_TEST("an at-time aggregate outside the rolling sixty-second window has no value")
    {
        auto world = world_with_objects(1);
        app::RunRecorder recorder;
        recorder.set_experiment("rolling-window");
        const auto body = world.body_ids().front();
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:early", "speed", body, ui::RunAggregator::at_time, 0.025 }),
            "the early sample can be requested before recording");
        recorder.begin_run(world, {}, {});
        for (int sample = 1; sample <= 2401; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
        const auto kept = recorder.kept("rolling-window");
        RIGIDBODIES_EXPECT(kept.size() == 1 && kept.front().series.time_s.front() > 0.025f,
            "the oldest sample is dropped once the 2,400-sample window advances");
        RIGIDBODIES_EXPECT(kept.front().pinned_results.size() == 1 && !kept.front().pinned_results.front(),
            "an aggregate that needs the dropped sample is unavailable");
    }

    RIGIDBODIES_TEST("recording retention stars pins and clearing write no run data")
    {
        TemporaryCurrentDirectory user_data;
        auto world = world_with_objects(2);
        app::RunRecorder recorder;
        recorder.set_experiment("no-disk-first");
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:max", "speed", world.body_ids().front(), ui::RunAggregator::maximum, 0.0 }),
            "the no-disk script pins a value");
        complete_run(recorder, world);
        RIGIDBODIES_EXPECT(recorder.star(1, true), "the no-disk script stars its first run");
        recorder.set_experiment("no-disk-second");
        for (int index = 0; index < 11; ++index)
            complete_run(recorder, world);
        recorder.clear_unstarred();
        RIGIDBODIES_EXPECT(std::filesystem::is_empty(user_data.path()),
            "the user data folder remains unchanged after recording, keeping, dropping, starring, pinning and clearing runs");
    }

    RIGIDBODIES_TEST("identical scripts produce bit-identical run series")
    {
        auto first_world = world_with_objects(3);
        auto second_world = world_with_objects(3);
        app::RunRecorder first, second;
        first.set_experiment("determinism");
        second.set_experiment("determinism");
        first.begin_run(first_world, {}, {});
        second.begin_run(second_world, {}, {});
        for (int sample = 1; sample <= 80; ++sample)
        {
            first_world.step(0.0125, true);
            second_world.step(0.0125, true);
            first.record_sample(first_world, sample * 0.025);
            second.record_sample(second_world, sample * 0.025);
        }
        const auto* a = first.current();
        const auto* b = second.current();
        RIGIDBODIES_EXPECT(a && b && a->series.time_s == b->series.time_s && a->series.scene == b->series.scene && a->series.objects == b->series.objects, "recording does not perturb deterministic results");
    }

    RIGIDBODIES_TEST("a prediction is consumed by exactly the next run")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "the prediction fixture opens");
        ui::UiCommand prediction;
        prediction.kind = ui::UiCommandKind::set_prediction;
        prediction.id = "Together";
        session.apply(prediction);
        session.stepper().set_paused(false);
        for (int frame = 0; frame < 40; ++frame)
            session.advance(0.025);
        session.reset_scenario();
        auto runs = session.build_model().runs;
        RIGIDBODIES_EXPECT(runs.size() == 1 && runs.front().prediction && runs.front().prediction->option == "Together",
            "the first completed run owns the pending prediction");
        session.stepper().set_paused(false);
        for (int frame = 0; frame < 40; ++frame)
            session.advance(0.025);
        session.reset_scenario();
        runs = session.build_model().runs;
        RIGIDBODIES_EXPECT(runs.size() == 2 && !runs.back().prediction, "later runs receive no prediction until one is made again");
    }

    RIGIDBODIES_TEST("removing and replacing pinned values never reuses a live identity")
    {
        auto world = world_with_objects(1);
        app::RunRecorder recorder;
        recorder.set_experiment("pin identity");
        ui::PinnedValue value { {}, "speed", world.body_ids().front(), ui::RunAggregator::maximum, 0.0 };
        for (int index = 0; index < 3; ++index)
            RIGIDBODIES_EXPECT(recorder.pin(value), "a value can be added");
        const auto first = recorder.pinned("pin identity")[0].key;
        const auto last = recorder.pinned("pin identity")[2].key;
        RIGIDBODIES_EXPECT(recorder.unpin(first) && recorder.pin(value), "a removed slot can be replaced");
        const auto replacement = recorder.pinned("pin identity")[2].key;
        RIGIDBODIES_EXPECT(replacement != last && replacement != first, "the replacement has a new stable identity");
        RIGIDBODIES_EXPECT(recorder.unpin(last) && recorder.pinned("pin identity").size() == 2, "removing the original value removes exactly one definition");
        value.key = replacement;
        RIGIDBODIES_EXPECT(!recorder.pin(value), "an explicit duplicate identity is rejected");
    }

    RIGIDBODIES_TEST("pinned times interpolate samples and missing body values remain unavailable")
    {
        auto world = world_with_objects(1);
        const auto body = world.body_ids().front();
        app::RunRecorder recorder;
        recorder.set_experiment("pin samples");
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:time", "speed", body, ui::RunAggregator::at_time, 0.0125 }), "a between-sample time can be measured");
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:end", "speed", body, ui::RunAggregator::at_end, 0.0 }), "an end value can be measured");
        RIGIDBODIES_EXPECT(recorder.pin({ "speed:future", "speed", body, ui::RunAggregator::at_time, 1.0 }), "a future instant can be requested before recording");
        recorder.begin_run(world, {}, {});
        world.find_body(body)->set_linear_velocity({ 1.0, 0.0 });
        recorder.record_sample(world, 0.0);
        world.find_body(body)->set_linear_velocity({ 3.0, 0.0 });
        recorder.record_sample(world, 0.025);
        world.destroy_body(body);
        for (int sample = 2; sample <= 20; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
        RIGIDBODIES_EXPECT(recorder.kept("pin samples").size() == 1, "the fixture records a complete half-second run");
        const auto& results = recorder.kept("pin samples").front().pinned_results;
        RIGIDBODIES_EXPECT(results[0].has_value(), "the requested time is in the recorded interval");
        RIGIDBODIES_EXPECT_NEAR(*results[0], 2.0, 1.0e-6, "At time measures that instant rather than always rounding forward");
        RIGIDBODIES_EXPECT(!results[1], "a missing final object does not expose NaN as a measured value");
        RIGIDBODIES_EXPECT(!results[2], "an instant beyond the last recorded sample never substitutes the final value");
    }

    RIGIDBODIES_TEST("invalid pinned scopes quantities and times cannot enter the runs table")
    {
        auto world = world_with_objects(1);
        app::RunRecorder recorder;
        recorder.set_experiment("pin validation");
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "speed", {}, ui::RunAggregator::maximum, 0.0 }), "Speed needs an object");
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "lost_impacts", world.body_ids().front(), ui::RunAggregator::maximum, 0.0 }), "contact losses need whole-scene scope");
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "unknown", {}, ui::RunAggregator::maximum, 0.0 }), "unknown channels are rejected");
        for (const auto time : { -1.0, 60.1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
            RIGIDBODIES_EXPECT(!recorder.pin({ {}, "mechanical", {}, ui::RunAggregator::at_time, time }), "invalid requested times cannot masquerade as zero");
        RIGIDBODIES_EXPECT(recorder.pinned("pin validation").empty(), "invalid values consume no pin slots");
        RIGIDBODIES_EXPECT(recorder.pin({ {}, "mechanical", {}, ui::RunAggregator::at_time, 0.7 }) &&
                recorder.pin({ {}, "mechanical", {}, ui::RunAggregator::at_time, 0.701 }),
            "valid requested times can be prepared before a run");
        recorder.begin_run(world, {}, {});
        recorder.record_sample(world, 0.0);
        for (int sample = 1; sample <= 28; ++sample)
            recorder.record_sample(world, sample * 0.025);
        recorder.close_run(true, true);
        RIGIDBODIES_EXPECT(recorder.kept("pin validation").size() == 1, "the fixture records all samples through 0.7 s");
        const auto& results = recorder.kept("pin validation").front().pinned_results;
        RIGIDBODIES_EXPECT(results[0].has_value() && !results[1], "float timestamp rounding preserves the exact recorded endpoint without admitting a later instant");
    }

    RIGIDBODIES_TEST("run comparison uses B minus A absolute-A percentages and exact setup differences")
    {
        ui::RunRecord a, b;
        a.pinned_results = { -2.0, 0.0, std::nullopt };
        b.pinned_results = { 1.0, 4.0, 9.0 };
        const auto negative = ui::compare_run_value(a, b, 0);
        RIGIDBODIES_EXPECT(negative.delta && negative.delta_percent, "complete values produce both comparison columns");
        RIGIDBODIES_EXPECT_NEAR(*negative.delta, 3.0, 1.0e-12, "delta is B minus A");
        RIGIDBODIES_EXPECT_NEAR(*negative.delta_percent, 1.5, 1.0e-12, "percentage divides by absolute A");
        const auto zero = ui::compare_run_value(a, b, 1);
        RIGIDBODIES_EXPECT(zero.delta && !zero.delta_percent, "zero A produces an em-dash percentage");
        const auto missing = ui::compare_run_value(a, b, 2);
        RIGIDBODIES_EXPECT(!missing.delta, "a missing recorded value produces no comparison");

        a.changes_from_original = {
            { "mass", {}, {}, {}, {}, "2 kg", ui::EditCategory::parameter },
            { "gravity", {}, {}, {}, {}, "Moon", ui::EditCategory::parameter }
        };
        b.changes_from_original = {
            { "mass", {}, {}, {}, {}, "3 kg", ui::EditCategory::parameter },
            { "material", {}, {}, {}, {}, "Steel", ui::EditCategory::parameter }
        };
        RIGIDBODIES_EXPECT(ui::setup_difference_count(a, b) == 3, "changed and one-sided setup keys are each counted exactly once");
    }

    // Advances the probe and the recorder together by world substeps, as the session does: a world
    // second is a lab nanosecond.
    double record_probe(app::RunRecorder& recorder, const physics::World& world, physics::RelativisticProbe& probe, double time_s, double until_s, double substep_s)
    {
        while (time_s < until_s - 1.0e-9)
        {
            probe.advance(substep_s * physics::relativity_lab_seconds_per_world_second);
            time_s += substep_s;
            recorder.record_sample(world, time_s, &probe);
        }
        return time_s;
    }

    RIGIDBODIES_TEST("relativity channels are sampled at 40 Hz and cut with the run")
    {
        physics::World world;
        physics::RelativisticProbe probe({ 1.0, 0.6 });
        app::RunRecorder recorder;
        recorder.set_experiment("relativity channels");
        recorder.begin_run(world, {}, {}, {}, true);
        recorder.record_sample(world, 0.0, &probe);
        // A 1/90 s substep does not divide the 25 ms grid, so most samples are taken past their
        // grid point; the clocks are still read at the grid point itself.
        auto time_s = record_probe(recorder, world, probe, 0.0, 10.0, 1.0 / 90.0);
        const auto& series = recorder.current()->series;
        RIGIDBODIES_EXPECT(series.time_s.size() == 401 && series.relativity.size() == 3 * series.time_s.size(), "ten seconds hold 401 samples of three relativity channels each");
        bool on_grid = true, clocks_agree = true;
        for (std::size_t sample = 0; sample < series.time_s.size(); ++sample)
        {
            const auto t = static_cast<double>(series.time_s[sample]);
            on_grid = on_grid && std::abs(t - 0.025 * static_cast<double>(sample)) < 1.0e-5;
            const auto lab_s = t * 1.0e-9;
            const auto tau = static_cast<double>(series.relativity[3 * sample]);
            const auto gap = static_cast<double>(series.relativity[3 * sample + 1]);
            clocks_agree = clocks_agree && std::abs(tau - 0.8 * lab_s) <= 1.0e-6 * lab_s + 1.0e-18 && std::abs(gap - 0.2 * lab_s) <= 1.0e-6 * lab_s + 1.0e-18 &&
                series.relativity[3 * sample + 2] == static_cast<float>(probe.factors().lorentz_factor_minus_one);
        }
        RIGIDBODIES_EXPECT(on_grid, "the samples lie on the 40 Hz grid");
        RIGIDBODIES_EXPECT(clocks_agree, "every sample reads τ = 0.8 t, t − τ = 0.2 t and γ − 1 = 0.25 at its own instant");

        recorder.cut_at(5.0);
        RIGIDBODIES_EXPECT(series.time_s.size() == 201 && series.relativity.size() == 603, "cutting a run trims its relativity block with its samples");
        RIGIDBODIES_EXPECT_NEAR(static_cast<double>(series.relativity[600]), 4.0e-9, 1.0e-14, "the last kept sample is the probe clock at 5 ns");

        // The run continues from the cut, as after an undo, until the 60-second window rolls.
        physics::RelativisticProbe resumed({ 1.0, 0.6 });
        resumed.advance(5.0e-9);
        time_s = record_probe(recorder, world, resumed, 5.0, 70.0, 1.0 / 120.0);
        RIGIDBODIES_EXPECT(series.time_s.size() == 2400 && series.relativity.size() == 7200, "the rolling window drops a sample's three channels with its time");
        RIGIDBODIES_EXPECT_NEAR(static_cast<double>(series.relativity[0]), 0.8e-9 * static_cast<double>(series.time_s.front()), 1.0e-14, "the oldest retained sample keeps its own clock reading");

        const auto before = recorder.memory_bytes();
        recorder.record_sample(world, time_s + 0.025, nullptr);
        RIGIDBODIES_EXPECT(series.relativity.size() == 3 * series.time_s.size() && std::isnan(series.relativity.back()), "a sample without the probe keeps the block aligned with NaN");
        RIGIDBODIES_EXPECT(recorder.memory_bytes() == before, "the block was reserved for a full run at the start");
    }

    RIGIDBODIES_TEST("Newtonian runs keep their 4 444 800-byte budget")
    {
        auto world = world_with_objects(32);
        physics::RelativisticProbe probe({ 1.0, 0.5 });
        app::RunRecorder recorder;
        recorder.set_experiment("newtonian budget");
        recorder.begin_run(world, {}, {});
        for (int sample = 1; sample <= 2400; ++sample)
            recorder.record_sample(world, sample * 0.025, &probe);
        const auto* run = recorder.current();
        RIGIDBODIES_EXPECT(run && run->series.relativity.empty() && run->series.relativity.capacity() == 0, "a Newtonian run records no relativity block, even when handed a probe");
        RIGIDBODIES_EXPECT(recorder.memory_bytes() == 4444800, "a full 32-object Newtonian run still occupies exactly 4,444,800 bytes");
        recorder.close_run(true, true);
        recorder.begin_run(world, {}, {}, {}, true);
        for (int sample = 1; sample <= 2400; ++sample)
            recorder.record_sample(world, sample * 0.025, &probe);
        RIGIDBODIES_EXPECT(recorder.current()->series.relativity.size() == 7200 && recorder.memory_bytes() == 2 * 4444800 + 2400 * 3 * 4, "a relativity run adds exactly its three channels: 28,800 bytes for 60 s");
    }

    RIGIDBODIES_TEST("relativity pins validate and aggregate")
    {
        physics::World world;
        app::RunRecorder recorder;
        recorder.set_experiment("relativity pins");
        const physics::BodyId object { 0, 1 };
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "probe_clock", object, ui::RunAggregator::at_end, 0.0 }), "the clocks belong to the experiment, not to an object");
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "lorentz", {}, ui::RunAggregator::at_first_impact, 0.0 }) && !recorder.pin({ {}, "lab_clock", {}, ui::RunAggregator::at_first_impact, 0.0 }), "nothing collides, so nothing is read at a first impact");
        RIGIDBODIES_EXPECT(!recorder.pin({ {}, "lab_clock", {}, ui::RunAggregator::at_time, 60.5 }) && !recorder.pin({ {}, "probe_time", {}, ui::RunAggregator::at_end, 0.0 }), "times past the window and unknown clocks are refused");
        RIGIDBODIES_EXPECT(recorder.pinned("relativity pins").empty(), "refused values take no slot");
        for (const ui::PinnedValue& value : { ui::PinnedValue { "lab", "lab_clock", {}, ui::RunAggregator::at_end, 0.0 }, ui::PinnedValue { "tau", "probe_clock", {}, ui::RunAggregator::at_end, 0.0 }, ui::PinnedValue { "tau5", "probe_clock", {}, ui::RunAggregator::at_time, 5.0 }, ui::PinnedValue { "tau6", "probe_clock", {}, ui::RunAggregator::at_time, 6.0125 }, ui::PinnedValue { "gap", "clock_gap", {}, ui::RunAggregator::maximum, 0.0 }, ui::PinnedValue { "slow", "lorentz", {}, ui::RunAggregator::minimum, 0.0 }, ui::PinnedValue { "fast", "lorentz", {}, ui::RunAggregator::maximum, 0.0 }, ui::PinnedValue { "lab5", "lab_clock", {}, ui::RunAggregator::at_time, 5.0 } })
            RIGIDBODIES_EXPECT(recorder.pin(value), "a clock or the Lorentz factor can be pinned: " + value.key);

        // 0.6 c for 5 ns, then 0.8 c for 5 ns.
        physics::RelativisticProbe probe({ 1.0, 0.6 });
        recorder.begin_run(world, {}, {}, {}, true);
        recorder.record_sample(world, 0.0, &probe);
        auto time_s = record_probe(recorder, world, probe, 0.0, 5.0, 1.0 / 120.0);
        probe.set_speed_fraction(0.8);
        time_s = record_probe(recorder, world, probe, time_s, 10.0, 1.0 / 120.0);
        recorder.close_run(true, true);
        const auto kept = recorder.kept("relativity pins");
        RIGIDBODIES_EXPECT(kept.size() == 1 && kept.front().pinned_results.size() == 8, "the run is kept with one result per pinned value");
        if (kept.size() != 1 || kept.front().pinned_results.size() != 8)
            return;
        const auto& results = kept.front().pinned_results;
        for (const auto& result : results)
            RIGIDBODIES_EXPECT(result.has_value(), "every pinned clock has a value");
        if (std::any_of(results.begin(), results.end(), [](const auto& result)
                {
                    return !result;
                }))
            return;
        RIGIDBODIES_EXPECT_NEAR(*results[0], 10.0e-9, 1.0e-15, "the lab clock at the end reads 10 ns");
        RIGIDBODIES_EXPECT_NEAR(*results[1], 7.0e-9, 1.0e-14, "the probe clock gains 0.8 × 5 + 0.6 × 5 = 7 ns");
        RIGIDBODIES_EXPECT_NEAR(*results[2], 4.0e-9, 1.0e-14, "at 5 ns the probe clock read 4 ns");
        RIGIDBODIES_EXPECT_NEAR(*results[3], 4.0e-9 + 0.6 * 1.0125e-9, 1.0e-14, "between samples the reading is interpolated");
        RIGIDBODIES_EXPECT_NEAR(*results[4], 3.0e-9, 1.0e-14, "the largest gap is the final 1 + 2 = 3 ns");
        RIGIDBODIES_EXPECT_NEAR(*results[5], 0.25, 1.0e-7, "the smallest γ − 1 is 0.25 at 0.6 c");
        RIGIDBODIES_EXPECT_NEAR(*results[6], 2.0 / 3.0, 1.0e-7, "the largest γ − 1 is 2/3 at 0.8 c");
        RIGIDBODIES_EXPECT_NEAR(*results[7], 5.0e-9, 1.0e-15, "the lab clock at 5 ns reads 5 ns");

        recorder.begin_run(world, {}, {});
        for (int sample = 0; sample <= 40; ++sample)
            recorder.record_sample(world, sample * 0.025, &probe);
        recorder.close_run(true, true);
        const auto& newtonian = recorder.kept("relativity pins").back().pinned_results;
        RIGIDBODIES_EXPECT(newtonian.size() == 8 && std::none_of(newtonian.begin(), newtonian.end(), [](const auto& result)
                                                        {
                                                            return result.has_value();
                                                        }),
            "a run without the relativity block has no clock values");
    }

    RIGIDBODIES_TEST("a relativity session records the clocks with every run")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light opens");
        ui::UiCommand speed;
        speed.kind = ui::UiCommandKind::set_relativity_speed;
        speed.value = 0.6;
        session.apply(speed);
        // The readings' Add to runs table sends the channel alone, read at the end of each run.
        for (const auto* channel : { "probe_clock", "lab_clock", "clock_gap", "lorentz" })
        {
            ui::UiCommand pin;
            pin.kind = ui::UiCommandKind::pin_run_value;
            pin.id = channel;
            session.apply(pin);
        }
        const auto pinned_model = session.build_model();
        RIGIDBODIES_EXPECT(pinned_model.pinned_values.size() == 4 && pinned_model.pinned_values.front().quantity == "probe_clock" && pinned_model.pinned_values.front().aggregator == ui::RunAggregator::at_end, "the clocks and γ are pinned for every run");
        RIGIDBODIES_EXPECT(std::none_of(pinned_model.notifications.begin(), pinned_model.notifications.end(), [](const auto& notification)
                               {
                                   return notification.source == "runs";
                               }),
            "no pin is refused");
        // The context menu cannot see the table, so a second Add to runs table on the same reading
        // is refused as Add value refuses it.
        ui::UiCommand again;
        again.kind = ui::UiCommandKind::pin_run_value;
        again.id = "probe_clock";
        session.apply(again);
        const auto repeated_model = session.build_model();
        RIGIDBODIES_EXPECT(repeated_model.pinned_values.size() == 4, "pinning the probe clock again takes no second slot");
        RIGIDBODIES_EXPECT(std::any_of(repeated_model.notifications.begin(), repeated_model.notifications.end(), [](const auto& notification)
                               {
                                   return notification.source == "runs" && notification.text == "This value is already pinned.";
                               }),
            "the repeated pin says the value is already pinned");
        session.stepper().set_paused(false);
        for (int frame = 0; frame < 400; ++frame)
            session.advance(0.025);
        const auto model = session.build_model();
        const auto* run = model.current_run;
        RIGIDBODIES_EXPECT(run && run->series.time_s.size() > 300 && run->series.relativity.size() == 3 * run->series.time_s.size(), "the run records three relativity channels with every sample");
        if (!run)
            return;
        bool proper_time = true;
        for (std::size_t sample = 0; sample < run->series.time_s.size(); ++sample)
        {
            const auto lab_s = static_cast<double>(run->series.time_s[sample]) * 1.0e-9;
            proper_time = proper_time && std::abs(static_cast<double>(run->series.relativity[3 * sample]) - 0.8 * lab_s) <= 1.0e-6 * lab_s + 1.0e-18;
        }
        RIGIDBODIES_EXPECT(proper_time, "each sample's probe clock is 0.8 of its lab time");
        session.reset_scenario();
        const auto kept = session.build_model().runs;
        RIGIDBODIES_EXPECT(kept.size() == 1 && kept.front().pinned_results.size() == 4 && kept.front().pinned_results.front(), "Back to start keeps the run with its pinned clock");
        if (kept.size() == 1 && kept.front().pinned_results.size() == 4 && kept.front().pinned_results.front())
            RIGIDBODIES_EXPECT_NEAR(*kept.front().pinned_results.front(), 0.8 * kept.front().duration_s * 1.0e-9, 1.0e-15, "the pinned probe clock reads 0.8 of the run's lab time");

        app::SimulationSession newtonian_session;
        RIGIDBODIES_EXPECT(newtonian_session.load_scenario("free_fall"), "a Newtonian experiment opens");
        newtonian_session.stepper().set_paused(false);
        for (int frame = 0; frame < 40; ++frame)
            newtonian_session.advance(0.025);
        const auto* newtonian = newtonian_session.build_model().current_run;
        RIGIDBODIES_EXPECT(newtonian && newtonian->series.relativity.empty(), "a Newtonian run records no relativity block");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
