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
}

int main()
{
    return rigidbodies::testing::run_all();
}
