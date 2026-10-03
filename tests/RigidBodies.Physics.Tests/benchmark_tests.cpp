#include <rigidbodies/physics/benchmark.hpp>

#include "test_framework.hpp"

#include <limits>
#include <set>

namespace
{
    using namespace rigidbodies::physics;

    BenchmarkReport report(Real duration = 1.0)
    {
        BenchmarkReport result;
        for (const auto* key : { "machine", "platform", "architecture", "cpu", "hardware_threads", "compiler", "build", "project_version" })
            result.metadata[key] = "fixture";
        BenchmarkResult workload { "sparse_128", "stable-content", {} };
        for (int index = 0; index < 3; ++index)
        {
            BenchmarkRun run;
            run.step = { duration, duration * 1.1, duration * 0.9, duration * 1.2 };
            run.phases["broad_phase"] = { duration * 0.1, duration * 0.11, duration * 0.09, duration * 0.12 };
            run.bodies = 128;
            run.state_digest = "final-state";
            workload.runs.push_back(run);
        }
        result.results.push_back(workload);
        return result;
    }

    RIGIDBODIES_TEST("benchmark catalogue has unique representative and scale workloads")
    {
        std::set<std::string> ids;
        for (const auto& workload : benchmark_workloads())
        {
            RIGIDBODIES_EXPECT(ids.insert(workload.id).second, "unique workload identifier");
            RIGIDBODIES_EXPECT(!workload.description.empty(), "human-readable workload description");
        }
        RIGIDBODIES_EXPECT(ids.count("scene_authored") && ids.count("dense_256") && ids.count("mechanisms_96") && ids.count("sleeping_512"), "representative content, contacts, mechanisms and sleeping cases");
    }

    RIGIDBODIES_TEST("generated workload seed determines identical body state and counts")
    {
        World first, second, different;
        populate_benchmark_world(first, "sparse_128", 42);
        populate_benchmark_world(second, "sparse_128", 42);
        populate_benchmark_world(different, "sparse_128", 43);
        const auto first_ids = first.body_ids(), second_ids = second.body_ids(), different_ids = different.body_ids();
        RIGIDBODIES_EXPECT(first_ids.size() == 128 && second_ids.size() == 128, "bounded known body count");
        for (std::size_t index = 0; index < first_ids.size(); ++index)
        {
            const auto& left = *first.find_body(first_ids[index]);
            const auto& right = *second.find_body(second_ids[index]);
            RIGIDBODIES_EXPECT(left.position_m() == right.position_m() && left.linear_velocity_m_s() == right.linear_velocity_m_s(), "fixed seed has exact states");
        }
        RIGIDBODIES_EXPECT(!(first.find_body(first_ids.front())->linear_velocity_m_s() == different.find_body(different_ids.front())->linear_velocity_m_s()), "seed changes specified integer RNG output");
    }

    RIGIDBODIES_TEST("mechanism and sleeping workloads have independent intended groups")
    {
        World mechanisms, sleepers;
        populate_benchmark_world(mechanisms, "mechanisms_96", 1);
        populate_benchmark_world(sleepers, "sleeping_512", 1);
        RIGIDBODIES_EXPECT(mechanisms.body_ids().size() == 120 && mechanisms.constraints().size() == 96, "24 anchors and 96 linked bodies");
        std::size_t sleeping = 0;
        for (const auto id : sleepers.body_ids())
            if (sleepers.find_body(id)->type() == BodyType::dynamic_body && !sleepers.find_body(id)->is_awake())
                ++sleeping;
        RIGIDBODIES_EXPECT(sleeping == 512, "sleeping workload starts settled");
    }

    RIGIDBODIES_TEST("percentiles use nearest rank and reject invalid samples")
    {
        const auto summary = summarize_benchmark_samples({ 4, 1, 3, 2, 5 });
        RIGIDBODIES_EXPECT(summary.median_ms == 3 && summary.p95_ms == 5 && summary.minimum_ms == 1 && summary.maximum_ms == 5, "known unsorted sample percentiles");
        for (const auto& invalid : { std::vector<Real> {}, std::vector<Real> { -1 }, std::vector<Real> { std::numeric_limits<Real>::infinity() } })
        {
            bool rejected = false;
            try
            {
                (void)summarize_benchmark_samples(invalid);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "invalid sample rejected");
        }
    }

    RIGIDBODIES_TEST("small benchmark checks deterministic repeat state without timing assertions")
    {
        BenchmarkSettings settings;
        settings.warmup_steps = 1;
        settings.sample_steps = 2;
        settings.repetitions = 2;
        settings.profiling = false;
        const auto measured = measure_benchmark("sparse_128", settings);
        RIGIDBODIES_EXPECT(measured.runs.size() == 2 && measured.runs.front().bodies == 128, "requested repetitions and workload body count");
        RIGIDBODIES_EXPECT(measured.runs.front().state_digest == measured.runs.back().state_digest, "identical starting states and fixed steps");
        RIGIDBODIES_EXPECT(measured.runs.front().phases.empty(), "profiling explicitly disabled");
    }

    RIGIDBODIES_TEST("benchmark rejects unsupported workload and unsafe iteration budgets")
    {
        bool unknown = false, budget = false;
        try
        {
            (void)measure_benchmark("unknown");
        }
        catch (const std::invalid_argument&)
        {
            unknown = true;
        }
        BenchmarkSettings settings;
        settings.sample_steps = 0;
        try
        {
            (void)measure_benchmark("sparse_128", settings);
        }
        catch (const std::invalid_argument&)
        {
            budget = true;
        }
        RIGIDBODIES_EXPECT(unknown && budget, "invalid runs fail before expensive work");
    }

    RIGIDBODIES_TEST("benchmark report round trip retains settings metadata timings counts and digest")
    {
        const auto original = report();
        BenchmarkReport decoded;
        std::string error;
        RIGIDBODIES_EXPECT(parse_benchmark_report(write_benchmark_report(original), decoded, error), error);
        RIGIDBODIES_EXPECT(decoded.metadata == original.metadata && decoded.results.size() == 1, "metadata and workload retained");
        RIGIDBODIES_EXPECT(decoded.settings.seed == original.settings.seed && decoded.results.front().runs.front().bodies == 128, "settings and counts retained");
        RIGIDBODIES_EXPECT(decoded.results.front().runs.front().phases.at("broad_phase").p95_ms == 0.11, "phase timings retained");
        RIGIDBODIES_EXPECT(decoded.results.front().runs.front().state_digest == "final-state", "state evidence retained");
    }

    RIGIDBODIES_TEST("malformed report is transactional and missing machine metadata is rejected")
    {
        auto decoded = report(5);
        std::string error;
        RIGIDBODIES_EXPECT(!parse_benchmark_report("{", decoded, error), "malformed JSON rejected");
        RIGIDBODIES_EXPECT(decoded.results.front().runs.front().step.median_ms == 5, "failed parse preserves target");
        auto missing = report();
        missing.metadata.erase("cpu");
        RIGIDBODIES_EXPECT(!parse_benchmark_report(write_benchmark_report(missing), decoded, error), "incomplete machine fingerprint rejected");
    }

    RIGIDBODIES_TEST("report parser rejects duplicate workloads missing repetitions and unordered percentiles")
    {
        auto invalid = report();
        invalid.results.push_back(invalid.results.front());
        BenchmarkReport decoded;
        std::string error;
        RIGIDBODIES_EXPECT(!parse_benchmark_report(write_benchmark_report(invalid), decoded, error), "duplicate workload rejected");
        invalid = report();
        invalid.results.front().runs.pop_back();
        RIGIDBODIES_EXPECT(!parse_benchmark_report(write_benchmark_report(invalid), decoded, error), "missing repetition rejected");
        invalid = report();
        invalid.results.front().runs.front().step.p95_ms = 0.5;
        RIGIDBODIES_EXPECT(!parse_benchmark_report(write_benchmark_report(invalid), decoded, error), "unordered timing rejected");
    }

    RIGIDBODIES_TEST("report parser accepts additive fields but rejects changed suite and major versions")
    {
        content::Json root;
        std::string error;
        RIGIDBODIES_EXPECT(content::parse_json(write_benchmark_report(report()), root, error), error);
        root["extra"] = "future";
        BenchmarkReport decoded;
        RIGIDBODIES_EXPECT(parse_benchmark_report(content::write_json(root), decoded, error), "additive metadata is harmless");
        root["suite_revision"] = 2;
        RIGIDBODIES_EXPECT(!parse_benchmark_report(content::write_json(root), decoded, error), "changed workload semantics rejected");
        root["suite_revision"] = 1;
        root["version"]["major"] = 2;
        RIGIDBODIES_EXPECT(!parse_benchmark_report(content::write_json(root), decoded, error), "future major format rejected");
    }

    RIGIDBODIES_TEST("same performance and faster performance do not regress")
    {
        RIGIDBODIES_EXPECT(compare_benchmarks(report(), report()).regressions.empty(), "identical timing passes");
        const auto comparison = compare_benchmarks(report(), report(0.5));
        RIGIDBODIES_EXPECT(comparison.compatible && comparison.compared_workloads == 1 && comparison.regressions.empty(), "improvement passes");
    }

    RIGIDBODIES_TEST("sustained median and tail slowdown is a confirmed regression")
    {
        const auto comparison = compare_benchmarks(report(), report(1.5));
        RIGIDBODIES_EXPECT(comparison.compatible && comparison.compared_workloads == 1 && comparison.regressions.size() == 2, "both metrics exceed threshold on every repetition");
    }

    RIGIDBODIES_TEST("tail-only sustained slowdown is detected independently of median")
    {
        auto slower = report();
        for (auto& run : slower.results.front().runs)
        {
            run.step.p95_ms = 2;
            run.step.maximum_ms = 2.1;
        }
        const auto comparison = compare_benchmarks(report(), slower);
        RIGIDBODIES_EXPECT(comparison.regressions.size() == 1 && comparison.regressions.front().find("p95") != std::string::npos, "tail deterioration detected");
    }

    RIGIDBODIES_TEST("relative and absolute allowances suppress immaterial differences")
    {
        RIGIDBODIES_EXPECT(compare_benchmarks(report(), report(1.1)).regressions.empty(), "ten percent is within twenty percent threshold");
        RIGIDBODIES_EXPECT(compare_benchmarks(report(0.001), report(0.010)).regressions.empty(), "microseconds fall under absolute floor");
    }

    RIGIDBODIES_TEST("different machine compiler worker count and content are skipped explicitly")
    {
        auto different = report(2);
        different.metadata["cpu"] = "another cpu";
        RIGIDBODIES_EXPECT(!compare_benchmarks(report(), different).compatible, "CPU mismatch cannot regress");
        different = report(2);
        different.metadata["compiler"] = "another compiler";
        RIGIDBODIES_EXPECT(!compare_benchmarks(report(), different).compatible, "compiler mismatch cannot regress");
        different = report(2);
        different.settings.worker_count = 0;
        RIGIDBODIES_EXPECT(!compare_benchmarks(report(), different).compatible, "automatic and serial are separate baselines");
        different = report(2);
        different.results.front().content_fingerprint = "edited scene";
        const auto comparison = compare_benchmarks(report(), different);
        RIGIDBODIES_EXPECT(comparison.compatible && comparison.compared_workloads == 0 && comparison.regressions.empty() && !comparison.skipped.empty(), "edited content is an explicit skipped workload");
    }

    RIGIDBODIES_TEST("recording label time and phase availability do not prevent comparable step timing")
    {
        auto newer = report();
        newer.metadata["recorded_at"] = "later";
        newer.metadata["label"] = "new revision";
        for (auto& run : newer.results.front().runs)
            run.phases.clear();
        const auto comparison = compare_benchmarks(report(), newer);
        RIGIDBODIES_EXPECT(comparison.compatible && comparison.compared_workloads == 1, "labels and optional instrumentation payload are not machine fingerprints");
    }

    RIGIDBODIES_TEST("noisy baseline or candidate is inconclusive rather than a false regression")
    {
        auto noisy = report();
        noisy.results.front().runs.back().step = { 2, 2.2, 1.8, 2.4 };
        const auto baseline = compare_benchmarks(noisy, report(4));
        const auto candidate = compare_benchmarks(report(0.1), noisy);
        RIGIDBODIES_EXPECT(baseline.regressions.empty() && baseline.compared_workloads == 0 && baseline.skipped.size() == 2, "noisy baseline skipped");
        RIGIDBODIES_EXPECT(candidate.regressions.empty() && candidate.compared_workloads == 0 && candidate.skipped.size() == 2, "noisy candidate skipped");
    }

    RIGIDBODIES_TEST("fewer than three repetitions cannot claim a timing regression")
    {
        auto baseline = report(), candidate = report(2);
        baseline.settings.repetitions = candidate.settings.repetitions = 1;
        baseline.results.front().runs.resize(1);
        candidate.results.front().runs.resize(1);
        const auto comparison = compare_benchmarks(baseline, candidate);
        RIGIDBODIES_EXPECT(comparison.compatible && comparison.compared_workloads == 0 && !comparison.skipped.empty(), "smoke timing is not regression evidence");
    }

    RIGIDBODIES_TEST("invalid comparison thresholds are rejected")
    {
        BenchmarkComparisonSettings settings;
        settings.relative_threshold = std::numeric_limits<Real>::quiet_NaN();
        bool rejected = false;
        try
        {
            (void)compare_benchmarks(report(), report(), settings);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "nonfinite threshold cannot disable checking silently");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
