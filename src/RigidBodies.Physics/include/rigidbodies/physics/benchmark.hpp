#pragma once

#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/world.hpp>

#include <map>
#include <string>
#include <vector>

namespace rigidbodies::physics
{
    struct BenchmarkSettings
    {
        std::size_t warmup_steps { 30 };
        std::size_t sample_steps { 120 };
        std::size_t repetitions { 3 };
        Real time_step_s { 1.0 / 120.0 };
        std::uint32_t seed { 17329 };
        std::size_t worker_count { 1 };
        bool profiling { true };
    };

    struct BenchmarkWorkload
    {
        std::string id;
        std::string description;
        std::string scenario_id;
    };

    struct BenchmarkTiming
    {
        Real median_ms {}, p95_ms {}, minimum_ms {}, maximum_ms {};
    };

    struct BenchmarkRun
    {
        BenchmarkTiming step;
        std::map<std::string, BenchmarkTiming, std::less<>> phases;
        std::size_t bodies {}, maximum_pairs {}, maximum_contacts {}, maximum_constraints {}, sleeping_bodies {};
        // Zero means this runtime evidence is unavailable.
        std::size_t maximum_islands {}, sleeping_islands {}, broad_phase_workers {}, narrow_phase_workers {};
        // A stable digest of the final body poses/velocities, independent of wall-clock timing.
        std::string state_digest;
    };

    struct BenchmarkResult
    {
        std::string workload;
        std::string content_fingerprint;
        std::vector<BenchmarkRun> runs;
    };

    struct BenchmarkReport
    {
        BenchmarkSettings settings;
        // Metadata is provided by the executable/runner, never discovered through Git.
        std::map<std::string, std::string, std::less<>> metadata;
        std::vector<BenchmarkResult> results;
    };

    struct BenchmarkComparisonSettings
    {
        Real relative_threshold { 0.20 };
        Real absolute_threshold_ms { 0.05 };
        Real noise_multiplier { 3.0 };
        Real maximum_relative_spread { 0.25 };
    };

    struct BenchmarkComparison
    {
        bool compatible { false };
        std::size_t compared_workloads {};
        std::vector<std::string> regressions;
        std::vector<std::string> skipped;
    };

    [[nodiscard]] const std::vector<BenchmarkWorkload>& benchmark_workloads();
    // Creates a deterministic initial state. Throws before returning on an unknown workload.
    void populate_benchmark_world(World& world, std::string_view workload, std::uint32_t seed);
    [[nodiscard]] BenchmarkTiming summarize_benchmark_samples(std::vector<Real> samples_ms);
    [[nodiscard]] BenchmarkResult measure_benchmark(std::string_view workload, const BenchmarkSettings& settings = {});
    [[nodiscard]] std::string write_benchmark_report(const BenchmarkReport& report);
    // Strict bounded validation; malformed or incomplete results leave the target unchanged.
    bool parse_benchmark_report(std::string_view text, BenchmarkReport& report, std::string& error);
    // Only matching machine/build/config/content is compared. No unstable or mismatched run is
    // called a regression. Both median and p95 require slowdown across every repetition.
    [[nodiscard]] BenchmarkComparison compare_benchmarks(const BenchmarkReport& baseline,
        const BenchmarkReport& candidate, const BenchmarkComparisonSettings& settings = {});
}
