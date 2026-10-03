#include <rigidbodies/physics/benchmark.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/project_identity.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace
{
    using namespace rigidbodies::physics;

    std::string environment(const char* name, std::string fallback)
    {
        const auto* value = std::getenv(name);
        return value && *value ? value : std::move(fallback);
    }

    std::string utc_now()
    {
        const auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        const auto* utc = std::gmtime(&time);
        std::ostringstream result;
        result << std::put_time(utc, "%Y-%m-%dT%H:%M:%SZ");
        return result.str();
    }

    std::map<std::string, std::string, std::less<>> metadata()
    {
        std::map<std::string, std::string, std::less<>> fields;
#if defined(_WIN32)
        fields["platform"] = "Windows";
#elif defined(__APPLE__)
        fields["platform"] = "macOS";
#elif defined(__linux__)
        fields["platform"] = "Linux";
#else
        fields["platform"] = "Other";
#endif
#if defined(_M_X64) || defined(__x86_64__)
        fields["architecture"] = "x86_64";
#elif defined(_M_ARM64) || defined(__aarch64__)
        fields["architecture"] = "aarch64";
#else
        fields["architecture"] = "other";
#endif
#if defined(__clang__)
        fields["compiler"] = "Clang " __clang_version__;
#elif defined(_MSC_FULL_VER)
        fields["compiler"] = "MSVC " + std::to_string(_MSC_FULL_VER);
#elif defined(__GNUC__)
        fields["compiler"] = "GCC " __VERSION__;
#else
        fields["compiler"] = "other";
#endif
#ifdef RIGIDBODIES_BENCHMARK_BUILD_CONFIGURATION
        fields["build"] = RIGIDBODIES_BENCHMARK_BUILD_CONFIGURATION;
#elif defined(NDEBUG)
        fields["build"] = "Release";
#else
        fields["build"] = "Debug";
#endif
        fields["machine"] = environment("COMPUTERNAME", environment("HOSTNAME", "unnamed-machine"));
        fields["cpu"] = environment("PROCESSOR_IDENTIFIER", "unspecified-use-cpu-option");
        fields["hardware_threads"] = std::to_string(std::thread::hardware_concurrency());
        fields["project_version"] = rigidbodies::project_version;
        fields["recorded_at"] = utc_now();
        fields["label"] = "local";
        return fields;
    }

    Real number(const std::string& text)
    {
        std::size_t consumed = 0;
        const auto value = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(value) || value < 0)
            throw std::invalid_argument("Expected a finite nonnegative number: " + text);
        return value;
    }

    std::size_t count(const std::string& text)
    {
        const auto value = number(text);
        if (value > 4294967295.0 || value != std::floor(value))
            throw std::invalid_argument("Expected a bounded unsigned integer: " + text);
        return static_cast<std::size_t>(value);
    }

    BenchmarkReport read_report(const std::filesystem::path& path)
    {
        if (std::filesystem::file_size(path) > 16 * 1024 * 1024)
            throw std::runtime_error("Benchmark report exceeds 16 MiB");
        std::ifstream input(path, std::ios::binary);
        if (!input)
            throw std::runtime_error("Could not open benchmark report: " + path.string());
        const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        BenchmarkReport report;
        std::string error;
        if (!parse_benchmark_report(text, report, error))
            throw std::runtime_error(path.string() + ": " + error);
        return report;
    }

    void help()
    {
        std::cout << "Usage: rigid_bodies_benchmark [options]\n"
                     "Runs reproducible workloads without a window and writes versioned JSON.\n"
                     "  --list                       List workload IDs\n"
                     "  --workload ID                Select one workload (repeatable; default all)\n"
                     "  --warmup N --samples N --repetitions N    Defaults 30, 120, 3\n"
                     "  --step SECONDS --seed N      Defaults 1/120, 17329\n"
                     "  --workers N                  1 serial (default), 0 automatic, or explicit count\n"
                     "  --no-profiling               Disable per-phase timing instrumentation\n"
                     "  --scenarios DIRECTORY        Use a specific scenario catalogue\n"
                     "  --machine NAME --cpu NAME --label TEXT   Machine, CPU and report labels\n"
                     "  --output FILE                Write JSON to file instead of stdout\n"
                     "  --baseline FILE              Compare matching workloads with this report\n"
                     "  --compare FILE               Compare an existing report without running\n"
                     "  --relative-threshold FRACTION --absolute-threshold-ms MS\n"
                     "  --noise-multiplier N --maximum-relative-spread FRACTION\n"
                     "Exit 0: measurement succeeded or no regression was confirmed; some comparisons may be skipped.\n"
                     "Exit 1: invalid input or failed measurement. Exit 2: confirmed timing regression.\n";
    }
}

int main(int argc, char** argv)
{
    try
    {
        BenchmarkReport report;
        report.metadata = metadata();
        BenchmarkComparisonSettings comparison_settings;
        std::vector<std::string> workloads;
        std::filesystem::path output, baseline, candidate, scenarios;
        for (int index = 1; index < argc; ++index)
        {
            const std::string option = argv[index];
            if (option == "--help")
            {
                help();
                return 0;
            }
            if (option == "--list")
            {
                for (const auto& workload : benchmark_workloads())
                    std::cout << workload.id << '\t' << workload.description << '\n';
                return 0;
            }
            if (option == "--no-profiling")
            {
                report.settings.profiling = false;
                continue;
            }
            if (index + 1 == argc)
                throw std::invalid_argument("Missing value for " + option);
            const std::string value = argv[++index];
            if (option == "--workload")
                workloads.push_back(value);
            else if (option == "--warmup")
                report.settings.warmup_steps = count(value);
            else if (option == "--samples")
                report.settings.sample_steps = count(value);
            else if (option == "--repetitions")
                report.settings.repetitions = count(value);
            else if (option == "--step")
                report.settings.time_step_s = number(value);
            else if (option == "--seed")
                report.settings.seed = static_cast<std::uint32_t>(count(value));
            else if (option == "--workers")
                report.settings.worker_count = count(value);
            else if (option == "--output")
                output = value;
            else if (option == "--baseline")
                baseline = value;
            else if (option == "--compare")
                candidate = value;
            else if (option == "--scenarios")
                scenarios = value;
            else if (option == "--machine")
                report.metadata["machine"] = value;
            else if (option == "--cpu")
                report.metadata["cpu"] = value;
            else if (option == "--label")
                report.metadata["label"] = value;
            else if (option == "--relative-threshold")
                comparison_settings.relative_threshold = number(value);
            else if (option == "--absolute-threshold-ms")
                comparison_settings.absolute_threshold_ms = number(value);
            else if (option == "--noise-multiplier")
                comparison_settings.noise_multiplier = number(value);
            else if (option == "--maximum-relative-spread")
                comparison_settings.maximum_relative_spread = number(value);
            else
                throw std::invalid_argument("Unknown benchmark option: " + option);
        }
        if (!candidate.empty())
        {
            if (baseline.empty())
                throw std::invalid_argument("--compare requires --baseline");
            report = read_report(candidate);
        }
        else
        {
#ifdef RIGIDBODIES_BENCHMARK_BASELINE
            if (report.settings.worker_count != 1 || report.settings.profiling)
                throw std::invalid_argument("The reference mode requires --workers 1 --no-profiling");
#endif
            if (scenarios.empty())
            {
                const auto staged = std::filesystem::absolute(argv[0]).parent_path() / "assets" / "scenarios";
                if (std::filesystem::is_directory(staged))
                    scenarios = staged;
            }
            if (!scenarios.empty())
            {
                std::string error;
                if (!initialize_scenario_catalogue(scenarios, error))
                    throw std::runtime_error(error);
            }
            if (workloads.empty())
                for (const auto& workload : benchmark_workloads())
                    workloads.push_back(workload.id);
            std::set<std::string> unique;
            for (const auto& workload : workloads)
            {
                if (!unique.insert(workload).second)
                    throw std::invalid_argument("Duplicate workload: " + workload);
                std::cerr << "Measuring " << workload << "...\n";
                report.results.push_back(measure_benchmark(workload, report.settings));
            }
        }
        const auto serialized = write_benchmark_report(report);
        if (!output.empty())
        {
            std::ofstream file(output, std::ios::binary | std::ios::trunc);
            file << serialized;
            file.close();
            if (!file)
                throw std::runtime_error("Could not write benchmark report: " + output.string());
        }
        else if (candidate.empty())
            std::cout << serialized;
        if (!baseline.empty())
        {
            const auto comparison = compare_benchmarks(read_report(baseline), report, comparison_settings);
            std::cerr << "Compared " << comparison.compared_workloads << " workloads; " << comparison.regressions.size() << " confirmed regressions.\n";
            for (const auto& reason : comparison.skipped)
                std::cerr << "SKIP " << reason << '\n';
            for (const auto& regression : comparison.regressions)
                std::cerr << "REGRESSION " << regression << '\n';
            return comparison.regressions.empty() ? 0 : 2;
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
