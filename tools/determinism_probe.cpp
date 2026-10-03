#include "determinism_workload.hpp"

#include <fstream>
#include <iostream>

namespace
{
    using namespace rigidbodies::testing::determinism;

    Json metadata(std::size_t workers, bool verified, const Evidence& evidence)
    {
        std::string platform, compiler;
#if defined(_WIN32)
        platform = "Windows";
#elif defined(__APPLE__)
        platform = "macOS";
#elif defined(__linux__)
        platform = "Linux";
#else
        platform = "Other";
#endif
#if defined(__clang__)
        compiler = "Clang " __clang_version__;
#elif defined(_MSC_VER)
        compiler = "MSVC " + std::to_string(_MSC_VER);
#elif defined(__GNUC__)
        compiler = "GCC " __VERSION__;
#else
        compiler = "Other";
#endif
        return Json::Object { { "platform", platform }, { "compiler", compiler }, { "requested_workers", workers }, { "same_build_repeat_and_workers_verified", verified }, { "broad_workers_used", evidence.broad_workers }, { "narrow_workers_used", evidence.narrow_workers }, { "contacts_observed", evidence.contacts }, { "speculative_observed", evidence.speculative }, { "contact_events_observed", evidence.events }, { "snapshot_replays_verified", evidence.snapshot_replays } };
    }
}

int main(int argc, char** argv)
{
    try
    {
        std::string output;
        std::size_t workers = 1;
        bool verify = false;
        for (int index = 1; index < argc; ++index)
        {
            const std::string option = argv[index];
            if (option == "--output" && index + 1 < argc)
                output = argv[++index];
            else if (option == "--workers" && index + 1 < argc)
            {
                const std::string value = argv[++index];
                if (value != "1" && value != "2" && value != "4")
                    throw std::invalid_argument("--workers requires 1, 2, or 4");
                workers = static_cast<std::size_t>(std::stoul(value));
            }
            else if (option == "--verify")
                verify = true;
            else if (option == "--help")
            {
                std::cout << "Usage: rigid_bodies_determinism_probe --output trace.json [--workers 1|2|4] [--verify]\n";
                return 0;
            }
            else
                throw std::invalid_argument("unknown or incomplete option: " + option);
        }
        if (output.empty())
            throw std::invalid_argument("--output is required");
        Evidence evidence;
        auto result = trace(workers, evidence);
        if (verify)
        {
            const auto canonical = content::write_json(result);
            for (const auto count : { workers, std::size_t { 1 }, std::size_t { 2 }, std::size_t { 4 } })
            {
                Evidence repeated;
                if (canonical != content::write_json(trace(count, repeated)))
                    throw std::runtime_error("same-build exact trace changed with repetition or worker count " + std::to_string(count));
                if (count > 1 && (repeated.broad_workers < count || repeated.narrow_workers < count))
                    throw std::runtime_error("probe failed to exercise requested collision workers");
            }
        }
        if (evidence.contacts == 0 || evidence.speculative == 0 || evidence.events == 0 || evidence.snapshot_replays != 5)
            throw std::runtime_error("determinism workload did not exercise its required paths");
        result["metadata"] = metadata(workers, verify, evidence);
        std::ofstream file(output, std::ios::binary | std::ios::trunc);
        if (!file)
            throw std::runtime_error("cannot open output: " + output);
        file << content::write_json(result) << '\n';
        file.close();
        if (!file)
            throw std::runtime_error("failed writing output: " + output);
        std::cout << "Wrote five deterministic workloads to " << output << "; " << evidence.snapshot_replays
                  << " exact snapshot replays verified" << (verify ? "; repeat and 1/2/4 workers verified" : "") << ".\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Determinism probe: " << error.what() << '\n';
        return 1;
    }
}
