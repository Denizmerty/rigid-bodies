#include "determinism_workload.hpp"
#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies::testing::determinism;

    RIGIDBODIES_TEST("complete deterministic trace repeats exactly and restores each workload checkpoint")
    {
        Evidence first, second;
        const auto reference = content::write_json(trace(1, first));
        RIGIDBODIES_EXPECT(reference == content::write_json(trace(1, second)), "every identity, event, value and ordering repeats exactly");
        RIGIDBODIES_EXPECT(first.snapshot_replays == 5 && second.snapshot_replays == 5, "both exact and general workloads replay snapshots");
        RIGIDBODIES_EXPECT(first.contacts > 0 && first.events > 0 && first.speculative > 0, "contacts, sensors and CCD paths execute");
    }

    RIGIDBODIES_TEST("complete deterministic trace is exact across one two and four collision workers")
    {
        Evidence serial;
        const auto reference = content::write_json(trace(1, serial));
        for (const auto workers : { std::size_t { 2 }, std::size_t { 4 } })
        {
            Evidence parallel;
            RIGIDBODIES_EXPECT(reference == content::write_json(trace(workers, parallel)), "canonical physical output is independent of scheduling");
            RIGIDBODIES_EXPECT(parallel.broad_workers == workers && parallel.narrow_workers == workers,
                "both collision phases actually use the requested workers");
        }
    }

    RIGIDBODIES_TEST("trace schema distinguishes exact portable arithmetic from bounded general mechanics")
    {
        Evidence evidence;
        const auto result = trace(1, evidence);
        const auto& scenarios = result.at("scenarios").as_array();
        RIGIDBODIES_EXPECT(scenarios.size() == 5, "all five versioned workloads are present");
        RIGIDBODIES_EXPECT(scenarios[0].at("numeric_contract").as_string() == "exact" &&
                scenarios[1].at("numeric_contract").as_string() == "exact",
            "dyadic impact and storage workloads require exact portable values");
        for (const auto& scenario : scenarios)
            RIGIDBODIES_EXPECT(scenario.at("records").as_array().size() == static_cast<std::size_t>(scenario.at("steps").as_number()),
                "every step is recorded, so transient event differences are visible");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
