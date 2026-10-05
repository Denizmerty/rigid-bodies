#include "fuzz/document_fuzz.hpp"

#include <rigidbodies/physics/content_json.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using namespace rigidbodies;

    struct Seed
    {
        std::string name;
        std::string text;
    };

    std::vector<Seed> corpus()
    {
        std::vector<std::filesystem::path> paths;
        for (const auto& entry : std::filesystem::directory_iterator(RIGIDBODIES_FUZZ_CORPUS_DIR))
            if (entry.is_regular_file())
                paths.push_back(entry.path());
        std::sort(paths.begin(), paths.end());
        std::vector<Seed> result;
        for (const auto& path : paths)
        {
            std::ifstream stream(path, std::ios::binary);
            RIGIDBODIES_EXPECT(stream.good(), "Fuzz corpus fixture opens");
            result.push_back({ path.filename().string(), { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() } });
        }
        RIGIDBODIES_EXPECT(result.size() >= 15, "Tracked corpus includes structured and malformed seeds");
        return result;
    }

    // Fixed algorithm rather than standard-library distributions, so the same seed makes the
    // same byte stream on every supported compiler and operating system.
    class MutationRandom
    {
    public:
        std::size_t below(std::size_t bound)
        {
            state_ ^= state_ << 13;
            state_ ^= state_ >> 7;
            state_ ^= state_ << 17;
            return static_cast<std::size_t>(state_ % static_cast<std::uint64_t>(bound));
        }

    private:
        std::uint64_t state_ { 0x9365b4ef19a207cdULL };
    };

    std::string mutate(const std::string& source, MutationRandom& random)
    {
        static constexpr std::array<std::string_view, 13> tokens {
            "null", "true", "-1", "1e309", "-0.0", "2147483648", "4294967296", "\"\\ud800\"", "[]", "{}", ":", ",", "\"unknown\""
        };
        auto result = source;
        const auto position = random.below(result.size() + 1);
        const auto available = result.size() - position;
        const auto count = random.below(std::min<std::size_t>(available, 32) + 1);
        switch (random.below(7))
        {
        case 0:
            if (available != 0)
                result[position] = static_cast<char>(random.below(256));
            break;
        case 1:
            result.insert(position, tokens[random.below(tokens.size())]);
            break;
        case 2:
            result.erase(position, count);
            break;
        case 3:
            result.resize(position);
            break;
        case 4:
            result.replace(position, count, tokens[random.below(tokens.size())]);
            break;
        case 5:
            result.insert(position, source.substr(position, count));
            break;
        default:
            if (available != 0)
                result[position] = static_cast<char>(static_cast<unsigned char>(result[position]) ^ (1u << random.below(8)));
            break;
        }
        return result;
    }

    RIGIDBODIES_TEST("document fuzz corpus reaches valid shapes scenarios and transactional rejection")
    {
        std::size_t scenarios = 0, shapes = 0, invalid_json = 0, rejected_worlds = 0;
        for (const auto& seed : corpus())
        {
            const auto result = fuzz::exercise_document(seed.text);
            RIGIDBODIES_EXPECT(!result.semantic_budget_exceeded, "Every tracked corpus seed reaches semantic parsers when JSON is valid: " + seed.name);
            if (seed.name == "scenario-circle.json" || seed.name == "scenario-mechanism.json" || seed.name == "scenario-authored.json" || seed.name == "scenario-kinematic.json" || seed.name == "scenario-relativity.json")
                RIGIDBODIES_EXPECT(result.scenario_accepted, "Valid scenario seed reaches world population: " + seed.name);
            if (seed.name == "shape-square.json" || seed.name == "shape-concave.json" || seed.name == "shape-curved.json")
                RIGIDBODIES_EXPECT(result.shape_accepted, "Valid shape seed reaches authored geometry: " + seed.name);
            scenarios += result.scenario_accepted ? 1 : 0;
            shapes += result.shape_accepted ? 1 : 0;
            invalid_json += result.json_accepted ? 0 : 1;
            rejected_worlds += result.world_rejection_checked ? 1 : 0;
        }
        RIGIDBODIES_EXPECT(scenarios >= 4 && shapes >= 3, "Seeds cover real primitive/mechanism/authored/kinematic scenarios and convex/concave/cubic authored shapes");
        RIGIDBODIES_EXPECT(invalid_json >= 4 && rejected_worlds >= 5, "Malformed bytes and semantic document failures exercise rejection invariants");
    }

    RIGIDBODIES_TEST("deterministic seeded document mutations preserve parser and roundtrip invariants")
    {
        MutationRandom random;
        std::size_t attempts = 0, accepted = 0, rejected = 0;
        for (const auto& seed : corpus())
        {
            for (std::size_t iteration = 0; iteration < 96; ++iteration)
            {
                const auto result = fuzz::exercise_document(mutate(seed.text, random));
                ++attempts;
                accepted += result.shape_accepted || result.scenario_accepted ? 1 : 0;
                rejected += result.json_accepted ? 0 : 1;
            }
        }
        RIGIDBODIES_EXPECT(attempts >= 1440 && accepted > 0 && rejected > 0, "Mutation smoke exercises successful documents and malformed parser paths");
    }

    RIGIDBODIES_TEST("structured numeric and reference mutations reach semantic validation")
    {
        using physics::content::Json;
        std::size_t checked = 0, accepted = 0, rejected = 0;
        for (const auto& seed : corpus())
        {
            if (seed.name != "scenario-circle.json" && seed.name != "shape-square.json")
                continue;
            Json original;
            std::string error;
            RIGIDBODIES_EXPECT(physics::content::parse_json(seed.text, original, error), error);
            for (const double number : { -1.0e308, -1.0, -0.0, 1.0e-300, 0.5, 1.0, 1.0e9, 1.0e308 })
            {
                auto changed = original;
                if (seed.name == "scenario-circle.json")
                    changed["world"]["bodies"].as_array()[0]["parts"].as_array()[0]["shape"]["radius_m"] = number;
                else
                    changed["shape"]["outline"]["nodes"].as_array()[1]["position_m"].as_array()[0] = number;
                const auto result = fuzz::exercise_document(physics::content::write_json(changed));
                ++checked;
                const bool success = result.scenario_accepted || result.shape_accepted;
                accepted += success ? 1 : 0;
                rejected += success ? 0 : 1;
            }
            auto changed = original;
            changed["required_features"] = Json::Array { "unsupported-fuzz-feature" };
            const auto result = fuzz::exercise_document(physics::content::write_json(changed));
            RIGIDBODIES_EXPECT(!result.scenario_accepted && !result.shape_accepted && result.world_rejection_checked, "Unsupported required feature preserves destination");
        }
        RIGIDBODIES_EXPECT(checked == 16 && accepted > 0 && rejected > 0, "Boundary mutations cover both successful and rejected geometry");
    }

    RIGIDBODIES_TEST("structured relativity mutations accept only setups below c")
    {
        using physics::content::Json;
        std::size_t checked = 0;
        for (const auto& seed : corpus())
        {
            if (seed.name != "scenario-relativity.json")
                continue;
            Json original;
            std::string error;
            RIGIDBODIES_EXPECT(physics::content::parse_json(seed.text, original, error), error);
            const std::pair<double, bool> speeds[] = { { -1.0, false }, { -1.0e-300, false }, { -0.0, true }, { 0.0, true }, { 1.0e-300, false }, { 1.0e-13, false }, { 1.0e-12, true }, { 0.5, true }, { 0.9999999, true }, { 0.99999995, false }, { 1.0, false }, { 1.0e308, false } };
            for (const auto& [speed, valid] : speeds)
            {
                auto changed = original;
                changed["relativity"]["speed_fraction_c"] = speed;
                const auto result = fuzz::exercise_document(physics::content::write_json(changed));
                RIGIDBODIES_EXPECT(result.scenario_accepted == valid, "only speeds from 0 c to the maximum are accepted");
                RIGIDBODIES_EXPECT(valid || result.world_rejection_checked, "a rejected setup preserves the destination");
                ++checked;
            }
            for (const double mass : { 0.0, 1.0e-7, 2.0e6, -1.0 })
            {
                auto changed = original;
                changed["relativity"]["rest_mass_kg"] = mass;
                const auto result = fuzz::exercise_document(physics::content::write_json(changed));
                RIGIDBODIES_EXPECT(!result.scenario_accepted && result.world_rejection_checked, "masses outside the relativity bounds are rejected");
                ++checked;
            }
            auto missing = original;
            missing.as_object().erase("relativity");
            RIGIDBODIES_EXPECT(!fuzz::exercise_document(physics::content::write_json(missing)).scenario_accepted, "the declared feature requires its object");
            auto undeclared = original;
            undeclared.as_object().erase("required_features");
            undeclared["relativity"]["speed_fraction_c"] = 1.0;
            RIGIDBODIES_EXPECT(fuzz::exercise_document(physics::content::write_json(undeclared)).scenario_accepted, "without the feature the object is an ignored extension");
        }
        RIGIDBODIES_EXPECT(checked == 16, "the relativity seed is in the corpus");
    }

    RIGIDBODIES_TEST("fuzz workload limits bound semantic expansion while preserving JSON coverage")
    {
        using physics::content::Json;
        const auto oversized = fuzz::exercise_document(std::string(fuzz::maximum_input_bytes + 1, ' '));
        RIGIDBODIES_EXPECT(!oversized.json_accepted && !oversized.scenario_accepted && !oversized.shape_accepted, "Oversized fuzzer buffers return immediately");
        const auto array = fuzz::exercise_document(physics::content::write_json(Json::Array(25, nullptr)));
        RIGIDBODIES_EXPECT(array.json_accepted && array.semantic_budget_exceeded, "Large arrays still exercise JSON but not unbounded semantic allocation");
        const auto tessellation = fuzz::exercise_document(R"({"options":{"max_render_vertices":4096}})");
        RIGIDBODIES_EXPECT(tessellation.json_accepted && tessellation.semantic_budget_exceeded, "Large tessellation requests do not consume fuzz iteration budgets");
    }

    RIGIDBODIES_TEST("raw invalid UTF8 and embedded NUL reach all public document rejection paths")
    {
        for (const auto& text : { std::string("\"\xc0\xaf\""), std::string("\"\xed\xa0\x80\""), std::string("\"\xf4\x90\x80\x80\""), std::string("null\0false", 10) })
        {
            const auto result = fuzz::exercise_document(text);
            RIGIDBODIES_EXPECT(!result.json_accepted && !result.shape_accepted && !result.scenario_accepted, "Invalid UTF8 and trailing bytes fail transactionally");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
