#pragma once

#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/world.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{
    struct ScenarioMetadata
    {
        std::string id;
        std::string title;
        std::string summary;
        std::vector<std::string> concepts;
        std::vector<std::string> prerequisites;
        std::vector<std::string> tags;
        std::string collection;
        std::string level;
        std::string hook;
        int collection_order {};
        int suggested_order {};
        bool lab { false };
    };

    // The source tree retains unknown additive fields for a lossless read/write cycle. A capture
    // can inherit that source while replacing the fields describing the edited arrangement.
    struct ScenarioDocument
    {
        content::Json root;
        ScenarioMetadata metadata;
    };

    bool parse_scenario_document(std::string_view text, ScenarioDocument& document, std::string& error);
    bool write_scenario_document(const ScenarioDocument& document, std::string& text, std::string& error);
    bool capture_scenario_document(const World& world, const ScenarioMetadata& metadata,
        ScenarioDocument& document, std::string& error, const ScenarioDocument* source = nullptr);

    // Loading is transactional. Documents store arrangements, not solver caches or time counters;
    // prescribed paths retain their current phase so resuming a saved scene does not jump.
    bool populate_world(const ScenarioDocument& document, World& world, std::string& error);
}
