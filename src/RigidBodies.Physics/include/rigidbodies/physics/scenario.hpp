#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/physics/world.hpp>

#include <string>
#include <string_view>
#include <filesystem>
#include <vector>

namespace rigidbodies::physics
{

    struct ScenarioDocument;

    // A named starting arrangement. Scenarios are how the playground introduces a topic: each one
    // sets up a small, recognisable situation and states what is worth watching in it.
    //
    // The catalogue reads versioned scenario documents from a directory, with no reference to
    // rendering or to the interface, so that each arrangement also runs in headless tools.
    struct ScenarioDescription
    {
        std::string id;
        std::string title;

        // One or two sentences shown alongside the scenario, describing the situation and the
        // quantity it is meant to make visible.
        std::string summary;
        std::vector<std::string> concepts;
        std::vector<std::string> prerequisites;
        std::vector<std::string> tags;
        std::string collection;
        std::string level;
        std::string hook;
        int collection_order { 0 };
        int suggested_order { 0 };
        bool lab { false };
    };

    // Validates every document and its prerequisite graph before replacing the live catalogue.
    // On failure the previous catalogue remains usable. Call before building any UI that holds
    // description references; successful replacement invalidates all previous views/pointers.
    bool initialize_scenario_catalogue(const std::filesystem::path& directory, std::string& error);

    [[nodiscard]] const std::filesystem::path& scenario_catalogue_directory();
    [[nodiscard]] const std::string& scenario_catalogue_error();

    [[nodiscard]] math::Span<const ScenarioDescription> available_scenarios();

    [[nodiscard]] const ScenarioDescription* find_scenario(std::string_view id);

    // The immutable source retains additive document fields when an arrangement is edited/saved.
    // Like descriptions, its lifetime extends until the next successful catalogue replacement.
    [[nodiscard]] const ScenarioDocument* scenario_document_for_id(std::string_view id);

    // Replaces the contents of the world with the named scenario. Returns false and leaves the
    // world untouched when the identifier is not known.
    bool load_scenario(World& world, std::string_view id);

    // Identifier of the arrangement the application opens with.
    [[nodiscard]] std::string_view default_scenario_id();

} // namespace rigidbodies::physics
