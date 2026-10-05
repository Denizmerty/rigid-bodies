#include <rigidbodies/physics/relativity_document.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/scenario_document.hpp>

#include "test_framework.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

namespace
{
    using namespace rigidbodies::physics;
    using content::Json;

    class CatalogueFixture
    {
    public:
        CatalogueFixture() : original_(scenario_catalogue_directory())
        {
            const auto* source = scenario_document_for_id("free_fall");
            RIGIDBODIES_EXPECT(source != nullptr, "bundled free-fall document exists");
            prototype_ = source->root;
            static unsigned sequence = 0;
            const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
            directory = std::filesystem::current_path() / ("scenario-catalogue-fixture-" + std::to_string(stamp) + "-" + std::to_string(++sequence));
            if (!std::filesystem::create_directory(directory))
                throw std::runtime_error("cannot create unique catalogue fixture");
        }

        ~CatalogueFixture()
        {
            std::string error;
            initialize_scenario_catalogue(original_, error);
            std::error_code ignored;
            for (const auto& path : written_)
                std::filesystem::remove(path, ignored);
            std::filesystem::remove(directory, ignored);
        }

        Json document(std::string id, int order = 10)
        {
            auto value = prototype_;
            value["metadata"]["id"] = std::move(id);
            value["metadata"]["title"] = "Scenario read from disk";
            value["metadata"]["suggested_order"] = order;
            value["metadata"]["prerequisites"] = Json::Array {};
            return value;
        }

        void write(const std::string& filename, const Json& document)
        {
            write_text(filename, content::write_json(document));
        }

        void write_text(const std::string& filename, const std::string& text)
        {
            const auto path = directory / filename;
            std::ofstream output(path, std::ios::binary);
            output << text;
            if (!output)
                throw std::runtime_error("cannot write catalogue fixture");
            written_.push_back(path);
        }

        std::filesystem::path directory;

    private:
        std::filesystem::path original_;
        std::vector<std::filesystem::path> written_;
        Json prototype_;
    };

    RIGIDBODIES_TEST("all disk scenarios expose teaching metadata in suggested order")
    {
        const auto scenarios = available_scenarios();
        RIGIDBODIES_EXPECT(!scenarios.empty(), scenario_catalogue_error());
        RIGIDBODIES_EXPECT(default_scenario_id() == "free_fall", "first suggested arrangement opens by default");
        int last_order = -1;
        for (const auto& scenario : scenarios)
        {
            RIGIDBODIES_EXPECT(!scenario.title.empty() && !scenario.summary.empty() && !scenario.concepts.empty(), "teaching metadata is present");
            RIGIDBODIES_EXPECT(scenario.suggested_order > last_order, "bundled suggested order is explicit and increasing");
            last_order = scenario.suggested_order;
            for (const auto& prerequisite : scenario.prerequisites)
                RIGIDBODIES_EXPECT(find_scenario(prerequisite) != nullptr, "prerequisites resolve to scenarios");
            const auto* document = scenario_document_for_id(scenario.id);
            RIGIDBODIES_EXPECT(document && document->metadata.id == scenario.id, "description and immutable source agree");
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, scenario.id), "every disk document populates through the public interface");
            RIGIDBODIES_EXPECT(!world.body_ids().empty(), "each teaching arrangement has objects");
        }
    }

    RIGIDBODIES_TEST("Chasing light joins the catalogue with a valid relativity setup")
    {
        const auto* description = find_scenario("chasing_light");
        RIGIDBODIES_EXPECT(description != nullptr, scenario_catalogue_error());
        RIGIDBODIES_EXPECT(description->title == "Chasing light" && description->collection == "special_relativity" && description->level == "further", "it is the special relativity collection's experiment");
        RIGIDBODIES_EXPECT(description->suggested_order == 250 && description->collection_order == 1, "it is ordered after its prerequisite and before Make your own");
        RIGIDBODIES_EXPECT(description->prerequisites.size() == 1 && description->prerequisites.front() == "collision_comparison", "it builds on Three kinds of collision");
        RIGIDBODIES_EXPECT(find_scenario("shape_workshop")->suggested_order == 260 && find_scenario("empty_lab")->suggested_order == 270, "Make your own follows it");
        const auto* document = scenario_document_for_id("chasing_light");
        RIGIDBODIES_EXPECT(document && declares_special_relativity(document->root), "the document requires special_relativity");
        RIGIDBODIES_EXPECT(document && read_relativity_setup(document->root) == RelativitySetup { 1.0, 0.0 }, "the probe is 1 kg at rest");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "chasing_light"), "its world loads");
        const auto ids = world.body_ids();
        RIGIDBODIES_EXPECT(ids.size() == 1 && world.find_body(ids.front())->type() == BodyType::static_body && world.force_generators().empty(), "the world is the fixed rail alone, with no gravity");
        const auto bounds = world.find_body(ids.front())->compute_bounds();
        RIGIDBODIES_EXPECT_NEAR(bounds.maximum.y, 0.0, 1.0e-12, "the rail's top surface is the track line y = 0");
        RIGIDBODIES_EXPECT(bounds.minimum.x < 0.0 && bounds.maximum.x > relativity_track_length_m, "the rail spans the whole track");
        // Capturing the loaded document with itself as the source keeps every field.
        ScenarioDocument captured;
        std::string error;
        RIGIDBODIES_EXPECT(capture_scenario_document(world, document->metadata, captured, error, document), error);
        RIGIDBODIES_EXPECT(content::write_json(captured.root) == content::write_json(document->root), "a capture of the bundled document changes nothing");
    }

    RIGIDBODIES_TEST("catalogue reads edited descriptions and sorts by suggested order then identifier")
    {
        CatalogueFixture fixture;
        fixture.write("a.JSON", fixture.document("last", 30));
        fixture.write("z.json", fixture.document("first", 10));
        fixture.write("m.json", fixture.document("middle", 10));
        fixture.write_text("README.md", "Not a scenario document.");
        std::string error;
        RIGIDBODIES_EXPECT(initialize_scenario_catalogue(fixture.directory, error), error);
        const auto scenarios = available_scenarios();
        RIGIDBODIES_EXPECT(scenarios.size() == 3, "only JSON documents are catalogued");
        RIGIDBODIES_EXPECT(scenarios[0].id == "first" && scenarios[1].id == "middle" && scenarios[2].id == "last", "order does not depend on directory iteration or filenames");
        RIGIDBODIES_EXPECT(scenarios[0].title == "Scenario read from disk", "metadata comes from the document");
        RIGIDBODIES_EXPECT(default_scenario_id() == "first", "default follows the supplied catalogue");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "middle") && !world.body_ids().empty(), "a newly discovered identifier populates a world");
    }

    RIGIDBODIES_TEST("invalid catalogue replacement preserves existing references and arrangements")
    {
        CatalogueFixture fixture;
        const auto* before = find_scenario("free_fall");
        const auto* source = scenario_document_for_id("free_fall");
        // The snapshot must not follow changes to the catalogue's stored directory.
        // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
        const auto before_directory = scenario_catalogue_directory();
        fixture.write("good.json", fixture.document("new_scenario"));
        fixture.write_text("bad.json", "{ this is not valid JSON }");
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error) && !error.empty(), "malformed documents reject the whole replacement");
        RIGIDBODIES_EXPECT(find_scenario("free_fall") == before && scenario_document_for_id("free_fall") == source, "failed replacement preserves description and document lifetimes");
        RIGIDBODIES_EXPECT(scenario_catalogue_directory() == before_directory && find_scenario("empty_lab") != nullptr, "previous directory and complete catalogue remain active");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "free_fall"), "previous scenarios remain loadable");
        RIGIDBODIES_EXPECT(find_scenario("new_scenario") == nullptr, "no partial new entries leak into the catalogue");
    }

    RIGIDBODIES_TEST("catalogue rejects duplicate identifiers across different files")
    {
        CatalogueFixture fixture;
        fixture.write("one.json", fixture.document("same"));
        fixture.write("two.json", fixture.document("same"));
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "duplicate IDs are ambiguous");
        RIGIDBODIES_EXPECT(error.find("duplicate scenario identifier") != std::string::npos, "diagnostic identifies the duplicate");
    }

    RIGIDBODIES_TEST("catalogue requires valid identifiers and useful teaching metadata")
    {
        CatalogueFixture fixture;
        auto document = fixture.document("bad/id");
        fixture.write("invalid.json", document);
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "IDs cannot contain path separators");
        document = fixture.document("valid");
        document["metadata"]["concepts"] = Json::Array {};
        fixture.write("invalid.json", document);
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "concepts cannot be empty");
        document["metadata"]["concepts"] = Json::Array { "gravity", "gravity" };
        fixture.write("invalid.json", document);
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "duplicate concepts are rejected");
        RIGIDBODIES_EXPECT(find_scenario("free_fall") != nullptr, "invalid teaching metadata preserves the old catalogue");
    }

    RIGIDBODIES_TEST("catalogue rejects missing prerequisite references")
    {
        CatalogueFixture fixture;
        auto document = fixture.document("first");
        document["metadata"]["prerequisites"] = Json::Array { "missing" };
        fixture.write("first.json", document);
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "missing prerequisites are rejected");
        RIGIDBODIES_EXPECT(error.find("unknown prerequisite") != std::string::npos, "diagnostic identifies a missing prerequisite");
    }

    RIGIDBODIES_TEST("catalogue rejects cyclic prerequisite graphs")
    {
        CatalogueFixture fixture;
        auto first = fixture.document("first");
        auto second = fixture.document("second", 20);
        first["metadata"]["prerequisites"] = Json::Array { "second" };
        second["metadata"]["prerequisites"] = Json::Array { "first" };
        fixture.write("first.json", first);
        fixture.write("second.json", second);
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "cycles have no valid learning sequence");
        RIGIDBODIES_EXPECT(error.find("cyclic scenario prerequisites") != std::string::npos, "diagnostic identifies the cycle");
    }

    RIGIDBODIES_TEST("catalogue accepts additive future fields and keeps the source document")
    {
        CatalogueFixture fixture;
        auto document = fixture.document("future");
        document["version"]["minor"] = 17;
        document["future_lesson_extension"] = Json::Object { { "caption", "Retain this for newer applications" } };
        fixture.write("future.json", document);
        std::string error;
        RIGIDBODIES_EXPECT(initialize_scenario_catalogue(fixture.directory, error), error);
        const auto* source = scenario_document_for_id("future");
        RIGIDBODIES_EXPECT(source && source->root.at("future_lesson_extension").at("caption").as_string() == "Retain this for newer applications", "unrecognized additive fields survive discovery");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "future"), "compatible minor versions remain usable");
    }

    RIGIDBODIES_TEST("a relativity document with a valid setup joins the catalogue")
    {
        CatalogueFixture fixture;
        auto document = fixture.document("relativity_fixture");
        document["required_features"] = Json::Array { "special_relativity" };
        document["relativity"] = Json::Object { { "rest_mass_kg", 1.0 }, { "speed_fraction_c", 0.0 } };
        fixture.write("relativity.json", document);
        std::string error;
        RIGIDBODIES_EXPECT(initialize_scenario_catalogue(fixture.directory, error), error);
        const auto* source = scenario_document_for_id("relativity_fixture");
        RIGIDBODIES_EXPECT(source && read_relativity_setup(source->root) == RelativitySetup { 1.0, 0.0 }, "the catalogued document keeps its setup");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "relativity_fixture") && !world.body_ids().empty(), "its ordinary world still loads");
    }

    RIGIDBODIES_TEST("a malformed relativity document rejects the whole catalogue")
    {
        CatalogueFixture fixture;
        const auto* before = find_scenario("free_fall");
        fixture.write("good.json", fixture.document("good"));
        const Json malformed[] = {
            Json::Object { { "rest_mass_kg", 1.0 }, { "speed_fraction_c", 1.0 } },
            Json::Object { { "rest_mass_kg", 1.0 }, { "speed_fraction_c", "0.5" } },
            Json::Object { { "rest_mass_kg", 0.0 }, { "speed_fraction_c", 0.5 } },
            Json::Object { { "rest_mass_kg", 1.0 } },
            Json(nullptr),
        };
        for (const auto& relativity : malformed)
        {
            auto document = fixture.document("relativity_fixture", 20);
            document["required_features"] = Json::Array { "special_relativity" };
            document["relativity"] = relativity;
            fixture.write("relativity.json", document);
            std::string error;
            RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "one malformed relativity document rejects the replacement");
            RIGIDBODIES_EXPECT(error.find("relativity.json: relativity") == 0 || error.find("relativity.json: A document that requires special_relativity") == 0, "the diagnostic names the file and the relativity field: " + error);
            RIGIDBODIES_EXPECT(find_scenario("free_fall") == before && find_scenario("good") == nullptr && find_scenario("relativity_fixture") == nullptr, "the live catalogue is unchanged");
        }
    }

    RIGIDBODIES_TEST("unsupported major versions cannot replace the live catalogue")
    {
        CatalogueFixture fixture;
        auto document = fixture.document("future");
        document["version"]["major"] = 2;
        fixture.write("future.json", document);
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error) && !error.empty(), "incompatible versions fail with an explanation");
        RIGIDBODIES_EXPECT(find_scenario("free_fall") != nullptr, "existing arrangements remain available");
    }

    RIGIDBODIES_TEST("empty or missing catalogue directories leave the current selection usable")
    {
        CatalogueFixture fixture;
        std::string error;
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory, error), "an empty catalogue is an error");
        RIGIDBODIES_EXPECT(!initialize_scenario_catalogue(fixture.directory / "absent", error), "missing directory is an error");
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "free_fall"), "errors do not clear the previous catalogue");
        const auto ids = world.body_ids();
        const auto first_position = world.find_body(ids.front())->position_m();
        RIGIDBODIES_EXPECT(!load_scenario(world, "absent"), "unknown document ID fails");
        RIGIDBODIES_EXPECT(world.body_ids() == ids && world.find_body(ids.front())->position_m() == first_position, "unknown ID leaves the world untouched");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
