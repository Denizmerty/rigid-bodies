#include <rigidbodies/app/experiment_content.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/measure_tabs.hpp>

#include "test_framework.hpp"

#include <array>
#include <set>

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("every bundled Content 1.1 guide resolves its controls bodies and instances")
    {
        const std::array<std::string_view, 8> collections { "motion_and_gravity", "spin_and_mass", "friction_and_support", "bounces_and_collisions", "moving_through_air", "springs_joints_and_machines", "special_relativity", "make_your_own" };
        std::set<std::pair<std::string, int>> orders;
        bool empty_lab_seen = false;
        for (const auto& description : physics::available_scenarios())
        {
            const auto* document = physics::scenario_document_for_id(description.id);
            RIGIDBODIES_EXPECT(document != nullptr, "every catalogue item retains its document");
            RIGIDBODIES_EXPECT(document->root.at("version").at("major").as_number() == 1 && document->root.at("version").at("minor").as_number() >= 1, "bundled content is version 1.1");
            RIGIDBODIES_EXPECT(std::find(collections.begin(), collections.end(), document->metadata.collection) != collections.end(), "collection uses one of the eight canonical keys");
            RIGIDBODIES_EXPECT(orders.emplace(document->metadata.collection, document->metadata.collection_order).second, "collection order is unique");
            RIGIDBODIES_EXPECT(document->metadata.hook.size() <= 90, "library hooks fit the card limit");
            const auto content = app::parse_experiment_content(*document);
            if (description.id == "empty_lab")
            {
                empty_lab_seen = true;
                RIGIDBODIES_EXPECT(content.guide.empty(), "Empty lab intentionally has no guide");
            }
            std::set<std::string> bodies;
            if (const auto* world = document->root.find("world"); world && world->is_object())
                if (const auto* source = world->find("bodies"); source && source->is_array())
                    for (const auto& body : source->as_array())
                        bodies.insert(body.at("id").as_string());
            std::set<std::string> instances;
            if (const auto* world = document->root.find("world"); world && world->is_object())
                for (const auto* key : { "joints", "springs" })
                    if (const auto* source = world->find(key); source && source->is_array())
                        for (const auto& item : source->as_array())
                        {
                            const auto* id = item.find("stable_key");
                            if ((!id || !id->is_string()) && item.is_object())
                                id = item.find("id");
                            if (id && id->is_string())
                                instances.insert(id->as_string());
                        }
            // The parser clears what does not resolve, so the bundled document's own guide is checked.
            const auto text = [](const physics::content::Json& item, std::string_view key)
            {
                const auto* value = item.find(std::string(key));
                RIGIDBODIES_EXPECT(!value || value->is_string(), "guide references are strings");
                return value ? value->as_string() : std::string {};
            };
            const auto validate = [&](const physics::content::Json& item, bool step)
            {
                RIGIDBODIES_EXPECT(item.is_object(), "guide entries are objects");
                const auto control = text(item, "control"), body = text(item, "body"), instance = text(item, "instance");
                if (!control.empty())
                    RIGIDBODIES_EXPECT(ui::find_control_spec(control) != nullptr, "guide control resolves through the registry: " + control);
                if (!body.empty())
                    RIGIDBODIES_EXPECT(bodies.find(body) != bodies.end(), "guide body resolves to a document id: " + body);
                if (!instance.empty())
                    RIGIDBODIES_EXPECT(instances.find(instance) != instances.end(), "guide instance resolves to a joint or spring key: " + instance);
                const auto open = text(item, "open");
                RIGIDBODIES_EXPECT(open.empty() || (step && ui::valid_guide_open_target(open)), "a step's open target is a control, a Measure tab or a legacy target: " + open);
            };
            std::size_t raw_variables = 0, raw_steps = 0;
            if (const auto* guide = document->root.find("guide"); guide && guide->is_object())
            {
                if (const auto* variables = guide->find("variables"); variables && variables->is_array())
                    for (const auto& variable : variables->as_array())
                    {
                        validate(variable, false);
                        ++raw_variables;
                    }
                if (const auto* steps = guide->find("steps"); steps && steps->is_array())
                    for (const auto& step : steps->as_array())
                    {
                        validate(step, true);
                        ++raw_steps;
                    }
            }
            RIGIDBODIES_EXPECT(content.guide.variables.size() == raw_variables && content.guide.steps.size() == raw_steps, "every guide entry reaches the content");
            for (std::size_t index = 0; index < content.guide.steps.size(); ++index)
            {
                const auto* raw = document->root.at("guide").at("steps").as_array()[index].find("open");
                RIGIDBODIES_EXPECT((raw ? raw->as_string() : std::string {}) == content.guide.steps[index].open, "a valid open target survives parsing");
            }
        }
        RIGIDBODIES_EXPECT(empty_lab_seen, "the catalogue includes Empty lab");
    }

    RIGIDBODIES_TEST("Content 1.0 without teaching extensions falls back cleanly")
    {
        auto document = *physics::scenario_document_for_id("free_fall");
        document.root.as_object().erase("guide");
        document.root.as_object().erase("presentation");
        const auto content = app::parse_experiment_content(document);
        RIGIDBODIES_EXPECT(content.guide.empty() && content.presentation.layers.empty(), "missing additive fields hide their sections");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
