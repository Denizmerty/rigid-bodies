#include <rigidbodies/app/experiment_content.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/ui/control_spec.hpp>

#include "test_framework.hpp"

#include <array>
#include <set>

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("every bundled Content 1.1 guide resolves its controls bodies and instances")
    {
        const std::array<std::string_view, 7> collections { "motion_and_gravity", "spin_and_mass", "friction_and_support", "bounces_and_collisions", "moving_through_air", "springs_joints_and_machines", "make_your_own" };
        std::set<std::pair<std::string, int>> orders;
        bool empty_lab_seen = false;
        for (const auto& description : physics::available_scenarios())
        {
            const auto* document = physics::scenario_document_for_id(description.id);
            RIGIDBODIES_EXPECT(document != nullptr, "every catalogue item retains its document");
            RIGIDBODIES_EXPECT(document->root.at("version").at("major").as_number() == 1 && document->root.at("version").at("minor").as_number() >= 1, "bundled content is version 1.1");
            RIGIDBODIES_EXPECT(std::find(collections.begin(), collections.end(), document->metadata.collection) != collections.end(), "collection uses one of the seven canonical keys");
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
            const auto validate = [&](std::string_view control, std::string_view body, std::string_view instance)
            {
                if (!control.empty())
                    RIGIDBODIES_EXPECT(ui::find_control_spec(control) != nullptr, "guide control resolves through the registry");
                if (!body.empty())
                    RIGIDBODIES_EXPECT(bodies.find(std::string(body)) != bodies.end(), "guide body resolves to a document id");
                if (!instance.empty())
                    RIGIDBODIES_EXPECT(instances.find(std::string(instance)) != instances.end(), "guide instance resolves to a joint or spring key");
            };
            for (const auto& variable : content.guide.variables)
                validate(variable.control, variable.body, variable.instance);
            for (const auto& step : content.guide.steps)
                validate(step.control, step.body, step.instance);
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
