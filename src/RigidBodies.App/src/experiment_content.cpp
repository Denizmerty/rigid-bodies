#include <rigidbodies/app/experiment_content.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/physics/relativity_document.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/measure_tabs.hpp>

#include <cmath>
#include <set>

namespace rigidbodies::app
{
    namespace
    {
        using Json = physics::content::Json;

        std::string text(const Json* value)
        {
            return value && value->is_string() ? value->as_string() : std::string {};
        }

        int integer(const Json* value)
        {
            return value && value->is_number() && std::isfinite(value->as_number()) ? static_cast<int>(value->as_number()) : 0;
        }

        bool boolean(const Json* value)
        {
            return value && value->is_bool() && value->as_bool();
        }

        std::vector<std::string> strings(const Json* value)
        {
            std::vector<std::string> result;
            if (!value || !value->is_array())
                return result;
            for (const auto& item : value->as_array())
                if (item.is_string())
                    result.push_back(item.as_string());
            return result;
        }

        std::string collection_title(std::string_view key)
        {
            if (key == "motion_and_gravity")
                return "Motion and gravity";
            if (key == "spin_and_mass")
                return "Spin and mass distribution";
            if (key == "friction_and_support")
                return "Friction and support";
            if (key == "bounces_and_collisions")
                return "Bounces and collisions";
            if (key == "moving_through_air")
                return "Moving through air";
            if (key == "springs_joints_and_machines")
                return "Springs, joints and machines";
            if (key == "special_relativity")
                return "Special relativity";
            if (key == "make_your_own")
                return "Make your own";
            return std::string(key);
        }

        std::string level_title(std::string_view key)
        {
            if (key == "intro")
                return "Introductory";
            if (key == "core")
                return "Core";
            if (key == "further")
                return "Further";
            return std::string(key);
        }

        void report_reference_once(std::string_view document, std::string_view kind, std::string_view value)
        {
            static std::set<std::string, std::less<>> reported;
            const auto key = std::string(document) + ':' + std::string(kind) + ':' + std::string(value);
            if (reported.insert(key).second)
                core::log_warning("content {} has unknown guide {} '{}'", document, kind, value);
        }
    }

    ui::ExperimentContent parse_experiment_content(const physics::ScenarioDocument& document)
    {
        ui::ExperimentContent result;
        result.id = document.metadata.id;
        result.title = document.metadata.title;
        result.summary = document.metadata.summary;
        result.collection = collection_title(document.metadata.collection);
        result.level = level_title(document.metadata.level);
        result.hook = document.metadata.hook;
        result.collection_order = document.metadata.collection_order;
        result.suggested_order = document.metadata.suggested_order;
        result.lab = document.metadata.lab;
        result.concepts = document.metadata.concepts;
        result.prerequisites = document.metadata.prerequisites;
        result.tags = document.metadata.tags;
        // The feature, never the id or collection: a titled save rewrites both.
        result.special_relativity = physics::declares_special_relativity(document.root);

        if (const auto* metadata = document.root.find("metadata"); metadata && metadata->is_object())
        {
            result.based_on = text(metadata->find("based_on"));
            if (result.collection.empty())
                result.collection = collection_title(text(metadata->find("collection")));
            if (result.level.empty())
                result.level = level_title(text(metadata->find("level")));
            if (result.hook.empty())
                result.hook = text(metadata->find("hook"));
            if (result.collection_order == 0)
                result.collection_order = integer(metadata->find("collection_order"));
            result.lab = result.lab || boolean(metadata->find("lab"));
        }

        if (const auto* guide = document.root.find("guide"); guide && guide->is_object())
        {
            result.guide.focus = text(guide->find("focus"));
            result.guide.expect = text(guide->find("expect"));
            result.guide.limits = text(guide->find("limits"));
            if (const auto* predict = guide->find("predict"); predict && predict->is_object())
            {
                result.guide.predict.prompt = text(predict->find("prompt"));
                result.guide.predict.options = strings(predict->find("options"));
            }
            if (const auto* variables = guide->find("variables"); variables && variables->is_array())
                for (const auto& item : variables->as_array())
                    if (item.is_object())
                        result.guide.variables.push_back({ text(item.find("control")), text(item.find("body")), text(item.find("instance")), text(item.find("label")) });
            if (const auto* steps = guide->find("steps"); steps && steps->is_array())
                for (const auto& item : steps->as_array())
                    if (item.is_object())
                        result.guide.steps.push_back({ text(item.find("text")), text(item.find("control")), text(item.find("body")), text(item.find("instance")), text(item.find("open")) });
        }

        if (const auto* presentation = document.root.find("presentation"); presentation && presentation->is_object())
        {
            result.presentation.layers = strings(presentation->find("layers"));
            if (const auto* view = presentation->find("view"); view && view->is_object())
            {
                const auto* x = view->find("center_x_m");
                const auto* y = view->find("center_y_m");
                const auto* height = view->find("height_m");
                if (x && y && height && x->is_number() && y->is_number() && height->is_number())
                {
                    result.presentation.has_view = true;
                    result.presentation.center_x_m = x->as_number();
                    result.presentation.center_y_m = y->as_number();
                    result.presentation.height_m = height->as_number();
                }
            }
        }

        if (const auto* world = document.root.find("world"); world && world->is_object())
            if (const auto* bodies = world->find("bodies"); bodies && bodies->is_array())
                for (const auto& body : bodies->as_array())
                    if (body.is_object())
                    {
                        ui::BodyAnnotation annotation;
                        annotation.document_id = text(body.find("id"));
                        annotation.label = text(body.find("label"));
                        annotation.role = text(body.find("role"));
                        if (annotation.role.empty())
                            annotation.role = "object";
                        annotation.caption = text(body.find("caption"));
                        result.bodies.push_back(std::move(annotation));
                    }

        std::set<std::string, std::less<>> body_ids, instance_ids;
        for (const auto& body : result.bodies)
            body_ids.insert(body.document_id);
        if (const auto* world = document.root.find("world"); world && world->is_object())
            for (const auto* key : { "joints", "springs" })
                if (const auto* values = world->find(key); values && values->is_array())
                    for (const auto& value : values->as_array())
                        if (value.is_object())
                        {
                            auto id = text(value.find("stable_key"));
                            if (id.empty())
                                id = text(value.find("id"));
                            if (!id.empty())
                                instance_ids.insert(std::move(id));
                        }
        const auto validate = [&](std::string& control, std::string& body, std::string& instance, std::string* open)
        {
            if (!control.empty() && !ui::find_control_spec(control))
            {
                report_reference_once(result.id, "control", control);
                control.clear();
            }
            if (!body.empty() && body_ids.find(body) == body_ids.end())
            {
                report_reference_once(result.id, "body", body);
                body.clear();
            }
            if (!instance.empty() && instance_ids.find(instance) == instance_ids.end())
            {
                report_reference_once(result.id, "instance", instance);
                instance.clear();
            }
            if (open && !open->empty() && !ui::valid_guide_open_target(*open))
            {
                report_reference_once(result.id, "open target", *open);
                open->clear();
            }
        };
        for (auto& variable : result.guide.variables)
            validate(variable.control, variable.body, variable.instance, nullptr);
        for (auto& step : result.guide.steps)
            validate(step.control, step.body, step.instance, &step.open);
        return result;
    }
}
