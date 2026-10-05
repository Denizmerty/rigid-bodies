#pragma once

#include <string>
#include <vector>

namespace rigidbodies::ui
{
    struct GuideVariable
    {
        std::string control, body, instance, label;
    };

    struct GuideStep
    {
        std::string text, control, body, instance, open;
    };

    struct PredictContent
    {
        std::string prompt;
        std::vector<std::string> options;
    };

    struct GuideContent
    {
        std::string focus, expect, limits;
        PredictContent predict;
        std::vector<GuideVariable> variables;
        std::vector<GuideStep> steps;

        [[nodiscard]] bool empty() const
        {
            return focus.empty() && expect.empty() && limits.empty() && predict.prompt.empty() && variables.empty() && steps.empty();
        }
    };

    struct PresentationContent
    {
        bool has_view {};
        double center_x_m {}, center_y_m {}, height_m {};
        std::vector<std::string> layers;
    };

    struct BodyAnnotation
    {
        std::string document_id, label, role { "object" }, caption;
    };

    struct ExperimentContent
    {
        std::string id, title, summary, collection, level, hook, based_on;
        int collection_order {}, suggested_order {};
        bool lab {};
        std::vector<std::string> concepts, prerequisites, tags;
        GuideContent guide;
        PresentationContent presentation;
        std::vector<BodyAnnotation> bodies;
        // The document requires special_relativity, so its experiment is the relativistic probe.
        bool special_relativity {};
    };
}
