#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/ui/control_spec.hpp>

#include <algorithm>
#include <optional>
#include <string_view>

namespace rigidbodies::ui
{
    // The Measure tab ids a model offers, in order, and its default tab. A special-relativity
    // experiment has no Newtonian energy, impacts or theory checks to show.
    [[nodiscard]] inline math::Span<const std::string_view> measure_tab_ids(bool relativity)
    {
        static constexpr std::string_view newtonian[] { "energy", "graph", "collisions", "runs", "theory" };
        static constexpr std::string_view special_relativity[] { "relativity", "graph", "runs" };
        if (relativity)
            return special_relativity;
        return newtonian;
    }

    [[nodiscard]] inline std::string_view default_measure_tab(bool relativity)
    {
        return relativity ? "relativity" : "energy";
    }

    // The clocks the Graph plots in a relativity experiment before the learner chooses others, and
    // the most it plots at once.
    [[nodiscard]] inline math::Span<const std::string_view> default_graph_clocks()
    {
        static constexpr std::string_view clocks[] { "lab_clock", "probe_clock" };
        return clocks;
    }
    inline constexpr std::size_t maximum_graph_quantities = 3;

    // The tabs whose drawer may use the full width (the document backend's "is-graph" class).
    [[nodiscard]] inline bool measure_tab_is_wide(std::string_view tab)
    {
        return tab == "graph" || tab == "relativity";
    }

    // The tab of a "measure.<tab>[.<rest>]" key when <tab> is any known tab id, else std::nullopt.
    [[nodiscard]] inline std::optional<std::string_view> measure_tab_of_key(std::string_view key)
    {
        constexpr std::string_view prefix = "measure.";
        if (key.substr(0, prefix.size()) != prefix)
            return std::nullopt;
        const auto rest = key.substr(prefix.size());
        const auto tab = rest.substr(0, rest.find('.'));
        for (const auto relativity : { false, true })
        {
            const auto ids = measure_tab_ids(relativity);
            if (const auto found = std::find(ids.begin(), ids.end(), tab); found != ids.end())
                return *found;
        }
        return std::nullopt;
    }

    // A Guide step's "open" target: a registered control key, "measure.<tab>" for a known tab, or one
    // of the legacy targets measure.graph, measure.collisions.list, measure.theory.collisions_run.
    [[nodiscard]] inline bool valid_guide_open_target(std::string_view key)
    {
        if (key == "measure.graph" || key == "measure.collisions.list" || key == "measure.theory.collisions_run")
            return true;
        if (find_control_spec(key))
            return true;
        const auto tab = measure_tab_of_key(key);
        return tab && key.size() == std::string_view("measure.").size() + tab->size();
    }
}
