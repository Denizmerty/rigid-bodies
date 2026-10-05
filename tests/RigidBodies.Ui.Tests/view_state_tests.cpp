#include <rigidbodies/ui/measure_tabs.hpp>
#include <rigidbodies/ui/view_state.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <array>

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("view state round trips tabs sections and checklists")
    {
        ui::ViewState source;
        source.set_active_tab("inspector.object", "motion");
        source.set_section_open("world.advanced", true);
        source.set_checklist("measure.graph.quantities", { "mechanical", "speed" });
        ui::ViewState loaded;
        const auto diagnostics = loaded.deserialize(source.serialize());
        const std::array<std::string_view, 2> tabs { "properties", "motion" };
        const std::array<std::string_view, 1> defaults { "mechanical" };
        RIGIDBODIES_EXPECT(diagnostics.empty(), "saved view state loads without diagnostics");
        RIGIDBODIES_EXPECT(loaded.active_tab("inspector.object", tabs, "properties") == "motion", "tab survives round trip");
        RIGIDBODIES_EXPECT(loaded.section_open("world.advanced", false), "section survives round trip");
        RIGIDBODIES_EXPECT(loaded.checklist("measure.graph.quantities", defaults).size() == 2, "checklist survives round trip");
    }

    RIGIDBODIES_TEST("remembered tabs fall back while absent and return when available")
    {
        ui::ViewState state;
        state.set_active_tab("inspector.object", "joints");
        const std::array<std::string_view, 2> without { "properties", "motion" };
        const std::array<std::string_view, 3> with { "properties", "motion", "joints" };
        RIGIDBODIES_EXPECT(state.active_tab("inspector.object", without, "properties") == "properties", "missing tab falls back");
        RIGIDBODIES_EXPECT(state.active_tab("inspector.object", with, "properties") == "joints", "remembered tab returns");
    }

    RIGIDBODIES_TEST("a stored relativity tab falls back to energy and back")
    {
        ui::ViewState state;
        const auto newtonian = ui::measure_tab_ids(false);
        const auto relativity = ui::measure_tab_ids(true);
        const auto newtonian_tab = [&]
        {
            return state.active_tab("measure.header.tabs", newtonian, ui::default_measure_tab(false));
        };
        const auto relativity_tab = [&]
        {
            return state.active_tab("measure.header.tabs", relativity, ui::default_measure_tab(true));
        };
        RIGIDBODIES_EXPECT(newtonian_tab() == "energy" && relativity_tab() == "relativity", "each model opens Measure on its own default tab");
        state.set_active_tab("measure.header.tabs", "relativity");
        RIGIDBODIES_EXPECT(newtonian_tab() == "energy", "a Newtonian experiment never shows the Relativity tab");
        RIGIDBODIES_EXPECT(relativity_tab() == "relativity", "the relativity experiment returns to it");
        state.set_active_tab("measure.header.tabs", "collisions");
        RIGIDBODIES_EXPECT(relativity_tab() == "relativity" && newtonian_tab() == "collisions", "a Newtonian-only tab falls back in the relativity experiment and returns afterwards");
        state.set_active_tab("measure.header.tabs", "graph");
        RIGIDBODIES_EXPECT(relativity_tab() == "graph" && newtonian_tab() == "graph", "Graph belongs to both models");

        state.set_active_tab("measure.header.tabs", "relativity");
        ui::ViewState loaded;
        RIGIDBODIES_EXPECT(loaded.deserialize(state.serialize()).empty(), "a stored relativity tab is valid state");
        RIGIDBODIES_EXPECT(loaded.active_tab("measure.header.tabs", newtonian, ui::default_measure_tab(false)) == "energy" && loaded.active_tab("measure.header.tabs", relativity, ui::default_measure_tab(true)) == "relativity", "the stored tab survives a restart and still falls back");
        RIGIDBODIES_EXPECT(ui::measure_tab_is_wide("graph") && ui::measure_tab_is_wide("relativity") && !ui::measure_tab_is_wide("energy") && !ui::measure_tab_is_wide("runs"), "the plotting tabs may use the drawer's full width");
    }

    RIGIDBODIES_TEST("malformed unknown and newer state is handled defensively")
    {
        ui::ViewState state;
        RIGIDBODIES_EXPECT(!state.deserialize("interface_state.version = 1\nunknown.key = value\nsection.bad = maybe\n").empty(), "unknown and malformed lines are reported");
        state.set_section_open("world.advanced", true);
        RIGIDBODIES_EXPECT(!state.deserialize("interface_state.version = 3\nsection.world.advanced = false\n").empty(), "newer version is reported");
        RIGIDBODIES_EXPECT(state.section_open("world.advanced", false), "newer version leaves existing state untouched");
    }

    RIGIDBODIES_TEST("run comparison choices are scoped per experiment and never persisted")
    {
        ui::ViewState state;
        state.set_experiment_context("free_fall");
        state.set_value("measure.runs.compare_a", "2");
        state.set_value("measure.runs.compare_b", "4");
        state.set_value("measure.graph.compare_with", "2");
        state.set_value("measure.graph.review_run", "3");
        state.set_value("measure.runs.inspect", "4");
        state.set_experiment_context("ramp");
        RIGIDBODIES_EXPECT(state.value("measure.runs.compare_a", "default") == "default", "another experiment uses its defaults");
        RIGIDBODIES_EXPECT(state.value("measure.graph.review_run").empty() && state.value("measure.runs.inspect").empty(), "saved run review never follows an unrelated experiment's same run number");
        state.set_value("measure.runs.compare_a", "7");
        state.set_experiment_context("free_fall");
        RIGIDBODIES_EXPECT(state.value("measure.runs.compare_a", "default") == "2" && state.value("measure.runs.compare_b", "default") == "4", "A and B return with their experiment");
        RIGIDBODIES_EXPECT(state.value("measure.graph.compare_with", "none") == "2", "graph comparison is also scoped to the experiment");
        RIGIDBODIES_EXPECT(state.value("measure.graph.review_run") == "3" && state.value("measure.runs.inspect") == "4", "the inspected run returns with its experiment");

        ui::ViewState loaded;
        RIGIDBODIES_EXPECT(loaded.deserialize(state.serialize()).empty(), "persistent state remains valid");
        loaded.set_experiment_context("free_fall");
        RIGIDBODIES_EXPECT(loaded.value("measure.runs.compare_a", "default") == "default" && loaded.value("measure.graph.compare_with", "none") == "none", "session comparison choices are not persisted");
        RIGIDBODIES_EXPECT(loaded.value("measure.graph.review_run").empty() && loaded.value("measure.runs.inspect").empty(), "unpersisted recordings cannot leave stale review selections after restart");
    }

    RIGIDBODIES_TEST("My setups keeps the newest eight unique paths and round trips metadata")
    {
        ui::ViewState state;
        for (std::uint64_t index = 1; index <= 9; ++index)
            state.merge_setup_file({ index, "Setup " + std::to_string(index), "free_fall", "2026-10-01", "C:/setups/" + std::to_string(index) + ".json" });
        state.merge_setup_file({ 10, "Renamed", "ramp", "2026-10-02", "C:/setups/5.json" });
        RIGIDBODIES_EXPECT(state.setup_files().size() == 8, "recent setup list is capped at eight");
        RIGIDBODIES_EXPECT(state.setup_files().front().title == "Renamed" && state.setup_files().front().serial == 10, "resaving a path replaces and promotes it");
        RIGIDBODIES_EXPECT(std::count_if(state.setup_files().begin(), state.setup_files().end(), [](const auto& file)
                               {
                                   return file.path == "C:/setups/5.json";
                               }) == 1,
            "each path appears once");

        ui::ViewState loaded;
        RIGIDBODIES_EXPECT(loaded.deserialize(state.serialize()).empty(), "recent setup metadata deserializes cleanly");
        RIGIDBODIES_EXPECT(loaded.setup_files().size() == 8 && loaded.setup_files().front().title == "Renamed" && loaded.setup_files().front().based_on == "ramp", "recent setup order and metadata survive restart");
        loaded.remove_setup_file("C:/setups/5.json");
        RIGIDBODIES_EXPECT(loaded.setup_files().size() == 7, "a missing setup can be removed from the list");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
