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
        state.set_experiment_context("ramp");
        RIGIDBODIES_EXPECT(state.value("measure.runs.compare_a", "default") == "default", "another experiment uses its defaults");
        state.set_value("measure.runs.compare_a", "7");
        state.set_experiment_context("free_fall");
        RIGIDBODIES_EXPECT(state.value("measure.runs.compare_a", "default") == "2" && state.value("measure.runs.compare_b", "default") == "4", "A and B return with their experiment");
        RIGIDBODIES_EXPECT(state.value("measure.graph.compare_with", "none") == "2", "graph comparison is also scoped to the experiment");

        ui::ViewState loaded;
        RIGIDBODIES_EXPECT(loaded.deserialize(state.serialize()).empty(), "persistent state remains valid");
        loaded.set_experiment_context("free_fall");
        RIGIDBODIES_EXPECT(loaded.value("measure.runs.compare_a", "default") == "default" && loaded.value("measure.graph.compare_with", "none") == "none", "session comparison choices are not persisted");
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
