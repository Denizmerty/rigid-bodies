#include <rigidbodies/ui/command_search.hpp>
#include <rigidbodies/ui/panels.hpp>
#include "relativity_fixture.hpp"
#include "test_framework.hpp"
#include <algorithm>
#include <iterator>
#include <utility>
using namespace rigidbodies;
RIGIDBODIES_TEST("command search ranks labels then words then locations")
{
    const auto gravity = ui::search_commands("gravity");
    RIGIDBODIES_EXPECT(!gravity.empty(), "gravity controls indexed");
    RIGIDBODIES_EXPECT(std::string(gravity.front().control->label).find("Gravity") != std::string::npos, "label match ranks before locations");
    const auto impulses = ui::search_commands("Reuse contact");
    RIGIDBODIES_EXPECT(!impulses.empty() && impulses.front().control->key == "world.advanced.reuse_impulses", "expert control found");
    RIGIDBODIES_EXPECT(ui::search_commands("a").size() <= 50, "results capped");
}
RIGIDBODIES_TEST("command search reveals fields and executes actions")
{
    const auto gravity = ui::search_commands("Gravity strength");
    RIGIDBODIES_EXPECT(!gravity.empty() && gravity.front().control, "gravity strength is searchable");
    RIGIDBODIES_EXPECT(gravity.front().command.kind == ui::UiCommandKind::none, "a value control does not execute with a fabricated value");
    RIGIDBODIES_EXPECT(gravity.front().command.detail == "search-reveal:world.gravity.strength", "value result reveals its exact control key");
    const auto frame = ui::search_commands("Frame everything");
    RIGIDBODIES_EXPECT(!frame.empty() && frame.front().command.kind == ui::UiCommandKind::frame_all, "an action result executes its command");
}
RIGIDBODIES_TEST("command search breadcrumbs name places the learner can see")
{
    // The surfaces the interface shows by these names: toolbar, main menu, Show, the Inspector's Selection and World pages and its joint or spring page, Preferences, Measure, Library, Guide, and the drawing bar and its options.
    const std::string_view surfaces[] { "Toolbar", "Menu", "Show", "Selection", "World", "Inspector", "Preferences", "Measure", "Library", "Guide", "Draw bar", "Drawing options" };
    for (const auto& spec : ui::control_specs())
    {
        const auto location = std::string(spec.location);
        const auto root = location.substr(0, location.find(" \xE2\x80\xBA "));
        RIGIDBODIES_EXPECT(std::find(std::begin(surfaces), std::end(surfaces), root) != std::end(surfaces), "a breadcrumb starts at a visible surface: " + std::string(spec.key) + " in " + location);
        RIGIDBODIES_EXPECT(location.find('>') == std::string::npos, "a breadcrumb uses the interface's separator: " + location);
    }
    const auto location_of = [](std::string_view key)
    {
        const auto* spec = ui::find_control_spec(key);
        return spec ? std::string(spec->location) : std::string {};
    };
    RIGIDBODIES_EXPECT(location_of("world.gravity.strength") == "World \xE2\x80\xBA Gravity" && location_of("world.air.resistance") == "World \xE2\x80\xBA Air", "world settings live under World");
    RIGIDBODIES_EXPECT(location_of("prefs.appearance.theme") == "Preferences \xE2\x80\xBA Appearance" && location_of("camera.frame.everything") == "Menu \xE2\x80\xBA View", "theme and framing name Preferences and the menu");
    const std::vector<ui::KeyReference> keys { { "Ctrl+Z", "Undo" } };
    const auto undo = ui::search_commands("undo", keys);
    RIGIDBODIES_EXPECT(std::count_if(undo.begin(), undo.end(), [](const auto& result)
                           {
                               return result.label == "Undo";
                           }) == 1 &&
            !undo.empty() && undo.front().shortcut == "Ctrl+Z",
        "a command found as a control and a key is listed once, with its shortcut");
}
RIGIDBODIES_TEST("relativity controls answer everyday words")
{
    const auto first_key = [](std::string_view query)
    {
        const auto results = ui::search_commands(query);
        return results.empty() ? std::string {} : results.front().key;
    };
    RIGIDBODIES_EXPECT(first_key("speed of light") == "world.relativity.speed", "\"speed of light\" finds the probe's speed");
    RIGIDBODIES_EXPECT(first_key("lorentz") == "measure.relativity.gamma", "\"lorentz\" finds the Lorentz factor");
    RIGIDBODIES_EXPECT(first_key("proper time") == "measure.relativity.probe_clock", "\"proper time\" finds the probe's clock");
    const auto speed = ui::search_commands("speed of light");
    RIGIDBODIES_EXPECT(!speed.empty() && speed.front().command.detail == "search-reveal:world.relativity.speed", "the speed is revealed for editing, not set to a made-up value");
}

namespace
{
    class Device final : public render::RenderDevice
    {
    public:
        std::string_view backend_name() const override
        {
            return "test";
        }
        render::ViewportSize drawable_size() const override
        {
            return { 1600, 900 };
        }
        float display_scale() const override
        {
            return 1.0f;
        }
        void set_vertical_sync(bool) override
        {
        }
        void begin_frame(const render::Color&) override
        {
        }
        void submit(const render::DrawList&) override
        {
        }
        void end_frame() override
        {
        }
        float measure_text_width(std::string_view text, float scale) const override
        {
            return static_cast<float>(text.size()) * 7.0f * scale;
        }
        float text_line_height(float scale) const override
        {
            return 14.0f * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    // The keys the search palette lists for a query (or its suggestions when the query is empty).
    std::vector<std::string> palette_keys(const ui::UiModel& model, std::string_view query)
    {
        Device device;
        render::Theme theme;
        render::DrawList draw;
        std::vector<ui::Hotspot> hotspots;
        ui::ViewState view;
        view.open_sheet("command_search");
        view.set_value("search.query", query);
        std::vector<ui::PanelRow> rows;
        ui::PanelBuilder builder(theme, device, 1.0f, { {}, { 800, 4000 } }, draw, hotspots, &view);
        builder.record_rows(rows);
        ui::CommandSearchPanel panel;
        panel.build(model, builder);
        std::vector<std::string> keys;
        for (const auto& row : rows)
            if (row.key == "search.result")
            {
                const auto& detail = row.command.detail;
                const auto separator = detail.find(':');
                keys.push_back(separator == std::string::npos ? detail : detail.substr(separator + 1));
            }
        return keys;
    }

    bool lists(const std::vector<std::string>& keys, std::string_view key)
    {
        return std::find(keys.begin(), keys.end(), key) != keys.end();
    }
}

RIGIDBODIES_TEST("search hides the other model's controls")
{
    ui::UiModel newtonian;
    ui::UiModel relativity;
    relativity.relativity = testing::relativity_model_at(0.5, 0.0);

    RIGIDBODIES_EXPECT(!lists(palette_keys(newtonian, "speed of light"), "world.relativity.speed") && !lists(palette_keys(newtonian, "lorentz"), "measure.relativity.gamma"), "a Newtonian experiment lists no relativity control");
    RIGIDBODIES_EXPECT(lists(palette_keys(relativity, "speed of light"), "world.relativity.speed") && lists(palette_keys(relativity, "lorentz"), "measure.relativity.gamma"), "the relativity experiment lists its own controls");
    RIGIDBODIES_EXPECT(lists(palette_keys(newtonian, "gravity"), "world.gravity.strength") && !lists(palette_keys(relativity, "gravity"), "world.gravity.strength"), "gravity is offered only where there are Newtonian objects");
    // Each query finds a Newtonian control in a Newtonian experiment and not in the relativity one.
    const std::pair<std::string_view, std::string_view> newtonian_only[] { { "mass", "object.properties.mass" }, { "draw", "draw.actions.apply" }, { "arrow", "show.arrows.auto_length" }, { "impact", "measure.collisions.pause_next" }, { "tools", "tools.mode" }, { "import", "library.top.import" } };
    for (const auto& [query, key] : newtonian_only)
        RIGIDBODIES_EXPECT(lists(palette_keys(newtonian, query), key) && !lists(palette_keys(relativity, query), key), "a search for " + std::string(query) + " lists " + std::string(key) + " only in a Newtonian experiment");
    RIGIDBODIES_EXPECT(!ui::control_available(relativity, "object.properties.mass") && !ui::control_available(relativity, "tools.mode") && !ui::control_available(relativity, "library.top.import") && !ui::control_available(relativity, "measure.graph.quantities"), "object, tool and import controls are hidden in relativity");
    RIGIDBODIES_EXPECT(ui::control_available(relativity, "prefs.appearance.theme") && ui::control_available(newtonian, "prefs.appearance.theme") && ui::control_available(relativity, "measure.graph.clocks") && !ui::control_available(newtonian, "measure.graph.clocks"), "shared controls are offered in both, and the clock graph only with clocks");

    const auto suggestions = palette_keys(relativity, "");
    RIGIDBODIES_EXPECT(lists(suggestions, "world.relativity.speed") && lists(suggestions, "world.relativity.preset") && lists(suggestions, "measure.relativity.curve") && !lists(suggestions, "world.gravity.strength"), "the relativity experiment suggests its speed and plot");
    RIGIDBODIES_EXPECT(lists(palette_keys(newtonian, ""), "world.gravity.strength"), "a Newtonian experiment still suggests gravity");
}

int main()
{
    return rigidbodies::testing::run_all();
}
