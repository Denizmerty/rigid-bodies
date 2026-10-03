#include <rigidbodies/ui/command_search.hpp>
#include "test_framework.hpp"
#include <algorithm>
#include <iterator>
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
int main()
{
    return rigidbodies::testing::run_all();
}
