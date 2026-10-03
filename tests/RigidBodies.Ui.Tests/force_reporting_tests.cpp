#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/ui_context.hpp>
#include <rigidbodies/physics/body_properties.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <rigidbodies/core/display_units.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>

namespace
{
    using namespace rigidbodies;
    class Device final : public render::RenderDevice
    {
    public:
        std::string_view backend_name() const override
        {
            return "test";
        }
        render::ViewportSize drawable_size() const override
        {
            return { 800, 600 };
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
            return 12.0f * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    std::vector<ui::PanelRow> rows(ui::Panel& panel, const ui::UiModel& model, ui::ViewState& view)
    {
        Device device;
        render::Theme theme;
        render::DrawList draw;
        std::vector<ui::Hotspot> hotspots;
        std::vector<ui::PanelRow> result;
        ui::PanelBuilder builder(theme, device, 1.0f, { {}, { 800, 8000 } }, draw, hotspots, &view);
        builder.record_rows(result);
        panel.build(model, builder);
        return result;
    }
    bool has_key(const std::vector<ui::PanelRow>& values, std::string_view key)
    {
        return std::any_of(values.begin(), values.end(), [&](const auto& row)
            {
                return row.key == key;
            });
    }
}

RIGIDBODIES_TEST("object inspector uses remembered tabs and exposes every force channel")
{
    using namespace rigidbodies;
    physics::World world;
    physics::load_scenario(world, "free_fall");
    const auto selected = world.body_ids().front();
    ui::UiModel model;
    model.world = &world;
    model.selection = selected;
    model.selected_bodies = { selected };
    ui::ViewState view;
    ui::InspectorPanel panel;
    view.set_active_tab("inspector.object", "properties");
    auto properties = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_key(properties, "object.properties.mass") && has_key(properties, "object.properties.material"), "properties are direct controls");
    view.set_active_tab("inspector.object", "forces");
    auto forces = rows(panel, model, view);
    RIGIDBODIES_EXPECT(std::none_of(forces.begin(), forces.end(), [](const auto& row)
                           {
                               return row.text.find("page") != std::string::npos;
                           }),
        "forces are a full list without paging");
}

RIGIDBODIES_TEST("world and show surfaces expose final controls without cycle commands")
{
    using namespace rigidbodies;
    physics::World world;
    physics::load_scenario(world, "free_fall");
    ui::UiModel model;
    model.world = &world;
    model.layers = render::LayerMask::defaults();
    ui::ViewState view;
    ui::InspectorPanel world_panel;
    ui::VisualizationPanel show_panel;
    view.set_section_open("world.advanced", true);
    const auto world_rows = rows(world_panel, model, view);
    RIGIDBODIES_EXPECT(has_key(world_rows, "world.gravity.strength") && has_key(world_rows, "world.advanced.integration_method"), "world sections use typed controls");
    const auto show_rows = rows(show_panel, model, view);
    RIGIDBODIES_EXPECT(has_key(show_rows, "show.presets.preset") && has_key(show_rows, "show.arrows.split"), "show uses switches and segments");
}

namespace
{
    const ui::PanelRow* find_row(const std::vector<ui::PanelRow>& values, std::string_view key)
    {
        const auto found = std::find_if(values.begin(), values.end(), [&](const auto& row)
            {
                return row.key == key;
            });
        return found == values.end() ? nullptr : &*found;
    }

    std::vector<physics::BodyId> free_bodies(const physics::World& world)
    {
        std::vector<physics::BodyId> result;
        for (const auto id : world.body_ids())
            if (const auto* body = world.find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                result.push_back(id);
        return result;
    }

    void add_catalogue(ui::UiModel& model)
    {
        for (const auto& description : physics::available_scenarios())
            model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
    }
}

RIGIDBODIES_TEST("a selection of several objects shows shared values or Mixed, never a placeholder")
{
    using namespace rigidbodies;
    {
        physics::World world;
        physics::load_scenario(world, "stable_stack");
        ui::UiModel model;
        model.world = &world;
        model.selected_bodies = free_bodies(world);
        model.selection = model.selected_bodies.back();
        ui::ViewState view;
        ui::InspectorPanel panel;
        const auto values = rows(panel, model, view);
        const auto* mass = find_row(values, "object.properties.mass");
        const auto* material = find_row(values, "object.properties.material");
        RIGIDBODIES_EXPECT(model.selected_bodies.size() == 6 && mass && !mass->mixed && std::abs(mass->number_si - world.find_body(model.selection)->mass_properties().mass_kg) < 1.0e-9, "six equal boxes show their shared mass");
        RIGIDBODIES_EXPECT(material && !material->mixed && material->selected_option == "oak_wood" && material->value.empty(), "a shared catalogue material selects its own option");
    }
    {
        physics::World world;
        physics::load_scenario(world, "free_fall");
        ui::UiModel model;
        model.world = &world;
        model.selected_bodies = free_bodies(world);
        model.selection = model.selected_bodies.back();
        ui::ViewState view;
        ui::InspectorPanel panel;
        const auto values = rows(panel, model, view);
        const auto* mass = find_row(values, "object.properties.mass");
        const auto* material = find_row(values, "object.properties.material");
        const auto* scale = find_row(values, "object.properties.gravity_scale");
        RIGIDBODIES_EXPECT(mass && mass->mixed && mass->value == "Mixed", "differing masses read Mixed");
        RIGIDBODIES_EXPECT(material && material->mixed, "differing materials read Mixed");
        RIGIDBODIES_EXPECT(scale && !scale->mixed && scale->number_si == 1.0, "a shared gravity scale is shown");
        const auto* stop = find_row(values, "object.motion.stop");
        RIGIDBODIES_EXPECT(stop && !stop->disabled_reason.empty(), "Stop motion is unavailable while nothing selected moves");
    }
    {
        physics::World world;
        physics::load_scenario(world, "ramp");
        ui::UiModel model;
        model.world = &world;
        model.selected_bodies = world.body_ids();
        model.selection = model.selected_bodies.front();
        ui::ViewState view;
        ui::InspectorPanel panel;
        const auto values = rows(panel, model, view);
        RIGIDBODIES_EXPECT(!find_row(values, "object.properties.mass") && !find_row(values, "object.properties.gravity_scale"), "mass and gravity scale are left out when fixed objects are selected");
    }
}

RIGIDBODIES_TEST("material rows show the body's own material and never fall back to the first option")
{
    using namespace rigidbodies;
    ui::ViewState view;
    ui::InspectorPanel panel;
    physics::World world;
    physics::load_scenario(world, "free_fall");
    const auto material_row = [&](physics::BodyId id)
    {
        ui::UiModel model;
        model.world = &world;
        model.selection = id;
        model.selected_bodies = { id };
        const auto values = rows(panel, model, view);
        const auto* found = find_row(values, "object.properties.material");
        return found ? *found : ui::PanelRow { ui::PanelRowKind::select };
    };
    for (const auto id : free_bodies(world))
    {
        const auto row = material_row(id);
        const auto name = world.find_body(id)->colliders().front().material.name;
        RIGIDBODIES_EXPECT(row.spec && row.selected_option == name && row.value.empty(), "a catalogue material selects its own entry: " + name);
    }
    const auto authored = free_bodies(world).front();
    physics::BodyPropertyEdit edit;
    edit.material = physics::materials::rubber();
    edit.material->name = "grippy_rubber";
    physics::edit_body_properties(world, authored, edit);
    const auto row = material_row(authored);
    RIGIDBODIES_EXPECT(row.selected_option == "grippy_rubber" && row.value == "Grippy rubber", "an authored material is named rather than shown as the first option");
}

RIGIDBODIES_TEST("the gravity preset follows the world's gravity")
{
    using namespace rigidbodies;
    physics::World world;
    physics::load_scenario(world, "free_fall");
    ui::UiModel model;
    model.world = &world;
    model.gravity_direction_degrees = -90.0;
    const auto preset = [&](math::Vec2 gravity)
    {
        auto settings = world.settings();
        settings.gravity_m_s2 = gravity;
        world.set_settings(settings);
        return std::string(ui::gravity_preset_id(model));
    };
    RIGIDBODIES_EXPECT(preset({ 0.0, -9.81 }) == "earth" && preset({ 0.0, -physics::standard_gravity_m_s2 }) == "earth", "Earth matches 9.81 and standard gravity");
    RIGIDBODIES_EXPECT(preset({ 0.0, -1.62 }) == "moon" && preset({ 0.0, -3.73 }) == "mars" && preset({ 0.0, -5.0 }) == "custom", "Moon, Mars and other strengths");
    RIGIDBODIES_EXPECT(preset({}).empty(), "no preset while gravity is off");
    model.gravity_direction_degrees = -60.0;
    RIGIDBODIES_EXPECT(preset({ 0.0, -9.81 }) == "custom", "tilted gravity is not a planet preset");
}

RIGIDBODIES_TEST("Show presets mark a custom layer mix and choosing Custom keeps it")
{
    using namespace rigidbodies;
    ui::UiModel model;
    model.layers = render::LayerMask::defaults();
    model.layers.set(render::VisualizationLayer::grid, !model.layers.is_enabled(render::VisualizationLayer::grid));
    ui::ViewState view;
    ui::VisualizationPanel panel;
    const auto values = rows(panel, model, view);
    const auto* presets = find_row(values, "show.presets.preset");
    RIGIDBODIES_EXPECT(presets && presets->selected_option == "custom", "a mix matching no preset selects Custom");
    RIGIDBODIES_EXPECT(presets && static_cast<std::uint32_t>(presets->command.value) == model.layers.bits(), "Custom carries the current mix");
}

RIGIDBODIES_TEST("the Present caption offers the step's control and leaves Present for Measure")
{
    using namespace rigidbodies;
    physics::World world;
    physics::load_scenario(world, "free_fall");
    ui::UiModel model;
    model.world = &world;
    for (const auto id : world.body_ids())
        if (const auto* body = world.find_body(id))
            model.objects.push_back({ id, "body-" + std::to_string(id.index) + "-" + std::to_string(id.index + 1), std::string(body->name()), "free", "object", {}, false, false });
    ui::ExperimentContent content;
    content.guide.variables = { { "object.properties.mass", "body-2-3", {}, "Wooden ball mass" } };
    content.guide.steps = { { "Set the wooden ball's mass.", "object.properties.mass", "body-2-3", {}, {} }, { "Choose Moon gravity.", "world.gravity.preset", {}, {}, {} }, { "Compare the runs in the graph.", {}, {}, {}, "measure.graph" } };
    model.scenario_content = content;
    ui::ViewState view;
    view.present().mode = true;
    ui::PresentCaptionPanel panel;
    const auto first = rows(panel, model, view);
    const auto* mass = find_row(first, "object.properties.mass");
    RIGIDBODIES_EXPECT(mass && mass->text == "Wooden ball mass" && mass->group == "caption-control", "the step's number control sits on the caption with its guide label");
    view.set_value("present.guide_step", "1");
    const auto second = rows(panel, model, view);
    const auto* preset = find_row(second, "world.gravity.preset");
    RIGIDBODIES_EXPECT(preset && preset->kind == ui::PanelRowKind::segmented && preset->selected_option == ui::gravity_preset_id(model), "the preset step shows the current preset");
    view.set_value("present.guide_step", "2");
    const auto third = rows(panel, model, view);
    const auto* show = find_row(third, "present.strip.show_me");
    RIGIDBODIES_EXPECT(show && show->text == "Exit and show me" && show->command.detail == "present-reveal:measure.graph", "a step that reads Measure leaves Present to show it");
}

RIGIDBODIES_TEST("the Library keeps each collection together and opens the selected card in place")
{
    using namespace rigidbodies;
    ui::UiModel model;
    add_catalogue(model);
    model.scenario_id = "stable_stack";
    ui::ViewState view;
    ui::LibraryPanel panel;
    const auto values = rows(panel, model, view);
    std::vector<std::string> headings;
    for (const auto& row : values)
        if (row.kind == ui::PanelRowKind::heading && row.text != "My setups")
            headings.push_back(row.text);
    auto unique = headings;
    std::sort(unique.begin(), unique.end());
    RIGIDBODIES_EXPECT(!headings.empty() && std::unique(unique.begin(), unique.end()) == unique.end(), "every collection heading appears once");
    const auto card = std::find_if(values.begin(), values.end(), [](const auto& row)
        {
            return row.kind == ui::PanelRowKind::list_item && row.selected;
        });
    RIGIDBODIES_EXPECT(card != values.end() && card->instance == "stable_stack" && card->hint.find("Contact forces support") != std::string::npos, "the selected card carries the full summary");
    RIGIDBODIES_EXPECT(card != values.end() && card->list_content, "the selected card has content");
    if (card == values.end() || !card->list_content)
        return;
    const auto& actions = card->list_content->actions;
    RIGIDBODIES_EXPECT(std::any_of(actions.begin(), actions.end(), [](const auto& action)
                           {
                               return action.label == "Builds on Free fall" && action.command.detail == "library-show" && !action.primary;
                           }),
        "prerequisites are named by title inside the card");
    RIGIDBODIES_EXPECT(!actions.empty() && actions.front().label == "Back to experiment" && actions.front().primary && actions.front().command.detail == "view:close", "the open experiment's card returns to it rather than reopening it");
}

RIGIDBODIES_TEST("every Library hook is a whole sentence, never a summary cut at a decimal point")
{
    using namespace rigidbodies;
    for (const auto& description : physics::available_scenarios())
    {
        const std::string hook(description.hook), summary(description.summary), id(description.id);
        RIGIDBODIES_EXPECT(hook.size() > 20 && hook.back() == '.', "the hook is a sentence: " + id);
        const auto cut_mid_number = summary.size() > hook.size() && summary.compare(0, hook.size(), hook) == 0 && std::isdigit(static_cast<unsigned char>(summary[hook.size()]));
        RIGIDBODIES_EXPECT(!cut_mid_number, "the hook is not the summary cut at a decimal point: " + id);
    }
}

RIGIDBODIES_TEST("opening World brings the World page forward even while an object is selected")
{
    using namespace rigidbodies;
    ui::UiContext context;
    RIGIDBODIES_EXPECT(context.initialize(ui::UiBackendKind::overlay, {}), "overlay interface starts");
    const std::array<std::string_view, 2> targets { "selection", "world" };
    context.view_state().set_active_tab("inspector.target", "selection");
    context.request(ui::ViewRequest::open_world);
    RIGIDBODIES_EXPECT(context.view_state().active_tab("inspector.target", targets, "selection") == "world", "W shows World");
    context.reveal("object.motion.spin");
    const std::array<std::string_view, 5> pages { "properties", "motion", "forces", "joints", "shape" };
    RIGIDBODIES_EXPECT(context.view_state().active_tab("inspector.target", targets, "world") == "selection" && context.view_state().active_tab("inspector.object", pages, "properties") == "motion", "revealing an object setting opens its page");
}

RIGIDBODIES_TEST("drawing tolerances read in one unit that never changes while editing")
{
    using namespace rigidbodies;
    const std::string nbsp = "\xC2\xA0";
    for (const auto* key : { "draw.precision.drawing", "draw.precision.collision", "draw.precision.simplify", "draw.precision.hollows" })
        RIGIDBODIES_EXPECT(ui::find_control_spec(key)->number.quantity == core::DisplayQuantity::fine_length, std::string("tolerance uses the fine length unit: ") + key);
    RIGIDBODIES_EXPECT(core::format_quantity(0.002, core::DisplayQuantity::fine_length, core::DisplayUnits::si) == "2.0" + nbsp + "mm", "2 mm");
    RIGIDBODIES_EXPECT(core::format_quantity(0.01, core::DisplayQuantity::fine_length, core::DisplayUnits::si) == "10.0" + nbsp + "mm", "a centimetre stays in millimetres");
    RIGIDBODIES_EXPECT(core::format_quantity(0.05, core::DisplayQuantity::fine_length, core::DisplayUnits::centimetre_gram) == "5.00" + nbsp + "cm", "CGS uses centimetres");
    const auto parsed = core::parse_quantity("10", core::DisplayQuantity::fine_length, core::DisplayUnits::si);
    RIGIDBODIES_EXPECT(parsed && std::abs(*parsed - 0.01) < 1.0e-12, "a bare number is read in millimetres");
}

int main()
{
    return rigidbodies::testing::run_all();
}
