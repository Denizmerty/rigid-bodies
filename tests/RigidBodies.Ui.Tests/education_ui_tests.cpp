#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/shape.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
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
        ui::PanelBuilder builder(theme, device, 1.0f, { {}, { 800, 4000 } }, draw, hotspots, &view);
        builder.record_rows(result);
        panel.build(model, builder);
        return result;
    }

    bool has_key(const std::vector<ui::PanelRow>& rows, std::string_view key)
    {
        return std::any_of(rows.begin(), rows.end(), [&](const auto& row)
            {
                return row.key == key;
            });
    }
}

RIGIDBODIES_TEST("Measure tabs expose graph collision and theory controls without pagers")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "graph");
    auto graph = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_key(graph, "measure.graph.quantities") && has_key(graph, "measure.graph.clear"), "graph controls use stable keys");
    view.set_active_tab("measure.header.tabs", "collisions");
    auto collisions = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_key(collisions, "measure.collisions.pause_each"), "collision pause is directly reachable");
    view.set_active_tab("measure.header.tabs", "theory");
    auto theory = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_key(theory, "measure.theory.integration_run") && has_key(theory, "measure.theory.bounce_run"), "theory cards are directly reachable");
}

namespace
{
    using namespace rigidbodies;

    ui::ImpactItem impact(std::size_t index, physics::BodyType second_type, bool selected = false)
    {
        ui::ImpactItem result;
        result.index = index;
        result.time_s = 0.5 + static_cast<double>(index);
        result.first_name = "Ball";
        result.second_name = second_type == physics::BodyType::static_body ? "Floor" : "Cart";
        result.second_type = second_type;
        result.selected = selected;
        return result;
    }

    std::vector<const ui::PanelRow*> rows_with(const std::vector<ui::PanelRow>& rows, ui::PanelRowKind kind, std::string_view key = {})
    {
        std::vector<const ui::PanelRow*> result;
        for (const auto& row : rows)
            if (row.kind == kind && (key.empty() || row.key == key))
                result.push_back(&row);
        return result;
    }

    std::string tab_label(const std::vector<ui::PanelRow>& rows, std::string_view id)
    {
        for (const auto* tabs : rows_with(rows, ui::PanelRowKind::tabs))
            for (const auto& option : tabs->options)
                if (option.id == id)
                    return std::string(option.label);
        return {};
    }

    bool has_text(const std::vector<ui::PanelRow>& rows, std::string_view text)
    {
        return std::any_of(rows.begin(), rows.end(), [&](const auto& row)
            {
                return row.text.find(text) != std::string::npos || row.value.find(text) != std::string::npos;
            });
    }
}

RIGIDBODIES_TEST("the collision filter narrows the list, the tab count and the toolbar badge alike")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.impacts = { impact(0, physics::BodyType::static_body, true), impact(1, physics::BodyType::dynamic_body) };
    view.set_active_tab("measure.header.tabs", "collisions");
    auto all = rows(panel, model, view);
    RIGIDBODIES_EXPECT(rows_with(all, ui::PanelRowKind::list_item, "measure.collisions.list").size() == 2 && tab_label(all, "collisions") == "Collisions (2)", "All contacts lists impacts with fixed bodies too");
    const auto filter = rows_with(all, ui::PanelRowKind::segmented, "measure.collisions.filter");
    RIGIDBODIES_EXPECT(filter.size() == 1 && filter.front()->command.detail == "state-id:measure.collisions.filter", "choosing a filter writes the view value the list reads");
    RIGIDBODIES_EXPECT(rows_with(all, ui::PanelRowKind::list_item, "measure.collisions.list").front()->selected, "the impact explained below is highlighted in the list");
    RIGIDBODIES_EXPECT(has_text(all, "Impulse on Ball") && !has_text(all, "Floor velocity"), "a fixed body gets no zero velocity rows and the impulse names the body that moves");

    view.set_value("measure.collisions.filter", "moving");
    auto moving = rows(panel, model, view);
    const auto listed = rows_with(moving, ui::PanelRowKind::list_item, "measure.collisions.list");
    RIGIDBODIES_EXPECT(listed.size() == 1 && listed.front()->instance == "1" && tab_label(moving, "collisions") == "Collisions (1)", "Moving pairs keeps only impacts between two free bodies and counts them");
    RIGIDBODIES_EXPECT(ui::listed_impact_count(model, "moving") == 1 && ui::listed_impact_count(model, "all") == 2, "the toolbar badge counts the same set as the tab");

    model.impacts = { impact(0, physics::BodyType::static_body) };
    RIGIDBODIES_EXPECT(has_text(rows(panel, model, view), "Choose All contacts to show the 1 impact"), "an empty filtered list says what is hidden and how to show it");
}

RIGIDBODIES_TEST("the Measure badge says what its number counts")
{
    using namespace rigidbodies;
    ui::CommandBarPanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.impacts = { impact(0, physics::BodyType::static_body), impact(1, physics::BodyType::dynamic_body) };
    const auto bar = rows(panel, model, view);
    const auto measure = std::find_if(bar.begin(), bar.end(), [](const auto& row)
        {
            return row.text == "Measure";
        });
    RIGIDBODIES_EXPECT(measure != bar.end() && measure->presentation.badge == "2" && measure->presentation.badge_description == "2 impacts recorded", "the badge carries an accessible description");
}

RIGIDBODIES_TEST("energy shares are parts of one whole, coloured like the plot series")
{
    using namespace rigidbodies;
    physics::World world;
    physics::BodyDefinition definition;
    definition.position_m = { 0.0, 2.0 };
    definition.linear_velocity_m_s = { 3.0, 0.0 };
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    definition.colliders.push_back(collider);
    world.create_body(definition);
    const auto energy = physics::measure_world_energy(world);

    // The run reached a point 10 J lower than now, so 10 J of height energy is still to give.
    ui::RunRecord run;
    run.number = 1;
    run.series.time_s = { 0.0f, 0.025f };
    run.series.scene.assign(16, 0.0f);
    run.series.scene[2] = static_cast<float>(energy.gravitational_potential_j);
    run.series.scene[8 + 2] = static_cast<float>(energy.gravitational_potential_j - 10.0);

    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.world = &world;
    model.current_run = &run;
    const auto energy_rows = rows(panel, model, view);
    const auto meters = rows_with(energy_rows, ui::PanelRowKind::meter);
    RIGIDBODIES_EXPECT(meters.size() == 4, "kinetic, potential and each contact loss of the table get a share");
    if (meters.size() != 4)
        return;
    int percent_total = 0;
    double fraction_total = 0.0;
    for (const auto* meter : meters)
    {
        percent_total += std::stoi(meter->value);
        fraction_total += meter->amount;
        RIGIDBODIES_EXPECT(meter->live, "shares refresh together with the live read-outs above them");
    }
    RIGIDBODIES_EXPECT(percent_total == 100 && std::abs(fraction_total - 1.0) < 1.0e-9, "the shares add up to 100 %");
    RIGIDBODIES_EXPECT(meters[0]->series == 1 && meters[1]->series == 3 && meters[2]->series == 2 && meters[3]->series == 2, "kinetic, potential and lost use the plot's blue, green and orange");
    RIGIDBODIES_EXPECT(meters[2]->text == "Lost in impacts" && meters[3]->text == "Lost to friction", "the loss shares carry the names of the table's rows");
    RIGIDBODIES_EXPECT(std::abs(meters[1]->amount - 10.0 / (10.0 + energy.kinetic_j())) < 1.0e-4, "potential counts from the lowest point reached in the run");
    RIGIDBODIES_EXPECT(has_text(energy_rows, "as zero height") && !has_text(energy_rows, "Only contact losses"), "the energy note explains zero height and does not limit losses to contacts");
    view.set_value("measure.energy.scope", "selected");
    const auto unselected = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_text(unselected, "Select an object") && rows_with(unselected, ui::PanelRowKind::meter).empty(), "Selected object with nothing selected asks for a selection instead of showing the scene's figures");
}

RIGIDBODIES_TEST("a damper's loss is a row and a share of the energy budget")
{
    using namespace rigidbodies;
    physics::WorldSettings settings;
    settings.gravity_m_s2 = {};
    physics::World world(settings);
    physics::BodyDefinition anchor_definition;
    anchor_definition.type = physics::BodyType::static_body;
    const auto anchor = world.create_body(anchor_definition);
    physics::BodyDefinition mass_definition;
    mass_definition.position_m = { 1.5, 0.0 };
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    mass_definition.colliders.push_back(collider);
    const auto mass = world.create_body(mass_definition);
    physics::LinearSpringDefinition spring;
    spring.first = anchor;
    spring.second = mass;
    spring.rest_length_m = 1.0;
    spring.stiffness_n_m = 20.0;
    spring.damping_n_s_m = 2.0;
    world.create_spring(spring);
    for (int step = 0; step < 60; ++step)
        world.step(1.0 / 120.0);

    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.world = &world;
    const auto energy_rows = rows(panel, model, view);
    const auto readout = [&](std::string_view label) -> const ui::PanelRow*
    {
        for (const auto* row : rows_with(energy_rows, ui::PanelRowKind::readout))
            if (row->text == label)
                return row;
        return nullptr;
    };
    const auto* damper_row = readout("Lost in dampers");
    RIGIDBODIES_EXPECT(damper_row && damper_row->value != "0.00\xC2\xA0J", "the damper's loss is listed with the other losses");
    const auto meters = rows_with(energy_rows, ui::PanelRowKind::meter);
    int percent_total = 0;
    bool damper_share = false;
    for (const auto* meter : meters)
    {
        percent_total += std::stoi(meter->value);
        damper_share = damper_share || (meter->text == "Lost in dampers" && meter->amount > 0.05);
    }
    RIGIDBODIES_EXPECT(damper_share && percent_total == 100, "the energy the damper removed is a share of the same whole");
    RIGIDBODIES_EXPECT(!readout("Lost to air"), "a scene without air lists no air loss");
}
RIGIDBODIES_TEST("energy figures share one unit and precision so they can be added by eye")
{
    using namespace rigidbodies;
    physics::World world;
    physics::BodyDefinition definition;
    definition.position_m = { 0.0, 0.0 };
    definition.linear_velocity_m_s = { 0.5, 0.0 };
    definition.angular_velocity_rad_s = 0.2;
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    definition.colliders.push_back(collider);
    world.create_body(definition);
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.world = &world;
    const auto energy_rows = rows(panel, model, view);
    for (const auto& row : energy_rows)
        if (row.kind == ui::PanelRowKind::readout && (row.text.rfind("Kinetic", 0) == 0 || row.text.rfind("Potential", 0) == 0 || row.text == "Mechanical energy"))
            RIGIDBODIES_EXPECT(!row.value.empty() && row.value.back() == 'J' && row.value.find("mJ") == std::string::npos && row.value.find("kJ") == std::string::npos, "every energy row uses joules: " + row.text + " " + row.value);
}

RIGIDBODIES_TEST("the Runs tab acknowledges the run being recorded")
{
    using namespace rigidbodies;
    ui::RunRecord current;
    current.number = 1;
    current.duration_s = 5.25;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.current_run = &current;
    view.set_active_tab("measure.header.tabs", "runs");
    const auto runs = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_text(runs, "Run 1 (this run)") && has_text(runs, "Recording") && tab_label(runs, "runs") == "Runs (1)", "the live run is listed and counted");
    RIGIDBODIES_EXPECT(!has_text(runs, "Play once") && has_text(runs, "Back to start (R)"), "the copy says how a run is kept instead of asking for Play again");
    model.current_run = nullptr;
    RIGIDBODIES_EXPECT(has_text(rows(panel, model, view), "Press Play"), "with no run at all the tab asks for Play");
}

RIGIDBODIES_TEST("the graph names its subject and uses display names for objects")
{
    using namespace rigidbodies;
    physics::World world;
    physics::BodyDefinition definition;
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    definition.colliders.push_back(collider);
    const auto ball = world.create_body(definition);
    ui::RunRecord run;
    run.number = 1;
    run.duration_s = 0.025;
    run.series.time_s = { 0.0f, 0.025f };
    run.series.scene.assign(16, 1.0f);
    run.series.object_ids = { ball };
    run.series.objects.assign(28, 1.0f);
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.world = &world;
    model.current_run = &run;
    ui::ObjectItem item;
    item.id = ball;
    item.display_name = "Steel ball";
    item.kind = "free";
    model.objects = { item };
    view.set_active_tab("measure.header.tabs", "graph");
    auto graph = rows(panel, model, view);
    const auto plot = rows_with(graph, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plot.size() == 1 && plot.front()->plot && plot.front()->plot->subject == "Whole scene (nothing selected)", "Follow selection says when it falls back to the whole scene");
    const auto scope = rows_with(graph, ui::PanelRowKind::select, "measure.graph.scope");
    RIGIDBODIES_EXPECT(scope.size() == 1 && scope.front()->spec && std::any_of(scope.front()->spec->options.begin(), scope.front()->spec->options.end(), [](const auto& option)
                                                                       {
                                                                           return option.label == "Steel ball";
                                                                       }),
        "Scope offers objects by the names the stage shows");
    model.selection = ball;
    graph = rows(panel, model, view);
    const auto selected_plot = rows_with(graph, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(selected_plot.size() == 1 && selected_plot.front()->plot && selected_plot.front()->plot->subject == "Steel ball", "a followed object is named after the quantity");

    view.set_checklist("measure.graph.quantities", { "potential_height", "kinetic_moving" });
    graph = rows(panel, model, view);
    const auto colours = rows_with(graph, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(colours.size() == 1 && colours.front()->plot && colours.front()->plot->series.size() >= 2 && colours.front()->plot->series[0].slot == 2 && colours.front()->plot->series[1].slot == 0, "potential is drawn green and kinetic blue, matching the energy shares");
}

RIGIDBODIES_TEST("the graph plots the budget's other rows and marks only the plotted objects' impacts")
{
    using namespace rigidbodies;
    physics::WorldSettings settings;
    settings.gravity_m_s2 = {};
    physics::World world(settings);
    physics::BodyDefinition anchor_definition;
    anchor_definition.type = physics::BodyType::static_body;
    const auto anchor = world.create_body(anchor_definition);
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    physics::BodyDefinition mass_definition;
    mass_definition.position_m = { 1.5, 0.0 };
    mass_definition.colliders.push_back(collider);
    const auto mass = world.create_body(mass_definition);
    mass_definition.position_m = { -1.5, 0.0 };
    const auto other = world.create_body(mass_definition);
    physics::LinearSpringDefinition spring;
    spring.first = anchor;
    spring.second = mass;
    spring.rest_length_m = 1.0;
    spring.stiffness_n_m = 20.0;
    spring.damping_n_s_m = 2.0;
    world.create_spring(spring);

    ui::RunRecord run;
    run.number = 1;
    run.duration_s = 2.0;
    run.series.time_s = { 0.0f, 1.0f, 2.0f };
    run.series.object_ids = { mass, other };
    run.series.scene.assign(3 * 8, 0.0f);
    run.series.ledger = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f };
    run.series.objects.assign(3 * 2 * 14, 0.0f);
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.world = &world;
    model.current_run = &run;
    for (std::size_t index = 0; index < 2; ++index)
    {
        ui::ImpactItem item;
        item.index = index;
        item.time_s = 0.5 + static_cast<double>(index);
        item.first = index == 0 ? mass : other;
        item.second = anchor;
        model.impacts.push_back(item);
    }
    view.set_active_tab("measure.header.tabs", "graph");
    view.set_value("measure.graph.scope", "scene");
    view.set_checklist("measure.graph.quantities", { "lost_dampers", "lost_air" });
    auto graph = rows(panel, model, view);
    const auto quantities = rows_with(graph, ui::PanelRowKind::checklist, "measure.graph.quantities");
    const auto offers = [&](std::string_view key)
    {
        return quantities.size() == 1 && std::any_of(quantities.front()->options.begin(), quantities.front()->options.end(), [&](const auto& option)
                                             {
                                                 return option.id == key;
                                             });
    };
    RIGIDBODIES_EXPECT(offers("lost_dampers") && offers("lost_joints") == false && offers("lost_air") == false, "the graph offers the losses the scene can have, as the Energy tab lists them");
    auto plot = rows_with(graph, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plot.size() == 1 && plot.front()->plot && plot.front()->plot->series.size() == 1 && plot.front()->plot->series[0].label == "Lost in dampers" && plot.front()->plot->series[0].values[2] == 0.8f, "the damper's loss is plotted from the recorded budget");
    const auto impact_markers = [](const ui::PlotData& data)
    {
        return std::count_if(data.markers.begin(), data.markers.end(), [](const auto& marker)
            {
                return marker.kind == ui::PlotMarkerKind::impact;
            });
    };
    RIGIDBODIES_EXPECT(plot.size() == 1 && impact_markers(*plot.front()->plot) == 2, "the whole scene marks every impact");
    view.set_value("measure.graph.scope", "selection");
    model.selection = mass;
    graph = rows(panel, model, view);
    plot = rows_with(graph, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plot.size() == 1 && impact_markers(*plot.front()->plot) == 1, "a plotted object shows only its own impacts");
}

int main()
{
    return rigidbodies::testing::run_all();
}
