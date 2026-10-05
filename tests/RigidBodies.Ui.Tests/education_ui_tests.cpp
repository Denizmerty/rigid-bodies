#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/shape.hpp>

#include "relativity_fixture.hpp"
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

RIGIDBODIES_TEST("saved graphs retain object scope and reviewing a run uses that run's time and data")
{
    using namespace rigidbodies;
    const physics::BodyId body { 3, 1 };
    ui::RunRecord saved;
    saved.number = 1;
    saved.duration_s = 80.0;
    saved.series.time_s = { 70.0f, 75.0f, 80.0f };
    saved.series.scene.assign(24, 1.0f);
    saved.series.object_ids = { body };
    saved.series.objects.assign(42, 7.0f);
    std::vector<ui::RunRecord> kept { saved };
    ui::UiModel model;
    model.runs = kept;
    model.previous_run = &kept.front();
    model.objects = { { body, {}, "Steel ball", "free" } };
    ui::MeasurePanel panel;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "graph");
    view.set_value("measure.graph.scope", "3:1");
    auto built = rows(panel, model, view);
    auto plots = rows_with(built, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plots.size() == 1 && plots.front()->plot->subject == "Steel ball", "Back to start keeps the recorded object as the graph subject");
    RIGIDBODIES_EXPECT(plots.front()->plot->series.front().values[0] == 7.0f, "saved object scope plots object data rather than silently summing the whole scene");
    const auto scopes = rows_with(built, ui::PanelRowKind::select, "measure.graph.scope");
    RIGIDBODIES_EXPECT(std::any_of(scopes.front()->spec->options.begin(), scopes.front()->spec->options.end(), [](const auto& option)
                           {
                               return option.id == "3:1";
                           }),
        "recorded objects remain selectable without an active run");
    model.previous_run = nullptr;
    view.set_value("measure.graph.compare_with", "1");
    built = rows(panel, model, view);
    plots = rows_with(built, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plots.front()->plot->x.maximum == 80.0 && plots.front()->plot->x.minimum == 70.0, "a historical-only comparison uses its recorded interval instead of an empty 0–10 s window");
    ui::RunRecord live = saved;
    live.number = 2;
    live.duration_s = 1.0;
    model.current_run = &live;
    view.set_value("measure.graph.review_run", "1");
    built = rows(panel, model, view);
    plots = rows_with(built, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(plots.front()->plot->x.maximum == 80.0 && plots.front()->plot->series.size() == 1 && plots.front()->plot->series.front().source == "Run 1", "reviewing Run 1 is independent of the new recording and never duplicates its comparison series");
    RIGIDBODIES_EXPECT(has_key(built, "measure.graph.live"), "review has an explicit way back to the live graph");
    view.set_value("measure.graph.scope", "9:1");
    built = rows(panel, model, view);
    plots = rows_with(built, ui::PanelRowKind::plot);
    RIGIDBODIES_EXPECT(!std::isfinite(plots.front()->plot->series.front().values[0]), "an absent explicitly chosen object never substitutes whole-scene values");
}

RIGIDBODIES_TEST("Add value can target objects before recording and validates scope and exact time")
{
    using namespace rigidbodies;
    physics::World world;
    physics::BodyDefinition definition;
    physics::Collider collider;
    collider.shape = physics::make_circle(0.1);
    definition.colliders.push_back(collider);
    const auto body = world.create_body(definition);
    ui::UiModel model;
    model.world = &world;
    model.objects = { { body, {}, "Steel ball", "free" } };
    ui::MeasurePanel panel;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "runs");
    view.set_value("measure.runs.add_open", "true");
    view.set_value("measure.runs.add_quantity", "speed");
    auto built = rows(panel, model, view);
    auto add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
    RIGIDBODIES_EXPECT(!add.front()->disabled_reason.empty() && has_text(built, "Choose an object to measure its speed"), "whole-scene Speed explains its required object");
    view.set_value("measure.runs.add_object", std::to_string(body.index) + ":" + std::to_string(body.generation));
    view.set_value("measure.runs.add_aggregator", "at_time");
    for (const auto* invalid : { "1oops", "NaN", "-1", "61", "" })
    {
        view.set_value("measure.runs.add_time", invalid);
        built = rows(panel, model, view);
        add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
        RIGIDBODIES_EXPECT(!add.front()->disabled_reason.empty(), "malformed and out-of-range times cannot silently become different measurements");
    }
    view.set_value("measure.runs.add_time", "0.25 s");
    built = rows(panel, model, view);
    add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
    RIGIDBODIES_EXPECT(add.front()->disabled_reason.empty() && add.front()->command.body == body && add.front()->command.value == 0.25, "a valid object measurement can be prepared before the first run");
}

RIGIDBODIES_TEST("one kept run shows scoped pinned results and comparison percentages use percent units")
{
    using namespace rigidbodies;
    const physics::BodyId body { 3, 1 };
    std::vector<ui::PinnedValue> pinned { { "speed:1", "speed", body, ui::RunAggregator::at_time, 1.5 } };
    std::vector<ui::RunRecord> kept(1);
    kept[0].number = 1;
    kept[0].pinned_results = { 2.0 };
    ui::UiModel model;
    model.objects = { { body, {}, "Steel ball", "free" } };
    model.runs = kept;
    model.pinned_values = pinned;
    ui::MeasurePanel panel;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "runs");
    auto built = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_key(built, "measure.runs.inspect_graph") && has_text(built, "Steel ball") && has_text(built, "At " + core::format_quantity(1.5, core::DisplayQuantity::time, model.display_units)) && has_text(built, core::format_quantity(2.0, core::DisplayQuantity::velocity, model.display_units)), "one kept run already exposes its measured value, object, requested time and unit");
    kept.push_back(kept.front());
    kept.back().number = 2;
    kept.back().pinned_results = { 3.0 };
    model.runs = kept;
    built = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_text(built, core::format_quantity(50.0, core::DisplayQuantity::percentage, model.display_units)), "an increase from 2 to 3 is shown as 50 percent, not 0.5 percent");
    model.display_units = core::DisplayUnits::centimetre_gram;
    built = rows(panel, model, view);
    RIGIDBODIES_EXPECT(has_text(built, "cm/s"), "pinned and comparison values honor the selected display units");
}

namespace
{
    using namespace rigidbodies;

    std::vector<std::string_view> tab_ids(const std::vector<ui::PanelRow>& rows)
    {
        std::vector<std::string_view> result;
        for (const auto* tabs : rows_with(rows, ui::PanelRowKind::tabs, "measure.header.tabs"))
            for (const auto& option : tabs->options)
                result.push_back(option.id);
        return result;
    }

    std::string selected_tab(const std::vector<ui::PanelRow>& rows)
    {
        const auto tabs = rows_with(rows, ui::PanelRowKind::tabs, "measure.header.tabs");
        return tabs.empty() ? std::string {} : tabs.front()->selected_option;
    }

    const ui::PanelRow* row_with_key(const std::vector<ui::PanelRow>& rows, std::string_view key, std::string_view instance = {})
    {
        const auto found = std::find_if(rows.begin(), rows.end(), [&](const auto& row)
            {
                return row.key == key && row.instance == instance;
            });
        return found == rows.end() ? nullptr : &*found;
    }

    // An action row by key alone: an action keeps its command's id as its instance.
    const ui::PanelRow* action_with_key(const std::vector<ui::PanelRow>& rows, std::string_view key)
    {
        const auto found = std::find_if(rows.begin(), rows.end(), [&](const auto& row)
            {
                return row.key == key;
            });
        return found == rows.end() ? nullptr : &*found;
    }

    const ui::PlotData* relativity_plot(const std::vector<ui::PanelRow>& rows)
    {
        const auto* row = row_with_key(rows, "measure.relativity.plot");
        return row && row->plot ? row->plot.get() : nullptr;
    }
}

RIGIDBODIES_TEST("relativity experiments offer Relativity, Graph and Runs")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    model.relativity = testing::relativity_model_at(0.5, 2.0e-9);
    ui::ViewState view;
    auto built = rows(panel, model, view);
    const std::vector<std::string_view> expected { "relativity", "graph", "runs" };
    RIGIDBODIES_EXPECT(tab_ids(built) == expected, "the relativity tab set is Relativity, Graph and Runs, in that order");
    RIGIDBODIES_EXPECT(selected_tab(built) == "relativity" && has_key(built, "measure.relativity.plot"), "Relativity is the default tab and builds its own rows");
    RIGIDBODIES_EXPECT(tab_label(built, "relativity") == "Relativity" && tab_label(built, "graph") == "Graph" && tab_label(built, "runs") == "Runs (0)", "the tabs read Relativity, Graph and Runs (n)");
    for (const auto* stored : { "energy", "collisions", "theory" })
    {
        view.set_active_tab("measure.header.tabs", stored);
        built = rows(panel, model, view);
        RIGIDBODIES_EXPECT(selected_tab(built) == "relativity" && !has_key(built, "measure.theory.integration_run") && !has_key(built, "measure.collisions.pause_each") && !has_key(built, "measure.energy.scope"), std::string("a stored Newtonian tab falls back to Relativity: ") + stored);
    }
}

RIGIDBODIES_TEST("Newtonian experiments never offer Relativity")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "relativity");
    const auto built = rows(panel, model, view);
    const std::vector<std::string_view> expected { "energy", "graph", "collisions", "runs", "theory" };
    RIGIDBODIES_EXPECT(tab_ids(built) == expected, "the Newtonian tab set is unchanged");
    RIGIDBODIES_EXPECT(selected_tab(built) == "energy" && !has_key(built, "measure.relativity.plot") && !has_key(built, "world.relativity.speed"), "a stored Relativity tab falls back to Energy");
}

RIGIDBODIES_TEST("the Relativity tab lists every required readout")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    model.relativity = testing::relativity_model_at(0.99999, 12.45e-9);
    ui::ViewState view;
    const auto built = rows(panel, model, view);
    // v, v/c, γ, K, p, m, t, τ and t − τ, each under its own key.
    for (const auto* key : { "measure.relativity.speed", "measure.relativity.fraction", "measure.relativity.gamma", "measure.relativity.energy", "measure.relativity.momentum", "measure.relativity.rest_mass", "measure.relativity.lab_clock", "measure.relativity.probe_clock", "measure.relativity.clock_gap" })
        RIGIDBODIES_EXPECT(row_with_key(built, key) != nullptr, std::string("the Relativity tab shows ") + key);
    const auto value = [&](std::string_view key)
    {
        const auto* row = row_with_key(built, key);
        return row ? row->value : std::string {};
    };
    const auto units = model.display_units;
    RIGIDBODIES_EXPECT(value("measure.relativity.fraction") == "0.99999", "v/c reads its nines with no unit");
    RIGIDBODIES_EXPECT(value("measure.relativity.speed") == core::format_quantity(model.relativity->speed_m_s, core::DisplayQuantity::relativistic_speed, units) && value("measure.relativity.speed").find("299\xE2\x80\x89"
                                                                                                                                                                                                           "789\xE2\x80\x89"
                                                                                                                                                                                                           "460") == 0,
        "v reads 299 789 460 m/s");
    RIGIDBODIES_EXPECT(value("measure.relativity.below_c") == core::format_quantity(model.relativity->below_light_m_s, core::DisplayQuantity::speed_gap, units), "the gap below c is shown beside v");
    RIGIDBODIES_EXPECT(value("measure.relativity.gamma") == "223.6", "γ reads 223.6");
    RIGIDBODIES_EXPECT(value("measure.relativity.energy") == core::format_quantity(model.relativity->kinetic_energy_j, core::DisplayQuantity::relativistic_energy, units) && value("measure.relativity.energy_ratio") == "445\xC3\x97", "K and its ratio to Newton's ½mv² are shown");
    RIGIDBODIES_EXPECT(value("measure.relativity.lab_clock") == "12.45\xC2\xA0ns" && value("measure.relativity.clock_rate") == "0.00447", "the lab clock and the clock rate read in ns and as a fraction");
    for (const auto* key : { "measure.relativity.lab_clock", "measure.relativity.probe_clock", "measure.relativity.clock_gap" })
        RIGIDBODIES_EXPECT(row_with_key(built, key)->live, std::string("a clock reading refreshes with the live readings: ") + key);
    for (const auto* key : { "measure.relativity.fraction", "measure.relativity.speed", "measure.relativity.gamma", "measure.relativity.energy", "measure.relativity.momentum" })
        RIGIDBODIES_EXPECT(!row_with_key(built, key)->live, std::string("a reading that follows the speed changes with the edit, in the plot's frame: ") + key);
    RIGIDBODIES_EXPECT(row_with_key(built, "measure.relativity.clock_gap")->presentation.emphasis == ui::RowEmphasis::primary, "the clock gap reads as the result of the clocks above it");
    const auto* speed = row_with_key(built, "world.relativity.speed", "measure");
    const auto* preset = row_with_key(built, "world.relativity.preset", "measure");
    RIGIDBODIES_EXPECT(speed && speed->kind == ui::PanelRowKind::number && speed->number_si == model.relativity->speed_fraction, "the tab carries its own copy of the speed field");
    RIGIDBODIES_EXPECT(preset && preset->selected_option == "0.99999" && preset->command.value == model.relativity->speed_fraction, "the preset chips mark 0.99999 c, and the row itself sends the current speed");
    RIGIDBODIES_EXPECT(row_with_key(built, "measure.relativity.curve") && row_with_key(built, "measure.relativity.range"), "the curve and range choosers are on the tab");

    model.relativity = testing::relativity_model_at(0.0, 0.0);
    const auto at_rest = rows(panel, model, view);
    const auto rest_value = [&](std::string_view key)
    {
        const auto* row = row_with_key(at_rest, key);
        return row ? row->value : std::string {};
    };
    RIGIDBODIES_EXPECT(rest_value("measure.relativity.gamma") == "1" && rest_value("measure.relativity.energy_ratio") == "\xE2\x80\x94" && rest_value("measure.relativity.clock_rate") == "1.00", "at rest γ is 1, the ratio has no value and the clock rate is 1.00");
}

RIGIDBODIES_TEST("the Relativity plot keeps fixed ranges and explains off-scale values")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.relativity = testing::relativity_model_at(0.5, 0.0);
    auto built = rows(panel, model, view);
    const auto* plot = relativity_plot(built);
    RIGIDBODIES_EXPECT(plot != nullptr && plot->series.size() == 2, "the plot has the relativistic and the Newtonian curve");
    if (!plot || plot->series.size() != 2)
        return;
    const auto* samples = plot->series.front().values;
    const auto count = plot->series.front().count;
    RIGIDBODIES_EXPECT(plot->x.minimum == 0.0 && plot->x.maximum == 1.0 && plot->y.minimum == 0.0 && plot->y.maximum == 5.0 && plot->y.scale == ui::AxisScale::linear && !plot->y.ticks.empty(), "Up to c plots kinetic energy from 0 to 5 mc² against v/c from 0 to c");
    RIGIDBODIES_EXPECT(plot->markers.size() == 1 && plot->markers.front().kind == ui::PlotMarkerKind::limit && plot->markers.front().time_s == 1.0 && plot->markers.front().label == "c", "c is a limit at the end of the axis");
    RIGIDBODIES_EXPECT(plot->operating_x && *plot->operating_x == 0.5 && plot->x.symbol == "v/c" && plot->x.readout == ui::AxisReadout::speed_fraction, "the operating point is at v/c and the read-out names v/c");
    RIGIDBODIES_EXPECT(plot->series.front().source == "Relativity" && plot->series.back().source == "Newton" && plot->series.back().style == ui::SeriesStyle::dashed, "the solid curve is relativity and the dashed one Newton");
    RIGIDBODIES_EXPECT(plot->series.front().times[count - 1] < 1.0f, "no sample lies on c");
    for (const auto& tick : plot->x.ticks)
        RIGIDBODIES_EXPECT(tick.value < 1.0 && tick.label != "1\xC2\xA0"
                                                             "c",
            "no tick names c itself: " + tick.label);
    RIGIDBODIES_EXPECT(has_text(built, "For this probe, mc\xC2\xB2 is " + core::format_quantity(model.relativity->rest_energy_j, core::DisplayQuantity::relativistic_energy, model.display_units) + ".") && !has_text(built, "Off the chart"), "inside the range the note explains the two curves");
    // A marked sample survives only while the panel keeps its samples; sampling again overwrites it.
    constexpr float marked = -12345.0f;
    const_cast<float*>(samples)[count / 2] = marked;

    model.relativity = testing::relativity_model_at(0.99999, 0.0);
    built = rows(panel, model, view);
    plot = relativity_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->y.maximum == 5.0 && plot->x.maximum == 1.0 && plot->operating_x && *plot->operating_x == 0.99999, "the ranges stay fixed as the speed nears c");
    RIGIDBODIES_EXPECT(plot && plot->series.front().values == samples && plot->series.front().count == count && plot->series.front().values[count / 2] == marked, "a new speed keeps the curve samples; only the operating point moves");
    RIGIDBODIES_EXPECT(has_text(built, "Off the chart: K = 223\xC2\xA0mc\xC2\xB2. Choose Near c to follow it."), "a value above the range is named and Near c offered");
    RIGIDBODIES_EXPECT(has_text(built, "At this scale 0.99999\xC2\xA0"
                                       "c looks like c, but it is still " +
                               core::format_quantity(model.relativity->below_light_m_s, core::DisplayQuantity::speed_gap, model.display_units) + " below c."),
        "near c the note says the point only looks like c");

    view.set_value("measure.relativity.curve", "gamma");
    built = rows(panel, model, view);
    plot = relativity_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->y.maximum == 10.0 && plot->series.front().values != nullptr && has_text(built, "Off the chart: \xCE\xB3 = 223.6."), "the Lorentz factor curve has its own fixed range and reads γ off the chart");
    RIGIDBODIES_EXPECT(plot && plot->series.front().count > count / 2 && plot->series.front().values[count / 2] != marked, "a new curve is sampled afresh");
    if (plot && plot->series.front().count > count / 2)
        const_cast<float*>(plot->series.front().values)[count / 2] = marked;

    view.set_value("measure.relativity.range", "near");
    built = rows(panel, model, view);
    plot = relativity_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->series.front().count > count / 2 && plot->series.front().values[count / 2] != marked, "a new range is sampled afresh");
    view.set_value("measure.relativity.curve", "energy");
    built = rows(panel, model, view);
    plot = relativity_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->y.scale == ui::AxisScale::log10 && plot->y.minimum == 1.0e-3 && plot->y.maximum == 1.0e4, "Near c plots energy on a log axis from 10⁻³ to 10⁴");
    RIGIDBODIES_EXPECT(plot && plot->x.maximum == physics::maximum_rapidity() && plot->markers.empty() && plot->x.readout == ui::AxisReadout::rapidity_speed_fraction, "Near c ends at the fastest speed, with no c on the axis");
    RIGIDBODIES_EXPECT(plot && plot->operating_x && *plot->operating_x == model.relativity->rapidity && *plot->operating_x < plot->x.maximum, "the operating point is the probe's rapidity, inside the axis");
    RIGIDBODIES_EXPECT(has_text(built, "Each step right adds a 9 to the speed.") && !has_text(built, "Off the chart"), "Near c explains its axis and the point stays on the chart");
    if (plot)
        for (const auto& tick : plot->x.ticks)
            RIGIDBODIES_EXPECT(tick.label.rfind("1", 0) != 0, "no Near c tick reads as c: " + tick.label);
}

RIGIDBODIES_TEST("relativity readings offer only the actions that apply to them")
{
    using namespace rigidbodies;
    ui::ContextMenuPanel panel;
    ui::UiModel model;
    model.relativity = testing::relativity_model_at(0.9, 3.0e-9);
    ui::ViewState view;
    view.set_value("context.kind", "readout");
    const auto actions = [&](std::string_view key)
    {
        view.set_value("context.key", key);
        view.set_value("context.value", "1.00");
        return rows(panel, model, view);
    };
    auto built = actions("measure.relativity.probe_clock");
    const auto* plot = action_with_key(built, "context.readout.plot");
    const auto* pin = action_with_key(built, "context.readout.pin");
    const auto* revert = action_with_key(built, "context.readout.revert");
    RIGIDBODIES_EXPECT(plot && plot->disabled_reason.empty() && plot->command.detail == "plot-clock:probe_clock", "a clock reading is added to the Graph's clocks and plotted over lab time");
    RIGIDBODIES_EXPECT(pin && pin->disabled_reason.empty() && pin->command.kind == ui::UiCommandKind::pin_run_value && pin->command.id == "probe_clock", "a clock reading is pinned by its run channel");
    RIGIDBODIES_EXPECT(revert && revert->disabled_reason == "Only a setting can be reverted.", "a reading cannot be reverted");
    built = actions("measure.relativity.gamma");
    RIGIDBODIES_EXPECT(action_with_key(built, "context.readout.pin")->command.id == "lorentz", "γ is pinned as the recorded Lorentz factor");
    RIGIDBODIES_EXPECT(action_with_key(built, "context.readout.plot")->command.detail == "plot-clock:lorentz", "γ is plotted as the recorded Lorentz factor");
    // The status line's and the Present strip's lab time are the lab clock Measure lists.
    for (const auto* key : { "bar.time.readout", "present.time" })
    {
        view.set_value("context.key", key);
        view.set_value("context.value", "Lab time 3.00Â ns");
        built = rows(panel, model, view);
        const auto* time_plot = action_with_key(built, "context.readout.plot");
        const auto* time_pin = action_with_key(built, "context.readout.pin");
        const auto* time_copy = action_with_key(built, "context.readout.copy");
        const auto* time_revert = action_with_key(built, "context.readout.revert");
        RIGIDBODIES_EXPECT(time_plot && time_plot->disabled_reason.empty() && time_plot->command.detail == "plot-clock:lab_clock", std::string("the lab time is plotted as the lab clock: ") + key);
        RIGIDBODIES_EXPECT(time_pin && time_pin->disabled_reason.empty() && time_pin->command.id == "lab_clock", std::string("the lab time is pinned as the lab clock: ") + key);
        RIGIDBODIES_EXPECT(time_copy && time_copy->command.detail == "copy-value:" + ui::now_text(model), std::string("the lab time copies as the time alone: ") + key);
        RIGIDBODIES_EXPECT(time_revert && time_revert->disabled_reason == "Only a setting can be reverted.", std::string("the lab time cannot be reverted: ") + key);
    }
    built = actions("measure.relativity.energy");
    RIGIDBODIES_EXPECT(action_with_key(built, "context.readout.plot")->disabled_reason == "This value depends only on the speed. See the Relativity graph." && action_with_key(built, "context.readout.pin")->disabled_reason == "Only values recorded over time can be added.", "a reading that follows the speed alone points to the Relativity graph");
    for (const auto* key : { "world.relativity.rest_mass", "world.relativity.light_speed", "measure.relativity.rest_mass", "measure.relativity.rest_energy" })
    {
        built = actions(key);
        RIGIDBODIES_EXPECT(action_with_key(built, "context.readout.plot")->disabled_reason == "This value is the same at every speed." && action_with_key(built, "context.readout.pin")->disabled_reason == "Only values recorded over time can be added.", std::string("a constant reading is never said to depend on the speed: ") + key);
    }
    built = actions("world.relativity.speed");
    RIGIDBODIES_EXPECT(action_with_key(built, "context.readout.revert")->disabled_reason.empty() && action_with_key(built, "context.readout.revert")->command.id == "world.relativity.speed", "the probe's speed can be reverted");

    view.set_value("context.kind", "graph");
    view.set_value("context.key", "measure.relativity.plot");
    built = rows(panel, model, view);
    const auto* range = action_with_key(built, "context.graph.range");
    RIGIDBODIES_EXPECT(range && range->text == "Show near c" && range->command.detail == "state:measure.relativity.range=near" && !action_with_key(built, "context.graph.clear"), "the Relativity plot offers the other speed range instead of Clear graph");
    view.set_value("measure.relativity.range", "near");
    built = rows(panel, model, view);
    RIGIDBODIES_EXPECT(action_with_key(built, "context.graph.range") && action_with_key(built, "context.graph.range")->text == "Show up to c", "near c it offers the scale up to c");
}

RIGIDBODIES_TEST("a guide never offers a control of the other kind of experiment")
{
    using namespace rigidbodies;
    ui::GuidePanel panel;
    ui::ViewState view;
    for (const auto relativity : { false, true })
    {
        ui::UiModel model;
        if (relativity)
            model.relativity = testing::relativity_model_at(0.5, 0.0);
        ui::ExperimentContent content;
        content.title = "Hand-made";
        content.guide.variables = { { "world.gravity.strength", {}, {}, {} }, { "world.relativity.speed", {}, {}, {} } };
        content.guide.steps = { { "Change gravity.", "world.gravity.strength", {}, {}, {} }, { "Change the speed.", "world.relativity.speed", {}, {}, {} } };
        model.scenario_content = content;
        const auto built = rows(panel, model, view);
        const auto own = relativity ? "world.relativity.speed" : "world.gravity.strength";
        const auto other = relativity ? "world.gravity.strength" : "world.relativity.speed";
        RIGIDBODIES_EXPECT(std::any_of(built.begin(), built.end(), [&](const ui::PanelRow& row)
                               {
                                   return row.key == own;
                               }) &&
                std::none_of(built.begin(), built.end(), [&](const ui::PanelRow& row)
                    {
                        return row.key == other;
                    }),
            std::string("the guide shows its experiment's control and not the other's: ") + own);
        RIGIDBODIES_EXPECT(has_text(built, "This control does not apply to this experiment."), "the other control is named as not applying");
        std::size_t disabled = 0;
        for (const auto& row : built)
            if (row.key == "guide.steps.show_me" && row.disabled_reason == "This control does not apply to this experiment.")
                ++disabled;
        RIGIDBODIES_EXPECT(disabled == 1, "the other control's Show me is disabled with that reason");
    }
}

RIGIDBODIES_TEST("Newtonian read-outs offer neither the runs table nor Revert")
{
    using namespace rigidbodies;
    ui::ContextMenuPanel panel;
    ui::UiModel model;
    ui::ViewState view;
    view.set_value("context.kind", "readout");
    for (const auto* key : { "bar.time.readout", "present.time", "camera.scale.height" })
    {
        view.set_value("context.key", key);
        view.set_value("context.value", "1.00Â s");
        const auto built = rows(panel, model, view);
        const auto* pin = action_with_key(built, "context.readout.pin");
        const auto* revert = action_with_key(built, "context.readout.revert");
        const auto* copy = action_with_key(built, "context.readout.copy");
        RIGIDBODIES_EXPECT(pin && pin->disabled_reason == "Only values recorded over time can be added.", std::string("a read-out is not offered to the runs table: ") + key);
        RIGIDBODIES_EXPECT(revert && revert->disabled_reason == "Only a setting can be reverted.", std::string("a read-out is not offered Revert: ") + key);
        RIGIDBODIES_EXPECT(copy && copy->disabled_reason.empty() && copy->command.detail == "copy-value:1.00Â s", std::string("a read-out can still be copied: ") + key);
    }
}

RIGIDBODIES_TEST("each sample up to c is worked out at the speed its axis stores")
{
    using namespace rigidbodies;
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.relativity = testing::relativity_model_at(0.9999999, 0.0);
    for (const auto* curve : { "gamma", "energy", "momentum", "clock_rate" })
    {
        view.set_value("measure.relativity.curve", curve);
        const auto built = rows(panel, model, view);
        const auto* plot = relativity_plot(built);
        RIGIDBODIES_EXPECT(plot && plot->series.size() == 2 && plot->series.front().count > 350, std::string("the curve is sampled: ") + curve);
        if (!plot || plot->series.empty())
            continue;
        const auto& solid = plot->series.front();
        for (std::size_t index = 0; index < solid.count; ++index)
        {
            // Near c the stored float is up to 6 × 10⁻⁸ from the gap the sample was asked for; its
            // value must belong to the float, which is the speed a read-out names.
            const auto factors = physics::lorentz_factors_from_gap(1.0 - static_cast<double>(solid.times[index]));
            const auto expected = !factors              ? 0.0
                : std::string_view(curve) == "gamma"    ? factors->lorentz_factor
                : std::string_view(curve) == "momentum" ? factors->speed_fraction_times_lorentz_factor
                : std::string_view(curve) == "energy"   ? factors->lorentz_factor_minus_one
                                                        : factors->inverse_lorentz_factor;
            if (solid.times[index] <= 0.0f)
                continue;
            RIGIDBODIES_EXPECT(factors && std::abs(static_cast<double>(solid.values[index]) - expected) <= 1.0e-5 * expected, std::string("a sample's value is the value at its own speed: ") + curve + " at v/c = " + std::to_string(solid.times[index]));
        }
    }
}

RIGIDBODIES_TEST("the Library names Special relativity where no heading does")
{
    using namespace rigidbodies;
    ui::LibraryPanel panel;
    ui::UiModel model;
    model.catalogue.push_back({ "chasing_light", "Chasing light", "Summary.", "Special relativity", "further", "A hook.", 1, 250, { "speed of light" }, {}, {}, false });
    model.catalogue.back().special_relativity = true;
    model.catalogue.push_back({ "free_fall", "Free fall", "Summary.", "Motion and gravity", "intro", "A hook.", 1, 10, { "gravity" }, {}, {}, false });
    model.relativity = testing::relativity_model_at(0.0, 0.0);
    ui::ViewState view;
    const auto badges = [&](std::string_view id)
    {
        for (const auto& row : rows(panel, model, view))
            if (row.key == "library.cards.card" && row.instance == id && row.list_content)
                return row.list_content->badges;
        return std::vector<std::string> {};
    };
    const auto marked = [&](std::string_view id)
    {
        const auto list = badges(id);
        return std::find(list.begin(), list.end(), "Special relativity") != list.end();
    };
    RIGIDBODIES_EXPECT(!marked("chasing_light") && has_text(rows(panel, model, view), "Special relativity"), "sorted by collection, the heading names it and the card does not repeat it");
    view.set_value("library.sort", "suggested");
    RIGIDBODIES_EXPECT(marked("chasing_light") && !marked("free_fall"), "in suggested order the card says Special relativity");
    const auto setups = [&]
    {
        view.set_section_open("library.setups", true);
        const auto built = rows(panel, model, view);
        const auto import = std::find_if(built.begin(), built.end(), [](const auto& row)
            {
                return row.text == "Import shape";
            });
        return import == built.end() ? std::string("missing") : import->disabled_reason;
    };
    RIGIDBODIES_EXPECT(setups() == "Shapes cannot be imported into a relativity experiment.", "Import shape says why it is unavailable here");
}

namespace
{
    using namespace rigidbodies;

    // A run of the relativity experiment as the recorder keeps it: a sample each world second (a lab
    // nanosecond) of the probe clock, the gap and γ − 1 at a constant speed.
    ui::RunRecord relativity_run(int number, double speed_fraction, int seconds)
    {
        const auto factors = *physics::lorentz_factors(speed_fraction);
        ui::RunRecord run;
        run.number = number;
        run.duration_s = seconds;
        for (int second = 0; second <= seconds; ++second)
        {
            const auto lab_s = second * 1.0e-9;
            run.series.time_s.push_back(static_cast<float>(second));
            run.series.relativity.push_back(static_cast<float>(lab_s * factors.inverse_lorentz_factor));
            run.series.relativity.push_back(static_cast<float>(lab_s * factors.clock_lag_rate));
            run.series.relativity.push_back(static_cast<float>(factors.lorentz_factor_minus_one));
        }
        run.series.scene.assign(run.series.time_s.size() * 8, 0.0f);
        run.series.ledger.assign(run.series.time_s.size() * 6, 0.0f);
        return run;
    }

    std::vector<std::string> option_ids(const ui::PanelRow& row)
    {
        std::vector<std::string> result;
        const auto options = !row.options.empty() ? math::Span<const ui::OptionSpec>(row.options) : row.spec ? row.spec->options
                                                                                                             : math::Span<const ui::OptionSpec> {};
        for (const auto& option : options)
            result.emplace_back(option.id);
        return result;
    }

    const ui::PlotData* graph_plot(const std::vector<ui::PanelRow>& rows)
    {
        const auto plots = rows_with(rows, ui::PanelRowKind::plot, "measure.graph.plot");
        return plots.size() == 1 && plots.front()->plot ? plots.front()->plot.get() : nullptr;
    }
}

RIGIDBODIES_TEST("the Graph tab plots clocks against lab time in nanoseconds")
{
    using namespace rigidbodies;
    auto run = relativity_run(2, 0.6, 2);
    ui::MeasurePanel panel;
    ui::UiModel model;
    ui::ViewState view;
    model.relativity = testing::relativity_model_at(0.6, 2.0e-9);
    model.current_run = &run;
    model.graph_markers = { { 1.0, "Speed 0.5\xC2\xA0"
                                   "c \xE2\x86\x92 0.6\xC2\xA0"
                                   "c" } };
    // Nothing collides with the probe, so a stray impact is never marked.
    ui::ImpactItem impact;
    impact.time_s = 0.5;
    model.impacts.push_back(impact);
    view.set_active_tab("measure.header.tabs", "graph");
    view.set_checklist("measure.graph.quantities", { "kinetic_moving" });
    auto built = rows(panel, model, view);

    const auto clocks = rows_with(built, ui::PanelRowKind::checklist, "measure.graph.clocks");
    const std::vector<std::string> clock_ids { "lab_clock", "probe_clock", "clock_gap", "lorentz" };
    RIGIDBODIES_EXPECT(clocks.size() == 1 && option_ids(*clocks.front()) == clock_ids, "the graph offers the lab clock, the probe clock, their gap and γ");
    RIGIDBODIES_EXPECT(!has_key(built, "measure.graph.quantities") && !has_key(built, "measure.graph.scope"), "neither the Newtonian quantities nor an object scope are offered");
    const auto* plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot != nullptr, "the graph is built");
    if (!plot)
        return;
    RIGIDBODIES_EXPECT(plot->x.label == "Lab time" && plot->x.unit == "ns" && plot->x.minimum == 0.0 && plot->x.maximum == 2.0, "the x axis is lab time in nanoseconds: a world second is a lab nanosecond");
    RIGIDBODIES_EXPECT(plot->series.size() == 2 && plot->series[0].label == "Lab clock t" && plot->series[1].label == "Probe clock \xCF\x84" && plot->series[0].unit == "ns" && plot->series[1].unit == "ns", "by default the two clocks are drawn, in nanoseconds");
    if (plot->series.size() == 2)
    {
        RIGIDBODIES_EXPECT(plot->series[0].values[2] == 2.0f && plot->series[0].slot != plot->series[1].slot, "the lab clock is the reference diagonal, in its own colour");
        RIGIDBODIES_EXPECT_NEAR(plot->series[1].values[2], 1.6, 1.0e-6, "the probe clock reads 1.6 ns after 2 ns at 0.6 c");
    }
    RIGIDBODIES_EXPECT(plot->y.label == "Clock reading" && plot->y.unit == "ns" && plot->subject == "Probe and lab clocks", "the value axis reads the clocks");
    RIGIDBODIES_EXPECT(plot->markers.size() == 1 && plot->markers.front().kind == ui::PlotMarkerKind::intervention && plot->markers.front().label.find("Speed") == 0, "speed changes are marked; impacts never are");
    const auto windows = rows_with(built, ui::PanelRowKind::segmented, "measure.graph.window");
    RIGIDBODIES_EXPECT(windows.size() == 1 && windows.front()->spec && windows.front()->spec->options.size() == 3 && windows.front()->spec->options[0].label == "10\xC2\xA0ns" && windows.front()->spec->options[2].label == "60\xC2\xA0ns", "the windows read 10, 30 and 60 ns");

    view.set_checklist("measure.graph.clocks", { "lorentz" });
    built = rows(panel, model, view);
    plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->series.size() == 1 && plot->y.label == "Lorentz factor \xCE\xB3" && plot->y.unit.empty() && plot->series.front().values[1] == 1.25f, "γ alone is drawn as γ, not γ − 1");
    view.set_checklist("measure.graph.clocks", { "clock_gap" });
    built = rows(panel, model, view);
    plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->series.size() == 1 && plot->y.label == "Lab \xE2\x88\x92 probe t \xE2\x88\x92 \xCF\x84" && plot->y.unit == "ns", "the gap alone names its axis");
    if (plot && plot->series.size() == 1)
        RIGIDBODIES_EXPECT_NEAR(plot->series.front().values[2], 0.4, 1.0e-6, "the gap after 2 ns at 0.6 c is 0.4 ns");
    view.set_checklist("measure.graph.clocks", { "probe_clock", "lorentz" });
    built = rows(panel, model, view);
    plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->y.label == "Value" && plot->y.unit.empty(), "a clock with γ shares an axis without a unit");
    RIGIDBODIES_EXPECT(view.checklist("measure.graph.quantities", {}) == std::vector<std::string> { "kinetic_moving" }, "the Newtonian quantity choice is left as it was");

    // A run recorded without the clocks has nothing to draw for them.
    view.set_checklist("measure.graph.clocks", { "probe_clock" });
    auto newtonian = run;
    newtonian.number = 1;
    newtonian.series.relativity.clear();
    model.previous_run = &newtonian;
    built = rows(panel, model, view);
    plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->series.size() == 2 && plot->series[1].source == "Previous run" && plot->series[1].style == ui::SeriesStyle::dashed && std::isnan(plot->series[1].values[0]), "a run without the relativity block draws no clock");

    model.current_run = nullptr;
    model.previous_run = nullptr;
    built = rows(panel, model, view);
    plot = graph_plot(built);
    RIGIDBODIES_EXPECT(plot && plot->empty_title == "Run the experiment to record a graph" && plot->empty_detail == "Press Play. The graph follows the last 10\xC2\xA0ns.", "with no run the graph asks for Play in lab nanoseconds");
}

RIGIDBODIES_TEST("Runs times and pins read in lab nanoseconds")
{
    using namespace rigidbodies;
    std::vector<ui::RunRecord> kept { relativity_run(1, 0.6, 12), relativity_run(2, 0.9, 12) };
    kept[0].duration_s = 12.5;
    kept[0].changes_from_previous = { { "relativity:speed", "world.relativity.speed", {}, "Probe speed", "0\xC2\xA0"
                                                                                                         "c",
        "0.6\xC2\xA0"
        "c",
        ui::EditCategory::parameter } };
    kept[1].series.time_s.front() = 1.0f;
    kept[0].pinned_results = { 4.0e-9, 2.5e-9, 0.25, 12.5e-9 };
    kept[1].pinned_results = { 4.0e-9, 3.0e-9, 1.2941573, 12.0e-9 };
    const std::vector<ui::PinnedValue> pinned { { "a", "probe_clock", {}, ui::RunAggregator::at_time, 5.0 }, { "b", "clock_gap", {}, ui::RunAggregator::at_end, 0.0 }, { "c", "lorentz", {}, ui::RunAggregator::maximum, 0.0 }, { "d", "lab_clock", {}, ui::RunAggregator::at_end, 0.0 } };
    ui::RunRecord current = relativity_run(3, 0.6, 7);
    ui::UiModel model;
    model.relativity = testing::relativity_model_at(0.6, 7.31e-9);
    model.elapsed_time_s = 7.31;
    model.current_run = &current;
    model.runs = kept;
    model.pinned_values = pinned;
    ui::MeasurePanel panel;
    ui::ViewState view;
    view.set_active_tab("measure.header.tabs", "runs");
    auto built = rows(panel, model, view);
    const auto nbsp = std::string("\xC2\xA0");
    RIGIDBODIES_EXPECT(has_text(built, "Recording \xC2\xB7 7.31" + nbsp + "ns"), "the run being recorded reads the lab clock");
    const auto run_rows = rows_with(built, ui::PanelRowKind::list_item, "measure.runs.row");
    RIGIDBODIES_EXPECT(run_rows.size() == 2 && run_rows.front()->hint.find("12.50" + nbsp + "ns \xC2\xB7 Probe speed 0" + nbsp + "c") == 0, "a kept run's length reads in lab nanoseconds, then the speed it was run at");
    RIGIDBODIES_EXPECT(has_text(built, "Values use the retained samples from 1.00" + nbsp + "ns to 12.00" + nbsp + "ns."), "the retained interval reads in lab nanoseconds");
    const auto value_of = [&](std::string_view label) -> std::string
    {
        for (const auto& row : built)
            if (row.kind == ui::PanelRowKind::readout && row.text == label)
                return row.value;
        return "missing";
    };
    RIGIDBODIES_EXPECT(value_of("Probe clock \xCF\x84 \xC2\xB7 At 5.00" + nbsp + "ns") == "4.00" + nbsp + "ns", "a probe clock pinned at a lab time reads in nanoseconds, with no object named");
    RIGIDBODIES_EXPECT(value_of("Lab \xE2\x88\x92 probe t \xE2\x88\x92 \xCF\x84 \xC2\xB7 At end") == "3.00" + nbsp + "ns", "the gap reads as a duration");
    RIGIDBODIES_EXPECT(value_of("Lorentz factor \xCE\xB3 \xC2\xB7 Maximum") == "2.294", "γ is read from the stored γ − 1");
    RIGIDBODIES_EXPECT(value_of("Lab clock t \xC2\xB7 At end") == "12.00" + nbsp + "ns", "the lab clock reads in nanoseconds");
    RIGIDBODIES_EXPECT(!has_text(built, "Whole scene"), "no pinned clock names the whole scene");
    // γ changed from 1.25 to 2.294: a change of 1.04, which is 83.5 % of the first γ (not of γ − 1).
    const auto change = core::format_significant(1.2941573 - 0.25) + " \xC2\xB7 " + core::format_quantity((1.2941573 - 0.25) / 1.25, core::DisplayQuantity::scale, model.display_units);
    // The pinned value's row comes first; the Compare row below it has the same label.
    std::vector<std::string> gamma_values;
    for (const auto& row : built)
        if (row.kind == ui::PanelRowKind::readout && row.text == "Lorentz factor \xCE\xB3 \xC2\xB7 Maximum")
            gamma_values.push_back(row.value);
    RIGIDBODIES_EXPECT(gamma_values.size() == 2 && gamma_values[1] == "1.25 \xC2\xB7 2.294 \xC2\xB7 " + change, "comparing runs gives the change in γ and its share of γ");

    view.set_value("measure.runs.add_open", "true");
    view.set_value("measure.runs.add_quantity", "speed");
    view.set_value("measure.runs.add_aggregator", "at_first_impact");
    built = rows(panel, model, view);
    const auto quantity = rows_with(built, ui::PanelRowKind::select, "measure.runs.add_quantity");
    const std::vector<std::string> clocks { "lab_clock", "probe_clock", "clock_gap", "lorentz" };
    RIGIDBODIES_EXPECT(quantity.size() == 1 && option_ids(*quantity.front()) == clocks && quantity.front()->selected_option == "probe_clock", "Add value offers the clocks and γ, and a Newtonian choice falls back to the probe clock");
    const auto aggregator = rows_with(built, ui::PanelRowKind::select, "measure.runs.add_aggregator");
    const std::vector<std::string> aggregators { "at_end", "maximum", "minimum", "at_time" };
    RIGIDBODIES_EXPECT(aggregator.size() == 1 && option_ids(*aggregator.front()) == aggregators && aggregator.front()->selected_option == "at_end", "nothing collides, so At first impact is not offered");
    RIGIDBODIES_EXPECT(!has_key(built, "measure.runs.add_object"), "no object is chosen for the probe's clocks");
    auto add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
    RIGIDBODIES_EXPECT(add.size() == 1 && add.front()->disabled_reason.empty() && add.front()->command.id == "probe_clock" && add.front()->command.detail == "at_end" && !add.front()->command.body.is_valid(), "the default adds the probe clock at the end of every run");

    view.set_value("measure.runs.add_aggregator", "at_time");
    for (const auto* invalid : { "61", "5 \xC2\xB5s", "-1", "oops", "" })
    {
        view.set_value("measure.runs.add_time", invalid);
        built = rows(panel, model, view);
        add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
        RIGIDBODIES_EXPECT(add.size() == 1 && !add.front()->disabled_reason.empty() && has_text(built, "Enter a lab time from 0 to 60" + nbsp + "ns."), std::string("a lab time outside 0 to 60 ns is refused: ") + invalid);
    }
    const auto field = [&]
    {
        const auto fields = rows_with(built, ui::PanelRowKind::text_field, "measure.runs.add_time");
        return fields.size() == 1 ? fields.front()->text : std::string {};
    };
    RIGIDBODIES_EXPECT(field() == "Lab time (0\xE2\x80\x93" + std::string("60") + nbsp + "ns)", "the time field asks for lab time in nanoseconds");
    for (const auto& [typed, world_s] : { std::pair { "7.5", 7.5 }, std::pair { "7.5 ns", 7.5 }, std::pair { "60", 60.0 }, std::pair { "0.03 \xC2\xB5s", 30.0 } })
    {
        view.set_value("measure.runs.add_time", typed);
        built = rows(panel, model, view);
        add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
        RIGIDBODIES_EXPECT(add.size() == 1 && add.front()->disabled_reason.empty() && add.front()->command.value == world_s && add.front()->command.detail == "at_time", std::string("a lab time is sent in world seconds, a billion times longer: ") + typed);
    }
    view.set_value("measure.runs.add_time", "5");
    built = rows(panel, model, view);
    add = rows_with(built, ui::PanelRowKind::action, "measure.runs.add");
    RIGIDBODIES_EXPECT(add.size() == 1 && add.front()->disabled_reason == "This value is already pinned.", "5 ns is recognised as the probe clock already pinned at 5 ns");
}

int main()
{
    return rigidbodies::testing::run_all();
}
