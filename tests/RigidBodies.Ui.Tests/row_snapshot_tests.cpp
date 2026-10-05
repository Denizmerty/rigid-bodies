#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/core/text_format.hpp>
#include "relativity_fixture.hpp"
#include "test_framework.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

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
            return { 1600, 900 };
        }
        float display_scale() const override
        {
            return 1;
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
            return static_cast<float>(text.size()) * scale;
        }
        float text_line_height(float scale) const override
        {
            return 14 * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    std::string clean(std::string value)
    {
        for (auto& character : value)
            if (character == '\n' || character == '\r' || character == '\t')
                character = ' ';
        return value;
    }

    std::string serialize(std::string_view name, ui::UiModel& model, ui::ViewState& view)
    {
        Device device;
        render::Theme theme;
        render::DrawList draw;
        std::vector<ui::Hotspot> hotspots;
        std::ostringstream out;
        for (auto& panel : ui::create_default_panels())
        {
            std::vector<ui::PanelRow> rows;
            ui::PanelBuilder builder(theme, device, 1, { {}, { 1200, 100000 } }, draw, hotspots, &view);
            builder.record_rows(rows);
            panel->build(model, builder);
            out << "surface=" << panel->id() << " model=" << name << '\n';
            for (const auto& row : rows)
            {
                out << "kind=" << static_cast<int>(row.kind) << "\tkey=" << row.key << "\tinstance=" << row.instance
                    << "\tlabel=" << clean(row.text) << "\tvalue=" << clean(row.value)
                    << "\tunit=" << (row.spec && (row.kind == ui::PanelRowKind::number || row.kind == ui::PanelRowKind::stepper) ? core::display_unit(row.spec->number.quantity, model.display_units) : std::string_view {})
                    << "\ttone=" << static_cast<int>(row.tone) << "\tenabled=" << row.disabled_reason.empty()
                    << "\treason=" << clean(row.disabled_reason) << "\tlive=" << row.live
                    << "\tspec=" << (row.spec ? row.spec->key : std::string_view {})
                    << "\tcommand=" << static_cast<int>(row.command.kind) << ',' << row.command.id << ',' << row.command.value << ',' << row.command.value_y << ',' << row.command.flag
                    << ',' << row.command.body.index << ':' << row.command.body.generation << ",bodies=";
                for (const auto body : row.command.bodies)
                    out << body.index << ':' << body.generation << ';';
                out << ",detail=" << clean(row.command.detail) << ",phase=" << static_cast<int>(row.command.phase);
                if (row.list_content)
                    for (const auto& action : row.list_content->actions)
                        out << "\taction=" << clean(action.label) << ',' << static_cast<int>(action.command.kind) << ',' << action.command.id << ",detail=" << clean(action.command.detail) << ",primary=" << action.primary;
                out << '\n';
            }
        }
        return out.str();
    }

    physics::BodyId body_of_type(const physics::World& world, physics::BodyType type)
    {
        for (const auto id : world.body_ids())
            if (const auto* body = world.find_body(id); body && body->type() == type)
                return id;
        return {};
    }

    void configure(ui::UiModel& model, physics::World& world, ui::ViewState& view, std::string_view model_name)
    {
        model.world = &world;
        model.scenario_title = "Golden fixture";
        model.scenario_id = "free_fall";
        model.layers = render::LayerMask::defaults();
        model.can_undo = true;
        model.can_redo = true;
        model.run_state = ui::RunState::ready;
        for (const auto* section : { "draw.node", "draw.snap", "draw.material", "draw.precision", "world.gravity", "world.air", "world.collisions", "world.advanced", "world.statistics", "show.arrows", "tools.reference" })
            view.set_section_open(section, true);
        if (model_name == "nothing_selected" || model_name == "multiple")
        {
        }
        else if (model_name == "fixed")
            model.selection = body_of_type(world, physics::BodyType::static_body);
        else if (model_name == "driven")
            model.selection = body_of_type(world, physics::BodyType::kinematic_body);
        else
            model.selection = body_of_type(world, physics::BodyType::dynamic_body);
        if (model.selection.is_valid())
            model.selected_bodies.push_back(model.selection);
        if (model_name == "free_running")
        {
            model.run_state = ui::RunState::running;
            model.elapsed_time_s = 3.0;
            view.set_active_tab("inspector.object", "motion");
        }
        if (model_name == "joint" || model_name == "broken_joint")
            view.set_active_tab("inspector.object", "joints");
        if (model_name == "spring")
            view.set_active_tab("inspector.object", "forces");
        if (model_name == "shape_valid" || model_name == "shape_invalid")
        {
            model.shape_editor_active = true;
            model.shape_node_selected = true;
            model.shape_node_count = model_name == "shape_valid" ? 4 : 2;
            model.shape_outline_closed = model_name == "shape_valid";
            model.shape_can_commit = model_name == "shape_valid";
            model.shape_diagnostic = model_name == "shape_valid" ? "Valid outline" : "At least three nodes are required";
        }
        if (model_name == "paused_impact")
        {
            model.run_state = ui::RunState::paused;
            model.impacts.push_back({ 0, 0.5, {}, {}, "A", "B", 1.0, {}, {}, {}, {}, {}, {}, 0.1, 0.1, true });
            view.set_active_tab("measure.header.tabs", "collisions");
        }
        if (model_name == "comparisons")
        {
            model.energy_comparison.resize(1);
            model.restitution_comparison.resize(1);
            model.collision_comparison.resize(1);
            view.set_active_tab("measure.header.tabs", "theory");
        }
        if (model_name == "cgs")
            model.display_units = core::DisplayUnits::centimetre_gram;
        if (model_name == "relativity")
        {
            // The probe at 0.99999 c after 12.45 ns of lab time, on its Relativity tab and World page.
            model.scenario_id = "chasing_light";
            model.scenario_title = "Chasing light";
            model.run_state = ui::RunState::running;
            model.elapsed_time_s = 12.45;
            model.relativity = testing::relativity_model_at(0.99999, 12.45e-9);
            model.changes.push_back({ "relativity:speed", "world.relativity.speed", {}, "Probe speed", "0\xC2\xA0"
                                                                                                       "c",
                "0.99999\xC2\xA0"
                "c",
                ui::EditCategory::parameter });
            view.set_active_tab("measure.header.tabs", "relativity");
            view.set_section_open("world.relativity", true);
        }
        if (model_name == "multiple")
            for (const auto id : world.body_ids())
                if (const auto* body = world.find_body(id); body && body->type() != physics::BodyType::static_body)
                    model.selected_bodies.push_back(id);
        model.selected_shape_authored = true;
        model.authored_part_count = 2;
        model.authored_part_name = "Part 1";
        model.authored_material_name = "steel";
        model.can_export_shape = true;
        model.can_split_shape = true;
        model.can_assemble_shapes = true;
        model.lab_changes.push_back({ "integrator", "Integration method", "Semi-implicit Euler", "Runge–Kutta 4" });
        for (const auto& description : physics::available_scenarios())
            model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
        if (model.relativity)
            for (auto& card : model.catalogue)
                card.special_relativity = card.id == "chasing_light";
    }
}

RIGIDBODIES_TEST("standard model row goldens remain stable")
{
    using namespace rigidbodies;
    const auto fixture_root = std::filesystem::path(RIGIDBODIES_SOURCE_ASSETS).parent_path() / "tests/fixtures/ui";
    const bool update = std::getenv("RIGIDBODIES_UPDATE_GOLDENS") != nullptr;
    const std::vector<std::pair<std::string, std::string>> fixtures {
        { "nothing_selected", "free_fall" }, { "free_ready", "free_fall" }, { "free_running", "free_fall" }, { "fixed", "ramp" }, { "driven", "prescribed_motion" }, { "multiple", "stable_stack" }, { "joint", "revolute_drive" }, { "broken_joint", "breakable_joint" }, { "spring", "spring_damping" }, { "shape_valid", "shape_workshop" }, { "shape_invalid", "shape_workshop" }, { "paused_impact", "collision_comparison" }, { "comparisons", "collision_comparison" }, { "runs", "free_fall" }, { "graph_runs", "free_fall" }, { "cgs", "free_fall" }, { "relativity", "chasing_light" }
    };
    if (update)
        std::filesystem::create_directories(fixture_root);
    for (const auto& [name, scenario] : fixtures)
    {
        physics::World world;
        RIGIDBODIES_EXPECT(physics::load_scenario(world, scenario), "golden scenario loads");
        ui::UiModel model;
        ui::ViewState view;
        configure(model, world, view, name);
        std::vector<ui::RunRecord> runs;
        std::vector<ui::PinnedValue> pinned;
        if (name == "runs" || name == "graph_runs")
        {
            runs.resize(2);
            runs[0].number = 1;
            runs[0].duration_s = 1.0;
            runs[0].starred = true;
            runs[0].prediction = ui::Prediction { "Together", {} };
            runs[1].number = 2;
            runs[1].duration_s = 1.25;
            runs[1].changed_during_run = true;
            runs[1].changes_from_previous = {
                { "body:ball:mass", "object.properties.mass", {}, "Mass of Steel ball", "1.00 kg", "2.00 kg", ui::EditCategory::parameter },
                { "world:gravity", "world.gravity.strength", {}, "Gravity", "9.81 m/s²", "1.62 m/s²", ui::EditCategory::parameter }
            };
            runs[0].changes_from_original = { { "body:ball:mass", "object.properties.mass", {}, "Mass", "1 kg", "2 kg", ui::EditCategory::parameter } };
            runs[1].changes_from_original = {
                { "body:ball:mass", "object.properties.mass", {}, "Mass", "1 kg", "3 kg", ui::EditCategory::parameter },
                { "world:gravity", "world.gravity.strength", {}, "Gravity", "9.81 m/s²", "1.62 m/s²", ui::EditCategory::parameter }
            };
            for (int index = 0; index < 12; ++index)
            {
                pinned.push_back({ "speed:" + std::to_string(index + 1), "speed", model.selection, ui::RunAggregator::maximum, 0.0 });
                runs[0].pinned_results.push_back(index == 1 ? 0.0 : 1.0 + index);
                runs[1].pinned_results.push_back(2.0 + index);
            }
            for (auto& run : runs)
            {
                run.series.time_s = { 0.0f, 0.5f, 1.0f };
                run.series.scene.assign(24, 0.0f);
            }
            model.runs = runs;
            model.pinned_values = pinned;
            model.previous_run = &runs[0];
            model.current_run = &runs[1];
            model.starred_run_count = 8;
            view.set_active_tab("measure.header.tabs", name == "runs" ? "runs" : "graph");
            if (name == "graph_runs")
                view.set_value("measure.graph.compare_with", "1");
        }
        const auto actual = serialize(name, model, view);
        const auto path = fixture_root / ("all." + name + ".rows.txt");
        if (update)
        {
            std::ofstream output(path, std::ios::trunc);
            output << actual;
            continue;
        }
        std::ifstream input(path);
        std::ostringstream expected;
        expected << input.rdbuf();
        if (expected.str() != actual)
        {
            std::cerr << "row golden differs: " << path.string() << '\n';
            std::istringstream left(expected.str()), right(actual);
            std::string a, b;
            std::size_t line = 1;
            while (std::getline(left, a) && std::getline(right, b) && a == b)
                ++line;
            std::cerr << "first difference at line " << line << "\nexpected: " << a << "\nactual:   " << b << '\n';
        }
        RIGIDBODIES_EXPECT(expected.str() == actual, "row golden matches (set RIGIDBODIES_UPDATE_GOLDENS=1 for an explicit update)");
    }
}

RIGIDBODIES_TEST("Library search uses metadata and cards expose an Enter action")
{
    using namespace rigidbodies;
    ui::UiModel model;
    for (const auto& description : physics::available_scenarios())
        model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
    ui::ViewState view;
    view.set_value("library.search", "friction");
    Device device;
    render::Theme theme;
    render::DrawList draw;
    std::vector<ui::Hotspot> hotspots;
    std::vector<ui::PanelRow> rows;
    ui::PanelBuilder builder(theme, device, 1, { {}, { 1200, 100000 } }, draw, hotspots, &view);
    builder.record_rows(rows);
    ui::LibraryPanel library;
    library.build(model, builder);
    std::vector<std::string> matches;
    bool enter_action = false;
    for (const auto& row : rows)
        if (row.kind == ui::PanelRowKind::list_item)
        {
            matches.push_back(row.text);
            enter_action = enter_action || row.alternate_command.kind == ui::UiCommandKind::load_scenario;
        }
    RIGIDBODIES_EXPECT(std::find(matches.begin(), matches.end(), "Ramp and friction") != matches.end(), "search finds Ramp and friction");
    RIGIDBODIES_EXPECT(std::find(matches.begin(), matches.end(), "Friction and rolling") != matches.end(), "search finds Friction and rolling");
    RIGIDBODIES_EXPECT(enter_action, "a highlighted card has an alternate open command for Enter and double-click");
}

int main()
{
    return rigidbodies::testing::run_all();
}
