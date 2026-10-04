#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/ui/overlay_backend.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include "test_framework.hpp"
#include <array>
#include <iostream>
#include <set>

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
            return static_cast<float>(text.size()) * 7 * scale;
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

    std::set<ui::UiCommandKind> expected_commands()
    {
        using K = ui::UiCommandKind;
        const std::set<K> developer_only { K::toggle_developer_overlay, K::set_physics_profiling, K::set_physics_workers, K::set_solver_parameter, K::set_force_generator_enabled };
        // The window's own title bar sends these, and only the document backend draws one; under
        // the overlay backend the platform's title bar does their work.
        const std::set<K> window_title_bar { K::minimize_window, K::toggle_maximize_window };
        std::set<K> result;
        for (int value = static_cast<int>(K::undo); value <= static_cast<int>(K::set_shape_node_position); ++value)
            if (!developer_only.count(static_cast<K>(value)) && !window_title_bar.count(static_cast<K>(value)))
                result.insert(static_cast<K>(value));
        // The combined velocity setter is emitted by the stage handle, not a panel row.
        result.erase(K::set_selected_velocity);
        return result;
    }

    std::set<ui::UiCommandKind> collect(ui::UiBackend& backend)
    {
        Device device;
        render::Theme theme;
        render::DrawList draw;
        ui::ViewState view;
        view.open_transient("add_menu");
        for (const auto* section : { "draw.node", "draw.snap", "draw.material", "draw.precision", "world.gravity", "world.air", "world.collisions", "world.advanced", "world.statistics", "show.arrows", "tools.reference" })
            view.set_section_open(section, true);
        auto panels = ui::create_default_panels();
        std::vector<ui::Panel*> panel_views;
        panel_views.reserve(panels.size());
        for (auto& panel : panels)
            panel_views.push_back(panel.get());
        ui::LayoutInput input;
        input.viewport = { 1600, 900 };
        input.inspector_open = true;
        input.measure_open = true;
        const auto layout = ui::compute_layout(input);
        std::set<ui::UiCommandKind> emitted;
        struct Fixture
        {
            const char* scenario;
            const char* object_tab;
            const char* measure_tab;
        };
        // The tab axes are independent. Cover each value at least once instead of constructing their
        // Cartesian product, which made the document-backend gate take several minutes without
        // exposing any additional row or hotspot.
        const std::array fixtures {
            Fixture { "free_fall", "properties", "guide" },
            Fixture { "free_fall", "motion", "guide" },
            Fixture { "free_fall", "forces", "guide" },
            Fixture { "free_fall", "joints", "guide" },
            Fixture { "free_fall", "shape", "guide" },
            Fixture { "free_fall", "properties", "energy" },
            Fixture { "free_fall", "properties", "graph" },
            Fixture { "free_fall", "properties", "collisions" },
            Fixture { "free_fall", "properties", "runs" },
            Fixture { "free_fall", "properties", "theory" },
            Fixture { "revolute_drive", "joints", "guide" },
            Fixture { "spring_damping", "properties", "guide" },
            Fixture { "compound_object", "properties", "guide" },
            Fixture { "shape_workshop", "properties", "guide" },
        };
        for (const auto& fixture : fixtures)
        {
            const auto* scenario = fixture.scenario;
            physics::World world;
            RIGIDBODIES_EXPECT(physics::load_scenario(world, scenario), "reachability fixture loads");
            ui::UiModel model;
            model.world = &world;
            model.scenario_id = scenario;
            model.scenario_title = scenario;
            model.layers = render::LayerMask::defaults();
            // Show offers the split direction only while arrows are split along a chosen one.
            model.vector_components = render::VectorComponents::custom_axes;
            model.can_undo = true;
            model.can_redo = true;
            model.elapsed_time_s = 1.0;
            model.shape_editor_active = std::string_view(scenario) == "shape_workshop";
            model.shape_outline_closed = true;
            model.shape_can_commit = true;
            model.shape_node_selected = true;
            model.draft.active = model.shape_editor_active;
            model.draft.closed = true;
            model.draft.valid = true;
            model.draft.target_name = "Fixture draft";
            model.draft.material_name = "steel";
            model.can_export_shape = true;
            model.can_split_shape = true;
            model.can_assemble_shapes = true;
            model.selected_shape_authored = true;
            model.authored_part_count = 2;
            model.authored_part_name = "Part 1";
            model.authored_material_name = "steel";
            model.impacts.push_back({ 0, 0.5, {}, {}, "A", "B", 1.0, {}, {}, {}, {}, {}, {}, 0.1, 0.1, true });
            model.lab_changes.push_back({ "integrator", "Integration method", "Semi-implicit Euler", "Runge–Kutta 4" });
            const auto body_ids = world.body_ids();
            for (const auto id : body_ids)
                if (const auto* body = world.find_body(id))
                {
                    model.objects.push_back({ id, std::string(body->name()), body->name().empty() ? "Object" : std::string(body->name()), body->type() == physics::BodyType::dynamic_body ? "free" : body->type() == physics::BodyType::kinematic_body ? "driven"
                                                                                                                                                                                                                                                       : "fixed",
                        "object",
                        {},
                        false,
                        false });
                    if (!model.selection.is_valid() && body->type() != physics::BodyType::static_body)
                        model.selection = id;
                }
            if (model.selection.is_valid())
            {
                model.selected_bodies.push_back(model.selection);
                // Stop motion is offered only for an object that is moving.
                if (auto* body = world.find_body(model.selection))
                    body->set_linear_velocity({ 1.0, 0.0 });
                for (auto& object : model.objects)
                    object.moving = object.id == model.selection;
            }
            for (const auto& description : physics::available_scenarios())
                model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
            std::vector<ui::RunRecord> runs(2);
            runs[0].number = 1;
            runs[0].duration_s = 1.0;
            runs[0].prediction = ui::Prediction { "Together", {} };
            runs[1].number = 2;
            runs[1].duration_s = 1.0;
            std::vector<ui::PinnedValue> pinned { { "speed:max", "speed", model.selection, ui::RunAggregator::maximum, 0.0 } };
            runs[0].pinned_results = { 1.0 };
            runs[1].pinned_results = { 2.0 };
            model.runs = runs;
            model.current_run = &runs[1];
            model.previous_run = &runs[0];
            model.pinned_values = pinned;
            ui::ExperimentContent content;
            content.title = "Fixture";
            content.guide.predict.prompt = "What happens?";
            content.guide.predict.options = { "Together", "Apart" };
            content.guide.focus = "Watch the scene.";
            model.scenario_content = std::move(content);
            model.changes.push_back({ "mass", "object.properties.mass", model.selection, "Mass", "1 kg", "2 kg", ui::EditCategory::parameter });
            view.set_active_tab("inspector.object", fixture.object_tab);
            view.set_active_tab("measure.header.tabs", fixture.measure_tab);
            if (std::string_view(fixture.measure_tab) == "guide")
            {
                model.runs = {};
                model.current_run = nullptr;
                model.previous_run = nullptr;
            }
            if (std::string_view(fixture.measure_tab) == "runs")
            {
                view.set_value("measure.runs.add_open", "true");
                view.set_value("measure.runs.confirm_clear", "true");
            }
            ui::UiFrameContext frame { &model, &device, &theme, 1.0f, { 1600, 900 }, &view, &layout, 1.0, {}, panel_views };
            const auto build = [&]
            {
                backend.build(frame, draw);
                for (const auto& command : backend.emitted_commands())
                    if (command.kind != ui::UiCommandKind::none)
                        emitted.insert(command.kind);
            };
            build();
            if (std::string_view(fixture.scenario) == "free_fall" && std::string_view(fixture.object_tab) == "properties" && std::string_view(fixture.measure_tab) == "guide")
            {
                model.selection = {};
                model.selected_bodies.clear();
                model.shape_editor_active = false;
                build();
            }
            if (std::string_view(fixture.scenario) == "shape_workshop")
            {
                model.shape_editor_active = true;
                model.shape_outline_closed = false;
                model.draft.closed = false;
                build();
                // Precision settings live in the draw bar's Options popover.
                view.open_transient("draw_options");
                build();
                view.close_transient("draw_options");
                if (body_ids.size() >= 2)
                {
                    model.selection = body_ids[0];
                    model.selected_bodies = { body_ids[0], body_ids[1] };
                    build();
                }
            }
            if (std::string_view(fixture.scenario) == "revolute_drive")
            {
                model.selected_connection = ui::SelectedConnection { "driven_hinge", "joint" };
                build();
            }
            if (std::string_view(fixture.scenario) == "spring_damping")
            {
                model.selected_connection = ui::SelectedConnection { "damped", "spring" };
                build();
            }
        }
        return emitted;
    }
}

RIGIDBODIES_TEST("every non-developer command is reachable with backend parity")
{
    using namespace rigidbodies;
    ui::OverlayBackend overlay;
    RIGIDBODIES_EXPECT(overlay.initialize({}), "overlay initializes");
    const auto overlay_commands = collect(overlay);
#if defined(RIGIDBODIES_HAS_RMLUI)
    ui::DocumentBackend document;
    RIGIDBODIES_EXPECT(document.initialize(RIGIDBODIES_SOURCE_ASSETS), "document initializes");
    const auto document_commands = collect(document);
    for (const auto command : overlay_commands)
        if (!document_commands.count(command))
            std::cerr << "document missing command " << static_cast<int>(command) << '\n';
    for (const auto command : document_commands)
        if (!overlay_commands.count(command))
            std::cerr << "overlay missing command " << static_cast<int>(command) << '\n';
    RIGIDBODIES_EXPECT(document_commands == overlay_commands, "document rows and overlay hotspots emit the same command set");
#endif
    const auto expected = expected_commands();
    for (const auto command : expected)
        if (!overlay_commands.count(command))
            std::cerr << "unreachable command " << static_cast<int>(command) << '\n';
    for (const auto command : overlay_commands)
        if (!expected.count(command))
            std::cerr << "unexpected command " << static_cast<int>(command) << '\n';
    RIGIDBODIES_EXPECT(overlay_commands == expected, "every non-developer command is emitted across the standard fixture states");
}

int main()
{
    return rigidbodies::testing::run_all();
}
