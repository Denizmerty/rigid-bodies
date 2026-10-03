#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/plot.hpp>
#include <rigidbodies/ui/ui_context.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <SDL3/SDL.h>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    using namespace rigidbodies;

    class KeyedPanel final : public ui::Panel
    {
    public:
        bool insert_above { false };
        bool tilt_only { false };
        double mass { 1.0 };
        std::string live_text { "0.00 s" };

        std::string_view id() const override
        {
            return "keyed";
        }
        std::string_view title() const override
        {
            return "Keyed controls";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            if (insert_above)
                builder.heading("Inserted above");
            if (tilt_only)
            {
                ui::UiCommand command;
                command.kind = ui::UiCommandKind::set_gravity_angle_degrees;
                builder.number_row(*ui::find_control_spec("world.gravity.tilt"), 0.0, command);
            }
            else
            {
                ui::UiCommand command;
                command.kind = ui::UiCommandKind::set_selected_mass;
                builder.number_row(*ui::find_control_spec("object.properties.mass"), mass, command);
                builder.readout("bar.time.readout", "Elapsed", live_text, ui::RowTone::normal, true);
            }
        }
    };

    class WidgetPanel final : public ui::Panel
    {
    public:
        std::string_view id() const override
        {
            return "widgets";
        }
        std::string_view title() const override
        {
            return "Widget vocabulary";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_selected_mass;
            builder.number_row(*ui::find_control_spec("object.properties.mass"), 2.0, command);
            command.kind = ui::UiCommandKind::set_gravity_angle_degrees;
            builder.number_row(*ui::find_control_spec("world.gravity.tilt"), 30.0, command);
            command.kind = ui::UiCommandKind::set_shape_vertex_budget;
            builder.stepper_row(*ui::find_control_spec("draw.precision.vertex_budget"), 128.0, command);
            command.kind = ui::UiCommandKind::set_integrator;
            builder.select_row(*ui::find_control_spec("world.advanced.integration_method"), "velocity_verlet", command);
            builder.radio_list_row(radio_spec(), "one", command);
            command.kind = ui::UiCommandKind::set_theme;
            builder.segmented_row(*ui::find_control_spec("prefs.appearance.theme"), "workbench_dark", command);
            command.kind = ui::UiCommandKind::set_gravity_enabled;
            builder.switch_row(*ui::find_control_spec("world.gravity.enabled"), true, command);
            builder.checkbox_row(checkbox_spec(), true, command);
            const std::string_view defaults[] { "mechanical" };
            (void)builder.checklist(checklist_spec(), defaults);
            (void)builder.tabs("object.header.tabs", tab_options(), "properties");
            (void)builder.section("world.advanced", "Advanced", true);
            ui::NotificationAction action { "Back to start", ui::UiCommand { ui::UiCommandKind::reset_scenario } };
            builder.notice("joint.notice.broken", ui::Severity::warning, "Broken at t = 2.31 s.", action);
            builder.readout("object.motion.speed", "Speed", "2.00 m/s", ui::RowTone::normal, true);
            builder.meter_row("Bounciness", 0.30, {});
            builder.list_item("object.shape.parts", "part_1", { "Part 1", "Oak", {}, { "Current" } }, command);
        }

    private:
        static const ui::ControlSpec& radio_spec()
        {
            static const ui::OptionSpec options[] { { "one", "One", {}, {} }, { "two", "Two", {}, {} } };
            static const ui::ControlSpec spec { "test.radio.choice", "Radio", "Tests", ui::ControlKind::radio_list, ui::UiCommandKind::set_integrator, {}, {}, options };
            return spec;
        }
        static const ui::ControlSpec& checkbox_spec()
        {
            static const ui::ControlSpec spec { "test.checkbox.enabled", "Checkbox", "Tests", ui::ControlKind::checkbox, ui::UiCommandKind::set_gravity_enabled };
            return spec;
        }
        static const ui::ControlSpec& checklist_spec()
        {
            static const ui::OptionSpec options[] { { "mechanical", "Mechanical", {}, {} }, { "speed", "Speed", {}, {} } };
            static const ui::ControlSpec spec { "measure.graph.quantities", "Quantities", "Measure", ui::ControlKind::checklist, ui::UiCommandKind::none, {}, {}, options };
            return spec;
        }
        static math::Span<const ui::OptionSpec> tab_options()
        {
            static const ui::OptionSpec options[] { { "properties", "Properties", {}, {} }, { "motion", "Motion", {}, {} } };
            return options;
        }
    };

    class PlotPanel final : public ui::Panel
    {
    public:
        bool empty { false };
        bool marker { false };
        double x_maximum { 1.0 };

        std::string_view id() const override
        {
            return "plot-test";
        }
        std::string_view title() const override
        {
            return "Plot stacking";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            ui::UiCommand select;
            select.kind = ui::UiCommandKind::set_integrator;
            builder.select_row(*ui::find_control_spec("world.advanced.integration_method"), "velocity_verlet", select);
            static constexpr float times[] { 0.0f, 0.5f, 1.0f };
            static constexpr float values[] { 0.0f, 1.0f, 0.25f };
            ui::PlotData plot;
            plot.x = { "Time", "s", 0.0, x_maximum };
            plot.y = { "Energy", "J", 0.0, 1.0 };
            plot.empty_title = "Run the experiment to record a graph";
            if (!empty)
                plot.series.push_back({ "Energy", values, times, 3, ui::SeriesStyle::solid, 0, "This run", "J" });
            if (marker)
                plot.markers.push_back({ 0.5, "Set gravity strength to 3.00 m/s", ui::PlotMarkerKind::intervention });
            builder.plot("measure.graph.plot", plot);
        }
    };

    struct Fixture
    {
        SDL_Surface* surface { nullptr };
        render::RenderDevicePtr device;
        ui::DocumentBackend backend;
        ui::UiModel model;
        ui::ViewState view;
        render::Theme theme;
        render::DrawList list;
        std::unique_ptr<ui::Panel> panel;
        double wall_time { 1.0 };

        explicit Fixture(std::unique_ptr<ui::Panel> value = std::make_unique<KeyedPanel>()) : panel(std::move(value))
        {
            surface = SDL_CreateSurface(1600, 900, SDL_PIXELFORMAT_RGBA32);
            RIGIDBODIES_EXPECT(surface != nullptr, "software surface is available");
            device = render::SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface));
            RIGIDBODIES_EXPECT(device != nullptr, "software render device is available");
            RIGIDBODIES_EXPECT(backend.initialize(RIGIDBODIES_SOURCE_ASSETS), "document backend initializes");
        }
        ~Fixture()
        {
            device.reset();
            SDL_DestroySurface(surface);
        }
        void build()
        {
            ui::UiFrameContext frame;
            frame.model = &model;
            frame.device = device.get();
            frame.theme = &theme;
            frame.viewport = device->drawable_size();
            frame.scale = 1.0f;
            frame.view = &view;
            frame.wall_time_s = wall_time;
            frame.toasts = model.notifications;
            frame.panels = { panel.get() };
            backend.build(frame, list);
        }
        std::vector<ui::UiCommand> event(const ui::UiEvent& value)
        {
            std::vector<ui::UiCommand> commands;
            backend.handle_event(value, commands);
            return commands;
        }
        std::vector<ui::UiCommand> key(ui::UiKey key, bool control = false)
        {
            ui::UiEvent value;
            value.kind = ui::UiEventKind::key_down;
            value.key = key;
            value.modifiers.control = control;
            return event(value);
        }
        void replace_text(std::string text)
        {
            (void)key(ui::UiKey::a, true);
            ui::UiEvent value;
            value.kind = ui::UiEventKind::text_input;
            value.text = std::move(text);
            (void)event(value);
        }
    };

    RIGIDBODIES_TEST("keyed reconciliation preserves focus and performs no steady-state structure work")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == field, "number field receives focus by its semantic identity");
        panel->insert_above = true;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == field, "focus survives a row inserted above it");
        fixture.build();
        const auto changes = fixture.backend.structure_change_count();
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.structure_change_count() == changes, "an unchanged frame creates moves or removes no document structure");
    }

    RIGIDBODIES_TEST("controlled number fields preserve typing commit units reject invalid input and cancel")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto row = ui::element_id("legacy", "object.properties.mass");
        const auto field = row + "--field";
        fixture.replace_text("2 kg");
        for (int frame = 0; frame < 60; ++frame)
        {
            panel->mass = 1.0 + frame * 0.01;
            fixture.wall_time += 1.0 / 60.0;
            fixture.build();
        }
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "2 kg" }, "focused typed text survives 60 model refreshes");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(row, "is-editing"), "focused field reports editing state");
        auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1, "2 kg sends exactly one command; observed " + std::to_string(commands.size()));
        RIGIDBODIES_EXPECT(commands.front().kind == ui::UiCommandKind::set_selected_mass, "2 kg keeps the mass command template");
        RIGIDBODIES_EXPECT(commands.front().phase == ui::UiEditPhase::commit, "2 kg is a commit phase");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 2.0, 1.0e-12, "2 kg converts to SI");

        fixture.replace_text("200 g");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && std::abs(commands.front().value - 0.2) < 1.0e-12, "200 g converts to 0.2 kg");

        fixture.replace_text("1001 kg");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.empty() && fixture.backend.element_has_class(field, "is-invalid"), "one step beyond the field range is rejected before command dispatch");
        fixture.replace_text("abc");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.empty() && fixture.backend.element_has_class(field, "is-invalid"), "invalid text is visibly rejected without a command");

        fixture.replace_text("3 kg");
        commands = fixture.key(ui::UiKey::escape);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::cancel, "Esc emits cancel");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) != std::optional<std::string> { "3 kg" }, "Esc restores the model value");
    }

    RIGIDBODIES_TEST("unitless field input is read in the unit shown beside the field")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        panel->mass = 0.005;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "5.0" }, "five grams is shown in grams");
        fixture.replace_text("2.0");
        auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && std::abs(commands.front().value - 0.002) < 1.0e-12, "a bare 2.0 beside 'g' means two grams, not two kilograms");

        panel->mass = 1.0;
        fixture.build();
        fixture.replace_text("2");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && std::abs(commands.front().value - 2.0) < 1.0e-12, "a bare 2 beside 'kg' means two kilograms");
        fixture.replace_text("300 g");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && std::abs(commands.front().value - 0.3) < 1.0e-12, "an explicit unit always wins");
    }

    RIGIDBODIES_TEST("a hidden inspector stays hidden until the learner selects something else")
    {
        auto* surface = SDL_CreateSurface(1600, 900, SDL_PIXELFORMAT_RGBA32);
        auto device = render::SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface));
        RIGIDBODIES_EXPECT(device != nullptr, "software render device is available");
        {
            ui::UiContext context;
            RIGIDBODIES_EXPECT(context.initialize(ui::UiBackendKind::document, RIGIDBODIES_SOURCE_ASSETS), "document context starts");
            render::DrawList list;
            ui::UiModel model;
            model.selection = physics::BodyId { 1, 1 };
            model.selected_bodies = { model.selection };
            context.build(model, *device, list);
            RIGIDBODIES_EXPECT(context.view_state().surface_open("inspector.open", false), "a new selection opens the inspector");
            context.request(ui::ViewRequest::toggle_inspector_pin);
            for (int frame = 0; frame < 3; ++frame)
                context.build(model, *device, list);
            RIGIDBODIES_EXPECT(!context.view_state().surface_open("inspector.open", true), "hiding it holds while the same object stays selected");
            model.selection = physics::BodyId { 2, 1 };
            model.selected_bodies = { model.selection };
            context.build(model, *device, list);
            RIGIDBODIES_EXPECT(context.view_state().surface_open("inspector.open", false), "selecting another object opens it again");
        }
        device.reset();
        SDL_DestroySurface(surface);
    }

    RIGIDBODIES_TEST("tilt fields transform learner-relative degrees to the session direction")
    {
        Fixture fixture;
        static_cast<KeyedPanel*>(fixture.panel.get())->tilt_only = true;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        fixture.replace_text("30\xC2\xB0");
        const auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_gravity_angle_degrees && std::abs(commands.front().value + 60.0) < 1.0e-12, "30 degrees of tilt sends the required -60 degree direction");
    }

    RIGIDBODIES_TEST("slider drag emits bounded previews then a commit across refresh")
    {
        Fixture fixture;
        fixture.build();
        const auto slider = ui::element_id("legacy", "object.properties.mass") + "--slider";
        const auto bounds = fixture.backend.element_bounds(slider);
        RIGIDBODIES_EXPECT(bounds.has_value(), "mass slider is rendered");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = { bounds->minimum.x + bounds->width() * 0.35, (bounds->minimum.y + bounds->maximum.y) * 0.5 };
        pointer.kind = ui::UiEventKind::pointer_down;
        auto commands = fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.65;
        const auto first_move = fixture.event(pointer);
        commands.insert(commands.end(), first_move.begin(), first_move.end());
        std::size_t previews = 0;
        for (const auto& command : commands)
            previews += command.phase == ui::UiEditPhase::preview;
        RIGIDBODIES_EXPECT(previews <= 1, "one frame sends at most one preview for the control");
        fixture.wall_time += 1.0 / 60.0;
        fixture.build();
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.8;
        commands = fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        const auto released = fixture.event(pointer);
        commands.insert(commands.end(), released.begin(), released.end());
        RIGIDBODIES_EXPECT(!commands.empty() && commands.back().phase == ui::UiEditPhase::commit, "a drag ends in one commit after the refreshed preview");
    }

    RIGIDBODIES_TEST("number labels scrub in four-pixel steps and support preview commit and cancel")
    {
        Fixture fixture;
        fixture.build();
        const auto label = ui::element_id("legacy", "object.properties.mass") + "--label";
        const auto bounds = fixture.backend.element_bounds(label);
        RIGIDBODIES_EXPECT(bounds.has_value(), "number label has a scrub target");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x += 8.0;
        auto commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::preview, "an eight-pixel scrub emits one preview");
        pointer.kind = ui::UiEventKind::pointer_up;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::commit, "scrub release emits one commit");

        fixture.build();
        pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x += 8.0;
        (void)fixture.event(pointer);
        commands = fixture.key(ui::UiKey::escape);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::cancel, "Esc cancels an active label scrub");
    }

    RIGIDBODIES_TEST("live readouts write text no more than ten times per second")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        fixture.build();
        const auto value_id = ui::element_id("legacy", "bar.time.readout") + "--value";
        const auto before = fixture.backend.text_write_count(value_id);
        for (int frame = 1; frame <= 60; ++frame)
        {
            fixture.wall_time = 1.0 + frame / 60.0;
            panel->live_text = std::to_string(frame);
            fixture.build();
        }
        const auto writes = fixture.backend.text_write_count(value_id) - before;
        RIGIDBODIES_EXPECT(writes <= 10, "60 model frames produce at most ten live text writes");
    }

    RIGIDBODIES_TEST("the full widget vocabulary creates semantic document rows")
    {
        Fixture fixture(std::make_unique<WidgetPanel>());
        fixture.build();
        const std::string keys[] {
            "object.properties.mass", "world.gravity.tilt", "draw.precision.vertex_budget", "world.advanced.integration_method", "test.radio.choice", "prefs.appearance.theme", "world.gravity.enabled", "test.checkbox.enabled", "measure.graph.quantities", "object.header.tabs", "world.advanced", "joint.notice.broken", "object.motion.speed", "object.shape.parts"
        };
        for (const auto& key : keys)
            if (key != "object.shape.parts")
                RIGIDBODIES_EXPECT(fixture.backend.element_for_key("legacy", key).has_value(), "widget row exists under semantic key " + key);
        RIGIDBODIES_EXPECT(fixture.backend.element_for_key("legacy", "object.shape.parts", "part_1").has_value(), "list row includes its stable instance identity");
        RIGIDBODIES_EXPECT(fixture.backend.element_bounds(ui::element_id("legacy", "world.gravity.tilt") + "--slider").has_value(), "dial is an interactive range part");
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find("0.30") != std::string::npos, "meter exposes its two-decimal value");

        const auto click = [&](const std::string& id)
        {
            const auto bounds = fixture.backend.element_bounds(id);
            RIGIDBODIES_EXPECT(bounds.has_value(), "local widget part has pointer bounds");
            ui::UiEvent event;
            event.button = ui::PointerButton::primary;
            event.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
            event.kind = ui::UiEventKind::pointer_down;
            (void)fixture.event(event);
            event.kind = ui::UiEventKind::pointer_up;
            (void)fixture.event(event);
        };
        const auto tabs = ui::element_id("legacy", "object.header.tabs");
        click(tabs + "--option_motion");
        const std::string_view available_tabs[] { "properties", "motion" };
        RIGIDBODIES_EXPECT(fixture.view.active_tab("object.header.tabs", available_tabs, "properties") == "motion", "tab activation updates ViewState");
        click(ui::element_id("legacy", "world.advanced"));
        RIGIDBODIES_EXPECT(!fixture.view.section_open("world.advanced", true), "disclosure activation updates ViewState");
        const auto checklist = ui::element_id("legacy", "measure.graph.quantities");
        click(checklist + "--field");
        fixture.build();
        click(checklist + "--option_speed");
        const std::string_view defaults[] { "mechanical" };
        const auto selected = fixture.view.checklist("measure.graph.quantities", defaults);
        RIGIDBODIES_EXPECT(std::find(selected.begin(), selected.end(), "speed") != selected.end(), "checklist option toggles without closing the local control");
        fixture.build();
        const auto summary = fixture.backend.element_attribute(checklist + "--field", "aria-label");
        RIGIDBODIES_EXPECT(summary && summary->find("Mechanical") != std::string::npos && summary->find("Speed") != std::string::npos, "the checklist summary names the chosen quantities rather than counting them");
    }

    RIGIDBODIES_TEST("plot geometry is clipped inside the document and precedes an open dropdown")
    {
        Fixture fixture(std::make_unique<PlotPanel>());
        ui::reset_plot_render_count();
        fixture.list.clear();
        fixture.build();
        const auto first_count = ui::plot_render_count();
        RIGIDBODIES_EXPECT(first_count == 1, "the inline plot renders once in the first document frame");

        const auto field = ui::element_id("legacy", "world.advanced.integration_method") + "--field";
        const auto field_bounds = fixture.backend.element_bounds(field);
        RIGIDBODIES_EXPECT(field_bounds.has_value(), "the dropdown used for the stacking check is laid out");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = { (field_bounds->minimum.x + field_bounds->maximum.x) * 0.5, (field_bounds->minimum.y + field_bounds->maximum.y) * 0.5 };
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        (void)fixture.event(pointer);

        fixture.list.clear();
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_render_count() == first_count + 1, "the plot is redrawn on the next frame");

        // Series colours come from the active theme's RCSS, so the mesh is found by the colour
        // the plot reports it painted with rather than by a literal.
        const auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.series_drawn == 1 && diagnostics.primary_colour[3] == 255, "the plot paints its series with an opaque theme colour");
        std::optional<std::size_t> plot_index;
        for (std::size_t index = 0; index < fixture.list.commands().size() && !plot_index; ++index)
        {
            const auto& command = fixture.list.commands()[index];
            if (command.kind != render::DrawCommandKind::indexed_mesh || !command.mesh || command.mesh->vertices.empty())
                continue;
            for (const auto& vertex : command.mesh->vertices)
                if (std::abs(vertex.color.red - diagnostics.primary_colour[0] / 255.0f) < 0.01f && std::abs(vertex.color.green - diagnostics.primary_colour[1] / 255.0f) < 0.01f && std::abs(vertex.color.blue - diagnostics.primary_colour[2] / 255.0f) < 0.01f && vertex.color.alpha > 0.99f)
                {
                    plot_index = index;
                    RIGIDBODIES_EXPECT(command.mesh->clip.has_value(), "plot geometry carries the RmlUi element clip");
                    break;
                }
        }
        RIGIDBODIES_EXPECT(plot_index.has_value(), "the plot mesh is present in the document draw list");
        const auto plot_end = *plot_index + (diagnostics.draw_calls - diagnostics.series_draw_index);
        const auto later_geometry = std::find_if(fixture.list.commands().begin() + static_cast<std::ptrdiff_t>(std::min(plot_end, fixture.list.commands().size())), fixture.list.commands().end(), [](const render::DrawCommand& command)
            {
                return command.kind == render::DrawCommandKind::indexed_mesh && command.mesh && !command.mesh->vertices.empty();
            });
        RIGIDBODIES_EXPECT(later_geometry != fixture.list.commands().end(), "the open dropdown geometry is emitted after the last plot mesh");
    }

    RIGIDBODIES_TEST("plot axes use 1-2-5 ticks and typographic numbers")
    {
        auto ticks = ui::plot_ticks(0.0, 9.3, 4, true);
        RIGIDBODIES_EXPECT(ticks.step == 2.0 && ticks.minimum == 0.0 && ticks.maximum == 10.0 && ticks.decimals == 0, "0 to 9.3 J widens to 0 to 10 in steps of 2");
        ticks = ui::plot_ticks(0.0, 0.137, 4, true);
        RIGIDBODIES_EXPECT(std::abs(ticks.step - 0.05) < 1.0e-12 && std::abs(ticks.maximum - 0.15) < 1.0e-12 && ticks.decimals == 2, "small ranges step by 0.05 with two decimals");
        ticks = ui::plot_ticks(-3.2, 7.9, 5, true);
        RIGIDBODIES_EXPECT(ticks.step == 2.0 && ticks.minimum == -4.0 && ticks.maximum == 8.0, "signed ranges widen outward on both sides");
        ticks = ui::plot_ticks(1.37, 9.62, 4, false);
        RIGIDBODIES_EXPECT(ticks.step == 2.0 && ticks.minimum == 2.0 && ticks.maximum == 8.0, "a kept time range places ticks inside it");
        ticks = ui::plot_ticks(0.0, 0.0, 4, true);
        RIGIDBODIES_EXPECT(ticks.maximum > ticks.minimum && ticks.step > 0.0, "an all-zero record still gets a usable axis");
        ticks = ui::plot_ticks(0.0, 1.155e-27, 4, true);
        RIGIDBODIES_EXPECT(ticks.minimum == 0.0 && std::abs(ticks.maximum - 1.0) < 1.0e-12 && ticks.decimals <= 1, "numerical noise around zero gets the zero axis, not stacked 0.000000000 labels");
        ticks = ui::plot_ticks(-47.0, 54.0, 3, true);
        RIGIDBODIES_EXPECT(ticks.step == 50.0 && ticks.minimum == -50.0 && std::abs(ticks.maximum - 59.05) < 1.0e-9, "a short axis stops just past the data instead of rounding out to a mostly empty step");
        RIGIDBODIES_EXPECT((54.0 + 47.0) / (ticks.maximum - ticks.minimum) > 0.8, "the data fills most of a short axis");
        ticks = ui::plot_ticks(-48.0, 54.0, 4, true);
        RIGIDBODIES_EXPECT(ticks.step == 20.0 && ticks.minimum == -60.0 && ticks.maximum == 60.0, "once four intervals fit, the axis hugs the data instead of leaving a third empty");
        ticks = ui::plot_ticks(0.0, 1.3, 5, true);
        RIGIDBODIES_EXPECT(ticks.step == 0.5 && ticks.maximum == 1.5, "an axis never crowds well past its target interval count");
        RIGIDBODIES_EXPECT(ui::format_plot_number(-1.5, 1) == "\xE2\x88\x92"
                                                              "1.5",
            "negative values use a true minus sign");
        RIGIDBODIES_EXPECT(ui::format_plot_number(-0.0001, 2) == "0.00", "rounding never shows negative zero");
        RIGIDBODIES_EXPECT(ui::format_plot_number(12.25, 2) == "12.25", "fixed decimals are kept");
        RIGIDBODIES_EXPECT(ui::format_plot_value(1.2516) == "1.25" && ui::format_plot_value(0.09514) == "0.0951" && ui::format_plot_value(1234.4) == "1234", "read-outs keep three significant figures whatever the axis step");
        RIGIDBODIES_EXPECT(ui::format_plot_value(9.996) == "10.0" && ui::format_plot_value(1.0e-9) == "0", "read-outs carry rounding into a new digit and show noise as zero");
        RIGIDBODIES_EXPECT(ui::format_plot_value(-0.31234) == "\xE2\x88\x92"
                                                              "0.312",
            "negative read-outs use a true minus sign");
    }

    RIGIDBODIES_TEST("plot hover snaps a cursor to samples and a click pins and releases it")
    {
        Fixture fixture(std::make_unique<PlotPanel>());
        fixture.build();
        const auto plot = ui::element_id("legacy", "measure.graph.plot") + "--plot";
        const auto bounds = fixture.backend.element_bounds(plot);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the plot canvas is laid out");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px = { bounds->minimum.x + bounds->width() * 0.92, (bounds->minimum.y + bounds->maximum.y) * 0.5 };
        (void)fixture.event(pointer);
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.cursor_t.has_value() && std::abs(*diagnostics.cursor_t - 1.0) < 1.0e-6 && !diagnostics.pinned, "hovering near the end shows the last recorded sample");

        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        (void)fixture.event(pointer);
        fixture.build();
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.3;
        (void)fixture.event(pointer);
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.pinned && diagnostics.cursor_t && std::abs(*diagnostics.cursor_t - 1.0) < 1.0e-6, "a click pins the cursor, which then ignores hover");

        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        (void)fixture.event(pointer);
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(!diagnostics.pinned && diagnostics.cursor_t && *diagnostics.cursor_t < 0.75, "a second click releases the pin back to hover");

        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px = { bounds->minimum.x - 40.0, bounds->minimum.y - 40.0 };
        (void)fixture.event(pointer);
        fixture.build();
        RIGIDBODIES_EXPECT(!ui::plot_diagnostics().cursor_t.has_value(), "leaving the plot hides the hover cursor");
    }

    RIGIDBODIES_TEST("plot without data draws an empty state instead of a blank canvas")
    {
        auto panel = std::make_unique<PlotPanel>();
        panel->empty = true;
        Fixture fixture(std::move(panel));
        fixture.build();
        const auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.empty_state && diagnostics.series_drawn == 0, "no series means the empty state");
        RIGIDBODIES_EXPECT(diagnostics.draw_calls >= 2, "the empty state draws a placeholder grid and its message");
    }

    RIGIDBODIES_TEST("measure graph keeps one colour per quantity across runs and shows an empty plot before any run")
    {
        Fixture fixture;
        ui::ViewState view;
        view.set_active_tab("measure.header.tabs", "graph");
        ui::MeasurePanel panel;
        auto plot_leads = false;
        const auto plot_row = [&](const ui::UiModel& model) -> std::shared_ptr<const ui::PlotData>
        {
            render::DrawList draw;
            std::vector<ui::Hotspot> hotspots;
            std::vector<ui::PanelRow> rows;
            ui::PanelBuilder builder(fixture.theme, *fixture.device, 1.0f, { {}, { 1200, 100000 } }, draw, hotspots, &view);
            builder.record_rows(rows);
            panel.build(model, builder);
            // The drawer's close control sits in the header corner, not in the content.
            rows.erase(std::remove_if(rows.begin(), rows.end(), [](const ui::PanelRow& row)
                           {
                               return row.presentation.header;
                           }),
                rows.end());
            plot_leads = rows.size() > 1 && rows[0].kind == ui::PanelRowKind::tabs && rows[1].kind == ui::PanelRowKind::plot;
            for (const auto& row : rows)
                if (row.kind == ui::PanelRowKind::plot)
                    return row.plot;
            return {};
        };

        ui::UiModel model;
        auto plot = plot_row(model);
        RIGIDBODIES_EXPECT(plot && plot->series.empty() && !plot->empty_title.empty(), "before any run the graph tab shows the plot's empty state");
        RIGIDBODIES_EXPECT(plot_leads, "the graph comes straight after the tabs, ahead of the controls that shape it");

        std::vector<ui::RunRecord> runs(2);
        for (std::size_t index = 0; index < runs.size(); ++index)
        {
            runs[index].number = static_cast<int>(index + 1);
            runs[index].duration_s = 1.0;
            runs[index].series.time_s = { 0.0f, 0.5f, 1.0f };
            runs[index].series.scene.assign(24, 1.0f);
        }
        model.runs = runs;
        model.previous_run = &runs[0];
        model.current_run = &runs[1];
        view.set_checklist("measure.graph.quantities", { "mechanical", "momentum_x" });
        plot = plot_row(model);
        RIGIDBODIES_EXPECT(plot && plot->series.size() == 4, "two quantities over the current and previous run make four series");
        RIGIDBODIES_EXPECT(plot->series[0].slot == 0 && plot->series[1].slot == 1 && plot->series[2].slot == 0 && plot->series[3].slot == 1, "each quantity keeps its colour slot in every run");
        RIGIDBODIES_EXPECT(plot->series[0].style == ui::SeriesStyle::solid && plot->series[2].style == ui::SeriesStyle::dashed && plot->series[2].source == "Previous run", "the previous run is a dashed comparison");
        RIGIDBODIES_EXPECT(plot->series[0].unit == "J" && !plot->series[1].unit.empty() && plot->series[1].unit != "J" && plot->y.unit.empty(), "mixed units stay on each series instead of mislabelling the shared axis");
        RIGIDBODIES_EXPECT(plot_leads, "the graph still leads the tab once runs exist");

        // Speed belongs to one object, so with nothing selected it has no data to draw.
        view.set_checklist("measure.graph.quantities", { "mechanical", "speed" });
        plot = plot_row(model);
        RIGIDBODIES_EXPECT(plot && plot->y.unit == "J" && plot->note.find("Select an object") != std::string::npos, "a chosen quantity without data leaves the axis to the drawn series and says why");
        view.set_checklist("measure.graph.quantities", { "speed" });
        plot = plot_row(model);
        RIGIDBODIES_EXPECT(plot && plot->empty_title.find("Select an object") == 0, "a per-object quantity without an object explains itself instead of asking for Play");
    }

    RIGIDBODIES_TEST("plot colours follow the active theme")
    {
        Fixture fixture(std::make_unique<PlotPanel>());
        fixture.build();
        const auto dark = ui::plot_diagnostics().primary_colour;
        fixture.model.theme_id = "workbench_light";
        fixture.build();
        const auto light = ui::plot_diagnostics().primary_colour;
        fixture.model.theme_id = "workbench_projector";
        fixture.build();
        const auto projector = ui::plot_diagnostics().primary_colour;
        RIGIDBODIES_EXPECT(dark[3] == 255 && light[3] == 255 && projector[3] == 255, "every theme gives the series an opaque colour");
        RIGIDBODIES_EXPECT(dark != light && light != projector, "series colours are read from the theme rather than fixed in code");
    }

    RIGIDBODIES_TEST("a pin the window moves past is released and the next click pins again")
    {
        auto owned = std::make_unique<PlotPanel>();
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        const auto plot = ui::element_id("legacy", "measure.graph.plot") + "--plot";
        const auto bounds = fixture.backend.element_bounds(plot);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the plot canvas is laid out");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        const auto click = [&]
        {
            pointer.kind = ui::UiEventKind::pointer_down;
            (void)fixture.event(pointer);
            pointer.kind = ui::UiEventKind::pointer_up;
            (void)fixture.event(pointer);
            fixture.build();
        };
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px = { bounds->minimum.x + bounds->width() * 0.92, bounds->minimum.y + 60.0 };
        (void)fixture.event(pointer);
        fixture.build();
        click();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().pinned, "a click pins the cursor at the last sample");

        panel->x_maximum = 0.6;
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(!diagnostics.pinned, "a pinned time outside the visible window is released");
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.5;
        (void)fixture.event(pointer);
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().cursor_t.has_value() && !ui::plot_diagnostics().pinned, "hover works again once the stale pin is gone");
        click();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().pinned, "one click places a new pin rather than only clearing an invisible one");
    }

    RIGIDBODIES_TEST("plot read-out stays inside the data area and labels sit in their own lane")
    {
        auto owned = std::make_unique<PlotPanel>();
        owned->marker = true;
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        const auto plot = ui::element_id("legacy", "measure.graph.plot") + "--plot";
        const auto bounds = fixture.backend.element_bounds(plot);
        RIGIDBODIES_EXPECT(bounds.has_value() && bounds->height() > 200.0, "the canvas grows to fill the visible height of its scrolling region");
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.marker_labels == 1, "an intervention keeps its label");
        const auto lane_top = diagnostics.frame[1];

        ui::UiEvent pointer;
        pointer.kind = ui::UiEventKind::pointer_move;
        for (const auto fraction : { 0.25, 0.92 })
        {
            pointer.pointer_px = { bounds->minimum.x + bounds->width() * fraction, bounds->minimum.y + bounds->height() * 0.5 };
            (void)fixture.event(pointer);
            fixture.build();
            diagnostics = ui::plot_diagnostics();
            const auto& box = diagnostics.tooltip;
            RIGIDBODIES_EXPECT(diagnostics.cursor_t.has_value() && box[2] > box[0], "hovering shows the read-out");
            RIGIDBODIES_EXPECT(box[0] >= diagnostics.frame[0] - 0.5f && box[2] <= diagnostics.frame[2] + 0.5f, "the read-out never covers the value axis labels or leaves the plot");
            RIGIDBODIES_EXPECT(box[3] <= diagnostics.frame[3] + 0.5f, "the read-out never covers the time axis");
        }

        fixture.panel = std::make_unique<PlotPanel>();
        fixture.build();
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().frame[1] < lane_top, "the label lane is only reserved while a labelled intervention is in view");
    }

    RIGIDBODIES_TEST("plot summary names the quantity, the series and the marked events")
    {
        static constexpr float times[] { 0.0f, 0.5f, 1.0f };
        static constexpr float values[] { 0.0f, 1.0f, 0.25f };
        ui::PlotData data;
        data.x = { "Time", "s", 0.0, 1.0 };
        data.y = { "Mechanical energy", "J", 0.0, 1.0 };
        data.empty_title = "Run the experiment to record a graph";
        RIGIDBODIES_EXPECT(ui::plot_summary(data) == "Empty graph of Mechanical energy (J). Run the experiment to record a graph.", "an empty plot says so and why");
        data.series.push_back({ "Mechanical energy", values, times, 3, ui::SeriesStyle::solid, 0, "This run", "J" });
        data.series.push_back({ "Mechanical energy", values, times, 3, ui::SeriesStyle::dashed, 0, "Previous run", "J" });
        data.markers.push_back({ 0.5, "Impact 1", ui::PlotMarkerKind::impact });
        RIGIDBODIES_EXPECT(ui::plot_summary(data) == "Graph of Mechanical energy (J) against Time (s), showing Mechanical energy and Mechanical energy (Previous run). 1 impact marked.", "a drawn plot names its axis, series and events");
    }

    RIGIDBODIES_TEST("toast reveal actions dismiss themselves after routing the reveal")
    {
        Fixture fixture;
        ui::NotificationAction action;
        action.label = "Inspect";
        action.reveal_key = "world.advanced.integration_method";
        fixture.model.notifications.push_back({ 42, ui::Severity::warning, "Check the method.", action, "method", false });
        fixture.build();

        const auto bounds = fixture.backend.element_bounds("toast-42--action");
        RIGIDBODIES_EXPECT(bounds.has_value(), "the toast action is rendered as a document control");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = { (bounds->minimum.x + bounds->maximum.x) * 0.5, (bounds->minimum.y + bounds->maximum.y) * 0.5 };
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        const auto commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 2, "one activation emits the reveal and its dismissal");
        RIGIDBODIES_EXPECT(commands[0].detail == "reveal:world.advanced.integration_method", "the reveal reaches UiContext's view-only route");
        RIGIDBODIES_EXPECT(commands[1].detail == "dismiss_toast:42", "activating the action dismisses the toast");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
