#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/plot.hpp>
#include <rigidbodies/ui/ui_context.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <SDL3/SDL.h>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

namespace
{
    using namespace rigidbodies;

    class KeyedPanel final : public ui::Panel
    {
    public:
        bool insert_above { false };
        bool tilt_only { false };
        bool show_number { true }, number_disabled { false };
        double mass { 1.0 };
        physics::BodyId target;
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
            if (!show_number)
                return;
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
                command.body = target;
                builder.number_row(*ui::find_control_spec("object.properties.mass"), mass, command);
                if (number_disabled)
                    builder.disable_last("No longer editable");
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

    // The probe speed control as the registry will hold it, so the document's rapidity paths are
    // exercised by a spec local to this test.
    const ui::ControlSpec& probe_speed_spec()
    {
        static const ui::ControlSpec spec = []
        {
            static constexpr double detents[] { 0.5, 0.9, 0.99, 0.999, 0.9999, 0.99999 };
            ui::ControlSpec value { "world.relativity.speed", "Probe speed", "World \xE2\x80\xBA Relativity", ui::ControlKind::number, ui::UiCommandKind::set_relativity_speed };
            value.category = ui::EditCategory::parameter;
            auto& number = value.number;
            number.minimum = 0.0;
            number.maximum = physics::maximum_speed_fraction;
            number.soft_minimum = 0.0;
            number.soft_maximum = 0.99999;
            number.smallest_nonzero = physics::minimum_moving_speed_fraction;
            number.step = physics::speed_rapidity_step;
            number.scale = ui::NumberScale::rapidity;
            number.quantity = core::DisplayQuantity::speed_fraction;
            number.detents = detents;
            number.slider = true;
            return value;
        }();
        return spec;
    }

    class SpeedPanel final : public ui::Panel
    {
    public:
        double speed { 0.9 };

        std::string_view id() const override
        {
            return "speed";
        }
        std::string_view title() const override
        {
            return "Probe speed";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_relativity_speed;
            builder.number_row(probe_speed_spec(), speed, command);
        }
    };

    // The stored speed is exactly the speed its own text reads.
    bool reads_as_itself(double fraction)
    {
        const auto text = core::format_value(fraction, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
        const auto parsed = core::parse_quantity(text, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
        return parsed && *parsed == fraction;
    }

    // Option lists that change with the model, as the Measure tabs and quantity lists do between
    // experiments.
    class OptionsPanel final : public ui::Panel
    {
    public:
        bool second_list { false };

        std::string_view id() const override
        {
            return "options";
        }
        std::string_view title() const override
        {
            return "Option lists";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            using Options = math::Span<const ui::OptionSpec>;
            static constexpr ui::OptionSpec first_tabs[] { { "energy", "Energy", {}, {} }, { "graph", "Graph", {}, {} }, { "theory", "Theory checks", {}, {} } };
            static constexpr ui::OptionSpec second_tabs[] { { "relativity", "Relativity", {}, {} }, { "graph", "Graph", {}, {} }, { "runs", "Runs", {}, {} } };
            (void)builder.tabs("test.options.tabs", second_list ? Options(second_tabs) : Options(first_tabs), second_list ? "relativity" : "energy");

            static constexpr ui::OptionSpec first_choices[] { { "one", "One", {}, {} }, { "two", "Two", {}, {} } };
            static constexpr ui::OptionSpec second_choices[] { { "two", "Two", {}, {} }, { "three", "Three", {}, {} } };
            static const ui::ControlSpec first_choice { "test.options.choice", "Choice", "Tests", ui::ControlKind::segmented, ui::UiCommandKind::set_integrator, {}, {}, first_choices };
            static const ui::ControlSpec second_choice { "test.options.choice", "Choice", "Tests", ui::ControlKind::segmented, ui::UiCommandKind::set_integrator, {}, {}, second_choices };
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_integrator;
            builder.segmented_row(second_list ? second_choice : first_choice, "two", command);

            static constexpr ui::OptionSpec first_quantities[] { { "mechanical", "Mechanical", {}, {} }, { "speed", "Speed", {}, {} } };
            static constexpr ui::OptionSpec second_quantities[] { { "speed", "Speed", {}, {} }, { "lab_clock", "Lab clock t", {}, {} } };
            static const ui::ControlSpec first_checklist { "test.options.quantities", "Quantities", "Tests", ui::ControlKind::checklist, ui::UiCommandKind::none, {}, {}, first_quantities };
            static const ui::ControlSpec second_checklist { "test.options.quantities", "Quantities", "Tests", ui::ControlKind::checklist, ui::UiCommandKind::none, {}, {}, second_quantities };
            const std::string_view defaults[] { "speed" };
            (void)builder.checklist(second_list ? second_checklist : first_checklist, defaults);

            static constexpr ui::OptionSpec first_picks[] { { "alpha", "Alpha", {}, {} }, { "beta", "Beta", {}, {} } };
            static constexpr ui::OptionSpec second_picks[] { { "alpha", "Alpha", {}, {} }, { "gamma", "Gamma", {}, {} } };
            static const ui::ControlSpec first_pick { "test.options.pick", "Pick", "Tests", ui::ControlKind::select, ui::UiCommandKind::set_integrator, {}, {}, first_picks };
            static const ui::ControlSpec second_pick { "test.options.pick", "Pick", "Tests", ui::ControlKind::select, ui::UiCommandKind::set_integrator, {}, {}, second_picks };
            builder.select_row(second_list ? second_pick : first_pick, "alpha", command);
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

    // A speed curve as the Relativity tab draws one: kinetic energy in units of mc², Einstein's
    // (γ − 1) solid and Newton's ½β² dashed, against v/c up to c with a limit at c, or against
    // rapidity near c on a log axis.
    class CurvePanel final : public ui::Panel
    {
    public:
        ui::PlotData plot;
        std::vector<float> speeds, relativity, newton;

        explicit CurvePanel(bool near_c)
        {
            const auto sample = [&](float x, const std::optional<physics::LorentzFactors>& factors)
            {
                speeds.push_back(x);
                relativity.push_back(static_cast<float>(factors->lorentz_factor_minus_one));
                newton.push_back(static_cast<float>(0.5 * factors->speed_fraction * factors->speed_fraction));
            };
            const auto speed_text = [](double fraction)
            {
                return core::format_quantity(fraction, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
            };
            if (near_c)
            {
                for (int index = 0; index <= 400; ++index)
                {
                    const auto rapidity = physics::maximum_rapidity() * index / 400.0;
                    sample(static_cast<float>(rapidity), physics::lorentz_factors_from_rapidity(rapidity));
                }
                plot.x = { "Speed (each step adds a 9)", "", 0.0, physics::maximum_rapidity(), ui::AxisScale::linear, {}, "v/c", ui::AxisReadout::rapidity_speed_fraction };
                for (const auto speed : { 0.0, 0.9, 0.99, 0.999, 0.9999, 0.99999, 0.999999, physics::maximum_speed_fraction })
                    plot.x.ticks.push_back({ physics::rapidity_from_speed_fraction(speed), speed == 0.0 ? std::string("0") : speed_text(speed) });
                plot.y = { "Kinetic energy", "mc\xC2\xB2", 1.0e-3, 1.0e4, ui::AxisScale::log10 };
            }
            else
            {
                for (int index = 0; index <= 90; ++index)
                    sample(static_cast<float>(index * 0.01), physics::lorentz_factors(index * 0.01));
                for (int index = 5; index <= 28; ++index)
                {
                    const auto gap = std::pow(10.0, -0.25 * index);
                    sample(static_cast<float>(1.0 - gap), physics::lorentz_factors_from_gap(gap));
                }
                plot.x = { "Speed", "", 0.0, 1.0, ui::AxisScale::linear, {}, "v/c", ui::AxisReadout::speed_fraction };
                plot.x.ticks.push_back({ 0.0, "0" });
                for (const auto speed : { 0.2, 0.4, 0.6, 0.8 })
                    plot.x.ticks.push_back({ speed, speed_text(speed) });
                plot.y = { "Kinetic energy", "mc\xC2\xB2", 0.0, 5.0 };
                plot.markers.push_back({ 1.0, "c", ui::PlotMarkerKind::limit });
            }
            plot.series.push_back({ "Kinetic energy", relativity.data(), speeds.data(), speeds.size(), ui::SeriesStyle::solid, 1, "Relativity", "mc\xC2\xB2" });
            plot.series.push_back({ "Kinetic energy", newton.data(), speeds.data(), speeds.size(), ui::SeriesStyle::dashed, 1, "Newton", "mc\xC2\xB2" });
            plot.subject = "Probe, rest mass 1.00\xC2\xA0kg";
        }

        std::string_view id() const override
        {
            return "curve-test";
        }
        std::string_view title() const override
        {
            return "Speed curve";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            builder.plot("measure.relativity.plot", plot);
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

    RIGIDBODIES_TEST("number drafts keep their displayed units when the model and preferences change")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        panel->mass = 0.005;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        fixture.replace_text("2");
        panel->mass = 3.0;
        ++fixture.model.change_serial;
        fixture.build();
        auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1, "the draft remains editable across live refreshes");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 0.002, 1e-12, "bare digits retain the original grams instead of becoming kilograms");

        panel->mass = 1.0;
        ++fixture.model.change_serial;
        fixture.build();
        fixture.replace_text("2");
        fixture.model.display_units = core::DisplayUnits::centimetre_gram;
        ++fixture.model.change_serial;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "2" }, "changing units preserves an unfinished draft");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1, "the preference change leaves one confirmable edit");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 2.0, 1e-12, "the draft still means the kilograms shown when typing began");
    }

    RIGIDBODIES_TEST("reusing a number row for another subject cancels the old draft")
    {
        for (const auto group_selection : { false, true })
        {
            Fixture fixture;
            auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
            if (group_selection)
                fixture.model.selected_bodies = { { 1, 1 }, { 2, 1 } };
            else
                panel->target = { 1, 1 };
            fixture.build();
            (void)fixture.key(ui::UiKey::tab);
            fixture.replace_text("9");
            panel->mass = 3.0;
            if (group_selection)
                fixture.model.selected_bodies = { { 2, 1 }, { 3, 1 } };
            else
                panel->target = { 2, 1 };
            ++fixture.model.change_serial;
            fixture.build();
            const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
            RIGIDBODIES_EXPECT(fixture.backend.focused_element() != field, "a changed target ends the previous field edit");
            RIGIDBODIES_EXPECT(fixture.backend.element_value(field) != std::optional<std::string> { "9" }, "the new target displays its own value");
            RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), "reconciliation never submits the old draft to either target");
        }
    }

    RIGIDBODIES_TEST("an unchanged focused number tracks external model updates without submitting its previous value")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        panel->mass = 7.0;
        ++fixture.model.change_serial;
        fixture.build();
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        const auto displayed = core::parse_quantity(fixture.backend.element_value(field).value_or("") + " kg", core::DisplayQuantity::mass, fixture.model.display_units);
        RIGIDBODIES_EXPECT(displayed && *displayed == 7.0, "a clean field follows external edits while focused");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), "confirming a clean field cannot undo the external edit");
    }

    RIGIDBODIES_TEST("opening another document cancels drafts even when scenario and body identifiers match")
    {
        Fixture fixture;
        fixture.model.scenario_id = "same_experiment";
        fixture.model.edit_document = std::make_shared<int>(1);
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        panel->target = { 1, 1 };
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        fixture.replace_text("9");
        fixture.model.edit_document = std::make_shared<int>(2);
        ++fixture.model.change_serial;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.focus_owner() != ui::FocusOwner::text_field, "a different document ends the edit despite reused identifiers");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), "the old draft is never submitted into the replacement document");
    }

    RIGIDBODIES_TEST("number fields support caret movement and selection with arrow keys")
    {
        Fixture fixture;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        fixture.replace_text("12");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::arrow_left).empty(), "moving the caret does not commit a number");
        ui::UiEvent text;
        text.kind = ui::UiEventKind::text_input;
        text.text = "3";
        (void)fixture.event(text);
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "132" }, "Left moves the caret between the digits before typing");

        (void)fixture.key(ui::UiKey::home);
        (void)fixture.key(ui::UiKey::arrow_right);
        ui::UiEvent selection;
        selection.kind = ui::UiEventKind::key_down;
        selection.key = ui::UiKey::arrow_right;
        selection.modifiers.shift = true;
        (void)fixture.event(selection);
        text.text = "4";
        (void)fixture.event(text);
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "142" }, "Right and Shift+Right select only the middle digit for replacement");

        (void)fixture.key(ui::UiKey::end);
        (void)fixture.key(ui::UiKey::arrow_left, true);
        text.text = "0";
        (void)fixture.event(text);
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "0142" }, "Ctrl+Left moves to the start of the numeric word");
    }

    RIGIDBODIES_TEST("number arrow adjustments update the focused field and survive leaving it")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "object.properties.mass") + "--field";
        const auto initial_text = fixture.backend.element_value(field);
        auto commands = fixture.key(ui::UiKey::arrow_up);
        RIGIDBODIES_EXPECT(commands.size() == 1, "Up commits a numeric adjustment");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 1.1, 1.0e-12, "Up takes one logarithmic step from the displayed mass");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) != initial_text, "the focused field displays the new mass immediately");

        commands = fixture.key(ui::UiKey::arrow_up);
        RIGIDBODIES_EXPECT(commands.size() == 1, "a second Up before the next frame also commits");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 1.21, 1.0e-12, "successive key events accumulate instead of repeating the same value");
        panel->mass = commands.front().value;
        fixture.build();
        commands.clear();
        fixture.backend.release_focus(commands);
        RIGIDBODIES_EXPECT(commands.empty(), "leaving the field never commits its old text over the accepted arrow adjustment");
    }

    RIGIDBODIES_TEST("number arrow adjustments retain their value when the display unit changes")
    {
        Fixture fixture;
        auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
        panel->mass = 0.99;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        auto commands = fixture.key(ui::UiKey::arrow_up);
        RIGIDBODIES_EXPECT(commands.size() == 1, "the arrow commits a value that crosses from grams to kilograms");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 1.089, 1.0e-12, "the adjustment keeps its full precision");
        panel->mass = commands.front().value;
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), "confirming unchanged formatted text neither rescales nor rounds the accepted value");
        fixture.build();
        commands.clear();
        fixture.backend.release_focus(commands);
        RIGIDBODIES_EXPECT(commands.empty(), "leaving the refreshed field preserves the precise accepted value");
    }

    RIGIDBODIES_TEST("number adjustments use the typed draft and cancellation restores the last confirmation")
    {
        Fixture fixture;
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto row = ui::element_id("legacy", "object.properties.mass");
        const auto field = row + "--field";
        fixture.replace_text("2 kg");
        auto commands = fixture.key(ui::UiKey::arrow_up);
        RIGIDBODIES_EXPECT(commands.size() == 1, "a valid draft can be adjusted directly");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, 2.2, 1.0e-12, "the arrow steps from the typed value rather than the previous model value");

        fixture.replace_text("abc");
        commands = fixture.key(ui::UiKey::arrow_down);
        RIGIDBODIES_EXPECT(commands.empty() && fixture.backend.element_value(field) == std::optional<std::string> { "abc" }, "an invalid draft is preserved for correction rather than overwritten by an arrow");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(field, "is-invalid") && !fixture.backend.element_text(row + "--field-error").empty(), "invalid adjustment explains the allowed range");
        (void)fixture.key(ui::UiKey::escape);
        RIGIDBODIES_EXPECT(!fixture.backend.element_has_class(field, "is-invalid") && fixture.backend.element_text(row + "--field-error").empty(), "Escape clears both validation state and its message");

        (void)fixture.key(ui::UiKey::tab);
        fixture.replace_text("3 kg");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 3.0, "Enter establishes a new accepted value");
        const auto accepted_text = fixture.backend.element_value(field);
        fixture.replace_text("9 kg");
        (void)fixture.key(ui::UiKey::escape);
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == accepted_text, "Escape before another frame restores the last confirmed value");
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

    RIGIDBODIES_TEST("tilt arrow adjustments retain the straight-down reference angle")
    {
        for (const auto direction : { ui::UiKey::arrow_up, ui::UiKey::arrow_down })
            for (const auto modifiers : { ui::KeyModifiers {}, ui::KeyModifiers { true, false, false }, ui::KeyModifiers { false, false, true } })
            {
                Fixture fixture;
                static_cast<KeyedPanel*>(fixture.panel.get())->tilt_only = true;
                fixture.build();
                (void)fixture.key(ui::UiKey::tab);
                ui::UiEvent event;
                event.kind = ui::UiEventKind::key_down;
                event.key = direction;
                event.modifiers = modifiers;
                const auto commands = fixture.event(event);
                const auto delta = (direction == ui::UiKey::arrow_up ? 1.0 : -1.0) * (modifiers.shift ? 1.0 : modifiers.alt ? 0.01
                                                                                                                            : 0.1);
                RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_gravity_angle_degrees, "the tilt field sends exactly one gravity angle adjustment");
                RIGIDBODIES_EXPECT_NEAR(commands.front().value, -90.0 + delta, 1.0e-12, "ordinary, coarse and fine arrow steps stay relative to straight down");
            }
    }

    RIGIDBODIES_TEST("tilt label scrubbing previews and commits a direction relative to straight down")
    {
        Fixture fixture;
        static_cast<KeyedPanel*>(fixture.panel.get())->tilt_only = true;
        fixture.build();
        const auto label = ui::element_id("legacy", "world.gravity.tilt") + "--label";
        const auto bounds = fixture.backend.element_bounds(label);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the tilt label exposes a scrub target");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x += 8.0;
        auto commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::preview, "scrubbing sends one preview");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, -89.8, 1.0e-12, "a 0.2 degree tilt preview means a -89.8 degree direction");
        pointer.kind = ui::UiEventKind::pointer_up;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::commit, "release commits the preview");
        RIGIDBODIES_EXPECT_NEAR(commands.front().value, -89.8, 1.0e-12, "release preserves the same transformed direction");
    }

    RIGIDBODIES_TEST("disappearing and disabled number rows cancel their gesture before losing the binding")
    {
        for (const auto native_slider : { false, true })
            for (const auto disable : { false, true })
            {
                Fixture fixture;
                fixture.build();
                const auto id = ui::element_id("legacy", "object.properties.mass") + (native_slider ? "--slider" : "--label");
                const auto bounds = fixture.backend.element_bounds(id);
                RIGIDBODIES_EXPECT(bounds.has_value(), "the numeric gesture target exists");
                ui::UiEvent pointer;
                pointer.button = ui::PointerButton::primary;
                pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
                pointer.kind = ui::UiEventKind::pointer_down;
                (void)fixture.event(pointer);
                pointer.kind = ui::UiEventKind::pointer_move;
                pointer.pointer_px.x += 20.0;
                (void)fixture.event(pointer);
                auto* panel = static_cast<KeyedPanel*>(fixture.panel.get());
                panel->number_disabled = disable;
                panel->show_number = disable;
                ++fixture.model.change_serial;
                fixture.build();
                std::vector<ui::UiCommand> commands;
                fixture.backend.take_pending_commands(commands);
                RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::cancel, "reconciliation immediately cancels instead of committing a removed or disabled edit");
                RIGIDBODIES_EXPECT(!fixture.backend.pointer_captured(), "the disappearing control releases pointer capture");
                pointer.kind = ui::UiEventKind::pointer_up;
                RIGIDBODIES_EXPECT(fixture.event(pointer).empty(), "a later release cannot revive or commit the cancelled edit");
            }
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

    RIGIDBODIES_TEST("the near-c read-out names the speed with its nines")
    {
        auto owned = std::make_unique<CurvePanel>(true);
        auto* panel = owned.get();
        panel->plot.cursor_t = physics::rapidity_from_speed_fraction(0.99999);
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().cursor_header == "v/c = 0.99999", "a rapidity read-out names v/c with its nines: " + ui::plot_diagnostics().cursor_header);

        // From here the hover reads the plot itself rather than a time the panel supplies.
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.kind = ui::UiEventKind::pointer_move;
        const auto hover = [&](float content_x)
        {
            const auto before = ui::plot_diagnostics();
            pointer.pointer_px = { before.origin[0] + content_x, before.origin[1] + (before.frame[1] + before.frame[3]) * 0.5f };
            (void)fixture.event(pointer);
            fixture.build();
            return ui::plot_diagnostics();
        };
        const auto click = [&]
        {
            pointer.kind = ui::UiEventKind::pointer_down;
            (void)fixture.event(pointer);
            pointer.kind = ui::UiEventKind::pointer_up;
            (void)fixture.event(pointer);
            pointer.kind = ui::UiEventKind::pointer_move;
            fixture.build();
            return ui::plot_diagnostics();
        };
        const auto reads_below_c = [](const ui::PlotDiagnostics& diagnostics, const std::string& nines, const std::string& where)
        {
            RIGIDBODIES_EXPECT(diagnostics.cursor_header.rfind("v/c = " + nines, 0) == 0 && diagnostics.cursor_header.find('1') == std::string::npos && diagnostics.cursor_header.find("\xE2\x80\x94") == std::string::npos, where + " reads its nines, never c: " + diagnostics.cursor_header);
            RIGIDBODIES_EXPECT(diagnostics.cursor_rows == 2, where + " reads both curves: " + std::to_string(diagnostics.cursor_rows));
        };
        panel->plot.cursor_t.reset();
        panel->plot.operating_x = physics::rapidity_from_speed_fraction(0.99999);
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.operating_px.has_value() && !diagnostics.cursor_t.has_value(), "the operating point is drawn and nothing is hovered yet");
        diagnostics = hover(diagnostics.operating_px.value_or(0.0f));
        RIGIDBODIES_EXPECT(diagnostics.cursor_t && *diagnostics.cursor_t == *panel->plot.operating_x && diagnostics.cursor_header == "v/c = 0.99999", "hovering the operating point names the current speed exactly: " + diagnostics.cursor_header);
        reads_below_c(diagnostics, "0.99999", "the operating point");
        diagnostics = hover(diagnostics.frame[2]);
        RIGIDBODIES_EXPECT(diagnostics.cursor_t && *diagnostics.cursor_t == physics::maximum_rapidity(), "the last rapidity sample lies a hair past the axis and reads at its end");
        reads_below_c(diagnostics, "0.9999999", "the right end near c");
        diagnostics = click();
        RIGIDBODIES_EXPECT(diagnostics.pinned && diagnostics.cursor_t == physics::maximum_rapidity(), "a click at the right end pins there and the pin stays in view");
        RIGIDBODIES_EXPECT(!click().pinned, "a second click releases the pin");

        auto up_to_c = std::make_unique<CurvePanel>(false);
        auto* linear = up_to_c.get();
        linear->plot.operating_x = 0.5;
        fixture.panel = std::move(up_to_c);
        fixture.build();
        fixture.build();
        diagnostics = hover(ui::plot_diagnostics().frame[2]);
        RIGIDBODIES_EXPECT(diagnostics.cursor_t && *diagnostics.cursor_t == static_cast<double>(linear->speeds.back()) && *diagnostics.cursor_t < 1.0, "the c line's pixel snaps to the fastest sample, just below c");
        reads_below_c(diagnostics, "0.999999", "the right end up to c");

        linear->plot.operating_x = 0.99999;
        fixture.build();
        diagnostics = hover(diagnostics.frame[2]);
        RIGIDBODIES_EXPECT(diagnostics.cursor_header == "v/c = 0.99999", "at this scale the right end is the operating point, so it names the current speed: " + diagnostics.cursor_header);
        reads_below_c(diagnostics, "0.99999", "the operating point at the right end");

        linear->plot.operating_x = physics::maximum_speed_fraction;
        fixture.build();
        diagnostics = hover(diagnostics.frame[2]);
        reads_below_c(diagnostics, "0.9999999", "the fastest speed, a hair past the last sample,");

        fixture.panel = std::make_unique<PlotPanel>();
        fixture.build();
        const auto time_bounds = fixture.backend.element_bounds(ui::element_id("legacy", "measure.graph.plot") + "--plot");
        RIGIDBODIES_EXPECT(time_bounds.has_value(), "the time plot is laid out");
        pointer.pointer_px = { time_bounds->minimum.x + time_bounds->width() * 0.92, (time_bounds->minimum.y + time_bounds->maximum.y) * 0.5 };
        (void)fixture.event(pointer);
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().cursor_header == "t = 1.000\xC2\xA0s", "a time axis still reads t in seconds, with a no-break space before the unit: " + ui::plot_diagnostics().cursor_header);
    }

    RIGIDBODIES_TEST("the operating point stays left of the c limit and survives hover")
    {
        auto owned = std::make_unique<CurvePanel>(false);
        owned->plot.operating_x = 0.99999;
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        const auto check = [](const std::string& when)
        {
            const auto diagnostics = ui::plot_diagnostics();
            RIGIDBODIES_EXPECT(diagnostics.operating_px.has_value() && diagnostics.limit_px.has_value(), "the operating point and the c limit are drawn " + when);
            RIGIDBODIES_EXPECT(diagnostics.operating_px && diagnostics.limit_px && *diagnostics.operating_px < *diagnostics.limit_px, "0.99999 c lies left of the c line " + when);
            RIGIDBODIES_EXPECT(diagnostics.operating_points == 1 && diagnostics.off_scale_points == 1, "Newton's value is a point and Einstein's is off the chart " + when);
            return diagnostics;
        };
        auto diagnostics = check("before a hover");
        RIGIDBODIES_EXPECT(!diagnostics.cursor_t.has_value() && diagnostics.marker_labels == 0, "without a hover there is no cursor, and a limit takes no place in the label lane");

        const auto plot = ui::element_id("legacy", "measure.relativity.plot") + "--plot";
        const auto bounds = fixture.backend.element_bounds(plot);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the plot canvas is laid out");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px = { bounds->minimum.x + bounds->width() * 0.5, (bounds->minimum.y + bounds->maximum.y) * 0.5 };
        (void)fixture.event(pointer);
        fixture.build();
        diagnostics = check("during a hover");
        RIGIDBODIES_EXPECT(diagnostics.cursor_t.has_value() && !diagnostics.pinned && diagnostics.cursor_header.rfind("v/c = 0.", 0) == 0, "the hover shows its own read-out beside the operating point");

        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        (void)fixture.event(pointer);
        fixture.build();
        diagnostics = check("with a pin");
        RIGIDBODIES_EXPECT(diagnostics.pinned, "the click pins the cursor");
    }

    RIGIDBODIES_TEST("a value above the range becomes a chevron")
    {
        auto owned = std::make_unique<CurvePanel>(false);
        auto* panel = owned.get();
        panel->plot.operating_x = 0.5;
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.operating_points == 2 && diagnostics.off_scale_points == 0, "at 0.5 c both curves are on the chart");

        panel->plot.operating_x = 0.99;
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.operating_points == 1 && diagnostics.off_scale_points == 1, "at 0.99 c the kinetic energy of 6.09 mc\xC2\xB2 leaves the 5 mc\xC2\xB2 chart as a chevron");

        // The fastest speed lies a hair past the last sample, which still marks both curves.
        panel->plot.operating_x = physics::maximum_speed_fraction;
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(physics::maximum_speed_fraction > static_cast<double>(panel->speeds.back()), "the fastest speed lies beyond the last sample");
        RIGIDBODIES_EXPECT(diagnostics.operating_points == 1 && diagnostics.off_scale_points == 1, "at the fastest speed Newton's value is a point and Einstein's a chevron");
        RIGIDBODIES_EXPECT(diagnostics.operating_px && diagnostics.limit_px && *diagnostics.operating_px < *diagnostics.limit_px, "the fastest speed still lies left of the c line");

        panel->plot.operating_x = 1.5;
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(!diagnostics.operating_px.has_value() && diagnostics.operating_points == 0 && diagnostics.off_scale_points == 0, "an operating point outside the speed range draws nothing");

        panel->plot.operating_x.reset();
        fixture.build();
        RIGIDBODIES_EXPECT(!ui::plot_diagnostics().operating_px.has_value(), "without an operating point none is drawn");
    }

    RIGIDBODIES_TEST("log axes label decades")
    {
        RIGIDBODIES_EXPECT(ui::format_plot_decade(-4) == "0.0001" && ui::format_plot_decade(-1) == "0.1" && ui::format_plot_decade(0) == "1", "small decades are written out");
        RIGIDBODIES_EXPECT(ui::format_plot_decade(1) == "10" && ui::format_plot_decade(3) == "1000", "decades up to a thousand are written out");
        RIGIDBODIES_EXPECT(ui::format_plot_decade(4) == "10\xE2\x81\xB4" && ui::format_plot_decade(12) == "10\xC2\xB9\xC2\xB2", "larger decades use a superscript power");
        RIGIDBODIES_EXPECT(ui::format_plot_decade(-5) == "10\xE2\x81\xBB\xE2\x81\xB5", "smaller decades use a superscript minus");

        auto owned = std::make_unique<CurvePanel>(true);
        auto* panel = owned.get();
        panel->plot.operating_x = 0.0;
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.log_y && diagnostics.y_ticks.minimum == 1.0e-3 && diagnostics.y_ticks.maximum == 1.0e4, "a log axis keeps its range as given");
        RIGIDBODIES_EXPECT(diagnostics.series_drawn == 2, "a zero at rest is a gap, and the rest of each curve is drawn");
        RIGIDBODIES_EXPECT(diagnostics.operating_points == 0 && diagnostics.off_scale_points == 2, "zero energy at rest lies below a log axis, so both curves show a downward chevron");

        panel->plot.operating_x = physics::rapidity_from_speed_fraction(0.99999);
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.operating_points == 2 && diagnostics.off_scale_points == 0, "near c both energies sit inside the decades");

        panel->plot.y.minimum = 0.0;
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.log_y && diagnostics.y_ticks.minimum > 0.0 && diagnostics.y_ticks.maximum > diagnostics.y_ticks.minimum, "a range a log axis cannot show falls back to whole decades around the positive data");
        RIGIDBODIES_EXPECT(std::abs(std::log10(diagnostics.y_ticks.minimum) - std::round(std::log10(diagnostics.y_ticks.minimum))) < 1.0e-9, "the fallback starts on a decade");
    }

    RIGIDBODIES_TEST("explicit ticks replace 1-2-5 ticks")
    {
        auto owned = std::make_unique<CurvePanel>(false);
        auto* panel = owned.get();
        panel->plot.x.ticks.push_back({ 1.5, "1.5 c" });
        panel->plot.y = { "Probe clock rate", "", 0.0, 1.05 };
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        auto diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.explicit_x_ticks == 5, "the five named speed ticks inside the axis replace the 1-2-5 steps, and one beyond c is left out");
        RIGIDBODIES_EXPECT(diagnostics.y_ticks.maximum > 1.05, "1-2-5 value ticks widen a range to whole steps");

        panel->plot.y.ticks = { { 1.0, "1" }, { 0.0, "0" }, { 0.5, "0.5" } };
        fixture.build();
        diagnostics = ui::plot_diagnostics();
        RIGIDBODIES_EXPECT(diagnostics.y_ticks.minimum == 0.0 && diagnostics.y_ticks.maximum == 1.05, "named value ticks keep the range as given");

        fixture.panel = std::make_unique<CurvePanel>(true);
        fixture.build();
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().explicit_x_ticks == 8, "the near-c axis ticks rest and every nine up to the fastest speed");

        fixture.panel = std::make_unique<PlotPanel>();
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().explicit_x_ticks == 0, "a time axis keeps its 1-2-5 steps");
    }

    RIGIDBODIES_TEST("a limit is summarised, not counted as a change")
    {
        static constexpr float speeds[] { 0.0f, 0.5f, 0.9f };
        static constexpr float relativity[] { 0.0f, 0.1547f, 1.294f };
        static constexpr float newton[] { 0.0f, 0.125f, 0.405f };
        ui::PlotData data;
        data.x = { "Speed", "", 0.0, 1.0, ui::AxisScale::linear, {}, "v/c", ui::AxisReadout::speed_fraction };
        data.y = { "Kinetic energy", "mc\xC2\xB2", 0.0, 5.0 };
        data.series.push_back({ "Kinetic energy", relativity, speeds, 3, ui::SeriesStyle::solid, 1, "Relativity", "mc\xC2\xB2" });
        data.series.push_back({ "Kinetic energy", newton, speeds, 3, ui::SeriesStyle::dashed, 1, "Newton", "mc\xC2\xB2" });
        data.markers.push_back({ 1.0, "c", ui::PlotMarkerKind::limit });
        data.operating_x = 0.99999;
        const std::string drawn = "Graph of Kinetic energy (mc\xC2\xB2) against Speed, showing Kinetic energy (Relativity) and Kinetic energy (Newton).";
        const std::string limit = " c is marked as a limit the curves approach but never reach.";
        const std::string speed = " The current speed is 0.99999\xC2\xA0"
                                  "c.";
        RIGIDBODIES_EXPECT(ui::plot_summary(data) == drawn + limit + speed, "the limit and the current speed are named: " + ui::plot_summary(data));

        data.markers.push_back({ 0.5, "Speed 0\xC2\xA0"
                                      "c \xE2\x86\x92 0.5\xC2\xA0"
                                      "c",
            ui::PlotMarkerKind::intervention });
        RIGIDBODIES_EXPECT(ui::plot_summary(data) == drawn + " 1 change marked." + limit + speed, "only the intervention counts as a change: " + ui::plot_summary(data));

        data.x.readout = ui::AxisReadout::rapidity_speed_fraction;
        data.operating_x = physics::rapidity_from_speed_fraction(0.99999);
        RIGIDBODIES_EXPECT(ui::plot_summary(data).find(speed) != std::string::npos, "a rapidity axis names the same speed");

        data.x.readout = ui::AxisReadout::number;
        RIGIDBODIES_EXPECT(ui::plot_summary(data).find("current speed") == std::string::npos, "a plain number axis names no speed");
    }

    RIGIDBODIES_TEST("scale, ticks and operating point change the data signature")
    {
        static constexpr float speeds[] { 0.0f, 0.5f, 0.9f };
        static constexpr float values[] { 0.0f, 0.1547f, 1.294f };
        ui::PlotData base;
        base.x = { "Speed", "", 0.0, 1.0 };
        base.y = { "Kinetic energy", "mc\xC2\xB2", 0.001, 5.0 };
        base.x.ticks = { { 0.0, "0" }, { 0.5, "0.5\xC2\xA0"
                                              "c" } };
        base.series.push_back({ "Kinetic energy", values, speeds, 3, ui::SeriesStyle::solid, 1, "Relativity", "mc\xC2\xB2" });
        base.markers.push_back({ 1.0, "c", ui::PlotMarkerKind::intervention });
        const auto signature = ui::plot_data_signature(base);
        RIGIDBODIES_EXPECT(ui::plot_data_signature(ui::PlotData(base)) == signature, "an identical copy has the same signature");
        const auto changes = [&](const char* what, const auto& change)
        {
            auto data = base;
            change(data);
            RIGIDBODIES_EXPECT(ui::plot_data_signature(data) != signature, std::string("the signature changes with ") + what);
        };
        changes("the value scale", [](ui::PlotData& data)
            {
                data.y.scale = ui::AxisScale::log10;
            });
        changes("an added tick", [](ui::PlotData& data)
            {
                data.x.ticks.push_back({ 0.9, "0.9 c" });
            });
        changes("a tick value", [](ui::PlotData& data)
            {
                data.x.ticks[1].value = 0.6;
            });
        changes("a tick label", [](ui::PlotData& data)
            {
                data.x.ticks[1].label = "0.6 c";
            });
        changes("a value tick", [](ui::PlotData& data)
            {
                data.y.ticks.push_back({ 1.0, "1" });
            });
        changes("the read-out symbol", [](ui::PlotData& data)
            {
                data.x.symbol = "v/c";
            });
        changes("the read-out form", [](ui::PlotData& data)
            {
                data.x.readout = ui::AxisReadout::speed_fraction;
            });
        changes("a marker's kind", [](ui::PlotData& data)
            {
                data.markers.front().kind = ui::PlotMarkerKind::limit;
            });
        changes("an operating point", [](ui::PlotData& data)
            {
                data.operating_x = 0.5;
            });
        auto moved = base;
        moved.operating_x = 0.5;
        const auto at_half = ui::plot_data_signature(moved);
        moved.operating_x = 0.9;
        RIGIDBODIES_EXPECT(ui::plot_data_signature(moved) != at_half, "the signature changes when the operating point moves");
        moved.operating_x = 0.0;
        RIGIDBODIES_EXPECT(ui::plot_data_signature(moved) != signature, "an operating point at zero differs from none");

        // The element redraws when only the scale changes.
        auto owned = std::make_unique<CurvePanel>(false);
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        fixture.build();
        RIGIDBODIES_EXPECT(!ui::plot_diagnostics().log_y, "the curve starts on a linear axis");
        panel->plot.y.scale = ui::AxisScale::log10;
        panel->plot.y.minimum = 1.0e-3;
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().log_y, "switching the scale redraws the plot");
        panel->plot.y.minimum = 0.0;
        panel->plot.y.scale = ui::AxisScale::linear;
        fixture.build();
        panel->plot.y.scale = ui::AxisScale::log10;
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().log_y, "the scale alone redraws the plot");
    }

    RIGIDBODIES_TEST("plot text is laid out again when a later label brings new glyphs")
    {
        // In a fresh document the value labels are laid out before the speed ticks bring "c" and
        // a no-break space into the same face, which rebuilds its glyph texture; the value labels
        // must not keep pointing at the old one.
        auto owned = std::make_unique<CurvePanel>(false);
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().layout_passes == 2, "the first layout is redone once the speed ticks add their glyphs: " + std::to_string(ui::plot_diagnostics().layout_passes));
        panel->plot.operating_x = 0.5;
        fixture.build();
        RIGIDBODIES_EXPECT(ui::plot_diagnostics().layout_passes == 1, "with every glyph in place one pass is enough");
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

    RIGIDBODIES_TEST("Probe speed accepts c, percent and kilometres per second")
    {
        auto owned = std::make_unique<SpeedPanel>();
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto row = ui::element_id("legacy", "world.relativity.speed");
        const auto field = row + "--field";
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == field, "the speed field takes focus");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == std::optional<std::string> { "0.9" }, "the field shows the speed as a fraction of c");
        struct Entry
        {
            const char* text;
            double expected;
        };
        for (const auto& entry : { Entry { "0.5 c", 0.5 }, Entry { "60 %", 0.6 }, Entry { "150000 km/s", 1.5e8 / 299792458.0 }, Entry { "99.999%", 0.99999 }, Entry { "299792158 m/s", 299792158.0 / 299792458.0 }, Entry { "0.25", 0.25 } })
        {
            fixture.replace_text(entry.text);
            const auto commands = fixture.key(ui::UiKey::enter);
            RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_relativity_speed && commands.front().phase == ui::UiEditPhase::commit, std::string("one speed command for ") + entry.text);
            RIGIDBODIES_EXPECT_NEAR(commands.front().value, entry.expected, 1.0e-15, std::string("the typed speed converts to a fraction of c: ") + entry.text);
            RIGIDBODIES_EXPECT(!fixture.backend.element_has_class(field, "is-invalid"), std::string("an accepted speed is not marked invalid: ") + entry.text);
            panel->speed = commands.front().value;
            ++fixture.model.change_serial;
            fixture.build();
        }
    }

    RIGIDBODIES_TEST("typing 1 c is refused with the never-reach-c message and sends nothing")
    {
        Fixture fixture(std::make_unique<SpeedPanel>());
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto row = ui::element_id("legacy", "world.relativity.speed");
        const auto field = row + "--field";
        const std::string message = "Probe speed must be from 0\xC2\xA0"
                                    "c to 0.9999999\xC2\xA0"
                                    "c. A massive object can get ever closer to c but never reach it.";
        for (const auto* text : { "1 c", "1", "100 %", "300000 km/s", "1.0000001", "-0.1", "abc", "c" })
        {
            fixture.replace_text(text);
            RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), std::string("nothing is sent for ") + text);
            RIGIDBODIES_EXPECT(fixture.backend.element_has_class(field, "is-invalid"), std::string("the field is marked invalid for ") + text);
            RIGIDBODIES_EXPECT(fixture.backend.element_text(row + "--field-error") == message, std::string("one message explains the limit for ") + text + ": " + fixture.backend.element_text(row + "--field-error"));
            RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::arrow_up).empty(), std::string("an arrow cannot step from a refused entry: ") + text);
        }
        fixture.replace_text("0.9999999");
        const auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == physics::maximum_speed_fraction, "the maximum itself is accepted");
        RIGIDBODIES_EXPECT(!fixture.backend.element_has_class(field, "is-invalid") && fixture.backend.element_text(row + "--field-error").empty(), "an accepted entry clears the message");
    }

    RIGIDBODIES_TEST("a speed too slow to set says what to type, and rest and the slowest speed reach each other")
    {
        auto owned = std::make_unique<SpeedPanel>();
        auto* panel = owned.get();
        panel->speed = 0.0;
        Fixture fixture(std::move(owned));
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto row = ui::element_id("legacy", "world.relativity.speed");
        const auto field = row + "--field";
        const std::string message = "Probe speed must be 0\xC2\xA0"
                                    "c for rest or at least 0.000000000001\xC2\xA0"
                                    "c.";
        for (const auto* text : { "1e-13", "0.0001 m/s", "1e-300" })
        {
            fixture.replace_text(text);
            RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), std::string("nothing is sent for ") + text);
            RIGIDBODIES_EXPECT(fixture.backend.element_has_class(field, "is-invalid") && fixture.backend.element_text(row + "--field-error") == message, std::string("the message says what to type for ") + text + ": " + fixture.backend.element_text(row + "--field-error"));
        }
        // Rest and the slowest moving speed are less than a millionth of a millionth apart, and
        // each is still sent from the other.
        fixture.replace_text("1e-12");
        auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == physics::minimum_moving_speed_fraction && !fixture.backend.element_has_class(field, "is-invalid"), "the slowest moving speed is sent from rest");
        panel->speed = physics::minimum_moving_speed_fraction;
        ++fixture.model.change_serial;
        fixture.build();
        fixture.replace_text("0");
        commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.0, "rest is sent from the slowest moving speed");
    }

    RIGIDBODIES_TEST("the rapidity slider previews then commits one quantised value")
    {
        auto owned = std::make_unique<SpeedPanel>();
        auto* panel = owned.get();
        panel->speed = 0.0;
        Fixture fixture(std::move(owned));
        fixture.build();
        const auto slider = ui::element_id("legacy", "world.relativity.speed") + "--slider";
        const auto bounds = fixture.backend.element_bounds(slider);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the speed slider is rendered");
        const auto middle_y = (bounds->minimum.y + bounds->maximum.y) * 0.5;
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = { bounds->minimum.x + bounds->width() * 0.35, middle_y };
        pointer.kind = ui::UiEventKind::pointer_down;
        auto commands = fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.6;
        const auto moved = fixture.event(pointer);
        commands.insert(commands.end(), moved.begin(), moved.end());
        std::size_t previews = 0;
        for (const auto& command : commands)
            previews += command.phase == ui::UiEditPhase::preview;
        RIGIDBODIES_EXPECT(previews <= 1, "one frame sends at most one preview");
        fixture.wall_time += 1.0 / 60.0;
        fixture.build();
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.8;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::preview && commands.front().kind == ui::UiCommandKind::set_relativity_speed, "dragging previews the speed");
        const auto previewed = commands.empty() ? -1.0 : commands.front().value;
        RIGIDBODIES_EXPECT(previewed > 0.5 && previewed < 0.99999 && reads_as_itself(previewed), "the previewed speed is below the slider's end and reads as itself");
        pointer.kind = ui::UiEventKind::pointer_up;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::commit && commands.front().value == previewed, "release commits exactly the previewed speed");

        // Dragging past the end lands on 0.99999 c exactly, never on c.
        panel->speed = previewed;
        ++fixture.model.change_serial;
        fixture.wall_time += 1.0 / 60.0;
        fixture.build();
        pointer.kind = ui::UiEventKind::pointer_down;
        pointer.pointer_px.x = bounds->minimum.x + bounds->width() * 0.5;
        (void)fixture.event(pointer);
        fixture.wall_time += 1.0 / 60.0;
        fixture.build();
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x = bounds->maximum.x + 60.0;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(!commands.empty() && commands.back().phase == ui::UiEditPhase::commit && commands.back().value == 0.99999, "the slider's end is 0.99999 c exactly");
    }

    RIGIDBODIES_TEST("arrows step rapidity, Shift climbs the ladder, Alt moves the last digit")
    {
        auto owned = std::make_unique<SpeedPanel>();
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        (void)fixture.key(ui::UiKey::tab);
        const auto field = ui::element_id("legacy", "world.relativity.speed") + "--field";
        const auto& number = probe_speed_spec().number;
        const auto press = [&](ui::UiKey key, ui::KeyModifiers modifiers)
        {
            ui::UiEvent event;
            event.kind = ui::UiEventKind::key_down;
            event.key = key;
            event.modifiers = modifiers;
            return fixture.event(event);
        };
        const auto shift = ui::KeyModifiers { true, false, false };
        const auto alt = ui::KeyModifiers { false, false, true };
        const auto set_speed = [&](double speed)
        {
            panel->speed = speed;
            ++fixture.model.change_serial;
            fixture.build();
        };

        auto commands = press(ui::UiKey::arrow_up, {});
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_relativity_speed && commands.front().phase == ui::UiEditPhase::commit, "Up commits one speed");
        const auto stepped = commands.front().value;
        RIGIDBODIES_EXPECT(stepped == ui::keyboard_step(number, 0.9, 1, ui::StepSize::normal) && stepped > 0.9 && stepped < 0.99 && reads_as_itself(stepped), "Up takes one rapidity step and lands on its own text");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(field) == core::format_value(stepped, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si), "the field shows the stepped speed at once");
        commands = press(ui::UiKey::arrow_up, shift);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.99, "Shift+Up climbs to the next speed on the ladder");
        commands = press(ui::UiKey::arrow_down, shift);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.9, "Shift+Down climbs back down");
        commands = press(ui::UiKey::arrow_down, alt);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.899, "Alt+Down moves the last digit");

        set_speed(0.99999);
        commands = press(ui::UiKey::arrow_up, alt);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.999991, "Alt+Up from 0.99999 adds a digit");
        commands = press(ui::UiKey::arrow_down, alt);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.99999, "Alt+Down comes back to 0.99999");
        set_speed(0.99999);
        commands = press(ui::UiKey::arrow_down, alt);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.999989, "Alt+Down from 0.99999 takes one away");

        set_speed(physics::maximum_speed_fraction);
        for (const auto modifiers : { ui::KeyModifiers {}, shift, alt })
        {
            commands = press(ui::UiKey::arrow_up, modifiers);
            RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == physics::maximum_speed_fraction, "no arrow passes the maximum");
        }
        set_speed(0.0);
        commands = press(ui::UiKey::arrow_down, {});
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.0, "rest is the bottom");
        commands = press(ui::UiKey::arrow_up, {});
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().value == 0.114, "one step from rest is 0.114 c");

        // Scrubbing the label takes the same steps, one per four pixels.
        set_speed(0.9);
        const auto label = ui::element_id("legacy", "world.relativity.speed") + "--label";
        const auto bounds = fixture.backend.element_bounds(label);
        RIGIDBODIES_EXPECT(bounds.has_value(), "the speed label is a scrub target");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
        pointer.kind = ui::UiEventKind::pointer_down;
        (void)fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_move;
        pointer.pointer_px.x += 8.0;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::preview && commands.front().value == ui::scrub_value(number, 0.9, 2, ui::StepSize::normal) && commands.front().value > 0.9, "an eight-pixel scrub previews two rapidity steps");
        pointer.kind = ui::UiEventKind::pointer_up;
        commands = fixture.event(pointer);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().phase == ui::UiEditPhase::commit && reads_as_itself(commands.front().value), "release commits a speed that reads as itself");
    }

    RIGIDBODIES_TEST("option buttons rebuild when the option list changes")
    {
        auto owned = std::make_unique<OptionsPanel>();
        auto* panel = owned.get();
        Fixture fixture(std::move(owned));
        fixture.build();
        const auto tabs = ui::element_id("legacy", "test.options.tabs");
        const auto choice = ui::element_id("legacy", "test.options.choice");
        const auto checklist = ui::element_id("legacy", "test.options.quantities");
        const auto select = ui::element_id("legacy", "test.options.pick") + "--field";
        const auto option_ids = [](const std::string& row, std::initializer_list<const char*> ids)
        {
            std::vector<std::string> result;
            for (const auto* id : ids)
                result.push_back(row + "--option_" + id);
            return result;
        };
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(tabs + "--options") == option_ids(tabs, { "energy", "graph", "theory" }), "the first tab set is built");
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(choice + "--options") == option_ids(choice, { "one", "two" }), "the first segments are built");
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(checklist + "--options") == option_ids(checklist, { "mechanical", "speed" }), "the first checklist is built");
        RIGIDBODIES_EXPECT(fixture.backend.select_option_values(select) == std::vector<std::string> { "alpha", "beta" }, "the first select options are built");

        for (int press = 0; press < 30 && fixture.backend.focused_element() != choice + "--option_two"; ++press)
            (void)fixture.key(ui::UiKey::tab);
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == choice + "--option_two", "a segment takes keyboard focus");
        fixture.build();
        auto changes = fixture.backend.structure_change_count();
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.structure_change_count() == changes, "unchanged option lists are left alone");

        panel->second_list = true;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(tabs + "--options") == option_ids(tabs, { "relativity", "graph", "runs" }), "a new tab set replaces the old tabs");
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(choice + "--options") == option_ids(choice, { "two", "three" }), "new segments replace the old ones");
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(checklist + "--options") == option_ids(checklist, { "speed", "lab_clock" }), "a new quantity list replaces the old one");
        RIGIDBODIES_EXPECT(fixture.backend.element_text(checklist + "--option_lab_clock") == "Lab clock t", "new checklist options carry their labels");
        RIGIDBODIES_EXPECT(fixture.backend.select_option_values(select) == std::vector<std::string> { "alpha", "gamma" }, "new select options replace the old ones");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(select) == std::optional<std::string> { "alpha" }, "the select keeps its chosen option");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(tabs + "--option_relativity", "selected"), "the new default tab is selected");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == choice + "--option_two", "focus stays on a segment whose id survives");
        fixture.build();
        changes = fixture.backend.structure_change_count();
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.structure_change_count() == changes, "the rebuilt lists are then left alone");

        const auto bounds = fixture.backend.element_bounds(choice + "--option_three");
        RIGIDBODIES_EXPECT(bounds.has_value(), "the new segment is laid out");
        ui::UiEvent pointer;
        pointer.button = ui::PointerButton::primary;
        pointer.pointer_px = (bounds->minimum + bounds->maximum) * 0.5;
        pointer.kind = ui::UiEventKind::pointer_down;
        auto commands = fixture.event(pointer);
        pointer.kind = ui::UiEventKind::pointer_up;
        const auto released = fixture.event(pointer);
        commands.insert(commands.end(), released.begin(), released.end());
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_integrator && commands.front().id == "three", "a new segment sends its own id");

        panel->second_list = false;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.child_ids(choice + "--options") == option_ids(choice, { "one", "two" }) && fixture.backend.select_option_values(select) == std::vector<std::string> { "alpha", "beta" }, "the first lists return when the model does");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() != choice + "--option_three", "focus never stays on a removed option");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
