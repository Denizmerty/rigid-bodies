#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/ui/panels.hpp>

#include "test_framework.hpp"

#include <regex>

namespace
{
    using namespace rigidbodies;
    std::vector<std::string> raw_body_identifiers;

    class TextDevice final : public render::RenderDevice
    {
    public:
        std::string_view backend_name() const override
        {
            return "text";
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
            return static_cast<float>(text.size()) * 8.0f * scale;
        }
        float text_line_height(float scale) const override
        {
            return 10.0f * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    void check_text(std::string_view text)
    {
        static const std::regex e_notation { "e[+-][0-9]" };
        static const std::regex ascii_negative { "-[0-9]" };
        static const std::regex unicode_negative_zero { "\xE2\x88\x92"
                                                        "0(?:\\.0+)?(?:[^0-9.]|$)" };
        const std::string owned { text };
        RIGIDBODIES_EXPECT(!std::regex_search(owned, e_notation), "learner text has no e notation: " + owned);
        RIGIDBODIES_EXPECT(!std::regex_search(owned, ascii_negative), "learner text has no ASCII numeric minus: " + owned);
        RIGIDBODIES_EXPECT(!std::regex_search(owned, unicode_negative_zero), "learner text has no Unicode negative zero: " + owned);
        for (const auto& identifier : raw_body_identifiers)
            RIGIDBODIES_EXPECT(owned.find(identifier) == std::string::npos, "learner text has no raw body identifier: " + owned);
        for (const auto forbidden : { "m/s2", "m/s^2", "kg/m3", "kg m^2", "N m", "Pa s", "deg", "unnamed" })
            RIGIDBODIES_EXPECT(owned.find(forbidden) == std::string::npos, "learner text has no forbidden spelling: " + owned);
    }

    std::vector<ui::PanelRow> rows_for(ui::Panel& panel, const ui::UiModel& model, TextDevice& device)
    {
        render::Theme theme;
        render::DrawList draw;
        std::vector<ui::Hotspot> hotspots;
        std::vector<ui::PanelRow> rows;
        ui::PanelBuilder builder { theme, device, 1.0f, { { 0.0, 0.0 }, { 1000.0, 2000.0 } }, draw, hotspots };
        builder.record_rows(rows);
        panel.build(model, builder);
        return rows;
    }

    void scan_model(const ui::UiModel& base, app::SimulationSession& session, TextDevice& device)
    {
        raw_body_identifiers.clear();
        session.world().for_each_body([](physics::BodyId, const physics::RigidBody& body)
            {
                if (body.name().find('_') != std::string::npos)
                    raw_body_identifiers.push_back(body.name());
            });
        auto panels = ui::create_default_panels();
        bool full_summary_found = base.scenario_summary.empty();
        for (auto& panel : panels)
        {
            const auto build_and_scan = [&](const ui::UiModel& model)
            {
                for (const auto& row : rows_for(*panel, model, device))
                {
                    check_text(row.text);
                    check_text(row.value);
                    check_text(row.hint);
                    // The Guide prints the summary as a paragraph; the Library's selected card
                    // carries it as the card's description.
                    if ((row.kind == ui::PanelRowKind::paragraph && row.text == base.scenario_summary) || (row.kind == ui::PanelRowKind::list_item && row.hint == base.scenario_summary))
                        full_summary_found = true;
                }
            };

            auto model = base;
            if (panel->id() == "object_inspector")
            {
                for (const auto id : session.world().body_ids())
                {
                    model.selection = id;
                    build_and_scan(model);
                }
            }
            else
                build_and_scan(model);
        }
        RIGIDBODIES_EXPECT(full_summary_found, "the complete scenario summary is retained in recorded rows");

        auto& settings = session.scene_settings();
        settings.display_units = base.display_units;
        settings.layers.set(render::VisualizationLayer::labels, true);
        render::DrawList scene;
        session.render(scene);
        check_text(scene.text_buffer());
    }

    RIGIDBODIES_TEST("all catalogue panels and scene labels obey learner text rules")
    {
        TextDevice device;
        for (const auto& scenario : physics::available_scenarios())
            for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
                for (const auto at_two_seconds : { false, true })
                {
                    app::SimulationSession session;
                    session.configure({});
                    session.set_viewport({ 1600, 900 });
                    RIGIDBODIES_EXPECT(session.load_scenario(scenario.id), "catalogue scenario loads");
                    if (at_two_seconds)
                        for (int step = 0; step < 240; ++step)
                            session.world().step(1.0 / 120.0);
                    auto model = session.build_model();
                    model.display_units = units;
                    scan_model(model, session, device);
                }
    }

    // A probe reading that would read as c itself: "1 c", "1.0 c" or "100 %" standing alone.
    void check_not_light_speed(std::string_view text, std::string_view light_digits, std::string_view label, const std::string& context)
    {
        static const std::regex whole_c { "(^|[^0-9.,])1(\\.0+)?(\\s|\xC2\xA0"
                                          ")c([^a-z]|$)" };
        static const std::regex whole_percent { "(^|[^0-9.,])100(\\.0+)?(\\s|\xC2\xA0)%" };
        const std::string owned { text };
        RIGIDBODIES_EXPECT(!std::regex_search(owned, whole_c), "a relativity reading never reads 1 c: " + owned + context);
        RIGIDBODIES_EXPECT(!std::regex_search(owned, whole_percent), "a relativity reading never reads 100 %: " + std::string(label) + " " + owned + context);
        if (owned.find(light_digits) != std::string::npos)
            RIGIDBODIES_EXPECT(label == "Speed of light c" || label == "Below c by", "only c itself, or the gap below it at rest, shows c's digits: " + std::string(label) + " " + owned + context);
    }

    RIGIDBODIES_TEST("relativity text never reads as light speed")
    {
        TextDevice device;
        std::vector<double> speeds(physics::speed_fraction_ladder.begin(), physics::speed_fraction_ladder.end());
        speeds.push_back(physics::maximum_speed_fraction);
        speeds.push_back(1.0e-9);
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            for (const auto speed : speeds)
            {
                app::SimulationSession session;
                session.configure({});
                session.set_viewport({ 1600, 900 });
                RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light loads");
                session.scene_settings().display_units = units;
                ui::UiCommand set;
                set.kind = ui::UiCommandKind::set_relativity_speed;
                set.value = speed;
                session.apply(set);
                session.stepper().set_paused(false);
                for (int frame = 0; frame < 120; ++frame)
                    session.advance(1.0 / 60.0);
                auto model = session.build_model();
                model.display_units = units;
                RIGIDBODIES_EXPECT(model.relativity && model.relativity->speed_fraction == speed && model.relativity->lab_time_s > 1.9e-9, "the probe is set and its clocks ran for about 2 ns");
                const auto light_digits = core::format_value(core::speed_of_light_m_s, core::DisplayQuantity::relativistic_speed, units);
                const auto context = " (" + core::format_quantity(speed, core::DisplayQuantity::speed_fraction, units) + (units == core::DisplayUnits::si ? ", SI)" : ", CGS)");
                // The Guide's authored text may name c ("Type 1 c into Probe speed"), and Preferences
                // sizes text in percent; every value, control and reading elsewhere, the Guide's own
                // copies of the speed controls included, must not read as c.
                const auto check_rows = [&](const std::vector<ui::PanelRow>& rows, std::string_view panel_id)
                {
                    for (const auto& row : rows)
                    {
                        check_text(row.text);
                        check_text(row.value);
                        if (panel_id == "preferences" || (panel_id == "guide" && (row.kind == ui::PanelRowKind::paragraph || row.kind == ui::PanelRowKind::checkbox)))
                            continue;
                        for (const auto* text : { &row.text, &row.value, &row.hint })
                            check_not_light_speed(*text, light_digits, row.text, context);
                        if (row.kind == ui::PanelRowKind::number && row.spec)
                            check_not_light_speed(core::format_quantity(row.number_si, row.spec->number.quantity, units), light_digits, row.text, context);
                        for (const auto& option : row.options)
                            check_not_light_speed(option.label, light_digits, row.text, context);
                        if (row.plot)
                        {
                            check_not_light_speed(row.plot->subject, light_digits, {}, context);
                            for (const auto* axis : { &row.plot->x, &row.plot->y })
                                for (const auto& tick : axis->ticks)
                                    check_not_light_speed(tick.label, light_digits, {}, context);
                        }
                    }
                };
                for (auto& panel : ui::create_default_panels())
                {
                    for (const auto* curve : { "energy", "momentum", "gamma", "clock_rate" })
                        for (const auto* range : { "full", "near" })
                        {
                            ui::ViewState view;
                            view.set_active_tab("measure.header.tabs", "relativity");
                            view.set_value("measure.relativity.curve", curve);
                            view.set_value("measure.relativity.range", range);
                            view.open_transient("playback_speed");
                            render::Theme theme;
                            render::DrawList draw;
                            std::vector<ui::Hotspot> hotspots;
                            std::vector<ui::PanelRow> rows;
                            ui::PanelBuilder builder { theme, device, 1.0f, { { 0.0, 0.0 }, { 1000.0, 2000.0 } }, draw, hotspots, &view };
                            builder.record_rows(rows);
                            panel->build(model, builder);
                            check_rows(rows, panel->id());
                            if (panel->id() != "measure")
                                break;
                        }
                }
                // The transport panel the menu no longer shows reads lab time too.
                ui::SimulationControlsPanel transport;
                const auto transport_rows = rows_for(transport, model, device);
                check_rows(transport_rows, transport.id());
                RIGIDBODIES_EXPECT(std::any_of(transport_rows.begin(), transport_rows.end(), [&](const auto& row)
                                       {
                                           return row.text == "Elapsed" && row.value == ui::now_text(model) && row.value.find("ns") != std::string::npos;
                                       }),
                    "the transport's elapsed time reads lab nanoseconds" + context);

                render::DrawList scene;
                session.render(scene);
                for (const auto& command : scene.commands())
                    if (command.kind == render::DrawCommandKind::text)
                    {
                        const auto stage = scene.text_buffer().substr(command.text_offset, command.text_length);
                        check_text(stage);
                        RIGIDBODIES_EXPECT(stage.find('?') == std::string::npos, "stage text has no missing glyphs: " + stage + context);
                        check_not_light_speed(stage, light_digits, "stage", context);
                    }
            }
    }

    RIGIDBODIES_TEST("overlay paragraphs stop on a whole ellipsised line")
    {
        TextDevice device;
        render::Theme theme;
        render::DrawList draw;
        std::vector<ui::Hotspot> hotspots;
        ui::PanelBuilder builder { theme, device, 1.0f, { { 0.0, 0.0 }, { 180.0, 34.0 } }, draw, hotspots };
        builder.paragraph("This full summary is deliberately long enough to require several complete wrapped lines in the overlay backend.");
        std::string last;
        for (const auto& command : draw.commands())
            if (command.kind == render::DrawCommandKind::text)
                last = draw.text_buffer().substr(command.text_offset, command.text_length);
        RIGIDBODIES_EXPECT(last.size() >= 3 && last.compare(last.size() - 3, 3, "\xE2\x80\xA6") == 0, "last visible overlay line ends in an ellipsis");
        RIGIDBODIES_EXPECT(builder.cursor_y() <= 34.0, "paragraph emits only whole lines that fit");
    }

    RIGIDBODIES_TEST("unnamed objects and Ready transport use learner-facing labels")
    {
        TextDevice device;
        physics::World world;
        const auto make_body = [&](physics::BodyType type)
        {
            physics::BodyDefinition definition;
            definition.type = type;
            physics::Collider collider;
            collider.shape = physics::make_circle(0.1);
            definition.colliders.push_back(collider);
            return world.create_body(definition);
        };
        const auto fixed = make_body(physics::BodyType::static_body);
        const auto free = make_body(physics::BodyType::dynamic_body);
        const auto driven = make_body(physics::BodyType::kinematic_body);

        ui::UiModel model;
        model.world = &world;
        ui::InspectorPanel inspector;
        for (const auto [id, expected] : { std::pair { free, std::string_view { "Object 1" } }, std::pair { driven, std::string_view { "Object 2" } }, std::pair { fixed, std::string_view { "Object 3" } } })
        {
            model.selection = id;
            const auto rows = rows_for(inspector, model, device);
            RIGIDBODIES_EXPECT(std::any_of(rows.begin(), rows.end(), [expected_name = expected](const auto& row)
                                   {
                                       return row.text == expected_name || row.value == expected_name;
                                   }),
                "unnamed body uses free, driven, fixed list position");
        }

        ui::SimulationControlsPanel transport;
        model.paused = true;
        auto rows = rows_for(transport, model, device);
        RIGIDBODIES_EXPECT(std::any_of(rows.begin(), rows.end(), [](const auto& row)
                               {
                                   return row.kind == ui::PanelRowKind::action && row.text == "Play";
                               }),
            "Ready transport begins with Play");
        model.paused = false;
        rows = rows_for(transport, model, device);
        RIGIDBODIES_EXPECT(std::any_of(rows.begin(), rows.end(), [](const auto& row)
                               {
                                   return row.kind == ui::PanelRowKind::action && row.text == "Pause";
                               }),
            "running transport begins with Pause");
        RIGIDBODIES_EXPECT(std::none_of(rows.begin(), rows.end(), [](const auto& row)
                               {
                                   return row.text == "Resume" || row.value == "Resume";
                               }),
            "Resume is not learner-visible transport text");
    }

    RIGIDBODIES_TEST("each command and state has one name and one key spelling everywhere")
    {
        app::SimulationSession session;
        session.configure({});
        session.set_viewport({ 1600, 900 });
        RIGIDBODIES_EXPECT(session.load_scenario("ramp"), "ramp loads");
        const auto model = session.build_model();
        const auto find_key = [&](std::string_view description)
        {
            return std::find_if(model.keyboard_reference.begin(), model.keyboard_reference.end(), [&](const auto& entry)
                {
                    return entry.description == description;
                });
        };
        for (const auto& option : ui::find_control_spec("tools.mode")->options)
            RIGIDBODIES_EXPECT(find_key(option.label) != model.keyboard_reference.end(), "the keyboard reference names the " + std::string(option.label) + " tool as the toolbar does");
        for (const auto& entry : model.keyboard_reference)
            RIGIDBODIES_EXPECT(std::count_if(model.keyboard_reference.begin(), model.keyboard_reference.end(), [&](const auto& other)
                                   {
                                       return other.description == entry.description;
                                   }) == 1,
                "a command is listed once, naming all of its keys: " + entry.description);
        const auto redo = find_key("Redo");
        RIGIDBODIES_EXPECT(redo != model.keyboard_reference.end() && redo->chord == "Ctrl+Shift+Z / Ctrl+Y", "Redo is one entry with both of its chords");
        RIGIDBODIES_EXPECT(find_key("Undo") != model.keyboard_reference.end() && find_key("Keyboard shortcuts") != model.keyboard_reference.end(), "Undo and Keyboard shortcuts use the menu's names");
        const auto impact = find_key("Play until next impact");
        RIGIDBODIES_EXPECT(impact != model.keyboard_reference.end() && impact->category == ui::KeyCategory::playback, "Play until next impact is listed with playback, as in the menu");
        const auto remove = find_key("Delete selection");
        RIGIDBODIES_EXPECT(remove != model.keyboard_reference.end() && remove->chord == "Del", "the Delete key is spelled Del, as in the menus");

        physics::World world;
        physics::BodyDefinition definition;
        physics::Collider collider;
        collider.shape = physics::make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto body = world.create_body(definition);
        RIGIDBODIES_EXPECT(!ui::body_moving(*world.find_body(body)), "an object at rest is resting");
        world.find_body(body)->set_linear_velocity({ 0.5, 0.0 });
        RIGIDBODIES_EXPECT(ui::body_moving(*world.find_body(body)), "a moving object is moving");
    }

    RIGIDBODIES_TEST("status, menu, World list and Library say what is selected and what each action needs")
    {
        TextDevice device;
        app::SimulationSession session;
        session.configure({});
        session.set_viewport({ 1600, 900 });
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "free fall loads");
        auto model = session.build_model();
        const auto find_row = [](const std::vector<ui::PanelRow>& rows, auto predicate)
        {
            return std::find_if(rows.begin(), rows.end(), predicate);
        };

        ui::MainMenuPanel menu;
        auto rows = rows_for(menu, model, device);
        const auto frame_selection = find_row(rows, [](const auto& row)
            {
                return row.text == "Frame selection";
            });
        RIGIDBODIES_EXPECT(frame_selection != rows.end() && frame_selection->disabled_reason == "Select an object first.", "the menu's Frame selection is unavailable with nothing selected");

        physics::BodyId selected;
        for (const auto& object : model.objects)
            if (object.kind == "free" && !selected.is_valid())
                selected = object.id;
        session.set_selection(selected);
        model = session.build_model();
        ui::StatusLinePanel status;
        rows = rows_for(status, model, device);
        RIGIDBODIES_EXPECT(find_row(rows, [](const auto& row)
                               {
                                   return row.key == "status.selection" && row.value == "1 selected";
                               }) != rows.end(),
            "the status line says the count is a selection");

        ui::ViewState view;
        view.set_active_tab("inspector.target", "world");
        view.mark_visited("free_fall");
        const auto rows_with_view = [&](ui::Panel& panel)
        {
            render::Theme theme;
            render::DrawList draw;
            std::vector<ui::Hotspot> hotspots;
            std::vector<ui::PanelRow> result;
            ui::PanelBuilder builder { theme, device, 1.0f, { { 0.0, 0.0 }, { 1000.0, 2000.0 } }, draw, hotspots, &view };
            builder.record_rows(result);
            panel.build(model, builder);
            return result;
        };
        ui::InspectorPanel inspector;
        rows = rows_with_view(inspector);
        std::size_t marked = 0;
        for (const auto& row : rows)
            if (row.key == "object.list.row")
            {
                RIGIDBODIES_EXPECT(row.selected == (row.command.body == selected), "only the selected object's row is marked: " + row.text);
                marked += row.selected ? 1 : 0;
            }
        RIGIDBODIES_EXPECT(marked == 1, "the World list marks the selected object");

        for (const auto& description : physics::available_scenarios())
            model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
        ui::LibraryPanel library;
        rows = rows_with_view(library);
        const auto card = find_row(rows, [](const auto& row)
            {
                return row.key == "library.cards.card" && row.instance == "free_fall";
            });
        RIGIDBODIES_EXPECT(card != rows.end() && card->list_content, "the current experiment has a card");
        if (card != rows.end() && card->list_content)
        {
            const auto& badges = card->list_content->badges;
            RIGIDBODIES_EXPECT(std::find(badges.begin(), badges.end(), "Current") != badges.end() && std::find(badges.begin(), badges.end(), "Visited") == badges.end(), "the current card says Current and not also Visited");
            RIGIDBODIES_EXPECT(!card->list_content->actions.empty() && card->list_content->actions.front().label == "Back to experiment" && card->list_content->actions.front().primary, "the selected card carries its own primary action");
        }
        RIGIDBODIES_EXPECT(find_row(rows, [](const auto& row)
                               {
                                   return row.kind == ui::PanelRowKind::action && row.text == "Back to experiment";
                               }) == rows.end(),
            "the action is not repeated beneath the card");
    }

    RIGIDBODIES_TEST("an experiment opens on its recommended view and Show names it")
    {
        TextDevice device;
        for (const auto* scenario : { "ramp", "free_fall", "empty_lab", "magnus_effect" })
        {
            app::SimulationSession session;
            session.configure({});
            session.set_viewport({ 1600, 900 });
            RIGIDBODIES_EXPECT(session.load_scenario("free_fall") && session.load_scenario(scenario), "the experiment loads after another one");
            auto model = session.build_model();
            RIGIDBODIES_EXPECT(model.layers.bits() == model.recommended_layers.bits(), std::string(scenario) + " opens on its recommended layers, not a mixture with the previous experiment's");
            ui::VisualizationPanel show;
            auto rows = rows_for(show, model, device);
            const auto presets = std::find_if(rows.begin(), rows.end(), [](const auto& row)
                {
                    return row.key == "show.presets.preset";
                });
            RIGIDBODIES_EXPECT(presets != rows.end() && presets->selected_option == "recommended", std::string(scenario) + ": an untouched experiment reads Recommended, not Custom");
            ui::UiCommand change;
            change.kind = ui::UiCommandKind::set_layer;
            change.id = "trajectories";
            change.flag = !model.layers.is_enabled(render::VisualizationLayer::trajectories);
            session.apply(change);
            model = session.build_model();
            rows = rows_for(show, model, device);
            RIGIDBODIES_EXPECT(std::any_of(rows.begin(), rows.end(), [](const auto& row)
                                   {
                                       return row.key == "show.presets.preset" && row.selected_option == "custom";
                                   }),
                std::string(scenario) + ": a changed layer reads Custom");
            ui::UiCommand recommended;
            recommended.kind = ui::UiCommandKind::set_layer_mask;
            recommended.id = "recommended";
            session.apply(recommended);
            RIGIDBODIES_EXPECT(session.build_model().layers.bits() == model.recommended_layers.bits(), std::string(scenario) + ": Recommended restores the experiment's view");
        }
        app::SimulationSession session;
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("ramp"), "ramp loads");
        const auto layers = session.build_model().recommended_layers;
        RIGIDBODIES_EXPECT(layers.is_enabled(render::VisualizationLayer::labels) && layers.is_enabled(render::VisualizationLayer::force_vectors), "the ramp's recommended view keeps the labels and forces it teaches with");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
