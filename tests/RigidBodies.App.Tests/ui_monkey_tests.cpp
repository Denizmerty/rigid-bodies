#include "learner_interface_harness.hpp"

#include <rigidbodies/physics/scenario.hpp>

#include <array>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <sstream>

namespace
{
    using namespace rigidbodies;
    using testing::LearnerInterface;
    using K = ui::UiCommandKind;

    // A fixed generator keeps the action sequence identical with every standard library.
    class SplitMix
    {
    public:
        explicit SplitMix(std::uint64_t seed) : state_(seed)
        {
        }
        std::uint64_t next()
        {
            auto value = (state_ += 0x9E3779B97F4A7C15ull);
            value = (value ^ (value >> 30u)) * 0xBF58476D1CE4E5B9ull;
            value = (value ^ (value >> 27u)) * 0x94D049BB133111EBull;
            return value ^ (value >> 31u);
        }
        std::size_t below(std::size_t count)
        {
            return count == 0 ? 0 : static_cast<std::size_t>(next() % count);
        }
        double unit()
        {
            return static_cast<double>(next() >> 11u) * (1.0 / 9007199254740992.0);
        }
        bool chance(double probability)
        {
            return unit() < probability;
        }
        template <typename T, std::size_t N>
        const T& pick(const std::array<T, N>& values)
        {
            return values[below(N)];
        }

    private:
        std::uint64_t state_;
    };

    std::string describe(const ui::UiCommand& command)
    {
        std::ostringstream text;
        text << "command " << static_cast<int>(command.kind) << " id=" << command.id << " detail=" << command.detail << " value=" << command.value << " flag=" << command.flag;
        return text.str();
    }

    std::string point_text(math::Vec2 point)
    {
        std::ostringstream text;
        text << static_cast<int>(point.x) << "," << static_cast<int>(point.y);
        return text.str();
    }

    // A hard crash cannot be caught as an exception, so the most recent actions are printed from
    // the signal handler to keep a failing seed reproducible from the ctest log.
    const std::deque<std::string>* crash_trace = nullptr;

    void print_trace_and_exit(int signal_number)
    {
        std::fprintf(stdout, "\nFATAL signal %d during the interface monkey; recent actions:\n", signal_number);
        if (crash_trace)
            for (const auto& line : *crash_trace)
                std::fprintf(stdout, "        %s\n", line.c_str());
        std::fflush(stdout);
        std::_Exit(3);
    }

    struct Monkey
    {
        LearnerInterface& harness;
        SplitMix random;
        std::deque<std::string> trace;
        std::size_t step { 0 }, next_submit_step { 0 };
        bool echo { std::getenv("RIGIDBODIES_MONKEY_TRACE") != nullptr };

        void note(std::string text)
        {
            if (echo)
            {
                const auto& view = harness.interface.view_state();
                std::cout << "  [" << step << "] " << text << "  {cmds " << harness.commands << " sheets " << view.sheets().size() << " transients " << view.transients().size() << " focus " << static_cast<int>(harness.interface.focus_owner()) << " present " << view.present().mode << " draw " << harness.session.build_model().shape_editor_active << "}" << std::endl;
            }
            trace.push_back(std::to_string(step) + ": " + std::move(text));
            if (trace.size() > 48)
                trace.pop_front();
        }

        [[nodiscard]] std::string recent() const
        {
            std::string text;
            for (const auto& line : trace)
                text.append("\n        ").append(line);
            return text;
        }

        void resize()
        {
            // Narrow, medium and wide logical widths at display densities from 0.75x to 2.5x.
            static constexpr std::array<render::ViewportSize, 11> sizes { { { 2560, 1440 }, { 1920, 1080 }, { 1600, 900 }, { 1366, 768 }, { 1280, 720 }, { 1024, 768 }, { 800, 600 }, { 720, 1280 }, { 640, 480 }, { 480, 320 }, { 360, 640 } } };
            static constexpr std::array<float, 8> scales { 0.75f, 1.0f, 1.25f, 1.5f, 1.75f, 2.0f, 2.25f, 2.5f };
            const auto size = random.pick(sizes);
            const auto scale = random.pick(scales);
            note("resize " + std::to_string(size.width) + "x" + std::to_string(size.height) + " at " + std::to_string(scale));
            harness.resize(size, scale);
        }

        void key()
        {
            struct Chord
            {
                ui::UiKey key;
                ui::KeyModifiers modifiers;
                const char* name;
            };
            static const std::array<Chord, 50> chords { {
                { ui::UiKey::tab, {}, "Tab" },
                { ui::UiKey::tab, {}, "Tab" },
                { ui::UiKey::tab, { true, false, false }, "Shift+Tab" },
                { ui::UiKey::enter, {}, "Enter" },
                { ui::UiKey::enter, {}, "Enter" },
                { ui::UiKey::space, {}, "Space" },
                { ui::UiKey::space, {}, "Space" },
                { ui::UiKey::escape, {}, "Escape" },
                { ui::UiKey::escape, {}, "Escape" },
                { ui::UiKey::escape, {}, "Escape" },
                { ui::UiKey::arrow_left, {}, "Left" },
                { ui::UiKey::arrow_right, {}, "Right" },
                { ui::UiKey::arrow_up, {}, "Up" },
                { ui::UiKey::arrow_down, {}, "Down" },
                { ui::UiKey::arrow_up, { false, false, true }, "Alt+Up" },
                { ui::UiKey::arrow_left, { true, false, true }, "Shift+Alt+Left" },
                { ui::UiKey::home, {}, "Home" },
                { ui::UiKey::end, {}, "End" },
                { ui::UiKey::backspace, {}, "Backspace" },
                { ui::UiKey::f6, {}, "F6" },
                { ui::UiKey::d, {}, "D" },
                { ui::UiKey::m, {}, "M" },
                { ui::UiKey::s, {}, "S" },
                { ui::UiKey::l, {}, "L" },
                { ui::UiKey::g, {}, "G" },
                { ui::UiKey::i, {}, "I" },
                { ui::UiKey::w, {}, "W" },
                { ui::UiKey::r, {}, "R" },
                { ui::UiKey::period, {}, "." },
                { ui::UiKey::period, { true, false, false }, "Shift+." },
                { ui::UiKey::left_bracket, {}, "[" },
                { ui::UiKey::right_bracket, {}, "]" },
                { ui::UiKey::v, {}, "V" },
                { ui::UiKey::t, {}, "T" },
                { ui::UiKey::p, {}, "P" },
                { ui::UiKey::f, {}, "F" },
                { ui::UiKey::delete_key, {}, "Delete" },
                { ui::UiKey::f1, {}, "F1" },
                { ui::UiKey::f5, {}, "F5" },
                { ui::UiKey::f10, {}, "F10" },
                { ui::UiKey::f10, { true, false, false }, "Shift+F10" },
                { ui::UiKey::menu, {}, "Menu" },
                { ui::UiKey::a, { true, false, false }, "Shift+A" },
                { ui::UiKey::a, { false, true, false }, "Ctrl+A" },
                { ui::UiKey::k, { false, true, false }, "Ctrl+K" },
                { ui::UiKey::z, { false, true, false }, "Ctrl+Z" },
                { ui::UiKey::y, { false, true, false }, "Ctrl+Y" },
                { ui::UiKey::comma, { false, true, false }, "Ctrl+," },
                { ui::UiKey::s, { true, true, false }, "Ctrl+Shift+S" },
                { ui::UiKey::g, { false, true, false }, "Ctrl+G" },
            } };
            const auto& chord = chords[random.below(chords.size())];
            const auto repeat = random.chance(0.05);
            note(std::string("key ") + chord.name + (repeat ? " (repeat)" : ""));
            harness.key(chord.key, chord.modifiers, repeat);
        }

        void click_control(ui::PointerButton button)
        {
            const auto controls = harness.visible_controls();
            if (controls.empty())
                return;
            const auto& [id, point] = controls[random.below(controls.size())];
            ui::KeyModifiers modifiers;
            modifiers.shift = random.chance(0.08);
            note(std::string(button == ui::PointerButton::secondary ? "right-click " : "click ") + id + " at " + point_text(point) + (modifiers.shift ? " with Shift" : ""));
            harness.click(point, button, modifiers);
        }

        void click_primary()
        {
            click_control(ui::PointerButton::primary);
        }

        void click_secondary()
        {
            click_control(ui::PointerButton::secondary);
        }

        void stage_click(ui::PointerButton button)
        {
            const auto point = harness.stage_point(0.05 + random.unit() * 0.9, 0.05 + random.unit() * 0.9);
            note(std::string(button == ui::PointerButton::secondary ? "right-click" : "click") + " stage at " + point_text(point));
            harness.click(point, button);
        }

        void stage_primary()
        {
            stage_click(ui::PointerButton::primary);
        }

        void drag()
        {
            const auto controls = harness.visible_controls();
            auto start = harness.stage_point(random.unit(), random.unit());
            if (!controls.empty() && random.chance(0.4))
                start = controls[random.below(controls.size())].second;
            const auto button = random.chance(0.8) ? ui::PointerButton::primary : random.chance(0.5) ? ui::PointerButton::middle
                                                                                                     : ui::PointerButton::secondary;
            const auto end = start + math::Vec2 { (random.unit() - 0.5) * 600.0, (random.unit() - 0.5) * 400.0 };
            note("drag from " + point_text(start) + " to " + point_text(end));
            harness.pointer(ui::UiEventKind::pointer_move, start, button);
            harness.pointer(ui::UiEventKind::pointer_down, start, button);
            const auto moves = 2 + static_cast<int>(random.below(5));
            for (int index = 1; index <= moves; ++index)
            {
                harness.pointer(ui::UiEventKind::pointer_move, start + (end - start) * (static_cast<double>(index) / moves), button);
                if (random.chance(0.3))
                    harness.frame();
            }
            if (random.chance(0.05))
            {
                note("drag interrupted by Escape");
                harness.key(ui::UiKey::escape);
            }
            if (random.chance(0.03))
            {
                note("drag interrupted by losing window focus");
                harness.focus_window(false);
                harness.focus_window(true);
            }
            harness.pointer(ui::UiEventKind::pointer_up, end, button);
        }

        void wheel()
        {
            const auto controls = harness.visible_controls();
            auto point = harness.stage_point(random.unit(), random.unit());
            if (!controls.empty() && random.chance(0.5))
                point = controls[random.below(controls.size())].second;
            const auto delta = random.chance(0.5) ? 1.0 : -2.0;
            note("wheel " + std::to_string(delta) + " at " + point_text(point));
            harness.wheel(point, delta);
        }

        void text()
        {
            // SDL delivers text only while the interface asks for it, so most attempts first
            // press a visible field to give it focus.
            if (!harness.interface.wants_text_input() && random.chance(0.7))
            {
                const auto controls = harness.visible_controls();
                std::vector<std::pair<std::string, math::Vec2>> fields;
                for (const auto& control : controls)
                    if (control.first.size() > 7 && control.first.compare(control.first.size() - 7, 7, "--field") == 0)
                        fields.push_back(control);
                if (!fields.empty())
                {
                    const auto& [id, point] = fields[random.below(fields.size())];
                    note("focus field " + id);
                    harness.click(point);
                }
            }
            if (!harness.interface.wants_text_input())
                return;
            static constexpr std::array<const char*, 9> values { "12.5", "-3", "abc", "0", "1e9", "4,2", "", "\xC3\xBC 2 kg", "99999999999999999999" };
            const std::string typed = random.pick(values);
            note("type \"" + typed + "\"");
            harness.text(typed);
            if (random.chance(0.5))
            {
                const auto commit = random.chance(0.7);
                note(commit ? "commit field with Enter" : "revert field with Escape");
                harness.key(commit ? ui::UiKey::enter : ui::UiKey::escape);
            }
        }

        void view_request()
        {
            const auto request = static_cast<ui::ViewRequest>(random.below(static_cast<std::size_t>(ui::ViewRequest::close_top_surface) + 1));
            static constexpr std::array<const char*, 5> keys { "", "world.gravity.enabled", "measure.graph.scope", "tools.reference", "object.properties.mass" };
            const std::string key = random.chance(0.3) ? random.pick(keys) : "";
            note("view request " + std::to_string(static_cast<int>(request)) + " " + key);
            if (request == ui::ViewRequest::open_context_menu && random.chance(0.6))
                harness.session.request_context_menu_for_selection();
            harness.interface.request(request, key);
        }

        void present()
        {
            switch (random.below(4))
            {
            case 0:
                note("toggle Present");
                harness.interface.request(ui::ViewRequest::toggle_present);
                return;
            case 1:
                note("key F5");
                harness.key(ui::UiKey::f5);
                return;
            case 2:
                note("Present next step");
                harness.key(ui::UiKey::arrow_right);
                return;
            default:
                note("Present previous step");
                harness.key(ui::UiKey::arrow_left);
                return;
            }
        }

        void draw()
        {
            const auto active = harness.session.build_model().shape_editor_active;
            if (!active || random.chance(0.2))
            {
                ui::UiCommand start;
                start.kind = random.chance(0.8) ? K::start_new_shape : K::edit_selected_shape;
                note(start.kind == K::start_new_shape ? "start drawing" : "edit selected shape");
                harness.apply_command(start);
                return;
            }
            if (random.chance(0.55))
            {
                const auto count = 1 + random.below(4);
                note("draw " + std::to_string(count) + " nodes");
                for (std::size_t index = 0; index < count; ++index)
                    harness.click(harness.stage_point(0.2 + random.unit() * 0.6, 0.2 + random.unit() * 0.6));
                return;
            }
            static constexpr std::array<K, 7> actions { K::cancel_shape_outline, K::close_shape_outline, K::commit_shape_outline, K::insert_shape_node, K::remove_shape_node, K::set_shape_snap_angles, K::set_shape_snap_grid };
            ui::UiCommand command;
            command.kind = random.pick(actions);
            command.flag = random.chance(0.5);
            note("draw " + describe(command));
            harness.apply_command(command);
        }

        void select()
        {
            const auto ids = harness.session.world().body_ids();
            ui::UiCommand command;
            const auto choice = random.below(4);
            if (choice == 0 || ids.empty())
                command.kind = K::clear_selection;
            else if (choice == 1)
                command.kind = K::select_all;
            else
            {
                command.kind = K::select_body;
                command.body = ids[random.below(ids.size())];
            }
            note("select " + describe(command));
            harness.apply_command(command);
        }

        void context_menu()
        {
            if (random.chance(0.4))
            {
                note("context menu for the selection");
                harness.handle_action(app::AppAction::open_context_menu);
            }
            else
                stage_click(ui::PointerButton::secondary);
        }

        void setting()
        {
            ui::UiCommand command;
            switch (random.below(7))
            {
            case 0:
            {
                static constexpr std::array<const char*, 3> themes { "workbench_dark", "workbench_light", "workbench_projector" };
                command.kind = K::set_theme;
                command.id = random.pick(themes);
                break;
            }
            case 1:
            {
                static constexpr std::array<double, 6> sizes { 0.75, 1.0, 1.25, 1.5, 1.75, 2.0 };
                command.kind = K::set_ui_scale;
                command.value = random.pick(sizes);
                break;
            }
            case 2:
                command.kind = K::set_preference;
                command.detail = "prefs.accessibility.reduce_motion";
                command.flag = !harness.session.build_model().reduce_motion;
                break;
            case 3:
                note("toggle the performance overlay");
                harness.interface.request(ui::ViewRequest::toggle_performance_overlay);
                return;
            case 4:
                command.kind = random.chance(0.5) ? K::undo : K::redo;
                break;
            case 5:
                command.kind = K::set_display_units;
                command.id = random.chance(0.5) ? "si" : "centimetre_gram";
                break;
            default:
                command.kind = K::set_layer_mask;
                command.id = random.chance(0.5) ? "all" : "recommended";
                break;
            }
            note("setting " + describe(command));
            harness.apply_command(command);
        }

        void scenario()
        {
            const auto scenarios = physics::available_scenarios();
            if (scenarios.empty())
                return;
            ui::UiCommand command;
            command.kind = K::load_scenario;
            command.id = std::string(scenarios[random.below(scenarios.size())].id);
            command.flag = random.chance(0.6);
            note("load " + command.id + (command.flag ? " (forced)" : ""));
            harness.apply_command(command);
        }

        void advance()
        {
            if (harness.session.build_model().paused && random.chance(0.7))
            {
                ui::UiCommand play;
                play.kind = K::toggle_pause;
                harness.apply_command(play);
            }
            static constexpr std::array<double, 3> frame_times { 1.0 / 60.0, 1.0 / 30.0, 0.1 };
            const auto count = 1 + static_cast<int>(random.below(4));
            const auto frame_time = random.pick(frame_times);
            note("advance " + std::to_string(count) + " frames of " + std::to_string(frame_time) + " s");
            harness.frames_for(count, frame_time);
        }

        void notice()
        {
            static constexpr std::array<ui::Severity, 4> severities { ui::Severity::info, ui::Severity::success, ui::Severity::warning, ui::Severity::error };
            note("notification");
            std::optional<ui::NotificationAction> action;
            if (random.chance(0.5))
            {
                ui::NotificationAction undo;
                undo.label = "Undo";
                ui::UiCommand command;
                command.kind = K::undo;
                undo.command = command;
                action = undo;
            }
            harness.session.notify(random.pick(severities), "Monkey notice " + std::to_string(step), "monkey", action);
        }

        void focus()
        {
            const auto lost = random.chance(0.5);
            note(lost ? "window focus lost" : "window focus gained");
            harness.focus_window(!lost);
        }

        void submit()
        {
            // SDL's software rasteriser is slow at the largest viewports, so replay is sampled.
            if (step < next_submit_step)
                return;
            next_submit_step = step + 400;
            note("submit to the software device");
            harness.present_to_device();
        }

        void hover()
        {
            const auto point = harness.stage_point(random.unit(), random.unit());
            note("hover " + point_text(point));
            harness.pointer(ui::UiEventKind::pointer_move, point);
        }

        void pick_surface()
        {
            note("arm surface pick");
            harness.interface.request(ui::ViewRequest::arm_pick_surface);
        }

        void confirmation()
        {
            const auto model = harness.session.build_model();
            if (!model.confirmation)
                return;
            const auto choice = random.below(3);
            const auto& command = choice == 0 ? model.confirmation->confirm : choice == 1 ? model.confirmation->cancel
                                                                                          : model.confirmation->save;
            note("confirmation " + describe(command));
            harness.apply_command(command);
        }

        void act()
        {
            struct Weighted
            {
                int weight;
                void (Monkey::*action)();
            };
            static const std::array<Weighted, 22> actions { {
                { 20, &Monkey::click_primary },
                { 3, &Monkey::click_secondary },
                { 16, &Monkey::key },
                { 5, &Monkey::stage_primary },
                { 3, &Monkey::drag },
                { 2, &Monkey::wheel },
                { 3, &Monkey::text },
                { 7, &Monkey::view_request },
                { 3, &Monkey::present },
                { 4, &Monkey::draw },
                { 3, &Monkey::select },
                { 3, &Monkey::context_menu },
                { 3, &Monkey::resize },
                { 4, &Monkey::setting },
                { 1, &Monkey::scenario },
                { 3, &Monkey::advance },
                { 1, &Monkey::notice },
                { 1, &Monkey::focus },
                { 1, &Monkey::submit },
                { 1, &Monkey::hover },
                { 1, &Monkey::pick_surface },
                { 1, &Monkey::confirmation },
            } };
            int total = 0;
            for (const auto& action : actions)
                total += action.weight;
            auto choice = static_cast<int>(random.below(static_cast<std::size_t>(total)));
            for (const auto& action : actions)
            {
                if (choice < action.weight)
                {
                    (this->*action.action)();
                    return;
                }
                choice -= action.weight;
            }
        }

        void expect_frame_invariants()
        {
            const auto present = harness.interface.view_state().present().mode;
            const auto& backend = harness.document();
            RIGIDBODIES_EXPECT(backend.element_visible("panel-command_bar") == !present, "the command bar is displayed exactly when Present is off");
            RIGIDBODIES_EXPECT(backend.element_visible("panel-present") == present, "the Present strip is displayed exactly when Present is on");
            if (harness.interface.focus_owner() == ui::FocusOwner::text_field)
                RIGIDBODIES_EXPECT(!backend.focused_element().empty() && backend.element_visible(backend.focused_element()), "a field that takes the keys is displayed");
            const auto& layout = harness.interface.layout();
            for (const auto& region : layout.regions)
                RIGIDBODIES_EXPECT(std::isfinite(region.bounds.minimum.x) && std::isfinite(region.bounds.maximum.y) && region.bounds.width() >= 0.0 && region.bounds.height() >= 0.0, "every layout region is finite and non-negative");
        }

        void run(std::size_t steps)
        {
            crash_trace = &trace;
            for (step = 0; step < steps; ++step)
            {
                try
                {
                    act();
                    // Several input events often arrive between two frames.
                    if (random.chance(0.4))
                    {
                        static constexpr std::array<double, 4> frame_times { 1.0 / 60.0, 1.0 / 60.0, 0.0, 1.0 / 30.0 };
                        harness.frame(random.pick(frame_times));
                        expect_frame_invariants();
                    }
                    if (step % 500 == 499)
                        harness.expect_usable("after step " + std::to_string(step));
                }
                catch (const std::exception& error)
                {
                    crash_trace = nullptr;
                    RIGIDBODIES_FAIL(std::string(error.what()) + "\n      at monkey step " + std::to_string(step) + "; recent actions:" + recent());
                }
            }
            crash_trace = nullptr;
        }
    };

    void install_crash_report()
    {
        std::signal(SIGSEGV, print_trace_and_exit);
        std::signal(SIGABRT, print_trace_and_exit);
        std::signal(SIGFPE, print_trace_and_exit);
        std::signal(SIGILL, print_trace_and_exit);
    }

    void run_monkey(std::uint64_t seed, std::size_t steps, const core::ApplicationConfig& config = {})
    {
        install_crash_report();
        LearnerInterface harness(config);
        Monkey monkey { harness, SplitMix(seed), {} };
        const auto started = std::chrono::steady_clock::now();
        monkey.run(steps);
        harness.expect_usable("after the seeded run");
        const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << "      seed " << seed << ": " << steps << " steps, " << harness.frames << " frames, " << harness.commands << " commands, " << harness.meshes << " meshes checked in " << elapsed << " s\n";
        RIGIDBODIES_EXPECT(harness.frames > steps / 4 && harness.meshes > harness.frames, "the run built frames and checked their geometry");
    }

    std::size_t environment_count(const char* name, std::size_t fallback)
    {
        if (const auto* text = std::getenv(name))
            return static_cast<std::size_t>(std::strtoull(text, nullptr, 10));
        return fallback;
    }

    void replay_historical_sequence(render::ViewportSize size, float scale)
    {
        LearnerInterface harness({}, size, scale);
        const auto context = " at " + std::to_string(size.width) + "x" + std::to_string(size.height) + " and " + std::to_string(scale) + "x";
        harness.key(ui::UiKey::m);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().surface_open("measure.open", false), "M opens Measure" + context);
        // Clear of the Measure sheet, which a narrow window raises over the lower stage.
        harness.click(harness.stage_point(0.5, 0.25), ui::PointerButton::secondary);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu"), "a right-click on the stage opens the context menu" + context);
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("context_menu"), "Escape closes the context menu" + context);
        harness.key(ui::UiKey::d);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.session.build_model().shape_editor_active, "D starts drawing" + context);
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.session.build_model().shape_editor_active, "Escape discards an empty drawing" + context);
        const auto paused = harness.session.build_model().paused;
        harness.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(harness.session.build_model().paused != paused, "Space reaches the scene and toggles the simulation" + context);
        harness.frames_for(30);
        harness.expect_usable("after the historical sequence" + context);
    }

    RIGIDBODIES_TEST("historical crash: Measure, stage context menu, Escape, Draw, Escape, Space")
    {
        install_crash_report();
        replay_historical_sequence({ 1600, 900 }, 1.5f);
        replay_historical_sequence({ 1600, 900 }, 1.0f);
        replay_historical_sequence({ 800, 600 }, 1.25f);
    }

    RIGIDBODIES_TEST("the historical sequence survives every Measure tab, a drawn draft and the main menu")
    {
        install_crash_report();
        LearnerInterface harness({}, { 1600, 900 }, 1.5f);
        // Every tab of a Newtonian experiment, then every tab of the relativity experiment, whose
        // tools refuse Newtonian edits.
        for (const auto* tab : { "energy", "graph", "collisions", "runs", "theory", "relativity", "graph", "runs" })
        {
            if (std::string_view(tab) == "relativity")
            {
                RIGIDBODIES_EXPECT(harness.session.load_scenario("chasing_light"), "the relativity experiment loads");
                harness.frame();
            }
            harness.interface.view_state().set_surface_open("measure.open", true);
            harness.interface.view_state().set_active_tab("measure.header.tabs", tab);
            harness.frame();
            harness.click(harness.stage_point(0.4, 0.35), ui::PointerButton::secondary);
            harness.frame();
            harness.key(ui::UiKey::escape);
            harness.frame();
            harness.key(ui::UiKey::d);
            harness.frame();
            harness.click(harness.stage_point(0.3, 0.3));
            harness.click(harness.stage_point(0.6, 0.3));
            harness.frame();
            harness.key(ui::UiKey::escape);
            harness.frame();
            harness.key(ui::UiKey::escape);
            harness.frame();
            harness.key(ui::UiKey::space);
            harness.frame();
            harness.key(ui::UiKey::f10);
            harness.frame();
            harness.key(ui::UiKey::s);
            harness.frame();
            harness.key(ui::UiKey::escape);
            harness.frame();
        }
        harness.expect_usable("after the Measure tab variants");
    }

    RIGIDBODIES_TEST("seeded interface monkey keeps the learner interface usable")
    {
        const auto steps = environment_count("RIGIDBODIES_MONKEY_STEPS", 700);
        std::uint64_t first_seed = 0x5EEDu;
        const auto* chosen_seed = std::getenv("RIGIDBODIES_MONKEY_SEED");
        if (chosen_seed)
            first_seed = std::strtoull(chosen_seed, nullptr, 0);
        const auto seeds = environment_count("RIGIDBODIES_MONKEY_SEEDS", chosen_seed ? 1 : 2);
        for (std::size_t index = 0; index < seeds; ++index)
            run_monkey(first_seed + index, steps);
    }

    RIGIDBODIES_TEST("seeded interface monkey with Reduce Motion, the light theme and large text from startup")
    {
        core::ApplicationConfig config;
        config.interface_settings.reduce_motion = true;
        config.interface_settings.theme = "workbench_light";
        config.interface_settings.interface_scale = 1.5;
        config.startup_scenario = "welded_assembly";
        run_monkey(0xC0FFEEu, environment_count("RIGIDBODIES_MONKEY_STEPS", 700) * 5 / 8, config);
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
