#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>
#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/relativity_document.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/panels.hpp>

#include "relativity_fixture.hpp"
#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>

namespace
{
    using namespace rigidbodies;

    constexpr double lab_seconds_per_world_second = physics::relativity_lab_seconds_per_world_second;

    ui::UiCommand speed(double value, std::string detail = {}, ui::UiEditPhase phase = ui::UiEditPhase::commit)
    {
        ui::UiCommand command;
        command.kind = ui::UiCommandKind::set_relativity_speed;
        command.value = value;
        command.detail = std::move(detail);
        command.phase = phase;
        return command;
    }

    void send(app::SimulationSession& session, ui::UiCommandKind kind, std::string id = {}, bool flag = false)
    {
        ui::UiCommand command;
        command.kind = kind;
        command.id = std::move(id);
        command.flag = flag;
        session.apply(command);
    }

    // Loads an experiment, confirming the departure if the session asks to protect unsaved edits.
    void open(app::SimulationSession& session, std::string_view id)
    {
        RIGIDBODIES_EXPECT(session.load_scenario(id), "the experiment exists");
        if (const auto confirmation = session.build_model().confirmation)
            session.apply(confirmation->confirm);
        RIGIDBODIES_EXPECT(session.scenario_id() == id, "the experiment opens");
    }

    void open_chasing_light(app::SimulationSession& session)
    {
        session.configure({});
        open(session, "chasing_light");
    }

    double speed_fraction(const app::SimulationSession& session)
    {
        return session.relativity_probe()->setup().speed_fraction;
    }

    const physics::RelativityRace& race(const app::SimulationSession& session)
    {
        return session.relativity_probe()->race();
    }

    void play(app::SimulationSession& session, int frames, double frame_s = 1.0 / 60.0)
    {
        if (session.stepper().is_paused())
            send(session, ui::UiCommandKind::toggle_pause);
        for (int frame = 0; frame < frames; ++frame)
            session.advance(frame_s);
    }

    std::size_t history_size(const app::SimulationSession& session)
    {
        return session.build_model().undo_history.size();
    }

    bool notified(const app::SimulationSession& session, std::string_view source)
    {
        const auto model = session.build_model();
        return std::any_of(model.notifications.begin(), model.notifications.end(), [&](const ui::Notification& item)
            {
                return item.source == source;
            });
    }

    // Lab time is the world clock times a billionth, step for step.
    void expect_lab_time_follows_world(const app::SimulationSession& session, std::string_view when)
    {
        const auto expected = session.world().statistics().elapsed_time_s * lab_seconds_per_world_second;
        const auto lab = race(session).lab_time_s;
        RIGIDBODIES_EXPECT(std::abs(lab - expected) <= 1.0e-12 * std::max(expected, 1.0e-30), "lab time is world time times 1e-9 " + std::string(when));
    }

    // The probe exists exactly when its setup does, and its speed is always the starting speed.
    void expect_consistent(const app::SimulationSession& session, std::string_view when)
    {
        const auto model = session.build_model();
        if (!session.relativity_active())
        {
            RIGIDBODIES_EXPECT(!session.relativity_setup() && !model.relativity, "a Newtonian experiment has no probe, setup or model " + std::string(when));
            return;
        }
        RIGIDBODIES_EXPECT(session.relativity_setup().has_value() && model.relativity.has_value(), "the probe has a setup and a model " + std::string(when));
        RIGIDBODIES_EXPECT(*session.relativity_setup() == session.relativity_probe()->setup(), "the live speed is the starting speed " + std::string(when));
        RIGIDBODIES_EXPECT(model.relativity->speed_fraction == speed_fraction(session), "the model reads the live speed " + std::string(when));
        RIGIDBODIES_EXPECT(speed_fraction(session) >= 0.0 && speed_fraction(session) <= physics::maximum_speed_fraction, "the speed stays below c " + std::string(when));
        RIGIDBODIES_EXPECT(model.relativity->original_speed_fraction == 0.0, "the authored speed is rest " + std::string(when));
        RIGIDBODIES_EXPECT(race(session).proper_time_s <= race(session).lab_time_s, "the probe clock never runs ahead " + std::string(when));
        expect_lab_time_follows_world(session, when);
    }

    // Where between the last two fixed steps the session draws: 1 while paused or stopped.
    double frame_alpha(app::SimulationSession& session)
    {
        return session.stepper().is_paused() || session.stepper().time_scale() == 0.0 ? 1.0 : session.stepper().interpolation_fraction();
    }

    // Renders a frame and checks the stage the renderer was handed: the probe's sample for that
    // frame, its factors and the session's playback speed and units, read exactly as the model reads
    // them.
    void expect_rendered_stage(app::SimulationSession& session, std::string_view when)
    {
        render::DrawList list;
        session.render(list);
        const auto& stage = session.scene_renderer().relativity_stage();
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(stage.has_value() && model.relativity.has_value(), "the renderer holds a stage " + std::string(when));
        if (!stage || !model.relativity)
            return;
        const auto& probe = *session.relativity_probe();
        const auto sample = probe.sample(frame_alpha(session));
        const auto& value = *model.relativity;
        RIGIDBODIES_EXPECT(stage->lab_time_s == sample.lab_time_s && stage->proper_time_s == sample.proper_time_s && stage->clock_lag_s == sample.clock_lag_s, "the stage's clocks are the frame's sample " + std::string(when));
        RIGIDBODIES_EXPECT(stage->probe_position_m == sample.probe_position_m && stage->light_lead_m == sample.light_lead_m && stage->light_finished == sample.light_finished, "the stage's race is the frame's sample " + std::string(when));
        RIGIDBODIES_EXPECT(stage->last_lap_margin_s.has_value() == sample.last_lap.has_value() && (!sample.last_lap || (*stage->last_lap_margin_s == sample.last_lap->margin_s && stage->last_lap_speed_changed == sample.last_lap->speed_changed)), "the stage's last lap is the frame's sample " + std::string(when));
        RIGIDBODIES_EXPECT(stage->lab_time_s == value.lab_time_s && stage->proper_time_s == value.proper_time_s && stage->clock_lag_s == value.clock_lag_s && stage->light_lead_m == value.light_lead_m && stage->light_finished == value.light_finished && stage->last_lap_margin_s == value.last_lap_margin_s, "the model reads the sample the stage draws " + std::string(when));
        RIGIDBODIES_EXPECT(stage->speed_fraction == value.speed_fraction && stage->one_minus_speed_fraction == value.one_minus_speed_fraction && stage->lorentz_factor_minus_one == value.lorentz_factor_minus_one && stage->inverse_lorentz_factor == value.inverse_lorentz_factor && stage->clock_lag_rate == value.clock_lag_rate, "the stage's factors are the model's " + std::string(when));
        RIGIDBODIES_EXPECT(stage->speed_m_s == value.speed_m_s && stage->below_light_m_s == value.below_light_m_s && stage->kinetic_energy_j == value.kinetic_energy_j && stage->momentum_kg_m_s == value.momentum_kg_m_s && stage->rest_mass_kg == value.rest_mass_kg, "the stage's readings are the model's " + std::string(when));
        RIGIDBODIES_EXPECT(stage->track_length_m == probe.track_length_m() && stage->mark_spacing_m == physics::light_nanosecond_m && stage->clock_period_s == 1.0e-9 && stage->tier == render::RelativityStageTier::full, "the stage has the track, marks, clocks and tier " + std::string(when));
        RIGIDBODIES_EXPECT(stage->lab_seconds_per_screen_second == session.stepper().time_scale() * lab_seconds_per_world_second && stage->units == session.scene_settings().display_units, "the slow-motion plate follows the playback speed and the units " + std::string(when));
    }

    // The reference stage area the document backend publishes for a 1600 × 900 window at 150 %.
    const render::ScreenRect reference_focus { 36.0, 102.0, 1108.0, 732.0 };

    bool has_handle(const ui::UiModel& model, std::string_view id)
    {
        return std::any_of(model.handles.begin(), model.handles.end(), [&](const ui::StageHandle& handle)
            {
                return handle.id == id;
            });
    }

    // Whether a frame draws the gravity dial, whose only text is its "g".
    bool drawn_gravity_dial(app::SimulationSession& session)
    {
        render::DrawList list;
        session.render(list);
        for (const auto& draw : list.commands())
            if (draw.kind == render::DrawCommandKind::text && std::string_view(list.text_buffer()).substr(draw.text_offset, draw.text_length) == "g")
                return true;
        return false;
    }

    // Whether a frame keeps the stage plates clear of the gravity dial: a square about the dial's
    // centre, at the scale an unscaled session draws it.
    bool reserved_gravity_dial(app::SimulationSession& session)
    {
        render::DrawList list;
        session.render(list);
        const auto dial = app::gravity_compass_centre(session.camera(), 1.0);
        const auto half = app::overlay::compass_radius + 4.0;
        const auto& areas = session.scene_renderer().overlay_areas();
        return std::any_of(areas.begin(), areas.end(), [&](const std::pair<math::Vec2, math::Vec2>& area)
            {
                return math::length(area.first - (dial - math::Vec2 { half, half })) < 1.0e-9 && math::length(area.second - (dial + math::Vec2 { half, half })) < 1.0e-9;
            });
    }

    // The command a key sends, as Application::handle_action and the learner harness send it.
    ui::UiCommand key_command(ui::UiKey key, bool shift = false, bool control = false, bool alt = false)
    {
        return app::command_for_action(app::action_for_key(key, { shift, control, alt }));
    }

    // Whether leaving for another experiment asks first. Nothing changes either way: the question
    // is cancelled, or the departure is undone.
    bool asks_before_leaving(app::SimulationSession& session)
    {
        ui::UiCommand leave;
        leave.kind = ui::UiCommandKind::load_scenario;
        leave.id = "free_fall";
        session.apply(leave);
        if (const auto confirmation = session.build_model().confirmation)
        {
            session.apply(confirmation->cancel);
            RIGIDBODIES_EXPECT(session.scenario_id() == "chasing_light" && !session.build_model().confirmation, "cancelling keeps the experiment");
            return true;
        }
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall", "an unchanged setup is left at once");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(session.scenario_id() == "chasing_light" && session.relativity_active(), "undo comes back");
        return false;
    }

    const ui::SetupChange* find_change(const std::vector<ui::SetupChange>& changes, std::string_view key)
    {
        const auto found = std::find_if(changes.begin(), changes.end(), [&](const ui::SetupChange& change)
            {
                return change.key == key;
            });
        return found == changes.end() ? nullptr : &*found;
    }

    const ui::KeyReference* find_key(const ui::UiModel& model, std::string_view description)
    {
        const auto found = std::find_if(model.keyboard_reference.begin(), model.keyboard_reference.end(), [&](const ui::KeyReference& entry)
            {
                return entry.description == description;
            });
        return found == model.keyboard_reference.end() ? nullptr : &*found;
    }

    const ui::Notification* find_notice(const ui::UiModel& model, std::string_view source)
    {
        const auto found = std::find_if(model.notifications.begin(), model.notifications.end(), [&](const ui::Notification& item)
            {
                return item.source == source;
            });
        return found == model.notifications.end() ? nullptr : &*found;
    }

    void expect_no_stage(app::SimulationSession& session, std::string_view when)
    {
        render::DrawList list;
        session.render(list);
        RIGIDBODIES_EXPECT(!session.scene_renderer().relativity_stage() && !session.scene_renderer().relativity_stage_layout(), "the renderer holds no stage " + std::string(when));
    }

    class MeasurementDevice final : public render::RenderDevice
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
            return 15.0f * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    std::vector<ui::PanelRow> guide_rows(const ui::UiModel& model)
    {
        MeasurementDevice device;
        render::Theme theme;
        render::DrawList list;
        std::vector<ui::Hotspot> hotspots;
        std::vector<ui::PanelRow> rows;
        ui::PanelBuilder builder { theme, device, 1.0f, { {}, { 1000, 100000 } }, list, hotspots };
        builder.record_rows(rows);
        ui::GuidePanel panel;
        panel.build(model, builder);
        return rows;
    }

    RIGIDBODIES_TEST("Chasing light opens Ready with both clocks at zero")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(reference_focus);
        session.configure({});
        RIGIDBODIES_EXPECT(!session.relativity_active() && !session.build_model().relativity, "a Newtonian experiment has no probe");
        session.stepper().set_paused(true);
        RIGIDBODIES_EXPECT(has_handle(session.build_model(), "gravity") && drawn_gravity_dial(session), "a paused Newtonian experiment shows and publishes its gravity dial");
        RIGIDBODIES_EXPECT(reserved_gravity_dial(session), "a paused Newtonian experiment keeps its plates clear of the dial");
        open(session, "chasing_light");
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(session.relativity_active() && model.relativity, "the document's required feature installs the probe");
        RIGIDBODIES_EXPECT(model.run_state == ui::RunState::ready && session.stepper().is_paused(), "it opens Ready and paused");
        RIGIDBODIES_EXPECT(race(session).lab_time_s == 0.0 && race(session).proper_time_s == 0.0 && race(session).clock_lag_s == 0.0, "both clocks read zero");
        RIGIDBODIES_EXPECT(model.relativity->lab_time_s == 0.0 && model.relativity->proper_time_s == 0.0, "the model's clocks read zero");
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && session.relativity_probe()->setup().rest_mass_kg == 1.0, "the probe is the authored 1 kg at rest");
        RIGIDBODIES_EXPECT(model.relativity->lorentz_factor == 1.0 && model.relativity->kinetic_energy_j == 0.0 && model.relativity->momentum_kg_m_s == 0.0, "at rest γ is 1 and nothing moves");
        RIGIDBODIES_EXPECT(model.relativity->rest_energy_j == physics::speed_of_light_m_s * physics::speed_of_light_m_s, "the rest energy is mc²");
        RIGIDBODIES_EXPECT(model.scenario_content && model.scenario_content->special_relativity && model.scenario_content->collection == "Special relativity", "the content knows its collection and feature");
        const auto card = std::find_if(model.catalogue.begin(), model.catalogue.end(), [](const ui::ExperimentCard& value)
            {
                return value.id == "chasing_light";
            });
        RIGIDBODIES_EXPECT(card != model.catalogue.end() && card->special_relativity && card->collection == "Special relativity" && card->title == "Chasing light", "the Library card is marked");
        RIGIDBODIES_EXPECT(std::none_of(model.catalogue.begin(), model.catalogue.end(), [](const ui::ExperimentCard& value)
                               {
                                   return value.id != "chasing_light" && value.special_relativity;
                               }),
            "no Newtonian card is marked");
        RIGIDBODIES_EXPECT(!has_handle(model, "gravity") && !drawn_gravity_dial(session), "the probe's track has no gravity dial to show or publish");
        RIGIDBODIES_EXPECT(session.stepper().is_paused() && !reserved_gravity_dial(session), "no plate is pushed away from a dial that is not drawn");
        // A press and drag where the dial would be is not a gravity edit.
        const auto entries = history_size(session);
        const auto dial = app::gravity_compass_centre(session.camera(), 1.0);
        for (const auto& [kind, point] : { std::pair { ui::UiEventKind::pointer_down, dial }, std::pair { ui::UiEventKind::pointer_move, dial + math::Vec2 { 0.0, 30.0 } }, std::pair { ui::UiEventKind::pointer_up, dial + math::Vec2 { 0.0, 30.0 } } })
        {
            ui::UiEvent event;
            event.kind = kind;
            event.pointer_px = point;
            event.button = ui::PointerButton::primary;
            session.set_pointer_position(point);
            session.handle_scene_event(event, false);
        }
        RIGIDBODIES_EXPECT(history_size(session) == entries && math::length(session.world().settings().gravity_m_s2) == 0.0, "dragging there changes nothing");
        expect_consistent(session, "at opening");
    }

    RIGIDBODIES_TEST("lab time is the world clock times a billionth at every playback speed")
    {
        for (const auto scale : { 0.25, 1.0, 4.0 })
        {
            app::SimulationSession session;
            open_chasing_light(session);
            ui::UiCommand playback;
            playback.kind = ui::UiCommandKind::set_time_scale;
            playback.value = scale;
            session.apply(playback);
            play(session, 240);
            RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s > 0.0, "the world clock runs");
            expect_lab_time_follows_world(session, "at " + std::to_string(scale) + "x");
            RIGIDBODIES_EXPECT(race(session).proper_time_s == race(session).lab_time_s && race(session).clock_lag_s == 0.0, "a probe at rest keeps time with the lab clock");
            RIGIDBODIES_EXPECT(race(session).lap_distance_m == 0.0 && race(session).completed_laps == 0, "a probe at rest stays on the start line");
        }
    }

    RIGIDBODIES_TEST("proper time follows the speed in force")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.6));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.6 && session.relativity_setup()->speed_fraction == 0.6, "0.6 c is set at Ready, live and setup together");
        play(session, 600);
        const auto lab = race(session).lab_time_s;
        RIGIDBODIES_EXPECT_NEAR(lab, 1.0e-8, 1.0e-11, "ten world seconds are ten lab nanoseconds");
        RIGIDBODIES_EXPECT_NEAR(race(session).proper_time_s, 0.8 * lab, 1.0e-12 * lab, "the probe clock reads t/γ = 0.8 t");
        RIGIDBODIES_EXPECT_NEAR(race(session).clock_lag_s, 0.2 * lab, 1.0e-12 * lab, "the lab clock leads by 0.2 t");
        const auto proper = race(session).proper_time_s;
        session.apply(speed(0.8));
        play(session, 300);
        const auto added = race(session).lab_time_s - lab;
        RIGIDBODIES_EXPECT(added > 0.0, "the clocks keep running after the change");
        RIGIDBODIES_EXPECT_NEAR(race(session).proper_time_s - proper, 0.6 * added, 1.0e-11 * added, "after the change the probe clock runs at 1/γ = 0.6");
        expect_consistent(session, "after two speeds");
    }

    RIGIDBODIES_TEST("speed commands outside zero to the maximum are refused without an undo entry")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.5));
        session.apply(speed(0.9));
        send(session, ui::UiCommandKind::undo);
        const auto before = session.build_model();
        RIGIDBODIES_EXPECT(before.can_redo && speed_fraction(session) == 0.5, "a redo entry exists");
        // Slower than the slowest moving speed is refused too: such a speed's text and energy would
        // lose their digits.
        const auto refused = { speed(1.0), speed(std::numeric_limits<double>::quiet_NaN()), speed(-0.1), speed(physics::maximum_speed_fraction + 1.0e-9), speed(std::numeric_limits<double>::infinity()), speed(0.7, "bogus"), speed(std::numeric_limits<double>::quiet_NaN(), "nudge"), speed(1.0e-13), speed(1.0e-300), speed(std::numeric_limits<double>::denorm_min()) };
        for (const auto& command : refused)
        {
            session.apply(command);
            const auto after = session.build_model();
            RIGIDBODIES_EXPECT(speed_fraction(session) == 0.5 && session.relativity_setup()->speed_fraction == 0.5, "the speed is unchanged");
            RIGIDBODIES_EXPECT(after.undo_history.size() == before.undo_history.size() && after.undo_label == before.undo_label, "no undo entry");
            RIGIDBODIES_EXPECT(after.can_redo && after.redo_label == before.redo_label, "the redo branch survives");
        }
        session.apply(speed(physics::maximum_speed_fraction));
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::maximum_speed_fraction, "the maximum itself is accepted");
        session.apply(speed(0.0));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0, "rest is accepted");
        session.apply(speed(physics::minimum_moving_speed_fraction));
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::minimum_moving_speed_fraction, "the slowest moving speed is accepted");
        session.apply(speed(0.5));
        session.apply(speed(-0.0));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && !std::signbit(speed_fraction(session)) && !std::signbit(session.relativity_setup()->speed_fraction), "a negative zero is stored as rest");
    }

    RIGIDBODIES_TEST("a speed edit is one parameter entry that never rewinds or pauses")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        play(session, 60);
        const auto lab = race(session).lab_time_s;
        const auto entries = history_size(session);
        session.apply(speed(0.9));
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.undo_history.size() == entries + 1 && model.undo_label == "Change probe speed", "one entry, labelled without its value");
        RIGIDBODIES_EXPECT(model.undo_history.front().category == ui::EditCategory::parameter, "the entry is a parameter edit");
        RIGIDBODIES_EXPECT(race(session).lab_time_s == lab && model.run_state == ui::RunState::running && model.held_reason.empty(), "the clocks are neither rewound nor held");
        RIGIDBODIES_EXPECT(session.relativity_setup()->speed_fraction == 0.9, "the starting speed changes with the live speed");
        play(session, 60);
        RIGIDBODIES_EXPECT(race(session).lab_time_s > lab, "the clocks keep running");
        expect_consistent(session, "after a running edit");
    }

    RIGIDBODIES_TEST("dragging the speed keeps the clocks running and commits one entry")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        play(session, 30);
        const auto entries = history_size(session);
        auto proper = race(session).proper_time_s;
        for (const auto value : { 0.3, 0.6, 0.9 })
        {
            session.apply(speed(value, {}, ui::UiEditPhase::preview));
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(model.held_reason.empty() && model.run_state == ui::RunState::running, "a speed preview never holds the clocks");
            RIGIDBODIES_EXPECT(speed_fraction(session) == value && session.relativity_setup()->speed_fraction == value, "the preview is live, in the setup too");
            play(session, 20);
            RIGIDBODIES_EXPECT(race(session).proper_time_s > proper, "the probe clock advances during the drag");
            proper = race(session).proper_time_s;
        }
        session.apply(speed(0.9, {}, ui::UiEditPhase::commit));
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.undo_history.size() == entries + 1 && model.undo_label == "Change probe speed", "the drag commits one entry");
        RIGIDBODIES_EXPECT(model.undo_history.front().category == ui::EditCategory::parameter, "the drag is a parameter edit");
        RIGIDBODIES_EXPECT(std::any_of(model.graph_markers.begin(), model.graph_markers.end(), [](const ui::InterventionMarker& marker)
                               {
                                   return marker.label == "Speed 0\xC2\xA0"
                                                          "c \xE2\x86\x92 0.9\xC2\xA0"
                                                          "c";
                               }),
            "the run marker names both speeds");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0, "one undo returns to the speed before the drag");
        expect_consistent(session, "after undoing a drag");
    }

    RIGIDBODIES_TEST("a typed speed a rounding error from a rung is that rung")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        // 99.999 % parses one unit in the last place below 0.99999, and 99.99999 % one below the
        // maximum: each reads exactly as its rung, so it is that rung, its chip and its limit.
        session.apply(speed(99.999 / 100.0));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.99999 && session.build_model().relativity->speed_fraction == 0.99999, "99.999 % is 0.99999 c");
        RIGIDBODIES_EXPECT(ui::relativity_preset_id(speed_fraction(session)) == "0.99999", "its chip is selected");
        session.apply(speed(99.99999 / 100.0));
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::maximum_speed_fraction, "99.99999 % is the maximum");
        const auto entries = history_size(session);
        session.apply(key_command(ui::UiKey::arrow_up));
        session.apply(key_command(ui::UiKey::arrow_up, true));
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::maximum_speed_fraction && history_size(session) == entries && notified(session, "relativity-top"), "Up and Shift+Up at the maximum change nothing and say why");
        // A file may hold a speed a rounding error from a rung; its chip is that rung's.
        RIGIDBODIES_EXPECT(ui::relativity_preset_id(std::nextafter(0.99999, 0.0)) == "0.99999" && ui::relativity_preset_id(std::nextafter(0.9, 1.0)) == "0.9", "a rung reached through rounding selects its chip");
        // A typed speed merely close to a rung keeps the learner's value.
        session.apply(speed(150000000.0 / physics::speed_of_light_m_s));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 150000000.0 / physics::speed_of_light_m_s && ui::relativity_preset_id(speed_fraction(session)).empty(), "150 000 km/s keeps its own value");
    }

    RIGIDBODIES_TEST("the run's graph marks each speed change where the clocks changed rate")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        play(session, 30);
        const auto time = [&]
        {
            return session.world().statistics().elapsed_time_s;
        };
        const auto markers = [&]
        {
            return session.build_model().graph_markers;
        };
        const auto marked = [&](double time_s, std::string_view label)
        {
            const auto list = markers();
            return std::any_of(list.begin(), list.end(), [&](const ui::InterventionMarker& marker)
                {
                    return marker.time_s == time_s && marker.label == label;
                });
        };
        const std::string zero = "Speed 0\xC2\xA0"
                                 "c \xE2\x86\x92 0.9\xC2\xA0"
                                 "c";
        const std::string back = "Speed 0.9\xC2\xA0"
                                 "c \xE2\x86\x92 0\xC2\xA0"
                                 "c";
        const std::string faster = "Speed 0.9\xC2\xA0"
                                   "c \xE2\x86\x92 0.99\xC2\xA0"
                                   "c";
        const std::string slower = "Speed 0.99\xC2\xA0"
                                   "c \xE2\x86\x92 0.9\xC2\xA0"
                                   "c";
        // A drag changes the clocks' rates as soon as it moves, so its marker goes there, not at
        // the release.
        const auto drag_start = time();
        session.apply(speed(0.5, {}, ui::UiEditPhase::preview));
        play(session, 20);
        session.apply(speed(0.9, {}, ui::UiEditPhase::preview));
        play(session, 20);
        session.apply(speed(0.9, {}, ui::UiEditPhase::commit));
        RIGIDBODIES_EXPECT(markers().size() == 1 && marked(drag_start, zero), "one marker, where the drag first changed the speed");
        // Undo and redo change the speed with the clocks running, so each is marked.
        play(session, 20);
        const auto undone = time();
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(markers().size() == 2 && marked(undone, back), "undo marks the speed it returned to");
        play(session, 20);
        const auto redone = time();
        send(session, ui::UiCommandKind::redo);
        RIGIDBODIES_EXPECT(markers().size() == 3 && marked(redone, zero), "redo marks the speed it restored");
        // A cancelled drag ran the clocks at the dragged speed until it was cancelled.
        play(session, 20);
        const auto cancelled_start = time();
        session.apply(speed(0.99, {}, ui::UiEditPhase::preview));
        play(session, 20);
        const auto cancelled = time();
        session.apply(speed(0.99, {}, ui::UiEditPhase::cancel));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && markers().size() == 5 && marked(cancelled_start, faster) && marked(cancelled, slower), "a cancelled drag is marked where it began and where it ended");
        // While paused no time passes under the drag, so nothing is marked.
        send(session, ui::UiCommandKind::toggle_pause);
        session.apply(speed(0.99, {}, ui::UiEditPhase::preview));
        session.apply(speed(0.99, {}, ui::UiEditPhase::cancel));
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && markers().size() == 5, "a drag cancelled while paused leaves no marker");
        // A per-item revert names both speeds, as an edit does.
        play(session, 20);
        const auto reverted = time();
        send(session, ui::UiCommandKind::revert_change, "relativity:speed");
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && markers().size() == 6 && marked(reverted, back), "a revert is marked with both speeds");
        expect_consistent(session, "after the marked changes");
    }

    RIGIDBODIES_TEST("cancelling a speed drag restores only the speed")
    {
        for (const auto focus_lost : { false, true })
        {
            app::SimulationSession session;
            open_chasing_light(session);
            session.apply(speed(0.5));
            play(session, 30);
            const auto entries = history_size(session);
            session.apply(speed(0.9, {}, ui::UiEditPhase::preview));
            play(session, 30);
            const auto lab = race(session).lab_time_s;
            const auto proper = race(session).proper_time_s;
            if (focus_lost)
            {
                ui::UiEvent event;
                event.kind = ui::UiEventKind::focus_lost;
                session.handle_scene_event(event, false);
            }
            else
                session.apply(speed(0.9, {}, ui::UiEditPhase::cancel));
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(speed_fraction(session) == 0.5 && session.relativity_setup()->speed_fraction == 0.5, "the speed returns to the drag's start");
            RIGIDBODIES_EXPECT(race(session).lab_time_s == lab && race(session).proper_time_s == proper, "the clocks are not rewound");
            RIGIDBODIES_EXPECT(model.undo_history.size() == entries && model.held_reason.empty() && model.run_state == ui::RunState::running, "no entry, no hold, still running");
            expect_consistent(session, focus_lost ? "after focus loss" : "after Escape");
        }
    }

    RIGIDBODIES_TEST("undoing a speed edit keeps time and play state")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        play(session, 30);
        session.apply(speed(0.9));
        play(session, 30);
        const auto lab = race(session).lab_time_s;
        const auto proper = race(session).proper_time_s;
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && session.relativity_setup()->speed_fraction == 0.0, "undo restores the earlier speed, in the setup too");
        RIGIDBODIES_EXPECT(race(session).lab_time_s == lab && race(session).proper_time_s == proper, "undo never rewinds the clocks");
        RIGIDBODIES_EXPECT(!session.stepper().is_paused() && session.build_model().run_state == ui::RunState::running, "undo keeps the experiment running");
        send(session, ui::UiCommandKind::redo);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && race(session).lab_time_s == lab, "redo restores the speed and keeps the time");
        expect_consistent(session, "after redo");
    }

    RIGIDBODIES_TEST("Up nudges and Shift+Up climbs the ladder")
    {
        // The keys reach the session through the one command mapping the application and the
        // learner harness share.
        const auto up = key_command(ui::UiKey::arrow_up), down = key_command(ui::UiKey::arrow_down);
        const auto shift_up = key_command(ui::UiKey::arrow_up, true), shift_down = key_command(ui::UiKey::arrow_down, true);
        RIGIDBODIES_EXPECT(up.kind == ui::UiCommandKind::set_relativity_speed && up.detail == "nudge" && up.value == 1.0 && down.kind == up.kind && down.detail == "nudge" && down.value == -1.0, "Up and Down nudge the speed one step either way");
        RIGIDBODIES_EXPECT(shift_up.kind == up.kind && shift_up.detail == "preset" && shift_up.value == 1.0 && shift_down.kind == up.kind && shift_down.detail == "preset" && shift_down.value == -1.0, "Shift+Up and Shift+Down move along the ladder");
        app::SimulationSession session;
        open_chasing_light(session);
        const auto entries = history_size(session);
        session.apply(down);
        session.apply(shift_down);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && history_size(session) == entries && !notified(session, "relativity-top"), "Down at rest does nothing");
        session.apply(up);
        const auto& number = ui::find_control_spec("world.relativity.speed")->number;
        RIGIDBODIES_EXPECT(speed_fraction(session) == ui::keyboard_step(number, 0.0, 1, ui::StepSize::normal) && speed_fraction(session) == 0.114, "one rapidity step from rest is 0.114 c, as its text reads");
        RIGIDBODIES_EXPECT(history_size(session) == entries + 1, "a nudge is an entry");
        session.apply(down);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0, "Down returns to rest");
        for (std::size_t rung = 1; rung < physics::speed_fraction_ladder.size(); ++rung)
        {
            session.apply(shift_up);
            RIGIDBODIES_EXPECT(speed_fraction(session) == physics::speed_fraction_ladder[rung], "Shift+Up climbs to the next rung");
        }
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::maximum_speed_fraction && !notified(session, "relativity-top"), "the ladder ends at the maximum");
        const auto at_top = history_size(session);
        session.apply(shift_up);
        session.apply(up);
        RIGIDBODIES_EXPECT(speed_fraction(session) == physics::maximum_speed_fraction && history_size(session) == at_top, "nothing climbs past the maximum, and no entry is made");
        const auto model = session.build_model();
        const auto notice = std::find_if(model.notifications.begin(), model.notifications.end(), [](const ui::Notification& item)
            {
                return item.source == "relativity-top";
            });
        RIGIDBODIES_EXPECT(notice != model.notifications.end() && notice->severity == ui::Severity::info, "the top gives an information notice");
        RIGIDBODIES_EXPECT(notice != model.notifications.end() && notice->text == "0.9999999\xC2\xA0"
                                                                                  "c is the fastest speed here. A massive object can get ever closer to c but never reach it.",
            "the notice says c is out of reach");
        session.apply(shift_down);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.999999, "Shift+Down steps back down the ladder");
        expect_consistent(session, "after the ladder");

        app::SimulationSession newtonian;
        newtonian.configure({});
        const auto before = newtonian.build_model();
        for (const auto& command : { up, shift_up, down, shift_down, speed(0.5) })
            newtonian.apply(command);
        const auto after = newtonian.build_model();
        RIGIDBODIES_EXPECT(!newtonian.relativity_active() && after.undo_history.size() == before.undo_history.size() && after.notifications.size() == before.notifications.size(), "a Newtonian experiment ignores the speed command");
        // Nor does the speed command touch a Newtonian drag in progress.
        physics::BodyId ball;
        for (const auto id : newtonian.world().body_ids())
            if (newtonian.world().find_body(id)->type() == physics::BodyType::dynamic_body)
                ball = id;
        const auto mass_before = newtonian.world().find_body(ball)->mass_properties().mass_kg;
        newtonian.set_selection(ball);
        ui::UiCommand mass;
        mass.kind = ui::UiCommandKind::set_selected_mass;
        mass.body = ball;
        mass.value = 2.0 * mass_before;
        mass.phase = ui::UiEditPhase::preview;
        newtonian.apply(mass);
        newtonian.apply(up);
        RIGIDBODIES_EXPECT(newtonian.build_model().held_reason == "adjusting", "the mass drag is still in progress");
        mass.phase = ui::UiEditPhase::cancel;
        newtonian.apply(mass);
        RIGIDBODIES_EXPECT(newtonian.world().find_body(ball)->mass_properties().mass_kg == mass_before && history_size(newtonian) == after.undo_history.size(), "cancelling the drag still restores its start");
    }

    RIGIDBODIES_TEST("Back to start keeps the chosen speed and zeroes the race; Replay plays")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.9));
        play(session, 1200);
        RIGIDBODIES_EXPECT(race(session).completed_laps >= 1 && race(session).last_lap.has_value(), "the probe finished a lap");
        RIGIDBODIES_EXPECT_NEAR(race(session).last_lap->margin_s, physics::relativity_track_length_m / physics::speed_of_light_m_s * (1.0 - 0.9) / 0.9, 1.0e-6 * 1.0e-9, "the light won by (L/c)(1 − β)/β");
        send(session, ui::UiCommandKind::reset_scenario);
        const auto& reset = race(session);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && session.relativity_setup()->speed_fraction == 0.9, "Back to start keeps the chosen speed");
        RIGIDBODIES_EXPECT(reset.lab_time_s == 0.0 && reset.proper_time_s == 0.0 && reset.clock_lag_s == 0.0 && reset.completed_laps == 0 && !reset.last_lap && reset.lap_distance_m == 0.0, "the clocks, lap and pulse are zero");
        RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.build_model().run_state == ui::RunState::ready, "Back to start pauses at Ready");
        expect_consistent(session, "after Back to start");
        play(session, 60);
        send(session, ui::UiCommandKind::reset_scenario, {}, true);
        RIGIDBODIES_EXPECT(race(session).lab_time_s == 0.0 && speed_fraction(session) == 0.9 && !session.stepper().is_paused(), "Replay restarts the clocks and plays");
        expect_consistent(session, "after Replay");
    }

    RIGIDBODIES_TEST("Restore original and per-item revert return to the authored speed")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        for (const auto* id : { "relativity:speed", "world.relativity.speed" })
        {
            session.apply(speed(0.9));
            play(session, 60);
            const auto lab = race(session).lab_time_s;
            send(session, ui::UiCommandKind::revert_change, id);
            RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && session.relativity_setup()->speed_fraction == 0.0, "revert returns to the authored speed");
            RIGIDBODIES_EXPECT(race(session).lab_time_s == lab && !session.stepper().is_paused(), "revert keeps the clocks running");
            RIGIDBODIES_EXPECT(session.build_model().undo_label == "Revert change" && session.build_model().undo_history.front().category == ui::EditCategory::parameter, "revert is one parameter entry");
            send(session, ui::UiCommandKind::undo);
            RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && race(session).lab_time_s == lab, "undoing the revert returns the speed only");
            const auto entries = history_size(session);
            session.apply(speed(0.0));
            send(session, ui::UiCommandKind::revert_change, id);
            RIGIDBODIES_EXPECT(history_size(session) == entries + 1, "reverting an unchanged speed adds nothing");
        }
        session.apply(speed(0.99));
        play(session, 60);
        send(session, ui::UiCommandKind::restore_original);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.0 && session.relativity_setup()->speed_fraction == 0.0, "Restore original returns to rest");
        RIGIDBODIES_EXPECT(race(session).lab_time_s == 0.0 && race(session).proper_time_s == 0.0 && session.stepper().is_paused(), "Restore original zeroes the clocks and pauses");
        expect_consistent(session, "after Restore original");
    }

    RIGIDBODIES_TEST("Keep as starting state restarts the clocks and keeps the speed")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.5));
        play(session, 90);
        RIGIDBODIES_EXPECT(race(session).lab_time_s > 0.0, "the clocks ran");
        send(session, ui::UiCommandKind::keep_state_as_setup);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.5 && session.relativity_setup()->speed_fraction == 0.5, "the speed is kept as the starting speed");
        RIGIDBODIES_EXPECT(race(session).lab_time_s == 0.0 && race(session).proper_time_s == 0.0 && race(session).lap_distance_m == 0.0, "the clocks and race start again");
        RIGIDBODIES_EXPECT(session.stepper().is_paused() && session.world().statistics().elapsed_time_s == 0.0, "the experiment waits at Ready");
        expect_consistent(session, "after Keep as starting state");
    }

    RIGIDBODIES_TEST("lab settings changed during a run leave Back to start at zero on both clocks")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.6));
        play(session, 300);
        ui::UiCommand integrator;
        integrator.kind = ui::UiCommandKind::set_integrator;
        integrator.id = "runge_kutta_4";
        session.apply(integrator);
        RIGIDBODIES_EXPECT(race(session).lab_time_s > 0.0 && !session.stepper().is_paused(), "the run continues with the new method");
        expect_consistent(session, "after a lab change during the run");
        play(session, 60);
        send(session, ui::UiCommandKind::reset_scenario);
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == 0.0 && race(session).lab_time_s == 0.0 && session.build_model().run_state == ui::RunState::ready, "Back to start zeroes the world and the lab clock and waits at Ready");
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.6 && session.world().integrator().name() == "runge_kutta_4", "the speed and the lab setting are kept");
        expect_consistent(session, "after Back to start");
        play(session, 120);
        ui::UiCommand step;
        step.kind = ui::UiCommandKind::set_fixed_step;
        step.value = 1.0 / 240.0;
        session.apply(step);
        play(session, 120);
        expect_consistent(session, "after a fixed-step change during the run");
        send(session, ui::UiCommandKind::reset_scenario, {}, true);
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s == 0.0 && race(session).lab_time_s == 0.0 && !session.stepper().is_paused(), "Replay restarts both clocks from zero");
        expect_consistent(session, "after Replay");
    }

    RIGIDBODIES_TEST("undoing an experiment switch restores the probe with its clocks")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.9));
        play(session, 120);
        expect_rendered_stage(session, "before the switch");
        const auto lab = race(session).lab_time_s;
        const auto proper = race(session).proper_time_s;
        open(session, "free_fall");
        RIGIDBODIES_EXPECT(!session.relativity_active() && !session.relativity_setup() && !session.build_model().relativity, "a Newtonian experiment removes the probe");
        expect_no_stage(session, "in free fall");
        expect_consistent(session, "in free fall");
        // Throw is a Newtonian tool; choosing it makes no history entry, so undo goes straight back.
        send(session, ui::UiCommandKind::set_interaction_mode, "throw");
        RIGIDBODIES_EXPECT(session.build_model().interaction_mode == "throw", "free fall takes the Throw tool");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(session.scenario_id() == "chasing_light" && session.relativity_active(), "undo reopens Chasing light");
        RIGIDBODIES_EXPECT(session.build_model().interaction_mode == "select", "the probe returns with the selection tool, never a tool its experiment refuses");
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.9 && race(session).lab_time_s == lab && race(session).proper_time_s == proper, "the probe returns with its speed and clocks");
        expect_rendered_stage(session, "after undoing the switch");
        RIGIDBODIES_EXPECT(session.scene_renderer().relativity_stage()->lab_time_s == lab && session.scene_renderer().relativity_stage()->speed_fraction == 0.9, "the stage returns with the probe's clocks and speed");
        expect_consistent(session, "after undoing the switch");
        send(session, ui::UiCommandKind::redo);
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall" && !session.relativity_active(), "redo leaves again");
        expect_no_stage(session, "after redoing the switch");
        expect_consistent(session, "after redoing the switch");
    }

    RIGIDBODIES_TEST("opening a relativity file installs its probe and an invalid one changes nothing")
    {
        app::SimulationSession session;
        session.configure({});
        std::string text, error;
        RIGIDBODIES_EXPECT(physics::write_scenario_document(*physics::scenario_document_for_id("chasing_light"), text, error), error);
        RIGIDBODIES_EXPECT(session.open_arrangement(text, error), error);
        RIGIDBODIES_EXPECT(session.relativity_active() && speed_fraction(session) == 0.0 && race(session).lab_time_s == 0.0, "the opened file brings its probe at rest");
        expect_consistent(session, "after opening a relativity file");
        session.apply(speed(0.5));
        play(session, 30);
        const auto lab = race(session).lab_time_s;
        physics::ScenarioDocument invalid = *physics::scenario_document_for_id("chasing_light");
        invalid.root["relativity"]["speed_fraction_c"] = 1.0;
        const auto invalid_text = physics::content::write_json(invalid.root);
        RIGIDBODIES_EXPECT(!session.open_arrangement(invalid_text, error) && error.find("speed_fraction_c") != std::string::npos, "a file at c is rejected by name");
        RIGIDBODIES_EXPECT(session.relativity_active() && speed_fraction(session) == 0.5 && race(session).lab_time_s == lab, "a rejected file leaves the probe untouched");
        std::string newtonian;
        RIGIDBODIES_EXPECT(physics::write_scenario_document(*physics::scenario_document_for_id("free_fall"), newtonian, error), error);
        RIGIDBODIES_EXPECT(session.open_arrangement(newtonian, error), error);
        RIGIDBODIES_EXPECT(!session.relativity_active(), "a Newtonian file removes the probe");
        expect_consistent(session, "after opening a Newtonian file");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(session.relativity_active() && speed_fraction(session) == 0.5 && race(session).lab_time_s == lab, "undo restores the probe the open replaced");
        expect_consistent(session, "after undoing the open");
    }

    RIGIDBODIES_TEST("the stage and the model read the frame's sample and match the physics helpers")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        expect_rendered_stage(session, "at Ready");
        session.apply(speed(0.9));
        ui::UiCommand playback;
        playback.kind = ui::UiCommandKind::set_time_scale;
        playback.value = 2.0;
        session.apply(playback);
        // Frames of a seventieth of a second at twice the speed end between two fixed steps, so the
        // frame's sample is interpolated; 26 lab nanoseconds is two laps at 0.9 c.
        play(session, 900, 1.0 / 70.0);
        RIGIDBODIES_EXPECT(frame_alpha(session) > 0.0 && frame_alpha(session) < 1.0, "the frame falls between two fixed steps");
        RIGIDBODIES_EXPECT(race(session).last_lap.has_value(), "a lap has finished");
        expect_rendered_stage(session, "while running at twice the speed");
        session.scene_settings().display_units = core::DisplayUnits::centimetre_gram;
        expect_rendered_stage(session, "in centimetre-gram units");
        session.scene_settings().display_units = core::DisplayUnits::si;
        send(session, ui::UiCommandKind::toggle_pause);
        expect_rendered_stage(session, "paused");
        auto model = session.build_model();
        const auto& now = race(session);
        RIGIDBODIES_EXPECT(model.relativity->lab_time_s == now.lab_time_s, "paused, the model reads the current state");
        const auto expected = testing::relativity_model_at(0.9, now.lab_time_s);
        const auto& actual = *model.relativity;
        const auto near = [](double a, double b)
        {
            return std::abs(a - b) <= 1.0e-9 * std::max(std::abs(b), 1.0e-30);
        };
        RIGIDBODIES_EXPECT(actual.speed_fraction == expected.speed_fraction && actual.lorentz_factor == expected.lorentz_factor && actual.kinetic_energy_j == expected.kinetic_energy_j && actual.momentum_kg_m_s == expected.momentum_kg_m_s, "speed-derived values are the helpers' own");
        RIGIDBODIES_EXPECT(actual.speed_m_s == expected.speed_m_s && actual.below_light_m_s == expected.below_light_m_s && actual.newtonian_kinetic_energy_j == expected.newtonian_kinetic_energy_j, "speeds and Newton's values are the helpers' own");
        RIGIDBODIES_EXPECT(near(actual.proper_time_s, expected.proper_time_s) && near(actual.clock_lag_s, expected.clock_lag_s), "many steps give the clocks of one step");
        RIGIDBODIES_EXPECT(actual.completed_laps == expected.completed_laps && actual.light_finished == expected.light_finished, "many steps give the race of one step");
        RIGIDBODIES_EXPECT(std::abs(actual.light_lead_m - expected.light_lead_m) <= 1.0e-12 + 1.0e-9 * expected.light_lead_m, "and the same light lead");
        RIGIDBODIES_EXPECT(actual.last_lap_margin_s.has_value() == expected.last_lap_margin_s.has_value() && (!actual.last_lap_margin_s || near(*actual.last_lap_margin_s, *expected.last_lap_margin_s)), "and the same last lap");
        RIGIDBODIES_EXPECT(actual.lab_seconds_per_world_second == 1.0e-9 && actual.maximum_speed_fraction == physics::maximum_speed_fraction, "the constants are published");
    }

    RIGIDBODIES_TEST("the Guide reads the probe speed and its preset")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        for (const auto value : { 0.9, 0.95, 0.99999 })
        {
            session.apply(speed(value));
            const auto rows = guide_rows(session.build_model());
            const auto field = std::find_if(rows.begin(), rows.end(), [](const ui::PanelRow& row)
                {
                    return row.key == "world.relativity.speed";
                });
            const auto chips = std::find_if(rows.begin(), rows.end(), [](const ui::PanelRow& row)
                {
                    return row.key == "world.relativity.preset";
                });
            RIGIDBODIES_EXPECT(field != rows.end() && field->number_si == value, "the Guide's speed field reads the live speed");
            RIGIDBODIES_EXPECT(chips != rows.end() && chips->selected_option == std::string(ui::relativity_preset_id(value)), "the Guide's chips select the exact preset");
        }
        RIGIDBODIES_EXPECT(ui::relativity_preset_id(0.9) == "0.9" && ui::relativity_preset_id(0.99999) == "0.99999" && ui::relativity_preset_id(0.0) == "0", "a preset's own value selects it");
        RIGIDBODIES_EXPECT(ui::relativity_preset_id(0.95).empty() && ui::relativity_preset_id(0.99998).empty() && ui::relativity_preset_id(0.9999901).empty(), "a speed between presets selects none");
        // Every chip but Rest reads exactly as the formatter writes its speed, and is on the ladder.
        for (const auto& option : ui::find_control_spec("world.relativity.preset")->options)
        {
            const auto value = std::strtod(std::string(option.id).c_str(), nullptr);
            RIGIDBODIES_EXPECT(std::find(physics::speed_fraction_ladder.begin(), physics::speed_fraction_ladder.end(), value) != physics::speed_fraction_ladder.end(), "a preset is a rung of the speed ladder");
            if (value > 0.0)
                RIGIDBODIES_EXPECT(option.label == core::format_quantity(value, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si), "a preset chip reads as its speed's own text: " + std::string(option.label));
        }
    }

    RIGIDBODIES_TEST("saving writes the speed and never the clocks")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.9));
        play(session, 300);
        RIGIDBODIES_EXPECT(race(session).lab_time_s > 0.0 && race(session).proper_time_s > 0.0, "the clocks ran before saving");
        for (const auto current_moment : { false, true })
            for (const std::string_view title : { "", "Fast probe" })
            {
                const auto saved = std::string(current_moment ? "the current moment" : "the starting setup") + (title.empty() ? "" : " under a new title");
                std::string text, error;
                RIGIDBODIES_EXPECT(session.save_arrangement(text, error, title, current_moment, true), error);
                physics::content::Json root;
                RIGIDBODIES_EXPECT(physics::content::parse_json(text, root, error), error);
                RIGIDBODIES_EXPECT(physics::declares_special_relativity(root), saved + " keeps the required feature");
                const auto* relativity = root.find("relativity");
                RIGIDBODIES_EXPECT(relativity && relativity->is_object() && relativity->as_object().size() == 2, saved + " holds only the rest mass and the speed");
                const auto setup = physics::read_relativity_setup(root);
                RIGIDBODIES_EXPECT(setup && setup->speed_fraction == 0.9 && setup->rest_mass_kg == 1.0, saved + " holds the chosen speed, not the authored one");
                for (const auto* counter : { "lab_time", "proper_time", "clock_lag", "completed_laps", "lap_lead", "lap_distance" })
                    RIGIDBODIES_EXPECT(text.find(counter) == std::string::npos, saved + " stores no clock or race reading: " + counter);
                if (!title.empty())
                    RIGIDBODIES_EXPECT(root["metadata"]["id"].as_string() == "fast_probe_setup" && root["metadata"]["collection"].as_string() == "make_your_own", "a titled save is a setup of its own");
                // Opening the file brings the probe back at its speed, with both clocks at zero.
                app::SimulationSession reopened;
                reopened.configure({});
                RIGIDBODIES_EXPECT(reopened.open_arrangement(text, error), error);
                const auto model = reopened.build_model();
                RIGIDBODIES_EXPECT(reopened.relativity_active() && speed_fraction(reopened) == 0.9 && race(reopened).lab_time_s == 0.0 && race(reopened).proper_time_s == 0.0, saved + " reopens at its speed with the clocks at zero");
                RIGIDBODIES_EXPECT(model.scenario_content && model.scenario_content->special_relativity && model.relativity && model.relativity->original_speed_fraction == 0.9, saved + " reopens as a relativity experiment whose own speed is 0.9 c");
                RIGIDBODIES_EXPECT(model.changes.empty() && model.run_state == ui::RunState::ready, saved + " reopens unchanged and Ready");
            }
    }

    RIGIDBODIES_TEST("a speed edit dirties a saved setup and running never does")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        play(session, 120);
        RIGIDBODIES_EXPECT(session.build_model().changes.empty() && !asks_before_leaving(session), "running the clocks is no change to the setup");
        session.apply(speed(0.5));
        RIGIDBODIES_EXPECT(asks_before_leaving(session), "an unsaved speed asks before it is lost");
        session.note_setup_file("probe.json", "Half light speed", "chasing_light");
        RIGIDBODIES_EXPECT(!asks_before_leaving(session), "the saved speed is clean");
        play(session, 240);
        RIGIDBODIES_EXPECT(race(session).lab_time_s > 0.0 && !asks_before_leaving(session), "running after the save stays clean");
        send(session, ui::UiCommandKind::reset_scenario);
        RIGIDBODIES_EXPECT(!asks_before_leaving(session), "Back to start stays clean");
        session.apply(speed(0.9));
        RIGIDBODIES_EXPECT(asks_before_leaving(session), "a later speed edit is unsaved");
        send(session, ui::UiCommandKind::undo);
        RIGIDBODIES_EXPECT(speed_fraction(session) == 0.5 && !asks_before_leaving(session), "undoing back to the saved speed is clean again");
        session.apply(speed(0.99));
        std::string error;
        const auto snapshot = session.capture_current_setup_save(error);
        RIGIDBODIES_EXPECT(snapshot && session.complete_setup_save(*snapshot, "probe.json"), "a quick save of the new speed completes and leaves the setup clean");
        play(session, 60);
        RIGIDBODIES_EXPECT(!asks_before_leaving(session), "and running after it stays clean");
    }

    RIGIDBODIES_TEST("an invalid relativity file is rejected before any prompt and changes nothing")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.5));
        play(session, 60);
        const auto lab = race(session).lab_time_s;
        const auto entries = history_size(session);
        const auto& source = *physics::scenario_document_for_id("chasing_light");
        auto at_light = source.root;
        at_light["relativity"]["speed_fraction_c"] = 1.0;
        auto missing = source.root;
        missing.as_object().erase("relativity");
        for (const auto& [root, field] : { std::pair { at_light, std::string_view { "speed_fraction_c" } }, std::pair { missing, std::string_view { "relativity object" } } })
        {
            std::string error;
            RIGIDBODIES_EXPECT(!session.request_open_arrangement(physics::content::write_json(root), error, "invalid.json"), "the file is rejected");
            RIGIDBODIES_EXPECT(error.find(field) != std::string::npos, "the rejection names " + std::string(field) + ": " + error);
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(!model.confirmation && model.undo_history.size() == entries, "no prompt and no history entry");
            RIGIDBODIES_EXPECT(session.relativity_active() && speed_fraction(session) == 0.5 && race(session).lab_time_s == lab && session.current_setup_path().empty(), "the probe, its clocks and the setup file are untouched");
        }
        std::string text, error;
        RIGIDBODIES_EXPECT(physics::write_scenario_document(source, text, error), error);
        RIGIDBODIES_EXPECT(session.request_open_arrangement(text, error, "valid.json") && session.build_model().confirmation, "a valid file still asks before replacing the unsaved speed");
    }

    RIGIDBODIES_TEST("Newtonian tools are refused cleanly with one notice")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.5));
        session.apply(speed(0.9));
        send(session, ui::UiCommandKind::undo);
        const auto before = session.build_model();
        const auto rail = session.world().body_ids().front();
        const auto rail_mass = session.world().find_body(rail)->mass_properties().mass_kg;
        const auto expect_untouched = [&](std::string_view when)
        {
            const auto model = session.build_model();
            RIGIDBODIES_EXPECT(model.undo_history.size() == before.undo_history.size() && model.undo_label == before.undo_label && model.can_redo && model.redo_label == before.redo_label, "no history entry, and the redo branch survives, after " + std::string(when));
            RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && session.world().find_body(rail)->mass_properties().mass_kg == rail_mass && math::length(session.world().settings().gravity_m_s2) == 0.0, "the World is still the bare track after " + std::string(when));
            RIGIDBODIES_EXPECT(!model.shape_editor_active && model.interaction_mode == "select" && !model.pause_on_impact && session.stepper().is_paused() && race(session).lab_time_s == 0.0, "no tool, impact pause or run starts after " + std::string(when));
            RIGIDBODIES_EXPECT(speed_fraction(session) == 0.5, "the speed is unchanged after " + std::string(when));
        };
        ui::UiCommand add { ui::UiCommandKind::add_object };
        add.id = "ball";
        ui::UiCommand edit { ui::UiCommandKind::edit_selected_shape };
        ui::UiCommand impact_pause { ui::UiCommandKind::set_pause_on_impact };
        impact_pause.flag = true;
        // Draw, Throw, Pull, Play until next impact, Delete, Import, Combine and Separate by their
        // keys, then Add and shape editing as their menus send them.
        const ui::UiCommand tools[] { key_command(ui::UiKey::d), key_command(ui::UiKey::t), key_command(ui::UiKey::p), key_command(ui::UiKey::space, true), key_command(ui::UiKey::delete_key), key_command(ui::UiKey::i, false, true), key_command(ui::UiKey::g, false, true), key_command(ui::UiKey::g, true, true), add, edit, impact_pause };
        std::uint64_t serial = 0;
        for (const auto& tool : tools)
        {
            session.apply(tool);
            const auto model = session.build_model();
            const auto* notice = find_notice(model, "relativity-newtonian");
            RIGIDBODIES_EXPECT(notice && notice->severity == ui::Severity::info && notice->text == "That tool works on Newtonian objects. In this experiment, change the probe's speed instead.", "a refused tool says what to do instead");
            if (notice && serial == 0)
                serial = notice->serial;
            RIGIDBODIES_EXPECT(notice && notice->serial == serial, "only the first refused tool posts the notice");
            expect_untouched("a refused tool");
        }
        // Settings the interface never offers here are refused without a word, even for the rail.
        ui::UiCommand gravity { ui::UiCommandKind::set_gravity_enabled };
        gravity.flag = true;
        ui::UiCommand strength { ui::UiCommandKind::set_gravity_magnitude };
        strength.value = 9.81;
        ui::UiCommand dial { ui::UiCommandKind::set_gravity_angle_degrees };
        dial.value = -45.0;
        ui::UiCommand mass { ui::UiCommandKind::set_selected_mass };
        mass.body = rail;
        mass.value = 5.0;
        ui::UiCommand drag { ui::UiCommandKind::set_drag_enabled };
        drag.flag = true;
        const ui::UiCommand settings[] { gravity, strength, dial, mass, drag, key_command(ui::UiKey::arrow_left, false, false, true), ui::UiCommand { ui::UiCommandKind::compare_collisions } };
        for (const auto& setting : settings)
        {
            session.apply(setting);
            const auto model = session.build_model();
            const auto* notice = find_notice(model, "relativity-newtonian");
            RIGIDBODIES_EXPECT(notice && notice->serial == serial, "a silent refusal posts nothing");
            expect_untouched("a silent refusal");
        }
        // The selection tool stays available, and a shape file cannot be imported either.
        session.apply(key_command(ui::UiKey::v));
        RIGIDBODIES_EXPECT(session.build_model().interaction_mode == "select", "V keeps the selection tool");
        std::string error;
        RIGIDBODIES_EXPECT(!session.import_shape("{}", error) && error == "Shapes cannot be imported into a relativity experiment.", "importing a shape fails closed with a reason");
        expect_untouched("an import");
        // A newly opened experiment explains once more.
        open(session, "chasing_light");
        session.apply(key_command(ui::UiKey::t));
        const auto reopened = session.build_model();
        const auto* again = find_notice(reopened, "relativity-newtonian");
        RIGIDBODIES_EXPECT(again && again->serial != serial && reopened.interaction_mode == "select", "the notice returns for the next opened experiment");
    }

    RIGIDBODIES_TEST("the keyboard reference lists the speed keys only in this experiment")
    {
        const std::pair<std::string_view, std::string_view> speed_keys[] { { "Raise probe speed a little", "Up" }, { "Lower probe speed a little", "Down" }, { "Next speed preset", "Shift+Up" }, { "Previous speed preset", "Shift+Down" } };
        const std::string_view newtonian_keys[] { "Draw shape", "Select all free objects", "Combine selected shapes", "Separate selected shape", "Previous object", "Next object", "Throw", "Pull", "Play until next impact", "Delete selection", "Frame selection", "Import shape", "Open Add menu", "Nudge selection left 1 cm", "Nudge selection right 1 cm", "Nudge selection up 1 cm", "Nudge selection down 1 cm", "Nudge selection left 10 cm", "Nudge selection right 10 cm", "Nudge selection up 10 cm", "Nudge selection down 10 cm" };
        const std::string_view shared_keys[] { "Play / pause", "Single step", "Back to start", "Replay from start", "Frame subject", "Undo", "Redo", "Select & move", "Keyboard shortcuts", "Toggle Measure", "Toggle Present", "Save setup" };
        const auto expect_listed_once = [](const ui::UiModel& model, std::string_view when)
        {
            for (const auto& entry : model.keyboard_reference)
                RIGIDBODIES_EXPECT(std::count_if(model.keyboard_reference.begin(), model.keyboard_reference.end(), [&](const ui::KeyReference& other)
                                       {
                                           return other.description == entry.description;
                                       }) == 1,
                    "each command is listed once " + std::string(when) + ": " + entry.description);
        };
        app::SimulationSession session;
        session.configure({});
        auto model = session.build_model();
        for (const auto& [description, chord] : speed_keys)
            RIGIDBODIES_EXPECT(!find_key(model, description), "a Newtonian experiment lists no speed key: " + std::string(description));
        for (const auto description : newtonian_keys)
            RIGIDBODIES_EXPECT(find_key(model, description), "a Newtonian experiment lists its object keys: " + std::string(description));
        open(session, "chasing_light");
        model = session.build_model();
        for (const auto& [description, chord] : speed_keys)
        {
            const auto* entry = find_key(model, description);
            RIGIDBODIES_EXPECT(entry && entry->chord == chord && entry->category == ui::KeyCategory::editing, "Chasing light lists " + std::string(chord) + " as " + std::string(description));
        }
        for (const auto description : newtonian_keys)
            RIGIDBODIES_EXPECT(!find_key(model, description), "Chasing light leaves out keys it has no use for: " + std::string(description));
        for (const auto description : shared_keys)
            RIGIDBODIES_EXPECT(find_key(model, description), "Chasing light keeps the keys every experiment has: " + std::string(description));
        expect_listed_once(model, "in Chasing light");
        send(session, ui::UiCommandKind::undo);
        model = session.build_model();
        RIGIDBODIES_EXPECT(!session.relativity_active() && !find_key(model, "Next speed preset") && find_key(model, "Draw shape"), "undoing the switch brings back the Newtonian list");
        expect_listed_once(model, "in free fall");
    }

    RIGIDBODIES_TEST("clicking the probe asks to reveal the speed control")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(reference_focus);
        open_chasing_light(session);
        session.set_focus_rect(reference_focus);
        session.apply(speed(0.6));
        session.stepper().set_paused(false);
        for (int frame = 0; frame < 40; ++frame)
            session.advance(1.0 / 60.0);
        session.stepper().set_paused(true);
        render::DrawList list;
        session.render(list);
        const auto& layout = session.scene_renderer().relativity_stage_layout();
        RIGIDBODIES_EXPECT(layout && layout->probe_radius > 0.0 && layout->clock_radius > 0.0, "the stage drew the probe and the clock it carries");
        const auto entries = history_size(session);
        const auto press = [&](math::Vec2 point, double logical_scale = 1.0)
        {
            ui::UiEvent event;
            event.button = ui::PointerButton::primary;
            event.click_count = 1;
            event.logical_pixel_scale = logical_scale;
            event.pointer_px = point;
            event.kind = ui::UiEventKind::pointer_move;
            session.handle_scene_event(event, false);
            event.kind = ui::UiEventKind::pointer_down;
            const auto consumed = session.handle_scene_event(event, false);
            event.kind = ui::UiEventKind::pointer_up;
            session.handle_scene_event(event, false);
            return consumed;
        };
        const auto revealed = [&]()
        {
            const auto request = session.build_model().reveal_request;
            return request && request->key == "world.relativity.speed" ? request->serial : std::uint64_t { 0 };
        };
        // Clear of the probe and its clock: no request, and the ordinary arrow.
        const auto away = layout->probe_center + math::Vec2 { 0.0, 120.0 };
        press(away);
        RIGIDBODIES_EXPECT(revealed() == 0 && session.scene_cursor() == ui::CursorShape::arrow, "a press clear of the probe asks for nothing");
        // On the marker's rim, within the 4 pixel tolerance, then on the face it carries.
        const auto marker = layout->probe_center + math::Vec2 { layout->probe_radius + 3.0, 0.0 };
        RIGIDBODIES_EXPECT(press(marker) && revealed() != 0, "a press on the probe asks to reveal the probe speed");
        RIGIDBODIES_EXPECT(session.scene_cursor() == ui::CursorShape::pointer, "the probe shows the pointer cursor");
        const auto first = revealed();
        RIGIDBODIES_EXPECT(press(layout->probe_clock_center) && revealed() > first, "a press on its clock asks again");
        const auto second = revealed();
        press(layout->probe_center + math::Vec2 { layout->probe_radius + 6.0, 0.0 });
        RIGIDBODIES_EXPECT(revealed() == second, "a press beyond the tolerance asks nothing more");
        // The probe's lower rim lies within the rail's hover margin: pointing there shows the
        // probe's pointer and highlights nothing, while the rail clear of the probe still answers.
        const auto move = [&](math::Vec2 point)
        {
            ui::UiEvent event;
            event.kind = ui::UiEventKind::pointer_move;
            event.logical_pixel_scale = 1.0;
            event.pointer_px = point;
            session.handle_scene_event(event, false);
        };
        move({ layout->track_right - 20.0, layout->rail_y + 1.0 });
        RIGIDBODIES_EXPECT(session.scene_settings().hover.is_valid() && session.scene_cursor() == ui::CursorShape::arrow, "the rail clear of the probe is hovered");
        move(layout->probe_center + math::Vec2 { 0.0, layout->probe_radius - 1.0 });
        RIGIDBODIES_EXPECT(layout->probe_center.y + layout->probe_radius > layout->rail_y - 6.0, "the probe's rim reaches into the rail's hover margin");
        RIGIDBODIES_EXPECT(!session.scene_settings().hover.is_valid() && session.scene_cursor() == ui::CursorShape::pointer, "over the probe's lower rim only the probe answers");
        RIGIDBODIES_EXPECT(history_size(session) == entries && speed_fraction(session) == 0.6 && session.stepper().is_paused(), "a click changes nothing but what is shown");
        expect_consistent(session, "after clicking the probe");
        // At 150 % Text size pointer events carry one and a half times the display scale, and the
        // cursor shows the pointer exactly where a press reveals the speed.
        ui::UiCommand text;
        text.kind = ui::UiCommandKind::set_ui_scale;
        text.value = 1.5;
        session.apply(text);
        session.render(list);
        RIGIDBODIES_EXPECT(layout && layout->probe_radius > 0.0, "the larger stage drew the probe");
        const auto third = revealed();
        RIGIDBODIES_EXPECT(press(layout->probe_center + math::Vec2 { layout->probe_radius + 5.5, 0.0 }, 1.5) && revealed() > third, "a press 5.5 pixels out is within the larger tolerance");
        RIGIDBODIES_EXPECT(session.scene_cursor() == ui::CursorShape::pointer, "and the cursor says so there");
        const auto fourth = revealed();
        press(layout->probe_center + math::Vec2 { layout->probe_radius + 6.5, 0.0 }, 1.5);
        RIGIDBODIES_EXPECT(revealed() == fourth && session.scene_cursor() != ui::CursorShape::pointer, "beyond it neither the press nor the cursor answers");
        // A Newtonian experiment has no probe to click.
        open(session, "free_fall");
        session.render(list);
        RIGIDBODIES_EXPECT(!session.scene_renderer().relativity_stage_layout(), "a Newtonian experiment draws no probe");
    }

    RIGIDBODIES_TEST("drawing the stage changes neither the probe nor the world")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(reference_focus);
        open_chasing_light(session);
        session.set_focus_rect(reference_focus);
        session.apply(speed(0.99999));
        session.stepper().set_paused(false);
        // Mid-way through a lap, with the clocks apart and the light ahead, still running.
        for (int frame = 0; frame < 330; ++frame)
            session.advance(1.0 / 60.0);
        render::DrawList list;
        session.render(list);
        const auto before = session.relativity_probe()->race();
        const auto statistics = session.world().statistics();
        const auto stage = *session.scene_renderer().relativity_stage();
        for (int pass = 0; pass < 3; ++pass)
            session.render(list);
        const auto& after = session.relativity_probe()->race();
        RIGIDBODIES_EXPECT(before.lab_time_s > 0.0 && before.clock_lag_s > 0.0 && !stage.light_finished, "the clocks are apart and the light is on its way");
        RIGIDBODIES_EXPECT(after.lab_time_s == before.lab_time_s && after.proper_time_s == before.proper_time_s && after.clock_lag_s == before.clock_lag_s && after.lap_distance_m == before.lap_distance_m && after.lap_lead_s == before.lap_lead_s && after.lap_elapsed_s == before.lap_elapsed_s && after.completed_laps == before.completed_laps, "drawing advances neither the probe's clocks nor its race");
        RIGIDBODIES_EXPECT(session.world().statistics().step_index == statistics.step_index && session.world().statistics().total_kinetic_energy_j == statistics.total_kinetic_energy_j, "drawing advances nothing in the world");
        const auto& drawn = *session.scene_renderer().relativity_stage();
        RIGIDBODIES_EXPECT(drawn.lab_time_s == stage.lab_time_s && drawn.proper_time_s == stage.proper_time_s && drawn.probe_position_m == stage.probe_position_m && drawn.light_lead_m == stage.light_lead_m, "every pass hands the renderer the same moment");
    }

    RIGIDBODIES_TEST("Changes and runs record the probe speed")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        session.apply(speed(0.9));
        auto model = session.build_model();
        const auto* change = find_change(model.changes, "relativity:speed");
        RIGIDBODIES_EXPECT(model.changes.size() == 1 && change, "the speed is the setup's one change");
        RIGIDBODIES_EXPECT(change && change->control_key == "world.relativity.speed" && change->label == "Probe speed" && !change->body.is_valid() && change->category == ui::EditCategory::parameter, "the change names the speed control and is a parameter");
        RIGIDBODIES_EXPECT(change && change->original_text == "0\xC2\xA0"
                                                              "c" &&
                change->current_text == "0.9\xC2\xA0"
                                        "c",
            "the change reads 0 c to 0.9 c");
        const auto current_text = change ? change->current_text : std::string {};
        play(session, 60);
        model = session.build_model();
        const auto* recorded = model.current_run ? find_change(model.current_run->changes_from_original, "relativity:speed") : nullptr;
        RIGIDBODIES_EXPECT(recorded && recorded->current_text == current_text, "the run records the speed it ran at");
        send(session, ui::UiCommandKind::reset_scenario);
        session.apply(speed(0.99));
        play(session, 60);
        model = session.build_model();
        const auto* from_previous = model.current_run ? find_change(model.current_run->changes_from_previous, "relativity:speed") : nullptr;
        RIGIDBODIES_EXPECT(from_previous && from_previous->current_text == "0.99\xC2\xA0"
                                                                           "c",
            "the next run records the speed it changed to since the last");
        send(session, ui::UiCommandKind::revert_change, "relativity:speed");
        RIGIDBODIES_EXPECT(session.build_model().changes.empty() && speed_fraction(session) == 0.0, "reverting the change removes it");
    }

    RIGIDBODIES_TEST("every command kind leaves the probe's experiment whole")
    {
        // Whatever a panel, a key or a stray menu sends, the experiment stays a bare track with a
        // consistent probe, and nothing throws.
        for (const auto running : { false, true })
        {
            // Opened at start-up, so no undo leads out of the experiment.
            app::SimulationSession session;
            core::ApplicationConfig config;
            config.startup_scenario = "chasing_light";
            session.configure(config);
            RIGIDBODIES_EXPECT(session.relativity_active() && !session.build_model().can_undo, "Chasing light opens at start-up");
            session.apply(speed(0.5));
            if (running)
                play(session, 30);
            const auto rail = session.world().body_ids().front();
            for (auto index = static_cast<int>(ui::UiCommandKind::none); index <= static_cast<int>(ui::UiCommandKind::set_relativity_speed); ++index)
                for (const auto flag : { false, true })
                {
                    ui::UiCommand command;
                    command.kind = static_cast<ui::UiCommandKind>(index);
                    // Quitting ends the session, and loading another experiment leaves this one.
                    if (command.kind == ui::UiCommandKind::quit || command.kind == ui::UiCommandKind::load_scenario)
                        continue;
                    command.flag = flag;
                    command.value = 1.0;
                    command.value_y = 1.0;
                    command.body = rail;
                    command.bodies = { rail };
                    session.apply(command);
                    session.advance(1.0 / 60.0);
                    const auto when = "after command kind " + std::to_string(index) + (flag ? " with its flag" : "") + (running ? " while running" : "");
                    RIGIDBODIES_EXPECT(session.relativity_active() && session.scenario_id() == "chasing_light", "the probe's experiment stays open " + when);
                    RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && math::length(session.world().settings().gravity_m_s2) == 0.0 && !session.build_model().selection.is_valid(), "the World stays the bare, unselectable track " + when);
                    expect_consistent(session, when);
                }
        }
    }

    RIGIDBODIES_TEST("the live speed always equals the starting speed")
    {
        app::SimulationSession session;
        open_chasing_light(session);
        std::mt19937 random(20261005u);
        std::uniform_int_distribution<int> operation(0, 19);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        const auto random_speed = [&]
        {
            // Half spread evenly, half spread over the nines.
            return unit(random) < 0.5 ? unit(random) * physics::maximum_speed_fraction : 1.0 - std::pow(10.0, -7.0 * unit(random));
        };
        for (int index = 0; index < 600; ++index)
        {
            const auto choice = operation(random);
            if (!session.relativity_active() && choice != 15)
            {
                // Away from the experiment only a few operations matter: come back by undo or anew.
                if (unit(random) < 0.5 && session.build_model().can_undo)
                    send(session, ui::UiCommandKind::undo);
                else
                    open(session, "chasing_light");
            }
            else
                switch (choice)
                {
                case 0:
                    session.apply(speed(random_speed()));
                    break;
                case 1:
                    session.apply(speed(unit(random) < 0.5 ? 1.0 : -0.1 - unit(random)));
                    break;
                case 2:
                    session.apply(speed(unit(random) < 0.5 ? 1.0 : -1.0, "nudge"));
                    break;
                case 3:
                    session.apply(speed(unit(random) < 0.5 ? 1.0 : -1.0, "preset"));
                    break;
                case 4:
                    session.apply(speed(random_speed(), {}, ui::UiEditPhase::preview));
                    session.advance(1.0 / 60.0);
                    session.apply(speed(random_speed(), {}, ui::UiEditPhase::preview));
                    break;
                case 5:
                    session.apply(speed(0.0, {}, unit(random) < 0.5 ? ui::UiEditPhase::commit : ui::UiEditPhase::cancel));
                    break;
                case 6:
                case 7:
                    send(session, ui::UiCommandKind::undo);
                    break;
                case 8:
                    send(session, ui::UiCommandKind::redo);
                    break;
                case 9:
                    send(session, ui::UiCommandKind::reset_scenario, {}, unit(random) < 0.3);
                    break;
                case 10:
                    send(session, ui::UiCommandKind::restore_original);
                    break;
                case 11:
                    send(session, ui::UiCommandKind::keep_state_as_setup);
                    break;
                case 12:
                    send(session, ui::UiCommandKind::revert_change, "relativity:speed");
                    break;
                case 13:
                    send(session, ui::UiCommandKind::toggle_pause);
                    break;
                case 14:
                    play(session, 1 + static_cast<int>(unit(random) * 40.0), 1.0 / 60.0);
                    break;
                case 15:
                    open(session, "free_fall");
                    break;
                case 16:
                {
                    ui::UiEvent event;
                    event.kind = ui::UiEventKind::focus_lost;
                    session.handle_scene_event(event, false);
                    break;
                }
                case 17:
                {
                    // Lab settings stay available here and must never carry a run into the setup.
                    static constexpr ui::UiCommandKind kinds[] { ui::UiCommandKind::set_integrator, ui::UiCommandKind::set_fixed_step, ui::UiCommandKind::revert_lab_settings };
                    static constexpr const char* methods[] { "semi_implicit_euler", "velocity_verlet", "runge_kutta_4" };
                    ui::UiCommand lab;
                    lab.kind = kinds[static_cast<std::size_t>(unit(random) * 2.999)];
                    lab.id = methods[static_cast<std::size_t>(unit(random) * 2.999)];
                    lab.value = unit(random) < 0.5 ? 1.0 / 120.0 : 1.0 / 240.0;
                    session.apply(lab);
                    break;
                }
                case 18:
                {
                    // Newtonian tools and settings are refused and must leave the probe alone.
                    static constexpr ui::UiCommandKind kinds[] { ui::UiCommandKind::add_object, ui::UiCommandKind::start_new_shape, ui::UiCommandKind::pause_at_next_impact, ui::UiCommandKind::set_gravity_enabled, ui::UiCommandKind::set_selected_mass, ui::UiCommandKind::delete_selected_body };
                    ui::UiCommand refused;
                    refused.kind = kinds[static_cast<std::size_t>(unit(random) * 5.999)];
                    refused.flag = true;
                    refused.value = 2.0;
                    session.apply(refused);
                    break;
                }
                default:
                {
                    ui::UiCommand step;
                    step.kind = ui::UiCommandKind::step_many;
                    step.value = 1.0 + std::floor(unit(random) * 5.0);
                    session.apply(step);
                    break;
                }
                }
            expect_consistent(session, "after operation " + std::to_string(index) + " (" + std::to_string(choice) + ")");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
