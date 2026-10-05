#include <rigidbodies/app/simulation_session.hpp>

#include <rigidbodies/app/setup_changes.hpp>
#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/ui/control_spec.hpp>

#include <algorithm>
#include <cmath>

namespace rigidbodies::app
{
    void SimulationSession::install_relativity(const std::optional<physics::RelativitySetup>& setup)
    {
        relativity_.reset();
        relativity_notice_shown_ = false;
        relativity_framed_.reset();
        // A newly opened experiment starts from the largest tier its stage fits, whatever size the
        // previous one was last drawn at.
        relativity_tier_ = render::RelativityStageTier::full;
        relativity_stage_size_.reset();
        if (!setup)
            return;
        relativity_.emplace(*setup);
        // Throw and Pull act on Newtonian objects, so the probe's experiment always has the
        // selection tool, as a newly loaded experiment does.
        interaction_.mode = InteractionMode::select;
    }

    bool SimulationSession::refuse_in_relativity(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        switch (command.kind)
        {
        // Tools the learner reaches for: refused with one notice that says what to do instead.
        case K::add_object:
        case K::start_new_shape:
        case K::edit_selected_shape:
        case K::import_shape:
        case K::delete_selected_body:
        case K::assemble_selected_bodies:
        case K::split_selected_body:
        case K::pause_at_next_impact:
        case K::set_pause_on_impact:
            break;
        case K::set_interaction_mode:
            if (command.id == "select" || command.id == "move")
                return false;
            break;
        // Settings of the World's bodies and environment, which the interface never offers here:
        // refused silently, so the World stays the probe's static track.
        case K::set_gravity_enabled:
        case K::set_gravity_magnitude:
        case K::set_gravity_angle_degrees:
        case K::set_gravity_preset:
        case K::set_energy_reference_height:
        case K::set_drag_enabled:
        case K::set_angular_drag_enabled:
        case K::set_magnus_enabled:
        case K::set_environment_parameter:
        case K::set_restitution_mixing:
        case K::set_continuous_collision:
        case K::set_force_generator_enabled:
        case K::set_selected_mass:
        case K::use_selected_density_mass:
        case K::set_selected_material:
        case K::set_selected_velocity_x:
        case K::set_selected_velocity_y:
        case K::set_selected_angular_velocity:
        case K::set_selected_gravity_scale:
        case K::set_selected_position:
        case K::set_selected_orientation:
        case K::set_selected_velocity:
        case K::stop_selected_motion:
        case K::select_impact:
        case K::compare_restitution:
        case K::compare_collisions:
        case K::compare_integrators:
            return true;
        // Everything else, the lab settings included, is session-wide and exact for the probe.
        default:
            return false;
        }
        if (!relativity_notice_shown_)
        {
            relativity_notice_shown_ = true;
            notify(ui::Severity::info, "That tool works on Newtonian objects. In this experiment, change the probe's speed instead.", "relativity-newtonian");
        }
        return true;
    }

    bool SimulationSession::apply_relativity_command(const ui::UiCommand& command)
    {
        if (command.kind != ui::UiCommandKind::set_relativity_speed)
            return false;
        // Consumed in every experiment, so a Newtonian one ignores the speed keys: no change and no
        // history entry.
        if (!relativity_ || !std::isfinite(command.value))
            return true;
        const auto now = relativity_->setup().speed_fraction;
        const auto direction = command.value > 0.0 ? 1 : -1;
        auto target = command.value;
        if (command.detail == "nudge")
            target = ui::keyboard_step(ui::find_control_spec("world.relativity.speed")->number, now, direction, ui::StepSize::normal);
        else if (command.detail == "preset")
            target = physics::adjacent_speed_preset(now, direction);
        else if (!command.detail.empty() || !physics::settable_speed_fraction(target))
            return true; // refused, never clamped into a different value than the learner asked for
        // A typed speed a rounding error from a rung (99.999 % parses just below 0.99999) is that
        // rung, so its chip, the limit notice and the next step agree with what it reads. Moving
        // between a rung and a speed that close to it would change nothing the learner can see.
        target = physics::speed_fraction_on_ladder(target);
        if (target == physics::speed_fraction_on_ladder(now))
        {
            if (!command.detail.empty() && direction > 0 && physics::speed_fraction_on_ladder(now) >= physics::maximum_speed_fraction)
                notify(ui::Severity::info, core::format_quantity(physics::maximum_speed_fraction, core::DisplayQuantity::speed_fraction, scene_settings_.display_units) + " is the fastest speed here. A massive object can get ever closer to c but never reach it.", "relativity-top");
            // No mark_edit_changed: an unchanged speed keeps the redo branch.
            return true;
        }
        change_relativity_speed(target);
        return true;
    }

    void SimulationSession::change_relativity_speed(double speed_fraction)
    {
        if (pending_edit_ && !speed_change_time_s_ && run_recorder_.current() && speed_fraction != relativity_->setup().speed_fraction)
            speed_change_time_s_ = world_.statistics().elapsed_time_s;
        relativity_->set_speed_fraction(speed_fraction);
        // A parameter: the live speed and the starting speed are always the same. The probe's value
        // is copied so a negative zero is stored as rest.
        setup_.relativity->speed_fraction = relativity_->setup().speed_fraction;
        mark_edit_changed();
    }

    void SimulationSession::mark_relativity_speed_change(double before, double after, double world_time_s)
    {
        if (!relativity_ || before == after || !run_recorder_.current())
            return;
        const auto units = scene_settings_.display_units;
        const auto speed = [&](double fraction)
        {
            return core::format_quantity(fraction, core::DisplayQuantity::speed_fraction, units);
        };
        run_recorder_.add_marker(world_time_s, core::substitute("Speed {} → {}", speed(before), speed(after)));
    }

    void SimulationSession::advance_relativity(double world_substep_s, bool first_substep)
    {
        if (!relativity_)
            return;
        // The same fixed steps as the World, so lab time is always the world clock times k and the
        // probe interpolates between the same two steps as the bodies.
        if (first_substep)
            relativity_->capture_previous();
        relativity_->advance(world_substep_s * physics::relativity_lab_seconds_per_world_second);
    }

    double SimulationSession::render_alpha() const
    {
        return stepper_.is_paused() || stepper_.time_scale() == 0.0 ? 1.0 : stepper_.interpolation_fraction();
    }

    double SimulationSession::relativity_text_pixel_scale() const
    {
        // The renderer's own text pixel scale at the text size the next frame draws with, so the
        // band and the stack that framing makes room for are the ones the stage draws.
        auto settings = scene_settings_;
        settings.text_scale = static_cast<float>(stage_text_scale());
        return render::stage_text_pixel_scale(settings);
    }

    void SimulationSession::update_relativity_tier(const render::ScreenRect& focus, bool settled)
    {
        const auto tps = relativity_text_pixel_scale();
        const auto units = scene_settings_.display_units;
        // A stage that holds still gets the largest tier that fits. While it keeps changing size,
        // as under a dragged window edge, hysteresis keeps the tier it had near a threshold.
        relativity_tier_ = settled ? render::fitting_relativity_tier(focus.width, focus.height, tps, units) : render::choose_relativity_tier(focus.width, focus.height, tps, units, relativity_tier_);
    }

    void SimulationSession::follow_relativity_stage_size(const render::ScreenRect& focus)
    {
        // The same area at the same text scale and units as last frame: the stage holds still, so a
        // smaller tier a passing size forced on it (Present's caption before it is measured, say)
        // gives way to the largest that fits.
        const RelativityStageSize size { focus, relativity_text_pixel_scale(), scene_settings_.display_units };
        const auto settled = relativity_stage_size_ && relativity_stage_size_->focus.left == focus.left && relativity_stage_size_->focus.top == focus.top && relativity_stage_size_->focus.width == focus.width &&
            relativity_stage_size_->focus.height == focus.height && std::abs(relativity_stage_size_->text_pixel_scale - size.text_pixel_scale) <= 1.0e-9 && relativity_stage_size_->units == size.units;
        update_relativity_tier(focus, settled);
        relativity_stage_size_ = size;
    }

    void SimulationSession::frame_relativity_stage()
    {
        const auto focus = camera_.focus_rect();
        if (focus.empty())
        {
            // Without a published focus area, as in a headless session, the document's view stands in.
            if (const auto authored = authored_view_bounds())
                camera_.frame_bounds(*authored, 0.0);
            relativity_framed_.reset();
            return;
        }
        // For the area the tier already follows, this changes nothing.
        update_relativity_tier(focus, false);
        const auto tps = relativity_text_pixel_scale();
        const auto units = scene_settings_.display_units;
        const auto band = render::relativity_band_height_px(focus.width, tps, units);
        // The track and its margins fill the width. The focus area never shows less than a metre of
        // height, and the whole view never more than the camera allows, so the camera keeps this scale.
        const auto most_ppm = focus.height / 1.0;
        const auto least_ppm = std::min(most_ppm, std::max(focus.height, static_cast<double>(camera_.viewport().height)) / camera_.maximum_view_height_m());
        const auto ppm = std::clamp(render::relativity_pixels_per_metre(focus.width, focus.height, tps), least_ppm, most_ppm);
        const auto stack = render::relativity_stack_px(relativity_tier_, tps, render::relativity_rail_thickness_m * ppm);
        // What the apparatus leaves over is shared above and below it, so it sits in the middle of
        // the room under the band.
        const auto slack = std::max(0.0, focus.height - band - stack.above_px - stack.below_px);
        const auto rail_y = focus.top + band + stack.above_px + slack * 0.5;
        const auto half_width_m = focus.width / (2.0 * ppm);
        const auto centre_x = 0.5 * physics::relativity_track_length_m;
        math::Aabb box;
        box.expand({ centre_x - half_width_m, -(focus.top + focus.height - rail_y) / ppm });
        box.expand({ centre_x + half_width_m, (rail_y - focus.top) / ppm });
        // The box has the focus area's aspect, so the camera reproduces this scale exactly.
        camera_.frame_bounds(box, 0.0);
        relativity_framed_ = RelativityFraming { focus, tps, units, relativity_tier_ };
    }

    bool SimulationSession::relativity_framing_stale(const render::ScreenRect& focus) const
    {
        if (!relativity_framed_)
            return true;
        const auto& framed = *relativity_framed_;
        return framed.focus.left != focus.left || framed.focus.top != focus.top || framed.focus.width != focus.width || framed.focus.height != focus.height ||
            std::abs(framed.text_pixel_scale - relativity_text_pixel_scale()) > 1.0e-9 || framed.units != scene_settings_.display_units || framed.tier != relativity_tier_;
    }

    std::vector<ui::SetupChange> SimulationSession::setup_changes() const
    {
        auto changes = compute_setup_changes(setup_view_, original_view_);
        if (setup_.relativity && original_.relativity && setup_.relativity->speed_fraction != original_.relativity->speed_fraction)
        {
            const auto units = scene_settings_.display_units;
            auto original = core::format_quantity(original_.relativity->speed_fraction, core::DisplayQuantity::speed_fraction, units);
            auto current = core::format_quantity(setup_.relativity->speed_fraction, core::DisplayQuantity::speed_fraction, units);
            changes.push_back({ "relativity:speed", "world.relativity.speed", {}, "Probe speed", std::move(original), std::move(current), ui::EditCategory::parameter });
        }
        return changes;
    }

    std::optional<render::RelativityStage> SimulationSession::relativity_stage() const
    {
        if (!relativity_)
            return std::nullopt;
        const auto& setup = relativity_->setup();
        const auto& factors = relativity_->factors();
        const auto quantities = physics::relativistic_quantities(setup.rest_mass_kg, factors);
        const auto sample = relativity_->sample(render_alpha());
        render::RelativityStage stage;
        stage.tier = relativity_tier_;
        stage.track_length_m = relativity_->track_length_m();
        stage.mark_spacing_m = physics::light_nanosecond_m;
        stage.clock_period_s = 1.0e-9;
        stage.rest_mass_kg = setup.rest_mass_kg;
        stage.speed_fraction = factors.speed_fraction;
        stage.one_minus_speed_fraction = factors.one_minus_speed_fraction;
        stage.lorentz_factor_minus_one = factors.lorentz_factor_minus_one;
        stage.inverse_lorentz_factor = factors.inverse_lorentz_factor;
        stage.clock_lag_rate = factors.clock_lag_rate;
        stage.speed_m_s = quantities.speed_m_s;
        stage.below_light_m_s = quantities.below_light_m_s;
        stage.kinetic_energy_j = quantities.kinetic_energy_j;
        stage.momentum_kg_m_s = quantities.momentum_kg_m_s;
        stage.lab_time_s = sample.lab_time_s;
        stage.proper_time_s = sample.proper_time_s;
        stage.clock_lag_s = sample.clock_lag_s;
        stage.probe_position_m = sample.probe_position_m;
        stage.light_lead_m = sample.light_lead_m;
        stage.light_finished = sample.light_finished;
        if (sample.last_lap)
        {
            stage.last_lap_margin_s = sample.last_lap->margin_s;
            stage.last_lap_speed_changed = sample.last_lap->speed_changed;
        }
        stage.lab_seconds_per_screen_second = stepper_.time_scale() * physics::relativity_lab_seconds_per_world_second;
        stage.units = scene_settings_.display_units;
        return stage;
    }

    bool SimulationSession::hit_relativity_probe(const math::Vec2& screen_point_px, double logical_scale) const
    {
        const auto& layout = scene_renderer_.relativity_stage_layout();
        if (!relativity_ || !layout || !math::is_finite(screen_point_px))
            return false;
        const auto tolerance = 4.0 * std::max(0.01, std::isfinite(logical_scale) ? logical_scale : 1.0);
        const auto within = [&](const math::Vec2& centre, double radius)
        {
            return radius > 0.0 && math::length(screen_point_px - centre) <= radius + tolerance;
        };
        return within(layout->probe_center, layout->probe_radius) || within(layout->probe_clock_center, layout->clock_radius);
    }

    void SimulationSession::populate_relativity_model(ui::UiModel& model) const
    {
        if (!relativity_)
            return;
        const auto& setup = relativity_->setup();
        const auto& factors = relativity_->factors();
        const auto quantities = physics::relativistic_quantities(setup.rest_mass_kg, factors);
        // The sample the stage draws this frame, so every reading agrees with the clocks on stage.
        const auto sample = relativity_->sample(render_alpha());
        ui::RelativityModel value;
        value.rest_mass_kg = setup.rest_mass_kg;
        value.speed_fraction = factors.speed_fraction;
        value.one_minus_speed_fraction = factors.one_minus_speed_fraction;
        value.rapidity = factors.rapidity;
        value.original_speed_fraction = original_.relativity ? original_.relativity->speed_fraction : 0.0;
        value.maximum_speed_fraction = physics::maximum_speed_fraction;
        value.speed_m_s = quantities.speed_m_s;
        value.below_light_m_s = quantities.below_light_m_s;
        value.lorentz_factor = factors.lorentz_factor;
        value.lorentz_factor_minus_one = factors.lorentz_factor_minus_one;
        value.inverse_lorentz_factor = factors.inverse_lorentz_factor;
        value.clock_lag_rate = factors.clock_lag_rate;
        value.rest_energy_j = quantities.rest_energy_j;
        value.kinetic_energy_j = quantities.kinetic_energy_j;
        value.newtonian_kinetic_energy_j = quantities.newtonian_kinetic_energy_j;
        value.momentum_kg_m_s = quantities.momentum_kg_m_s;
        value.newtonian_momentum_kg_m_s = quantities.newtonian_momentum_kg_m_s;
        value.lab_time_s = sample.lab_time_s;
        value.proper_time_s = sample.proper_time_s;
        value.clock_lag_s = sample.clock_lag_s;
        value.light_lead_m = sample.light_lead_m;
        value.light_finished = sample.light_finished;
        value.completed_laps = sample.completed_laps;
        if (sample.last_lap)
        {
            value.last_lap_margin_s = sample.last_lap->margin_s;
            value.last_lap_speed_changed = sample.last_lap->speed_changed;
        }
        value.lab_seconds_per_world_second = physics::relativity_lab_seconds_per_world_second;
        model.relativity = value;
    }
}
