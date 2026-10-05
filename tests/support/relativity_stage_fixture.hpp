#pragma once

#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <rigidbodies/render/relativity_stage.hpp>

#include <algorithm>

namespace rigidbodies::testing
{
    // The stage a session hands the renderer for a 1 kg probe set to this speed at Ready and run for
    // this much lab time, built with the same physics helpers, for tests that draw it without a session.
    inline render::RelativityStage relativity_stage_at(double speed_fraction, double lab_time_s, render::RelativityStageTier tier = render::RelativityStageTier::full, core::DisplayUnits units = core::DisplayUnits::si)
    {
        physics::RelativisticProbe probe({ 1.0, speed_fraction });
        probe.advance(lab_time_s);
        const auto& factors = probe.factors();
        const auto quantities = physics::relativistic_quantities(probe.setup().rest_mass_kg, factors);
        const auto sample = probe.sample(1.0);
        render::RelativityStage stage;
        stage.tier = tier;
        stage.track_length_m = probe.track_length_m();
        stage.mark_spacing_m = physics::light_nanosecond_m;
        stage.clock_period_s = 1.0e-9;
        stage.rest_mass_kg = probe.setup().rest_mass_kg;
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
        stage.lab_seconds_per_screen_second = physics::relativity_lab_seconds_per_world_second;
        stage.units = units;
        return stage;
    }

    // A camera framed the way relativity framing frames a focus area: the track and its margins
    // across the width (or a metre of height), the band and the tier's stack from the top, and any
    // height left over shared above and below the apparatus.
    inline render::Camera2D relativity_camera(const render::ViewportSize& viewport, const render::ScreenRect& focus, double text_pixel_scale, render::RelativityStageTier tier, core::DisplayUnits units = core::DisplayUnits::si)
    {
        render::Camera2D camera;
        camera.set_viewport(viewport);
        camera.set_focus_rect(focus);
        const auto band = render::relativity_band_height_px(focus.width, text_pixel_scale, units);
        const auto pixels_per_metre = render::relativity_pixels_per_metre(focus.width, focus.height, text_pixel_scale);
        const auto stack = render::relativity_stack_px(tier, text_pixel_scale, render::relativity_rail_thickness_m * pixels_per_metre);
        const auto slack = std::max(0.0, focus.height - band - stack.above_px - stack.below_px);
        const auto rail_y = focus.top + band + stack.above_px + slack * 0.5;
        const auto half_width_m = focus.width / (2.0 * pixels_per_metre);
        const auto centre_x_m = 0.5 * physics::relativity_track_length_m;
        math::Aabb box;
        box.expand({ centre_x_m - half_width_m, -(focus.top + focus.height - rail_y) / pixels_per_metre });
        box.expand({ centre_x_m + half_width_m, (rail_y - focus.top) / pixels_per_metre });
        camera.frame_bounds(box, 0.0);
        return camera;
    }
}
