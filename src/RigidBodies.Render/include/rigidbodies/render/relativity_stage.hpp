#pragma once

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/render/camera2d.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace rigidbodies::render
{
    // How much of the relativity apparatus the stage has room for. Every tier draws both clock faces:
    // full has large faces with two lab plates and two race plates, compact smaller faces with one of
    // each, and minimal small faces beside one-line plates, with the probe's face as its marker.
    enum class RelativityStageTier : std::uint8_t
    {
        full,
        compact,
        minimal
    };

    // One frame of the relativity apparatus, already interpolated by the application. SI throughout.
    // The renderer treats non-finite numbers as absent and clamps β into [0, 1).
    struct RelativityStage
    {
        RelativityStageTier tier { RelativityStageTier::full };
        double track_length_m {};         // the rail runs from x = 0 to x = L on y = 0
        double mark_spacing_m {};         // c × 1 ns
        double clock_period_s { 1.0e-9 }; // one turn of either hand
        double rest_mass_kg {};
        double speed_fraction {}, one_minus_speed_fraction { 1.0 };
        double lorentz_factor_minus_one {}, inverse_lorentz_factor { 1.0 }, clock_lag_rate {};
        double speed_m_s {}, below_light_m_s {}, kinetic_energy_j {}, momentum_kg_m_s {};
        double lab_time_s {}, proper_time_s {}, clock_lag_s {};
        double probe_position_m {}; // [0, L)
        double light_lead_m {};     // ≥ 0
        bool light_finished {};
        std::optional<double> last_lap_margin_s;
        bool last_lap_speed_changed {};
        double lab_seconds_per_screen_second { 1.0e-9 }; // k × playback speed, for the slow-motion plate
        core::DisplayUnits units { core::DisplayUnits::si };
    };

    // What the last frame drew, in screen pixels: for the application's click test and for tests.
    struct RelativityStageLayout
    {
        RelativityStageTier tier {};
        ScreenRect band; // empty when not drawn
        Vec2 probe_center;
        double probe_radius {};
        Vec2 probe_clock_center;
        double clock_radius {}; // both faces have this radius, in every tier
        Vec2 lab_clock_center;
        double rail_y {}, track_left {}, track_right {};
        std::vector<ScreenRect> fixed_plates; // band plates, lab plates, race plates and the caption
        double text_pixel_scale {};           // what every size above was multiplied by
        Vec2 pulse_center;
        double pulse_radius {}; // radius 0 once the pulse has reached the finish
    };

    struct SceneRenderSettings;
    // The text pixel scale stage text is drawn at: density times the reader's text size. Relativity
    // framing reproduces it, so the band and stack it makes room for are the ones the stage draws.
    [[nodiscard]] double stage_text_pixel_scale(const SceneRenderSettings& settings);

    inline constexpr double relativity_view_width_m = physics::relativity_track_length_m + 0.5; // the track plus 0.25 m each side
    inline constexpr double relativity_rail_thickness_m = 0.012;                                // the bundled document's rail
    // Logical pixels kept clear at each end of the framed view, beyond its quarter metre, for the
    // labels under the start and finish lines.
    inline constexpr double relativity_end_margin_logical_px = 16.0;
    // The scale relativity framing draws at: the view's width across the framing width less both end
    // margins, but never less than a metre of its height. 0 for a width too narrow for the margins.
    // The session's framing, the tiers and the stage all work from it.
    [[nodiscard]] double relativity_pixels_per_metre(double width_px, double height_px, double text_pixel_scale);
    struct RelativityStack
    {
        double above_px {}, below_px {};
    };
    // Pixels the apparatus needs above and below the rail line for a tier, the scale key's row at
    // the foot included.
    [[nodiscard]] RelativityStack relativity_stack_px(RelativityStageTier tier, double text_pixel_scale, double rail_thickness_px);
    // Height of the instrument band for a framing width: worst-case strings, so it never changes with values.
    [[nodiscard]] double relativity_band_height_px(double framing_width_px, double text_pixel_scale, core::DisplayUnits units);
    // The tier for a framing size, with ±16 logical px of hysteresis around the previous tier.
    [[nodiscard]] RelativityStageTier choose_relativity_tier(double width_px, double height_px, double text_pixel_scale, core::DisplayUnits units, RelativityStageTier previous);
    // The largest tier that fits a framing size, without hysteresis: what a stage settles on once
    // its size holds still.
    [[nodiscard]] RelativityStageTier fitting_relativity_tier(double width_px, double height_px, double text_pixel_scale, core::DisplayUnits units);
}
