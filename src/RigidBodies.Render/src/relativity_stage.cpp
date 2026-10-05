#include <rigidbodies/render/relativity_stage.hpp>

#include <rigidbodies/physics/units.hpp>
#include <rigidbodies/render/scene_renderer.hpp>

#include "scene_style.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace rigidbodies::render
{
    static_assert(core::speed_of_light_m_s == physics::speed_of_light_m_s, "Core's copy of c must be the SI definition Physics uses");

    namespace
    {
        using core::DisplayQuantity;
        using namespace detail;

        // Logical pixels, multiplied by the text pixel scale.
        constexpr double band_inset_x = 12.0, band_inset_top = 8.0, band_inset_bottom = 6.0;
        constexpr double band_gap = 6.0, band_row_gap = 4.0;
        // The "how close to c" gauge: eight cells of 12 × 8 with 2 between, and a gap either side.
        constexpr int gauge_cells = 8;
        constexpr double gauge_cell_width = 12.0, gauge_cell_height = 8.0, gauge_cell_gap = 2.0;
        constexpr double gauge_cells_width = gauge_cells * gauge_cell_width + (gauge_cells - 1) * gauge_cell_gap;
        constexpr double gauge_gap = 6.0;
        constexpr std::size_t band_plate_count = 8;
        // The scale key stands this far above the framing area's foot; when the apparatus fills the
        // height it shares the footer row with the caption or the lab plate.
        constexpr double key_inset = 10.0;
        // Fixed plates beside the lab clock start this far right of its face, and the race plates
        // end this far past the finish line. On the footer row the lab face stands this far left of
        // its plate.
        constexpr double lab_plate_gap = 8.0, race_plate_overhang = 8.0, footer_gap = 3.0;
        // The radius a face's ticks, inset and hub were drawn for; other faces scale them.
        constexpr double reference_face_radius = 18.0;
        // The tops of the start and finish lines, and how far marks reach either side of the track.
        constexpr double finish_rise = 20.0, mark_reach = 6.0;
        // The light pulse's glow and the tail it leaves behind on a dark stage.
        constexpr double pulse_glow_radius = 11.0, pulse_tail_length = 28.0;

        // The label solver's key namespace for the stage (kind 5): the probe's clock plate, and the
        // obstacles it keeps clear of.
        constexpr std::uint64_t stage_kind = std::uint64_t { 5 } << 56u;
        constexpr std::uint64_t probe_clock_key = stage_kind ^ 1u;
        constexpr std::uint64_t pulse_obstacle = stage_kind ^ 2u;
        constexpr std::uint64_t numbers_obstacle = stage_kind ^ 3u;

        // Where each tier puts the apparatus, in logical pixels from the rail line. Every tier draws
        // both clock faces at one radius, so the two clocks compare at a glance. The probe's face
        // passes beneath the lab plates, the race plates and the lab face with a few pixels to spare.
        struct TierGeometry
        {
            double above {};                         // the highest drawing above the rail line, with a margin
            double lane {}, lane_clearance {};       // the light lane's depth below the line, and below a thick rail
            double numbers {}, footer {};            // below the lane: the mark numbers' centre and the footer row's top
            double marker_radius {}, marker_rise {}; // radius 0: the probe's own face is its marker
            double face_radius {}, probe_face_rise {};
            double lab_face_rise {};          // 0: the lab face stands on the footer row, beside its plate
            double lab_row {}, second_row {}; // plate row centres above the line; 0 where the tier has none
            double pulse_radius {};
        };

        // Full: large faces, the lab face on its post with its two plates beside it, and the race on
        // the same two rows at the finish. Compact: smaller faces, the lab plate on the footer row
        // under the post and one race plate. Minimal: small faces, the probe's face riding the rail as
        // its marker and the lab face beside its plate on the footer row; with the band it fits a 720p
        // projector at 150 % text.
        constexpr TierGeometry full_geometry { 144.0, 22.0, 8.0, 13.0, 22.0, 8.0, 9.0, 27.0, 54.0, 113.0, 124.5, 101.5, 5.0 };
        constexpr TierGeometry compact_geometry { 93.0, 18.0, 7.0, 13.0, 22.0, 7.0, 8.0, 16.0, 37.0, 73.0, 0.0, 67.0, 4.0 };
        constexpr TierGeometry minimal_geometry { 25.0, 11.0, 6.0, 12.0, 21.0, 0.0, 0.0, 11.0, 12.0, 0.0, 0.0, 0.0, 4.0 };

        const TierGeometry& tier_geometry(RelativityStageTier tier)
        {
            switch (tier)
            {
            case RelativityStageTier::full:
                return full_geometry;
            case RelativityStageTier::compact:
                return compact_geometry;
            case RelativityStageTier::minimal:
                break;
            }
            return minimal_geometry;
        }

        double usable_scale(double text_pixel_scale)
        {
            return std::isfinite(text_pixel_scale) && text_pixel_scale > 0.0 ? text_pixel_scale : 1.0;
        }

        // The light lane runs below the rail, clear of a thick rail as well as of the marks.
        double lane_depth_px(const TierGeometry& geometry, double text_pixel_scale, double rail_thickness_px)
        {
            const auto rail = std::isfinite(rail_thickness_px) ? std::max(0.0, rail_thickness_px) : 0.0;
            return std::max(geometry.lane * text_pixel_scale, rail + geometry.lane_clearance * text_pixel_scale);
        }

        // The rail as relativity framing draws it in a framing area.
        double framed_rail_thickness_px(double width_px, double height_px, double text_pixel_scale)
        {
            return relativity_rail_thickness_m * relativity_pixels_per_metre(width_px, height_px, text_pixel_scale);
        }

        // The widest text any of these values formats to, so a plate sized for it never changes width
        // as the value changes.
        double widest(std::initializer_list<double> values, DisplayQuantity quantity, core::DisplayUnits units, float scale, bool with_unit = true)
        {
            double width = 0.0;
            for (const auto value : values)
                width = std::max(width, text_width(with_unit ? core::format_quantity(value, quantity, units) : core::format_value(value, quantity, units), scale));
            return width;
        }

        std::string_view band_name(std::size_t index, int level)
        {
            constexpr std::array<std::string_view, band_plate_count> full_names { "Rest mass m", "Speed v", "v/c", "Lorentz factor \xCE\xB3", "Kinetic energy K", "Momentum p", "How close to c", "" };
            constexpr std::array<std::string_view, band_plate_count> symbols { "m", "v", "v/c", "\xCE\xB3", "K", "p", "", "" };
            return index < band_plate_count ? (level == 0 ? full_names[index] : symbols[index]) : std::string_view {};
        }

        std::string slow_motion_text(double lab_seconds_per_screen_second, core::DisplayUnits units, int level)
        {
            const auto value = core::format_quantity(lab_seconds_per_screen_second, DisplayQuantity::fine_time, units);
            return level == 0 ? "Slow motion: 1\xC2\xA0s on screen = " + value + " in the lab" : "1\xC2\xA0s = " + value;
        }

        // The width of each band plate at a level (0: full names, 1: symbols), from the worst-case
        // text of each reading in these units. The rest mass is any mass a document may hold.
        std::array<double, band_plate_count> band_plate_widths(int level, double text_pixel_scale, core::DisplayUnits units)
        {
            const auto metrics = label_metrics_at(static_cast<float>(text_pixel_scale));
            const auto scale = metrics.scale;
            const auto text = [&](std::string_view value)
            {
                return text_width(value, scale);
            };
            const auto space = text(" ");
            const auto name = [&](std::size_t index)
            {
                const auto shown = band_name(index, level);
                return shown.empty() ? 0.0 : text(shown) + space;
            };
            constexpr auto c = core::speed_of_light_m_s;
            // Speeds are sized from 10⁻⁹ c (a typed 0.3 m/s) up to the maximum. Below half c the v/c
            // text gains a digit for each decade nearer rest, so its widest case is three figures in
            // the slowest decade, 0.00000000123; anything slower is abbreviated when drawn.
            constexpr double slowest_sized_speed_fraction = 1.23e-9;
            constexpr auto fastest = physics::maximum_speed_fraction;
            const auto mass = widest({ physics::minimum_relativity_mass_kg, 8.88e-6, 8.88e-3, 8.88, 888.0, 8.88e4, 8.88e5, physics::maximum_relativity_mass_kg }, DisplayQuantity::mass, units, scale);
            const auto speed = widest({ fastest * c, 0.99999985 * c, slowest_sized_speed_fraction * c }, DisplayQuantity::relativistic_speed, units, scale);
            const auto fraction = widest({ fastest, 0.99999985, slowest_sized_speed_fraction }, DisplayQuantity::speed_fraction, units, scale, false);
            // Shown beside v/c from 0.9 c on, where the gap is largest.
            const auto below = text("\xC2\xB7 below c by ") + widest({ 0.1 * c }, DisplayQuantity::speed_gap, units, scale);
            const auto lorentz = widest({ 5.0e-19, 8.88e-18 }, DisplayQuantity::lorentz_factor_excess, units, scale);
            const auto energy = widest({ 8.88e20, 8.88e-18 }, DisplayQuantity::relativistic_energy, units, scale);
            const auto momentum = widest({ 8.88e11, 8.88e-18 }, DisplayQuantity::relativistic_momentum, units, scale);
            const auto arrow = text("\xE2\x86\x92 c");
            const auto pad = 2.0 * metrics.pad_x;
            return {
                pad + name(0) + mass,
                pad + name(1) + speed,
                pad + name(2) + fraction + (level == 0 ? space + below : 0.0),
                pad + name(3) + lorentz,
                pad + name(4) + energy,
                pad + name(5) + momentum,
                pad + (level == 0 ? text(band_name(6, 0)) + gauge_gap * text_pixel_scale : 0.0) + (gauge_cells_width + gauge_gap) * text_pixel_scale + arrow,
                pad + text(slow_motion_text(0.05e-9, units, level)),
            };
        }

        struct BandLayout
        {
            int level {};
            std::size_t rows {};
            double height_px {};
            // Relative to the framing area's top-left corner.
            std::array<ScreenRect, band_plate_count> plates {};
        };

        // The instrument band across the top of the framing area: plates in reading order, filled
        // greedily into rows. Full names when they fit in two rows, else symbols.
        BandLayout relativity_band_layout(double framing_width_px, double text_pixel_scale, core::DisplayUnits units)
        {
            const auto tps = usable_scale(text_pixel_scale);
            const auto plate_height = label_metrics_at(static_cast<float>(tps)).height;
            const auto inner = std::isfinite(framing_width_px) ? std::max(0.0, framing_width_px - 2.0 * band_inset_x * tps) : 0.0;
            BandLayout layout;
            for (const auto level : { 0, 1 })
            {
                const auto widths = band_plate_widths(level, tps, units);
                layout.level = level;
                layout.rows = 1;
                double x = 0.0;
                for (std::size_t index = 0; index < widths.size(); ++index)
                {
                    if (x > 0.0 && x + widths[index] > inner)
                    {
                        ++layout.rows;
                        x = 0.0;
                    }
                    const auto top = band_inset_top * tps + static_cast<double>(layout.rows - 1) * (plate_height + band_row_gap * tps);
                    layout.plates[index] = { band_inset_x * tps + x, top, widths[index], plate_height };
                    x += widths[index] + band_gap * tps;
                }
                if (layout.rows <= 2)
                    break;
            }
            const auto rows = static_cast<double>(layout.rows);
            layout.height_px = band_inset_top * tps + rows * plate_height + (rows - 1.0) * band_row_gap * tps + band_inset_bottom * tps;
            return layout;
        }

        double finite_or(double value, double fallback)
        {
            return std::isfinite(value) ? value : fallback;
        }

        std::pair<Vec2, Vec2> corners(const ScreenRect& rect)
        {
            return { { rect.left, rect.top }, { rect.left + rect.width, rect.top + rect.height } };
        }

        ScreenRect square_around(const Vec2& centre, double radius)
        {
            return { centre.x - radius, centre.y - radius, 2.0 * radius, 2.0 * radius };
        }

        bool overlaps(const ScreenRect& a, const ScreenRect& b)
        {
            return a.left < b.left + b.width && b.left < a.left + a.width && a.top < b.top + b.height && b.top < a.top + a.height;
        }

        // A house plate: the plate tone at partial opacity, with a hairline edge where the theme has one.
        void draw_plate(DrawList& list, const ScreenRect& rect, const LabelMetrics& metrics, const Theme& theme, double ds, bool edged = true)
        {
            const auto [minimum, maximum] = corners(rect);
            list.add_rounded_rectangle_fill(minimum, maximum, static_cast<float>(metrics.radius), theme.label_plate);
            if (edged && theme.label_border.alpha > 0.0f)
                list.add_rounded_rectangle_outline(minimum + Vec2 { 0.5 * ds, 0.5 * ds }, maximum - Vec2 { 0.5 * ds, 0.5 * ds }, static_cast<float>(metrics.radius), theme.label_border, static_cast<float>(ds));
        }

        // Where text sits in a plate row whose top is `top`, as place_labels sets it.
        double text_baseline(double top, const LabelMetrics& metrics)
        {
            return std::round(top + (metrics.height + metrics.cap_height) * 0.5 - metrics.ascender);
        }

        // A fixed plate's two runs: a reading and an optional muted remark the plate drops first.
        struct PlateText
        {
            std::string primary, secondary;
            Color primary_color, secondary_color;
        };

        double plate_width(const PlateText& text, const LabelMetrics& metrics, double separator)
        {
            const auto secondary = text.secondary.empty() ? 0.0 : separator + text_width(text.secondary, metrics.scale);
            return std::round(2.0 * metrics.pad_x + text_width(text.primary, metrics.scale) + secondary);
        }

        void draw_plate_text(DrawList& list, const ScreenRect& rect, const PlateText& text, const LabelMetrics& metrics, double separator)
        {
            const auto y = text_baseline(rect.top, metrics);
            const auto x = rect.left + metrics.pad_x;
            list.add_text({ x, y }, text.primary, text.primary_color, metrics.scale);
            if (!text.secondary.empty())
                list.add_text({ x + text_width(text.primary, metrics.scale) + separator, y }, text.secondary, text.secondary_color, metrics.scale);
        }

        // How a face is inked: its rim, its hand and the hub the hand turns on.
        struct ClockStyle
        {
            Color rim;
            float rim_width {};
            Color hand, hub;
        };

        // A clock face in the gravity dial's vocabulary: a plate-toned disc over a soft shadow, a rim,
        // ten ticks (a tenth of a turn each, the twelve o'clock one longer), a hand and a hub. Ticks
        // and inset keep their proportions at every radius, and the hub grows with a larger face; on
        // a smaller one the hub keeps its 2.5 logical pixels and no tick is shorter than 2.
        void draw_clock_face(DrawList& list, const Vec2& centre, double radius, double time_s, double period_s, const ClockStyle& style, const SceneRenderSettings& settings, double tps)
        {
            const auto& theme = settings.theme;
            const auto ds = static_cast<double>(pixel_scale(settings));
            const auto proportion = radius / (reference_face_radius * tps);
            list.add_circle_shadow(centre, static_cast<float>(radius), theme.shadow.with_alpha(theme.shadow.alpha * 0.45f), static_cast<float>(5.0 * ds), Vec2 { 0.0, 1.5 * ds });
            list.add_circle_fill(centre, static_cast<float>(radius), theme.label_plate);
            list.add_circle_outline(centre, static_cast<float>(radius), style.rim, style.rim_width);
            const auto tick_color = theme.label_muted.with_alpha(theme.label_muted.alpha * 0.8f);
            const auto outer = radius - 2.0 * proportion * tps;
            for (int tick = 0; tick < 10; ++tick)
            {
                const auto angle = math::two_pi * static_cast<double>(tick) / 10.0;
                const Vec2 radial { std::sin(angle), -std::cos(angle) };
                const auto inner = outer - std::max((tick == 0 ? 6.0 : 3.0) * proportion, 2.0) * tps;
                list.add_line(centre + radial * inner, centre + radial * outer, tick_color, stroke(settings, 1.0f), StrokeCap::butt);
            }
            // One turn per period, clockwise from twelve o'clock: the reading's fraction of a period.
            if (std::isfinite(time_s) && std::isfinite(period_s) && period_s > 0.0)
            {
                auto turn = std::fmod(time_s, period_s) / period_s;
                if (turn < 0.0)
                    turn += 1.0;
                const auto angle = math::two_pi * turn;
                const Vec2 radial { std::sin(angle), -std::cos(angle) };
                list.add_line(centre, centre + radial * (0.78 * radius), style.hand, stroke(settings, 2.0f), StrokeCap::round);
            }
            list.add_circle_fill(centre, static_cast<float>(std::max(2.5, 2.5 * proportion) * tps), style.hub);
        }

        // A disc's outline for the label solver, which treats obstacles as polygons.
        std::array<Vec2, 12> disc_outline(const Vec2& centre, double radius)
        {
            std::array<Vec2, 12> points {};
            for (std::size_t index = 0; index < points.size(); ++index)
            {
                const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(points.size());
                points[index] = centre + Vec2 { std::cos(angle), std::sin(angle) } * radius;
            }
            return points;
        }

        // The probe clock's rate in words: exact at rest, the loss per nanosecond while it is under a
        // thousandth, else the fraction of the lab rate.
        std::string clock_rate_text(const RelativityStage& stage)
        {
            if (stage.speed_fraction == 0.0)
                return "ticks at the lab rate";
            if (!std::isfinite(stage.clock_lag_rate) || !std::isfinite(stage.inverse_lorentz_factor))
                return {};
            if (stage.clock_lag_rate < 0.001)
                return "loses " + core::format_quantity(stage.clock_lag_rate * 1.0e-9, DisplayQuantity::duration, stage.units) + " each lab nanosecond";
            return "runs at " + core::format_significant(stage.inverse_lorentz_factor) + " of the lab rate";
        }
    }

    double stage_text_pixel_scale(const SceneRenderSettings& settings)
    {
        return static_cast<double>(text_pixel_scale(settings));
    }

    double relativity_pixels_per_metre(double width_px, double height_px, double text_pixel_scale)
    {
        const auto margins_px = 2.0 * relativity_end_margin_logical_px * text_pixel_scale;
        return std::max(0.0, std::min((width_px - margins_px) / relativity_view_width_m, height_px / 1.0));
    }

    RelativityStack relativity_stack_px(RelativityStageTier tier, double text_pixel_scale, double rail_thickness_px)
    {
        const auto tps = usable_scale(text_pixel_scale);
        const auto& geometry = tier_geometry(tier);
        const auto plate = label_metrics_at(static_cast<float>(tps)).height;
        // Below the lane: the marks and their numbers, then the footer row and the scale key's inset.
        return { geometry.above * tps, lane_depth_px(geometry, tps, rail_thickness_px) + geometry.footer * tps + plate + key_inset * tps };
    }

    double relativity_band_height_px(double framing_width_px, double text_pixel_scale, core::DisplayUnits units)
    {
        return relativity_band_layout(framing_width_px, text_pixel_scale, units).height_px;
    }

    RelativityStageTier choose_relativity_tier(double width_px, double height_px, double text_pixel_scale, core::DisplayUnits units, RelativityStageTier previous)
    {
        if (!std::isfinite(width_px) || !std::isfinite(height_px) || width_px <= 0.0 || height_px <= 0.0)
            return RelativityStageTier::minimal;
        const auto tps = usable_scale(text_pixel_scale);
        const auto band = relativity_band_height_px(width_px, tps, units);
        const auto rail = framed_rail_thickness_px(width_px, height_px, tps);
        const auto needed = [&](RelativityStageTier tier)
        {
            const auto stack = relativity_stack_px(tier, tps, rail);
            return band + stack.above_px + stack.below_px;
        };
        const auto logical_width = width_px / tps;
        // A tier is entered with 16 logical pixels to spare and kept until it no longer fits, so a
        // size near a threshold does not flip between tiers.
        const auto enters = [&](RelativityStageTier tier, double minimum_width)
        {
            return logical_width >= minimum_width && height_px >= needed(tier) + 16.0 * tps;
        };
        const auto stays = [&](RelativityStageTier tier, double minimum_width)
        {
            return logical_width >= minimum_width - 16.0 && height_px >= needed(tier);
        };
        if (enters(RelativityStageTier::full, 700.0) || (previous == RelativityStageTier::full && stays(RelativityStageTier::full, 700.0)))
            return RelativityStageTier::full;
        if (enters(RelativityStageTier::compact, 420.0) || (previous != RelativityStageTier::minimal && stays(RelativityStageTier::compact, 420.0)))
            return RelativityStageTier::compact;
        return RelativityStageTier::minimal;
    }

    RelativityStageTier fitting_relativity_tier(double width_px, double height_px, double text_pixel_scale, core::DisplayUnits units)
    {
        // From the full tier, the hysteresis keeps whichever tier still fits: the largest one.
        return choose_relativity_tier(width_px, height_px, text_pixel_scale, units, RelativityStageTier::full);
    }

    void SceneRenderer::set_relativity_stage(std::optional<RelativityStage> stage)
    {
        // A Newtonian experiment keeps nothing of the apparatus: no drawn geometry, no reserved
        // instrument areas and no remembered side for the probe's clock plate.
        if (!stage)
        {
            relativity_layout_.reset();
            stage_instrument_areas_.clear();
            detector_fill_ = 0.0;
            label_memory_.erase(std::remove_if(label_memory_.begin(), label_memory_.end(), [](const std::pair<std::uint64_t, int>& entry)
                                    {
                                        return (entry.first >> 56u) == (stage_kind >> 56u);
                                    }),
                label_memory_.end());
        }
        relativity_stage_ = std::move(stage);
    }

    void SceneRenderer::draw_relativity_stage(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list)
    {
        relativity_layout_.reset();
        if (!relativity_stage_)
            return;
        const auto& stage = *relativity_stage_;
        const auto length_m = stage.track_length_m, spacing_m = stage.mark_spacing_m;
        const auto pixels_per_metre = camera.pixels_per_metre();
        // Without a track there is nothing to draw the apparatus on.
        if (!(std::isfinite(length_m) && length_m > 0.0) || !(std::isfinite(spacing_m) && spacing_m > 0.0) || !(std::isfinite(pixels_per_metre) && pixels_per_metre > 0.0))
            return;
        const auto origin = camera.world_to_screen({ 0.0, 0.0 });
        const auto finish = camera.world_to_screen({ length_m, 0.0 });
        if (!math::is_finite(origin) || !math::is_finite(finish))
            return;

        const auto& theme = settings.theme;
        const auto ds = static_cast<double>(pixel_scale(settings));
        const auto tps = static_cast<double>(text_pixel_scale(settings));
        const auto metrics = label_metrics(settings);
        const auto separator = std::round(5.0 * tps);
        const auto units = stage.units;
        auto focus = camera.focus_rect();
        if (focus.empty())
            focus = { 0.0, 0.0, static_cast<double>(camera.viewport().width), static_cast<double>(camera.viewport().height) };
        const auto x_at = [&](double metres)
        {
            return camera.world_to_screen({ metres, 0.0 }).x;
        };
        const auto rail_y = origin.y;
        const auto band = relativity_band_layout(focus.width, tps, units);
        auto geometry = tier_geometry(stage.tier);
        if (stage.tier == RelativityStageTier::compact)
        {
            // Height to spare between the band and the apparatus lets the compact faces grow, up to
            // the full tier's size: each pixel of radius lifts the probe's face by one, the race row
            // by two, the lab face by three and the top of the apparatus by four. The room is the
            // half of the spare height that framing leaves above the apparatus, worked out from the
            // framing area rather than from where the camera puts the rail, so panning or zooming
            // never resizes the faces.
            const auto stack = relativity_stack_px(stage.tier, tps, framed_rail_thickness_px(focus.width, focus.height, tps));
            const auto room = std::max(0.0, focus.height - band.height_px - stack.above_px - stack.below_px) / (2.0 * tps);
            const auto grow = std::clamp(room / 4.0, 0.0, full_geometry.face_radius - geometry.face_radius);
            geometry.face_radius += grow;
            geometry.probe_face_rise += grow;
            geometry.second_row += 2.0 * grow;
            geometry.lab_face_rise += 3.0 * grow;
            geometry.above += 4.0 * grow;
        }
        // In the minimal tier the probe's face is its marker, and the lab face stands on the footer
        // row beside its plate rather than on a post at the start line.
        const auto face_is_marker = geometry.marker_radius <= 0.0;
        const auto lab_on_footer = geometry.lab_face_rise <= 0.0;
        const auto start_x = origin.x, finish_x = finish.x;
        const auto lane_y = rail_y + lane_depth_px(geometry, tps, relativity_rail_thickness_m * pixels_per_metre);
        // β is kept in [0, 1) and the probe and its pulse on the track, with the pulse never behind.
        const auto speed_fraction = std::isfinite(stage.speed_fraction) ? std::clamp(stage.speed_fraction, 0.0, std::nextafter(1.0, 0.0)) : 0.0;
        const auto probe_m = std::clamp(finite_or(stage.probe_position_m, 0.0), 0.0, length_m);
        const auto pulse_m = std::clamp(probe_m + std::max(0.0, finite_or(stage.light_lead_m, 0.0)), probe_m, length_m);
        // Text reads "—" for a value that is absent rather than inventing one.
        constexpr auto absent = std::numeric_limits<double>::quiet_NaN();
        const auto shown_fraction = std::isfinite(stage.speed_fraction) && stage.speed_fraction >= 0.0 && stage.speed_fraction < 1.0 ? stage.speed_fraction : absent;

        RelativityStageLayout layout;
        layout.tier = stage.tier;
        layout.rail_y = rail_y;
        layout.track_left = start_x;
        layout.track_right = finish_x;
        layout.text_pixel_scale = tps;
        const auto previous_layer = list.layer();
        const auto reserve = [&](const ScreenRect& rect, bool plate)
        {
            const auto [minimum, maximum] = corners(rect);
            reserved_label_areas_.emplace_back(minimum, maximum);
            stage_instrument_areas_.emplace_back(minimum, maximum);
            if (plate)
                layout.fixed_plates.push_back(rect);
        };

        // The instrument band across the top of the framing area, above every scene layer.
        list.set_layer(instrument_layer);
        {
            layout.band = { focus.left, focus.top, focus.width, band.height_px };
            reserve(layout.band, false);
            const auto level = band.level;
            const auto gap = finite_or(stage.one_minus_speed_fraction, 1.0 - speed_fraction);
            struct Reading
            {
                std::string value, secondary;
                Color color;
                // A shorter form for a value too wide for its plate, as for a speed far below the
                // slowest the band is sized for; empty when the value is clipped instead.
                std::string fallback;
            };
            const auto significant_quantity = [&](double value_si, DisplayQuantity quantity)
            {
                const auto unit = core::display_unit(quantity, units);
                return core::format_significant(core::display_value(value_si, quantity, units)) + (unit.empty() ? "" : "\xC2\xA0" + std::string(unit));
            };
            std::array<Reading, 6> readings {};
            readings[0] = { core::format_quantity(stage.rest_mass_kg, DisplayQuantity::mass, units), {}, theme.label_text, {} };
            readings[1] = { core::format_quantity(stage.speed_m_s, DisplayQuantity::relativistic_speed, units), {}, theme.velocity, {} };
            readings[2] = { core::format_value(shown_fraction, DisplayQuantity::speed_fraction, units), {}, theme.velocity, {} };
            // Only far below half c can the speeds outgrow their plates; there a mantissa says the
            // same without reading anywhere near c.
            if (std::isfinite(shown_fraction) && shown_fraction < 0.5)
            {
                readings[1].fallback = significant_quantity(stage.speed_m_s, DisplayQuantity::relativistic_speed);
                readings[2].fallback = core::format_significant(shown_fraction);
            }
            if (level == 0 && speed_fraction >= 0.9 && std::isfinite(stage.below_light_m_s))
                readings[2].secondary = "\xC2\xB7 below c by " + core::format_quantity(stage.below_light_m_s, DisplayQuantity::speed_gap, units);
            readings[3] = { core::format_value(stage.lorentz_factor_minus_one, DisplayQuantity::lorentz_factor_excess, units), {}, theme.label_text, {} };
            readings[4] = { core::format_quantity(stage.kinetic_energy_j, DisplayQuantity::relativistic_energy, units), {}, theme.label_text, {} };
            readings[5] = { core::format_quantity(stage.momentum_kg_m_s, DisplayQuantity::relativistic_momentum, units), {}, theme.momentum, {} };

            for (std::size_t index = 0; index < band.plates.size(); ++index)
            {
                const auto& relative = band.plates[index];
                const ScreenRect rect { std::round(focus.left + relative.left), std::round(focus.top + relative.top), std::round(relative.width), relative.height };
                draw_plate(list, rect, metrics, theme, ds);
                layout.fixed_plates.push_back(rect);
                const auto y = text_baseline(rect.top, metrics);
                auto x = rect.left + metrics.pad_x;
                const auto name = band_name(index, level);
                if (!name.empty())
                {
                    list.add_text({ x, y }, name, theme.label_muted, metrics.scale);
                    x += text_width(name, metrics.scale) + text_width(" ", metrics.scale);
                }
                const auto inner_right = rect.left + rect.width - metrics.pad_x;
                if (index < readings.size())
                {
                    // The reading stands at the plate's right edge, so a column of plates reads like
                    // instruments; a remark goes first when it does not fit, then the long form.
                    auto reading = readings[index];
                    const auto block = [&]()
                    {
                        return text_width(reading.value, metrics.scale) + (reading.secondary.empty() ? 0.0 : separator + text_width(reading.secondary, metrics.scale));
                    };
                    const auto room = inner_right - x;
                    if (block() > room)
                        reading.secondary.clear();
                    if (block() > room && !reading.fallback.empty())
                        reading.value = reading.fallback;
                    const auto clipped = block() > room;
                    if (clipped)
                        list.push_clip({ static_cast<int>(std::floor(x)), static_cast<int>(std::floor(rect.top)), static_cast<int>(std::ceil(room)), static_cast<int>(std::ceil(rect.height)) });
                    const auto value_x = clipped ? x : inner_right - block();
                    list.add_text({ value_x, y }, reading.value, reading.color, metrics.scale);
                    if (!reading.secondary.empty())
                        list.add_text({ value_x + text_width(reading.value, metrics.scale) + separator, y }, reading.secondary, theme.label_muted, metrics.scale);
                    if (clipped)
                        list.pop_clip();
                }
                else if (index == 6)
                {
                    // How close to c: one cell per nine of β, filled in the speed's colour. At the
                    // maximum (seven nines, or a hair past them after rounding) the last cell stays
                    // empty, and c lies beyond the bar.
                    const auto nines = gap > 0.0 && gap <= 1.0 && std::isfinite(gap) ? std::min(-std::log10(gap), gauge_cells - 1.0) : 0.0;
                    if (!name.empty())
                        x += gauge_gap * tps - text_width(" ", metrics.scale);
                    const auto cell_top = std::round(rect.top + 0.5 * (rect.height - gauge_cell_height * tps));
                    for (int cell = 0; cell < gauge_cells; ++cell)
                    {
                        const auto cell_left = std::round(x + static_cast<double>(cell) * (gauge_cell_width + gauge_cell_gap) * tps);
                        const Vec2 minimum { cell_left, cell_top }, maximum { cell_left + gauge_cell_width * tps, cell_top + gauge_cell_height * tps };
                        list.add_rectangle_fill(minimum, maximum, theme.grid_major);
                        const auto fill = std::clamp(nines - static_cast<double>(cell), 0.0, 1.0);
                        if (fill > 0.0)
                            list.add_rectangle_fill(minimum, { minimum.x + fill * gauge_cell_width * tps, maximum.y }, theme.velocity);
                        list.add_rectangle_outline(minimum + Vec2 { 0.5 * ds, 0.5 * ds }, maximum - Vec2 { 0.5 * ds, 0.5 * ds }, theme.label_border, static_cast<float>(ds));
                    }
                    list.add_text({ std::round(x + (gauge_cells_width + gauge_gap) * tps), y }, "\xE2\x86\x92 c", theme.label_muted, metrics.scale);
                }
                else
                    list.add_text({ x, y }, slow_motion_text(stage.lab_seconds_per_screen_second, units, level), theme.label_muted, metrics.scale);
            }
        }

        // The track: the light lane under the rail, a mark wherever light is each nanosecond, the
        // start and finish lines, and the marks' numbers. Above the rail's own body, below the probe.
        list.set_layer(-6);
        const auto muted_line = theme.label_muted;
        const auto hairline = stroke(settings, 1.0f);
        list.add_dashed_line({ start_x, lane_y }, { finish_x, lane_y }, muted_line.with_alpha(muted_line.alpha * 0.45f), hairline, static_cast<float>(1.5 * ds), static_cast<float>(3.0 * ds));
        const auto marks = static_cast<int>(std::clamp(std::round(length_m / spacing_m), 1.0, 1000.0));
        for (int mark = 1; mark < marks; ++mark)
        {
            const auto x = x_at(static_cast<double>(mark) * spacing_m);
            list.add_line({ x, rail_y - mark_reach * tps }, { x, lane_y + mark_reach * tps }, muted_line, hairline, StrokeCap::butt);
        }
        // The footer row under the marks' numbers: the caption or the lab plate at the left, the
        // scale key at the right.
        const auto footer_top = std::round(lane_y + geometry.footer * tps);
        // The lab clock tops a post at the start line, or stands on the footer row left of its
        // plate, in the margin before the track. Faces sit on whole pixel rows, so their rims and
        // ticks stay crisp.
        const auto face_radius = geometry.face_radius * tps;
        const auto lab_centre = lab_on_footer ? Vec2 { std::round(start_x - metrics.pad_x - (geometry.face_radius + footer_gap) * tps), std::round(footer_top + 0.5 * metrics.height) } : Vec2 { start_x, std::round(rail_y - geometry.lab_face_rise * tps) };
        const auto start_top = lab_on_footer ? rail_y - finish_rise * tps : lab_centre.y + face_radius;
        list.add_line({ start_x, start_top }, { start_x, lane_y + mark_reach * tps }, muted_line, hairline, StrokeCap::butt);
        list.add_line({ finish_x, rail_y - finish_rise * tps }, { finish_x, lane_y + mark_reach * tps }, muted_line, hairline, StrokeCap::butt);
        const auto number_scale = metrics.scale * 11.0f / 12.0f;
        const auto number_size = static_cast<double>(font_pixels(number_scale));
        const auto numbers_y = lane_y + geometry.numbers * tps;
        const auto number_baseline = std::round(numbers_y + 0.5 * number_size * 1490.0 / 2048.0 - std::ceil(number_size * 1984.0 / 2048.0));
        double numbers_width = 0.0;
        for (int mark = 0; mark <= marks; ++mark)
        {
            const auto shown = stage.tier == RelativityStageTier::full || (stage.tier == RelativityStageTier::compact ? mark % 5 == 0 || mark == marks : mark == 0 || mark == marks);
            if (!shown)
                continue;
            const auto text = std::to_string(mark);
            const auto width = text_width(text, number_scale);
            numbers_width = std::max(numbers_width, width);
            list.add_text({ std::round(x_at(static_cast<double>(mark) * spacing_m) - 0.5 * width), number_baseline }, text, theme.label_muted, number_scale);
        }
        // The digits' ink ends half a cap height below their centre; the footer row starts a few
        // pixels lower, so the scale key standing there keeps clear of them.
        const auto numbers_bottom = numbers_y + 0.5 * number_size * 1490.0 / 2048.0 + 1.0 * tps;
        {
            // The track and its numbers are one instrument to the scale key; to plates they are an
            // obstacle the probe's clock keeps clear of.
            const ScreenRect track { start_x - 0.5 * numbers_width - 2.0 * tps, rail_y - finish_rise * tps, finish_x - start_x + numbers_width + 4.0 * tps, numbers_bottom - (rail_y - finish_rise * tps) };
            const auto [minimum, maximum] = corners(track);
            stage_instrument_areas_.emplace_back(minimum, maximum);
            const auto numbers_top = 2.0 * numbers_y - numbers_bottom;
            const std::array<Vec2, 4> row { Vec2 { track.left, numbers_top }, Vec2 { track.left + track.width, numbers_top }, Vec2 { track.left + track.width, numbers_bottom }, Vec2 { track.left, numbers_bottom } };
            add_obstacle(row.data(), row.size(), numbers_obstacle);
        }

        // The light pulse on its lane while it is on the track, the finish detector it fills when it
        // arrives, and the probe on the rail. The probe never runs ahead of its pulse.
        list.set_layer(0);
        const auto glow = settings.transitions && !theme.light_stage;
        const auto finished = stage.light_finished;
        const auto pulse_radius = geometry.pulse_radius * tps;
        if (!finished)
        {
            const Vec2 pulse { x_at(pulse_m), lane_y };
            if (glow)
            {
                const auto tail_start = std::max(start_x, pulse.x - pulse_tail_length * tps);
                if (pulse.x - tail_start > 1.0)
                {
                    auto mesh = acquire_mesh();
                    append_soft_segment(*mesh, { tail_start, lane_y }, pulse, 1.5 * tps, ds, theme.label_text.with_alpha(0.0f), theme.label_text.with_alpha(theme.label_text.alpha * 0.55f));
                    list.add_indexed_mesh(mesh);
                }
                list.add_circle_shadow(pulse, static_cast<float>(pulse_radius), theme.label_text.with_alpha(theme.label_text.alpha * 0.35f), static_cast<float>(pulse_glow_radius * tps - pulse_radius), {});
            }
            list.add_circle_fill(pulse, static_cast<float>(pulse_radius), theme.label_text);
            layout.pulse_center = pulse;
            layout.pulse_radius = pulse_radius;
            const auto outline = disc_outline(pulse, pulse_radius + 1.0 * tps);
            add_obstacle(outline.data(), outline.size(), pulse_obstacle);
        }
        {
            // The detector fills as the light arrives, fading in step with the rest of the stage.
            const auto target = finished ? 1.0 : 0.0;
            detector_fill_ = settings.transitions ? std::clamp(detector_fill_ + (target - detector_fill_) * presentation_fraction_, 0.0, 1.0) : target;
            const Vec2 half { 5.0 * tps, 7.0 * tps };
            const Vec2 centre { finish_x, lane_y };
            const auto radius = static_cast<float>(2.0 * tps);
            list.add_rounded_rectangle_fill(centre - half, centre + half, radius, theme.label_plate);
            if (detector_fill_ > 0.0)
                list.add_rounded_rectangle_fill(centre - half, centre + half, radius, theme.label_text.with_alpha(theme.label_text.alpha * 0.7f * static_cast<float>(detector_fill_)));
            list.add_rounded_rectangle_outline(centre - half, centre + half, radius, theme.label_text, hairline);
        }
        // The probe's marker on the rail, with a stem up to the face it carries; a face that is the
        // marker rides the rail itself.
        const Vec2 probe_face { x_at(probe_m), std::round(rail_y - geometry.probe_face_rise * tps) };
        const auto marker_radius = face_is_marker ? face_radius : geometry.marker_radius * tps;
        const auto marker = face_is_marker ? probe_face : Vec2 { probe_face.x, rail_y - geometry.marker_rise * tps };
        if (!face_is_marker)
        {
            list.add_line({ marker.x, marker.y - marker_radius }, { marker.x, probe_face.y + face_radius }, theme.velocity.with_alpha(theme.velocity.alpha * 0.6f), hairline, StrokeCap::butt);
            list.add_circle_fill(marker, static_cast<float>(marker_radius), theme.velocity);
            list.add_circle_outline(marker, static_cast<float>(marker_radius), theme.label_plate, stroke(settings, 1.5f));
        }
        layout.probe_center = marker;
        layout.probe_radius = marker_radius;
        stage_instrument_areas_.push_back(corners(square_around(marker, marker_radius)));

        // The clocks and the fixed plates that read them, beside the readings the label solver places.
        list.set_layer(20);
        const auto lab_time = stage.lab_time_s, proper_time = stage.proper_time_s, lag = stage.clock_lag_s;
        const auto fine_time = [&](double seconds)
        {
            return core::format_quantity(seconds, DisplayQuantity::fine_time, units);
        };
        const auto duration = [&](double seconds)
        {
            return core::format_quantity(seconds, DisplayQuantity::duration, units);
        };
        const auto row_top = [&](double rise)
        {
            return std::round(rail_y - rise * tps - 0.5 * metrics.height);
        };
        const auto plate_at = [&](double left, double top, const PlateText& text, bool edged = true)
        {
            const ScreenRect rect { std::round(left), top, plate_width(text, metrics, separator), metrics.height };
            draw_plate(list, rect, metrics, theme, ds, edged);
            draw_plate_text(list, rect, text, metrics, separator);
            reserve(rect, true);
            return rect;
        };
        // Both clocks, in every tier and at one size: the lab clock with a muted rim and a hand in
        // the lab's ink, the probe's in its own colour, with the hub in it too where the face is
        // the marker.
        draw_clock_face(list, lab_centre, face_radius, lab_time, stage.clock_period_s, { theme.label_muted.with_alpha(theme.label_muted.alpha * 0.45f), hairline, theme.label_text, theme.label_text }, settings, tps);
        draw_clock_face(list, probe_face, face_radius, proper_time, stage.clock_period_s, { theme.velocity, stroke(settings, 2.0f), theme.velocity, face_is_marker ? theme.velocity : theme.label_text }, settings, tps);
        const auto lab_face = square_around(lab_centre, face_radius);
        reserve(lab_face, false);
        if (!face_is_marker)
            stage_instrument_areas_.push_back(corners(square_around(probe_face, face_radius)));
        layout.lab_clock_center = lab_centre;
        layout.probe_clock_center = probe_face;
        layout.clock_radius = face_radius;
        std::array<ScreenRect, 3> lab_items { lab_face };
        std::size_t lab_item_count = 1;
        const PlateText lab_reading { "t " + fine_time(lab_time) + " \xC2\xB7 t \xE2\x88\x92 \xCF\x84 " + duration(lag), {}, theme.label_text, {} };
        if (stage.tier == RelativityStageTier::full)
        {
            // Two plates beside the face, centred on it as a pair.
            const auto lab_left = start_x + (geometry.face_radius + lab_plate_gap) * tps;
            lab_items[lab_item_count++] = plate_at(lab_left, row_top(geometry.lab_row), { "Lab clock " + fine_time(lab_time), {}, theme.label_text, {} });
            const auto agree = lag == 0.0;
            lab_items[lab_item_count++] = plate_at(lab_left, row_top(geometry.second_row), { agree ? std::string("Clocks agree") : "Probe clock behind by " + duration(lag), {}, theme.label_text, {} });
        }
        else
        {
            // On the footer row, its reading starting under the start line.
            lab_items[lab_item_count++] = plate_at(start_x - metrics.pad_x, footer_top, lab_reading);
        }

        if (stage.tier != RelativityStageTier::minimal)
        {
            // The race, read at the finish: the light's lead while its pulse is on the track, then
            // how much sooner it finished the last whole lap. Before the light has any lead (at
            // Ready, and as each lap's pulse leaves) the two are side by side at the start.
            const auto lead_m = std::max(0.0, finite_or(stage.light_lead_m, 0.0));
            std::string race = "Light and probe start together";
            if (finished)
                race = "Light finished first";
            else if (lead_m > 0.0)
                race = "Light ahead by " + core::format_quantity(lead_m, DisplayQuantity::small_length, units);
            PlateText now { std::move(race), {}, theme.label_text, theme.label_muted };
            std::optional<PlateText> last;
            if (stage.last_lap_margin_s)
                last = PlateText { "Last lap: light won by " + duration(*stage.last_lap_margin_s), stage.last_lap_speed_changed ? "\xC2\xB7 speed changed" : "", theme.label_text, theme.label_muted };
            const auto race_right = finish_x + race_plate_overhang * tps;
            const auto race_at = [&](PlateText text, double rise)
            {
                ScreenRect rect { 0.0, row_top(rise), plate_width(text, metrics, separator), metrics.height };
                rect.left = std::round(race_right - rect.width);
                // At the narrowest stage of a tier a long result could reach the lab clock or a lab
                // plate on its row: the remark goes first.
                for (std::size_t index = 0; index < lab_item_count && !text.secondary.empty(); ++index)
                    if (overlaps(rect, lab_items[index]))
                    {
                        text.secondary.clear();
                        rect.width = plate_width(text, metrics, separator);
                        rect.left = std::round(race_right - rect.width);
                    }
                plate_at(rect.left, rect.top, text);
            };
            if (stage.tier == RelativityStageTier::full)
            {
                race_at(now, geometry.lab_row);
                if (last)
                    race_at(*last, geometry.second_row);
            }
            else
                race_at(last ? *last : now, geometry.second_row);
        }

        if (stage.tier == RelativityStageTier::full)
        {
            // The caption states the marks' spacing in the stage's units, on the footer row the scale
            // key shares, in the key's borderless plate.
            const PlateText caption { "Marks " + core::format_quantity(spacing_m, DisplayQuantity::length, units) + " apart: light passes one each nanosecond", {}, theme.label_muted, {} };
            plate_at(start_x - metrics.pad_x, footer_top, caption, false);
        }

        // The probe's own clock reading rides with it, placed by the label solver beside its face;
        // the rate remark is dropped first when the stage is crowded.
        {
            LabelRequest label;
            label.key = probe_clock_key;
            label.priority = 120;
            label.required = true;
            label.essential = true;
            label.placement = LabelPlacement::around_box;
            const auto anchor = probe_face;
            const auto radius = face_radius;
            label.anchor = anchor;
            label.box.expand(anchor - Vec2 { radius, radius });
            label.box.expand(anchor + Vec2 { radius, radius });
            label.primary = (stage.tier == RelativityStageTier::minimal ? "\xCF\x84 " : "Probe clock ") + fine_time(proper_time);
            label.secondary = clock_rate_text(stage);
            // The rate remark rides along only while the plate fits beside the clock on one side or
            // the other; on a narrow stage the reading alone stays next to its clock.
            if (!label.secondary.empty())
            {
                const auto width = 2.0 * metrics.pad_x + text_width(label.primary, metrics.scale) + separator + text_width(label.secondary, metrics.scale);
                const auto room_right = focus.left + focus.width - (anchor.x + radius + metrics.gap);
                const auto room_left = anchor.x - radius - metrics.gap - focus.left;
                if (width > std::max(room_left, room_right))
                    label.secondary.clear();
            }
            label.primary_color = theme.label_text;
            label.secondary_color = theme.label_muted;
            const auto outline = disc_outline(anchor, radius + 1.0 * tps);
            add_obstacle(outline.data(), outline.size(), probe_clock_key);
            if (!face_is_marker)
            {
                const auto own_marker = disc_outline(marker, marker_radius + 1.0 * tps);
                add_obstacle(own_marker.data(), own_marker.size(), probe_clock_key);
            }
            queue_label(std::move(label));
        }

        list.set_layer(previous_layer);
        relativity_layout_ = std::move(layout);
    }
}
