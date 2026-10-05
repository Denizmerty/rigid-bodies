#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/render/font_atlas.hpp>
#include <rigidbodies/render/relativity_stage.hpp>
#include <rigidbodies/render/scene_renderer.hpp>

#include "relativity_stage_fixture.hpp"
#include "scene_style.hpp"
#include "test_framework.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using namespace rigidbodies;
    using render::DrawCommandKind;
    using render::RelativityStageTier;

    constexpr std::array<RelativityStageTier, 3> tiers { RelativityStageTier::full, RelativityStageTier::compact, RelativityStageTier::minimal };

    bool same_color(const render::Color& left, const render::Color& right)
    {
        return left.red == right.red && left.green == right.green && left.blue == right.blue && left.alpha == right.alpha;
    }

    render::SceneRenderSettings stage_settings(float display_scale = 1.0f, float text_scale = 1.0f, const char* theme = "workbench_dark")
    {
        render::SceneRenderSettings settings;
        settings.theme = render::theme_by_name(theme);
        settings.layers = render::LayerMask::none();
        for (const auto layer : { render::VisualizationLayer::grid, render::VisualizationLayer::bodies, render::VisualizationLayer::labels })
            settings.layers.set(layer, true);
        settings.display_scale = display_scale;
        settings.text_scale = text_scale;
        settings.transitions = false;
        return settings;
    }

    // A focus area that holds a tier: roomy for the full tier, and for the smaller tiers no wider
    // than the next tier allows and only just tall enough, the way a crowded stage leaves them.
    render::ScreenRect focus_for(RelativityStageTier tier, double tps, core::DisplayUnits units = core::DisplayUnits::si)
    {
        auto width = 575.0 * tps;
        if (tier == RelativityStageTier::full)
            width = 1000.0 * tps;
        else if (tier == RelativityStageTier::compact)
            width = 600.0 * tps;
        const auto band = render::relativity_band_height_px(width, tps, units);
        // Before the height is known: it is worked out below to fit the band and the stack.
        const auto pixels_per_metre = render::relativity_pixels_per_metre(width, std::numeric_limits<double>::infinity(), tps);
        const auto stack = render::relativity_stack_px(tier, tps, render::relativity_rail_thickness_m * pixels_per_metre);
        const auto height = tier == RelativityStageTier::full ? 600.0 * tps : std::ceil(band + stack.above_px + stack.below_px) + 1.0;
        return { 80.0, 50.0, width, height };
    }

    struct Drawn
    {
        render::ScreenRect focus;
        render::Camera2D camera;
        double tps {};
    };

    // Draws a stage in a focus area sized for its tier, framed as the session frames it.
    Drawn draw_stage(render::SceneRenderer& renderer, render::DrawList& list, const render::RelativityStage& stage, const render::SceneRenderSettings& settings, std::optional<render::ScreenRect> focus = std::nullopt)
    {
        Drawn drawn;
        drawn.tps = render::stage_text_pixel_scale(settings);
        drawn.focus = focus ? *focus : focus_for(stage.tier, drawn.tps, stage.units);
        const render::ViewportSize viewport { static_cast<int>(std::ceil(2.0 * drawn.focus.left + drawn.focus.width)), static_cast<int>(std::ceil(2.0 * drawn.focus.top + drawn.focus.height)) };
        drawn.camera = testing::relativity_camera(viewport, drawn.focus, drawn.tps, stage.tier, stage.units);
        renderer.set_relativity_stage(stage);
        physics::World world;
        renderer.render(world, drawn.camera, settings, list);
        return drawn;
    }

    std::string_view text_of(const render::DrawList& list, const render::DrawCommand& command)
    {
        return std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length);
    }

    render::ScreenRect bounds_of(const render::DrawList& list, const render::DrawCommand& command)
    {
        math::Vec2 minimum { 1.0e300, 1.0e300 }, maximum { -1.0e300, -1.0e300 };
        for (std::size_t index = 0; index < command.vertex_count; ++index)
        {
            minimum = math::min_components(minimum, list.vertices()[command.vertex_offset + index]);
            maximum = math::max_components(maximum, list.vertices()[command.vertex_offset + index]);
        }
        return { minimum.x, minimum.y, maximum.x - minimum.x, maximum.y - minimum.y };
    }

    // Every plate the frame drew over the scene: the band, the fixed plates, the label solver's
    // plates and the scale key.
    std::vector<render::ScreenRect> drawn_plates(const render::DrawList& list, const render::Theme& theme)
    {
        std::vector<render::ScreenRect> plates;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::rounded_rectangle_fill && (command.layer == 20 || command.layer == render::instrument_layer) && same_color(command.color, theme.label_plate))
                plates.push_back(bounds_of(list, command));
        return plates;
    }

    double overlap(const render::ScreenRect& a, const render::ScreenRect& b)
    {
        const auto width = std::min(a.left + a.width, b.left + b.width) - std::max(a.left, b.left);
        const auto height = std::min(a.top + a.height, b.top + b.height) - std::max(a.top, b.top);
        return width > 0.0 && height > 0.0 ? width * height : 0.0;
    }

    bool inside(const render::ScreenRect& rect, const render::ScreenRect& area, double tolerance = 1.0e-6)
    {
        return rect.left >= area.left - tolerance && rect.top >= area.top - tolerance && rect.left + rect.width <= area.left + area.width + tolerance && rect.top + rect.height <= area.top + area.height + tolerance;
    }

    std::vector<std::uint32_t> code_points(std::string_view text)
    {
        std::vector<std::uint32_t> codes;
        for (std::size_t index = 0; index < text.size();)
        {
            const auto first = static_cast<unsigned char>(text[index++]);
            std::uint32_t code = first;
            int extra = first >= 0xf0 ? 3 : first >= 0xe0 ? 2
                : first >= 0xc0                           ? 1
                                                          : 0;
            if (extra > 0)
                code = first & (extra == 3 ? 0x07u : extra == 2 ? 0x0fu
                                                                : 0x1fu);
            for (; extra > 0 && index < text.size(); --extra)
                code = (code << 6u) | (static_cast<unsigned char>(text[index++]) & 0x3fu);
            codes.push_back(code);
        }
        return codes;
    }

    // The scene atlas bakes Latin-1 and its own short list; anything else draws as "?".
    bool baked(std::uint32_t code)
    {
        const auto extra = render::font_atlas_extra_code_points();
        return (code >= 32 && code <= 255) || std::find(extra.begin(), extra.end(), code) != extra.end();
    }

    // The hand drawn from a clock's centre in a colour, as a clockwise angle from twelve o'clock.
    std::optional<double> hand_angle(const render::DrawList& list, const math::Vec2& centre, double radius, const render::Color& color)
    {
        for (const auto& command : list.commands())
        {
            if (command.kind != DrawCommandKind::line || !same_color(command.color, color) || command.vertex_count != 2)
                continue;
            const auto from = list.vertices()[command.vertex_offset], to = list.vertices()[command.vertex_offset + 1];
            if (math::length(from - centre) > 1.0e-9 || std::abs(math::length(to - from) - 0.78 * radius) > 1.0e-6)
                continue;
            auto angle = std::atan2(to.x - from.x, -(to.y - from.y));
            if (angle < 0.0)
                angle += math::two_pi;
            return angle;
        }
        return std::nullopt;
    }

    double angle_between(double a, double b)
    {
        const auto difference = std::fmod(std::abs(a - b), math::two_pi);
        return std::min(difference, math::two_pi - difference);
    }

    // A clock face is a plate-toned disc on the clocks' layer.
    std::size_t faces_at(const render::DrawList& list, const render::Theme& theme, const math::Vec2& centre, double radius)
    {
        std::size_t count = 0;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::circle_fill && command.layer == 20 && same_color(command.color, theme.label_plate) && math::length(list.vertices()[command.vertex_offset] - centre) <= 1.0e-9 && std::abs(static_cast<double>(command.radius) - radius) <= 1.0e-4)
                ++count;
        return count;
    }

    // How far a disc reaches into a rectangle: positive when they overlap.
    double disc_overlap(const math::Vec2& centre, double radius, const render::ScreenRect& rect)
    {
        const auto x = std::clamp(centre.x, rect.left, rect.left + rect.width);
        const auto y = std::clamp(centre.y, rect.top, rect.top + rect.height);
        return radius - math::length(centre - math::Vec2 { x, y });
    }

    // A focus area for a tier at a width in logical pixels: only just tall enough for the band and
    // the tier's apparatus, or with room to spare.
    render::ScreenRect focus_at(RelativityStageTier tier, double logical_width, double tps, bool roomy)
    {
        const auto width = logical_width * tps;
        const auto band = render::relativity_band_height_px(width, tps, core::DisplayUnits::si);
        // Before the height is known: it is worked out below to fit the band and the stack.
        const auto pixels_per_metre = render::relativity_pixels_per_metre(width, std::numeric_limits<double>::infinity(), tps);
        const auto stack = render::relativity_stack_px(tier, tps, render::relativity_rail_thickness_m * pixels_per_metre);
        return { 80.0, 50.0, width, std::ceil(band + stack.above_px + stack.below_px) + 1.0 + (roomy ? 240.0 * tps : 0.0) };
    }

    RIGIDBODIES_TEST("the stack keeps the light lane clear of the rail and shrinks with the tier")
    {
        for (const auto tps : { 1.0, 1.5, 2.8 })
        {
            const auto plate = render::detail::label_metrics_at(static_cast<float>(tps)).height;
            const auto full = render::relativity_stack_px(RelativityStageTier::full, tps, 4.0);
            const auto compact = render::relativity_stack_px(RelativityStageTier::compact, tps, 4.0);
            const auto minimal = render::relativity_stack_px(RelativityStageTier::minimal, tps, 4.0);
            RIGIDBODIES_EXPECT_NEAR(full.above_px, 144.0 * tps, 1.0e-12, "the full tier rises 144 logical pixels above the rail");
            // The 22 pixel lane, the marks' numbers and the footer row 22 below it, and the scale key's inset.
            RIGIDBODIES_EXPECT_NEAR(full.below_px, 22.0 * tps + 22.0 * tps + plate + 10.0 * tps, 1.0e-12, "a thin rail keeps the 22 pixel lane");
            RIGIDBODIES_EXPECT(full.above_px > compact.above_px && compact.above_px > minimal.above_px && full.below_px > compact.below_px && compact.below_px > minimal.below_px, "full needs more room than compact, compact more than minimal");
            for (const auto rail : { 4.0, 40.0 * tps })
            {
                const auto thick_full = render::relativity_stack_px(RelativityStageTier::full, tps, rail);
                const auto thick_compact = render::relativity_stack_px(RelativityStageTier::compact, tps, rail);
                const auto thick_minimal = render::relativity_stack_px(RelativityStageTier::minimal, tps, rail);
                RIGIDBODIES_EXPECT(thick_full.below_px > thick_compact.below_px && thick_compact.below_px > thick_minimal.below_px, "the tiers keep their order under a thick rail");
            }
            const auto thick = render::relativity_stack_px(RelativityStageTier::full, tps, 40.0 * tps);
            RIGIDBODIES_EXPECT_NEAR(thick.below_px, 48.0 * tps + 22.0 * tps + plate + 10.0 * tps, 1.0e-12, "a thick rail pushes the lane 8 pixels below it");
        }
        const auto odd = render::relativity_stack_px(RelativityStageTier::full, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity());
        RIGIDBODIES_EXPECT(std::isfinite(odd.above_px) && std::isfinite(odd.below_px), "non-finite inputs give a finite stack");
    }

    RIGIDBODIES_TEST("the band is whole rows of plates and grows as the framing narrows")
    {
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            for (const auto tps : { 1.0, 1.4, 2.0 })
            {
                // A house plate: the label font (12 logical pixels) plus 7 pixels of padding.
                const auto plate = std::round(static_cast<double>(std::lround(12.0 * tps)) + 7.0 * tps);
                double previous = 0.0;
                for (double width = 2400.0; width >= 200.0; width -= 50.0)
                {
                    const auto height = render::relativity_band_height_px(width, tps, units);
                    RIGIDBODIES_EXPECT(std::isfinite(height) && height > 0.0, "the band has a finite height");
                    RIGIDBODIES_EXPECT(height >= previous - 1.0e-9, "a narrower framing never needs a shorter band");
                    // 8 above, the rows with 4 between them, and 6 below.
                    const auto rows = (height - 14.0 * tps + 4.0 * tps) / (plate + 4.0 * tps);
                    RIGIDBODIES_EXPECT(rows >= 1.0 - 1.0e-9 && std::abs(rows - std::round(rows)) <= 1.0e-9, "the band is a whole number of plate rows");
                    previous = height;
                }
                RIGIDBODIES_EXPECT_NEAR(render::relativity_band_height_px(4000.0 * tps, tps, units), 8.0 * tps + plate + 6.0 * tps, 1.0e-9, "a very wide framing holds every plate in one row");
            }
        // Two rows of plates at a wide desktop stage: 8 + 2 plates + one gap + 6.
        const auto wide = render::relativity_band_height_px(1108.0, 1.0, core::DisplayUnits::si);
        RIGIDBODIES_EXPECT(wide <= 8.0 + 2.0 * 19.0 + 4.0 + 6.0 + 1.0e-9, "a 1108 pixel stage fits the band in two rows");
        RIGIDBODIES_EXPECT(std::isfinite(render::relativity_band_height_px(std::numeric_limits<double>::quiet_NaN(), 1.0, core::DisplayUnits::si)), "a non-finite width still gives a finite band");
    }

    RIGIDBODIES_TEST("the band sizes v/c for every speed from a billionth of c to the maximum")
    {
        // The band's v/c plate is sized for 0.00000000123, the widest text from 10⁻⁹ c up. Every digit
        // is tabular, so no v/c text in that range may have more characters.
        const auto widest = core::format_value(1.23e-9, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
        RIGIDBODIES_EXPECT(widest == "0.00000000123", "the sizing text is three figures in the slowest decade");
        const auto fits = [&](double fraction)
        {
            return core::format_value(fraction, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si).size() <= widest.size();
        };
        for (const auto fraction : { 0.0, 1.0e-9, 9.99e-9, 3.3e-7, 0.00123, 0.4999, 0.5, 0.99999985, physics::maximum_speed_fraction })
            RIGIDBODIES_EXPECT(fits(fraction), "v/c fits its plate at " + std::to_string(fraction));
        for (double fraction = 1.0e-9; fraction < 0.5; fraction *= 1.0137)
            RIGIDBODIES_EXPECT(fits(fraction), "v/c fits its plate below half c");
        for (double gap = 0.5; gap > 1.0 - physics::maximum_speed_fraction; gap /= 1.0137)
            RIGIDBODIES_EXPECT(fits(1.0 - gap), "v/c fits its plate above half c");
    }

    RIGIDBODIES_TEST("tiers follow the stage size with hysteresis")
    {
        const auto si = core::DisplayUnits::si;
        const auto tier = [&](double width, double height, RelativityStageTier previous, double tps = 1.0)
        {
            return render::choose_relativity_tier(width, height, tps, si, previous);
        };
        RIGIDBODIES_EXPECT(tier(1108.0, 732.0, RelativityStageTier::minimal) == RelativityStageTier::full, "a desktop stage takes the full tier");
        RIGIDBODIES_EXPECT(tier(690.0, 732.0, RelativityStageTier::full) == RelativityStageTier::full, "a full stage stays full until it is 16 pixels below 700");
        RIGIDBODIES_EXPECT(tier(690.0, 732.0, RelativityStageTier::compact) == RelativityStageTier::compact, "a compact stage needs 700 pixels to become full");
        RIGIDBODIES_EXPECT(tier(680.0, 732.0, RelativityStageTier::full) == RelativityStageTier::compact, "below 684 pixels the full tier gives way");
        RIGIDBODIES_EXPECT(tier(410.0, 732.0, RelativityStageTier::compact) == RelativityStageTier::compact && tier(410.0, 732.0, RelativityStageTier::minimal) == RelativityStageTier::minimal, "the compact tier has the same hysteresis at 420");
        RIGIDBODIES_EXPECT(tier(400.0, 732.0, RelativityStageTier::compact) == RelativityStageTier::minimal, "below 404 pixels only the minimal tier fits");
        RIGIDBODIES_EXPECT(tier(1108.0, 120.0, RelativityStageTier::full) == RelativityStageTier::minimal, "a short stage takes the minimal tier");
        RIGIDBODIES_EXPECT(tier(1108.0, 640.0, RelativityStageTier::minimal, 2.0) == RelativityStageTier::compact, "larger text needs a smaller tier sooner");
        RIGIDBODIES_EXPECT(tier(std::numeric_limits<double>::quiet_NaN(), 700.0, RelativityStageTier::full) == RelativityStageTier::minimal, "a non-finite stage takes the minimal tier");
        for (const auto previous : tiers)
            for (double height = 100.0; height <= 900.0; height += 25.0)
            {
                const auto band = render::relativity_band_height_px(1108.0, 1.0, si);
                const auto chosen = tier(1108.0, height, previous);
                const auto stack = render::relativity_stack_px(chosen, 1.0, render::relativity_rail_thickness_m * render::relativity_pixels_per_metre(1108.0, height, 1.0));
                if (chosen != RelativityStageTier::minimal)
                    RIGIDBODIES_EXPECT(height >= band + stack.above_px + stack.below_px, "a chosen tier always fits its band and stack");
            }
    }

    RIGIDBODIES_TEST("tiers keep 16 pixels of height hysteresis")
    {
        const auto si = core::DisplayUnits::si;
        for (const auto tps : { 1.0, 1.5 })
        {
            // Wide enough for the full tier, so only the height decides.
            const auto width = 1108.0 * tps;
            const auto needed = [&](RelativityStageTier tier, double height)
            {
                const auto rail = render::relativity_rail_thickness_m * render::relativity_pixels_per_metre(width, height, tps);
                const auto stack = render::relativity_stack_px(tier, tps, rail);
                return render::relativity_band_height_px(width, tps, si) + stack.above_px + stack.below_px;
            };
            const auto tier = [&](double height, RelativityStageTier previous)
            {
                return render::choose_relativity_tier(width, height, tps, si, previous);
            };
            // The rail, and so the need, hardly depends on the height; evaluate it near the need.
            const auto full = needed(RelativityStageTier::full, needed(RelativityStageTier::full, 1000.0));
            const auto compact = needed(RelativityStageTier::compact, needed(RelativityStageTier::compact, 1000.0));
            RIGIDBODIES_EXPECT(full > compact + 32.0 * tps, "the full tier needs more height than the compact tier and both margins");
            RIGIDBODIES_EXPECT(tier(full + 24.0 * tps, RelativityStageTier::compact) == RelativityStageTier::full, "16 pixels to spare promote a compact stage to full");
            RIGIDBODIES_EXPECT(tier(full + 8.0 * tps, RelativityStageTier::full) == RelativityStageTier::full, "a full stage that still fits stays full");
            RIGIDBODIES_EXPECT(tier(full + 8.0 * tps, RelativityStageTier::compact) == RelativityStageTier::compact, "a compact stage is not promoted without 16 pixels to spare");
            RIGIDBODIES_EXPECT(tier(full + 8.0 * tps, RelativityStageTier::minimal) == RelativityStageTier::compact, "a minimal stage with room for compact and its margin becomes compact");
            RIGIDBODIES_EXPECT(tier(full - 1.0 * tps, RelativityStageTier::full) == RelativityStageTier::compact, "a full stage that no longer fits gives way");
            RIGIDBODIES_EXPECT(tier(compact + 24.0 * tps, RelativityStageTier::minimal) == RelativityStageTier::compact, "16 pixels to spare promote a minimal stage to compact");
            RIGIDBODIES_EXPECT(tier(compact + 8.0 * tps, RelativityStageTier::compact) == RelativityStageTier::compact, "a compact stage that still fits stays compact");
            RIGIDBODIES_EXPECT(tier(compact + 8.0 * tps, RelativityStageTier::full) == RelativityStageTier::compact, "a full stage with room only for compact keeps compact");
            RIGIDBODIES_EXPECT(tier(compact + 8.0 * tps, RelativityStageTier::minimal) == RelativityStageTier::minimal, "a minimal stage is not promoted without 16 pixels to spare");
            RIGIDBODIES_EXPECT(tier(compact - 1.0 * tps, RelativityStageTier::compact) == RelativityStageTier::minimal, "a compact stage that no longer fits gives way");
        }
    }

    RIGIDBODIES_TEST("the minimal tier and its band fit a 720p projector stage at 150 % text")
    {
        // Present at 1280 x 720 with 150 % text leaves a stage 1208 x 300 pixels at a text pixel
        // scale of 2.1 under the caption: the narrowest and shortest stage a lesson is checked in.
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
        {
            const auto width = 1208.0, height = 300.0, tps = 2.1;
            const auto pixels_per_metre = render::relativity_pixels_per_metre(width, height, tps);
            const auto stack = render::relativity_stack_px(RelativityStageTier::minimal, tps, render::relativity_rail_thickness_m * pixels_per_metre);
            RIGIDBODIES_EXPECT(render::relativity_band_height_px(width, tps, units) + stack.above_px + stack.below_px <= height, "the band and the minimal apparatus fit the projector's stage");
            RIGIDBODIES_EXPECT(render::choose_relativity_tier(width, height, tps, units, RelativityStageTier::full) == RelativityStageTier::minimal, "that stage takes the minimal tier");
        }
    }

    RIGIDBODIES_TEST("a stage that holds still settles on the largest tier that fits")
    {
        const auto si = core::DisplayUnits::si;
        // A tier fits a stage at least as wide as the tier is kept down to (684 logical pixels for
        // the full tier, 404 for the compact one) and tall enough for the band and its apparatus,
        // with the rail as framing draws it.
        const auto keep_width = [](RelativityStageTier tier)
        {
            return tier == RelativityStageTier::full ? 684.0 : 404.0;
        };
        std::set<RelativityStageTier> settled;
        for (const auto tps : { 1.0, 1.5, 2.1 })
            for (const auto logical_width : { 300.0, 410.0, 600.0, 690.0, 1108.0 })
                for (double logical_height = 60.0; logical_height <= 420.0; logical_height += 4.0)
                {
                    const auto width = logical_width * tps, height = logical_height * tps;
                    const auto band = render::relativity_band_height_px(width, tps, si);
                    const auto rail = render::relativity_rail_thickness_m * render::relativity_pixels_per_metre(width, height, tps);
                    const auto fits = [&](RelativityStageTier tier)
                    {
                        const auto stack = render::relativity_stack_px(tier, tps, rail);
                        return logical_width >= keep_width(tier) && band + stack.above_px + stack.below_px <= height;
                    };
                    const auto fitting = render::fitting_relativity_tier(width, height, tps, si);
                    settled.insert(fitting);
                    const auto when = " at " + std::to_string(static_cast<int>(logical_width)) + " x " + std::to_string(static_cast<int>(logical_height)) + " logical px, text pixel scale " + std::to_string(tps);
                    if (fitting != RelativityStageTier::minimal)
                        RIGIDBODIES_EXPECT(fits(fitting), "the tier a still stage settles on is wide enough and holds the band and its apparatus" + when);
                    for (const auto larger : tiers)
                        if (static_cast<int>(larger) < static_cast<int>(fitting))
                            RIGIDBODIES_EXPECT(!fits(larger), "no larger tier fits the stage" + when);
                    RIGIDBODIES_EXPECT(render::choose_relativity_tier(width, height, tps, si, fitting) == fitting, "hysteresis keeps the tier a still stage settled on" + when);
                    // A smaller tier left over from a passing size may stay, but never gives more.
                    for (const auto previous : { RelativityStageTier::compact, RelativityStageTier::minimal })
                        RIGIDBODIES_EXPECT(static_cast<int>(render::choose_relativity_tier(width, height, tps, si, previous)) >= static_cast<int>(fitting), "a tier left over from before never gives a still stage more than the largest that fits" + when);
                }
        RIGIDBODIES_EXPECT(settled.size() == tiers.size(), "the sizes tried settle on every tier");
    }

    RIGIDBODIES_TEST("every tier draws both clock faces with their hands, larger in larger tiers")
    {
        for (const auto display_scale : { 1.0f, 1.5f, 2.0f })
        {
            const auto settings = stage_settings(display_scale);
            std::array<double, 3> radii {};
            for (std::size_t index = 0; index < tiers.size(); ++index)
                for (const auto speed : { 0.0, 0.6, 0.99999 })
                {
                    const auto tier = tiers[index];
                    render::SceneRenderer renderer;
                    render::DrawList list;
                    const auto stage = testing::relativity_stage_at(speed, 7.3e-9, tier);
                    const auto drawn = draw_stage(renderer, list, stage, settings);
                    const auto& layout = *renderer.relativity_stage_layout();
                    const auto when = " in tier " + std::to_string(static_cast<int>(tier)) + " at display scale " + std::to_string(display_scale) + ", " + std::to_string(speed) + " c";
                    const auto radius = layout.clock_radius;
                    radii[index] = radius / drawn.tps;
                    RIGIDBODIES_EXPECT(radius > 0.0 && faces_at(list, settings.theme, layout.lab_clock_center, radius) == 1 && faces_at(list, settings.theme, layout.probe_clock_center, radius) == 1, "the lab clock and the probe's clock are drawn as faces of one size" + when);
                    // Faces sit on whole pixel rows, so rims and ticks are drawn alike every frame.
                    RIGIDBODIES_EXPECT(layout.lab_clock_center.y == std::round(layout.lab_clock_center.y) && layout.probe_clock_center.y == std::round(layout.probe_clock_center.y), "the faces are centred on whole pixel rows" + when);
                    const auto lab = hand_angle(list, layout.lab_clock_center, radius, settings.theme.label_text);
                    const auto probe = hand_angle(list, layout.probe_clock_center, radius, settings.theme.velocity);
                    const auto expected = [](double seconds)
                    {
                        return math::two_pi * std::fmod(seconds, 1.0e-9) / 1.0e-9;
                    };
                    RIGIDBODIES_EXPECT(lab && angle_between(*lab, expected(stage.lab_time_s)) < 1.0e-6, "the lab clock's hand shows lab time" + when);
                    RIGIDBODIES_EXPECT(probe && angle_between(*probe, expected(stage.proper_time_s)) < 1.0e-6, "the probe clock's hand shows its own time" + when);
                    // The probe rides the rail with its clock: the clock is the marker in the minimal
                    // tier, and above it on a stem otherwise.
                    RIGIDBODIES_EXPECT(layout.probe_clock_center.x == layout.probe_center.x && layout.probe_clock_center.y <= layout.probe_center.y && layout.probe_center.y < layout.rail_y, "the probe carries its clock along the rail" + when);
                    // Both faces stay inside the focus area, below the band.
                    for (const auto& centre : { layout.lab_clock_center, layout.probe_clock_center })
                        RIGIDBODIES_EXPECT(inside({ centre.x - radius, centre.y - radius, 2.0 * radius, 2.0 * radius }, drawn.focus, 0.5) && centre.y - radius >= layout.band.top + layout.band.height, "both faces are on the stage, below the band" + when);
                }
            RIGIDBODIES_EXPECT(radii[0] >= 26.0 && radii[0] > radii[1] && radii[1] > radii[2] && radii[2] >= 10.0, "full faces are at least 26 logical pixels in radius, compact faces smaller and minimal faces smallest, never under 10");
        }
    }

    RIGIDBODIES_TEST("compact faces grow into spare height up to the full tier's size")
    {
        for (const auto display_scale : { 1.0f, 1.5f })
        {
            const auto settings = stage_settings(display_scale);
            const auto tps = render::stage_text_pixel_scale(settings);
            const auto radius_at = [&](RelativityStageTier tier, double extra_logical)
            {
                auto focus = focus_at(tier, 600.0, tps, false);
                focus.height += extra_logical * tps;
                render::SceneRenderer renderer;
                render::DrawList list;
                draw_stage(renderer, list, testing::relativity_stage_at(0.9, 3.0e-9, tier), settings, focus);
                return renderer.relativity_stage_layout()->clock_radius / tps;
            };
            const auto tight = radius_at(RelativityStageTier::compact, 0.0);
            const auto some = radius_at(RelativityStageTier::compact, 40.0);
            const auto plenty = radius_at(RelativityStageTier::compact, 400.0);
            RIGIDBODIES_EXPECT(tight < some && some < plenty, "a compact stage with more height to spare has larger faces");
            RIGIDBODIES_EXPECT_NEAR(plenty, radius_at(RelativityStageTier::full, 400.0), 1.0e-9, "they grow no larger than the full tier's");
            RIGIDBODIES_EXPECT_NEAR(radius_at(RelativityStageTier::full, 0.0), radius_at(RelativityStageTier::full, 400.0), 1.0e-9, "full faces keep their size");
            RIGIDBODIES_EXPECT_NEAR(radius_at(RelativityStageTier::minimal, 0.0), radius_at(RelativityStageTier::minimal, 400.0), 1.0e-9, "minimal faces keep their size");
        }
    }

    RIGIDBODIES_TEST("panning or zooming a compact stage leaves its faces and plates where they were on it")
    {
        for (const auto display_scale : { 1.0f, 1.5f })
        {
            const auto settings = stage_settings(display_scale);
            const auto tps = render::stage_text_pixel_scale(settings);
            // Room for faces part way to the full tier's, so any change in the room would show.
            auto focus = focus_at(RelativityStageTier::compact, 600.0, tps, false);
            focus.height += 40.0 * tps;
            const auto stage = testing::relativity_stage_at(0.9, 3.0e-9, RelativityStageTier::compact);
            render::SceneRenderer renderer;
            render::DrawList list;
            const auto drawn = draw_stage(renderer, list, stage, settings, focus);
            const auto framed = *renderer.relativity_stage_layout();
            RIGIDBODIES_EXPECT(framed.clock_radius > 16.0 * tps && framed.clock_radius < 27.0 * tps, "the framed faces have grown part of the way");
            const auto race_rise = [&](const render::RelativityStageLayout& layout)
            {
                // The race plate is the lowest fixed plate that ends past the finish line; band plates
                // that reach as far stand above it, and a pan may slide it under the band.
                auto rise = std::numeric_limits<double>::quiet_NaN();
                for (const auto& plate : layout.fixed_plates)
                    if (plate.left + plate.width > layout.track_right && !(layout.rail_y - plate.top >= rise))
                        rise = layout.rail_y - plate.top;
                return rise;
            };
            RIGIDBODIES_EXPECT(std::isfinite(race_rise(framed)), "the compact stage draws its race plate");

            for (int move = 0; move < 4; ++move)
            {
                auto camera = drawn.camera;
                const auto origin = camera.world_to_screen({ 0.0, 0.0 });
                if (move == 0)
                    camera.pan_by_screen_delta({ 37.0, 23.0 });
                else if (move == 1)
                    camera.pan_by_screen_delta({ -29.0, -31.0 });
                else
                    camera.zoom_about_screen_point(move == 2 ? 0.8 : 1.25, origin);
                render::DrawList moved_list;
                physics::World world;
                renderer.render(world, camera, settings, moved_list);
                const auto& moved = *renderer.relativity_stage_layout();
                const auto when = " after move " + std::to_string(move) + " at display scale " + std::to_string(display_scale);
                RIGIDBODIES_EXPECT(moved.rail_y != framed.rail_y || moved.track_right != framed.track_right, "the camera moved the rail" + when);
                RIGIDBODIES_EXPECT_NEAR(moved.clock_radius, framed.clock_radius, 1.0e-9, "the faces keep their size" + when);
                // Faces and plates sit on whole pixel rows, so they may move by a rounding pixel.
                RIGIDBODIES_EXPECT_NEAR(moved.rail_y - moved.lab_clock_center.y, framed.rail_y - framed.lab_clock_center.y, 1.0, "the lab face keeps its height above the rail" + when);
                RIGIDBODIES_EXPECT_NEAR(moved.rail_y - moved.probe_clock_center.y, framed.rail_y - framed.probe_clock_center.y, 1.0, "the probe's face keeps its height above the rail" + when);
                RIGIDBODIES_EXPECT_NEAR(race_rise(moved), race_rise(framed), 1.0, "the race plate keeps its height above the rail" + when);
            }
        }
    }

    RIGIDBODIES_TEST("plates keep clear of both faces while the probe runs under them")
    {
        struct Width
        {
            RelativityStageTier tier;
            double logical;
            bool roomy;
        };
        // Each tier at the narrowest width it is kept down to (for the minimal tier, the stage of the
        // smallest window in Present) and at a roomier one, only just tall enough, and compact
        // stages whose faces have grown into spare height.
        const std::array<Width, 8> widths { Width { RelativityStageTier::full, 684.0, false }, { RelativityStageTier::full, 1000.0, false }, { RelativityStageTier::compact, 404.0, false }, { RelativityStageTier::compact, 404.0, true }, { RelativityStageTier::compact, 600.0, false }, { RelativityStageTier::compact, 600.0, true }, { RelativityStageTier::minimal, 380.0, false }, { RelativityStageTier::minimal, 575.0, false } };
        for (const auto& [display_scale, text_scale] : { std::pair { 1.0f, 1.0f }, { 1.5f, 1.0f }, { 1.5f, 1.4f }, { 2.0f, 1.0f } })
            for (const auto& width : widths)
                for (const auto speed : { 0.5, 0.99999 })
                    for (double time = 0.0; time <= 21.0e-9; time += 0.7e-9)
                    {
                        const auto settings = stage_settings(display_scale, text_scale);
                        const auto tps = render::stage_text_pixel_scale(settings);
                        const auto focus = focus_at(width.tier, width.logical, tps, width.roomy);
                        auto stage = testing::relativity_stage_at(speed, time, width.tier);
                        stage.last_lap_speed_changed = stage.last_lap_margin_s.has_value();
                        render::SceneRenderer renderer;
                        render::DrawList list;
                        draw_stage(renderer, list, stage, settings, focus);
                        const auto& layout = *renderer.relativity_stage_layout();
                        const auto when = " in tier " + std::to_string(static_cast<int>(width.tier)) + " " + std::to_string(static_cast<int>(width.logical)) + (width.roomy ? " px across with room to spare" : " px across") + " at display scale " + std::to_string(display_scale) + ", text " + std::to_string(text_scale) + ", " + std::to_string(speed) + " c, " + std::to_string(time * 1.0e9) + " ns";
                        RIGIDBODIES_EXPECT(render::choose_relativity_tier(focus.width, focus.height, tps, stage.units, width.tier) == width.tier, "the focus area holds the tier" + when);
                        const auto radius = layout.clock_radius;
                        const auto plates = drawn_plates(list, settings.theme);
                        RIGIDBODIES_EXPECT(plates.size() >= 11, "the stage shows its plates" + when);
                        for (std::size_t a = 0; a < plates.size(); ++a)
                        {
                            RIGIDBODIES_EXPECT(inside(plates[a], focus, 0.5), "every plate stays inside the focus" + when);
                            for (const auto& centre : { layout.lab_clock_center, layout.probe_clock_center })
                                RIGIDBODIES_EXPECT(disc_overlap(centre, radius, plates[a]) <= 0.5, "no plate lies over a clock face" + when);
                            for (std::size_t b = a + 1; b < plates.size(); ++b)
                                RIGIDBODIES_EXPECT(overlap(plates[a], plates[b]) == 0.0, "no plate lies over another" + when);
                        }
                        RIGIDBODIES_EXPECT(math::length(layout.lab_clock_center - layout.probe_clock_center) >= 2.0 * radius + 2.0 * tps, "the probe's face passes the lab face with room between them" + when);
                    }
    }

    RIGIDBODIES_TEST("the renderer keeps the stage it is given and clears it for a Newtonian experiment")
    {
        render::SceneRenderer renderer;
        RIGIDBODIES_EXPECT(!renderer.relativity_stage() && !renderer.relativity_stage_layout(), "a new renderer has no apparatus");
        render::RelativityStage stage;
        stage.speed_fraction = 0.9;
        stage.lab_time_s = 3.0e-9;
        renderer.set_relativity_stage(stage);
        RIGIDBODIES_EXPECT(renderer.relativity_stage() && renderer.relativity_stage()->speed_fraction == 0.9 && renderer.relativity_stage()->lab_time_s == 3.0e-9, "the stage is stored as given");
        renderer.set_relativity_stage(std::nullopt);
        RIGIDBODIES_EXPECT(!renderer.relativity_stage() && !renderer.relativity_stage_layout(), "nothing survives a Newtonian experiment");
    }

    RIGIDBODIES_TEST("the stage draws finite geometry at every speed")
    {
        const auto finite_list = [](const render::DrawList& list)
        {
            for (const auto& point : list.vertices())
                if (!math::is_finite(point))
                    return false;
            for (const auto& command : list.commands())
                if (command.mesh)
                    for (const auto& vertex : command.mesh->vertices)
                        if (!math::is_finite(vertex.position))
                            return false;
            return true;
        };
        for (const auto tier : tiers)
            for (const auto speed : { 0.0, 1.0e-9, 0.5, 0.99, 0.99999, physics::maximum_speed_fraction })
                for (const auto time : { 0.0, 4.2e-9, 31.7e-9 })
                {
                    render::SceneRenderer renderer;
                    render::DrawList list;
                    draw_stage(renderer, list, testing::relativity_stage_at(speed, time, tier), stage_settings());
                    const auto& layout = renderer.relativity_stage_layout();
                    RIGIDBODIES_EXPECT(layout && layout->tier == tier && finite_list(list), "every vertex the stage draws is finite at " + std::to_string(speed) + " c");
                    RIGIDBODIES_EXPECT(layout->probe_center.x >= layout->track_left - 1.0e-9 && layout->probe_center.x <= layout->track_right + 1.0e-9 && layout->probe_center.y < layout->rail_y, "the probe rides on the track");
                    RIGIDBODIES_EXPECT(layout->clock_radius > 0.0, "every tier draws the clock faces");
                }

        // A stage with no usable track draws nothing at all.
        for (const auto broken : { 0, 1 })
        {
            auto stage = testing::relativity_stage_at(0.5, 3.0e-9);
            (broken == 0 ? stage.track_length_m : stage.mark_spacing_m) = std::numeric_limits<double>::quiet_NaN();
            render::SceneRenderer renderer;
            render::DrawList list;
            draw_stage(renderer, list, stage, stage_settings());
            RIGIDBODIES_EXPECT(!renderer.relativity_stage_layout() && finite_list(list), "a stage without a track draws no apparatus");
            for (const auto& command : list.commands())
                RIGIDBODIES_EXPECT(command.kind != DrawCommandKind::text || text_of(list, command).find("clock") == std::string_view::npos, "and no clock reading");
        }

        // Absent readings draw nothing of their own: no hand, no pulse, "—" for a value.
        auto stage = testing::relativity_stage_at(0.5, 3.0e-9);
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        for (auto* field : { &stage.lab_time_s, &stage.proper_time_s, &stage.clock_lag_s, &stage.speed_fraction, &stage.one_minus_speed_fraction, &stage.lorentz_factor_minus_one, &stage.inverse_lorentz_factor, &stage.clock_lag_rate, &stage.speed_m_s, &stage.below_light_m_s, &stage.kinetic_energy_j, &stage.momentum_kg_m_s, &stage.probe_position_m, &stage.light_lead_m, &stage.rest_mass_kg, &stage.lab_seconds_per_screen_second })
            *field = nan;
        const auto settings = stage_settings();
        render::SceneRenderer renderer;
        render::DrawList list;
        draw_stage(renderer, list, stage, settings);
        const auto& layout = renderer.relativity_stage_layout();
        RIGIDBODIES_EXPECT(layout && finite_list(list), "absent readings still give finite geometry");
        RIGIDBODIES_EXPECT(!hand_angle(list, layout->lab_clock_center, layout->clock_radius, settings.theme.label_text) && !hand_angle(list, layout->probe_clock_center, layout->clock_radius, settings.theme.velocity), "a clock without a reading has no hand");
        RIGIDBODIES_EXPECT(layout->probe_center.x == layout->track_left, "a probe without a position stands at the start");
        bool dash = false;
        for (const auto& command : list.commands())
            dash = dash || (command.kind == DrawCommandKind::text && text_of(list, command) == "Lab clock \xE2\x80\x94");
        RIGIDBODIES_EXPECT(dash, "an absent lab time reads as a dash");
    }

    RIGIDBODIES_TEST("the probe is never drawn ahead of the light pulse")
    {
        const auto settings = stage_settings();
        for (const auto speed : { 0.0, 0.3, 0.5, 0.9, 0.99999, physics::maximum_speed_fraction })
            for (double time = 0.0; time < 32.0e-9; time += 0.73e-9)
            {
                render::SceneRenderer renderer;
                render::DrawList list;
                draw_stage(renderer, list, testing::relativity_stage_at(speed, time), settings);
                const auto& layout = *renderer.relativity_stage_layout();
                if (layout.pulse_radius > 0.0)
                    RIGIDBODIES_EXPECT(layout.pulse_center.x >= layout.probe_center.x - 1.0e-9 && layout.pulse_center.x <= layout.track_right + 1.0e-9, "the pulse is level with the probe or ahead of it, and on the track");
            }
        // Leads the physics never produces are still drawn behind the finish and never behind the probe.
        for (const auto lead : { -1.0, 0.0, 1.0e-12, 2.0, 50.0 })
        {
            auto stage = testing::relativity_stage_at(0.9, 4.0e-9);
            stage.light_lead_m = lead;
            render::SceneRenderer renderer;
            render::DrawList list;
            draw_stage(renderer, list, stage, settings);
            const auto& layout = *renderer.relativity_stage_layout();
            RIGIDBODIES_EXPECT(layout.pulse_radius > 0.0 && layout.pulse_center.x >= layout.probe_center.x - 1.0e-9 && layout.pulse_center.x <= layout.track_right + 1.0e-9, "an odd lead keeps the pulse between the probe and the finish");
        }
    }

    RIGIDBODIES_TEST("clock hands show lab and probe time")
    {
        const auto settings = stage_settings();
        for (const auto speed : { 0.0, 0.6, 0.99999 })
            for (const auto time : { 0.0, 0.25e-9, 3.7e-9, 12.45e-9, 1234.56e-9 })
            {
                render::SceneRenderer renderer;
                render::DrawList list;
                const auto stage = testing::relativity_stage_at(speed, time);
                draw_stage(renderer, list, stage, settings);
                const auto& layout = *renderer.relativity_stage_layout();
                const auto expected = [](double seconds)
                {
                    return math::two_pi * std::fmod(seconds, 1.0e-9) / 1.0e-9;
                };
                const auto lab = hand_angle(list, layout.lab_clock_center, layout.clock_radius, settings.theme.label_text);
                const auto probe = hand_angle(list, layout.probe_clock_center, layout.clock_radius, settings.theme.velocity);
                RIGIDBODIES_EXPECT(lab && angle_between(*lab, expected(stage.lab_time_s)) < 1.0e-6, "the lab hand turns once per lab nanosecond");
                RIGIDBODIES_EXPECT(probe && angle_between(*probe, expected(stage.proper_time_s)) < 1.0e-6, "the probe hand turns once per nanosecond of its own time");
            }
    }

    RIGIDBODIES_TEST("the race starts side by side and then reports the light's lead")
    {
        // At Ready the pulse and the probe share the start line, so there is no lead to report yet.
        const auto race_texts = [](const render::RelativityStage& stage)
        {
            render::SceneRenderer renderer;
            render::DrawList list;
            draw_stage(renderer, list, stage, stage_settings());
            std::vector<std::string> texts;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::text)
                    if (const auto text = text_of(list, command); text.rfind("Light ", 0) == 0)
                        texts.emplace_back(text);
            return texts;
        };
        for (const auto tier : { RelativityStageTier::full, RelativityStageTier::compact })
            for (const auto speed : { 0.0, 0.5, physics::maximum_speed_fraction })
            {
                const auto ready = race_texts(testing::relativity_stage_at(speed, 0.0, tier));
                RIGIDBODIES_EXPECT(ready.size() == 1 && ready.front() == "Light and probe start together", "at Ready the light and the probe start together, not with a lead of 0");
                const auto racing = testing::relativity_stage_at(speed, 4.0e-9, tier);
                const auto texts = race_texts(racing);
                RIGIDBODIES_EXPECT(texts.size() == 1 && texts.front() == "Light ahead by " + core::format_quantity(racing.light_lead_m, core::DisplayQuantity::small_length, racing.units), "under way the plate gives the light's lead");
            }
    }

    RIGIDBODIES_TEST("stage text has no missing glyphs")
    {
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            for (const auto tier : tiers)
                for (const auto speed : { 0.0, 1.0e-9, 0.5, 0.95, 0.99999, physics::maximum_speed_fraction })
                    for (const auto time : { 0.0, 7.5e-9, 41.0e-9, 3600.0e-9 })
                    {
                        auto stage = testing::relativity_stage_at(speed, time, tier, units);
                        stage.last_lap_speed_changed = stage.last_lap_margin_s.has_value();
                        render::SceneRenderer renderer;
                        render::DrawList list;
                        draw_stage(renderer, list, stage, stage_settings());
                        // The slow-motion plate's "1 s" keeps its unit after a no-break space, like
                        // the formatted value it is equated with.
                        const auto lab = core::format_quantity(stage.lab_seconds_per_screen_second, core::DisplayQuantity::fine_time, units);
                        bool slow_motion = false;
                        std::size_t texts = 0;
                        for (const auto& command : list.commands())
                        {
                            if (command.kind != DrawCommandKind::text)
                                continue;
                            ++texts;
                            const auto text = text_of(list, command);
                            RIGIDBODIES_EXPECT(text.find('?') == std::string_view::npos, "stage text never needs a question mark: " + std::string(text));
                            for (const auto code : code_points(text))
                                RIGIDBODIES_EXPECT(baked(code), "every character on the stage is in the scene font: " + std::string(text));
                            slow_motion = slow_motion || text == "Slow motion: 1\xC2\xA0s on screen = " + lab + " in the lab" || text == "1\xC2\xA0s = " + lab;
                        }
                        RIGIDBODIES_EXPECT(texts >= 12, "the stage draws its readings");
                        RIGIDBODIES_EXPECT(slow_motion, "the band states the slow motion with a no-break space before each unit");
                    }
    }

    RIGIDBODIES_TEST("plates never overlap and stay inside the focus")
    {
        for (const auto display_scale : { 1.0f, 1.5f, 2.0f })
            for (const auto text_scale : { 1.0f, 1.4f })
                for (const auto tier : tiers)
                    for (const auto speed : { 0.0, 0.5, 0.99999 })
                        for (const auto time : { 0.0, 4.6e-9, 9.5e-9, 26.0e-9 })
                        {
                            auto stage = testing::relativity_stage_at(speed, time, tier);
                            stage.last_lap_speed_changed = stage.last_lap_margin_s.has_value();
                            const auto settings = stage_settings(display_scale, text_scale);
                            render::SceneRenderer renderer;
                            render::DrawList list;
                            const auto drawn = draw_stage(renderer, list, stage, settings);
                            const auto when = " at display scale " + std::to_string(display_scale) + ", text " + std::to_string(text_scale) + ", tier " + std::to_string(static_cast<int>(tier)) + ", " + std::to_string(speed) + " c, " + std::to_string(time * 1.0e9) + " ns";
                            RIGIDBODIES_EXPECT(render::choose_relativity_tier(drawn.focus.width, drawn.focus.height, drawn.tps, stage.units, tier) == tier, "the focus area holds the tier" + when);
                            const auto plates = drawn_plates(list, settings.theme);
                            // The band's eight, the lab and race plates, the probe's clock and the scale key.
                            RIGIDBODIES_EXPECT(plates.size() >= 11, "the stage shows its plates" + when);
                            for (std::size_t a = 0; a < plates.size(); ++a)
                            {
                                RIGIDBODIES_EXPECT(inside(plates[a], drawn.focus, 0.5), "every plate stays inside the focus" + when);
                                for (std::size_t b = a + 1; b < plates.size(); ++b)
                                    RIGIDBODIES_EXPECT(overlap(plates[a], plates[b]) == 0.0, "no plate lies over another" + when);
                            }
                        }
    }

    RIGIDBODIES_TEST("readings keep 4.5:1 contrast in every theme")
    {
        const auto luminance = [](const render::Color& color)
        {
            const auto linear = [](float value)
            {
                return value <= 0.03928f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
            };
            return 0.2126f * linear(color.red) + 0.7152f * linear(color.green) + 0.0722f * linear(color.blue);
        };
        for (const auto* name : { "workbench_dark", "workbench_light", "workbench_projector" })
            for (const auto tier : tiers)
            {
                const auto settings = stage_settings(1.0f, 1.0f, name);
                const auto& theme = settings.theme;
                const auto plate = render::mix(theme.background, theme.label_plate.with_alpha(1.0f), theme.label_plate.alpha);
                render::SceneRenderer renderer;
                render::DrawList list;
                draw_stage(renderer, list, testing::relativity_stage_at(0.99999, 23.0e-9, tier), settings);
                const auto plates = drawn_plates(list, theme);
                std::set<std::string> hues;
                for (const auto& command : list.commands())
                {
                    if (command.kind != DrawCommandKind::text)
                        continue;
                    const auto origin = list.vertices()[command.vertex_offset];
                    const auto on_plate = std::any_of(plates.begin(), plates.end(), [&](const render::ScreenRect& rect)
                        {
                            return origin.x >= rect.left && origin.x <= rect.left + rect.width && origin.y >= rect.top - 4.0 && origin.y <= rect.top + rect.height;
                        });
                    if (!on_plate)
                        continue;
                    const auto ink = render::mix(plate, command.color.with_alpha(1.0f), command.color.alpha);
                    const auto lighter = std::max(luminance(ink), luminance(plate)), darker = std::min(luminance(ink), luminance(plate));
                    RIGIDBODIES_EXPECT((lighter + 0.05f) / (darker + 0.05f) >= 4.5f, std::string("every reading on a plate keeps at least 4.5:1 in ") + name + ": " + std::string(text_of(list, command)));
                    for (const auto& [hue, role] : { std::pair<const char*, render::Color> { "text", theme.label_text }, { "velocity", theme.velocity }, { "momentum", theme.momentum } })
                        if (same_color(command.color, role))
                            hues.insert(hue);
                }
                RIGIDBODIES_EXPECT(hues.size() == 3, std::string("the band shows label text, velocity and momentum readings in ") + name);
            }
    }

    RIGIDBODIES_TEST("the scale key avoids the band and the instruments")
    {
        for (const auto display_scale : { 1.0f, 1.5f })
            for (const auto text_scale : { 1.0f, 1.4f })
                for (const auto tier : tiers)
                    for (const auto speed : { 0.0, 0.99999 })
                    {
                        const auto settings = stage_settings(display_scale, text_scale);
                        render::SceneRenderer renderer;
                        render::DrawList list;
                        draw_stage(renderer, list, testing::relativity_stage_at(speed, 6.0e-9, tier), settings);
                        std::optional<render::ScreenRect> key;
                        for (const auto& command : list.commands())
                            if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.layer == render::instrument_layer && same_color(command.color, settings.theme.label_plate))
                                key = bounds_of(list, command);
                        const auto& layout = *renderer.relativity_stage_layout();
                        const auto when = " in tier " + std::to_string(static_cast<int>(tier)) + " at display scale " + std::to_string(display_scale) + ", text " + std::to_string(text_scale);
                        RIGIDBODIES_EXPECT(key && std::none_of(layout.fixed_plates.begin(), layout.fixed_plates.end(), [&](const render::ScreenRect& plate)
                                                      {
                                                          return plate.left == key->left && plate.top == key->top;
                                                      }),
                            "the scale key is drawn after the band" + when);
                        RIGIDBODIES_EXPECT(!renderer.stage_instrument_areas().empty(), "the stage lists its instruments" + when);
                        for (const auto& [minimum, maximum] : renderer.stage_instrument_areas())
                            RIGIDBODIES_EXPECT(overlap(*key, { minimum.x, minimum.y, maximum.x - minimum.x, maximum.y - minimum.y }) == 0.0, "the scale key lies clear of every instrument" + when);
                    }
    }

    RIGIDBODIES_TEST("on a stage too short for the apparatus the scale key never covers an instrument")
    {
        std::size_t drawn = 0, left_out = 0;
        for (const auto text_scale : { 1.0f, 2.0f })
            for (const auto width : { 560.0, 800.0, 1108.0 })
                for (double height = 100.0; height <= 460.0; height += 20.0)
                {
                    const auto settings = stage_settings(1.0f, text_scale);
                    const auto tps = render::stage_text_pixel_scale(settings);
                    const render::ScreenRect focus { 40.0, 30.0, width, height };
                    auto stage = testing::relativity_stage_at(0.99999, 6.0e-9, RelativityStageTier::minimal);
                    stage.tier = render::choose_relativity_tier(focus.width, focus.height, tps, core::DisplayUnits::si, RelativityStageTier::minimal);
                    render::SceneRenderer renderer;
                    render::DrawList list;
                    draw_stage(renderer, list, stage, settings, focus);
                    // The key is the last plate drawn on the instrument layer, unless that is one of
                    // the stage's own fixed plates.
                    std::optional<render::ScreenRect> last;
                    for (const auto& command : list.commands())
                        if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.layer == render::instrument_layer && same_color(command.color, settings.theme.label_plate))
                            last = bounds_of(list, command);
                    const auto& layout = *renderer.relativity_stage_layout();
                    const auto fixed = last && std::any_of(layout.fixed_plates.begin(), layout.fixed_plates.end(), [&](const render::ScreenRect& plate)
                                                   {
                                                       return plate.left == last->left && plate.top == last->top;
                                                   });
                    if (!last || fixed)
                    {
                        ++left_out;
                        continue;
                    }
                    ++drawn;
                    const auto when = " at " + std::to_string(static_cast<int>(width)) + " x " + std::to_string(static_cast<int>(height)) + ", text " + std::to_string(text_scale);
                    for (const auto& [minimum, maximum] : renderer.stage_instrument_areas())
                        RIGIDBODIES_EXPECT(overlap(*last, { minimum.x, minimum.y, maximum.x - minimum.x, maximum.y - minimum.y }) == 0.0, "a drawn scale key lies clear of every instrument" + when);
                }
        RIGIDBODIES_EXPECT(drawn > 0 && left_out > 0, "the key is drawn where a corner is free and left out where none is");
    }

    RIGIDBODIES_TEST("the band height matches what is drawn and ignores the values")
    {
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            for (const auto text_scale : { 1.0f, 1.4f })
                for (const auto width : { 575.0, 760.0, 1300.0 })
                {
                    const auto settings = stage_settings(1.5f, text_scale);
                    const auto tps = render::stage_text_pixel_scale(settings);
                    const render::ScreenRect focus { 60.0, 40.0, width * tps, 600.0 * tps };
                    std::optional<render::ScreenRect> first;
                    std::vector<render::ScreenRect> first_plates;
                    for (const auto& [speed, lab_time_s] : { std::pair { 0.0, 2.0e-9 }, std::pair { 1.0e-9, 0.0 }, std::pair { 0.5, 7.3e-9 }, std::pair { 0.99999, 3.0e-6 }, std::pair { physics::maximum_speed_fraction, 2.0e-9 } })
                    {
                        auto stage = testing::relativity_stage_at(speed, lab_time_s, RelativityStageTier::full, units);
                        stage.tier = render::choose_relativity_tier(focus.width, focus.height, tps, units, RelativityStageTier::full);
                        render::SceneRenderer renderer;
                        render::DrawList list;
                        draw_stage(renderer, list, stage, settings, focus);
                        const auto& layout = *renderer.relativity_stage_layout();
                        const auto band = layout.band;
                        RIGIDBODIES_EXPECT(band.left == focus.left && band.top == focus.top && band.width == focus.width, "the band spans the top of the focus area");
                        RIGIDBODIES_EXPECT(band.height == render::relativity_band_height_px(focus.width, tps, units), "the band is as tall as the framing made room for");
                        // Its plates are the first eight fixed plates, 8 pixels below its top and 6 above its foot.
                        double top = 1.0e300, bottom = -1.0e300;
                        for (std::size_t index = 0; index < 8; ++index)
                        {
                            const auto& plate = layout.fixed_plates[index];
                            RIGIDBODIES_EXPECT(inside(plate, band, 0.5), "every band plate lies in the band");
                            top = std::min(top, plate.top);
                            bottom = std::max(bottom, plate.top + plate.height);
                        }
                        RIGIDBODIES_EXPECT_NEAR(top - band.top, 8.0 * tps, 0.5, "the first row stands 8 pixels below the top");
                        RIGIDBODIES_EXPECT_NEAR(band.top + band.height - bottom, 6.0 * tps, 0.5, "the last row ends 6 pixels above the band's foot");
                        if (!first)
                            first = band;
                        RIGIDBODIES_EXPECT(band.height == first->height, "the band is as tall at the maximum speed as at rest");
                        // Each plate is sized for its reading's widest text, so no plate moves or
                        // changes size as the readings change.
                        if (first_plates.empty())
                            first_plates.assign(layout.fixed_plates.begin(), layout.fixed_plates.begin() + 8);
                        for (std::size_t index = 0; index < 8; ++index)
                        {
                            const auto& plate = layout.fixed_plates[index];
                            const auto& before = first_plates[index];
                            RIGIDBODIES_EXPECT(plate.left == before.left && plate.top == before.top && plate.width == before.width && plate.height == before.height, "band plate " + std::to_string(index) + " keeps its place and size at " + std::to_string(speed) + " c");
                        }
                    }
                }
    }

    RIGIDBODIES_TEST("every band value fits its plate")
    {
        for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            for (const auto width : { 600.0, 1300.0 })
                for (const auto speed : { 0.0, 1.0e-9, 3.3e-7, 0.5, 0.99999985, physics::maximum_speed_fraction, 3.3e-11, 1.0e-15 })
                {
                    const auto settings = stage_settings();
                    const auto metrics = render::detail::label_metrics(settings);
                    const render::ScreenRect focus { 40.0, 30.0, width, 640.0 };
                    auto stage = testing::relativity_stage_at(speed, 3.0e-9, RelativityStageTier::full, units);
                    stage.tier = render::choose_relativity_tier(focus.width, focus.height, 1.0, units, RelativityStageTier::full);
                    render::SceneRenderer renderer;
                    render::DrawList list;
                    draw_stage(renderer, list, stage, settings, focus);
                    const auto& layout = *renderer.relativity_stage_layout();
                    const auto when = " at " + std::to_string(speed) + " c, " + std::to_string(static_cast<int>(width)) + " px across" + (units == core::DisplayUnits::si ? "" : " in centimetre-gram units");
                    std::size_t inside_band = 0;
                    for (const auto& command : list.commands())
                    {
                        if (command.kind != DrawCommandKind::text || command.layer != render::instrument_layer)
                            continue;
                        const auto origin = list.vertices()[command.vertex_offset];
                        if (origin.y > layout.band.top + layout.band.height)
                            continue;
                        const auto text = text_of(list, command);
                        const auto right = origin.x + render::detail::text_width(text, command.text_scale);
                        const auto plate = std::find_if(layout.fixed_plates.begin(), layout.fixed_plates.begin() + 8, [&](const render::ScreenRect& rect)
                            {
                                return origin.x >= rect.left && origin.x < rect.left + rect.width && origin.y >= rect.top - metrics.height && origin.y < rect.top + rect.height;
                            });
                        RIGIDBODIES_EXPECT(plate != layout.fixed_plates.begin() + 8, "every band text sits on a band plate" + when);
                        ++inside_band;
                        RIGIDBODIES_EXPECT(origin.x >= plate->left + metrics.pad_x - 0.5 && right <= plate->left + plate->width - metrics.pad_x + 0.5, "the text fits inside its plate's padding" + when + ": " + std::string(text));
                        RIGIDBODIES_EXPECT(!command.clip, "a value is shortened rather than cut" + when + ": " + std::string(text));
                    }
                    RIGIDBODIES_EXPECT(inside_band >= 14, "the band draws its names and readings" + when);
                }
    }

    RIGIDBODIES_TEST("the gauge fills one cell per nine and is never full")
    {
        const auto settings = stage_settings();
        for (const auto speed : { 0.0, 0.5, 0.9, 0.99, 0.999, 0.9999, 0.99999, 0.999999, physics::maximum_speed_fraction })
        {
            render::SceneRenderer renderer;
            render::DrawList list;
            const auto stage = testing::relativity_stage_at(speed, 1.0e-9);
            draw_stage(renderer, list, stage, settings);
            double filled = 0.0, cells = 0.0, cell_width = 0.0;
            for (const auto& command : list.commands())
            {
                if (command.kind != DrawCommandKind::rectangle_fill || command.layer != render::instrument_layer)
                    continue;
                const auto bounds = bounds_of(list, command);
                if (same_color(command.color, settings.theme.velocity))
                    filled += bounds.width;
                else if (same_color(command.color, settings.theme.grid_major))
                {
                    cells += 1.0;
                    cell_width = bounds.width;
                }
            }
            const auto nines = speed == 0.0 ? 0.0 : -std::log10(stage.one_minus_speed_fraction);
            RIGIDBODIES_EXPECT(cells == 8.0 && cell_width > 0.0, "the gauge has eight cells");
            RIGIDBODIES_EXPECT_NEAR(filled / (8.0 * cell_width), nines / 8.0, 1.0e-9, "the gauge is filled to the number of nines over eight at " + std::to_string(speed) + " c");
            RIGIDBODIES_EXPECT(filled <= 7.0 * cell_width + 1.0e-9, "at least one cell is always empty");
        }
    }

    RIGIDBODIES_TEST("reduce motion removes only the glow and tail")
    {
        const auto stage = testing::relativity_stage_at(0.5, 4.0e-9);
        RIGIDBODIES_EXPECT(!stage.light_finished, "the pulse is on the track");
        const auto draw = [&](bool transitions)
        {
            auto settings = stage_settings();
            settings.transitions = transitions;
            render::SceneRenderer renderer;
            render::DrawList list;
            draw_stage(renderer, list, stage, settings);
            return list;
        };
        const auto moving = draw(true), calm = draw(false);
        const auto decorations = [](const render::DrawList& list)
        {
            std::size_t count = 0;
            for (const auto& command : list.commands())
                count += command.layer == 0 && (command.kind == DrawCommandKind::indexed_mesh || command.kind == DrawCommandKind::shadow) ? 1u : 0u;
            return count;
        };
        RIGIDBODIES_EXPECT(decorations(moving) == 2 && decorations(calm) == 0, "the pulse's glow and tail are drawn only with transitions on");
        // Everything else is the same drawing: texts, and every other command with its colour.
        const auto rest = [](const render::DrawList& list)
        {
            std::vector<std::pair<int, std::string>> items;
            for (const auto& command : list.commands())
                if (!(command.layer == 0 && (command.kind == DrawCommandKind::indexed_mesh || command.kind == DrawCommandKind::shadow)))
                    items.emplace_back(static_cast<int>(command.kind), command.kind == DrawCommandKind::text ? std::string(text_of(list, command)) : std::to_string(command.color.red) + std::to_string(command.color.alpha));
            return items;
        };
        RIGIDBODIES_EXPECT(rest(moving) == rest(calm), "everything else, the probe, the pulse and both hands included, is drawn alike");
    }

    RIGIDBODIES_TEST("an extra render pass is identical")
    {
        render::SceneRenderer renderer;
        auto settings = stage_settings();
        settings.transitions = true;
        const auto stage = testing::relativity_stage_at(0.9, 13.0e-9);
        render::DrawList first, second;
        renderer.advance_presentation(1.0 / 60.0);
        const auto drawn = draw_stage(renderer, first, stage, settings);
        physics::World world;
        renderer.render(world, drawn.camera, settings, second);
        RIGIDBODIES_EXPECT(first.commands().size() == second.commands().size() && first.vertices() == second.vertices() && first.text_buffer() == second.text_buffer(), "a second pass draws the same apparatus");
        for (std::size_t index = 0; index < first.commands().size(); ++index)
            RIGIDBODIES_EXPECT(first.commands()[index].kind == second.commands()[index].kind && same_color(first.commands()[index].color, second.commands()[index].color), "command by command");
    }

    RIGIDBODIES_TEST("the finish detector fills as the light arrives and fades in only with transitions")
    {
        // On the track at 0.5 c, then 15 ns in: the light finished at 10 ns, the probe is mid-lap.
        const auto racing = testing::relativity_stage_at(0.5, 4.0e-9);
        const auto arrived = testing::relativity_stage_at(0.5, 15.0e-9);
        RIGIDBODIES_EXPECT(!racing.light_finished && arrived.light_finished, "the light arrives between the two moments");
        // The detector is the only rounded box on the probe's layer: a plate-toned box, and over it
        // a label_text fill whose opacity is 70 % of the ink's once the light is in.
        const auto fill = [](const render::DrawList& list, const render::Theme& theme)
        {
            std::size_t boxes = 0;
            double share = 0.0;
            for (const auto& command : list.commands())
            {
                if (command.layer != 0 || command.kind != DrawCommandKind::rounded_rectangle_fill)
                    continue;
                if (++boxes == 2)
                    share = static_cast<double>(command.color.alpha) / static_cast<double>(theme.label_text.alpha);
            }
            RIGIDBODIES_EXPECT(boxes == 1 || boxes == 2, "the detector is one box with at most one fill over it");
            return share;
        };
        const auto same_drawing = [](const render::DrawList& first, const render::DrawList& second)
        {
            if (first.commands().size() != second.commands().size() || !(first.vertices() == second.vertices()) || first.text_buffer() != second.text_buffer())
                return false;
            for (std::size_t index = 0; index < first.commands().size(); ++index)
            {
                const auto& a = first.commands()[index];
                const auto& b = second.commands()[index];
                if (a.kind != b.kind || a.layer != b.layer || !same_color(a.color, b.color))
                    return false;
            }
            return true;
        };

        // Reduce motion: the detector is full on the frame the light arrives.
        {
            const auto settings = stage_settings();
            render::SceneRenderer renderer;
            render::DrawList list;
            const auto drawn = draw_stage(renderer, list, racing, settings);
            RIGIDBODIES_EXPECT(fill(list, settings.theme) == 0.0, "an empty detector while the light is on its way");
            renderer.set_relativity_stage(arrived);
            physics::World world;
            renderer.render(world, drawn.camera, settings, list);
            RIGIDBODIES_EXPECT_NEAR(fill(list, settings.theme), 0.7, 1.0e-6, "with reduce motion the detector fills at once");
        }

        // Transitions: the fill eases in with the presentation clock, and a repeat pass without it
        // draws the same frame, the detector's opacity included.
        auto settings = stage_settings();
        settings.transitions = true;
        render::SceneRenderer renderer;
        render::DrawList list, repeat;
        renderer.advance_presentation(1.0 / 60.0);
        const auto drawn = draw_stage(renderer, list, racing, settings);
        RIGIDBODIES_EXPECT(fill(list, settings.theme) == 0.0, "an empty detector while the light is on its way, with transitions too");
        physics::World world;
        renderer.set_relativity_stage(arrived);
        renderer.advance_presentation(1.0 / 60.0);
        renderer.render(world, drawn.camera, settings, list);
        auto previous = fill(list, settings.theme);
        RIGIDBODIES_EXPECT(previous > 0.0 && previous < 0.5 * 0.7, "one frame after the light arrives the detector has only begun to fill");
        renderer.render(world, drawn.camera, settings, repeat);
        RIGIDBODIES_EXPECT(same_drawing(list, repeat), "an extra pass without the clock draws the same frame, command by command");
        for (int frame = 0; frame < 30; ++frame)
        {
            renderer.advance_presentation(1.0 / 60.0);
            renderer.render(world, drawn.camera, settings, list);
            const auto now = fill(list, settings.theme);
            RIGIDBODIES_EXPECT(now >= previous && now <= 0.7 + 1.0e-6, "the fill rises frame by frame up to 70 %");
            previous = now;
        }
        RIGIDBODIES_EXPECT_NEAR(previous, 0.7, 1.0e-3, "after half a second the detector is full");
        // Back on the track (a new lap's pulse), it empties the same way.
        renderer.set_relativity_stage(racing);
        renderer.advance_presentation(1.0 / 60.0);
        renderer.render(world, drawn.camera, settings, list);
        const auto emptying = fill(list, settings.theme);
        RIGIDBODIES_EXPECT(emptying > 0.0 && emptying < previous, "a new lap fades the detector out");
    }

    RIGIDBODIES_TEST("clearing the stage removes its plates, areas and label memory")
    {
        const auto settings = stage_settings();
        // Mid-track, clear of the lab and race plates on either side.
        const auto stage = testing::relativity_stage_at(0.5, 9.0e-9);
        const auto probe_plate = [](const render::DrawList& list)
        {
            for (const auto& command : list.commands())
                if (const auto text = text_of(list, command); command.kind == DrawCommandKind::text && text.rfind("Probe clock ", 0) == 0 && text.rfind("Probe clock behind", 0) != 0)
                    return std::optional<math::Vec2> { list.vertices()[command.vertex_offset] };
            return std::optional<math::Vec2> {};
        };
        // With the place right of the probe's clock taken by an overlay, its plate goes to the next
        // place round the clock; once that place is remembered, it keeps it after the overlay goes.
        const auto run = [&](bool clear_between)
        {
            render::SceneRenderer renderer;
            render::DrawList list;
            auto drawn = draw_stage(renderer, list, stage, settings);
            const auto face = renderer.relativity_stage_layout()->probe_clock_center;
            const auto radius = renderer.relativity_stage_layout()->clock_radius;
            renderer.set_overlay_areas({ { face + math::Vec2 { radius + 1.0, -4.0 }, face + math::Vec2 { radius + 400.0, 4.0 } } });
            physics::World world;
            renderer.render(world, drawn.camera, settings, list);
            const auto blocked = probe_plate(list);
            if (clear_between)
            {
                renderer.set_relativity_stage(std::nullopt);
                RIGIDBODIES_EXPECT(!renderer.relativity_stage_layout() && renderer.stage_instrument_areas().empty(), "clearing the stage forgets its layout and its instrument areas");
                renderer.set_relativity_stage(stage);
            }
            renderer.set_overlay_areas({});
            renderer.render(world, drawn.camera, settings, list);
            return std::pair { blocked, probe_plate(list) };
        };
        const auto [kept_blocked, kept] = run(false);
        const auto [cleared_blocked, cleared] = run(true);
        RIGIDBODIES_EXPECT(kept_blocked && kept && cleared_blocked && cleared, "the probe's clock plate is drawn");
        RIGIDBODIES_EXPECT(kept->y == kept_blocked->y, "a remembered place is kept");
        RIGIDBODIES_EXPECT(cleared->y > cleared_blocked->y, "after the stage was cleared the plate returns to its first place, level with the clock");

        render::SceneRenderer renderer;
        render::DrawList list;
        const auto drawn = draw_stage(renderer, list, stage, settings);
        renderer.set_relativity_stage(std::nullopt);
        physics::World world;
        renderer.render(world, drawn.camera, settings, list);
        RIGIDBODIES_EXPECT(!renderer.relativity_stage_layout() && renderer.stage_instrument_areas().empty() && drawn_plates(list, settings.theme).size() == 1, "a Newtonian frame keeps nothing of the apparatus but the scale key");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
