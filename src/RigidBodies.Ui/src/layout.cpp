#include <rigidbodies/ui/layout.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace rigidbodies::ui
{
    namespace
    {
        Rect scaled(double x, double y, double w, double h, double scale)
        {
            return { { x * scale, y * scale }, { (x + w) * scale, (y + h) * scale } };
        }

        void add(LayoutResult& out, RegionId id, double x, double y, double w, double h,
            double scale, RegionPresentation presentation)
        {
            if (w > 0.0 && h > 0.0)
                out.regions.push_back({ id, scaled(x, y, w, h, scale), presentation });
        }

    }

    const LayoutRegion* LayoutResult::find(RegionId id) const
    {
        const auto it = std::find_if(regions.begin(), regions.end(), [id](const auto& region)
            {
                return region.id == id;
            });
        return it == regions.end() ? nullptr : &*it;
    }

    LayoutResult compute_layout(const LayoutInput& input)
    {
        LayoutResult out;
        const double scale = std::max(0.01, static_cast<double>(input.scale));
        const double w = static_cast<double>(input.viewport.width) / scale;
        const double h = static_cast<double>(input.viewport.height) / scale;
        out.logical_width = w;
        out.logical_height = h;

        if (w >= 1432.0 || (w >= 1400.0 && input.previous_mode == LayoutMode::wide))
            out.mode = LayoutMode::wide;
        else if (w < 760.0 || (w < 792.0 && input.previous_mode == LayoutMode::narrow))
            out.mode = LayoutMode::narrow;
        else
            out.mode = LayoutMode::medium;

        const double top = input.present ? 56.0 : 44.0 + (input.draw_bar ? 36.0 : 0.0);
        // The Measure sheet overlays the stage foot; framing keeps the subject above it, the way it
        // keeps the subject above the Present caption.
        std::optional<double> sheet_framing_bottom;
        // A side sheet likewise takes its side of the stage out of the framing while it is open.
        std::optional<double> sheet_framing_left, sheet_framing_right;
        // As the window's title bar the strip keeps one surface across the window, under the app
        // mark, the controls and whatever dims it; only the strip's own content is narrowed. The
        // controls are three 46 dp caption buttons after a divider with an 8 dp gap either side.
        const double strip = input.present ? 56.0 : 44.0;
        const double brand = input.window_controls ? 36.0 : 0.0;
        const double controls = input.window_controls ? 3.0 * 46.0 + 17.0 : 0.0;
        if (input.window_controls)
        {
            add(out, RegionId::title_bar, 0, 0, w, strip, scale, RegionPresentation::docked);
            add(out, RegionId::window_brand, 0, 0, brand, strip, scale, RegionPresentation::docked);
            add(out, RegionId::window_controls, std::max(brand, w - controls), 0, std::min(controls, w - brand), strip, scale, RegionPresentation::docked);
        }
        const double strip_width = std::max(0.0, w - brand - controls);
        if (input.present)
            add(out, RegionId::present_strip, brand, 0, strip_width, strip, scale, RegionPresentation::docked);
        else
        {
            add(out, RegionId::command_bar, brand, 0, strip_width, strip, scale, RegionPresentation::docked);
            if (input.draw_bar)
                add(out, RegionId::draw_bar, 0, strip, w, 36, scale, RegionPresentation::docked);
        }

        if (input.present)
        {
            out.stage = scaled(0, top, w, h - top, scale);
            add(out, RegionId::stage, 0, top, w, h - top, scale, RegionPresentation::docked);
        }
        else if (input.overlay_backend)
        {
            const double list = std::min(300.0, std::max(0.0, w - 520.0));
            add(out, RegionId::overlay_list, 0, top, list, h - top, scale, RegionPresentation::docked);
            out.stage = scaled(list, top, w - list, h - top, scale);
            add(out, RegionId::stage, list, top, w - list, h - top, scale, RegionPresentation::docked);
            add(out, RegionId::status_line, list, h - 28, w - list, 28, scale, RegionPresentation::docked);
        }
        else
        {
            double left = 0.0, right = 0.0, ribbon = 0.0;
            bool guide_sheet = false, inspector_sheet = false;
            auto guide_docked = input.guide_available && input.guide_expanded;
            auto inspector_docked = input.inspector_open;

            // Wide and medium collapse a side to its 28 dp rail; the full-width ribbon exists only
            // in narrow mode, where no rail fits beside the stage.
            if (out.mode == LayoutMode::wide)
            {
                left = guide_docked ? 320.0 : (input.guide_available ? 28.0 : 0.0);
                right = inspector_docked ? 320.0 : 0.0;
            }
            else if (out.mode == LayoutMode::medium)
            {
                if (guide_docked && inspector_docked)
                {
                    if (input.least_recently_used == SideSurface::guide)
                        guide_docked = false;
                    else
                        inspector_docked = false;
                }
                left = guide_docked ? 272.0 : (input.guide_available ? 28.0 : 0.0);
                right = inspector_docked ? 280.0 : (input.inspector_open ? 28.0 : 0.0);
            }
            else
            {
                guide_sheet = input.guide_available && input.narrow_guide_sheet;
                inspector_sheet = input.narrow_inspector_sheet;
                // One side sheet at a time: two stacked sheets would bury the stage entirely.
                if (guide_sheet && inspector_sheet)
                {
                    if (input.least_recently_used == SideSurface::guide)
                        guide_sheet = false;
                    else
                        inspector_sheet = false;
                }
                // The ribbon is the way to the Guide; while the Guide sheet is open its own header
                // closes it.
                ribbon = input.guide_available && !guide_sheet ? 32.0 : 0.0;
            }

            if (w - left - right < 520.0)
            {
                if (input.least_recently_used == SideSurface::guide && left > 28.0)
                {
                    left = input.guide_available ? 28.0 : 0.0;
                }
                else if (right > 28.0)
                {
                    right = input.inspector_open ? 28.0 : 0.0;
                    // Too narrow to dock beside the stage: on request the Inspector covers the
                    // stage as a sheet, as it does in a narrow window, rather than staying a rail.
                    inspector_sheet = input.inspector_open && input.narrow_inspector_sheet;
                }
            }
            // Beside a docked Guide even the Inspector's rail may not fit; the Inspector then
            // leaves the side and appears only on request, as a sheet.
            if (w - left - right < 520.0 && right > 0.0)
            {
                right = 0.0;
                inspector_sheet = input.inspector_open && input.narrow_inspector_sheet;
            }

            const double stage_top = top + ribbon;
            double stage_bottom = h;
            bool drawer_sheet = out.mode == LayoutMode::narrow;
            const auto available_height = std::max(0.0, h - stage_top);
            const auto maximum_drawer_height = (out.mode == LayoutMode::wide ? 0.5 : 0.45) * available_height;
            const auto minimum_drawer_height = std::min(160.0, maximum_drawer_height);
            // Until the learner drags the drawer it takes a share of the window, so a large window
            // gets a readable graph and key figures without scrolling.
            const auto requested_drawer_height = input.measure_height_logical > 0.0 ? input.measure_height_logical : 0.4 * available_height;
            double drawer_height = std::clamp(requested_drawer_height, minimum_drawer_height, maximum_drawer_height);
            if (input.measure_open && !drawer_sheet && h - stage_top - drawer_height < 300.0)
                drawer_sheet = true;
            if (input.measure_open && !drawer_sheet)
                stage_bottom -= drawer_height;

            if (left > 0.0)
                add(out, left > 28.0 ? RegionId::guide_panel : RegionId::guide_rail, 0, top, left, h - top, scale, left > 28.0 ? RegionPresentation::docked : RegionPresentation::rail);
            if (ribbon > 0.0)
                add(out, RegionId::guide_ribbon, left, top, w - left - right, ribbon, scale, RegionPresentation::ribbon);
            if (right > 0.0)
                add(out, right > 28.0 ? RegionId::inspector : RegionId::inspector_rail, w - right, top, right, h - top, scale, right > 28.0 ? RegionPresentation::docked : RegionPresentation::rail);

            out.stage = scaled(left, stage_top, w - left - right, stage_bottom - stage_top, scale);
            add(out, RegionId::stage, left, stage_top, w - left - right, stage_bottom - stage_top, scale, RegionPresentation::docked);
            add(out, RegionId::status_line, left, stage_bottom - 28, w - left - right, 28, scale, RegionPresentation::docked);
            // Sheets stop above the status line, and side sheets stop above an open Measure sheet,
            // so time, selection and view stay readable and no sheet buries another.
            const auto status_top = stage_bottom - 28.0;
            // A Measure sheet is resized from its top edge like the docked drawer, whose height
            // is kept as that edge's distance from the window bottom; some stage stays in view.
            const auto measure_sheet_room = std::max(0.0, status_top - stage_top);
            const auto measure_sheet_maximum = std::max(std::min(160.0, measure_sheet_room), measure_sheet_room - 96.0);
            const auto measure_sheet_height = input.measure_height_logical > 0.0
                ? std::clamp(input.measure_height_logical - (h - status_top), std::min(160.0, measure_sheet_maximum), measure_sheet_maximum)
                : std::min(0.45 * h, measure_sheet_room);
            if (input.measure_open)
                add(out, RegionId::measure_drawer, left, drawer_sheet ? status_top - measure_sheet_height : stage_bottom, w - left - right, drawer_sheet ? measure_sheet_height : drawer_height, scale, drawer_sheet ? RegionPresentation::sheet : RegionPresentation::docked);
            const auto side_sheet_bottom = input.measure_open && drawer_sheet ? status_top - measure_sheet_height : status_top;
            if (input.measure_open && drawer_sheet)
                sheet_framing_bottom = side_sheet_bottom - 16.0;
            const auto side_sheet_width = std::min(300.0, w * 0.85);
            if (guide_sheet)
            {
                add(out, RegionId::guide_panel, 0, stage_top, side_sheet_width, side_sheet_bottom - stage_top, scale, RegionPresentation::sheet);
                sheet_framing_left = side_sheet_width + 16.0;
            }
            if (inspector_sheet)
            {
                add(out, RegionId::inspector, w - side_sheet_width, stage_top, side_sheet_width, side_sheet_bottom - stage_top, scale, RegionPresentation::sheet);
                sheet_framing_right = w - side_sheet_width - 16.0;
            }
        }

        // The caption box is the most the card may grow to; its content decides the actual height,
        // measured from the bottom edge, and a longer card scrolls. Framing keeps the subject above
        // the whole box, so on a short projector the card never takes more than about a third of
        // the window from the demonstration.
        const double caption_height = input.present && input.present_caption ? std::min({ 260.0, std::max(0.0, (h - top) * 0.5), h * 0.35 }) : 0.0;
        if (caption_height > 0.0)
        {
            const auto caption_w = std::min(920.0, std::max(0.0, w - 48.0));
            add(out, RegionId::present_caption, (w - caption_w) * 0.5, h - 20.0 - caption_height, caption_w, caption_height, scale, RegionPresentation::sheet);
        }
        const double inset = 24.0 * scale;
        // Framing reserves the caption card's measured height once known, not the whole box.
        const double reserved_caption = input.present_caption_height > 0.0 ? std::min(caption_height, input.present_caption_height) : caption_height;
        const double bottom_inset = caption_height > 0.0 ? reserved_caption + 32.0 : input.present ? 68.0
                                                                                                   : 44.0;
        out.focus = { { out.stage.minimum.x + inset, out.stage.minimum.y + inset },
            { out.stage.maximum.x - inset, out.stage.maximum.y - bottom_inset * scale } };
        if (sheet_framing_bottom)
            out.focus.maximum.y = std::min(out.focus.maximum.y, *sheet_framing_bottom * scale);
        // With too little stage left beside a side sheet to frame anything in, the sheet simply
        // covers the stage.
        constexpr double minimum_framed_width = 160.0;
        if (sheet_framing_left && out.focus.maximum.x - *sheet_framing_left * scale >= minimum_framed_width * scale)
            out.focus.minimum.x = std::max(out.focus.minimum.x, *sheet_framing_left * scale);
        if (sheet_framing_right && *sheet_framing_right * scale - out.focus.minimum.x >= minimum_framed_width * scale)
            out.focus.maximum.x = std::min(out.focus.maximum.x, *sheet_framing_right * scale);
        if (out.focus.maximum.x < out.focus.minimum.x)
            out.focus.maximum.x = out.focus.minimum.x;
        if (out.focus.maximum.y < out.focus.minimum.y)
            out.focus.maximum.y = out.focus.minimum.y;

        // Simulation notices float over the stage and deliberately do not alter its focus rect.
        // This keeps camera framing stable when a pause or experiment-specific notice appears.
        const auto stage_x = out.stage.minimum.x / scale;
        const auto stage_y = out.stage.minimum.y / scale;
        const auto stage_w = out.stage.width() / scale;
        const auto banner_w = std::min(480.0, std::max(0.0, stage_w - 32.0));
        add(out, RegionId::banner, stage_x + (stage_w - banner_w) * 0.5, stage_y + 8.0, banner_w, 36.0, scale, RegionPresentation::sheet);

        // Overlay geometry is published even while closed so surfaces can be positioned without a
        // second layout calculation.
        const double lw = out.logical_width, lh = out.logical_height;
        const double library_w = out.mode == LayoutMode::wide ? 720.0 : out.mode == LayoutMode::medium ? 400.0
                                                                                                       : lw;
        // The library slides out beneath the toolbar so the transport and its own button stay reachable.
        add(out, RegionId::library_sheet, 0, top, library_w, lh - top, scale, RegionPresentation::sheet);
        add(out, RegionId::show_popover, std::max(0.0, lw - 328.0), out.stage.minimum.y / scale + 8.0, out.mode == LayoutMode::narrow ? lw : 320.0, std::max(0.0, out.stage.height() / scale - 16.0), scale, RegionPresentation::sheet);
        // A narrow window's dialogs keep a gutter on every side, so a card always reads as a card
        // rather than a band with rounded corners against the window edge.
        constexpr double narrow_modal_gutter = 16.0;
        const double modal_w = out.mode == LayoutMode::wide ? 640.0 : out.mode == LayoutMode::medium ? 600.0
                                                                                                     : std::max(0.0, lw - 2.0 * narrow_modal_gutter);
        // The box is the most a dialog may occupy; the document centres a shorter card inside it.
        const double modal_margin = out.mode == LayoutMode::narrow ? narrow_modal_gutter : std::min(48.0, lh * 0.06);
        // A dialog never covers the window controls, which stay usable above its scrim.
        const double modal_top = input.window_controls ? std::max(modal_margin, strip + 8.0) : modal_margin;
        const double modal_h = std::max(0.0, lh - modal_top - modal_margin);
        add(out, RegionId::modal, (lw - modal_w) * 0.5, modal_top, modal_w, modal_h, scale, RegionPresentation::sheet);
        // The document backend sizes the overlay's box to its text inside this area, anchored at
        // its top right; the overlay backend fills it.
        add(out, RegionId::performance_overlay, std::max(0.0, out.stage.maximum.x / scale - 268.0), out.stage.minimum.y / scale + 8.0, 260.0, 176.0, scale, RegionPresentation::sheet);

        return out;
    }
}
