#pragma once

#include <rigidbodies/render/render_device.hpp>
#include <rigidbodies/ui/geometry.hpp>

#include <cstdint>
#include <vector>

namespace rigidbodies::ui
{
    enum class LayoutMode : std::uint8_t
    {
        wide,
        medium,
        narrow
    };
    enum class RegionId : std::uint8_t
    {
        command_bar,
        draw_bar,
        present_strip,
        guide_panel,
        guide_rail,
        guide_ribbon,
        inspector,
        inspector_rail,
        measure_drawer,
        stage,
        status_line,
        toasts,
        banner,
        library_sheet,
        show_popover,
        changes_popover,
        modal,
        performance_overlay,
        overlay_list,
        present_caption,
        // When the interface draws the window's own title bar: the full-width strip surface, the
        // app mark at its left end and the window controls at its right end.
        title_bar,
        window_brand,
        window_controls
    };
    enum class RegionPresentation : std::uint8_t
    {
        docked,
        rail,
        ribbon,
        sheet,
        hidden
    };
    enum class SideSurface : std::uint8_t
    {
        guide,
        inspector
    };

    struct LayoutInput
    {
        render::ViewportSize viewport;
        float scale { 1.0f };
        LayoutMode previous_mode { LayoutMode::medium };
        bool present { false }, draw_bar { false }, overlay_backend { false };
        // Present mode shows the lesson step on a caption card floating over the stage's foot.
        bool present_caption { false };
        // The caption card's height in logical pixels as last laid out; zero until measured.
        double present_caption_height { 0.0 };
        // A narrow window covers the stage with a side surface only on request, independently of
        // whether the surface is docked open in wider windows.
        bool narrow_guide_sheet { false }, narrow_inspector_sheet { false };
        bool guide_available { false }, guide_expanded { false }, inspector_open { true };
        bool measure_open { false };
        // The docked Measure drawer height the learner dragged to; zero until then, when the
        // drawer takes a share of the window instead.
        double measure_height_logical { 0.0 };
        SideSurface least_recently_used { SideSurface::guide };
        // The top strip doubles as the window's title bar: the app mark and the window controls
        // take its ends, and the command bar or Present strip the width between them.
        bool window_controls { false };
    };

    struct LayoutRegion
    {
        RegionId id;
        Rect bounds;
        RegionPresentation presentation { RegionPresentation::hidden };
    };

    struct LayoutResult
    {
        LayoutMode mode { LayoutMode::medium };
        std::vector<LayoutRegion> regions;
        Rect stage, focus;
        double logical_width { 0.0 }, logical_height { 0.0 };

        [[nodiscard]] const LayoutRegion* find(RegionId id) const;
    };

    [[nodiscard]] LayoutResult compute_layout(const LayoutInput& input);
}
