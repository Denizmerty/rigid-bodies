#include <rigidbodies/ui/layout.hpp>
#include "test_framework.hpp"

#include <cmath>
#include <utility>

using namespace rigidbodies;

RIGIDBODIES_TEST("layout breakpoints publish the prescribed stage and focus rectangles")
{
    ui::LayoutInput input;
    input.viewport = { 1600, 900 };
    input.scale = 1.5f;
    input.inspector_open = true;
    const auto medium = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(medium.mode == ui::LayoutMode::medium, "1067 logical pixels selects Medium");
    RIGIDBODIES_EXPECT_NEAR(medium.stage.width() / input.scale, 786.6666667, 0.01, "Medium leaves 280 logical pixels for the inspector");
    RIGIDBODIES_EXPECT_NEAR(medium.stage.height() / input.scale, 556.0, 0.01, "the 44 pixel command bar leaves the prescribed stage height");
    RIGIDBODIES_EXPECT(medium.stage.width() * medium.stage.height() >= 0.60 * input.viewport.width * input.viewport.height, "the stage retains at least sixty percent of the window");
    RIGIDBODIES_EXPECT_NEAR(medium.focus.minimum.x - medium.stage.minimum.x, 36.0, 0.01, "focus has a 24 logical pixel horizontal inset");

    input.viewport = { 512, 320 };
    input.scale = 1.0f;
    input.previous_mode = medium.mode;
    const auto narrow = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(narrow.mode == ui::LayoutMode::narrow && narrow.stage.width() == 512.0 && narrow.stage.height() == 276.0, "Narrow keeps a 512 by 276 stage");
}

RIGIDBODIES_TEST("an active draft reserves the draw bar beneath the command bar in every mode")
{
    for (const auto width : { 1600, 1100, 700 })
    {
        ui::LayoutInput input;
        input.viewport = { width, 800 };
        input.draw_bar = true;
        const auto result = ui::compute_layout(input);
        const auto* bar = result.find(ui::RegionId::draw_bar);
        RIGIDBODIES_EXPECT(bar != nullptr, "the draw bar region exists while drawing at " + std::to_string(width));
        if (!bar)
            continue;
        RIGIDBODIES_EXPECT_NEAR(bar->bounds.minimum.y, 44.0, 0.01, "the draw bar starts below the command bar");
        RIGIDBODIES_EXPECT_NEAR(bar->bounds.width(), static_cast<double>(width), 0.01, "the draw bar spans the window");
        RIGIDBODIES_EXPECT(result.stage.minimum.y >= bar->bounds.maximum.y, "the stage begins below the draw bar");
        input.present = true;
        RIGIDBODIES_EXPECT(!ui::compute_layout(input).find(ui::RegionId::draw_bar), "Present mode replaces the draw bar with its own strip");
    }
}

RIGIDBODIES_TEST("a collapsed Guide is a rail beside the stage, and a ribbon only when narrow")
{
    ui::LayoutInput input;
    input.viewport = { 1600, 900 };
    input.guide_available = true;
    input.guide_expanded = false;
    const auto wide = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(wide.find(ui::RegionId::guide_rail) && !wide.find(ui::RegionId::guide_ribbon), "wide shows only the rail");
    RIGIDBODIES_EXPECT(wide.find(ui::RegionId::guide_rail)->presentation == ui::RegionPresentation::rail, "the rail is presented as a rail");
    RIGIDBODIES_EXPECT_NEAR(wide.stage.minimum.y, 44.0, 0.01, "no ribbon pushes the stage down beside a rail");
    input.viewport = { 600, 700 };
    const auto narrow = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(!narrow.find(ui::RegionId::guide_rail) && narrow.find(ui::RegionId::guide_ribbon), "narrow replaces the rail with a ribbon");
}

RIGIDBODIES_TEST("layout hysteresis and side-surface arbitration are stable")
{
    ui::LayoutInput input;
    input.viewport = { 1410, 800 };
    input.scale = 1.0f;
    input.previous_mode = ui::LayoutMode::wide;
    RIGIDBODIES_EXPECT(ui::compute_layout(input).mode == ui::LayoutMode::wide, "Wide remains selected through its hysteresis band");
    input.previous_mode = ui::LayoutMode::medium;
    RIGIDBODIES_EXPECT(ui::compute_layout(input).mode == ui::LayoutMode::medium, "Medium remains selected through the same band");
    input.viewport = { 1000, 700 };
    input.guide_available = input.guide_expanded = input.inspector_open = true;
    input.least_recently_used = ui::SideSurface::guide;
    const auto result = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(result.find(ui::RegionId::guide_rail) && result.find(ui::RegionId::inspector), "the least-recently-used side collapses to a rail");
}

RIGIDBODIES_TEST("beside a docked Guide, an Inspector with no room even for its rail appears only on request")
{
    ui::LayoutInput input;
    input.viewport = { 800, 600 };
    input.guide_available = input.guide_expanded = input.inspector_open = true;
    input.least_recently_used = ui::SideSurface::inspector;
    const auto guided = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(guided.mode == ui::LayoutMode::medium, "800 logical pixels is Medium");
    RIGIDBODIES_EXPECT(guided.find(ui::RegionId::guide_panel) != nullptr, "the Guide keeps the dock it was given");
    RIGIDBODIES_EXPECT(!guided.find(ui::RegionId::inspector) && !guided.find(ui::RegionId::inspector_rail), "an unrequested Inspector does not cover the stage");
    RIGIDBODIES_EXPECT(guided.stage.width() >= 520.0, "the stage keeps its minimum width beside the Guide");
    input.narrow_inspector_sheet = true;
    const auto* sheet = ui::compute_layout(input).find(ui::RegionId::inspector);
    RIGIDBODIES_EXPECT(sheet && sheet->presentation == ui::RegionPresentation::sheet, "a requested Inspector opens as a sheet");
}

int main()
{
    return rigidbodies::testing::run_all();
}

RIGIDBODIES_TEST("Present places the lesson caption over the stage foot and frames the subject above it")
{
    ui::LayoutInput input;
    input.viewport = { 1600, 900 };
    input.present = true;
    const auto plain = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(plain.find(ui::RegionId::present_caption) == nullptr, "no caption box without a lesson step or prompt");

    input.present_caption = true;
    const auto result = ui::compute_layout(input);
    const auto* caption = result.find(ui::RegionId::present_caption);
    RIGIDBODIES_EXPECT(caption != nullptr, "a caption box exists in Present when there is a step to show");
    if (!caption)
        return;
    RIGIDBODIES_EXPECT(caption->bounds.minimum.y >= result.stage.minimum.y && caption->bounds.maximum.y <= result.stage.maximum.y, "the caption floats inside the stage");
    RIGIDBODIES_EXPECT(caption->bounds.minimum.x > result.stage.minimum.x && caption->bounds.maximum.x < result.stage.maximum.x, "the caption is inset from the stage edges");
    RIGIDBODIES_EXPECT_NEAR(caption->bounds.minimum.x + caption->bounds.maximum.x, result.stage.minimum.x + result.stage.maximum.x, 0.01, "the caption is centred");
    RIGIDBODIES_EXPECT(caption->bounds.width() <= 920.0 + 0.01, "the caption keeps a readable line length");
    RIGIDBODIES_EXPECT(result.focus.maximum.y <= caption->bounds.minimum.y, "camera framing keeps the subject above the caption box");
    RIGIDBODIES_EXPECT(plain.focus.maximum.y > result.focus.maximum.y, "framing only gives way when a caption is shown");
    RIGIDBODIES_EXPECT(result.find(ui::RegionId::command_bar) == nullptr && result.find(ui::RegionId::status_line) == nullptr, "Present still hides the workbench chrome");
}

RIGIDBODIES_TEST("on a short projector the Present caption leaves the demonstration at least two thirds of the window")
{
    ui::LayoutInput input;
    input.present = true;
    input.present_caption = true;
    // 1280 x 720 at 150 % text and 1024 x 768 at 100 %: the classroom sizes Present is for.
    for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 1280, 720 }, 1.5f }, std::pair<render::ViewportSize, float> { { 1024, 768 }, 1.0f }, std::pair<render::ViewportSize, float> { { 1280, 720 }, 1.0f } })
    {
        input.viewport = size;
        input.scale = scale;
        const auto result = ui::compute_layout(input);
        const auto* caption = result.find(ui::RegionId::present_caption);
        RIGIDBODIES_EXPECT(caption != nullptr, "a caption box exists");
        if (caption)
            RIGIDBODIES_EXPECT(caption->bounds.height() <= 0.35 * size.height + 0.5, "the caption box is at most about a third of the window height");
    }
}

RIGIDBODIES_TEST("a narrow window's dialogs keep a gutter on every side")
{
    ui::LayoutInput input;
    input.viewport = { 700, 900 };
    const auto result = ui::compute_layout(input);
    const auto* modal = result.find(ui::RegionId::modal);
    RIGIDBODIES_EXPECT(result.mode == ui::LayoutMode::narrow && modal != nullptr, "a narrow window publishes the dialog box");
    if (modal)
        RIGIDBODIES_EXPECT(modal->bounds.minimum.x >= 16.0 - 0.01 && modal->bounds.maximum.x <= 700.0 - 16.0 + 0.01 && modal->bounds.minimum.y >= 16.0 - 0.01 && modal->bounds.maximum.y <= 900.0 - 16.0 + 0.01, "the dialog box stops 16 dp short of each window edge");
}

RIGIDBODIES_TEST("a narrow window shows one side sheet at a time, the most recently used")
{
    ui::LayoutInput input;
    input.viewport = { 600, 800 };
    input.guide_available = true;
    input.guide_expanded = true;
    input.inspector_open = true;
    const auto unrequested = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(!unrequested.find(ui::RegionId::inspector) && !unrequested.find(ui::RegionId::guide_panel), "surfaces docked open in wider windows do not cover a narrow stage unasked");
    input.narrow_guide_sheet = true;
    input.narrow_inspector_sheet = true;
    input.least_recently_used = ui::SideSurface::guide;
    const auto inspector_recent = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(inspector_recent.mode == ui::LayoutMode::narrow, "600 logical pixels is Narrow");
    RIGIDBODIES_EXPECT(inspector_recent.find(ui::RegionId::inspector) != nullptr && inspector_recent.find(ui::RegionId::guide_panel) == nullptr, "the inspector, used last, is the only sheet");
    input.least_recently_used = ui::SideSurface::inspector;
    const auto guide_recent = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(guide_recent.find(ui::RegionId::guide_panel) != nullptr && guide_recent.find(ui::RegionId::inspector) == nullptr, "the guide, used last, is the only sheet");
    RIGIDBODIES_EXPECT(guide_recent.find(ui::RegionId::guide_ribbon) == nullptr, "the ribbon steps aside while its own sheet is open; the sheet header closes it");
    RIGIDBODIES_EXPECT(inspector_recent.find(ui::RegionId::guide_ribbon) != nullptr, "with the inspector sheet open the ribbon remains the way to the guide");
}

RIGIDBODIES_TEST("the docked Measure drawer takes a share of the window until the learner drags it")
{
    ui::LayoutInput input;
    input.viewport = { 1920, 1080 };
    input.measure_open = true;
    const auto wide = ui::compute_layout(input);
    const auto* drawer = wide.find(ui::RegionId::measure_drawer);
    RIGIDBODIES_EXPECT(drawer && drawer->presentation == ui::RegionPresentation::docked, "a large window docks the drawer");
    RIGIDBODIES_EXPECT(drawer && std::abs(drawer->bounds.height() - 0.4 * (1080.0 - 44.0)) < 0.01, "the default drawer is 40 percent of the height below the command bar");
    RIGIDBODIES_EXPECT(wide.stage.maximum.y <= drawer->bounds.minimum.y + 0.01, "the docked drawer shortens the stage instead of covering it");

    input.measure_height_logical = 250.0;
    const auto dragged = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(std::abs(dragged.find(ui::RegionId::measure_drawer)->bounds.height() - 250.0) < 0.01, "a dragged height is kept");
    input.measure_height_logical = 5000.0;
    RIGIDBODIES_EXPECT(ui::compute_layout(input).find(ui::RegionId::measure_drawer)->bounds.height() <= 0.5 * (1080.0 - 44.0) + 0.01, "a dragged height is still limited to half the window");
}

RIGIDBODIES_TEST("a Measure sheet is resized from its top edge like the docked drawer")
{
    ui::LayoutInput input;
    input.viewport = { 600, 600 };
    input.measure_open = true;
    const auto sheet = ui::compute_layout(input);
    const auto* region = sheet.find(ui::RegionId::measure_drawer);
    RIGIDBODIES_EXPECT(region && region->presentation == ui::RegionPresentation::sheet && std::abs(region->bounds.height() - 0.45 * 600.0) < 0.01, "a narrow window opens Measure as a sheet of under half the window");
    if (!region)
        return;
    // The grip stores the top edge's distance from the window bottom, as for the docked drawer.
    input.measure_height_logical = 600.0 - region->bounds.minimum.y + 80.0;
    const auto taller = ui::compute_layout(input).find(ui::RegionId::measure_drawer);
    RIGIDBODIES_EXPECT(taller && std::abs(taller->bounds.minimum.y - (region->bounds.minimum.y - 80.0)) < 0.01, "dragging the top edge up raises the sheet's top edge with it");
    input.measure_height_logical = 5000.0;
    const auto tallest = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(tallest.find(ui::RegionId::measure_drawer)->bounds.minimum.y >= tallest.stage.minimum.y + 95.0, "some of the stage stays in view above the tallest sheet");
    input.measure_height_logical = 1.0;
    RIGIDBODIES_EXPECT(ui::compute_layout(input).find(ui::RegionId::measure_drawer)->bounds.height() >= 159.9, "the sheet keeps room for its tabs and a few rows");
}

RIGIDBODIES_TEST("framing keeps the subject above an open Measure sheet")
{
    for (const auto& [width, height] : { std::pair { 900, 900 }, std::pair { 1800, 780 } })
    {
        ui::LayoutInput input;
        input.viewport = { width, height };
        input.scale = 1.5f;
        const auto closed = ui::compute_layout(input);
        input.measure_open = true;
        const auto open = ui::compute_layout(input);
        const auto* sheet = open.find(ui::RegionId::measure_drawer);
        RIGIDBODIES_EXPECT(sheet && sheet->presentation == ui::RegionPresentation::sheet, "a narrow or short window opens Measure as a sheet");
        if (!sheet)
            continue;
        RIGIDBODIES_EXPECT(open.focus.maximum.y <= sheet->bounds.minimum.y, "the framed area ends above the sheet");
        RIGIDBODIES_EXPECT(open.focus.height() > 0.0 && open.focus.maximum.y < closed.focus.maximum.y, "framing gives way only while the sheet is open");
        RIGIDBODIES_EXPECT(open.stage.height() == closed.stage.height(), "the sheet still overlays a full-height stage");
    }
}

RIGIDBODIES_TEST("the library sheet opens beneath the toolbar so transport stays reachable")
{
    for (const auto draw_bar : { false, true })
    {
        ui::LayoutInput input;
        input.viewport = { 1600, 900 };
        input.draw_bar = draw_bar;
        const auto result = ui::compute_layout(input);
        const auto* library = result.find(ui::RegionId::library_sheet);
        const auto* toolbar = result.find(draw_bar ? ui::RegionId::draw_bar : ui::RegionId::command_bar);
        RIGIDBODIES_EXPECT(library && toolbar && library->bounds.minimum.y >= toolbar->bounds.maximum.y - 0.01, "the library starts below the toolbar");
        RIGIDBODIES_EXPECT(library && library->bounds.maximum.y <= 900.0 + 0.01, "the library ends at the window bottom");
    }
}

RIGIDBODIES_TEST("as the window's title bar the top strip gives its ends to the app mark and the window controls")
{
    ui::LayoutInput input;
    input.viewport = { 1600, 900 };
    input.scale = 1.5f;
    const auto without = ui::compute_layout(input);
    RIGIDBODIES_EXPECT(!without.find(ui::RegionId::title_bar) && !without.find(ui::RegionId::window_brand) && !without.find(ui::RegionId::window_controls), "the platform's title bar leaves the strip as it was");
    input.window_controls = true;
    for (const auto present : { false, true })
        for (const auto draw_bar : { false, true })
        {
            input.present = present;
            input.draw_bar = draw_bar;
            const auto result = ui::compute_layout(input);
            const auto strip_height = (present ? 56.0 : 44.0) * input.scale;
            const auto* title_bar = result.find(ui::RegionId::title_bar);
            const auto* brand = result.find(ui::RegionId::window_brand);
            const auto* controls = result.find(ui::RegionId::window_controls);
            const auto* strip = result.find(present ? ui::RegionId::present_strip : ui::RegionId::command_bar);
            RIGIDBODIES_EXPECT(title_bar && brand && controls && strip, "the title bar, the mark, the controls and the strip are all laid out");
            if (!title_bar || !brand || !controls || !strip)
                continue;
            RIGIDBODIES_EXPECT_NEAR(title_bar->bounds.width(), 1600.0, 0.01, "one surface spans the window");
            RIGIDBODIES_EXPECT_NEAR(title_bar->bounds.height(), strip_height, 0.01, "at the strip's height, not the draw bar's");
            RIGIDBODIES_EXPECT_NEAR(controls->bounds.height(), strip_height, 0.01, "the controls stand only as tall as the strip");
            RIGIDBODIES_EXPECT_NEAR(controls->bounds.maximum.x, 1600.0, 0.01, "the controls reach the window's right edge");
            RIGIDBODIES_EXPECT_NEAR(controls->bounds.width(), (3.0 * 46.0 + 17.0) * input.scale, 0.01, "three 46 dp caption buttons after their divider");
            RIGIDBODIES_EXPECT_NEAR(brand->bounds.minimum.x, 0.0, 0.01, "the mark starts the strip");
            RIGIDBODIES_EXPECT_NEAR(strip->bounds.minimum.x, brand->bounds.maximum.x, 0.01, "the strip's content follows the mark");
            RIGIDBODIES_EXPECT_NEAR(strip->bounds.maximum.x, controls->bounds.minimum.x, 0.01, "and ends where the controls begin");
            if (const auto* bar = result.find(ui::RegionId::draw_bar))
                RIGIDBODIES_EXPECT_NEAR(bar->bounds.width(), 1600.0, 0.01, "the draw bar beneath still spans the window");
            const auto* modal = result.find(ui::RegionId::modal);
            RIGIDBODIES_EXPECT(modal && modal->bounds.minimum.y >= strip_height + 8.0 * input.scale - 0.01, "a dialog never covers the window controls");
            RIGIDBODIES_EXPECT(result.stage.minimum.y >= strip_height - 0.01, "the stage begins below the strip");
        }
    input.present = false;
    input.draw_bar = false;
    input.viewport = { 600, 700 };
    input.scale = 1.0f;
    const auto narrow = ui::compute_layout(input);
    const auto* modal = narrow.find(ui::RegionId::modal);
    RIGIDBODIES_EXPECT(narrow.mode == ui::LayoutMode::narrow && modal && modal->bounds.minimum.y >= 52.0 - 0.01, "a narrow window's dialog starts below the controls too");
    const auto stage_without = [&]
    {
        auto plain = input;
        plain.window_controls = false;
        return ui::compute_layout(plain).stage;
    }();
    RIGIDBODIES_EXPECT(narrow.stage.minimum.y == stage_without.minimum.y && narrow.stage.height() == stage_without.height(), "the stage keeps its place either way");
}
