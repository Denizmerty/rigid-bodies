#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/scenario_document.hpp>
#ifdef RIGIDBODIES_HAS_RMLUI
#include <rigidbodies/ui/document_backend.hpp>
#else
#include <rigidbodies/ui/overlay_backend.hpp>
#endif

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
    using namespace rigidbodies;

    render::ScreenRect backend_focus()
    {
#ifdef RIGIDBODIES_HAS_RMLUI
        ui::DocumentBackend backend;
#else
        ui::OverlayBackend backend;
#endif
        ui::LayoutInput input;
        input.viewport = { 1600, 900 };
        input.scale = 1.5f;
        const auto focus = ui::compute_layout(input).focus;
        return { focus.minimum.x, focus.minimum.y, focus.width(), focus.height() };
    }

    render::ScreenRect focus_of(const ui::LayoutInput& input)
    {
        const auto focus = ui::compute_layout(input).focus;
        return { focus.minimum.x, focus.minimum.y, focus.width(), focus.height() };
    }

    struct StageLayout
    {
        std::string name;
        ui::LayoutInput input;
    };

    // The windows a lesson is checked in: a laptop at 150 % with the Inspector or the Guide docked,
    // a full-HD monitor with both side panels, a small window, and Present mode with its caption
    // on the laptop and on a 720p projector, at standard and at large text.
    std::vector<StageLayout> lesson_layouts()
    {
        std::vector<StageLayout> layouts(7);
        layouts[0].name = "1600x900 at 1.5";
        layouts[0].input.viewport = { 1600, 900 };
        layouts[0].input.scale = 1.5f;
        layouts[0].input.guide_available = true;
        layouts[1].name = "1920x1080 at 1";
        layouts[1].input.viewport = { 1920, 1080 };
        layouts[1].input.guide_available = true;
        layouts[1].input.guide_expanded = true;
        layouts[2].name = "800x600 at 1";
        layouts[2].input.viewport = { 800, 600 };
        layouts[2].input.guide_available = true;
        layouts[3].name = "Present at 1600x900 at 1.5";
        layouts[3].input.viewport = { 1600, 900 };
        layouts[3].input.scale = 1.5f;
        layouts[3].input.present = true;
        layouts[3].input.present_caption = true;
        layouts[4].name = "1600x900 at 1.5 with the Guide docked";
        layouts[4].input = layouts[0].input;
        layouts[4].input.guide_expanded = true;
        layouts[4].input.least_recently_used = ui::SideSurface::inspector;
        layouts[5].name = "Present at 1280x720 at 1";
        layouts[5].input.viewport = { 1280, 720 };
        layouts[5].input.present = true;
        layouts[5].input.present_caption = true;
        layouts[6].name = "Present at 1280x720 with 150 % text";
        layouts[6].input = layouts[5].input;
        layouts[6].input.scale = 1.5f;
        return layouts;
    }

    void expect_subject_inside_focus(const app::SimulationSession& session, const render::ScreenRect& focus)
    {
        bool found = false;
        session.world().for_each_body([&](physics::BodyId, const physics::RigidBody& body)
            {
                if (body.type() == physics::BodyType::static_body)
                    return;
                found = true;
                const auto bounds = body.compute_bounds(body.interpolated_transform(session.scene_settings().interpolation_alpha));
                for (const auto point : { bounds.minimum, bounds.maximum, math::Vec2 { bounds.minimum.x, bounds.maximum.y }, math::Vec2 { bounds.maximum.x, bounds.minimum.y } })
                {
                    const auto screen = session.camera().world_to_screen(point);
                    RIGIDBODIES_EXPECT(screen.x >= focus.left - 1.0e-8 && screen.x <= focus.left + focus.width + 1.0e-8, "moving subject is horizontally inside focus");
                    RIGIDBODIES_EXPECT(screen.y >= focus.top - 1.0e-8 && screen.y <= focus.top + focus.height + 1.0e-8, "moving subject is vertically inside focus");
                }
            });
        RIGIDBODIES_EXPECT(found, "catalogue experiment has a free or driven subject");
        const auto visible_focus_height = session.camera().view_height_m() * focus.height / session.camera().viewport().height;
        RIGIDBODIES_EXPECT(visible_focus_height >= 1.0 - 1.0e-8 && visible_focus_height <= 500.0 + 1.0e-8, "automatic subject framing respects visible-height limits");

        const auto subject = session.subject_bounds();
        RIGIDBODIES_EXPECT(!subject.is_empty(), "catalogue subject bounds are available");
        const auto minimum = session.camera().world_to_screen(subject.minimum);
        const auto maximum = session.camera().world_to_screen(subject.maximum);
        const auto width_fraction = std::abs(maximum.x - minimum.x) / focus.width;
        const auto height_fraction = std::abs(maximum.y - minimum.y) / focus.height;
        if (visible_focus_height > 1.0 + 1.0e-8)
            RIGIDBODIES_EXPECT(std::max(width_fraction, height_fraction) >= 0.84 - 1.0e-8, "subject fills at least 84 percent of the limiting focus dimension");
    }

    RIGIDBODIES_TEST("every catalogue experiment frames its subject into the published focus rectangle")
    {
        const auto reference = backend_focus();
#ifdef RIGIDBODIES_HAS_RMLUI
        RIGIDBODIES_EXPECT(reference.left == 36.0 && reference.top == 102.0 && reference.width == 1108.0 && reference.height == 732.0, "document backend supplies the prescribed reference focus rectangle");
#endif
        for (const auto& layout : lesson_layouts())
        {
            const auto focus = focus_of(layout.input);
            app::SimulationSession session;
            session.set_viewport(layout.input.viewport);
            session.set_focus_rect(focus);
            session.configure({});
            session.set_focus_rect(focus);
            for (const auto& scenario : physics::available_scenarios())
            {
                RIGIDBODIES_EXPECT(session.load_scenario(scenario.id), "catalogue experiment loads");
                // A relativity experiment's subject is its probe, not a World body; its own framing
                // is checked against the band and the apparatus.
                if (session.relativity_active())
                    continue;
                if (scenario.id == "empty_lab")
                {
                    const auto ids = session.world().body_ids();
                    RIGIDBODIES_EXPECT(ids.size() == 1 && session.world().find_body(ids.front())->type() == physics::BodyType::static_body,
                        "Empty lab intentionally frames its lone fixed floor rather than inventing a moving subject");
                    continue;
                }
                expect_subject_inside_focus(session, focus);
                ui::UiCommand frame;
                frame.kind = ui::UiCommandKind::frame_subject;
                session.apply(frame);
                expect_subject_inside_focus(session, focus);
                const auto camera_before_reset = session.camera();
                session.reset_scenario();
                RIGIDBODIES_EXPECT(session.camera().center_m() == camera_before_reset.center_m() && session.camera().view_height_m() == camera_before_reset.view_height_m(), "Reset keeps the framed camera");
            }
        }
    }

    bool bounds_inside_focus(const app::SimulationSession& session, const math::Aabb& bounds, const render::ScreenRect& focus)
    {
        for (const auto point : { bounds.minimum, bounds.maximum, math::Vec2 { bounds.minimum.x, bounds.maximum.y }, math::Vec2 { bounds.maximum.x, bounds.minimum.y } })
        {
            const auto screen = session.camera().world_to_screen(point);
            if (screen.x < focus.left - 1.0e-6 || screen.x > focus.left + focus.width + 1.0e-6 || screen.y < focus.top - 1.0e-6 || screen.y > focus.top + focus.height + 1.0e-6)
                return false;
        }
        return true;
    }

    // Names of the bodies (all moving ones for an empty list) that must be fully in view.
    void expect_in_view(const app::SimulationSession& session, const render::ScreenRect& focus, std::string_view scenario, const std::vector<std::string_view>& names, std::string_view when)
    {
        std::size_t checked = 0;
        session.world().for_each_body([&](physics::BodyId, const physics::RigidBody& body)
            {
                const auto named = std::find(names.begin(), names.end(), body.name()) != names.end();
                if (names.empty() ? body.type() == physics::BodyType::static_body : !named)
                    return;
                ++checked;
                if (!bounds_inside_focus(session, body.compute_bounds(), focus))
                    RIGIDBODIES_FAIL(std::string(scenario) + ": " + std::string(body.name()) + " leaves the recommended view " + std::string(when));
            });
        RIGIDBODIES_EXPECT(checked > 0 && (names.empty() || checked == names.size()), "every named body exists");
    }

    RIGIDBODIES_TEST("the recommended view keeps each lesson's apparatus and its motion in view")
    {
        struct Lesson
        {
            std::string_view scenario;
            std::vector<std::string_view> at_start, while_running;
            int frames { 80 };
        };
        const std::vector<std::string_view> lanes { "friction_lane_1", "friction_lane_2", "friction_lane_3", "friction_lane_4", "slide_along_grain", "slide_across_grain", "ideal_rolling_ball", "resisted_rolling_ball" };
        const std::vector<Lesson> lessons {
            { "fast_projectile", { "fast_ball", "thin_target_wall" }, { "thin_target_wall" } },
            // The free ball is the control: it lands on the floor and stays beside the other.
            { "targeted_force", { "attracted_ball", "free_ball", "attraction_point" }, { "attracted_ball", "free_ball", "attraction_point" } },
            { "targeted_force", { "attracted_ball", "free_ball", "attraction_point" }, { "attracted_ball", "free_ball", "attraction_point" }, 300 },
            { "magnus_effect", {}, {} },
            { "distance_chain", {}, {} },
            { "prismatic_drive", {}, {} },
            { "ramp", { "block", "ramp" }, { "block", "ramp" } },
            { "welded_assembly", {}, {} },
            { "collision_comparison", {}, {} },
            // The elastic pair, the reference row, stays in view for over a second and a half
            // after the discs part.
            { "collision_comparison", {}, {}, 115 },
            // Every lane is in view with its end stop, and every object is still on its lane
            // well after the ideal ball has rolled to its stop.
            { "friction_comparison", lanes, lanes },
            { "friction_comparison", lanes, lanes, 360 },
            { "compound_object", { "hammer" }, { "hammer" } },
            { "free_fall", {}, {} },
        };
        for (const auto& layout : lesson_layouts())
        {
            const auto focus = focus_of(layout.input);
            for (const auto& lesson : lessons)
            {
                app::SimulationSession session;
                session.set_viewport(layout.input.viewport);
                session.set_focus_rect(focus);
                session.configure({});
                RIGIDBODIES_EXPECT(session.load_scenario(lesson.scenario), "lesson loads");
                expect_in_view(session, focus, lesson.scenario, lesson.at_start, "at the start in " + layout.name);
                session.stepper().set_paused(false);
                for (int frame = 0; frame < lesson.frames; ++frame)
                    session.advance(1.0 / 60.0);
                expect_in_view(session, focus, lesson.scenario, lesson.while_running, "after " + std::to_string(lesson.frames) + " frames in " + layout.name);
            }
        }
    }

    RIGIDBODIES_TEST("every experiment keeps its moving objects in view at the start and while it runs")
    {
        for (const auto& layout : lesson_layouts())
        {
            const auto focus = focus_of(layout.input);
            app::SimulationSession session;
            session.set_viewport(layout.input.viewport);
            session.set_focus_rect(focus);
            session.configure({});
            for (const auto& scenario : physics::available_scenarios())
            {
                // The empty lab has nothing that moves, and the fast ball may rebound off its wall
                // or pass through it out of view; the lesson check above keeps the wall in view.
                if (scenario.id == "empty_lab" || scenario.id == "fast_projectile")
                    continue;
                RIGIDBODIES_EXPECT(session.load_scenario(scenario.id), "catalogue experiment loads");
                // A relativity experiment moves its probe, not a World body.
                if (session.relativity_active())
                    continue;
                expect_in_view(session, focus, scenario.id, {}, "at the start in " + layout.name);
                session.stepper().set_paused(false);
                for (int frame = 0; frame < 80; ++frame)
                    session.advance(1.0 / 60.0);
                expect_in_view(session, focus, scenario.id, {}, "after 80 frames in " + layout.name);
                session.stepper().set_paused(true);
            }
        }
    }

    RIGIDBODIES_TEST("free fall's balls are drawn large enough to tell apart")
    {
        const auto layouts = lesson_layouts();
        for (const auto& [index, smallest_px] : { std::pair { std::size_t { 0 }, 20.0 }, std::pair { std::size_t { 1 }, 20.0 }, std::pair { std::size_t { 5 }, 10.0 } })
        {
            const auto& layout = layouts[index];
            const auto focus = focus_of(layout.input);
            app::SimulationSession session;
            session.set_viewport(layout.input.viewport);
            session.set_focus_rect(focus);
            session.configure({});
            RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "free fall loads");
            std::size_t balls = 0;
            session.world().for_each_body([&, minimum_px = smallest_px](physics::BodyId, const physics::RigidBody& body)
                {
                    if (body.type() != physics::BodyType::dynamic_body)
                        return;
                    ++balls;
                    const auto bounds = body.compute_bounds();
                    const auto across_px = session.camera().world_to_screen_length(bounds.maximum.x - bounds.minimum.x);
                    RIGIDBODIES_EXPECT(across_px >= minimum_px, std::string(body.name()) + " is at least " + std::to_string(static_cast<int>(minimum_px)) + " px across in " + layout.name);
                });
            RIGIDBODIES_EXPECT(balls == 3, "free fall drops three balls");
        }
    }

    RIGIDBODIES_TEST("a floating subject is framed without a floor it never reaches, a falling one with it")
    {
        const auto focus = backend_focus();
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(focus);
        session.configure({});
        const auto top_of = [&](std::string_view name)
        {
            double top = -1.0e9;
            session.world().for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    if (body.name() == name)
                        top = body.compute_bounds().maximum.y;
                });
            return top;
        };
        // The hammer spins in place without gravity, 1.5 m above the ground.
        RIGIDBODIES_EXPECT(session.load_scenario("compound_object"), "compound object loads");
        RIGIDBODIES_EXPECT(session.subject_bounds().minimum.y > top_of("ground") + 0.5, "the ground the hammer never reaches stays out of its frame");
        const auto framed_height = session.camera().view_height_m() * focus.height / 900.0;
        RIGIDBODIES_EXPECT(framed_height <= 1.0 + 1.0e-9, "the hammer is framed at the closest recommended view");
        // Falling balls land on the ground, so it belongs to their frame however long the fall.
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "free fall loads");
        RIGIDBODIES_EXPECT(session.subject_bounds().minimum.y <= top_of("ground"), "the ground the balls land on is framed");
        RIGIDBODIES_EXPECT(session.load_scenario("asymmetric_aerodynamics"), "slow fall loads");
        RIGIDBODIES_EXPECT(session.subject_bounds().minimum.y <= top_of("ground"), "a slow fall that lands after the look-ahead still frames its ground");
    }

    RIGIDBODIES_TEST("a slider's whole travel and a struck fixture are framed without an authored view")
    {
        const auto focus = backend_focus();
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(focus);
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("prismatic_drive"), "slider lesson loads");
        // The carriage is 0.3 m wide and its stops sit 0.9 m either side of the rail's centre.
        for (const auto x : { -1.05, 1.05 })
        {
            const auto screen = session.camera().world_to_screen({ x, 0.14 });
            RIGIDBODIES_EXPECT(screen.x >= focus.left && screen.x <= focus.left + focus.width, "the carriage at either stop is in view");
        }
        RIGIDBODIES_EXPECT(session.load_scenario("fast_projectile"), "projectile lesson loads");
        const auto wall = session.camera().world_to_screen({ 1.2, 0.7 });
        RIGIDBODIES_EXPECT(wall.x <= focus.left + focus.width && wall.y >= focus.top, "the wall the ball is fired at is in view");
        const auto before = session.camera();
        session.set_focus_rect({ focus.left, focus.top, focus.width - 200.0, focus.height });
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(session.camera().center_m() == before.center_m() && session.camera().view_height_m() == before.view_height_m(), "reframing an unchanged world gives the identical view");
    }

    RIGIDBODIES_TEST("an authored view fits its width as well as its height")
    {
        app::SimulationSession session;
        session.set_viewport({ 900, 900 });
        const render::ScreenRect narrow { 0.0, 60.0, 500.0, 800.0 };
        session.set_focus_rect(narrow);
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("magnus_effect"), "Magnus lesson loads");
        // magnus_effect authors a 2.9 m wide, 1.6 m tall view centred at (-0.4, 0).
        const auto left = session.camera().world_to_screen({ -1.85, 0.0 });
        const auto right = session.camera().world_to_screen({ 1.05, 0.0 });
        RIGIDBODIES_EXPECT_NEAR(left.x, narrow.left, 0.5, "the authored left edge meets the stage's left edge");
        RIGIDBODIES_EXPECT_NEAR(right.x, narrow.left + narrow.width, 0.5, "the authored right edge meets the stage's right edge");
    }

    RIGIDBODIES_TEST("the view height readout describes the stage the reader sees")
    {
        const auto focus = backend_focus();
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(focus);
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("empty_lab"), "lab loads");
        const auto framed = session.camera().screen_to_world({ 0.0, focus.top }).y - session.camera().screen_to_world({ 0.0, focus.top + focus.height }).y;
        RIGIDBODIES_EXPECT_NEAR(session.build_model().view_height_m, framed, 1.0e-9, "without a known stage the readout is the height of the framed area, not of the window");
        // The whole stage between the toolbar and the status line is in view, not only the inset
        // area framing fills.
        ui::LayoutInput input;
        input.viewport = { 1600, 900 };
        input.scale = 1.5f;
        const auto layout = ui::compute_layout(input);
        const auto* status = layout.find(ui::RegionId::status_line);
        RIGIDBODIES_EXPECT(status != nullptr, "the layout has a status line");
        const render::ScreenRect visible { layout.stage.minimum.x, layout.stage.minimum.y, layout.stage.width(), status->bounds.minimum.y - layout.stage.minimum.y };
        session.set_visible_stage_rect(visible);
        const auto shown = session.camera().screen_to_world({ 0.0, visible.top }).y - session.camera().screen_to_world({ 0.0, visible.top + visible.height }).y;
        RIGIDBODIES_EXPECT(shown > framed, "the visible stage is taller than the framed area");
        RIGIDBODIES_EXPECT_NEAR(session.build_model().view_height_m, shown, 1.0e-9, "the readout is the height of the stage the reader sees");
        const auto centre = session.camera().screen_to_world(focus.center());
        ui::UiCommand height;
        height.kind = ui::UiCommandKind::set_view_height;
        height.value = 3.0;
        session.apply(height);
        RIGIDBODIES_EXPECT_NEAR(session.build_model().view_height_m, 3.0, 1.0e-9, "typing a height shows exactly that much stage");
        const auto typed = session.camera().screen_to_world({ 0.0, visible.top }).y - session.camera().screen_to_world({ 0.0, visible.top + visible.height }).y;
        RIGIDBODIES_EXPECT_NEAR(typed, 3.0, 1.0e-9, "the visible stage spans the typed height from top to bottom");
        const auto kept = session.camera().screen_to_world(focus.center());
        RIGIDBODIES_EXPECT_NEAR(kept.x, centre.x, 1.0e-9, "the stage keeps its centre");
        RIGIDBODIES_EXPECT_NEAR(kept.y, centre.y, 1.0e-9, "the stage keeps its centre");
    }

    RIGIDBODIES_TEST("a side sheet over a small stage moves the framing into the stage it leaves uncovered")
    {
        for (const auto guide : { false, true })
        {
            ui::LayoutInput input;
            input.viewport = { 960, 1080 };
            input.scale = 1.5f;
            input.guide_available = true;
            input.narrow_guide_sheet = guide;
            input.narrow_inspector_sheet = !guide;
            input.least_recently_used = guide ? ui::SideSurface::inspector : ui::SideSurface::guide;
            const auto layout = ui::compute_layout(input);
            const auto* sheet = layout.find(guide ? ui::RegionId::guide_panel : ui::RegionId::inspector);
            RIGIDBODIES_EXPECT(layout.mode == ui::LayoutMode::narrow && sheet != nullptr && sheet->presentation == ui::RegionPresentation::sheet, "a narrow window shows the side surface as a sheet");
            if (guide)
                RIGIDBODIES_EXPECT(layout.focus.minimum.x >= sheet->bounds.maximum.x, "framing starts right of the Guide sheet");
            else
                RIGIDBODIES_EXPECT(layout.focus.maximum.x <= sheet->bounds.minimum.x, "framing ends left of the Inspector sheet");
            RIGIDBODIES_EXPECT(layout.focus.width() >= 160.0 * 1.5, "the uncovered stage still has room to frame");
            input.narrow_guide_sheet = input.narrow_inspector_sheet = false;
            RIGIDBODIES_EXPECT(ui::compute_layout(input).focus.width() > layout.focus.width() + 300.0, "closing the sheet gives the framing back the whole stage");
        }
        // Beside a sheet over almost the whole of a tiny window there is nothing left to frame in.
        ui::LayoutInput tiny;
        tiny.viewport = { 420, 700 };
        tiny.narrow_inspector_sheet = true;
        const auto covered = ui::compute_layout(tiny);
        tiny.narrow_inspector_sheet = false;
        RIGIDBODIES_EXPECT(covered.focus.width() == ui::compute_layout(tiny).focus.width(), "a sheet over a tiny window leaves the framing alone");
    }

    RIGIDBODIES_TEST("a sheet opened over the selection keeps it in view without undoing the reader's own view")
    {
        ui::LayoutInput input;
        input.viewport = { 960, 1080 };
        input.scale = 1.5f;
        input.guide_available = true;
        const auto open_focus = [&](bool sheet)
        {
            input.narrow_inspector_sheet = sheet;
            return focus_of(input);
        };
        const auto inside = [](const app::SimulationSession& session, physics::BodyId id, const render::ScreenRect& focus)
        {
            const auto bounds = session.world().find_body(id)->compute_bounds();
            const auto a = session.camera().world_to_screen(bounds.minimum), b = session.camera().world_to_screen(bounds.maximum);
            return std::min(a.x, b.x) >= focus.left - 1.0e-6 && std::max(a.x, b.x) <= focus.left + focus.width + 1.0e-6 && std::min(a.y, b.y) >= focus.top - 1.0e-6 && std::max(a.y, b.y) <= focus.top + focus.height + 1.0e-6;
        };
        app::SimulationSession session;
        session.set_viewport(input.viewport);
        session.set_focus_rect(open_focus(false));
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("ramp"), "ramp loads");
        physics::BodyId block;
        session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.name() == "block")
                    block = id;
            });
        session.set_selection(block);
        // The recommended view reframes the whole apparatus into the uncovered stage.
        session.set_focus_rect(open_focus(true));
        RIGIDBODIES_EXPECT(!session.camera_user_moved() && inside(session, block, open_focus(true)), "the block stays in view beside the sheet");
        // A reader who moved the view keeps it: only a pan that brings the block back out from
        // under the sheet is added, and no zoom.
        session.set_focus_rect(open_focus(false));
        const auto block_px = session.camera().world_to_screen(session.world().find_body(block)->position_m());
        const auto full = open_focus(false);
        session.pan_view({ full.left + full.width - 40.0 - block_px.x, 0.0 });
        const auto height = session.camera().view_height_m();
        RIGIDBODIES_EXPECT(!inside(session, block, open_focus(true)) && session.camera_user_moved(), "the reader moved the block to where the sheet will open");
        session.set_focus_rect(open_focus(true));
        RIGIDBODIES_EXPECT(inside(session, block, open_focus(true)), "the block is moved out from under the sheet");
        RIGIDBODIES_EXPECT(session.camera().view_height_m() == height && session.camera_user_moved(), "the reader's zoom is kept");
        // A selection the reader had already moved out of sight is left there.
        session.set_focus_rect(open_focus(false));
        session.pan_view({ -5000.0, 0.0 });
        const auto away = session.camera().world_to_screen(session.world().find_body(block)->position_m());
        session.set_focus_rect(open_focus(true));
        const auto still = session.camera().world_to_screen(session.world().find_body(block)->position_m());
        RIGIDBODIES_EXPECT(!inside(session, block, open_focus(true)) && std::abs((still.y - away.y)) < 1.0e-6, "an off-stage selection is not pulled back");
    }

    // The text pixel scale the stage was last drawn at, by the renderer's own rule. Every check
    // below measures the framing with it, so a session that framed with any other scale fails them.
    double stage_text_pixel_scale(const app::SimulationSession& session)
    {
        return render::stage_text_pixel_scale(session.scene_settings());
    }

    // Draws a frame and returns the tier the relativity stage was handed.
    render::RelativityStageTier drawn_tier(app::SimulationSession& session)
    {
        render::DrawList list;
        session.render(list);
        const auto& stage = session.scene_renderer().relativity_stage();
        RIGIDBODIES_EXPECT(stage.has_value(), "the renderer holds the relativity stage");
        return stage ? stage->tier : render::RelativityStageTier::full;
    }

    // Relativity framing leaves the instrument band across the top of the focus area, the
    // apparatus's stack above and below the rail under it, and the track and its margins across
    // the width, all worked out from the same free functions the stage draws with. Returns whether
    // the band and the whole stack fit; when they cannot, everything above the rail still keeps
    // clear of the band.
    bool expect_relativity_framing(app::SimulationSession& session, const render::ScreenRect& focus, std::string_view when)
    {
        const auto tier = drawn_tier(session);
        const auto tps = stage_text_pixel_scale(session);
        const auto units = session.scene_settings().display_units;
        const auto& camera = session.camera();
        const auto ppm = camera.pixels_per_metre();
        const auto band = render::relativity_band_height_px(focus.width, tps, units);
        const auto stack = render::relativity_stack_px(tier, tps, render::relativity_rail_thickness_m * ppm);
        const auto start = camera.world_to_screen({ 0.0, 0.0 });
        const auto finish = camera.world_to_screen({ physics::relativity_track_length_m, 0.0 });
        const auto context = " " + std::string(when);
        RIGIDBODIES_EXPECT(tier == render::choose_relativity_tier(focus.width, focus.height, tps, units, tier), "the stage is drawn in the tier its size calls for" + context);
        RIGIDBODIES_EXPECT_NEAR(ppm, std::min((focus.width - 32.0 * tps) / render::relativity_view_width_m, focus.height), 1.0e-9 * ppm, "the track and its margins fill the width, or a metre the height" + context);
        RIGIDBODIES_EXPECT_NEAR(0.5 * (start.x + finish.x), focus.left + 0.5 * focus.width, 1.0e-6, "the track is centred across the stage" + context);
        RIGIDBODIES_EXPECT(start.x >= focus.left + 16.0 * tps - 1.0e-6 && finish.x <= focus.left + focus.width - 16.0 * tps + 1.0e-6, "both ends of the track are inside the stage with room for their labels" + context);
        const auto slack = focus.height - band - stack.above_px - stack.below_px;
        const auto fits = slack >= -1.0e-6;
        RIGIDBODIES_EXPECT_NEAR(start.y, focus.top + band + stack.above_px + std::max(0.0, slack) * 0.5, 1.0e-6, "the rail sits below the band and the apparatus above it, in the middle of the room left" + context);
        if (fits)
            RIGIDBODIES_EXPECT(start.y + stack.below_px <= focus.top + focus.height + 1.0e-6, "the lane, marks and caption below the rail end inside the stage" + context);
        const auto visible_height_m = focus.height / ppm;
        RIGIDBODIES_EXPECT(visible_height_m >= 1.0 - 1.0e-9 && visible_height_m <= 500.0 + 1.0e-9, "the stage shows between 1 m and 500 m of height" + context);
        return fits;
    }

    bool same_camera(const render::Camera2D& a, const render::Camera2D& b)
    {
        return a.center_m() == b.center_m() && a.view_height_m() == b.view_height_m();
    }

    // What the stage last drew lies inside the focus area: the band across its top, the track's
    // ends and rail, both clock faces (drawn in every tier) and every fixed plate, all of the
    // apparatus below the band.
    void expect_drawn_apparatus_inside(const app::SimulationSession& session, const render::ScreenRect& focus, std::string_view when)
    {
        const auto& layout = session.scene_renderer().relativity_stage_layout();
        const auto context = " " + std::string(when);
        RIGIDBODIES_EXPECT(layout.has_value(), "the stage records what it drew" + context);
        const auto right = focus.left + focus.width, bottom = focus.top + focus.height;
        const auto within = [&](double left, double top, double width, double height)
        {
            return left >= focus.left - 1.0e-6 && top >= focus.top - 1.0e-6 && left + width <= right + 1.0e-6 && top + height <= bottom + 1.0e-6;
        };
        const auto& band = layout->band;
        const auto band_bottom = band.top + band.height;
        RIGIDBODIES_EXPECT(band.left == focus.left && band.top == focus.top && band.width == focus.width && band_bottom <= bottom, "the band lies across the top of the stage" + context);
        RIGIDBODIES_EXPECT(layout->track_left >= focus.left && layout->track_right <= right && layout->track_left < layout->track_right, "both ends of the track are on the stage" + context);
        RIGIDBODIES_EXPECT(layout->rail_y > band_bottom && layout->rail_y < bottom, "the rail runs below the band" + context);
        const auto radius = layout->clock_radius;
        RIGIDBODIES_EXPECT(radius >= 10.0 * layout->text_pixel_scale, "both clock faces are drawn, at least 10 logical pixels across the radius" + context);
        for (const auto& centre : { layout->lab_clock_center, layout->probe_clock_center })
        {
            RIGIDBODIES_EXPECT(within(centre.x - radius, centre.y - radius, 2.0 * radius, 2.0 * radius), "both clock faces are on the stage" + context);
            RIGIDBODIES_EXPECT(centre.y - radius >= band_bottom, "the band ends above both clocks" + context);
        }
        RIGIDBODIES_EXPECT(layout->fixed_plates.size() > 8, "the band's plates and the lab plates are drawn" + context);
        for (std::size_t index = 0; index < layout->fixed_plates.size(); ++index)
        {
            const auto& plate = layout->fixed_plates[index];
            RIGIDBODIES_EXPECT(within(plate.left, plate.top, plate.width, plate.height), "every fixed plate is on the stage" + context);
            if (index >= 8)
                RIGIDBODIES_EXPECT(plate.top >= band_bottom, "every plate of the apparatus lies below the band" + context);
        }
        RIGIDBODIES_EXPECT(layout->probe_center.y - layout->probe_radius >= band_bottom && layout->probe_center.y < layout->rail_y, "the probe rides on the rail below the band" + context);
    }

    RIGIDBODIES_TEST("Chasing light frames the track, both clocks and the band without overlap in every layout")
    {
        for (const auto& layout : lesson_layouts())
            for (const auto units : { core::DisplayUnits::si, core::DisplayUnits::centimetre_gram })
            {
                const auto focus = focus_of(layout.input);
                app::SimulationSession session;
                session.set_viewport(layout.input.viewport);
                session.set_focus_rect(focus);
                session.configure({});
                session.scene_settings().display_scale = layout.input.scale;
                session.scene_settings().display_units = units;
                session.set_presenting(layout.input.present);
                RIGIDBODIES_EXPECT(session.load_scenario("chasing_light") && session.relativity_active(), "Chasing light loads");
                session.set_focus_rect(focus);
                const auto when = "in " + layout.name + (units == core::DisplayUnits::si ? "" : " in centimetre-gram units");
                // Every layout holds the band and a tier's apparatus, a 720p projector with 150 % text
                // (a stage 300 px tall under the caption) included.
                RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "at the start " + when), "the band and the apparatus fit the stage " + when);
                expect_drawn_apparatus_inside(session, focus, "at the start " + when);
                const auto framed = session.camera();
                // The apparatus moves nothing in the World, so the framing holds while it runs.
                ui::UiCommand speed;
                speed.kind = ui::UiCommandKind::set_relativity_speed;
                speed.value = 0.99999;
                session.apply(speed);
                session.stepper().set_paused(false);
                for (int frame = 0; frame < 80; ++frame)
                {
                    session.set_focus_rect(focus);
                    session.advance(1.0 / 60.0);
                    render::DrawList list;
                    session.render(list);
                }
                RIGIDBODIES_EXPECT(session.relativity_probe()->race().lab_time_s > 0.0 && same_camera(session.camera(), framed), "the framing holds while the probe runs " + when);
                RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "after 80 frames at 0.99999 c " + when), "the band and the apparatus still fit " + when);
                expect_drawn_apparatus_inside(session, focus, "after 80 frames at 0.99999 c " + when);
                const auto tps = stage_text_pixel_scale(session);
                RIGIDBODIES_EXPECT(drawn_tier(session) == render::fitting_relativity_tier(focus.width, focus.height, tps, units), "a stage that holds still shows the largest tier that fits " + when);
            }
    }

    RIGIDBODIES_TEST("relativity framing follows Present, text size and units but never a moved camera")
    {
        const auto layout = lesson_layouts().front();
        const auto focus = focus_of(layout.input);
        app::SimulationSession session;
        session.set_viewport(layout.input.viewport);
        session.set_focus_rect(focus);
        session.configure({});
        session.scene_settings().display_scale = layout.input.scale;
        RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light loads");
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "at the start"), "the band and the apparatus fit the stage at the start");
        // Each of these changes the band or the stack without changing the stage's rect, which
        // the application publishes again every frame.
        auto before = session.camera();
        session.set_presenting(true);
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(!same_camera(session.camera(), before) && !session.camera_user_moved(), "Present's larger text reframes an unmoved camera");
        RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "in Present"), "the band and the apparatus fit the stage in Present");
        before = session.camera();
        session.set_presenting(false);
        ui::UiCommand text;
        text.kind = ui::UiCommandKind::set_ui_scale;
        text.value = 1.75;
        session.apply(text);
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(!same_camera(session.camera(), before), "a larger text size reframes it");
        RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "at a larger text size"), "the band and the apparatus fit the stage at a larger text size");
        ui::UiCommand units;
        units.kind = ui::UiCommandKind::set_display_units;
        units.id = "centimetre_gram";
        session.apply(units);
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "after switching to centimetre-gram units"), "the band and the apparatus fit the stage after switching to centimetre-gram units");
        before = session.camera();
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(same_camera(session.camera(), before), "an unchanged stage is not reframed");
        // A view the learner moved stays theirs until F.
        session.pan_view({ 120.0, -40.0 });
        before = session.camera();
        text.value = 1.0;
        session.apply(text);
        session.set_focus_rect(focus);
        session.set_focus_rect({ focus.left, focus.top, focus.width - 100.0, focus.height });
        session.set_focus_rect(focus);
        RIGIDBODIES_EXPECT(session.camera_user_moved() && session.camera().view_height_m() == before.view_height_m(), "a moved camera keeps its zoom through text size and stage changes");
        ui::UiCommand frame;
        frame.kind = ui::UiCommandKind::frame_subject;
        session.apply(frame);
        RIGIDBODIES_EXPECT(!session.camera_user_moved(), "F restores the framing");
        RIGIDBODIES_EXPECT(expect_relativity_framing(session, focus, "after F"), "the band and the apparatus fit the stage after F");
        const auto restored = session.camera();
        session.pan_view({ 50.0, 0.0 });
        frame.kind = ui::UiCommandKind::frame_all;
        session.apply(frame);
        RIGIDBODIES_EXPECT(!session.camera_user_moved() && same_camera(session.camera(), restored), "framing everything frames the apparatus the same way");
    }

    RIGIDBODIES_TEST("the relativity stage changes tier with hysteresis as the stage shrinks and grows")
    {
        const auto layout = lesson_layouts()[1];
        const auto full = focus_of(layout.input);
        app::SimulationSession session;
        session.set_viewport(layout.input.viewport);
        session.set_focus_rect(full);
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light loads");
        session.set_focus_rect(full);
        RIGIDBODIES_EXPECT(drawn_tier(session) == render::RelativityStageTier::full, "a large stage shows everything");
        auto previous = render::RelativityStageTier::full;
        std::vector<render::RelativityStageTier> seen;
        const auto resize = [&](double width, double height)
        {
            const render::ScreenRect focus { full.left, full.top, width, height };
            session.set_focus_rect(focus);
            const auto tier = drawn_tier(session);
            const auto tps = stage_text_pixel_scale(session);
            RIGIDBODIES_EXPECT(tier == render::choose_relativity_tier(width, height, tps, session.scene_settings().display_units, previous), "the tier follows the stage size from the tier before");
            expect_relativity_framing(session, focus, "at " + std::to_string(static_cast<int>(width)) + " px across");
            previous = tier;
            if (seen.empty() || seen.back() != tier)
                seen.push_back(tier);
        };
        for (double width = full.width; width >= 300.0; width -= 20.0)
            resize(width, full.height);
        for (double width = 300.0; width <= full.width; width += 20.0)
            resize(width, full.height);
        RIGIDBODIES_EXPECT(seen.size() >= 5 && seen[1] == render::RelativityStageTier::compact && seen[2] == render::RelativityStageTier::minimal && seen.back() == render::RelativityStageTier::full, "a narrowing stage drops to compact and minimal and returns to full");
        // The full tier is entered at 700 logical px and kept down to 684, so a width between the
        // two keeps whichever tier the stage already had.
        const auto at_width = [&](double width)
        {
            session.set_focus_rect({ full.left, full.top, width, full.height });
            return drawn_tier(session);
        };
        RIGIDBODIES_EXPECT(at_width(700.0) == render::RelativityStageTier::full && at_width(690.0) == render::RelativityStageTier::full, "a stage narrowing below 700 px keeps the full tier");
        RIGIDBODIES_EXPECT(at_width(680.0) == render::RelativityStageTier::compact, "below 684 px it gives way");
        RIGIDBODIES_EXPECT(at_width(695.0) == render::RelativityStageTier::compact, "widening again keeps the compact tier until 700 px");
        RIGIDBODIES_EXPECT(at_width(700.0) == render::RelativityStageTier::full, "at 700 px the full tier returns");
        // Hysteresis only steadies a stage that is being resized: one that stops between the two
        // thresholds settles on the largest tier that fits it on the next frame.
        RIGIDBODIES_EXPECT(at_width(680.0) == render::RelativityStageTier::compact && at_width(695.0) == render::RelativityStageTier::compact, "a stage widening from 680 px keeps the compact tier at 695 px");
        RIGIDBODIES_EXPECT(at_width(695.0) == render::RelativityStageTier::full, "once it holds still at 695 px it settles on the full tier, which fits down to 684 px");
        expect_relativity_framing(session, { full.left, full.top, 695.0, full.height }, "after settling at 695 px");
        RIGIDBODIES_EXPECT(at_width(695.0) == render::RelativityStageTier::full, "and keeps it");
    }

    // Present on a 1600 x 900 window at display scale 1.5, as the application drives it: the stage
    // first frames under the caption's whole box, then under the caption the interface measured.
    struct PresentFrames
    {
        render::ScreenRect ordinary, unmeasured, measured;
    };

    PresentFrames present_frames(const ui::LayoutInput& ordinary_input, double caption_height)
    {
        auto present = ordinary_input;
        present.present = true;
        present.present_caption = true;
        auto measured = present;
        measured.present_caption_height = caption_height;
        return { focus_of(ordinary_input), focus_of(present), focus_of(measured) };
    }

    RIGIDBODIES_TEST("Present settles on the largest tier that fits once its caption is measured")
    {
        // The guide's first step measures 160 logical pixels as a two-line caption on this window.
        // A window's client area is a little smaller than the window: there the compact tier fits
        // with less than its 16 pixel margin, so hysteresis alone would keep the minimal tier.
        for (const auto& viewport : { render::ViewportSize { 1600, 900 }, render::ViewportSize { 1578, 889 } })
        {
            const auto client_area = viewport.width == 1578;
            ui::LayoutInput input;
            input.viewport = viewport;
            input.scale = 1.5f;
            input.guide_available = true;
            const auto frames = present_frames(input, 160.0);
            const auto when = " on a " + std::to_string(viewport.width) + " x " + std::to_string(viewport.height) + " window";
            app::SimulationSession session;
            session.set_viewport(viewport);
            session.set_focus_rect(frames.ordinary);
            session.configure({});
            session.scene_settings().display_scale = input.scale;
            RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light loads");
            for (int frame = 0; frame < 3; ++frame)
                session.set_focus_rect(frames.ordinary);
            RIGIDBODIES_EXPECT(drawn_tier(session) == render::RelativityStageTier::full, "the ordinary stage shows the full tier" + when);
            // The application publishes the focus area before it hands the session Present, so the
            // larger text first meets the ordinary area, then the caption's box, then the caption.
            session.set_presenting(true);
            session.set_focus_rect(frames.ordinary);
            session.set_focus_rect(frames.unmeasured);
            RIGIDBODIES_EXPECT(drawn_tier(session) == render::RelativityStageTier::minimal, "under the caption's whole box only the minimal tier fits" + when);
            // Present's text scale, as the frame just drawn used it.
            const auto tps = stage_text_pixel_scale(session);
            const auto units = session.scene_settings().display_units;
            RIGIDBODIES_EXPECT(expect_relativity_framing(session, frames.unmeasured, "under the caption's whole box" + when), "the minimal tier fits under the caption's whole box" + when);
            expect_drawn_apparatus_inside(session, frames.unmeasured, "under the caption's whole box" + when);
            session.set_focus_rect(frames.measured);
            const auto fitting = render::fitting_relativity_tier(frames.measured.width, frames.measured.height, tps, units);
            RIGIDBODIES_EXPECT(fitting == render::RelativityStageTier::compact, "the measured caption leaves room for the compact tier, with its clocks and race" + when);
            if (client_area)
                RIGIDBODIES_EXPECT(render::choose_relativity_tier(frames.measured.width, frames.measured.height, tps, units, render::RelativityStageTier::minimal) == render::RelativityStageTier::minimal, "hysteresis alone would keep the minimal tier" + when);
            for (int frame = 0; frame < 3; ++frame)
            {
                session.set_focus_rect(frames.measured);
                session.advance(1.0 / 60.0);
            }
            RIGIDBODIES_EXPECT(drawn_tier(session) == fitting, "once the layout holds still Present shows the largest tier that fits" + when);
            RIGIDBODIES_EXPECT(expect_relativity_framing(session, frames.measured, "in Present" + when), "the band and the compact apparatus fit in Present" + when);
            expect_drawn_apparatus_inside(session, frames.measured, "in Present" + when);
            // Leaving Present goes back to the full tier the same way.
            session.set_presenting(false);
            session.set_focus_rect(frames.measured);
            for (int frame = 0; frame < 3; ++frame)
                session.set_focus_rect(frames.ordinary);
            RIGIDBODIES_EXPECT(drawn_tier(session) == render::RelativityStageTier::full, "after Present the ordinary stage shows the full tier again" + when);
        }
    }

    RIGIDBODIES_TEST("a saved relativity setup opens on its saved view, and the document's own view gives way")
    {
        const auto layout = lesson_layouts().front();
        const auto focus = focus_of(layout.input);
        const auto session_at = [&](app::SimulationSession& session)
        {
            session.set_viewport(layout.input.viewport);
            session.set_focus_rect(focus);
            session.configure({});
        };
        app::SimulationSession saving;
        session_at(saving);
        RIGIDBODIES_EXPECT(saving.load_scenario("chasing_light"), "Chasing light loads");
        saving.set_focus_rect(focus);
        saving.pan_view({ 80.0, 30.0 });
        saving.zoom_view(2.0, focus.center());
        const auto saved_view = saving.camera();
        std::string text, error;
        RIGIDBODIES_EXPECT(saving.save_arrangement(text, error, "My probe", false, true), error);
        app::SimulationSession opening;
        session_at(opening);
        RIGIDBODIES_EXPECT(opening.open_arrangement(text, error) && opening.relativity_active(), error);
        opening.set_focus_rect(focus);
        RIGIDBODIES_EXPECT_NEAR(opening.camera().view_height_m(), saved_view.view_height_m(), 1.0e-9, "the saved zoom is kept");
        RIGIDBODIES_EXPECT_NEAR(math::distance(opening.camera().center_m(), saved_view.center_m()), 0.0, 1.0e-9, "the saved centre is kept");
        opening.set_focus_rect({ focus.left, focus.top, focus.width - 200.0, focus.height });
        opening.set_focus_rect(focus);
        expect_relativity_framing(opening, focus, "once the stage changes");
        // A pan made in another experiment before the file was opened is not the saved view's:
        // that view still gives way once the stage changes.
        app::SimulationSession panned;
        session_at(panned);
        RIGIDBODIES_EXPECT(panned.load_scenario("free_fall"), "Free fall loads");
        panned.set_focus_rect(focus);
        panned.pan_view({ 40.0, 20.0 });
        panned.zoom_view(1.0, focus.center());
        RIGIDBODIES_EXPECT(panned.open_arrangement(text, error) && panned.relativity_active(), error);
        panned.set_focus_rect(focus);
        RIGIDBODIES_EXPECT_NEAR(panned.camera().view_height_m(), saved_view.view_height_m(), 1.0e-9, "the saved zoom is kept after an earlier pan");
        panned.set_focus_rect({ focus.left, focus.top, focus.width - 200.0, focus.height });
        panned.set_focus_rect(focus);
        expect_relativity_framing(panned, focus, "once the stage changes after a pan made before opening");
        // The bundled document has a top-level view for headless sessions only.
        std::string bundled;
        RIGIDBODIES_EXPECT(physics::write_scenario_document(*physics::scenario_document_for_id("chasing_light"), bundled, error), error);
        app::SimulationSession authored;
        session_at(authored);
        RIGIDBODIES_EXPECT(authored.open_arrangement(bundled, error), error);
        authored.set_focus_rect(focus);
        expect_relativity_framing(authored, focus, "when the document has only its own view");
        // Without a stage the document's view stands in.
        app::SimulationSession headless;
        headless.set_viewport(layout.input.viewport);
        headless.configure({});
        RIGIDBODIES_EXPECT(headless.load_scenario("chasing_light"), "Chasing light loads");
        RIGIDBODIES_EXPECT_NEAR(headless.camera().center_m().x, 1.49896229, 1.0e-9, "a session without a stage centres the document's view");
    }

    RIGIDBODIES_TEST("F maps to subject framing and empty selection leaves the camera alone")
    {
        const auto focus = backend_focus();
        const auto command = app::command_for_action(app::action_for_key(ui::UiKey::f, {}));
        RIGIDBODIES_EXPECT(command.kind == ui::UiCommandKind::frame_subject, "F dispatches frame subject");
        const auto binding = std::find_if(app::key_bindings.begin(), app::key_bindings.end(), [](const auto& candidate)
            {
                return candidate.key == ui::UiKey::f && !candidate.modifiers.shift;
            });
        RIGIDBODIES_EXPECT(binding != app::key_bindings.end() && binding->description == "Frame subject", "keyboard reference describes F");

        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect(focus);
        session.configure({});
        session.camera().set_center({ 9.0, -4.0 });
        session.camera().set_view_height(17.0);
        const auto before = session.camera();
        session.frame_selection();
        RIGIDBODIES_EXPECT(session.camera().center_m() == before.center_m() && session.camera().view_height_m() == before.view_height_m(), "empty selection does not move the camera");
        const auto model = session.build_model();
        const auto notice = std::find_if(model.notifications.begin(), model.notifications.end(), [](const ui::Notification& item)
            {
                return item.text == "Nothing selected";
            });
        RIGIDBODIES_EXPECT(notice != model.notifications.end(), "empty selection posts the prescribed feedback toast");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
