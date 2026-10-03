#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/scenario.hpp>
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
