#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>
#include <rigidbodies/core/display_units.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace
{
    using namespace rigidbodies;

    physics::BodyId add_circle(app::SimulationSession& session, math::Vec2 position)
    {
        physics::BodyDefinition definition;
        definition.position_m = position;
        definition.name = "target";
        physics::Collider collider;
        collider.shape = physics::make_circle(0.2);
        definition.colliders.push_back(collider);
        return session.world().create_body(definition);
    }

    void paused_lab(app::SimulationSession& session)
    {
        session.world().clear();
        session.set_viewport({ 1000, 800 });
        session.camera().set_view_height(4.0);
        session.stepper().set_paused(true);
    }

    math::Vec2 handle_centre(const ui::UiModel& model, std::string_view id)
    {
        for (const auto& handle : model.handles)
            if (handle.id == id)
                return handle.screen_bounds.center();
        RIGIDBODIES_FAIL("the requested stage handle should be published");
    }

    void drag(app::SimulationSession& session, math::Vec2 from, math::Vec2 to, double scale = 1.0)
    {
        ui::UiEvent event;
        event.logical_pixel_scale = scale;
        event.button = ui::PointerButton::primary;
        for (const auto& [kind, point] : { std::pair { ui::UiEventKind::pointer_down, from }, std::pair { ui::UiEventKind::pointer_move, to }, std::pair { ui::UiEventKind::pointer_up, to } })
        {
            event.kind = kind;
            event.pointer_px = point;
            session.handle_scene_event(event);
        }
    }

    RIGIDBODIES_TEST("gravity dial sits in the stage corner and answers across its whole face at every display scale")
    {
        for (const auto scale : { 1.0, 1.5, 2.0 })
        {
            app::SimulationSession session;
            paused_lab(session);
            session.set_focus_rect({ 100.0, 60.0, 900.0, 600.0 });
            session.scene_settings().display_scale = static_cast<float>(scale);
            const auto centre = handle_centre(session.build_model(), "gravity");
            RIGIDBODIES_EXPECT_NEAR(centre.x, 1000.0 - app::overlay::compass_radius * scale, 1.0e-9, "dial hugs the right edge of the framed stage area");
            RIGIDBODIES_EXPECT_NEAR(centre.y, 60.0 + app::overlay::compass_radius * scale, 1.0e-9, "dial hugs the top of the framed stage area");
            // A press near the rim lies beyond a plain 12 px target, and the drag must measure
            // its angle from the same centre the dial is drawn at.
            drag(session, centre + math::Vec2 { 0.0, 15.0 * scale }, centre + math::Vec2 { 40.0 * scale, 0.0 }, scale);
            const auto gravity = session.world().settings().gravity_m_s2;
            RIGIDBODIES_EXPECT(gravity.x > 9.0 && std::abs(gravity.y) < 1.0e-9, "dragging toward the right turns gravity to point right");
        }
    }

    RIGIDBODIES_TEST("velocity handle sits on the scene's velocity arrow and maps pixels back through the same scale")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto id = add_circle(session, {});
        session.world().find_body(id)->set_linear_velocity({ 1.0, 0.0 });
        session.set_selection(id);
        const auto pixels = session.scene_settings().vector_scales.velocity;
        const auto tip = handle_centre(session.build_model(), "velocity");
        const auto centre = session.camera().world_to_screen({});
        RIGIDBODIES_EXPECT_NEAR(tip.x - centre.x, pixels, 1.0e-9, "one metre per second is drawn at the scene's pixels per unit");
        RIGIDBODIES_EXPECT_NEAR(tip.y - centre.y, 0.0, 1.0e-9, "a horizontal velocity keeps the knob level with the centre");
        drag(session, tip, tip + math::Vec2 { pixels, -0.5 * pixels });
        const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
        RIGIDBODIES_EXPECT_NEAR(velocity.x, 2.0, 1.0e-9, "dragging one scale unit right adds one metre per second");
        RIGIDBODIES_EXPECT_NEAR(velocity.y, 0.5, 1.0e-9, "screen up is positive world velocity");
    }

    RIGIDBODIES_TEST("a velocity arrow the stage caps keeps one arrow and one reading, its knob on a tether past the head")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto id = add_circle(session, {});
        session.world().find_body(id)->set_linear_velocity({ 40.0, 0.0 });
        session.set_selection(id);
        render::DrawList list;
        session.render(list);
        const auto& velocity = session.scene_settings().theme.velocity;
        std::size_t heads = 0, readings = 0;
        for (const auto& command : list.commands())
        {
            if (command.kind == render::DrawCommandKind::polygon_fill && command.color.red == velocity.red && command.color.green == velocity.green && command.color.blue == velocity.blue && command.color.alpha == velocity.alpha)
                ++heads;
            if (command.kind == render::DrawCommandKind::text && std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length).find("40.00") != std::string_view::npos)
                ++readings;
        }
        RIGIDBODIES_EXPECT(heads == 1, "the handle adds no second arrowhead to the stage's capped arrow");
        RIGIDBODIES_EXPECT(readings == 1, "the speed is read once, on the stage's plate");
    }

    RIGIDBODIES_TEST("rotation knob marks the body's up axis so turning it is continuous")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto id = add_circle(session, {});
        auto* body = session.world().find_body(id);
        body->set_orientation(math::degrees_to_radians(30.0));
        session.set_selection(id);
        const auto centre = session.camera().world_to_screen({});
        const auto radius = app::rotation_ring_radius(session.camera(), *body, 1.0);
        const auto knob = handle_centre(session.build_model(), "rotation");
        RIGIDBODIES_EXPECT_NEAR(knob.x, centre.x - radius * 0.5, 1.0e-9, "a 30 degree turn moves the knob left of the centre");
        RIGIDBODIES_EXPECT_NEAR(knob.y, centre.y - radius * std::sqrt(3.0) * 0.5, 1.0e-9, "the knob stays on the body's local up axis");
        drag(session, knob, app::rotation_knob_position(centre, radius, math::degrees_to_radians(40.0)));
        RIGIDBODIES_EXPECT_NEAR(math::radians_to_degrees(session.world().find_body(id)->orientation_rad()), 40.0, 1.0e-9, "grabbing the knob and turning it ten degrees adds ten degrees");
    }

    RIGIDBODIES_TEST("stage overlays draw above the scene, repeat exactly and leave the world untouched")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto first = add_circle(session, { -0.5, 0.0 });
        const auto second = add_circle(session, { 0.5, 0.0 });
        session.world().find_body(first)->set_linear_velocity({ 1.0, 2.0 });
        session.set_selections({ first, second });
        const auto before = session.world().snapshot();
        render::DrawList list, again;
        session.render(list);
        session.render(again);
        RIGIDBODIES_EXPECT(list.vertices() == again.vertices() && list.commands().size() == again.commands().size(), "an unchanged paused frame draws identical overlays");
        int scene_layer = std::numeric_limits<int>::min();
        std::size_t overlay_commands = 0;
        for (const auto& command : list.commands())
            if (command.layer == app::overlay::overlay_layer)
                ++overlay_commands;
            else
                scene_layer = std::max(scene_layer, command.layer);
        RIGIDBODIES_EXPECT(overlay_commands > 0 && scene_layer < app::overlay::overlay_layer, "handles, outlines and the dial sort above every scene layer");
        physics::World expected;
        expected.restore(before);
        for (const auto id : { first, second })
            RIGIDBODIES_EXPECT(session.world().find_body(id)->position_m() == expected.find_body(id)->position_m() && session.world().find_body(id)->linear_velocity_m_s() == expected.find_body(id)->linear_velocity_m_s(), "drawing overlays never moves or accelerates a body");
    }

    RIGIDBODIES_TEST("gesture overlays describe a pull and a throw without touching the world")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto id = add_circle(session, {});
        const auto press = session.camera().world_to_screen({});
        const auto mode = [&](const char* name)
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_interaction_mode;
            command.id = name;
            session.apply(command);
        };
        const auto send = [&](ui::UiEventKind kind, math::Vec2 point, double time_s)
        {
            ui::UiEvent event;
            event.kind = kind;
            event.button = ui::PointerButton::primary;
            event.pointer_px = point;
            event.timestamp_s = time_s;
            session.handle_scene_event(event);
        };
        const auto text_drawn = [](const render::DrawList& list, std::string_view text)
        {
            return list.text_buffer().find(text) != std::string::npos;
        };

        mode("pull");
        send(ui::UiEventKind::pointer_down, press, 1.0);
        send(ui::UiEventKind::pointer_move, press + math::Vec2 { 120.0, 40.0 }, 1.1);
        const auto pulled = session.world().snapshot();
        render::DrawList list;
        session.render(list);
        RIGIDBODIES_EXPECT(text_drawn(list, "Resume to pull"), "a paused pull explains that it applies no force yet");
        physics::World expected;
        expected.restore(pulled);
        RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s() == expected.find_body(id)->linear_velocity_m_s(), "drawing the pull applies no force");
        send(ui::UiEventKind::pointer_up, press + math::Vec2 { 120.0, 40.0 }, 1.2);

        mode("throw");
        const auto start = session.camera().world_to_screen(session.world().find_body(id)->position_m());
        send(ui::UiEventKind::pointer_down, start, 2.0);
        for (int step = 1; step <= 4; ++step)
            send(ui::UiEventKind::pointer_move, start + math::Vec2 { 15.0 * step, -6.0 * step }, 2.0 + 0.02 * step);
        const auto velocity = session.world().find_body(id)->linear_velocity_m_s();
        session.render(list);
        const auto preview = std::count_if(list.commands().begin(), list.commands().end(), [](const render::DrawCommand& command)
            {
                return command.layer == app::overlay::overlay_layer && command.kind == render::DrawCommandKind::indexed_mesh && command.mesh && command.mesh->vertices.size() > 16;
            });
        RIGIDBODIES_EXPECT(preview == 1 && text_drawn(list, "m/s"), "a throw in progress previews its release arc and speed");
        RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s() == velocity, "the preview does not set the release velocity early");
    }

    const render::IndexedMesh& only_mesh(const render::DrawList& list)
    {
        RIGIDBODIES_EXPECT(list.commands().size() == 1 && list.commands().front().kind == render::DrawCommandKind::indexed_mesh && list.commands().front().mesh, "one indexed mesh command");
        return *list.commands().front().mesh;
    }

    RIGIDBODIES_TEST("dashed strokes are one mesh with whole dashes and a bounded size")
    {
        render::DrawList list;
        const app::OverlayPainter paint(list, render::Theme {}, 1.0);
        // A 400 px closed square at 4 on / 3 off holds 57.14 periods; the pattern is stretched to
        // exactly 57 so that no partial dash meets the seam.
        paint.dashed_path({ { 0.0, 0.0 }, { 100.0, 0.0 }, { 100.0, 100.0 }, { 0.0, 100.0 } }, true, render::Color {}, 1.5);
        const auto& square = only_mesh(list);
        RIGIDBODIES_EXPECT(square.indices.size() % 6 == 0 && square.vertices.size() * 6 == square.indices.size() * 4, "each dash piece is one quad");
        double drawn = 0.0;
        for (std::size_t quad = 0; quad < square.vertices.size(); quad += 4)
            drawn += math::length((square.vertices[quad + 2].position + square.vertices[quad + 3].position) * 0.5 - (square.vertices[quad].position + square.vertices[quad + 1].position) * 0.5);
        RIGIDBODIES_EXPECT_NEAR(drawn, 400.0 * 4.0 / 7.0, 1.0e-6, "dashes cover exactly the on fraction of a whole number of periods");
        for (const auto& vertex : square.vertices)
            RIGIDBODIES_EXPECT(vertex.color.alpha == 1.0f && vertex.color.red == vertex.color.alpha, "mesh colours are premultiplied");

        list.clear();
        paint.dashed_path({ { 0.0, 0.0 }, { 1.0e7, 0.0 } }, false, render::Color {}, 1.5);
        RIGIDBODIES_EXPECT(only_mesh(list).vertices.size() <= 4 * 4100, "an enormous path stretches its pattern instead of emitting millions of dashes");

        list.clear();
        paint.dashed_path({ { 0.0, 0.0 }, { 0.0, 0.0 } }, false, render::Color {}, 1.5);
        paint.dashed_path({ { 0.0, 0.0 }, { std::nan(""), 1.0 } }, false, render::Color {}, 1.5);
        RIGIDBODIES_EXPECT(list.commands().empty(), "degenerate or non-finite paths draw nothing");
    }

    RIGIDBODIES_TEST("tapered dashes fade and narrow from the first point to the last")
    {
        render::DrawList list;
        const app::OverlayPainter paint(list, render::Theme {}, 2.0);
        paint.tapered_dashes({ { 0.0, 0.0 }, { 300.0, 0.0 } }, render::Color {}, 2.0, 1.0, 0.25f, 2.0, 4.0);
        const auto& mesh = only_mesh(list);
        const auto& first = mesh.vertices.front();
        const auto& last = mesh.vertices.back();
        RIGIDBODIES_EXPECT_NEAR(std::abs(first.position.y) * 2.0, 4.0, 1.0e-9, "the start is two logical pixels wide at twice the scale");
        RIGIDBODIES_EXPECT(std::abs(last.position.y) < std::abs(first.position.y) && last.color.alpha < first.color.alpha, "the far end is thinner and fainter");
    }

    RIGIDBODIES_TEST("spotlight shades the whole viewport outside a feathered clear circle")
    {
        for (const auto& centre : { math::Vec2 { 400.0, 300.0 }, math::Vec2 { -200.0, 900.0 } })
        {
            render::DrawList list;
            const render::ViewportSize viewport { 800, 600 };
            app::add_spotlight(list, viewport, centre, 80.0, 48.0, { 0.0f, 0.0f, 0.0f, 0.5f });
            const auto& mesh = only_mesh(list);
            double clear = 1.0e9, outer = 0.0;
            for (const auto& vertex : mesh.vertices)
            {
                const auto distance = math::length(vertex.position - centre);
                if (vertex.color.alpha == 0.0f)
                    clear = std::min(clear, distance);
                outer = std::max(outer, distance);
                RIGIDBODIES_EXPECT(vertex.color.alpha <= 0.5f, "the shade never exceeds its requested opacity");
            }
            RIGIDBODIES_EXPECT_NEAR(clear, 80.0, 1.0e-9, "the clear circle has the requested radius");
            const auto segments = static_cast<double>(mesh.vertices.size() / 5);
            for (const auto& corner : { math::Vec2 { 0.0, 0.0 }, math::Vec2 { 800.0, 0.0 }, math::Vec2 { 0.0, 600.0 }, math::Vec2 { 800.0, 600.0 } })
                RIGIDBODIES_EXPECT(math::length(corner - centre) < outer * std::cos(math::pi / segments), "the outer ring covers every viewport corner");
        }
        render::DrawList list;
        app::add_spotlight(list, { 800, 600 }, { std::nan(""), 0.0 }, 80.0, 48.0, {});
        RIGIDBODIES_EXPECT(list.is_empty(), "an unknown pointer position draws no spotlight");
    }

    RIGIDBODIES_TEST("overlay palette follows the theme and keeps danger distinct from the accent")
    {
        for (const auto* name : { "workbench_dark", "workbench_light", "workbench_projector" })
        {
            const auto theme = render::theme_by_name(name);
            const auto palette = app::overlay_palette(theme);
            RIGIDBODIES_EXPECT(palette.dark == (std::string_view(name) == "workbench_dark"), "brightness of the stage selects the dark palette");
            RIGIDBODIES_EXPECT(palette.accent.blue == theme.panel_accent.blue && palette.selection.red == theme.selection.red, "accent and selection come from the scene theme");
            RIGIDBODIES_EXPECT(palette.danger.red > palette.danger.blue + 0.3f && palette.accent.blue > palette.accent.red, "danger reads as red and the accent as blue");
        }
    }

    physics::BodyId add_box(app::SimulationSession& session, math::Vec2 position, double width_m, double height_m)
    {
        physics::BodyDefinition definition;
        definition.position_m = position;
        physics::Collider collider;
        collider.shape = physics::make_box(width_m, height_m);
        definition.colliders.push_back(collider);
        return session.world().create_body(definition);
    }

    const render::DrawCommand* drawn_text(const render::DrawList& list, std::string_view text)
    {
        for (const auto& draw : list.commands())
            if (draw.kind == render::DrawCommandKind::text && std::string_view(list.text_buffer()).substr(draw.text_offset, draw.text_length) == text)
                return &draw;
        return nullptr;
    }

    RIGIDBODIES_TEST("a resting body's velocity knob parks beside its centre of mass, which still moves the body")
    {
        for (const auto scale : { 1.0, 2.0 })
        {
            app::SimulationSession session;
            paused_lab(session);
            session.scene_settings().display_scale = static_cast<float>(scale);
            const auto id = add_circle(session, {});
            session.set_selection(id);
            const auto centre = session.camera().world_to_screen({});
            const auto knob = handle_centre(session.build_model(), "velocity");
            RIGIDBODIES_EXPECT_NEAR(knob.x - centre.x, app::overlay::velocity_knob_park_distance * scale, 1.0e-9, "a zero velocity parks its knob to the right of the centre");
            RIGIDBODIES_EXPECT_NEAR(knob.y, centre.y, 1.0e-9, "the parked knob stays level with the centre");
            RIGIDBODIES_EXPECT(math::length(knob - centre) > app::overlay::handle_hit_radius * scale, "the centre of mass lies outside the knob's target");

            drag(session, centre, centre + math::Vec2 { 0.0, -40.0 * scale }, scale);
            const auto* body = session.world().find_body(id);
            RIGIDBODIES_EXPECT(body->position_m().y > 0.1 && math::length(body->linear_velocity_m_s()) == 0.0, "pressing the centre of mass moves the body instead of setting a velocity");

            const auto moved = session.camera().world_to_screen(body->world_center_of_mass_m());
            const auto parked = handle_centre(session.build_model(), "velocity");
            drag(session, parked, moved + math::Vec2 { 48.0, 0.0 }, scale);
            // Arrow scales are logical pixels per metre per second, so a denser display draws
            // the arrow, and maps the drag, over proportionally more device pixels.
            const auto pixels = session.scene_settings().vector_scales.velocity * scale;
            RIGIDBODIES_EXPECT_NEAR(session.world().find_body(id)->linear_velocity_m_s().x, 48.0 / pixels, 1.0e-9, "dragging the parked knob puts the arrow's tip under the pointer");
        }
    }

    RIGIDBODIES_TEST("the rotation grip and the velocity knob are distinct and the grip keeps off a neighbour's centre")
    {
        app::SimulationSession session;
        paused_lab(session);
        session.camera().set_view_height(3.0);
        const auto lower = add_box(session, { 0.0, -0.15 }, 0.45, 0.3);
        const auto upper = add_box(session, { 0.0, 0.152 }, 0.45, 0.3);
        session.set_selection(lower);
        const auto grip = handle_centre(session.build_model(), "rotation");
        const auto neighbour = session.camera().world_to_screen(session.world().find_body(upper)->world_center_of_mass_m());
        RIGIDBODIES_EXPECT(math::length(grip - neighbour) >= app::overlay::knob_radius + app::overlay::rotation_grip_radius, "the grip never sits on the box above's centre-of-mass symbol");
        const auto centre = session.camera().world_to_screen(session.world().find_body(lower)->world_center_of_mass_m());
        RIGIDBODIES_EXPECT_NEAR(grip.x, centre.x, 1.0e-9, "the grip still marks the unrotated body's up axis");

        render::DrawList list;
        session.render(list);
        bool solid_grip = false, ringed_knob = false;
        for (const auto& draw : list.commands())
        {
            if (draw.layer != app::overlay::overlay_layer || draw.vertex_count == 0)
                continue;
            const auto at = list.vertices()[draw.vertex_offset];
            solid_grip = solid_grip || (draw.kind == render::DrawCommandKind::circle_fill && math::length(at - grip) < 1.0e-9 && draw.color.blue == session.scene_settings().theme.panel_accent.blue);
            ringed_knob = ringed_knob || (draw.kind == render::DrawCommandKind::circle_outline && draw.color.green == session.scene_settings().theme.velocity.green);
        }
        RIGIDBODIES_EXPECT(solid_grip && ringed_knob, "the grip is a solid accent dot and the velocity knob a ring in the velocity colour");
    }

    RIGIDBODIES_TEST("several selected objects show no single-body handles")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto first = add_circle(session, { -0.5, 0.0 });
        const auto second = add_circle(session, { 0.5, 0.0 });
        session.set_selections({ first, second });
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(std::none_of(model.handles.begin(), model.handles.end(), [](const auto& handle)
                               {
                                   return handle.id == "velocity" || handle.id == "rotation";
                               }),
            "a group publishes neither a velocity nor a rotation handle");
        const auto centre = session.camera().world_to_screen({ 0.5, 0.0 });
        drag(session, centre + math::Vec2 { app::overlay::velocity_knob_park_distance, 0.0 }, centre + math::Vec2 { 60.0, 0.0 });
        for (const auto id : { first, second })
            RIGIDBODIES_EXPECT(session.world().find_body(id)->linear_velocity_m_s() == math::Vec2 {} && session.world().find_body(id)->position_m().x != (id == first ? -0.5 : 0.5), "a press where one member's knob would be moves the whole group");
    }

    RIGIDBODIES_TEST("present mode enlarges stage text, chips and handles together")
    {
        app::SimulationSession session;
        paused_lab(session);
        session.set_focus_rect({ 100.0, 60.0, 900.0, 600.0 });
        const auto compass = [&]()
        {
            for (const auto& handle : session.build_model().handles)
                if (handle.id == "gravity")
                    return handle.screen_bounds.width;
            return 0.0;
        };
        render::DrawList list;
        session.render(list);
        const auto ordinary_text = session.scene_settings().text_scale;
        const auto ordinary_dial = compass();
        session.set_presenting(true);
        session.render(list);
        RIGIDBODIES_EXPECT_NEAR(session.scene_settings().text_scale, ordinary_text * app::overlay::present_scale, 1.0e-6, "stage labels grow by the Present factor");
        RIGIDBODIES_EXPECT_NEAR(compass(), ordinary_dial * app::overlay::present_scale, 1.0e-9, "the gravity dial grows with them");
        session.set_presenting(false);
        session.render(list);
        RIGIDBODIES_EXPECT(session.scene_settings().text_scale == ordinary_text, "leaving Present restores the ordinary size");
    }

    RIGIDBODIES_TEST("no gravity dial in relativity experiments")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect({ 100.0, 60.0, 1100.0, 760.0 });
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("chasing_light") && session.relativity_active(), "Chasing light loads");
        session.stepper().set_paused(true);
        render::DrawList list;
        session.render(list);
        const auto dial = app::gravity_compass_centre(session.camera(), 1.0);
        const auto reach = app::overlay::compass_radius + 4.0;
        for (const auto& command : list.commands())
        {
            if (command.layer != app::overlay::overlay_layer)
                continue;
            RIGIDBODIES_EXPECT(command.kind != render::DrawCommandKind::text || std::string_view(list.text_buffer()).substr(command.text_offset, command.text_length) != "g", "the dial's g is not drawn");
            if (command.kind == render::DrawCommandKind::circle_fill || command.kind == render::DrawCommandKind::circle_outline)
                RIGIDBODIES_EXPECT(math::length(list.vertices()[command.vertex_offset] - dial) > reach, "nothing is drawn as a dial in the stage corner");
        }
        for (const auto& [minimum, maximum] : session.scene_renderer().overlay_areas())
            RIGIDBODIES_EXPECT(!(dial.x >= minimum.x && dial.x <= maximum.x && dial.y >= minimum.y && dial.y <= maximum.y), "no plate keeps clear of a dial that is not there");
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(std::none_of(model.handles.begin(), model.handles.end(), [](const ui::StageHandle& handle)
                               {
                                   return handle.id == "gravity";
                               }),
            "no gravity handle is published");
        RIGIDBODIES_EXPECT(session.scene_renderer().relativity_stage_layout().has_value(), "the apparatus is drawn instead");
    }

    RIGIDBODIES_TEST("Present enlarges the stage clocks by 1.4")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        const render::ScreenRect focus { 100.0, 60.0, 1300.0, 780.0 };
        session.set_focus_rect(focus);
        session.configure({});
        RIGIDBODIES_EXPECT(session.load_scenario("chasing_light"), "Chasing light loads");
        session.set_focus_rect(focus);
        render::DrawList list;
        session.render(list);
        const auto ordinary = *session.scene_renderer().relativity_stage_layout();
        session.set_presenting(true);
        session.set_focus_rect(focus);
        session.render(list);
        const auto presented = *session.scene_renderer().relativity_stage_layout();
        RIGIDBODIES_EXPECT(ordinary.tier == render::RelativityStageTier::full && presented.tier == render::RelativityStageTier::full, "a large stage keeps the full tier in Present");
        RIGIDBODIES_EXPECT_NEAR(presented.clock_radius, ordinary.clock_radius * app::overlay::present_scale, 1.0e-6, "both clock faces grow by the Present factor");
        RIGIDBODIES_EXPECT_NEAR(presented.probe_radius, ordinary.probe_radius * app::overlay::present_scale, 1.0e-6, "and so does the probe");
        RIGIDBODIES_EXPECT(presented.band.height > ordinary.band.height && presented.fixed_plates.front().height > ordinary.fixed_plates.front().height, "the band and its plates grow with the text");
        session.set_presenting(false);
        session.set_focus_rect(focus);
        session.render(list);
        RIGIDBODIES_EXPECT(session.scene_renderer().relativity_stage_layout()->clock_radius == ordinary.clock_radius, "leaving Present restores the ordinary size");
    }

    RIGIDBODIES_TEST("stage labels are placed clear of the gravity dial")
    {
        app::SimulationSession session;
        paused_lab(session);
        session.set_focus_rect({ 100.0, 60.0, 900.0, 600.0 });
        const auto dial = app::gravity_compass_centre(session.camera(), 1.0);
        physics::BodyDefinition definition;
        definition.position_m = session.camera().screen_to_world(dial + math::Vec2 { -55.0, 12.0 });
        definition.name = "target";
        physics::Collider collider;
        collider.shape = physics::make_circle(0.05);
        definition.colliders.push_back(collider);
        session.world().create_body(definition);
        session.scene_settings().layers.set(render::VisualizationLayer::labels, true);
        render::DrawList list;
        session.render(list);
        const auto* label = drawn_text(list, "Target");
        RIGIDBODIES_EXPECT(label != nullptr, "the body is labelled");
        const auto origin = list.vertices()[label->vertex_offset];
        const auto reach = app::overlay::compass_radius + 2.0;
        const auto overlaps = origin.x < dial.x + reach && origin.x + 30.0 > dial.x - reach && origin.y < dial.y + reach && origin.y + 10.0 > dial.y - reach;
        RIGIDBODIES_EXPECT(!overlaps, "the label moves out from under the dial");
    }

    RIGIDBODIES_TEST("a hover card stands in for the hovered body's label and reports its motion")
    {
        app::SimulationSession session;
        paused_lab(session);
        const auto id = add_circle(session, {});
        session.world().find_body(id)->set_linear_velocity({ 1.5, 0.0 });
        session.scene_settings().layers.set(render::VisualizationLayer::labels, true);
        session.set_wall_time(10.0);
        ui::UiEvent move;
        move.kind = ui::UiEventKind::pointer_move;
        move.pointer_px = session.camera().world_to_screen({});
        session.handle_scene_event(move);
        render::DrawList list;
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Target") != nullptr, "the label stays until the card appears");
        session.set_wall_time(10.7);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.hover && model.hover->card_lines.size() == 3 && model.hover->card_lines.front() == "Target", "the card names the object");
        RIGIDBODIES_EXPECT(model.hover->card_lines[1].find(core::format_quantity(1.5, core::DisplayQuantity::velocity, core::DisplayUnits::si)) != std::string::npos, "the card reports the live speed the label does not");
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Target") == nullptr, "the card replaces the label it would cover");
        session.set_hover_cards_allowed(false);
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Target") != nullptr, "with no room for a card the label returns");
    }

    RIGIDBODIES_TEST("drawing dims the scene beneath the scale bar and leaves no pointer tool active")
    {
        app::SimulationSession session;
        paused_lab(session);
        session.set_focus_rect({ 100.0, 60.0, 900.0, 600.0 });
        session.scene_settings().layers.set(render::VisualizationLayer::labels, true);
        ui::UiCommand draw;
        draw.kind = ui::UiCommandKind::start_new_shape;
        session.apply(draw);
        RIGIDBODIES_EXPECT(session.build_model().interaction_mode.empty(), "the tool switch shows no active tool while drawing");
        render::DrawList list;
        session.render(list);
        int veil = std::numeric_limits<int>::max(), bar = std::numeric_limits<int>::min();
        for (const auto& draw_command : list.commands())
        {
            if (draw_command.kind == render::DrawCommandKind::rectangle_fill && draw_command.vertex_count == 2 && list.vertices()[draw_command.vertex_offset + 1].x >= 1000.0)
                veil = draw_command.layer;
            if (draw_command.kind == render::DrawCommandKind::text && std::string_view(list.text_buffer()).substr(draw_command.text_offset, draw_command.text_length).find("\xC2\xA0m") != std::string_view::npos)
                bar = draw_command.layer;
        }
        RIGIDBODIES_EXPECT(veil != std::numeric_limits<int>::max() && bar > veil, "the scale bar sorts above the drawing veil");
        RIGIDBODIES_EXPECT(veil > 20 && veil < app::overlay::overlay_layer, "the veil covers bodies and labels but not the editor");
        ui::UiCommand discard;
        discard.kind = ui::UiCommandKind::cancel_shape_outline;
        session.apply(discard);
        RIGIDBODIES_EXPECT(session.build_model().interaction_mode == "select", "the pointer tool returns once the draft is discarded");
    }

    RIGIDBODIES_TEST("handle geometry helpers invert each other")
    {
        render::Camera2D camera;
        camera.set_viewport({ 1000, 800 });
        camera.set_view_height(4.0);
        const math::Vec2 centre_m { 0.3, -0.2 }, velocity { -1.25, 2.5 };
        const auto tip = app::velocity_handle_tip(camera, centre_m, velocity, 24.0);
        const auto recovered = app::velocity_from_handle(camera, centre_m, tip, 24.0);
        RIGIDBODIES_EXPECT_NEAR(recovered.x, velocity.x, 1.0e-12, "x velocity survives the round trip");
        RIGIDBODIES_EXPECT_NEAR(recovered.y, velocity.y, 1.0e-12, "y velocity survives the round trip");
        for (const auto degrees : { -170.0, -45.0, 0.0, 30.0, 135.0 })
        {
            const math::Vec2 centre_px { 200.0, 150.0 };
            const auto knob = app::rotation_knob_position(centre_px, 40.0, math::degrees_to_radians(degrees));
            RIGIDBODIES_EXPECT_NEAR(math::radians_to_degrees(app::orientation_from_knob(centre_px, knob)), degrees, 1.0e-9, "orientation survives the knob round trip");
        }
        RIGIDBODIES_EXPECT(app::rotation_knob_position({ 0.0, 0.0 }, 10.0, 0.0).y < 0.0, "an unrotated body's knob sits above it on screen");
    }

    RIGIDBODIES_TEST("stage labels give way whole to the hover card rather than showing past its edge")
    {
        app::SimulationSession session;
        paused_lab(session);
        session.set_focus_rect({ 100.0, 60.0, 900.0, 600.0 });
        session.scene_settings().layers.set(render::VisualizationLayer::labels, true);
        const auto hovered = add_circle(session, { -0.6, 0.0 });
        physics::BodyDefinition definition;
        definition.position_m = { 0.6, 0.0 };
        definition.name = "neighbour";
        physics::Collider collider;
        collider.shape = physics::make_circle(0.2);
        definition.colliders.push_back(collider);
        session.world().create_body(definition);
        session.set_wall_time(10.0);
        ui::UiEvent move;
        move.kind = ui::UiEventKind::pointer_move;
        move.pointer_px = session.camera().world_to_screen(session.world().find_body(hovered)->world_center_of_mass_m());
        session.handle_scene_event(move);
        session.set_wall_time(10.7);
        render::DrawList list;
        session.render(list);
        const auto* label = drawn_text(list, "Neighbour");
        RIGIDBODIES_EXPECT(label != nullptr, "the neighbour is labelled");
        const auto origin = list.vertices()[label->vertex_offset];
        // The interface reports its card over the neighbour's label.
        const render::ScreenRect card { origin.x - 20.0, origin.y - 30.0, 160.0, 60.0 };
        session.set_hover_card_area(card);
        session.render(list);
        if (const auto* moved = drawn_text(list, "Neighbour"))
        {
            const auto at = list.vertices()[moved->vertex_offset];
            const auto overlaps = at.x < card.left + card.width && at.x + 50.0 > card.left && at.y - 12.0 < card.top + card.height && at.y + 4.0 > card.top;
            RIGIDBODIES_EXPECT(!overlaps, "the label moves clear of the card instead of peeking out from under it");
        }
        // Once the pointer leaves, a card that is no longer shown reserves nothing.
        move.pointer_px = { 990.0, 650.0 };
        session.handle_scene_event(move);
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Neighbour") != nullptr && drawn_text(list, "Target") != nullptr, "both labels are back once the card is gone");
    }

    RIGIDBODIES_TEST("beside a hover card another object's label may sit against the card's edge rather than leave the stage")
    {
        physics::World world;
        for (const auto& [name, x] : { std::pair { "hovered", -1.0 }, std::pair { "neighbour", 1.0 } })
        {
            physics::BodyDefinition definition;
            definition.position_m = { x, 0.0 };
            definition.name = name;
            physics::Collider collider;
            collider.shape = physics::make_circle(0.2);
            definition.colliders.push_back(collider);
            world.create_body(definition);
        }
        physics::BodyId hovered;
        world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.name() == "hovered")
                    hovered = id;
            });
        render::Camera2D camera;
        camera.set_viewport({ 1000, 800 });
        camera.set_view_height(4.0);
        render::SceneRenderSettings settings;
        settings.layers = render::LayerMask {};
        settings.layers.set(render::VisualizationLayer::bodies, true);
        settings.layers.set(render::VisualizationLayer::labels, true);
        render::SceneRenderer renderer;
        render::DrawList list;
        renderer.render(world, camera, settings, list);
        const auto* label = drawn_text(list, "Neighbour");
        RIGIDBODIES_EXPECT(label != nullptr, "the neighbour is labelled");
        if (!label)
            return;
        const auto origin = list.vertices()[label->vertex_offset];
        std::optional<std::pair<math::Vec2, math::Vec2>> plate;
        for (const auto& draw : list.commands())
            if (draw.kind == render::DrawCommandKind::rounded_rectangle_fill && draw.vertex_count == 2)
            {
                const auto low = list.vertices()[draw.vertex_offset], high = list.vertices()[draw.vertex_offset + 1];
                if (origin.x >= low.x && origin.x <= high.x && origin.y >= low.y - 1.0 && origin.y <= high.y + 1.0)
                    plate = std::pair { low, high };
            }
        RIGIDBODIES_EXPECT(plate.has_value(), "the label sits on a plate");
        if (!plate)
            return;
        // Overlays fill the whole stage but for a one-pixel ring around the plate, so no place
        // keeps the usual spacing from them and only the plate's own place is clear of them.
        const auto [low, high] = *plate;
        const std::vector<std::pair<math::Vec2, math::Vec2>> around { { { 0.0, 0.0 }, { 1000.0, low.y - 1.0 } }, { { 0.0, high.y + 1.0 }, { 1000.0, 800.0 } }, { { 0.0, low.y - 1.0 }, { low.x - 1.0, high.y + 1.0 } }, { { high.x + 1.0, low.y - 1.0 }, { 1000.0, high.y + 1.0 } } };
        renderer.set_overlay_areas(around);
        renderer.render(world, camera, settings, list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Neighbour") == nullptr, "without a card, a label with no clear place gives way as before");
        settings.label_replaced = hovered;
        settings.hover = hovered;
        renderer.render(world, camera, settings, list);
        const auto* kept = drawn_text(list, "Neighbour");
        RIGIDBODIES_EXPECT(kept != nullptr, "while a card describes another object, the label keeps its place against the overlay's edge");
        if (kept)
        {
            RIGIDBODIES_EXPECT_NEAR(list.vertices()[kept->vertex_offset].x, origin.x, 0.5, "the label is not moved under the card");
            RIGIDBODIES_EXPECT_NEAR(list.vertices()[kept->vertex_offset].y, origin.y, 0.5, "the label is not moved under the card");
        }
        RIGIDBODIES_EXPECT(drawn_text(list, "Hovered") == nullptr, "the card stands in for its own body's name");
    }

    RIGIDBODIES_TEST("weight and support arrows on stacked lanes stay inside their own lane")
    {
        const auto arrow_px = [](std::string_view scenario, double& lane_gap_px)
        {
            app::SimulationSession session;
            session.set_viewport({ 1600, 900 });
            session.set_focus_rect({ 36.0, 102.0, 1108.0, 732.0 });
            session.configure({});
            RIGIDBODIES_EXPECT(session.load_scenario(scenario), "the lesson loads");
            session.scene_settings().layers.set(render::VisualizationLayer::force_vectors, true);
            session.stepper().set_paused(false);
            for (int frame = 0; frame < 30; ++frame)
                session.advance(1.0 / 60.0);
            render::DrawList list;
            session.render(list);
            const auto gravity = math::length(session.world().settings().gravity_m_s2);
            double heaviest_n = 0.0;
            std::vector<double> levels;
            session.world().for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    if (body.type() == physics::BodyType::dynamic_body)
                        heaviest_n = std::max(heaviest_n, body.mass_properties().mass_kg * gravity);
                    else
                        levels.push_back(body.position_m().y);
                });
            std::sort(levels.begin(), levels.end());
            lane_gap_px = std::numeric_limits<double>::infinity();
            for (std::size_t index = 1; index < levels.size(); ++index)
                lane_gap_px = std::min(lane_gap_px, session.camera().world_to_screen_length(levels[index] - levels[index - 1]));
            return session.build_model().drawn_force_scale * heaviest_n;
        };
        double gap_px = 0.0;
        const auto lanes = arrow_px("friction_comparison", gap_px);
        RIGIDBODIES_EXPECT(std::isfinite(gap_px) && gap_px > 0.0, "the lanes are stacked one above another");
        RIGIDBODIES_EXPECT(lanes > 14.0 && 2.0 * lanes < 0.75 * gap_px, "a support arrow and the weight reaching down from the lane above leave room between them");
        // With no fixed surface above, arrows keep the length the stage gives them.
        const auto open = arrow_px("free_fall", gap_px);
        RIGIDBODIES_EXPECT(open > 1.5 * lanes, "a floor with nothing above it leaves the arrows their full length");
    }

    RIGIDBODIES_TEST("a body under the pointer wins over the connection anchored at its centre, and a connection's card stands by it")
    {
        app::SimulationSession session;
        session.set_viewport({ 1600, 900 });
        session.set_focus_rect({ 36.0, 102.0, 1108.0, 732.0 });
        session.configure({});
        double now = 10.0;
        const auto hover_at = [&](const math::Vec2& point)
        {
            now += 5.0;
            session.set_wall_time(now);
            ui::UiEvent move;
            move.kind = ui::UiEventKind::pointer_move;
            move.pointer_px = point;
            session.handle_scene_event(move);
            session.set_wall_time(now + 0.7);
            return session.build_model().hover;
        };
        const auto body_named = [&](std::string_view name)
        {
            physics::BodyId found;
            session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.name() == name)
                        found = id;
                });
            RIGIDBODIES_EXPECT(found.is_valid(), "the lesson has the named body");
            return found;
        };
        const auto centre_of = [&](physics::BodyId id)
        {
            return session.camera().world_to_screen(session.world().find_body(id)->world_center_of_mass_m());
        };
        const auto beside = [](const ui::HoverModel& hover, const math::Vec2& point)
        {
            const auto& box = hover.screen_bounds;
            return box.width > 0.0 && box.height > 0.0 && point.x >= box.left && point.x <= box.left + box.width && point.y >= box.top && point.y <= box.top + box.height;
        };

        RIGIDBODIES_EXPECT(session.load_scenario("spring_damping"), "the spring lesson loads");
        session.scene_settings().layers.set(render::VisualizationLayer::constraints, true);
        const auto mass = body_named("damped_mass");
        const auto on_mass = hover_at(centre_of(mass));
        RIGIDBODIES_EXPECT(on_mass && on_mass->kind == ui::StageTargetKind::object && on_mass->body == mass, "the mass under the pointer is hovered, not the spring anchored at its centre");
        std::optional<math::Vec2> coil;
        for (const auto id : session.world().spring_ids())
            if (const auto report = session.world().spring_report(id); report && report->second == mass)
                coil = (session.camera().world_to_screen(report->first_anchor_m) + session.camera().world_to_screen(report->second_anchor_m)) * 0.5;
        RIGIDBODIES_EXPECT(coil.has_value(), "the mass hangs on a spring");
        const auto on_coil = hover_at(*coil);
        RIGIDBODIES_EXPECT(on_coil && on_coil->kind == ui::StageTargetKind::connection && !on_coil->card_lines.empty(), "the coil between the bodies is the spring's");
        RIGIDBODIES_EXPECT(beside(*on_coil, *coil), "the spring's card is placed by the spring, not in the window's corner");

        RIGIDBODIES_EXPECT(session.load_scenario("distance_chain"), "the chain lesson loads");
        session.scene_settings().layers.set(render::VisualizationLayer::constraints, true);
        const auto link = body_named("chain_mass_1");
        const auto on_link = hover_at(centre_of(link));
        RIGIDBODIES_EXPECT(on_link && on_link->kind == ui::StageTargetKind::object && on_link->body == link, "a chain link's centre is the link, though its rods meet there");
        const auto rod = (centre_of(body_named("chain_anchor")) + centre_of(link)) * 0.5;
        const auto on_rod = hover_at(rod);
        RIGIDBODIES_EXPECT(on_rod && on_rod->kind == ui::StageTargetKind::connection && beside(*on_rod, rod), "the rod between the links is the joint, with its card by the rod");

        // Hinge, slider and weld symbols stay the joint's even where they sit over a body.
        for (const auto* lesson : { "welded_assembly", "revolute_drive", "prismatic_drive", "breakable_joint" })
        {
            RIGIDBODIES_EXPECT(session.load_scenario(lesson), "the joint lesson loads");
            session.scene_settings().layers.set(render::VisualizationLayer::constraints, true);
            std::size_t symbols = 0, reached = 0;
            for (const auto& handle : session.build_model().handles)
                if (handle.kind == ui::StageTargetKind::connection)
                {
                    ++symbols;
                    const auto hover = hover_at(handle.screen_bounds.center());
                    reached += hover && hover->kind == ui::StageTargetKind::connection ? 1u : 0u;
                }
            RIGIDBODIES_EXPECT(symbols > 0 && reached > 0, std::string("a joint symbol can be pointed at in ") + lesson);
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
