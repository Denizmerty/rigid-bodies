#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/force_generator.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{

    using namespace rigidbodies::physics;
    using namespace rigidbodies::render;
    using namespace rigidbodies::math;

    bool same_color(const Color& left, const Color& right)
    {
        return left.red == right.red && left.green == right.green && left.blue == right.blue && left.alpha == right.alpha;
    }

    const DrawCommand& find_command(const DrawList& list, DrawCommandKind kind, const Color& color)
    {
        for (const auto& command : list.commands())
        {
            if (command.kind == kind && same_color(command.color, color))
            {
                return command;
            }
        }
        RIGIDBODIES_FAIL("the expected render command exists");
    }

    void expect_vertex(const DrawList& list, const DrawCommand& command, std::size_t index, const Vec2& expected)
    {
        RIGIDBODIES_EXPECT(index < command.vertex_count, "command contains the expected vertex");
        const auto& actual = list.vertices().at(command.vertex_offset + index);
        RIGIDBODIES_EXPECT_NEAR(actual.x, expected.x, 1.0e-9, "rendered vertex x matches");
        RIGIDBODIES_EXPECT_NEAR(actual.y, expected.y, 1.0e-9, "rendered vertex y matches");
    }

    struct DrawnArrow
    {
        Vec2 start, tip;
    };

    // Arrows are a butt-capped shaft followed by a head: filled heads list their tip first and
    // open heads are a three-point chevron around it. The tip, not the shaft end, carries the
    // scaled magnitude.
    std::vector<DrawnArrow> arrows(const DrawList& list, const Color& color)
    {
        std::vector<DrawnArrow> result;
        bool awaiting_head = false;
        for (const auto& command : list.commands())
        {
            if (!same_color(command.color, color))
                continue;
            if (command.kind == DrawCommandKind::line && !awaiting_head)
            {
                result.push_back({ list.vertices().at(command.vertex_offset), {} });
                awaiting_head = true;
            }
            else if (awaiting_head && command.kind == DrawCommandKind::polygon_fill)
            {
                result.back().tip = list.vertices().at(command.vertex_offset);
                awaiting_head = false;
            }
            else if (awaiting_head && command.kind == DrawCommandKind::polyline && command.vertex_count == 3)
            {
                result.back().tip = list.vertices().at(command.vertex_offset + 1);
                awaiting_head = false;
            }
            else if (awaiting_head && command.kind == DrawCommandKind::polygon_outline && command.vertex_count == 3)
            {
                result.back().tip = list.vertices().at(command.vertex_offset);
                awaiting_head = false;
            }
        }
        RIGIDBODIES_EXPECT(!awaiting_head, "every arrow shaft has a head");
        return result;
    }

    // Whether one text command reads exactly this, rather than merely containing it.
    bool has_text(const DrawList& list, std::string_view wanted)
    {
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::text && std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length) == wanted)
                return true;
        return false;
    }

    // Loads start clear of the centre symbol along their own line, possibly stepped sideways
    // off an arrow already drawn there; velocity keeps its tail on the centre itself.
    void expect_from_center(const DrawnArrow& arrow, const Vec2& center, double clearance, const char* message)
    {
        const auto span = arrow.tip - arrow.start;
        RIGIDBODIES_EXPECT(length(span) > 0.0, message);
        const auto direction = span / length(span);
        const auto offset = arrow.start - center;
        RIGIDBODIES_EXPECT(dot(offset, direction) >= -1.0e-9 && dot(offset, direction) <= clearance + 1.0e-9, message);
        RIGIDBODIES_EXPECT(std::abs(cross(offset, direction)) <= 12.0 + 1.0e-9, message);
    }

    WorldSettings zero_gravity()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        return settings;
    }

    Camera2D make_camera()
    {
        Camera2D camera;
        camera.set_viewport({ 1000, 800 });
        camera.set_view_height(8.0);
        return camera;
    }

    RIGIDBODIES_TEST("circle geometry, overlays, labels and vectors share the interpolated mass centre")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        definition.name = "offset disc";
        definition.position_m = { 2.0, 3.0 };
        definition.linear_velocity_m_s = { -6.0, -2.0 };
        definition.angular_velocity_rad_s = pi;
        Collider collider;
        collider.shape = make_circle(0.25);
        collider.local_transform.translation = { 1.5, 0.5 };
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(1.0);
        world.find_body(id)->apply_force_at_center({ 10.0, 0.0 });
        world.step(1.0);

        const auto camera = make_camera();
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        for (const auto layer : { VisualizationLayer::bodies, VisualizationLayer::outlines, VisualizationLayer::center_of_mass, VisualizationLayer::bounding_boxes, VisualizationLayer::selection, VisualizationLayer::labels, VisualizationLayer::velocity_vectors, VisualizationLayer::force_vectors, VisualizationLayer::acceleration_vectors, VisualizationLayer::momentum_vectors, VisualizationLayer::angular_velocity })
        {
            settings.layers.set(layer, true);
        }
        settings.interpolation_alpha = 0.5;
        settings.selection = id;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);

        const auto center = camera.world_to_screen({ 5.5, 2.5 });
        expect_vertex(list, find_command(list, DrawCommandKind::circle_fill, settings.theme.body_fill), 0, center);
        expect_vertex(list, find_command(list, DrawCommandKind::circle_outline, settings.theme.body_outline), 0, center);
        expect_vertex(list, find_command(list, DrawCommandKind::circle_fill, settings.theme.center_of_mass), 0, center);
        expect_vertex(list, find_command(list, DrawCommandKind::line, settings.theme.body_outline.with_alpha(0.6f)), 1, camera.world_to_screen({ 5.5, 2.75 }));

        const auto& bounds = find_command(list, DrawCommandKind::rectangle_outline, settings.theme.bounds);
        expect_vertex(list, bounds, 0, { center.x - 25.0, center.y - 25.0 });
        expect_vertex(list, bounds, 1, { center.x + 25.0, center.y + 25.0 });
        // Selection follows the actual outline: a ring just outside the disc, not a box.
        const auto& selection = find_command(list, DrawCommandKind::circle_outline, settings.theme.selection);
        expect_vertex(list, selection, 0, center);
        RIGIDBODIES_EXPECT(selection.radius > 25.0f && selection.radius < 29.0f && selection.layer == 5, "selection ring hugs the shape above the body plane");
        // The name sits on a plate beside the disc rather than over it.
        const auto& name = find_command(list, DrawCommandKind::text, settings.theme.label_text);
        RIGIDBODIES_EXPECT(list.text_buffer().substr(name.text_offset, name.text_length) == "Offset disc", "the body label leads with its name");
        bool plate_clear = false;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::rounded_rectangle_fill && same_color(command.color, settings.theme.label_plate))
            {
                const auto minimum = list.vertices().at(command.vertex_offset), maximum = list.vertices().at(command.vertex_offset + 1);
                const auto nearest = Vec2 { std::clamp(center.x, minimum.x, maximum.x), std::clamp(center.y, minimum.y, maximum.y) };
                const auto& text = list.vertices().at(name.vertex_offset);
                if (text.x >= minimum.x && text.x <= maximum.x && text.y >= minimum.y - 4.0 && text.y <= maximum.y)
                    plate_clear = length(nearest - center) >= 25.0;
            }
        RIGIDBODIES_EXPECT(plate_clear, "the name plate does not cover the object it describes");

        expect_vertex(list, find_command(list, DrawCommandKind::line, settings.theme.velocity), 0, center);
        for (const auto color : { settings.theme.force, settings.theme.acceleration, settings.theme.momentum })
            expect_from_center(arrows(list, color).front(), center, 6.0, "each load arrow leaves the mass centre clear of its symbol");
        expect_vertex(list, find_command(list, DrawCommandKind::polyline, settings.theme.momentum.with_alpha(0.85f)), 0, { center.x + 18.0, center.y });
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->world_center_of_mass_m().x, 7.5, 1.0e-12, "rendering leaves simulation at its current pose");
    }

    RIGIDBODIES_TEST("polygon and segment vertices use the interpolated translation and rotation")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        definition.linear_velocity_m_s = { 4.0, 0.0 };
        definition.angular_velocity_rad_s = pi;
        Collider collider;
        collider.shape = make_box(2.0, 1.0);
        definition.colliders.push_back(collider);
        const auto box_id = world.create_body(definition);
        RIGIDBODIES_EXPECT(world.is_valid(box_id), "the polygon body exists");
        definition.position_m = { 0.0, 3.0 };
        definition.colliders.front().shape = make_segment({ 0.0, 0.0 }, { 2.0, 0.0 });
        definition.type = BodyType::kinematic_body;
        const auto segment_id = world.create_body(definition);
        RIGIDBODIES_EXPECT(world.is_valid(segment_id), "the segment body exists");
        world.step(1.0);

        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::outlines, true);
        settings.interpolation_alpha = 0.5;
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);

        const auto& polygon = find_command(list, DrawCommandKind::polygon_outline, settings.theme.body_outline);
        RIGIDBODIES_EXPECT(polygon.vertex_count == 4, "the box keeps four corners");
        expect_vertex(list, polygon, 0, camera.world_to_screen({ 2.5, -1.0 }));
        expect_vertex(list, polygon, 1, camera.world_to_screen({ 2.5, 1.0 }));
        expect_vertex(list, polygon, 2, camera.world_to_screen({ 1.5, 1.0 }));
        expect_vertex(list, polygon, 3, camera.world_to_screen({ 1.5, -1.0 }));
        const auto& segment = find_command(list, DrawCommandKind::polyline, settings.theme.kinematic_body_outline);
        expect_vertex(list, segment, 0, camera.world_to_screen({ 2.0, 3.0 }));
        expect_vertex(list, segment, 1, camera.world_to_screen({ 2.0, 5.0 }));
    }

    RIGIDBODIES_TEST("broad-phase connection endpoints follow rendered bodies")
    {
        World world { zero_gravity() };
        // Deliberate overlap creates a broad-phase pair; isolate its interpolation from response.
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        BodyDefinition definition;
        definition.linear_velocity_m_s = { 4.0, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.25);
        definition.colliders.push_back(collider);
        const auto first_id = world.create_body(definition);
        definition.position_m.x = 0.2;
        const auto second_id = world.create_body(definition);
        RIGIDBODIES_EXPECT(world.is_valid(first_id) && world.is_valid(second_id), "both bodies exist");
        world.step(1.0);

        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::broad_phase_pairs, true);
        settings.interpolation_alpha = 0.5;
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);

        RIGIDBODIES_EXPECT(list.commands().size() == 1, "one candidate pair is visualized");
        expect_vertex(list, list.commands().front(), 0, camera.world_to_screen({ 2.0, 0.0 }));
        expect_vertex(list, list.commands().front(), 1, camera.world_to_screen({ 2.2, 0.0 }));
    }

    RIGIDBODIES_TEST("linear spring coils follow interpolated local anchors and report extension and force")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        definition.linear_velocity_m_s = { 2.0, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto first = world.create_body(definition);
        definition.position_m = { 2.0, 0.0 };
        definition.linear_velocity_m_s = { 4.0, 0.0 };
        const auto second = world.create_body(definition);
        LinearSpringDefinition spring;
        spring.first = first;
        spring.second = second;
        spring.local_anchor_first_m = { 0.2, 0.1 };
        spring.local_anchor_second_m = { -0.2, -0.1 };
        spring.stiffness_n_m = 0.0;
        world.create_spring(spring);
        world.step(0.1);
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::constraints, true);
        settings.interpolation_alpha = 0.5;
        DrawList list;
        SceneRenderer renderer;
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        // The coil and its end caps are in the neutral connection tone so arrows along it stay
        // readable; whether it is stretched or squeezed is said in words.
        const auto& coil = find_command(list, DrawCommandKind::polyline, settings.theme.panel_muted);
        expect_vertex(list, coil, 0, camera.world_to_screen({ 0.3, 0.1 }));
        expect_vertex(list, coil, coil.vertex_count - 1, camera.world_to_screen({ 2.0, -0.1 }));
        bool force_hued_coil = false;
        for (const auto& command : list.commands())
            force_hued_coil = force_hued_coil || ((command.kind == DrawCommandKind::polyline || command.kind == DrawCommandKind::line || command.kind == DrawCommandKind::circle_outline) && same_color(command.color, settings.theme.force));
        RIGIDBODIES_EXPECT(!force_hued_coil, "the force hue is kept for arrows");
        expect_vertex(list, find_command(list, DrawCommandKind::circle_outline, settings.theme.panel_muted), 0, camera.world_to_screen({ 0.3, 0.1 }));
        RIGIDBODIES_EXPECT(list.text_buffer().find("extension") != std::string::npos && list.text_buffer().find("pull") != std::string::npos, "spring readout shows extension and force even when body labels are off");
        RIGIDBODIES_EXPECT(LayerMask::defaults().is_enabled(VisualizationLayer::constraints), "spring connections are visible on first opening a demonstration");
    }

    RIGIDBODIES_TEST("torsional spring arc follows unwrapped interpolated angle and reports torque")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        definition.type = BodyType::static_body;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto first = world.create_body(definition);
        definition.type = BodyType::dynamic_body;
        definition.position_m = { 2.0, 0.0 };
        definition.angular_velocity_rad_s = 8.0 * pi;
        const auto second = world.create_body(definition);
        AngularSpringDefinition spring;
        spring.first = first;
        spring.second = second;
        spring.stiffness_n_m_rad = 0.0;
        world.create_spring(spring);
        world.step(0.25);
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::constraints, true);
        settings.interpolation_alpha = 0.5;
        DrawList list;
        SceneRenderer renderer;
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        const auto center = camera.world_to_screen({ 2.0, 0.0 });
        // The clock spring's inner end turns with the body: half a turn at the interpolated pose.
        // It winds two and a half turns in the connection tone, clear of the centre symbol.
        const auto& spiral = find_command(list, DrawCommandKind::polyline, settings.theme.panel_muted);
        expect_vertex(list, spiral, 0, center + Vec2 { -7.0, 0.0 });
        RIGIDBODIES_EXPECT(spiral.vertex_count == 97, "the spiral is fine enough to read as several turns");
        // The deflection arc runs from the rest angle to the unwrapped current angle.
        const DrawCommand* arc = nullptr;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::polyline && same_color(command.color, settings.theme.momentum) && command.vertex_count == 25)
                arc = &command;
        RIGIDBODIES_EXPECT(arc != nullptr, "the deflection is drawn as its own arc");
        expect_vertex(list, *arc, 0, center + Vec2 { 25.0, 0.0 });
        expect_vertex(list, *arc, arc->vertex_count - 1, center + Vec2 { -25.0, 0.0 });
        RIGIDBODIES_EXPECT(list.text_buffer().find("angle") != std::string::npos && list.text_buffer().find("torque") != std::string::npos, "torsional readout shows angle and torque");
    }

    RIGIDBODIES_TEST("disabled springs and destroyed endpoints leave no stale connection drawing")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        const auto first = world.create_body(definition);
        definition.position_m = { 1.0, 0.0 };
        const auto second = world.create_body(definition);
        LinearSpringDefinition spring;
        spring.first = first;
        spring.second = second;
        const auto id = world.create_spring(spring);
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::constraints, true);
        SceneRenderer renderer;
        DrawList list;
        world.set_spring_enabled(id, false);
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "disabled spring is hidden");
        world.set_spring_enabled(id, true);
        world.destroy_body(second);
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "destroying either endpoint removes the connection");
    }

    std::pair<BodyId, BodyId> moving_joint_endpoints(World& world)
    {
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto first = world.create_body(definition);
        definition.position_m = { 2.0, 0.0 };
        const auto second = world.create_body(definition);
        world.find_body(first)->set_simulated_pose({ 2.0, 1.0 }, pi);
        world.find_body(second)->set_simulated_pose({ 4.0, 2.0 }, -pi * 0.5);
        return { first, second };
    }

    SceneRenderSettings connections_only()
    {
        SceneRenderSettings result;
        result.depth_background = false;
        result.layers = LayerMask::none();
        result.layers.set(VisualizationLayer::constraints, true);
        result.interpolation_alpha = 0.5;
        return result;
    }

    RIGIDBODIES_TEST("joint rods and endpoint markers use local anchors on interpolated body poses")
    {
        World world { zero_gravity() };
        const auto [first, second] = moving_joint_endpoints(world);
        DistanceJointDefinition definition;
        definition.first = first;
        definition.second = second;
        definition.local_anchor_first_m = { 0.3, 0.1 };
        definition.local_anchor_second_m = { -0.2, 0.15 };
        world.add_constraint(std::make_shared<JointConstraint>(definition), "render_rod");
        auto settings = connections_only();
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        const auto original_first = world.find_body(first)->transform();
        for (const auto alpha : { 0.0, 0.5, 1.0 })
        {
            settings.interpolation_alpha = alpha;
            renderer.render(world, camera, settings, list);
            const auto expected_first = camera.world_to_screen(transform_point(world.find_body(first)->interpolated_transform(alpha), definition.local_anchor_first_m));
            const auto expected_second = camera.world_to_screen(transform_point(world.find_body(second)->interpolated_transform(alpha), definition.local_anchor_second_m));
            const auto& rod = find_command(list, DrawCommandKind::line, settings.theme.panel_accent);
            expect_vertex(list, rod, 0, expected_first);
            expect_vertex(list, rod, 1, expected_second);
            std::size_t markers = 0;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::circle_fill && same_color(command.color, settings.theme.panel_accent))
                    expect_vertex(list, command, 0, markers++ == 0 ? expected_first : expected_second);
            RIGIDBODIES_EXPECT(markers == 2, "both attachment points remain visible");
            RIGIDBODIES_EXPECT(list.text_buffer().find("Rod") == std::string::npos, "an unloaded rod stays quiet until one of its bodies is singled out");
        }
        settings.selection = second;
        renderer.render(world, camera, settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("Rod") != std::string::npos, "the selection's rod is named");
        RIGIDBODIES_EXPECT(list.text_buffer().find("N\xC2\xB7m") == std::string::npos, "a rod reports no torque it cannot carry");
        RIGIDBODIES_EXPECT(world.find_body(first)->transform().translation == original_first.translation, "render interpolation never edits physics placement");
    }

    RIGIDBODIES_TEST("hinge limit and motor arcs follow the interpolated pin and reference orientation")
    {
        World world { zero_gravity() };
        const auto [first, second] = moving_joint_endpoints(world);
        RevoluteJointDefinition definition;
        definition.first = first;
        definition.second = second;
        definition.local_anchor_first_m = { 0.2, 0.1 };
        definition.local_anchor_second_m = { 0.0, 0.3 };
        definition.reference_angle_rad = 0.2;
        definition.limits_enabled = true;
        definition.lower_angle_rad = -0.4;
        definition.upper_angle_rad = 0.7;
        definition.motor_enabled = true;
        definition.motor_speed_rad_s = -1.0;
        definition.maximum_motor_torque_n_m = 2.0;
        world.add_constraint(std::make_shared<JointConstraint>(definition), "render_hinge");
        auto settings = connections_only();
        settings.hover = second;
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto a = camera.world_to_screen(transform_point(world.find_body(first)->interpolated_transform(0.5), definition.local_anchor_first_m));
        const auto b = camera.world_to_screen(transform_point(world.find_body(second)->interpolated_transform(0.5), definition.local_anchor_second_m));
        const auto pin = (a + b) * 0.5;
        expect_vertex(list, find_command(list, DrawCommandKind::circle_outline, settings.theme.panel_accent), 0, pin);
        const auto reference = world.find_body(first)->interpolated_orientation_rad(0.5) + definition.reference_angle_rad - half_pi;
        const auto& stops = find_command(list, DrawCommandKind::polyline, settings.theme.contact);
        const auto start = reference + definition.lower_angle_rad;
        const auto end = reference + definition.upper_angle_rad;
        expect_vertex(list, stops, 0, pin + Vec2 { std::cos(start), -std::sin(start) } * 32.0);
        expect_vertex(list, stops, stops.vertex_count - 1, pin + Vec2 { std::cos(end), -std::sin(end) } * 32.0);
        // The drive is drawn in the connection's hue, so it never reads as a body's velocity.
        const auto& motor = find_command(list, DrawCommandKind::polyline, settings.theme.panel_accent);
        const auto motor_end = reference - 0.9 * pi;
        expect_vertex(list, motor, motor.vertex_count - 1, pin + Vec2 { std::cos(motor_end), -std::sin(motor_end) } * 20.0);
        for (const auto& command : list.commands())
            RIGIDBODIES_EXPECT(!same_color(command.color, settings.theme.velocity), "no part of a connection takes the velocity hue");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Hinge motor") != std::string::npos, "motor state has a visible label");
    }

    RIGIDBODIES_TEST("slider rail and limit marks follow the first body's interpolated local axis")
    {
        World world { zero_gravity() };
        const auto [first, second] = moving_joint_endpoints(world);
        PrismaticJointDefinition definition;
        definition.first = first;
        definition.second = second;
        definition.local_anchor_first_m = { 0.2, 0.1 };
        definition.local_anchor_second_m = { 0.0, -0.1 };
        definition.local_axis_first = { 1.0, 0.0 };
        definition.limits_enabled = true;
        definition.lower_translation_m = -0.8;
        definition.upper_translation_m = 1.2;
        definition.motor_enabled = true;
        definition.motor_speed_m_s = 0.5;
        definition.maximum_motor_force_n = 5.0;
        world.add_constraint(std::make_shared<JointConstraint>(definition), "render_slider");
        auto settings = connections_only();
        settings.hover = second;
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto placement = world.find_body(first)->interpolated_transform(0.5);
        const auto anchor = transform_point(placement, definition.local_anchor_first_m);
        const auto axis = transform_direction(placement, definition.local_axis_first);
        const auto& rail = find_command(list, DrawCommandKind::line, settings.theme.panel_accent);
        expect_vertex(list, rail, 0, camera.world_to_screen(anchor + axis * definition.lower_translation_m));
        expect_vertex(list, rail, 1, camera.world_to_screen(anchor + axis * definition.upper_translation_m));
        const auto slider_point = camera.world_to_screen(transform_point(world.find_body(second)->interpolated_transform(0.5), definition.local_anchor_second_m));
        const auto& carriage = find_command(list, DrawCommandKind::rectangle_outline, settings.theme.panel_accent);
        expect_vertex(list, carriage, 0, slider_point - Vec2 { 6.0, 6.0 });
        expect_vertex(list, carriage, 1, slider_point + Vec2 { 6.0, 6.0 });
        RIGIDBODIES_EXPECT(list.text_buffer().find("Slider motor") != std::string::npos, "slider drive is distinguished from a free rail");
        (void)find_command(list, DrawCommandKind::line, settings.theme.contact);
        // The drive arrow runs along the rail ahead of the carriage, in the connection's hue,
        // where a weight hanging below the carriage's centre never crosses it.
        const auto screen_axis = normalized(camera.world_to_screen_direction(axis));
        const auto carriage_bounds = world.find_body(second)->compute_bounds(world.find_body(second)->interpolated_transform(0.5));
        double carriage_reach = 0.0;
        for (const auto corner : { carriage_bounds.minimum, carriage_bounds.maximum, Vec2 { carriage_bounds.minimum.x, carriage_bounds.maximum.y }, Vec2 { carriage_bounds.maximum.x, carriage_bounds.minimum.y } })
            carriage_reach = std::max(carriage_reach, dot(camera.world_to_screen(corner) - slider_point, screen_axis));
        bool drive_ahead = false;
        for (const auto& command : list.commands())
        {
            RIGIDBODIES_EXPECT(!same_color(command.color, settings.theme.velocity), "no part of a connection takes the velocity hue");
            if (command.kind == DrawCommandKind::line && same_color(command.color, settings.theme.panel_accent))
            {
                const auto from = list.vertices().at(command.vertex_offset);
                drive_ahead = drive_ahead || (dot(from - slider_point, screen_axis) >= carriage_reach && std::abs(cross(from - slider_point, screen_axis)) < 1.0);
            }
        }
        RIGIDBODIES_EXPECT(drive_ahead, "the drive arrow starts on the rail beyond the carriage's leading edge");
    }

    RIGIDBODIES_TEST("weld marker disabled joints and stale endpoints have distinct finite drawings")
    {
        World world { zero_gravity() };
        const auto [first, second] = moving_joint_endpoints(world);
        WeldJointDefinition definition;
        definition.first = first;
        definition.second = second;
        definition.local_anchor_first_m = { 0.1, 0.2 };
        definition.local_anchor_second_m = { -0.1, -0.2 };
        const auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, "render_weld");
        auto settings = connections_only();
        settings.hover = second;
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto a = camera.world_to_screen(transform_point(world.find_body(first)->interpolated_transform(0.5), definition.local_anchor_first_m));
        const auto b = camera.world_to_screen(transform_point(world.find_body(second)->interpolated_transform(0.5), definition.local_anchor_second_m));
        const auto midpoint = (a + b) * 0.5;
        // The weld point lies outside both discs, so a rigid bracket reaches each of them.
        std::size_t brackets = 0;
        const DrawCommand* diamond = nullptr;
        for (const auto& command : list.commands())
        {
            if (command.kind == DrawCommandKind::polygon_fill && same_color(command.color, settings.theme.panel_accent.with_alpha(0.85f)))
                ++brackets;
            if (command.kind == DrawCommandKind::polygon_outline && same_color(command.color, settings.theme.panel_accent) && command.vertex_count == 4 && list.vertices().at(command.vertex_offset) == midpoint + Vec2 { 0.0, -7.0 })
                diamond = &command;
        }
        RIGIDBODIES_EXPECT(brackets == 2, "a weld point outside both parts is bracketed to each of them");
        RIGIDBODIES_EXPECT(diamond != nullptr, "the weld point keeps its diamond");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Weld") != std::string::npos, "weld has a recognizable marker and label");
        settings.hover = {};
        joint->set_enabled(false);
        renderer.render(world, camera, settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("Joint disabled") != std::string::npos, "disabled relationship is drawn muted and explicitly labeled");
        for (const auto& vertex : list.vertices())
            RIGIDBODIES_EXPECT(is_finite(vertex), "disabled marker geometry remains finite");
        settings.layers.set(VisualizationLayer::constraints, false);
        renderer.render(world, camera, settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "Connections toggle suppresses all joint geometry");
        settings.layers.set(VisualizationLayer::constraints, true);
        world.destroy_body(second);
        renderer.render(world, camera, settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "removed endpoints never leave a stale pin or reaction arrow");
    }

    RIGIDBODIES_TEST("failed joints display their break load while intact links retain reaction arrows")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "breakable_joint"), "paired load demonstration exists");
        world.step(1.0 / 120.0);
        const auto weak = std::dynamic_pointer_cast<JointConstraint>(world.constraint_by_key("weak_link"));
        RIGIDBODIES_EXPECT(weak && weak->is_broken(), "weak link breaks under the initial gravity load");
        auto settings = connections_only();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("Broken at ") != std::string::npos && list.text_buffer().find("Rod \xC2\xB7") != std::string::npos, "failed and intact links remain distinguishable");
        // The load the link failed under and its authored limit are both stated, and apart, so the
        // failing load is never read as the limit.
        const auto broken_at = weak->broken_force_n();
        RIGIDBODIES_EXPECT(broken_at > 3.0, "the weak link failed above its limit");
        RIGIDBODIES_EXPECT(list.text_buffer().find("force limit 3.00\xC2\xA0N") != std::string::npos, "failure label states the authored force limit");
        const auto at = list.text_buffer().find("Broken at ");
        RIGIDBODIES_EXPECT(at != std::string::npos && std::abs(std::stod(list.text_buffer().substr(at + 10, 8)) - broken_at) <= 0.01 * broken_at, "failure label states the load the link broke under");
        // A joint's pull on a body is a load like any other: the force hue with the outlined head.
        const auto reaction_arrows = [&]
        {
            std::size_t heads = 0;
            for (const auto& command : list.commands())
                heads += command.kind == DrawCommandKind::polygon_outline && same_color(command.color, settings.theme.force) ? 1u : 0u;
            return heads;
        };
        RIGIDBODIES_EXPECT(reaction_arrows() == 0, "a link's pull is not drawn while neither of its bodies is singled out");
        // A broken link keeps only short torn ends in the muted connection tone, so the fallen
        // load never looks attached and no torn end reads as a force arrow.
        const auto torn_from = make_camera().world_to_screen(weak->report(world).first_anchor_m);
        std::size_t torn = 0;
        bool force_hued = false;
        for (const auto& command : list.commands())
        {
            force_hued = force_hued || (command.kind == DrawCommandKind::line && same_color(command.color, settings.theme.force));
            if (command.kind == DrawCommandKind::line && same_color(command.color, settings.theme.panel_muted) && length(list.vertices().at(command.vertex_offset) - torn_from) <= 24.0 + 1.0e-9 &&
                length(list.vertices().at(command.vertex_offset + 1) - torn_from) <= 24.0 + 1.0e-9 && length(list.vertices().at(command.vertex_offset + 1) - list.vertices().at(command.vertex_offset)) > 1.0)
                ++torn;
        }
        RIGIDBODIES_EXPECT(torn >= 1, "the anchor keeps a short torn end");
        RIGIDBODIES_EXPECT(!force_hued, "a broken link is never drawn in the force hue");
        // Singling out the intact link's load adds the rod's pull to that body's own picture.
        settings.layers.set(VisualizationLayer::force_vectors, true);
        const auto strong = std::dynamic_pointer_cast<JointConstraint>(world.constraint_by_key("strong_link"));
        RIGIDBODIES_EXPECT(strong && !strong->is_broken(), "the strong link holds");
        settings.selection = strong->second_body();
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(reaction_arrows() == 1, "intact support has a scaled reaction arrow on the singled-out body");
        for (const auto& vertex : list.vertices())
            RIGIDBODIES_EXPECT(is_finite(vertex), "broken-link gap and reaction drawings remain finite");
    }

    RIGIDBODIES_TEST("slider labels include first-anchor bending load and its torque failure reason")
    {
        for (const auto break_torque : { 10.0, 5.0 })
        {
            World world { zero_gravity() };
            BodyDefinition body;
            body.type = BodyType::static_body;
            Collider collider;
            collider.shape = make_circle(0.1);
            body.colliders.push_back(collider);
            const auto first = world.create_body(body);
            body.type = BodyType::dynamic_body;
            body.position_m = { 2.0, 0.0 };
            const auto second = world.create_body(body);
            PrismaticJointDefinition definition;
            definition.first = first;
            definition.second = second;
            definition.break_torque_n_m = break_torque;
            const auto joint = std::make_shared<JointConstraint>(definition);
            world.add_constraint(joint, "bending_guide");
            world.find_body(second)->apply_force_at_center({ 0.0, 3.0 });
            world.step(0.01);
            const auto report = joint->report(world);
            RIGIDBODIES_EXPECT_NEAR(report.reaction_torque_first_n_m, 6.0, 1.0e-6, "guide support carries the offset transverse load");
            RIGIDBODIES_EXPECT_NEAR(report.reaction_torque_n_m, 0.0, 1.0e-10, "slider end has no pure couple");
            SceneRenderer renderer;
            DrawList list;
            renderer.render(world, make_camera(), connections_only(), list);
            RIGIDBODIES_EXPECT(list.text_buffer().find("6.00\xC2\xA0N\xC2\xB7m") != std::string::npos && list.text_buffer().find("peak") == std::string::npos, "load label includes the larger first-anchor torque, named as a torque");
            if (break_torque < 6.0)
                RIGIDBODIES_EXPECT(list.text_buffer().find("Broken at 6.00\xC2\xA0N\xC2\xB7m \xC2\xB7 torque limit 5.00\xC2\xA0N\xC2\xB7m") != std::string::npos, "bending failure states the failing torque and its limit apart");
            else
                RIGIDBODIES_EXPECT(list.text_buffer().find("Slider \xC2\xB7") != std::string::npos, "unbroken guide retains its ordinary label");
        }
    }

    RIGIDBODIES_TEST("authored outlines render once per logical part at the interpolated local placement")
    {
        World world { zero_gravity() };
        Outline outline;
        outline.closed = true;
        for (const auto point : { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 0.0 }, Vec2 { 2.0, 1.0 }, Vec2 { 1.0, 1.0 }, Vec2 { 1.0, 2.0 }, Vec2 { 0.0, 2.0 } })
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        AuthoredPartDefinition part;
        part.shape = build_authored_shape(outline).shape;
        part.local_transform = Transform2 { { 0.3, -0.5 }, Rotation2 { 0.2 } };
        const auto id = create_authored_body(world, {}, { part });
        auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT(body->colliders().size() > 1, "fixture decomposes into multiple collision cells");
        body->set_simulated_pose({ 2.0, 1.0 }, 0.6);
        const auto shape = authored_parts(*body).front()->shape;
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.layers.set(VisualizationLayer::outlines, true);
        const auto camera = make_camera();
        SceneRenderer renderer;
        DrawList list;
        for (const auto alpha : { 0.0, 0.5, 1.0 })
        {
            settings.interpolation_alpha = alpha;
            renderer.render(world, camera, settings, list);
            const auto placement = concatenate(body->interpolated_transform(alpha), part.local_transform);
            std::size_t outlines = 0, fills = 0;
            for (const auto& command : list.commands())
            {
                if (command.kind == DrawCommandKind::polyline)
                {
                    ++outlines;
                    RIGIDBODIES_EXPECT(command.vertex_count == shape->render_outline.size() + 1, "outline closes without internal collision-cell seams");
                    for (std::size_t index = 0; index < shape->render_outline.size(); ++index)
                        expect_vertex(list, command, index, camera.world_to_screen(transform_point(placement, shape->render_outline[index])));
                    expect_vertex(list, command, shape->render_outline.size(), camera.world_to_screen(transform_point(placement, shape->render_outline.front())));
                }
                if (command.kind == DrawCommandKind::polygon_fill)
                {
                    RIGIDBODIES_EXPECT(command.vertex_count == 3, "concave fill uses valid triangles");
                    for (std::size_t index = 0; index < 3; ++index)
                        expect_vertex(list, command, index, camera.world_to_screen(transform_point(placement, shape->render_triangles[fills][index])));
                    ++fills;
                }
            }
            RIGIDBODIES_EXPECT(outlines == 1 && fills == shape->render_triangles.size(), "one logical boundary and one triangulated fill render per authored part");
            RIGIDBODIES_EXPECT(authored_parts(*body).front()->shape == shape, "rendering reuses cached geometry");
        }
    }

    RIGIDBODIES_TEST("fine curve rendering survives coarse collision approximation and snapshot restoration")
    {
        World world;
        load_scenario(world, "shape_workshop");
        BodyId curve;
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == "Curved profile")
                curve = id;
        const auto shape = authored_parts(*world.find_body(curve)).front()->shape;
        RIGIDBODIES_EXPECT(shape->render_outline.size() > shape->collision_outline.size(), "render profile retains finer sampled curvature");
        const auto checkpoint = world.snapshot();
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::outlines, true);
        SceneRenderer renderer;
        DrawList original, restored;
        renderer.render(world, make_camera(), settings, original);
        world.step(0.1);
        world.restore(checkpoint);
        renderer.render(world, make_camera(), settings, restored);
        RIGIDBODIES_EXPECT(original.vertices() == restored.vertices(), "restored logical grouping reproduces outline geometry exactly");
        RIGIDBODIES_EXPECT(original.commands().size() == restored.commands().size(), "snapshot does not duplicate authored outlines across collision cells");
        bool found_curve = false;
        for (const auto& command : restored.commands())
            found_curve = found_curve || (command.kind == DrawCommandKind::polyline && command.vertex_count == shape->render_outline.size() + 1);
        RIGIDBODIES_EXPECT(found_curve, "outline layer uses the fine curve mesh rather than collision hulls");
    }

    BodyId vector_body(World& world, const Vec2& velocity = { 3.0, 4.0 }, const Vec2& position = {})
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        Collider collider;
        collider.shape = make_circle(0.25);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(2.0);
        return id;
    }

    SceneRenderSettings vectors_only()
    {
        SceneRenderSettings settings;
        // These tests isolate physical geometry and measurement contracts; richer default
        // presentation has separate material, motion, lifetime and pixel coverage.
        settings.material_shading = false;
        settings.depth_background = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        for (const auto layer : { VisualizationLayer::velocity_vectors, VisualizationLayer::force_vectors, VisualizationLayer::acceleration_vectors, VisualizationLayer::momentum_vectors })
            settings.layers.set(layer, true);
        return settings;
    }

    RIGIDBODIES_TEST("arrow scales are logical pixels, so a denser display draws the same picture")
    {
        World world { zero_gravity() };
        vector_body(world, { 3.0, 4.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.vector_scales.velocity = 10.0;
        const auto measure = [&](float display_scale)
        {
            auto scaled = settings;
            scaled.display_scale = display_scale;
            SceneRenderer renderer;
            DrawList list;
            renderer.render(world, make_camera(), scaled, list);
            const auto drawn = arrows(list, settings.theme.velocity);
            RIGIDBODIES_EXPECT(drawn.size() == 1, "one velocity arrow at each density");
            RIGIDBODIES_EXPECT_NEAR(renderer.drawn_velocity_scale(), 10.0 * display_scale, 1.0e-9, "the published factor is in device pixels");
            return drawn.empty() ? 0.0 : length(drawn.front().tip - drawn.front().start);
        };
        const auto single = measure(1.0f);
        const auto twice = measure(2.0f);
        RIGIDBODIES_EXPECT_NEAR(single, 50.0, 1.0e-6, "5 m/s at 10 logical px per m/s");
        RIGIDBODIES_EXPECT_NEAR(twice, 2.0 * single, 1.0e-6, "twice the pixel density draws the arrow twice as many pixels long");
    }

    RIGIDBODIES_TEST("every quantity arrow reports its physical magnitude and independent manual scale")
    {
        World world { zero_gravity() };
        const auto id = vector_body(world);
        world.find_body(id)->apply_force_at_center({ 6.0, 8.0 });
        world.step(0.01);
        world.find_body(id)->set_linear_velocity({ 3.0, 4.0 });
        auto settings = vectors_only();
        settings.vector_scales.velocity = 10.0;
        settings.vector_scales.force = 7.0;
        settings.vector_scales.acceleration = 12.0;
        settings.vector_scales.momentum = 8.0;
        SceneRenderer renderer;
        DrawList list;
        const auto camera = make_camera();
        const auto start = camera.world_to_screen(world.find_body(id)->world_center_of_mass_m());
        renderer.render(world, camera, settings, list);
        const auto quantities = std::vector<std::pair<Color, double>> { { settings.theme.velocity, 50.0 }, { settings.theme.force, 70.0 }, { settings.theme.acceleration, 60.0 }, { settings.theme.momentum, 80.0 } };
        std::vector<Vec2> geometry_si;
        std::vector<Vec2> tails;
        for (const auto& expected : quantities)
        {
            const auto drawn = arrows(list, expected.first);
            RIGIDBODIES_EXPECT(drawn.size() == 1, "each quantity has one arrow");
            // Parallel quantities step sideways so each stays visible; velocity holds the centre.
            expect_from_center(drawn.front(), start, 0.0, "arrow leaves from the mass centre's line");
            if (same_color(expected.first, settings.theme.velocity))
            {
                RIGIDBODIES_EXPECT_NEAR(drawn.front().start.x, start.x, 1.0e-9, "velocity starts at the mass centre");
                RIGIDBODIES_EXPECT_NEAR(drawn.front().start.y, start.y, 1.0e-9, "velocity starts at the mass centre");
            }
            const auto span = drawn.front().tip - drawn.front().start;
            RIGIDBODIES_EXPECT_NEAR(span.x, 0.6 * expected.second, 1.0e-9, "arrow length is the scaled magnitude");
            RIGIDBODIES_EXPECT_NEAR(span.y, -0.8 * expected.second, 1.0e-9, "arrow length is the scaled magnitude");
            for (const auto& tail : tails)
                RIGIDBODIES_EXPECT(length(tail - drawn.front().start) > 1.0, "parallel arrows never share one line");
            tails.push_back(drawn.front().start);
            geometry_si.push_back(drawn.front().start);
            geometry_si.push_back(drawn.front().tip);
        }
        for (const auto text : { "v 5.00\xC2\xA0m/s", "Applied 10.0\xC2\xA0N", "a 5.00\xC2\xA0m/s\xC2\xB2", "p 10.0\xC2\xA0kg\xC2\xB7m/s" })
            RIGIDBODIES_EXPECT(list.text_buffer().find(text) != std::string::npos, "each arrow has its own quantity name, true magnitude and units");
        settings.display_units = rigidbodies::core::DisplayUnits::centimetre_gram;
        renderer.render(world, camera, settings, list);
        std::vector<Vec2> geometry_cgs;
        for (const auto& expected : quantities)
            for (const auto& drawn : arrows(list, expected.first))
            {
                geometry_cgs.push_back(drawn.start);
                geometry_cgs.push_back(drawn.tip);
            }
        // Value plates size to their text, so only the arrows themselves must be unit-independent.
        RIGIDBODIES_EXPECT(geometry_si == geometry_cgs, "unit choices change labels without changing arrow geometry or SI factors");
        for (const auto text : { "v 500.0\xC2\xA0"
                                 "cm/s",
                 "Applied 1.00 \xC3\x97 10\xE2\x81\xB6\xC2\xA0"
                 "dyn",
                 "a 500\xC2\xA0"
                 "cm/s\xC2\xB2",
                 "p 1.00 \xC3\x97 10\xE2\x81\xB6\xC2\xA0g\xC2\xB7"
                 "cm/s" })
            RIGIDBODIES_EXPECT(list.text_buffer().find(text) != std::string::npos, "the alternative unit system converts every vector dimension");
        RIGIDBODIES_EXPECT(world.find_body(id)->linear_velocity_m_s() == Vec2(3.0, 4.0), "rendering and display conversion do not write simulation state");
    }

    RIGIDBODIES_TEST("signed world and custom components reconstruct the original velocity")
    {
        World world { zero_gravity() };
        const auto id = vector_body(world, { -3.0, 4.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.vector_scales.velocity = 10.0;
        settings.vector_components = VectorComponents::world_axes;
        SceneRenderer renderer;
        DrawList list;
        const auto camera = make_camera();
        const auto start = camera.world_to_screen(world.find_body(id)->world_center_of_mass_m());
        for (const auto angle : { 0.0, half_pi, 0.37 })
        {
            settings.vector_components = angle == 0.0 ? VectorComponents::world_axes : VectorComponents::custom_axes;
            settings.component_angle_rad = angle;
            renderer.render(world, camera, settings, list);
            Vec2 sum;
            const auto components = arrows(list, settings.theme.velocity.with_alpha(0.65f));
            for (const auto& component : components)
            {
                RIGIDBODIES_EXPECT_NEAR(component.start.x, start.x, 1.0e-9, "component starts at the mass centre");
                RIGIDBODIES_EXPECT_NEAR(component.start.y, start.y, 1.0e-9, "component starts at the mass centre");
                sum += component.tip - start;
            }
            RIGIDBODIES_EXPECT(components.size() == 2, "the two orthogonal signed components are visible");
            RIGIDBODIES_EXPECT_NEAR(sum.x, -30.0, 1.0e-9, "horizontal screen components reconstruct the original vector");
            RIGIDBODIES_EXPECT_NEAR(sum.y, -40.0, 1.0e-9, "vertical screen components reconstruct the original vector");
            if (angle == 0.0)
            {
                RIGIDBODIES_EXPECT(list.text_buffer().find("v.x \xE2\x88\x92"
                                                           "3.00\xC2\xA0m/s") != std::string::npos,
                    "negative x is explicitly signed");
                RIGIDBODIES_EXPECT(list.text_buffer().find("v.y +4.00\xC2\xA0m/s") != std::string::npos, "positive y is explicitly signed");
            }
            if (angle == half_pi)
            {
                RIGIDBODIES_EXPECT(list.text_buffer().find("v.u +4.00\xC2\xA0m/s") != std::string::npos, "chosen-axis projection uses the specified angle");
                RIGIDBODIES_EXPECT(list.text_buffer().find("v.v +3.00\xC2\xA0m/s") != std::string::npos, "perpendicular projection retains its signed value");
            }
        }
    }

    RIGIDBODIES_TEST("contact components follow a real supporting normal and have an explicit missing-contact state")
    {
        World world { zero_gravity() };
        BodyDefinition floor;
        floor.type = BodyType::static_body;
        floor.position_m = { 0.0, -0.5 };
        Collider collider;
        collider.shape = make_box(10.0, 1.0);
        floor.colliders.push_back(collider);
        world.create_body(floor);
        const auto id = vector_body(world, { 3.0, -2.0 }, { 0.0, 0.24 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(!world.manifolds().empty(), "fixture has a real floor contact");
        world.find_body(id)->set_linear_velocity({ 3.0, -2.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.vector_components = VectorComponents::contact_axes;
        settings.selection = id;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("v.n \xE2\x88\x92"
                                                   "2.00\xC2\xA0m/s") != std::string::npos,
            "normal projection points toward the supported body");
        RIGIDBODIES_EXPECT(list.text_buffer().find("v.t \xE2\x88\x92"
                                                   "3.00\xC2\xA0m/s") != std::string::npos,
            "tangent is the normal's counter-clockwise perpendicular");
        World empty_contact { zero_gravity() };
        settings.selection = vector_body(empty_contact);
        renderer.render(empty_contact, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("Contact axes: no solid contact") != std::string::npos, "the renderer never invents a contact frame for a free body");
        RIGIDBODIES_EXPECT(list.text_buffer().find("v.n") == std::string::npos, "unavailable contact components are omitted");
    }

    RIGIDBODIES_TEST("automatic scales preserve relative magnitudes and independently fit each quantity")
    {
        World world { zero_gravity() };
        const auto first = vector_body(world, { 0.0, 0.0 }, { -1.0, 0.0 });
        const auto second = vector_body(world, { 0.0, 0.0 }, { 1.0, 0.0 });
        world.find_body(first)->override_mass(10.0);
        world.find_body(first)->apply_force_at_center({ 5.0, 0.0 });
        world.find_body(second)->apply_force_at_center({ 20.0, 0.0 });
        world.step(0.01);
        world.find_body(first)->set_linear_velocity({ 3.0, 0.0 });
        world.find_body(second)->set_linear_velocity({ 6.0, 0.0 });
        auto settings = vectors_only();
        settings.vector_scales.automatic = true;
        settings.vector_scales.minimum_drawn_length_px = 0.0f;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        for (const auto color : { settings.theme.velocity, settings.theme.acceleration, settings.theme.force, settings.theme.momentum })
        {
            std::vector<double> lengths;
            for (const auto& drawn : arrows(list, color))
                lengths.push_back(length(drawn.tip - drawn.start));
            RIGIDBODIES_EXPECT(lengths.size() == 2, "each body's nonzero quantity has an arrow");
            RIGIDBODIES_EXPECT_NEAR(std::max(lengths[0], lengths[1]), 120.0, 1.0e-9, "each quantity has its own automatic maximum");
            if (same_color(color, settings.theme.velocity))
                RIGIDBODIES_EXPECT_NEAR(lengths[0] / lengths[1], 0.5, 1.0e-12, "autoscale preserves comparisons between bodies");
            if (same_color(color, settings.theme.force))
                RIGIDBODIES_EXPECT_NEAR(lengths[0] / lengths[1], 0.25, 1.0e-12, "force scale uses force maximum rather than velocity maximum");
            if (same_color(color, settings.theme.momentum))
                RIGIDBODIES_EXPECT_NEAR(lengths[1] / lengths[0], 0.4, 1.0e-12, "the heavier slower body remains the larger momentum arrow");
        }
        RIGIDBODIES_EXPECT_NEAR(settings.vector_scales.velocity, 24.0, 0.0, "automatic mode leaves the manual settings available for restoration");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->linear_velocity_m_s().x, 3.0, 0.0, "autoscaling is read-only");
    }

    RIGIDBODIES_TEST("clipped and invalid vector scales never hide a changed numeric magnitude or emit nonfinite vertices")
    {
        World world { zero_gravity() };
        // Stay below the world's speed limit; this test isolates display clipping.
        const auto id = vector_body(world, { 30.0, 40.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        const auto arrow = arrows(list, settings.theme.velocity).front();
        RIGIDBODIES_EXPECT_NEAR(length(arrow.tip - arrow.start), 400.0, 1.0e-9, "long arrow is bounded in screen space");
        RIGIDBODIES_EXPECT(list.text_buffer().find("v 50.00\xC2\xA0m/s") != std::string::npos, "the true number survives the cap");
        RIGIDBODIES_EXPECT(list.text_buffer().find("not to scale") != std::string::npos, "an uncrowded label discloses the cap in words");
        for (const auto invalid : { 0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
        {
            settings.vector_scales.velocity = invalid;
            renderer.render(world, make_camera(), settings, list);
            RIGIDBODIES_EXPECT(list.is_empty(), "invalid manual scales produce no invalid geometry");
        }
        world.find_body(id)->set_linear_velocity({});
        settings.vector_scales.automatic = true;
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "a stationary scene does not divide by zero or invent arrow directions");
        world.find_body(id)->set_linear_velocity({ 1.0, 0.0 });
        settings.vector_scales.automatic_target_length_px = std::numeric_limits<float>::quiet_NaN();
        settings.vector_scales.maximum_drawn_length_px = std::numeric_limits<float>::infinity();
        settings.component_angle_rad = std::numeric_limits<double>::quiet_NaN();
        settings.vector_components = VectorComponents::custom_axes;
        renderer.render(world, make_camera(), settings, list);
        for (const auto& vertex : list.vertices())
            RIGIDBODIES_EXPECT(is_finite(vertex), "invalid optional presentation settings have finite fallbacks");
        RIGIDBODIES_EXPECT(list.text_buffer().find("v.v 0.00\xC2\xA0m/s") != std::string::npos, "zero projections remain readable without a false arrow");
    }

    // A floor, gravity and bodies dropped onto it, stepped the way the application steps them
    // so the renderer knows the substep its contact impulses were solved over.
    struct RestingScene
    {
        World world;
        SceneRenderer renderer;
        std::vector<BodyId> bodies;
    };

    void add_floor(World& world)
    {
        BodyDefinition floor;
        floor.type = BodyType::static_body;
        floor.position_m = { 0.0, -0.5 };
        Collider collider;
        collider.shape = make_box(12.0, 1.0);
        floor.colliders.push_back(collider);
        world.create_body(floor);
    }

    void settle(World& world, SceneRenderer& renderer, const SceneRenderSettings& settings, int steps)
    {
        for (int step = 0; step < steps; ++step)
        {
            world.step(1.0 / 120.0);
            renderer.record_visual_sample(world, 1.0 / 120.0, settings);
        }
    }

    SceneRenderSettings forces_only()
    {
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::force_vectors, true);
        settings.layers.set(VisualizationLayer::labels, true);
        return settings;
    }

    RIGIDBODIES_TEST("a resting body's weight is balanced by the contact force solved for it")
    {
        World world;
        world.add_force_generator(std::make_shared<UniformGravity>());
        add_floor(world);
        BodyDefinition definition;
        definition.position_m = { 0.0, 0.25 };
        Collider collider;
        collider.shape = make_box(0.5, 0.5);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        world.step(1.0 / 120.0);
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.contact).empty(), "without a known substep the renderer never invents a contact force");
        settle(world, renderer, settings, 240);
        renderer.render(world, make_camera(), settings, list);
        const auto weight = arrows(list, settings.theme.force);
        const auto contact = arrows(list, settings.theme.contact);
        RIGIDBODIES_EXPECT(weight.size() == 1 && contact.size() == 1, "weight and support are each drawn once");
        RIGIDBODIES_EXPECT(weight.front().tip.y > weight.front().start.y && contact.front().tip.y < contact.front().start.y, "weight points down and the support points up");
        const auto weight_length = length(weight.front().tip - weight.front().start);
        const auto contact_length = length(contact.front().tip - contact.front().start);
        RIGIDBODIES_EXPECT(std::abs(weight_length - contact_length) <= 0.05 * weight_length, "a resting body's free-body picture balances");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Weight") != std::string::npos && list.text_buffer().find("Contact") != std::string::npos, "each arrow is named for what it holds");
        RIGIDBODIES_EXPECT(list.text_buffer().find("F ") == std::string::npos, "a lone weight is not presented as a net force");
        RIGIDBODIES_EXPECT(world.find_body(id)->linear_velocity_m_s().y > -0.1, "the fixture is at rest");
    }

    // A camera on the moving bodies of a demonstration, with room around them for arrows that
    // reach well beyond them.
    Camera2D framed_camera(const World& world)
    {
        auto camera = make_camera();
        Aabb bounds;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                if (body.type() != BodyType::dynamic_body)
                    return;
                const auto box = body.compute_bounds();
                bounds.expand(box.minimum);
                bounds.expand(box.maximum);
            });
        camera.set_center((bounds.minimum + bounds.maximum) * 0.5);
        camera.set_view_height(std::max(2.0 * std::max(bounds.extents().y, bounds.extents().x * 0.8), 1.0));
        return camera;
    }

    BodyId body_named(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == name)
                return id;
        RIGIDBODIES_FAIL("the named body exists");
    }

    RIGIDBODIES_TEST("a block resting on an incline shows a support that exactly balances its weight")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "ramp"), "the incline demonstration exists");
        const auto block = body_named(world, "block");
        auto settings = forces_only();
        settings.selection = block;
        SceneRenderer renderer;
        DrawList list;
        const auto camera = framed_camera(world);
        bool slept = false, checked_awake = false;
        // Two substeps per frame, as the application steps it, over ten seconds.
        for (int step = 1; step <= 1200; ++step)
        {
            world.step(1.0 / 120.0);
            renderer.record_visual_sample(world, 1.0 / 120.0, settings);
            if (step < 12 || step % 2 == 1)
                continue;
            renderer.render(world, camera, settings, list);
            const auto weight = arrows(list, settings.theme.force);
            const auto contact = arrows(list, settings.theme.contact);
            RIGIDBODIES_EXPECT(weight.size() == 1 && contact.size() == 1, "the block shows its weight and its support");
            const auto drawn_weight = weight.front().tip - weight.front().start;
            const auto drawn_contact = contact.front().tip - contact.front().start;
            RIGIDBODIES_EXPECT(length(drawn_weight) > 40.0, "the weight is drawn to scale, not as a stub");
            RIGIDBODIES_EXPECT(length(drawn_weight + drawn_contact) <= 0.01 * length(drawn_weight), "the support is equal and opposite to the weight of a block at rest");
            const auto awake = world.find_body(block)->is_awake();
            checked_awake = checked_awake || awake;
            slept = slept || !awake;
        }
        RIGIDBODIES_EXPECT(checked_awake && slept, "the balance holds while the block settles and after it sleeps");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Contact 21.2\xC2\xA0N") != std::string::npos && list.text_buffer().find("Weight 21.2\xC2\xA0N") != std::string::npos, "the two plates read the same value");
    }

    RIGIDBODIES_TEST("every box of a resting stack balances, and a singled-out box shows each neighbour's push")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "stable_stack"), "the stack demonstration exists");
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        const auto camera = framed_camera(world);
        for (const auto steps : { 60, 300 })
        {
            for (int step = 0; step < steps; ++step)
            {
                world.step(1.0 / 120.0);
                renderer.record_visual_sample(world, 1.0 / 120.0, settings);
            }
            settings.selection = {};
            renderer.render(world, camera, settings, list);
            const auto weights = arrows(list, settings.theme.force);
            const auto contacts = arrows(list, settings.theme.contact);
            RIGIDBODIES_EXPECT(weights.size() == 6 && contacts.size() == 6, "each box shows its weight and its net support");
            for (const auto& weight : weights)
            {
                const auto nearest = std::min_element(contacts.begin(), contacts.end(), [&](const DrawnArrow& a, const DrawnArrow& b)
                    {
                        return length(a.start - weight.start) < length(b.start - weight.start);
                    });
                const auto drawn_weight = weight.tip - weight.start;
                RIGIDBODIES_EXPECT(length(drawn_weight) > 16.0, "each weight is drawn to scale, not as a stub");
                RIGIDBODIES_EXPECT(length(drawn_weight + (nearest->tip - nearest->start)) <= 0.01 * length(drawn_weight), "each box's net support balances its weight");
            }
        }
        RIGIDBODIES_EXPECT(!world.find_body(body_named(world, "stack_box_1"))->is_awake(), "the stack has fallen asleep");
        // Every arrow stays on its own box: from the centre of mass, inside that box, never
        // running into a neighbour.
        const auto screen_box = [&](std::string_view name)
        {
            const auto bounds = world.find_body(body_named(world, name))->compute_bounds();
            const auto a = camera.world_to_screen(bounds.minimum), b = camera.world_to_screen(bounds.maximum);
            return std::pair { min_components(a, b), max_components(a, b) };
        };
        const auto inside = [](const std::pair<Vec2, Vec2>& box, const Vec2& point)
        {
            return point.x >= box.first.x - 1.0e-6 && point.x <= box.second.x + 1.0e-6 && point.y >= box.first.y - 1.0e-6 && point.y <= box.second.y + 1.0e-6;
        };
        {
            const auto weights = arrows(list, settings.theme.force);
            const auto contacts = arrows(list, settings.theme.contact);
            for (const auto& arrow : weights)
            {
                bool own = false;
                for (const auto* name : { "stack_box_1", "stack_box_2", "stack_box_3", "stack_box_4", "stack_box_5", "stack_box_6" })
                    own = own || (inside(screen_box(name), arrow.start) && inside(screen_box(name), arrow.tip));
                RIGIDBODIES_EXPECT(own, "a box's weight starts and ends inside that box");
            }
            RIGIDBODIES_EXPECT(contacts.size() == 6, "every box shows one support arrow");
        }
        // The bottom box, singled out, shows the floor's push and the box above's push apart.
        // Each push rests its head on the face it presses and lies in the body exerting it; the
        // push of the box above stops short of the box above that one, and says it is shortened.
        settings.selection = body_named(world, "stack_box_1");
        renderer.render(world, camera, settings, list);
        const auto pushes = arrows(list, settings.theme.contact);
        const auto weight = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(pushes.size() == 2 && weight.size() == 1, "two neighbours press on the bottom box");
        const auto bottom = screen_box("stack_box_1"), above = screen_box("stack_box_2");
        for (const auto& push : pushes)
        {
            RIGIDBODIES_EXPECT(std::abs(push.tip.y - bottom.first.y) <= 3.0 || std::abs(push.tip.y - bottom.second.y) <= 3.0, "a push ends on a face of the box it presses");
            if (push.tip.y > push.start.y)
                RIGIDBODIES_EXPECT(inside(above, push.start), "the box above pushes down from inside itself, never from the box beyond it");
            else
                RIGIDBODIES_EXPECT(push.start.y > bottom.second.y, "the floor pushes up from below the box");
        }
        RIGIDBODIES_EXPECT(list.text_buffer().find("Push from Stack floor") != std::string::npos && list.text_buffer().find("Push from Stack box 2") != std::string::npos, "each push is named as a push from the neighbour exerting it");
        // The plates balance: the floor's push less the box above's is the box's weight.
        const auto value_after = [&](std::string_view prefix)
        {
            const auto at = list.text_buffer().find(prefix);
            return at == std::string::npos ? 0.0 : std::stod(list.text_buffer().substr(at + prefix.size(), 8));
        };
        const auto floor_push = value_after("Push from Stack floor "), box_push = value_after("Push from Stack box 2 "), box_weight = value_after("Weight ");
        RIGIDBODIES_EXPECT(box_push > 4.0 * box_weight, "the boxes above press on the bottom box with several times its weight");
        RIGIDBODIES_EXPECT(std::abs(floor_push - box_push - box_weight) <= 0.01 * floor_push, "the pushes on the bottom box and its weight balance");
    }

    RIGIDBODIES_TEST("each load on a body has its own named arrow, and none reads as zero")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "aerodynamic_profiles"), "the falling plates demonstration exists");
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        for (int step = 0; step < 90; ++step)
        {
            world.step(1.0 / 120.0);
            renderer.record_visual_sample(world, 1.0 / 120.0, settings);
        }
        renderer.render(world, framed_camera(world), settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("Air drag") != std::string::npos && list.text_buffer().find("Weight") != std::string::npos, "the air's force and gravity are named apart");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Applied") == std::string::npos, "no arrow sums gravity with another source");
        RIGIDBODIES_EXPECT(list.text_buffer().find(" 0\xC2\xA0") == std::string::npos, "no arrow is drawn for a value that reads as zero");
        bool hollow = false;
        for (const auto& command : list.commands())
            hollow = hollow || (command.kind == DrawCommandKind::polygon_outline && same_color(command.color, settings.theme.force));
        RIGIDBODIES_EXPECT(hollow, "a load sharing the weight's hue has a differently shaped head");

        // A small load on a large body is drawn beyond its outline, where its direction can be seen.
        World quiet { zero_gravity() };
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.6);
        definition.colliders.push_back(collider);
        const auto id = quiet.create_body(definition);
        quiet.find_body(id)->apply_force_at_center({ 3.0, 0.0 });
        quiet.step(0.01);
        SceneRenderer small;
        small.render(quiet, make_camera(), settings, list);
        const auto applied = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(applied.size() == 1, "the load is drawn");
        const auto center = make_camera().world_to_screen(quiet.find_body(id)->world_center_of_mass_m());
        RIGIDBODIES_EXPECT(length(applied.front().tip - center) > make_camera().world_to_screen_length(0.6) + 4.0, "the arrowhead lies outside the body");
    }

    RIGIDBODIES_TEST("a spring's plate and the arrow it puts on its body read the same force at the same instant")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "spring_damping"), "the spring demonstration exists");
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::constraints, true);
        SceneRenderer renderer;
        DrawList list;
        for (int step = 0; step < 150; ++step)
        {
            world.step(1.0 / 120.0);
            renderer.record_visual_sample(world, 1.0 / 120.0, settings);
        }
        std::vector<std::string> arrow_values, plate_values, stretch_only;
        const auto read = [&]
        {
            arrow_values.clear();
            plate_values.clear();
            stretch_only.clear();
            for (const auto& command : list.commands())
            {
                if (command.kind != DrawCommandKind::text)
                    continue;
                const auto text = std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length);
                if (text.substr(0, 7) == "Spring ")
                    arrow_values.emplace_back(text.substr(7));
                bool force = false;
                for (const std::string_view word : { "push ", "pull " })
                    if (const auto at = text.find(word); at != std::string_view::npos)
                    {
                        plate_values.emplace_back(text.substr(at + word.size()));
                        force = true;
                    }
                if (!force && (text.substr(0, 10) == "extension " || text.substr(0, 12) == "compression "))
                    stretch_only.emplace_back(text);
            }
        };
        // Pointing at a mass guarantees its spring's plate, which reads the same force as the
        // spring's arrow on the mass.
        settings.hover = body_named(world, "elastic_mass");
        renderer.render(world, framed_camera(world), settings, list);
        read();
        RIGIDBODIES_EXPECT(!arrow_values.empty() && !plate_values.empty(), "the hovered mass's spring plate and spring arrow are shown");
        for (const auto& value : arrow_values)
            RIGIDBODIES_EXPECT(std::find(plate_values.begin(), plate_values.end(), value) != plate_values.end(), "the arrow repeats its spring's plate exactly");
        // Singling the mass out labels its spring arrow, so its spring's plate keeps only the
        // stretch rather than stating the same force twice.
        settings.hover = {};
        settings.selection = body_named(world, "elastic_mass");
        renderer.render(world, framed_camera(world), settings, list);
        read();
        RIGIDBODIES_EXPECT(!arrow_values.empty(), "the selection's spring arrow states the force");
        RIGIDBODIES_EXPECT(stretch_only.size() == 1, "the selection's spring plate keeps only the stretch");
        RIGIDBODIES_EXPECT(list.text_buffer().find("extension \xE2\x88\x92") == std::string::npos, "a squeezed spring is called compressed, never negatively extended");
    }

    RIGIDBODIES_TEST("every body of a multiple selection keeps its vectors in focus")
    {
        World world { zero_gravity() };
        const auto first = vector_body(world, { 2.0, 0.0 }, { -2.0, 0.0 });
        const auto second = vector_body(world, { 2.0, 0.0 }, { 0.0, 0.0 });
        vector_body(world, { 2.0, 0.0 }, { 2.0, 0.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.selection = second;
        SceneRenderer renderer;
        renderer.set_selected_bodies({ first, second });
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.velocity).size() == 2, "both selected bodies' arrows are at full strength");
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.velocity.with_alpha(settings.theme.velocity.alpha * 0.4f)).size() == 1, "only the unselected body recedes");
        std::size_t rings = 0;
        for (const auto& command : list.commands())
            rings += command.kind == DrawCommandKind::circle_outline && same_color(command.color, settings.theme.selection) ? 1u : 0u;
        RIGIDBODIES_EXPECT(rings == 2, "every selected body has the same selection ring");
        renderer.set_selected_bodies({});
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.velocity).size() == 1, "a set no longer naming the selection is ignored");
    }

    RIGIDBODIES_TEST("a receding reading stays legible on its plate in every theme")
    {
        const auto luminance = [](const Color& color)
        {
            const auto linear = [](float value)
            {
                return value <= 0.03928f ? value / 12.92f : std::pow((value + 0.055f) / 1.055f, 2.4f);
            };
            return 0.2126f * linear(color.red) + 0.7152f * linear(color.green) + 0.0722f * linear(color.blue);
        };
        World world { zero_gravity() };
        const auto first = vector_body(world, { 2.0, 0.0 }, { -2.0, 0.0 });
        vector_body(world, { 1.5, 0.0 }, { 2.0, 0.0 });
        for (const auto* name : { "workbench_dark", "workbench_light", "workbench_projector" })
        {
            auto settings = vectors_only();
            settings.layers.set(VisualizationLayer::labels, true);
            settings.theme = theme_by_name(name);
            settings.selection = first;
            SceneRenderer renderer;
            DrawList list;
            renderer.render(world, make_camera(), settings, list);
            const auto& theme = settings.theme;
            const auto plate = mix(theme.background, theme.label_plate.with_alpha(1.0f), theme.label_plate.alpha);
            std::size_t readings = 0;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::text && std::string_view { list.text_buffer() }.substr(command.text_offset, 2) == "v ")
                {
                    ++readings;
                    const auto text = mix(plate, command.color.with_alpha(1.0f), command.color.alpha);
                    const auto lighter = std::max(luminance(text), luminance(plate)), darker = std::min(luminance(text), luminance(plate));
                    RIGIDBODIES_EXPECT((lighter + 0.05f) / (darker + 0.05f) >= 4.5f, "every velocity reading keeps at least 4.5:1 against its plate");
                }
            RIGIDBODIES_EXPECT(readings == 2, "the selected and the receding readings are both shown");
        }
    }

    RIGIDBODIES_TEST("the projector's arrow colours stay far apart, the balancing pair furthest")
    {
        // CIEDE2000 between two sRGB colours.
        const auto lab = [](const Color& color)
        {
            const auto linear = [](double value)
            {
                return value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4);
            };
            const auto r = linear(color.red), g = linear(color.green), b = linear(color.blue);
            const auto f = [](double t)
            {
                return t > 0.008856 ? std::cbrt(t) : 7.787 * t + 16.0 / 116.0;
            };
            const auto x = f((0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.95047), y = f(0.2126 * r + 0.7152 * g + 0.0722 * b), z = f((0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.08883);
            return std::array<double, 3> { 116.0 * y - 16.0, 500.0 * (x - y), 200.0 * (y - z) };
        };
        const auto difference = [&](const Color& first, const Color& second)
        {
            const auto [l1, a1, b1] = lab(first);
            const auto [l2, a2, b2] = lab(second);
            const auto degrees = 180.0 / pi;
            const auto c_bar = (std::hypot(a1, b1) + std::hypot(a2, b2)) * 0.5;
            const auto g = 0.5 * (1.0 - std::sqrt(std::pow(c_bar, 7.0) / (std::pow(c_bar, 7.0) + std::pow(25.0, 7.0))));
            const auto a1p = (1.0 + g) * a1, a2p = (1.0 + g) * a2;
            const auto c1 = std::hypot(a1p, b1), c2 = std::hypot(a2p, b2);
            auto h1 = std::fmod(std::atan2(b1, a1p) * degrees + 360.0, 360.0), h2 = std::fmod(std::atan2(b2, a2p) * degrees + 360.0, 360.0);
            auto dh = h2 - h1;
            if (dh > 180.0)
                dh -= 360.0;
            else if (dh < -180.0)
                dh += 360.0;
            const auto big_dh = 2.0 * std::sqrt(c1 * c2) * std::sin(dh / degrees * 0.5);
            const auto l_bar = (l1 + l2) * 0.5, cp_bar = (c1 + c2) * 0.5;
            auto h_bar = std::abs(h1 - h2) <= 180.0 ? (h1 + h2) * 0.5 : (h1 + h2 < 360.0 ? (h1 + h2 + 360.0) * 0.5 : (h1 + h2 - 360.0) * 0.5);
            const auto t = 1.0 - 0.17 * std::cos((h_bar - 30.0) / degrees) + 0.24 * std::cos(2.0 * h_bar / degrees) + 0.32 * std::cos((3.0 * h_bar + 6.0) / degrees) - 0.20 * std::cos((4.0 * h_bar - 63.0) / degrees);
            const auto theta = 30.0 * std::exp(-std::pow((h_bar - 275.0) / 25.0, 2.0));
            const auto rc = 2.0 * std::sqrt(std::pow(cp_bar, 7.0) / (std::pow(cp_bar, 7.0) + std::pow(25.0, 7.0)));
            const auto sl = 1.0 + 0.015 * std::pow(l_bar - 50.0, 2.0) / std::sqrt(20.0 + std::pow(l_bar - 50.0, 2.0));
            const auto sc = 1.0 + 0.045 * cp_bar, sh = 1.0 + 0.015 * cp_bar * t;
            const auto rt = -std::sin(2.0 * theta / degrees) * rc;
            const auto dl = (l2 - l1) / sl, dc = (c2 - c1) / sc, dhh = big_dh / sh;
            return std::sqrt(dl * dl + dc * dc + dhh * dhh + rt * dc * dhh);
        };
        RIGIDBODIES_EXPECT_NEAR(difference(Color::from_bytes(140, 74, 0), Color::from_bytes(186, 44, 12)), 15.7, 0.1, "the colour difference matches its published measure");
        const auto projector = theme_by_name("workbench_projector");
        const std::array<Color, 5> arrows_in_use { projector.force, projector.contact, projector.velocity, projector.momentum, projector.acceleration };
        for (std::size_t first = 0; first < arrows_in_use.size(); ++first)
            for (std::size_t second = first + 1; second < arrows_in_use.size(); ++second)
                RIGIDBODIES_EXPECT(difference(arrows_in_use[first], arrows_in_use[second]) >= 25.0, "every pair of projector arrow colours is clearly distinct");
        for (const auto* name : { "workbench_dark", "workbench_light", "workbench_projector" })
        {
            const auto theme = theme_by_name(name);
            RIGIDBODIES_EXPECT(difference(theme.force, theme.contact) >= 25.0, "weight and support never look alike");
        }
    }

    RIGIDBODIES_TEST("a selected concave part's glow follows its outline instead of filling its notch")
    {
        World world { zero_gravity() };
        Outline outline;
        outline.closed = true;
        for (const auto point : { Vec2 { 0.0, 0.0 }, Vec2 { 2.0, 0.0 }, Vec2 { 2.0, 0.5 }, Vec2 { 0.5, 0.5 }, Vec2 { 0.5, 2.0 }, Vec2 { 0.0, 2.0 } })
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        AuthoredPartDefinition part;
        part.shape = build_authored_shape(outline).shape;
        const auto bracket = create_authored_body(world, {}, { part });
        SceneRenderSettings settings;
        settings.material_shading = false;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.layers.set(VisualizationLayer::outlines, true);
        settings.selection = bracket;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        bool glow = false;
        for (const auto& command : list.commands())
        {
            RIGIDBODIES_EXPECT(command.kind != DrawCommandKind::shadow, "no soft fill is fanned across the notch");
            glow = glow || (command.kind == DrawCommandKind::polyline && command.layer == -4);
        }
        RIGIDBODIES_EXPECT(glow, "the glow is a band along the outline itself");
    }

    RIGIDBODIES_TEST("a slider's plate reports the couple at its carriage, which does not grow with travel")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "prismatic_drive"), "the slider demonstration exists");
        auto settings = connections_only();
        settings.layers.set(VisualizationLayer::labels, true);
        settings.selection = body_named(world, "driven_carriage");
        SceneRenderer renderer;
        DrawList list;
        for (int step = 1; step <= 200; ++step)
        {
            world.step(1.0 / 120.0);
            if (step % 40 != 0)
                continue;
            renderer.render(world, framed_camera(world), settings, list);
            RIGIDBODIES_EXPECT(list.text_buffer().find("support 3.92\xC2\xA0N") != std::string::npos, "the rail's support of the carriage is named as such");
            RIGIDBODIES_EXPECT(list.text_buffer().find("torque") == std::string::npos, "a carriage riding level carries no couple, wherever it is on the rail");
        }
    }

    RIGIDBODIES_TEST("weights spanning a scene stay comparable and a tiny one keeps a marked stub")
    {
        World world;
        world.add_force_generator(std::make_shared<UniformGravity>());
        std::vector<BodyId> balls;
        for (const auto scale : { 0.125, 1.0, 2.0 })
        {
            BodyDefinition definition;
            definition.position_m = { -2.0 + 2.0 * static_cast<double>(balls.size()), 2.0 };
            definition.gravity_scale = scale;
            Collider collider;
            collider.shape = make_circle(0.1);
            definition.colliders.push_back(collider);
            balls.push_back(world.create_body(definition));
            world.find_body(balls.back())->override_mass(0.1);
        }
        world.step(1.0 / 120.0);
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        auto drawn = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(drawn.size() == 3, "no weight vanishes for being small");
        std::sort(drawn.begin(), drawn.end(), [](const DrawnArrow& a, const DrawnArrow& b)
            {
                return a.start.x < b.start.x;
            });
        const auto smallest = length(drawn[0].tip - drawn[0].start), normal = length(drawn[1].tip - drawn[1].start), largest = length(drawn[2].tip - drawn[2].start);
        RIGIDBODIES_EXPECT_NEAR(largest / normal, 2.0, 1.0e-9, "comparable weights keep their true ratio");
        // The default 6 px/N would draw 1.96 N as 12 px; the scale fits the stage instead.
        const auto longest = std::clamp(0.16 * 800.0, 48.0, 140.0);
        RIGIDBODIES_EXPECT(largest >= 0.25 * longest - 1.0e-9 && largest <= longest + 1.0e-9, "the heaviest weight is drawn long enough to read and short enough to stay on stage");
        RIGIDBODIES_EXPECT(smallest >= 14.0 - 1.0e-9 && smallest < normal, "the lightest keeps a stub shorter than the next");
        RIGIDBODIES_EXPECT(list.text_buffer().find("not to scale") != std::string::npos, "the stub says it is not to scale");
        // The key beside the scale bar states the force scale the arrows were drawn at, as a
        // round value without trailing zeros, in the unit of the scene's plates.
        settings.layers.set(VisualizationLayer::grid, true);
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(has_text(list, "1\xC2\xA0N") || has_text(list, "2\xC2\xA0N") || has_text(list, "0.5\xC2\xA0N"), "a force key accompanies the scale bar");
    }

    RIGIDBODIES_TEST("the force key holds still from the first frame and shares the plates' unit")
    {
        World world;
        BodyDefinition definition;
        definition.position_m = { 0.0, 1.0 };
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(0.008);
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::grid, true);
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        const auto before = renderer.drawn_force_scale();
        world.step(1.0 / 120.0);
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT_NEAR(renderer.drawn_force_scale(), before, 1.0e-9, "the weight sets the scale before the first step has applied it");
        RIGIDBODIES_EXPECT(list.text_buffer().find("Weight 78.5\xC2\xA0mN") != std::string::npos, "a scene of small forces reads in millinewtons");
        bool key_in_milli = false;
        for (const auto text : { "20\xC2\xA0mN", "50\xC2\xA0mN", "100\xC2\xA0mN", "10\xC2\xA0mN" })
            key_in_milli = key_in_milli || has_text(list, text);
        RIGIDBODIES_EXPECT(key_in_milli, "the key is a round value in the same unit");

        World weightless { zero_gravity() };
        vector_body(weightless, { 1.0, 0.0 });
        SceneRenderer fresh;
        fresh.render(weightless, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(list.text_buffer().find("N") == std::string::npos, "no key is shown before any force has set the scale");
    }

    RIGIDBODIES_TEST("arrows end on the visible stage and plates stay inside it")
    {
        World world { zero_gravity() };
        const auto id = vector_body(world, { 0.0, -12.0 }, { 0.0, -2.5 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        auto camera = make_camera();
        camera.set_focus_rect({ 100.0, 60.0, 800.0, 660.0 });
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto drawn = arrows(list, settings.theme.velocity);
        RIGIDBODIES_EXPECT(drawn.size() == 1, "the velocity is drawn");
        RIGIDBODIES_EXPECT(drawn.front().tip.y <= 720.0, "the arrowhead stays above the stage's lower edge");
        RIGIDBODIES_EXPECT(list.text_buffer().find("not to scale") != std::string::npos, "the shortened arrow says so");
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::rounded_rectangle_fill)
            {
                const auto minimum = list.vertices().at(command.vertex_offset), maximum = list.vertices().at(command.vertex_offset + 1);
                RIGIDBODIES_EXPECT(minimum.x >= 100.0 - 1.0e-9 && minimum.y >= 60.0 - 1.0e-9 && maximum.x <= 900.0 + 1.0e-9 && maximum.y <= 720.0 + 1.0e-9, "value plates stay within the visible stage");
            }
        RIGIDBODIES_EXPECT(world.find_body(id)->linear_velocity_m_s().y == -12.0, "rendering is read-only");
    }

    RIGIDBODIES_TEST("selecting a body brings its arrows forward and the rest recede")
    {
        World world { zero_gravity() };
        const auto first = vector_body(world, { 2.0, 0.0 }, { -2.0, 0.0 });
        vector_body(world, { 2.0, 0.0 }, { 2.0, 0.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.selection = first;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.velocity).size() == 1, "the selection's arrow is at full strength");
        RIGIDBODIES_EXPECT(arrows(list, settings.theme.velocity.with_alpha(settings.theme.velocity.alpha * 0.4f)).size() == 1, "another body's arrow is dimmed");
    }

    RIGIDBODIES_TEST("the centre-of-mass symbol keeps one size while a thin body turns")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_box(1.6, 0.16);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        SceneRenderSettings settings;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.layers.set(VisualizationLayer::center_of_mass, true);
        SceneRenderer renderer;
        DrawList list;
        std::vector<float> radii;
        for (const auto angle : { 0.0, 0.4, 0.785, 1.2, half_pi })
        {
            world.find_body(id)->set_simulated_pose({}, angle);
            world.find_body(id)->capture_previous_transform();
            renderer.render(world, make_camera(), settings, list);
            float radius = 0.0f;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::circle_fill && same_color(command.color, settings.theme.center_of_mass))
                    radius = command.radius;
            radii.push_back(radius);
        }
        RIGIDBODIES_EXPECT(radii.front() > 0.0f, "a thin body still shows its centre");
        for (const auto radius : radii)
            RIGIDBODIES_EXPECT(radius == radii.front(), "the symbol neither pops nor resizes as the body turns");
    }

    RIGIDBODIES_TEST("a shelled part shows its bore inside the outer envelope")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.5);
        collider.shell_thickness_m = 0.1;
        definition.colliders.push_back(collider);
        world.create_body(definition);
        SceneRenderSettings settings;
        settings.material_shading = false;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        SceneRenderer renderer;
        DrawList list;
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        std::vector<float> fills;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::circle_fill)
                fills.push_back(command.radius);
        RIGIDBODIES_EXPECT(fills.size() == 2, "the tube draws its wall and its bore");
        RIGIDBODIES_EXPECT_NEAR(fills.back(), camera.world_to_screen_length(0.4), 1.0e-4, "the bore is the outer radius less the wall");
    }

    // Whether a shaft drawn in this colour is dotted, as a stub longer than its value is.
    bool dotted_shaft(const DrawList& list, const Color& color)
    {
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::line && same_color(command.color, color) && command.dash_length > 0.0f)
                return true;
        return false;
    }

    RIGIDBODIES_TEST("a value too short to draw to scale is a dotted stub whose plate says so however crowded")
    {
        World world { zero_gravity() };
        for (int index = 0; index < 8; ++index)
            vector_body(world, { 0.8, 0.0 }, { -3.0 + index * 0.9, 0.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.vector_scales.velocity = 10.0;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        const auto drawn = arrows(list, settings.theme.velocity);
        RIGIDBODIES_EXPECT(drawn.size() == 8, "every slow body keeps an arrow");
        for (const auto& arrow : drawn)
            RIGIDBODIES_EXPECT_NEAR(length(arrow.tip - arrow.start), 14.0, 1.0e-9, "8 px of value is drawn as the 14 px stub");
        RIGIDBODIES_EXPECT(dotted_shaft(list, settings.theme.velocity), "the stub's shaft is dotted, unlike the break of a shortened arrow");
        std::size_t plates = 0, noted = 0;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::text)
            {
                const auto text = std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length);
                plates += text.substr(0, 2) == "v " ? 1u : 0u;
                noted += text == "not to scale" ? 1u : 0u;
            }
        std::size_t marks = 0;
        for (const auto& command : list.commands())
            marks += command.kind == DrawCommandKind::line && command.dash_length > 0.0f && same_color(command.color, settings.theme.label_muted) ? 1u : 0u;
        RIGIDBODIES_EXPECT(plates > 0 && noted + marks >= plates, "every stub's plate says it is not to scale, in words or with the dotted mark");
    }

    RIGIDBODIES_TEST("a fixed support pushes on the face it touches, from outside the body")
    {
        World world;
        world.add_force_generator(std::make_shared<UniformGravity>());
        add_floor(world);
        BodyDefinition definition;
        definition.position_m = { 0.0, 1.5 };
        Collider collider;
        collider.shape = make_box(2.0, 3.0);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 240);
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        const auto weight = arrows(list, settings.theme.force);
        const auto contact = arrows(list, settings.theme.contact);
        RIGIDBODIES_EXPECT(weight.size() == 1 && contact.size() == 1, "the block shows its weight and its support");
        const auto centre = camera.world_to_screen(world.find_body(id)->world_center_of_mass_m());
        const auto face_y = camera.world_to_screen({ 0.0, 0.0 }).y;
        RIGIDBODIES_EXPECT(length(weight.front().start - centre) < 10.0, "the weight leaves the centre of mass");
        RIGIDBODIES_EXPECT(weight.front().tip.y < face_y, "a weight shorter than half the block stays inside it");
        RIGIDBODIES_EXPECT(std::abs(contact.front().tip.y - face_y) <= 2.0 && contact.front().start.y > face_y, "the support comes from below and rests its head on the bottom face");
        RIGIDBODIES_EXPECT(std::abs(length(weight.front().tip - weight.front().start) - length(contact.front().tip - contact.front().start)) <= 0.05 * length(weight.front().tip - weight.front().start), "the push is drawn to the same scale as the weight it balances");
    }

    RIGIDBODIES_TEST("a joint's pull is a load beside its link that never reaches another body")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "distance_chain"), "the chain demonstration exists");
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::constraints, true);
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 90);
        const auto camera = framed_camera(world);
        settings.selection = body_named(world, "chain_mass_2");
        renderer.render(world, camera, settings, list);
        std::size_t reactions = 0;
        for (const auto& command : list.commands())
            reactions += command.kind == DrawCommandKind::polygon_outline && same_color(command.color, settings.theme.force) ? 1u : 0u;
        RIGIDBODIES_EXPECT(reactions == 2, "a middle mass shows the pull of both of its rods");
        std::vector<std::pair<Vec2, Vec2>> rods;
        for (const auto& constraint : world.constraints())
            if (const auto* joint = dynamic_cast<const JointConstraint*>(constraint.get()))
            {
                const auto report = joint->report(world);
                rods.emplace_back(camera.world_to_screen(report.first_anchor_m), camera.world_to_screen(report.second_anchor_m));
            }
        // The pulls are the force-hued arrows with an outlined head.
        std::vector<DrawnArrow> pulls;
        for (std::size_t index = 0; index + 1 < list.commands().size(); ++index)
        {
            const auto& shaft = list.commands()[index];
            const auto& head = list.commands()[index + 1];
            if (shaft.kind == DrawCommandKind::line && head.kind == DrawCommandKind::polygon_outline && same_color(shaft.color, settings.theme.force) && same_color(head.color, settings.theme.force))
                pulls.push_back({ list.vertices().at(shaft.vertex_offset), list.vertices().at(head.vertex_offset) });
        }
        RIGIDBODIES_EXPECT(pulls.size() == 2, "both pulls are drawn");
        for (const auto& arrow : pulls)
        {
            world.for_each_body([&](BodyId id, const RigidBody& body)
                {
                    if (!(id == settings.selection))
                        RIGIDBODIES_EXPECT(!body.contains_world_point(camera.screen_to_world(arrow.tip)), "a pull never ends inside another body");
                });
            const auto middle = (arrow.start + arrow.tip) * 0.5;
            for (const auto& [a, b] : rods)
            {
                const auto edge = b - a;
                const auto t = std::clamp(dot(middle - a, edge) / std::max(1.0e-9, length_squared(edge)), 0.0, 1.0);
                RIGIDBODIES_EXPECT(length(middle - (a + edge * t)) >= 6.0, "a pull is drawn beside the rod it acts through, never along it");
            }
        }

        RIGIDBODIES_EXPECT(has_text(list, "Rod \xC2\xB7 tension"), "the plate of a rod on the selection leaves the force to the arrow that states it");
        // With the whole chain selected only the primary selection shows its rods' pulls.
        renderer.set_selected_bodies({ body_named(world, "chain_mass_1"), body_named(world, "chain_mass_2"), body_named(world, "chain_mass_3"), body_named(world, "chain_mass_4"), body_named(world, "chain_mass_5") });
        renderer.render(world, camera, settings, list);
        reactions = 0;
        for (const auto& command : list.commands())
            reactions += command.kind == DrawCommandKind::polygon_outline && same_color(command.color, settings.theme.force) ? 1u : 0u;
        RIGIDBODIES_EXPECT(reactions == 2, "a large selection keeps only the primary selection's pulls");
    }

    RIGIDBODIES_TEST("a reading repeated on several bodies is said once, for each of them")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "stable_stack"), "the stack demonstration exists");
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 360);
        renderer.render(world, framed_camera(world), settings, list);
        std::size_t weights = 0;
        bool each = false;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::text)
            {
                const auto text = std::string_view { list.text_buffer() }.substr(command.text_offset, command.text_length);
                if (text.substr(0, 7) == "Weight ")
                {
                    ++weights;
                    each = text.size() > 5 && text.substr(text.size() - 5) == " each";
                }
            }
        RIGIDBODIES_EXPECT(weights == 1 && each, "six equal weights read once, as each box's");
    }

    RIGIDBODIES_TEST("bodies on shelves above one another keep their arrows inside their own lane")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "friction_comparison"), "the lanes demonstration exists");
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 30);
        auto camera = make_camera();
        Aabb bounds;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                const auto box = body.compute_bounds();
                bounds.expand(box.minimum);
                bounds.expand(box.maximum);
            });
        camera.set_center((bounds.minimum + bounds.maximum) * 0.5);
        camera.set_view_height(bounds.extents().y * 2.4);
        renderer.render(world, camera, settings, list);
        std::vector<double> shelf_tops;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                if (body.type() == BodyType::static_body && body.compute_bounds().extents().x > 2.0)
                    shelf_tops.push_back(camera.world_to_screen({ 0.0, body.compute_bounds().maximum.y }).y);
            });
        std::sort(shelf_tops.begin(), shelf_tops.end());
        const auto weights = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(!weights.empty() && shelf_tops.size() >= 3, "the lanes show their weights over their shelves");
        for (const auto& arrow : weights)
        {
            // The shelf a body rests on is the first top below its centre; the next one belongs to
            // the lane below, which its weight never reaches.
            const auto below = std::upper_bound(shelf_tops.begin(), shelf_tops.end(), arrow.start.y + 1.0);
            if (below != shelf_tops.end() && std::next(below) != shelf_tops.end())
                RIGIDBODIES_EXPECT(arrow.tip.y < *std::next(below) - 10.0, "a weight stops well short of the next lane's shelf");
        }
    }

    RIGIDBODIES_TEST("the scale key leaves a corner that an arrow runs through")
    {
        World world;
        world.add_force_generator(std::make_shared<UniformGravity>());
        BodyDefinition definition;
        definition.position_m = { -4.2, -3.0 };
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        world.create_body(definition);
        world.step(1.0 / 120.0);
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::grid, true);
        auto camera = make_camera();
        camera.set_focus_rect({ 0.0, 0.0, 1000.0, 800.0 });
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto weight = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(weight.size() >= 1, "the body near the lower-left corner shows its weight");
        bool found = false;
        for (const auto& command : list.commands())
            if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.layer == instrument_layer)
            {
                found = true;
                const auto minimum = list.vertices().at(command.vertex_offset), maximum = list.vertices().at(command.vertex_offset + 1);
                // The key's own arrow is drawn in the force hue too; only the body's is checked.
                for (const auto& arrow : weight)
                    for (int sample = 0; sample <= 8 && arrow.start.x < 200.0; ++sample)
                    {
                        const auto point = arrow.start + (arrow.tip - arrow.start) * (sample / 8.0);
                        RIGIDBODIES_EXPECT(!(point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y), "the key never covers an arrow");
                    }
            }
        RIGIDBODIES_EXPECT(found, "the scale key is drawn");
    }

    RIGIDBODIES_TEST("a body of several parts is outlined once around all of them when selected")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        for (const auto x : { -0.5, 0.5 })
        {
            Collider collider;
            collider.shape = make_box(1.0, 0.6);
            collider.local_transform.translation = { x, 0.0 };
            definition.colliders.push_back(collider);
        }
        const auto id = world.create_body(definition);
        SceneRenderSettings settings;
        settings.material_shading = false;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.layers.set(VisualizationLayer::outlines, true);
        settings.selection = id;
        SceneRenderer renderer;
        DrawList list;
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        const auto seam = camera.world_to_screen({ 0.0, 0.0 }).x;
        const auto top = camera.world_to_screen({ 0.0, 0.3 }).y, bottom = camera.world_to_screen({ 0.0, -0.3 }).y;
        std::size_t strokes = 0;
        for (const auto& command : list.commands())
            if (command.layer == 5 && same_color(command.color, settings.theme.selection) && (command.kind == DrawCommandKind::polyline || command.kind == DrawCommandKind::line))
                for (std::size_t index = 0; index < command.vertex_count; ++index)
                {
                    ++strokes;
                    const auto point = list.vertices().at(command.vertex_offset + index);
                    RIGIDBODIES_EXPECT(!(std::abs(point.x - seam) < 4.0 && point.y > top + 4.0 && point.y < bottom - 4.0), "no selection line runs down the seam between the parts");
                }
        RIGIDBODIES_EXPECT(strokes > 0, "the selection is outlined");
    }

    RIGIDBODIES_TEST("arrows may use the margin beyond the framing area but never reach under a sheet beside it")
    {
        World world { zero_gravity() };
        vector_body(world, { 0.0, -12.0 }, { 0.0, -1.5 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        auto camera = make_camera();
        camera.set_focus_rect({ 100.0, 60.0, 800.0, 600.0 });
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera, settings, list);
        const auto framed = arrows(list, settings.theme.velocity).front().tip.y;
        renderer.set_visible_stage({ 50.0, 40.0, 900.0, 720.0 });
        renderer.render(world, camera, settings, list);
        const auto visible = arrows(list, settings.theme.velocity).front().tip.y;
        RIGIDBODIES_EXPECT(framed <= 660.0 && visible > framed + 6.0 && visible <= 672.0, "the arrow runs on into the margin below the framing area, and no further");

        // A sheet over the stage's right side: the framing area stops 16 px short of it, and so,
        // with room to spare, does every arrow, though the visible stage runs on beneath it.
        World sideways { zero_gravity() };
        vector_body(sideways, { 12.0, 0.0 }, { 0.0, 0.0 });
        camera.set_focus_rect({ 100.0, 60.0, 600.0, 600.0 });
        renderer.set_visible_stage({ 0.0, 0.0, 1000.0, 800.0 });
        renderer.render(sideways, camera, settings, list);
        const auto tip = arrows(list, settings.theme.velocity).front().tip.x;
        RIGIDBODIES_EXPECT(tip > 700.0 && tip <= 716.0, "the arrow runs into the margin but stops short of the sheet beside the framing area");
    }

    RIGIDBODIES_TEST("without gravity the force scale follows the loads down once they stay far below it")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "magnus_effect"), "the spinning-disc demonstration exists");
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::grid, true);
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 24);
        renderer.render(world, framed_camera(world), settings, list);
        const auto launch = renderer.drawn_force_scale();
        bool announced = false;
        for (int step = 0; step < 360; step += 6)
        {
            settle(world, renderer, settings, 6);
            renderer.render(world, framed_camera(world), settings, list);
            announced = announced || has_text(list, "rescaled");
        }
        RIGIDBODIES_EXPECT(renderer.drawn_force_scale() > 2.0 * launch, "the scale grows as the discs slow and their loads fall away");
        RIGIDBODIES_EXPECT(announced, "the key says when the forces were rescaled");
    }

    bool within_rect(const std::pair<Vec2, Vec2>& rect, const Vec2& point)
    {
        return point.x >= rect.first.x && point.x <= rect.second.x && point.y >= rect.first.y && point.y <= rect.second.y;
    }

    RIGIDBODIES_TEST("arrows in focus are cased in the plate tone, beneath every arrow")
    {
        World world;
        world.add_force_generator(std::make_shared<UniformGravity>());
        add_floor(world);
        BodyDefinition definition;
        definition.position_m = { 0.0, 1.5 };
        Collider collider;
        collider.shape = make_box(2.0, 3.0);
        definition.colliders.push_back(collider);
        world.create_body(definition);
        auto settings = forces_only();
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 240);
        renderer.render(world, make_camera(), settings, list);
        const auto weight = arrows(list, settings.theme.force);
        RIGIDBODIES_EXPECT(weight.size() == 1, "the block shows its weight");
        std::size_t casing = list.commands().size(), shaft = list.commands().size();
        for (std::size_t index = 0; index < list.commands().size(); ++index)
        {
            const auto& command = list.commands()[index];
            if (casing == list.commands().size() && command.kind == DrawCommandKind::indexed_mesh && command.layer == 20)
                casing = index;
            if (shaft == list.commands().size() && command.kind == DrawCommandKind::line && same_color(command.color, settings.theme.force))
                shaft = index;
        }
        RIGIDBODIES_EXPECT(casing < shaft, "the casings are listed before the first arrow, so they lie beneath it");
        const auto& mesh = *list.commands()[casing].mesh;
        const auto plate = settings.theme.label_plate;
        bool cased = false;
        for (const auto& vertex : mesh.vertices)
            cased = cased || (std::abs(vertex.color.alpha - 0.85f) < 1.0e-3f && std::abs(vertex.color.red - plate.red * 0.85f) < 1.0e-3f && length(vertex.position - weight.front().start) < 4.0);
        RIGIDBODIES_EXPECT(cased, "the weight's shaft is cased in the plate tone");
    }

    RIGIDBODIES_TEST("no plate covers an arrowhead, of the selection's arrows or the receding rest")
    {
        struct Case
        {
            const char* scenario;
            int steps;
            const char* selected;
            bool whole_chain;
        };
        for (const auto& item : { Case { "shape_workshop", 105, nullptr, false }, Case { "free_fall", 240, "wooden_ball", false }, Case { "distance_chain", 96, "chain_mass_5", true }, Case { "stable_stack", 330, "stack_box_1", false } })
        {
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, item.scenario), "the demonstration exists");
            auto settings = forces_only();
            settings.layers.set(VisualizationLayer::velocity_vectors, true);
            settings.layers.set(VisualizationLayer::constraints, true);
            settings.layers.set(VisualizationLayer::center_of_mass, true);
            SceneRenderer renderer;
            DrawList list;
            settle(world, renderer, settings, item.steps);
            if (item.selected)
                settings.selection = body_named(world, item.selected);
            if (item.whole_chain)
                renderer.set_selected_bodies({ body_named(world, "chain_mass_5"), body_named(world, "chain_mass_4"), body_named(world, "chain_mass_3"), body_named(world, "chain_mass_2"), body_named(world, "chain_mass_1") });
            renderer.render(world, framed_camera(world), settings, list);
            std::vector<std::pair<Vec2, Vec2>> plates;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.layer != instrument_layer && same_color(command.color, settings.theme.label_plate))
                    plates.emplace_back(list.vertices().at(command.vertex_offset) + Vec2 { 1.0, 1.0 }, list.vertices().at(command.vertex_offset + 1) - Vec2 { 1.0, 1.0 });
            RIGIDBODIES_EXPECT(!plates.empty(), "the scene's readings are on plates");
            for (const auto& color : { settings.theme.force, settings.theme.contact, settings.theme.velocity })
                for (const auto& shade : { color, color.with_alpha(color.alpha * 0.4f) })
                    for (const auto& arrow : arrows(list, shade))
                        for (const auto& plate : plates)
                            RIGIDBODIES_EXPECT(!within_rect(plate, arrow.tip), item.scenario);
        }
    }

    RIGIDBODIES_TEST("a value a few pixels short of the stub is drawn true, unmarked")
    {
        World world { zero_gravity() };
        vector_body(world, { 1.2, 0.0 });
        auto settings = vectors_only();
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::velocity_vectors, true);
        settings.layers.set(VisualizationLayer::labels, true);
        settings.vector_scales.velocity = 10.0;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, make_camera(), settings, list);
        const auto drawn = arrows(list, settings.theme.velocity);
        RIGIDBODIES_EXPECT(drawn.size() == 1, "the velocity is drawn");
        RIGIDBODIES_EXPECT_NEAR(length(drawn.front().tip - drawn.front().start), 12.0, 1.0e-9, "12 px of value is drawn as 12 px");
        RIGIDBODIES_EXPECT(!dotted_shaft(list, settings.theme.velocity), "its shaft is solid");
        RIGIDBODIES_EXPECT(list.text_buffer().find("not to scale") == std::string::npos, "and its plate makes no claim that it is off scale");
    }

    RIGIDBODIES_TEST("a reaction arrow starts beyond its body's centre symbol")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "distance_chain"), "the chain demonstration exists");
        auto settings = forces_only();
        settings.layers.set(VisualizationLayer::constraints, true);
        settings.layers.set(VisualizationLayer::center_of_mass, true);
        SceneRenderer renderer;
        DrawList list;
        settle(world, renderer, settings, 90);
        const auto camera = framed_camera(world);
        settings.selection = body_named(world, "chain_mass_1");
        renderer.render(world, camera, settings, list);
        const auto centre = camera.world_to_screen(world.find_body(settings.selection)->world_center_of_mass_m());
        std::size_t pulls = 0;
        for (std::size_t index = 0; index + 1 < list.commands().size(); ++index)
        {
            const auto& shaft = list.commands()[index];
            const auto& head = list.commands()[index + 1];
            if (shaft.kind == DrawCommandKind::line && head.kind == DrawCommandKind::polygon_outline && same_color(shaft.color, settings.theme.force) && same_color(head.color, settings.theme.force))
            {
                ++pulls;
                RIGIDBODIES_EXPECT(length(list.vertices().at(shaft.vertex_offset) - centre) > 6.0, "the pull's shaft starts clear of the centre symbol");
            }
        }
        RIGIDBODIES_EXPECT(pulls == 2, "both rods' pulls are drawn");
    }

    RIGIDBODIES_TEST("an edge two parts of a body share is outlined once")
    {
        World world { zero_gravity() };
        BodyDefinition definition;
        Collider wide, narrow;
        wide.shape = make_box(2.0, 0.6);
        narrow.shape = make_box(1.0, 0.6);
        narrow.local_transform.translation = { 0.25, 0.0 };
        definition.colliders = { wide, narrow };
        const auto id = world.create_body(definition);
        SceneRenderSettings settings;
        settings.material_shading = false;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.layers.set(VisualizationLayer::outlines, true);
        settings.selection = id;
        SceneRenderer renderer;
        DrawList list;
        const auto camera = make_camera();
        renderer.render(world, camera, settings, list);
        const auto top = camera.world_to_screen({ 0.0, 0.3 }).y;
        double stroked = 0.0, left = 1.0e9, right = -1.0e9;
        for (const auto& command : list.commands())
            if (command.layer == 5 && same_color(command.color, settings.theme.selection) && (command.kind == DrawCommandKind::polyline || command.kind == DrawCommandKind::line))
                for (std::size_t index = 0; index + 1 < command.vertex_count; ++index)
                {
                    const auto a = list.vertices().at(command.vertex_offset + index), b = list.vertices().at(command.vertex_offset + index + 1);
                    if (std::abs(a.y - b.y) < 0.5 && a.y < top && a.y > top - 6.0)
                    {
                        stroked += std::abs(b.x - a.x);
                        left = std::min({ left, a.x, b.x });
                        right = std::max({ right, a.x, b.x });
                    }
                }
        RIGIDBODIES_EXPECT(stroked > 0.0 && stroked <= right - left + 6.0, "the top edge is stroked once along its length");
    }

    RIGIDBODIES_TEST("unit conversion respects dimensions and leaves unavailable numbers explicit")
    {
        using namespace rigidbodies::core;
        const auto cgs = DisplayUnits::centimetre_gram;
        RIGIDBODIES_EXPECT_NEAR(display_value(0.025, DisplayQuantity::length, cgs), 2.5, 1.0e-12, "metres convert to centimetres");
        RIGIDBODIES_EXPECT_NEAR(display_value(0.25, DisplayQuantity::mass, cgs), 250.0, 1.0e-12, "kilograms convert to grams");
        RIGIDBODIES_EXPECT_NEAR(display_value(1.0, DisplayQuantity::force, cgs), 100000.0, 1.0e-9, "newtons convert consistently to dynes");
        RIGIDBODIES_EXPECT_NEAR(display_value(1.0, DisplayQuantity::energy, cgs), 10000000.0, 1.0e-9, "joules convert consistently to ergs");
        RIGIDBODIES_EXPECT_NEAR(display_value(1.0, DisplayQuantity::inertia, cgs), 10000000.0, 1.0e-9, "inertia applies mass times squared length conversion");
        RIGIDBODIES_EXPECT_NEAR(display_value(1000.0, DisplayQuantity::density, cgs), 1.0, 1.0e-12, "density applies inverse cubic length conversion");
        RIGIDBODIES_EXPECT(format_quantity(0.25, DisplayQuantity::mass, cgs) == "250\xC2\xA0g", "small household masses use familiar grams");
        RIGIDBODIES_EXPECT(format_quantity(std::numeric_limits<double>::infinity(), DisplayQuantity::velocity, cgs) == "\xE2\x80\x94", "nonfinite readouts never masquerade as measurements");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
