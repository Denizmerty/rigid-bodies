#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/render/draw_compiler.hpp>
#include <rigidbodies/physics/authored_body.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::render;
    using namespace rigidbodies::physics;
    using namespace rigidbodies::math;

    World make_world()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        return World { settings };
    }

    Camera2D camera()
    {
        Camera2D value;
        value.set_viewport({ 800, 600 });
        value.set_view_height(6.0);
        return value;
    }

    BodyId ball(World& world, Vec2 position = {}, Vec2 velocity = {}, Material material = materials::rubber())
    {
        BodyDefinition definition;
        definition.name = "Reference ball";
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        Collider collider;
        collider.shape = make_circle(0.25);
        collider.material = std::move(material);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    void floor(World& world)
    {
        BodyDefinition definition;
        definition.type = BodyType::static_body;
        Collider collider;
        collider.shape = make_box(30.0, 0.1);
        collider.material = materials::steel();
        definition.colliders.push_back(collider);
        world.create_body(definition);
    }

    void advance_to_impact(World& world, SceneRenderer& renderer, const SceneRenderSettings& settings)
    {
        for (int step = 0; step < 100; ++step)
        {
            world.step(0.01);
            renderer.record_visual_sample(world, 0.01, settings);
            if (renderer.effect_statistics().impact_bursts > 0)
                return;
        }
        RIGIDBODIES_FAIL("fixture creates a measured collision episode");
    }

    SceneRenderSettings effects_only()
    {
        SceneRenderSettings settings;
        settings.layers = LayerMask::none();
        settings.depth_background = false;
        settings.transitions = false;
        settings.motion_trails = false;
        settings.directional_blur = false;
        return settings;
    }

    std::size_t count_kind(const DrawList& list, DrawCommandKind kind)
    {
        return static_cast<std::size_t>(std::count_if(list.commands().begin(), list.commands().end(), [kind](const auto& command)
            {
                return command.kind == kind;
            }));
    }

    RIGIDBODIES_TEST("grid rendering skips unrepresentable imported or panned camera centers")
    {
        auto world = make_world();
        auto settings = effects_only();
        settings.layers.set(VisualizationLayer::grid, true);
        SceneRenderer renderer;
        for (const auto center : { Vec2 { 1.0e20, 0.0 }, Vec2 { 0.0, -1.0e20 }, Vec2 { std::numeric_limits<double>::max(), std::numeric_limits<double>::max() } })
        {
            auto view = camera();
            view.set_center(center);
            DrawList list;
            renderer.render(world, view, settings, list);
            RIGIDBODIES_EXPECT(count_kind(list, DrawCommandKind::line) == 0, "grid smaller than coordinate precision is skipped instead of looping forever");
        }
        auto overflowing_view = camera();
        overflowing_view.set_viewport({ std::numeric_limits<int>::max(), 1 });
        overflowing_view.set_view_height_limits(1.0, std::numeric_limits<double>::max());
        overflowing_view.set_view_height(1.0e308);
        DrawList overflow;
        renderer.render(world, overflowing_view, settings, overflow);
        RIGIDBODIES_EXPECT(count_kind(overflow, DrawCommandKind::line) == 0, "overflowed visible bounds are rejected before grid division or integer conversion");
    }

    RIGIDBODIES_TEST("grid bounds extreme viewport work and retains ordinary line positions")
    {
        auto world = make_world();
        auto settings = effects_only();
        settings.layers.set(VisualizationLayer::grid, true);
        SceneRenderer renderer;
        auto view = camera();
        DrawList ordinary;
        renderer.render(world, view, settings, ordinary);
        // Grid lines share one mesh; the two origin axes are separate strokes.
        RIGIDBODIES_EXPECT(count_kind(ordinary, DrawCommandKind::line) == 2 && count_kind(ordinary, DrawCommandKind::indexed_mesh) == 1, "one grid mesh and two axes");
        const auto& grid = *std::find_if(ordinary.commands().begin(), ordinary.commands().end(), [](const auto& command)
            {
                return command.kind == DrawCommandKind::indexed_mesh;
            })->mesh;
        const auto has_vertex = [&](Vec2 expected)
        {
            return std::any_of(grid.vertices.begin(), grid.vertices.end(), [&](const MeshVertex& vertex)
                {
                    return std::abs(vertex.position.x - expected.x) < 1.0e-9 && std::abs(vertex.position.y - expected.y) < 1.0e-9;
                });
        };
        RIGIDBODIES_EXPECT(has_vertex({ 0.5, 0.0 }) && has_vertex({ -0.5, 600.0 }), "first vertical grid line retains its exact screen position");
        RIGIDBODIES_EXPECT(grid.feathered && grid.vertices.size() % 8 == 0, "every grid line carries its own feathered edges");
        // Eight by six metres at 100 px/m: decimetre lines are 10 px apart and only just fading
        // in, metre lines carry the emphasis.
        const auto alpha_at = [&](double x)
        {
            float alpha = 0.0f;
            for (const auto& vertex : grid.vertices)
                if (std::abs(vertex.position.x - x - 0.5) < 1.0e-9 && vertex.position.y == 0.0)
                    alpha = vertex.color.alpha;
            return alpha;
        };
        const auto metre_x = view.world_to_screen({ 1.0, 0.0 }).x, decimetre_x = view.world_to_screen({ 1.1, 0.0 }).x;
        RIGIDBODIES_EXPECT(alpha_at(metre_x) > alpha_at(decimetre_x) * 4.0f && alpha_at(decimetre_x) > 0.0f, "fine lines fade with on-screen spacing while coarser decades stay");
        view.set_viewport({ std::numeric_limits<int>::max(), 1 });
        DrawList extreme;
        renderer.render(world, view, settings, extreme);
        RIGIDBODIES_EXPECT(count_kind(extreme, DrawCommandKind::line) <= 2, "the axes are the only grid strokes");
        for (const auto& command : extreme.commands())
            if (command.mesh)
            {
                RIGIDBODIES_EXPECT(command.mesh->vertices.size() <= 2 * 4096 * 8, "each axis has a hard line budget independent of viewport ratio");
                for (const auto& vertex : command.mesh->vertices)
                    RIGIDBODIES_EXPECT(is_finite(vertex.position), "bounded grid mesh contains only finite coordinates");
            }
        for (const auto vertex : extreme.vertices())
            RIGIDBODIES_EXPECT(is_finite(vertex), "bounded output contains only finite coordinates");
    }

    RIGIDBODIES_TEST("physical material records determine distinct surface and softness responses")
    {
        const auto oak = material_appearance(materials::oak_wood());
        const auto steel = material_appearance(materials::steel());
        const auto rubber = material_appearance(materials::rubber());
        RIGIDBODIES_EXPECT(oak.surface.red > oak.surface.blue, "oak has warm grain-colour shading");
        RIGIDBODIES_EXPECT(steel.surface.blue > steel.surface.red, "steel has a cool metal surface");
        RIGIDBODIES_EXPECT(steel.roughness < rubber.roughness, "metal has a tighter brighter rim than rubber");
        RIGIDBODIES_EXPECT(rubber.softness > 0.5f && steel.softness == 0.0f, "only soft materials receive squash cues");
        auto changed = materials::rubber();
        changed.restitution = 0.1;
        RIGIDBODIES_EXPECT(material_appearance(changed).softness < rubber.softness, "restitution changes the visible soft response");
    }

    RIGIDBODIES_TEST("default scene shades material with directional gradient and keeps overlays above geometry")
    {
        auto world = make_world();
        const auto id = ball(world, {}, {}, materials::steel());
        SceneRenderSettings settings;
        settings.selection = id;
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(list.commands().front().layer == -30, "background has an explicit far layer");
        bool shaded = false, selection = false;
        for (const auto& command : list.commands())
        {
            if (command.kind == DrawCommandKind::indexed_mesh && command.layer == 0 && command.mesh->feathered)
            {
                // The surface is lit from the upper left: inside the outline, the face toward
                // that corner is brighter than the face toward the lower right.
                Vec2 minimum { 1.0e300, 1.0e300 }, maximum { -1.0e300, -1.0e300 };
                for (const auto& vertex : command.mesh->vertices)
                {
                    minimum = min_components(minimum, vertex.position);
                    maximum = max_components(maximum, vertex.position);
                }
                const auto middle = (minimum + maximum) * 0.5;
                const auto reach = (maximum.x - minimum.x) * 0.2;
                const auto nearest = [&](Vec2 target)
                {
                    const MeshVertex* best = nullptr;
                    for (const auto& vertex : command.mesh->vertices)
                        if (!best || length(vertex.position - target) < length(best->position - target))
                            best = &vertex;
                    return best;
                };
                const auto* lit = nearest(middle - Vec2 { reach, reach });
                const auto* away = nearest(middle + Vec2 { reach, reach });
                shaded = lit && away && lit->color.red > away->color.red;
                for (const auto& vertex : command.mesh->vertices)
                    RIGIDBODIES_EXPECT(vertex.color.red <= vertex.color.alpha + 1.0e-6f, "surface vertices are premultiplied");
            }
            if (command.kind == DrawCommandKind::circle_outline && command.layer == 5 && command.color.blue == settings.theme.selection.blue)
                selection = true;
        }
        RIGIDBODIES_EXPECT(shaded && selection, "material form and shape-following selection are both visible");
        settings.shading_body_budget = 0;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(std::none_of(list.commands().begin(), list.commands().end(), [](const auto& command)
                               {
                                   return command.kind == DrawCommandKind::indexed_mesh && command.layer == 0;
                               }) &&
                count_kind(list, DrawCommandKind::circle_fill) > 0,
            "zero shading budget retains readable flat body geometry");
    }

    RIGIDBODIES_TEST("concave authored material uses one continuous gradient mesh across cached triangles")
    {
        auto world = make_world();
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
        part.material = materials::glass();
        create_authored_body(world, {}, { part });
        auto settings = effects_only();
        settings.layers.set(VisualizationLayer::bodies, true);
        SceneRenderer renderer;
        DrawList list;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(count_kind(list, DrawCommandKind::indexed_mesh) == 1, "triangles share one surface without internal feather seams");
        for (const auto& command : list.commands())
            if (command.mesh)
            {
                RIGIDBODIES_EXPECT(command.mesh->indices.size() == part.shape->render_triangles.size() * 3, "all cached concave triangles are present");
                for (const auto& vertex : command.mesh->vertices)
                    RIGIDBODIES_EXPECT(vertex.color.red <= vertex.color.alpha && vertex.color.green <= vertex.color.alpha && vertex.color.blue <= vertex.color.alpha, "translucent glass uses valid premultiplied mesh colours");
            }
    }

    RIGIDBODIES_TEST("impact bursts have fixed per-effect budgets and repeated renders cannot duplicate them")
    {
        auto world = make_world();
        floor(world);
        for (int index = 0; index < 8; ++index)
            ball(world, { static_cast<double>(index) - 4.0, 0.35 }, { 0.0, -5.0 });
        auto settings = effects_only();
        settings.impact_flash_budget = 1;
        settings.spark_budget = 3;
        settings.dust_budget = 2;
        settings.deformation_budget = 1;
        SceneRenderer renderer;
        advance_to_impact(world, renderer, settings);
        const auto before = renderer.effect_statistics();
        RIGIDBODIES_EXPECT(before.flashes == 1 && before.sparks == 3 && before.dust == 2 && before.deformations == 1, "hard ceilings hold when simultaneous impacts saturate pools");
        DrawList first, second;
        renderer.render(world, camera(), settings, first);
        renderer.record_visual_sample(world, 0.01, settings);
        renderer.render(world, camera(), settings, second);
        const auto after = renderer.effect_statistics();
        RIGIDBODIES_EXPECT(after.impact_bursts == before.impact_bursts && after.sparks == before.sparks, "duplicate sampling and pause do not create or age particles");
        RIGIDBODIES_EXPECT(first.commands().size() == second.commands().size() && first.vertices() == second.vertices(), "repeated rendering records identical geometry");
    }

    RIGIDBODIES_TEST("impact flash sparks and dust are independently switchable and expire")
    {
        auto world = make_world();
        floor(world);
        ball(world, { 0.0, 0.35 }, { 0.0, -5.0 });
        auto settings = effects_only();
        SceneRenderer renderer;
        advance_to_impact(world, renderer, settings);
        DrawList list;
        settings.impact_flashes = settings.impact_sparks = settings.impact_dust = false;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(list.is_empty(), "all impact visuals can be hidden immediately while paused");
        settings.impact_flashes = true;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(count_kind(list, DrawCommandKind::circle_outline) > 0 && count_kind(list, DrawCommandKind::line) == 0, "flash does not require sparks");
        settings.impact_flashes = false;
        settings.impact_sparks = true;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(count_kind(list, DrawCommandKind::line) > 0 && count_kind(list, DrawCommandKind::circle_fill) == 0, "sparks do not require dust");
        settings.impact_sparks = false;
        settings.impact_dust = true;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(count_kind(list, DrawCommandKind::instanced_mesh) == 1 && count_kind(list, DrawCommandKind::line) == 0, "dust uses one shared mesh batch without requiring sparks");
        for (int step = 0; step < 100; ++step)
        {
            world.step(0.01);
            renderer.record_visual_sample(world, 0.01, settings);
        }
        const auto expired = renderer.effect_statistics();
        RIGIDBODIES_EXPECT(expired.flashes == 0 && expired.sparks == 0 && expired.dust == 0 && expired.deformations == 0, "every transient pool releases its entries after a bounded lifetime");
    }

    RIGIDBODIES_TEST("harder measured impulses produce stronger reproducible visual bursts")
    {
        auto gentle = make_world();
        auto hard = make_world();
        floor(gentle);
        floor(hard);
        ball(gentle, { 0.0, 0.35 }, { 0.0, -1.0 });
        ball(hard, { 0.0, 0.35 }, { 0.0, -8.0 });
        auto settings = effects_only();
        SceneRenderer gentle_renderer, hard_renderer, repeated_renderer;
        advance_to_impact(gentle, gentle_renderer, settings);
        advance_to_impact(hard, hard_renderer, settings);
        RIGIDBODIES_EXPECT(hard_renderer.effect_statistics().sparks > gentle_renderer.effect_statistics().sparks, "a larger measured impulse emits more sparks inside the same budget");
        repeated_renderer.record_visual_sample(hard, 0.01, settings);
        DrawList first, second;
        hard_renderer.render(hard, camera(), settings, first);
        repeated_renderer.render(hard, camera(), settings, second);
        RIGIDBODIES_EXPECT(first.vertices() == second.vertices() && first.commands().size() == second.commands().size(), "body identifiers and physics step deterministically seed particle geometry");
        for (std::size_t index = 0; index < first.commands().size(); ++index)
        {
            const auto& command = first.commands()[index];
            const auto& repeated = second.commands()[index];
            if (command.mesh)
            {
                RIGIDBODIES_EXPECT(command.mesh->instances.size() == repeated.mesh->instances.size(), "dust has the same bounded instance count");
                for (std::size_t instance = 0; instance < command.mesh->instances.size(); ++instance)
                    RIGIDBODIES_EXPECT(command.mesh->instances[instance].translation == repeated.mesh->instances[instance].translation && command.mesh->instances[instance].tint.alpha == repeated.mesh->instances[instance].tint.alpha, "instanced dust positions and opacity replay exactly");
            }
        }
    }

    RIGIDBODIES_TEST("soft impact deformation changes drawing but never collider geometry or body state")
    {
        auto world = make_world();
        floor(world);
        const auto id = ball(world, { 0.0, 0.35 }, { 0.0, -5.0 });
        auto settings = effects_only();
        settings.layers.set(VisualizationLayer::bodies, true);
        SceneRenderer renderer;
        advance_to_impact(world, renderer, settings);
        const auto* body = world.find_body(id);
        const auto position = body->position_m();
        const auto velocity = body->linear_velocity_m_s();
        const auto collider = body->colliders().front().shape;
        // The drawn ball's aspect: one when round, below one while squashed against the floor.
        const auto aspect = [](const DrawList& list)
        {
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::indexed_mesh && command.layer == 0 && command.mesh)
                {
                    Vec2 minimum { 1.0e300, 1.0e300 }, maximum { -1.0e300, -1.0e300 };
                    for (const auto& vertex : command.mesh->vertices)
                    {
                        minimum = min_components(minimum, vertex.position);
                        maximum = max_components(maximum, vertex.position);
                    }
                    return (maximum.y - minimum.y) / (maximum.x - minimum.x);
                }
            RIGIDBODIES_FAIL("the ball surface is drawn");
        };
        DrawList deformed, original;
        renderer.render(world, camera(), settings, deformed);
        settings.deformation_budget = 0;
        renderer.render(world, camera(), settings, original);
        RIGIDBODIES_EXPECT(aspect(deformed) < 0.995 && std::abs(aspect(original) - 1.0) < 0.01, "zero deformation budget removes the cue immediately while paused");
        settings.deformation_budget = 32;
        settings.soft_deformation = false;
        renderer.render(world, camera(), settings, original);
        RIGIDBODIES_EXPECT(std::abs(aspect(original) - 1.0) < 0.01, "deformation has an independent toggle");
        RIGIDBODIES_EXPECT(body->position_m() == position && body->linear_velocity_m_s() == velocity && body->colliders().front().shape == collider, "rendering cannot change physical pose velocity or geometry");
        RIGIDBODIES_EXPECT_NEAR(dynamic_cast<const CircleShape*>(collider.get())->radius_m(), 0.25, 0.0, "collision radius stays exact");
    }

    RIGIDBODIES_TEST("visual sampling with all effects preserves a parallel physics run exactly")
    {
        auto observed = make_world();
        auto reference = make_world();
        floor(observed);
        floor(reference);
        const auto observed_id = ball(observed, { 0.0, 0.5 }, { 0.5, -3.0 });
        const auto reference_id = ball(reference, { 0.0, 0.5 }, { 0.5, -3.0 });
        SceneRenderer renderer;
        SceneRenderSettings settings;
        DrawList list;
        for (int step = 0; step < 120; ++step)
        {
            observed.step(1.0 / 120.0);
            renderer.record_visual_sample(observed, 1.0 / 120.0, settings);
            renderer.advance_presentation(1.0 / 60.0);
            renderer.render(observed, camera(), settings, list);
            reference.step(1.0 / 120.0);
        }
        const auto* first = observed.find_body(observed_id);
        const auto* second = reference.find_body(reference_id);
        RIGIDBODIES_EXPECT(first->position_m() == second->position_m() && first->linear_velocity_m_s() == second->linear_velocity_m_s(), "position and velocity remain bit-identical with visual work enabled");
        RIGIDBODIES_EXPECT(first->orientation_rad() == second->orientation_rad() && first->angular_velocity_rad_s() == second->angular_velocity_rad_s(), "rotation remains bit-identical");
        RIGIDBODIES_EXPECT(observed.statistics().total_kinetic_energy_j == reference.statistics().total_kinetic_energy_j, "energy results remain identical");
    }

    RIGIDBODIES_TEST("motion history and directional blur obey separate budgets and slot generations")
    {
        auto world = make_world();
        std::vector<BodyId> ids;
        ids.reserve(10);
        for (int index = 0; index < 10; ++index)
            ids.push_back(ball(world, { static_cast<double>(index) - 5.0, 1.0 }, { 5.0, 0.0 }));
        SceneRenderSettings settings;
        settings.depth_background = false;
        settings.layers = LayerMask::none();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.motion_body_budget = 3;
        settings.directional_blur_budget = 2;
        SceneRenderer renderer;
        for (int step = 0; step < 5; ++step)
        {
            world.step(0.01);
            renderer.record_visual_sample(world, 0.01, settings);
        }
        RIGIDBODIES_EXPECT(renderer.effect_statistics().motion_bodies == 3, "history body budget bounds pool occupancy");
        settings.motion_trails = false;
        DrawList list;
        renderer.render(world, camera(), settings, list);
        const auto behind = static_cast<std::size_t>(std::count_if(list.commands().begin(), list.commands().end(), [](const auto& command)
            {
                return command.layer == -10;
            }));
        RIGIDBODIES_EXPECT(behind == 6, "two bodies each use exactly three bounded blur bands independently of trails");
        for (const auto id : ids)
            world.destroy_body(id);
        world.step(0.01);
        renderer.record_visual_sample(world, 0.01, settings);
        RIGIDBODIES_EXPECT(renderer.effect_statistics().motion_bodies == 0, "destroyed generations release stale trail slots");
        renderer.clear_trajectories();
        RIGIDBODIES_EXPECT(renderer.effect_statistics().impact_bursts == 0, "existing world reset hook clears all visual history");
    }

    RIGIDBODIES_TEST("contact shadows identify support without the contact-point overlay")
    {
        auto world = make_world();
        floor(world);
        ball(world, { 0.0, 0.35 }, { 0.0, -5.0 });
        auto settings = effects_only();
        settings.layers.set(VisualizationLayer::bodies, true);
        settings.contact_shadow_budget = 1;
        SceneRenderer renderer;
        advance_to_impact(world, renderer, settings);
        DrawList list;
        renderer.render(world, camera(), settings, list);
        // Occlusion sits above fixed ground (-8) and below moving bodies (0) as one soft mesh
        // whose rim is fully transparent.
        const auto occlusion = std::find_if(list.commands().begin(), list.commands().end(), [](const auto& command)
            {
                return command.layer == -6 && command.kind == DrawCommandKind::indexed_mesh;
            });
        RIGIDBODIES_EXPECT(occlusion != list.commands().end() && occlusion->mesh->feathered, "contact occlusion is drawn between ground and bodies");
        RIGIDBODIES_EXPECT(occlusion->mesh->vertices.size() == 1 + 4 * 24, "one contact receives one soft occlusion with a bounded budget");
        RIGIDBODIES_EXPECT(occlusion->mesh->vertices.back().color.alpha == 0.0f && occlusion->mesh->vertices.front().color.alpha > 0.0f, "occlusion fades from its centre to nothing");
        settings.contact_shadows = false;
        renderer.render(world, camera(), settings, list);
        RIGIDBODIES_EXPECT(std::none_of(list.commands().begin(), list.commands().end(), [](const auto& command)
                               {
                                   return command.kind == DrawCommandKind::shadow || command.layer == -6 || command.layer == -7;
                               }),
            "shadow toggle removes contact and drop shadows without changing contacts");
    }

    RIGIDBODIES_TEST("scene typography scales and theme easing advances only once per frame")
    {
        auto world = make_world();
        ball(world);
        SceneRenderSettings settings;
        settings.display_scale = 2.0f;
        settings.layers.set(VisualizationLayer::labels, true);
        SceneRenderer renderer;
        DrawList initial, transitioning, repeated;
        renderer.render(world, camera(), settings, initial);
        // Annotations are 12 logical pixels against the atlas' 14 pixel base.
        constexpr float label_ratio = 12.0f / 14.0f;
        std::size_t name_runs = 0, value_runs = 0, plates = 0;
        for (const auto& command : initial.commands())
        {
            if (command.kind == DrawCommandKind::text)
            {
                RIGIDBODIES_EXPECT(command.text_scale == 2.0f * label_ratio, "all scene labels honor display density");
                const auto text = initial.text_buffer().substr(command.text_offset, command.text_length);
                name_runs += text == "Reference ball" && command.color.red == settings.theme.label_text.red ? 1u : 0u;
                value_runs += text == "1.00\xC2\xA0kg" && command.color.red == settings.theme.label_muted.red ? 1u : 0u;
            }
            if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.color.alpha >= 0.8f && command.layer == 20)
                ++plates;
        }
        RIGIDBODIES_EXPECT(name_runs == 1 && value_runs == 1 && plates >= 1, "name and muted mass share one readable plate over any material");
        settings.theme = theme_by_name("workbench_light");
        renderer.advance_presentation(1.0 / 60.0);
        renderer.render(world, camera(), settings, transitioning);
        renderer.render(world, camera(), settings, repeated);
        const auto start = initial.commands().front().color.red;
        const auto eased = transitioning.commands().front().color.red;
        RIGIDBODIES_EXPECT(eased > start && eased < settings.theme.background.red, "theme crosses an intermediate eased value");
        RIGIDBODIES_EXPECT(eased == repeated.commands().front().color.red, "an extra export pass does not consume another easing step");
        settings.display_scale = std::numeric_limits<float>::quiet_NaN();
        renderer.render(world, camera(), settings, repeated);
        for (const auto& command : repeated.commands())
            if (command.kind == DrawCommandKind::text)
                RIGIDBODIES_EXPECT(command.text_scale == label_ratio, "invalid scale uses a finite readable fallback");
    }

    RIGIDBODIES_TEST("crowded annotations never overlap and the selection keeps its readouts")
    {
        auto world = make_world();
        std::vector<BodyId> ids;
        for (int index = 0; index < 9; ++index)
            ids.push_back(ball(world, { -1.2 + index * 0.3, (index % 3) * 0.25 }, { 1.5 + index * 0.2, 1.0 }));
        SceneRenderSettings settings;
        settings.depth_background = false;
        settings.transitions = false;
        settings.layers = LayerMask::none();
        for (const auto layer : { VisualizationLayer::bodies, VisualizationLayer::labels, VisualizationLayer::velocity_vectors, VisualizationLayer::momentum_vectors, VisualizationLayer::center_of_mass })
            settings.layers.set(layer, true);
        settings.selection = ids[4];
        SceneRenderer renderer;
        DrawList list;
        for (const auto scale : { 1.0f, 1.5f, 2.0f })
        {
            settings.display_scale = scale;
            renderer.render(world, camera(), settings, list);
            std::vector<std::pair<Vec2, Vec2>> plates;
            for (const auto& command : list.commands())
                if (command.kind == DrawCommandKind::rounded_rectangle_fill && command.layer == 20)
                    plates.emplace_back(list.vertices().at(command.vertex_offset), list.vertices().at(command.vertex_offset + 1));
            RIGIDBODIES_EXPECT(!plates.empty(), "labels are drawn");
            for (std::size_t a = 0; a < plates.size(); ++a)
                for (std::size_t b = a + 1; b < plates.size(); ++b)
                {
                    const auto width = std::min(plates[a].second.x, plates[b].second.x) - std::max(plates[a].first.x, plates[b].first.x);
                    const auto height = std::min(plates[a].second.y, plates[b].second.y) - std::max(plates[a].first.y, plates[b].first.y);
                    RIGIDBODIES_EXPECT(width <= 0.0 || height <= 0.0, "annotation plates never overlap one another");
                }
            for (const auto& plate : plates)
                RIGIDBODIES_EXPECT(plate.first.x >= 0.0 && plate.first.y >= 0.0 && plate.second.x <= 800.0 && plate.second.y <= 600.0, "plates stay on the stage");
        }
        const auto selected = world.find_body(ids[4]);
        const auto speed = rigidbodies::core::format_quantity(length(selected->linear_velocity_m_s()), rigidbodies::core::DisplayQuantity::velocity, settings.display_units);
        RIGIDBODIES_EXPECT(list.text_buffer().find("v " + speed) != std::string::npos, "the selected body's velocity readout survives crowding");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
