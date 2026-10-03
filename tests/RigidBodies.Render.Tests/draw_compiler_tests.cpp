#include <rigidbodies/render/draw_compiler.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace
{
    using namespace rigidbodies::render;
    using namespace rigidbodies::math;

    struct Bounds
    {
        Vec2 minimum { 1.0e30, 1.0e30 }, maximum { -1.0e30, -1.0e30 };
    };

    Bounds bounds(const IndexedMesh& mesh, bool opaque_only = false)
    {
        Bounds result;
        for (const auto& value : mesh.vertices)
        {
            if (opaque_only && value.color.alpha == 0.0f)
                continue;
            result.minimum = min_components(result.minimum, value.position);
            result.maximum = max_components(result.maximum, value.position);
        }
        return result;
    }

    void expect_valid(const IndexedMesh& mesh)
    {
        RIGIDBODIES_EXPECT(mesh.indices.size() % 3 == 0, "complete triangles");
        for (const auto& value : mesh.vertices)
        {
            RIGIDBODIES_EXPECT(is_finite(value.position), "finite positions");
            RIGIDBODIES_EXPECT(value.color.red <= value.color.alpha + 1.0e-6f && value.color.green <= value.color.alpha + 1.0e-6f && value.color.blue <= value.color.alpha + 1.0e-6f, "premultiplied colour does not exceed alpha");
        }
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            const auto a = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i])).position;
            const auto b = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i + 1])).position;
            const auto c = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i + 2])).position;
            RIGIDBODIES_EXPECT(cross(b - a, c - a) > 0.0, "consistent nondegenerate winding");
        }
    }

    double core_area(const IndexedMesh& mesh)
    {
        double area = 0.0;
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            const auto& a = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i]));
            const auto& b = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i + 1]));
            const auto& c = mesh.vertices.at(static_cast<std::size_t>(mesh.indices[i + 2]));
            if (a.color.alpha > 0.0f && b.color.alpha > 0.0f && c.color.alpha > 0.0f)
                area += cross(b.position - a.position, c.position - a.position) * 0.5;
        }
        return area;
    }

    RIGIDBODIES_TEST("convex fills have premultiplied feather coverage independent of input winding")
    {
        for (const auto& points : { std::vector<Vec2> { { 10, 10 }, { 30, 10 }, { 30, 30 }, { 10, 30 } }, std::vector<Vec2> { { 10, 30 }, { 30, 30 }, { 30, 10 }, { 10, 10 } } })
        {
            DrawList list;
            list.add_polygon_fill(points, { 1.0f, 0.4f, 0.2f, 0.5f });
            const auto compiled = compile_draw_list(list);
            RIGIDBODIES_EXPECT(compiled.meshes.size() == 1, "one compiled mesh");
            const auto& mesh = compiled.meshes.front();
            expect_valid(mesh);
            RIGIDBODIES_EXPECT_NEAR(core_area(mesh), 400.0, 1.0e-8, "interior retained exactly");
            RIGIDBODIES_EXPECT_NEAR(bounds(mesh).minimum.x, 9.0, 1.0e-8, "one pixel outward feather");
            RIGIDBODIES_EXPECT_NEAR(mesh.vertices.front().color.red, 0.5, 1.0e-6, "colour premultiplied once");
            RIGIDBODIES_EXPECT(std::count_if(mesh.vertices.begin(), mesh.vertices.end(), [](const auto& value)
                                   {
                                       return value.color.alpha == 0.0f;
                                   }) == 4,
                "transparent outside boundary");
        }
    }

    RIGIDBODIES_TEST("curve tessellation follows the screen error budget as radius grows")
    {
        int previous = 0;
        for (const auto radius : { 2.0f, 10.0f, 100.0f, 1000.0f })
        {
            const auto count = curve_segment_count(radius);
            RIGIDBODIES_EXPECT(count >= previous && count <= 4096, "adaptive bounded count");
            RIGIDBODIES_EXPECT(radius * (1.0 - std::cos(pi / count)) <= 0.25001, "actual arc error below quarter pixel");
            previous = count;
        }
        RIGIDBODIES_EXPECT(curve_segment_count(-1.0f) == 0 && curve_segment_count(std::numeric_limits<float>::infinity()) == 0, "invalid radii rejected");
        RIGIDBODIES_EXPECT(curve_segment_count(100.0f, 0.05f) > curve_segment_count(100.0f, 0.5f), "tighter error adds vertices");
    }

    RIGIDBODIES_TEST("path caps cover their intended extent and remain feathered")
    {
        for (const auto cap : { StrokeCap::butt, StrokeCap::square, StrokeCap::round })
        {
            DrawList list;
            list.add_path({ { 20, 20 }, { 80, 20 } }, {}, 10.0f, false, StrokeJoin::round, cap);
            const auto compiled = compile_draw_list(list);
            const auto& mesh = compiled.meshes.front();
            expect_valid(mesh);
            const auto extent = bounds(mesh, true);
            RIGIDBODIES_EXPECT_NEAR(extent.minimum.x, cap == StrokeCap::butt ? 20.0 : 15.0, 0.251, "cap start extent");
            RIGIDBODIES_EXPECT_NEAR(extent.maximum.x, cap == StrokeCap::butt ? 80.0 : 85.0, 0.251, "cap end extent");
            RIGIDBODIES_EXPECT(bounds(mesh).minimum.y < extent.minimum.y, "stroke edge feather present");
        }
    }

    RIGIDBODIES_TEST("joined paths have a continuous interior and bounded acute corners")
    {
        for (const auto join : { StrokeJoin::miter, StrokeJoin::bevel, StrokeJoin::round })
        {
            DrawList list;
            list.add_path({ { 10, 30 }, { 30, 30 }, { 30, 10 } }, {}, 10.0f, false, join, StrokeCap::butt);
            const auto result = compile_draw_list(list);
            expect_valid(result.meshes.front());
            const auto area = core_area(result.meshes.front());
            RIGIDBODIES_EXPECT(area >= 387.4 && area <= 400.01, "joined area includes the exterior corner with no quad overlap");
        }
        DrawList acute;
        acute.add_path({ { 10, 10 }, { 30, 10 }, { 10, 10.01 } }, {}, 10.0f, false, StrokeJoin::miter, StrokeCap::butt);
        const auto result = compile_draw_list(acute);
        expect_valid(result.meshes.front());
        RIGIDBODIES_EXPECT(bounds(result.meshes.front()).maximum.x <= 55.0, "miter limit bounds near reversals");
    }

    RIGIDBODIES_TEST("closed outlines join the seam and preserve the hollow interior")
    {
        DrawList list;
        list.add_path({ { 10, 10 }, { 30, 10 }, { 30, 30 }, { 10, 30 } }, {}, 2.0f, true, StrokeJoin::miter);
        const auto result = compile_draw_list(list);
        expect_valid(result.meshes.front());
        RIGIDBODIES_EXPECT_NEAR(core_area(result.meshes.front()), 160.0, 1.0e-8, "square ring area is outer minus inner");
        RIGIDBODIES_EXPECT_NEAR(bounds(result.meshes.front(), true).minimum.x, 9.0, 1.0e-8, "outside stroke extent");
    }

    RIGIDBODIES_TEST("rounded fills trim corner pixels and gradients follow a shared axis")
    {
        DrawList rounded;
        rounded.add_rounded_rectangle_fill({ 0, 0 }, { 80, 40 }, 8.0f, {});
        const auto compiled = compile_draw_list(rounded);
        expect_valid(compiled.meshes.front());
        RIGIDBODIES_EXPECT(core_area(compiled.meshes.front()) < 3200.0 && core_area(compiled.meshes.front()) > 3100.0, "corners rounded away");
        RIGIDBODIES_EXPECT(std::none_of(compiled.meshes.front().vertices.begin(), compiled.meshes.front().vertices.end(), [](const auto& value)
                               {
                                   return value.position == Vec2 { 0, 0 };
                               }),
            "square corner is absent");
        DrawList gradient;
        gradient.add_gradient_polygon_fill({ { 0, 0 }, { 40, 0 }, { 40, 40 }, { 0, 40 } }, { 1, 0, 0, 1 }, { 0, 0, 1, 1 }, { 0, 0 }, { 40, 0 });
        const auto mesh = compile_draw_list(gradient).meshes.front();
        RIGIDBODIES_EXPECT(mesh.vertices[0].color.red == 1.0f && mesh.vertices[1].color.blue == 1.0f, "gradient endpoint colours preserved");
    }

    RIGIDBODIES_TEST("drop shadows offset geometry and fall to a transparent blur boundary")
    {
        DrawList list;
        list.add_rounded_rectangle_shadow({ 10, 10 }, { 40, 30 }, 4.0f, { 0, 0, 0, 0.4f }, 8.0f, { 3, 5 });
        const auto mesh = compile_draw_list(list).meshes.front();
        expect_valid(mesh);
        const auto solid = bounds(mesh, true), soft = bounds(mesh);
        RIGIDBODIES_EXPECT_NEAR(solid.minimum.x, 13.0, 1.0e-8, "shadow translated horizontally");
        RIGIDBODIES_EXPECT_NEAR(solid.maximum.y, 35.0, 1.0e-8, "shadow translated vertically");
        RIGIDBODIES_EXPECT(soft.minimum.x < 6.0 && soft.maximum.y > 42.0, "blur surrounds shadow");
    }

    RIGIDBODIES_TEST("a soft shadow keeps its blur width around a thin tip instead of spiking")
    {
        // A 13 degree tip: a mitred offset would push the blur band far beyond the point.
        const std::vector<Vec2> blade { { 0.0, 0.0 }, { 120.0, 13.7 }, { 0.0, 27.4 } };
        DrawList list;
        list.add_shadow(blade, { 0, 0, 0, 0.5f }, 13.5f, {});
        const auto mesh = compile_draw_list(list).meshes.front();
        expect_valid(mesh);
        const auto distance_to_blade = [&](const Vec2& point)
        {
            auto best = 1.0e30;
            for (std::size_t index = 0; index < blade.size(); ++index)
            {
                const auto& a = blade[index];
                const auto& b = blade[(index + 1) % blade.size()];
                const auto t = std::clamp(dot(point - a, b - a) / length_squared(b - a), 0.0, 1.0);
                best = std::min(best, length(point - (a + (b - a) * t)));
            }
            return best;
        };
        double farthest = 0.0, past_tip = 0.0;
        for (const auto& value : mesh.vertices)
        {
            farthest = std::max(farthest, distance_to_blade(value.position));
            if (value.position.x > 120.0)
                past_tip = std::max(past_tip, length(value.position - blade[1]));
        }
        RIGIDBODIES_EXPECT(past_tip <= 13.5 * 1.01, "around the tip the blur band is a rounded cap of the blur radius");
        RIGIDBODIES_EXPECT(farthest <= 13.5 * 2.0 + 1.0e-9, "no corner pushes the band beyond the mitre limit");
        RIGIDBODIES_EXPECT(bounds(mesh).maximum.x >= 120.0 + 13.5 * 0.95, "the band still reaches its full width past the tip");
    }

    RIGIDBODIES_TEST("layer ordering is stable and batching does not reorder transparent paint")
    {
        DrawList list;
        list.set_layer(20);
        list.add_rectangle_fill({ 20, 0 }, { 21, 1 }, { 1, 0, 0, 0.5f });
        list.set_layer(-10);
        list.add_rectangle_fill({ 1, 0 }, { 2, 1 }, {});
        list.set_layer(20);
        list.add_rectangle_fill({ 30, 0 }, { 31, 1 }, { 0, 1, 0, 0.5f });
        list.add_rectangle_fill({ 40, 0 }, { 41, 1 }, { 0, 0, 1, 0.5f });
        const auto result = compile_draw_list(list);
        RIGIDBODIES_EXPECT(result.meshes.size() == 2, "only compatible commands on same layer batch");
        RIGIDBODIES_EXPECT(result.meshes[0].vertices[0].position.x == 1 && result.meshes[1].vertices[0].position.x == 20, "back layer sorted first");
        RIGIDBODIES_EXPECT(result.meshes[1].vertices[8].position.x == 30 && result.meshes[1].vertices[16].position.x == 40, "same-layer paint remains ordered");
        RIGIDBODIES_EXPECT(result.stats.command_count == 4 && result.stats.mesh_count == 2 && result.stats.triangle_count > 0, "compile statistics describe output");
    }

    RIGIDBODIES_TEST("nested clipping intersects and prevents unsafe batching")
    {
        DrawList list;
        list.push_clip({ 10, 10, 40, 40 });
        list.add_circle_fill({ 25, 25 }, 8, {});
        list.push_clip({ 20, 0, 50, 30 });
        list.add_circle_fill({ 25, 25 }, 8, {});
        list.push_clip({ 100, 100, 4, 4 });
        list.add_circle_fill({ 25, 25 }, 8, {});
        list.pop_clip();
        list.pop_clip();
        list.add_circle_fill({ 25, 25 }, 8, {});
        list.pop_clip();
        list.add_circle_fill({ 25, 25 }, 8, {});
        const auto result = compile_draw_list(list);
        RIGIDBODIES_EXPECT(result.meshes.size() == 4, "empty clip discarded and changed scissor separates batches");
        const auto clip = result.meshes[1].clip;
        RIGIDBODIES_EXPECT(clip && clip->x == 20 && clip->y == 10 && clip->width == 30 && clip->height == 20, "nested intersection exact");
        RIGIDBODIES_EXPECT(!result.meshes.back().clip, "pop restores unclipped paint");
        list.clear();
        list.add_circle_fill({}, 1, {});
        RIGIDBODIES_EXPECT(!list.commands().front().clip && list.layer() == 0, "clear resets layer and clip scopes");
    }

    RIGIDBODIES_TEST("textured quads retain ownership and batch only adjacent compatible textures")
    {
        auto texture = std::make_shared<TexturePixels>();
        texture->width = texture->height = 1;
        texture->rgba = { 255, 255, 255, 255 };
        const std::weak_ptr<const TexturePixels> weak = texture;
        DrawList list;
        list.add_textured_quad(texture, { 0, 0 }, { 20, 20 }, { 1, 1, 1, 0.5f });
        list.add_textured_quad(texture, { 20, 0 }, { 40, 20 });
        list.add_rectangle_fill({ 0, 0 }, { 1, 1 }, {});
        list.add_textured_quad(texture, { 40, 0 }, { 60, 20 });
        auto result = compile_draw_list(list);
        list.clear();
        texture.reset();
        RIGIDBODIES_EXPECT(result.meshes.size() == 3 && !weak.expired(), "frame owns texture and preserves intervening paint");
        RIGIDBODIES_EXPECT(result.meshes.front().vertices[0].uv == Vec2 { 0, 0 } && result.meshes.front().vertices[2].uv == Vec2 { 1, 1 }, "quad UV range preserved");
        result.meshes.clear();
        RIGIDBODIES_EXPECT(weak.expired(), "texture dies after last compiled frame reference");
    }

    RIGIDBODIES_TEST("instance batches preserve local geometry and independent transforms")
    {
        DrawList prototype;
        prototype.add_circle_fill({}, 3, {});
        auto mesh = std::make_shared<IndexedMesh>(compile_draw_list(prototype).meshes.front());
        DrawList list;
        list.add_instanced_mesh(mesh, { { { 10, 20 }, { 2, 3 }, 0.2f, { 1, 0, 0, 0.5f } }, { { 30, 40 }, { 1, 1 }, 0.0f, {} } });
        list.add_indexed_mesh(mesh);
        const auto result = compile_draw_list(list);
        RIGIDBODIES_EXPECT(result.meshes.size() == 2 && result.meshes.front().instances.size() == 2, "instances survive compilation without flattening or merging");
        RIGIDBODIES_EXPECT(result.meshes.front().instances[0].translation == Vec2 { 10, 20 }, "transform retained");
        RIGIDBODIES_EXPECT(result.meshes.front().vertices.front().position == mesh->vertices.front().position && mesh->instances.empty(), "base geometry remains immutable and local");
        RIGIDBODIES_EXPECT(result.stats.triangle_count == mesh->indices.size(), "triangle statistics include instance count");
    }

    RIGIDBODIES_TEST("font callback follows the same layer clipping and batching contracts")
    {
        DrawList list;
        list.push_clip({ 1, 2, 30, 40 });
        list.add_text({ 10, 20 }, "Width 10", { 1, 1, 1, 0.5f }, 2.0f);
        bool invoked = false;
        const auto result = compile_draw_list(list, [&](Vec2 position, std::string_view text, Color color, float scale)
            {
                invoked = position == Vec2 { 10, 20 } && text == "Width 10" && color.alpha == 0.5f && scale == 2.0f;
                DrawList glyph;
                glyph.add_rectangle_fill(position, position + Vec2 { 10, 10 }, color);
                return compile_draw_list(glyph).meshes;
            });
        RIGIDBODIES_EXPECT(invoked && result.meshes.size() == 1 && result.meshes.front().clip->x == 1, "font geometry retains text inputs and scissor");
    }

    RIGIDBODIES_TEST("invalid degenerate input is discarded and batch size is bounded")
    {
        DrawList list;
        list.add_line({ 1, 1 }, { 1, 1 }, {});
        list.add_circle_fill({}, -2, {});
        list.add_polygon_fill({ { 0, 0 }, { 1, 1 }, { 2, 2 } }, {});
        list.add_line({ std::numeric_limits<double>::infinity(), 0 }, { 1, 1 }, {});
        auto malformed = std::make_shared<IndexedMesh>();
        malformed->vertices.push_back({ {}, {}, {} });
        malformed->indices = { 0, 1, 2 };
        list.add_indexed_mesh(malformed);
        RIGIDBODIES_EXPECT(compile_draw_list(list).meshes.empty(), "invalid draws create no device work");
        list.clear();
        for (int i = 0; i < 5; ++i)
            list.add_rectangle_fill({ i * 2.0, 0 }, { i * 2.0 + 1.0, 1 }, {});
        DrawCompileOptions options;
        options.max_batch_vertices = 16;
        const auto result = compile_draw_list(list, {}, options);
        RIGIDBODIES_EXPECT(result.meshes.size() == 3, "bounded adjacent batching splits commands safely");
        for (const auto& mesh : result.meshes)
            expect_valid(mesh);
    }

    RIGIDBODIES_TEST("raw document meshes feather only exposed boundaries and never internal triangle seams")
    {
        auto mesh = std::make_shared<IndexedMesh>();
        // Two triangles intentionally duplicate the diagonal's endpoint indices, as document
        // tessellators can do. Geometry topology must match positions rather than buffer indices.
        mesh->vertices = { { { 10, 10 }, {}, {} }, { { 30, 10 }, {}, {} }, { { 30, 30 }, {}, {} }, { { 10, 10 }, {}, {} }, { { 30, 30 }, {}, {} }, { { 10, 30 }, {}, {} } };
        mesh->indices = { 0, 1, 2, 3, 4, 5 };
        DrawList list;
        list.add_indexed_mesh(mesh);
        const auto result = compile_draw_list(list);
        const auto& compiled = result.meshes.front();
        expect_valid(compiled);
        RIGIDBODIES_EXPECT(compiled.indices.size() == 30, "only four exposed edges receive two feather triangles each");
        RIGIDBODIES_EXPECT_NEAR(core_area(compiled), 400.0, 1.0e-8, "interior and diagonal remain unchanged");
        RIGIDBODIES_EXPECT_NEAR(bounds(compiled).minimum.x, 9.0, 1.0e-8, "document surface gets same one pixel feather");
        DrawList second;
        second.add_indexed_mesh(std::make_shared<IndexedMesh>(compiled));
        const auto repeated = compile_draw_list(second);
        RIGIDBODIES_EXPECT(repeated.meshes.front().indices.size() == compiled.indices.size(), "already feathered boundary is idempotent");
    }

    RIGIDBODIES_TEST("pixel aligned interface meshes keep grid edges hard and feather only the rest")
    {
        const auto compile = [](std::vector<MeshVertex> vertices, std::vector<int> indices, bool pixel_aligned, bool instanced = false)
        {
            auto mesh = std::make_shared<IndexedMesh>();
            mesh->vertices = std::move(vertices);
            mesh->indices = std::move(indices);
            mesh->pixel_aligned = pixel_aligned;
            DrawList list;
            if (instanced)
                list.add_instanced_mesh(mesh, { { { 0, 0 }, { 1, 1 }, 0.5f, {} } });
            else
                list.add_indexed_mesh(mesh);
            return compile_draw_list(list).meshes.front();
        };
        // A one pixel divider on the grid stays one pixel wide instead of a three pixel smear.
        const auto rule = compile({ { { 10, 20 }, {}, {} }, { { 90, 20 }, {}, {} }, { { 90, 21 }, {}, {} }, { { 10, 21 }, {}, {} } }, { 0, 1, 2, 0, 2, 3 }, true);
        RIGIDBODIES_EXPECT(rule.indices.size() == 6 && rule.vertices.size() == 4, "grid-aligned rule gains no band");
        RIGIDBODIES_EXPECT_NEAR(bounds(rule).minimum.y, 20.0, 0.0, "rule keeps its exact top edge");
        // Half-pixel positions and slanted edges still need coverage antialiasing.
        const auto offset = compile({ { { 10.5, 20 }, {}, {} }, { { 90.5, 20 }, {}, {} }, { { 90.5, 21 }, {}, {} }, { { 10.5, 21 }, {}, {} } }, { 0, 1, 2, 0, 2, 3 }, true);
        RIGIDBODIES_EXPECT_NEAR(bounds(offset).minimum.x, 9.5, 1.0e-8, "off-grid vertical edges are feathered");
        RIGIDBODIES_EXPECT_NEAR(bounds(offset).minimum.y, 20.0, 1.0e-8, "on-grid horizontal edges in the same mesh stay hard");
        const auto corner = compile({ { { 10, 10 }, {}, {} }, { { 30, 10 }, {}, {} }, { { 10, 30 }, {}, {} } }, { 0, 1, 2 }, true);
        RIGIDBODIES_EXPECT(corner.indices.size() == 3 + 2 * 3 && corner.vertices.size() == 3 + 2, "only the diagonal of a corner wedge is banded");
        {
            auto wedge = std::make_shared<IndexedMesh>();
            wedge->vertices = { { { 10, 10 }, {}, {} }, { { 30, 10 }, {}, {} }, { { 10, 30 }, {}, {} } };
            wedge->indices = { 0, 1, 2 };
            wedge->pixel_aligned = true;
            DrawList list;
            list.add_indexed_mesh(wedge);
            DrawCompileOptions multisampled;
            multisampled.multisampled = true;
            RIGIDBODIES_EXPECT(compile_draw_list(list, {}, multisampled).meshes.front().indices.size() == 3, "a multisampled target antialiases interface curves itself");
            wedge->pixel_aligned = false;
            RIGIDBODIES_EXPECT(compile_draw_list(list, {}, multisampled).meshes.front().indices.size() == 3 + 3 * 6, "scene meshes keep their band on every target");
        }
        // Instance vertices are local units, so the flag cannot vouch for their screen alignment.
        const auto rotated = compile({ { { 0, 0 }, {}, {} }, { { 20, 0 }, {}, {} }, { { 20, 20 }, {}, {} }, { { 0, 20 }, {}, {} } }, { 0, 1, 2, 0, 2, 3 }, true, true);
        RIGIDBODIES_EXPECT(rotated.indices.size() == 30, "rotated instances keep a band on every exposed edge");
        const auto unflagged = compile({ { { 10, 20 }, {}, {} }, { { 90, 20 }, {}, {} }, { { 90, 21 }, {}, {} }, { { 10, 21 }, {}, {} } }, { 0, 1, 2, 0, 2, 3 }, false);
        RIGIDBODIES_EXPECT(unflagged.indices.size() == 30, "scene meshes keep their full band");
    }

    RIGIDBODIES_TEST("raw mesh feathering preserves holes and leaves glyph atlas quads untouched")
    {
        DrawList outline;
        outline.add_path({ { 10, 10 }, { 30, 10 }, { 30, 30 }, { 10, 30 } }, {}, 2, true, StrokeJoin::miter);
        DrawCompileOptions hard;
        hard.feather = 0;
        auto ring = std::make_shared<IndexedMesh>(compile_draw_list(outline, {}, hard).meshes.front());
        DrawList list;
        list.add_indexed_mesh(ring);
        const auto result = compile_draw_list(list);
        RIGIDBODIES_EXPECT_NEAR(core_area(result.meshes.front()), 160.0, 1.0e-8, "ring retains the central hole");
        RIGIDBODIES_EXPECT(std::any_of(result.meshes.front().vertices.begin(), result.meshes.front().vertices.end(), [](const auto& value)
                               {
                                   return value.color.alpha == 0.0f && value.position == Vec2 { 12, 12 };
                               }),
            "inner feather expands into hole");
        auto atlas = std::make_shared<TexturePixels>();
        atlas->width = atlas->height = 1;
        atlas->rgba = { 255, 255, 255, 255 };
        ring->texture = atlas;
        DrawList glyph;
        glyph.add_indexed_mesh(ring);
        const auto untouched = compile_draw_list(glyph);
        RIGIDBODIES_EXPECT(untouched.meshes.front().vertices.size() == ring->vertices.size() && untouched.meshes.front().indices == ring->indices, "textured atlas coverage is retained exactly");
    }

    RIGIDBODIES_TEST("malformed texture buffers and nonfinite mesh inputs never reach a device upload")
    {
        auto invalid = std::make_shared<TexturePixels>();
        invalid->width = invalid->height = 32;
        invalid->rgba = { 255, 255, 255, 255 };
        DrawList textured;
        textured.add_textured_quad(invalid, {}, { 20, 20 });
        RIGIDBODIES_EXPECT(compile_draw_list(textured).meshes.empty(), "truncated pixel buffer is rejected before upload reads it");
        DrawList source;
        source.add_rectangle_fill({}, { 10, 10 }, {});
        auto mesh = std::make_shared<IndexedMesh>(compile_draw_list(source).meshes.front());
        mesh->vertices[0].color.alpha = std::numeric_limits<float>::quiet_NaN();
        DrawList color;
        color.add_indexed_mesh(mesh);
        RIGIDBODIES_EXPECT(compile_draw_list(color).meshes.empty(), "nonfinite premultiplied colors cannot reach blending");
        mesh->vertices[0].color.alpha = 1.0f;
        DrawList instances;
        instances.add_instanced_mesh(mesh, { { { std::numeric_limits<double>::infinity(), 0 }, { 1, 1 }, 0, {} } });
        RIGIDBODIES_EXPECT(compile_draw_list(instances).meshes.empty(), "all-invalid instances do not draw an unexpected origin copy");
    }

    RIGIDBODIES_TEST("mixed and anisotropic raw instances retain a one pixel feather and stable painter order")
    {
        auto quad = std::make_shared<IndexedMesh>();
        quad->vertices = { { { 0, 0 }, {}, {} }, { { 1, 0 }, {}, {} }, { { 1, 1 }, {}, {} }, { { 0, 1 }, {}, {} } };
        quad->indices = { 0, 1, 2, 0, 2, 3 };
        DrawList list;
        list.add_instanced_mesh(quad, { { { 20, 20 }, { 1, 1 }, 0, {} }, { { 50, 20 }, { 100, 10 }, 0, {} }, { { 70, 20 }, { -100, 10 }, 0.3f, {} }, { { 90, 20 }, { 1, 1 }, 0, {} } });
        const auto result = compile_draw_list(list);
        RIGIDBODIES_EXPECT(result.meshes.size() == 3, "only consecutive matching scale magnitudes share a group");
        const auto small = bounds(result.meshes[0]), large = bounds(result.meshes[1]);
        RIGIDBODIES_EXPECT_NEAR(small.minimum.x, -1.0, 1.0e-6, "small instance receives a full pixel feather");
        RIGIDBODIES_EXPECT_NEAR(large.minimum.x, -1.0, 1.0e-6, "magnified instance receives the same pixel feather");
        RIGIDBODIES_EXPECT_NEAR(large.maximum.x, 101.0, 1.0e-6, "large horizontal scale does not inflate feather");
        RIGIDBODIES_EXPECT_NEAR(large.maximum.y, 11.0, 1.0e-6, "anisotropic vertical scale keeps correct coverage");
        RIGIDBODIES_EXPECT(result.meshes[1].instances.size() == 2 && result.meshes[1].instances[1].scale.x == -1.0 && result.meshes[1].instances[1].rotation == 0.3f, "shared GPU group retains reflection and rotation");
        RIGIDBODIES_EXPECT(result.meshes[2].instances[0].translation.x == 90.0, "nonadjacent small instance stays last in paint order");
    }

    RIGIDBODIES_TEST("label contrast plates measure proportional glyph ink and respect scale layer and clipping")
    {
        auto atlas = std::make_shared<TexturePixels>();
        atlas->width = atlas->height = 1;
        atlas->rgba = { 255, 255, 255, 255 };
        const TextMeshBuilder glyphs = [&](Vec2 position, std::string_view text, Color, float scale)
        {
            const double width = (text == "WWW" ? 36.0 : 9.0) * scale;
            const auto start = position + Vec2 { 2.0 * scale, 4.0 * scale };
            IndexedMesh mesh;
            mesh.texture = atlas;
            mesh.vertices = { { start, { 0, 0 }, {} }, { start + Vec2 { width, 0 }, { 1, 0 }, {} }, { start + Vec2 { width, 10.0 * scale }, { 1, 1 }, {} }, { start + Vec2 { 0, 10.0 * scale }, { 0, 1 }, {} } };
            mesh.indices = { 0, 1, 2, 0, 2, 3 };
            return std::vector<IndexedMesh> { std::move(mesh) };
        };
        const auto compile_label = [&](std::string_view text, float scale)
        {
            DrawList list;
            list.set_layer(10);
            list.push_clip({ 5, 6, 120, 80 });
            list.add_text_label({ 20, 20 }, text, {}, { 0.1f, 0.15f, 0.2f, 0.95f }, scale);
            list.pop_clip();
            list.set_layer(-1);
            list.add_rectangle_fill({ 0, 0 }, { 10, 10 }, {});
            RIGIDBODIES_EXPECT(list.commands().front().kind == DrawCommandKind::text && list.vertices().front() == Vec2 { 20, 20 }, "label recording retains text kind and anchor");
            return compile_draw_list(list, glyphs);
        };
        const auto wide = compile_label("WWW", 1), narrow = compile_label("iii", 1), large = compile_label("WWW", 2);
        RIGIDBODIES_EXPECT(wide.meshes.size() == 3 && wide.meshes[1].clip && wide.meshes[2].clip, "scene layer, label surface and atlas paint retain order and clipping");
        const auto wide_box = bounds(wide.meshes[1], true), narrow_box = bounds(narrow.meshes[1], true), large_box = bounds(large.meshes[1], true);
        RIGIDBODIES_EXPECT_NEAR(wide_box.maximum.x - wide_box.minimum.x, 44.0, 1.0e-6, "wide glyph ink plus exact padding");
        RIGIDBODIES_EXPECT_NEAR(narrow_box.maximum.x - narrow_box.minimum.x, 17.0, 1.0e-6, "same character count with narrow glyphs uses a narrower plate");
        RIGIDBODIES_EXPECT_NEAR(large_box.maximum.x - large_box.minimum.x, 88.0, 1.0e-6, "glyph ink and padding scale together");
        RIGIDBODIES_EXPECT_NEAR(wide_box.minimum.x, 18.0, 1.0e-6, "glyph bearing is included in plate placement");
        RIGIDBODIES_EXPECT(wide.meshes[1].clip->x == 5 && wide.meshes[2].clip->x == 5 && !wide.meshes.front().clip, "both plate and text obey their original clip scope");
    }

    RIGIDBODIES_TEST("dashed strokes compile into one mesh whose gaps follow the requested pitch")
    {
        DrawList list;
        list.add_dashed_line({ 0, 10 }, { 70, 10 }, { 1.0f, 1.0f, 1.0f, 1.0f }, 2.0f, 4.0f, 3.0f);
        RIGIDBODIES_EXPECT(list.commands().size() == 1, "a dashed stroke is one command");
        DrawCompileOptions options;
        options.feather = 0.0f;
        const auto compiled = compile_draw_list(list, {}, options);
        RIGIDBODIES_EXPECT(compiled.meshes.size() == 1, "every dash shares one mesh");
        const auto& mesh = compiled.meshes.front();
        expect_valid(mesh);
        // Seventy pixels at four on, three off: ten dashes of four by two pixels.
        RIGIDBODIES_EXPECT_NEAR(core_area(mesh), 80.0, 1.0e-6, "coverage matches the dash pattern");
        for (const auto& value : mesh.vertices)
            RIGIDBODIES_EXPECT(!(value.position.x > 4.0 + 1.0e-9 && value.position.x < 7.0 - 1.0e-9), "gaps stay empty");

        DrawList closed;
        closed.add_dashed_polyline({ { 0, 0 }, { 40, 0 }, { 40, 40 }, { 0, 40 } }, {}, 1.5f, 6.0f, 4.0f, true);
        const auto outline = compile_draw_list(closed);
        RIGIDBODIES_EXPECT(outline.meshes.size() == 1 && !outline.meshes.front().indices.empty(), "closed dashed outlines include the closing edge");
        expect_valid(outline.meshes.front());
        RIGIDBODIES_EXPECT(bounds(outline.meshes.front(), true).maximum.y > 39.0, "the dash pattern continues around corners");
    }

    RIGIDBODIES_TEST("tapered strokes interpolate width and colour by arc length")
    {
        DrawList list;
        list.add_tapered_polyline({ { 0, 0 }, { 50, 0 }, { 100, 0 } }, { 1.0f, 0.0f, 0.0f, 0.2f }, { 1.0f, 0.0f, 0.0f, 1.0f }, 2.0f, 10.0f);
        DrawCompileOptions options;
        options.feather = 0.0f;
        const auto compiled = compile_draw_list(list, {}, options);
        RIGIDBODIES_EXPECT(compiled.meshes.size() == 1, "one ribbon mesh");
        const auto& mesh = compiled.meshes.front();
        expect_valid(mesh);
        RIGIDBODIES_EXPECT_NEAR(core_area(mesh), 600.0, 1.0e-6, "the ribbon widens linearly from two to ten pixels");
        bool middle = false;
        for (const auto& value : mesh.vertices)
            if (std::abs(value.position.x - 50.0) < 1.0e-9 && std::abs(std::abs(value.position.y) - 3.0) < 1.0e-9)
                middle = middle || std::abs(value.color.alpha - 0.6f) < 1.0e-6f;
        RIGIDBODIES_EXPECT(middle, "the halfway station has the halfway width and opacity");
    }

    RIGIDBODIES_TEST("a densely sampled trail keeps its ends and shape with one station per pixel")
    {
        // A slow body sampled every step: 400 points along a quarter circle about 94 pixels long.
        std::vector<Vec2> samples;
        for (int index = 0; index <= 400; ++index)
        {
            const auto angle = rigidbodies::math::half_pi * index / 400.0;
            samples.push_back({ 60.0 * std::cos(angle), 60.0 * std::sin(angle) });
        }
        DrawList list;
        list.add_tapered_polyline(samples, { 1.0f, 1.0f, 1.0f, 0.2f }, { 1.0f, 1.0f, 1.0f, 1.0f }, 2.0f, 10.0f);
        DrawCompileOptions options;
        options.feather = 0.0f;
        const auto compiled = compile_draw_list(list, {}, options);
        const auto& mesh = compiled.meshes.front();
        expect_valid(mesh);
        RIGIDBODIES_EXPECT(mesh.vertices.size() <= 4 * 100 && mesh.vertices.size() >= 4 * 40, "stations closer than a pixel are merged and the curve keeps its stations");
        const auto near = [&](Vec2 point, double half_width)
        {
            return std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [&](const auto& value)
                {
                    return std::abs(rigidbodies::math::length(value.position - point) - half_width) < 1.0e-6;
                });
        };
        RIGIDBODIES_EXPECT(near({ 60.0, 0.0 }, 1.0) && near({ 0.0, 60.0 }, 5.0), "both ends keep their exact station and width");
    }

    RIGIDBODIES_TEST("producer feathered meshes pass through without a second antialiasing band")
    {
        const auto make = [](bool feathered)
        {
            auto mesh = std::make_shared<IndexedMesh>();
            mesh->vertices = { { { 0, 0 }, {}, { 1, 1, 1, 1 } }, { { 20, 0 }, {}, { 1, 1, 1, 1 } }, { { 0, 20 }, {}, { 1, 1, 1, 1 } } };
            mesh->indices = { 0, 1, 2 };
            mesh->feathered = feathered;
            DrawList list;
            list.add_indexed_mesh(mesh);
            return compile_draw_list(list).meshes.front().vertices.size();
        };
        RIGIDBODIES_EXPECT(make(true) == 3 && make(false) > 3, "only meshes without their own rim receive the boundary feather");
    }

    RIGIDBODIES_TEST("arrow tips land on their target and every head style has a distinct silhouette")
    {
        for (const auto head : { ArrowHead::filled, ArrowHead::open, ArrowHead::double_filled })
        {
            DrawList list;
            list.add_arrow({ 0, 0 }, { 100, 0 }, { 1, 1, 1, 1 }, 2.0f, 10.0f, head);
            const auto& shaft = list.commands().front();
            RIGIDBODIES_EXPECT(shaft.kind == DrawCommandKind::line && shaft.cap == StrokeCap::butt, "the shaft is a butt-capped line");
            RIGIDBODIES_EXPECT(list.vertices().at(shaft.vertex_offset) == Vec2(0, 0) && list.vertices().at(shaft.vertex_offset + 1).x < 100.0, "the shaft stops inside the head so the tip stays sharp");
            const auto fills = std::count_if(list.commands().begin(), list.commands().end(), [](const auto& command)
                {
                    return command.kind == DrawCommandKind::polygon_fill;
                });
            const auto& tip_command = list.commands().at(1);
            const auto tip = list.vertices().at(tip_command.vertex_offset + (head == ArrowHead::open ? 1 : 0));
            RIGIDBODIES_EXPECT(tip == Vec2(100, 0), "the head's point is exactly the target");
            RIGIDBODIES_EXPECT(fills == (head == ArrowHead::open ? 0 : head == ArrowHead::filled ? 1
                                                                                                 : 2),
                "filled, open and doubled heads differ in shape");
            const auto compiled = compile_draw_list(list);
            for (const auto& mesh : compiled.meshes)
                expect_valid(mesh);
        }
        DrawList shadow;
        shadow.add_circle_shadow({ 50, 50 }, 10.0f, { 0, 0, 0, 0.5f }, 6.0f, { 0, 2 });
        RIGIDBODIES_EXPECT(shadow.commands().size() == 1 && shadow.commands().front().kind == DrawCommandKind::shadow, "circle shadows reuse the soft polygon shadow");
        const auto soft = compile_draw_list(shadow);
        RIGIDBODIES_EXPECT_NEAR(bounds(soft.meshes.front()).maximum.y, 68.0, 1.0, "the blur band surrounds the offset disc");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
