#include <rigidbodies/physics/shape_authoring.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    Outline polygon(std::initializer_list<math::Vec2> points)
    {
        Outline result;
        result.closed = true;
        for (const auto& point : points)
        {
            OutlineNode node;
            node.position_m = point;
            result.nodes.push_back(node);
        }
        return result;
    }

    Outline square()
    {
        return polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 1.0 }, { 0.0, 1.0 } });
    }

    Outline dome()
    {
        auto result = square();
        result.nodes[2].outgoing_edge = OutlineEdgeKind::cubic;
        result.nodes[2].outgoing_handle_m = { 0.0, 1.0 };
        result.nodes[3].incoming_handle_m = { 0.0, 1.0 };
        return result;
    }

    bool has_code(const ShapeBuildResult& result, OutlineDiagnosticCode code)
    {
        return std::any_of(result.diagnostics.begin(), result.diagnostics.end(), [&](const auto& item)
            {
                return item.code == code;
            });
    }

    math::Vec2 evaluate(const OutlineNode& first, const OutlineNode& second, Real t)
    {
        const auto s = 1.0 - t;
        return first.position_m * (s * s * s) + (first.position_m + first.outgoing_handle_m) * (3.0 * s * s * t) +
            (second.position_m + second.incoming_handle_m) * (3.0 * s * t * t) + second.position_m * (t * t * t);
    }

    Real distance_to_outline(const math::Vec2& point, const std::vector<math::Vec2>& outline)
    {
        Real best = std::numeric_limits<Real>::infinity();
        for (std::size_t i = 0; i < outline.size(); ++i)
        {
            const auto a = outline[i], edge = outline[(i + 1) % outline.size()] - a;
            const auto t = math::clamp(math::dot(point - a, edge) / math::length_squared(edge), 0.0, 1.0);
            best = std::min(best, math::distance(point, a + edge * t));
        }
        return best;
    }

    Real triangle_area(const std::array<math::Vec2, 3>& triangle)
    {
        return math::cross(triangle[1] - triangle[0], triangle[2] - triangle[0]) * 0.5;
    }

    RIGIDBODIES_TEST("clockwise polygons and redundant closing vertices normalize into one convex shape")
    {
        auto source = polygon({ { 0.0, 0.0 }, { 0.0, 1.0 }, { 2.0, 1.0 }, { 2.0, 0.0 }, { 2.0, 0.0 }, { 0.0, 0.0 } });
        const auto result = build_authored_shape(source);
        RIGIDBODIES_EXPECT(result.succeeded(), "duplicate cleanup accepts an otherwise valid clockwise polygon");
        RIGIDBODIES_EXPECT(result.shape->render_outline.size() == 4 && result.shape->collision_outline.size() == 4, "adjacent and closing duplicates are removed");
        RIGIDBODIES_EXPECT_NEAR(math::signed_area(result.shape->collision_outline), 2.0, 1.0e-12, "collision winding becomes CCW with unchanged area");
        RIGIDBODIES_EXPECT(result.shape->convex_parts.size() == 1, "convex triangle neighbors merge back into one collider");
        RIGIDBODIES_EXPECT(result.shape->source.nodes.size() == 6, "source edit history geometry remains available without destructive normalization");
    }

    RIGIDBODIES_TEST("concave triangulation preserves area and leaves the notch unfilled")
    {
        const auto source = polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 2.0 }, { 0.0, 2.0 } });
        ShapeAuthoringOptions options;
        options.simplification_tolerance_m = 0.0;
        const auto result = build_authored_shape(source, options);
        RIGIDBODIES_EXPECT(result.succeeded() && result.shape->convex_parts.size() > 1, "L-shaped profile requires multiple convex pieces");
        Real collision_area = 0.0, render_area = 0.0;
        for (const auto& part : result.shape->convex_parts)
        {
            RIGIDBODIES_EXPECT(math::is_convex(part), "every emitted collider polygon is convex");
            collision_area += math::signed_area(part);
        }
        for (const auto& triangle : result.shape->render_triangles)
        {
            RIGIDBODIES_EXPECT(triangle_area(triangle) > 0.0, "render triangles are CCW");
            render_area += triangle_area(triangle);
        }
        RIGIDBODIES_EXPECT_NEAR(collision_area, 3.0, 1.0e-12, "convex parts tile the concave profile without extra area");
        RIGIDBODIES_EXPECT_NEAR(render_area, 3.0, 1.0e-12, "render fill uses concave triangles rather than a convex hull");
        RIGIDBODIES_EXPECT_NEAR(result.shape->approximation_added_area_m2, 0.0, 0.0, "zero concavity tolerance leaves the exact decomposition");
    }

    RIGIDBODIES_TEST("concavity tolerance trades bounded notch filling for fewer convex parts")
    {
        const auto source = polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 2.0 }, { 0.0, 2.0 } });
        ShapeAuthoringOptions exact;
        exact.simplification_tolerance_m = 0.0;
        auto approximate = exact;
        approximate.concavity_tolerance_m = 0.8;
        const auto fine = build_authored_shape(source, exact), coarse = build_authored_shape(source, approximate);
        RIGIDBODIES_EXPECT(fine.succeeded() && coarse.succeeded(), "both fidelity choices produce valid shapes");
        RIGIDBODIES_EXPECT(coarse.shape->convex_parts.size() < fine.shape->convex_parts.size(), "positive concavity tolerance reduces part count");
        RIGIDBODIES_EXPECT_NEAR(coarse.shape->approximation_added_area_m2, 0.5, 1.0e-12, "filled triangular notch area is reported");
        RIGIDBODIES_EXPECT(coarse.shape->render_outline == fine.shape->render_outline, "collision approximation preserves the rendered source outline");
        for (const auto& node : source.nodes)
            RIGIDBODIES_EXPECT(distance_to_outline(node.position_m, coarse.shape->collision_outline) <= approximate.concavity_tolerance_m, "every replaced source vertex stays within the chosen geometric error");
    }

    RIGIDBODIES_TEST("render and collision cubic tessellations obey independent flatness bounds")
    {
        const auto source = dome();
        ShapeAuthoringOptions options;
        options.render_tolerance_m = 0.0005;
        options.collision_tolerance_m = 0.03;
        options.simplification_tolerance_m = 0.0;
        const auto result = build_authored_shape(source, options);
        RIGIDBODIES_EXPECT(result.succeeded(), "cubic domed profile builds");
        RIGIDBODIES_EXPECT(result.shape->render_outline.size() > result.shape->collision_outline.size(), "fine render tolerance creates more segments than collision tolerance");
        for (int i = 0; i <= 1000; ++i)
        {
            const auto point = evaluate(source.nodes[2], source.nodes[3], static_cast<Real>(i) / 1000.0);
            RIGIDBODIES_EXPECT(distance_to_outline(point, result.shape->render_outline) <= options.render_tolerance_m + 1.0e-12, "sampled true curve stays inside render chord error bound");
            RIGIDBODIES_EXPECT(distance_to_outline(point, result.shape->collision_outline) <= options.collision_tolerance_m + 1.0e-12, "sampled true curve stays inside collision chord error bound");
        }
    }

    RIGIDBODIES_TEST("vertex and subdivision budgets reject unfinished curves without publishing partial shapes")
    {
        ShapeAuthoringOptions options;
        options.max_render_vertices = 8;
        options.render_tolerance_m = 1.0e-5;
        auto result = build_authored_shape(dome(), options);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::vertex_budget), "hard vertex budget has a specific actionable diagnostic");
        options.max_render_vertices = 512;
        options.max_subdivision_depth = 1;
        result = build_authored_shape(dome(), options);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::subdivision_budget), "depth exhaustion never silently degrades curve tolerance");
        options.max_subdivision_depth = 16;
        options.render_tolerance_m = 0.01;
        options.collision_tolerance_m = 1.0e-5;
        options.max_collision_vertices = 8;
        result = build_authored_shape(dome(), options);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::vertex_budget), "collision budget is enforced separately from rendering");
    }

    RIGIDBODIES_TEST("outline validation diagnoses crossing touching retracing and insufficient area")
    {
        const auto crossed = build_authored_shape(polygon({ { 0.0, 0.0 }, { 2.0, 2.0 }, { 0.0, 2.0 }, { 2.0, 0.0 } }));
        RIGIDBODIES_EXPECT(!crossed.succeeded() && has_code(crossed, OutlineDiagnosticCode::self_intersection), "bow-tie crossing is reported before signed-area cancellation");
        const auto touched = build_authored_shape(polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 2.0 }, { 1.0, 1.0 }, { 0.0, 2.0 }, { 1.0, 1.0 } }));
        RIGIDBODIES_EXPECT(!touched.succeeded() && has_code(touched, OutlineDiagnosticCode::self_intersection), "non-neighbor touching does not create a pinched collider");
        const auto retraced = build_authored_shape(polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 1.0 }, { 0.0, 1.0 } }));
        RIGIDBODIES_EXPECT(!retraced.succeeded() && has_code(retraced, OutlineDiagnosticCode::degenerate_edge), "adjacent retracing edges are rejected");
        const auto tiny = build_authored_shape(polygon({ { 0.0, 0.0 }, { 0.0001, 0.0 }, { 0.0001, 0.0001 } }));
        RIGIDBODIES_EXPECT(!tiny.succeeded() && has_code(tiny, OutlineDiagnosticCode::insufficient_area), "tiny nondegenerate polygon reports area threshold");
    }

    RIGIDBODIES_TEST("cubic overshoots and loops are checked as curves rather than only anchor polygons")
    {
        auto source = square();
        source.nodes[0].outgoing_edge = OutlineEdgeKind::cubic;
        source.nodes[0].outgoing_handle_m = { 0.0, 3.0 };
        source.nodes[1].incoming_handle_m = { 0.0, 3.0 };
        auto result = build_authored_shape(source);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::self_intersection), "arched bottom edge crosses the top edge despite simple anchors");
        source.nodes[0].outgoing_handle_m = { 4.0, 0.0 };
        source.nodes[1].incoming_handle_m = { -4.0, 0.0 };
        result = build_authored_shape(source);
        RIGIDBODIES_EXPECT(!result.succeeded(), "collinear overshooting cubic cannot be accepted as a single straight edge");
    }

    RIGIDBODIES_TEST("collision simplification preserves topology and the total original-chain error bound")
    {
        const auto source = polygon({ { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 1.0 }, { 1.8, 1.036 }, { 1.6, 1.064 }, { 1.4, 1.084 }, { 1.2, 1.096 }, { 1.0, 1.1 }, { 0.8, 1.096 }, { 0.6, 1.084 }, { 0.4, 1.064 }, { 0.2, 1.036 }, { 0.0, 1.0 } });
        ShapeAuthoringOptions options;
        options.simplification_tolerance_m = 0.025;
        const auto result = build_authored_shape(source, options);
        RIGIDBODIES_EXPECT(result.succeeded(), "simplified bowed outline remains valid");
        RIGIDBODIES_EXPECT(result.shape->collision_outline.size() < result.shape->render_outline.size(), "only collision representation loses insignificant vertices");
        for (const auto& node : source.nodes)
            RIGIDBODIES_EXPECT(distance_to_outline(node.position_m, result.shape->collision_outline) <= options.simplification_tolerance_m + 1.0e-12, "repeated simplification cannot accumulate beyond the original boundary tolerance");
        RIGIDBODIES_EXPECT(distance_to_outline({ 1.0, 1.1 }, result.shape->collision_outline) < 0.026, "many short segments cannot flatten a significant bow");
    }

    RIGIDBODIES_TEST("moving a node carries tangent offsets while mirrored and aligned joins retain their contracts")
    {
        auto source = dome();
        source.nodes[2].incoming_handle_m = { 0.0, -2.0 };
        RIGIDBODIES_EXPECT(set_outline_continuity(source, 2, OutlineContinuity::aligned), "join becomes smooth");
        RIGIDBODIES_EXPECT(move_outline_handle(source, 2, false, { 0.5, 0.0 }), "outgoing tangent is edited");
        RIGIDBODIES_EXPECT_NEAR(math::length(source.nodes[2].incoming_handle_m), 2.0, 1.0e-12, "aligned join preserves the opposite handle length");
        RIGIDBODIES_EXPECT_NEAR(math::cross(source.nodes[2].incoming_handle_m, source.nodes[2].outgoing_handle_m), 0.0, 1.0e-12, "aligned handles remain collinear");
        RIGIDBODIES_EXPECT(math::dot(source.nodes[2].incoming_handle_m, source.nodes[2].outgoing_handle_m) < 0.0, "tangents point in opposite directions around the node");
        set_outline_continuity(source, 2, OutlineContinuity::mirrored);
        move_outline_handle(source, 2, true, { -0.2, 0.3 });
        RIGIDBODIES_EXPECT(source.nodes[2].incoming_handle_m == -source.nodes[2].outgoing_handle_m, "mirrored join matches both tangent direction and length");
        const auto before = source.nodes[2].outgoing_handle_m;
        move_outline_node(source, 2, { 3.0, 2.0 });
        RIGIDBODIES_EXPECT(source.nodes[2].outgoing_handle_m == before, "translation moves the anchor without changing handle offsets");
        set_outline_continuity(source, 2, OutlineContinuity::corner);
        move_outline_handle(source, 2, true, { 1.0, 1.0 });
        RIGIDBODIES_EXPECT(source.nodes[2].outgoing_handle_m == before, "corner handles edit independently");
    }

    RIGIDBODIES_TEST("cubic insertion preserves the exact original curve on both parameter intervals")
    {
        auto source = dome();
        source.nodes[2].continuity = OutlineContinuity::mirrored;
        const auto first = source.nodes[2], second = source.nodes[3];
        const auto inserted = insert_outline_node(source, 2, 0.3);
        RIGIDBODIES_EXPECT(inserted == 3 && source.nodes.size() == 5, "split inserts between the selected edge endpoints");
        RIGIDBODIES_EXPECT(source.nodes[2].continuity == OutlineContinuity::aligned, "exact split preserves tangent direction while relaxing mirrored length at endpoint");
        for (int i = 0; i <= 100; ++i)
        {
            const auto t = static_cast<Real>(i) / 100.0;
            const auto expected = evaluate(first, second, t);
            const auto actual = t <= 0.3 ? evaluate(source.nodes[2], source.nodes[3], t / 0.3) : evaluate(source.nodes[3], source.nodes[4], (t - 0.3) / 0.7);
            RIGIDBODIES_EXPECT_NEAR(math::distance(actual, expected), 0.0, 1.0e-12, "de Casteljau split reproduces every sampled original point");
        }
    }

    RIGIDBODIES_TEST("line edge conversion insertion and removal support closed outline editing")
    {
        auto source = square();
        set_outline_edge_kind(source, 3, OutlineEdgeKind::cubic);
        RIGIDBODIES_EXPECT_NEAR(math::distance(evaluate(source.nodes[3], source.nodes[0], 0.5), { 0.0, 0.5 }), 0.0, 1.0e-12, "line-to-cubic conversion starts as the same straight geometry");
        const auto inserted = insert_outline_node(source, 3);
        RIGIDBODIES_EXPECT(inserted == 4 && source.nodes[4].outgoing_edge == OutlineEdgeKind::cubic, "closing cubic can split across the first-node wrap");
        RIGIDBODIES_EXPECT(build_authored_shape(source).succeeded(), "closed outline remains valid after insertion");
        RIGIDBODIES_EXPECT(remove_outline_node(source, inserted), "inserted node can be removed");
        set_outline_edge_kind(source, 3, OutlineEdgeKind::line);
        RIGIDBODIES_EXPECT(build_authored_shape(source).succeeded(), "curve can return to a line after closed editing");
        source.closed = false;
        RIGIDBODIES_EXPECT(!set_outline_edge_kind(source, source.nodes.size() - 1, OutlineEdgeKind::cubic), "open outline has no closing edge to convert");
    }

    RIGIDBODIES_TEST("cache reuses immutable geometry and invalidates for source settings and failed previews")
    {
        ShapeAuthoringCache cache;
        auto source = dome();
        const auto original = cache.build(source);
        const auto reused = cache.build(source);
        RIGIDBODIES_EXPECT(original.succeeded() && original.shape == reused.shape && cache.build_count() == 1, "identical previews reuse the same immutable decomposition");
        move_outline_node(source, 1, { 2.1, 0.0 });
        const auto edited = cache.build(source);
        RIGIDBODIES_EXPECT(edited.succeeded() && edited.shape != original.shape && cache.build_count() == 2, "node edit invalidates geometry cache");
        RIGIDBODIES_EXPECT(original.shape->source.nodes[1].position_m == math::Vec2 { 2.0, 0.0 }, "earlier committed immutable shape is unaffected by an editor mutation");
        ShapeAuthoringOptions options;
        options.render_tolerance_m = 0.001;
        const auto changed = cache.build(source, options);
        RIGIDBODIES_EXPECT(changed.succeeded() && changed.shape != edited.shape && cache.build_count() == 3, "tolerance edit invalidates cache");
        source.closed = false;
        const auto invalid = cache.build(source, options);
        const auto invalid_again = cache.build(source, options);
        RIGIDBODIES_EXPECT(!invalid.succeeded() && !invalid_again.succeeded() && cache.build_count() == 4, "unchanged invalid drafts cache their diagnostics too");
        RIGIDBODIES_EXPECT(has_code(invalid, OutlineDiagnosticCode::open_outline), "invalid draft does not reuse the last valid body's geometry");
        cache.clear();
        (void)cache.build(source, options);
        RIGIDBODIES_EXPECT(cache.build_count() == 5, "explicit clear forces a later rebuild");
    }

    RIGIDBODIES_TEST("nonfinite inputs invalid options and nonfinite edits fail without corrupting geometry")
    {
        auto source = square();
        source.nodes[0].incoming_handle_m.x = std::numeric_limits<Real>::quiet_NaN();
        auto result = build_authored_shape(source);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::non_finite), "all handles are validated even on line edges");
        ShapeAuthoringOptions options;
        options.max_collision_vertices = 2;
        result = build_authored_shape(square(), options);
        RIGIDBODIES_EXPECT(!result.succeeded() && has_code(result, OutlineDiagnosticCode::invalid_options), "impossible vertex budget is rejected");
        source = square();
        bool rejected = false;
        try
        {
            move_outline_node(source, 0, { std::numeric_limits<Real>::infinity(), 0.0 });
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && source.nodes[0].position_m == math::Vec2 {}, "nonfinite coordinate edit is atomic");
        rejected = false;
        try
        {
            (void)insert_outline_node(source, 0, 1.0);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && source.nodes.size() == 4, "endpoint insertion cannot create a zero-length edge");
    }

    RIGIDBODIES_TEST("deterministic star decompositions cover area across many simple concave profiles")
    {
        for (int sides = 3; sides <= 12; ++sides)
        {
            Outline source;
            source.closed = true;
            for (int i = 0; i < sides * 2; ++i)
            {
                const auto angle = static_cast<Real>(i) * math::pi / static_cast<Real>(sides);
                const auto radius = i % 2 == 0 ? 1.0 : 0.45;
                OutlineNode node;
                node.position_m = { radius * std::cos(angle), radius * std::sin(angle) };
                source.nodes.push_back(node);
            }
            ShapeAuthoringOptions options;
            options.simplification_tolerance_m = 0.0;
            const auto first = build_authored_shape(source, options), second = build_authored_shape(source, options);
            RIGIDBODIES_EXPECT(first.succeeded() && second.succeeded(), "valid radial concave outline decomposes");
            RIGIDBODIES_EXPECT(first.shape->convex_parts == second.shape->convex_parts, "part geometry and order are deterministic across independent builds");
            Real area = 0.0;
            for (const auto& part : first.shape->convex_parts)
            {
                RIGIDBODIES_EXPECT(math::is_convex(part), "star decomposition emits only convex pieces");
                area += math::signed_area(part);
            }
            RIGIDBODIES_EXPECT_NEAR(area, math::signed_area(first.shape->collision_outline), 1.0e-10, "decomposition preserves every star's area without overlaps or missing triangles");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
