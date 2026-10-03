#include <rigidbodies/physics/shape_authoring.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {
        using Polygon = std::vector<math::Vec2>;
        constexpr Real predicate_epsilon = 1.0e-12;

        Real magnitude(const math::Vec2& value)
        {
            return std::hypot(value.x, value.y);
        }

        Real polygon_area(const Polygon& polygon)
        {
            Real result = 0.0;
            if (polygon.size() < 3)
                return result;
            for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
                result += math::cross(polygon[i] - polygon[0], polygon[i + 1] - polygon[0]);
            return result * 0.5;
        }

        Real segment_distance(const math::Vec2& point, const math::Vec2& a, const math::Vec2& b)
        {
            const auto axis = b - a;
            const auto length = magnitude(axis);
            if (length == 0.0)
                return magnitude(point - a);
            const auto direction = axis / length;
            const auto projection = math::clamp(math::dot(point - a, direction), 0.0, length);
            return magnitude(point - (a + direction * projection));
        }

        void diagnostic(std::vector<OutlineDiagnostic>& output, OutlineDiagnosticCode code, std::string message, std::size_t edge = 0)
        {
            output.push_back({ code, std::move(message), edge });
        }

        bool options_valid(const ShapeAuthoringOptions& options)
        {
            const auto nonnegative = [](Real value)
            {
                return math::is_finite(value) && value >= 0.0;
            };
            return nonnegative(options.render_tolerance_m) && options.render_tolerance_m > 0.0 &&
                nonnegative(options.collision_tolerance_m) && options.collision_tolerance_m > 0.0 &&
                nonnegative(options.simplification_tolerance_m) && nonnegative(options.concavity_tolerance_m) &&
                options.max_render_vertices >= 3 && options.max_render_vertices <= 4096 &&
                options.max_collision_vertices >= 3 && options.max_collision_vertices <= 512 &&
                options.max_subdivision_depth > 0 && options.max_subdivision_depth <= 24 &&
                nonnegative(options.minimum_area_m2) && options.minimum_area_m2 >= math::geometric_epsilon &&
                nonnegative(options.duplicate_tolerance_m) && options.duplicate_tolerance_m > 0.0 &&
                math::is_finite(options.collision_tolerance_m + options.simplification_tolerance_m + options.concavity_tolerance_m + options.duplicate_tolerance_m);
        }

        bool node_finite(const OutlineNode& node)
        {
            return math::is_finite(node.position_m) && math::is_finite(node.incoming_handle_m) && math::is_finite(node.outgoing_handle_m) &&
                math::is_finite(node.position_m + node.incoming_handle_m) && math::is_finite(node.position_m + node.outgoing_handle_m);
        }

        struct Cubic
        {
            math::Vec2 a, b, c, d;
        };

        std::pair<Cubic, Cubic> split_cubic(const Cubic& curve, Real t)
        {
            const auto ab = math::lerp(curve.a, curve.b, t), bc = math::lerp(curve.b, curve.c, t), cd = math::lerp(curve.c, curve.d, t);
            const auto abc = math::lerp(ab, bc, t), bcd = math::lerp(bc, cd, t), middle = math::lerp(abc, bcd, t);
            return { { curve.a, ab, abc, middle }, { middle, bcd, cd, curve.d } };
        }

        bool append_vertex(Polygon& vertices, const math::Vec2& point, std::size_t budget, std::vector<OutlineDiagnostic>& issues, std::size_t edge)
        {
            if (!math::is_finite(point))
            {
                diagnostic(issues, OutlineDiagnosticCode::non_finite, "Curve subdivision produced an unrepresentable point.", edge);
                return false;
            }
            // The final repeated closing point is removed before the result is published.
            if (vertices.size() >= budget + 1)
            {
                diagnostic(issues, OutlineDiagnosticCode::vertex_budget, "Outline exceeds the vertex budget; increase the tolerance or simplify its curves.", edge);
                return false;
            }
            vertices.push_back(point);
            return true;
        }

        bool tessellate_cubic(const Cubic& curve, Real tolerance, unsigned depth, const ShapeAuthoringOptions& options,
            std::size_t budget, Polygon& vertices, std::vector<OutlineDiagnostic>& issues, std::size_t edge)
        {
            const auto flatness = std::max(segment_distance(curve.b, curve.a, curve.d), segment_distance(curve.c, curve.a, curve.d));
            if (!math::is_finite(flatness))
            {
                diagnostic(issues, OutlineDiagnosticCode::non_finite, "Curve geometry cannot be represented at this scale.", edge);
                return false;
            }
            if (flatness <= tolerance)
                return append_vertex(vertices, curve.d, budget, issues, edge);
            if (depth >= options.max_subdivision_depth)
            {
                diagnostic(issues, OutlineDiagnosticCode::subdivision_budget, "The curve needs more subdivisions to meet this tolerance. Increase the subdivision limit or use a larger tolerance.", edge);
                return false;
            }
            const auto halves = split_cubic(curve, 0.5);
            return tessellate_cubic(halves.first, tolerance, depth + 1, options, budget, vertices, issues, edge) &&
                tessellate_cubic(halves.second, tolerance, depth + 1, options, budget, vertices, issues, edge);
        }

        Polygon clean_polygon(const Polygon& source, Real tolerance)
        {
            Polygon result;
            for (const auto& point : source)
                if (result.empty() || magnitude(point - result.back()) > tolerance)
                    result.push_back(point);
            if (result.size() > 1 && magnitude(result.front() - result.back()) <= tolerance)
                result.pop_back();
            return result;
        }

        bool tessellate(const Outline& outline, const ShapeAuthoringOptions& options, Real tolerance, std::size_t budget,
            Polygon& result, std::vector<OutlineDiagnostic>& issues)
        {
            result.push_back(outline.nodes.front().position_m);
            for (std::size_t i = 0; i < outline.nodes.size(); ++i)
            {
                const auto& first = outline.nodes[i];
                const auto& second = outline.nodes[(i + 1) % outline.nodes.size()];
                if (first.outgoing_edge == OutlineEdgeKind::line)
                {
                    if (!append_vertex(result, second.position_m, budget, issues, i))
                        return false;
                }
                else
                {
                    const Cubic curve { first.position_m, first.position_m + first.outgoing_handle_m, second.position_m + second.incoming_handle_m, second.position_m };
                    if (!tessellate_cubic(curve, tolerance, 0, options, budget, result, issues, i))
                        return false;
                }
            }
            result = clean_polygon(result, options.duplicate_tolerance_m);
            return true;
        }

        bool segments_touch(const math::Vec2& a, const math::Vec2& b, const math::Vec2& c, const math::Vec2& d, Real tolerance)
        {
            if (std::max(a.x, b.x) + tolerance < std::min(c.x, d.x) || std::max(c.x, d.x) + tolerance < std::min(a.x, b.x) ||
                std::max(a.y, b.y) + tolerance < std::min(c.y, d.y) || std::max(c.y, d.y) + tolerance < std::min(a.y, b.y))
                return false;
            if (segment_distance(a, c, d) <= tolerance || segment_distance(b, c, d) <= tolerance ||
                segment_distance(c, a, b) <= tolerance || segment_distance(d, a, b) <= tolerance)
                return true;
            const auto ab = b - a, cd = d - c;
            const auto first = math::cross(ab, c - a), second = math::cross(ab, d - a);
            const auto third = math::cross(cd, a - c), fourth = math::cross(cd, b - c);
            return ((first > 0.0 && second < 0.0) || (first < 0.0 && second > 0.0)) &&
                ((third > 0.0 && fourth < 0.0) || (third < 0.0 && fourth > 0.0));
        }

        bool validate_polygon(const Polygon& polygon, const ShapeAuthoringOptions& options, std::vector<OutlineDiagnostic>& issues)
        {
            if (polygon.size() < 3)
            {
                diagnostic(issues, OutlineDiagnosticCode::insufficient_vertices, "A closed outline needs at least three distinct tessellated vertices.");
                return false;
            }
            for (std::size_t i = 0; i < polygon.size(); ++i)
            {
                const auto& a = polygon[i];
                const auto& b = polygon[(i + 1) % polygon.size()];
                const auto& c = polygon[(i + 2) % polygon.size()];
                if (!math::is_finite(a) || !math::is_finite(b - a) || !math::is_finite(math::cross(b - a, c - b)))
                {
                    diagnostic(issues, OutlineDiagnosticCode::non_finite, "Outline geometry exceeds the supported numeric range.", i);
                    return false;
                }
                if (magnitude(b - a) <= options.duplicate_tolerance_m ||
                    (segment_distance(c, a, b) <= options.duplicate_tolerance_m && math::dot(b - a, c - b) < 0.0))
                {
                    diagnostic(issues, OutlineDiagnosticCode::degenerate_edge, "Outline contains a zero-length or retracing edge.", i);
                    return false;
                }
                for (std::size_t j = i + 1; j < polygon.size(); ++j)
                {
                    if (j == i + 1 || (i == 0 && j + 1 == polygon.size()))
                        continue;
                    if (segments_touch(a, b, polygon[j], polygon[(j + 1) % polygon.size()], options.duplicate_tolerance_m))
                    {
                        diagnostic(issues, OutlineDiagnosticCode::self_intersection, "Outline edges cross or touch somewhere other than a shared endpoint.", i);
                        return false;
                    }
                }
            }
            const auto area = polygon_area(polygon);
            if (!math::is_finite(area))
            {
                diagnostic(issues, OutlineDiagnosticCode::non_finite, "Outline area cannot be represented at this scale.");
                return false;
            }
            if (std::abs(area) < options.minimum_area_m2)
            {
                diagnostic(issues, OutlineDiagnosticCode::insufficient_area, "Outline encloses less than the minimum supported area.");
                return false;
            }
            return true;
        }

        bool point_in_triangle(const math::Vec2& point, const math::Vec2& a, const math::Vec2& b, const math::Vec2& c)
        {
            return math::cross(b - a, point - a) >= -predicate_epsilon &&
                math::cross(c - b, point - b) >= -predicate_epsilon && math::cross(a - c, point - c) >= -predicate_epsilon;
        }

        Polygon remove_collinear(Polygon points)
        {
            bool changed = true;
            while (changed && points.size() > 3)
            {
                changed = false;
                for (std::size_t i = 0; i < points.size(); ++i)
                {
                    const auto& a = points[(i + points.size() - 1) % points.size()];
                    const auto& b = points[i];
                    const auto& c = points[(i + 1) % points.size()];
                    if (std::abs(math::cross(b - a, c - b)) <= predicate_epsilon && math::dot(b - a, c - b) >= 0.0)
                    {
                        points.erase(points.begin() + static_cast<std::ptrdiff_t>(i));
                        changed = true;
                        break;
                    }
                }
            }
            return points;
        }

        bool triangulate(const Polygon& polygon, std::vector<std::array<math::Vec2, 3>>& triangles)
        {
            auto work = remove_collinear(polygon);
            while (work.size() > 3)
            {
                bool found = false;
                for (std::size_t i = 0; i < work.size(); ++i)
                {
                    const auto previous = (i + work.size() - 1) % work.size(), next = (i + 1) % work.size();
                    const auto& a = work[previous];
                    const auto& b = work[i];
                    const auto& c = work[next];
                    if (math::cross(b - a, c - b) <= predicate_epsilon)
                        continue;
                    bool occupied = false;
                    for (std::size_t j = 0; j < work.size(); ++j)
                        if (j != previous && j != i && j != next && point_in_triangle(work[j], a, b, c))
                        {
                            occupied = true;
                            break;
                        }
                    if (!occupied)
                    {
                        triangles.push_back({ a, b, c });
                        work.erase(work.begin() + static_cast<std::ptrdiff_t>(i));
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return false;
            }
            if (work.size() != 3 || math::cross(work[1] - work[0], work[2] - work[0]) <= predicate_epsilon)
                return false;
            triangles.push_back({ work[0], work[1], work[2] });
            return true;
        }

        Polygon convex_hull(Polygon points)
        {
            std::sort(points.begin(), points.end(), [](const auto& a, const auto& b)
                {
                    return a.x < b.x || (a.x == b.x && a.y < b.y);
                });
            points.erase(std::unique(points.begin(), points.end()), points.end());
            if (points.size() < 3)
                return {};
            Polygon hull;
            for (const auto& point : points)
            {
                while (hull.size() >= 2 && math::cross(hull.back() - hull[hull.size() - 2], point - hull.back()) <= predicate_epsilon)
                    hull.pop_back();
                hull.push_back(point);
            }
            const auto lower = hull.size();
            for (std::size_t i = points.size() - 1; i > 0; --i)
            {
                const auto& point = points[i - 1];
                while (hull.size() > lower && math::cross(hull.back() - hull[hull.size() - 2], point - hull.back()) <= predicate_epsilon)
                    hull.pop_back();
                hull.push_back(point);
            }
            hull.pop_back();
            return hull;
        }

        bool share_edge(const Polygon& first, const Polygon& second)
        {
            for (std::size_t i = 0; i < first.size(); ++i)
                for (std::size_t j = 0; j < second.size(); ++j)
                    if (first[i] == second[(j + 1) % second.size()] && first[(i + 1) % first.size()] == second[j])
                        return true;
            return false;
        }

        bool decompose(const Polygon& polygon, std::vector<Polygon>& parts)
        {
            std::vector<std::array<math::Vec2, 3>> triangles;
            if (!triangulate(polygon, triangles))
                return false;
            for (const auto& triangle : triangles)
                parts.emplace_back(triangle.begin(), triangle.end());
            bool changed = true;
            while (changed)
            {
                changed = false;
                for (std::size_t i = 0; i < parts.size() && !changed; ++i)
                    for (std::size_t j = i + 1; j < parts.size(); ++j)
                    {
                        if (!share_edge(parts[i], parts[j]))
                            continue;
                        auto points = parts[i];
                        points.insert(points.end(), parts[j].begin(), parts[j].end());
                        auto hull = convex_hull(std::move(points));
                        const auto total_area = polygon_area(parts[i]) + polygon_area(parts[j]);
                        if (hull.size() >= 3 && std::abs(polygon_area(hull) - total_area) <= predicate_epsilon * std::max(1.0, total_area))
                        {
                            parts[i] = std::move(hull);
                            parts.erase(parts.begin() + static_cast<std::ptrdiff_t>(j));
                            changed = true;
                            break;
                        }
                    }
            }
            for (const auto& part : parts)
                if (!math::is_convex(part))
                    return false;
            return true;
        }

        Polygon simplify(Polygon polygon, Real tolerance, bool only_reflex, const ShapeAuthoringOptions& options)
        {
            if (tolerance == 0.0)
                return polygon;
            // Each surviving edge retains its entire original replaced subchain. This prevents
            // many individually small removals from accumulating into a large unreported error.
            std::vector<Polygon> sources;
            sources.reserve(polygon.size());
            for (std::size_t i = 0; i < polygon.size(); ++i)
                sources.push_back({ polygon[i], polygon[(i + 1) % polygon.size()] });
            bool changed = true;
            while (changed && polygon.size() > 3)
            {
                changed = false;
                for (std::size_t i = 0; i < polygon.size(); ++i)
                {
                    const auto previous = (i + polygon.size() - 1) % polygon.size(), next = (i + 1) % polygon.size();
                    if (only_reflex && math::cross(polygon[i] - polygon[previous], polygon[next] - polygon[i]) >= -predicate_epsilon)
                        continue;
                    auto chain = sources[previous];
                    chain.insert(chain.end(), sources[i].begin() + 1, sources[i].end());
                    bool close = true;
                    for (const auto& point : chain)
                        close = close && segment_distance(point, polygon[previous], polygon[next]) <= tolerance;
                    if (!close)
                        continue;
                    auto candidate = polygon;
                    candidate.erase(candidate.begin() + static_cast<std::ptrdiff_t>(i));
                    std::vector<OutlineDiagnostic> ignored;
                    if (!validate_polygon(candidate, options, ignored) || polygon_area(candidate) <= 0.0)
                        continue;
                    sources[previous] = std::move(chain);
                    sources.erase(sources.begin() + static_cast<std::ptrdiff_t>(i));
                    polygon = std::move(candidate);
                    changed = true;
                    break;
                }
            }
            return polygon;
        }

        bool same_options(const ShapeAuthoringOptions& a, const ShapeAuthoringOptions& b)
        {
            return a.render_tolerance_m == b.render_tolerance_m && a.collision_tolerance_m == b.collision_tolerance_m &&
                a.simplification_tolerance_m == b.simplification_tolerance_m && a.concavity_tolerance_m == b.concavity_tolerance_m &&
                a.max_render_vertices == b.max_render_vertices && a.max_collision_vertices == b.max_collision_vertices &&
                a.max_subdivision_depth == b.max_subdivision_depth && a.minimum_area_m2 == b.minimum_area_m2 && a.duplicate_tolerance_m == b.duplicate_tolerance_m;
        }

        bool same_outline(const Outline& a, const Outline& b)
        {
            if (a.closed != b.closed || a.nodes.size() != b.nodes.size())
                return false;
            for (std::size_t i = 0; i < a.nodes.size(); ++i)
            {
                const auto& x = a.nodes[i];
                const auto& y = b.nodes[i];
                if (!(x.position_m == y.position_m) || !(x.incoming_handle_m == y.incoming_handle_m) || !(x.outgoing_handle_m == y.outgoing_handle_m) ||
                    x.outgoing_edge != y.outgoing_edge || x.continuity != y.continuity)
                    return false;
            }
            return true;
        }

        void require_finite(const math::Vec2& value)
        {
            if (!math::is_finite(value))
                throw std::invalid_argument("Outline edit coordinates must be finite");
        }

        bool edge_exists(const Outline& outline, std::size_t index)
        {
            return index < outline.nodes.size() && outline.nodes.size() >= 2 && (outline.closed || index + 1 < outline.nodes.size());
        }

        void apply_handle(OutlineNode& node, bool incoming, const math::Vec2& offset)
        {
            auto& moved = incoming ? node.incoming_handle_m : node.outgoing_handle_m;
            auto& opposite = incoming ? node.outgoing_handle_m : node.incoming_handle_m;
            const auto old_length = magnitude(opposite);
            moved = offset;
            if (node.continuity == OutlineContinuity::mirrored)
                opposite = -offset;
            else if (node.continuity == OutlineContinuity::aligned && magnitude(offset) > 0.0)
                opposite = -offset * ((old_length > 0.0 ? old_length : magnitude(offset)) / magnitude(offset));
        }
    }

    ShapeBuildResult build_authored_shape(const Outline& outline, const ShapeAuthoringOptions& options)
    {
        ShapeBuildResult result;
        if (!options_valid(options))
        {
            diagnostic(result.diagnostics, OutlineDiagnosticCode::invalid_options, "Check the tolerances, minimum area, vertex limit and subdivision limit. All must be finite and within their supported ranges.");
            return result;
        }
        if (!outline.closed)
        {
            diagnostic(result.diagnostics, OutlineDiagnosticCode::open_outline, "Close the outline before creating a body.");
            return result;
        }
        if (outline.nodes.size() < 2)
        {
            diagnostic(result.diagnostics, OutlineDiagnosticCode::insufficient_vertices, "Add enough distinct outline nodes to enclose an area.");
            return result;
        }
        for (std::size_t i = 0; i < outline.nodes.size(); ++i)
            if (!node_finite(outline.nodes[i]))
            {
                diagnostic(result.diagnostics, OutlineDiagnosticCode::non_finite, "Outline nodes and tangent handles must be finite and representable.", i);
                return result;
            }
        AuthoredShape shape;
        shape.source = outline;
        shape.options = options;
        if (!tessellate(outline, options, options.render_tolerance_m, options.max_render_vertices, shape.render_outline, result.diagnostics) ||
            !validate_polygon(shape.render_outline, options, result.diagnostics))
            return result;
        if (!tessellate(outline, options, options.collision_tolerance_m, options.max_collision_vertices, shape.collision_outline, result.diagnostics) ||
            !validate_polygon(shape.collision_outline, options, result.diagnostics))
            return result;
        if (polygon_area(shape.render_outline) < 0.0)
            std::reverse(shape.render_outline.begin(), shape.render_outline.end());
        if (polygon_area(shape.collision_outline) < 0.0)
            std::reverse(shape.collision_outline.begin(), shape.collision_outline.end());
        shape.collision_outline = simplify(std::move(shape.collision_outline), options.simplification_tolerance_m, false, options);
        const auto exact_area = polygon_area(shape.collision_outline);
        shape.collision_outline = simplify(std::move(shape.collision_outline), options.concavity_tolerance_m, true, options);
        shape.approximation_added_area_m2 = std::max(0.0, polygon_area(shape.collision_outline) - exact_area);
        shape.max_collision_error_m = options.collision_tolerance_m + options.simplification_tolerance_m + options.concavity_tolerance_m + options.duplicate_tolerance_m;
        if (!triangulate(shape.render_outline, shape.render_triangles) || !decompose(shape.collision_outline, shape.convex_parts))
        {
            diagnostic(result.diagnostics, OutlineDiagnosticCode::decomposition_failed, "The outline could not be split into convex collision shapes. Remove very small features and try again.");
            return result;
        }
        result.shape = std::make_shared<const AuthoredShape>(std::move(shape));
        return result;
    }

    ShapeBuildResult ShapeAuthoringCache::build(const Outline& outline, const ShapeAuthoringOptions& options)
    {
        if (!populated_ || !same_outline(outline, source_) || !same_options(options, options_))
        {
            auto result = build_authored_shape(outline, options);
            source_ = outline;
            options_ = options;
            result_ = std::move(result);
            populated_ = true;
            ++build_count_;
        }
        return result_;
    }

    void ShapeAuthoringCache::clear()
    {
        populated_ = false;
        result_ = {};
        source_ = {};
    }
    std::size_t ShapeAuthoringCache::build_count() const
    {
        return build_count_;
    }

    bool move_outline_node(Outline& outline, std::size_t index, const math::Vec2& position)
    {
        require_finite(position);
        if (index >= outline.nodes.size())
            return false;
        auto changed = outline.nodes[index];
        changed.position_m = position;
        if (!node_finite(changed))
            throw std::invalid_argument("Outline edit would make a handle unrepresentable");
        outline.nodes[index] = changed;
        return true;
    }

    std::size_t insert_outline_node(Outline& outline, std::size_t edge, Real parameter)
    {
        if (!math::is_finite(parameter) || parameter <= 0.0 || parameter >= 1.0)
            throw std::invalid_argument("Outline insertion parameter must lie strictly between zero and one");
        if (!edge_exists(outline, edge))
            return outline.nodes.size();
        const auto next = (edge + 1) % outline.nodes.size();
        auto first = outline.nodes[edge], second = outline.nodes[next];
        OutlineNode inserted;
        inserted.outgoing_edge = first.outgoing_edge;
        if (first.outgoing_edge == OutlineEdgeKind::cubic)
        {
            const auto parts = split_cubic({ first.position_m, first.position_m + first.outgoing_handle_m, second.position_m + second.incoming_handle_m, second.position_m }, parameter);
            inserted.position_m = parts.first.d;
            inserted.incoming_handle_m = parts.first.c - inserted.position_m;
            inserted.outgoing_handle_m = parts.second.b - inserted.position_m;
            inserted.continuity = OutlineContinuity::aligned;
            first.outgoing_handle_m = parts.first.b - first.position_m;
            second.incoming_handle_m = parts.second.c - second.position_m;
            if (first.continuity == OutlineContinuity::mirrored)
                first.continuity = OutlineContinuity::aligned;
            if (second.continuity == OutlineContinuity::mirrored)
                second.continuity = OutlineContinuity::aligned;
        }
        else
            inserted.position_m = math::lerp(first.position_m, second.position_m, parameter);
        if (!node_finite(first) || !node_finite(second) || !node_finite(inserted))
            throw std::invalid_argument("Outline insertion cannot be represented");
        outline.nodes[edge] = first;
        outline.nodes[next] = second;
        outline.nodes.insert(outline.nodes.begin() + static_cast<std::ptrdiff_t>(edge + 1), inserted);
        return edge + 1;
    }

    bool remove_outline_node(Outline& outline, std::size_t index)
    {
        if (index >= outline.nodes.size())
            return false;
        outline.nodes.erase(outline.nodes.begin() + static_cast<std::ptrdiff_t>(index));
        return true;
    }

    bool set_outline_edge_kind(Outline& outline, std::size_t index, OutlineEdgeKind kind)
    {
        if (!edge_exists(outline, index))
            return false;
        const auto next = (index + 1) % outline.nodes.size();
        auto first = outline.nodes[index], second = outline.nodes[next];
        if (first.outgoing_edge == kind)
            return true;
        first.outgoing_edge = kind;
        if (kind == OutlineEdgeKind::cubic)
        {
            const auto tangent = (second.position_m - first.position_m) / 3.0;
            apply_handle(first, false, tangent);
            apply_handle(second, true, -tangent);
        }
        if (!node_finite(first) || !node_finite(second))
            throw std::invalid_argument("Outline edge conversion cannot be represented");
        outline.nodes[index] = first;
        outline.nodes[next] = second;
        return true;
    }

    bool set_outline_continuity(Outline& outline, std::size_t index, OutlineContinuity continuity)
    {
        if (index >= outline.nodes.size())
            return false;
        auto changed = outline.nodes[index];
        changed.continuity = continuity;
        if (continuity != OutlineContinuity::corner)
        {
            auto direction = changed.outgoing_handle_m;
            if (magnitude(direction) == 0.0)
                direction = -changed.incoming_handle_m;
            if (magnitude(direction) == 0.0 && outline.nodes.size() > 1)
            {
                const auto previous = (index + outline.nodes.size() - 1) % outline.nodes.size(), next = (index + 1) % outline.nodes.size();
                direction = (outline.nodes[next].position_m - outline.nodes[previous].position_m) / 6.0;
            }
            apply_handle(changed, false, direction);
        }
        if (!node_finite(changed))
            throw std::invalid_argument("Outline join conversion cannot be represented");
        outline.nodes[index] = changed;
        return true;
    }

    bool move_outline_handle(Outline& outline, std::size_t index, bool incoming, const math::Vec2& offset)
    {
        require_finite(offset);
        if (index >= outline.nodes.size())
            return false;
        auto changed = outline.nodes[index];
        apply_handle(changed, incoming, offset);
        if (!node_finite(changed))
            throw std::invalid_argument("Outline tangent edit cannot be represented");
        outline.nodes[index] = changed;
        return true;
    }
}
