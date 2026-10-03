#include <rigidbodies/physics/narrow_phase.hpp>
#include <rigidbodies/physics/convex_distance.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace rigidbodies::physics
{
    namespace
    {
        using math::Vec2;
        constexpr Real tolerance = 1.0e-9;
        struct Polygon
        {
            std::vector<Vec2> vertices;
            std::vector<Vec2> normals;
        };
        struct Candidate
        {
            Vec2 first;
            Vec2 second;
            ContactFeatureId feature;
        };
        struct ClipVertex
        {
            Vec2 point;
            std::uint16_t incident_vertex;
            std::uint16_t reference_vertex;
        };

        Vec2 unit(const Vec2& vector, const Vec2& fallback = { 1.0, 0.0 })
        {
            const auto length = std::hypot(vector.x, vector.y);
            return length > 0.0 && math::is_finite(length) ? vector / length : fallback;
        }

        Polygon polygon(const Shape& shape, const math::Transform2& transform)
        {
            Polygon result;
            if (const auto* convex = dynamic_cast<const ConvexPolygonShape*>(&shape))
            {
                if (convex->vertices().size() > 65534)
                    return result;
                // Existing authoring accepts repeated consecutive vertices. Their zero-length
                // edges have no face normal; support mapping still represents the convex hull.
                for (const auto& normal : convex->normals())
                    if (math::length_squared(normal) < 0.5)
                        return result;
                for (const auto& vertex : convex->vertices())
                    result.vertices.push_back(math::transform_point(transform, vertex));
                for (const auto& normal : convex->normals())
                    result.normals.push_back(math::transform_direction(transform, normal));
            }
            else if (const auto* segment = dynamic_cast<const SegmentShape*>(&shape))
            {
                result.vertices = { math::transform_point(transform, segment->start_m()), math::transform_point(transform, segment->end_m()) };
                const auto edge = result.vertices[1] - result.vertices[0];
                const auto normal = unit({ edge.y, -edge.x });
                result.normals = { normal, -normal };
            }
            return result;
        }

        Vec2 closest_on_edge(const Vec2& point, const Vec2& start, const Vec2& end, Real& fraction)
        {
            const auto edge = end - start;
            const auto length_squared = math::length_squared(edge);
            fraction = length_squared > 0.0 ? math::clamp(math::dot(point - start, edge) / length_squared, 0.0, 1.0) : 0.0;
            return start + edge * fraction;
        }

        void add(ContactManifold& manifold, const NarrowPhaseQuery& query, const Candidate& candidate)
        {
            const auto separation = math::dot(candidate.second - candidate.first, manifold.normal);
            if (!math::is_finite(separation) || separation > query.contact_margin_m + tolerance ||
                !math::is_finite(candidate.first) || !math::is_finite(candidate.second) || manifold.point_count >= maximum_manifold_points)
                return;
            const auto midpoint = candidate.first * 0.5 + candidate.second * 0.5;
            for (std::size_t index = 0; index < manifold.point_count; ++index)
                if (math::length_squared(midpoint - manifold.points[index].world_position_m) <= tolerance * tolerance)
                    return;
            auto& point = manifold.points[manifold.point_count++];
            point.world_position_m = midpoint;
            point.separation_m = separation;
            point.feature = candidate.feature;
            point.local_anchor_first_m = math::inverse_transform_point(query.first_transform, candidate.first);
            point.local_anchor_second_m = math::inverse_transform_point(query.second_transform, candidate.second);
            manifold.is_speculative = manifold.is_speculative || separation > 0.0;
        }

        bool circle_circle(const CircleShape& first, const CircleShape& second, const NarrowPhaseQuery& query, ContactManifold& manifold)
        {
            const auto a = math::transform_point(query.first_transform, first.local_center_m());
            const auto b = math::transform_point(query.second_transform, second.local_center_m());
            const auto difference = b - a;
            const auto distance = std::hypot(difference.x, difference.y);
            if (distance - first.radius_m() - second.radius_m() > query.contact_margin_m + tolerance)
                return false;
            manifold.normal = unit(difference, math::transform_direction(query.first_transform, { 1.0, 0.0 }));
            add(manifold, query, { a + manifold.normal * first.radius_m(), b - manifold.normal * second.radius_m(), {} });
            return !manifold.is_empty();
        }

        bool circle_polygon(const CircleShape& circle, const Polygon& poly, bool circle_first,
            const NarrowPhaseQuery& query, ContactManifold& manifold)
        {
            const auto& transform = circle_first ? query.first_transform : query.second_transform;
            const auto center = math::transform_point(transform, circle.local_center_m());
            Real face_distance = -std::numeric_limits<Real>::max();
            std::size_t face = 0;
            if (poly.vertices.size() > 2)
            {
                for (std::size_t index = 0; index < poly.vertices.size(); ++index)
                {
                    const auto distance = math::dot(poly.normals[index], center - poly.vertices[index]);
                    if (distance > face_distance)
                    {
                        face_distance = distance;
                        face = index;
                    }
                }
            }
            Vec2 point;
            Vec2 normal;
            std::size_t edge = face;
            std::uint16_t vertex = 0;
            if (poly.vertices.size() > 2 && face_distance <= 0.0)
            {
                normal = -poly.normals[face];
                point = center - poly.normals[face] * face_distance;
            }
            else
            {
                Real closest_distance = std::numeric_limits<Real>::max();
                const auto edges = poly.vertices.size() == 2 ? 1 : poly.vertices.size();
                for (std::size_t index = 0; index < edges; ++index)
                {
                    Real fraction = 0.0;
                    const auto next = (index + 1) % poly.vertices.size();
                    const auto candidate = closest_on_edge(center, poly.vertices[index], poly.vertices[next], fraction);
                    const auto distance = std::hypot(candidate.x - center.x, candidate.y - center.y);
                    if (distance < closest_distance)
                    {
                        closest_distance = distance;
                        point = candidate;
                        edge = index;
                        vertex = fraction <= 0.0 ? static_cast<std::uint16_t>(index + 1)
                            : fraction >= 1.0    ? static_cast<std::uint16_t>(next + 1)
                                                 : 0;
                    }
                }
                if (closest_distance - circle.radius_m() > query.contact_margin_m + tolerance)
                    return false;
                normal = unit(point - center, -poly.normals[edge]);
            }
            const auto circle_point = center + normal * circle.radius_m();
            Candidate candidate;
            if (circle_first)
            {
                manifold.normal = normal;
                candidate = { circle_point, point, {} };
                candidate.feature.outgoing_edge = static_cast<std::uint16_t>(edge + 1);
                candidate.feature.outgoing_vertex = vertex;
            }
            else
            {
                manifold.normal = -normal;
                candidate = { point, circle_point, {} };
                candidate.feature.incoming_edge = static_cast<std::uint16_t>(edge + 1);
                candidate.feature.incoming_vertex = vertex;
            }
            add(manifold, query, candidate);
            return !manifold.is_empty();
        }

        std::pair<Real, std::size_t> separation(const Polygon& first, const Polygon& second)
        {
            std::pair<Real, std::size_t> best { -std::numeric_limits<Real>::max(), 0 };
            for (std::size_t edge = 0; edge < first.vertices.size(); ++edge)
            {
                Real distance = std::numeric_limits<Real>::max();
                for (const auto& point : second.vertices)
                    distance = std::min(distance, math::dot(first.normals[edge], point - first.vertices[edge]));
                if (distance > best.first)
                    best = { distance, edge };
            }
            return best;
        }

        std::vector<ClipVertex> clip(const std::vector<ClipVertex>& input, const Vec2& normal,
            Real offset, std::uint16_t reference_vertex)
        {
            if (input.empty())
                return {};
            if (input.size() == 1)
                return math::dot(normal, input[0].point) <= offset + tolerance ? input : std::vector<ClipVertex> {};
            std::vector<ClipVertex> result;
            const auto first_distance = math::dot(normal, input[0].point) - offset;
            const auto second_distance = math::dot(normal, input[1].point) - offset;
            if (first_distance <= tolerance)
                result.push_back(input[0]);
            if (second_distance <= tolerance)
                result.push_back(input[1]);
            if (first_distance * second_distance < 0.0)
            {
                const auto fraction = first_distance / (first_distance - second_distance);
                result.push_back({ input[0].point + (input[1].point - input[0].point) * fraction, 0, reference_vertex });
            }
            return result;
        }

        bool fallback(const NarrowPhaseQuery& query, ContactManifold& manifold)
        {
            const auto distance = convex_distance(*query.first_shape, query.first_transform, *query.second_shape, query.second_transform);
            if (!distance.valid || (!distance.intersecting && distance.distance_m > query.contact_margin_m + tolerance))
                return false;
            manifold.normal = distance.normal;
            add(manifold, query, { distance.first_point_m, distance.second_point_m, {} });
            return !manifold.is_empty();
        }

        bool polygon_polygon(const Polygon& first, const Polygon& second, const NarrowPhaseQuery& query, ContactManifold& manifold)
        {
            const auto a = separation(first, second);
            const auto b = separation(second, first);
            if (a.first > query.contact_margin_m + tolerance || b.first > query.contact_margin_m + tolerance)
                return false;
            const auto reference_first = b.first <= a.first + tolerance;
            const auto& reference = reference_first ? first : second;
            const auto& incident = reference_first ? second : first;
            const auto reference_edge = reference_first ? a.second : b.second;
            const auto reference_next = (reference_edge + 1) % reference.vertices.size();
            const auto normal = reference.normals[reference_edge];
            std::size_t incident_edge = 0;
            for (std::size_t edge = 1; edge < incident.normals.size(); ++edge)
                if (math::dot(incident.normals[edge], normal) < math::dot(incident.normals[incident_edge], normal))
                    incident_edge = edge;
            const auto incident_next = (incident_edge + 1) % incident.vertices.size();
            const auto tangent = unit(reference.vertices[reference_next] - reference.vertices[reference_edge]);
            std::vector<ClipVertex> clipped {
                { incident.vertices[incident_edge], static_cast<std::uint16_t>(incident_edge + 1), 0 },
                { incident.vertices[incident_next], static_cast<std::uint16_t>(incident_next + 1), 0 }
            };
            clipped = clip(clipped, -tangent, -math::dot(tangent, reference.vertices[reference_edge]), static_cast<std::uint16_t>(reference_edge + 1));
            clipped = clip(clipped, tangent, math::dot(tangent, reference.vertices[reference_next]), static_cast<std::uint16_t>(reference_next + 1));
            manifold.normal = reference_first ? normal : -normal;
            std::sort(clipped.begin(), clipped.end(), [&](const ClipVertex& left, const ClipVertex& right)
                {
                    return math::dot(left.point, tangent) < math::dot(right.point, tangent);
                });
            for (const auto& vertex : clipped)
            {
                const auto depth = math::dot(normal, vertex.point - reference.vertices[reference_edge]);
                const auto reference_point = vertex.point - normal * depth;
                Candidate candidate;
                candidate.first = reference_first ? reference_point : vertex.point;
                candidate.second = reference_first ? vertex.point : reference_point;
                candidate.feature.incoming_edge = static_cast<std::uint16_t>((reference_first ? reference_edge : incident_edge) + 1);
                candidate.feature.outgoing_edge = static_cast<std::uint16_t>((reference_first ? incident_edge : reference_edge) + 1);
                candidate.feature.incoming_vertex = reference_first ? vertex.reference_vertex : vertex.incident_vertex;
                candidate.feature.outgoing_vertex = reference_first ? vertex.incident_vertex : vertex.reference_vertex;
                add(manifold, query, candidate);
            }
            return !manifold.is_empty() || fallback(query, manifold);
        }
    }

    std::string_view CollisionNarrowPhase::name() const
    {
        return "convex_collision";
    }

    bool CollisionNarrowPhase::collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const
    {
        manifold = {};
        manifold.first = query.first;
        manifold.second = query.second;
        manifold.first_collider = query.first_collider;
        manifold.second_collider = query.second_collider;
        manifold.material = query.material;
        manifold.is_sensor = query.is_sensor;
        if (!query.first_shape || !query.second_shape || !math::is_finite(query.contact_margin_m) || query.contact_margin_m < 0.0)
            return false;
        const auto valid_pose = [](const math::Transform2& transform)
        {
            const auto axis = math::transform_direction(transform, { 1.0, 0.0 });
            return math::is_finite(transform.translation) && math::is_finite(axis) && std::abs(math::length_squared(axis) - 1.0) < 1.0e-8;
        };
        if (!valid_pose(query.first_transform) || !valid_pose(query.second_transform))
            return false;
        const auto* first_circle = dynamic_cast<const CircleShape*>(query.first_shape);
        const auto* second_circle = dynamic_cast<const CircleShape*>(query.second_shape);
        if (first_circle && second_circle)
            return circle_circle(*first_circle, *second_circle, query, manifold);
        const auto first_polygon = polygon(*query.first_shape, query.first_transform);
        const auto second_polygon = polygon(*query.second_shape, query.second_transform);
        if (first_circle && !second_polygon.vertices.empty())
            return circle_polygon(*first_circle, second_polygon, true, query, manifold);
        if (second_circle && !first_polygon.vertices.empty())
            return circle_polygon(*second_circle, first_polygon, false, query, manifold);
        if (!first_polygon.vertices.empty() && !second_polygon.vertices.empty())
            return polygon_polygon(first_polygon, second_polygon, query, manifold);
        return fallback(query, manifold);
    }
}
