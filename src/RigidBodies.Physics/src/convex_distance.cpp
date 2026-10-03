#include <rigidbodies/physics/convex_distance.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace rigidbodies::physics
{
    namespace
    {
        using math::Vec2;
        struct Vertex
        {
            Vec2 first;
            Vec2 second;
            Vec2 point;
        };
        struct Simplex
        {
            std::vector<Vertex> vertices;
            std::array<Real, 3> weights { 1.0, 0.0, 0.0 };
            Vec2 closest {};
        };

        Vec2 unit(const Vec2& value, const Vec2& fallback = { 1.0, 0.0 })
        {
            const auto length = std::hypot(value.x, value.y);
            return length > 0.0 && math::is_finite(length) ? value / length : fallback;
        }

        bool valid_transform(const math::Transform2& value)
        {
            const auto x = math::transform_direction(value, { 1.0, 0.0 });
            const auto y = math::transform_direction(value, { 0.0, 1.0 });
            return math::is_finite(value.translation) && math::is_finite(x) && math::is_finite(y) &&
                std::abs(math::length_squared(x) - 1.0) < 1.0e-8 && std::abs(math::dot(x, y)) < 1.0e-8;
        }

        Vertex support(const Shape& first, const math::Transform2& a, const Shape& second,
            const math::Transform2& b, const Vec2& direction)
        {
            const auto n = unit(direction);
            const auto p = math::transform_point(a, first.support_point(math::inverse_transform_direction(a, n)));
            const auto q = math::transform_point(b, second.support_point(math::inverse_transform_direction(b, -n)));
            return { p, q, p - q };
        }

        bool finite(const Vertex& vertex)
        {
            return math::is_finite(vertex.first) && math::is_finite(vertex.second) && math::is_finite(vertex.point);
        }

        bool reduce(Simplex& simplex)
        {
            if (simplex.vertices.size() == 1)
            {
                simplex.closest = simplex.vertices[0].point;
                simplex.weights = { 1.0, 0.0, 0.0 };
                return false;
            }
            if (simplex.vertices.size() == 3)
            {
                const auto a = simplex.vertices[0].point;
                const auto b = simplex.vertices[1].point;
                const auto c = simplex.vertices[2].point;
                const auto area = math::cross(b - a, c - a);
                if (area != 0.0 && math::is_finite(area))
                {
                    const auto wa = math::cross(b, c) / area;
                    const auto wb = math::cross(c, a) / area;
                    const auto wc = 1.0 - wa - wb;
                    if (wa >= 0.0 && wb >= 0.0 && wc >= 0.0)
                    {
                        simplex.weights = { wa, wb, wc };
                        simplex.closest = {};
                        return true;
                    }
                }
            }
            Real best = std::numeric_limits<Real>::max();
            std::size_t first = 0;
            std::size_t second = 1;
            Real fraction = 0.0;
            for (std::size_t index = 0; index < simplex.vertices.size(); ++index)
            {
                const auto next = (index + 1) % simplex.vertices.size();
                const auto start = simplex.vertices[index].point;
                const auto edge = simplex.vertices[next].point - start;
                const auto denominator = math::length_squared(edge);
                const auto t = denominator > 0.0 ? math::clamp(-math::dot(start, edge) / denominator, 0.0, 1.0) : 0.0;
                const auto point = start + edge * t;
                const auto distance = math::length_squared(point);
                if (distance < best)
                {
                    best = distance;
                    first = index;
                    second = next;
                    fraction = t;
                    simplex.closest = point;
                }
            }
            const auto a = simplex.vertices[first];
            const auto b = simplex.vertices[second];
            simplex.vertices.clear();
            if (fraction <= 0.0)
            {
                simplex.vertices.push_back(a);
                simplex.weights = { 1.0, 0.0, 0.0 };
            }
            else if (fraction >= 1.0)
            {
                simplex.vertices.push_back(b);
                simplex.weights = { 1.0, 0.0, 0.0 };
            }
            else
            {
                simplex.vertices = { a, b };
                simplex.weights = { 1.0 - fraction, fraction, 0.0 };
            }
            return false;
        }

        ConvexDistanceResult witnesses(const Simplex& simplex, const Vec2& fallback)
        {
            ConvexDistanceResult result;
            for (std::size_t index = 0; index < simplex.vertices.size(); ++index)
            {
                result.first_point_m += simplex.vertices[index].first * simplex.weights[index];
                result.second_point_m += simplex.vertices[index].second * simplex.weights[index];
            }
            const auto delta = result.second_point_m - result.first_point_m;
            result.distance_m = std::hypot(delta.x, delta.y);
            result.normal = unit(delta, fallback);
            result.valid = math::is_finite(result.distance_m) && math::is_finite(result.first_point_m) && math::is_finite(result.second_point_m);
            return result;
        }

        std::vector<Vertex> hull(std::vector<Vertex> points, Real tolerance)
        {
            std::sort(points.begin(), points.end(), [](const Vertex& a, const Vertex& b)
                {
                    return a.point.x < b.point.x || (a.point.x == b.point.x && a.point.y < b.point.y);
                });
            points.erase(std::unique(points.begin(), points.end(), [tolerance](const Vertex& a, const Vertex& b)
                             {
                                 return math::length_squared(a.point - b.point) <= tolerance * tolerance;
                             }),
                points.end());
            if (points.size() <= 2)
                return points;
            std::vector<Vertex> result;
            for (const auto& point : points)
            {
                while (result.size() >= 2 && math::cross(result.back().point - result[result.size() - 2].point, point.point - result.back().point) <= 0.0)
                    result.pop_back();
                result.push_back(point);
            }
            const auto lower = result.size();
            for (std::size_t index = points.size() - 1; index-- > 0;)
            {
                const auto& point = points[index];
                while (result.size() > lower && math::cross(result.back().point - result[result.size() - 2].point, point.point - result.back().point) <= 0.0)
                    result.pop_back();
                result.push_back(point);
            }
            result.pop_back();
            return result;
        }

        ConvexDistanceResult penetration(const Shape& first, const math::Transform2& a, const Shape& second,
            const math::Transform2& b, const Simplex& simplex, Real tolerance, const Vec2& fallback)
        {
            auto points = simplex.vertices;
            for (int index = 0; index < 8; ++index)
            {
                const auto angle = math::two_pi * static_cast<Real>(index) / 8.0;
                const auto vertex = support(first, a, second, b, math::transform_direction(a, { std::cos(angle), std::sin(angle) }));
                if (!finite(vertex))
                    return {};
                points.push_back(vertex);
            }
            points = hull(std::move(points), tolerance);
            if (points.size() < 3)
            {
                auto result = witnesses(simplex, fallback);
                result.intersecting = result.valid;
                result.distance_m = 0.0;
                return result;
            }
            ConvexDistanceResult result;
            for (int iteration = 0; iteration < 256; ++iteration)
            {
                Real depth = std::numeric_limits<Real>::max();
                std::size_t edge_index = 0;
                Vec2 normal;
                for (std::size_t index = 0; index < points.size(); ++index)
                {
                    const auto edge = points[(index + 1) % points.size()].point - points[index].point;
                    const auto n = unit({ edge.y, -edge.x });
                    const auto distance = math::dot(n, points[index].point);
                    if (distance < depth)
                    {
                        depth = distance;
                        edge_index = index;
                        normal = n;
                    }
                }
                if (depth < -tolerance)
                    return {};
                const auto& p = points[edge_index];
                const auto& q = points[(edge_index + 1) % points.size()];
                const auto edge = q.point - p.point;
                const auto denominator = math::length_squared(edge);
                const auto t = denominator > 0.0 ? math::clamp(-math::dot(p.point, edge) / denominator, 0.0, 1.0) : 0.0;
                result.valid = true;
                result.intersecting = true;
                result.penetration_depth_m = std::max(0.0, depth);
                result.normal = normal;
                result.first_point_m = p.first + (q.first - p.first) * t;
                result.second_point_m = p.second + (q.second - p.second) * t;
                const auto vertex = support(first, a, second, b, normal);
                if (!finite(vertex))
                    return {};
                const auto gap = math::dot(vertex.point, normal) - depth;
                if (gap <= tolerance || std::any_of(points.begin(), points.end(), [&](const Vertex& old)
                                            {
                                                return math::length_squared(old.point - vertex.point) <= tolerance * tolerance;
                                            }))
                    return result;
                points.insert(points.begin() + static_cast<std::ptrdiff_t>(edge_index + 1), vertex);
            }
            // EPA's inscribed polytope provides a conservative penetration estimate at its limit.
            return result;
        }
    }

    ConvexDistanceResult convex_distance(const Shape& first, const math::Transform2& a,
        const Shape& second, const math::Transform2& b)
    {
        if (!valid_transform(a) || !valid_transform(b))
            return {};
        const auto scale = std::max({ 1.0, first.bounding_radius(), second.bounding_radius(), std::hypot(a.translation.x - b.translation.x, a.translation.y - b.translation.y) });
        if (!math::is_finite(scale))
            return {};
        const auto tolerance = 1.0e-10 * scale;
        const auto fallback = unit(b.translation - a.translation, math::transform_direction(a, { 1.0, 0.0 }));
        Simplex simplex;
        const auto initial = support(first, a, second, b, fallback);
        if (!finite(initial))
            return {};
        simplex.vertices.push_back(initial);
        for (int iteration = 0; iteration < 96; ++iteration)
        {
            const auto inside = reduce(simplex);
            const auto distance_squared = math::length_squared(simplex.closest);
            if (!math::is_finite(distance_squared))
                return {};
            if (inside || distance_squared <= tolerance * tolerance)
                return penetration(first, a, second, b, simplex, tolerance, fallback);
            const auto vertex = support(first, a, second, b, -simplex.closest);
            if (!finite(vertex))
                return {};
            const auto improvement = distance_squared - math::dot(simplex.closest, vertex.point);
            if (improvement <= tolerance * std::max(tolerance, std::sqrt(distance_squared)) ||
                std::any_of(simplex.vertices.begin(), simplex.vertices.end(), [&](const Vertex& old)
                    {
                        return math::length_squared(old.point - vertex.point) <= tolerance * tolerance;
                    }))
                return witnesses(simplex, fallback);
            simplex.vertices.push_back(vertex);
        }
        (void)reduce(simplex);
        return witnesses(simplex, fallback);
    }
}
