#include <rigidbodies/math/polygon.hpp>

#include <algorithm>
#include <cstddef>

namespace rigidbodies::math
{

    Real signed_area(VertexSpan vertices)
    {
        if (vertices.size() < 3)
        {
            return 0.0;
        }
        Real accumulated = 0.0;
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto& current = vertices[index];
            const auto& next = vertices[(index + 1) % vertices.size()];
            accumulated += cross(current, next);
        }
        return 0.5 * accumulated;
    }

    Vec2 centroid(VertexSpan vertices)
    {
        if (vertices.empty())
        {
            return {};
        }
        if (vertices.size() < 3)
        {
            Vec2 average {};
            for (const auto& vertex : vertices)
            {
                average += vertex;
            }
            return average / static_cast<Real>(vertices.size());
        }

        const auto area = signed_area(vertices);
        if (std::abs(area) <= geometric_epsilon)
        {
            // A degenerate outline has no meaningful area centroid, so fall back to the vertex
            // average rather than dividing by a vanishing area.
            Vec2 average {};
            for (const auto& vertex : vertices)
            {
                average += vertex;
            }
            return average / static_cast<Real>(vertices.size());
        }

        Vec2 accumulated {};
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto& current = vertices[index];
            const auto& next = vertices[(index + 1) % vertices.size()];
            const auto weight = cross(current, next);
            accumulated += (current + next) * weight;
        }
        return accumulated / (6.0 * area);
    }

    Real second_moment_of_area(VertexSpan vertices)
    {
        if (vertices.size() < 3)
        {
            return 0.0;
        }

        const auto area = signed_area(vertices);
        if (std::abs(area) <= geometric_epsilon)
        {
            return 0.0;
        }

        // Accumulate the polar second moment about the origin, then shift to the centroid with
        // the parallel-axis theorem.
        const auto reference = centroid(vertices);
        Real numerator = 0.0;
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto current = vertices[index] - reference;
            const auto next = vertices[(index + 1) % vertices.size()] - reference;
            const auto weight = cross(current, next);
            numerator += weight * (length_squared(current) + dot(current, next) + length_squared(next));
        }
        return std::abs(numerator / 12.0);
    }

    Aabb compute_bounds(VertexSpan vertices)
    {
        Aabb bounds;
        for (const auto& vertex : vertices)
        {
            bounds.expand(vertex);
        }
        return bounds;
    }

    bool is_convex(VertexSpan vertices)
    {
        if (vertices.size() < 3)
        {
            return false;
        }

        bool saw_positive = false;
        bool saw_negative = false;
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto& current = vertices[index];
            const auto& next = vertices[(index + 1) % vertices.size()];
            const auto& following = vertices[(index + 2) % vertices.size()];
            const auto turn = cross(next - current, following - next);
            if (turn > geometric_epsilon)
            {
                saw_positive = true;
            }
            else if (turn < -geometric_epsilon)
            {
                saw_negative = true;
            }
            if (saw_positive && saw_negative)
            {
                return false;
            }
        }
        return saw_positive != saw_negative;
    }

    std::vector<Vec2> ensure_counter_clockwise(VertexSpan vertices)
    {
        std::vector<Vec2> result(vertices.begin(), vertices.end());
        if (signed_area(result) < 0.0)
        {
            std::reverse(result.begin(), result.end());
        }
        return result;
    }

    Vec2 support_point(VertexSpan vertices, const Vec2& direction)
    {
        if (vertices.empty())
        {
            return {};
        }

        auto best = vertices.front();
        auto best_projection = dot(best, direction);
        for (const auto& vertex : vertices.subspan(1))
        {
            const auto projection = dot(vertex, direction);
            if (projection > best_projection)
            {
                best_projection = projection;
                best = vertex;
            }
        }
        return best;
    }

    std::vector<Vec2> edge_normals(VertexSpan vertices)
    {
        std::vector<Vec2> normals;
        if (vertices.size() < 2)
        {
            return normals;
        }

        normals.reserve(vertices.size());
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto edge = vertices[(index + 1) % vertices.size()] - vertices[index];
            // For a counter-clockwise outline the outward normal is the edge rotated a quarter
            // turn clockwise.
            normals.push_back(normalized({ edge.y, -edge.x }));
        }
        return normals;
    }

} // namespace rigidbodies::math
