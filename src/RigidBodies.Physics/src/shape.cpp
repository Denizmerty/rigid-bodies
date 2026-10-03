#include <rigidbodies/physics/shape.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {

        Real farthest_vertex_distance(const std::vector<math::Vec2>& vertices)
        {
            Real farthest = 0.0;
            for (const auto& vertex : vertices)
            {
                farthest = std::max(farthest, math::length(vertex));
            }
            return farthest;
        }

    } // namespace

    CircleShape::CircleShape(Real radius_m, const math::Vec2& local_center_m) : radius_m_(radius_m), local_center_m_(local_center_m)
    {
        if (!math::is_finite(radius_m) || radius_m <= 0.0 || !math::is_finite(local_center_m))
        {
            throw std::invalid_argument("circle radius must be positive and finite, and its centre must be finite");
        }
    }

    ShapeKind CircleShape::kind() const
    {
        return ShapeKind::circle;
    }

    std::shared_ptr<Shape> CircleShape::clone() const
    {
        return std::make_shared<CircleShape>(*this);
    }

    math::Aabb CircleShape::compute_bounds(const math::Transform2& transform) const
    {
        const auto center = math::transform_point(transform, local_center_m_);
        math::Aabb bounds;
        bounds.expand(center - math::Vec2 { radius_m_, radius_m_ });
        bounds.expand(center + math::Vec2 { radius_m_, radius_m_ });
        return bounds;
    }

    MassProperties CircleShape::compute_mass_properties(Real areal_density_kg_m2) const
    {
        MassProperties properties;
        const auto area = math::pi * radius_m_ * radius_m_;
        properties.mass_kg = area * areal_density_kg_m2;
        properties.center_of_mass_m = local_center_m_;
        // A uniform disc has a second moment of m r^2 / 2 about its centre.
        properties.inertia_kg_m2 = 0.5 * properties.mass_kg * radius_m_ * radius_m_;
        return properties;
    }

    math::Vec2 CircleShape::support_point(const math::Vec2& local_direction) const
    {
        return local_center_m_ + math::normalized(local_direction) * radius_m_;
    }

    bool CircleShape::contains_local_point(const math::Vec2& local_point) const
    {
        return math::length_squared(local_point - local_center_m_) <= radius_m_ * radius_m_;
    }

    Real CircleShape::bounding_radius() const
    {
        return math::length(local_center_m_) + radius_m_;
    }

    Real CircleShape::radius_m() const
    {
        return radius_m_;
    }

    const math::Vec2& CircleShape::local_center_m() const
    {
        return local_center_m_;
    }

    ConvexPolygonShape::ConvexPolygonShape(std::vector<math::Vec2> vertices)
    {
        for (const auto& vertex : vertices)
        {
            if (!math::is_finite(vertex))
            {
                throw std::invalid_argument("polygon vertices must be finite");
            }
        }
        vertices_ = math::ensure_counter_clockwise(vertices);
        if (vertices_.size() < 3)
        {
            throw std::invalid_argument("a convex polygon needs at least three vertices");
        }
        if (!math::is_convex(vertices_))
        {
            throw std::invalid_argument("polygon outline is not convex; decompose it before constructing a collider");
        }
        normals_ = math::edge_normals(vertices_);
        bounding_radius_ = farthest_vertex_distance(vertices_);
    }

    ConvexPolygonShape ConvexPolygonShape::box(Real width_m, Real height_m)
    {
        if (!math::is_finite(width_m) || !math::is_finite(height_m) || width_m <= 0.0 || height_m <= 0.0)
        {
            throw std::invalid_argument("box dimensions must be positive finite lengths");
        }
        const auto half_width = 0.5 * width_m;
        const auto half_height = 0.5 * height_m;
        return ConvexPolygonShape { { { -half_width, -half_height }, { half_width, -half_height }, { half_width, half_height }, { -half_width, half_height } } };
    }

    ConvexPolygonShape ConvexPolygonShape::regular(int side_count, Real circumradius_m)
    {
        if (side_count < 3)
        {
            throw std::invalid_argument("a regular polygon needs at least three sides");
        }
        if (!math::is_finite(circumradius_m) || circumradius_m <= 0.0)
        {
            throw std::invalid_argument("circumradius must be a positive finite length");
        }

        std::vector<math::Vec2> vertices;
        vertices.reserve(static_cast<std::size_t>(side_count));
        for (int index = 0; index < side_count; ++index)
        {
            const auto angle = math::two_pi * static_cast<Real>(index) / static_cast<Real>(side_count);
            vertices.push_back({ circumradius_m * std::cos(angle), circumradius_m * std::sin(angle) });
        }
        return ConvexPolygonShape { std::move(vertices) };
    }

    ShapeKind ConvexPolygonShape::kind() const
    {
        return ShapeKind::convex_polygon;
    }

    std::shared_ptr<Shape> ConvexPolygonShape::clone() const
    {
        return std::make_shared<ConvexPolygonShape>(*this);
    }

    math::Aabb ConvexPolygonShape::compute_bounds(const math::Transform2& transform) const
    {
        math::Aabb bounds;
        for (const auto& vertex : vertices_)
        {
            bounds.expand(math::transform_point(transform, vertex));
        }
        return bounds;
    }

    MassProperties ConvexPolygonShape::compute_mass_properties(Real areal_density_kg_m2) const
    {
        MassProperties properties;
        const auto area = std::abs(math::signed_area(vertices_));
        properties.mass_kg = area * areal_density_kg_m2;
        properties.center_of_mass_m = math::centroid(vertices_);
        properties.inertia_kg_m2 = math::second_moment_of_area(vertices_) * areal_density_kg_m2;
        return properties;
    }

    math::Vec2 ConvexPolygonShape::support_point(const math::Vec2& local_direction) const
    {
        return math::support_point(vertices_, local_direction);
    }

    bool ConvexPolygonShape::contains_local_point(const math::Vec2& local_point) const
    {
        // The outline is convex and counter-clockwise, so the point is inside exactly when it
        // lies behind every edge plane.
        for (std::size_t index = 0; index < vertices_.size(); ++index)
        {
            if (math::dot(normals_[index], local_point - vertices_[index]) > math::geometric_epsilon)
            {
                return false;
            }
        }
        return true;
    }

    Real ConvexPolygonShape::bounding_radius() const
    {
        return bounding_radius_;
    }

    const std::vector<math::Vec2>& ConvexPolygonShape::vertices() const
    {
        return vertices_;
    }

    const std::vector<math::Vec2>& ConvexPolygonShape::normals() const
    {
        return normals_;
    }

    SegmentShape::SegmentShape(const math::Vec2& start_m, const math::Vec2& end_m) : start_m_(start_m), end_m_(end_m)
    {
        if (!math::is_finite(start_m) || !math::is_finite(end_m) || math::distance(start_m, end_m) <= math::geometric_epsilon)
        {
            throw std::invalid_argument("segment endpoints must be finite and distinct");
        }
    }

    ShapeKind SegmentShape::kind() const
    {
        return ShapeKind::segment;
    }

    std::shared_ptr<Shape> SegmentShape::clone() const
    {
        return std::make_shared<SegmentShape>(*this);
    }

    math::Aabb SegmentShape::compute_bounds(const math::Transform2& transform) const
    {
        math::Aabb bounds;
        bounds.expand(math::transform_point(transform, start_m_));
        bounds.expand(math::transform_point(transform, end_m_));
        return bounds;
    }

    MassProperties SegmentShape::compute_mass_properties(Real) const
    {
        // A segment encloses no area. It contributes no mass, which is why it is only usable as
        // static geometry.
        MassProperties properties;
        properties.center_of_mass_m = (start_m_ + end_m_) * 0.5;
        return properties;
    }

    math::Vec2 SegmentShape::support_point(const math::Vec2& local_direction) const
    {
        return math::dot(start_m_, local_direction) >= math::dot(end_m_, local_direction) ? start_m_ : end_m_;
    }

    bool SegmentShape::contains_local_point(const math::Vec2&) const
    {
        return false;
    }

    Real SegmentShape::bounding_radius() const
    {
        return std::max(math::length(start_m_), math::length(end_m_));
    }

    const math::Vec2& SegmentShape::start_m() const
    {
        return start_m_;
    }

    const math::Vec2& SegmentShape::end_m() const
    {
        return end_m_;
    }

    ShapePtr make_circle(Real radius_m, const math::Vec2& local_center_m)
    {
        return std::make_shared<CircleShape>(radius_m, local_center_m);
    }

    ShapePtr make_box(Real width_m, Real height_m)
    {
        return std::make_shared<ConvexPolygonShape>(ConvexPolygonShape::box(width_m, height_m));
    }

    ShapePtr make_regular_polygon(int side_count, Real circumradius_m)
    {
        return std::make_shared<ConvexPolygonShape>(ConvexPolygonShape::regular(side_count, circumradius_m));
    }

    ShapePtr make_segment(const math::Vec2& start_m, const math::Vec2& end_m)
    {
        return std::make_shared<SegmentShape>(start_m, end_m);
    }

} // namespace rigidbodies::physics
