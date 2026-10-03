#include <rigidbodies/physics/compound_mass.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {

        using Wide = long double;
        constexpr Wide tolerance = 64.0L * std::numeric_limits<Real>::epsilon();
        const Wide pi = std::acos(-1.0L);
        const Wide full_turn = 2.0L * pi;

        struct Point
        {
            Wide x {};
            Wide y {};
        };

        Point operator+(Point a, Point b)
        {
            return { a.x + b.x, a.y + b.y };
        }

        Point operator-(Point a, Point b)
        {
            return { a.x - b.x, a.y - b.y };
        }

        Point operator*(Point a, Wide scalar)
        {
            return { a.x * scalar, a.y * scalar };
        }

        Wide dot(Point a, Point b)
        {
            return a.x * b.x + a.y * b.y;
        }

        Wide cross(Point a, Point b)
        {
            return a.x * b.y - a.y * b.x;
        }

        Wide length(Point value)
        {
            return std::hypot(value.x, value.y);
        }

        Point left_normal(Point value)
        {
            return { -value.y, value.x };
        }

        Point placed(const math::Transform2& transform, math::Vec2 value)
        {
            const auto result = math::transform_point(transform, value);
            if (!math::is_finite(result))
            {
                throw std::invalid_argument("mass geometry requires finite placements and coordinates");
            }
            return { result.x, result.y };
        }

        struct Outline
        {
            bool circular { false };
            Point center;
            Wide radius {};
            std::vector<Point> vertices;

            bool empty() const
            {
                return circular ? radius <= 0.0L : vertices.size() < 3;
            }
        };

        struct Part
        {
            Outline outer;
            Outline inner;
            Wide density {};
            Wide depth {};
            Wide wall { -1.0L };
        };

        struct Boundary
        {
            // All outlines run counter-clockwise. The material-density jump across each curve
            // supplies its signed weight, including the reversed contribution of a hole.
            bool circular { false };
            Point center;
            Wide radius {};
            Point start;
            Point end;
            std::vector<Wide> cuts;
        };

        struct Moments
        {
            Wide mass {};
            Wide first_x {};
            Wide first_y {};
            Wide polar {};

            void add(const Moments& value, Wide weight)
            {
                mass += value.mass * weight;
                first_x += value.first_x * weight;
                first_y += value.first_y * weight;
                polar += value.polar * weight;
            }
        };

        Outline inset_polygon(const Outline& outer, Wide wall)
        {
            Outline inner = outer;
            for (std::size_t edge = 0; edge < outer.vertices.size() && !inner.vertices.empty(); ++edge)
            {
                const auto start = outer.vertices[edge];
                const auto direction = outer.vertices[(edge + 1) % outer.vertices.size()] - start;
                const auto offset = wall * length(direction);
                const auto distance = [&](Point point)
                {
                    return cross(direction, point - start) - offset;
                };
                std::vector<Point> clipped;
                auto previous = inner.vertices.back();
                auto previous_distance = distance(previous);
                for (const auto current : inner.vertices)
                {
                    const auto current_distance = distance(current);
                    if ((previous_distance >= 0.0L) != (current_distance >= 0.0L))
                    {
                        const auto fraction = previous_distance / (previous_distance - current_distance);
                        clipped.push_back(previous + (current - previous) * fraction);
                    }
                    if (current_distance >= 0.0L)
                    {
                        clipped.push_back(current);
                    }
                    previous = current;
                    previous_distance = current_distance;
                }
                inner.vertices = std::move(clipped);
            }
            // The offset may collapse to an edge or point; neither removes any material area.
            Wide twice_area = 0.0L;
            for (std::size_t index = 0; index < inner.vertices.size(); ++index)
            {
                twice_area += cross(inner.vertices[index], inner.vertices[(index + 1) % inner.vertices.size()]);
            }
            if (twice_area <= tolerance * tolerance)
            {
                inner.vertices.clear();
            }
            return inner;
        }

        void append_boundaries(const Outline& outline, std::vector<Boundary>& boundaries)
        {
            if (outline.empty())
            {
                return;
            }
            if (outline.circular)
            {
                Boundary circle;
                circle.circular = true;
                circle.center = outline.center;
                circle.radius = outline.radius;
                circle.cuts = { 0.0L, full_turn };
                boundaries.push_back(std::move(circle));
                return;
            }
            for (std::size_t index = 0; index < outline.vertices.size(); ++index)
            {
                Boundary edge;
                edge.start = outline.vertices[index];
                edge.end = outline.vertices[(index + 1) % outline.vertices.size()];
                if (length(edge.end - edge.start) <= tolerance)
                {
                    continue;
                }
                edge.cuts = { 0.0L, 1.0L };
                boundaries.push_back(std::move(edge));
            }
        }

        void cut_line(Boundary& edge, Wide parameter)
        {
            if (parameter >= -tolerance && parameter <= 1.0L + tolerance)
            {
                edge.cuts.push_back(std::clamp(parameter, 0.0L, 1.0L));
            }
        }

        void cut_circle(Boundary& circle, Point point)
        {
            const auto delta = point - circle.center;
            auto angle = std::atan2(delta.y, delta.x);
            if (angle < 0.0L)
            {
                angle += full_turn;
            }
            circle.cuts.push_back(angle);
        }

        void split_lines(Boundary& first, Boundary& second)
        {
            const auto a = first.end - first.start;
            const auto b = second.end - second.start;
            const auto offset = second.start - first.start;
            const auto denominator = cross(a, b);
            if (std::abs(denominator) > tolerance * length(a) * length(b))
            {
                const auto first_parameter = cross(offset, b) / denominator;
                const auto second_parameter = cross(offset, a) / denominator;
                if (first_parameter >= -tolerance && first_parameter <= 1.0L + tolerance &&
                    second_parameter >= -tolerance && second_parameter <= 1.0L + tolerance)
                {
                    cut_line(first, first_parameter);
                    cut_line(second, second_parameter);
                }
                return;
            }
            if (std::abs(cross(a, offset)) > tolerance * length(a))
            {
                return;
            }
            // Collinear intervals are split at each other's endpoints so duplicate boundary
            // pieces can be counted exactly once, including partly coincident box edges.
            cut_line(first, dot(second.start - first.start, a) / dot(a, a));
            cut_line(first, dot(second.end - first.start, a) / dot(a, a));
            cut_line(second, dot(first.start - second.start, b) / dot(b, b));
            cut_line(second, dot(first.end - second.start, b) / dot(b, b));
        }

        void split_line_circle(Boundary& edge, Boundary& circle)
        {
            const auto direction = edge.end - edge.start;
            const auto offset = edge.start - circle.center;
            const auto denominator = dot(direction, direction);
            const auto closest_parameter = -dot(offset, direction) / denominator;
            const auto closest = offset + direction * closest_parameter;
            const auto squared_height = circle.radius * circle.radius - dot(closest, closest);
            if (squared_height < -tolerance * circle.radius)
            {
                return;
            }
            const auto delta = std::sqrt(std::max(0.0L, squared_height) / denominator);
            for (const auto parameter : { closest_parameter - delta, closest_parameter + delta })
            {
                if (parameter >= -tolerance && parameter <= 1.0L + tolerance)
                {
                    cut_line(edge, parameter);
                    cut_circle(circle, edge.start + direction * std::clamp(parameter, 0.0L, 1.0L));
                }
            }
        }

        void split_circles(Boundary& first, Boundary& second)
        {
            const auto delta = second.center - first.center;
            const auto distance = length(delta);
            if (distance <= tolerance || distance > first.radius + second.radius + tolerance ||
                distance < std::abs(first.radius - second.radius) - tolerance)
            {
                return;
            }
            const auto along = (first.radius * first.radius - second.radius * second.radius + distance * distance) / (2.0L * distance);
            const auto squared_height = first.radius * first.radius - along * along;
            if (squared_height < -tolerance * first.radius)
            {
                return;
            }
            const auto unit = delta * (1.0L / distance);
            const auto middle = first.center + unit * along;
            const auto offset = left_normal(unit) * std::sqrt(std::max(0.0L, squared_height));
            for (const auto point : { middle + offset, middle - offset })
            {
                cut_circle(first, point);
                cut_circle(second, point);
            }
        }

        void split_boundaries(std::vector<Boundary>& boundaries)
        {
            for (std::size_t first = 0; first < boundaries.size(); ++first)
            {
                for (std::size_t second = first + 1; second < boundaries.size(); ++second)
                {
                    auto& a = boundaries[first];
                    auto& b = boundaries[second];
                    if (a.circular && b.circular)
                    {
                        split_circles(a, b);
                    }
                    else if (a.circular)
                    {
                        split_line_circle(b, a);
                    }
                    else if (b.circular)
                    {
                        split_line_circle(a, b);
                    }
                    else
                    {
                        split_lines(a, b);
                    }
                }
            }
            for (auto& boundary : boundaries)
            {
                std::sort(boundary.cuts.begin(), boundary.cuts.end());
                boundary.cuts.erase(std::unique(boundary.cuts.begin(), boundary.cuts.end(), [](Wide a, Wide b)
                                        {
                                            return std::abs(a - b) <= tolerance;
                                        }),
                    boundary.cuts.end());
            }
        }

        bool inside_on_side(const Outline& outline, Point point, Point side)
        {
            if (outline.empty())
            {
                return false;
            }
            if (outline.circular)
            {
                const auto delta = point - outline.center;
                const auto distance = length(delta) - outline.radius;
                if (std::abs(distance) <= tolerance)
                {
                    return dot(delta, side) < 0.0L;
                }
                return distance < 0.0L;
            }
            for (std::size_t index = 0; index < outline.vertices.size(); ++index)
            {
                const auto start = outline.vertices[index];
                const auto direction = outline.vertices[(index + 1) % outline.vertices.size()] - start;
                const auto edge_length = length(direction);
                if (edge_length <= tolerance)
                {
                    continue;
                }
                const auto distance = cross(direction, point - start) / edge_length;
                if (distance < -tolerance || (std::abs(distance) <= tolerance && cross(direction, side) <= 0.0L))
                {
                    return false;
                }
            }
            return true;
        }

        Wide areal_density_on_side(const std::vector<Part>& parts, Point point, Point side)
        {
            Wide covered_depth = 0.0L;
            Wide density = 0.0L;
            for (const auto& part : parts)
            {
                if (part.depth <= covered_depth || !inside_on_side(part.outer, point, side) || inside_on_side(part.inner, point, side))
                {
                    continue;
                }
                // All slabs share z=0. Earlier material occupies a centred interval and a later
                // part adds only its extra thickness beyond the deepest interval already filled.
                density += part.density * (part.depth - covered_depth);
                covered_depth = part.depth;
            }
            return density;
        }

        bool is_duplicate_piece(const std::vector<Boundary>& boundaries, std::size_t current, Point midpoint)
        {
            const auto& boundary = boundaries[current];
            for (std::size_t index = 0; index < current; ++index)
            {
                const auto& earlier = boundaries[index];
                if (boundary.circular != earlier.circular)
                {
                    continue;
                }
                if (boundary.circular)
                {
                    if (length(boundary.center - earlier.center) <= tolerance && std::abs(boundary.radius - earlier.radius) <= tolerance)
                    {
                        return true;
                    }
                }
                else
                {
                    const auto direction = earlier.end - earlier.start;
                    const auto distance = std::abs(cross(direction, midpoint - earlier.start)) / length(direction);
                    const auto parameter = dot(midpoint - earlier.start, direction) / dot(direction, direction);
                    if (distance <= tolerance && parameter > -tolerance && parameter < 1.0L + tolerance &&
                        std::abs(cross(direction, boundary.end - boundary.start)) <= tolerance * length(direction) * length(boundary.end - boundary.start))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        Moments line_moments(Point a, Point b)
        {
            const auto determinant = cross(a, b);
            return { determinant * 0.5L,
                determinant * (a.x + b.x) / 6.0L,
                determinant * (a.y + b.y) / 6.0L,
                determinant * (dot(a, a) + dot(a, b) + dot(b, b)) / 12.0L };
        }

        Moments arc_moments(const Boundary& boundary, Wide start, Wide end)
        {
            // Green's theorem with homogeneous polynomial fields gives A = integral(cross)/2,
            // first moments = integral(position * cross)/3, and J = integral(r^2 * cross)/4.
            // These primitives integrate the actual circular arcs between arrangement vertices.
            const auto span = end - start;
            const auto middle = (start + end) * 0.5L;
            const auto sine_half = std::sin(span * 0.5L);
            const auto cosine_integral = 2.0L * std::cos(middle) * sine_half;
            const auto sine_integral = 2.0L * std::sin(middle) * sine_half;
            const auto cosine_squared_integral = span * 0.5L + std::cos(start + end) * std::sin(span) * 0.5L;
            const auto sine_squared_integral = span * 0.5L - std::cos(start + end) * std::sin(span) * 0.5L;
            const auto product_integral = std::sin(start + end) * std::sin(span) * 0.5L;
            const auto x = boundary.center.x;
            const auto y = boundary.center.y;
            const auto radius = boundary.radius;
            const auto squared_radius = radius * radius;
            const auto squared_center = x * x + y * y;
            const auto projection_integral = x * cosine_integral + y * sine_integral;
            const auto projection_squared_integral = x * x * cosine_squared_integral +
                2.0L * x * y * product_integral + y * y * sine_squared_integral;

            Moments result;
            result.mass = (squared_radius * span + radius * projection_integral) * 0.5L;
            result.first_x = (x * squared_radius * span + radius * (x * x + squared_radius) * cosine_integral +
                                 radius * x * y * sine_integral + squared_radius * x * cosine_squared_integral + squared_radius * y * product_integral) /
                3.0L;
            result.first_y = (y * squared_radius * span + radius * x * y * cosine_integral +
                                 radius * (y * y + squared_radius) * sine_integral + squared_radius * x * product_integral + squared_radius * y * sine_squared_integral) /
                3.0L;
            result.polar = ((squared_center + squared_radius) * squared_radius * span +
                               radius * (squared_center + 3.0L * squared_radius) * projection_integral + 2.0L * squared_radius * projection_squared_integral) *
                0.25L;
            return result;
        }

        Moments integrate(const std::vector<Part>& parts, const std::vector<Boundary>& boundaries)
        {
            Moments result;
            for (std::size_t index = 0; index < boundaries.size(); ++index)
            {
                const auto& boundary = boundaries[index];
                for (std::size_t piece = 1; piece < boundary.cuts.size(); ++piece)
                {
                    const auto start = boundary.cuts[piece - 1];
                    const auto end = boundary.cuts[piece];
                    const auto middle = (start + end) * 0.5L;
                    Point midpoint;
                    Point inward;
                    Moments moments;
                    if (boundary.circular)
                    {
                        const Point radial { std::cos(middle), std::sin(middle) };
                        midpoint = boundary.center + radial * boundary.radius;
                        inward = radial * -1.0L;
                        moments = arc_moments(boundary, start, end);
                    }
                    else
                    {
                        const auto direction = boundary.end - boundary.start;
                        midpoint = boundary.start + direction * middle;
                        inward = left_normal(direction) * (1.0L / length(direction));
                        moments = line_moments(boundary.start + direction * start, boundary.start + direction * end);
                    }
                    if (is_duplicate_piece(boundaries, index, midpoint))
                    {
                        continue;
                    }
                    const auto density_jump = areal_density_on_side(parts, midpoint, inward) - areal_density_on_side(parts, midpoint, inward * -1.0L);
                    result.add(moments, density_jump);
                }
            }
            return result;
        }

    } // namespace

    MassProperties compute_compound_mass_properties(const std::vector<Collider>& colliders)
    {
        std::vector<Part> parts;
        Point minimum { std::numeric_limits<Wide>::infinity(), std::numeric_limits<Wide>::infinity() };
        Point maximum { -std::numeric_limits<Wide>::infinity(), -std::numeric_limits<Wide>::infinity() };
        const auto expand = [&](Point point)
        {
            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
        };
        for (const auto& collider : colliders)
        {
            const auto areal_density = collider.areal_density_kg_m2();
            if (collider.shell_thickness_m && (!math::is_finite(*collider.shell_thickness_m) || *collider.shell_thickness_m < 0.0))
            {
                throw std::invalid_argument("shell thickness must be finite and non-negative");
            }
            if (!collider.shape || areal_density == 0.0 || (collider.shell_thickness_m && *collider.shell_thickness_m == 0.0))
            {
                continue;
            }
            const auto* circle = dynamic_cast<const CircleShape*>(collider.shape.get());
            const auto* polygon = dynamic_cast<const ConvexPolygonShape*>(collider.shape.get());
            if (dynamic_cast<const SegmentShape*>(collider.shape.get()) != nullptr)
                continue;
            if (circle == nullptr && polygon == nullptr)
            {
                // ShapeKind is a geometric category, not proof of a concrete C++ type. A custom
                // support shape supplies its own mass distribution through the virtual interface.
                // Exact overlap subtraction and hollowing require boundary geometry this interface
                // does not provide; reject those combinations instead of inventing their mass.
                if (colliders.size() != 1 || collider.shell_thickness_m)
                    throw std::invalid_argument("custom shape mass requires a single solid collider");
                const auto result = transformed(collider.shape->compute_mass_properties(areal_density), collider.local_transform);
                if (!result.is_valid())
                    throw std::invalid_argument("custom shape mass properties must be finite and non-negative");
                return result;
            }
            Part part;
            part.density = collider.effective_density_kg_m3();
            part.depth = collider.depth_m;
            part.wall = collider.shell_thickness_m.value_or(-1.0);
            if (circle != nullptr)
            {
                part.outer.circular = true;
                part.outer.center = placed(collider.local_transform, circle->local_center_m());
                part.outer.radius = circle->radius_m();
                expand(part.outer.center - Point { part.outer.radius, part.outer.radius });
                expand(part.outer.center + Point { part.outer.radius, part.outer.radius });
            }
            else
            {
                for (const auto vertex : polygon->vertices())
                {
                    const auto point = placed(collider.local_transform, vertex);
                    part.outer.vertices.push_back(point);
                    expand(point);
                }
            }
            parts.push_back(std::move(part));
        }
        if (parts.empty())
        {
            return {};
        }

        if (parts.size() == 1 && parts.front().outer.circular)
        {
            // The common single-disc/tube case has a simpler closed form. Factoring R^2-r^2
            // as wall*(2R-wall) retains thin-wall accuracy instead of subtracting nearly equal
            // complete-disc moments, even when the wall is tiny compared with the radius.
            const auto& part = parts.front();
            const auto radius = part.outer.radius;
            const auto hollow = part.wall >= 0.0L && part.wall < radius;
            const auto inner_radius = hollow ? radius - part.wall : 0.0L;
            const auto area = pi * (hollow ? part.wall * (2.0L * radius - part.wall) : radius * radius);
            const auto mass = area * part.density * part.depth;
            MassProperties result { static_cast<Real>(mass),
                { static_cast<Real>(part.outer.center.x), static_cast<Real>(part.outer.center.y) },
                static_cast<Real>(mass * (radius * radius + inner_radius * inner_radius) * 0.5L) };
            if (!result.is_valid())
            {
                throw std::overflow_error("compound mass properties exceed the finite numeric range");
            }
            return result;
        }

        const auto origin = minimum + (maximum - minimum) * 0.5L;
        const auto scale = std::max(maximum.x - minimum.x, maximum.y - minimum.y);
        if (!std::isfinite(scale) || scale <= 0.0L || !std::isfinite(origin.x) || !std::isfinite(origin.y))
        {
            throw std::overflow_error("compound geometry exceeds the finite numeric range");
        }
        std::vector<Boundary> boundaries;
        for (auto& part : parts)
        {
            if (part.outer.circular)
            {
                part.outer.center = (part.outer.center - origin) * (1.0L / scale);
                part.outer.radius /= scale;
                if (part.wall >= 0.0L && part.wall < part.outer.radius * scale)
                {
                    part.inner = part.outer;
                    part.inner.radius -= part.wall / scale;
                }
            }
            else
            {
                for (auto& vertex : part.outer.vertices)
                {
                    vertex = (vertex - origin) * (1.0L / scale);
                }
                if (part.wall >= 0.0L)
                {
                    part.inner = inset_polygon(part.outer, part.wall / scale);
                }
            }
            append_boundaries(part.outer, boundaries);
            append_boundaries(part.inner, boundaries);
        }
        split_boundaries(boundaries);
        const auto moments = integrate(parts, boundaries);
        if (moments.mass <= 0.0L)
        {
            return {};
        }
        const Point center { moments.first_x / moments.mass, moments.first_y / moments.mass };
        const auto center_in_body = origin + center * scale;
        const auto squared_scale = scale * scale;
        MassProperties result;
        result.mass_kg = static_cast<Real>(moments.mass * squared_scale);
        result.center_of_mass_m = { static_cast<Real>(center_in_body.x), static_cast<Real>(center_in_body.y) };
        result.inertia_kg_m2 = static_cast<Real>(std::max(0.0L, moments.polar - moments.mass * dot(center, center)) * squared_scale * squared_scale);
        if (!result.is_valid())
        {
            throw std::overflow_error("compound mass properties exceed the finite numeric range");
        }
        return result;
    }

} // namespace rigidbodies::physics
