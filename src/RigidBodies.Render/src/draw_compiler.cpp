#include <rigidbodies/render/draw_compiler.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <cstdint>
#include <numeric>

namespace rigidbodies::render
{
    namespace
    {
        constexpr double epsilon = 1.0e-8;

        Color premultiply(Color color, float coverage = 1.0f)
        {
            const auto unit = [](float value)
            {
                return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f;
            };
            const auto alpha = unit(color.alpha) * unit(coverage);
            return { unit(color.red) * alpha, unit(color.green) * alpha, unit(color.blue) * alpha, alpha };
        }

        void triangle(IndexedMesh& mesh, int a, int b, int c)
        {
            const auto area = math::cross(mesh.vertices[static_cast<std::size_t>(b)].position - mesh.vertices[static_cast<std::size_t>(a)].position,
                mesh.vertices[static_cast<std::size_t>(c)].position - mesh.vertices[static_cast<std::size_t>(a)].position);
            if (std::abs(area) <= epsilon)
                return;
            if (area < 0.0)
                std::swap(b, c);
            mesh.indices.insert(mesh.indices.end(), { a, b, c });
        }

        int vertex(IndexedMesh& mesh, Vec2 point, Color color, Vec2 uv = {})
        {
            const auto index = static_cast<int>(mesh.vertices.size());
            mesh.vertices.push_back({ point, uv, color });
            return index;
        }

        bool same_clip(const std::optional<MeshClip>& a, const std::optional<MeshClip>& b)
        {
            return a.has_value() == b.has_value() && (!a || (a->x == b->x && a->y == b->y && a->width == b->width && a->height == b->height));
        }

        std::optional<MeshClip> intersect_clip(const std::optional<MeshClip>& a, const std::optional<MeshClip>& b)
        {
            if (!a)
                return b;
            if (!b)
                return a;
            const auto x = std::max(a->x, b->x), y = std::max(a->y, b->y);
            const auto right = std::min(static_cast<long long>(a->x) + std::max(0, a->width), static_cast<long long>(b->x) + std::max(0, b->width));
            const auto bottom = std::min(static_cast<long long>(a->y) + std::max(0, a->height), static_cast<long long>(b->y) + std::max(0, b->height));
            return MeshClip { x, y, static_cast<int>(std::max(0LL, right - x)), static_cast<int>(std::max(0LL, bottom - y)) };
        }

        // Removes repeated points, and a closing duplicate, in place. Any nonfinite point
        // empties the path.
        void clean_points(std::vector<Vec2>& points, bool closed)
        {
            std::size_t kept = 0;
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const auto point = points[i];
                if (!math::is_finite(point))
                {
                    points.clear();
                    return;
                }
                if (kept == 0 || math::length_squared(point - points[kept - 1]) > epsilon * epsilon)
                    points[kept++] = point;
            }
            points.resize(kept);
            if (closed && points.size() > 1 && math::length_squared(points.front() - points.back()) < epsilon * epsilon)
                points.pop_back();
        }

        // Unit vectors at evenly spaced angles, advanced by complex rotation so that long arcs
        // cost one sine and cosine pair instead of one per vertex.
        struct ArcStepper
        {
            double c, s, step_c, step_s;
            ArcStepper(double start, double step) : c(std::cos(start)), s(std::sin(start)), step_c(std::cos(step)), step_s(std::sin(step))
            {
            }
            void advance()
            {
                const auto next_c = c * step_c - s * step_s;
                s = s * step_c + c * step_s;
                c = next_c;
            }
        };

        void circle_points(Vec2 center, float radius, float tolerance, std::vector<Vec2>& points)
        {
            points.clear();
            const auto count = curve_segment_count(radius, tolerance);
            points.reserve(static_cast<std::size_t>(count));
            ArcStepper arc { 0.0, math::two_pi / std::max(1, count) };
            for (int i = 0; i < count; ++i, arc.advance())
                points.push_back(center + Vec2 { arc.c, arc.s } * radius);
        }

        void rectangle_points(Vec2 a, Vec2 b, float radius, float tolerance, std::vector<Vec2>& points)
        {
            points.clear();
            const auto lo = math::min_components(a, b), hi = math::max_components(a, b);
            const auto r = std::clamp(static_cast<double>(radius), 0.0, std::min(hi.x - lo.x, hi.y - lo.y) * 0.5);
            if (!(r > epsilon))
            {
                points.insert(points.end(), { lo, { hi.x, lo.y }, hi, { lo.x, hi.y } });
                return;
            }
            const auto count = curve_segment_count(static_cast<float>(r), tolerance, static_cast<float>(math::pi * 0.5));
            const std::array<Vec2, 4> centers { Vec2 { hi.x - r, lo.y + r }, Vec2 { hi.x - r, hi.y - r }, Vec2 { lo.x + r, hi.y - r }, Vec2 { lo.x + r, lo.y + r } };
            points.reserve(static_cast<std::size_t>(4 * (count + 1)));
            for (int corner = 0; corner < 4; ++corner)
            {
                ArcStepper arc { (corner - 1) * math::pi * 0.5, math::pi * 0.5 / count };
                for (int i = 0; i <= count; ++i, arc.advance())
                    points.push_back(centers[static_cast<std::size_t>(corner)] + Vec2 { arc.c, arc.s } * r);
            }
            clean_points(points, true);
        }

        // The offset that keeps a band of unit width along both edges of a corner. Past the
        // limit it stays on the bisector, shortened, so a thin tip is never pushed along one edge.
        Vec2 miter_direction(Vec2 previous_normal, Vec2 next_normal, double limit = 4.0)
        {
            const auto sum = previous_normal + next_normal;
            const auto length = math::length(sum);
            if (!(length > 1.0e-9))
                return math::perpendicular(previous_normal);
            const auto bisector = sum / length;
            const auto cosine = math::dot(bisector, next_normal);
            return bisector * (cosine * limit > 1.0 ? 1.0 / cosine : limit);
        }

        Color point_color(const DrawCommand& command, Vec2 point)
        {
            if (command.kind != DrawCommandKind::gradient_polygon_fill && command.kind != DrawCommandKind::gradient_circle_fill)
                return command.color;
            const auto axis = command.gradient_to - command.gradient_from;
            const auto length = math::length_squared(axis);
            const auto fraction = length > epsilon ? std::clamp(math::dot(point - command.gradient_from, axis) / length, 0.0, 1.0) : 0.0;
            return mix(command.color, command.gradient_end, static_cast<float>(fraction));
        }

        // Convex fills have one opaque interior and one feather band. Reversed winding has the
        // same coverage; the band always expands outwards, including at rounded corners.
        // Consumes the outline; it is cleaned and reoriented in place.
        IndexedMesh fill_polygon(std::vector<Vec2>& points, const DrawCommand& command, double feather)
        {
            IndexedMesh mesh;
            clean_points(points, true);
            if (points.size() < 3)
                return mesh;
            double area = 0.0;
            for (std::size_t i = 0; i < points.size(); ++i)
                area += math::cross(points[i], points[(i + 1) % points.size()]);
            if (std::abs(area) < epsilon)
                return mesh;
            if (area < 0.0)
                std::reverse(points.begin(), points.end());
            mesh.vertices.reserve(points.size() * 2);
            mesh.indices.reserve(points.size() * 9);
            for (const auto& point : points)
                vertex(mesh, point, premultiply(point_color(command, point)));
            for (std::size_t i = 1; i + 1 < points.size(); ++i)
                triangle(mesh, 0, static_cast<int>(i), static_cast<int>(i + 1));
            if (feather <= 0.0)
                return mesh;
            // The band keeps one width all round. A sharp convex corner gets a rounded fan
            // rather than a long mitre, which would draw a wide blur as a spike off a thin tip.
            const auto count = static_cast<int>(points.size());
            thread_local std::vector<int> first_outer, last_outer;
            first_outer.assign(points.size(), 0);
            last_outer.assign(points.size(), 0);
            for (int i = 0; i < count; ++i)
            {
                const auto& point = points[static_cast<std::size_t>(i)];
                const auto previous = math::normalized(point - points[static_cast<std::size_t>((i + count - 1) % count)]);
                const auto next = math::normalized(points[static_cast<std::size_t>((i + 1) % count)] - point);
                const auto previous_normal = -math::perpendicular(previous), next_normal = -math::perpendicular(next);
                const auto clear = premultiply(point_color(command, point), 0.0f);
                const auto cosine = math::dot(previous_normal, next_normal);
                if (cosine >= -0.5 || math::cross(previous, next) <= 0.0)
                {
                    first_outer[static_cast<std::size_t>(i)] = last_outer[static_cast<std::size_t>(i)] = vertex(mesh, point + miter_direction(previous_normal, next_normal, 2.0) * feather, clear);
                    continue;
                }
                const auto turn = std::atan2(math::cross(previous_normal, next_normal), cosine);
                const auto steps = std::max(2, static_cast<int>(std::ceil(std::abs(turn) / (math::pi / 6.0))));
                first_outer[static_cast<std::size_t>(i)] = static_cast<int>(mesh.vertices.size());
                for (int step = 0; step <= steps; ++step)
                {
                    const auto angle = turn * static_cast<double>(step) / steps;
                    const auto c = std::cos(angle), s = std::sin(angle);
                    const Vec2 direction { previous_normal.x * c - previous_normal.y * s, previous_normal.x * s + previous_normal.y * c };
                    const auto outer = vertex(mesh, point + direction * feather, clear);
                    if (step > 0)
                        triangle(mesh, i, outer - 1, outer);
                }
                last_outer[static_cast<std::size_t>(i)] = static_cast<int>(mesh.vertices.size()) - 1;
            }
            for (int i = 0; i < count; ++i)
            {
                const auto next = (i + 1) % count;
                triangle(mesh, i, next, first_outer[static_cast<std::size_t>(next)]);
                triangle(mesh, i, first_outer[static_cast<std::size_t>(next)], last_outer[static_cast<std::size_t>(i)]);
            }
            return mesh;
        }

        struct Station
        {
            Vec2 left, right, outer_left, outer_right;
        };

        IndexedMesh stroke_path(const std::vector<Vec2>& source, const DrawCommand& command, const DrawCompileOptions& options, bool closed)
        {
            IndexedMesh mesh;
            thread_local std::vector<Vec2> points, directions;
            thread_local std::vector<Station> stations;
            points.assign(source.begin(), source.end());
            clean_points(points, closed);
            if (points.size() < 2 || !std::isfinite(command.thickness) || command.thickness <= 0.0f)
                return mesh;
            const auto half = static_cast<double>(command.thickness) * 0.5;
            const auto feather = std::max(0.0, static_cast<double>(options.feather));
            const auto outer = half + feather;
            const auto tolerance = std::isfinite(options.curve_tolerance) ? std::max(0.01, static_cast<double>(options.curve_tolerance)) : 0.25;
            directions.clear();
            directions.reserve(points.size());
            for (std::size_t i = 0; i + 1 < points.size(); ++i)
                directions.push_back(math::normalized(points[i + 1] - points[i]));
            if (closed)
                directions.push_back(math::normalized(points.front() - points.back()));
            if (!closed && command.cap == StrokeCap::square)
            {
                points.front() -= directions.front() * half;
                points.back() += directions.back() * half;
            }
            stations.clear();
            stations.reserve(points.size() + 8);
            const auto add_station = [&](Vec2 point, Vec2 left, Vec2 right)
            {
                stations.push_back({ point + left * half, point + right * half, point + left * outer, point + right * outer });
            };
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (!closed && (i == 0 || i + 1 == points.size()))
                {
                    const auto n = math::perpendicular(i == 0 ? directions.front() : directions.back());
                    add_station(points[i], n, -n);
                    continue;
                }
                const auto previous = directions[(i + directions.size() - 1) % directions.size()];
                const auto next = directions[i % directions.size()];
                const auto p = math::perpendicular(previous), n = math::perpendicular(next);
                const auto miter = miter_direction(p, n);
                const auto turn = math::cross(previous, next);
                if (std::abs(turn) < epsilon || command.join == StrokeJoin::miter)
                {
                    add_station(points[i], miter, -miter);
                    continue;
                }
                const auto side = turn > 0.0 ? -1.0 : 1.0;
                const auto start = p * side, finish = n * side;
                const auto angle = std::atan2(math::cross(start, finish), math::dot(start, finish));
                const auto segments = command.join == StrokeJoin::round ? curve_segment_count(static_cast<float>(outer), options.curve_tolerance, static_cast<float>(std::abs(angle))) : 1;
                // A one-segment round join is a bevel. Where the mitre tip stays within the curve
                // tolerance of the true arc, a single mitred station is indistinguishable from it.
                if (command.join == StrokeJoin::round && segments == 1 && outer * (1.0 / std::cos(angle * 0.5) - 1.0) <= tolerance)
                {
                    add_station(points[i], miter, -miter);
                    continue;
                }
                ArcStepper arc { 0.0, angle / segments };
                for (int j = 0; j <= segments; ++j, arc.advance())
                {
                    const Vec2 edge { start.x * arc.c - start.y * arc.s, start.x * arc.s + start.y * arc.c };
                    if (turn > 0.0)
                        add_station(points[i], miter, edge);
                    else
                        add_station(points[i], edge, -miter);
                }
            }
            mesh.vertices.reserve(stations.size() * 4 + 80);
            mesh.indices.reserve(stations.size() * 18 + 240);
            const auto color = premultiply(command.color), transparent = premultiply(command.color, 0.0f);
            for (const auto& station : stations)
            {
                vertex(mesh, station.left, color);
                vertex(mesh, station.right, color);
                vertex(mesh, station.outer_left, transparent);
                vertex(mesh, station.outer_right, transparent);
            }
            const auto count = static_cast<int>(stations.size());
            for (int i = 0; i < count - (closed ? 0 : 1); ++i)
            {
                const auto a = i * 4, b = ((i + 1) % count) * 4;
                triangle(mesh, a, a + 1, b + 1);
                triangle(mesh, a, b + 1, b);
                if (feather > 0.0)
                {
                    triangle(mesh, a, b, b + 2);
                    triangle(mesh, a, b + 2, a + 2);
                    triangle(mesh, a + 1, a + 3, b + 3);
                    triangle(mesh, a + 1, b + 3, b + 1);
                }
            }
            if (closed)
                return mesh;
            for (int end = 0; end < 2; ++end)
            {
                const auto center = end == 0 ? points.front() : points.back();
                const auto direction = (end == 0 ? -directions.front() : directions.back());
                const auto normal = math::perpendicular(direction);
                if (command.cap == StrokeCap::round)
                {
                    const auto segments = curve_segment_count(static_cast<float>(outer), options.curve_tolerance, static_cast<float>(math::pi));
                    const auto origin = vertex(mesh, center, color);
                    int previous_inner = -1, previous_outer = -1;
                    ArcStepper arc { -math::pi * 0.5, math::pi / segments };
                    for (int i = 0; i <= segments; ++i, arc.advance())
                    {
                        const auto radial = direction * arc.c + normal * arc.s;
                        const auto inner_index = vertex(mesh, center + radial * half, color);
                        const auto outer_index = vertex(mesh, center + radial * outer, transparent);
                        if (i > 0)
                        {
                            triangle(mesh, origin, previous_inner, inner_index);
                            if (feather > 0.0)
                            {
                                triangle(mesh, previous_inner, previous_outer, outer_index);
                                triangle(mesh, previous_inner, outer_index, inner_index);
                            }
                        }
                        previous_inner = inner_index;
                        previous_outer = outer_index;
                    }
                }
                else if (feather > 0.0)
                {
                    const auto a = vertex(mesh, center + normal * half, color), b = vertex(mesh, center - normal * half, color);
                    const auto c = vertex(mesh, center - normal * outer + direction * feather, transparent), d = vertex(mesh, center + normal * outer + direction * feather, transparent);
                    triangle(mesh, a, b, c);
                    triangle(mesh, a, c, d);
                }
            }
            return mesh;
        }

        void append_into(IndexedMesh& target, const IndexedMesh& source)
        {
            if (source.indices.empty())
                return;
            const auto base = static_cast<int>(target.vertices.size());
            target.vertices.insert(target.vertices.end(), source.vertices.begin(), source.vertices.end());
            target.indices.reserve(target.indices.size() + source.indices.size());
            for (const auto index : source.indices)
                target.indices.push_back(base + index);
        }

        // Splits a stroke into dashes measured along the cleaned path, stroking every dash into
        // one mesh so a dashed outline costs one command and one batch like a solid one.
        IndexedMesh stroke_dashed(const std::vector<Vec2>& source, const DrawCommand& command, const DrawCompileOptions& options, bool closed)
        {
            IndexedMesh mesh;
            thread_local std::vector<Vec2> path, dash;
            path.assign(source.begin(), source.end());
            clean_points(path, closed);
            if (path.size() < 2)
                return mesh;
            if (closed)
                path.push_back(path.front());
            const auto on = static_cast<double>(command.dash_length), off = static_cast<double>(command.gap_length);
            double total = 0.0;
            for (std::size_t i = 0; i + 1 < path.size(); ++i)
                total += math::length(path[i + 1] - path[i]);
            if (!std::isfinite(total) || total <= epsilon || total / (on + off) > 4096.0)
                return stroke_path(source, command, options, closed);
            DrawCommand piece = command;
            piece.dash_length = piece.gap_length = 0.0f;
            bool drawing = true;
            auto remaining = on;
            dash.clear();
            dash.push_back(path.front());
            for (std::size_t i = 0; i + 1 < path.size(); ++i)
            {
                const auto length = math::length(path[i + 1] - path[i]);
                if (length <= epsilon)
                    continue;
                const auto direction = (path[i + 1] - path[i]) / length;
                double travelled = 0.0;
                while (length - travelled > remaining)
                {
                    travelled += remaining;
                    const auto point = path[i] + direction * travelled;
                    if (drawing)
                    {
                        dash.push_back(point);
                        append_into(mesh, stroke_path(dash, piece, options, false));
                    }
                    dash.clear();
                    dash.push_back(point);
                    drawing = !drawing;
                    remaining = drawing ? on : off;
                }
                remaining -= length - travelled;
                if (drawing)
                    dash.push_back(path[i + 1]);
            }
            if (drawing && dash.size() >= 2)
                append_into(mesh, stroke_path(dash, piece, options, false));
            return mesh;
        }

        // An open butt-ended ribbon whose half-width and premultiplied colour interpolate by arc
        // length. Interior stations use a limited mitre so sharp reversals stay bounded.
        IndexedMesh taper_path(const std::vector<Vec2>& source, const DrawCommand& command, const DrawCompileOptions& options)
        {
            IndexedMesh mesh;
            thread_local std::vector<Vec2> points;
            thread_local std::vector<double> distances;
            points.assign(source.begin(), source.end());
            clean_points(points, false);
            if (points.size() < 2)
                return mesh;
            // Trails are sampled once per physics step, so a slow body records many points inside
            // one device pixel; they add vertices without changing the picture.
            if (points.size() > 2)
            {
                std::size_t kept = 1;
                for (std::size_t i = 1; i + 1 < points.size(); ++i)
                    if (math::length_squared(points[i] - points[kept - 1]) >= 1.0)
                        points[kept++] = points[i];
                points[kept++] = points.back();
                points.resize(kept);
            }
            const auto width = [](float value)
            {
                return std::isfinite(value) ? std::max(0.0, static_cast<double>(value)) : 0.0;
            };
            const auto start_width = width(command.thickness), end_width = width(command.radius);
            const auto feather = std::max(0.0, static_cast<double>(options.feather));
            distances.assign(points.size(), 0.0);
            for (std::size_t i = 1; i < points.size(); ++i)
                distances[i] = distances[i - 1] + math::length(points[i] - points[i - 1]);
            const auto total = distances.back();
            if (!std::isfinite(total) || total <= epsilon)
                return mesh;
            mesh.vertices.reserve(points.size() * 4 + 8);
            mesh.indices.reserve(points.size() * 18 + 12);
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const auto previous = math::normalized(points[i] - points[i == 0 ? 0 : i - 1]);
                const auto next = math::normalized(points[i + 1 < points.size() ? i + 1 : i] - points[i]);
                const auto tangent_previous = i == 0 ? next : previous;
                const auto tangent_next = i + 1 == points.size() ? previous : next;
                auto normal = miter_direction(math::perpendicular(tangent_previous), math::perpendicular(tangent_next));
                if (math::length_squared(normal) > 4.0)
                    normal = math::normalized(normal) * 2.0;
                const auto fraction = static_cast<float>(distances[i] / total);
                const auto half = (start_width + (end_width - start_width) * fraction) * 0.5;
                const auto color = premultiply(mix(command.color, command.gradient_end, fraction));
                const auto transparent = premultiply(mix(command.color, command.gradient_end, fraction), 0.0f);
                vertex(mesh, points[i] + normal * half, color);
                vertex(mesh, points[i] - normal * half, color);
                vertex(mesh, points[i] + normal * (half + feather), transparent);
                vertex(mesh, points[i] - normal * (half + feather), transparent);
            }
            const auto count = static_cast<int>(points.size());
            for (int i = 0; i + 1 < count; ++i)
            {
                const auto a = i * 4, b = (i + 1) * 4;
                triangle(mesh, a, a + 1, b + 1);
                triangle(mesh, a, b + 1, b);
                if (feather > 0.0)
                {
                    triangle(mesh, a, b, b + 2);
                    triangle(mesh, a, b + 2, a + 2);
                    triangle(mesh, a + 1, a + 3, b + 3);
                    triangle(mesh, a + 1, b + 3, b + 1);
                }
            }
            if (feather > 0.0)
                for (int end = 0; end < 2; ++end)
                {
                    const auto station = end == 0 ? 0 : (count - 1) * 4;
                    const auto direction = end == 0 ? -math::normalized(points[1] - points[0]) : math::normalized(points[static_cast<std::size_t>(count - 1)] - points[static_cast<std::size_t>(count - 2)]);
                    const auto shift = direction * feather;
                    const auto transparent = mesh.vertices[static_cast<std::size_t>(station + 2)].color;
                    const auto left = vertex(mesh, mesh.vertices[static_cast<std::size_t>(station)].position + shift, transparent);
                    const auto right = vertex(mesh, mesh.vertices[static_cast<std::size_t>(station + 1)].position + shift, transparent);
                    triangle(mesh, station, station + 1, right);
                    triangle(mesh, station, right, left);
                }
            return mesh;
        }

        IndexedMesh shadow_polygon(std::vector<Vec2>& points, const DrawCommand& command)
        {
            for (auto& point : points)
                point += command.shadow_offset;
            // A wide smooth band falls to zero without nested translucent polygons, avoiding
            // darker seams when the shadow is composited over the background.
            return fill_polygon(points, command, std::max(0.0f, command.blur_radius));
        }

        bool dashed(const DrawCommand& command)
        {
            return std::isfinite(command.dash_length) && std::isfinite(command.gap_length) && command.dash_length > 0.0f && command.gap_length > 0.0f;
        }

        void feather_mesh_boundary(IndexedMesh& mesh, float feather)
        {
            if (mesh.texture || mesh.feathered || feather <= 0.0f || mesh.vertices.empty() || mesh.indices.size() % 3 != 0)
                return;
            // Document and authored meshes may repeat vertices across triangles. Weld by exact
            // position for topology only, preserving each original colour in the visible mesh.
            // Interior edges cancel; outer and hole contours retain their oriented boundary.
            // Welded positions are numbered by first appearance and edges visited in welded
            // order, so the generated band is identical however the input is laid out.
            const auto vertex_count = mesh.vertices.size();
            thread_local std::vector<int> order, first_of, welded;
            order.resize(vertex_count);
            for (std::size_t i = 0; i < vertex_count; ++i)
            {
                if (!math::is_finite(mesh.vertices[i].position))
                    return;
                order[i] = static_cast<int>(i);
            }
            const auto position = [&](int index) -> const Vec2&
            {
                return mesh.vertices[static_cast<std::size_t>(index)].position;
            };
            std::sort(order.begin(), order.end(), [&](int a, int b)
                {
                    const auto& p = position(a);
                    const auto& q = position(b);
                    if (p.x != q.x)
                        return p.x < q.x;
                    if (p.y != q.y)
                        return p.y < q.y;
                    return a < b;
                });
            first_of.resize(vertex_count);
            for (std::size_t i = 0; i < vertex_count; ++i)
            {
                const auto index = order[i];
                const auto repeated = i > 0 && position(order[i - 1]).x == position(index).x && position(order[i - 1]).y == position(index).y;
                first_of[static_cast<std::size_t>(index)] = repeated ? first_of[static_cast<std::size_t>(order[i - 1])] : index;
            }
            welded.resize(vertex_count);
            int unique = 0;
            for (std::size_t i = 0; i < vertex_count; ++i)
            {
                const auto first = static_cast<std::size_t>(first_of[i]);
                welded[i] = first == i ? unique++ : welded[first];
            }
            struct Edge
            {
                std::uint64_t key;
                int sequence, from, to, original_from, original_to;
                Vec2 normal;
            };
            thread_local std::vector<Edge> edges;
            edges.clear();
            edges.reserve(mesh.indices.size());
            for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
            {
                std::array<int, 3> indices { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
                if (std::any_of(indices.begin(), indices.end(), [&](int index)
                        {
                            return index < 0 || static_cast<std::size_t>(index) >= mesh.vertices.size();
                        }))
                    return;
                const auto a = mesh.vertices[static_cast<std::size_t>(indices[0])].position;
                const auto b = mesh.vertices[static_cast<std::size_t>(indices[1])].position;
                const auto c = mesh.vertices[static_cast<std::size_t>(indices[2])].position;
                const auto area = math::cross(b - a, c - a);
                if (std::abs(area) <= epsilon)
                    continue;
                if (area < 0.0)
                    std::swap(indices[1], indices[2]);
                for (int side = 0; side < 3; ++side)
                {
                    const auto original_from = indices[static_cast<std::size_t>(side)], original_to = indices[static_cast<std::size_t>((side + 1) % 3)];
                    const auto from = welded[static_cast<std::size_t>(original_from)], to = welded[static_cast<std::size_t>(original_to)];
                    if (from == to)
                        continue;
                    const auto key = (static_cast<std::uint64_t>(std::min(from, to)) << 32u) | static_cast<std::uint32_t>(std::max(from, to));
                    const auto normal = -math::perpendicular(math::normalized(mesh.vertices[static_cast<std::size_t>(original_to)].position - mesh.vertices[static_cast<std::size_t>(original_from)].position));
                    edges.push_back({ key, static_cast<int>(edges.size()), from, to, original_from, original_to, normal });
                }
            }
            std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b)
                {
                    return a.key != b.key ? a.key < b.key : a.sequence < b.sequence;
                });
            const auto exposed = [&](std::size_t i)
            {
                return (i == 0 || edges[i - 1].key != edges[i].key) && (i + 1 == edges.size() || edges[i + 1].key != edges[i].key);
            };
            // An edge running along a pixel boundary already rasterises crisply; a band there
            // would only blur it. Interface rules, panel edges and dividers are such edges.
            // Instanced vertices are local units, so only screen-space meshes qualify.
            const auto on_grid = [](double value)
            {
                return std::abs(value - std::round(value)) <= 1.0e-3;
            };
            const auto snapped = mesh.pixel_aligned && mesh.instances.empty();
            const auto crisp = [&](const Edge& edge)
            {
                if (!snapped)
                    return false;
                const auto& from = mesh.vertices[static_cast<std::size_t>(edge.original_from)].position;
                const auto& to = mesh.vertices[static_cast<std::size_t>(edge.original_to)].position;
                return (from.x == to.x && on_grid(from.x)) || (from.y == to.y && on_grid(from.y));
            };
            const auto feathered_edge = [&](std::size_t i)
            {
                return exposed(i) && !crisp(edges[i]);
            };
            thread_local std::vector<Vec2> normal_sums;
            normal_sums.assign(static_cast<std::size_t>(unique), Vec2 {});
            std::size_t exposed_count = 0;
            for (std::size_t i = 0; i < edges.size(); ++i)
                if (feathered_edge(i))
                {
                    normal_sums[static_cast<std::size_t>(edges[i].from)] += edges[i].normal;
                    normal_sums[static_cast<std::size_t>(edges[i].to)] += edges[i].normal;
                    ++exposed_count;
                }
            mesh.vertices.reserve(mesh.vertices.size() + exposed_count * 2);
            mesh.indices.reserve(mesh.indices.size() + exposed_count * 6);
            for (std::size_t i = 0; i < edges.size(); ++i)
            {
                if (!feathered_edge(i))
                    continue;
                const auto& edge = edges[i];
                // A mesh already produced by this compiler (or a document shadow) exposes a
                // zero-alpha perimeter. Never grow it again or brighten its existing feather.
                const auto from = mesh.vertices[static_cast<std::size_t>(edge.original_from)];
                const auto to = mesh.vertices[static_cast<std::size_t>(edge.original_to)];
                if (from.color.alpha <= 0.0f && to.color.alpha <= 0.0f)
                    continue;
                const auto expanded = [&](Vec2 point, int welded_index)
                {
                    const auto sum = normal_sums[static_cast<std::size_t>(welded_index)];
                    const auto divisor = math::dot(sum, edge.normal);
                    const auto direction = divisor > 0.125 ? sum / divisor : edge.normal * 4.0;
                    return point + direction * feather;
                };
                const auto a = vertex(mesh, expanded(from.position, edge.from), { 0, 0, 0, 0 });
                const auto b = vertex(mesh, expanded(to.position, edge.to), { 0, 0, 0, 0 });
                triangle(mesh, edge.original_from, edge.original_to, b);
                triangle(mesh, edge.original_from, b, a);
            }
        }

        void append_mesh(CompiledDrawList& result, IndexedMesh mesh, const DrawCompileOptions& options, bool barrier)
        {
            if (mesh.indices.empty() || mesh.vertices.empty() || (mesh.clip && (mesh.clip->width <= 0 || mesh.clip->height <= 0)))
                return;
            if (mesh.texture && (mesh.texture->width <= 0 || mesh.texture->height <= 0 || mesh.texture->width > 16384 || mesh.texture->height > 16384 || mesh.texture->rgba.size() != static_cast<std::size_t>(mesh.texture->width) * static_cast<std::size_t>(mesh.texture->height) * 4))
                return;
            if (mesh.indices.size() % 3 != 0 || std::any_of(mesh.indices.begin(), mesh.indices.end(), [&](int index)
                                                    {
                                                        return index < 0 || static_cast<std::size_t>(index) >= mesh.vertices.size();
                                                    }))
                return;
            if (std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [](const MeshVertex& value)
                    {
                        return !math::is_finite(value.position) || !math::is_finite(value.uv) || std::abs(value.position.x) > std::numeric_limits<float>::max() || std::abs(value.position.y) > std::numeric_limits<float>::max() || !std::isfinite(value.color.red) || !std::isfinite(value.color.green) || !std::isfinite(value.color.blue) || !std::isfinite(value.color.alpha);
                    }))
                return;
            if (!mesh.instances.empty())
            {
                mesh.instances.erase(std::remove_if(mesh.instances.begin(), mesh.instances.end(), [](const MeshInstance& instance)
                                         {
                                             return !math::is_finite(instance.translation) || !math::is_finite(instance.scale) || !std::isfinite(instance.rotation) || !std::isfinite(instance.tint.red) || !std::isfinite(instance.tint.green) || !std::isfinite(instance.tint.blue) || !std::isfinite(instance.tint.alpha);
                                         }),
                    mesh.instances.end());
                // An all-invalid instance set means no draw, never an accidental origin copy.
                if (mesh.instances.empty())
                    return;
            }
            if (!barrier && !result.meshes.empty())
            {
                auto& previous = result.meshes.back();
                if (previous.instances.empty() && mesh.instances.empty() && previous.texture == mesh.texture && same_clip(previous.clip, mesh.clip) && previous.vertices.size() + mesh.vertices.size() <= std::min(options.max_batch_vertices, static_cast<std::size_t>(std::numeric_limits<int>::max())))
                {
                    const auto base = static_cast<int>(previous.vertices.size());
                    previous.vertices.insert(previous.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
                    for (const auto index : mesh.indices)
                        previous.indices.push_back(base + index);
                    return;
                }
            }
            result.meshes.push_back(std::move(mesh));
        }

        // An untextured, uninstanced mesh that already carries its own rim is validated in
        // place and copied once, straight into the current batch where painter order allows.
        void append_borrowed(CompiledDrawList& result, const IndexedMesh& mesh, const std::optional<MeshClip>& clip, const DrawCompileOptions& options, bool barrier)
        {
            if (mesh.indices.empty() || mesh.vertices.empty() || (clip && (clip->width <= 0 || clip->height <= 0)) || mesh.indices.size() % 3 != 0)
                return;
            const auto count = mesh.vertices.size();
            for (const auto index : mesh.indices)
                if (index < 0 || static_cast<std::size_t>(index) >= count)
                    return;
            for (const auto& value : mesh.vertices)
                if (!math::is_finite(value.position) || !math::is_finite(value.uv) || std::abs(value.position.x) > std::numeric_limits<float>::max() || std::abs(value.position.y) > std::numeric_limits<float>::max() || !std::isfinite(value.color.red) || !std::isfinite(value.color.green) || !std::isfinite(value.color.blue) || !std::isfinite(value.color.alpha))
                    return;
            if (!barrier && !result.meshes.empty())
            {
                auto& previous = result.meshes.back();
                if (previous.instances.empty() && !previous.texture && same_clip(previous.clip, clip) && previous.vertices.size() + count <= std::min(options.max_batch_vertices, static_cast<std::size_t>(std::numeric_limits<int>::max())))
                {
                    const auto base = static_cast<int>(previous.vertices.size());
                    previous.vertices.insert(previous.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
                    previous.indices.reserve(previous.indices.size() + mesh.indices.size());
                    for (const auto index : mesh.indices)
                        previous.indices.push_back(base + index);
                    return;
                }
            }
            IndexedMesh copy;
            copy.vertices = mesh.vertices;
            copy.indices = mesh.indices;
            copy.clip = clip;
            result.meshes.push_back(std::move(copy));
        }
    }

    int curve_segment_count(float radius, float tolerance, float arc_radians)
    {
        if (!std::isfinite(radius) || !std::isfinite(arc_radians) || radius <= 0.0f || arc_radians <= 0.0f)
            return 0;
        const auto error = std::isfinite(tolerance) ? std::max(0.01, static_cast<double>(tolerance)) : 0.25;
        const auto arc = std::min(math::two_pi, static_cast<double>(arc_radians));
        const auto angle = 2.0 * std::acos(std::clamp(1.0 - error / radius, -1.0, 1.0));
        const auto full_circle = arc > math::two_pi - 1.0e-5;
        const auto count = std::ceil(arc / std::max(1.0e-9, angle));
        return static_cast<int>(std::clamp(count, full_circle ? 3.0 : 1.0, 4096.0));
    }

    CompiledDrawList compile_draw_list(const DrawList& list, const TextMeshBuilder& text_builder, const DrawCompileOptions& requested_options)
    {
        auto options = requested_options;
        if (!std::isfinite(options.feather))
            options.feather = 1.0f;
        options.feather = std::clamp(options.feather, 0.0f, 16.0f);
        CompiledDrawList result;
        result.stats.command_count = list.commands().size();
        std::vector<std::size_t> order(list.commands().size());
        std::iota(order.begin(), order.end(), std::size_t { 0 });
        std::stable_sort(order.begin(), order.end(), [&](auto a, auto b)
            {
                return list.commands()[a].layer < list.commands()[b].layer;
            });
        std::optional<int> last_layer;
        for (const auto index : order)
        {
            const auto& command = list.commands()[index];
            const auto barrier = !last_layer || *last_layer != command.layer;
            last_layer = command.layer;
            if (command.clip && (command.clip->width <= 0 || command.clip->height <= 0))
                continue;
            if (command.vertex_offset > list.vertices().size() || command.vertex_count > list.vertices().size() - command.vertex_offset)
                continue;
            thread_local std::vector<Vec2> points, outline;
            points.assign(list.vertices().begin() + static_cast<std::ptrdiff_t>(command.vertex_offset), list.vertices().begin() + static_cast<std::ptrdiff_t>(command.vertex_offset + command.vertex_count));
            if (std::any_of(points.begin(), points.end(), [](Vec2 point)
                    {
                        return !math::is_finite(point);
                    }))
                continue;
            IndexedMesh mesh;
            switch (command.kind)
            {
            case DrawCommandKind::indexed_mesh:
            case DrawCommandKind::instanced_mesh:
                if (command.mesh)
                {
                    if (command.mesh->feathered && command.mesh->instances.empty() && !command.mesh->texture)
                    {
                        append_borrowed(result, *command.mesh, intersect_clip(command.mesh->clip, command.clip), options, barrier);
                        continue;
                    }
                    mesh = *command.mesh;
                    if (!(mesh.pixel_aligned && options.multisampled))
                        feather_mesh_boundary(mesh, options.feather);
                    if (!mesh.instances.empty() && mesh.vertices.size() != command.mesh->vertices.size())
                    {
                        // A newly generated feather is a screen-space width, not a local-unit
                        // decoration. Bake each distinct scale into the prototype before adding
                        // its band, then retain translation, rotation, reflection and instancing.
                        // Only consecutive matching scales are grouped, preserving transparent
                        // painter order even for a mixed 100:1 scale range. Soft particle meshes
                        // already have a transparent boundary and never take this path.
                        std::size_t first = 0;
                        auto group_barrier = barrier;
                        while (first < command.mesh->instances.size())
                        {
                            const auto& instance = command.mesh->instances[first];
                            const Vec2 scale { std::abs(instance.scale.x), std::abs(instance.scale.y) };
                            auto last = first + 1;
                            while (last < command.mesh->instances.size() && std::abs(command.mesh->instances[last].scale.x) == scale.x && std::abs(command.mesh->instances[last].scale.y) == scale.y)
                                ++last;
                            auto group = *command.mesh;
                            group.instances.assign(command.mesh->instances.begin() + static_cast<std::ptrdiff_t>(first), command.mesh->instances.begin() + static_cast<std::ptrdiff_t>(last));
                            for (auto& value : group.vertices)
                                value.position = { value.position.x * scale.x, value.position.y * scale.y };
                            for (auto& value : group.instances)
                                value.scale = { std::copysign(1.0, value.scale.x), std::copysign(1.0, value.scale.y) };
                            feather_mesh_boundary(group, options.feather);
                            group.clip = intersect_clip(group.clip, command.clip);
                            append_mesh(result, std::move(group), options, group_barrier);
                            group_barrier = false;
                            first = last;
                        }
                        continue;
                    }
                }
                break;
            case DrawCommandKind::text:
                if (text_builder && !points.empty() && command.text_offset <= list.text_buffer().size() && command.text_length <= list.text_buffer().size() - command.text_offset)
                {
                    auto text_meshes = text_builder(points.front(), std::string_view(list.text_buffer()).substr(command.text_offset, command.text_length), command.color, command.text_scale);
                    auto text_barrier = barrier;
                    if (command.text_background)
                    {
                        Vec2 minimum { std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
                        Vec2 maximum = -minimum;
                        bool has_ink = false;
                        for (const auto& glyphs : text_meshes)
                            for (const auto& value : glyphs.vertices)
                                if (math::is_finite(value.position))
                                {
                                    minimum = math::min_components(minimum, value.position);
                                    maximum = math::max_components(maximum, value.position);
                                    has_ink = true;
                                }
                        if (has_ink)
                        {
                            const auto padding_scale = std::isfinite(command.text_scale) ? std::clamp(command.text_scale, 0.5f, 8.0f) : 1.0f;
                            const auto padding = (std::isfinite(command.text_padding) ? std::clamp(command.text_padding, 0.0f, 32.0f) : 4.0f) * padding_scale;
                            DrawCommand plate;
                            plate.color = *command.text_background;
                            rectangle_points(minimum - Vec2 { padding, padding }, maximum + Vec2 { padding, padding }, padding + 2.0f * padding_scale, options.curve_tolerance, outline);
                            auto background = fill_polygon(outline, plate, options.feather);
                            background.clip = command.clip;
                            append_mesh(result, std::move(background), options, text_barrier);
                            text_barrier = false;
                        }
                    }
                    for (auto& glyphs : text_meshes)
                    {
                        glyphs.clip = intersect_clip(glyphs.clip, command.clip);
                        append_mesh(result, std::move(glyphs), options, text_barrier);
                        text_barrier = false;
                    }
                }
                continue;
            case DrawCommandKind::line:
            case DrawCommandKind::polyline:
            case DrawCommandKind::polygon_outline:
            {
                const auto closed = command.closed || command.kind == DrawCommandKind::polygon_outline || (points.size() > 2 && points.front() == points.back());
                mesh = dashed(command) ? stroke_dashed(points, command, options, closed) : stroke_path(points, command, options, closed);
                break;
            }
            case DrawCommandKind::tapered_polyline:
                mesh = taper_path(points, command, options);
                break;
            case DrawCommandKind::circle_fill:
            case DrawCommandKind::gradient_circle_fill:
            case DrawCommandKind::circle_outline:
                if (!points.empty())
                {
                    circle_points(points.front(), command.radius, options.curve_tolerance, outline);
                    if (command.kind == DrawCommandKind::circle_outline)
                        mesh = dashed(command) ? stroke_dashed(outline, command, options, true) : stroke_path(outline, command, options, true);
                    else
                        mesh = fill_polygon(outline, command, options.feather);
                }
                break;
            case DrawCommandKind::rectangle_fill:
            case DrawCommandKind::rectangle_outline:
            case DrawCommandKind::rounded_rectangle_fill:
            case DrawCommandKind::rounded_rectangle_outline:
            case DrawCommandKind::rounded_rectangle_shadow:
            case DrawCommandKind::textured_quad:
                if (points.size() >= 2)
                {
                    const auto minimum = math::min_components(points[0], points[1]), maximum = math::max_components(points[0], points[1]);
                    rectangle_points(minimum, maximum, command.radius, options.curve_tolerance, outline);
                    if (command.kind == DrawCommandKind::rectangle_outline || command.kind == DrawCommandKind::rounded_rectangle_outline)
                        mesh = dashed(command) ? stroke_dashed(outline, command, options, true) : stroke_path(outline, command, options, true);
                    else if (command.kind == DrawCommandKind::rounded_rectangle_shadow)
                        mesh = shadow_polygon(outline, command);
                    else
                        mesh = fill_polygon(outline, command, options.feather);
                    if (command.kind == DrawCommandKind::textured_quad)
                    {
                        mesh.texture = command.texture;
                        for (auto& value : mesh.vertices)
                        {
                            const auto u = maximum.x > minimum.x ? (value.position.x - minimum.x) / (maximum.x - minimum.x) : 0.0;
                            const auto v = maximum.y > minimum.y ? (value.position.y - minimum.y) / (maximum.y - minimum.y) : 0.0;
                            value.uv = { command.uv_min.x + std::clamp(u, 0.0, 1.0) * (command.uv_max.x - command.uv_min.x), command.uv_min.y + std::clamp(v, 0.0, 1.0) * (command.uv_max.y - command.uv_min.y) };
                        }
                    }
                }
                break;
            case DrawCommandKind::polygon_fill:
            case DrawCommandKind::gradient_polygon_fill:
                mesh = fill_polygon(points, command, options.feather);
                break;
            case DrawCommandKind::shadow:
                mesh = shadow_polygon(points, command);
                break;
            }
            mesh.clip = intersect_clip(mesh.clip, command.clip);
            append_mesh(result, std::move(mesh), options, barrier);
        }
        result.stats.mesh_count = result.meshes.size();
        for (const auto& mesh : result.meshes)
            result.stats.triangle_count += mesh.indices.size() / 3 * std::max(std::size_t { 1 }, mesh.instances.size());
        return result;
    }
}
