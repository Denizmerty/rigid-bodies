#include <rigidbodies/render/scene_renderer.hpp>

#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/force_generator.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/render/draw_compiler.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>
#include <tuple>

namespace rigidbodies::render
{
    namespace
    {

        using physics::BodyType;
        using math::Real;

        // The stage is lit from the upper left. Screen y grows downward.
        const Vec2 toward_light = math::normalized(Vec2 { -0.55, -1.0 });

        double vector_length(const Vec2& value)
        {
            return std::hypot(value.x, value.y);
        }

        float pixel_scale(const SceneRenderSettings& settings)
        {
            return std::isfinite(settings.display_scale) ? std::clamp(settings.display_scale, 0.5f, 4.0f) : 1.0f;
        }

        // Stroke weights follow the house hierarchy (hairline 1, standard 1.5, emphasis 2
        // logical pixels), scaled for density and for the projector's heavier lines.
        float stroke(const SceneRenderSettings& settings, float logical)
        {
            const auto weight = std::isfinite(settings.theme.stroke_weight) ? std::clamp(settings.theme.stroke_weight, 0.5f, 3.0f) : 1.0f;
            return logical * pixel_scale(settings) * weight;
        }

        double smoothstep(double edge0, double edge1, double value)
        {
            const auto t = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
            return t * t * (3.0 - 2.0 * t);
        }

        Color premultiplied(const Color& color)
        {
            const auto unit = [](float value)
            {
                return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f;
            };
            const auto alpha = unit(color.alpha);
            return { unit(color.red) * alpha, unit(color.green) * alpha, unit(color.blue) * alpha, alpha };
        }

        float luminance(const Color& color)
        {
            return 0.2126f * color.red + 0.7152f * color.green + 0.0722f * color.blue;
        }

        Color toward(const Color& color, float red, float green, float blue, float fraction)
        {
            return mix(color, Color { red, green, blue, color.alpha }, fraction);
        }

        // ---------------------------------------------------------------------------------
        // Typography. Scene text uses Inter Medium through the device atlas, which rasterises
        // at round(14 * scale) pixels with hinted advances and tabular digits. Mirroring those
        // metrics lets the renderer reserve label space without a device round trip.

        constexpr std::array<std::uint16_t, 95> inter_medium_advances {
            546, 623, 1012, 1308, 1323, 2034, 1338, 641, 755, 755, 1066, 1367, 621, 947, 621, 757, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 621, 646, 1367, 1367, 1367, 1080, 2012, 1452, 1345, 1502, 1478, 1235, 1207, 1531, 1525, 558, 1178, 1408, 1158, 1869, 1549, 1570, 1314, 1574, 1327, 1323, 1337, 1516, 1452, 2054, 1435, 1426, 1312, 755, 757, 755, 976, 948, 690, 1163, 1266, 1182, 1266, 1203, 777, 1269, 1232, 516, 516, 1145, 516, 1819, 1232, 1237, 1266, 1266, 792, 1103, 697, 1232, 1177, 1698, 1141, 1178, 1145, 902, 708, 902, 1367
        };

        // Annotations are 12 logical pixels against the atlas' 14 pixel base.
        constexpr float label_text_ratio = 12.0f / 14.0f;

        // Density times the reader's text-size preference: what stage text and its plates scale by.
        float text_pixel_scale(const SceneRenderSettings& settings)
        {
            const auto preference = std::isfinite(settings.text_scale) ? std::clamp(settings.text_scale, 0.5f, 2.0f) : 1.0f;
            return pixel_scale(settings) * preference;
        }

        float label_scale(const SceneRenderSettings& settings)
        {
            return text_pixel_scale(settings) * label_text_ratio;
        }

        int font_pixels(float scale)
        {
            return static_cast<int>(std::lround(std::clamp(std::isfinite(scale) ? scale * 14.0f : 14.0f, 7.0f, 112.0f)));
        }

        std::uint16_t advance_units(std::uint32_t code)
        {
            if (code >= 32 && code < 127)
                return inter_medium_advances[code - 32];
            switch (code)
            {
            case 0xa0:
                return 546;
            case 0xb7:
                return 621;
            case 0xb2:
            case 0xb3:
            case 0xb9:
            case 0x2070:
            case 0x2074:
            case 0x2075:
            case 0x2076:
            case 0x2077:
            case 0x2078:
            case 0x2079:
            case 0x207b:
                return 930;
            case 0xb0:
                return 936;
            case 0xd7:
            case 0x2212:
                return 1367;
            case 0x2013:
                return 1024;
            case 0x2014:
                return 2048;
            default:
                return 1240;
            }
        }

        double text_width(std::string_view text, float scale)
        {
            const auto size = static_cast<double>(font_pixels(scale));
            double width = 0.0;
            for (std::size_t index = 0; index < text.size();)
            {
                const auto first = static_cast<unsigned char>(text[index++]);
                std::uint32_t code = first;
                int extra = first >= 0xf0 ? 3 : first >= 0xe0 ? 2
                    : first >= 0xc0                           ? 1
                                                              : 0;
                if (extra > 0)
                    code = first & (extra == 3 ? 0x07u : extra == 2 ? 0x0fu
                                                                    : 0x1fu);
                for (; extra > 0 && index < text.size(); --extra)
                    code = (code << 6u) | (static_cast<unsigned char>(text[index++]) & 0x3fu);
                width += std::round(static_cast<double>(advance_units(code)) * size / 2048.0);
            }
            return width;
        }

        struct LabelMetrics
        {
            float scale {};
            double size {}, ascender {}, cap_height {}, height {}, pad_x {}, gap {}, radius {};
        };

        LabelMetrics label_metrics(const SceneRenderSettings& settings)
        {
            LabelMetrics result;
            const auto ds = static_cast<double>(text_pixel_scale(settings));
            result.scale = label_scale(settings);
            result.size = static_cast<double>(font_pixels(result.scale));
            result.ascender = std::ceil(result.size * 1984.0 / 2048.0);
            result.cap_height = result.size * 1490.0 / 2048.0;
            result.height = std::round(result.size + 7.0 * ds);
            result.pad_x = std::round(6.0 * ds);
            result.gap = 6.0 * ds;
            result.radius = 4.0 * ds;
            return result;
        }

        // ---------------------------------------------------------------------------------

        Theme interpolate_theme(const Theme& from, const Theme& to, float fraction)
        {
            Theme value;
#define RIGIDBODIES_BLEND_COLOR(member) value.member = mix(from.member, to.member, fraction)
            RIGIDBODIES_BLEND_COLOR(background);
            RIGIDBODIES_BLEND_COLOR(stage_highlight);
            RIGIDBODIES_BLEND_COLOR(stage_vignette);
            RIGIDBODIES_BLEND_COLOR(grid_minor);
            RIGIDBODIES_BLEND_COLOR(grid_major);
            RIGIDBODIES_BLEND_COLOR(axis);
            RIGIDBODIES_BLEND_COLOR(body_fill);
            RIGIDBODIES_BLEND_COLOR(body_outline);
            RIGIDBODIES_BLEND_COLOR(static_body_fill);
            RIGIDBODIES_BLEND_COLOR(static_body_outline);
            RIGIDBODIES_BLEND_COLOR(kinematic_body_fill);
            RIGIDBODIES_BLEND_COLOR(kinematic_body_outline);
            RIGIDBODIES_BLEND_COLOR(ground_surface);
            RIGIDBODIES_BLEND_COLOR(ground_hatch);
            RIGIDBODIES_BLEND_COLOR(selection);
            RIGIDBODIES_BLEND_COLOR(center_of_mass);
            RIGIDBODIES_BLEND_COLOR(center_of_mass_ink);
            RIGIDBODIES_BLEND_COLOR(shadow);
            RIGIDBODIES_BLEND_COLOR(velocity);
            RIGIDBODIES_BLEND_COLOR(acceleration);
            RIGIDBODIES_BLEND_COLOR(force);
            RIGIDBODIES_BLEND_COLOR(momentum);
            RIGIDBODIES_BLEND_COLOR(contact);
            RIGIDBODIES_BLEND_COLOR(bounds);
            RIGIDBODIES_BLEND_COLOR(trajectory);
            RIGIDBODIES_BLEND_COLOR(label_plate);
            RIGIDBODIES_BLEND_COLOR(label_border);
            RIGIDBODIES_BLEND_COLOR(label_text);
            RIGIDBODIES_BLEND_COLOR(label_muted);
            RIGIDBODIES_BLEND_COLOR(panel_background);
            RIGIDBODIES_BLEND_COLOR(panel_border);
            RIGIDBODIES_BLEND_COLOR(panel_title);
            RIGIDBODIES_BLEND_COLOR(panel_text);
            RIGIDBODIES_BLEND_COLOR(panel_accent);
            RIGIDBODIES_BLEND_COLOR(panel_muted);
#undef RIGIDBODIES_BLEND_COLOR
            value.stroke_weight = from.stroke_weight + (to.stroke_weight - from.stroke_weight) * fraction;
            value.light_stage = fraction < 0.5f ? from.light_stage : to.light_stage;
            return value;
        }

        SceneRenderSettings resolved_settings(const physics::World& world, const SceneRenderSettings& requested)
        {
            auto result = requested;
            auto& scales = result.vector_scales;
            if (!scales.automatic)
                return result;
            double velocity = 0.0, acceleration = 0.0, force = 0.0, momentum = 0.0;
            const auto accumulate = [](double& maximum, const Vec2& value)
            {
                const auto magnitude = vector_length(value);
                if (std::isfinite(magnitude))
                    maximum = std::max(maximum, magnitude);
            };
            world.for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    if (body.type() == BodyType::static_body)
                        return;
                    accumulate(velocity, body.linear_velocity_m_s());
                    accumulate(force, body.applied_force_n());
                    accumulate(momentum, body.linear_momentum_kg_m_s());
                    if (body.inverse_mass() > 0.0)
                        accumulate(acceleration, body.applied_force_n() * body.inverse_mass());
                });
            if (result.layers.is_enabled(VisualizationLayer::constraints))
                for (const auto& constraint : world.constraints())
                    if (const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get()); joint && joint->is_enabled() && !joint->is_broken())
                        accumulate(force, joint->report(world).reaction_force_n);
            const auto target = std::isfinite(scales.automatic_target_length_px) && scales.automatic_target_length_px > 0.0f
                ? static_cast<double>(scales.automatic_target_length_px)
                : 120.0;
            const auto factor = [target](double maximum, double fallback)
            {
                return maximum > math::geometric_epsilon ? target / maximum : fallback;
            };
            scales.velocity = factor(velocity, scales.velocity);
            scales.acceleration = factor(acceleration, scales.acceleration);
            scales.force = factor(force, scales.force);
            scales.momentum = factor(momentum, scales.momentum);
            return result;
        }

        struct BodyColors
        {
            Color fill;
            Color outline;
        };

        BodyColors colors_for(const physics::RigidBody& body, const Theme& theme)
        {
            switch (body.type())
            {
            case BodyType::static_body:
                return { theme.static_body_fill, theme.static_body_outline };
            case BodyType::kinematic_body:
                return { theme.kinematic_body_fill, theme.kinematic_body_outline };
            case BodyType::dynamic_body:
                break;
            }
            return { theme.body_fill, theme.body_outline };
        }

        std::vector<Vec2> collider_outline_world(const physics::Collider& collider, const math::Transform2& body_transform)
        {
            std::vector<Vec2> points;
            if (!collider.shape)
            {
                return points;
            }

            const auto placement = math::concatenate(body_transform, collider.local_transform);
            if (const auto* polygon = dynamic_cast<const physics::ConvexPolygonShape*>(collider.shape.get()); polygon != nullptr)
            {
                points.reserve(polygon->vertices().size());
                for (const auto& vertex : polygon->vertices())
                {
                    points.push_back(math::transform_point(placement, vertex));
                }
            }
            else if (const auto* segment = dynamic_cast<const physics::SegmentShape*>(collider.shape.get()); segment != nullptr)
            {
                points.push_back(math::transform_point(placement, segment->start_m()));
                points.push_back(math::transform_point(placement, segment->end_m()));
            }
            return points;
        }

        // ---------------------------------------------------------------------------------
        // Mesh building. Every surface below carries its own transparent rim, so the compiler
        // can skip its boundary weld for them.

        int add_vertex(IndexedMesh& mesh, const Vec2& position, const Color& premultiplied_color)
        {
            mesh.vertices.push_back({ position, {}, premultiplied_color });
            return static_cast<int>(mesh.vertices.size()) - 1;
        }

        void add_triangle(IndexedMesh& mesh, int a, int b, int c)
        {
            mesh.indices.insert(mesh.indices.end(), { a, b, c });
        }

        // A straight antialiased stroke with butt ends and colours graded between its ends.
        void append_soft_segment(IndexedMesh& mesh, const Vec2& a, const Vec2& b, double half_width, double feather, const Color& color_a, const Color& color_b)
        {
            const auto delta = b - a;
            if (math::length_squared(delta) < 1.0e-12 || !math::is_finite(a) || !math::is_finite(b))
                return;
            const auto normal = math::perpendicular(math::normalized(delta));
            const auto first = premultiplied(color_a), second = premultiplied(color_b);
            const Color clear { 0.0f, 0.0f, 0.0f, 0.0f };
            const auto base = static_cast<int>(mesh.vertices.size());
            for (const auto& [point, color] : { std::pair<Vec2, Color> { a, first }, std::pair<Vec2, Color> { b, second } })
            {
                add_vertex(mesh, point + normal * (half_width + feather), clear);
                add_vertex(mesh, point + normal * half_width, color);
                add_vertex(mesh, point - normal * half_width, color);
                add_vertex(mesh, point - normal * (half_width + feather), clear);
            }
            for (int strip = 0; strip < 3; ++strip)
            {
                add_triangle(mesh, base + strip, base + strip + 1, base + 4 + strip + 1);
                add_triangle(mesh, base + strip, base + 4 + strip + 1, base + 4 + strip);
            }
        }

        // A filled convex polygon with a transparent rim, for shapes collected into one mesh.
        void append_soft_polygon(IndexedMesh& mesh, const std::array<Vec2, 4>& points, std::size_t count, double feather, const Color& color)
        {
            if (count < 3)
                return;
            Vec2 centroid;
            for (std::size_t index = 0; index < count; ++index)
            {
                if (!math::is_finite(points[index]))
                    return;
                centroid += points[index];
            }
            centroid = centroid / static_cast<double>(count);
            const auto fill = premultiplied(color);
            const Color clear { 0.0f, 0.0f, 0.0f, 0.0f };
            const auto centre = add_vertex(mesh, centroid, fill);
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto outward = points[index] - centroid;
                add_vertex(mesh, points[index], fill);
                add_vertex(mesh, points[index] + (math::length_squared(outward) > 1.0e-12 ? math::normalized(outward) : Vec2 {}) * feather, clear);
            }
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto inner = centre + 1 + static_cast<int>(index) * 2, next = centre + 1 + static_cast<int>((index + 1) % count) * 2;
                add_triangle(mesh, centre, inner, next);
                add_triangle(mesh, inner, inner + 1, next + 1);
                add_triangle(mesh, inner, next + 1, next);
            }
        }

        // The casing under an arrow drawn by DrawList::add_arrow with the same geometry: a band
        // `margin` wider than the shaft and the head on every side.
        void append_arrow_casing(IndexedMesh& mesh, const Vec2& from, const Vec2& to, double thickness, double head_length, ArrowHead head, double margin, double feather, const Color& color)
        {
            const auto length = math::length(to - from);
            if (!(length > 1.0e-9) || !std::isfinite(length))
                return;
            const auto direction = (to - from) / length;
            const auto head_size = std::min(head_length, length * 0.45);
            const auto base = to - direction * head_size;
            const auto side = math::perpendicular(direction) * (head_size * 0.42);
            if (head == ArrowHead::open)
            {
                const auto half = thickness * 0.5 + margin;
                append_soft_segment(mesh, from - direction * margin, to, half, feather, color, color);
                append_soft_segment(mesh, base + side, to + direction * margin, half, feather, color, color);
                append_soft_segment(mesh, base - side, to + direction * margin, half, feather, color, color);
                return;
            }
            append_soft_segment(mesh, from - direction * margin, base + direction * (head_size * 0.3), thickness * 0.5 + margin, feather, color, color);
            // The head's outline set out by the margin, measured square to each of its sides.
            const auto back = head == ArrowHead::double_filled ? direction * (head_size * 0.6) : Vec2 {};
            const auto spread = math::length(side) > 1.0e-9 ? math::normalized(side) : Vec2 {};
            const auto slant = margin * std::sqrt(1.0 + (head_size * head_size) / std::max(1.0e-9, math::length_squared(side)));
            const std::array<Vec2, 4> outline { to + direction * slant, base - back + side + spread * margin - direction * margin, base - back - side - spread * margin - direction * margin, {} };
            append_soft_polygon(mesh, outline, 3, feather, color);
        }

        // Clips the infinite line through `origin` along `direction` to a convex polygon.
        bool clip_line(const std::vector<Vec2>& polygon, const Vec2& origin, const Vec2& direction, Vec2& start, Vec2& end)
        {
            if (polygon.size() < 3)
                return false;
            double twice_area = 0.0;
            for (std::size_t index = 0; index < polygon.size(); ++index)
                twice_area += math::cross(polygon[index], polygon[(index + 1) % polygon.size()]);
            const auto sign = twice_area < 0.0 ? 1.0 : -1.0;
            double low = -1.0e9, high = 1.0e9;
            for (std::size_t index = 0; index < polygon.size(); ++index)
            {
                const auto& a = polygon[index];
                const auto& b = polygon[(index + 1) % polygon.size()];
                const auto outward = math::perpendicular(b - a) * sign;
                const auto denominator = math::dot(outward, direction);
                const auto numerator = math::dot(outward, a - origin);
                if (std::abs(denominator) < 1.0e-12)
                {
                    if (numerator < 0.0)
                        return false;
                    continue;
                }
                const auto t = numerator / denominator;
                if (denominator > 0.0)
                    high = std::min(high, t);
                else
                    low = std::max(low, t);
                if (low >= high)
                    return false;
            }
            start = origin + direction * low;
            end = origin + direction * high;
            return true;
        }

        struct SurfaceStyle
        {
            double outline_width {};
            Color outline;
            double feather { 1.0 };
            double bevel {};
            int rings { 2 };
            double max_segment { 1.0e9 };
        };

        // A convex shaded surface built as concentric rings: transparent rim, outline, lit
        // bevel, then interior rings toward the centroid so that smooth shading functions are
        // sampled densely enough for Gouraud interpolation to stay free of facets. The inset
        // polygon just inside the bevel is returned for details clipped to the surface.
        template <class Shade, class Edge>
        bool append_surface(IndexedMesh& mesh, const std::vector<Vec2>& boundary, const SurfaceStyle& style, const Shade& shade, const Edge& edge, std::vector<Vec2>* inset = nullptr)
        {
            thread_local std::vector<Vec2> points, refined, offsets, normals, ring;
            points.clear();
            for (const auto& point : boundary)
                if (math::is_finite(point) && (points.empty() || math::length_squared(point - points.back()) > 1.0e-10))
                    points.push_back(point);
            if (points.size() > 2 && math::length_squared(points.front() - points.back()) <= 1.0e-10)
                points.pop_back();
            const auto count = points.size();
            if (count < 3)
                return false;
            double twice_area = 0.0;
            Vec2 centroid;
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto& a = points[index];
                const auto& b = points[(index + 1) % count];
                const auto cross = math::cross(a, b);
                twice_area += cross;
                centroid += (a + b) * cross;
            }
            if (std::abs(twice_area) < 1.0e-6)
                return false;
            centroid = centroid / (3.0 * twice_area);
            const auto sign = twice_area < 0.0 ? 1.0 : -1.0;
            const auto edge_normal = [&](std::size_t index)
            {
                return math::normalized(math::perpendicular(points[(index + 1) % count] - points[index])) * sign;
            };
            refined.clear();
            offsets.clear();
            normals.clear();
            double inradius = 1.0e9;
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto previous = edge_normal((index + count - 1) % count);
                const auto next = edge_normal(index);
                const auto sum = previous + next;
                const auto divisor = math::dot(sum, next);
                // Sharp corners keep their offset on the bisector, limited in length, so rings
                // stay symmetric about a thin tip instead of sliding along one edge.
                auto miter = divisor > 0.25 ? sum / divisor : math::length_squared(sum) > 1.0e-12 ? math::normalized(sum) * 2.5
                                                                                                  : next;
                if (math::length_squared(miter) > 6.25)
                    miter = math::normalized(miter) * 2.5;
                refined.push_back(points[index]);
                offsets.push_back(miter);
                normals.push_back(math::normalized(sum));
                const auto& a = points[index];
                const auto& b = points[(index + 1) % count];
                inradius = std::min(inradius, math::dot(a - centroid, next));
                const auto length = math::length(b - a);
                const auto pieces = static_cast<int>(std::clamp(std::ceil(length / std::max(1.0, style.max_segment)), 1.0, 64.0));
                for (int piece = 1; piece < pieces; ++piece)
                {
                    refined.push_back(math::lerp(a, b, static_cast<double>(piece) / pieces));
                    offsets.push_back(next);
                    normals.push_back(next);
                }
            }
            if (!(inradius > 0.0))
                return false;
            auto half_outline = std::max(0.0, style.outline_width * 0.5);
            auto feather = std::max(0.0, style.feather);
            auto bevel = std::max(0.0, style.bevel);
            const auto limit = inradius * 0.6;
            if (half_outline + feather + bevel > limit)
                bevel = std::max(0.0, limit - half_outline - feather);
            if (half_outline + feather > limit)
            {
                const auto shrink = limit / (half_outline + feather);
                half_outline *= shrink;
                feather *= shrink;
            }
            const auto m = static_cast<int>(refined.size());
            const auto clear = Color { 0.0f, 0.0f, 0.0f, 0.0f };
            int ring_count = 0;
            const auto first_vertex = static_cast<int>(mesh.vertices.size());
            const auto push_ring = [&](double offset, int kind)
            {
                // kind 0: transparent rim, 1: outline, 2: lit edge, 3: shaded interior.
                for (int index = 0; index < m; ++index)
                {
                    const auto point = refined[static_cast<std::size_t>(index)] + offsets[static_cast<std::size_t>(index)] * offset;
                    Color color;
                    if (kind == 0)
                        color = clear;
                    else if (kind == 1)
                        color = premultiplied(style.outline);
                    else if (kind == 2)
                        color = premultiplied(edge(point, normals[static_cast<std::size_t>(index)]));
                    else
                        color = premultiplied(shade(point));
                    add_vertex(mesh, point, color);
                }
                ++ring_count;
            };
            double inner = 0.0;
            if (half_outline > 0.0)
            {
                push_ring(half_outline + feather, 0);
                push_ring(half_outline, 1);
                push_ring(-half_outline, 1);
                push_ring(-(half_outline + feather), 2);
                inner = half_outline + feather;
            }
            else
            {
                push_ring(feather * 0.5, 0);
                push_ring(-feather * 0.5, 2);
                inner = feather * 0.5;
            }
            if (bevel > 0.0)
            {
                inner += bevel;
                push_ring(-inner, 3);
            }
            if (inset)
            {
                inset->clear();
                for (int index = 0; index < m; ++index)
                    inset->push_back(refined[static_cast<std::size_t>(index)] - offsets[static_cast<std::size_t>(index)] * inner);
            }
            const auto base_ring = mesh.vertices.size() - static_cast<std::size_t>(m);
            ring.assign(m, Vec2 {});
            for (int index = 0; index < m; ++index)
                ring[static_cast<std::size_t>(index)] = mesh.vertices[base_ring + static_cast<std::size_t>(index)].position;
            const auto interior = std::max(0, style.rings);
            for (int level = 1; level <= interior; ++level)
            {
                const auto fraction = 1.0 - static_cast<double>(level) / static_cast<double>(interior + 1);
                for (int index = 0; index < m; ++index)
                {
                    const auto point = centroid + (ring[static_cast<std::size_t>(index)] - centroid) * fraction;
                    add_vertex(mesh, point, premultiplied(shade(point)));
                }
                ++ring_count;
            }
            const auto center = add_vertex(mesh, centroid, premultiplied(shade(centroid)));
            for (int level = 0; level + 1 < ring_count; ++level)
            {
                const auto outer = first_vertex + level * m;
                const auto inner_ring = outer + m;
                for (int index = 0; index < m; ++index)
                {
                    const auto next = (index + 1) % m;
                    add_triangle(mesh, outer + index, outer + next, inner_ring + next);
                    add_triangle(mesh, outer + index, inner_ring + next, inner_ring + index);
                }
            }
            const auto last = first_vertex + (ring_count - 1) * m;
            for (int index = 0; index < m; ++index)
                add_triangle(mesh, last + index, last + (index + 1) % m, center);
            return true;
        }

        // Body-wide lighting: every collider of a body shares one gradient so compound bodies
        // read as one object under one light.
        struct BodyLighting
        {
            MaterialAppearance look;
            double t_min {}, t_max { 1.0 };
            bool round {};
            Vec2 center;
            double radius {};
        };

        Color body_shade(const BodyLighting& lighting, const Vec2& point)
        {
            const auto& look = lighting.look;
            const auto span = std::max(1.0e-6, lighting.t_max - lighting.t_min);
            const auto t = std::clamp((math::dot(point, toward_light) - lighting.t_min) / span, 0.0, 1.0);
            auto color = mix(look.shade, look.surface, static_cast<float>(smoothstep(0.0, 0.72, t)));
            color = mix(color, look.highlight, static_cast<float>(0.55 * smoothstep(0.62, 1.0, t)));
            if (look.specular > 0.0f)
            {
                const auto offset = (t - 0.66) / 0.11;
                const auto band = std::exp(-offset * offset);
                color = mix(color, look.rim, static_cast<float>(look.specular * band));
            }
            if (lighting.round && look.gloss > 0.0f && lighting.radius > 0.0)
            {
                const auto spot = lighting.center + toward_light * (lighting.radius * 0.42);
                const auto distance = math::length_squared(point - spot) / (lighting.radius * lighting.radius * 0.42 * 0.42);
                color = mix(color, look.rim, static_cast<float>(look.gloss * std::exp(-distance)));
            }
            return color;
        }

        Color body_edge(const BodyLighting& lighting, const Vec2& point, const Vec2& normal)
        {
            auto color = body_shade(lighting, point);
            const auto facing = math::dot(normal, toward_light);
            if (facing > 0.0)
                return mix(color, lighting.look.rim, static_cast<float>(lighting.look.edge_light * facing * std::sqrt(facing)));
            const auto alpha = color.alpha;
            color = color.scaled(static_cast<float>(1.0 + 0.6 * lighting.look.edge_light * facing));
            color.alpha = alpha;
            return color;
        }

        // Outline colour: a crisp edge one step away from the fill in the direction that
        // separates it from the stage.
        Color outline_for(const MaterialAppearance& look, const Theme& theme)
        {
            if (look.translucent)
                return theme.light_stage ? toward(look.shade, 0.0f, 0.0f, 0.0f, 0.3f).with_alpha(0.9f) : look.rim;
            const auto light = luminance(look.surface);
            if (theme.light_stage || light > 0.32f)
                return toward(look.shade, 0.0f, 0.0f, 0.0f, theme.light_stage ? 0.3f : 0.2f).with_alpha(1.0f);
            return toward(look.surface, 1.0f, 1.0f, 1.0f, 0.2f).with_alpha(1.0f);
        }

        std::uint64_t body_key(physics::BodyId id, std::uint64_t kind)
        {
            return (kind << 56u) ^ (static_cast<std::uint64_t>(id.generation & 0xffffffu) << 32u) ^ id.index;
        }

        constexpr std::string_view gravity_channel = "uniform_gravity";
        constexpr std::string_view weight_name = "Weight";

        // The stage names a load by its source, in the words the inspector and guides use.
        std::string_view load_name(std::string_view channel)
        {
            if (channel == gravity_channel)
                return weight_name;
            if (channel == "aerodynamic_drag")
                return "Air drag";
            if (channel == "magnus")
                return "Magnus lift";
            if (channel == "point_attractor")
                return "Attraction";
            if (channel == "Pointer spring")
                return "Pull";
            return "Applied";
        }

        bool spring_channel(std::string_view channel)
        {
            return channel.substr(0, 7) == "spring/";
        }

        // Below this a force reads as zero in every unit system, so it is never drawn.
        constexpr double readable_force_n = 1.0e-5;

        // The weight gravity gives a body: as applied in the last substep, or before the first
        // step as the gravity sources will apply it, so the force scale and its key hold still
        // from the moment a scene opens.
        Vec2 weight_n(const physics::World& world, const physics::RigidBody& body)
        {
            if (body.type() != BodyType::dynamic_body)
                return {};
            Vec2 weight;
            bool applied = false;
            for (const auto& channel : body.applied_force_channels())
                if (channel.name == gravity_channel && math::is_finite(channel.force_n))
                {
                    weight += channel.force_n;
                    applied = true;
                }
            if (applied)
                return weight;
            for (const auto& generator : world.force_generators())
                if (generator && generator->is_enabled() && generator->name() == gravity_channel)
                    weight += world.settings().gravity_m_s2 * (body.mass_properties().mass_kg * body.gravity_scale());
            return math::is_finite(weight) ? weight : Vec2 {};
        }

        // Plain decimal digits with a true minus sign, at three significant figures.
        std::string three_figures(double value)
        {
            const auto magnitude = std::abs(value);
            const auto decimals = magnitude > 0.0 ? std::clamp(2 - static_cast<int>(std::floor(std::log10(magnitude))), 0, 9) : 2;
            auto text = core::fixed(magnitude, decimals);
            return value < 0.0 && text.find_first_not_of("0.") != std::string::npos ? "\xE2\x88\x92" + text : text;
        }

        // The centre-of-mass symbol is sized from the body's own frame, so a thin body keeps one
        // symbol size however it turns. A body barely larger than the symbol is its own mark and
        // goes without, except a body of several parts, whose centre is the point of interest.
        float center_marker_radius(const physics::RigidBody& body, const math::Aabb& local, const Camera2D& camera, float scale, bool shaded, bool& raised)
        {
            raised = false;
            const auto thickness = local.is_empty() ? 0.0 : camera.world_to_screen_length(std::min(local.extents().x, local.extents().y));
            const auto radius = static_cast<float>(std::min(5.0 * scale, thickness * 0.5 / 2.2));
            if (radius >= 2.5f * scale)
                return radius;
            if (shaded && body.colliders().size() < 2)
                return 0.0f;
            raised = true;
            return 2.5f * scale;
        }

        // Monotone chain hull for the soft shadow of a concave authored part.
        void convex_hull(std::vector<Vec2>& points)
        {
            if (points.size() < 3)
                return;
            std::sort(points.begin(), points.end(), [](const Vec2& a, const Vec2& b)
                {
                    return a.x < b.x || (a.x == b.x && a.y < b.y);
                });
            thread_local std::vector<Vec2> hull;
            hull.clear();
            for (int pass = 0; pass < 2; ++pass)
            {
                const auto start = hull.size();
                for (std::size_t index = 0; index < points.size(); ++index)
                {
                    const auto& point = pass == 0 ? points[index] : points[points.size() - 1 - index];
                    while (hull.size() >= start + 2 && math::cross(hull[hull.size() - 1] - hull[hull.size() - 2], point - hull[hull.size() - 2]) <= 0.0)
                        hull.pop_back();
                    hull.push_back(point);
                }
                hull.pop_back();
            }
            points.assign(hull.begin(), hull.end());
        }

        double polygon_area(const std::vector<Vec2>& points)
        {
            double twice = 0.0;
            for (std::size_t index = 0; index < points.size(); ++index)
                twice += math::cross(points[index], points[(index + 1) % points.size()]);
            return std::abs(twice) * 0.5;
        }

        // A soft fill under an outline is fanned across its hull, which for a notched outline
        // would paint the notch.
        bool nearly_convex(const std::vector<Vec2>& outline)
        {
            thread_local std::vector<Vec2> hull;
            hull = outline;
            convex_hull(hull);
            return hull.size() >= 3 && polygon_area(outline) >= polygon_area(hull) * 0.98;
        }

        void offset_outline(const std::vector<Vec2>& source, double distance, std::vector<Vec2>& result)
        {
            result.clear();
            const auto count = source.size();
            if (count < 3)
            {
                result = source;
                return;
            }
            double twice_area = 0.0;
            for (std::size_t index = 0; index < count; ++index)
                twice_area += math::cross(source[index], source[(index + 1) % count]);
            const auto sign = twice_area < 0.0 ? 1.0 : -1.0;
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto previous = math::normalized(math::perpendicular(source[index] - source[(index + count - 1) % count])) * sign;
                const auto next = math::normalized(math::perpendicular(source[(index + 1) % count] - source[index])) * sign;
                const auto sum = previous + next;
                const auto divisor = math::dot(sum, next);
                auto miter = divisor > 0.25 ? sum / divisor : math::length_squared(sum) > 1.0e-12 ? math::normalized(sum) * 2.0
                                                                                                  : next;
                if (math::length_squared(miter) > 4.0)
                    miter = math::normalized(miter) * 2.0;
                result.push_back(source[index] + miter * distance);
            }
        }

        // The inward parallel of a convex outline, or false when the distance would consume it.
        bool inset_outline(const std::vector<Vec2>& source, double distance, std::vector<Vec2>& result)
        {
            result.clear();
            const auto count = source.size();
            if (count < 3 || !(distance > 0.0))
                return false;
            Vec2 centroid;
            for (const auto& point : source)
                centroid += point;
            centroid = centroid / static_cast<double>(count);
            double twice_area = 0.0;
            for (std::size_t index = 0; index < count; ++index)
                twice_area += math::cross(source[index], source[(index + 1) % count]);
            const auto sign = twice_area < 0.0 ? 1.0 : -1.0;
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto& a = source[index];
                const auto& b = source[(index + 1) % count];
                if (math::dot(centroid - a, math::normalized(math::perpendicular(b - a)) * -sign) < distance * 1.5)
                    return false;
            }
            offset_outline(source, -distance, result);
            return result.size() == count;
        }

        // Screen axis-aligned rectangle used by the label solver.
        struct Rect
        {
            Vec2 minimum, maximum;
        };

        double overlap_area(const Rect& a, const Rect& b)
        {
            const auto width = std::min(a.maximum.x, b.maximum.x) - std::max(a.minimum.x, b.minimum.x);
            const auto height = std::min(a.maximum.y, b.maximum.y) - std::max(a.minimum.y, b.minimum.y);
            return width > 0.0 && height > 0.0 ? width * height : 0.0;
        }

        double rect_distance(const Rect& rect, const Vec2& minimum, const Vec2& maximum)
        {
            const auto dx = std::max({ 0.0, minimum.x - rect.maximum.x, rect.minimum.x - maximum.x });
            const auto dy = std::max({ 0.0, minimum.y - rect.maximum.y, rect.minimum.y - maximum.y });
            return std::hypot(dx, dy);
        }

        Vec2 nearest_on_rect(const Rect& rect, const Vec2& point)
        {
            return { std::clamp(point.x, rect.minimum.x, rect.maximum.x), std::clamp(point.y, rect.minimum.y, rect.maximum.y) };
        }

        // Distance from a point inside a rectangle to its edge along a direction; negative when
        // the point lies outside.
        double room_along(const Vec2& start, const Vec2& direction, const Rect& rect)
        {
            if (start.x < rect.minimum.x || start.y < rect.minimum.y || start.x > rect.maximum.x || start.y > rect.maximum.y)
                return -1.0;
            auto room = 1.0e300;
            if (direction.x > 1.0e-9)
                room = std::min(room, (rect.maximum.x - start.x) / direction.x);
            else if (direction.x < -1.0e-9)
                room = std::min(room, (rect.minimum.x - start.x) / direction.x);
            if (direction.y > 1.0e-9)
                room = std::min(room, (rect.maximum.y - start.y) / direction.y);
            else if (direction.y < -1.0e-9)
                room = std::min(room, (rect.minimum.y - start.y) / direction.y);
            return room;
        }

        Rect stage_rect(const Camera2D& camera)
        {
            const auto& focus = camera.focus_rect();
            if (focus.empty())
                return { {}, { static_cast<double>(camera.viewport().width), static_cast<double>(camera.viewport().height) } };
            return { { focus.left, focus.top }, { focus.left + focus.width, focus.top + focus.height } };
        }

        // The stage in view, which the framing area sits inside with a margin; arrows and plates
        // may use most of that margin too. The framing area keeps a narrower margin from a sheet
        // laid over the stage beside it, so growing it by that narrower margin never reaches
        // under the sheet.
        Rect visible_rect(const Camera2D& camera, const ScreenRect& visible, double sheet_margin)
        {
            if (visible.empty())
                return stage_rect(camera);
            const Vec2 viewport { static_cast<double>(camera.viewport().width), static_cast<double>(camera.viewport().height) };
            Rect result { math::max_components(Vec2 { visible.left, visible.top }, Vec2 {}), math::min_components(Vec2 { visible.left + visible.width, visible.top + visible.height }, viewport) };
            if (!camera.focus_rect().empty())
            {
                const auto framed = stage_rect(camera);
                result.minimum = math::max_components(result.minimum, framed.minimum - Vec2 { sheet_margin, sheet_margin });
                result.maximum = math::min_components(result.maximum, framed.maximum + Vec2 { sheet_margin, sheet_margin });
            }
            return result;
        }

        Vec2 nearest_on_outline(const Vec2* points, std::size_t count, const Vec2& point)
        {
            Vec2 best = points[0];
            auto best_distance = math::length_squared(point - best);
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto& a = points[index];
                const auto& b = points[(index + 1) % count];
                const auto edge = b - a;
                const auto length = math::length_squared(edge);
                const auto t = length > 0.0 ? std::clamp(math::dot(point - a, edge) / length, 0.0, 1.0) : 0.0;
                const auto candidate = a + edge * t;
                const auto distance = math::length_squared(point - candidate);
                if (distance < best_distance)
                {
                    best_distance = distance;
                    best = candidate;
                }
            }
            return best;
        }

        // Separating-axis test of a segment against a convex polygon.
        bool segment_crosses_polygon(const Vec2& a, const Vec2& b, const Vec2* points, std::size_t count)
        {
            const auto overlaps = [&](const Vec2& axis)
            {
                double low = 1.0e300, high = -1.0e300;
                for (std::size_t index = 0; index < count; ++index)
                {
                    const auto value = math::dot(points[index], axis);
                    low = std::min(low, value);
                    high = std::max(high, value);
                }
                const auto first = math::dot(a, axis), second = math::dot(b, axis);
                return std::max(first, second) >= low && std::min(first, second) <= high;
            };
            if (!overlaps(math::perpendicular(b - a)))
                return false;
            for (std::size_t index = 0; index < count; ++index)
                if (!overlaps(math::perpendicular(points[(index + 1) % count] - points[index])))
                    return false;
            return true;
        }

        // Crossing-number test; the outline may be concave.
        bool point_in_polygon(const Vec2& point, const Vec2* points, std::size_t count)
        {
            bool inside = false;
            for (std::size_t index = 0, previous = count - 1; index < count; previous = index++)
            {
                const auto& a = points[index];
                const auto& b = points[previous];
                if ((a.y > point.y) != (b.y > point.y) && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x)
                    inside = !inside;
            }
            return inside;
        }

        // Where a ray first meets an outline (or a segment, for two points), within `length`:
        // zero when it starts inside. The outline may be concave.
        bool ray_entry(const Vec2* points, std::size_t count, const Vec2& from, const Vec2& direction, double length, double& entry)
        {
            if (count >= 3 && point_in_polygon(from, points, count))
            {
                entry = 0.0;
                return true;
            }
            auto best = length;
            bool found = false;
            const auto edges = count >= 3 ? count : count - 1;
            for (std::size_t index = 0; index < edges; ++index)
            {
                const auto& a = points[index];
                const auto edge = points[(index + 1) % count] - a;
                const auto denominator = math::cross(direction, edge);
                if (std::abs(denominator) < 1.0e-12)
                    continue;
                const auto offset = a - from;
                const auto t = math::cross(offset, edge) / denominator;
                const auto s = math::cross(offset, direction) / denominator;
                if (t >= 0.0 && t <= best && s >= 0.0 && s <= 1.0)
                {
                    best = t;
                    found = true;
                }
            }
            entry = best;
            return found;
        }

        double segment_point_distance(const Vec2& a, const Vec2& b, const Vec2& point)
        {
            const auto edge = b - a;
            const auto length = math::length_squared(edge);
            const auto t = length > 0.0 ? std::clamp(math::dot(point - a, edge) / length, 0.0, 1.0) : 0.0;
            return math::length(point - (a + edge * t));
        }

        bool segments_cross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d)
        {
            const auto first = math::cross(b - a, c - a), second = math::cross(b - a, d - a);
            const auto third = math::cross(d - c, a - c), fourth = math::cross(d - c, b - c);
            return ((first > 0.0) != (second > 0.0)) && ((third > 0.0) != (fourth > 0.0));
        }

        // Value arrows shorter than this many logical pixels are drawn at this length with a
        // dotted shaft instead: long enough for a readable head and two dots.
        constexpr double stub_length_dp = 14.0;

        // How far a push may reach back into a movable body exerting it: most of the way across
        // that body, judged from its centre of mass, so its tail stays inside the body and never
        // reaches the body beyond. Zero for a fixed support.
        double push_reach(const physics::RigidBody& pusher, const Camera2D& camera, double alpha, const Vec2& face, const Vec2& direction)
        {
            if (pusher.type() != BodyType::dynamic_body)
                return 0.0;
            const auto centre = camera.world_to_screen(math::transform_point(pusher.interpolated_transform(alpha), pusher.mass_properties().center_of_mass_m));
            return std::max(1.7 * math::dot(face - centre, direction), 1.0);
        }

        bool rect_hits_polygon(const Rect& rect, const Vec2* points, std::size_t count)
        {
            if (count == 2)
            {
                // A segment: separating axes are the rectangle's and the segment normal.
                const auto& a = points[0];
                const auto& b = points[1];
                if (std::max(a.x, b.x) < rect.minimum.x || std::min(a.x, b.x) > rect.maximum.x || std::max(a.y, b.y) < rect.minimum.y || std::min(a.y, b.y) > rect.maximum.y)
                    return false;
                const auto normal = math::perpendicular(b - a);
                const std::array<Vec2, 4> corners { rect.minimum, Vec2 { rect.maximum.x, rect.minimum.y }, rect.maximum, Vec2 { rect.minimum.x, rect.maximum.y } };
                double low = 1.0e300, high = -1.0e300;
                for (const auto& corner : corners)
                {
                    const auto value = math::dot(corner - a, normal);
                    low = std::min(low, value);
                    high = std::max(high, value);
                }
                return low <= 0.0 && high >= 0.0;
            }
            const std::array<Vec2, 4> corners { rect.minimum, Vec2 { rect.maximum.x, rect.minimum.y }, rect.maximum, Vec2 { rect.minimum.x, rect.maximum.y } };
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto& a = points[index];
                const auto& b = points[(index + 1) % count];
                const auto axis = math::perpendicular(b - a);
                double polygon_low = 1.0e300, polygon_high = -1.0e300, rect_low = 1.0e300, rect_high = -1.0e300;
                for (std::size_t other = 0; other < count; ++other)
                {
                    const auto value = math::dot(points[other], axis);
                    polygon_low = std::min(polygon_low, value);
                    polygon_high = std::max(polygon_high, value);
                }
                for (const auto& corner : corners)
                {
                    const auto value = math::dot(corner, axis);
                    rect_low = std::min(rect_low, value);
                    rect_high = std::max(rect_high, value);
                }
                if (rect_high < polygon_low || polygon_high < rect_low)
                    return false;
            }
            return true;
        }

    } // namespace

    std::shared_ptr<IndexedMesh> SceneRenderer::acquire_mesh() const
    {
        if (mesh_pool_used_ < mesh_pool_.size())
        {
            auto& slot = mesh_pool_[mesh_pool_used_];
            if (slot.use_count() != 1)
                slot = std::make_shared<IndexedMesh>();
            ++mesh_pool_used_;
            slot->vertices.clear();
            slot->indices.clear();
            slot->texture.reset();
            slot->clip.reset();
            slot->instances.clear();
            slot->feathered = true;
            return slot;
        }
        mesh_pool_.push_back(std::make_shared<IndexedMesh>());
        ++mesh_pool_used_;
        mesh_pool_.back()->feathered = true;
        return mesh_pool_.back();
    }

    void SceneRenderer::queue_label(LabelRequest request) const
    {
        label_requests_.push_back(std::move(request));
    }

    void SceneRenderer::set_selected_bodies(const std::vector<physics::BodyId>& bodies)
    {
        selected_bodies_.assign(bodies.begin(), bodies.end());
    }

    bool SceneRenderer::is_selected(physics::BodyId id, const SceneRenderSettings& settings) const
    {
        if (!id.is_valid() || !settings.selection.is_valid())
            return false;
        if (id == settings.selection)
            return true;
        // The set describes the settings' selection only while it contains it.
        const auto contains = [&](physics::BodyId wanted)
        {
            return std::find(selected_bodies_.begin(), selected_bodies_.end(), wanted) != selected_bodies_.end();
        };
        return contains(settings.selection) && contains(id);
    }

    bool SceneRenderer::is_focus(physics::BodyId id, const SceneRenderSettings& settings) const
    {
        if (id == settings.selection && id.is_valid())
            return true;
        return selected_bodies_.size() <= 2 && is_selected(id, settings);
    }

    std::string SceneRenderer::force_text(double value_n, const SceneRenderSettings& settings) const
    {
        if (settings.display_units != core::DisplayUnits::si || !std::isfinite(value_n) || std::abs(value_n) < readable_force_n || std::abs(value_n) >= 10000.0)
            return core::format_quantity(value_n, core::DisplayQuantity::force, settings.display_units);
        return three_figures(force_milli_ ? value_n * 1000.0 : value_n) + (force_milli_ ? "\xC2\xA0mN" : "\xC2\xA0N");
    }

    std::string SceneRenderer::body_display_name(const physics::World& world, physics::BodyId wanted) const
    {
        const auto* body = world.find_body(wanted);
        if (!body)
            return {};
        const auto named = std::lower_bound(body_names_.begin(), body_names_.end(), wanted.index, [](const auto& entry, std::uint32_t index)
            {
                return entry.first.index < index;
            });
        if (named != body_names_.end() && named->first == wanted && !named->second.empty())
            return named->second;
        if (!body->name().empty())
            return core::humanise_identifier(body->name());
        std::size_t position = 0;
        for (const auto type : { physics::BodyType::dynamic_body, physics::BodyType::kinematic_body, physics::BodyType::static_body })
            for (const auto id : world.body_ids())
                if (const auto* candidate = world.find_body(id); candidate != nullptr && candidate->type() == type)
                {
                    ++position;
                    if (id == wanted)
                        return core::substitute("Object {}", position);
                }
        return std::string { "Object" };
    }

    void SceneRenderer::gather_connection_loads(const physics::World& world, const SceneRenderSettings& settings) const
    {
        std::fill(joint_forces_n_.begin(), joint_forces_n_.end(), Vec2 {});
        std::fill(spring_forces_n_.begin(), spring_forces_n_.end(), Vec2 {});
        std::fill(spring_counts_.begin(), spring_counts_.end(), std::uint8_t {});
        std::fill(spring_stated_.begin(), spring_stated_.end(), std::uint8_t {});
        const auto add = [](std::vector<Vec2>& forces, physics::BodyId id, const Vec2& force)
        {
            if (!math::is_finite(force))
                return;
            if (id.index >= forces.size())
                forces.resize(static_cast<std::size_t>(id.index) + 1);
            forces[id.index] += force;
        };
        for (const auto& constraint : world.constraints())
            if (const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get()); joint && joint->is_enabled() && !joint->is_broken())
            {
                const auto report = joint->report(world);
                if (!report.enabled)
                    continue;
                add(joint_forces_n_, report.second, report.reaction_force_n);
                add(joint_forces_n_, report.first, -report.reaction_force_n);
            }
        if (!settings.layers.is_enabled(VisualizationLayer::force_vectors))
            return;
        for (const auto id : world.spring_ids())
            if (const auto report = world.spring_report(id); report && report->enabled && report->kind == physics::SpringKind::linear)
            {
                add(spring_forces_n_, report->first, report->force_on_first_n);
                add(spring_forces_n_, report->second, -report->force_on_first_n);
                for (const auto body : { report->first, report->second })
                {
                    if (body.index >= spring_counts_.size())
                        spring_counts_.resize(static_cast<std::size_t>(body.index) + 1);
                    spring_counts_[body.index] = static_cast<std::uint8_t>(std::min(spring_counts_[body.index] + 1, 2));
                }
            }
    }

    void SceneRenderer::collect_loads(physics::BodyId id, const physics::RigidBody& body, BodyLoads& loads) const
    {
        loads.count = 0;
        const auto add = [&](std::string_view name, const Vec2& force)
        {
            if (!math::is_finite(force))
                return;
            for (std::size_t index = 0; index < loads.count; ++index)
                if (loads.items[index].name == name)
                {
                    loads.items[index].force_n += force;
                    return;
                }
            if (loads.count < loads.items.size())
                loads.items[loads.count++] = { name, force, name == weight_name ? ArrowHead::filled : ArrowHead::hollow };
        };
        for (const auto& channel : body.applied_force_channels())
            if (!spring_channel(channel.name))
                add(load_name(channel.name), channel.force_n);
        if (id.index < spring_forces_n_.size() && math::length_squared(spring_forces_n_[id.index]) > 0.0)
            add("Spring", spring_forces_n_[id.index]);
    }

    void SceneRenderer::keep_shown_loads(BodyLoads& loads, bool compact)
    {
        double largest = 0.0;
        for (std::size_t index = 0; index < loads.count; ++index)
            largest = std::max(largest, vector_length(loads.items[index].force_n));
        std::size_t kept = 0;
        Vec2 others;
        std::size_t folded = 0;
        for (std::size_t index = 0; index < loads.count; ++index)
        {
            const auto load = loads.items[index];
            const auto magnitude = vector_length(load.force_n);
            // A weight always shows; another load shows when it would not read as zero and is
            // not a sliver beside the body's largest load.
            if (!std::isfinite(magnitude) || magnitude < readable_force_n || (load.name != weight_name && magnitude < 0.01 * largest))
                continue;
            if (compact && load.name != weight_name)
            {
                others += load.force_n;
                ++folded;
                if (folded == 1)
                    loads.items[kept++] = load;
                continue;
            }
            loads.items[kept++] = load;
        }
        loads.count = kept;
        if (folded < 2)
            return;
        // Several sources fold into one resultant, which keeps the place of the first of them.
        for (std::size_t index = 0; index < loads.count; ++index)
            if (loads.items[index].name != weight_name)
            {
                loads.items[index] = { "Other loads", others, ArrowHead::hollow };
                break;
            }
        if (!(vector_length(others) >= readable_force_n))
        {
            std::size_t write = 0;
            for (std::size_t index = 0; index < loads.count; ++index)
                if (loads.items[index].name == weight_name)
                    loads.items[write++] = loads.items[index];
            loads.count = write;
        }
    }

    void SceneRenderer::add_obstacle(const Vec2* points, std::size_t count, std::uint64_t owner, std::uint64_t plate, bool receding) const
    {
        if (count < 2)
            return;
        LabelObstacle obstacle;
        obstacle.offset = obstacle_points_.size();
        obstacle.count = count;
        obstacle.owner = owner;
        obstacle.plate = plate;
        obstacle.receding = receding;
        obstacle.minimum = { 1.0e300, 1.0e300 };
        obstacle.maximum = { -1.0e300, -1.0e300 };
        for (std::size_t index = 0; index < count; ++index)
        {
            if (!math::is_finite(points[index]))
                return;
            obstacle.minimum = math::min_components(obstacle.minimum, points[index]);
            obstacle.maximum = math::max_components(obstacle.maximum, points[index]);
        }
        obstacle_points_.insert(obstacle_points_.end(), points, points + count);
        label_obstacles_.push_back(obstacle);
    }

    void SceneRenderer::advance_presentation(double real_delta_s)
    {
        if (std::isfinite(real_delta_s) && real_delta_s > 0.0)
            presentation_delta_s_ = std::min(presentation_delta_s_ + real_delta_s, 0.1);
    }

    void SceneRenderer::render(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& requested, DrawList& list)
    {
        list.clear();
        casing_mesh_.reset();
        mesh_pool_used_ = 0;
        label_requests_.clear();
        label_obstacles_.clear();
        obstacle_points_.clear();
        reserved_label_areas_.clear();
        center_markers_.clear();
        drawn_arrows_.clear();
        for (auto& frame : body_frames_)
            frame = {};
        // Arrow scales are logical pixels per unit, so arrows keep their proportions to the stage
        // on a high-density display, as every other stroke and label does.
        auto logical = requested;
        {
            const auto ds = pixel_scale(requested);
            auto& scales = logical.vector_scales;
            scales.velocity *= static_cast<double>(ds);
            scales.acceleration *= static_cast<double>(ds);
            scales.force *= static_cast<double>(ds);
            scales.momentum *= static_cast<double>(ds);
            scales.minimum_drawn_length_px *= ds;
            scales.maximum_drawn_length_px *= ds;
            scales.automatic_target_length_px *= ds;
            drawn_pixel_scale_ = static_cast<double>(ds);
        }
        auto settings = resolved_settings(world, logical);
        drawn_velocity_scale_ = settings.vector_scales.velocity;
        gather_connection_loads(world, settings);
        gather_contact_forces(world);
        heaviest_weight_n_ = 0.0;
        if (settings.layers.is_enabled(VisualizationLayer::force_vectors) && !settings.vector_scales.automatic)
            settings.vector_scales.force = fitted_force_scale(world, camera, settings);
        drawn_force_scale_ = settings.vector_scales.force;
        {
            // A scene whose forces are all below a newton reads them in millinewtons, key and
            // plates alike, so no plate needs its unit compared against another's.
            const auto reference = heaviest_weight_n_ > math::geometric_epsilon ? heaviest_weight_n_ : peak_applied_n_;
            force_milli_ = reference > math::geometric_epsilon && reference < 1.0;
        }
        if (!presentation_initialized_ || !settings.transitions)
        {
            presentation_theme_ = settings.theme;
            presentation_selection_ = settings.selection;
            selection_opacity_ = 1.0;
            presentation_initialized_ = true;
        }
        else
        {
            const auto fraction = static_cast<float>(1.0 - std::exp(-presentation_delta_s_ * 18.0));
            presentation_theme_ = interpolate_theme(presentation_theme_, settings.theme, fraction);
            if (!(presentation_selection_ == settings.selection))
            {
                presentation_selection_ = settings.selection;
                selection_opacity_ = 0.55;
            }
            selection_opacity_ += (1.0 - selection_opacity_) * fraction;
        }
        presentation_delta_s_ = 0.0;
        settings.theme = presentation_theme_;
        settings.theme.selection.alpha *= static_cast<float>(selection_opacity_);

        list.set_layer(-30);
        draw_background(camera, settings, list);
        list.set_layer(-20);

        if (settings.layers.is_enabled(VisualizationLayer::grid))
        {
            draw_grid(camera, settings, list);
        }

        if (settings.layers.is_enabled(VisualizationLayer::trajectories))
        {
            draw_trajectories(camera, settings, list);
        }

        list.set_layer(-10);
        if (settings.layers.is_enabled(VisualizationLayer::bodies))
        {
            draw_visual_motion(world, camera, settings, list);
            draw_contact_shadows(world, camera, settings, list);
        }

        list.set_layer(0);
        std::size_t shaded_bodies = 0;
        world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                const auto placement = body.interpolated_transform(settings.interpolation_alpha);
                auto body_settings = settings;
                body_settings.material_shading = settings.material_shading && shaded_bodies++ < std::min<std::size_t>(settings.shading_body_budget, 4096);
                draw_body(id, body, placement, camera, body_settings, list, is_selected(id, settings), settings.layers.is_enabled(VisualizationLayer::labels) && !(id == settings.label_replaced) ? body_display_name(world, id) : std::string {});
            });

        list.set_layer(10);
        draw_visual_impacts(camera, settings, list);
        list.set_layer(20);
        {
            // Count the readings this frame will actually show: a scene of resting bodies has no
            // velocity plates competing for room, however many bodies it holds.
            const auto minimum = std::isfinite(settings.vector_scales.minimum_drawn_length_px) ? std::max(0.0, static_cast<double>(settings.vector_scales.minimum_drawn_length_px)) : 6.0;
            const auto shown = [&](const Vec2& value, double factor)
            {
                const auto length = vector_length(value) * factor;
                return std::isfinite(length) && length >= minimum ? 1u : 0u;
            };
            std::size_t readings = 0, loads_shown = 0, compact_shown = 0;
            BodyLoads loads;
            world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.type() == BodyType::static_body || (settings.vectors_selected_only && !(id == settings.selection)))
                        return;
                    if (settings.layers.is_enabled(VisualizationLayer::velocity_vectors))
                        readings += shown(body.linear_velocity_m_s(), settings.vector_scales.velocity);
                    if (settings.layers.is_enabled(VisualizationLayer::momentum_vectors))
                        readings += shown(body.linear_momentum_kg_m_s(), settings.vector_scales.momentum);
                    if (settings.layers.is_enabled(VisualizationLayer::acceleration_vectors))
                        readings += shown(body.applied_force_n() * body.inverse_mass(), settings.vector_scales.acceleration);
                    if (settings.layers.is_enabled(VisualizationLayer::force_vectors))
                    {
                        const auto contact = vector_length(contact_force_n(id)) > math::geometric_epsilon ? 1u : 0u;
                        collect_loads(id, body, loads);
                        keep_shown_loads(loads, false);
                        loads_shown += loads.count + contact;
                        if (!is_focus(id, settings))
                            keep_shown_loads(loads, true);
                        compact_shown += loads.count + contact;
                    }
                });
            compact_loads_ = loads_shown > 12u;
            readings += compact_loads_ ? compact_shown : loads_shown;
            sparse_vector_labels_ = readings * (settings.vector_components != VectorComponents::none ? 3u : 1u) > 12u;
        }
        // The selection's arrows come last, so they step aside from any receding arrow already
        // drawn along their line, such as a neighbour's own weight inside the neighbour that
        // pushes on the selection.
        for (const auto focus_pass : { false, true })
            world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if ((settings.vectors_selected_only && !(id == settings.selection)) || is_focus(id, settings) != focus_pass)
                        return;
                    const auto placement = body.interpolated_transform(settings.interpolation_alpha);
                    const auto center = math::transform_point(placement, body.mass_properties().center_of_mass_m);
                    draw_body_vectors(world, id, body, center, camera, settings, list);
                });

        draw_springs(world, camera, settings, list);
        draw_joints(world, camera, settings, list);
        draw_contacts(world, camera, settings, list);
        if (settings.layers.is_enabled(VisualizationLayer::grid) && settings.layers.is_enabled(VisualizationLayer::labels))
            draw_scale_key(camera, settings, list);
        draw_center_markers(settings, list);
        place_labels(camera, settings, list);
        list.set_layer(20);
    }

    void SceneRenderer::draw_background(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        if (!settings.depth_background)
            return;
        const auto width = static_cast<double>(camera.viewport().width);
        const auto height = static_cast<double>(camera.viewport().height);
        const auto& theme = settings.theme;
        // A faint fall-off from the top keeps the canvas calm but not flat.
        list.add_gradient_polygon_fill({ { 0.0, 0.0 }, { width, 0.0 }, { width, height }, { 0.0, height } }, theme.stage_highlight, theme.background, { 0.0, 0.0 }, { 0.0, height * 0.85 });
        if (!(theme.stage_vignette.alpha > 0.002f))
            return;
        auto focus = camera.focus_rect();
        if (focus.empty())
            focus = { 0.0, 0.0, width, height };
        const auto inset = std::min(focus.width, focus.height) * 0.28;
        const Rect inner { { focus.left + inset, focus.top + inset * 0.7 }, { focus.left + focus.width - inset, focus.top + focus.height - inset * 0.8 } };
        if (inner.maximum.x <= inner.minimum.x || inner.maximum.y <= inner.minimum.y)
            return;
        auto mesh = acquire_mesh();
        const auto edge = premultiplied(theme.stage_vignette);
        const Color clear { 0.0f, 0.0f, 0.0f, 0.0f };
        const std::array<Vec2, 4> outer_points { Vec2 { 0.0, 0.0 }, Vec2 { width, 0.0 }, Vec2 { width, height }, Vec2 { 0.0, height } };
        const std::array<Vec2, 4> inner_points { inner.minimum, Vec2 { inner.maximum.x, inner.minimum.y }, inner.maximum, Vec2 { inner.minimum.x, inner.maximum.y } };
        for (const auto& point : outer_points)
            add_vertex(*mesh, point, edge);
        for (const auto& point : inner_points)
            add_vertex(*mesh, point, clear);
        for (int index = 0; index < 4; ++index)
        {
            const auto next = (index + 1) % 4;
            add_triangle(*mesh, index, next, 4 + next);
            add_triangle(*mesh, index, 4 + next, 4 + index);
        }
        list.add_indexed_mesh(mesh);
    }

    void SceneRenderer::draw_joints(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        if (!settings.layers.is_enabled(VisualizationLayer::constraints))
            return;
        const auto ds = pixel_scale(settings);
        const auto standard = stroke(settings, 1.5f), hairline = stroke(settings, 1.0f), emphasis = stroke(settings, 2.0f);
        const auto& theme = settings.theme;
        const auto arc = [&](const Vec2& center, Real radius, Real start, Real span, const Color& color, float width)
        {
            std::vector<Vec2> points;
            const auto segments = std::clamp(static_cast<int>(std::ceil(std::abs(span) * radius / (3.0 * ds))), 8, 64);
            points.reserve(static_cast<std::size_t>(segments) + 1);
            for (int index = 0; index <= segments; ++index)
            {
                const auto angle = start + span * static_cast<Real>(index) / segments;
                points.push_back(center + Vec2 { std::cos(angle), -std::sin(angle) } * radius);
            }
            list.add_polyline(points, color, width);
            return points;
        };
        // A pin reads as a ringed pivot: a raised disc in the stage colour inside the joint ring.
        const auto pin = [&](const Vec2& point, const Color& color)
        {
            list.add_circle_fill(point, 3.5f * ds, color);
            list.add_circle_fill(point, 1.75f * ds, theme.center_of_mass);
        };
        std::uint64_t index = 0;
        for (const auto& constraint : world.constraints())
        {
            ++index;
            const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get());
            if (!joint)
                continue;
            const auto* first = world.find_body(joint->first_body());
            const auto* second = world.find_body(joint->second_body());
            if (!first || !second)
                continue;
            const auto report = joint->report(world);
            const auto first_pose = first->interpolated_transform(settings.interpolation_alpha);
            const auto second_pose = second->interpolated_transform(settings.interpolation_alpha);
            const auto& definition = joint->definition();
            const auto endpoints = std::visit([](const auto& value) -> const physics::JointEndpoints&
                {
                    return value;
                },
                definition);
            // Reports intentionally describe the last solved substep. Geometry always comes from
            // local anchors and interpolated poses, so the connection stays attached to the drawing.
            const auto first_anchor = math::transform_point(first_pose, endpoints.local_anchor_first_m);
            const auto second_anchor = math::transform_point(second_pose, endpoints.local_anchor_second_m);
            const auto first_point = camera.world_to_screen(first_anchor);
            const auto second_point = camera.world_to_screen(second_anchor);
            const auto midpoint = (first_point + second_point) * 0.5;
            const auto key = (std::uint64_t { 3 } << 56u) ^ index;
            // A broken link is drawn in the muted connection tone: in the force hue its torn ends
            // would read as force arrows.
            const auto color = report.enabled && !report.broken ? theme.panel_accent : theme.panel_muted;
            LabelRequest label;
            label.key = key;
            label.priority = 70;
            label.required = true;
            label.placement = LabelPlacement::around_point;
            label.anchor = midpoint;
            label.primary_color = theme.label_text;
            // A joint's plate is guaranteed only while one of its bodies is singled out; the
            // others give way to body names rather than crowd them.
            const auto attended = is_selected(joint->first_body(), settings) || is_selected(joint->second_body(), settings) || joint->first_body() == settings.hover || joint->second_body() == settings.hover;
            if (report.broken)
            {
                // Two short torn ends with a clear gap between them, so the load reads as free. A
                // torn end is drawn only outside the body it hangs from, and beneath the arrows,
                // where it cannot be taken for a part of that body's own arrows or cross them.
                const auto vector_layer = list.layer();
                list.set_layer(vector_layer - 1);
                const auto delta = second_point - first_point;
                const auto length = math::length(delta);
                const auto direction = length > 1.0e-9 ? delta / length : Vec2 { 0.0, 1.0 };
                const auto stub = std::min(0.35 * length, 24.0 * ds);
                const auto torn = first_point + direction * stub;
                const auto outside_from = [&](const physics::RigidBody& body, const math::Transform2& pose, const Vec2& from, const Vec2& to)
                {
                    const auto from_m = camera.screen_to_world(from), to_m = camera.screen_to_world(to);
                    if (!body.contains_world_point(from_m, pose))
                        return 0.0;
                    if (body.contains_world_point(to_m, pose))
                        return 1.0;
                    double inside = 0.0, outside = 1.0;
                    for (int iteration = 0; iteration < 12; ++iteration)
                    {
                        const auto middle = (inside + outside) * 0.5;
                        (body.contains_world_point(math::lerp(from_m, to_m, middle), pose) ? inside : outside) = middle;
                    }
                    return outside;
                };
                const auto torn_end = [&](const physics::RigidBody& body, const math::Transform2& pose, const Vec2& from, const Vec2& to)
                {
                    const auto start = math::lerp(from, to, outside_from(body, pose, from, to));
                    if (math::length(to - start) > 3.0 * ds)
                        list.add_line(start, to, color, standard);
                };
                torn_end(*first, first_pose, first_point, torn);
                torn_end(*second, second_pose, second_point, second_point - direction * stub);
                pin(first_point, color);
                pin(second_point, color);
                const auto normal = math::perpendicular(direction) * (5.0 * ds);
                const auto along = direction * (2.5 * ds);
                list.add_line(torn + along - normal, torn + along + normal, color, standard);
                list.add_line(torn + along * 3.0 - normal, torn + along * 3.0 + normal, color, standard);
                list.set_layer(vector_layer);
                // The load the link failed under and the limit it was set to are named apart, so
                // the failing load is never read as the limit.
                const auto force_failure = joint->broken_force_n() > endpoints.break_force_n;
                const auto torque_failure = joint->broken_torque_n_m() > endpoints.break_torque_n_m;
                const auto torque_text = [&](double value)
                {
                    return core::format_quantity(value, core::DisplayQuantity::torque, settings.display_units);
                };
                if (force_failure && torque_failure)
                    label.primary = core::substitute("Broken at {}, {} \xC2\xB7 limits {}, {}", force_text(joint->broken_force_n(), settings), torque_text(joint->broken_torque_n_m()), force_text(endpoints.break_force_n, settings), torque_text(endpoints.break_torque_n_m));
                else if (torque_failure)
                    label.primary = core::substitute("Broken at {} \xC2\xB7 torque limit {}", torque_text(joint->broken_torque_n_m()), torque_text(endpoints.break_torque_n_m));
                else
                    label.primary = core::substitute("Broken at {} \xC2\xB7 force limit {}", force_text(joint->broken_force_n(), settings), force_text(endpoints.break_force_n, settings));
                label.primary_color = theme.label_text;
                label.anchor = torn + along * 2.0;
                queue_label(std::move(label));
                continue;
            }
            if (!report.enabled)
            {
                list.add_dashed_line(first_point, second_point, color, hairline, 4.0f * ds, 3.0f * ds);
                pin(first_point, color);
                pin(second_point, color);
                label.primary = "Joint disabled";
                label.primary_color = theme.label_muted;
                queue_label(std::move(label));
                continue;
            }

            std::string_view name;
            // The drawn link a plate stands beside: the rod, the rail, or the weld's brackets.
            auto link_from = first_point, link_to = second_point;
            if (std::holds_alternative<physics::DistanceJointDefinition>(definition))
            {
                name = "Rod";
                list.add_line(first_point, second_point, color, standard);
                pin(first_point, color);
                pin(second_point, color);
            }
            else if (const auto* hinge = std::get_if<physics::RevoluteJointDefinition>(&definition))
            {
                name = hinge->motor_enabled ? "Hinge motor" : "Hinge";
                const auto center = camera.world_to_screen(math::transform_point(second_pose, second->mass_properties().center_of_mass_m));
                list.add_line(second_point, center, color.with_alpha(0.45f), hairline);
                const auto lever = second->mass_properties().center_of_mass_m - hinge->local_anchor_second_m;
                const auto local_angle = math::length_squared(lever) > 1.0e-12 ? std::atan2(lever.y, lever.x) : 0.0;
                const auto reference = first->interpolated_orientation_rad(settings.interpolation_alpha) + hinge->reference_angle_rad + local_angle;
                if (hinge->limits_enabled)
                {
                    const auto points = arc(midpoint, 32.0 * ds, reference + hinge->lower_angle_rad, math::clamp(hinge->upper_angle_rad - hinge->lower_angle_rad, 0.0, math::two_pi), theme.contact, standard);
                    list.add_line(midpoint, points.front(), theme.contact.with_alpha(0.55f), hairline);
                    list.add_line(midpoint, points.back(), theme.contact.with_alpha(0.55f), hairline);
                }
                if (hinge->motor_enabled)
                {
                    // The drive is part of the connection, so it takes the connection's hue and an
                    // open head, never a body vector's.
                    const auto span = hinge->motor_speed_rad_s >= 0.0 ? 0.9 * math::pi : -0.9 * math::pi;
                    const auto points = arc(midpoint, 20.0 * ds, reference, span, color, standard);
                    const auto tangent = math::normalized(points.back() - points[points.size() - 2]);
                    list.add_arrow(points.back() - tangent * (7.0 * ds), points.back() + tangent * (1.0 * ds), color, standard, 7.0f * ds, ArrowHead::open);
                }
                // Ringed pivot: accent ring, raised face, dark axle.
                list.add_circle_fill(midpoint, 7.0f * ds, theme.label_plate.with_alpha(0.92f));
                list.add_circle_outline(midpoint, 7.0f * ds, color, emphasis);
                list.add_circle_fill(midpoint, 2.25f * ds, color);
            }
            else if (const auto* slider = std::get_if<physics::PrismaticJointDefinition>(&definition))
            {
                name = slider->motor_enabled ? "Slider motor" : "Slider";
                const auto axis = math::normalized(math::transform_direction(first_pose, slider->local_axis_first));
                const auto translation = math::dot(second_anchor - first_anchor, axis);
                const auto lower = slider->limits_enabled ? slider->lower_translation_m : std::min(translation - 0.65, -0.65);
                const auto upper = slider->limits_enabled ? slider->upper_translation_m : std::max(translation + 0.65, 0.65);
                const auto start = camera.world_to_screen(first_anchor + axis * lower);
                const auto end = camera.world_to_screen(first_anchor + axis * upper);
                link_from = start;
                link_to = end;
                const auto screen_axis = math::normalized(camera.world_to_screen_direction(axis));
                const auto normal = math::perpendicular(screen_axis);
                // A rail: centre line between two guide rails.
                list.add_line(start, end, color, standard);
                list.add_line(start + normal * (4.0 * ds), end + normal * (4.0 * ds), color.with_alpha(0.4f), hairline, StrokeCap::butt);
                list.add_line(start - normal * (4.0 * ds), end - normal * (4.0 * ds), color.with_alpha(0.4f), hairline, StrokeCap::butt);
                list.add_rectangle_fill(second_point - Vec2 { 6.0 * ds, 6.0 * ds }, second_point + Vec2 { 6.0 * ds, 6.0 * ds }, theme.label_plate.with_alpha(0.92f));
                list.add_rectangle_outline(second_point - Vec2 { 6.0 * ds, 6.0 * ds }, second_point + Vec2 { 6.0 * ds, 6.0 * ds }, color, standard);
                list.add_circle_fill(second_point, 2.0f * ds, color);
                if (slider->limits_enabled)
                {
                    list.add_line(start - normal * (8.0 * ds), start + normal * (8.0 * ds), theme.contact, emphasis, StrokeCap::butt);
                    list.add_line(end - normal * (8.0 * ds), end + normal * (8.0 * ds), theme.contact, emphasis, StrokeCap::butt);
                }
                if (slider->motor_enabled)
                {
                    // The drive runs along the rail just ahead of the carriage, in the connection's
                    // hue, clear of the weight that hangs below the carriage's centre.
                    const auto direction = slider->motor_speed_m_s >= 0.0 ? screen_axis : -screen_axis;
                    double ahead = 6.0 * ds;
                    if (const auto bounds = second->compute_bounds(second_pose); !bounds.is_empty())
                        for (const auto& corner : { bounds.minimum, bounds.maximum, Vec2 { bounds.minimum.x, bounds.maximum.y }, Vec2 { bounds.maximum.x, bounds.minimum.y } })
                            ahead = std::max(ahead, math::dot(camera.world_to_screen(corner) - second_point, direction));
                    const auto from = second_point + direction * (ahead + 6.0 * ds);
                    list.add_arrow(from, from + direction * (22.0 * ds), color, emphasis, 8.0f * ds, ArrowHead::open);
                }
            }
            else
            {
                name = "Weld";
                // The weld point is shared, so a body it lies outside is reached by a rigid
                // bracket to that body's edge; without it the two parts look unconnected.
                const auto bracket = [&](const physics::RigidBody& body, const math::Transform2& pose, const Vec2& anchor_m)
                {
                    const auto center_m = math::transform_point(pose, body.mass_properties().center_of_mass_m);
                    if (body.contains_world_point(anchor_m, pose) || !body.contains_world_point(center_m, pose))
                        return midpoint;
                    double outside = 0.0, inside = 1.0;
                    for (int iteration = 0; iteration < 16; ++iteration)
                    {
                        const auto middle = (outside + inside) * 0.5;
                        (body.contains_world_point(math::lerp(anchor_m, center_m, middle), pose) ? inside : outside) = middle;
                    }
                    const auto edge = camera.world_to_screen(math::lerp(anchor_m, center_m, inside));
                    const auto reach = edge - midpoint;
                    const auto reach_length = math::length(reach);
                    if (!(reach_length > 1.0))
                        return midpoint;
                    const auto along = reach / reach_length;
                    const auto across = math::perpendicular(along) * (2.0 * ds);
                    const auto end = edge + along * (2.0 * ds);
                    list.add_polygon_fill({ midpoint + across, end + across, end - across, midpoint - across }, color.with_alpha(0.85f));
                    list.add_polygon_outline({ midpoint + across, end + across, end - across, midpoint - across }, theme.label_plate.with_alpha(0.6f), hairline);
                    list.add_rectangle_fill(edge - Vec2 { 3.0 * ds, 3.0 * ds }, edge + Vec2 { 3.0 * ds, 3.0 * ds }, color);
                    const std::array<Vec2, 4> footprint { midpoint + across * 2.0, end + across * 2.0, end - across * 2.0, midpoint - across * 2.0 };
                    add_obstacle(footprint.data(), footprint.size(), key);
                    return edge;
                };
                link_from = bracket(*first, first_pose, first_anchor);
                link_to = bracket(*second, second_pose, second_anchor);
                const std::vector<Vec2> diamond { midpoint + Vec2 { 0.0, -7.0 * ds }, midpoint + Vec2 { 7.0 * ds, 0.0 }, midpoint + Vec2 { 0.0, 7.0 * ds }, midpoint + Vec2 { -7.0 * ds, 0.0 } };
                list.add_polygon_fill(diamond, theme.label_plate.with_alpha(0.92f));
                list.add_polygon_outline(diamond, color, standard);
                list.add_polygon_fill({ midpoint + Vec2 { 0.0, -3.0 * ds }, midpoint + Vec2 { 3.0 * ds, 0.0 }, midpoint + Vec2 { 0.0, 3.0 * ds }, midpoint + Vec2 { -3.0 * ds, 0.0 } }, color);
            }
            // The link itself keeps plates off it, and a joint's pull is never drawn along it.
            if (std::holds_alternative<physics::DistanceJointDefinition>(definition))
            {
                const auto across = math::perpendicular(math::length_squared(second_point - first_point) > 1.0e-12 ? math::normalized(second_point - first_point) : Vec2 { 1.0, 0.0 }) * (2.0 * ds);
                const std::array<Vec2, 4> link { first_point + across, second_point + across, second_point - across, first_point - across };
                add_obstacle(link.data(), link.size(), key);
            }
            // A joint's pull on a body belongs to that body's free-body picture, so it is drawn for
            // a body singled out, at that body's own anchor. Like every load it takes the force
            // hue, with the outlined head of a load that is not a weight, and it stands
            // well beside the link it acts through. It stops short of the link's far end and of
            // any other body, so it can never be taken for a load on a neighbour.
            const auto reactions_shown = settings.layers.is_enabled(VisualizationLayer::force_vectors);
            const auto anchor_gap = math::length(second_point - first_point);
            bool arrow_states_force = false;
            if (reactions_shown)
            {
                if (drawn_arrows_.size() <= 256)
                    drawn_arrows_.push_back({ first_point, second_point, key });
                const std::array<std::tuple<physics::BodyId, Vec2, Vec2>, 2> ends { std::tuple { joint->first_body(), first_anchor, -report.reaction_force_n }, std::tuple { joint->second_body(), second_anchor, report.reaction_force_n } };
                for (const auto& [body_id, anchor_m, force] : ends)
                {
                    const auto* body = world.find_body(body_id);
                    const auto selected = is_focus(body_id, settings);
                    if (!body || body->type() != BodyType::dynamic_body || !(selected || (body_id == settings.hover && !settings.selection.is_valid())))
                        continue;
                    VectorOptions reaction;
                    reaction.significant = selected && vector_length(force) >= readable_force_n;
                    reaction.may_shift = true;
                    reaction.shift_step_px = 10.0 * ds;
                    reaction.stops_at_bodies = true;
                    reaction.focus = selected;
                    reaction.optional_label = !selected;
                    reaction.owner = body_key(body_id, 1);
                    if (body_id.index < body_frames_.size())
                    {
                        const auto& frame = body_frames_[body_id.index];
                        reaction.owner_box = frame.screen_box;
                        // An anchor at the centre of mass would hide the arrow's root under its
                        // symbol, so the shaft starts beyond it, as a load's does.
                        const auto centre = camera.world_to_screen(math::transform_point(body->interpolated_transform(settings.interpolation_alpha), body->mass_properties().center_of_mass_m));
                        if (frame.marker_radius_px > 0.0f && math::length(camera.world_to_screen(anchor_m) - centre) < static_cast<double>(frame.marker_radius_px))
                            reaction.start_offset_px = static_cast<double>(frame.marker_radius_px) + 1.0 * ds;
                    }
                    reaction.maximum_length_px = anchor_gap > 24.0 * ds ? 0.7 * anchor_gap - reaction.start_offset_px : 0.0;
                    // The force a singled-out body's reaction arrow states is left off the plate.
                    if (draw_scaled_vector(anchor_m, force, settings.vector_scales.force, theme.force, camera, settings, list, name, core::DisplayQuantity::force, vector_length(force), ArrowHead::hollow, 1.34f, key ^ (body_id == joint->first_body() ? 0x5a5a : 0xa5a5), reaction) && selected)
                        arrow_states_force = true;
                }
            }
            // A plate names only what this joint can carry: a rod pulls or pushes along its
            // length; torque appears where rotation is locked or driven and is not negligible.
            // A slider's couple is the one at the carriage it guides; the moment at its rail mount
            // grows with travel and is named apart where it can break the joint.
            const auto force_n = vector_length(report.reaction_force_n);
            const auto prismatic = report.kind == physics::JointKind::prismatic;
            const auto anchor_torque = prismatic ? std::abs(report.reaction_torque_n_m) : std::max(std::abs(report.reaction_torque_first_n_m), std::abs(report.reaction_torque_n_m));
            const auto mount_torque = prismatic && std::isfinite(endpoints.break_torque_n_m) ? std::abs(report.reaction_torque_first_n_m) : 0.0;
            const auto carries_torque = report.kind == physics::JointKind::weld || prismatic || (report.kind == physics::JointKind::revolute && (report.motor_enabled || report.limits_enabled));
            const auto loaded = force_n > 1.0e-3 || (carries_torque && std::max(anchor_torque, mount_torque) > 1.0e-4);
            const std::array<Vec2, 4> footprint { midpoint + Vec2 { -8.0 * ds, -8.0 * ds }, midpoint + Vec2 { 8.0 * ds, -8.0 * ds }, midpoint + Vec2 { 8.0 * ds, 8.0 * ds }, midpoint + Vec2 { -8.0 * ds, 8.0 * ds } };
            add_obstacle(footprint.data(), footprint.size(), key);
            if (!loaded && !attended)
                continue;
            std::string reading = arrow_states_force ? std::string {} : force_text(force_n, settings);
            const auto append = [&](const std::string& part)
            {
                reading = reading.empty() ? part : core::substitute("{} \xC2\xB7 {}", reading, part);
            };
            if (report.kind == physics::JointKind::distance && force_n > 1.0e-3)
            {
                const auto tension = math::dot(report.reaction_force_n, first_anchor - second_anchor) >= 0.0;
                reading = arrow_states_force ? std::string { tension ? "tension" : "compression" } : core::substitute("{} {}", tension ? "tension" : "compression", reading);
            }
            if (prismatic)
            {
                // The rail holds the carriage across its axis; the motor and stops act along it.
                const auto axis = math::length_squared(report.axis) > 0.0 ? math::normalized(report.axis) : Vec2 { 1.0, 0.0 };
                const auto along = math::dot(report.reaction_force_n, axis);
                reading = core::substitute("support {}", force_text(std::abs(math::cross(axis, report.reaction_force_n)), settings));
                if (report.motor_enabled)
                    reading = core::substitute("{} \xC2\xB7 motor {}", reading, force_text(std::abs(report.motor_force_n), settings));
                if (std::abs(along - report.motor_force_n) > 1.0e-3)
                    reading = core::substitute("{} \xC2\xB7 {} {}", reading, report.limits_enabled ? "stop" : "along rail", force_text(std::abs(along - report.motor_force_n), settings));
            }
            if (carries_torque && anchor_torque > 1.0e-4)
                append(core::substitute("torque {}", core::format_quantity(anchor_torque, core::DisplayQuantity::torque, settings.display_units)));
            if (mount_torque > 1.0e-4)
                append(core::substitute("rail torque {}", core::format_quantity(mount_torque, core::DisplayQuantity::torque, settings.display_units)));
            label.primary = loaded && !reading.empty() ? core::substitute("{} \xC2\xB7 {}", name, reading) : std::string { name };
            label.required = attended;
            label.priority = attended ? 70 : 30;
            if (report.kind == physics::JointKind::revolute)
            {
                label.placement = LabelPlacement::around_box;
                label.box.expand(midpoint - Vec2 { 34.0 * ds, 34.0 * ds });
                label.box.expand(midpoint + Vec2 { 34.0 * ds, 34.0 * ds });
            }
            else if (math::length(link_to - link_from) > 16.0 * ds)
            {
                // A rod or a weld's bracket is named beside it, never across it.
                label.placement = LabelPlacement::beside_line;
                label.origin = link_from;
                label.direction = link_to - link_from;
                label.clearance = 6.0 * ds;
            }
            queue_label(std::move(label));
        }
    }

    void SceneRenderer::draw_springs(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        if (!settings.layers.is_enabled(VisualizationLayer::constraints))
            return;
        const auto ds = pixel_scale(settings);
        const auto standard = stroke(settings, 1.5f), hairline = stroke(settings, 1.0f), emphasis_width = stroke(settings, 2.0f);
        const auto& theme = settings.theme;
        std::uint64_t index = 0;
        for (const auto id : world.spring_ids())
        {
            ++index;
            const auto* definition = world.spring_definition(id);
            const auto report = world.spring_report(id);
            if (!definition || !report || !report->enabled)
                continue;
            const auto* first = world.find_body(report->first);
            const auto* second = world.find_body(report->second);
            if (!first || !second)
                continue;
            const auto key = (std::uint64_t { 4 } << 56u) ^ index;
            const auto first_pose = first->interpolated_transform(settings.interpolation_alpha);
            const auto second_pose = second->interpolated_transform(settings.interpolation_alpha);
            if (const auto* linear = std::get_if<physics::LinearSpringDefinition>(definition))
            {
                const auto first_anchor_m = math::transform_point(first_pose, linear->local_anchor_first_m);
                const auto second_anchor_m = math::transform_point(second_pose, linear->local_anchor_second_m);
                const auto first_point = camera.world_to_screen(first_anchor_m);
                const auto second_point = camera.world_to_screen(second_anchor_m);
                const auto delta = second_point - first_point;
                const auto length = math::length(delta);
                const auto direction = length > 1.0e-9 ? delta / length : Vec2 { 1.0, 0.0 };
                const auto side = math::perpendicular(direction);
                // The coil is drawn in the neutral connection tone so that a force or velocity
                // arrow lying along it stays readable; its reading says in words whether it is
                // stretched or squeezed.
                const auto color = theme.panel_muted;
                // The coil occupies only the free gap between the bodies; inside a body the
                // spring continues as a straight lead to its anchor pin.
                const auto exit_fraction = [](const physics::RigidBody& body, const math::Transform2& pose, const Vec2& from, const Vec2& to)
                {
                    if (!body.contains_world_point(from, pose))
                        return 0.0;
                    if (body.contains_world_point(to, pose))
                        return 1.0;
                    double inside = 0.0, outside = 1.0;
                    for (int iteration = 0; iteration < 14; ++iteration)
                    {
                        const auto middle = (inside + outside) * 0.5;
                        (body.contains_world_point(math::lerp(from, to, middle), pose) ? inside : outside) = middle;
                    }
                    return outside;
                };
                auto free_start = exit_fraction(*first, first_pose, first_anchor_m, second_anchor_m);
                auto free_end = 1.0 - exit_fraction(*second, second_pose, second_anchor_m, first_anchor_m);
                if ((free_end - free_start) * length < 24.0 * ds)
                {
                    free_start = 0.0;
                    free_end = 1.0;
                }
                const auto gap_length = (free_end - free_start) * length;
                const auto damped = linear->damping_n_s_m > 0.0 && gap_length > 40.0 * ds;
                // Straight leads, then a zigzag of fixed coil count so stretching reads as
                // spacing. A damped spring runs beside a dashpot between shared end bars.
                const auto lead = std::min(gap_length * 0.14, 8.0 * ds);
                const auto amplitude = damped ? std::min(3.5 * ds, gap_length * 0.07) : std::min(5.0 * ds, gap_length * 0.09);
                const auto offset = damped ? side * (7.0 * ds) : Vec2 {};
                std::vector<Vec2> points { first_point };
                const auto coil_start = math::lerp(first_point, second_point, free_start) + direction * lead;
                const auto coil_end = math::lerp(first_point, second_point, free_end) - direction * lead;
                points.push_back(coil_start);
                if (damped)
                    points.push_back(coil_start + offset);
                constexpr int peaks = 10;
                for (int peak = 0; peak < peaks; ++peak)
                {
                    const auto fraction = (static_cast<double>(peak) + 0.5) / peaks;
                    points.push_back(math::lerp(coil_start, coil_end, fraction) + offset + side * (peak % 2 == 0 ? amplitude : -amplitude));
                }
                if (damped)
                    points.push_back(coil_end + offset);
                points.push_back(coil_end);
                points.push_back(second_point);
                list.add_path(points, color, standard, false, StrokeJoin::miter, StrokeCap::round);
                if (damped)
                {
                    // Dashpot: a cylinder open toward the far bar, a piston and its rod.
                    const auto down = -offset;
                    list.add_line(coil_start, coil_start + down, color, standard, StrokeCap::butt);
                    list.add_line(coil_end, coil_end + down, color, standard, StrokeCap::butt);
                    const auto cylinder_start = math::lerp(coil_start, coil_end, 0.16) + down;
                    const auto cylinder_end = math::lerp(coil_start, coil_end, 0.64) + down;
                    const auto piston = math::lerp(coil_start, coil_end, 0.46) + down;
                    const auto width = side * (4.0 * ds);
                    list.add_line(coil_start + down, cylinder_start, color, standard, StrokeCap::butt);
                    list.add_path({ cylinder_end + width, cylinder_start + width, cylinder_start - width, cylinder_end - width }, color, standard, false, StrokeJoin::miter, StrokeCap::butt);
                    list.add_line(piston + width * 0.7, piston - width * 0.7, color, emphasis_width, StrokeCap::butt);
                    list.add_line(piston, coil_end + down, color, standard, StrokeCap::butt);
                }
                for (const auto& end : { first_point, second_point })
                {
                    list.add_circle_fill(end, 3.0f * ds, theme.label_plate.with_alpha(0.95f));
                    list.add_circle_outline(end, 3.0f * ds, color, standard);
                }
                // The same present-state reading the spring's arrows on its bodies are drawn from.
                // While one of its bodies is singled out, that body's spring arrow states the
                // force, so the plate keeps only the stretch. The plate is guaranteed only then;
                // otherwise it stands beside the coil where there is room, never over the coil,
                // its anchor or its mass.
                const auto tension = math::dot(report->force_on_first_n, math::normalized(report->second_anchor_m - report->first_anchor_m));
                const auto attended = is_selected(report->first, settings) || is_selected(report->second, settings) || report->first == settings.hover || report->second == settings.hover;
                const auto stated_on = [&](physics::BodyId body)
                {
                    return is_focus(body, settings) && body.index < spring_stated_.size() && spring_stated_[body.index] != 0 && body.index < spring_counts_.size() && spring_counts_[body.index] == 1;
                };
                const auto arrow_states_force = stated_on(report->first) || stated_on(report->second);
                const auto stretch = core::substitute("{} {}", report->extension_m < 0.0 ? "compression" : "extension", core::format_quantity(std::abs(report->extension_m), core::DisplayQuantity::length, settings.display_units));
                LabelRequest label;
                label.key = key;
                label.priority = attended ? 68 : 32;
                label.required = attended;
                label.placement = LabelPlacement::beside_line;
                label.origin = first_point;
                label.direction = second_point - first_point;
                label.anchor = (first_point + second_point) * 0.5;
                label.clearance = (damped ? 11.0 * ds : amplitude) + 4.0 * ds;
                label.primary = arrow_states_force ? stretch : core::substitute("{} \xC2\xB7 {} {}", stretch, tension < 0.0 ? "push" : "pull", force_text(std::abs(tension), settings));
                label.primary_color = theme.label_text;
                queue_label(std::move(label));
                const auto above = damped ? 7.0 * ds + amplitude + 2.0 * ds : amplitude + 3.0 * ds;
                const auto below = damped ? 7.0 * ds + 6.0 * ds : amplitude + 3.0 * ds;
                const std::array<Vec2, 4> footprint { first_point + side * above, second_point + side * above, second_point - side * below, first_point - side * below };
                add_obstacle(footprint.data(), footprint.size(), key);
            }
            else
            {
                const auto center = camera.world_to_screen(math::transform_point(second_pose, second->mass_properties().center_of_mass_m));
                const auto a = std::get<physics::AngularSpringDefinition>(*definition);
                const auto reference_angle = first->interpolated_orientation_rad(settings.interpolation_alpha) + a.rest_angle_rad;
                const auto current_angle = second->interpolated_orientation_rad(settings.interpolation_alpha);
                const auto deflection = math::clamp(current_angle - reference_angle, -math::two_pi, math::two_pi);
                const auto on_screen = [&](double angle, double radius)
                {
                    return center + Vec2 { std::cos(angle), -std::sin(angle) } * radius;
                };
                // A clock spring: a spiral of two and a half turns whose inner end turns with the
                // body and whose outer end is held at the rest angle, so it visibly winds up as the
                // body is turned. It starts outside the centre-of-mass symbol and is drawn in the
                // connection tone of the other springs.
                const auto inner = 7.0 * ds, outer = 18.0 * ds;
                const auto turns = 2.5 * math::two_pi;
                std::vector<Vec2> spiral;
                spiral.reserve(97);
                for (int step = 0; step <= 96; ++step)
                {
                    const auto t = static_cast<double>(step) / 96.0;
                    spiral.push_back(on_screen(current_angle + (reference_angle - turns - current_angle) * t, inner + (outer - inner) * t));
                }
                list.add_polyline(spiral, theme.panel_muted, standard);
                list.add_line(spiral.back(), on_screen(reference_angle - turns, outer + 5.0 * ds), theme.panel_muted, standard);
                // The deflection from rest, opened to a readable sweep when it is small; the plate
                // keeps the true angle.
                const double radius = 25.0 * ds;
                const auto minimum_sweep = math::degrees_to_radians(12.0);
                const auto magnified = std::abs(deflection) > math::degrees_to_radians(0.2) && std::abs(deflection) < minimum_sweep;
                const auto sweep = magnified ? std::copysign(minimum_sweep, deflection) : deflection;
                list.add_dashed_line(on_screen(reference_angle, outer + 2.0 * ds), on_screen(reference_angle, radius + 5.0 * ds), theme.panel_muted, hairline, 3.0f * ds, 2.0f * ds);
                list.add_line(on_screen(reference_angle + sweep, outer + 2.0 * ds), on_screen(reference_angle + sweep, radius + 5.0 * ds), theme.momentum, standard);
                if (std::abs(sweep) > 1.0e-4)
                {
                    std::vector<Vec2> arc;
                    for (int step = 0; step <= 24; ++step)
                        arc.push_back(on_screen(reference_angle + sweep * static_cast<double>(step) / 24.0, radius));
                    list.add_polyline(arc, theme.momentum, hairline);
                    if (std::abs(sweep) * radius > 10.0 * ds)
                    {
                        const auto tangent = math::normalized(arc.back() - arc[arc.size() - 2]);
                        list.add_arrow(arc.back() - tangent * (6.0 * ds), arc.back() + tangent * (1.0 * ds), theme.momentum, hairline, 6.0f * ds);
                    }
                }
                // The plate's leader ends on the spiral's held outer end, so it names the spring
                // rather than an empty point beside it.
                LabelRequest label;
                label.key = key;
                label.priority = 68;
                label.required = true;
                label.placement = LabelPlacement::around_point;
                label.anchor = on_screen(reference_angle - turns, outer + 5.0 * ds);
                label.primary = core::substitute("angle {} \xC2\xB7 torque {}", core::format_quantity(math::radians_to_degrees(report->angular_displacement_rad), core::DisplayQuantity::angle, settings.display_units), core::format_quantity(report->torque_on_second_n_m, core::DisplayQuantity::torque, settings.display_units));
                label.primary_color = theme.momentum;
                if (magnified)
                {
                    label.secondary = "arc not to scale";
                    label.secondary_color = theme.label_muted;
                }
                queue_label(std::move(label));
                const std::array<Vec2, 4> footprint { center + Vec2 { -radius, -radius }, center + Vec2 { radius, -radius }, center + Vec2 { radius, radius }, center + Vec2 { -radius, radius } };
                add_obstacle(footprint.data(), footprint.size(), key);
            }
        }
    }

    void SceneRenderer::record_trajectory_sample(const physics::World& world, std::size_t maximum_samples)
    {
        world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.type() == BodyType::static_body)
                {
                    return;
                }

                auto& trail = trajectories_[id.index];
                if (trail.generation != id.generation)
                {
                    trail.generation = id.generation;
                    trail.samples.clear();
                }

                trail.samples.push_back(body.world_center_of_mass_m());
                while (trail.samples.size() > maximum_samples)
                {
                    trail.samples.pop_front();
                }
            });
    }

    void SceneRenderer::clear_trajectories()
    {
        trajectories_.clear();
        clear_visual_effects();
    }

    void SceneRenderer::set_body_names(std::vector<std::pair<physics::BodyId, std::string>> names)
    {
        std::sort(names.begin(), names.end(), [](const auto& left, const auto& right)
            {
                return left.first.index < right.first.index;
            });
        body_names_ = std::move(names);
    }

    void SceneRenderer::set_overlay_areas(const std::vector<std::pair<Vec2, Vec2>>& areas)
    {
        overlay_label_areas_.assign(areas.begin(), areas.end());
    }

    void SceneRenderer::set_overlay_rings(const std::vector<std::pair<Vec2, double>>& rings)
    {
        overlay_rings_.assign(rings.begin(), rings.end());
    }

    void SceneRenderer::set_visible_stage(const ScreenRect& stage)
    {
        visible_stage_ = stage;
    }

    void SceneRenderer::draw_grid(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        const auto visible = camera.visible_bounds_m();
        if (visible.is_empty() || !math::is_finite(visible.minimum) || !math::is_finite(visible.maximum))
        {
            return;
        }
        const auto ds = static_cast<double>(pixel_scale(settings));
        const auto pixels_per_metre = camera.pixels_per_metre();
        if (!std::isfinite(pixels_per_metre) || pixels_per_metre <= 0.0)
            return;
        // Decade levels fade in by their on-screen spacing, so zooming never pops lines in or
        // out: the finest level appears from 7 logical pixels apart and is fully present at 22;
        // a level is emphasised as a major line between 70 and 220 logical pixels.
        const auto fade_start = 7.0 * ds, fade_full = 22.0 * ds, emphasis_start = 70.0 * ds, emphasis_full = 220.0 * ds;
        const auto spacing = std::pow(10.0, std::ceil(std::log10(fade_start / pixels_per_metre)));
        if (!std::isfinite(spacing) || spacing <= 0.0)
            return;
        const auto first_x = std::floor(visible.minimum.x / spacing) * spacing;
        const auto first_y = std::floor(visible.minimum.y / spacing) * spacing;
        const auto viewport = camera.viewport();
        // Beyond this range a grid cell may be smaller than the coordinate's ULP. Avoid both
        // a nonadvancing floating-point loop and an out-of-range llround conversion. The bound
        // is deliberately inside the signed integer range, including after double rounding.
        constexpr double maximum_grid_index = 0x1p62;
        const auto representable_axis = [&](double first, double last)
        {
            return std::isfinite(first) && std::isfinite(first + spacing) && first + spacing > first &&
                std::isfinite(first / spacing) && std::isfinite(last / spacing) &&
                std::abs(first / spacing) < maximum_grid_index && std::abs(last / spacing) < maximum_grid_index;
        };
        if (!representable_axis(first_x, visible.maximum.x) || !representable_axis(first_y, visible.maximum.y))
            return;

        const auto& theme = settings.theme;
        const auto line_color = [&](long long index) -> Color
        {
            auto level = spacing;
            if (index % 100 == 0)
                level *= 100.0;
            else if (index % 10 == 0)
                level *= 10.0;
            const auto screen = level * pixels_per_metre;
            const auto presence = static_cast<float>(smoothstep(fade_start, fade_full, screen));
            const auto emphasis = static_cast<float>(smoothstep(emphasis_start, emphasis_full, screen));
            auto color = mix(theme.grid_minor, theme.grid_major, emphasis);
            color.alpha *= presence;
            return color;
        };
        constexpr std::size_t maximum_axis_lines = 4096;
        const auto half = static_cast<double>(stroke(settings, 1.0f)) * 0.5;
        const auto feather = ds;
        auto mesh = acquire_mesh();
        const auto width = static_cast<double>(viewport.width), height = static_cast<double>(viewport.height);
        const auto draw_axis = [&](double first, double last, bool horizontal)
        {
            auto coordinate = first;
            for (std::size_t line = 0; line < maximum_axis_lines && coordinate <= last; ++line)
            {
                const auto index = std::llround(coordinate / spacing);
                const auto color = line_color(index);
                const auto screen = camera.world_to_screen(horizontal ? Vec2 { camera.center_m().x, coordinate } : Vec2 { coordinate, camera.center_m().y });
                // The origin lines are drawn once as axes.
                if (index != 0 && color.alpha > 0.002f && math::is_finite(screen))
                {
                    if (horizontal)
                        append_soft_segment(*mesh, { 0.0, screen.y }, { width, screen.y }, half, feather, color, color);
                    else
                        append_soft_segment(*mesh, { screen.x, 0.0 }, { screen.x, height }, half, feather, color, color);
                }
                const auto next = coordinate + spacing;
                if (!std::isfinite(next) || next <= coordinate)
                    break;
                coordinate = next;
            }
        };
        draw_axis(first_x, visible.maximum.x, false);
        draw_axis(first_y, visible.maximum.y, true);
        if (!mesh->indices.empty())
            list.add_indexed_mesh(mesh);

        const auto origin = camera.world_to_screen({ 0.0, 0.0 });
        if (!math::is_finite(origin))
            return;
        if (origin.y >= -2.0 && origin.y <= height + 2.0)
            list.add_line({ 0.0, origin.y }, { width, origin.y }, theme.axis, stroke(settings, 1.0f), StrokeCap::butt);
        if (origin.x >= -2.0 && origin.x <= width + 2.0)
            list.add_line({ origin.x, 0.0 }, { origin.x, height }, theme.axis, stroke(settings, 1.0f), StrokeCap::butt);
    }

    void SceneRenderer::draw_scale_key(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list)
    {
        const auto ds = static_cast<double>(pixel_scale(settings));
        const auto pixels_per_metre = camera.pixels_per_metre();
        if (!std::isfinite(pixels_per_metre) || pixels_per_metre <= 0.0)
            return;
        const auto& theme = settings.theme;
        const auto width = static_cast<double>(camera.viewport().width), height = static_cast<double>(camera.viewport().height);
        // A scale bar of a round 1-2-5 length, tucked into a corner of the stage.
        auto focus = camera.focus_rect();
        if (focus.empty())
            focus = { 0.0, 0.0, width, height };
        const auto target = 64.0 * ds / pixels_per_metre;
        const auto magnitude = std::pow(10.0, std::floor(std::log10(target)));
        auto bar_m = magnitude;
        for (const auto multiple : { 1.0, 2.0, 5.0, 10.0 })
            if (magnitude * multiple * pixels_per_metre <= 110.0 * ds)
                bar_m = magnitude * multiple;
        const auto bar_px = bar_m * pixels_per_metre;
        if (!std::isfinite(bar_px) || bar_px < 8.0 * ds || bar_px > focus.width * 0.5)
            return;
        const auto metrics = label_metrics(settings);
        const auto text = core::format_quantity(bar_m, core::DisplayQuantity::length, settings.display_units);
        const auto text_px = text_width(text, metrics.scale);
        // Force arrows are drawn to a scale fitted to the scene's weights, so a key beside the
        // length bar states it: an arrow of a round force value at the drawn scale.
        // It is left out while no force has set the scale, as before the first step of a scene
        // without gravity, so it never shows a value that is about to change.
        double key_n = 0.0, key_px = 0.0;
        std::string key_text;
        const auto scaled = settings.vector_scales.automatic || heaviest_weight_n_ > math::geometric_epsilon || peak_applied_n_ > math::geometric_epsilon;
        if (settings.layers.is_enabled(VisualizationLayer::force_vectors) && scaled && std::isfinite(drawn_force_scale_) && drawn_force_scale_ > 0.0)
        {
            const auto most = 56.0 * ds / drawn_force_scale_;
            const auto decade = std::pow(10.0, std::floor(std::log10(most)));
            for (const auto multiple : { 1.0, 2.0, 5.0 })
                if (decade * multiple <= most)
                    key_n = decade * multiple;
            key_px = key_n * drawn_force_scale_;
            if (std::isfinite(key_px) && key_px >= 12.0 * ds)
            {
                // A round value reads round: '50 mN' and '0.5 N', in the unit of the scene's plates.
                const auto shown = settings.display_units == core::DisplayUnits::si && force_milli_ ? key_n * 1000.0 : key_n;
                if (settings.display_units != core::DisplayUnits::si || shown >= 10000.0)
                    key_text = core::format_quantity(key_n, core::DisplayQuantity::force, settings.display_units);
                else
                {
                    auto figure = core::fixed(shown, std::clamp(-static_cast<int>(std::floor(std::log10(shown))), 0, 9));
                    if (figure.find('.') != std::string::npos)
                    {
                        while (figure.back() == '0')
                            figure.pop_back();
                        if (figure.back() == '.')
                            figure.pop_back();
                    }
                    key_text = figure + (force_milli_ ? "\xC2\xA0mN" : "\xC2\xA0N");
                }
            }
        }
        // After a scene without gravity rescales its forces, the key says so for a moment.
        const auto rescaled = !key_text.empty() && rescaled_note_s_ > 0.0 && !(heaviest_weight_n_ > math::geometric_epsilon);
        const std::string_view rescaled_text = "rescaled";
        const auto key_width = key_text.empty() ? 0.0 : 14.0 * ds + key_px + 6.0 * ds + text_width(key_text, metrics.scale) + (rescaled ? 8.0 * ds + text_width(rescaled_text, metrics.scale) : 0.0);
        const Vec2 plate_size { metrics.pad_x * 2.0 + bar_px + 8.0 * ds + text_px + key_width, metrics.height };
        // Lower left, lower right, then upper left: the upper right holds the gravity dial. A
        // corner counts as taken by any arrow, moving body or overlay inside it.
        const auto corner_rect = [&](int corner)
        {
            const auto left = corner == 1 ? focus.left + focus.width - 10.0 * ds - plate_size.x : focus.left + 10.0 * ds;
            const auto top = corner == 2 ? focus.top + 10.0 * ds : focus.top + focus.height - 10.0 * ds - metrics.height;
            return Rect { { left, top }, { left + plate_size.x, top + plate_size.y } };
        };
        const auto taken = [&](const Rect& rect)
        {
            const Rect padded { rect.minimum - Vec2 { 4.0 * ds, 4.0 * ds }, rect.maximum + Vec2 { 4.0 * ds, 4.0 * ds } };
            std::size_t count = 0;
            for (const auto& arrow : drawn_arrows_)
            {
                const std::array<Vec2, 2> segment { arrow.start, arrow.end };
                count += rect_hits_polygon(padded, segment.data(), segment.size()) ? 1u : 0u;
            }
            for (const auto& frame : body_frames_)
                if (frame.movable && !frame.screen_box.is_empty() && overlap_area(padded, { frame.screen_box.minimum, frame.screen_box.maximum }) > 0.0)
                    ++count;
            for (const auto& [minimum, maximum] : overlay_label_areas_)
                count += overlap_area(padded, { minimum, maximum }) > 0.0 ? 1u : 0u;
            return count;
        };
        auto chosen = std::clamp(key_corner_, 0, 2);
        if (taken(corner_rect(chosen)) > 0)
        {
            auto fewest = taken(corner_rect(chosen));
            for (const auto corner : { 0, 1, 2 })
                if (const auto count = taken(corner_rect(corner)); count < fewest)
                {
                    fewest = count;
                    chosen = corner;
                }
        }
        // The key returns home once the lower-left corner has stayed free for a moment, so a
        // passing arrow does not leave it in another corner for good, nor set it flickering.
        key_home_free_frames_ = chosen != 0 && taken(corner_rect(0)) == 0 ? key_home_free_frames_ + 1 : 0;
        if (key_home_free_frames_ > 45)
        {
            chosen = 0;
            key_home_free_frames_ = 0;
        }
        key_corner_ = chosen;
        const auto plate = corner_rect(chosen);
        const auto grid_layer = list.layer();
        list.set_layer(instrument_layer);
        list.add_rounded_rectangle_fill(plate.minimum, plate.maximum, static_cast<float>(metrics.radius), theme.label_plate);
        const auto bar_y = std::round(plate.minimum.y + metrics.height * 0.5) + 0.5;
        const auto bar_left = plate.minimum.x + metrics.pad_x;
        const auto tick = 3.0 * ds;
        list.add_path({ { bar_left, bar_y - tick }, { bar_left, bar_y }, { bar_left + bar_px, bar_y }, { bar_left + bar_px, bar_y - tick } }, theme.label_muted, stroke(settings, 1.0f), false, StrokeJoin::miter, StrokeCap::butt);
        const auto text_y = plate.minimum.y + (metrics.height + metrics.cap_height) * 0.5 - metrics.ascender;
        list.add_text({ bar_left + bar_px + 8.0 * ds, std::round(text_y) }, text, theme.label_muted, metrics.scale);
        if (!key_text.empty())
        {
            const auto arrow_left = bar_left + bar_px + 8.0 * ds + text_px + 14.0 * ds;
            const auto arrow_y = bar_y - 0.5;
            list.add_arrow({ arrow_left, arrow_y }, { arrow_left + key_px, arrow_y }, theme.force, stroke(settings, 1.5f), static_cast<float>(7.0 * ds));
            list.add_text({ arrow_left + key_px + 6.0 * ds, std::round(text_y) }, key_text, theme.label_muted, metrics.scale);
            if (rescaled)
                list.add_text({ arrow_left + key_px + 14.0 * ds + text_width(key_text, metrics.scale), std::round(text_y) }, rescaled_text, theme.label_text, metrics.scale);
        }
        list.set_layer(grid_layer);
        reserved_label_areas_.emplace_back(plate.minimum, plate.maximum);
    }

    void SceneRenderer::draw_body(physics::BodyId id, const physics::RigidBody& body, const math::Transform2& body_placement, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list, bool selected, std::string_view display_name) const
    {
        const auto& theme = settings.theme;
        const auto colors = colors_for(body, theme);
        const auto scale = pixel_scale(settings);
        const auto ds = static_cast<double>(scale);
        const auto is_static = body.type() == BodyType::static_body;
        const auto is_kinematic = body.type() == BodyType::kinematic_body;
        const auto hovered = id == settings.hover && !selected;
        const auto show_bodies = settings.layers.is_enabled(VisualizationLayer::bodies);
        const auto show_outlines = settings.layers.is_enabled(VisualizationLayer::outlines);
        const auto shaded = settings.material_shading && show_bodies;
        const auto body_layer = is_static ? -8 : 0;
        const auto key = body_key(id, 1);
        const auto mass_center = math::transform_point(body_placement, body.mass_properties().center_of_mass_m);
        const auto project = [&](const Vec2& world_point)
        {
            return camera.world_to_screen(deform_point(id, world_point, mass_center, settings.soft_deformation, settings.deformation_budget));
        };

        // Shared lighting frame: the body's projection on the light direction.
        BodyLighting lighting;
        math::Aabb screen_box;
        {
            const auto bounds = body.compute_bounds(body_placement);
            if (!bounds.is_empty())
            {
                const auto a = camera.world_to_screen(bounds.minimum), b = camera.world_to_screen(bounds.maximum);
                screen_box.expand(a);
                screen_box.expand(b);
                lighting.t_min = 1.0e300;
                lighting.t_max = -1.0e300;
                for (const auto& corner : { a, b, Vec2 { a.x, b.y }, Vec2 { b.x, a.y } })
                {
                    lighting.t_min = std::min(lighting.t_min, math::dot(corner, toward_light));
                    lighting.t_max = std::max(lighting.t_max, math::dot(corner, toward_light));
                }
            }
        }
        const auto selection_color = theme.selection;
        const auto glow_layer = is_static ? -9 : -4;
        const auto line_color = selected ? selection_color : selection_color.with_alpha(selection_color.alpha * 0.55f);
        const auto line_width = selected ? stroke(settings, 2.0f) : stroke(settings, 1.5f);
        // A body of several parts is outlined once around all of them, so the seams between its
        // parts never read as several selected objects.
        const auto compound = (selected || hovered) && body.colliders().size() > 1;
        thread_local std::vector<std::vector<Vec2>> part_outlines;
        std::size_t part_count = 0;
        const auto highlight = [&](const std::vector<Vec2>& outline, bool closed_shape, const Vec2* circle_center, double circle_radius)
        {
            if (!selected && !hovered)
                return;
            const auto previous = list.layer();
            // A soft halo under the body, then an accent line that follows the outline.
            list.set_layer(glow_layer);
            const auto glow = selection_color.with_alpha(selection_color.alpha * (selected ? 0.32f : 0.14f));
            if (circle_center)
                list.add_circle_shadow(*circle_center, static_cast<float>(circle_radius), glow, 7.0f * scale, {});
            else if (closed_shape && outline.size() >= 3 && nearly_convex(outline))
                list.add_shadow(outline, glow, 7.0f * scale, {});
            else if (closed_shape && outline.size() >= 3)
                list.add_polyline(outline, glow.with_alpha(glow.alpha * 0.7f), 7.0f * scale, true);
            list.set_layer(previous);
            if (compound && closed_shape && outline.size() >= 3)
            {
                if (part_count == part_outlines.size())
                    part_outlines.emplace_back();
                part_outlines[part_count++].assign(outline.begin(), outline.end());
                return;
            }
            list.set_layer(5);
            const auto color = line_color;
            const auto width = line_width;
            const auto gap = 1.0 * ds + width * 0.5;
            if (circle_center)
                list.add_circle_outline(*circle_center, static_cast<float>(circle_radius + gap), color, width);
            else if (closed_shape && outline.size() >= 3)
            {
                thread_local std::vector<Vec2> expanded;
                offset_outline(outline, gap, expanded);
                list.add_polyline(expanded, color, width, true);
            }
            else if (outline.size() >= 2)
                list.add_polyline(outline, color, width + 2.0f * scale);
            list.set_layer(previous);
        };
        const auto obstacle = [&](const std::vector<Vec2>& outline)
        {
            if (outline.size() <= 24)
                add_obstacle(outline.data(), outline.size(), key);
            else
            {
                std::array<Vec2, 12> reduced {};
                for (std::size_t index = 0; index < reduced.size(); ++index)
                    reduced[index] = outline[index * outline.size() / reduced.size()];
                add_obstacle(reduced.data(), reduced.size(), key);
            }
        };

        // A surface mesh per body keeps a compound body one draw item.
        std::shared_ptr<IndexedMesh> mesh;
        bool mesh_listed = false;
        const auto surface_mesh = [&]() -> IndexedMesh&
        {
            if (!mesh)
                mesh = acquire_mesh();
            return *mesh;
        };
        // The list shares the mesh, so later colliders may still append to it; listing it as
        // soon as it has content keeps per-collider details such as index marks above it.
        const auto commit = [&]
        {
            if (!mesh || mesh_listed || mesh->indices.empty())
                return;
            const auto previous = list.layer();
            list.set_layer(body_layer);
            list.add_indexed_mesh(mesh);
            list.set_layer(previous);
            mesh_listed = true;
        };
        const auto feather = static_cast<double>(scale);
        thread_local std::vector<Vec2> inset, hull, cavity;
        // A shelled part is hollow: its cavity is drawn recessed, lit on the far wall, inside an
        // inner rim. Collision still uses the outer envelope, so the cavity is not see-through.
        const auto draw_cavity = [&](const std::vector<Vec2>& outline, const MaterialAppearance& look, const Color& rim)
        {
            if (outline.size() < 3)
                return;
            const auto recess = mix(theme.background, look.shade, theme.light_stage ? 0.5f : 0.3f).with_alpha(1.0f);
            const auto recess_lit = mix(recess, look.surface, 0.25f);
            double low = 1.0e300, high = -1.0e300;
            for (const auto& point : outline)
            {
                low = std::min(low, math::dot(point, toward_light));
                high = std::max(high, math::dot(point, toward_light));
            }
            const auto span = std::max(1.0e-6, high - low);
            const auto shade = [&](const Vec2& point)
            {
                return mix(recess_lit, recess, static_cast<float>(smoothstep(0.1, 0.9, (math::dot(point, toward_light) - low) / span)));
            };
            SurfaceStyle style;
            style.outline_width = static_cast<double>(stroke(settings, 1.0f));
            style.outline = rim;
            style.feather = feather;
            style.rings = 2;
            append_surface(surface_mesh(), outline, style, shade, [&](const Vec2& point, const Vec2&)
                {
                    return shade(point);
                });
        };

        // Fixed bodies: a neutral section with a lit surface line and a hatched interior.
        const auto ground = [&](const std::vector<Vec2>& outline)
        {
            const auto top = [&]
            {
                double value = -1.0e300;
                for (const auto& point : outline)
                    value = std::max(value, -point.y);
                return value;
            }();
            const auto fill_top = toward(theme.static_body_fill, 1.0f, 1.0f, 1.0f, theme.light_stage ? 0.25f : 0.035f);
            const auto fill_bottom = toward(theme.static_body_fill, 0.0f, 0.0f, 0.0f, theme.light_stage ? 0.02f : 0.18f);
            const auto shade = [&](const Vec2& point)
            {
                return mix(fill_top, fill_bottom, static_cast<float>(smoothstep(0.0, 90.0 * ds, top + point.y)));
            };
            SurfaceStyle style;
            style.outline_width = show_outlines ? static_cast<double>(stroke(settings, 1.0f)) : 0.0;
            style.outline = theme.static_body_outline;
            style.feather = feather;
            style.rings = 1;
            auto& target = surface_mesh();
            if (!append_surface(target, outline, style, shade, [&](const Vec2& point, const Vec2&)
                    {
                        return shade(point);
                    },
                    &inset))
                return;
            // Section hatching anchored to the world origin so it travels with the ground.
            const Vec2 along = math::normalized(Vec2 { 1.0, -1.0 });
            const Vec2 across = math::perpendicular(along);
            const auto spacing = 9.0 * ds;
            const auto origin = camera.world_to_screen({ 0.0, 0.0 });
            double low = 1.0e300, high = -1.0e300;
            for (const auto& point : inset)
            {
                const auto value = math::dot(point - origin, across);
                low = std::min(low, value);
                high = std::max(high, value);
            }
            // Only the visible part of a long ground is hatched.
            double view_low = 1.0e300, view_high = -1.0e300;
            const auto width = static_cast<double>(camera.viewport().width), height = static_cast<double>(camera.viewport().height);
            for (const auto& corner : { Vec2 { 0.0, 0.0 }, Vec2 { width, 0.0 }, Vec2 { width, height }, Vec2 { 0.0, height } })
            {
                view_low = std::min(view_low, math::dot(corner - origin, across));
                view_high = std::max(view_high, math::dot(corner - origin, across));
            }
            low = std::max(low, view_low);
            high = std::min(high, view_high);
            if (math::is_finite(origin) && std::isfinite(low) && std::isfinite(high) && (high - low) / spacing < 2000.0)
            {
                const auto hatch_half = static_cast<double>(stroke(settings, 1.0f)) * 0.5;
                for (auto step = std::ceil(low / spacing); step * spacing <= high; step += 1.0)
                {
                    Vec2 start, end;
                    if (!clip_line(inset, origin + across * (step * spacing), along, start, end))
                        continue;
                    const auto fade = [&](const Vec2& point)
                    {
                        auto color = theme.ground_hatch;
                        color.alpha *= static_cast<float>(1.0 - 0.6 * smoothstep(0.0, 48.0 * ds, top + point.y));
                        return color;
                    };
                    append_soft_segment(target, start, end, hatch_half, feather, fade(start), fade(end));
                }
            }
            // The lit surface line on every upward-facing edge.
            double twice_area = 0.0;
            for (std::size_t index = 0; index < outline.size(); ++index)
                twice_area += math::cross(outline[index], outline[(index + 1) % outline.size()]);
            const auto sign = twice_area < 0.0 ? 1.0 : -1.0;
            for (std::size_t index = 0; index < outline.size(); ++index)
            {
                const auto& a = outline[index];
                const auto& b = outline[(index + 1) % outline.size()];
                const auto normal = math::normalized(math::perpendicular(b - a)) * sign;
                if (normal.y < -0.35)
                {
                    const auto strength = static_cast<float>(smoothstep(0.35, 0.8, -normal.y));
                    const auto color = theme.ground_surface.with_alpha(theme.ground_surface.alpha * strength);
                    append_soft_segment(target, a, b, static_cast<double>(stroke(settings, 1.5f)) * 0.5, feather, color, color);
                }
            }
        };

        std::vector<const physics::AuthoredPartDefinition*> rendered_parts;
        for (const auto& collider : body.colliders())
        {
            if (!collider.shape)
            {
                continue;
            }
            auto look = material_appearance(collider.material);
            BodyLighting part_lighting = lighting;
            part_lighting.look = look;
            const auto outline_color = outline_for(look, theme);

            if (collider.authored_part && collider.authored_part->shape)
            {
                const auto* part = collider.authored_part.get();
                if (std::find(rendered_parts.begin(), rendered_parts.end(), part) != rendered_parts.end())
                    continue;
                rendered_parts.push_back(part);
                const auto placement = math::concatenate(body_placement, part->local_transform);
                scratch_points_.clear();
                for (const auto& point : part->shape->render_outline)
                    scratch_points_.push_back(project(math::transform_point(placement, point)));
                const auto part_look = material_appearance(part->material);
                part_lighting.look = part_look;
                if (show_bodies)
                {
                    if (shaded && !is_static)
                    {
                        // Concave authored parts keep their cached triangulation; the shading is
                        // the body-wide light gradient, which interpolates exactly per vertex.
                        list.set_layer(-7);
                        hull = scratch_points_;
                        convex_hull(hull);
                        // A hull shadow would darken the notch of a concave part, so only convex
                        // outlines cast the soft drop shadow; contact occlusion still applies.
                        const auto area = [](const std::vector<Vec2>& points)
                        {
                            double twice = 0.0;
                            for (std::size_t index = 0; index < points.size(); ++index)
                                twice += math::cross(points[index], points[(index + 1) % points.size()]);
                            return std::abs(twice) * 0.5;
                        };
                        if (settings.contact_shadows && hull.size() >= 3 && area(scratch_points_) >= area(hull) * 0.98)
                            list.add_shadow(hull, theme.shadow.with_alpha(theme.shadow.alpha * 0.45f), 9.0f * scale, Vec2 { 1.5 * ds, 3.5 * ds });
                        list.set_layer(body_layer);
                        auto authored = acquire_mesh();
                        authored->feathered = false;
                        for (const auto& triangle : part->shape->render_triangles)
                            for (const auto& point : triangle)
                            {
                                const auto projected = project(math::transform_point(placement, point));
                                authored->indices.push_back(static_cast<int>(authored->vertices.size()));
                                authored->vertices.push_back({ projected, {}, premultiplied(body_shade(part_lighting, projected)) });
                            }
                        list.add_indexed_mesh(authored);
                    }
                    else
                    {
                        list.set_layer(body_layer);
                        for (const auto& triangle : part->shape->render_triangles)
                        {
                            std::vector<Vec2> points;
                            points.reserve(triangle.size());
                            for (const auto& point : triangle)
                                points.push_back(project(math::transform_point(placement, point)));
                            list.add_polygon_fill(points, colors.fill);
                        }
                    }
                    // A feathered perimeter covers the outer edge without feathering the
                    // triangulation seams inside a concave authored shape.
                    if (!show_outlines)
                    {
                        const auto edge = shaded ? part_look.surface : colors.fill;
                        list.add_polyline(scratch_points_, edge, 1.0f * scale, true);
                    }
                }
                list.set_layer(body_layer);
                if (show_outlines)
                {
                    const auto color = shaded && !is_static ? outline_for(part_look, theme) : colors.outline;
                    if (is_kinematic)
                        list.add_dashed_polyline(scratch_points_, colors.outline, stroke(settings, 1.5f), 4.0f * scale, 3.0f * scale, true);
                    else
                        list.add_polyline(scratch_points_, color, stroke(settings, 1.5f), true);
                }
                highlight(scratch_points_, true, nullptr, 0.0);
                obstacle(scratch_points_);
                continue;
            }

            if (const auto* circle = dynamic_cast<const physics::CircleShape*>(collider.shape.get()); circle != nullptr)
            {
                const auto placement = math::concatenate(body_placement, collider.local_transform);
                const auto world_center = math::transform_point(placement, circle->local_center_m());
                const auto center = project(world_center);
                const auto radius = static_cast<float>(camera.world_to_screen_length(circle->radius_m()));
                const auto deformed = math::length_squared(deform_point(id, world_center + Vec2 { circle->radius_m(), 0.0 }, mass_center, settings.soft_deformation, settings.deformation_budget) - (world_center + Vec2 { circle->radius_m(), 0.0 })) > 1.0e-16;
                scratch_points_.clear();
                const auto segments = std::clamp(curve_segment_count(radius, 0.25f), 12, 160);
                for (int index = 0; index < segments; ++index)
                {
                    const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(segments);
                    scratch_points_.push_back(project(world_center + Vec2 { std::cos(angle), std::sin(angle) } * circle->radius_m()));
                }
                // The bore of a tube, wherever it is wide enough to see.
                cavity.clear();
                double bore = 0.0;
                if (collider.shell_thickness_m && *collider.shell_thickness_m > 0.0 && *collider.shell_thickness_m < circle->radius_m())
                {
                    const auto inner_m = circle->radius_m() - *collider.shell_thickness_m;
                    bore = camera.world_to_screen_length(inner_m);
                    if (bore > 2.0 * ds)
                        for (int index = 0; index < segments; ++index)
                        {
                            const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(segments);
                            cavity.push_back(project(world_center + Vec2 { std::cos(angle), std::sin(angle) } * inner_m));
                        }
                    else
                        bore = 0.0;
                }
                list.set_layer(body_layer);
                if (shaded)
                {
                    if (is_static)
                    {
                        ground(scratch_points_);
                        commit();
                    }
                    else
                    {
                        part_lighting.round = true;
                        part_lighting.center = center;
                        part_lighting.radius = radius;
                        if (settings.contact_shadows)
                        {
                            list.set_layer(-7);
                            list.add_circle_shadow(center, radius, theme.shadow.with_alpha(theme.shadow.alpha * 0.45f), 9.0f * scale, Vec2 { 1.5 * ds, 3.5 * ds });
                            list.set_layer(body_layer);
                        }
                        SurfaceStyle style;
                        style.outline_width = show_outlines && !is_kinematic ? static_cast<double>(stroke(settings, 1.25f)) : 0.0;
                        style.outline = outline_color;
                        style.feather = feather;
                        style.bevel = 1.5 * ds;
                        style.rings = radius > 40.0f * scale ? 4 : radius > 14.0f * scale ? 3
                                                                                          : 2;
                        append_surface(surface_mesh(), scratch_points_, style, [&](const Vec2& point)
                            {
                                return body_shade(part_lighting, point);
                            },
                            [&](const Vec2& point, const Vec2& normal)
                            {
                                return body_edge(part_lighting, point, normal);
                            },
                            &inset);
                        if (look.grain > 0.0f && radius > 6.0f * scale)
                        {
                            const auto axis = math::normalized(camera.world_to_screen_direction(math::transform_direction(placement, { 1.0, 0.0 })));
                            const auto across = math::perpendicular(axis);
                            const auto grain = look.shade.with_alpha(look.grain);
                            for (const auto offset : { -0.62, -0.31, -0.08, 0.17, 0.44, 0.71 })
                            {
                                Vec2 start, end;
                                if (clip_line(inset, center + across * (offset * radius), axis, start, end))
                                    append_soft_segment(surface_mesh(), start, end, 0.5 * ds, feather, grain, grain);
                            }
                        }
                        draw_cavity(cavity, look, outline_color);
                        commit();
                    }
                }
                else if (show_bodies)
                {
                    if (deformed)
                        list.add_polygon_fill(scratch_points_, colors.fill);
                    else
                        list.add_circle_fill(center, radius, colors.fill);
                    if (bore > 0.0)
                    {
                        list.add_circle_fill(center, static_cast<float>(bore), mix(theme.background, colors.fill, 0.35f).with_alpha(1.0f));
                        list.add_circle_outline(center, static_cast<float>(bore), colors.outline, stroke(settings, 1.0f));
                    }
                }
                if (show_outlines)
                {
                    if (!shaded)
                    {
                        if (deformed)
                            list.add_polyline(scratch_points_, colors.outline, 1.5f * scale, true);
                        else
                            list.add_circle_outline(center, radius, colors.outline, 1.5f * scale);
                        // A spoke makes the rotation of an otherwise featureless disc visible,
                        // which matters as soon as friction and spin are part of the lesson.
                        const auto spoke_end = math::transform_point(placement, circle->local_center_m() + Vec2 { circle->radius_m(), 0.0 });
                        list.add_line(center, project(spoke_end), colors.outline.with_alpha(0.6f), scale);
                    }
                    else if (!is_static)
                    {
                        if (is_kinematic)
                            list.add_dashed_polyline(scratch_points_, colors.outline, stroke(settings, 1.5f), 4.0f * scale, 3.0f * scale, true);
                        // A short index mark near the rim keeps spin visible on a shaded disc; a
                        // tube carries it on its wall.
                        const auto direction = math::normalized(project(math::transform_point(placement, circle->local_center_m() + Vec2 { circle->radius_m(), 0.0 })) - center);
                        const auto inner = bore > 0.0 ? bore + 2.0 * ds : std::max(static_cast<double>(radius) * 0.5, 7.5 * ds);
                        const auto outer = static_cast<double>(radius) * (bore > 0.0 ? 0.93 : 0.84);
                        if (outer - inner > 2.0 * ds)
                            list.add_line(center + direction * inner, center + direction * outer, outline_color.with_alpha(0.7f), stroke(settings, 1.5f));
                    }
                }
                const auto circle_center = center;
                highlight(scratch_points_, true, deformed ? nullptr : &circle_center, radius);
                const std::array<Vec2, 8> octagon { center + Vec2 { radius, 0.0 }, center + Vec2 { radius * 0.71, radius * 0.71 }, center + Vec2 { 0.0, radius }, center + Vec2 { -radius * 0.71, radius * 0.71 }, center + Vec2 { -radius, 0.0 }, center + Vec2 { -radius * 0.71, -radius * 0.71 }, center + Vec2 { 0.0, -radius }, center + Vec2 { radius * 0.71, -radius * 0.71 } };
                add_obstacle(octagon.data(), octagon.size(), key);
                continue;
            }

            scratch_points_ = collider_outline_world(collider, body_placement);
            if (scratch_points_.empty())
            {
                continue;
            }
            for (auto& point : scratch_points_)
            {
                point = project(point);
            }

            list.set_layer(body_layer);
            if (scratch_points_.size() >= 3)
            {
                if (shaded)
                {
                    if (is_static)
                    {
                        ground(scratch_points_);
                        commit();
                    }
                    else
                    {
                        if (settings.contact_shadows)
                        {
                            list.set_layer(-7);
                            list.add_shadow(scratch_points_, theme.shadow.with_alpha(theme.shadow.alpha * 0.45f), 9.0f * scale, Vec2 { 1.5 * ds, 3.5 * ds });
                            list.set_layer(body_layer);
                        }
                        math::Aabb local;
                        for (const auto& point : scratch_points_)
                            local.expand(point);
                        const auto size = std::min(local.extents().x, local.extents().y);
                        SurfaceStyle style;
                        style.outline_width = show_outlines && !is_kinematic ? static_cast<double>(stroke(settings, 1.25f)) : 0.0;
                        style.outline = outline_color;
                        style.feather = feather;
                        style.bevel = 1.5 * ds;
                        style.rings = look.specular > 0.0f ? 5 : 2;
                        style.max_segment = std::max(6.0 * ds, std::max(local.extents().x, local.extents().y) / (look.specular > 0.0f ? 9.0 : 4.0));
                        append_surface(surface_mesh(), scratch_points_, style, [&](const Vec2& point)
                            {
                                return body_shade(part_lighting, point);
                            },
                            [&](const Vec2& point, const Vec2& normal)
                            {
                                return body_edge(part_lighting, point, normal);
                            },
                            &inset);
                        if (look.grain > 0.0f && size > 8.0 * ds)
                        {
                            // Wood figure runs along the part's own axis and turns with it.
                            const auto placement = math::concatenate(body_placement, collider.local_transform);
                            const auto axis = math::normalized(camera.world_to_screen_direction(math::transform_direction(placement, { 1.0, 0.0 })));
                            const auto across = math::perpendicular(axis);
                            const auto middle = (local.minimum + local.maximum) * 0.5;
                            double low = 1.0e300, high = -1.0e300;
                            for (const auto& point : inset)
                            {
                                low = std::min(low, math::dot(point - middle, across));
                                high = std::max(high, math::dot(point - middle, across));
                            }
                            auto seed = static_cast<std::uint32_t>(id.index * 2654435761u + 977u);
                            const auto spacing = std::max(4.0 * ds, (high - low) / 7.0);
                            for (auto position = low + spacing * 0.6; position < high - spacing * 0.3; position += spacing)
                            {
                                seed = seed * 1664525u + 1013904223u;
                                const auto jitter = (static_cast<double>(seed >> 8u) / 16777216.0 - 0.5) * spacing * 0.7;
                                const auto strength = 0.55f + 0.45f * static_cast<float>((seed >> 4u) & 0xffu) / 255.0f;
                                Vec2 start, end;
                                if (clip_line(inset, middle + across * (position + jitter), axis, start, end))
                                {
                                    const auto grain = look.shade.with_alpha(look.grain * strength);
                                    append_soft_segment(surface_mesh(), start, end, 0.5 * ds, feather, grain, grain);
                                }
                            }
                        }
                        if (collider.shell_thickness_m && inset_outline(scratch_points_, camera.world_to_screen_length(*collider.shell_thickness_m), cavity))
                            draw_cavity(cavity, look, outline_color);
                        commit();
                    }
                }
                else if (show_bodies)
                {
                    list.add_polygon_fill(scratch_points_, colors.fill);
                    if (collider.shell_thickness_m && inset_outline(scratch_points_, camera.world_to_screen_length(*collider.shell_thickness_m), cavity))
                    {
                        list.add_polygon_fill(cavity, mix(theme.background, colors.fill, 0.35f).with_alpha(1.0f));
                        list.add_polygon_outline(cavity, colors.outline, stroke(settings, 1.0f));
                    }
                }
                if (show_outlines)
                {
                    if (is_kinematic)
                        list.add_dashed_polyline(scratch_points_, colors.outline, stroke(settings, 1.5f), 4.0f * scale, 3.0f * scale, true);
                    else if (!shaded)
                        list.add_polygon_outline(scratch_points_, colors.outline, 1.5f * scale);
                }
                highlight(scratch_points_, true, nullptr, 0.0);
                obstacle(scratch_points_);
            }
            else
            {
                // A segment: fixed segments are drawn as ground lines with section ticks.
                if (shaded && is_static && scratch_points_.size() == 2)
                {
                    auto& target = surface_mesh();
                    const auto a = scratch_points_[0], b = scratch_points_[1];
                    append_soft_segment(target, a, b, static_cast<double>(stroke(settings, 1.5f)) * 0.5, feather, theme.ground_surface, theme.ground_surface);
                    const auto direction = math::normalized(b - a);
                    auto below = math::perpendicular(direction);
                    if (below.y < 0.0)
                        below = -below;
                    const auto length = math::length(b - a);
                    const auto tick = (below * 0.7 - direction * 0.7) * (6.0 * ds);
                    for (double position = 4.0 * ds; position < length && position < 4096.0 * ds; position += 8.0 * ds)
                        append_soft_segment(target, a + direction * position, a + direction * position + tick, static_cast<double>(stroke(settings, 1.0f)) * 0.5, feather, theme.ground_hatch, theme.ground_hatch.with_alpha(0.0f));
                    commit();
                }
                else if (show_outlines)
                {
                    list.add_polyline(scratch_points_, colors.outline, 2.0f * scale);
                }
                highlight(scratch_points_, false, nullptr, 0.0);
                obstacle(scratch_points_);
            }
        }
        commit();
        if (part_count > 0)
        {
            // Each part's outline, set out by the selection gap, is kept only where it lies outside
            // every other part of the body: what remains traces the silhouette of the whole. The
            // other parts are set out too, an earlier one by less than the gap and a later one by
            // more, so abutting corners leave no stub at their seam and an edge two parts share
            // is drawn once, by the later part.
            const auto previous = list.layer();
            list.set_layer(5);
            const auto gap = 1.0 * ds + line_width * 0.5;
            thread_local std::vector<Vec2> expanded, run;
            thread_local std::vector<std::pair<double, double>> kept, split;
            thread_local std::vector<std::vector<Vec2>> narrow_clips, wide_clips;
            if (narrow_clips.size() < part_count)
            {
                narrow_clips.resize(part_count);
                wide_clips.resize(part_count);
            }
            for (std::size_t part = 0; part < part_count; ++part)
            {
                offset_outline(part_outlines[part], gap * 0.75, narrow_clips[part]);
                offset_outline(part_outlines[part], gap * 1.25, wide_clips[part]);
            }
            for (std::size_t part = 0; part < part_count; ++part)
            {
                offset_outline(part_outlines[part], gap, expanded);
                run.clear();
                const auto flush = [&]
                {
                    if (run.size() >= 2)
                        list.add_path(run, line_color, line_width, false, StrokeJoin::round, StrokeCap::round);
                    run.clear();
                };
                for (std::size_t index = 0; index < expanded.size(); ++index)
                {
                    const auto a = expanded[index], b = expanded[(index + 1) % expanded.size()];
                    kept.assign(1, { 0.0, 1.0 });
                    for (std::size_t other = 0; other < part_count && !kept.empty(); ++other)
                    {
                        Vec2 enter, leave;
                        if (other == part || !clip_line(other < part ? narrow_clips[other] : wide_clips[other], a, b - a, enter, leave))
                            continue;
                        const auto span = math::length_squared(b - a);
                        if (!(span > 0.0))
                            continue;
                        const auto low = math::dot(enter - a, b - a) / span, high = math::dot(leave - a, b - a) / span;
                        split.clear();
                        for (const auto& [from, to] : kept)
                        {
                            if (high <= from || low >= to)
                            {
                                split.emplace_back(from, to);
                                continue;
                            }
                            if (low > from)
                                split.emplace_back(from, low);
                            if (high < to)
                                split.emplace_back(high, to);
                        }
                        kept.swap(split);
                    }
                    for (const auto& [from, to] : kept)
                    {
                        const auto first = math::lerp(a, b, from), last = math::lerp(a, b, to);
                        const auto clipped = from > 0.0 || to < 1.0;
                        if (!(math::length(last - first) > (clipped ? 0.5 * gap : 0.0)))
                            continue;
                        if (run.empty() || math::length_squared(run.back() - first) > 1.0e-6)
                        {
                            flush();
                            run.push_back(first);
                        }
                        run.push_back(last);
                    }
                }
                flush();
            }
            list.set_layer(previous);
        }

        list.set_layer(20);
        if (settings.layers.is_enabled(VisualizationLayer::bounding_boxes))
        {
            const auto bounds = body.compute_bounds(body_placement);
            if (!bounds.is_empty())
            {
                const auto minimum = camera.world_to_screen({ bounds.minimum.x, bounds.maximum.y });
                const auto maximum = camera.world_to_screen({ bounds.maximum.x, bounds.minimum.y });
                list.add_rectangle_outline(minimum, maximum, theme.bounds, 1.0f * scale);
            }
        }

        const auto center = camera.world_to_screen(mass_center);
        if (id.index >= body_frames_.size())
            body_frames_.resize(static_cast<std::size_t>(id.index) + 1);
        auto& frame = body_frames_[id.index];
        frame.screen_box = screen_box;
        frame.movable = !is_static;
        const auto local_bounds = body.compute_bounds(math::Transform2 {});
        frame.smaller_extent_px = local_bounds.is_empty() ? 0.0f : static_cast<float>(camera.world_to_screen_length(std::min(local_bounds.extents().x, local_bounds.extents().y)));
        if (settings.layers.is_enabled(VisualizationLayer::center_of_mass) && !is_static)
        {
            bool raised = false;
            const auto drawn = center_marker_radius(body, local_bounds, camera, scale, shaded, raised);
            if (drawn > 0.0f)
            {
                frame.marker_radius_px = drawn;
                center_markers_.push_back({ center, drawn, raised });
                const std::array<Vec2, 4> footprint { center + Vec2 { -drawn, -drawn }, center + Vec2 { drawn, -drawn }, center + Vec2 { drawn, drawn }, center + Vec2 { -drawn, drawn } };
                add_obstacle(footprint.data(), footprint.size(), key);
            }
        }

        if (settings.layers.is_enabled(VisualizationLayer::labels) && !display_name.empty())
        {
            LabelRequest label;
            label.key = key;
            label.anchor = center;
            label.box = screen_box;
            label.primary = std::string(display_name);
            label.primary_color = theme.label_text;
            if (is_static)
            {
                // Ground-sized fixtures are named in the inspector; on the stage they speak only
                // when singled out. Small fixtures such as anchors keep a quiet name.
                auto focus = camera.focus_rect();
                if (focus.empty())
                    focus = { 0.0, 0.0, static_cast<double>(camera.viewport().width), static_cast<double>(camera.viewport().height) };
                const auto large = screen_box.is_empty() || screen_box.extents().x > focus.width * 0.35 || screen_box.extents().y > focus.height * 0.35;
                if (large && !selected && !hovered)
                    return;
                label.placement = large ? LabelPlacement::inside_box : LabelPlacement::around_box;
                label.marker = !large && screen_box.extents().y <= 3.0 * ds && screen_box.extents().x >= 12.0 * ds;
                label.priority = selected ? 75 : hovered ? 45
                                                         : 12;
                label.required = selected;
                label.primary_color = theme.label_muted;
            }
            else
            {
                label.placement = LabelPlacement::around_box;
                label.priority = selected ? 100 : hovered ? 50
                                                          : 40;
                label.required = selected;
                label.secondary = core::format_quantity(body.mass_properties().mass_kg, core::DisplayQuantity::mass, settings.display_units);
                label.secondary_color = theme.label_muted;
            }
            queue_label(std::move(label));
        }
    }

    void SceneRenderer::draw_center_markers(const SceneRenderSettings& settings, DrawList& list) const
    {
        // The engineering centre-of-gravity symbol: a ring with alternating filled quadrants.
        const auto& theme = settings.theme;
        for (const auto& marker : center_markers_)
        {
            const auto radius = static_cast<double>(marker.radius);
            if (marker.halo)
                list.add_circle_fill(marker.point, marker.radius + stroke(settings, 1.0f), theme.background);
            list.add_circle_fill(marker.point, marker.radius, theme.center_of_mass);
            for (const auto start : { 0.0, math::pi })
            {
                std::vector<Vec2> sector { marker.point };
                const auto segments = std::clamp(curve_segment_count(marker.radius, 0.2f, static_cast<float>(math::half_pi)), 3, 16);
                for (int index = 0; index <= segments; ++index)
                {
                    const auto angle = start + math::half_pi * static_cast<double>(index) / segments;
                    sector.push_back(marker.point + Vec2 { std::cos(angle), -std::sin(angle) } * radius);
                }
                list.add_polygon_fill(sector, theme.center_of_mass_ink);
            }
            list.add_circle_outline(marker.point, marker.radius, theme.center_of_mass_ink, stroke(settings, 1.0f));
        }
    }

    Vec2 SceneRenderer::contact_force_n(physics::BodyId id) const
    {
        return id.index < contact_forces_n_.size() ? contact_forces_n_[id.index] : Vec2 {};
    }

    void SceneRenderer::gather_contact_forces(const physics::World& world) const
    {
        std::fill(contact_forces_n_.begin(), contact_forces_n_.end(), Vec2 {});
        std::fill(contact_partners_.begin(), contact_partners_.end(), ContactPartners {});
        // A sleeping body touching a support is at rest, so its contacts carry exactly what
        // balances its other loads. Its manifolds keep the impulses of the substep it fell
        // asleep in, which a settling body need not have balanced.
        for (const auto& manifold : world.manifolds())
        {
            if (manifold.is_sensor || manifold.point_count == 0)
                continue;
            double normal_impulse = 0.0;
            Vec2 weighted_point;
            for (std::size_t index = 0; index < std::min(manifold.point_count, physics::maximum_manifold_points); ++index)
            {
                const auto& point = manifold.points[index];
                normal_impulse += point.normal_impulse_n_s;
                if (point.normal_impulse_n_s > 0.0 && math::is_finite(point.world_position_m))
                    weighted_point += point.world_position_m * point.normal_impulse_n_s;
            }
            if (!(normal_impulse > 0.0))
                continue;
            for (const auto& [id, other] : { std::pair { manifold.first, manifold.second }, std::pair { manifold.second, manifold.first } })
            {
                if (id.index >= contact_partners_.size())
                    contact_partners_.resize(static_cast<std::size_t>(id.index) + 1);
                auto& partners = contact_partners_[id.index];
                if (!partners.first.is_valid())
                    partners.first = other;
                else if (!(partners.first == other))
                    partners.several = true;
                partners.weighted_point_m += weighted_point;
                partners.weight += normal_impulse;
            }
            for (const auto id : { manifold.first, manifold.second })
                if (const auto* body = world.find_body(id); body && body->type() == BodyType::dynamic_body && !body->is_awake())
                {
                    if (id.index >= contact_forces_n_.size())
                        contact_forces_n_.resize(static_cast<std::size_t>(id.index) + 1);
                    const auto joint = id.index < joint_forces_n_.size() ? joint_forces_n_[id.index] : Vec2 {};
                    const auto balance = -(body->applied_force_n() + joint);
                    if (math::is_finite(balance))
                        contact_forces_n_[id.index] = balance;
                }
        }
        // Everything else averages its recorded substeps over at least a sixtieth of a second and
        // at least two substeps, the span over which a resting solver's alternation cancels.
        for (std::size_t index = 0; index < contact_history_.size(); ++index)
        {
            const auto& history = contact_history_[index];
            if (history.count == 0)
                continue;
            const physics::BodyId id { static_cast<std::uint32_t>(index), history.generation };
            const auto* body = world.find_body(id);
            if (!body || body->type() == BodyType::static_body || (body->type() == BodyType::dynamic_body && !body->is_awake()))
                continue;
            Vec2 impulse;
            double span = 0.0;
            for (std::size_t taken = 0; taken < history.count; ++taken)
            {
                const auto slot = (history.next + history.force_n.size() - 1 - taken) % history.force_n.size();
                impulse += history.force_n[slot] * history.step_s[slot];
                span += history.step_s[slot];
                if (taken >= 1 && span >= 1.0 / 60.0 - 1.0e-9)
                    break;
            }
            if (!(span > 0.0))
                continue;
            if (index >= contact_forces_n_.size())
                contact_forces_n_.resize(index + 1);
            contact_forces_n_[index] = impulse / span;
        }
    }

    double SceneRenderer::fitted_force_scale(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings) const
    {
        const auto requested = settings.vector_scales.force;
        if (!std::isfinite(requested) || requested <= 0.0)
            return requested;
        // Weight is the force every lesson shares and it holds still during a run, so it sets the
        // scale: one fixed factor would draw a 15 g disc's weight as nothing and a 30 kg crate's
        // across the whole stage. Larger loads and impacts are clipped and say so instead.
        double largest_applied = 0.0;
        world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.type() == BodyType::dynamic_body && (!settings.vectors_selected_only || id == settings.selection))
                {
                    heaviest_weight_n_ = std::max(heaviest_weight_n_, vector_length(weight_n(world, body)));
                    const auto applied = vector_length(body.applied_force_n());
                    if (std::isfinite(applied))
                        largest_applied = std::max(largest_applied, applied);
                }
            });
        peak_applied_n_ = std::max(peak_applied_n_, largest_applied);
        const auto reference = heaviest_weight_n_ > math::geometric_epsilon ? heaviest_weight_n_ : peak_applied_n_;
        if (!(reference > math::geometric_epsilon))
            return requested;
        const auto ds = static_cast<double>(pixel_scale(settings));
        const auto stage = stage_rect(camera);
        const auto longest = std::clamp(0.16 * std::min(stage.maximum.x - stage.minimum.x, stage.maximum.y - stage.minimum.y), 48.0 * ds, 140.0 * ds);
        auto factor = requested;
        if (reference * factor > longest || reference * factor < longest * 0.25)
            factor = longest * 0.8 / reference;
        // Bodies resting on one another share a line of action. Their weight and support arrows
        // stay within half the gap between centres, so each body's pair reads on its own.
        const auto shaded = settings.material_shading && settings.layers.is_enabled(VisualizationLayer::bodies);
        const auto show_centers = settings.layers.is_enabled(VisualizationLayer::center_of_mass);
        for (const auto& manifold : world.manifolds())
        {
            if (manifold.is_sensor || manifold.is_speculative || manifold.point_count == 0)
                continue;
            const auto* first = world.find_body(manifold.first);
            const auto* second = world.find_body(manifold.second);
            if (!first || !second || first->type() != BodyType::dynamic_body || second->type() != BodyType::dynamic_body)
                continue;
            if (settings.vectors_selected_only && !(manifold.first == settings.selection) && !(manifold.second == settings.selection))
                continue;
            if (math::length_squared(first->linear_velocity_m_s()) > 0.0625 || math::length_squared(second->linear_velocity_m_s()) > 0.0625)
                continue;
            const auto weight = weight_n(world, *first);
            const auto weight_length = vector_length(weight);
            if (!(weight_length > math::geometric_epsilon))
                continue;
            const auto down = math::normalized(camera.world_to_screen_direction(weight / weight_length));
            const auto gap = camera.world_to_screen(second->world_center_of_mass_m()) - camera.world_to_screen(first->world_center_of_mass_m());
            const auto along = std::abs(math::dot(gap, down));
            if (!(along > 0.0) || std::abs(math::cross(gap, down)) > 0.35 * along)
                continue;
            for (const auto* body : { first, second })
            {
                bool raised = false;
                const auto marker = show_centers ? static_cast<double>(center_marker_radius(*body, body->compute_bounds(math::Transform2 {}), camera, pixel_scale(settings), shaded, raised)) : 0.0;
                const auto room = std::max(0.36 * along - marker - ds, stub_length_dp * ds);
                const auto magnitude = vector_length(weight_n(world, *body));
                if (magnitude > math::geometric_epsilon)
                    factor = std::min(factor, room / magnitude);
            }
        }
        // A body on one of several fixed shelves, as in lanes stacked one above another, keeps
        // its weight and its support inside its own lane: neither reaches more than part of the
        // way to the next shelf beyond the one it rests on, or to the shelf above it.
        thread_local std::vector<math::Aabb> shelves;
        shelves.clear();
        world.for_each_body([&](physics::BodyId, const physics::RigidBody& body)
            {
                if (body.type() == BodyType::static_body && shelves.size() <= 32)
                    shelves.push_back(body.compute_bounds());
            });
        if (shelves.size() < 2 || shelves.size() > 32)
            return factor;
        world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (body.type() != BodyType::dynamic_body || (settings.vectors_selected_only && !(id == settings.selection)))
                    return;
                const auto weight = weight_n(world, body);
                const auto magnitude = vector_length(weight);
                if (!(magnitude > math::geometric_epsilon))
                    return;
                const auto down = weight / magnitude;
                const auto across = math::perpendicular(down);
                const auto centre = body.world_center_of_mass_m();
                const auto depth = math::dot(centre, down), side = math::dot(centre, across);
                double first_below = 1.0e300, second_below = 1.0e300, above = 1.0e300;
                for (const auto& shelf : shelves)
                {
                    if (shelf.is_empty())
                        continue;
                    double low_depth = 1.0e300, high_depth = -1.0e300, low_side = 1.0e300, high_side = -1.0e300;
                    for (const auto& corner : { shelf.minimum, shelf.maximum, Vec2 { shelf.minimum.x, shelf.maximum.y }, Vec2 { shelf.maximum.x, shelf.minimum.y } })
                    {
                        low_depth = std::min(low_depth, math::dot(corner, down));
                        high_depth = std::max(high_depth, math::dot(corner, down));
                        low_side = std::min(low_side, math::dot(corner, across));
                        high_side = std::max(high_side, math::dot(corner, across));
                    }
                    if (side < low_side || side > high_side)
                        continue;
                    if (low_depth > depth)
                    {
                        const auto gap = low_depth - depth;
                        if (gap < first_below)
                        {
                            second_below = first_below;
                            first_below = gap;
                        }
                        else
                            second_below = std::min(second_below, gap);
                    }
                    else if (high_depth < depth)
                        above = std::min(above, depth - high_depth);
                }
                const auto supported = vector_length(contact_force_n(id)) > math::geometric_epsilon;
                for (const auto gap : { second_below, supported ? above : 1.0e300 })
                    if (gap < 1.0e299)
                        factor = std::min(factor, std::max(0.42 * camera.world_to_screen_length(gap), stub_length_dp * ds) / magnitude);
            });
        return factor;
    }

    double SceneRenderer::clear_length(const Vec2& from, const Vec2& direction, double length, std::uint64_t owner, std::uint64_t allowed) const
    {
        auto result = length;
        if (!(length > 0.0) || !math::is_finite(from) || !math::is_finite(direction))
            return result;
        const auto to = from + direction * length;
        const auto low = math::min_components(from, to), high = math::max_components(from, to);
        for (const auto& obstacle : label_obstacles_)
        {
            if ((obstacle.owner >> 56u) != 1u || obstacle.owner == owner || (allowed != 0 && obstacle.owner == allowed) || obstacle.count < 2)
                continue;
            if (obstacle.maximum.x < low.x || obstacle.minimum.x > high.x || obstacle.maximum.y < low.y || obstacle.minimum.y > high.y)
                continue;
            double entry = 0.0;
            if (ray_entry(obstacle_points_.data() + obstacle.offset, obstacle.count, from, direction, result, entry))
                result = std::min(result, entry);
        }
        return result;
    }

    bool SceneRenderer::draw_scaled_vector(const Vec2& world_anchor_m, const Vec2& quantity, double pixels_per_unit, const Color& requested_color, const Camera2D& camera,
        const SceneRenderSettings& settings, DrawList& list, std::string_view label, core::DisplayQuantity unit, double signed_value, ArrowHead head, float weight, std::uint64_t key,
        const VectorOptions& options) const
    {
        const auto magnitude = vector_length(quantity);
        if (!math::is_finite(world_anchor_m) || !math::is_finite(quantity) || !std::isfinite(magnitude) || magnitude <= math::geometric_epsilon || !std::isfinite(pixels_per_unit) || pixels_per_unit <= 0.0)
        {
            return false;
        }

        const auto scale = pixel_scale(settings);
        const auto minimum = std::isfinite(settings.vector_scales.minimum_drawn_length_px)
            ? std::max(0.0, static_cast<double>(settings.vector_scales.minimum_drawn_length_px))
            : 6.0;
        const auto maximum = std::isfinite(settings.vector_scales.maximum_drawn_length_px) && settings.vector_scales.maximum_drawn_length_px > 0.0f
            ? static_cast<double>(settings.vector_scales.maximum_drawn_length_px)
            : 400.0;
        // A value too short to show its head is drawn as a stub of fixed length with a dotted
        // shaft, so a drawing longer than its value never passes for one drawn to scale. A
        // significant value, such as a weight, keeps its stub however small it is. One a few
        // pixels short of the stub still shows its head and is drawn true.
        const auto stub = std::max(minimum, stub_length_dp * scale) + (std::isfinite(options.stub_extra_px) ? std::max(0.0, options.stub_extra_px) : 0.0);
        auto screen_length = magnitude * pixels_per_unit;
        if (screen_length < minimum && !options.significant)
            return false;
        const auto lengthened = screen_length < stub - 4.0 * scale && !options.true_length;
        if (lengthened)
            screen_length = stub;
        auto shortened = screen_length > maximum;
        screen_length = std::min(screen_length, maximum);
        const auto cut = [&](double limit)
        {
            if (limit >= 0.0 && screen_length > limit && screen_length > stub)
            {
                screen_length = std::max(limit, stub);
                shortened = true;
            }
        };
        if (std::isfinite(options.maximum_length_px) && options.maximum_length_px > 0.0)
            cut(options.maximum_length_px);

        const auto direction = quantity / magnitude;
        // The arrow is built in screen space so that its drawn length is exactly the scaled value,
        // independent of how far the camera is zoomed.
        const auto screen_direction = math::normalized(camera.world_to_screen_direction(direction));
        const auto anchor = camera.world_to_screen(world_anchor_m);
        if (!math::is_finite(anchor) || !math::is_finite(screen_direction))
            return false;
        const auto stage = visible_rect(camera, visible_stage_, 12.0 * scale);
        auto offset = std::isfinite(options.start_offset_px) ? std::max(0.0, options.start_offset_px) : 0.0;
        const auto exit = std::isfinite(options.exit_offset_px) ? options.exit_offset_px : 0.0;
        // An arrow that would end inside its own body is drawn just beyond the outline at its full
        // length, joined to its point of action by a thin lead, when nothing lies out there. Into
        // a neighbour or off the stage it stays inside its own body instead.
        double lead_offset = -1.0;
        if (!options.ends_at_anchor && exit > offset && offset + screen_length < exit + 12.0 * scale)
        {
            const auto outside = anchor + screen_direction * (exit + 1.0 * scale);
            const auto clearance = screen_length + 3.0 * scale;
            if (room_along(outside, screen_direction, stage) - 6.0 * scale >= screen_length && clear_length(outside, screen_direction, clearance, options.owner, 0) >= clearance)
            {
                lead_offset = offset;
                offset = exit + 1.0 * scale;
            }
        }
        auto start = anchor + screen_direction * offset;
        if (options.ends_at_anchor)
        {
            // A push lies in the body exerting it, in view, and stops short of any body beyond.
            cut(room_along(anchor, -screen_direction, stage) - 6.0 * scale);
            if (const auto clear = clear_length(anchor - screen_direction * (1.0 * scale), -screen_direction, screen_length, options.owner, options.pusher); clear < screen_length)
                cut(clear - 3.0 * scale);
            start = anchor - screen_direction * screen_length;
        }
        else if (options.stops_at_bodies)
        {
            if (const auto clear = clear_length(start, screen_direction, screen_length, options.owner, 0); clear < screen_length)
                cut(clear - 3.0 * scale);
        }
        Vec2 shift;
        if (options.may_shift && !options.dimmed && drawn_arrows_.size() <= 256)
        {
            // An arrow lying along one already drawn would merge into it; step aside instead.
            const auto side = math::perpendicular(screen_direction);
            const auto step_px = std::isfinite(options.shift_step_px) && options.shift_step_px > 0.0 ? options.shift_step_px : 6.0 * scale;
            const auto overlaps = [&](const Vec2& from, const Vec2& to)
            {
                for (const auto& other : drawn_arrows_)
                {
                    const auto span = other.end - other.start;
                    const auto span_length = math::length(span);
                    if (!(span_length > 1.0))
                        continue;
                    const auto along = span / span_length;
                    if (std::abs(math::cross(along, screen_direction)) > 0.15 || std::abs(math::cross(from - other.start, along)) > std::max(4.0 * scale, step_px * 0.7))
                        continue;
                    // Two heads meeting tip to tip on one line merge as surely as overlapping shafts.
                    const auto a = math::dot(from - other.start, along), b = math::dot(to - other.start, along);
                    if (std::min(std::max(a, b), span_length) - std::max(std::min(a, b), 0.0) > -4.0 * scale)
                        return true;
                }
                return false;
            };
            for (const auto step : { 0.0, 1.0, -1.0, 2.0, -2.0 })
            {
                const auto shifted = start + side * (step * step_px);
                if (!overlaps(shifted, shifted + screen_direction * screen_length))
                {
                    shift = side * (step * step_px);
                    start = shifted;
                    break;
                }
            }
        }
        // A vector never runs off the visible stage: it stops short, with its head and plate in
        // view, and is marked as shortened.
        if (!options.ends_at_anchor)
            cut(room_along(start, screen_direction, stage) - 6.0 * scale);
        const auto end = start + screen_direction * screen_length;
        if (!math::is_finite(end))
            return false;
        auto color = requested_color;
        if (options.dimmed)
            color.alpha *= 0.4f;
        const auto width = stroke(settings, 1.5f) * weight;
        const auto head_length = 9.0f * scale * std::max(1.0f, weight * 0.9f);
        // A casing in the plate tone keeps an arrow in focus legible over a body as light or as
        // dark as its own hue; a receding arrow is left without. The casings share one mesh,
        // listed once it has content, before the first arrow in focus, so they lie beneath every
        // such arrow; the list shares the mesh, so later casings still reach it.
        const auto casing_width = 1.25 * scale;
        const auto casing = settings.theme.label_plate.with_alpha(0.85f);
        if (!casing_mesh_ && !options.dimmed)
            casing_mesh_ = acquire_mesh();
        const auto lead_start = anchor + screen_direction * lead_offset + shift;
        const auto lead_end = start - screen_direction * (1.0 * scale);
        const auto lead = lead_offset >= 0.0 && math::dot(lead_end - lead_start, screen_direction) > 2.0 * scale;
        if (!options.dimmed)
        {
            const auto listed = !casing_mesh_->indices.empty();
            if (lead)
                append_soft_segment(*casing_mesh_, lead_start, lead_end, stroke(settings, 1.0f) * 0.5 + casing_width, scale, casing.with_alpha(casing.alpha * 0.6f), casing.with_alpha(casing.alpha * 0.6f));
            append_arrow_casing(*casing_mesh_, start, end, width, head_length, head, casing_width, scale, casing);
            if (!listed)
                list.add_indexed_mesh(casing_mesh_);
        }
        // The lead is a thin solid line, never dotted, so it cannot be read as part of a stub's
        // dotted shaft.
        if (lead)
            list.add_line(lead_start, lead_end, color.with_alpha(color.alpha * 0.7f), stroke(settings, 1.0f), StrokeCap::butt);
        // A dotted shaft marks a drawing longer than its value.
        list.add_arrow(start, end, color, width, head_length, head, 0.0f, lengthened ? std::max(1.5f * scale, width) : 0.0f);
        drawn_arrows_.push_back({ start, end, options.owner });
        // A slanted break cut through the shaft marks a drawing shorter than its value.
        if (shortened)
        {
            const auto normal = math::perpendicular(screen_direction) * (5.0 * scale);
            const auto at = screen_length > 32.0 * scale ? end - screen_direction * (20.0 * scale) : start + screen_direction * (screen_length * 0.4);
            list.add_line(at - normal + screen_direction * (2.5 * scale), at + normal - screen_direction * (2.5 * scale), settings.theme.background, stroke(settings, 2.5f), StrokeCap::butt);
        }
        const auto plate_key = key ^ (std::hash<std::string_view> {}(label) << 8u);
        // Plates keep off arrows. A receding arrow is kept clear only by the values of receding
        // bodies, which laid on it would read as its body's; a name, or a reading of the
        // selection, may still pass over it. The obstacle widens toward the head, since a plate
        // over the head hides where it points.
        {
            const auto tail = start + screen_direction * std::min(8.0 * scale, screen_length * 0.5);
            const auto thin = math::perpendicular(screen_direction) * (0.5 * scale), wide = math::perpendicular(screen_direction) * (5.0 * scale);
            const auto tip = end + screen_direction * (2.0 * scale);
            const std::array<Vec2, 4> outline { tail - thin, tip - wide, tip + wide, tail + thin };
            add_obstacle(outline.data(), outline.size(), key, plate_key, options.dimmed);
        }
        const auto essential = (key >> 56u) != 2u || options.focus;
        if (sparse_vector_labels_ && !essential)
            return false;
        const auto signed_prefix = label.find('.') != std::string_view::npos && signed_value > 0.0 ? "+" : "";
        LabelRequest request;
        request.key = plate_key;
        request.required = !options.optional_label;
        request.essential = essential && !options.optional_label;
        // The selection's readouts claim their space before the rest of a crowded scene.
        request.priority = (label.find('.') != std::string_view::npos ? 84 : 90) + (options.focus ? 8 : 0);
        request.placement = LabelPlacement::along_vector;
        // A plate is read at the head: for a push, on the face it presses, never at the tail in
        // the body exerting it.
        request.anchor = end;
        request.origin = start;
        request.direction = screen_direction;
        request.owner = options.owner;
        request.box = options.owner_box;
        const auto value = unit == core::DisplayQuantity::force ? force_text(signed_value, settings) : core::format_quantity(signed_value, unit, settings.display_units);
        request.primary = core::substitute("{} {}{}", label, signed_prefix, value);
        // Receding text keeps its hue but leans toward the muted ink, which stays legible on the
        // plate in every theme, where fading it would not.
        request.primary_color = options.dimmed ? mix(requested_color, settings.theme.label_muted, 0.55f).with_alpha(1.0f) : requested_color;
        // Every drawing off its scale says so on its plate, in words, or where the words find no
        // room with the arrow's own mark in miniature.
        if (shortened || lengthened)
        {
            request.secondary = "not to scale";
            request.secondary_color = settings.theme.label_muted;
            request.scale_note = lengthened ? ScaleNote::longer : ScaleNote::shorter;
        }
        queue_label(std::move(request));
        return true;
    }

    void SceneRenderer::draw_body_vectors(const physics::World& world, physics::BodyId id, const physics::RigidBody& body, const Vec2& center, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        if (body.type() == BodyType::static_body)
        {
            return;
        }

        const auto& scales = settings.vector_scales;
        const auto scale = pixel_scale(settings);
        const auto key = body_key(id, 2);
        auto components = settings.vector_components != VectorComponents::none;
        Vec2 first_axis { 1.0, 0.0 };
        std::string_view first_name = "x", second_name = "y";
        if (settings.vector_components == VectorComponents::custom_axes)
        {
            const auto angle = std::isfinite(settings.component_angle_rad) ? settings.component_angle_rad : 0.0;
            first_axis = { std::cos(angle), std::sin(angle) };
            first_name = "u";
            second_name = "v";
        }
        else if (settings.vector_components == VectorComponents::contact_axes)
        {
            first_name = "n";
            second_name = "t";
            components = false;
            // Use the first actual solid contact in stable manifold order. The normal points
            // toward this body, so n is the supporting direction and t is its CCW perpendicular.
            for (const auto& manifold : world.manifolds())
                if (!manifold.is_empty() && !manifold.is_sensor && !manifold.is_speculative && (manifold.first == id || manifold.second == id))
                {
                    const auto magnitude = vector_length(manifold.normal);
                    if (std::isfinite(magnitude) && magnitude > math::geometric_epsilon)
                    {
                        first_axis = manifold.normal / magnitude * (manifold.first == id ? -1.0 : 1.0);
                        components = true;
                        break;
                    }
                }
            if (!components && id == settings.selection)
            {
                LabelRequest request;
                request.key = key ^ 0x77;
                request.priority = 60;
                request.required = true;
                request.placement = LabelPlacement::around_point;
                request.anchor = camera.world_to_screen(center);
                request.primary = "Contact axes: no solid contact";
                request.primary_color = settings.theme.label_muted;
                queue_label(std::move(request));
            }
        }
        const auto second_axis = math::perpendicular(first_axis);
        // Velocity stays anchored on the centre of mass, where the selection's velocity knob
        // expects its tip, and a stub of it reaches past the centre symbol; loads start clear of
        // the centre symbol, or of a body too small to carry one, and step aside from the
        // velocity arrow when they share its line.
        VectorOptions anchored;
        anchored.owner = body_key(id, 1);
        anchored.focus = is_focus(id, settings);
        anchored.dimmed = settings.selection.is_valid() && !is_selected(id, settings) && world.find_body(settings.selection) != nullptr;
        if (id.index < body_frames_.size())
            anchored.owner_box = body_frames_[id.index].screen_box;
        auto load = anchored;
        load.may_shift = true;
        anchored.keeps_tail = true;
        if (id.index < body_frames_.size())
        {
            const auto& frame = body_frames_[id.index];
            if (frame.marker_radius_px > 0.0f)
            {
                load.start_offset_px = static_cast<double>(frame.marker_radius_px) + 1.0 * scale;
                anchored.stub_extra_px = load.start_offset_px;
            }
            else if (frame.smaller_extent_px > 0.0f && frame.smaller_extent_px < 24.0f * scale)
                load.start_offset_px = static_cast<double>(frame.smaller_extent_px) * 0.5 + 1.0 * scale;
        }
        // Distance in pixels from the centre of mass to the outline along a world direction, for
        // arrows short enough to end inside the body. Zero where the centre lies outside it.
        const auto placement = body.interpolated_transform(settings.interpolation_alpha);
        const auto reach_px = anchored.owner_box.is_empty() ? 0.0 : math::length(anchored.owner_box.extents());
        const auto* disc = body.colliders().size() == 1 && !body.colliders().front().authored_part ? dynamic_cast<const physics::CircleShape*>(body.colliders().front().shape.get()) : nullptr;
        const auto exit_px = [&](const Vec2& world_direction, double drawn_px)
        {
            if (!(reach_px > 0.0) || drawn_px > reach_px)
                return 0.0;
            // A lone disc about its own centre needs no search.
            if (disc && math::length_squared(math::transform_point(body.colliders().front().local_transform, disc->local_center_m()) - body.mass_properties().center_of_mass_m) < 1.0e-12)
                return camera.world_to_screen_length(disc->radius_m());
            if (!body.contains_world_point(center, placement))
                return 0.0;
            const auto reach_m = reach_px / std::max(camera.pixels_per_metre(), 1.0e-9);
            double inside = 0.0, outside = reach_m;
            if (body.contains_world_point(center + world_direction * outside, placement))
                return 0.0;
            for (int iteration = 0; iteration < 14; ++iteration)
            {
                const auto middle = (inside + outside) * 0.5;
                (body.contains_world_point(center + world_direction * middle, placement) ? inside : outside) = middle;
            }
            return camera.world_to_screen_length(outside);
        };
        const auto stub_px = std::max(stub_length_dp * scale, std::isfinite(scales.minimum_drawn_length_px) ? static_cast<double>(scales.minimum_drawn_length_px) : 6.0);
        const auto quantity = [&](const Vec2& value, double factor, const Color& color, std::string_view label, core::DisplayQuantity unit, ArrowHead head, float weight, const VectorOptions& requested)
        {
            auto options = requested;
            if (const auto magnitude = vector_length(value); !options.keeps_tail && std::isfinite(magnitude) && magnitude > math::geometric_epsilon && std::isfinite(factor))
                options.exit_offset_px = exit_px(value / magnitude, std::max(magnitude * factor, stub_px) + options.start_offset_px);
            const auto stated = draw_scaled_vector(center, value, factor, color, camera, settings, list, label, unit, vector_length(value), head, weight, key, options);
            if (!components || !math::is_finite(value) || vector_length(value) <= math::geometric_epsilon)
                return stated;
            auto component_options = options;
            component_options.significant = false;
            component_options.true_length = true;
            component_options.exit_offset_px = 0.0;
            const auto draw_component = [&](const Vec2& axis, std::string_view axis_name)
            {
                const auto projection = math::dot(value, axis);
                const auto component = axis * projection;
                const auto component_label = core::substitute("{}.{}", label, axis_name);
                const auto component_color = color.with_alpha(0.65f);
                draw_scaled_vector(center, component, factor, component_color, camera, settings, list, component_label, unit, projection, ArrowHead::filled, 0.7f, key, component_options);
                // A zero component has no direction and therefore no arrow, but remains readable.
                if (std::abs(projection) <= math::geometric_epsilon)
                {
                    LabelRequest request;
                    request.key = key ^ std::hash<std::string> {}(component_label);
                    request.priority = 84;
                    request.required = true;
                    request.placement = LabelPlacement::around_point;
                    request.anchor = camera.world_to_screen(center);
                    request.primary = core::substitute("{} {}", component_label, core::format_quantity(0.0, unit, settings.display_units));
                    request.primary_color = component_color;
                    queue_label(std::move(request));
                }
            };
            draw_component(first_axis, first_name);
            draw_component(second_axis, second_name);
            return stated;
        };

        if (settings.layers.is_enabled(VisualizationLayer::velocity_vectors))
        {
            quantity(body.linear_velocity_m_s(), scales.velocity, settings.theme.velocity, "v", core::DisplayQuantity::velocity, ArrowHead::filled, 1.0f, anchored);
        }

        if (settings.layers.is_enabled(VisualizationLayer::momentum_vectors))
        {
            auto options = anchored;
            options.may_shift = true;
            quantity(body.linear_momentum_kg_m_s(), scales.momentum, settings.theme.momentum, "p", core::DisplayQuantity::momentum, ArrowHead::double_filled, 1.0f, options);
        }

        if (settings.layers.is_enabled(VisualizationLayer::force_vectors))
        {
            // Each source has its own arrow and name, so a load being taught is never hidden in a
            // sum with gravity. Bodies outside the selection of a crowded scene fold their other
            // loads into one resultant. Contact and joint impulses are reported apart from these.
            BodyLoads loads;
            collect_loads(id, body, loads);
            keep_shown_loads(loads, compact_loads_ && !anchored.focus);
            double largest = 0.0;
            for (std::size_t index = 0; index < loads.count; ++index)
                largest = std::max(largest, vector_length(loads.items[index].force_n));
            for (std::size_t index = 0; index < loads.count; ++index)
            {
                const auto& item = loads.items[index];
                auto options = load;
                options.significant = item.name == weight_name || vector_length(item.force_n) >= 0.05 * largest;
                if (quantity(item.force_n, scales.force, settings.theme.force, item.name, core::DisplayQuantity::force, item.head, 1.34f, options) && item.name == "Spring")
                {
                    if (id.index >= spring_stated_.size())
                        spring_stated_.resize(static_cast<std::size_t>(id.index) + 1);
                    spring_stated_[id.index] = 1;
                }
            }
            const auto contact = contact_force_n(id);
            const auto contact_magnitude = vector_length(contact);
            auto options = load;
            options.significant = contact_magnitude >= 0.25 * largest || contact_magnitude >= 0.02 * heaviest_weight_n_;
            const auto saved = components;
            components = false;
            // A body pressed from several sides shows, while singled out, what each neighbour
            // exerts; otherwise their resultant.
            const auto* partners = id.index < contact_partners_.size() ? &contact_partners_[id.index] : nullptr;
            const auto several = partners && partners->several;
            if (several && anchored.focus && contact_magnitude > math::geometric_epsilon)
                draw_body_contacts(world, id, camera, settings, list, options);
            else if (contact_magnitude > math::geometric_epsilon)
            {
                // A support pushes on the face it touches, so its arrow comes from outside and
                // rests its head on that face, where it reads as a push and not as a pull from
                // the far side. That holds for a fixed support, or for any one neighbour while
                // the body is singled out; it gives way to the centre-of-mass arrow where the push
                // would lie along this body's own arrows, as along a weight long enough to leave
                // the body through the same face.
                const auto* pusher = several || !partners || !(partners->weight > 0.0) ? nullptr : world.find_body(partners->first);
                bool pushed = false;
                if (pusher && (pusher->type() != BodyType::dynamic_body || anchored.focus) && std::isfinite(scales.force))
                {
                    const auto face_m = math::transform_point(placement, math::inverse_transform_point(body.transform(), partners->weighted_point_m / partners->weight));
                    const auto face = camera.world_to_screen(face_m);
                    const auto direction = math::normalized(camera.world_to_screen_direction(contact / contact_magnitude));
                    const auto tail = face - direction * std::max(contact_magnitude * scales.force, stub_px);
                    bool crowded = !math::is_finite(face) || !math::is_finite(tail);
                    // This body's own arrows are the latest drawn.
                    for (auto arrow_it = drawn_arrows_.rbegin(); arrow_it != drawn_arrows_.rend() && arrow_it->owner == anchored.owner && !crowded; ++arrow_it)
                    {
                        const auto& arrow = *arrow_it;
                        int near = 0;
                        for (int sample = 0; sample <= 4; ++sample)
                            near += segment_point_distance(arrow.start, arrow.end, math::lerp(tail, face, sample / 4.0)) < 6.0 * scale ? 1 : 0;
                        crowded = near >= 2;
                    }
                    if (!crowded)
                    {
                        auto push = options;
                        push.start_offset_px = 0.0;
                        push.ends_at_anchor = true;
                        push.pusher = body_key(partners->first, 1);
                        // A push lying along the pusher's own arrows steps clear of them and of its
                        // centre-of-mass symbol.
                        const auto marker = partners->first.index < body_frames_.size() ? static_cast<double>(body_frames_[partners->first.index].marker_radius_px) : 0.0;
                        push.shift_step_px = std::max(6.0 * scale, marker + 5.0 * scale);
                        push.maximum_length_px = push_reach(*pusher, camera, settings.interpolation_alpha, face, direction);
                        draw_scaled_vector(face_m, contact, scales.force, settings.theme.contact, camera, settings, list, "Contact", core::DisplayQuantity::force, contact_magnitude, ArrowHead::open, 1.2f, key, push);
                        pushed = true;
                    }
                }
                // Otherwise the support is drawn from the centre of mass, inside the body when it is
                // short: it acts on a face, so it never moves out to the far side of the body.
                if (!pushed)
                {
                    auto centred = options;
                    centred.keeps_tail = true;
                    quantity(contact, scales.force, settings.theme.contact, several ? "Net contact" : "Contact", core::DisplayQuantity::force, ArrowHead::open, 1.2f, centred);
                }
            }
            components = saved;
        }

        if (settings.layers.is_enabled(VisualizationLayer::acceleration_vectors) && body.inverse_mass() > 0.0)
        {
            quantity(body.applied_force_n() * body.inverse_mass(), scales.acceleration, settings.theme.acceleration, "a", core::DisplayQuantity::acceleration, ArrowHead::open, 1.0f, load);
        }

        if (settings.layers.is_enabled(VisualizationLayer::angular_velocity) && std::abs(body.angular_velocity_rad_s()) > 1.0e-4)
        {
            // Spin has no direction in the plane, so it is drawn as an arc whose extent grows with
            // the rate and whose sweep follows the sense of rotation. A slow spin keeps an arc long
            // enough to carry its head, and its plate says that arc is not to scale.
            const auto screen_center = camera.world_to_screen(center);
            const auto radius = 18.0 * scale;
            const auto scaled_sweep = math::clamp(body.angular_velocity_rad_s() * 0.4, -math::pi * 1.5, math::pi * 1.5);
            const auto minimum_sweep = 16.0 * scale / radius;
            const auto opened = std::abs(scaled_sweep) < minimum_sweep;
            const auto sweep = opened ? std::copysign(minimum_sweep, scaled_sweep) : scaled_sweep;
            const int segments = 32;

            scratch_points_.clear();
            scratch_points_.reserve(segments + 1);
            for (int index = 0; index <= segments; ++index)
            {
                const auto fraction = static_cast<double>(index) / static_cast<double>(segments);
                const auto angle = sweep * fraction;
                // Screen y grows downward, so the sign of the vertical term is flipped to keep a
                // positive angular velocity reading as counter-clockwise on screen.
                scratch_points_.push_back({ screen_center.x + radius * std::cos(angle), screen_center.y - radius * std::sin(angle) });
            }
            auto color = settings.theme.momentum.with_alpha(0.85f);
            if (anchored.dimmed)
                color.alpha *= 0.4f;
            list.add_polyline(scratch_points_, color, stroke(settings, 1.5f));
            const auto tangent = math::normalized(scratch_points_.back() - scratch_points_[scratch_points_.size() - 2]);
            if (std::abs(sweep) * radius > 12.0 * scale)
                list.add_arrow(scratch_points_.back() - tangent * (6.0 * scale), scratch_points_.back() + tangent * (1.5 * scale), color, stroke(settings, 1.5f), 7.0f * scale);
            // The rate is read on a plate at the arc's head, like a vector's value.
            if (settings.layers.is_enabled(VisualizationLayer::labels) && (!sparse_vector_labels_ || anchored.focus) && math::is_finite(tangent))
            {
                LabelRequest request;
                request.key = key ^ 0x3c3c3cu;
                request.required = true;
                request.essential = anchored.focus;
                request.priority = 82 + (anchored.focus ? 8 : 0);
                request.placement = LabelPlacement::along_vector;
                request.anchor = scratch_points_.back();
                request.origin = scratch_points_.front();
                request.direction = math::normalized(scratch_points_.back() - screen_center);
                request.owner = anchored.owner;
                request.box = anchored.owner_box;
                request.primary = core::substitute("\xCF\x89 {}", core::format_quantity(body.angular_velocity_rad_s(), core::DisplayQuantity::angular_velocity, settings.display_units));
                request.primary_color = anchored.dimmed ? mix(settings.theme.momentum, settings.theme.label_muted, 0.55f).with_alpha(1.0f) : settings.theme.momentum;
                if (opened)
                {
                    request.secondary = "not to scale";
                    request.secondary_color = settings.theme.label_muted;
                }
                queue_label(std::move(request));
            }
        }
    }

    void SceneRenderer::draw_body_contacts(const physics::World& world, physics::BodyId id, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list, const VectorOptions& options) const
    {
        // One arrow per neighbour, at the face it presses on, named for that neighbour. Their
        // sum is the averaged contact resultant; the substep's own split is kept, and whatever
        // it lacks from that average is shared evenly, so a sleeping pile still balances.
        struct Partner
        {
            physics::BodyId other;
            Vec2 force_n, point_m;
            std::size_t points {};
        };
        std::array<Partner, 6> partners {};
        std::size_t count = 0;
        Vec2 raw_total;
        const auto step = contact_step_s_ > 0.0 && std::isfinite(contact_step_s_) ? contact_step_s_ : 1.0 / 120.0;
        for (const auto& manifold : world.manifolds())
        {
            if (manifold.is_sensor || manifold.point_count == 0 || !(manifold.first == id || manifold.second == id))
                continue;
            const auto length = vector_length(manifold.normal);
            if (!std::isfinite(length) || length <= math::geometric_epsilon)
                continue;
            const auto normal = manifold.normal / length;
            double normal_impulse = 0.0, tangent_impulse = 0.0;
            Vec2 point;
            const auto points = std::min(manifold.point_count, physics::maximum_manifold_points);
            for (std::size_t index = 0; index < points; ++index)
            {
                normal_impulse += manifold.points[index].normal_impulse_n_s;
                tangent_impulse += manifold.points[index].tangent_impulse_n_s;
                point += manifold.points[index].world_position_m;
            }
            if (!(normal_impulse > 0.0))
                continue;
            auto force = (normal * normal_impulse + math::perpendicular(normal) * tangent_impulse) / step;
            if (manifold.first == id)
                force = -force;
            const auto other = manifold.first == id ? manifold.second : manifold.first;
            Partner* partner = nullptr;
            for (std::size_t index = 0; index < count && !partner; ++index)
                if (partners[index].other == other)
                    partner = &partners[index];
            if (!partner)
            {
                if (count == partners.size())
                    continue;
                partner = &partners[count++];
                *partner = Partner {};
                partner->other = other;
            }
            partner->force_n += force;
            partner->point_m += point;
            partner->points += points;
            raw_total += force;
        }
        const auto* body = world.find_body(id);
        if (count == 0 || !body)
            return;
        const auto solved = body->transform();
        const auto drawn = body->interpolated_transform(settings.interpolation_alpha);
        const auto correction = (contact_force_n(id) - raw_total) / static_cast<double>(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            auto& partner = partners[index];
            const auto force = partner.force_n + correction;
            const auto magnitude = vector_length(force);
            if (!(magnitude >= readable_force_n) || partner.points == 0)
                continue;
            const auto name = body_display_name(world, partner.other);
            auto partner_options = options;
            partner_options.start_offset_px = 0.0;
            partner_options.exit_offset_px = 0.0;
            partner_options.significant = true;
            partner_options.ends_at_anchor = true;
            partner_options.pusher = body_key(partner.other, 1);
            // The arrow comes from the neighbour's side and ends on the shared face, the way the
            // neighbour pushes this body; its plate names the push, so it never reads as a
            // property of the neighbour.
            const auto face = math::transform_point(drawn, math::inverse_transform_point(solved, partner.point_m / static_cast<double>(partner.points)));
            if (const auto* pusher = world.find_body(partner.other))
            {
                const auto marker = partner.other.index < body_frames_.size() ? static_cast<double>(body_frames_[partner.other.index].marker_radius_px) : 0.0;
                const auto ds = static_cast<double>(pixel_scale(settings));
                partner_options.shift_step_px = std::max(6.0 * ds, marker + 5.0 * ds);
                partner_options.maximum_length_px = push_reach(*pusher, camera, settings.interpolation_alpha, camera.world_to_screen(face), math::normalized(camera.world_to_screen_direction(force / magnitude)));
            }
            const auto label = name.empty() ? std::string { "Contact" } : core::substitute("Push from {}", name);
            draw_scaled_vector(face, force, settings.vector_scales.force, settings.theme.contact, camera, settings, list, label, core::DisplayQuantity::force, magnitude, ArrowHead::open, 1.2f, body_key(id, 2) ^ (static_cast<std::uint64_t>(partner.other.index + 1) << 40u), partner_options);
        }
    }

    void SceneRenderer::draw_contacts(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        const auto scale = pixel_scale(settings);
        if (settings.layers.is_enabled(VisualizationLayer::broad_phase_pairs))
        {
            for (const auto& pair : world.broad_phase_pairs())
            {
                const auto* first = world.find_body(pair.first);
                const auto* second = world.find_body(pair.second);
                if (first == nullptr || second == nullptr)
                {
                    continue;
                }
                const auto first_center = math::transform_point(first->interpolated_transform(settings.interpolation_alpha), first->mass_properties().center_of_mass_m);
                const auto second_center = math::transform_point(second->interpolated_transform(settings.interpolation_alpha), second->mass_properties().center_of_mass_m);
                list.add_dashed_line(camera.world_to_screen(first_center), camera.world_to_screen(second_center), settings.theme.bounds, stroke(settings, 1.0f), 4.0f * scale, 3.0f * scale);
            }
        }

        const auto show_points = settings.layers.is_enabled(VisualizationLayer::contact_points);
        const auto show_normals = settings.layers.is_enabled(VisualizationLayer::contact_normals);
        if (!show_points && !show_normals)
        {
            return;
        }

        // A body only a few pixels across would vanish under a full contact dot, so its contacts
        // are drawn as small rings set onto the surface it touches.
        const auto small = [&](physics::BodyId id)
        {
            if (id.index >= body_frames_.size())
                return false;
            const auto extent = body_frames_[id.index].smaller_extent_px;
            const auto* body = world.find_body(id);
            return body && body->type() != BodyType::static_body && extent > 0.0f && extent < 24.0f * scale;
        };
        for (const auto& manifold : world.manifolds())
        {
            const auto small_first = manifold.point_count > 0 && show_points && small(manifold.first);
            const auto small_second = manifold.point_count > 0 && show_points && small(manifold.second);
            const auto screen_normal = math::normalized(camera.world_to_screen_direction(manifold.normal));
            for (std::size_t index = 0; index < manifold.point_count; ++index)
            {
                const auto& point = manifold.points[index];
                const auto screen_point = camera.world_to_screen(point.world_position_m);

                if (show_normals)
                {
                    list.add_arrow(screen_point, screen_point + screen_normal * (24.0 * scale), settings.theme.contact, stroke(settings, 1.5f), 7.0f * scale);
                }
                if (show_points)
                {
                    if (small_first != small_second && math::is_finite(screen_normal))
                    {
                        // The normal points from the first body to the second; the ring sits on
                        // the larger body's side of the contact.
                        const auto ring = 2.0f * scale;
                        const auto onto = small_second ? -screen_normal : screen_normal;
                        list.add_circle_outline(screen_point + onto * static_cast<double>(ring + stroke(settings, 1.0f)), ring, settings.theme.contact, stroke(settings, 1.0f));
                        continue;
                    }
                    list.add_circle_fill(screen_point, 3.5f * scale, settings.theme.contact);
                    list.add_circle_outline(screen_point, 3.5f * scale, settings.theme.label_plate.with_alpha(0.9f), stroke(settings, 1.0f));
                }
            }
        }
    }

    void SceneRenderer::draw_trajectories(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        const auto width = stroke(settings, 1.5f);
        for (const auto& [index, trail] : trajectories_)
        {
            if (trail.samples.size() < 2)
            {
                continue;
            }

            scratch_points_.clear();
            scratch_points_.reserve(trail.samples.size());
            for (const auto& sample : trail.samples)
            {
                scratch_points_.push_back(camera.world_to_screen(sample));
            }
            // Oldest first: the path fades and thins toward where the body has been.
            list.add_tapered_polyline(scratch_points_, settings.theme.trajectory.with_alpha(0.0f), settings.theme.trajectory, width * 0.35f, width);
        }
    }

    void SceneRenderer::place_labels(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list)
    {
        next_label_memory_.clear();
        if (label_requests_.empty())
        {
            label_memory_.clear();
            return;
        }
        const auto metrics = label_metrics(settings);
        const auto ds = static_cast<double>(pixel_scale(settings));
        const auto& theme = settings.theme;
        // Labels stay on the visible stage: never under the toolbar, rails, panels or status line.
        const auto viewport_width = static_cast<double>(camera.viewport().width), viewport_height = static_cast<double>(camera.viewport().height);
        auto stage = visible_rect(camera, visible_stage_, 12.0 * ds);
        stage.minimum = math::max_components(stage.minimum, Vec2 {});
        stage.maximum = math::min_components(stage.maximum, Vec2 { viewport_width, viewport_height });

        // A reading repeated word for word on several bodies, such as the weights of a chain of
        // equal masses, is said once for all of them, after the names that tell the bodies apart.
        // While a body is singled out its own reading shows and the others' repeats give way.
        const auto value_plate_of = [](const LabelRequest& request)
        {
            return request.placement == LabelPlacement::along_vector && request.owner != 0;
        };
        for (std::size_t index = 0; index < label_requests_.size(); ++index)
        {
            auto& request = label_requests_[index];
            if (!value_plate_of(request) || request.repeated)
                continue;
            const auto text = request.primary;
            bool shared = false, essential = request.essential;
            for (std::size_t other = index + 1; other < label_requests_.size(); ++other)
                if (const auto& candidate = label_requests_[other]; value_plate_of(candidate) && candidate.owner != request.owner && candidate.primary == text)
                {
                    shared = true;
                    essential = essential || candidate.essential;
                }
            if (!shared)
                continue;
            for (std::size_t member = index; member < label_requests_.size(); ++member)
            {
                auto& candidate = label_requests_[member];
                if (!value_plate_of(candidate) || candidate.primary != text)
                    continue;
                candidate.repeated = true;
                if (candidate.essential)
                    continue;
                candidate.required = false;
                candidate.priority = std::min(candidate.priority, 30);
                if (!essential)
                    candidate.primary += " each";
            }
        }

        thread_local std::vector<std::size_t> order;
        order.resize(label_requests_.size());
        std::iota(order.begin(), order.end(), std::size_t { 0 });
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b)
            {
                return label_requests_[a].priority > label_requests_[b].priority;
            });
        thread_local std::vector<Rect> placed, candidates;
        thread_local std::vector<std::size_t> placed_repeats;
        placed.clear();
        placed_repeats.clear();
        for (const auto& [minimum, maximum] : reserved_label_areas_)
            placed.push_back({ minimum, maximum });
        const auto overlays_begin = placed.size();
        for (const auto& [minimum, maximum] : overlay_label_areas_)
            placed.push_back({ minimum, maximum });
        const auto overlays_end = placed.size();
        // A card over the stage stands in for its own body's plates alone. Any other plate with no
        // clear place beside it may then meet the card's edge, or tuck under it by no more than the
        // usual spacing, rather than leave the stage.
        const auto replaced_key = settings.label_replaced.is_valid() && overlays_end > overlays_begin ? body_key(settings.label_replaced, 1) : std::uint64_t { 0 };
        const auto separator = std::round(5.0 * static_cast<double>(text_pixel_scale(settings)));
        const auto margin = 2.0 * ds;
        // A small copy of an off-scale arrow's mark, for a plate with no room for the words.
        const auto mark_width = std::round(18.0 * static_cast<double>(text_pixel_scale(settings)));

        const auto support = [](double width, double height, const Vec2& direction)
        {
            return std::abs(direction.x) * width * 0.5 + std::abs(direction.y) * height * 0.5;
        };
        const auto rotate = [](const Vec2& value, double angle)
        {
            const auto c = std::cos(angle), s = std::sin(angle);
            return Vec2 { value.x * c - value.y * s, value.x * s + value.y * c };
        };
        const auto centered = [](const Vec2& center, double width, double height)
        {
            return Rect { center - Vec2 { width * 0.5, height * 0.5 }, center + Vec2 { width * 0.5, height * 0.5 } };
        };

        const auto build_candidates = [&](const LabelRequest& request, double width)
        {
            candidates.clear();
            const auto height = metrics.height;
            const auto gap = metrics.gap;
            switch (request.placement)
            {
            case LabelPlacement::along_vector:
            {
                const auto direction = math::length_squared(request.direction) > 0.0 ? math::normalized(request.direction) : Vec2 { 1.0, 0.0 };
                for (const auto distance : { gap, gap * 3.5 })
                    for (const auto angle : { 0.0, 0.9, -0.9, 1.6, -1.6, 2.3, -2.3 })
                    {
                        const auto u = rotate(direction, angle);
                        candidates.push_back(centered(request.anchor + u * (distance + support(width, height, u)), width, height));
                    }
                // Beside the shaft just behind the head, clear of the head's width, where a wide
                // plate on a long arrow need not lie across the shaft.
                for (const auto sign : { 1.0, -1.0 })
                {
                    const auto across = math::perpendicular(direction) * sign;
                    candidates.push_back(centered(request.anchor - direction * (support(width, height, direction) + gap) + across * (gap + 5.0 * ds + support(width, height, across)), width, height));
                }
                // A short arrow ends inside its own body; these keep the plate just outside it.
                if (!request.box.is_empty())
                {
                    const auto& box = request.box;
                    const auto within = [](double value, double low, double high)
                    {
                        return low <= high ? std::clamp(value, low, high) : (low + high) * 0.5;
                    };
                    const auto middle = within(request.anchor.y, box.minimum.y + height * 0.5, box.maximum.y - height * 0.5);
                    const auto centre = within(request.anchor.x, box.minimum.x + width * 0.5, box.maximum.x - width * 0.5);
                    const auto toward_right = direction.x >= 0.0;
                    const auto toward_down = direction.y >= 0.0;
                    for (const auto right : { toward_right, !toward_right })
                        candidates.push_back(right ? Rect { { box.maximum.x + gap, middle - height * 0.5 }, { box.maximum.x + gap + width, middle + height * 0.5 } }
                                                   : Rect { { box.minimum.x - gap - width, middle - height * 0.5 }, { box.minimum.x - gap, middle + height * 0.5 } });
                    for (const auto down : { toward_down, !toward_down })
                        candidates.push_back(down ? Rect { { centre - width * 0.5, box.maximum.y + gap }, { centre + width * 0.5, box.maximum.y + gap + height } }
                                                  : Rect { { centre - width * 0.5, box.minimum.y - gap - height }, { centre + width * 0.5, box.minimum.y - gap } });
                }
                break;
            }
            case LabelPlacement::beside_line:
            {
                // Above the line first, then below it, at its middle and then toward its ends;
                // past either end last.
                const auto length = math::length(request.direction);
                const auto along = length > 1.0e-9 ? request.direction / length : Vec2 { 1.0, 0.0 };
                auto normal = math::perpendicular(along);
                if (normal.y > 0.0)
                    normal = -normal;
                for (const auto distance : { request.clearance + gap * 0.5, request.clearance + gap * 3.5, request.clearance + gap * 7.0 })
                    for (const auto fraction : { 0.5, 0.3, 0.7, 0.15, 0.85 })
                        for (const auto sign : { 1.0, -1.0 })
                        {
                            const auto u = normal * sign;
                            candidates.push_back(centered(request.origin + request.direction * fraction + u * (distance + support(width, height, u)), width, height));
                        }
                for (const auto end : { 1.0, 0.0 })
                {
                    const auto u = along * (end > 0.5 ? 1.0 : -1.0);
                    candidates.push_back(centered(request.origin + request.direction * end + u * (gap * 2.0 + support(width, height, u)), width, height));
                }
                break;
            }
            case LabelPlacement::around_point:
            {
                for (const auto distance : { 10.0 * ds, 26.0 * ds })
                    for (const auto& u : { Vec2 { 0.8, -0.6 }, Vec2 { 0.8, 0.6 }, Vec2 { -0.8, -0.6 }, Vec2 { -0.8, 0.6 }, Vec2 { 0.0, -1.0 }, Vec2 { 0.0, 1.0 }, Vec2 { 1.0, 0.0 }, Vec2 { -1.0, 0.0 } })
                        candidates.push_back(centered(request.anchor + u * (distance + support(width, height, u)), width, height));
                break;
            }
            case LabelPlacement::around_box:
            case LabelPlacement::inside_box:
            {
                auto box = request.box;
                if (box.is_empty())
                {
                    box.expand(request.anchor);
                }
                if (request.placement == LabelPlacement::inside_box)
                {
                    // Fixed bodies carry a quiet name inside their visible section, else beside it.
                    const auto left = std::max(box.minimum.x, stage.minimum.x) + 10.0 * ds;
                    const auto right = std::min(box.maximum.x, stage.maximum.x) - 10.0 * ds;
                    const auto top = std::max(box.minimum.y, stage.minimum.y);
                    const auto bottom = std::min(box.maximum.y, stage.maximum.y);
                    if (bottom - top >= height + 6.0 * ds && right - left >= width)
                    {
                        candidates.push_back({ { left, bottom - height - 3.0 * ds }, { left + width, bottom - 3.0 * ds } });
                        candidates.push_back({ { right - width, bottom - height - 3.0 * ds }, { right, bottom - 3.0 * ds } });
                    }
                    candidates.push_back({ { left, box.maximum.y + gap }, { left + width, box.maximum.y + gap + height } });
                    candidates.push_back({ { left, box.minimum.y - gap - height }, { left + width, box.minimum.y - gap } });
                    break;
                }
                // A line marker is named level with the line at either end, where nothing else
                // can be mistaken for what it names.
                if (request.marker)
                {
                    // Centred on the line first, then resting on it where something sits close
                    // below, such as the floor under a lowest marker, then just above or below it
                    // where neighbouring lines are close. Every place at the line's right end comes
                    // before any at its left, so a set of lines is named on one side.
                    const auto y = (box.minimum.y + box.maximum.y) * 0.5;
                    for (const auto right : { true, false })
                        for (const auto rise : { 0.5, 0.8, 1.15, -0.15 })
                            candidates.push_back(right ? Rect { { box.maximum.x + gap, y - height * rise }, { box.maximum.x + gap + width, y + height * (1.0 - rise) } }
                                                       : Rect { { box.minimum.x - gap - width, y - height * rise }, { box.minimum.x - gap, y + height * (1.0 - rise) } });
                }
                // Beside, diagonally off a corner, then above or below; a second, wider ring
                // follows for crowded neighbourhoods.
                for (const auto ring : { 1.0, 3.5 })
                {
                    const auto distance = gap * ring;
                    candidates.push_back({ { box.maximum.x + distance, request.anchor.y - height * 0.5 }, { box.maximum.x + distance + width, request.anchor.y + height * 0.5 } });
                    candidates.push_back({ { box.maximum.x + distance * 0.5, box.minimum.y - distance * 0.5 - height }, { box.maximum.x + distance * 0.5 + width, box.minimum.y - distance * 0.5 } });
                    candidates.push_back({ { request.anchor.x - width * 0.5, box.minimum.y - distance - height }, { request.anchor.x + width * 0.5, box.minimum.y - distance } });
                    candidates.push_back({ { box.minimum.x - distance - width, request.anchor.y - height * 0.5 }, { box.minimum.x - distance, request.anchor.y + height * 0.5 } });
                    candidates.push_back({ { box.minimum.x - distance * 0.5 - width, box.minimum.y - distance * 0.5 - height }, { box.minimum.x - distance * 0.5, box.minimum.y - distance * 0.5 } });
                    candidates.push_back({ { box.maximum.x + distance * 0.5, box.maximum.y + distance * 0.5 }, { box.maximum.x + distance * 0.5 + width, box.maximum.y + distance * 0.5 + height } });
                    candidates.push_back({ { request.anchor.x - width * 0.5, box.maximum.y + distance }, { request.anchor.x + width * 0.5, box.maximum.y + distance + height } });
                    candidates.push_back({ { box.minimum.x - distance * 0.5 - width, box.maximum.y + distance * 0.5 }, { box.minimum.x - distance * 0.5, box.maximum.y + distance * 0.5 + height } });
                }
                // Around the anchor itself, which suits long rotated bodies whose box is mostly
                // empty space.
                for (const auto distance : { 2.0 * gap, 6.0 * gap })
                    for (const auto& u : { Vec2 { 0.71, -0.71 }, Vec2 { -0.71, 0.71 }, Vec2 { -0.71, -0.71 }, Vec2 { 0.71, 0.71 }, Vec2 { 1.0, 0.0 }, Vec2 { -1.0, 0.0 }, Vec2 { 0.0, -1.0 }, Vec2 { 0.0, 1.0 } })
                        candidates.push_back(centered(request.anchor + u * (distance + support(width, height, u)), width, height));
                // A body larger than the free stage keeps its label over itself near the anchor.
                candidates.push_back(centered(request.anchor + Vec2 { width * 0.5 + 8.0 * ds, -height * 0.5 - 8.0 * ds }, width, height));
                break;
            }
            }
        };

        // The outlines of the object the label being placed names, and of the other bodies near
        // it, gathered once per label; the obstacles near the places being tried, once per
        // evaluation; and every body short enough to be read as a plate's subject, once.
        thread_local std::vector<std::size_t> own_obstacles, nearby_bodies, local_obstacles, body_obstacles;
        body_obstacles.clear();
        for (std::size_t item = 0; item < label_obstacles_.size(); ++item)
            if (const auto& obstacle = label_obstacles_[item]; (obstacle.owner >> 56u) == 1u && obstacle.maximum.x - obstacle.minimum.x < 0.5 * (stage.maximum.x - stage.minimum.x))
                body_obstacles.push_back(item);
        const auto own_outline = [&](const LabelRequest& request, const Rect& rect, Vec2& target)
        {
            target = request.anchor;
            auto best = 1.0e300;
            const auto center = (rect.minimum + rect.maximum) * 0.5;
            for (const auto item : own_obstacles)
                if (const auto& obstacle = label_obstacles_[item]; obstacle.count >= 2)
                {
                    const auto point = nearest_on_outline(obstacle_points_.data() + obstacle.offset, obstacle.count, center);
                    const auto distance = math::length(nearest_on_rect(rect, point) - point);
                    if (distance < best)
                    {
                        best = distance;
                        target = point;
                    }
                }
            return best < 1.0e299 ? best : math::length(nearest_on_rect(rect, request.anchor) - request.anchor);
        };

        // A label pushed away from what it names gets a hairline leader back to it.
        const auto leader = [&](const LabelRequest& request, const Rect& rect, Vec2& from, Vec2& to, double& distance)
        {
            to = request.anchor;
            distance = math::length(nearest_on_rect(rect, request.anchor) - request.anchor);
            double threshold = 3.0 * metrics.gap + 12.0 * ds;
            if (request.placement == LabelPlacement::along_vector)
                threshold = 1.5 * metrics.gap;
            if (request.placement == LabelPlacement::around_box)
            {
                distance = own_outline(request, rect, to);
                threshold = request.marker ? 0.5 * metrics.gap : 2.0 * metrics.gap;
            }
            if (request.placement == LabelPlacement::beside_line)
            {
                const auto center = (rect.minimum + rect.maximum) * 0.5;
                const auto length = math::length_squared(request.direction);
                const auto t = length > 0.0 ? std::clamp(math::dot(center - request.origin, request.direction) / length, 0.0, 1.0) : 0.0;
                to = request.origin + request.direction * t;
                distance = math::length(nearest_on_rect(rect, to) - to);
                threshold = request.clearance + 1.5 * metrics.gap;
            }
            from = nearest_on_rect(rect, to);
            return distance > threshold;
        };
        thread_local std::vector<std::pair<Vec2, Vec2>> leaders;
        leaders.clear();

        struct Choice
        {
            int index { -1 };
            double cost { 1.0e300 };
            bool conflict { true };
            // Set when the plate or its leader would lie over a plate already placed, or a value
            // plate over another body or another arrow.
            bool covers_plate { true };
            // Set when the plate lies nearer another object than the one it names.
            bool astray {};
            Rect rect;
        };
        // A fixed body spanning much of the stage is ground: a plate may rest on its section the
        // way a caption rests on a page, where on any other body it would hide that body.
        const auto ground = [&](const LabelObstacle& obstacle)
        {
            const auto slot = static_cast<std::size_t>(obstacle.owner & 0xffffffffu);
            return (obstacle.owner >> 56u) == 1u && slot < body_frames_.size() && !body_frames_[slot].movable &&
                (obstacle.maximum.x - obstacle.minimum.x > 0.35 * (stage.maximum.x - stage.minimum.x) || obstacle.maximum.y - obstacle.minimum.y > 0.35 * (stage.maximum.y - stage.minimum.y));
        };
        // Candidates are scored in preference order; a candidate stops being scored as soon as
        // it cannot beat the best so far, which keeps crowded frames cheap.
        constexpr double memory_bonus = 3.0, label_penalty = 600.0, stage_penalty = 400.0;
        const auto evaluate = [&](const LabelRequest& request, double width, int remembered, bool strict, bool own_body_allowed, bool flush_to_overlays = false)
        {
            build_candidates(request, width);
            // Only obstacles near some candidate can be met, counting where a place running off
            // the stage would be moved onto it; they are gathered once per evaluation.
            Rect near { { 1.0e300, 1.0e300 }, { -1.0e300, -1.0e300 } };
            for (const auto& candidate : candidates)
            {
                near.minimum = math::min_components(near.minimum, candidate.minimum);
                near.maximum = math::max_components(near.maximum, candidate.minimum + Vec2 { std::round(width), metrics.height });
            }
            const Vec2 low_overshoot = math::max_components(stage.minimum - near.minimum, Vec2 {}), high_overshoot = math::max_components(near.maximum - stage.maximum, Vec2 {});
            near.minimum -= high_overshoot + Vec2 { margin + 2.0, margin + 2.0 };
            near.maximum += low_overshoot + Vec2 { margin + 2.0, margin + 2.0 };
            local_obstacles.clear();
            const auto receding_blocks = value_plate_of(request) && !request.essential;
            for (std::size_t item = 0; item < label_obstacles_.size(); ++item)
                if (const auto& obstacle = label_obstacles_[item]; (receding_blocks || !obstacle.receding) && obstacle.maximum.x >= near.minimum.x && obstacle.minimum.x <= near.maximum.x && obstacle.maximum.y >= near.minimum.y && obstacle.minimum.y <= near.maximum.y)
                    local_obstacles.push_back(item);
            // A body's name and a connection's reading keep off the very thing they name.
            const auto name_plate = request.placement == LabelPlacement::around_box && request.owner == 0 && (request.key >> 56u) == 1u && !request.marker;
            const auto connection = (request.key >> 56u) == 3u || (request.key >> 56u) == 4u;
            Choice best;
            for (std::size_t index = 0; index < candidates.size(); ++index)
            {
                double cost = static_cast<double>(index) - (static_cast<int>(index) == remembered ? memory_bonus : 0.0);
                // A marker named anywhere but level with its line is easily read against the
                // wrong line, so remembered sides never outweigh an inline place.
                if (request.marker && index >= 8)
                    cost += 6.0;
                if (cost >= best.cost)
                    continue;
                auto rect = candidates[index];
                // Whole device pixels keep plate edges and glyphs crisp.
                rect.minimum = { std::round(rect.minimum.x), std::round(rect.minimum.y) };
                rect.maximum = rect.minimum + Vec2 { std::round(width), metrics.height };
                bool conflict = false, covers_plate = false, astray = false;
                if (rect.minimum.x < stage.minimum.x || rect.minimum.y < stage.minimum.y || rect.maximum.x > stage.maximum.x || rect.maximum.y > stage.maximum.y)
                {
                    // A place running off the stage is judged where it would be drawn, moved
                    // onto the stage, so it can never land on a plate unseen.
                    if (stage.maximum.x - stage.minimum.x >= width && stage.maximum.y - stage.minimum.y >= metrics.height)
                    {
                        const Vec2 shift { std::max(0.0, stage.minimum.x - rect.minimum.x) - std::max(0.0, rect.maximum.x - stage.maximum.x),
                            std::max(0.0, stage.minimum.y - rect.minimum.y) - std::max(0.0, rect.maximum.y - stage.maximum.y) };
                        rect.minimum = { std::round(rect.minimum.x + shift.x), std::round(rect.minimum.y + shift.y) };
                        rect.maximum = rect.minimum + Vec2 { std::round(width), metrics.height };
                        cost += 4.0 + 0.1 * math::length(shift) / ds;
                    }
                    else
                    {
                        cost += stage_penalty;
                        conflict = true;
                    }
                }
                const Rect padded { rect.minimum - Vec2 { margin, margin }, rect.maximum + Vec2 { margin, margin } };
                const Rect inset { rect.minimum + Vec2 { margin, margin }, rect.maximum - Vec2 { margin, margin } };
                for (std::size_t other = 0; other < placed.size() && cost < best.cost; ++other)
                    if (const auto area = overlap_area(flush_to_overlays && other >= overlays_begin && other < overlays_end ? inset : padded, placed[other]); area > 0.0)
                    {
                        cost += label_penalty + area * 0.5;
                        conflict = covers_plate = true;
                    }
                // A label that may give way is never placed in conflict, so such a place need not
                // be scored any further.
                if (cost >= best.cost || (strict && conflict))
                    continue;
                // A plate prefers to leave a stroked overlay ring, such as the selection's rotation
                // ring, clear.
                for (const auto& [centre, radius] : overlay_rings_)
                {
                    const auto nearest = math::length(nearest_on_rect(padded, centre) - centre);
                    const auto farthest = std::max({ math::length(padded.minimum - centre), math::length(padded.maximum - centre), math::length(Vec2 { padded.minimum.x, padded.maximum.y } - centre), math::length(Vec2 { padded.maximum.x, padded.minimum.y } - centre) });
                    if (nearest <= radius + 2.0 * ds && farthest >= radius - 2.0 * ds)
                        cost += 45.0;
                }
                // A leader never splits another plate or crosses another leader, and a plate never
                // sits on another's leader.
                Vec2 leader_from, leader_to;
                double leader_length = 0.0;
                {
                    const auto led = leader(request, rect, leader_from, leader_to, leader_length);
                    const std::array<Vec2, 2> segment { leader_from, leader_to };
                    for (std::size_t other = 0; led && other < placed.size() && cost < best.cost; ++other)
                        if (rect_hits_polygon(placed[other], segment.data(), segment.size()))
                        {
                            cost += label_penalty;
                            conflict = covers_plate = true;
                        }
                    for (std::size_t other = 0; other < leaders.size() && cost < best.cost; ++other)
                    {
                        const std::array<Vec2, 2> line { leaders[other].first, leaders[other].second };
                        if (rect_hits_polygon(padded, line.data(), line.size()))
                        {
                            cost += label_penalty;
                            conflict = covers_plate = true;
                        }
                        if (led && segments_cross(leader_from, leader_to, leaders[other].first, leaders[other].second))
                            cost += 60.0;
                    }
                }
                // A label nearer another object's centre than its own reads as the wrong caption:
                // optional labels give way, required ones pay to avoid it. A value plate measures
                // from the tip of its arrow.
                const auto center = (rect.minimum + rect.maximum) * 0.5;
                const auto value_plate = value_plate_of(request);
                // A marker named level with its line is unambiguous whatever lies nearby.
                const auto inline_marker = request.marker && index < 8;
                const auto own_distance = (request.placement == LabelPlacement::around_box && !inline_marker) || value_plate ? math::length(center - request.anchor) : -1.0;
                const auto own_body = value_plate ? request.owner : request.key;
                if (request.placement == LabelPlacement::around_box)
                {
                    // Nearer is clearer: a plate pays for every pixel between it and its object.
                    cost += 0.04 * leader_length / ds;
                    // A name as near another body as its own, as at the seam between two stacked
                    // boxes, could name either.
                    if (name_plate)
                        for (const auto item : nearby_bodies)
                            if (const auto& obstacle = label_obstacles_[item]; rect_distance(rect, obstacle.minimum, obstacle.maximum) < leader_length + 2.0 * ds)
                            {
                                cost += 20.0;
                                break;
                            }
                    // A marker's leader never runs through another object, such as the floor
                    // under the lowest reference line.
                    if (request.marker)
                    {
                        for (const auto& obstacle : label_obstacles_)
                            if (obstacle.owner != request.key && (obstacle.owner >> 56u) == 1u && obstacle.count >= 3 && segment_crosses_polygon(leader_from, leader_to, obstacle_points_.data() + obstacle.offset, obstacle.count))
                            {
                                cost += label_penalty;
                                conflict = true;
                                break;
                            }
                    }
                }
                else if (request.placement == LabelPlacement::beside_line)
                    cost += 0.04 * leader_length / ds;
                else if (value_plate)
                {
                    cost += 0.04 * math::length(nearest_on_rect(rect, request.anchor) - request.anchor) / ds;
                    // A value plate wholly behind its arrow's tail reads as the opposite direction,
                    // the more so the further behind it lies.
                    const Vec2 leading { request.direction.x >= 0.0 ? rect.maximum.x : rect.minimum.x, request.direction.y >= 0.0 ? rect.maximum.y : rect.minimum.y };
                    if (const auto behind = -math::dot(leading - request.origin, request.direction); math::length_squared(request.direction) > 0.0 && behind > 0.0)
                    {
                        cost += 150.0 + 1.5 * behind / ds;
                        if (strict)
                        {
                            cost += label_penalty;
                            conflict = true;
                        }
                    }
                }
                for (std::size_t body = 0; own_distance >= 0.0 && body < body_obstacles.size() && cost < best.cost && !(strict && conflict); ++body)
                {
                    const auto& obstacle = label_obstacles_[body_obstacles[body]];
                    if (obstacle.owner != own_body && math::length_squared(center - (obstacle.minimum + obstacle.maximum) * 0.5) < own_distance * own_distance * (0.85 * 0.85))
                    {
                        cost += request.required ? 60.0 : 14.0;
                        astray = true;
                        if (strict)
                        {
                            cost += label_penalty;
                            conflict = true;
                        }
                    }
                }
                for (std::size_t slot = 0; slot < local_obstacles.size() && cost < best.cost && !(strict && conflict); ++slot)
                {
                    const auto& obstacle = label_obstacles_[local_obstacles[slot]];
                    if (obstacle.maximum.x < padded.minimum.x || obstacle.minimum.x > padded.maximum.x || obstacle.maximum.y < padded.minimum.y || obstacle.minimum.y > padded.maximum.y)
                        continue;
                    if (rect_hits_polygon(padded, obstacle_points_.data() + obstacle.offset, obstacle.count))
                    {
                        if (ground(obstacle) && obstacle.owner != request.key)
                        {
                            cost += 6.0;
                            continue;
                        }
                        const auto own = obstacle.owner == request.key;
                        // An optional label never covers another object; it gives way instead.
                        const auto other_body = obstacle.owner != own_body && (obstacle.owner >> 56u) == 1u;
                        cost += own || (obstacle.owner >> 56u) == 2u ? 30.0 : 22.0;
                        // A name plate laid over an arrow hides the arrow it sits on.
                        if (!value_plate && (obstacle.owner >> 56u) == 2u)
                            cost += 90.0;
                        // A value plate keeps off the body it describes whenever a place exists, and
                        // off another body's arrows: one that can find no other place is left out
                        // unless it is the selection's. Laid over another body it would hide it, or
                        // read as that body's value.
                        if (value_plate && obstacle.owner == own_body)
                            cost += 300.0;
                        if (value_plate && other_body)
                        {
                            cost += 120.0;
                            covers_plate = true;
                        }
                        if (value_plate && (obstacle.owner >> 56u) == 2u && (obstacle.owner != (own_body ^ (std::uint64_t { 3 } << 56u)) || (obstacle.plate != 0 && obstacle.plate != request.key)))
                        {
                            cost += 200.0;
                            covers_plate = true;
                        }
                        // A body's name, or a connection's reading, laid over the body or the drawing
                        // it names hides it: such a place is taken only when no other is free, and
                        // a connection's reading pays as much for covering a body it joins.
                        if ((name_plate && own && !own_body_allowed) || (connection && (own || other_body)))
                        {
                            cost += 250.0;
                            if (strict)
                            {
                                cost += label_penalty;
                                conflict = true;
                            }
                        }
                        // An arrow or a connection is never covered by a label that may give way.
                        const auto arrow = (obstacle.owner >> 56u) == 2u || ((obstacle.owner >> 56u) == 3u && !own) || ((obstacle.owner >> 56u) == 4u && !own);
                        if (strict && (other_body || arrow))
                        {
                            cost += label_penalty;
                            conflict = true;
                        }
                    }
                }
                if (cost < best.cost && !(strict && conflict))
                {
                    best.cost = cost;
                    best.index = static_cast<int>(index);
                    best.conflict = conflict;
                    best.covers_plate = covers_plate;
                    best.astray = astray;
                    best.rect = rect;
                }
            }
            return best;
        };

        // In a crowded scene only the selected body's vector values are guaranteed a plate;
        // the others are placed while space remains.
        const auto vector_labels = static_cast<std::size_t>(std::count_if(label_requests_.begin(), label_requests_.end(), [](const LabelRequest& request)
            {
                return request.placement == LabelPlacement::along_vector;
            }));
        const Rect reach { stage.minimum - Vec2 { 120.0 * ds, 120.0 * ds }, stage.maximum + Vec2 { 120.0 * ds, 120.0 * ds } };
        for (const auto request_index : order)
        {
            const auto& request = label_requests_[request_index];
            if (request.primary.empty())
                continue;
            // A label for something far off the stage could never be read.
            if (request.anchor.x < reach.minimum.x || request.anchor.y < reach.minimum.y || request.anchor.x > reach.maximum.x || request.anchor.y > reach.maximum.y)
                continue;
            // A repeated reading already said for one body is not said again for another.
            if (request.repeated && !request.essential && std::any_of(placed_repeats.begin(), placed_repeats.end(), [&](std::size_t other)
                                                              {
                                                                  return label_requests_[other].primary == request.primary;
                                                              }))
                continue;
            const auto required = request.required && (request.essential || request.placement != LabelPlacement::along_vector || vector_labels <= 12);
            own_obstacles.clear();
            nearby_bodies.clear();
            const auto names_body = request.placement == LabelPlacement::around_box && request.owner == 0 && (request.key >> 56u) == 1u && !request.marker;
            auto near = request.box;
            near.expand(request.anchor);
            const auto reach_px = 160.0 * ds;
            for (std::size_t item = 0; item < label_obstacles_.size(); ++item)
            {
                const auto& obstacle = label_obstacles_[item];
                if (obstacle.owner == request.key)
                    own_obstacles.push_back(item);
                else if (names_body && (obstacle.owner >> 56u) == 1u && obstacle.minimum.x < near.maximum.x + reach_px && obstacle.maximum.x > near.minimum.x - reach_px &&
                    obstacle.minimum.y < near.maximum.y + reach_px && obstacle.maximum.y > near.minimum.y - reach_px && !ground(obstacle))
                    nearby_bodies.push_back(item);
            }
            int remembered = -1;
            for (const auto& [key, choice] : label_memory_)
                if (key == request.key)
                    remembered = choice;
            const auto primary_width = text_width(request.primary, metrics.scale);
            const auto secondary_width = request.secondary.empty() ? 0.0 : text_width(request.secondary, metrics.scale);
            auto with_secondary = !request.secondary.empty();
            auto with_mark = !with_secondary && request.scale_note != ScaleNote::none;
            auto width = metrics.pad_x * 2.0 + primary_width + (with_secondary ? separator + secondary_width : with_mark ? separator + mark_width
                                                                                                                         : 0.0);
            auto choice = evaluate(request, width, remembered, !required, false);
            // The secondary text is a supplement: crowding drops it before it overlaps anything.
            // A note that the drawing is off its scale gives way to a small mark in place of the
            // words even where the words would only lay the plate over a body.
            if ((choice.conflict || (request.scale_note != ScaleNote::none && choice.cost >= 100.0)) && with_secondary)
            {
                const auto marked = request.scale_note != ScaleNote::none;
                const auto compact_width = metrics.pad_x * 2.0 + primary_width + (marked ? separator + mark_width : 0.0);
                const auto compact = evaluate(request, compact_width, remembered, !required, false);
                if (!compact.conflict && (choice.conflict || compact.cost < choice.cost))
                {
                    choice = compact;
                    width = compact_width;
                    with_secondary = false;
                    with_mark = marked;
                }
            }
            if (replaced_key != 0 && request.key != replaced_key && request.owner != replaced_key &&
                (choice.index < 0 || (choice.conflict && !required) || (choice.covers_plate && !request.essential && request.priority < 100)))
                choice = evaluate(request, width, remembered, !required, false, true);
            // A body's name finds a place over its own body only when no place beside it is free,
            // and then only the selection's: another body's name gives way instead.
            if (required && (choice.index < 0 || choice.conflict) && names_body)
                if (const auto over = evaluate(request, width, remembered, !required, true); over.index >= 0 && (!over.conflict || choice.index < 0))
                    choice = over;
            if (choice.index < 0 || (choice.conflict && !required))
                continue;
            // Two plates on top of each other leave neither legible, so a label that is not the
            // selection's own is left out rather than laid over another plate or across another
            // body or its arrow.
            if (choice.covers_plate && !request.essential && request.priority < 100)
                continue;
            auto rect = choice.rect;
            // A required plate with no place on the stage is brought onto it; its leader then
            // points back to what it names.
            if (stage.maximum.x - stage.minimum.x >= rect.maximum.x - rect.minimum.x && stage.maximum.y - stage.minimum.y >= rect.maximum.y - rect.minimum.y)
            {
                const Vec2 shift { std::max(0.0, stage.minimum.x - rect.minimum.x) - std::max(0.0, rect.maximum.x - stage.maximum.x),
                    std::max(0.0, stage.minimum.y - rect.minimum.y) - std::max(0.0, rect.maximum.y - stage.maximum.y) };
                rect.minimum += shift;
                rect.maximum += shift;
            }
            placed.push_back(rect);
            if (request.repeated)
                placed_repeats.push_back(request_index);
            next_label_memory_.emplace_back(request.key, choice.index);
            double distance = 0.0;
            // A plate that stands nearer another object than its own always has a leader, so it
            // is never read as that object's.
            if (Vec2 from, target; leader(request, rect, from, target, distance) || (choice.astray && distance > 0.5 * metrics.gap))
            {
                list.add_line(from, target, theme.label_muted.with_alpha(theme.label_muted.alpha * 0.7f), static_cast<float>(ds));
                list.add_circle_fill(target, static_cast<float>(1.5 * ds), theme.label_muted);
                leaders.emplace_back(from, target);
            }
            list.add_rounded_rectangle_fill(rect.minimum, rect.maximum, static_cast<float>(metrics.radius), theme.label_plate);
            if (theme.label_border.alpha > 0.0f)
                list.add_rounded_rectangle_outline(rect.minimum + Vec2 { 0.5 * ds, 0.5 * ds }, rect.maximum - Vec2 { 0.5 * ds, 0.5 * ds }, static_cast<float>(metrics.radius), theme.label_border, static_cast<float>(ds));
            const auto text_y = std::round(rect.minimum.y + (metrics.height + metrics.cap_height) * 0.5 - metrics.ascender);
            const auto text_x = rect.minimum.x + metrics.pad_x;
            list.add_text({ text_x, text_y }, request.primary, request.primary_color, metrics.scale);
            if (with_secondary)
                list.add_text({ text_x + primary_width + separator, text_y }, request.secondary, request.secondary_color, metrics.scale);
            if (with_mark)
            {
                // The arrow's own mark in miniature: a dotted shaft for a drawing longer than its
                // value, a slanted break for one shorter.
                const auto tps = static_cast<double>(text_pixel_scale(settings));
                const auto y = std::round(rect.minimum.y + metrics.height * 0.5) + 0.5;
                const Vec2 from { text_x + primary_width + separator, y }, to { text_x + primary_width + separator + mark_width, y };
                const auto ink = request.secondary_color;
                if (request.scale_note == ScaleNote::longer)
                    list.add_arrow(from, to, ink, static_cast<float>(1.5 * tps), static_cast<float>(6.0 * tps), ArrowHead::filled, 0.0f, static_cast<float>(2.0 * tps));
                else
                {
                    list.add_arrow(from, to, ink, static_cast<float>(1.5 * tps), static_cast<float>(6.0 * tps));
                    const auto at = from + Vec2 { mark_width * 0.36, 0.0 };
                    list.add_line(at + Vec2 { -2.0 * tps, 4.0 * tps }, at + Vec2 { 2.0 * tps, -4.0 * tps }, theme.label_plate.with_alpha(1.0f), static_cast<float>(2.5 * tps), StrokeCap::butt);
                }
            }
        }
        label_memory_.swap(next_label_memory_);
    }

} // namespace rigidbodies::render
