#include <rigidbodies/app/stage_overlay_drawing.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace rigidbodies::app
{
    using math::Vec2;
    using render::Color;

    namespace
    {
        // The compiler expects premultiplied vertex colours on meshes built by hand.
        Color premultiplied(const Color& color)
        {
            const auto alpha = std::isfinite(color.alpha) ? std::clamp(color.alpha, 0.0f, 1.0f) : 0.0f;
            return { color.red * alpha, color.green * alpha, color.blue * alpha, alpha };
        }

        float luminance(const Color& color)
        {
            return 0.2126f * color.red + 0.7152f * color.green + 0.0722f * color.blue;
        }

        struct DashStyle
        {
            double on {}, off {}, start_width {}, end_width {};
            Color color;
            float end_opacity { 1.0f };
        };

        // One quad per visible dash. A dash that spans a vertex is split there; at overlay stroke
        // widths the seam is well under a pixel, and the compiler feathers every quad edge.
        void append_dashes(render::IndexedMesh& mesh, const std::vector<Vec2>& points, bool closed, DashStyle style)
        {
            const auto count = points.size();
            if (count < 2)
                return;
            const auto segments = closed ? count : count - 1;
            double total = 0.0;
            for (std::size_t index = 0; index < segments; ++index)
                total += math::length(points[(index + 1) % count] - points[index]);
            if (!std::isfinite(total) || total <= 0.0 || !std::isfinite(style.on) || !std::isfinite(style.off))
                return;
            style.on = std::max(style.on, 0.25);
            style.off = std::max(style.off, 0.25);
            constexpr double maximum_dashes = 4096.0;
            if (total / (style.on + style.off) > maximum_dashes)
            {
                const auto stretch = total / (style.on + style.off) / maximum_dashes;
                style.on *= stretch;
                style.off *= stretch;
            }
            if (closed)
            {
                const auto whole = std::max(1.0, std::round(total / (style.on + style.off)));
                const auto factor = total / (whole * (style.on + style.off));
                style.on *= factor;
                style.off *= factor;
            }
            const auto colour_at = [&](double fraction)
            {
                auto value = style.color;
                value.alpha *= static_cast<float>(1.0 + (static_cast<double>(style.end_opacity) - 1.0) * fraction);
                return premultiplied(value);
            };
            const auto half_width_at = [&](double fraction)
            {
                return 0.5 * (style.start_width + (style.end_width - style.start_width) * fraction);
            };
            mesh.vertices.reserve(mesh.vertices.size() + static_cast<std::size_t>(total / (style.on + style.off) + 2.0) * 4 + 8);
            mesh.indices.reserve(mesh.indices.size() + static_cast<std::size_t>(total / (style.on + style.off) + 2.0) * 6 + 12);
            bool drawing = true;
            auto remaining = style.on;
            double travelled = 0.0;
            for (std::size_t index = 0; index < segments; ++index)
            {
                const auto& start = points[index];
                const auto delta = points[(index + 1) % count] - start;
                const auto length = math::length(delta);
                if (!(length > 1.0e-9))
                    continue;
                const auto direction = delta / length;
                const auto normal = math::perpendicular(direction);
                double position = 0.0;
                while (position < length - 1.0e-9)
                {
                    const auto step = std::min(remaining, length - position);
                    if (drawing && step > 1.0e-6)
                    {
                        const auto first_fraction = (travelled + position) / total;
                        const auto second_fraction = (travelled + position + step) / total;
                        const auto first = start + direction * position;
                        const auto second = start + direction * (position + step);
                        const auto first_half = half_width_at(first_fraction), second_half = half_width_at(second_fraction);
                        const auto first_colour = colour_at(first_fraction), second_colour = colour_at(second_fraction);
                        const auto base = static_cast<int>(mesh.vertices.size());
                        mesh.vertices.push_back({ first + normal * first_half, {}, first_colour });
                        mesh.vertices.push_back({ first - normal * first_half, {}, first_colour });
                        mesh.vertices.push_back({ second - normal * second_half, {}, second_colour });
                        mesh.vertices.push_back({ second + normal * second_half, {}, second_colour });
                        for (const auto offset : { 0, 1, 2, 0, 2, 3 })
                            mesh.indices.push_back(base + offset);
                    }
                    position += step;
                    remaining -= step;
                    if (remaining <= 1.0e-9)
                    {
                        drawing = !drawing;
                        remaining = drawing ? style.on : style.off;
                    }
                }
                travelled += length;
            }
        }

        double glyph_em(unsigned char code)
        {
            if (code >= '0' && code <= '9')
                return 0.62;
            switch (code)
            {
            case ' ':
                return 0.27;
            case '.':
            case ',':
            case ':':
            case ';':
            case '\'':
            case '|':
                return 0.28;
            case 'i':
            case 'l':
            case 'j':
            case 'I':
                return 0.25;
            case 'f':
            case 't':
            case 'r':
            case '/':
            case '(':
            case ')':
                return 0.37;
            case 'm':
            case 'w':
            case 'M':
            case 'W':
                return 0.85;
            default:
                break;
            }
            if (code >= 'A' && code <= 'Z')
                return 0.68;
            if (code >= 0xc0)
                return 0.6;
            if (code >= 0x80)
                return 0.0;
            return 0.55;
        }
    }

    OverlayPalette overlay_palette(const render::Theme& theme)
    {
        OverlayPalette palette;
        const auto background = luminance(theme.background);
        palette.dark = background < 0.5f;
        const auto projector = !palette.dark && background > 0.995f && luminance(theme.panel_title) < 0.01f;
        palette.accent = theme.panel_accent;
        palette.selection = theme.selection;
        palette.surface = palette.dark ? Color::from_bytes(244, 246, 250) : Color::from_bytes(255, 255, 255);
        palette.plate = theme.panel_background.with_alpha(0.88f);
        palette.text = theme.panel_title;
        palette.muted = theme.panel_text;
        palette.shadow = { 0.0f, 0.0f, 0.0f, palette.dark ? 0.35f : 0.16f };
        palette.danger = palette.dark ? Color::from_bytes(242, 107, 112) : projector ? Color::from_bytes(179, 0, 27)
                                                                                     : Color::from_bytes(220, 38, 38);
        return palette;
    }

    OverlayPainter::OverlayPainter(render::DrawList& list, const render::Theme& theme, double scale)
        : list_(&list), palette_(overlay_palette(theme)), scale_(std::isfinite(scale) ? std::clamp(scale, 0.25, 8.0) : 1.0)
    {
    }

    double OverlayPainter::px(double logical) const
    {
        return logical * scale_;
    }

    float OverlayPainter::width(double logical) const
    {
        return static_cast<float>(logical * scale_);
    }

    const OverlayPalette& OverlayPainter::palette() const
    {
        return palette_;
    }

    render::DrawList& OverlayPainter::list() const
    {
        return *list_;
    }

    void OverlayPainter::dashed_path(const std::vector<Vec2>& points, bool closed, const Color& color, double width_logical, double on_logical, double off_logical) const
    {
        auto mesh = std::make_shared<render::IndexedMesh>();
        append_dashes(*mesh, points, closed, { px(on_logical), px(off_logical), px(width_logical), px(width_logical), color, 1.0f });
        list_->add_indexed_mesh(std::move(mesh));
    }

    void OverlayPainter::dashed_circle(const Vec2& centre, double radius_px, const Color& color, double width_logical, double on_logical, double off_logical) const
    {
        if (!math::is_finite(centre) || !std::isfinite(radius_px) || radius_px <= 0.0)
            return;
        const auto segments = static_cast<int>(std::clamp(std::ceil(math::two_pi * radius_px / 3.0), 24.0, 512.0));
        std::vector<Vec2> points;
        points.reserve(static_cast<std::size_t>(segments));
        for (int index = 0; index < segments; ++index)
        {
            const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(segments);
            points.push_back(centre + Vec2 { std::cos(angle), std::sin(angle) } * radius_px);
        }
        dashed_path(points, true, color, width_logical, on_logical, off_logical);
    }

    void OverlayPainter::tapered_dashes(const std::vector<Vec2>& points, const Color& color, double start_width_logical, double end_width_logical,
        float end_opacity, double on_logical, double off_logical) const
    {
        auto mesh = std::make_shared<render::IndexedMesh>();
        append_dashes(*mesh, points, false, { px(on_logical), px(off_logical), px(start_width_logical), px(end_width_logical), color, end_opacity });
        list_->add_indexed_mesh(std::move(mesh));
    }

    void OverlayPainter::halo(const Vec2& centre, double radius_px, const Color& color) const
    {
        list_->add_circle_fill(centre, static_cast<float>(radius_px), color);
    }

    void OverlayPainter::knob(const Vec2& centre, HandleState state, const Color& ring) const
    {
        const auto radius = px(state == HandleState::normal ? overlay::knob_radius : overlay::knob_radius + 1.0);
        if (state != HandleState::normal)
            halo(centre, radius + px(5.0), ring.with_alpha(0.20f));
        list_->add_circle_fill(centre + Vec2 { 0.0, px(0.75) }, static_cast<float>(radius + px(1.0)), palette_.shadow);
        list_->add_circle_fill(centre, static_cast<float>(radius), state == HandleState::active ? ring : palette_.surface);
        list_->add_circle_outline(centre, static_cast<float>(radius), ring, width(overlay::standard));
    }

    void OverlayPainter::grip(const Vec2& centre, HandleState state, const Color& fill) const
    {
        const auto radius = px(state == HandleState::normal ? overlay::rotation_grip_radius : overlay::rotation_grip_radius + 1.0);
        if (state != HandleState::normal)
            halo(centre, radius + px(5.0), fill.with_alpha(0.20f));
        list_->add_circle_fill(centre + Vec2 { 0.0, px(0.75) }, static_cast<float>(radius + px(1.5)), palette_.shadow);
        list_->add_circle_fill(centre, static_cast<float>(radius + px(1.25)), palette_.surface);
        list_->add_circle_fill(centre, static_cast<float>(radius), fill);
    }

    void OverlayPainter::square_handle(const Vec2& centre, HandleState state, bool selected) const
    {
        const auto half = px(state == HandleState::normal ? overlay::vertex_half_size : overlay::vertex_half_size + 1.0);
        const Vec2 extent { half, half };
        if (state != HandleState::normal)
        {
            const Vec2 glow { half + px(4.0), half + px(4.0) };
            list_->add_rounded_rectangle_fill(centre - glow, centre + glow, width(3.0), palette_.accent.with_alpha(0.20f));
        }
        const Vec2 drop { 0.0, px(0.75) };
        const Vec2 shadow_extent { half + px(1.0), half + px(1.0) };
        list_->add_rectangle_fill(centre - shadow_extent + drop, centre + shadow_extent + drop, palette_.shadow);
        list_->add_rectangle_fill(centre - extent, centre + extent, selected || state == HandleState::active ? palette_.accent : palette_.surface);
        list_->add_rectangle_outline(centre - extent, centre + extent, palette_.accent, width(overlay::standard));
    }

    void OverlayPainter::reticle(const Vec2& centre, const Color& color) const
    {
        const auto radius = px(6.0);
        list_->add_circle_outline(centre, static_cast<float>(radius), color, width(overlay::standard));
        for (const auto& direction : { Vec2 { 1.0, 0.0 }, Vec2 { -1.0, 0.0 }, Vec2 { 0.0, 1.0 }, Vec2 { 0.0, -1.0 } })
            list_->add_line(centre + direction * (radius - px(2.5)), centre + direction * (radius + px(3.5)), color, width(overlay::standard));
        list_->add_circle_fill(centre, width(1.25), color);
    }

    double OverlayPainter::chip_text_width(std::string_view text) const
    {
        const auto font_px = std::round(overlay::label_text_scale * scale_ * 14.0);
        double em = 0.0;
        for (const auto character : text)
            em += glyph_em(static_cast<unsigned char>(character));
        return em * font_px;
    }

    double OverlayPainter::chip_height() const
    {
        const auto text_scale = overlay::label_text_scale * scale_;
        return std::round(text_scale * 14.0) * 0.73 + 2.0 * 4.5 * text_scale;
    }

    void OverlayPainter::chip(const Vec2& anchor, std::string_view text, ChipAlign align, const Color& text_color) const
    {
        if (text.empty() || !math::is_finite(anchor))
            return;
        const auto text_scale = overlay::label_text_scale * scale_;
        const auto font_px = std::round(text_scale * 14.0);
        const auto text_width = chip_text_width(text);
        const auto left = align == ChipAlign::left ? anchor.x : align == ChipAlign::centre ? anchor.x - text_width * 0.5
                                                                                           : anchor.x - text_width;
        // Inter's cap height is centred about 0.61 em below the top of its line box. Whole pixel
        // origins keep the glyph atlas sampled without blur.
        const Vec2 origin { std::round(left), std::round(anchor.y - font_px * 0.61) };
        list_->add_text_label(origin, text, text_color, palette_.plate, static_cast<float>(text_scale), 4.5f);
    }

    Vec2 gravity_compass_centre(const render::Camera2D& camera, double scale)
    {
        const auto radius = overlay::compass_radius * scale;
        const auto& focus = camera.focus_rect();
        if (focus.empty())
            return { static_cast<double>(camera.viewport().width) - 24.0 * scale - radius, 24.0 * scale + radius };
        return { focus.left + focus.width - radius, focus.top + radius };
    }

    Vec2 velocity_handle_tip(const render::Camera2D& camera, const Vec2& centre_m, const Vec2& velocity_m_s, double pixels_per_m_s)
    {
        return camera.world_to_screen(centre_m) + Vec2 { velocity_m_s.x * pixels_per_m_s, -velocity_m_s.y * pixels_per_m_s };
    }

    Vec2 velocity_from_handle(const render::Camera2D& camera, const Vec2& centre_m, const Vec2& pointer_px, double pixels_per_m_s)
    {
        const auto offset = pointer_px - camera.world_to_screen(centre_m);
        const auto factor = std::max(1.0e-9, pixels_per_m_s);
        return { offset.x / factor, -offset.y / factor };
    }

    Vec2 velocity_knob_position(const render::Camera2D& camera, const Vec2& centre_m, const Vec2& velocity_m_s, double pixels_per_m_s, double scale)
    {
        const auto centre = camera.world_to_screen(centre_m);
        const auto tip = velocity_handle_tip(camera, centre_m, velocity_m_s, pixels_per_m_s);
        const auto park = overlay::velocity_knob_park_distance * std::clamp(std::isfinite(scale) ? scale : 1.0, 0.25, 8.0);
        const auto shaft = tip - centre;
        const auto length = math::length(shaft);
        if (!(length < park))
            return tip;
        return centre + (length > 1.0e-6 ? shaft / length : Vec2 { 1.0, 0.0 }) * park;
    }

    double rotation_ring_radius(const render::Camera2D& camera, const physics::RigidBody& body, double scale)
    {
        const auto bounds = body.compute_bounds(math::Transform2 {});
        const auto extent = bounds.is_empty() ? 0.0 : std::max(bounds.extents().x, bounds.extents().y);
        return std::max(overlay::rotation_ring_minimum_radius * scale, camera.world_to_screen_length(extent * 0.65));
    }

    Vec2 rotation_knob_position(const Vec2& centre_px, double radius_px, double orientation_rad)
    {
        return centre_px + Vec2 { -std::sin(orientation_rad), -std::cos(orientation_rad) } * radius_px;
    }

    double orientation_from_knob(const Vec2& centre_px, const Vec2& pointer_px)
    {
        const auto offset = pointer_px - centre_px;
        return math::wrap_angle(std::atan2(-offset.y, offset.x) - math::half_pi);
    }

    void add_spotlight(render::DrawList& list, const render::ViewportSize& viewport, const Vec2& centre_px, double clear_radius_px, double feather_px, const Color& shade)
    {
        if (!math::is_finite(centre_px) || !std::isfinite(clear_radius_px) || !std::isfinite(feather_px) || clear_radius_px <= 0.0)
            return;
        const auto width = static_cast<double>(viewport.width), height = static_cast<double>(viewport.height);
        double farthest = 0.0;
        for (const auto& corner : { Vec2 { 0.0, 0.0 }, Vec2 { width, 0.0 }, Vec2 { 0.0, height }, Vec2 { width, height } })
            farthest = std::max(farthest, math::length(corner - centre_px));
        const auto feather = std::max(0.0, feather_px);
        const auto edge = clear_radius_px + feather;
        const auto segments = static_cast<int>(std::clamp(std::ceil(math::two_pi * edge / 6.0), 64.0, 256.0));
        // The outer ring is a polygon, so its radius allows for the chord sag between vertices.
        const auto outer = farthest / std::cos(math::pi / segments) + 2.0;
        // Smoothstep samples across the feather avoid a visible ring where a linear ramp meets
        // the flat shade.
        std::array<double, 5> radii { clear_radius_px, clear_radius_px + feather / 3.0, clear_radius_px + feather * 2.0 / 3.0, edge, outer };
        constexpr std::array<float, 5> coverage { 0.0f, 0.259f, 0.741f, 1.0f, 1.0f };
        const auto rings = outer > edge + 1.0 ? radii.size() : radii.size() - 1;
        auto mesh = std::make_shared<render::IndexedMesh>();
        mesh->vertices.reserve(rings * static_cast<std::size_t>(segments));
        mesh->indices.reserve((rings - 1) * static_cast<std::size_t>(segments) * 6);
        for (std::size_t ring = 0; ring < rings; ++ring)
        {
            const auto colour = premultiplied(shade.with_alpha(shade.alpha * coverage[ring]));
            for (int index = 0; index < segments; ++index)
            {
                const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(segments);
                mesh->vertices.push_back({ centre_px + Vec2 { std::cos(angle), std::sin(angle) } * radii[ring], {}, colour });
            }
        }
        for (std::size_t ring = 0; ring + 1 < rings; ++ring)
            for (int index = 0; index < segments; ++index)
            {
                const auto inner = static_cast<int>(ring) * segments;
                const auto next = (index + 1) % segments;
                const auto a = inner + index, b = inner + next, c = inner + segments + next, d = inner + segments + index;
                for (const auto value : { a, b, c, a, c, d })
                    mesh->indices.push_back(value);
            }
        list.add_indexed_mesh(std::move(mesh));
    }
}
