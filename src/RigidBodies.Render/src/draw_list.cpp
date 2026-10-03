#include <rigidbodies/render/draw_list.hpp>

#include <rigidbodies/render/draw_compiler.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace rigidbodies::render
{

    void DrawList::push_command(DrawCommand command)
    {
        command.layer = layer_;
        if (!clips_.empty())
            command.clip = clips_.back();
        commands_.push_back(std::move(command));
    }

    void DrawList::set_layer(int value)
    {
        layer_ = value;
    }
    int DrawList::layer() const
    {
        return layer_;
    }

    void DrawList::push_clip(MeshClip clip)
    {
        clip.width = std::max(0, clip.width);
        clip.height = std::max(0, clip.height);
        if (!clips_.empty())
        {
            const auto& parent = clips_.back();
            const auto right = std::min(static_cast<long long>(clip.x) + clip.width, static_cast<long long>(parent.x) + parent.width);
            const auto bottom = std::min(static_cast<long long>(clip.y) + clip.height, static_cast<long long>(parent.y) + parent.height);
            clip.x = std::max(clip.x, parent.x);
            clip.y = std::max(clip.y, parent.y);
            clip.width = static_cast<int>(std::max(0LL, right - clip.x));
            clip.height = static_cast<int>(std::max(0LL, bottom - clip.y));
        }
        clips_.push_back(clip);
    }

    void DrawList::pop_clip()
    {
        if (!clips_.empty())
            clips_.pop_back();
    }

    void DrawList::add_instanced_mesh(const std::shared_ptr<const IndexedMesh>& mesh, std::vector<MeshInstance> instances)
    {
        if (!mesh || mesh->vertices.empty() || mesh->indices.empty() || instances.empty())
            return;
        auto copy = std::make_shared<IndexedMesh>(*mesh);
        copy->instances = std::move(instances);
        add_indexed_mesh(std::move(copy));
        if (!commands_.empty())
            commands_.back().kind = DrawCommandKind::instanced_mesh;
    }

    void DrawList::add_path(const std::vector<Vec2>& points, const Color& color, float thickness, bool closed, StrokeJoin join, StrokeCap cap)
    {
        if (points.size() < 2)
            return;
        add_polyline(points, color, thickness, closed);
        commands_.back().join = join;
        commands_.back().cap = cap;
        commands_.back().closed = closed;
    }

    void DrawList::add_rounded_rectangle_fill(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color)
    {
        add_rectangle_fill(minimum, maximum, color);
        commands_.back().kind = DrawCommandKind::rounded_rectangle_fill;
        commands_.back().radius = radius;
    }

    void DrawList::add_rounded_rectangle_outline(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color, float thickness)
    {
        add_rectangle_outline(minimum, maximum, color, thickness);
        commands_.back().kind = DrawCommandKind::rounded_rectangle_outline;
        commands_.back().radius = radius;
    }

    void DrawList::add_gradient_polygon_fill(const std::vector<Vec2>& points, const Color& start, const Color& end, const Vec2& from, const Vec2& to)
    {
        if (points.size() < 3)
            return;
        add_polygon_fill(points, start);
        auto& command = commands_.back();
        command.kind = DrawCommandKind::gradient_polygon_fill;
        command.gradient_end = end;
        command.gradient_from = from;
        command.gradient_to = to;
    }

    void DrawList::add_gradient_circle_fill(const Vec2& center, float radius, const Color& start, const Color& end, const Vec2& from, const Vec2& to)
    {
        add_circle_fill(center, radius, start);
        auto& command = commands_.back();
        command.kind = DrawCommandKind::gradient_circle_fill;
        command.gradient_end = end;
        command.gradient_from = from;
        command.gradient_to = to;
    }

    void DrawList::add_shadow(const std::vector<Vec2>& points, const Color& color, float blur_radius, const Vec2& offset)
    {
        if (points.size() < 3)
            return;
        add_polygon_fill(points, color);
        auto& command = commands_.back();
        command.kind = DrawCommandKind::shadow;
        command.blur_radius = blur_radius;
        command.shadow_offset = offset;
    }

    void DrawList::add_rounded_rectangle_shadow(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color, float blur_radius, const Vec2& offset)
    {
        add_rounded_rectangle_fill(minimum, maximum, radius, color);
        auto& command = commands_.back();
        command.kind = DrawCommandKind::rounded_rectangle_shadow;
        command.blur_radius = blur_radius;
        command.shadow_offset = offset;
    }

    void DrawList::add_textured_quad(std::shared_ptr<const TexturePixels> texture, const Vec2& minimum, const Vec2& maximum, const Color& tint, const Vec2& uv_min, const Vec2& uv_max)
    {
        if (!texture)
            return;
        add_rectangle_fill(minimum, maximum, tint);
        auto& command = commands_.back();
        command.kind = DrawCommandKind::textured_quad;
        command.texture = std::move(texture);
        command.uv_min = uv_min;
        command.uv_max = uv_max;
    }

    void DrawList::clear()
    {
        // The buffers keep their capacity, so a steady frame allocates nothing after the first.
        commands_.clear();
        vertices_.clear();
        text_buffer_.clear();
        clips_.clear();
        layer_ = 0;
    }

    void DrawList::add_indexed_mesh(std::shared_ptr<const IndexedMesh> mesh)
    {
        if (!mesh || mesh->vertices.empty() || mesh->indices.empty())
            return;
        DrawCommand command;
        command.kind = DrawCommandKind::indexed_mesh;
        command.mesh = std::move(mesh);
        push_command(std::move(command));
    }

    bool DrawList::is_empty() const
    {
        return commands_.empty();
    }

    const std::vector<DrawCommand>& DrawList::commands() const
    {
        return commands_;
    }

    const std::vector<Vec2>& DrawList::vertices() const
    {
        return vertices_;
    }

    const std::string& DrawList::text_buffer() const
    {
        return text_buffer_;
    }

    std::size_t DrawList::push_vertices(const Vec2* points, std::size_t count)
    {
        const auto offset = vertices_.size();
        vertices_.insert(vertices_.end(), points, points + count);
        return offset;
    }

    void DrawList::add_line(const Vec2& from, const Vec2& to, const Color& color, float thickness)
    {
        const std::array<Vec2, 2> points { from, to };
        DrawCommand command;
        command.kind = DrawCommandKind::line;
        command.color = color;
        command.thickness = thickness;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_polyline(const std::vector<Vec2>& points, const Color& color, float thickness, bool closed)
    {
        if (points.size() < 2)
        {
            return;
        }

        DrawCommand command;
        command.kind = DrawCommandKind::polyline;
        command.color = color;
        command.thickness = thickness;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        if (closed)
        {
            vertices_.push_back(points.front());
            ++command.vertex_count;
        }
        push_command(std::move(command));
    }

    void DrawList::add_polygon_fill(const std::vector<Vec2>& points, const Color& color)
    {
        if (points.size() < 3)
        {
            return;
        }

        DrawCommand command;
        command.kind = DrawCommandKind::polygon_fill;
        command.color = color;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_polygon_outline(const std::vector<Vec2>& points, const Color& color, float thickness)
    {
        if (points.size() < 2)
        {
            return;
        }

        DrawCommand command;
        command.kind = DrawCommandKind::polygon_outline;
        command.color = color;
        command.thickness = thickness;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_circle_fill(const Vec2& center, float radius, const Color& color)
    {
        DrawCommand command;
        command.kind = DrawCommandKind::circle_fill;
        command.color = color;
        command.radius = radius;
        command.vertex_offset = push_vertices(&center, 1);
        command.vertex_count = 1;
        push_command(std::move(command));
    }

    void DrawList::add_circle_outline(const Vec2& center, float radius, const Color& color, float thickness)
    {
        DrawCommand command;
        command.kind = DrawCommandKind::circle_outline;
        command.color = color;
        command.radius = radius;
        command.thickness = thickness;
        command.vertex_offset = push_vertices(&center, 1);
        command.vertex_count = 1;
        push_command(std::move(command));
    }

    void DrawList::add_rectangle_fill(const Vec2& minimum, const Vec2& maximum, const Color& color)
    {
        const std::array<Vec2, 2> points { minimum, maximum };
        DrawCommand command;
        command.kind = DrawCommandKind::rectangle_fill;
        command.color = color;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_rectangle_outline(const Vec2& minimum, const Vec2& maximum, const Color& color, float thickness)
    {
        const std::array<Vec2, 2> points { minimum, maximum };
        DrawCommand command;
        command.kind = DrawCommandKind::rectangle_outline;
        command.color = color;
        command.thickness = thickness;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_text(const Vec2& position, std::string_view text, const Color& color, float scale)
    {
        if (text.empty())
        {
            return;
        }

        DrawCommand command;
        command.kind = DrawCommandKind::text;
        command.color = color;
        command.text_scale = scale;
        command.vertex_offset = push_vertices(&position, 1);
        command.vertex_count = 1;
        command.text_offset = text_buffer_.size();
        command.text_length = text.size();
        text_buffer_.append(text);
        push_command(std::move(command));
    }

    void DrawList::add_text_label(const Vec2& position, std::string_view text, const Color& color, const Color& background, float scale, float padding)
    {
        if (text.empty())
            return;
        add_text(position, text, color, scale);
        commands_.back().text_background = background;
        commands_.back().text_padding = padding;
    }

    void DrawList::add_arrow(const Vec2& from, const Vec2& to, const Color& color, float thickness, float head_length)
    {
        add_arrow(from, to, color, thickness, head_length, ArrowHead::filled);
    }

    void DrawList::add_arrow(const Vec2& from, const Vec2& to, const Color& color, float thickness, float head_length, ArrowHead head_style, float head_width, float shaft_dash)
    {
        const auto shaft = to - from;
        const auto length = math::length(shaft);
        if (length <= math::geometric_epsilon || !std::isfinite(length))
        {
            add_line(from, to, color, thickness, StrokeCap::butt);
            return;
        }

        // The head takes at most this share of a short vector, so a short arrow keeps a head that
        // shows its direction rather than turning into a blob of arrowhead or a bare sliver.
        const auto direction = shaft / length;
        const auto requested = std::max(0.0, std::isfinite(head_length) ? static_cast<double>(head_length) : 10.0);
        const auto head = std::min(requested, length * 0.45);
        const auto ratio = head_width > 0.0f && std::isfinite(head_width) && requested > 0.0 ? static_cast<double>(head_width) / requested : 0.42;
        const auto side = math::perpendicular(direction) * (head * ratio);
        const auto base = to - direction * head;
        // A shallow notch reads as a drawn arrow rather than a triangle stamped on a line.
        const auto notch = to - direction * (head * 0.78);
        const auto add_shaft = [&](const Vec2& end)
        {
            add_line(from, end, color, thickness, StrokeCap::butt);
            if (shaft_dash > 0.0f && std::isfinite(shaft_dash))
            {
                commands_.back().dash_length = shaft_dash;
                commands_.back().gap_length = shaft_dash;
            }
        };
        switch (head_style)
        {
        case ArrowHead::open:
        {
            add_shaft(to - direction * std::min(head * 0.2, static_cast<double>(thickness) * 0.5));
            add_path({ base + side, to, base - side }, color, thickness, false, StrokeJoin::miter, StrokeCap::round);
            return;
        }
        case ArrowHead::double_filled:
        {
            add_shaft(notch - direction * (head * 0.6));
            const auto back = direction * (head * 0.6);
            add_polygon_fill({ to, base + side, notch, base - side }, color);
            add_polygon_fill({ to - back, base - back + side, notch - back, base - back - side }, color);
            return;
        }
        case ArrowHead::hollow:
            add_shaft(base);
            add_polygon_outline({ to, base + side, base - side }, color, thickness);
            return;
        case ArrowHead::filled:
            break;
        }
        add_shaft(notch + direction * std::min(head * 0.1, 0.5));
        add_polygon_fill({ to, base + side, notch, base - side }, color);
    }

    void DrawList::add_line(const Vec2& from, const Vec2& to, const Color& color, float thickness, StrokeCap cap)
    {
        add_line(from, to, color, thickness);
        commands_.back().cap = cap;
    }

    void DrawList::add_dashed_line(const Vec2& from, const Vec2& to, const Color& color, float thickness, float dash_length, float gap_length)
    {
        add_line(from, to, color, thickness, StrokeCap::butt);
        commands_.back().dash_length = dash_length;
        commands_.back().gap_length = gap_length;
    }

    void DrawList::add_dashed_polyline(const std::vector<Vec2>& points, const Color& color, float thickness, float dash_length, float gap_length, bool closed)
    {
        if (points.size() < 2)
            return;
        add_path(points, color, thickness, closed, StrokeJoin::round, StrokeCap::butt);
        commands_.back().dash_length = dash_length;
        commands_.back().gap_length = gap_length;
    }

    void DrawList::add_tapered_polyline(const std::vector<Vec2>& points, const Color& start_color, const Color& end_color, float start_thickness, float end_thickness)
    {
        if (points.size() < 2)
            return;
        DrawCommand command;
        command.kind = DrawCommandKind::tapered_polyline;
        command.color = start_color;
        command.gradient_end = end_color;
        command.thickness = start_thickness;
        command.radius = end_thickness;
        command.cap = StrokeCap::butt;
        command.vertex_offset = push_vertices(points.data(), points.size());
        command.vertex_count = points.size();
        push_command(std::move(command));
    }

    void DrawList::add_circle_shadow(const Vec2& center, float radius, const Color& color, float blur_radius, const Vec2& offset)
    {
        if (!std::isfinite(radius) || radius <= 0.0f || !math::is_finite(center))
            return;
        const auto count = std::clamp(curve_segment_count(radius, 0.5f), 8, 96);
        thread_local std::vector<Vec2> points;
        points.clear();
        for (int index = 0; index < count; ++index)
        {
            const auto angle = math::two_pi * static_cast<double>(index) / static_cast<double>(count);
            points.push_back(center + Vec2 { std::cos(angle), std::sin(angle) } * static_cast<double>(radius));
        }
        add_shadow(points, color, blur_radius, offset);
    }

    void DrawList::add_cross(const Vec2& center, float radius, const Color& color, float thickness)
    {
        const auto extent = static_cast<double>(radius);
        add_line({ center.x - extent, center.y }, { center.x + extent, center.y }, color, thickness);
        add_line({ center.x, center.y - extent }, { center.x, center.y + extent }, color, thickness);
    }

} // namespace rigidbodies::render
