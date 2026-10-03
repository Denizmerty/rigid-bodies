#include <rigidbodies/app/shape_editor.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>

#include <rigidbodies/core/text_format.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace rigidbodies::app
{
    using math::Vec2;
    using physics::OutlineEdgeKind;
    using physics::OutlineContinuity;

    namespace
    {
        render::Color premultiplied(const render::Color& color)
        {
            const auto alpha = std::clamp(color.alpha, 0.0f, 1.0f);
            return { color.red * alpha, color.green * alpha, color.blue * alpha, alpha };
        }

        // The panel shows the full diagnostic; the stage only needs a glanceable reason.
        std::string_view short_reason(physics::OutlineDiagnosticCode code)
        {
            using Code = physics::OutlineDiagnosticCode;
            switch (code)
            {
            case Code::self_intersection:
                return "Edges cross";
            case Code::degenerate_edge:
                return "Zero-length edge";
            case Code::insufficient_area:
                return "Area too small";
            case Code::insufficient_vertices:
                return "Needs three distinct points";
            case Code::vertex_budget:
                return "Too many vertices";
            case Code::subdivision_budget:
                return "Curve too detailed";
            case Code::decomposition_failed:
                return "Features too small";
            case Code::non_finite:
                return "Out of range";
            case Code::invalid_options:
                return "Invalid tolerances";
            case Code::open_outline:
                return "Outline is open";
            }
            return "Invalid outline";
        }

        // Proper crossings between non-adjacent edges of a closed screen-space path.
        void find_crossings(const std::vector<Vec2>& path, std::vector<Vec2>& crossings, std::size_t limit)
        {
            const auto count = path.size();
            for (std::size_t first = 0; first < count && crossings.size() < limit; ++first)
                for (std::size_t second = first + 2; second < count && crossings.size() < limit; ++second)
                {
                    if (first == 0 && second + 1 == count)
                        continue;
                    const auto a = path[first], c = path[second];
                    const auto r = path[(first + 1) % count] - a, s = path[(second + 1) % count] - c;
                    const auto denominator = math::cross(r, s);
                    if (std::abs(denominator) < 1.0e-12)
                        continue;
                    const auto t = math::cross(c - a, s) / denominator, u = math::cross(c - a, r) / denominator;
                    if (t > 0.0 && t < 1.0 && u > 0.0 && u < 1.0)
                        crossings.push_back(a + r * t);
                }
        }
    }

    void ShapeEditor::begin(physics::Outline outline, math::Transform2 placement,
        std::optional<physics::ShapeAuthoringOptions> options)
    {
        outline_ = std::move(outline);
        placement_ = placement;
        if (options)
            options_ = *options;
        selected_.reset();
        drag_ = DragKind::none;
        active_ = true;
        pointer_known_ = false;
        pointer_snap_ = SnapKind::none;
        snap_origin_world_.reset();
        edit_message_.clear();
        cache_.clear();
    }

    void ShapeEditor::end()
    {
        active_ = false;
        drag_ = DragKind::none;
        selected_.reset();
        outline_ = {};
        pointer_known_ = false;
        pointer_snap_ = SnapKind::none;
        snap_origin_world_.reset();
        cache_.clear();
        edit_message_.clear();
    }
    bool ShapeEditor::active() const
    {
        return active_;
    }
    bool ShapeEditor::has_pointer_capture() const
    {
        return drag_ != DragKind::none;
    }
    const physics::Outline& ShapeEditor::outline() const
    {
        return outline_;
    }
    const physics::ShapeAuthoringOptions& ShapeEditor::options() const
    {
        return options_;
    }
    const math::Transform2& ShapeEditor::placement() const
    {
        return placement_;
    }
    std::optional<std::size_t> ShapeEditor::selected_node() const
    {
        return selected_;
    }
    physics::ShapeBuildResult ShapeEditor::build() const
    {
        return cache_.build(outline_, options_);
    }
    bool ShapeEditor::snap_grid() const
    {
        return snap_grid_;
    }
    bool ShapeEditor::snap_vertices() const
    {
        return snap_vertices_;
    }
    bool ShapeEditor::snap_angles() const
    {
        return snap_angles_;
    }
    double ShapeEditor::grid_spacing_m() const
    {
        return grid_spacing_m_;
    }

    std::string ShapeEditor::diagnostic() const
    {
        if (!edit_message_.empty())
            return edit_message_;
        if (!outline_.closed)
            return outline_.nodes.size() < 3 ? "Click at least three points, then close the outline." : "Click the first node or press Enter to close.";
        const auto result = build();
        if (!result.diagnostics.empty())
            return result.diagnostics.front().message;
        return result.succeeded() ? "Ready to apply. Drag points or handles to adjust the outline." : "The outline could not be built.";
    }

    Vec2 ShapeEditor::curve_point(std::size_t edge, double t) const
    {
        const auto& a = outline_.nodes[edge];
        const auto& b = outline_.nodes[(edge + 1) % outline_.nodes.size()];
        if (a.outgoing_edge == OutlineEdgeKind::line)
            return math::lerp(a.position_m, b.position_m, t);
        const auto u = 1.0 - t;
        return a.position_m * (u * u * u) + (a.position_m + a.outgoing_handle_m) * (3.0 * u * u * t) + (b.position_m + b.incoming_handle_m) * (3.0 * u * t * t) + b.position_m * (t * t * t);
    }

    bool ShapeEditor::incoming_visible(std::size_t node) const
    {
        return !outline_.nodes.empty() && (outline_.closed || node > 0) && outline_.nodes[(node + outline_.nodes.size() - 1) % outline_.nodes.size()].outgoing_edge == OutlineEdgeKind::cubic;
    }
    bool ShapeEditor::outgoing_visible(std::size_t node) const
    {
        return (outline_.closed || node + 1 < outline_.nodes.size()) && outline_.nodes[node].outgoing_edge == OutlineEdgeKind::cubic;
    }

    bool ShapeEditor::edge_exists(std::size_t edge) const
    {
        return edge < outline_.nodes.size() && (outline_.closed || edge + 1 < outline_.nodes.size());
    }

    ShapeEditor::SnapResult ShapeEditor::snap(const Vec2& world, const render::Camera2D& camera,
        const std::vector<Vec2>& existing_vertices, std::optional<std::size_t> moving_node, double scale, bool temporary_angle) const
    {
        SnapResult result { world, SnapKind::none, std::nullopt };
        if (snap_grid_)
        {
            result.point = { std::round(world.x / grid_spacing_m_) * grid_spacing_m_, std::round(world.y / grid_spacing_m_) * grid_spacing_m_ };
            result.kind = SnapKind::grid;
        }
        std::optional<Vec2> origin;
        if (moving_node && (*moving_node > 0 || (outline_.closed && !outline_.nodes.empty())))
            origin = math::transform_point(placement_, outline_.nodes[(*moving_node + outline_.nodes.size() - 1) % outline_.nodes.size()].position_m);
        else if (!moving_node && !outline_.nodes.empty())
            origin = math::transform_point(placement_, outline_.nodes.back().position_m);
        if ((snap_angles_ || temporary_angle) && origin)
        {
            const auto delta = result.point - *origin;
            const auto angle_step = math::pi / 12.0;
            const auto angle = std::round(std::atan2(delta.y, delta.x) / angle_step) * angle_step;
            result.point = *origin + Vec2 { std::cos(angle), std::sin(angle) } * math::length(delta);
            result.kind = SnapKind::angle;
            result.origin = origin;
        }
        if (snap_vertices_)
        {
            const auto radius = overlay::snap_radius * std::max(0.01, scale);
            auto best_squared = radius * radius;
            const auto screen = camera.world_to_screen(world);
            const auto consider = [&](const Vec2& vertex)
            {
                const auto distance = math::length_squared(camera.world_to_screen(vertex) - screen);
                if (distance < best_squared)
                {
                    best_squared = distance;
                    result.point = vertex;
                    result.kind = SnapKind::vertex;
                }
            };
            for (const auto& vertex : existing_vertices)
                consider(vertex);
            for (std::size_t index = 0; index < outline_.nodes.size(); ++index)
                if (!moving_node || index != *moving_node)
                    consider(math::transform_point(placement_, outline_.nodes[index].position_m));
        }
        return result;
    }

    bool ShapeEditor::handle_event(const ui::UiEvent& event, const render::Camera2D& camera,
        const std::vector<Vec2>& existing_vertices, bool interface_consumed)
    {
        if (!active_)
            return false;
        if (event.kind == ui::UiEventKind::pointer_up && event.button == ui::PointerButton::primary)
        {
            const auto captured = has_pointer_capture();
            if (close_pending_ && drag_threshold_pending_ && outline_.nodes.size() >= 3)
                outline_.closed = true;
            drag_ = DragKind::none;
            smooth_node_pending_ = false;
            drag_threshold_pending_ = false;
            close_pending_ = false;
            return captured || !interface_consumed;
        }
        const auto scale = std::max(0.01, event.logical_pixel_scale);
        if (event.kind == ui::UiEventKind::pointer_move)
        {
            if (interface_consumed && !has_pointer_capture())
            {
                pointer_known_ = false;
                return false;
            }
            pointer_shift_ = event.modifiers.shift;
            // Hovering previews the point a click would add, so it snaps exactly as a press does;
            // only a node drag excludes the moving node and measures angles from its predecessor.
            const auto moving = drag_ == DragKind::node ? selected_ : std::nullopt;
            track_pointer(snap(camera.screen_to_world(event.pointer_px), camera, existing_vertices, moving, scale, event.modifiers.shift), event.pointer_px);
            if (selected_ && has_pointer_capture())
            {
                if (drag_threshold_pending_)
                {
                    if (math::length(event.pointer_px - press_screen_px_) < 4.0 * std::max(1.0, event.logical_pixel_scale))
                        return true;
                    drag_threshold_pending_ = false;
                    close_pending_ = false;
                }
                const auto local = math::inverse_transform_point(placement_, pointer_world_);
                if (drag_ == DragKind::node)
                    physics::move_outline_node(outline_, *selected_, local);
                else
                {
                    // Tangents are precise local vectors: positional grid/vertex snaps would
                    // collapse short handles, so only node positions use those snap modes.
                    const auto point = math::inverse_transform_point(placement_, camera.screen_to_world(event.pointer_px));
                    physics::move_outline_handle(outline_, *selected_, drag_ == DragKind::incoming, point - outline_.nodes[*selected_].position_m);
                }
                edit_message_.clear();
            }
            return true;
        }
        if (interface_consumed)
            return false;
        if (event.kind == ui::UiEventKind::key_down)
        {
            ui::UiCommand command;
            if (event.key == ui::UiKey::enter)
                command.kind = ui::UiCommandKind::close_shape_outline;
            else if (event.key == ui::UiKey::delete_key || event.key == ui::UiKey::backspace)
                command.kind = ui::UiCommandKind::remove_shape_node;
            else
                return false;
            return apply(command);
        }
        if (event.kind != ui::UiEventKind::pointer_down || event.button != ui::PointerButton::primary)
            return false;
        edit_message_.clear();
        press_screen_px_ = event.pointer_px;
        const auto screen_node = [&](std::size_t index)
        {
            return camera.world_to_screen(math::transform_point(placement_, outline_.nodes[index].position_m));
        };
        if (selected_)
        {
            const auto& node = outline_.nodes[*selected_];
            for (const auto incoming : { true, false })
            {
                if (!(incoming ? incoming_visible(*selected_) : outgoing_visible(*selected_)))
                    continue;
                const auto handle = node.position_m + (incoming ? node.incoming_handle_m : node.outgoing_handle_m);
                const auto radius = overlay::handle_hit_radius * scale;
                if (math::length_squared(camera.world_to_screen(math::transform_point(placement_, handle)) - event.pointer_px) <= radius * radius)
                {
                    drag_ = incoming ? DragKind::incoming : DragKind::outgoing;
                    drag_threshold_pending_ = true;
                    return true;
                }
            }
        }
        for (std::size_t index = 0; index < outline_.nodes.size(); ++index)
        {
            const auto radius = overlay::handle_hit_radius * scale;
            if (math::length_squared(screen_node(index) - event.pointer_px) <= radius * radius)
            {
                selected_ = index;
                if (event.modifiers.alt)
                {
                    const auto current = outline_.nodes[index].continuity;
                    physics::set_outline_continuity(outline_, index, current == OutlineContinuity::corner ? OutlineContinuity::aligned : OutlineContinuity::corner);
                    drag_ = DragKind::none;
                    return true;
                }
                drag_ = DragKind::node;
                drag_threshold_pending_ = true;
                close_pending_ = !outline_.closed && index == 0 && outline_.nodes.size() >= 3;
                return true;
            }
        }
        if (!outline_.closed)
        {
            if (outline_.nodes.size() >= 4096)
            {
                edit_message_ = "Node limit reached. Remove nodes before adding more.";
                return true;
            }
            track_pointer(snap(camera.screen_to_world(event.pointer_px), camera, existing_vertices, std::nullopt, scale, event.modifiers.shift), event.pointer_px);
            physics::OutlineNode node;
            node.position_m = math::inverse_transform_point(placement_, pointer_world_);
            outline_.nodes.push_back(node);
            selected_ = outline_.nodes.size() - 1;
            drag_ = DragKind::outgoing;
            smooth_node_pending_ = true;
            drag_threshold_pending_ = true;
            return true;
        }
        // Clicking an edge selects its starting node, making insertion and line/cubic changes
        // available even when the edge's endpoint lies outside the visible portion of the scene.
        auto best_squared = overlay::edge_hit_distance * overlay::edge_hit_distance * scale * scale;
        auto best_parameter = 0.5;
        selected_.reset();
        for (std::size_t edge = 0; edge < outline_.nodes.size(); ++edge)
        {
            auto previous = screen_node(edge);
            for (int sample = 1; sample <= 32; ++sample)
            {
                const auto next = camera.world_to_screen(math::transform_point(placement_, curve_point(edge, sample / 32.0)));
                const auto delta = next - previous;
                const auto denominator = math::length_squared(delta);
                const auto t = denominator > 0.0 ? math::clamp(math::dot(event.pointer_px - previous, delta) / denominator, 0.0, 1.0) : 0.0;
                const auto distance = math::length_squared(event.pointer_px - (previous + delta * t));
                if (distance < best_squared)
                {
                    best_squared = distance;
                    selected_ = edge;
                    best_parameter = (static_cast<double>(sample - 1) + t) / 32.0;
                }
                previous = next;
            }
        }
        if (event.click_count >= 2 && selected_)
        {
            const auto inserted = physics::insert_outline_node(outline_, *selected_, best_parameter);
            if (inserted < outline_.nodes.size())
                selected_ = inserted;
        }
        return true;
    }

    bool ShapeEditor::apply(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        if (!active_)
            return false;
        edit_message_.clear();
        switch (command.kind)
        {
        case K::close_shape_outline:
            if (outline_.nodes.size() >= 3)
                outline_.closed = true;
            else
                edit_message_ = "Add at least three nodes before closing.";
            break;
        case K::insert_shape_node:
            if (selected_ && (outline_.closed || *selected_ + 1 < outline_.nodes.size()))
            {
                const auto inserted = physics::insert_outline_node(outline_, *selected_);
                if (inserted < outline_.nodes.size())
                    selected_ = inserted;
                drag_ = DragKind::none;
            }
            break;
        case K::remove_shape_node:
            drag_ = DragKind::none;
            if (selected_ && physics::remove_outline_node(outline_, *selected_))
            {
                selected_.reset();
                if (outline_.nodes.size() < 3)
                    outline_.closed = false;
            }
            break;
        case K::set_shape_edge:
            if (selected_ && (command.id == "straight" || command.id == "curved"))
                physics::set_outline_edge_kind(outline_, *selected_, command.id == "curved" ? OutlineEdgeKind::cubic : OutlineEdgeKind::line);
            break;
        case K::set_shape_continuity:
            if (selected_)
            {
                if (command.id == "corner")
                    physics::set_outline_continuity(outline_, *selected_, OutlineContinuity::corner);
                else if (command.id == "smooth")
                    physics::set_outline_continuity(outline_, *selected_, OutlineContinuity::aligned);
                else if (command.id == "symmetric")
                    physics::set_outline_continuity(outline_, *selected_, OutlineContinuity::mirrored);
            }
            break;
        case K::set_shape_snap_grid:
            snap_grid_ = command.flag;
            break;
        case K::set_shape_snap_vertices:
            snap_vertices_ = command.flag;
            break;
        case K::set_shape_snap_angles:
            snap_angles_ = command.flag;
            break;
        case K::set_shape_grid_spacing:
            if (std::isfinite(command.value) && command.value >= 1.0e-6 && command.value <= 1.0e6)
                grid_spacing_m_ = command.value;
            else
                edit_message_ = "Grid spacing must be between 0.000001 and 1000000 m.";
            break;
        case K::set_shape_vertex_budget:
            if (std::isfinite(command.value) && command.value >= 3.0 && command.value <= 512.0)
            {
                options_.max_collision_vertices = static_cast<std::size_t>(command.value);
                options_.max_render_vertices = std::max(std::size_t { 512 }, options_.max_collision_vertices * 4);
            }
            else
                edit_message_ = "Collision vertex budget must be between 3 and 512.";
            break;
        case K::set_shape_render_tolerance:
        case K::set_shape_collision_tolerance:
        case K::set_shape_simplification_tolerance:
        case K::set_shape_concavity_tolerance:
        {
            const auto zero_allowed = command.kind == K::set_shape_simplification_tolerance || command.kind == K::set_shape_concavity_tolerance;
            if (!std::isfinite(command.value) || command.value < 0.0 || (!zero_allowed && command.value == 0.0) || command.value > 1.0e3)
            {
                edit_message_ = "Tolerances must be finite and greater than zero. Simplification and concavity tolerances can also be zero.";
                break;
            }
            if (command.kind == K::set_shape_render_tolerance)
                options_.render_tolerance_m = command.value;
            if (command.kind == K::set_shape_collision_tolerance)
                options_.collision_tolerance_m = command.value;
            if (command.kind == K::set_shape_simplification_tolerance)
                options_.simplification_tolerance_m = command.value;
            if (command.kind == K::set_shape_concavity_tolerance)
                options_.concavity_tolerance_m = command.value;
            break;
        }
        default:
            return false;
        }
        return true;
    }

    void ShapeEditor::track_pointer(const SnapResult& snapped, const Vec2& pointer_px)
    {
        pointer_world_ = snapped.point;
        pointer_snap_ = snapped.kind;
        snap_origin_world_ = snapped.origin;
        pointer_screen_px_ = pointer_px;
        pointer_known_ = true;
    }

    void ShapeEditor::draw(const render::Camera2D& camera, const render::Theme& theme, render::DrawList& list, core::DisplayUnits units) const
    {
        if (!active_)
            return;
        const OverlayPainter paint(list, theme, view_scale_);
        const auto& palette = paint.palette();
        const auto project = [&](const Vec2& local)
        {
            return camera.world_to_screen(math::transform_point(placement_, local));
        };
        const auto count = outline_.nodes.size();
        const auto result = build();
        const auto invalid = outline_.closed && !result.succeeded();
        const auto line_color = invalid ? palette.danger : palette.accent;
        const auto closable = !outline_.closed && count >= 3;

        // One mesh for the whole fill: separately feathered triangles would leave visible seams
        // across a translucent surface.
        const auto add_fill = [&](const physics::AuthoredShape& shape, float opacity)
        {
            auto fill = std::make_shared<render::IndexedMesh>();
            const auto tint = premultiplied(palette.accent.with_alpha(opacity));
            fill->vertices.reserve(shape.render_triangles.size() * 3);
            fill->indices.reserve(shape.render_triangles.size() * 3);
            for (const auto& triangle : shape.render_triangles)
                for (const auto& point : triangle)
                {
                    fill->indices.push_back(static_cast<int>(fill->vertices.size()));
                    fill->vertices.push_back({ project(point), {}, tint });
                }
            list.add_indexed_mesh(std::move(fill));
        };
        // An open draft that could be closed previews what Enter would create.
        auto closing_valid = false;
        if (closable)
        {
            closing_preview_.nodes.assign(outline_.nodes.begin(), outline_.nodes.end());
            closing_preview_.closed = true;
            const auto closed = closing_cache_.build(closing_preview_, options_);
            closing_valid = closed.succeeded();
            if (closing_valid)
                add_fill(*closed.shape, 0.06f);
        }

        if (result.shape)
        {
            add_fill(*result.shape, 0.14f);
            std::vector<Vec2> vertices;
            for (const auto& part : result.shape->convex_parts)
            {
                vertices.clear();
                for (const auto& vertex : part)
                    vertices.push_back(project(vertex));
                list.add_polygon_outline(vertices, theme.velocity.with_alpha(0.45f), paint.width(overlay::hairline));
            }
        }

        std::vector<Vec2> path;
        if (result.shape)
        {
            path.reserve(result.shape->render_outline.size());
            for (const auto& vertex : result.shape->render_outline)
                path.push_back(project(vertex));
        }
        else
        {
            // Invalid/open drafts still need a visible editable preview. Valid closed geometry
            // always uses the cached adaptive outline that also generated the filled triangles.
            const auto edge_count = outline_.closed ? count : count == 0 ? 0
                                                                         : count - 1;
            for (std::size_t edge = 0; edge < edge_count; ++edge)
            {
                const auto samples = outline_.nodes[edge].outgoing_edge == OutlineEdgeKind::cubic ? 32 : 1;
                for (int sample = 0; sample < samples; ++sample)
                    path.push_back(project(curve_point(edge, static_cast<double>(sample) / samples)));
            }
            if (!outline_.closed && count > 0)
                path.push_back(project(outline_.nodes.back().position_m));
        }
        if (path.size() >= 2)
        {
            if (invalid)
            {
                list.add_polyline(path, palette.danger.with_alpha(0.16f), paint.width(5.0), true);
                paint.dashed_path(path, true, palette.danger, overlay::standard);
            }
            else
                list.add_polyline(path, palette.accent, paint.width(overlay::standard), outline_.closed);
        }

        const auto edge_points = [&](std::size_t edge)
        {
            std::vector<Vec2> points;
            const auto samples = outline_.nodes[edge].outgoing_edge == OutlineEdgeKind::cubic ? 32 : 1;
            points.reserve(static_cast<std::size_t>(samples) + 1);
            for (int sample = 0; sample <= samples; ++sample)
                points.push_back(project(curve_point(edge, static_cast<double>(sample) / samples)));
            return points;
        };

        // Hover mirrors the press priority in handle_event: tangent handles, then nodes, then
        // edges of a closed outline.
        const auto hit = overlay::handle_hit_radius * view_scale_;
        const auto idle = drag_ == DragKind::none && pointer_known_;
        const auto near_pointer = [&](const Vec2& point)
        {
            return math::length_squared(point - pointer_screen_px_) <= hit * hit;
        };
        std::optional<bool> hovered_handle;
        std::optional<std::size_t> hovered_node, hovered_edge;
        if (idle && selected_ && *selected_ < count)
            for (const auto incoming : { true, false })
            {
                const auto& node = outline_.nodes[*selected_];
                if ((incoming ? incoming_visible(*selected_) : outgoing_visible(*selected_)) && near_pointer(project(node.position_m + (incoming ? node.incoming_handle_m : node.outgoing_handle_m))))
                {
                    hovered_handle = incoming;
                    break;
                }
            }
        if (idle && !hovered_handle)
            for (std::size_t index = 0; index < count; ++index)
                if (near_pointer(project(outline_.nodes[index].position_m)))
                {
                    hovered_node = index;
                    break;
                }
        if (idle && !hovered_handle && !hovered_node && outline_.closed)
        {
            auto best_squared = overlay::edge_hit_distance * overlay::edge_hit_distance * view_scale_ * view_scale_;
            for (std::size_t edge = 0; edge < count; ++edge)
            {
                const auto points = edge_points(edge);
                for (std::size_t index = 1; index < points.size(); ++index)
                {
                    const auto delta = points[index] - points[index - 1];
                    const auto denominator = math::length_squared(delta);
                    const auto t = denominator > 0.0 ? math::clamp(math::dot(pointer_screen_px_ - points[index - 1], delta) / denominator, 0.0, 1.0) : 0.0;
                    const auto distance = math::length_squared(pointer_screen_px_ - (points[index - 1] + delta * t));
                    if (distance < best_squared)
                    {
                        best_squared = distance;
                        hovered_edge = edge;
                    }
                }
            }
        }

        // The selected node's outgoing edge is the one Straight, Curved and Insert act on.
        if (selected_ && edge_exists(*selected_))
        {
            const auto points = edge_points(*selected_);
            list.add_polyline(points, line_color.with_alpha(0.22f), paint.width(6.0));
            list.add_polyline(points, line_color, paint.width(overlay::emphasis));
        }
        if (hovered_edge && hovered_edge != selected_)
            list.add_polyline(edge_points(*hovered_edge), line_color.with_alpha(0.6f), paint.width(overlay::emphasis));

        // Chips are collected and drawn last so that no handle or guide covers a reading.
        struct PendingChip
        {
            Vec2 anchor;
            std::string text;
            ChipAlign align;
            render::Color color;
        };
        std::vector<PendingChip> chips;
        // Segment readings sit on the side of the segment away from the rest of the outline.
        Vec2 centroid {};
        for (const auto& node : outline_.nodes)
            centroid += project(node.position_m) / static_cast<double>(std::max<std::size_t>(count, 1));
        const auto segment_chip = [&](const Vec2& from, const Vec2& to, const Vec2& away_from, std::string text)
        {
            const auto delta = to - from;
            const auto length = math::length(delta);
            if (length < paint.px(28.0))
                return;
            auto normal = math::perpendicular(delta / length);
            const auto side = math::dot(normal, away_from - (from + to) * 0.5);
            if (side > 0.0 || (side == 0.0 && normal.y > 0.0))
                normal = -normal;
            // Push the centred chip far enough along the normal that its nearest corner, not
            // just its centre, clears a sloping segment.
            const auto half_width = paint.chip_text_width(text) * 0.5 + paint.px(4.0);
            const auto half_height = paint.chip_height() * 0.5;
            const auto offset = paint.px(5.0) + half_width * std::abs(normal.x) + half_height * std::abs(normal.y);
            chips.push_back({ (from + to) * 0.5 + normal * offset, std::move(text), ChipAlign::centre, palette.text });
        };
        const auto snap_marker = [&](const Vec2& at)
        {
            switch (pointer_snap_)
            {
            case SnapKind::vertex:
            {
                const Vec2 extent { paint.px(6.5), paint.px(6.5) };
                list.add_rectangle_outline(at - extent, at + extent, palette.accent, paint.width(overlay::standard));
                break;
            }
            case SnapKind::angle:
                if (snap_origin_world_)
                {
                    const auto origin = camera.world_to_screen(*snap_origin_world_);
                    const auto delta = at - origin;
                    const auto length = math::length(delta);
                    if (length > 1.0)
                        paint.dashed_path({ at, at + delta / length * std::max(paint.px(56.0), length * 0.6) }, false, palette.accent.with_alpha(0.65f), overlay::hairline, 1.5, 3.0);
                }
                break;
            case SnapKind::grid:
                list.add_cross(at, paint.width(3.5), palette.accent.with_alpha(0.8f), paint.width(overlay::hairline));
                break;
            case SnapKind::none:
                break;
            }
        };

        if (invalid)
        {
            std::vector<Vec2> crossings;
            if (path.size() <= 1024)
                find_crossings(path, crossings, 8);
            for (const auto& point : crossings)
            {
                list.add_circle_fill(point, paint.width(5.5), palette.danger.with_alpha(0.18f));
                list.add_circle_outline(point, paint.width(5.5), palette.danger, paint.width(overlay::standard));
            }
            const auto reason = result.diagnostics.empty() ? std::string_view { "Invalid outline" } : short_reason(result.diagnostics.front().code);
            if (!crossings.empty())
                chips.push_back({ crossings.front() + Vec2 { paint.px(12.0), -paint.px(12.0) }, std::string(reason), ChipAlign::left, palette.danger });
            else if (!path.empty())
            {
                auto top = path.front();
                for (const auto& point : path)
                    if (point.y < top.y)
                        top = point;
                chips.push_back({ top - Vec2 { 0.0, paint.px(10.0) + paint.chip_height() * 0.5 }, std::string(reason), ChipAlign::centre, palette.danger });
            }
        }

        const auto closing = closable && idle && near_pointer(project(outline_.nodes.front().position_m));
        // The implied closing edge, in the danger colour when closing now would not build.
        if (closable)
            paint.dashed_path(edge_points(count - 1), false, closing_valid ? palette.accent.with_alpha(0.55f) : palette.danger.with_alpha(0.65f), overlay::hairline);
        if (!outline_.closed && count > 0 && idle)
        {
            const auto last = project(outline_.nodes.back().position_m);
            const auto end = closing ? project(outline_.nodes.front().position_m) : camera.world_to_screen(pointer_world_);
            paint.dashed_path({ last, end }, false, palette.accent.with_alpha(0.85f), overlay::standard);
            if (!closing)
            {
                snap_marker(end);
                const auto delta = pointer_world_ - math::transform_point(placement_, outline_.nodes.back().position_m);
                if (math::length(delta) > 1.0e-9)
                    segment_chip(last, end, count >= 2 ? project(outline_.nodes[count - 2].position_m) : last + Vec2 { 0.0, 1.0 }, core::substitute("{} \xC2\xB7 {}", core::format_quantity(math::length(delta), core::DisplayQuantity::length, units), core::format_quantity(std::round(math::radians_to_degrees(std::atan2(delta.y, delta.x))), core::DisplayQuantity::angle, units)));
            }
        }
        if (closable)
        {
            const auto first = project(outline_.nodes.front().position_m);
            if (closing)
            {
                paint.halo(first, paint.px(13.0), palette.accent.with_alpha(0.18f));
                list.add_circle_outline(first, paint.width(9.0), palette.accent, paint.width(overlay::emphasis));
                chips.push_back({ first + Vec2 { paint.px(16.0), -paint.px(14.0) }, "Close shape", ChipAlign::left, palette.text });
            }
            else
                list.add_circle_outline(first, paint.width(9.0), palette.accent.with_alpha(0.45f), paint.width(overlay::hairline));
        }

        const auto dragging = drag_ != DragKind::none && !drag_threshold_pending_;
        if (selected_ && *selected_ < count)
        {
            const auto& node = outline_.nodes[*selected_];
            const auto origin = project(node.position_m);
            for (const auto incoming : { true, false })
            {
                if (!(incoming ? incoming_visible(*selected_) : outgoing_visible(*selected_)))
                    continue;
                const auto handle = project(node.position_m + (incoming ? node.incoming_handle_m : node.outgoing_handle_m));
                const auto active = dragging && drag_ == (incoming ? DragKind::incoming : DragKind::outgoing);
                const auto hover = hovered_handle && *hovered_handle == incoming;
                const auto radius = paint.px(active || hover ? 4.5 : 3.5);
                list.add_line(origin, handle, palette.accent.with_alpha(0.7f), paint.width(overlay::hairline));
                if (active || hover)
                    paint.halo(handle, radius + paint.px(4.0), palette.accent.with_alpha(0.2f));
                list.add_circle_fill(handle, static_cast<float>(radius), active ? palette.accent : palette.surface);
                list.add_circle_outline(handle, static_cast<float>(radius), palette.accent, paint.width(overlay::standard));
            }
        }

        for (std::size_t index = 0; index < count; ++index)
        {
            const auto selected = selected_ && *selected_ == index;
            const auto state = selected && drag_ == DragKind::node ? HandleState::active : hovered_node == index ? HandleState::hover
                                                                                                                 : HandleState::normal;
            paint.square_handle(project(outline_.nodes[index].position_m), state, selected);
        }

        if (dragging && drag_ == DragKind::node && selected_ && *selected_ < count)
        {
            const auto index = *selected_;
            snap_marker(project(outline_.nodes[index].position_m));
            const auto length_chip = [&](std::size_t edge)
            {
                if (!edge_exists(edge) || outline_.nodes[edge].outgoing_edge != OutlineEdgeKind::line)
                    return;
                const auto& from = outline_.nodes[edge].position_m;
                const auto& to = outline_.nodes[(edge + 1) % count].position_m;
                segment_chip(project(from), project(to), centroid, core::format_quantity(math::length(to - from), core::DisplayQuantity::length, units));
            };
            if (outline_.closed || index > 0)
                length_chip((index + count - 1) % count);
            length_chip(index);
        }

        for (const auto& chip : chips)
            paint.chip(chip.anchor, chip.text, chip.align, chip.color);
    }
}
