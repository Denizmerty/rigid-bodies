#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>
#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/convex_distance.hpp>
#include <rigidbodies/physics/joint.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace rigidbodies::app
{
    namespace
    {
        bool finite(const math::Vec2& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y);
        }

        double segment_distance_squared(const math::Vec2& point, const math::Vec2& first, const math::Vec2& second)
        {
            const auto delta = second - first;
            const auto squared = math::length_squared(delta);
            const auto t = squared > 0.0 ? std::clamp(math::dot(point - first, delta) / squared, 0.0, 1.0) : 0.0;
            return math::length_squared(point - (first + delta * t));
        }

        bool near_shape(const physics::Shape& shape, const math::Vec2& local, double tolerance)
        {
            if (shape.contains_local_point(local))
                return true;
            if (const auto* circle = dynamic_cast<const physics::CircleShape*>(&shape))
                return math::length(local - circle->local_center_m()) <= circle->radius_m() + tolerance;
            const auto squared = tolerance * tolerance;
            if (const auto* polygon = dynamic_cast<const physics::ConvexPolygonShape*>(&shape))
            {
                const auto& vertices = polygon->vertices();
                for (std::size_t i = 0; i < vertices.size(); ++i)
                    if (segment_distance_squared(local, vertices[i], vertices[(i + 1) % vertices.size()]) <= squared)
                        return true;
                return false;
            }
            if (const auto* segment = dynamic_cast<const physics::SegmentShape*>(&shape))
                return segment_distance_squared(local, segment->start_m(), segment->end_m()) <= squared;
            const physics::CircleShape probe(tolerance, local);
            const auto distance = physics::convex_distance(shape, {}, probe, {});
            return distance.valid && distance.intersecting;
        }

        math::Vec2 limited(const math::Vec2& value, double maximum)
        {
            if (!finite(value) || !std::isfinite(maximum) || maximum <= 0.0)
                return {};
            const auto magnitude = std::hypot(value.x, value.y);
            return magnitude > maximum ? value * (maximum / magnitude) : value;
        }

        // Shared by the force generator and the overlay's force readout.
        math::Vec2 pointer_spring_acceleration(const physics::RigidBody& body, const math::Vec2& anchor, const math::Vec2& target_world_m)
        {
            return limited((target_world_m - anchor) * 60.0 - body.velocity_at_world_point(anchor) * 14.0, 100.0);
        }

        // A temporary per-body generator lets staged integrators sample the spring at their own
        // predicted poses. The target is fixed during a physical substep, while the local grab
        // point, point velocity and damping are recomputed by every stage.
        class PointerSpring final : public physics::ForceGenerator
        {
        public:
            math::Vec2 local_anchor_m, target_world_m;

            [[nodiscard]] std::shared_ptr<physics::ForceGenerator> clone() const override
            {
                return std::make_shared<PointerSpring>(*this);
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "pointer_spring";
            }

            void apply(physics::RigidBody& body, const physics::ForceContext&) override
            {
                const auto anchor = math::transform_point(body.transform(), local_anchor_m);
                const auto acceleration = pointer_spring_acceleration(body, anchor, target_world_m);
                const auto force = acceleration * body.mass_properties().mass_kg;
                if (finite(force))
                    body.apply_force_at_world_point(force, anchor, "Pointer spring");
            }
        };

        // Additional selections follow each part's actual outline, matching the scene's own
        // selection treatment instead of an axis-aligned box. Glows go first so that no outline
        // of a compound body is covered by a neighbouring part's glow.
        void draw_selection_outline(const OverlayPainter& paint, const render::Camera2D& camera, const physics::RigidBody& body,
            const math::Transform2& placement, float opacity = 1.0f)
        {
            auto& list = paint.list();
            const auto color = paint.palette().selection.with_alpha(paint.palette().selection.alpha * opacity);
            std::vector<const physics::AuthoredPartDefinition*> parts;
            std::vector<math::Vec2> points;
            for (const auto glow : { true, false })
            {
                parts.clear();
                const auto stroke_color = glow ? color.with_alpha(color.alpha * 0.18f) : color;
                const auto stroke_width = paint.width(glow ? 5.0 : overlay::standard);
                for (const auto& collider : body.colliders())
                {
                    if (!collider.shape)
                        continue;
                    points.clear();
                    if (collider.authored_part && collider.authored_part->shape)
                    {
                        const auto* part = collider.authored_part.get();
                        if (std::find(parts.begin(), parts.end(), part) != parts.end())
                            continue;
                        parts.push_back(part);
                        const auto part_placement = math::concatenate(placement, part->local_transform);
                        for (const auto& point : part->shape->render_outline)
                            points.push_back(camera.world_to_screen(math::transform_point(part_placement, point)));
                        list.add_polyline(points, stroke_color, stroke_width, true);
                        continue;
                    }
                    const auto collider_placement = math::concatenate(placement, collider.local_transform);
                    if (const auto* circle = dynamic_cast<const physics::CircleShape*>(collider.shape.get()))
                    {
                        const auto centre = camera.world_to_screen(math::transform_point(collider_placement, circle->local_center_m()));
                        list.add_circle_outline(centre, static_cast<float>(camera.world_to_screen_length(circle->radius_m())), stroke_color, stroke_width);
                    }
                    else if (const auto* polygon = dynamic_cast<const physics::ConvexPolygonShape*>(collider.shape.get()))
                    {
                        for (const auto& vertex : polygon->vertices())
                            points.push_back(camera.world_to_screen(math::transform_point(collider_placement, vertex)));
                        list.add_polyline(points, stroke_color, stroke_width, true);
                    }
                    else if (const auto* segment = dynamic_cast<const physics::SegmentShape*>(collider.shape.get()))
                    {
                        points.push_back(camera.world_to_screen(math::transform_point(collider_placement, segment->start_m())));
                        points.push_back(camera.world_to_screen(math::transform_point(collider_placement, segment->end_m())));
                        list.add_polyline(points, stroke_color, stroke_width, false);
                    }
                }
            }
        }

        // An elastic band from the grab point to the pointer: as it stretches the stroke grows
        // heavier and more opaque and its dashes open up, so tension reads without a number. A
        // faint continuous core keeps the band reading as one connection across the gaps.
        void draw_pull_spring(const OverlayPainter& paint, const math::Vec2& anchor, const math::Vec2& target, const render::Color& color)
        {
            if (!(math::length(target - anchor) > paint.px(2.0)))
                return;
            const auto tension = std::clamp(math::length(target - anchor) / paint.px(180.0), 0.0, 1.0);
            paint.list().add_line(anchor, target, color.with_alpha(color.alpha * 0.3f), paint.width(overlay::hairline));
            paint.dashed_path({ anchor, target }, false, color.with_alpha(color.alpha * static_cast<float>(0.6 + 0.4 * tension)), 1.25 + tension, 6.0 + 4.0 * tension, 3.0 + 4.0 * tension);
        }

        render::Color with_opacity(const render::Color& color, float factor)
        {
            return color.with_alpha(color.alpha * factor);
        }

        void sample_pointer(InteractionState& state, double time_s)
        {
            if (!std::isfinite(time_s) || time_s < 0.0)
                return;
            if (!state.samples.empty() && time_s < state.samples.back().time_s)
                return;
            if (!state.samples.empty() && time_s == state.samples.back().time_s)
                state.samples.back().position_m = state.target_world_m;
            else
                state.samples.push_back({ state.target_world_m, time_s });
            // Keep the nearest sample before the 120 ms window as well as those inside it.
            while (state.samples.size() > 2 && state.samples[1].time_s < time_s - 0.12)
                state.samples.pop_front();
            while (state.samples.size() > 32)
                state.samples.pop_front();
        }
    }

    InteractionMode SimulationSession::interaction_mode() const
    {
        return interaction_.mode;
    }

    bool SimulationSession::interaction_active() const
    {
        return interaction_.active || handle_drag_.active;
    }

    ui::CursorShape SimulationSession::scene_cursor() const
    {
        if (handle_drag_.active)
            return ui::CursorShape::grab;
        if (!interaction_.active && !handle_at(camera_.world_to_screen(pointer_world_m_), overlay_scale()).empty())
            return ui::CursorShape::open_hand;
        if (shape_editor_.active())
            return ui::CursorShape::pen;
        if (interaction_.mode == InteractionMode::throw_body)
            return ui::CursorShape::crosshair;
        if (interaction_.mode == InteractionMode::pull)
            return ui::CursorShape::grab;
        if (world_.is_valid(interaction_.hover_body))
            if (const auto* body = world_.find_body(interaction_.hover_body); body && body->type() == physics::BodyType::dynamic_body)
                return ui::CursorShape::open_hand;
        return ui::CursorShape::arrow;
    }

    physics::BodyId SimulationSession::pick_body_at(const math::Vec2& screen_point_px, double logical_scale) const
    {
        physics::BodyId hit;
        double smallest_area = std::numeric_limits<double>::infinity();
        const auto tolerance = 6.0 * std::max(0.01, logical_scale);
        const auto point = camera_.screen_to_world(screen_point_px);
        const auto tolerance_m = camera_.screen_to_world_length(tolerance);
        world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
            {
                if (is_marker(id))
                    return;
                const auto placement = body.interpolated_transform(scene_settings_.interpolation_alpha);
                const auto bounds = body.compute_bounds(placement);
                if (bounds.is_empty())
                    return;
                const auto a = camera_.world_to_screen(bounds.minimum);
                const auto b = camera_.world_to_screen(bounds.maximum);
                const auto minimum = math::Vec2 { std::min(a.x, b.x) - tolerance, std::min(a.y, b.y) - tolerance };
                const auto maximum = math::Vec2 { std::max(a.x, b.x) + tolerance, std::max(a.y, b.y) + tolerance };
                if (screen_point_px.x < minimum.x || screen_point_px.x > maximum.x || screen_point_px.y < minimum.y || screen_point_px.y > maximum.y)
                    return;
                const auto size = bounds.maximum - bounds.minimum;
                const auto area = std::abs(size.x * size.y);
                // Bounds are only a broad check: empty corners and gaps between compound parts
                // must remain clickable. Keep a small screen-space margin around real geometry.
                const auto near_collider = std::any_of(body.colliders().begin(), body.colliders().end(), [&](const auto& collider)
                    {
                        return collider.shape && near_shape(*collider.shape, math::inverse_transform_point(math::concatenate(placement, collider.local_transform), point), tolerance_m);
                    });
                if (!near_collider)
                    return;
                if (area < smallest_area)
                {
                    smallest_area = area;
                    hit = id;
                }
            });
        return hit;
    }

    std::optional<std::pair<math::Vec2, math::Vec2>> SimulationSession::connection_anchors_m(std::string_view key, std::string_view kind) const
    {
        if (kind == "spring")
        {
            for (const auto id : world_.spring_ids())
                if (world_.spring_key(id) == key)
                    if (const auto report = world_.spring_report(id))
                        return std::pair { report->first_anchor_m, report->second_anchor_m };
            return std::nullopt;
        }
        for (const auto& constraint : world_.constraints())
            if (world_.constraint_key(constraint) == key)
            {
                if (const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get()))
                {
                    const auto report = joint->report(world_);
                    return std::pair { report.first_anchor_m, report.second_anchor_m };
                }
                const auto* first = world_.find_body(constraint->first_body());
                const auto* second = world_.find_body(constraint->second_body());
                if (first && second)
                    return std::pair { first->world_center_of_mass_m(), second->world_center_of_mass_m() };
            }
        return std::nullopt;
    }

    std::optional<ui::SelectedConnection> SimulationSession::pick_connection_at(const math::Vec2& point, double logical_scale) const
    {
        if (!scene_settings_.layers.is_enabled(render::VisualizationLayer::constraints))
            return {};
        const auto tolerance_squared = std::pow(6.0 * std::max(0.01, logical_scale), 2.0);
        // The stage draws joint symbols about 7 px across at the display density alone.
        const auto symbol_radius = 9.0 * std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
        const auto near_segment = [&](math::Vec2 first, math::Vec2 second)
        {
            const auto delta = second - first;
            const auto length_squared = math::length_squared(delta);
            const auto t = length_squared > 0.0 ? std::clamp(math::dot(point - first, delta) / length_squared, 0.0, 1.0) : 0.0;
            return math::length_squared(point - (first + delta * t)) <= tolerance_squared;
        };
        // Inside the outline of a moving body it joins, the pointer means that body: the drawn
        // line runs on to an anchor that is often the body's centre of mass, exactly where a
        // reader points at or grabs the body. A joint's own symbol stays the joint's unless it
        // sits on that centre of mass. A fixed anchor never hides what is attached to it.
        const auto world_point = camera_.screen_to_world(point);
        const auto over_joined_body = [&](physics::BodyId id, const std::optional<math::Vec2>& symbol)
        {
            const auto* body = world_.find_body(id);
            if (body == nullptr || body->type() == physics::BodyType::static_body || is_marker(id) || !body->contains_world_point(world_point, body->interpolated_transform(scene_settings_.interpolation_alpha)))
                return false;
            return !symbol || math::length(camera_.world_to_screen(body->world_center_of_mass_m()) - *symbol) < symbol_radius;
        };
        const auto picked = [&](physics::BodyId first, physics::BodyId second, const math::Vec2& first_anchor, const math::Vec2& second_anchor, const std::optional<math::Vec2>& symbol)
        {
            const auto on_symbol = symbol && math::length_squared(point - *symbol) <= symbol_radius * symbol_radius;
            if (!on_symbol && !near_segment(first_anchor, second_anchor))
                return false;
            const auto symbol_hit = on_symbol ? symbol : std::nullopt;
            return !over_joined_body(first, symbol_hit) && !over_joined_body(second, symbol_hit);
        };
        for (const auto& constraint : world_.constraints())
        {
            if (const auto* joint = dynamic_cast<const physics::JointConstraint*>(constraint.get()))
            {
                const auto report = joint->report(world_);
                const auto first_anchor = camera_.world_to_screen(report.first_anchor_m);
                const auto second_anchor = camera_.world_to_screen(report.second_anchor_m);
                std::optional<math::Vec2> symbol;
                if (!report.broken && report.enabled && report.kind != physics::JointKind::distance)
                    symbol = report.kind == physics::JointKind::prismatic ? second_anchor : (first_anchor + second_anchor) * 0.5;
                if (picked(report.first, report.second, first_anchor, second_anchor, symbol))
                    return ui::SelectedConnection { world_.constraint_key(constraint), "joint" };
                continue;
            }
            const auto* first = world_.find_body(constraint->first_body());
            const auto* second = world_.find_body(constraint->second_body());
            if (first && second && picked(constraint->first_body(), constraint->second_body(), camera_.world_to_screen(first->world_center_of_mass_m()), camera_.world_to_screen(second->world_center_of_mass_m()), std::nullopt))
                return ui::SelectedConnection { world_.constraint_key(constraint), "joint" };
        }
        for (const auto id : world_.spring_ids())
            if (const auto report = world_.spring_report(id); report && picked(report->first, report->second, camera_.world_to_screen(report->first_anchor_m), camera_.world_to_screen(report->second_anchor_m), std::nullopt))
                return ui::SelectedConnection { std::string(world_.spring_key(id)), "spring" };
        return {};
    }

    std::string SimulationSession::handle_at(const math::Vec2& point, double logical_scale) const
    {
        if (!stepper_.is_paused())
            return {};
        const auto scale = std::max(0.01, logical_scale);
        const auto within = [&](const math::Vec2& target, double radius)
        {
            return math::length_squared(point - target) <= radius * radius;
        };
        const auto drawn_scale = overlay_scale();
        // The draft editor owns the stage and a group has no single-body handles, so neither
        // can take a press meant for a node or for one of the group.
        if (const auto* body = handle_body())
        {
            const auto centre_m = body->world_center_of_mass_m();
            if (within(velocity_knob_position(camera_, centre_m, body->linear_velocity_m_s(), velocity_handle_scale(), drawn_scale), overlay::handle_hit_radius * scale))
                return "velocity";
            const auto ring = handle_ring_radius(*body);
            // A grip that happens to lie on another object's centre of mass gives that press to
            // the object, so it can still be selected and moved.
            bool on_other_centre = false;
            world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& other)
                {
                    if (!(id == selection()) && other.type() != physics::BodyType::static_body && !is_marker(id) && within(camera_.world_to_screen(other.world_center_of_mass_m()), 6.0 * scale))
                        on_other_centre = true;
                });
            if (!on_other_centre && within(rotation_knob_position(camera_.world_to_screen(centre_m), ring, body->orientation_rad()), overlay::handle_hit_radius * scale))
                return "rotation";
        }
        if (within(gravity_compass_centre(camera_, drawn_scale), std::max(overlay::handle_hit_radius * scale, (overlay::compass_radius + 2.0) * drawn_scale)))
            return "gravity";
        return {};
    }

    bool SimulationSession::handle_stage_handle_event(const ui::UiEvent& event, bool interface_consumed)
    {
        const auto make_command = [&]()
        {
            ui::UiCommand command;
            command.body = handle_drag_.body;
            command.phase = handle_drag_.moved ? ui::UiEditPhase::commit : ui::UiEditPhase::preview;
            command.detail = "handle";
            if (handle_drag_.id == "velocity")
            {
                command.kind = ui::UiCommandKind::set_selected_velocity;
                if (const auto* body = world_.find_body(handle_drag_.body))
                {
                    auto velocity = velocity_from_handle(camera_, body->world_center_of_mass_m(), event.pointer_px, velocity_handle_scale());
                    if (event.modifiers.shift)
                    {
                        auto magnitude = std::round(math::length(velocity) * 2.0) * 0.5;
                        const auto angle = std::round(std::atan2(velocity.y, velocity.x) / (math::pi / 12.0)) * (math::pi / 12.0);
                        velocity = { std::cos(angle) * magnitude, std::sin(angle) * magnitude };
                    }
                    command.value = velocity.x;
                    command.value_y = velocity.y;
                }
            }
            else if (handle_drag_.id == "rotation")
            {
                command.kind = ui::UiCommandKind::set_selected_orientation;
                if (const auto* body = world_.find_body(handle_drag_.body))
                {
                    auto degrees = math::radians_to_degrees(orientation_from_knob(camera_.world_to_screen(body->world_center_of_mass_m()), event.pointer_px));
                    if (event.modifiers.shift)
                        degrees = std::round(degrees / 15.0) * 15.0;
                    command.value = degrees;
                }
            }
            else
            {
                command.kind = ui::UiCommandKind::set_gravity_angle_degrees;
                const auto compass = gravity_compass_centre(camera_, overlay_scale());
                auto degrees = math::radians_to_degrees(std::atan2(-(event.pointer_px.y - compass.y), event.pointer_px.x - compass.x));
                if (event.modifiers.shift)
                    degrees = std::round(degrees / 15.0) * 15.0;
                command.value = degrees;
            }
            return command;
        };

        if (handle_drag_.active)
        {
            if (event.kind == ui::UiEventKind::key_down && event.key == ui::UiKey::escape)
            {
                cancel_edit();
                handle_drag_ = {};
                synchronize_render_history();
                return true;
            }
            if (event.kind == ui::UiEventKind::pointer_move)
            {
                if (!handle_drag_.moved && math::length(event.pointer_px - handle_drag_.press_screen_px) < 4.0 * std::max(0.01, event.logical_pixel_scale))
                    return true;
                handle_drag_.moved = true;
                auto command = make_command();
                command.phase = ui::UiEditPhase::preview;
                apply(command);
                return true;
            }
            if (event.kind == ui::UiEventKind::pointer_up && event.button == ui::PointerButton::primary)
            {
                if (handle_drag_.moved)
                {
                    auto command = make_command();
                    command.phase = ui::UiEditPhase::commit;
                    apply(command);
                }
                handle_drag_ = {};
                return true;
            }
            return event.kind == ui::UiEventKind::wheel;
        }
        if (interface_consumed || event.kind != ui::UiEventKind::pointer_down || event.button != ui::PointerButton::primary)
            return false;
        const auto id = handle_at(event.pointer_px, event.logical_pixel_scale);
        if (id.empty())
            return false;
        handle_drag_.id = id;
        handle_drag_.body = selection();
        handle_drag_.press_screen_px = event.pointer_px;
        handle_drag_.velocity_scale = velocity_handle_scale();
        handle_drag_.active = true;
        return true;
    }

    void SimulationSession::cancel_gesture()
    {
        if (handle_drag_.active)
        {
            cancel_edit();
            handle_drag_ = {};
            synchronize_render_history();
        }
        cancel_interaction();
        dragging_view_ = false;
        secondary_pan_pending_ = false;
        shape_editor_.release_pointer();
    }

    void SimulationSession::deselect_draft_node()
    {
        shape_editor_.deselect_node();
    }

    bool SimulationSession::apply_interaction_command(const ui::UiCommand& command)
    {
        if (command.kind == ui::UiCommandKind::select_all)
        {
            if (!shape_editor_.active())
            {
                cancel_interaction();
                auto free_objects = world_.body_ids();
                free_objects.erase(std::remove_if(free_objects.begin(), free_objects.end(), [&](physics::BodyId id)
                                       {
                                           const auto* body = world_.find_body(id);
                                           return body == nullptr || body->type() != physics::BodyType::dynamic_body || is_marker(id);
                                       }),
                    free_objects.end());
                set_selections(free_objects);
            }
            return true;
        }
        if (command.kind != ui::UiCommandKind::set_interaction_mode)
            return false;
        InteractionMode mode;
        if (command.id == "select" || command.id == "move")
            mode = InteractionMode::select;
        else if (command.id == "throw")
            mode = InteractionMode::throw_body;
        else if (command.id == "pull")
            mode = InteractionMode::pull;
        else
            return true;
        if (shape_editor_.active())
            return true;
        if (pending_edit_command_)
        {
            cancel_edit();
            held_reason_.clear();
        }
        if (mode != interaction_.mode)
            cancel_interaction();
        interaction_.mode = mode;
        return true;
    }

    void SimulationSession::cancel_interaction()
    {
        if (!interaction_.active)
            return;
        const auto mode = interaction_.mode;
        interaction_ = {};
        interaction_.mode = mode;
        cancel_edit();
        synchronize_render_history();
    }

    bool SimulationSession::handle_interaction_event(const ui::UiEvent& event, bool interface_consumed)
    {
        if (event.kind == ui::UiEventKind::focus_lost)
        {
            const auto captured = interaction_.active || dragging_view_;
            cancel_interaction();
            dragging_view_ = false;
            secondary_pan_pending_ = false;
            return captured;
        }
        if (event.kind == ui::UiEventKind::pointer_leave)
        {
            // Nothing on the stage stays highlighted once the pointer has left it for the title
            // bar or another window; a drag in progress carries on to its release.
            if (!interaction_.active)
            {
                interaction_.hover_body = {};
                interaction_.hover_connection_key.clear();
                interaction_.hover_connection_kind.clear();
                scene_settings_.hover = {};
            }
            return false;
        }
        if (shape_editor_.active())
            return false;
        if (interaction_.active && interaction_.marquee && event.kind == ui::UiEventKind::pointer_down && event.button == ui::PointerButton::primary)
        {
            const auto mode = interaction_.mode;
            interaction_ = {};
            interaction_.mode = mode;
        }
        if (!interface_consumed && event.kind == ui::UiEventKind::pointer_move && !interaction_.active)
        {
            if (const auto connection = pick_connection_at(event.pointer_px, event.logical_pixel_scale))
            {
                if (connection->key != interaction_.hover_connection_key || connection->kind != interaction_.hover_connection_kind)
                    interaction_.hover_started_s = has_wall_time_ ? wall_time_s_ : event.timestamp_s;
                interaction_.hover_body = {};
                interaction_.hover_connection_key = connection->key;
                interaction_.hover_connection_kind = connection->kind;
                scene_settings_.hover = {};
                return false;
            }
            interaction_.hover_connection_key.clear();
            interaction_.hover_connection_kind.clear();
            auto hit = pick_body_at(event.pointer_px, event.logical_pixel_scale);
            if (!hit.is_valid())
            {
                const auto tolerance = 6.0 * std::max(0.01, event.logical_pixel_scale);
                double area = std::numeric_limits<double>::infinity();
                world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                    {
                        if (!is_marker(id))
                            return;
                        const auto bounds = body.compute_bounds(body.interpolated_transform(scene_settings_.interpolation_alpha));
                        const auto a = camera_.world_to_screen(bounds.minimum), b = camera_.world_to_screen(bounds.maximum);
                        const auto minimum = math::min_components(a, b) - math::Vec2 { tolerance, tolerance };
                        const auto maximum = math::max_components(a, b) + math::Vec2 { tolerance, tolerance };
                        if (event.pointer_px.x < minimum.x || event.pointer_px.x > maximum.x || event.pointer_px.y < minimum.y || event.pointer_px.y > maximum.y)
                            return;
                        const auto size = maximum - minimum;
                        if (size.x * size.y < area)
                        {
                            area = size.x * size.y;
                            hit = id;
                        }
                    });
            }
            if (!(hit == interaction_.hover_body))
            {
                interaction_.hover_body = hit;
                interaction_.hover_started_s = has_wall_time_ ? wall_time_s_ : event.timestamp_s;
            }
            scene_settings_.hover = hit;
        }
        if (interaction_.active)
        {
            if (event.kind == ui::UiEventKind::wheel)
                return true;
            if (event.kind == ui::UiEventKind::key_down && event.key == ui::UiKey::escape)
            {
                cancel_interaction();
                return true;
            }
            if (!interaction_.marquee && !world_.is_valid(interaction_.anchor_body))
            {
                cancel_interaction();
                return true;
            }
            if (event.kind == ui::UiEventKind::pointer_up && event.button == ui::PointerButton::primary)
            {
                // Platforms can coalesce motion immediately before a button event. Its final
                // position is authoritative even if there was no separate motion notification.
                auto final_motion = event;
                final_motion.kind = ui::UiEventKind::pointer_move;
                (void)handle_interaction_event(final_motion, interface_consumed);
                if (interaction_.marquee)
                {
                    if (interaction_.moved)
                    {
                        const auto minimum = math::min_components(interaction_.press_world_m, interaction_.target_world_m);
                        const auto maximum = math::max_components(interaction_.press_world_m, interaction_.target_world_m);
                        math::Aabb marquee;
                        marquee.minimum = minimum;
                        marquee.maximum = maximum;
                        auto selected = selections();
                        world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                            {
                                if (body.type() != physics::BodyType::static_body && !is_marker(id) && math::overlaps(marquee, body.compute_bounds()) &&
                                    std::find(selected.begin(), selected.end(), id) == selected.end())
                                    selected.push_back(id);
                            });
                        set_selections(selected);
                    }
                    const auto mode = interaction_.mode;
                    interaction_ = {};
                    interaction_.mode = mode;
                    return true;
                }
                // A release over a panel still terminates the capture. Include its time so a
                // pointer held still before release does not keep an old throwing velocity.
                sample_pointer(interaction_, event.timestamp_s);
                if (interaction_.mode == InteractionMode::throw_body && interaction_.moved && interaction_.samples.size() > 1)
                {
                    const auto& first = interaction_.samples.front();
                    const auto& last = interaction_.samples.back();
                    const auto duration = last.time_s - first.time_s;
                    const auto maximum = std::min(25.0, world_.settings().limits.maximum_linear_speed_m_s);
                    const auto velocity = duration > 1.0e-6 ? limited((last.position_m - first.position_m) / duration, maximum) : math::Vec2 {};
                    for (const auto& item : interaction_.bodies)
                        if (auto* body = world_.find_body(item.id))
                        {
                            body->set_linear_velocity(velocity);
                            (void)world_.notify_body_properties_changed(item.id);
                            mark_edit_changed();
                        }
                }
                interaction_.active = false;
                if (interaction_.pull_generator)
                {
                    (void)world_.remove_force_generator(interaction_.anchor_body, interaction_.pull_generator);
                    interaction_.pull_generator.reset();
                }
                interaction_.bodies.clear();
                interaction_.samples.clear();
                commit_edit();
                synchronize_render_history();
                if (interaction_.moved || interaction_.mode == InteractionMode::pull)
                    reset_measurements();
                return true;
            }
            if (event.kind != ui::UiEventKind::pointer_move)
                return false;
            if (!interaction_.moved &&
                math::length(event.pointer_px - interaction_.press_screen_px) < 4.0 * std::max(1.0, event.logical_pixel_scale))
                return true;
            const auto target = camera_.screen_to_world(event.pointer_px);
            if (!finite(target))
                return true;
            interaction_.target_world_m = target;
            if (interaction_.marquee)
            {
                interaction_.moved = true;
                return true;
            }
            if (interaction_.mode == InteractionMode::pull)
            {
                if (!interaction_.pull_generator)
                {
                    auto spring = std::make_shared<PointerSpring>();
                    spring->local_anchor_m = interaction_.local_anchor_m;
                    spring->target_world_m = interaction_.target_world_m;
                    interaction_.pull_generator = spring;
                    (void)world_.add_force_generator(interaction_.anchor_body, std::move(spring));
                    interaction_.moved = true;
                }
                const auto maximum = world_.settings().limits.maximum_position_m;
                interaction_.target_world_m.x = std::clamp(target.x, -maximum, maximum);
                interaction_.target_world_m.y = std::clamp(target.y, -maximum, maximum);
                return true;
            }
            auto delta = target - interaction_.press_world_m;
            const auto maximum = world_.settings().limits.maximum_position_m;
            math::Vec2 minimum_delta { -2.0 * maximum, -2.0 * maximum };
            math::Vec2 maximum_delta { 2.0 * maximum, 2.0 * maximum };
            for (const auto& item : interaction_.bodies)
            {
                minimum_delta.x = std::max(minimum_delta.x, -maximum - item.original_position_m.x);
                minimum_delta.y = std::max(minimum_delta.y, -maximum - item.original_position_m.y);
                maximum_delta.x = std::min(maximum_delta.x, maximum - item.original_position_m.x);
                maximum_delta.y = std::min(maximum_delta.y, maximum - item.original_position_m.y);
            }
            delta.x = std::clamp(delta.x, minimum_delta.x, maximum_delta.x);
            delta.y = std::clamp(delta.y, minimum_delta.y, maximum_delta.y);
            interaction_.target_world_m = interaction_.press_world_m + delta;
            for (const auto& item : interaction_.bodies)
                if (auto* body = world_.find_body(item.id))
                {
                    const auto position = item.original_position_m + delta;
                    if (math::length_squared(position - body->position_m()) > 0.0)
                    {
                        body->set_position(position);
                        body->set_linear_velocity({});
                        body->set_angular_velocity(0.0);
                        (void)world_.notify_body_properties_changed(item.id);
                        mark_edit_changed();
                        interaction_.moved = true;
                    }
                }
            sample_pointer(interaction_, event.timestamp_s);
            return true;
        }
        if (interface_consumed || dragging_view_ || event.kind != ui::UiEventKind::pointer_down || event.button != ui::PointerButton::primary)
            return false;
        const auto point = camera_.screen_to_world(event.pointer_px);
        if (!finite(point))
            return true;
        if (const auto connection = pick_connection_at(event.pointer_px, event.logical_pixel_scale))
        {
            selected_connection_ = connection;
            set_selection({});
            return true;
        }
        const auto hit = pick_body_at(event.pointer_px, event.logical_pixel_scale);
        auto selected = selections();
        const auto found = std::find(selected.begin(), selected.end(), hit);
        if (event.modifiers.shift)
        {
            if (hit.is_valid())
            {
                if (found == selected.end())
                    selected.push_back(hit);
                else
                    selected.erase(found);
                set_selections(selected);
            }
            else if (interaction_.mode == InteractionMode::select)
            {
                interaction_.active = true;
                interaction_.marquee = true;
                interaction_.moved = false;
                interaction_.press_world_m = interaction_.target_world_m = point;
                interaction_.press_screen_px = event.pointer_px;
            }
            return true;
        }
        if (found == selected.end() || !hit.is_valid())
            set_selection(hit);
        if (!hit.is_valid())
            return true;
        auto* body = world_.find_body(hit);
        if (body == nullptr || body->type() != physics::BodyType::dynamic_body)
            return true;
        begin_edit(interaction_.mode == InteractionMode::pull ? "Pull body" : interaction_.mode == InteractionMode::throw_body ? "Throw selection"
                                                                                                                               : "Move selection");
        if (!edit_in_progress())
            return true;
        interaction_.active = true;
        interaction_.moved = false;
        interaction_.anchor_body = hit;
        interaction_.press_world_m = interaction_.target_world_m = point;
        interaction_.press_screen_px = event.pointer_px;
        // The displayed point may be interpolated; preserve the corresponding local material
        // anchor when attaching the pull to the current physical pose.
        interaction_.local_anchor_m = math::inverse_transform_point(body->interpolated_transform(scene_settings_.interpolation_alpha), point);
        interaction_.bodies.clear();
        for (const auto id : selections())
            if (const auto* selected_body = world_.find_body(id); selected_body && selected_body->type() == physics::BodyType::dynamic_body)
                interaction_.bodies.push_back({ id, selected_body->position_m() });
        interaction_.samples.clear();
        sample_pointer(interaction_, event.timestamp_s);
        return true;
    }

    void SimulationSession::apply_interaction_forces()
    {
        if (!interaction_.active || interaction_.mode != InteractionMode::pull)
            return;
        auto* body = world_.find_body(interaction_.anchor_body);
        if (!body || body->type() != physics::BodyType::dynamic_body)
            return;
        auto* spring = dynamic_cast<PointerSpring*>(interaction_.pull_generator.get());
        if (!spring)
            return;
        spring->target_world_m = interaction_.target_world_m;
        body->wake();
        mark_edit_changed();
    }

    void SimulationSession::render_interaction(render::DrawList& list) const
    {
        const auto previous_layer = list.layer();
        list.set_layer(overlay::overlay_layer);
        const OverlayPainter paint(list, scene_settings_.theme, overlay_scale());
        const auto& palette = paint.palette();
        const auto& theme = scene_settings_.theme;
        const auto units = scene_settings_.display_units;
        const auto alpha = scene_settings_.interpolation_alpha;

        // Every selected body has the renderer's own selection outline, one style for all of them.
        if (interaction_.active && interaction_.marquee)
        {
            const auto first = camera_.world_to_screen(interaction_.press_world_m);
            const auto second = camera_.world_to_screen(interaction_.target_world_m);
            const auto minimum = math::min_components(first, second), maximum = math::max_components(first, second);
            list.add_rectangle_fill(minimum, maximum, with_opacity(palette.selection, 0.07f));
            // Objects the release would add are previewed with a fainter selection outline,
            // using the same test the release applies.
            if (interaction_.moved)
            {
                math::Aabb marquee;
                marquee.minimum = math::min_components(interaction_.press_world_m, interaction_.target_world_m);
                marquee.maximum = math::max_components(interaction_.press_world_m, interaction_.target_world_m);
                const auto& selected = selections();
                world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                    {
                        if (body.type() != physics::BodyType::static_body && !is_marker(id) && math::overlaps(marquee, body.compute_bounds()) &&
                            std::find(selected.begin(), selected.end(), id) == selected.end())
                            draw_selection_outline(paint, camera_, body, body.interpolated_transform(alpha), 0.5f);
                    });
            }
            paint.dashed_path({ minimum, { maximum.x, minimum.y }, maximum, { minimum.x, maximum.y } }, true, palette.selection, overlay::standard);
        }

        if (stepper_.is_paused())
        {
            const auto pointer = camera_.world_to_screen(pointer_world_m_);
            const auto hovered = handle_drag_.active ? std::string {} : handle_at(pointer, overlay_scale());
            const auto state_of = [&](std::string_view id)
            {
                if (handle_drag_.active)
                    return handle_drag_.id == id ? HandleState::active : HandleState::normal;
                return hovered == id ? HandleState::hover : HandleState::normal;
            };
            // Body handles step aside while the draft editor owns the stage, while several
            // objects are selected, or while another gesture (move, throw, pull, marquee) is in
            // progress.
            const auto* body = interaction_.active ? nullptr : handle_body();
            if (body)
            {
                const auto centre_m = body->world_center_of_mass_m();
                const auto centre = camera_.world_to_screen(centre_m);
                const auto ring = handle_ring_radius(*body);
                const auto rotation_state = state_of("rotation");
                const auto rotation_knob = rotation_knob_position(centre, ring, body->orientation_rad());
                list.set_layer(overlay::selection_ring_layer);
                paint.dashed_circle(centre, ring, with_opacity(palette.accent, rotation_state == HandleState::normal ? 0.6f : 0.95f), overlay::hairline);
                list.set_layer(overlay::overlay_layer);
                if (rotation_state != HandleState::normal)
                    list.add_line(centre, rotation_knob, with_opacity(palette.accent, 0.6f), paint.width(overlay::hairline));

                const auto velocity = body->linear_velocity_m_s();
                const auto speed = math::length(velocity);
                const auto tip = velocity_handle_tip(camera_, centre_m, velocity, velocity_handle_scale());
                const auto velocity_state = state_of("velocity");
                // While dragged the knob is the arrow's tip under the pointer; idle, a short
                // arrow parks it clear of the centre-of-mass symbol.
                const auto knob = velocity_state == HandleState::active ? tip : velocity_knob_position(camera_, centre_m, velocity, velocity_handle_scale(), overlay_scale());
                const auto knob_radius = paint.px(velocity_state == HandleState::normal ? overlay::knob_radius : overlay::knob_radius + 1.0);
                const auto shaft = tip - centre;
                const auto shaft_length = math::length(shaft);
                const auto parked = math::length_squared(knob - tip) > 0.25;
                const auto knob_offset = knob - centre;
                const auto knob_distance = math::length(knob_offset);
                const auto direction = shaft_length > 0.0 ? shaft / shaft_length : knob_distance > 0.0 ? knob_offset / knob_distance
                                                                                                       : math::Vec2 { 0.0, 0.0 };
                // When the scene already draws this velocity arrow and its reading, the handle adds
                // only its knob rather than a second arrow and a duplicate value. Where the scene
                // caps a long arrow, the knob at the true tip hangs on a tether past the capped head.
                const auto& scales = scene_settings_.vector_scales;
                const auto density = std::clamp(static_cast<double>(scene_settings_.display_scale), 0.5, 4.0);
                const auto drawn_cap = static_cast<double>(scales.maximum_drawn_length_px) * density;
                const auto scene_arrow = scene_settings_.layers.is_enabled(render::VisualizationLayer::velocity_vectors) && !scales.automatic &&
                    shaft_length >= static_cast<double>(scales.minimum_drawn_length_px) * density;
                // The arrowhead stops at the knob's rim so that the point stays visible. A parked
                // knob hangs on a faint dashed tether from the centre of mass instead.
                if (parked || (scene_arrow && shaft_length > drawn_cap))
                {
                    const auto start = parked ? centre + direction * paint.px(7.0) : centre + direction * drawn_cap;
                    const auto end = knob - direction * (knob_radius + paint.px(1.5));
                    if (math::dot(end - start, direction) > paint.px(2.0))
                        paint.dashed_path({ start, end }, false, with_opacity(theme.velocity, 0.75f), overlay::hairline, 2.0, 2.0);
                }
                else if (!scene_arrow && shaft_length > knob_radius + paint.px(3.0))
                    list.add_arrow(centre, tip - direction * (knob_radius + paint.px(1.5)), theme.velocity, paint.width(overlay::standard), paint.width(9.0));

                paint.grip(rotation_knob, rotation_state, palette.accent);
                paint.knob(knob, velocity_state, theme.velocity);

                if (!scene_arrow && (speed > 0.005 || velocity_state != HandleState::normal))
                {
                    const auto below = direction.y <= 0.5;
                    const auto offset = knob_radius + paint.px(6.0) + paint.chip_height() * 0.5;
                    paint.chip(knob + math::Vec2 { 0.0, below ? offset : -offset }, core::format_quantity(speed, core::DisplayQuantity::velocity, units), ChipAlign::centre, palette.text);
                }
                if (rotation_state != HandleState::normal)
                {
                    const auto outward = math::normalized(rotation_knob - centre);
                    const auto anchor = rotation_knob + outward * (knob_radius + paint.px(8.0)) + math::Vec2 { 0.0, outward.y * paint.chip_height() * 0.5 };
                    const auto align = outward.x > 0.35 ? ChipAlign::left : outward.x < -0.35 ? ChipAlign::right
                                                                                              : ChipAlign::centre;
                    const auto degrees = math::radians_to_degrees(math::wrap_angle(body->orientation_rad()));
                    paint.chip(anchor, core::format_quantity(std::round(degrees), core::DisplayQuantity::angle, units), align, palette.text);
                }
            }

            // A small dial: the needle shows the direction of gravity in the force colour, and is
            // drawn muted with no arrowhead when gravity is switched off.
            const auto compass = gravity_compass_centre(camera_, overlay_scale());
            const auto radius = paint.px(overlay::compass_radius);
            const auto compass_state = state_of("gravity");
            const auto gravity_on = gravity_ && gravity_->is_enabled() && math::length_squared(world_.settings().gravity_m_s2) > 0.0;
            const auto angle = math::degrees_to_radians(gravity_direction_degrees_);
            const math::Vec2 down { std::cos(angle), -std::sin(angle) };
            if (compass_state != HandleState::normal)
                paint.halo(compass, radius + paint.px(4.0), with_opacity(palette.accent, 0.18f));
            list.add_circle_fill(compass + math::Vec2 { 0.0, paint.px(1.0) }, static_cast<float>(radius + paint.px(1.5)), with_opacity(palette.shadow, 0.6f));
            list.add_circle_fill(compass, static_cast<float>(radius), palette.plate);
            list.add_circle_outline(compass, static_cast<float>(radius), compass_state == HandleState::normal ? with_opacity(palette.muted, 0.45f) : palette.accent, paint.width(compass_state == HandleState::normal ? overlay::hairline : overlay::standard));
            for (int tick = 0; tick < 8; ++tick)
            {
                const auto tick_angle = math::pi * 0.25 * static_cast<double>(tick);
                const math::Vec2 radial { std::cos(tick_angle), std::sin(tick_angle) };
                // The label sits opposite the needle, so the ticks beside it are left out.
                if (math::dot(radial, -down) > 0.6)
                    continue;
                const auto inner = radius - paint.px(tick % 2 == 0 ? 4.0 : 2.5);
                list.add_line(compass + radial * inner, compass + radial * (radius - paint.px(1.0)), with_opacity(palette.muted, 0.55f), paint.width(overlay::hairline));
            }
            const auto needle_color = gravity_on ? theme.force : with_opacity(palette.muted, 0.8f);
            if (gravity_on)
                list.add_arrow(compass, compass + down * (radius - paint.px(4.5)), needle_color, paint.width(overlay::standard), paint.width(6.0));
            else
                list.add_line(compass, compass + down * (radius - paint.px(5.0)), needle_color, paint.width(overlay::standard));
            list.add_circle_fill(compass, paint.width(1.75), palette.text);
            const auto glyph = std::round(overlay::label_text_scale * overlay_scale() * 14.0);
            const auto label = compass - down * paint.px(8.5);
            list.add_text({ std::round(label.x - glyph * 0.28), std::round(label.y - glyph * 0.80) }, "g", palette.muted, static_cast<float>(overlay::label_text_scale * overlay_scale()));
            if (compass_state != HandleState::normal)
                paint.chip(compass + math::Vec2 { radius, radius + paint.px(8.0) + paint.chip_height() * 0.5 },
                    core::substitute("Gravity {}", core::format_quantity(std::round(gravity_direction_degrees_), core::DisplayQuantity::angle, units)),
                    ChipAlign::right,
                    palette.text);
        }

        if (interaction_.active && interaction_.mode == InteractionMode::pull)
            if (const auto* body = world_.find_body(interaction_.anchor_body))
            {
                const auto anchor_m = math::transform_point(body->interpolated_transform(alpha), interaction_.local_anchor_m);
                const auto anchor = camera_.world_to_screen(anchor_m);
                const auto target = camera_.world_to_screen(interaction_.target_world_m);
                draw_pull_spring(paint, anchor, target, theme.force);
                list.add_circle_fill(anchor, paint.width(3.0), palette.surface);
                list.add_circle_outline(anchor, paint.width(3.0), theme.force, paint.width(overlay::standard));
                paint.reticle(target, theme.force);
                // The reading sits beyond the reticle, away from the body and its own labels.
                const auto reach = target - anchor;
                if (math::length(reach) > paint.px(24.0))
                {
                    const auto outward = math::normalized(reach);
                    const auto chip_anchor = target + outward * paint.px(18.0) + math::Vec2 { 0.0, outward.y * paint.chip_height() * 0.5 };
                    const auto align = outward.x > 0.35 ? ChipAlign::left : outward.x < -0.35 ? ChipAlign::right
                                                                                              : ChipAlign::centre;
                    if (stepper_.is_paused())
                        paint.chip(chip_anchor, "Resume to pull", align, palette.muted);
                    else
                    {
                        const auto physical_anchor = math::transform_point(body->transform(), interaction_.local_anchor_m);
                        const auto force = pointer_spring_acceleration(*body, physical_anchor, interaction_.target_world_m) * body->mass_properties().mass_kg;
                        paint.chip(chip_anchor, core::format_quantity(math::length(force), core::DisplayQuantity::force, units), align, palette.text);
                    }
                }
            }

        // While throwing, the release velocity the gesture would produce is previewed as a fading
        // dotted ballistic arc that stops at the first fixed surface. Drag, spin and moving bodies
        // are ignored, so it is a guide rather than a forecast.
        if (interaction_.active && interaction_.mode == InteractionMode::throw_body && interaction_.moved && interaction_.samples.size() > 1)
            if (const auto* body = world_.find_body(interaction_.anchor_body))
            {
                const auto& first = interaction_.samples.front();
                const auto& last = interaction_.samples.back();
                const auto duration = last.time_s - first.time_s;
                const auto maximum = std::min(25.0, world_.settings().limits.maximum_linear_speed_m_s);
                const auto velocity = duration > 1.0e-6 ? limited((last.position_m - first.position_m) / duration, maximum) : math::Vec2 {};
                if (math::length(velocity) > 0.05)
                {
                    const auto gravity = gravity_ && gravity_->is_enabled() ? world_.settings().gravity_m_s2 * body->gravity_scale() : math::Vec2 {};
                    const auto& focus = camera_.focus_rect();
                    math::Aabb visible;
                    if (focus.empty())
                        visible = camera_.visible_bounds_m();
                    else
                    {
                        visible.expand(camera_.screen_to_world({ focus.left, focus.top }));
                        visible.expand(camera_.screen_to_world({ focus.left + focus.width, focus.top + focus.height }));
                    }
                    const auto start = body->world_center_of_mass_m();
                    std::vector<math::Vec2> path;
                    constexpr int samples = 48;
                    constexpr double horizon_s = 0.8;
                    path.reserve(samples + 1);
                    bool landed = false;
                    for (int index = 0; index <= samples && !landed; ++index)
                    {
                        const auto time = horizon_s * static_cast<double>(index) / samples;
                        const auto point = start + velocity * time + gravity * (0.5 * time * time);
                        if (index > 0 && !visible.contains(point))
                            break;
                        if (index > 0)
                            world_.for_each_body([&](physics::BodyId id, const physics::RigidBody& other)
                                {
                                    landed = landed || (other.type() == physics::BodyType::static_body && !is_marker(id) && other.contains_world_point(point));
                                });
                        path.push_back(camera_.world_to_screen(point));
                    }
                    if (path.size() >= 2)
                    {
                        const auto color = theme.trajectory.with_alpha(1.0f);
                        paint.tapered_dashes(path, color, 2.0, 1.25, 0.25f, 2.0, 4.0);
                        const auto end = path.back();
                        list.add_circle_outline(end, paint.width(4.0), with_opacity(color, landed ? 0.85f : 0.4f), paint.width(overlay::standard));
                        if (landed)
                            list.add_circle_fill(end, paint.width(1.5), with_opacity(color, 0.85f));
                        const auto inward = focus.empty() || end.x < focus.left + focus.width * 0.5;
                        paint.chip(end + math::Vec2 { inward ? paint.px(10.0) : -paint.px(10.0), -paint.px(14.0) }, core::format_quantity(math::length(velocity), core::DisplayQuantity::velocity, units), inward ? ChipAlign::left : ChipAlign::right, palette.muted);
                    }
                }
            }
        list.set_layer(previous_layer);
    }
}
