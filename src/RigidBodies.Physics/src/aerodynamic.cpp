#include <rigidbodies/physics/aerodynamic.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/force_generator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        // Positive abscissae and weights on [-1,1]; reflecting them gives the full rule.
        constexpr std::array<Real, 4> gauss8_x { 0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363 };
        constexpr std::array<Real, 4> gauss8_w { 0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763 };
        constexpr std::array<Real, 8> gauss16_x { 0.0950125098376374, 0.2816035507792589, 0.4580167776572274, 0.6178762444026437, 0.7554044083550030, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499 };
        constexpr std::array<Real, 8> gauss16_w { 0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767, 0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541 };

        Real magnitude(const math::Vec2& value)
        {
            return std::hypot(value.x, value.y);
        }

        void nonnegative(Real value, const char* message)
        {
            if (!math::is_finite(value) || value < 0.0)
                throw std::invalid_argument(message);
        }

        void finite_result(Real value)
        {
            if (!math::is_finite(value))
                throw std::overflow_error("aerodynamic result exceeds finite SI range");
        }

        void finite_result(const math::Vec2& value)
        {
            finite_result(value.x);
            finite_result(value.y);
        }

        void validate_context(const ForceContext& context)
        {
            nonnegative(context.air_density_kg_m3, "air density must be finite and non-negative");
            if (!math::is_finite(context.air_dynamic_viscosity_pa_s) || context.air_dynamic_viscosity_pa_s <= 0.0 ||
                !math::is_finite(context.air_velocity_m_s) || !math::is_finite(context.gravity_m_s2))
                throw std::invalid_argument("air velocity and gravity must be finite and dynamic viscosity must be positive");
        }

        void validate_transform(const math::Transform2& transform)
        {
            if (!math::is_finite(transform.translation) || !math::is_finite(transform.rotation.sine) ||
                !math::is_finite(transform.rotation.cosine))
                throw std::invalid_argument("aerodynamic outline placement must be finite");
        }

        const std::vector<math::Vec2>* polygon_outline(const Collider& collider)
        {
            if (collider.authored_part && collider.authored_part->shape)
                return &collider.authored_part->shape->render_outline;
            if (const auto* polygon = dynamic_cast<const ConvexPolygonShape*>(collider.shape.get()))
                return &polygon->vertices();
            return nullptr;
        }

        math::Transform2 aerodynamic_transform(const Collider& collider, const math::Transform2& body)
        {
            return math::concatenate(body, collider.authored_part ? collider.authored_part->local_transform : collider.local_transform);
        }

        MassProperties profile_mass(const Collider& collider)
        {
            if (collider.authored_part && collider.authored_part->shape)
            {
                const auto& outline = collider.authored_part->shape->render_outline;
                return { std::abs(math::signed_area(outline)), math::centroid(outline), math::second_moment_of_area(outline) };
            }
            return collider.shape->compute_mass_properties(1.0);
        }

        math::Vec2 outline_normal(const std::vector<math::Vec2>& vertices, std::size_t index)
        {
            return -math::perpendicular(math::normalized(vertices[(index + 1) % vertices.size()] - vertices[index]));
        }

        bool first_logical_collider(const Collider& collider, std::vector<const AuthoredPartDefinition*>& visited)
        {
            if (!collider.authored_part)
                return true;
            const auto* identity = collider.authored_part.get();
            if (std::find(visited.begin(), visited.end(), identity) != visited.end())
                return false;
            visited.push_back(identity);
            return true;
        }

        template <class Visitor>
        void sample_edge(const math::Vec2& first, const math::Vec2& second, const Visitor& visit)
        {
            const auto length = magnitude(second - first);
            if (length == 0.0)
                return;
            const auto center = first * 0.5 + second * 0.5;
            const auto half_edge = (second - first) * 0.5;
            for (std::size_t index = 0; index < gauss8_x.size(); ++index)
            {
                const auto weight = length * 0.5 * gauss8_w[index];
                visit(center + half_edge * gauss8_x[index], weight);
                visit(center - half_edge * gauss8_x[index], weight);
            }
        }

        Real rotational_integral(const Shape& shape, const std::vector<math::Vec2>* outline, const math::Transform2& transform,
            const math::Vec2& center, Real depth)
        {
            Real result = 0.0;
            const auto add = [&](const math::Vec2& point, Real length)
            {
                const auto radius = magnitude(point - center);
                result += depth * length * radius * radius * radius;
            };
            if (outline)
            {
                for (std::size_t index = 0; index < outline->size(); ++index)
                    sample_edge(math::transform_point(transform, (*outline)[index]),
                        math::transform_point(transform, (*outline)[(index + 1) % outline->size()]),
                        add);
            }
            else if (const auto* circle = dynamic_cast<const CircleShape*>(&shape))
            {
                const auto circle_center = math::transform_point(transform, circle->local_center_m());
                const auto radius = circle->radius_m();
                constexpr int sample_count = 64;
                for (int index = 0; index < sample_count; ++index)
                {
                    const auto angle = math::two_pi * (static_cast<Real>(index) + 0.5) / sample_count;
                    add(circle_center + math::Vec2 { std::cos(angle), std::sin(angle) } * radius,
                        math::two_pi * radius / sample_count);
                }
            }
            else if (const auto* segment = dynamic_cast<const SegmentShape*>(&shape))
            {
                // Both faces of the zero-thickness slab contribute to surface resistance.
                sample_edge(math::transform_point(transform, segment->start_m()), math::transform_point(transform, segment->end_m()), [&](const math::Vec2& point, Real length)
                    {
                        add(point, 2.0 * length);
                    });
            }
            else
            {
                // Plugin outlines without accessible vertices use their enclosing-circle proxy.
                const auto radius = shape.bounding_radius();
                add(transform.translation, math::two_pi * radius);
                result += depth * math::two_pi * radius * radius * radius * radius;
            }
            finite_result(result);
            return result;
        }

        Real reynolds(Real speed, Real length, const ForceContext& context)
        {
            const auto result = context.air_density_kg_m3 * speed * length / context.air_dynamic_viscosity_pa_s;
            finite_result(result);
            return result;
        }

        // Algebraically evaluate Cd0 * (rho |v| + 24 mu/L), avoiding a singular Cd at rest.
        Real resistance_factor(Real base, Real speed, Real length, const ForceContext& context,
            const AerodynamicSettings& settings)
        {
            return 0.5 * base * (context.air_density_kg_m3 * speed + (settings.reynolds_correction && length > 0.0 ? 24.0 * context.air_dynamic_viscosity_pa_s / length : 0.0));
        }

        void check_report(const AerodynamicReport& report)
        {
            finite_result(report.drag_force_n);
            finite_result(report.magnus_force_n);
            for (const auto value : { report.drag_torque_n_m, report.angular_drag_torque_n_m, report.magnus_torque_n_m, report.projected_area_m2, report.effective_drag_coefficient, report.reynolds_number, report.relative_speed_m_s })
                finite_result(value);
        }
    }

    void validate_aerodynamic_settings(const AerodynamicSettings& settings)
    {
        nonnegative(settings.angular_drag_coefficient, "angular drag coefficient must be finite and non-negative");
        nonnegative(settings.magnus_efficiency, "Magnus efficiency must be finite and non-negative");
        nonnegative(settings.maximum_lift_coefficient, "maximum lift coefficient must be finite and non-negative");
    }

    Real aerodynamic_drag_coefficient(Real base_coefficient, Real reynolds_number, const AerodynamicSettings& settings)
    {
        validate_aerodynamic_settings(settings);
        nonnegative(base_coefficient, "base drag coefficient must be finite and non-negative");
        nonnegative(reynolds_number, "Reynolds number must be finite and non-negative");
        const auto result = settings.reynolds_correction && reynolds_number > 0.0
            ? base_coefficient + (base_coefficient * 24.0) / reynolds_number
            : base_coefficient;
        finite_result(result);
        return result;
    }

    AerodynamicProfile project_aerodynamic_profile(const Collider& collider, const math::Transform2& body_transform,
        const math::Vec2& direction, const math::Vec2& world_center_of_mass_m, const AerodynamicSettings& settings)
    {
        validate_aerodynamic_settings(settings);
        nonnegative(collider.depth_m, "aerodynamic depth must be finite and non-negative");
        validate_transform(body_transform);
        validate_transform(collider.local_transform);
        if (!math::is_finite(direction) || !math::is_finite(world_center_of_mass_m))
            throw std::invalid_argument("aerodynamic direction and centre of mass must be finite");
        AerodynamicProfile result;
        const auto speed = magnitude(direction);
        finite_result(speed);
        if (!collider.shape || speed == 0.0)
            return result;
        const auto unit = direction / speed;
        const auto across = math::perpendicular(unit);
        const auto transform = aerodynamic_transform(collider, body_transform);
        const auto properties = profile_mass(collider);
        const auto* outline = polygon_outline(collider);
        result.geometric_center_m = math::transform_point(transform, properties.center_of_mass_m);
        result.center_of_pressure_m = result.geometric_center_m;
        result.equivalent_radius_m = std::sqrt(std::max(properties.mass_kg, 0.0) / math::pi);
        const auto span = [&](const math::Vec2& axis)
        {
            const auto local_axis = math::inverse_transform_direction(transform, axis);
            const auto positive = outline ? math::support_point(*outline, local_axis) : collider.shape->support_point(local_axis);
            const auto negative = outline ? math::support_point(*outline, -local_axis) : collider.shape->support_point(-local_axis);
            return std::max(0.0, math::dot(positive - negative, local_axis));
        };
        const auto width = span(across);
        result.projected_area_m2 = width * collider.depth_m;
        result.characteristic_length_m = std::max(width, span(unit));
        bool estimated = false;
        Real estimate = 0.0;
        if (!outline && dynamic_cast<const CircleShape*>(collider.shape.get()))
        {
            const auto* circle = static_cast<const CircleShape*>(collider.shape.get());
            estimated = true;
            estimate = 1.2;
            result.center_of_pressure_m += unit * (math::pi * circle->radius_m() * 0.25);
        }
        else if (outline)
        {
            estimated = true;
            Real projected_length = 0.0;
            Real bluntness = 0.0;
            math::Vec2 weighted_center;
            const auto& vertices = *outline;
            for (std::size_t index = 0; index < vertices.size(); ++index)
            {
                const auto first = math::transform_point(transform, vertices[index]);
                const auto second = math::transform_point(transform, vertices[(index + 1) % vertices.size()]);
                const auto normal = math::transform_direction(transform, outline_normal(vertices, index));
                const auto facing = std::max(0.0, math::dot(normal, unit));
                const auto contribution = facing * magnitude(second - first);
                projected_length += contribution;
                weighted_center += (first * 0.5 + second * 0.5) * contribution;
                bluntness += contribution * facing * facing;
            }
            if (projected_length > 0.0)
            {
                result.center_of_pressure_m = weighted_center / projected_length;
                estimate = 0.8 + 0.6 * bluntness / projected_length;
            }
        }
        else if (dynamic_cast<const SegmentShape*>(collider.shape.get()))
        {
            estimated = true;
            estimate = 1.8;
        }
        result.coefficient_source = collider.drag_coefficient_override ? DragCoefficientSource::collider_override
            : estimated && settings.estimate_outline_coefficient       ? DragCoefficientSource::outline
                                                                       : DragCoefficientSource::material;
        result.base_drag_coefficient = collider.drag_coefficient_override.value_or(
            estimated && settings.estimate_outline_coefficient ? estimate : collider.material.drag_coefficient);
        nonnegative(result.base_drag_coefficient, "drag coefficient must be finite and non-negative");
        result.rotational_area_m5 = rotational_integral(*collider.shape, outline, transform, world_center_of_mass_m, collider.depth_m);
        finite_result(result.projected_area_m2);
        finite_result(result.characteristic_length_m);
        finite_result(result.equivalent_radius_m);
        finite_result(result.center_of_pressure_m);
        finite_result(result.geometric_center_m);
        return result;
    }

    AerodynamicReport evaluate_aerodynamics(const RigidBody& body, const ForceContext& context, const AerodynamicSettings& settings)
    {
        validate_context(context);
        validate_aerodynamic_settings(settings);
        AerodynamicReport report;
        if (body.type() != BodyType::dynamic_body)
            return report;
        if (!math::is_finite(body.linear_velocity_m_s()) || !math::is_finite(body.angular_velocity_rad_s()))
            throw std::invalid_argument("aerodynamic body velocity must be finite");
        report.relative_speed_m_s = magnitude(body.linear_velocity_m_s() - context.air_velocity_m_s);
        finite_result(report.relative_speed_m_s);
        if (context.air_density_kg_m3 == 0.0)
            return report;
        const auto center_of_mass = body.world_center_of_mass_m();
        const auto angular_speed = body.angular_velocity_rad_s();
        std::vector<const AuthoredPartDefinition*> visited;
        for (const auto& collider : body.colliders())
        {
            if (!collider.shape || collider.is_sensor || collider.depth_m == 0.0 || !first_logical_collider(collider, visited))
                continue;
            const auto transform = aerodynamic_transform(collider, body.transform());
            const auto* outline = polygon_outline(collider);
            const auto local_center = profile_mass(collider).center_of_mass_m;
            const auto center = math::transform_point(transform, local_center);
            const auto center_velocity = body.velocity_at_world_point(center) - context.air_velocity_m_s;
            const auto center_speed = magnitude(center_velocity);
            const auto direction = center_speed > 0.0 ? center_velocity / center_speed : math::Vec2 { 1.0, 0.0 };
            const auto profile = project_aerodynamic_profile(collider, body.transform(), direction, center_of_mass, settings);
            const auto length = profile.characteristic_length_m;
            if (length <= 0.0)
                continue;
            if (center_speed > 0.0)
            {
                const auto re = reynolds(center_speed, length, context);
                report.projected_area_m2 += profile.projected_area_m2;
                report.effective_drag_coefficient += profile.projected_area_m2 * aerodynamic_drag_coefficient(profile.base_drag_coefficient, re, settings);
                report.reynolds_number += profile.projected_area_m2 * re;
            }
            const auto apply_drag_sample = [&](const math::Vec2& point, const math::Vec2& normal, Real area)
            {
                const auto velocity = body.velocity_at_world_point(point) - context.air_velocity_m_s;
                const auto speed = magnitude(velocity);
                if (speed == 0.0)
                    return;
                const auto facing = std::max(0.0, math::dot(normal, velocity / speed));
                const auto force = -velocity * (resistance_factor(profile.base_drag_coefficient, speed, length, context, settings) * area * facing);
                report.drag_force_n += force;
                report.drag_torque_n_m += math::cross(point - center_of_mass, force);
            };
            if (!outline && dynamic_cast<const CircleShape*>(collider.shape.get()))
            {
                const auto* circle = static_cast<const CircleShape*>(collider.shape.get());
                if (center_speed > 0.0)
                {
                    // Own-centre rotation has zero normal velocity, so exactly one semicircle
                    // faces the centre's relative flow even while the disk spins rapidly.
                    const auto angle = std::atan2(direction.y, direction.x);
                    const auto radius = circle->radius_m();
                    for (std::size_t index = 0; index < gauss16_x.size(); ++index)
                        for (const auto sign : { -1.0, 1.0 })
                        {
                            const auto sample_angle = angle + sign * math::pi * 0.5 * gauss16_x[index];
                            const math::Vec2 normal { std::cos(sample_angle), std::sin(sample_angle) };
                            apply_drag_sample(center + normal * radius, normal, collider.depth_m * radius * math::pi * 0.5 * gauss16_w[index]);
                        }
                }
            }
            else if (outline)
            {
                const auto& vertices = *outline;
                Real exposed_length = 0.0;
                if (collider.authored_part)
                    for (std::size_t index = 0; index < vertices.size(); ++index)
                        exposed_length += std::max(0.0, math::dot(math::transform_direction(transform, outline_normal(vertices, index)), direction)) *
                            magnitude(vertices[(index + 1) % vertices.size()] - vertices[index]);
                // Re-entrant boundaries can face the flow more than once along a single ray.
                // Normalize their pressure quadrature to the frontal span, retaining the model's
                // terminal balance while avoiding both collision seams and double-counted area.
                const auto pressure_scale = collider.authored_part && exposed_length > 0.0
                    ? profile.projected_area_m2 / (collider.depth_m * exposed_length)
                    : 1.0;
                for (std::size_t index = 0; index < vertices.size(); ++index)
                {
                    const auto normal = math::transform_direction(transform, outline_normal(vertices, index));
                    sample_edge(math::transform_point(transform, vertices[index]), math::transform_point(transform, vertices[(index + 1) % vertices.size()]), [&](const math::Vec2& point, Real edge_length)
                        {
                            apply_drag_sample(point, normal, edge_length * collider.depth_m * pressure_scale);
                        });
                }
            }
            else if (const auto* segment = dynamic_cast<const SegmentShape*>(collider.shape.get()))
            {
                const auto first = math::transform_point(transform, segment->start_m());
                const auto second = math::transform_point(transform, segment->end_m());
                const auto edge_length = magnitude(second - first);
                if (edge_length > 0.0)
                {
                    const auto normal = math::perpendicular((second - first) / edge_length);
                    sample_edge(first, second, [&](const math::Vec2& point, Real weight)
                        {
                            apply_drag_sample(point, normal, weight * collider.depth_m);
                            apply_drag_sample(point, -normal, weight * collider.depth_m);
                        });
                }
            }
            else if (center_speed > 0.0)
            {
                const auto force = -center_velocity * (resistance_factor(profile.base_drag_coefficient, center_speed, length, context, settings) * profile.projected_area_m2);
                report.drag_force_n += force;
                report.drag_torque_n_m += math::cross(center - center_of_mass, force);
            }

            if (settings.angular_drag)
                report.angular_drag_torque_n_m -= 0.5 * context.air_density_kg_m3 * profile.base_drag_coefficient * settings.angular_drag_coefficient *
                    profile.rotational_area_m5 * angular_speed * std::abs(angular_speed);
            if (settings.magnus_lift && center_speed > 0.0 && profile.equivalent_radius_m > 0.0)
            {
                const auto circulation = settings.magnus_efficiency * math::two_pi * profile.equivalent_radius_m * profile.equivalent_radius_m * angular_speed;
                const auto lift_factor = context.air_density_kg_m3 * circulation * collider.depth_m;
                const auto maximum_factor = 0.5 * context.air_density_kg_m3 * center_speed * profile.projected_area_m2 * settings.maximum_lift_coefficient;
                const auto lift = math::perpendicular(center_velocity) * math::clamp(lift_factor, -maximum_factor, maximum_factor);
                report.magnus_force_n += lift;
                report.magnus_torque_n_m += math::cross(center - center_of_mass, lift);
            }
        }
        if (report.projected_area_m2 > 0.0)
        {
            report.effective_drag_coefficient /= report.projected_area_m2;
            report.reynolds_number /= report.projected_area_m2;
        }
        check_report(report);
        return report;
    }

    std::optional<TerminalSpeedEstimate> estimate_terminal_speed(const RigidBody& body, const ForceContext& context,
        const AerodynamicSettings& settings)
    {
        validate_context(context);
        validate_aerodynamic_settings(settings);
        if (body.type() != BodyType::dynamic_body || body.mass_properties().mass_kg <= 0.0)
            return std::nullopt;
        const auto acceleration = context.gravity_m_s2 * body.gravity_scale();
        const auto gravity = magnitude(acceleration);
        TerminalSpeedEstimate result;
        result.world_velocity_m_s = context.air_velocity_m_s;
        result.weight_n = body.mass_properties().mass_kg * gravity;
        finite_result(result.weight_n);
        if (gravity == 0.0)
            return result;
        if (context.air_density_kg_m3 == 0.0)
            return std::nullopt;
        const auto direction = acceleration / gravity;
        Real quadratic = 0.0;
        Real linear = 0.0;
        std::vector<const AuthoredPartDefinition*> visited;
        for (const auto& collider : body.colliders())
        {
            if (!collider.shape || collider.is_sensor || !first_logical_collider(collider, visited))
                continue;
            const auto profile = project_aerodynamic_profile(collider, body.transform(), direction, body.world_center_of_mass_m(), settings);
            result.projected_area_m2 += profile.projected_area_m2;
            const auto area_coefficient = profile.projected_area_m2 * profile.base_drag_coefficient;
            quadratic += 0.5 * context.air_density_kg_m3 * area_coefficient;
            if (settings.reynolds_correction && profile.characteristic_length_m > 0.0)
                linear += 12.0 * context.air_dynamic_viscosity_pa_s * area_coefficient / profile.characteristic_length_m;
        }
        finite_result(quadratic);
        finite_result(linear);
        if (quadratic <= 0.0 && linear <= 0.0)
            return std::nullopt;
        // Positive quadratic root without subtractive cancellation or squaring a large weight.
        const auto discriminant_root = std::hypot(linear, 2.0 * std::sqrt(quadratic) * std::sqrt(result.weight_n));
        result.speed_m_s = result.weight_n / (0.5 * linear + 0.5 * discriminant_root);
        result.relative_velocity_m_s = direction * result.speed_m_s;
        result.world_velocity_m_s += result.relative_velocity_m_s;
        result.drag_force_n = quadratic * result.speed_m_s * result.speed_m_s + linear * result.speed_m_s;
        finite_result(result.speed_m_s);
        finite_result(result.world_velocity_m_s);
        finite_result(result.drag_force_n);
        finite_result(result.projected_area_m2);
        return result;
    }
}
