#include <rigidbodies/physics/contact_solver.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>

namespace rigidbodies::physics
{
    namespace
    {
        Real nonnegative(Real value)
        {
            return math::is_finite(value) ? std::max(value, 0.0) : 0.0;
        }

        math::Vec2 relative_velocity(const RigidBody& first, const RigidBody& second,
            const math::Vec2& first_radius, const math::Vec2& second_radius)
        {
            return second.linear_velocity_m_s() + math::cross(second.angular_velocity_rad_s(), second_radius) -
                first.linear_velocity_m_s() - math::cross(first.angular_velocity_rad_s(), first_radius);
        }

        void apply_impulse(RigidBody& first, RigidBody& second, const math::Vec2& first_radius,
            const math::Vec2& second_radius, const math::Vec2& impulse, ContactManifold& manifold, bool friction)
        {
            const auto kinetic = [](const RigidBody& body)
            {
                return body.type() == BodyType::dynamic_body ? body.kinetic_energy_j() : 0.0;
            };
            const auto before = kinetic(first) + kinetic(second);
            Real external = 0.0;
            if (first.type() == BodyType::kinematic_body)
                external += math::dot(impulse, first.linear_velocity_m_s() + math::cross(first.angular_velocity_rad_s(), first_radius));
            if (second.type() == BodyType::kinematic_body)
                external -= math::dot(impulse, second.linear_velocity_m_s() + math::cross(second.angular_velocity_rad_s(), second_radius));
            first.set_simulated_velocity(first.linear_velocity_m_s() - impulse * first.inverse_mass(),
                first.angular_velocity_rad_s() - first.inverse_inertia() * math::cross(first_radius, impulse));
            second.set_simulated_velocity(second.linear_velocity_m_s() + impulse * second.inverse_mass(),
                second.angular_velocity_rad_s() + second.inverse_inertia() * math::cross(second_radius, impulse));
            auto& energy = manifold.energy;
            (friction ? energy.friction_energy_change_j : energy.normal_energy_change_j) += kinetic(first) + kinetic(second) - before;
            (friction ? energy.friction_external_work_j : energy.normal_external_work_j) += external;
            manifold.applied_impulse_on_second_n_s += impulse;
            manifold.applied_angular_impulse_first_n_m_s -= math::cross(first_radius, impulse);
            manifold.applied_angular_impulse_second_n_m_s += math::cross(second_radius, impulse);
            (friction ? manifold.applied_tangent_impulse_n_s : manifold.applied_normal_impulse_n_s) += math::dot(impulse, friction ? math::perpendicular(manifold.normal) : manifold.normal);
            if (!friction)
            {
                manifold.applied_normal_angular_impulse_first_n_m_s -= math::cross(first_radius, impulse);
                manifold.applied_normal_angular_impulse_second_n_m_s += math::cross(second_radius, impulse);
            }
        }

        void apply_angular_impulse(RigidBody& first, RigidBody& second, Real impulse, ContactManifold& manifold)
        {
            const auto kinetic = [](const RigidBody& body)
            {
                return body.type() == BodyType::dynamic_body ? body.rotational_kinetic_energy_j() : 0.0;
            };
            const auto before = kinetic(first) + kinetic(second);
            if (first.type() == BodyType::kinematic_body)
                manifold.energy.friction_external_work_j += impulse * first.angular_velocity_rad_s();
            if (second.type() == BodyType::kinematic_body)
                manifold.energy.friction_external_work_j -= impulse * second.angular_velocity_rad_s();
            first.set_simulated_velocity(first.linear_velocity_m_s(), first.angular_velocity_rad_s() - first.inverse_inertia() * impulse);
            second.set_simulated_velocity(second.linear_velocity_m_s(), second.angular_velocity_rad_s() + second.inverse_inertia() * impulse);
            manifold.energy.friction_energy_change_j += kinetic(first) + kinetic(second) - before;
            manifold.applied_angular_impulse_first_n_m_s -= impulse;
            manifold.applied_angular_impulse_second_n_m_s += impulse;
        }

        Real inverse_effective_mass(const RigidBody& first, const RigidBody& second,
            const math::Vec2& first_radius, const math::Vec2& second_radius, const math::Vec2& direction)
        {
            const auto first_lever = math::cross(first_radius, direction);
            const auto second_lever = math::cross(second_radius, direction);
            return first.inverse_mass() + second.inverse_mass() + first.inverse_inertia() * first_lever * first_lever +
                second.inverse_inertia() * second_lever * second_lever;
        }

        Real directional_scale(const DirectionalFriction& material, const RigidBody& body, std::size_t collider_index, const math::Vec2& tangent)
        {
            if (!math::is_finite(material.axis_local) || math::length_squared(material.axis_local) <= math::geometric_epsilon * math::geometric_epsilon)
                return 1.0;
            auto axis = math::normalized(material.axis_local);
            if (collider_index < body.colliders().size())
                axis = math::rotate(body.colliders()[collider_index].local_transform.rotation, axis);
            axis = math::rotate(body.transform().rotation, axis);
            const auto parallel_squared = math::clamp(math::dot(axis, tangent) * math::dot(axis, tangent), 0.0, 1.0);
            const auto ratio = nonnegative(material.ratio);
            if (ratio == 0.0)
                return parallel_squared <= 1.0e-24 ? 1.0 : 0.0;
            return 1.0 / std::hypot(std::sqrt(parallel_squared) / ratio, std::sqrt(1.0 - parallel_squared));
        }

        bool moving_kinematic(const RigidBody& body)
        {
            return body.type() == BodyType::kinematic_body &&
                (math::length_squared(body.linear_velocity_m_s()) > 0.0 || body.angular_velocity_rad_s() != 0.0);
        }

        void move_by_position_impulse(RigidBody& body, const math::Vec2& radius, const math::Vec2& impulse)
        {
            if (body.type() != BodyType::dynamic_body || !body.is_awake())
                return;
            const auto center = body.world_center_of_mass_m() + impulse * body.inverse_mass();
            const auto angle = body.orientation_rad() + body.inverse_inertia() * math::cross(radius, impulse);
            body.set_simulated_pose(center - math::rotate(math::Rotation2 { angle }, body.mass_properties().center_of_mass_m), angle);
        }
    }

    std::string_view SequentialImpulseContactSolver::name() const
    {
        return "sequential_impulse";
    }

    void SequentialImpulseContactSolver::prepare(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real dt)
    {
        prepared_.clear();
        prepared_.resize(manifolds.size());
        for (auto& manifold : manifolds)
        {
            manifold.energy = {};
            manifold.applied_impulse_on_second_n_s = {};
            manifold.applied_angular_impulse_first_n_m_s = manifold.applied_angular_impulse_second_n_m_s = 0.0;
            manifold.applied_normal_impulse_n_s = manifold.applied_tangent_impulse_n_s = 0.0;
            manifold.applied_normal_angular_impulse_first_n_m_s = manifold.applied_normal_angular_impulse_second_n_m_s = 0.0;
        }
        if (!math::is_finite(dt) || dt <= 0.0)
            return;

        // First measure every incoming velocity. Warming one manifold must not change the
        // restitution threshold or target of a later manifold in the same contact island.
        for (std::size_t index = 0; index < manifolds.size(); ++index)
        {
            auto& manifold = manifolds[index];
            auto& prepared = prepared_[index];
            auto* first = world.find_body(manifold.first);
            auto* second = world.find_body(manifold.second);
            if (!first || !second || first == second || manifold.is_sensor || manifold.point_count == 0 ||
                !math::is_finite(manifold.normal) || math::length_squared(manifold.normal) <= math::geometric_epsilon * math::geometric_epsilon)
                continue;
            const auto awake_dynamic = [](const RigidBody& body)
            {
                return body.type() == BodyType::dynamic_body && body.is_awake();
            };
            if (!awake_dynamic(*first) && !awake_dynamic(*second) && !moving_kinematic(*first) && !moving_kinematic(*second))
                continue; // Keep sleeping contact impulses available without applying them.
            if (first->type() == BodyType::dynamic_body && !first->is_awake())
                first->wake();
            if (second->type() == BodyType::dynamic_body && !second->is_awake())
                second->wake();
            if (first->inverse_mass() + second->inverse_mass() + first->inverse_inertia() + second->inverse_inertia() <= 0.0)
                continue;

            prepared.first = manifold.first;
            prepared.second = manifold.second;
            prepared.active = true;
            prepared.normal = math::normalized(manifold.normal);
            manifold.normal = prepared.normal;
            prepared.tangent = math::perpendicular(prepared.normal);
            prepared.point_count = std::min(manifold.point_count, maximum_manifold_points);
            prepared.static_friction = nonnegative(manifold.material.static_friction);
            prepared.kinetic_friction = std::min(nonnegative(manifold.material.kinetic_friction), prepared.static_friction);
            if (manifold.material.first_direction.ratio != 1.0 || manifold.material.second_direction.ratio != 1.0)
            {
                const auto first_scale = directional_scale(manifold.material.first_direction, *first, manifold.first_collider, prepared.tangent);
                const auto second_scale = directional_scale(manifold.material.second_direction, *second, manifold.second_collider, prepared.tangent);
                prepared.static_friction = mix_material_values(manifold.material.first_direction.static_friction * first_scale,
                    manifold.material.second_direction.static_friction * second_scale,
                    manifold.material.friction_mixing);
                prepared.kinetic_friction = std::min(prepared.static_friction, mix_material_values(manifold.material.first_direction.kinetic_friction * first_scale, manifold.material.second_direction.kinetic_friction * second_scale, manifold.material.friction_mixing));
            }
            prepared.rolling_friction_m = nonnegative(manifold.material.rolling_friction_m);
            prepared.spinning_friction_m = nonnegative(manifold.material.spinning_friction_m);

            for (std::size_t point_index = 0; point_index < prepared.point_count; ++point_index)
            {
                auto& point = manifold.points[point_index];
                auto& cached = prepared.points[point_index];
                if (!math::is_finite(point.world_position_m) || !math::is_finite(point.separation_m))
                {
                    prepared.active = false;
                    break;
                }
                const auto first_witness = point.body_anchors_valid ? math::transform_point(first->transform(), point.local_anchor_first_m) : point.world_position_m - prepared.normal * (0.5 * point.separation_m);
                const auto second_witness = point.body_anchors_valid ? math::transform_point(second->transform(), point.local_anchor_second_m) : point.world_position_m + prepared.normal * (0.5 * point.separation_m);
                if (!math::is_finite(first_witness) || !math::is_finite(second_witness))
                {
                    prepared.active = false;
                    break;
                }
                if (!point.body_anchors_valid)
                {
                    point.local_anchor_first_m = math::inverse_transform_point(first->transform(), first_witness);
                    point.local_anchor_second_m = math::inverse_transform_point(second->transform(), second_witness);
                    point.body_anchors_valid = true;
                }
                // Equal and opposite velocity impulses act at the same midpoint, preserving
                // angular momentum during penetration. A speculative future contact instead
                // uses its transported feature witnesses, avoiding artificial lever arms caused
                // by tangential travel before the two features actually meet.
                const auto separated = point.separation_m > math::geometric_epsilon;
                const auto first_velocity_point = separated ? first_witness : point.world_position_m;
                const auto second_velocity_point = separated ? second_witness : point.world_position_m;
                cached.radius_first_m = first_velocity_point - first->world_center_of_mass_m();
                cached.radius_second_m = second_velocity_point - second->world_center_of_mass_m();
                const auto normal_k = inverse_effective_mass(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.normal);
                const auto tangent_k = inverse_effective_mass(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.tangent);
                cached.normal_mass = normal_k > 0.0 ? 1.0 / normal_k : 0.0;
                cached.tangent_mass = tangent_k > 0.0 ? 1.0 / tangent_k : 0.0;
                point.pre_solve_normal_velocity_m_s = math::dot(relative_velocity(*first, *second, cached.radius_first_m, cached.radius_second_m), prepared.normal);
                cached.incoming_impact_speed_m_s = std::max(nonnegative(point.pending_impact_speed_m_s), std::max(-point.pre_solve_normal_velocity_m_s, 0.0));
                cached.friction_enabled = !separated;
                if (separated)
                {
                    const auto bias = math::is_finite(point.speculative_velocity_bias_m_s) ? point.speculative_velocity_bias_m_s : 0.0;
                    cached.target_normal_velocity_m_s = -point.separation_m / dt + bias;
                }
                else if (cached.incoming_impact_speed_m_s > nonnegative(settings.restitution_threshold_m_s))
                {
                    cached.target_normal_velocity_m_s = math::clamp(nonnegative(manifold.material.restitution), 0.0, 1.0) * cached.incoming_impact_speed_m_s;
                }
                point.pending_impact_speed_m_s = 0.0;
                if (!settings.warm_starting)
                {
                    point.normal_impulse_n_s = 0.0;
                    point.tangent_impulse_n_s = 0.0;
                    point.rolling_impulse_n_m_s = 0.0;
                    point.spinning_impulse_n_m_s = 0.0;
                }
                point.normal_impulse_n_s = nonnegative(point.normal_impulse_n_s);
                const auto tangent_bound = cached.friction_enabled ? prepared.static_friction * point.normal_impulse_n_s : 0.0;
                point.tangent_impulse_n_s = math::is_finite(point.tangent_impulse_n_s) ? math::clamp(point.tangent_impulse_n_s, -tangent_bound, tangent_bound) : 0.0;
                const auto rolling_bound = cached.friction_enabled ? prepared.rolling_friction_m * point.normal_impulse_n_s : 0.0;
                const auto spinning_bound = cached.friction_enabled ? prepared.spinning_friction_m * point.normal_impulse_n_s : 0.0;
                point.rolling_impulse_n_m_s = math::is_finite(point.rolling_impulse_n_m_s) ? math::clamp(point.rolling_impulse_n_m_s, -rolling_bound, rolling_bound) : 0.0;
                point.spinning_impulse_n_m_s = math::is_finite(point.spinning_impulse_n_m_s) ? math::clamp(point.spinning_impulse_n_m_s, -spinning_bound, spinning_bound) : 0.0;
            }
            if (prepared.point_count == 2 && prepared.active)
            {
                const auto& a = prepared.points[0];
                const auto& b = prepared.points[1];
                prepared.k11 = a.normal_mass > 0.0 ? 1.0 / a.normal_mass : 0.0;
                prepared.k22 = b.normal_mass > 0.0 ? 1.0 / b.normal_mass : 0.0;
                prepared.k12 = first->inverse_mass() + second->inverse_mass() +
                    first->inverse_inertia() * math::cross(a.radius_first_m, prepared.normal) * math::cross(b.radius_first_m, prepared.normal) +
                    second->inverse_inertia() * math::cross(a.radius_second_m, prepared.normal) * math::cross(b.radius_second_m, prepared.normal);
                const auto determinant = prepared.k11 * prepared.k22 - prepared.k12 * prepared.k12;
                prepared.block_normal = determinant > 1.0e-6 * prepared.k11 * prepared.k22;
            }
        }
        for (std::size_t index = 0; index < prepared_.size(); ++index)
        {
            const auto& prepared = prepared_[index];
            if (!prepared.active)
                continue;
            auto* first = world.find_body(prepared.first);
            auto* second = world.find_body(prepared.second);
            for (std::size_t point_index = 0; point_index < prepared.point_count; ++point_index)
            {
                const auto& point = manifolds[index].points[point_index];
                const auto& cached = prepared.points[point_index];
                apply_impulse(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.normal * point.normal_impulse_n_s, manifolds[index], false);
                apply_impulse(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.tangent * point.tangent_impulse_n_s, manifolds[index], true);
                apply_angular_impulse(*first, *second, point.rolling_impulse_n_m_s + point.spinning_impulse_n_m_s, manifolds[index]);
            }
        }
    }

    void SequentialImpulseContactSolver::solve_velocity(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings&, Real)
    {
        for (std::size_t index = 0; index < std::min(prepared_.size(), manifolds.size()); ++index)
        {
            const auto& prepared = prepared_[index];
            if (!prepared.active)
                continue;
            auto* first = world.find_body(prepared.first);
            auto* second = world.find_body(prepared.second);
            if (!first || !second)
                continue;
            auto& manifold = manifolds[index];
            bool block_solved = false;
            if (prepared.block_normal)
            {
                const auto old_first = manifold.points[0].normal_impulse_n_s;
                const auto old_second = manifold.points[1].normal_impulse_n_s;
                const auto& a = prepared.points[0];
                const auto& b = prepared.points[1];
                const auto b1 = math::dot(relative_velocity(*first, *second, a.radius_first_m, a.radius_second_m), prepared.normal) - a.target_normal_velocity_m_s -
                    prepared.k11 * old_first - prepared.k12 * old_second;
                const auto b2 = math::dot(relative_velocity(*first, *second, b.radius_first_m, b.radius_second_m), prepared.normal) - b.target_normal_velocity_m_s -
                    prepared.k12 * old_first - prepared.k22 * old_second;
                const auto determinant = prepared.k11 * prepared.k22 - prepared.k12 * prepared.k12;
                Real x1 = (prepared.k12 * b2 - prepared.k22 * b1) / determinant;
                Real x2 = (prepared.k12 * b1 - prepared.k11 * b2) / determinant;
                block_solved = x1 >= 0.0 && x2 >= 0.0;
                if (!block_solved)
                {
                    x1 = -b1 / prepared.k11;
                    x2 = 0.0;
                    block_solved = x1 >= 0.0 && prepared.k12 * x1 + b2 >= 0.0;
                }
                if (!block_solved)
                {
                    x1 = 0.0;
                    x2 = -b2 / prepared.k22;
                    block_solved = x2 >= 0.0 && prepared.k12 * x2 + b1 >= 0.0;
                }
                if (!block_solved && b1 >= 0.0 && b2 >= 0.0)
                {
                    x1 = 0.0;
                    x2 = 0.0;
                    block_solved = true;
                }
                if (block_solved)
                {
                    apply_impulse(*first, *second, a.radius_first_m, a.radius_second_m, prepared.normal * (x1 - old_first), manifold, false);
                    apply_impulse(*first, *second, b.radius_first_m, b.radius_second_m, prepared.normal * (x2 - old_second), manifold, false);
                    manifold.points[0].normal_impulse_n_s = x1;
                    manifold.points[1].normal_impulse_n_s = x2;
                }
            }
            if (!block_solved)
            {
                for (std::size_t point_index = 0; point_index < prepared.point_count; ++point_index)
                {
                    auto& point = manifold.points[point_index];
                    const auto& cached = prepared.points[point_index];
                    const auto velocity = math::dot(relative_velocity(*first, *second, cached.radius_first_m, cached.radius_second_m), prepared.normal);
                    const auto old = point.normal_impulse_n_s;
                    point.normal_impulse_n_s = std::max(old + cached.normal_mass * (cached.target_normal_velocity_m_s - velocity), 0.0);
                    apply_impulse(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.normal * (point.normal_impulse_n_s - old), manifold, false);
                }
            }
            for (std::size_t point_index = 0; point_index < prepared.point_count; ++point_index)
            {
                auto& point = manifold.points[point_index];
                const auto& cached = prepared.points[point_index];
                if (!cached.friction_enabled)
                {
                    point.pending_impact_speed_m_s = point.normal_impulse_n_s > 0.0 ? cached.incoming_impact_speed_m_s : 0.0;
                    continue;
                }
                const auto tangent_velocity = math::dot(relative_velocity(*first, *second, cached.radius_first_m, cached.radius_second_m), prepared.tangent);
                const auto desired = point.tangent_impulse_n_s - cached.tangent_mass * tangent_velocity;
                const auto static_bound = prepared.static_friction * point.normal_impulse_n_s;
                const auto kinetic_bound = prepared.kinetic_friction * point.normal_impulse_n_s;
                const auto next = std::abs(desired) <= static_bound ? desired : math::clamp(desired, -kinetic_bound, kinetic_bound);
                apply_impulse(*first, *second, cached.radius_first_m, cached.radius_second_m, prepared.tangent * (next - point.tangent_impulse_n_s), manifold, true);
                point.tangent_impulse_n_s = next;

                const auto inverse_angular_mass = first->inverse_inertia() + second->inverse_inertia();
                if (inverse_angular_mass > 0.0)
                {
                    const auto old = point.rolling_impulse_n_m_s + point.spinning_impulse_n_m_s;
                    const auto angular_velocity = second->angular_velocity_rad_s() - first->angular_velocity_rad_s();
                    const auto rolling_bound = prepared.rolling_friction_m * point.normal_impulse_n_s;
                    const auto spinning_bound = prepared.spinning_friction_m * point.normal_impulse_n_s;
                    const auto total = math::clamp(old - angular_velocity / inverse_angular_mass, -rolling_bound - spinning_bound, rolling_bound + spinning_bound);
                    point.rolling_impulse_n_m_s = math::clamp(total, -rolling_bound, rolling_bound);
                    point.spinning_impulse_n_m_s = total - point.rolling_impulse_n_m_s;
                    apply_angular_impulse(*first, *second, total - old, manifold);
                }
            }
        }
    }

    void SequentialImpulseContactSolver::solve_position(World& world, std::vector<ContactManifold>& manifolds, const SolverSettings& settings, Real)
    {
        const auto slop = nonnegative(settings.linear_slop_m);
        const auto fraction = math::clamp(nonnegative(settings.position_correction_fraction), 0.0, 1.0);
        const auto maximum_correction = nonnegative(settings.maximum_position_correction_m);
        for (std::size_t index = 0; index < std::min(prepared_.size(), manifolds.size()); ++index)
        {
            const auto& prepared = prepared_[index];
            if (!prepared.active)
                continue;
            auto* first = world.find_body(prepared.first);
            auto* second = world.find_body(prepared.second);
            if (!first || !second)
                continue;
            for (std::size_t point_index = 0; point_index < prepared.point_count; ++point_index)
            {
                auto& point = manifolds[index].points[point_index];
                const auto first_witness = math::transform_point(first->transform(), point.local_anchor_first_m);
                const auto second_witness = math::transform_point(second->transform(), point.local_anchor_second_m);
                const auto separation = math::dot(second_witness - first_witness, prepared.normal);
                const auto correction = math::clamp(fraction * (separation + slop), -maximum_correction, 0.0);
                const auto first_radius = first_witness - first->world_center_of_mass_m();
                const auto second_radius = second_witness - second->world_center_of_mass_m();
                const auto inverse_mass = inverse_effective_mass(*first, *second, first_radius, second_radius, prepared.normal);
                if (inverse_mass > 0.0 && correction < 0.0)
                {
                    const auto impulse = prepared.normal * (-correction / inverse_mass);
                    move_by_position_impulse(*first, first_radius, -impulse);
                    move_by_position_impulse(*second, second_radius, impulse);
                }
            }
        }
    }
} // namespace rigidbodies::physics
