#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace rigidbodies::physics
{
    namespace
    {
        bool changed(const math::Vec2& a, const math::Vec2& b)
        {
            return a.x != b.x || a.y != b.y;
        }

        math::Vec2 bounded_speed(const math::Vec2& velocity, Real maximum)
        {
            if (!math::is_finite(velocity))
                return {};
            const auto scale = std::max(std::abs(velocity.x), std::abs(velocity.y));
            if (scale == 0.0)
                return velocity;
            const auto unit = velocity / scale;
            const auto length = std::hypot(unit.x, unit.y);
            if (scale <= maximum / length)
                return velocity;
            return unit * (maximum / length);
        }

        bool path_moves(const KinematicMotion& motion)
        {
            return std::visit([](const auto& path)
                {
                    using Path = std::decay_t<decltype(path)>;
                    if constexpr (std::is_same_v<Path, LinearMotion>)
                    {
                        return path.velocity_m_s.x != 0.0 || path.velocity_m_s.y != 0.0 || path.angular_velocity_rad_s != 0.0;
                    }
                    else if constexpr (std::is_same_v<Path, HarmonicMotion>)
                    {
                        return path.frequency_hz > 0.0 && (path.translation_amplitude_m.x != 0.0 || path.translation_amplitude_m.y != 0.0 || path.rotation_amplitude_rad != 0.0);
                    }
                    else
                    {
                        return path.angular_speed_rad_s != 0.0 && (path.radius_m > 0.0 || path.orient_to_path);
                    }
                },
                motion.definition());
        }
    }

    bool World::set_kinematic_motion(BodyId id, const KinematicMotion& motion)
    {
        auto* body = find_body(id);
        if (!body || body->type() != BodyType::kinematic_body)
            return false;
        KinematicState state;
        try
        {
            state = motion.sample(0.0);
        }
        catch (const std::exception&)
        {
            report_motion_limit(id, MotionLimitKind::kinematic_path);
            return false;
        }
        if (!accept_kinematic_state(id, state))
            return false;
        auto& slot = slots_[id.index];
        slot.motion = motion;
        slot.motion_start_time_s = statistics_.elapsed_time_s;
        body->set_simulated_pose(state.position_m - math::rotate(math::Rotation2 { state.orientation_rad }, body->mass_properties().center_of_mass_m), state.orientation_rad);
        body->set_simulated_velocity(state.linear_velocity_m_s, state.angular_velocity_rad_s);
        body->motion_edited_ = true;
        body->capture_previous_transform();
        return true;
    }

    bool World::accept_kinematic_state(BodyId id, const KinematicState& state)
    {
        const auto* body = find_body(id);
        if (!body)
            return false;
        const auto& limits = settings_.limits;
        const auto orientation_valid = math::is_finite(state.orientation_rad) &&
            std::abs(state.orientation_rad) <= limits.maximum_orientation_rad;
        auto position_valid = math::is_finite(state.position_m);
        if (position_valid && math::is_finite(state.orientation_rad))
        {
            const auto position = state.position_m -
                math::rotate(math::Rotation2 { state.orientation_rad }, body->mass_properties().center_of_mass_m);
            position_valid = math::is_finite(position) && std::abs(position.x) <= limits.maximum_position_m &&
                std::abs(position.y) <= limits.maximum_position_m;
        }
        const auto velocity_valid = math::is_finite(state.linear_velocity_m_s) &&
            std::hypot(state.linear_velocity_m_s.x, state.linear_velocity_m_s.y) <= limits.maximum_linear_speed_m_s;
        const auto angular_valid = math::is_finite(state.angular_velocity_rad_s) &&
            std::abs(state.angular_velocity_rad_s) <= limits.maximum_angular_speed_rad_s;
        if (!position_valid)
            report_motion_limit(id, MotionLimitKind::position);
        if (!orientation_valid)
            report_motion_limit(id, MotionLimitKind::orientation);
        if (!velocity_valid)
            report_motion_limit(id, MotionLimitKind::linear_velocity);
        if (!angular_valid)
            report_motion_limit(id, MotionLimitKind::angular_velocity);
        const auto accepted = position_valid && orientation_valid && velocity_valid && angular_valid;
        if (!accepted)
            report_motion_limit(id, MotionLimitKind::kinematic_path);
        return accepted;
    }

    bool World::clear_kinematic_motion(BodyId id)
    {
        if (!is_valid(id) || !slots_[id.index].motion)
            return false;
        slots_[id.index].motion.reset();
        return true;
    }

    const KinematicMotion* World::kinematic_motion(BodyId id) const
    {
        return is_valid(id) && slots_[id.index].motion ? &*slots_[id.index].motion : nullptr;
    }

    const std::vector<MotionLimitEvent>& World::motion_limit_events() const
    {
        return motion_limit_events_;
    }

    void World::report_motion_limit(BodyId id, MotionLimitKind kind)
    {
        for (const auto& event : motion_limit_events_)
        {
            if (event.body == id && event.kind == kind)
                return;
        }
        motion_limit_events_.push_back({ id, kind });
        if (statistics_.limit_event_count < std::numeric_limits<std::uint64_t>::max())
            ++statistics_.limit_event_count;
        statistics_.last_step_limit_event_count = motion_limit_events_.size();
    }

    void World::enforce_motion_limits()
    {
        const auto& limits = settings_.limits;
        for_each_body([&](BodyId id, RigidBody& body)
            {
                const auto prior_position = body.position_m();
                auto position = prior_position;
                if (!math::is_finite(position))
                {
                    position = math::is_finite(body.previous_transform_.translation) ? body.previous_transform_.translation : math::Vec2 {};
                }
                position.x = math::clamp(position.x, -limits.maximum_position_m, limits.maximum_position_m);
                position.y = math::clamp(position.y, -limits.maximum_position_m, limits.maximum_position_m);
                auto orientation = body.orientation_rad();
                if (!math::is_finite(orientation))
                    orientation = math::is_finite(body.previous_orientation_rad_) ? body.previous_orientation_rad_ : 0.0;
                orientation = math::clamp(orientation, -limits.maximum_orientation_rad, limits.maximum_orientation_rad);
                const auto pose_changed = changed(position, prior_position) || orientation != body.orientation_rad();
                if (changed(position, prior_position))
                    report_motion_limit(id, MotionLimitKind::position);
                if (orientation != body.orientation_rad())
                    report_motion_limit(id, MotionLimitKind::orientation);
                if (pose_changed)
                {
                    body.set_simulated_pose(position, orientation);
                    body.capture_previous_transform();
                    body.wake();
                }

                const auto velocity = bounded_speed(body.linear_velocity_m_s(), limits.maximum_linear_speed_m_s);
                const auto angular = math::is_finite(body.angular_velocity_rad_s())
                    ? math::clamp(body.angular_velocity_rad_s(), -limits.maximum_angular_speed_rad_s, limits.maximum_angular_speed_rad_s)
                    : 0.0;
                if (changed(velocity, body.linear_velocity_m_s()))
                    report_motion_limit(id, MotionLimitKind::linear_velocity);
                if (angular != body.angular_velocity_rad_s())
                    report_motion_limit(id, MotionLimitKind::angular_velocity);
                body.set_simulated_velocity(velocity, angular);

                if (!math::is_finite(body.accumulated_force_n()) || !math::is_finite(body.accumulated_torque_n_m()))
                {
                    body.clear_accumulators();
                    report_motion_limit(id, MotionLimitKind::non_finite_load);
                }
            });
    }

    void World::set_solver_update(bool value)
    {
        for_each_body([&](BodyId, RigidBody& body)
            {
                body.solver_update_ = value;
            });
    }

    void World::wake_connected_bodies(Real time_step_s)
    {
        const auto ids = body_ids();
        if (std::none_of(ids.begin(), ids.end(), [&](BodyId id)
                {
                    const auto& body = *find_body(id);
                    return body.type() == BodyType::dynamic_body && !body.is_awake();
                }))
            return;
        const auto active = [&](BodyId id, const RigidBody& body)
        {
            const auto& motion = slots_[id.index].motion;
            if (body.type() == BodyType::kinematic_body && motion && path_moves(*motion))
                return true;
            return body.motion_edited_ || (body.is_awake() && (math::length(body.linear_velocity_m_s()) > settings_.sleep.linear_speed_m_s || std::abs(body.angular_velocity_rad_s()) > settings_.sleep.angular_speed_rad_s || math::length(body.accumulated_force_n()) * body.inverse_mass() > settings_.sleep.linear_acceleration_m_s2 || std::abs(body.accumulated_torque_n_m()) * body.inverse_inertia() > settings_.sleep.angular_acceleration_rad_s2));
        };
        const auto bounds = [&](BodyId id, const RigidBody& body, const Collider& collider)
        {
            auto swept = collider.compute_bounds(body.transform());
            auto destination = body.transform();
            const auto velocity = bounded_speed(body.linear_velocity_m_s() + body.accumulated_force_n() * body.inverse_mass() * time_step_s,
                settings_.limits.maximum_linear_speed_m_s);
            const auto maximum_position = settings_.limits.maximum_position_m;
            destination.translation = {
                math::clamp(std::fma(velocity.x, time_step_s, destination.translation.x), -maximum_position, maximum_position),
                math::clamp(std::fma(velocity.y, time_step_s, destination.translation.y), -maximum_position, maximum_position)
            };
            swept.expand(collider.compute_bounds(destination));
            const auto& slot = slots_[id.index];
            if (slot.motion && body.type() == BodyType::kinematic_body)
            {
                try
                {
                    const auto end = slot.motion->sample(statistics_.elapsed_time_s + time_step_s - slot.motion_start_time_s);
                    const auto pose = math::Transform2::from_angle(end.position_m - math::rotate(math::Rotation2 { end.orientation_rad }, body.mass_properties().center_of_mass_m), end.orientation_rad);
                    swept.expand(collider.compute_bounds(pose));
                    // A curved path can leave the chord between its endpoints, including when
                    // both endpoints coincide after a complete period. Bound its full periodic
                    // envelope conservatively so sleepers on that arc are never missed.
                    const auto reach = collider.shape->bounding_radius() +
                        math::length(collider.local_transform.translation - body.mass_properties().center_of_mass_m);
                    std::visit([&](const auto& path)
                        {
                            using Path = std::decay_t<decltype(path)>;
                            math::Aabb envelope;
                            if constexpr (std::is_same_v<Path, HarmonicMotion>)
                            {
                                const math::Vec2 amplitude { std::abs(path.translation_amplitude_m.x), std::abs(path.translation_amplitude_m.y) };
                                envelope.expand(path.origin_m - amplitude);
                                envelope.expand(path.origin_m + amplitude);
                            }
                            else if constexpr (std::is_same_v<Path, CircularMotion>)
                            {
                                envelope.expand(path.center_m - math::Vec2 { path.radius_m, path.radius_m });
                                envelope.expand(path.center_m + math::Vec2 { path.radius_m, path.radius_m });
                            }
                            envelope.grow(reach);
                            swept.expand(envelope);
                        },
                        slot.motion->definition());
                }
                catch (const std::exception&)
                { /* The step reports and stops an invalid drive. */
                }
            }
            // Rotation sweeps are bounded by the collider's enclosing radius.
            const auto angle = std::abs(body.angular_velocity_rad_s()) * time_step_s;
            if (angle > 0.0 && collider.shape)
            {
                swept.grow(std::min(angle, 2.0) * (collider.shape->bounding_radius() + math::length(collider.local_transform.translation - body.mass_properties().center_of_mass_m)));
            }
            swept.grow(settings_.broad_phase_margin_m);
            return swept;
        };

        // Build the geometric candidate graph once. A spatial tree excludes distant pairs
        // before their endpoints are connected, instead of repeating all-pairs scans for each
        // propagation hop through a sleeping stack or chain.
        std::vector<BroadPhaseProxy> wake_proxies;
        for (const auto id : ids)
        {
            const auto& body = *find_body(id);
            const auto& colliders = body.colliders();
            for (std::size_t index = 0; index < colliders.size(); ++index)
            {
                const auto& collider = colliders[index];
                if (!collider.shape || collider.is_sensor)
                    continue;
                BroadPhaseProxy proxy;
                proxy.body = id;
                proxy.collider_index = index;
                proxy.bounds = bounds(id, body, collider);
                proxy.filter = collider.filter;
                proxy.body_type = body.type();
                wake_proxies.push_back(proxy);
            }
        }
        std::vector<BroadPhasePair> candidates;
        DynamicTreeBroadPhase {}.find_pairs(wake_proxies, candidates);
        std::vector<std::vector<BodyId>> neighbors(slots_.size());
        const auto connect = [&](BodyId first, BodyId second)
        {
            if (first == second || !is_valid(first) || !is_valid(second))
                return;
            neighbors[first.index].push_back(second);
            neighbors[second.index].push_back(first);
        };
        for (const auto& pair : candidates)
            if (bodies_can_collide(pair.first, pair.second))
                connect(pair.first, pair.second);
        // Teleports reset interpolation history, so new bounds cannot reveal a support that
        // moved away. Previous solid contacts identify the bodies that have lost support.
        for (const auto& manifold : manifolds_)
            if (!manifold.is_sensor && !manifold.is_empty())
                connect(manifold.first, manifold.second);
        for (const auto& constraint : constraints_)
            if (constraint && constraint_active(*constraint))
                connect(constraint->first_body(), constraint->second_body());
        for (const auto index : ordered_spring_slots_)
            std::visit([&](const auto& spring)
                {
                    if (spring.enabled)
                        connect(spring.first, spring.second);
                },
                *spring_slots_[index].definition);

        std::vector<bool> queued(slots_.size(), false);
        std::vector<BodyId> queue;
        queue.reserve(ids.size());
        for (const auto id : ids)
            if (active(id, *find_body(id)))
            {
                queued[id.index] = true;
                queue.push_back(id);
            }
        for (std::size_t cursor = 0; cursor < queue.size(); ++cursor)
            for (const auto neighbor : neighbors[queue[cursor].index])
            {
                auto& body = *find_body(neighbor);
                // A prescribed or fixed boundary can initiate a wake, but does not relay one
                // from a dynamic neighbor into a separate group attached to the same support.
                if (body.type() != BodyType::dynamic_body)
                    continue;
                if (!body.is_awake())
                    body.wake();
                if (queued[neighbor.index])
                    continue;
                queued[neighbor.index] = true;
                queue.push_back(neighbor);
            }
    }

    void World::update_sleep(Real time_step_s)
    {
        std::vector<bool> supported(slots_.size(), false);
        const auto& sleep = settings_.sleep;
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                supported[id.index] = body.type() == BodyType::static_body ||
                    (body.type() == BodyType::kinematic_body &&
                        math::length(body.linear_velocity_m_s()) <= sleep.linear_speed_m_s &&
                        std::abs(body.angular_velocity_rad_s()) <= sleep.angular_speed_rad_s);
            });
        std::vector<math::Vec2> joint_reactions(slots_.size());
        std::vector<Real> joint_torques(slots_.size());
        for (const auto& constraint : constraints_)
            if (constraint_active(*constraint) && constraint->supports_sleeping_load())
                if (const auto* rows = constraint->velocity_rows())
                    for (const auto& row : *rows)
                    {
                        if (is_valid(row.first))
                        {
                            joint_reactions[row.first.index] += row.linear_first * (row.impulse / time_step_s);
                            joint_torques[row.first.index] += row.angular_first * (row.impulse / time_step_s);
                        }
                        if (is_valid(row.second))
                        {
                            joint_reactions[row.second.index] += row.linear_second * (row.impulse / time_step_s);
                            joint_torques[row.second.index] += row.angular_second * (row.impulse / time_step_s);
                        }
                    }
        // Mere overlap is not support: two touching objects in free fall must not put each
        // other to sleep. Follow actual, load-opposing contact impulses back to a fixed support.
        for (std::size_t pass = 0; pass < ordered_slots_.size(); ++pass)
        {
            bool added = false;
            for (const auto& manifold : manifolds_)
            {
                if (manifold.is_sensor || manifold.is_empty() || !is_valid(manifold.first) ||
                    !is_valid(manifold.second) || !math::is_finite(manifold.normal))
                    continue;
                bool has_impulse = false;
                for (std::size_t index = 0; index < std::min(manifold.point_count, manifold.points.size()); ++index)
                {
                    has_impulse = has_impulse || manifold.points[index].normal_impulse_n_s > 0.0;
                }
                if (!has_impulse)
                    continue;
                const auto add_support = [&](BodyId id, BodyId other, const math::Vec2& direction)
                {
                    const auto& body = *find_body(id);
                    if (!supported[id.index] && supported[other.index] &&
                        (math::dot(body.applied_force_n(), direction) < 0.0 ||
                            math::length(body.applied_force_n()) * body.inverse_mass() <= sleep.linear_acceleration_m_s2))
                    {
                        supported[id.index] = true;
                        added = true;
                    }
                };
                add_support(manifold.first, manifold.second, -manifold.normal);
                add_support(manifold.second, manifold.first, manifold.normal);
            }
            for (const auto& constraint : constraints_)
                if (constraint_active(*constraint) && constraint->supports_sleeping_load())
                {
                    const auto add_joint_support = [&](BodyId id, BodyId other)
                    {
                        const auto& body = *find_body(id);
                        const auto linear_balanced = math::length(body.applied_force_n()) * body.inverse_mass() <= sleep.linear_acceleration_m_s2 ||
                            math::dot(body.applied_force_n(), joint_reactions[id.index]) < 0.0;
                        const auto angular_balanced = std::abs(body.applied_torque_n_m()) * body.inverse_inertia() <= sleep.angular_acceleration_rad_s2 ||
                            body.applied_torque_n_m() * joint_torques[id.index] < 0.0;
                        if (!supported[id.index] && supported[other.index] && linear_balanced && angular_balanced)
                        {
                            supported[id.index] = true;
                            added = true;
                        }
                    };
                    add_joint_support(constraint->first_body(), constraint->second_body());
                    add_joint_support(constraint->second_body(), constraint->first_body());
                }
            if (!added)
                break;
        }
        // Dynamic neighbors settle as one group. A shared static floor is a support, not a
        // connection between every otherwise independent object resting anywhere on that floor.
        std::vector<std::uint32_t> parents(slots_.size());
        for (std::size_t index = 0; index < parents.size(); ++index)
            parents[index] = static_cast<std::uint32_t>(index);
        const auto representative = [&](std::uint32_t index)
        {
            auto root = index;
            while (parents[root] != root)
                root = parents[root];
            while (parents[index] != index)
            {
                const auto next = parents[index];
                parents[index] = root;
                index = next;
            }
            return root;
        };
        const auto join = [&](BodyId first, BodyId second)
        {
            if (!is_valid(first) || !is_valid(second) || find_body(first)->type() != BodyType::dynamic_body ||
                find_body(second)->type() != BodyType::dynamic_body)
                return;
            auto a = representative(first.index);
            auto b = representative(second.index);
            if (a > b)
                std::swap(a, b);
            parents[b] = a;
        };
        for (const auto& manifold : manifolds_)
        {
            if (manifold.is_sensor || manifold.is_empty())
                continue;
            bool touching = false;
            for (std::size_t index = 0; index < std::min(manifold.point_count, manifold.points.size()); ++index)
                touching = touching || manifold.points[index].separation_m <= settings_.solver.linear_slop_m;
            if (touching)
                join(manifold.first, manifold.second);
        }
        for (const auto& constraint : constraints_)
            if (constraint && constraint_active(*constraint))
                join(constraint->first_body(), constraint->second_body());
        for (const auto index : ordered_spring_slots_)
            std::visit([&](const auto& spring)
                {
                    if (spring.enabled)
                        join(spring.first, spring.second);
                },
                *spring_slots_[index].definition);

        std::vector<bool> all_quiet(slots_.size(), true);
        std::vector<bool> any_awake(slots_.size(), false);
        for_each_body([&](BodyId id, RigidBody& body)
            {
                if (body.type() == BodyType::dynamic_body)
                {
                    if (!sleep.enabled || !body.is_sleep_enabled())
                        body.wake();
                    else if (body.is_awake())
                    {
                        const auto slow = math::length(body.linear_velocity_m_s()) <= sleep.linear_speed_m_s &&
                            std::abs(body.angular_velocity_rad_s()) <= sleep.angular_speed_rad_s;
                        const auto balanced = math::length(body.applied_force_n()) * body.inverse_mass() <= sleep.linear_acceleration_m_s2 &&
                            std::abs(body.applied_torque_n_m()) * body.inverse_inertia() <= sleep.angular_acceleration_rad_s2;
                        if (slow && (balanced || supported[id.index]))
                        {
                            body.quiet_time_s_ += time_step_s;
                        }
                        else
                            body.quiet_time_s_ = 0.0;
                    }
                    const auto group = representative(id.index);
                    all_quiet[group] = all_quiet[group] && sleep.enabled && body.is_sleep_enabled() &&
                        (!body.is_awake() || body.quiet_time_s_ >= sleep.quiet_duration_s);
                    any_awake[group] = any_awake[group] || body.is_awake();
                }
            });
        // Decide after every member's timer was evaluated. Sleeping the first quiet body while
        // another member still moves would make support impulses wake it again on every step.
        for_each_body([&](BodyId id, RigidBody& body)
            {
                if (body.type() == BodyType::dynamic_body)
                {
                    const auto group = representative(id.index);
                    if (all_quiet[group])
                        body.set_awake(false);
                    else if (any_awake[group] && !body.is_awake())
                        body.wake();
                }
                body.motion_edited_ = false;
            });
    }
}
