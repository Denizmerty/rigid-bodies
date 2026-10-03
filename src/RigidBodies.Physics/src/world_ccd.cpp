#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/continuous_collision.hpp>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace rigidbodies::physics
{
    void World::constrain_swept_motion(Real time_step_s, const std::vector<IntegratedMotion>& starts)
    {
        if (!settings_.collision.continuous || dynamic_cast<const CollisionNarrowPhase*>(narrow_phase_.get()) == nullptr)
            return;
        const auto ids = body_ids();
        std::vector<IntegratedMotion> targets(slots_.size());
        std::vector<std::size_t> ranks(slots_.size());
        std::vector<bool> clamped(slots_.size(), false);
        std::vector<BroadPhaseProxy> proxies;
        for (std::size_t ordinal = 0; ordinal < ids.size(); ++ordinal)
        {
            const auto id = ids[ordinal];
            if (id.index >= starts.size())
                continue;
            ranks[id.index] = ordinal;
            const auto& body = *find_body(id);
            auto& target = targets[id.index];
            target = { body.world_center_of_mass_m(), body.orientation_rad(), body.linear_velocity_m_s(), body.angular_velocity_rad_s() };
            for (std::size_t index = 0; index < body.colliders().size(); ++index)
            {
                const auto& collider = body.colliders()[index];
                if (!collider.shape || collider.is_sensor)
                    continue;
                const auto sweep = make_sweep(id, collider, time_step_s, target, &starts[id.index]);
                BroadPhaseProxy proxy;
                proxy.body = id;
                proxy.collider_index = index;
                proxy.body_type = body.type();
                proxy.filter = collider.filter;
                proxy.bounds = collider.shape->compute_bounds(sweep.transform_at_fraction(0.0));
                proxy.bounds.expand(collider.shape->compute_bounds(sweep.transform_at_fraction(1.0)));
                proxy.bounds.grow(sweep.nonlinear_travel_m + settings_.collision.sweep_tolerance_m);
                proxy.displacement_m = target.center_position_m - starts[id.index].center_position_m;
                proxies.push_back(proxy);
            }
        }

        // Keep the published broad-phase/manifold/event state unchanged. A plugin's internal
        // acceleration structure may synchronize here, and will synchronize again next step.
        std::vector<BroadPhasePair> candidates;
        broad_phase_->find_pairs(proxies, candidates);
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](const BroadPhasePair& pair)
                             {
                                 if (!is_valid(pair.first) || !is_valid(pair.second) || pair.first == pair.second || !bodies_can_collide(pair.first, pair.second) ||
                                     pair.first.index >= starts.size() || pair.second.index >= starts.size())
                                     return true;
                                 const auto& first = *find_body(pair.first);
                                 const auto& second = *find_body(pair.second);
                                 if (pair.first_collider >= first.colliders().size() || pair.second_collider >= second.colliders().size())
                                     return true;
                                 const auto& a = first.colliders()[pair.first_collider];
                                 const auto& b = second.colliders()[pair.second_collider];
                                 return !a.shape || !b.shape || a.is_sensor || b.is_sensor || !should_collide(a.filter, b.filter) ||
                                     (first.type() != BodyType::dynamic_body && second.type() != BodyType::dynamic_body);
                             }),
            candidates.end());
        for (auto& pair : candidates)
        {
            if (ranks[pair.second.index] < ranks[pair.first.index])
            {
                std::swap(pair.first, pair.second);
                std::swap(pair.first_collider, pair.second_collider);
            }
        }
        const auto key = [&](const BroadPhasePair& pair)
        {
            return std::make_tuple(ranks[pair.first.index], ranks[pair.second.index], pair.first_collider, pair.second_collider);
        };
        std::sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b)
            {
                return key(a) < key(b);
            });
        candidates.erase(std::unique(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b)
                             {
                                 return key(a) == key(b);
                             }),
            candidates.end());

        // Initial contacts belong to the ordinary solver. Repeatedly clipping them would pin a
        // resting or separating body at fraction zero and prevent sliding along its support.
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](const BroadPhasePair& pair)
                             {
                                 const auto& a = find_body(pair.first)->colliders()[pair.first_collider];
                                 const auto& b = find_body(pair.second)->colliders()[pair.second_collider];
                                 const auto first = make_sweep(pair.first, a, time_step_s, targets[pair.first.index], &starts[pair.first.index]);
                                 const auto second = make_sweep(pair.second, b, time_step_s, targets[pair.second.index], &starts[pair.second.index]);
                                 const auto distance = convex_distance(*a.shape, first.transform_at_fraction(0.0), *b.shape, second.transform_at_fraction(0.0));
                                 return distance.valid && (distance.intersecting || distance.distance_m <= settings_.collision.sweep_tolerance_m);
                             }),
            candidates.end());

        const auto apply_fraction = [&](BodyId id, Real fraction)
        {
            auto& body = *find_body(id);
            if (body.type() != BodyType::dynamic_body)
                return false;
            auto& target = targets[id.index];
            const auto& start = starts[id.index];
            if (target.center_position_m == start.center_position_m && target.orientation_rad == start.orientation_rad)
                return false;
            target.center_position_m = math::lerp(start.center_position_m, target.center_position_m, fraction);
            target.orientation_rad = math::lerp(start.orientation_rad, target.orientation_rad, fraction);
            body.set_simulated_pose(target.center_position_m - math::rotate(math::Rotation2 { target.orientation_rad }, body.mass_properties().center_of_mass_m), target.orientation_rad);
            clamped[id.index] = true;
            return true;
        };

        constexpr int maximum_passes = 8;
        for (int pass = 0; pass < maximum_passes; ++pass)
        {
            std::vector<Real> fractions(slots_.size(), 1.0);
            for (const auto& pair : candidates)
            {
                const auto& first_collider = find_body(pair.first)->colliders()[pair.first_collider];
                const auto& second_collider = find_body(pair.second)->colliders()[pair.second_collider];
                const auto first = make_sweep(pair.first, first_collider, time_step_s, targets[pair.first.index], &starts[pair.first.index]);
                const auto second = make_sweep(pair.second, second_collider, time_step_s, targets[pair.second.index], &starts[pair.second.index]);
                const auto sweep = sweep_shapes(first, second, settings_.collision.sweep_tolerance_m, settings_.collision.maximum_sweep_iterations);
                if (!sweep.converged)
                    ++statistics_.sweep_iteration_limit_count;
                if (!sweep.hit && sweep.converged)
                    continue;
                const auto fraction = sweep.distance.valid ? math::clamp(sweep.fraction, 0.0, 1.0) : 0.0;
                if (fraction >= 1.0 - 1.0e-10)
                    continue;
                const auto translation = second.linear_displacement_m - first.linear_displacement_m;
                const auto travel = std::hypot(translation.x, translation.y) + first.nonlinear_travel_m + second.nonlinear_travel_m;
                // Conservative advancement deliberately stops just outside contact. If the
                // actual endpoint is within that remaining tolerance, retain the integrator's
                // ordinary landing instead of rewriting its pose for a sub-tolerance distance.
                if (sweep.distance.valid && (1.0 - fraction) * travel <= settings_.collision.sweep_tolerance_m)
                    continue;
                fractions[pair.first.index] = std::min(fractions[pair.first.index], fraction);
                fractions[pair.second.index] = std::min(fractions[pair.second.index], fraction);
            }
            bool changed = false;
            for (const auto id : ids)
                if (id.index < starts.size() && fractions[id.index] < 1.0)
                    changed = apply_fraction(id, fractions[id.index]) || changed;
            if (!changed)
                break;
            if (pass + 1 == maximum_passes)
            {
                // Changing one body's path can expose another crossing. If a chain still needs
                // work, restore every dynamic start pose rather than permit unresolved travel.
                ++statistics_.sweep_iteration_limit_count;
                for (const auto id : ids)
                    if (id.index < starts.size())
                        (void)apply_fraction(id, 0.0);
            }
        }
        statistics_.ccd_clamped_body_count += static_cast<std::size_t>(std::count(clamped.begin(), clamped.end(), true));
    }
}
