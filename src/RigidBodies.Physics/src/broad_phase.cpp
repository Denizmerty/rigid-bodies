#include <rigidbodies/physics/broad_phase.hpp>
#include <rigidbodies/physics/parallel_executor.hpp>

namespace rigidbodies::physics
{
    namespace
    {
        bool valid_bounds(const math::Aabb& bounds)
        {
            return !bounds.is_empty() && math::is_finite(bounds.minimum) && math::is_finite(bounds.maximum);
        }
    }

    void BroadPhase::query_point(const std::vector<BroadPhaseProxy>& proxies, const math::Vec2& point_m, const std::function<void(const BroadPhaseProxy&)>& visitor) const
    {
        if (!math::is_finite(point_m))
        {
            return;
        }
        for (const auto& proxy : proxies)
        {
            if (valid_bounds(proxy.bounds) && proxy.bounds.contains(point_m))
            {
                visitor(proxy);
            }
        }
    }

    void BroadPhase::query_bounds(const std::vector<BroadPhaseProxy>& proxies, const math::Aabb& bounds, const std::function<void(const BroadPhaseProxy&)>& visitor) const
    {
        if (!valid_bounds(bounds))
        {
            return;
        }
        for (const auto& proxy : proxies)
        {
            if (valid_bounds(proxy.bounds) && math::overlaps(proxy.bounds, bounds))
            {
                visitor(proxy);
            }
        }
    }

    std::string_view BruteForceBroadPhase::name() const
    {
        return "brute_force";
    }

    void BruteForceBroadPhase::find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const
    {
        for (std::size_t first = 0; first + 1 < proxies.size(); ++first)
        {
            for (std::size_t second = first + 1; second < proxies.size(); ++second)
            {
                const auto& left = proxies[first];
                const auto& right = proxies[second];

                // Two immovable bodies need no physical response, but sensors still report their
                // overlaps. Two colliders on the same body are parts of one object, not a pair.
                if (left.body_type != BodyType::dynamic_body && right.body_type != BodyType::dynamic_body && !left.is_sensor && !right.is_sensor)
                {
                    continue;
                }
                if (left.body == right.body)
                {
                    continue;
                }
                if (!should_collide(left.filter, right.filter))
                {
                    continue;
                }
                if (!valid_bounds(left.bounds) || !valid_bounds(right.bounds) || !math::overlaps(left.bounds, right.bounds))
                {
                    continue;
                }

                pairs.push_back({ left.body, right.body, left.collider_index, right.collider_index });
            }
        }
    }

    std::size_t BruteForceBroadPhase::find_pairs_parallel(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs,
        ParallelExecutor& executor, std::size_t minimum_batch_size) const
    {
        std::vector<std::vector<BroadPhasePair>> results(executor.worker_count());
        const auto partitions = executor.run(proxies.size(), minimum_batch_size, [&](std::size_t begin, std::size_t end, std::size_t partition)
            {
                auto& result = results[partition];
                for (auto first = begin; first < end; ++first)
                {
                    const auto& left = proxies[first];
                    if (!valid_bounds(left.bounds))
                        continue;
                    for (auto second = first + 1; second < proxies.size(); ++second)
                    {
                        const auto& right = proxies[second];
                        if ((left.body_type != BodyType::dynamic_body && right.body_type != BodyType::dynamic_body && !left.is_sensor && !right.is_sensor) ||
                            left.body == right.body || !should_collide(left.filter, right.filter) ||
                            !valid_bounds(right.bounds) || !math::overlaps(left.bounds, right.bounds))
                            continue;
                        result.push_back({ left.body, right.body, left.collider_index, right.collider_index });
                    }
                }
            });
        for (std::size_t partition = 0; partition < partitions; ++partition)
            pairs.insert(pairs.end(), results[partition].begin(), results[partition].end());
        return partitions;
    }

} // namespace rigidbodies::physics
