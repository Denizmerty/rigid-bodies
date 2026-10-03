#include <rigidbodies/physics/broad_phase.hpp>
#include <rigidbodies/physics/parallel_executor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {

        bool valid_bounds(const math::Aabb& bounds)
        {
            return !bounds.is_empty() && math::is_finite(bounds.minimum) && math::is_finite(bounds.maximum);
        }

        bool contains(const math::Aabb& outer, const math::Aabb& inner)
        {
            return outer.minimum.x <= inner.minimum.x && outer.minimum.y <= inner.minimum.y &&
                outer.maximum.x >= inner.maximum.x && outer.maximum.y >= inner.maximum.y;
        }

        Real saturated_add(Real first, Real second)
        {
            constexpr auto maximum = std::numeric_limits<Real>::max();
            if (second > 0.0 && first > maximum - second)
            {
                return maximum;
            }
            if (second < 0.0 && first < -maximum - second)
            {
                return -maximum;
            }
            return first + second;
        }

        Real saturated_scale(Real value, Real scale)
        {
            constexpr auto maximum = std::numeric_limits<Real>::max();
            if (scale > 0.0 && std::abs(value) > maximum / scale)
            {
                return std::copysign(maximum, value);
            }
            return value * scale;
        }

        Real perimeter(const math::Aabb& bounds)
        {
            const auto width = saturated_add(bounds.maximum.x, -bounds.minimum.x);
            const auto height = saturated_add(bounds.maximum.y, -bounds.minimum.y);
            return saturated_scale(saturated_add(width, height), 2.0);
        }

        bool eligible_pair(const BroadPhaseProxy& first, const BroadPhaseProxy& second)
        {
            return !(first.body == second.body) &&
                (first.body_type == BodyType::dynamic_body || second.body_type == BodyType::dynamic_body || first.is_sensor || second.is_sensor) &&
                should_collide(first.filter, second.filter) && math::overlaps(first.bounds, second.bounds);
        }

    } // namespace

    DynamicTreeBroadPhase::DynamicTreeBroadPhase(const DynamicTreeSettings& settings) : settings_(settings)
    {
        if (!math::is_finite(settings.base_margin_m) || settings.base_margin_m < 0.0 ||
            !math::is_finite(settings.displacement_multiplier) || settings.displacement_multiplier < 0.0)
        {
            throw std::invalid_argument("Dynamic-tree margins and displacement multipliers must be finite and non-negative");
        }
    }

    bool DynamicTreeBroadPhase::ProxyKey::operator<(const ProxyKey& other) const
    {
        return std::tie(body_index, generation, collider_index) < std::tie(other.body_index, other.generation, other.collider_index);
    }

    DynamicTreeBroadPhase::ProxyKey DynamicTreeBroadPhase::key_of(const BroadPhaseProxy& proxy)
    {
        return { proxy.body.index, proxy.body.generation, proxy.collider_index };
    }

    std::string_view DynamicTreeBroadPhase::name() const
    {
        return "dynamic_aabb_tree";
    }

    const DynamicTreeSettings& DynamicTreeBroadPhase::settings() const
    {
        return settings_;
    }

    const DynamicTreeStatistics& DynamicTreeBroadPhase::statistics() const
    {
        return statistics_;
    }

    void DynamicTreeBroadPhase::clear()
    {
        nodes_.clear();
        leaves_.clear();
        root_ = -1;
        free_list_ = -1;
        statistics_ = {};
    }

    math::Aabb DynamicTreeBroadPhase::fat_bounds(const BroadPhaseProxy& proxy) const
    {
        auto bounds = proxy.bounds;
        bounds.minimum.x = saturated_add(bounds.minimum.x, -settings_.base_margin_m);
        bounds.minimum.y = saturated_add(bounds.minimum.y, -settings_.base_margin_m);
        bounds.maximum.x = saturated_add(bounds.maximum.x, settings_.base_margin_m);
        bounds.maximum.y = saturated_add(bounds.maximum.y, settings_.base_margin_m);
        // A non-finite prediction is unusable as a hint; finite authored bounds still remain
        // queryable. Saturation keeps huge but finite hints from poisoning the hierarchy.
        if (math::is_finite(proxy.displacement_m))
        {
            const auto x = saturated_scale(proxy.displacement_m.x, settings_.displacement_multiplier);
            const auto y = saturated_scale(proxy.displacement_m.y, settings_.displacement_multiplier);
            if (x < 0.0)
            {
                bounds.minimum.x = saturated_add(bounds.minimum.x, x);
            }
            else
            {
                bounds.maximum.x = saturated_add(bounds.maximum.x, x);
            }
            if (y < 0.0)
            {
                bounds.minimum.y = saturated_add(bounds.minimum.y, y);
            }
            else
            {
                bounds.maximum.y = saturated_add(bounds.maximum.y, y);
            }
        }
        return bounds;
    }

    int DynamicTreeBroadPhase::allocate_node() const
    {
        int index;
        if (free_list_ != -1)
        {
            index = free_list_;
            free_list_ = nodes_[static_cast<std::size_t>(index)].next_free;
            nodes_[static_cast<std::size_t>(index)] = {};
        }
        else
        {
            index = static_cast<int>(nodes_.size());
            nodes_.emplace_back();
        }
        nodes_[static_cast<std::size_t>(index)].height = 0;
        return index;
    }

    void DynamicTreeBroadPhase::release_node(int index) const
    {
        auto& node = nodes_[static_cast<std::size_t>(index)];
        node = {};
        node.next_free = free_list_;
        free_list_ = index;
    }

    void DynamicTreeBroadPhase::update_node(int index) const
    {
        auto& node = nodes_[static_cast<std::size_t>(index)];
        if (!node.is_leaf())
        {
            const auto& left = nodes_[static_cast<std::size_t>(node.left)];
            const auto& right = nodes_[static_cast<std::size_t>(node.right)];
            node.bounds = math::combined(left.bounds, right.bounds);
            node.height = 1 + std::max(left.height, right.height);
        }
    }

    int DynamicTreeBroadPhase::balance(int index) const
    {
        auto& node = nodes_[static_cast<std::size_t>(index)];
        if (node.is_leaf() || node.height < 2)
        {
            return index;
        }
        const auto difference = nodes_[static_cast<std::size_t>(node.right)].height - nodes_[static_cast<std::size_t>(node.left)].height;
        if (difference >= -1 && difference <= 1)
        {
            return index;
        }

        const bool right_heavy = difference > 1;
        const auto promoted_index = right_heavy ? node.right : node.left;
        auto& promoted = nodes_[static_cast<std::size_t>(promoted_index)];
        const auto old_parent = node.parent;
        const auto first = promoted.left;
        const auto second = promoted.right;
        // Retain the taller grandchild under the promoted node. This bounds every height
        // difference while preserving all leaf membership; bounds are refitted below.
        const auto keep_first = nodes_[static_cast<std::size_t>(first)].height > nodes_[static_cast<std::size_t>(second)].height ||
            (nodes_[static_cast<std::size_t>(first)].height == nodes_[static_cast<std::size_t>(second)].height && first < second);
        const auto retained = keep_first ? first : second;
        const auto transferred = keep_first ? second : first;
        promoted.parent = old_parent;
        node.parent = promoted_index;
        if (old_parent == -1)
        {
            root_ = promoted_index;
        }
        else
        {
            auto& parent = nodes_[static_cast<std::size_t>(old_parent)];
            if (parent.left == index)
            {
                parent.left = promoted_index;
            }
            else
            {
                parent.right = promoted_index;
            }
        }
        if (right_heavy)
        {
            promoted.left = index;
            promoted.right = retained;
            node.right = transferred;
        }
        else
        {
            promoted.left = retained;
            promoted.right = index;
            node.left = transferred;
        }
        nodes_[static_cast<std::size_t>(retained)].parent = promoted_index;
        nodes_[static_cast<std::size_t>(transferred)].parent = index;
        update_node(index);
        update_node(promoted_index);
        return promoted_index;
    }

    void DynamicTreeBroadPhase::refit_upwards(int index) const
    {
        while (index != -1)
        {
            update_node(index);
            index = balance(index);
            index = nodes_[static_cast<std::size_t>(index)].parent;
        }
    }

    void DynamicTreeBroadPhase::insert_leaf(int index) const
    {
        if (root_ == -1)
        {
            root_ = index;
            nodes_[static_cast<std::size_t>(index)].parent = -1;
            return;
        }
        const auto leaf_bounds = nodes_[static_cast<std::size_t>(index)].bounds;
        auto sibling = root_;
        // Descend to a leaf, choosing least perimeter growth. Inserting beside a leaf changes
        // subtree height by at most one, which permits ordinary bounded AVL-style rotations.
        while (!nodes_[static_cast<std::size_t>(sibling)].is_leaf())
        {
            const auto& current = nodes_[static_cast<std::size_t>(sibling)];
            const auto left = current.left;
            const auto right = current.right;
            const auto& a = nodes_[static_cast<std::size_t>(left)];
            const auto& b = nodes_[static_cast<std::size_t>(right)];
            const auto grown_a = perimeter(math::combined(a.bounds, leaf_bounds));
            const auto grown_b = perimeter(math::combined(b.bounds, leaf_bounds));
            const auto cost_a = grown_a - perimeter(a.bounds);
            const auto cost_b = grown_b - perimeter(b.bounds);
            sibling = std::make_tuple(cost_a, grown_a, a.height, left) < std::make_tuple(cost_b, grown_b, b.height, right) ? left : right;
        }

        const auto old_parent = nodes_[static_cast<std::size_t>(sibling)].parent;
        const auto parent_index = allocate_node();
        auto& parent = nodes_[static_cast<std::size_t>(parent_index)];
        parent.parent = old_parent;
        parent.left = sibling;
        parent.right = index;
        nodes_[static_cast<std::size_t>(sibling)].parent = parent_index;
        nodes_[static_cast<std::size_t>(index)].parent = parent_index;
        if (old_parent == -1)
        {
            root_ = parent_index;
        }
        else
        {
            auto& ancestor = nodes_[static_cast<std::size_t>(old_parent)];
            if (ancestor.left == sibling)
            {
                ancestor.left = parent_index;
            }
            else
            {
                ancestor.right = parent_index;
            }
        }
        refit_upwards(parent_index);
    }

    void DynamicTreeBroadPhase::remove_leaf(int index) const
    {
        if (root_ == index)
        {
            root_ = -1;
            nodes_[static_cast<std::size_t>(index)].parent = -1;
            return;
        }
        const auto parent_index = nodes_[static_cast<std::size_t>(index)].parent;
        const auto parent = nodes_[static_cast<std::size_t>(parent_index)];
        const auto sibling = parent.left == index ? parent.right : parent.left;
        if (parent.parent == -1)
        {
            root_ = sibling;
            nodes_[static_cast<std::size_t>(sibling)].parent = -1;
        }
        else
        {
            auto& ancestor = nodes_[static_cast<std::size_t>(parent.parent)];
            if (ancestor.left == parent_index)
            {
                ancestor.left = sibling;
            }
            else
            {
                ancestor.right = sibling;
            }
            nodes_[static_cast<std::size_t>(sibling)].parent = parent.parent;
        }
        release_node(parent_index);
        nodes_[static_cast<std::size_t>(index)].parent = -1;
        refit_upwards(parent.parent);
    }

    void DynamicTreeBroadPhase::synchronize(const std::vector<BroadPhaseProxy>& proxies) const
    {
        std::vector<std::pair<ProxyKey, std::size_t>> incoming;
        incoming.reserve(proxies.size());
        for (std::size_t index = 0; index < proxies.size(); ++index)
        {
            if (valid_bounds(proxies[index].bounds))
            {
                incoming.emplace_back(key_of(proxies[index]), index);
            }
        }
        if (incoming.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) / 2)
        {
            throw std::length_error("Dynamic tree proxy count exceeds the node-index range");
        }
        std::sort(incoming.begin(), incoming.end(), [](const auto& first, const auto& second)
            {
                return first.first < second.first;
            });
        for (std::size_t index = 1; index < incoming.size(); ++index)
        {
            if (!(incoming[index - 1].first < incoming[index].first))
            {
                throw std::invalid_argument("Dynamic tree requires unique body-generation and collider-index identities");
            }
        }
        statistics_.last_reinsertions = 0;
        statistics_.last_node_visits = 0;
        nodes_.reserve(std::max(nodes_.size(), incoming.size() * 2));
        for (auto found = leaves_.begin(); found != leaves_.end();)
        {
            const auto incoming_match = std::lower_bound(incoming.begin(), incoming.end(), found->first, [](const auto& entry, const ProxyKey& key)
                {
                    return entry.first < key;
                });
            if (incoming_match == incoming.end() || found->first < incoming_match->first)
            {
                remove_leaf(found->second);
                release_node(found->second);
                found = leaves_.erase(found);
                ++statistics_.removal_count;
            }
            else
            {
                ++found;
            }
        }
        for (const auto& entry : incoming)
        {
            const auto& proxy = proxies[entry.second];
            const auto desired = fat_bounds(proxy);
            const auto found = leaves_.find(entry.first);
            if (found == leaves_.end())
            {
                const auto index = allocate_node();
                auto& leaf = nodes_[static_cast<std::size_t>(index)];
                leaf.bounds = desired;
                leaf.key = entry.first;
                leaf.input_index = entry.second;
                leaves_.emplace(entry.first, index);
                insert_leaf(index);
                ++statistics_.insertion_count;
            }
            else
            {
                const auto index = found->second;
                auto& leaf = nodes_[static_cast<std::size_t>(index)];
                leaf.input_index = entry.second;
                const auto excessively_large = perimeter(leaf.bounds) > saturated_scale(perimeter(desired), 4.0);
                if (!contains(leaf.bounds, proxy.bounds) || excessively_large)
                {
                    remove_leaf(index);
                    leaf.bounds = desired;
                    insert_leaf(index);
                    ++statistics_.reinsertion_count;
                    ++statistics_.last_reinsertions;
                }
            }
        }
        statistics_.proxy_count = leaves_.size();
        statistics_.node_count = leaves_.empty() ? 0 : leaves_.size() * 2 - 1;
        statistics_.height = root_ == -1 ? 0 : nodes_[static_cast<std::size_t>(root_)].height;
    }

    std::vector<std::size_t> DynamicTreeBroadPhase::overlapping_indices(const math::Aabb& bounds) const
    {
        std::vector<std::size_t> matches;
        std::vector<int> stack;
        overlapping_indices(bounds, matches, stack, statistics_.last_node_visits);
        return matches;
    }

    void DynamicTreeBroadPhase::overlapping_indices(const math::Aabb& bounds, std::vector<std::size_t>& matches,
        std::vector<int>& stack, std::size_t& visits) const
    {
        matches.clear();
        stack.clear();
        if (root_ == -1 || !valid_bounds(bounds))
        {
            return;
        }
        stack.push_back(root_);
        while (!stack.empty())
        {
            const auto index = stack.back();
            stack.pop_back();
            ++visits;
            const auto& node = nodes_[static_cast<std::size_t>(index)];
            if (!math::overlaps(node.bounds, bounds))
            {
                continue;
            }
            if (node.is_leaf())
            {
                matches.push_back(node.input_index);
            }
            else
            {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
        std::sort(matches.begin(), matches.end());
    }

    void DynamicTreeBroadPhase::find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const
    {
        synchronize(proxies);
        for (std::size_t first = 0; first < proxies.size(); ++first)
        {
            if (!valid_bounds(proxies[first].bounds))
            {
                continue;
            }
            for (const auto second : overlapping_indices(proxies[first].bounds))
            {
                if (second > first && eligible_pair(proxies[first], proxies[second]))
                {
                    pairs.push_back({ proxies[first].body, proxies[second].body, proxies[first].collider_index, proxies[second].collider_index });
                }
            }
        }
    }

    std::size_t DynamicTreeBroadPhase::find_pairs_parallel(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs,
        ParallelExecutor& executor, std::size_t minimum_batch_size) const
    {
        // Cache mutation is complete before any reader starts. Each partition owns traversal
        // scratch and statistics; merging in range order matches the serial first/second order.
        synchronize(proxies);
        struct Partition
        {
            std::vector<BroadPhasePair> pairs;
            std::vector<std::size_t> matches;
            std::vector<int> stack;
            std::size_t visits {};
        };
        std::vector<Partition> results(executor.worker_count());
        const auto partitions = executor.run(proxies.size(), minimum_batch_size, [&](std::size_t begin, std::size_t end, std::size_t partition)
            {
                auto& result = results[partition];
                for (auto first = begin; first < end; ++first)
                {
                    overlapping_indices(proxies[first].bounds, result.matches, result.stack, result.visits);
                    for (const auto second : result.matches)
                    {
                        if (second > first && eligible_pair(proxies[first], proxies[second]))
                            result.pairs.push_back({ proxies[first].body, proxies[second].body, proxies[first].collider_index, proxies[second].collider_index });
                    }
                }
            });
        for (std::size_t partition = 0; partition < partitions; ++partition)
        {
            pairs.insert(pairs.end(), results[partition].pairs.begin(), results[partition].pairs.end());
            statistics_.last_node_visits += results[partition].visits;
        }
        return partitions;
    }

    void DynamicTreeBroadPhase::query_point(const std::vector<BroadPhaseProxy>& proxies, const math::Vec2& point_m, const std::function<void(const BroadPhaseProxy&)>& visitor) const
    {
        synchronize(proxies);
        if (!math::is_finite(point_m))
        {
            return;
        }
        const math::Aabb point_bounds { point_m, point_m };
        for (const auto index : overlapping_indices(point_bounds))
        {
            if (proxies[index].bounds.contains(point_m))
            {
                visitor(proxies[index]);
            }
        }
    }

    void DynamicTreeBroadPhase::query_bounds(const std::vector<BroadPhaseProxy>& proxies, const math::Aabb& bounds, const std::function<void(const BroadPhaseProxy&)>& visitor) const
    {
        synchronize(proxies);
        for (const auto index : overlapping_indices(bounds))
        {
            if (math::overlaps(proxies[index].bounds, bounds))
            {
                visitor(proxies[index]);
            }
        }
    }

    bool DynamicTreeBroadPhase::validate_tree() const
    {
        const auto valid_index = [&](int index)
        {
            return index >= 0 && static_cast<std::size_t>(index) < nodes_.size();
        };
        if ((root_ == -1) != leaves_.empty() || (root_ != -1 && (!valid_index(root_) || nodes_[static_cast<std::size_t>(root_)].parent != -1)))
        {
            return false;
        }
        std::vector<bool> visited(nodes_.size());
        std::vector<int> stack;
        if (root_ != -1)
        {
            stack.push_back(root_);
        }
        std::size_t leaf_count = 0;
        std::size_t active_count = 0;
        while (!stack.empty())
        {
            const auto index = stack.back();
            stack.pop_back();
            if (!valid_index(index) || visited[static_cast<std::size_t>(index)])
            {
                return false;
            }
            visited[static_cast<std::size_t>(index)] = true;
            ++active_count;
            const auto& node = nodes_[static_cast<std::size_t>(index)];
            if (node.height < 0 || !valid_bounds(node.bounds))
            {
                return false;
            }
            if (node.is_leaf())
            {
                const auto found = leaves_.find(node.key);
                if (node.height != 0 || node.right != -1 || found == leaves_.end() || found->second != index)
                {
                    return false;
                }
                ++leaf_count;
            }
            else
            {
                if (!valid_index(node.left) || !valid_index(node.right))
                {
                    return false;
                }
                const auto& left = nodes_[static_cast<std::size_t>(node.left)];
                const auto& right = nodes_[static_cast<std::size_t>(node.right)];
                const auto combined = math::combined(left.bounds, right.bounds);
                if (left.parent != index || right.parent != index || std::abs(left.height - right.height) > 1 ||
                    node.height != 1 + std::max(left.height, right.height) ||
                    !(node.bounds.minimum == combined.minimum) || !(node.bounds.maximum == combined.maximum))
                {
                    return false;
                }
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
        for (auto free = free_list_; free != -1; free = nodes_[static_cast<std::size_t>(free)].next_free)
        {
            if (!valid_index(free) || visited[static_cast<std::size_t>(free)] || nodes_[static_cast<std::size_t>(free)].height != -1)
            {
                return false;
            }
            visited[static_cast<std::size_t>(free)] = true;
        }
        return leaf_count == leaves_.size() && active_count == statistics_.node_count &&
            std::all_of(visited.begin(), visited.end(), [](bool value)
                {
                    return value;
                });
    }

} // namespace rigidbodies::physics
