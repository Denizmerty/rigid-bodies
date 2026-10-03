#pragma once

#include <rigidbodies/physics/contact.hpp>

#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{
    class ParallelExecutor;

    // A collider registered with the broad phase, described only by what the broad phase needs:
    // where it is, who owns it, and what it is willing to touch.
    struct BroadPhaseProxy
    {
        BodyId body;
        std::size_t collider_index { 0 };
        math::Aabb bounds;
        CollisionFilter filter;
        BodyType body_type { BodyType::dynamic_body };

        // Expected translation during this update. Spatial structures may extend their cached
        // bounds in the travel direction; exact pair and query results still use bounds above.
        math::Vec2 displacement_m {};
        bool is_sensor { false };
    };

    // Reduces the quadratic number of possible collider pairs to the few worth testing exactly.
    // The interface takes the full proxy set each step rather than incremental updates, which
    // keeps the first implementation trivial while leaving an incremental tree free to cache
    // whatever it needs behind the same call.
    class BroadPhase
    {
    public:
        BroadPhase() = default;
        BroadPhase(const BroadPhase&) = default;
        BroadPhase(BroadPhase&&) = default;
        BroadPhase& operator=(const BroadPhase&) = default;
        BroadPhase& operator=(BroadPhase&&) = default;
        virtual ~BroadPhase() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<BroadPhase> clone() const
        {
            throw std::logic_error("BroadPhase does not support snapshots");
        }

        [[nodiscard]] virtual std::string_view name() const = 0;

        // Appends every candidate pair to the output. The output is not cleared, so several proxy
        // sets can be accumulated into one list.
        virtual void find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const = 0;

        // Reports every proxy whose bounds contain the point, used for picking in the interface.
        virtual void query_point(const std::vector<BroadPhaseProxy>& proxies, const math::Vec2& point_m, const std::function<void(const BroadPhaseProxy&)>& visitor) const;

        virtual void query_bounds(const std::vector<BroadPhaseProxy>& proxies, const math::Aabb& bounds, const std::function<void(const BroadPhaseProxy&)>& visitor) const;
    };

    using BroadPhasePtr = std::shared_ptr<BroadPhase>;

    // Tests every pair. Exact and obviously correct, and entirely adequate at the object counts a
    // household-scale demonstration reaches. It is the reference used to validate spatial
    // structures.
    class BruteForceBroadPhase final : public BroadPhase
    {
    public:
        [[nodiscard]] std::shared_ptr<BroadPhase> clone() const override
        {
            return std::make_shared<BruteForceBroadPhase>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const override;
        std::size_t find_pairs_parallel(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs,
            ParallelExecutor& executor, std::size_t minimum_batch_size) const;
    };

    struct DynamicTreeSettings
    {
        Real base_margin_m { 0.02 };
        Real displacement_multiplier { 2.0 };
    };

    struct DynamicTreeStatistics
    {
        std::size_t proxy_count {};
        std::size_t node_count {};
        int height {};
        std::size_t insertion_count {};
        std::size_t removal_count {};
        std::size_t reinsertion_count {};
        std::size_t last_reinsertions {};
        std::size_t last_node_visits {};
    };

    // A persistent binary AABB tree with height-balanced insertion/removal and a recyclable node
    // pool. A leaf is identified by the complete body generation plus collider index, so reusing
    // a world slot cannot inherit another collider's cached bounds. Small moves stay inside fat
    // bounds; displacement extends the margin in the direction of travel. Overgrown bounds shrink
    // when their perimeter exceeds four times the freshly computed fat bounds.
    //
    // Each operation synchronizes against its complete input set. Missing or invalid-bound proxies
    // are removed; duplicate valid identities are rejected before changing the cache. Exact pair
    // results and query visitation follow input order, matching BruteForceBroadPhase. The const
    // interface mutates the spatial cache and is therefore not safe for concurrent calls.
    // find_pairs_parallel synchronizes first, then uses concurrent read-only tree traversals;
    // each partition's visits and exact pairs are merged in the original input order.
    class DynamicTreeBroadPhase final : public BroadPhase
    {
    public:
        explicit DynamicTreeBroadPhase(const DynamicTreeSettings& settings = {});

        [[nodiscard]] std::shared_ptr<BroadPhase> clone() const override
        {
            return std::make_shared<DynamicTreeBroadPhase>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const override;
        std::size_t find_pairs_parallel(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs,
            ParallelExecutor& executor, std::size_t minimum_batch_size) const;
        void query_point(const std::vector<BroadPhaseProxy>& proxies, const math::Vec2& point_m, const std::function<void(const BroadPhaseProxy&)>& visitor) const override;
        void query_bounds(const std::vector<BroadPhaseProxy>& proxies, const math::Aabb& bounds, const std::function<void(const BroadPhaseProxy&)>& visitor) const override;

        [[nodiscard]] const DynamicTreeSettings& settings() const;
        [[nodiscard]] const DynamicTreeStatistics& statistics() const;
        [[nodiscard]] bool validate_tree() const;
        void clear();

    private:
        struct ProxyKey
        {
            std::uint32_t body_index {};
            std::uint32_t generation {};
            std::size_t collider_index {};

            bool operator<(const ProxyKey& other) const;
        };

        struct Node
        {
            math::Aabb bounds;
            int parent { -1 };
            int left { -1 };
            int right { -1 };
            int height { -1 };
            int next_free { -1 };
            std::size_t input_index {};
            ProxyKey key;

            [[nodiscard]] bool is_leaf() const
            {
                return left == -1;
            }
        };

        [[nodiscard]] static ProxyKey key_of(const BroadPhaseProxy& proxy);
        void synchronize(const std::vector<BroadPhaseProxy>& proxies) const;
        [[nodiscard]] math::Aabb fat_bounds(const BroadPhaseProxy& proxy) const;
        [[nodiscard]] int allocate_node() const;
        void release_node(int index) const;
        void insert_leaf(int index) const;
        void remove_leaf(int index) const;
        void refit_upwards(int index) const;
        [[nodiscard]] int balance(int index) const;
        void update_node(int index) const;
        [[nodiscard]] std::vector<std::size_t> overlapping_indices(const math::Aabb& bounds) const;
        void overlapping_indices(const math::Aabb& bounds, std::vector<std::size_t>& matches, std::vector<int>& stack, std::size_t& visits) const;

        DynamicTreeSettings settings_;
        mutable DynamicTreeStatistics statistics_;
        mutable std::vector<Node> nodes_;
        mutable std::map<ProxyKey, int> leaves_;
        mutable int root_ { -1 };
        mutable int free_list_ { -1 };
    };

} // namespace rigidbodies::physics
