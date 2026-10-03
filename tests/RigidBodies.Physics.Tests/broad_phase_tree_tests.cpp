#include <rigidbodies/physics/broad_phase.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <tuple>

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Aabb;
    using rigidbodies::math::Vec2;
    using Identity = std::tuple<std::uint32_t, std::uint32_t, std::size_t>;

    BroadPhaseProxy proxy(std::uint32_t index, Vec2 center, Vec2 extent = { 0.2, 0.2 }, std::uint32_t generation = 1, std::size_t collider_index = 0)
    {
        BroadPhaseProxy result;
        result.body = { index, generation };
        result.collider_index = collider_index;
        result.bounds = { center - extent, center + extent };
        return result;
    }

    Identity identity(const BroadPhaseProxy& value)
    {
        return { value.body.index, value.body.generation, value.collider_index };
    }

    bool same_pair(const BroadPhasePair& first, const BroadPhasePair& second)
    {
        return first.first == second.first && first.second == second.second &&
            first.first_collider == second.first_collider && first.second_collider == second.second_collider;
    }

    std::vector<BroadPhasePair> expect_reference_pairs(DynamicTreeBroadPhase& tree, const std::vector<BroadPhaseProxy>& proxies)
    {
        std::vector<BroadPhasePair> expected;
        std::vector<BroadPhasePair> actual;
        BruteForceBroadPhase {}.find_pairs(proxies, expected);
        tree.find_pairs(proxies, actual);
        RIGIDBODIES_EXPECT(tree.validate_tree(), "tree links, bounds, height balance and free list are valid");
        RIGIDBODIES_EXPECT(actual.size() == expected.size(), "tree candidate count matches exhaustive reference");
        for (std::size_t index = 0; index < actual.size(); ++index)
        {
            RIGIDBODIES_EXPECT(same_pair(actual[index], expected[index]), "exact pairs match reference order and collider identity");
        }
        return actual;
    }

    std::vector<Identity> point_hits(const BroadPhase& phase, const std::vector<BroadPhaseProxy>& proxies, Vec2 point)
    {
        std::vector<Identity> result;
        phase.query_point(proxies, point, [&](const BroadPhaseProxy& hit)
            {
                result.push_back(identity(hit));
            });
        return result;
    }

    std::vector<Identity> bounds_hits(const BroadPhase& phase, const std::vector<BroadPhaseProxy>& proxies, const Aabb& bounds)
    {
        std::vector<Identity> result;
        phase.query_bounds(proxies, bounds, [&](const BroadPhaseProxy& hit)
            {
                result.push_back(identity(hit));
            });
        return result;
    }

    RIGIDBODIES_TEST("dynamic tree matches exact filtering and includes immovable sensor pairs")
    {
        std::vector<BroadPhaseProxy> proxies { proxy(1, {}), proxy(2, {}), proxy(3, {}), proxy(4, { 2.0, 0.0 }), proxy(1, {}, { 0.3, 0.3 }, 1, 1) };
        proxies[0].body_type = BodyType::static_body;
        proxies[1].body_type = BodyType::kinematic_body;
        proxies[2].body_type = BodyType::static_body;
        proxies[2].is_sensor = true;
        proxies[4].body_type = BodyType::static_body;
        DynamicTreeBroadPhase tree;
        const auto pairs = expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(pairs.size() == 3, "sensor reports all three other immovable colliders without a physical static pair");
        proxies[2].filter.mask = 0;
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).empty(), "sensors still obey category masks");
        proxies[2].filter.group = 4;
        proxies[1].filter.group = 4;
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).size() == 1, "positive shared group overrides masks");
        proxies[2].filter.group = -4;
        proxies[1].filter.group = -4;
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).empty(), "negative shared group suppresses sensor pairs");
    }

    RIGIDBODIES_TEST("fat proxies retain small moves but exact queries and pairs reject margin-only overlap")
    {
        DynamicTreeBroadPhase tree { { 0.5, 2.0 } };
        std::vector<BroadPhaseProxy> proxies { proxy(1, {}), proxy(2, { 0.8, 0.0 }) };
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).empty(), "fat boxes alone do not produce a candidate");
        const auto inserted = tree.statistics().insertion_count;
        proxies[0].bounds.minimum.x += 0.05;
        proxies[0].bounds.maximum.x += 0.05;
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().last_reinsertions == 0 && tree.statistics().insertion_count == inserted, "small move reuses the cached leaf");
        RIGIDBODIES_EXPECT(point_hits(tree, proxies, { 0.4, 0.0 }).empty(), "point in fat margin is not a collider hit");
        RIGIDBODIES_EXPECT(bounds_hits(tree, proxies, { { 0.3, -0.01 }, { 0.4, 0.01 } }).empty(), "AABB in fat margin is not a collider hit");
        proxies[0].bounds.minimum.x += 1.0;
        proxies[0].bounds.maximum.x += 1.0;
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().last_reinsertions == 1, "leaving fat bounds reinserts exactly the moved leaf");
    }

    RIGIDBODIES_TEST("displacement expands fat proxies in travel direction and stopped oversized proxies shrink")
    {
        DynamicTreeBroadPhase tree { { 0.01, 3.0 } };
        auto moving = proxy(1, {});
        moving.displacement_m = { 2.0, -1.0 };
        std::vector<BroadPhaseProxy> proxies { moving };
        expect_reference_pairs(tree, proxies);
        proxies[0].bounds.minimum += Vec2 { 1.0, -0.5 };
        proxies[0].bounds.maximum += Vec2 { 1.0, -0.5 };
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().last_reinsertions == 0, "predicted forward travel stays inside the original fat proxy");
        proxies[0].displacement_m = {};
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().last_reinsertions == 1, "stopping shrinks an excessive old motion envelope");
        const auto retained = tree.statistics().reinsertion_count;
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().reinsertion_count == retained, "unchanged stopped proxy does not churn");
    }

    RIGIDBODIES_TEST("tree synchronizes removals body generations collider indices and empty resets")
    {
        DynamicTreeBroadPhase tree;
        std::vector<BroadPhaseProxy> proxies { proxy(1, {}), proxy(1, { 1.0, 0.0 }, { 0.2, 0.2 }, 1, 1), proxy(2, { 2.0, 0.0 }) };
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 3 && tree.statistics().node_count == 5, "three distinct collider identities own three leaves");
        proxies.erase(proxies.begin() + 1);
        proxies[0].body.generation = 2;
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().removal_count == 2 && tree.statistics().insertion_count == 4, "deleted collider and stale generation are removed before replacement insertion");
        RIGIDBODIES_EXPECT(point_hits(tree, proxies, { 1.0, 0.0 }).empty(), "removed collider is absent from tree queries");
        expect_reference_pairs(tree, {});
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 0 && tree.statistics().node_count == 0 && tree.statistics().height == 0, "empty update removes every leaf");
        expect_reference_pairs(tree, { proxy(4, {}) });
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 1 && tree.statistics().height == 0, "free node pool can be reused after an empty world");
        tree.clear();
        RIGIDBODIES_EXPECT(tree.validate_tree() && tree.statistics().insertion_count == 0, "explicit clear resets storage and diagnostic counters");
    }

    RIGIDBODIES_TEST("tree remains height balanced for sorted insertions and adversarial removals")
    {
        DynamicTreeBroadPhase tree;
        std::vector<BroadPhaseProxy> proxies;
        proxies.reserve(512);
        for (std::uint32_t index = 0; index < 512; ++index)
        {
            proxies.push_back(proxy(index, { static_cast<Real>(index), 0.0 }));
        }
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).empty(), "separated proxies have no pairs");
        RIGIDBODIES_EXPECT(tree.statistics().height <= 14, "sorted geometry cannot turn the tree into a linear chain");
        RIGIDBODIES_EXPECT(tree.statistics().last_node_visits < proxies.size() * proxies.size() / 4, "tree prunes most unrelated leaf comparisons");
        for (int iteration = 0; iteration < 8; ++iteration)
        {
            proxies.erase(proxies.begin(), proxies.begin() + 50);
            expect_reference_pairs(tree, proxies);
            RIGIDBODIES_EXPECT(tree.statistics().height <= 14, "bulk removal preserves logarithmic tree height");
        }
        const auto hits = point_hits(tree, proxies, { 450.0, 0.0 });
        RIGIDBODIES_EXPECT(hits.size() == 1 && std::get<0>(hits.front()) == 450, "tree point query finds its exact collider");
        RIGIDBODIES_EXPECT(tree.statistics().last_node_visits < 40, "point query visits a small branch instead of every proxy");
    }

    RIGIDBODIES_TEST("randomized moving and filtered proxy sets match exhaustive pairs and queries after every update")
    {
        DynamicTreeBroadPhase tree;
        BruteForceBroadPhase reference;
        std::mt19937 random { 982451653 };
        std::uniform_real_distribution<Real> coordinate { -5.0, 5.0 };
        std::uniform_real_distribution<Real> extent { 0.02, 0.8 };
        std::uniform_real_distribution<Real> motion { -0.04, 0.04 };
        std::vector<BroadPhaseProxy> proxies;
        for (std::uint32_t index = 0; index < 180; ++index)
        {
            auto value = proxy(index / 2, { coordinate(random), coordinate(random) }, { extent(random), extent(random) }, 1, index % 2);
            value.body_type = index % 4 == 0 ? BodyType::static_body : (index % 4 == 1 ? BodyType::kinematic_body : BodyType::dynamic_body);
            value.is_sensor = index % 7 == 0;
            value.filter.category = 1u << (index % 3);
            value.filter.mask = index % 5 == 0 ? 0x3u : 0x7u;
            value.filter.group = index % 11 == 0 ? -2 : (index % 13 == 0 ? 3 : 0);
            proxies.push_back(value);
        }
        for (int update = 0; update < 160; ++update)
        {
            for (auto& value : proxies)
            {
                const Vec2 displacement { motion(random), motion(random) };
                value.bounds.minimum += displacement;
                value.bounds.maximum += displacement;
                value.displacement_m = displacement;
            }
            if (update % 9 == 0)
            {
                const auto changed = static_cast<std::size_t>(random()) % proxies.size();
                proxies[changed].is_sensor = !proxies[changed].is_sensor;
                proxies[changed].filter.mask ^= 0x7u;
            }
            if (update % 13 == 0)
            {
                const auto changed = static_cast<std::size_t>(random()) % proxies.size();
                ++proxies[changed].body.generation;
                const auto center = Vec2 { coordinate(random), coordinate(random) };
                const auto half = Vec2 { extent(random), extent(random) };
                proxies[changed].bounds = { center - half, center + half };
            }
            if (update % 17 == 0)
            {
                std::shuffle(proxies.begin(), proxies.end(), random);
            }
            expect_reference_pairs(tree, proxies);
            const Vec2 point { coordinate(random), coordinate(random) };
            const Aabb query { point - Vec2 { 0.4, 0.7 }, point + Vec2 { 0.4, 0.7 } };
            RIGIDBODIES_EXPECT(point_hits(tree, proxies, point) == point_hits(reference, proxies, point), "random point hits match reference and input order");
            RIGIDBODIES_EXPECT(bounds_hits(tree, proxies, query) == bounds_hits(reference, proxies, query), "random AABB hits match reference and input order");
            RIGIDBODIES_EXPECT(tree.validate_tree(), "query synchronization keeps tree valid");
        }
    }

    RIGIDBODIES_TEST("tree clones preserve independent caches counters and exact output order")
    {
        DynamicTreeBroadPhase tree { { 0.03, 1.5 } };
        std::vector<BroadPhaseProxy> proxies { proxy(3, {}), proxy(1, { 0.1, 0.0 }), proxy(2, { 0.2, 0.0 }) };
        const auto original_pairs = expect_reference_pairs(tree, proxies);
        const auto cloned = std::dynamic_pointer_cast<DynamicTreeBroadPhase>(tree.clone());
        RIGIDBODIES_EXPECT(cloned && cloned.get() != &tree && cloned->validate_tree(), "clone has independent valid node storage");
        const auto insertion_count = tree.statistics().insertion_count;
        const auto clone_pairs = expect_reference_pairs(*cloned, proxies);
        RIGIDBODIES_EXPECT(clone_pairs.size() == original_pairs.size(), "clone reports the same candidates");
        expect_reference_pairs(*cloned, { proxy(99, {}) });
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 3 && tree.statistics().insertion_count == insertion_count, "mutating cloned tree cannot alter original cache or counters");
        expect_reference_pairs(tree, proxies);
        std::reverse(proxies.begin(), proxies.end());
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().last_reinsertions == 0, "input reorder only changes visitation order, not leaf placement");
    }

    RIGIDBODIES_TEST("candidate reporting appends without clearing caller-owned pairs")
    {
        DynamicTreeBroadPhase tree;
        const BroadPhasePair previous { { 99, 1 }, { 100, 1 }, 2, 3 };
        std::vector<BroadPhasePair> pairs { previous };
        tree.find_pairs({ proxy(1, {}), proxy(2, {}) }, pairs);
        RIGIDBODIES_EXPECT(pairs.size() == 2 && same_pair(pairs.front(), previous), "preexisting candidate prefix is preserved");
    }

    RIGIDBODIES_TEST("empty nonfinite and degenerate bounds stay consistent with the reference")
    {
        DynamicTreeBroadPhase tree;
        auto empty = proxy(2, {});
        empty.bounds = {};
        auto nonfinite = proxy(3, {});
        nonfinite.bounds.maximum.x = std::numeric_limits<Real>::quiet_NaN();
        auto point = proxy(4, {}, {});
        std::vector<BroadPhaseProxy> proxies { proxy(1, {}), empty, nonfinite, point };
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, proxies).size() == 1, "zero-area finite bounds touch while malformed bounds are absent");
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 2, "malformed bounds cannot poison tree storage");
        RIGIDBODIES_EXPECT(point_hits(tree, proxies, {}).size() == 2, "point queries include finite degenerate proxies");
        RIGIDBODIES_EXPECT(bounds_hits(tree, proxies, {}).empty(), "empty query cannot match any fat leaf");
        RIGIDBODIES_EXPECT(point_hits(tree, proxies, { std::numeric_limits<Real>::infinity(), 0.0 }).empty(), "nonfinite point query is empty");
        proxies[0].bounds = {};
        expect_reference_pairs(tree, proxies);
        RIGIDBODIES_EXPECT(tree.statistics().proxy_count == 1 && tree.statistics().removal_count == 1, "a newly invalid bound removes its prior cached leaf");
    }

    RIGIDBODIES_TEST("duplicate identities fail before mutation and invalid tree settings are rejected")
    {
        DynamicTreeBroadPhase tree;
        expect_reference_pairs(tree, { proxy(1, {}) });
        bool duplicate_rejected = false;
        try
        {
            std::vector<BroadPhasePair> pairs;
            tree.find_pairs({ proxy(1, {}), proxy(1, { 2.0, 0.0 }) }, pairs);
        }
        catch (const std::invalid_argument&)
        {
            duplicate_rejected = true;
        }
        RIGIDBODIES_EXPECT(duplicate_rejected && tree.validate_tree() && tree.statistics().proxy_count == 1, "duplicate key is rejected without discarding valid cache");
        for (const auto value : { -1.0, std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
        {
            for (const auto invalid : { DynamicTreeSettings { value, 2.0 }, DynamicTreeSettings { 0.02, value } })
            {
                bool rejected = false;
                try
                {
                    const DynamicTreeBroadPhase bad { invalid };
                }
                catch (const std::invalid_argument&)
                {
                    rejected = true;
                }
                RIGIDBODIES_EXPECT(rejected, "negative and nonfinite tree tuning is rejected");
            }
        }
    }

    RIGIDBODIES_TEST("finite extreme bounds and unusable displacement hints cannot corrupt tree geometry")
    {
        DynamicTreeBroadPhase tree;
        const auto largest = std::numeric_limits<Real>::max();
        auto left = proxy(1, {});
        left.bounds = { { -largest, -largest }, { 0.0, 0.0 } };
        left.displacement_m = { -largest, -largest };
        auto right = proxy(2, {});
        right.bounds = { { 0.0, 0.0 }, { largest, largest } };
        right.displacement_m = { largest, largest };
        RIGIDBODIES_EXPECT(expect_reference_pairs(tree, { left, right }).size() == 1, "extreme finite boxes touching at origin remain valid candidates");
        right.displacement_m = { std::numeric_limits<Real>::infinity(), 0.0 };
        expect_reference_pairs(tree, { left, right });
        RIGIDBODIES_EXPECT(tree.validate_tree(), "invalid prediction hint cannot produce nonfinite cached bounds");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
