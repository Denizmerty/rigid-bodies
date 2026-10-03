#include <rigidbodies/physics/parallel_executor.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>
#include <thread>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    bool same_pair(const BroadPhasePair& first, const BroadPhasePair& second)
    {
        return first.first == second.first && first.second == second.second &&
            first.first_collider == second.first_collider && first.second_collider == second.second_collider;
    }

    void expect_pairs(const std::vector<BroadPhasePair>& actual, const std::vector<BroadPhasePair>& expected)
    {
        RIGIDBODIES_EXPECT(actual.size() == expected.size(), "pair count is unchanged by scheduling");
        for (std::size_t index = 0; index < actual.size(); ++index)
            RIGIDBODIES_EXPECT(same_pair(actual[index], expected[index]), "pair order and collider generations are exact");
    }

    void expect_events(const std::vector<CollisionEvent>& actual, const std::vector<CollisionEvent>& expected)
    {
        RIGIDBODIES_EXPECT(actual.size() == expected.size(), "event count matches");
        for (std::size_t index = 0; index < actual.size(); ++index)
            RIGIDBODIES_EXPECT(same_pair(actual[index].pair, expected[index].pair) && actual[index].kind == expected[index].kind &&
                    actual[index].is_sensor == expected[index].is_sensor,
                "begin and end events retain serial ordering including sensors");
    }

    void expect_world(const World& actual, const World& expected)
    {
        const auto ids = expected.body_ids();
        RIGIDBODIES_EXPECT(actual.body_ids() == ids, "body identities retain their canonical order");
        for (const auto id : ids)
        {
            const auto& a = *actual.find_body(id);
            const auto& b = *expected.find_body(id);
            RIGIDBODIES_EXPECT(a.position_m() == b.position_m() && a.orientation_rad() == b.orientation_rad() &&
                    a.linear_velocity_m_s() == b.linear_velocity_m_s() && a.angular_velocity_rad_s() == b.angular_velocity_rad_s() &&
                    a.is_awake() == b.is_awake() && a.quiet_time_s() == b.quiet_time_s(),
                "body motion and sleeping state are bit-for-bit identical");
        }
        expect_pairs(actual.broad_phase_pairs(), expected.broad_phase_pairs());
        expect_events(actual.pair_events(), expected.pair_events());
        expect_events(actual.contact_events(), expected.contact_events());
        const auto& a = actual.manifolds();
        const auto& b = expected.manifolds();
        RIGIDBODIES_EXPECT(a.size() == b.size(), "manifold count matches");
        for (std::size_t index = 0; index < a.size(); ++index)
        {
            RIGIDBODIES_EXPECT(a[index].first == b[index].first && a[index].second == b[index].second &&
                    a[index].first_collider == b[index].first_collider && a[index].second_collider == b[index].second_collider &&
                    a[index].normal == b[index].normal && a[index].is_sensor == b[index].is_sensor &&
                    a[index].is_speculative == b[index].is_speculative && a[index].point_count == b[index].point_count &&
                    a[index].applied_impulse_on_second_n_s == b[index].applied_impulse_on_second_n_s,
                "manifold identity, normal, class and response are exact");
            for (std::size_t point = 0; point < a[index].point_count; ++point)
            {
                const auto& x = a[index].points[point];
                const auto& y = b[index].points[point];
                RIGIDBODIES_EXPECT(x.world_position_m == y.world_position_m && x.separation_m == y.separation_m &&
                        x.feature.key() == y.feature.key() && x.normal_impulse_n_s == y.normal_impulse_n_s &&
                        x.tangent_impulse_n_s == y.tangent_impulse_n_s && x.rolling_impulse_n_m_s == y.rolling_impulse_n_m_s &&
                        x.spinning_impulse_n_m_s == y.spinning_impulse_n_m_s &&
                        x.local_anchor_first_m == y.local_anchor_first_m && x.local_anchor_second_m == y.local_anchor_second_m &&
                        x.speculative_velocity_bias_m_s == y.speculative_velocity_bias_m_s &&
                        x.pending_impact_speed_m_s == y.pending_impact_speed_m_s,
                    "contact witnesses and persistent impulses remain bit-for-bit identical");
            }
        }
        RIGIDBODIES_EXPECT(actual.statistics().total_kinetic_energy_j == expected.statistics().total_kinetic_energy_j &&
                actual.statistics().persistent_contact_point_count == expected.statistics().persistent_contact_point_count &&
                actual.statistics().sweep_iteration_limit_count == expected.statistics().sweep_iteration_limit_count &&
                actual.statistics().ccd_clamped_body_count == expected.statistics().ccd_clamped_body_count,
            "energy and collision evidence are schedule independent");
    }

    BodyDefinition body(ShapePtr shape, Vec2 position, BodyType type = BodyType::dynamic_body, Vec2 velocity = {})
    {
        BodyDefinition result;
        result.position_m = position;
        result.type = type;
        result.linear_velocity_m_s = velocity;
        Collider collider;
        collider.shape = std::move(shape);
        result.colliders.push_back(std::move(collider));
        return result;
    }

    void populate(World& world)
    {
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_broad_phase(std::make_shared<DynamicTreeBroadPhase>());
        for (int cell = 0; cell < 24; ++cell)
        {
            const auto x = static_cast<Real>(cell % 8) * 4.0;
            const auto y = static_cast<Real>(cell / 8) * 4.0;
            (void)world.create_body(body(make_box(2.5, 0.1), { x, y }, BodyType::static_body));
            (void)world.create_body(body(make_circle(0.2), { x - 0.7, y + 0.45 }, BodyType::dynamic_body, { 0.3, -0.4 }));
            (void)world.create_body(body(make_box(0.4, 0.4), { x + 0.1, y + 0.25 }));
            auto sensor = body(make_box(1.0, 0.9), { x, y + 0.4 }, BodyType::static_body);
            sensor.colliders[0].is_sensor = true;
            (void)world.create_body(sensor);
        }
    }

    RIGIDBODIES_TEST("persistent executor partitions exact coverage and reuses real worker threads")
    {
        ParallelExecutor executor(4);
        std::array<std::thread::id, 4> first_ids {};
        std::array<std::thread::id, 4> second_ids {};
        std::vector<int> visits(103);
        const auto run = [&](auto& ids)
        {
            return executor.run(visits.size(), 8, [&](std::size_t begin, std::size_t end, std::size_t partition)
                {
                    ids[partition] = std::this_thread::get_id();
                    for (auto index = begin; index < end; ++index)
                        ++visits[index];
                });
        };
        RIGIDBODIES_EXPECT(run(first_ids) == 4 && run(second_ids) == 4, "all four workers participate in both batches");
        RIGIDBODIES_EXPECT(first_ids == second_ids, "workers persist across invocations");
        RIGIDBODIES_EXPECT(first_ids.back() == std::this_thread::get_id(), "the calling thread participates");
        for (std::size_t first = 0; first < first_ids.size(); ++first)
            for (auto second = first + 1; second < first_ids.size(); ++second)
                RIGIDBODIES_EXPECT(first_ids[first] != first_ids[second], "partitions execute on distinct operating-system threads");
        RIGIDBODIES_EXPECT(std::all_of(visits.begin(), visits.end(), [](int count)
                               {
                                   return count == 2;
                               }),
            "every item runs exactly once per batch");
    }

    RIGIDBODIES_TEST("small and empty parallel batches stay local without changing range semantics")
    {
        ParallelExecutor executor(4);
        std::size_t calls = 0;
        const auto caller = std::this_thread::get_id();
        RIGIDBODIES_EXPECT(executor.run(0, 64, {}) == 0, "empty batches require no operation");
        const auto partitions = executor.run(127, 64, [&](std::size_t begin, std::size_t end, std::size_t partition)
            {
                ++calls;
                RIGIDBODIES_EXPECT(begin == 0 && end == 127 && partition == 0 && std::this_thread::get_id() == caller, "subthreshold work stays on the caller");
            });
        RIGIDBODIES_EXPECT(partitions == 1 && calls == 1, "batch size is a minimum per participant");
    }

    RIGIDBODIES_TEST("parallel worker failures complete all partitions and rethrow in deterministic order")
    {
        ParallelExecutor executor(4);
        std::array<bool, 4> completed {};
        bool caught = false;
        try
        {
            executor.run(32, 1, [&](std::size_t, std::size_t, std::size_t partition)
                {
                    completed[partition] = true;
                    throw std::runtime_error(std::to_string(partition));
                });
        }
        catch (const std::runtime_error& error)
        {
            caught = std::string(error.what()) == "0";
        }
        RIGIDBODIES_EXPECT(caught && std::all_of(completed.begin(), completed.end(), [](bool value)
                                         {
                                             return value;
                                         }),
            "first partition error is reported only after all workers finish");
        RIGIDBODIES_EXPECT(executor.run(32, 1, [](std::size_t, std::size_t, std::size_t)
                               {
                               }) == 4,
            "pool remains usable after failure");
    }

    RIGIDBODIES_TEST("workers safely alternate empty serial partial and full batches without stale work")
    {
        ParallelExecutor executor(4);
        const std::array<std::size_t, 6> sizes { 8, 2, 0, 1, 15, 3 };
        for (std::size_t batch = 0; batch < 240; ++batch)
        {
            std::vector<int> values(sizes[batch % sizes.size()]);
            const auto partitions = executor.run(values.size(), 1, [&](std::size_t begin, std::size_t end, std::size_t)
                {
                    for (auto index = begin; index < end; ++index)
                        ++values[index];
                });
            RIGIDBODIES_EXPECT(partitions == std::min<std::size_t>(4, values.size()) &&
                    std::all_of(values.begin(), values.end(), [](int count)
                        {
                            return count == 1;
                        }),
                "inactive workers never reuse a previous batch or duplicate current work");
        }
    }

    RIGIDBODIES_TEST("scheduling rejects unbounded workers and zero batch sizes without changing world settings")
    {
        World world;
        world.set_parallel_settings({ 2, 4 });
        for (const auto invalid : { ParallelSettings { 33, 1 }, ParallelSettings { 2, 0 } })
        {
            bool rejected = false;
            try
            {
                world.set_parallel_settings(invalid);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected && world.parallel_settings().worker_count == 2 && world.parallel_settings().minimum_batch_size == 4, "invalid scheduling is transactional");
        }
        ParallelExecutor automatic;
        RIGIDBODIES_EXPECT(automatic.worker_count() >= 1 && automatic.worker_count() <= 8, "automatic scheduling is bounded on every machine");
    }

    RIGIDBODIES_TEST("reentrant scheduling is rejected on both caller and worker paths without deadlock")
    {
        for (const auto workers : { 1U, 4U })
        {
            ParallelExecutor executor(workers);
            std::array<bool, 4> rejected {};
            executor.run(16, 1, [&](std::size_t, std::size_t, std::size_t partition)
                {
                    try
                    {
                        executor.run(1, 1, [](std::size_t, std::size_t, std::size_t)
                            {
                            });
                    }
                    catch (const std::logic_error&)
                    {
                        rejected[partition] = true;
                    }
                });
            RIGIDBODIES_EXPECT(std::all_of(rejected.begin(), rejected.begin() + workers, [](bool value)
                                   {
                                       return value;
                                   }),
                "every reentrant participant receives a clear error");
            RIGIDBODIES_EXPECT(executor.run(16, 1, [](std::size_t, std::size_t, std::size_t)
                                   {
                                   }) == workers,
                "rejected nesting leaves pool usable");
        }
    }

    RIGIDBODIES_TEST("parallel tree and exhaustive queries match exact filtering order and node statistics")
    {
        std::vector<BroadPhaseProxy> proxies;
        for (std::uint32_t index = 0; index < 90; ++index)
        {
            BroadPhaseProxy proxy;
            proxy.body = { index / 2, index % 3 + 1 };
            proxy.collider_index = index % 2;
            const Vec2 center { static_cast<Real>(index % 15) * 0.2, static_cast<Real>(index / 15) * 0.2 };
            proxy.bounds = { center - Vec2 { 0.25, 0.25 }, center + Vec2 { 0.25, 0.25 } };
            proxy.body_type = index % 3 == 0 ? BodyType::static_body : BodyType::dynamic_body;
            proxy.is_sensor = index % 7 == 0;
            proxy.filter.mask = index % 13 == 0 ? 0 : 0xffff;
            proxies.push_back(proxy);
        }
        proxies.back().bounds.minimum.x = std::numeric_limits<Real>::infinity();
        for (const auto workers : { 1U, 2U, 4U })
        {
            ParallelExecutor executor(workers);
            DynamicTreeBroadPhase tree;
            DynamicTreeBroadPhase serial_tree;
            for (int step = 0; step < 3; ++step)
            {
                std::vector<BroadPhasePair> expected, actual, exhaustive;
                serial_tree.find_pairs(proxies, expected);
                RIGIDBODIES_EXPECT(tree.find_pairs_parallel(proxies, actual, executor, 1) == workers, "tree partitions use requested workers");
                BruteForceBroadPhase {}.find_pairs_parallel(proxies, exhaustive, executor, 1);
                expect_pairs(actual, expected);
                expect_pairs(exhaustive, expected);
                RIGIDBODIES_EXPECT(tree.validate_tree() && tree.statistics().last_node_visits == serial_tree.statistics().last_node_visits, "parallel readers preserve cache validity and exact visit totals");
                proxies[3].bounds.minimum.x += 0.04;
                proxies[3].bounds.maximum.x += 0.04;
            }
        }
    }

    RIGIDBODIES_TEST("one two and four workers replay exact bodies contacts sensors and persistent impulses")
    {
        World serial, two, four;
        populate(serial);
        const auto initial = serial.snapshot();
        two.restore(initial);
        four.restore(initial);
        serial.set_parallel_settings({ 1, 1 });
        two.set_parallel_settings({ 2, 1 });
        four.set_parallel_settings({ 4, 1 });
        for (int step = 0; step < 60; ++step)
        {
            serial.step(1.0 / 120.0);
            two.step(1.0 / 120.0);
            four.step(1.0 / 120.0);
            expect_world(two, serial);
            expect_world(four, serial);
        }
        RIGIDBODIES_EXPECT(four.profile().broad_phase_workers == 4 && four.profile().narrow_phase_workers == 4, "both geometric phases actually use all four workers");
        const auto removed = serial.body_ids()[2];
        RIGIDBODIES_EXPECT(serial.destroy_body(removed) && two.destroy_body(removed) && four.destroy_body(removed), "same contacted body is removed");
        serial.step(1.0 / 120.0);
        two.step(1.0 / 120.0);
        four.step(1.0 / 120.0);
        expect_world(two, serial);
        expect_world(four, serial);
    }

    RIGIDBODIES_TEST("parallel CCD sweeps preserve fast projectile clamping and speculative results")
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        World serial(settings), parallel(settings);
        serial.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        serial.set_broad_phase(std::make_shared<DynamicTreeBroadPhase>());
        serial.set_contact_solver(std::make_shared<NullContactSolver>());
        for (int index = 0; index < 24; ++index)
        {
            const auto y = static_cast<Real>(index) * 2.0;
            (void)serial.create_body(body(make_circle(0.05), { -1.0, y }, BodyType::dynamic_body, { 100.0, 0.0 }));
            (void)serial.create_body(body(make_segment({ 0.0, -0.5 }, { 0.0, 0.5 }), { 0.0, y }, BodyType::static_body));
        }
        parallel.restore(serial.snapshot());
        serial.set_parallel_settings({ 1, 1 });
        parallel.set_parallel_settings({ 4, 1 });
        serial.step(0.02);
        parallel.step(0.02);
        expect_world(parallel, serial);
        RIGIDBODIES_EXPECT(parallel.statistics().ccd_clamped_body_count == 24 && parallel.profile().narrow_phase_workers == 4, "fast independent crossings exercise parallel continuous collision");
    }

    class RecordingNarrowPhase final : public NarrowPhase
    {
    public:
        mutable std::vector<std::thread::id> callers;
        std::string_view name() const override
        {
            return "recording";
        }
        bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const override
        {
            callers.push_back(std::this_thread::get_id());
            return CollisionNarrowPhase {}.collide(query, manifold);
        }
    };

    class RecordingBroadPhase final : public BroadPhase
    {
    public:
        mutable std::thread::id caller;
        std::string_view name() const override
        {
            return "recording";
        }
        void find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const override
        {
            caller = std::this_thread::get_id();
            BruteForceBroadPhase {}.find_pairs(proxies, pairs);
        }
    };

    RIGIDBODIES_TEST("custom broad and narrow callbacks stay on the calling thread without clone requirements")
    {
        World world;
        populate(world);
        const auto broad = std::make_shared<RecordingBroadPhase>();
        const auto narrow = std::make_shared<RecordingNarrowPhase>();
        world.set_broad_phase(broad);
        world.set_narrow_phase(narrow);
        world.set_parallel_settings({ 4, 1 });
        world.step(1.0 / 120.0);
        RIGIDBODIES_EXPECT(broad->caller == std::this_thread::get_id() && narrow->callers.size() > 8 &&
                std::all_of(narrow->callers.begin(), narrow->callers.end(), [](auto caller)
                    {
                        return caller == std::this_thread::get_id();
                    }),
            "custom cache and callback code remains serial");
        RIGIDBODIES_EXPECT(world.profile().broad_phase_workers == 1 && world.profile().narrow_phase_workers == 1, "fallback counters report actual serial execution");
    }

    class RecordingShape final : public Shape
    {
    public:
        mutable std::vector<std::thread::id> callers;
        ShapeKind kind() const override
        {
            return ShapeKind::circle;
        }
        std::shared_ptr<Shape> clone() const override
        {
            return std::make_shared<RecordingShape>(*this);
        }
        rigidbodies::math::Aabb compute_bounds(const rigidbodies::math::Transform2& transform) const override
        {
            return circle.compute_bounds(transform);
        }
        MassProperties compute_mass_properties(Real density) const override
        {
            return circle.compute_mass_properties(density);
        }
        Vec2 support_point(const Vec2& direction) const override
        {
            callers.push_back(std::this_thread::get_id());
            return circle.support_point(direction);
        }
        bool contains_local_point(const Vec2& point) const override
        {
            return circle.contains_local_point(point);
        }
        Real bounding_radius() const override
        {
            return circle.bounding_radius();
        }

    private:
        CircleShape circle { 0.5 };
    };

    RIGIDBODIES_TEST("custom support geometry keeps built-in narrow phase serial even when shapes are shared")
    {
        World world;
        const auto shape = std::make_shared<RecordingShape>();
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_parallel_settings({ 4, 1 });
        for (int index = 0; index < 16; ++index)
            (void)world.create_body(body(shape, { static_cast<Real>(index) * 0.6, 0.0 }));
        world.step(1.0 / 120.0);
        RIGIDBODIES_EXPECT(!shape->callers.empty() && std::all_of(shape->callers.begin(), shape->callers.end(), [](auto caller)
                                                          {
                                                              return caller == std::this_thread::get_id();
                                                          }),
            "custom support callbacks are never invoked concurrently");
        RIGIDBODIES_EXPECT(world.profile().narrow_phase_workers == 1, "custom geometry activates the serial fallback");
    }

    RIGIDBODIES_TEST("snapshot restores scheduling choices and worlds execute independently after restoration")
    {
        World first, second;
        populate(first);
        first.set_parallel_settings({ 2, 3 });
        first.step(1.0 / 120.0);
        second.restore(first.snapshot());
        RIGIDBODIES_EXPECT(second.parallel_settings().worker_count == 2 && second.parallel_settings().minimum_batch_size == 3, "snapshot retains runtime scheduling preferences");
        std::thread worker([&]
            {
                first.step(1.0 / 120.0);
            });
        second.step(1.0 / 120.0);
        worker.join();
        expect_world(second, first);
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
