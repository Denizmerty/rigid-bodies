#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition ball(std::string name = "ball")
    {
        BodyDefinition result;
        result.name = std::move(name);
        Collider collider;
        collider.shape = make_circle(0.5);
        result.colliders.push_back(collider);
        return result;
    }

    template <typename Function>
    bool rejects(Function&& function)
    {
        try
        {
            function();
        }
        catch (const std::logic_error&)
        {
            return true;
        }
        return false;
    }

    class CountingForce final : public ForceGenerator
    {
    public:
        int calls { 0 };
        std::string_view name() const override
        {
            return "counting_force";
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<CountingForce>(*this);
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center({ static_cast<Real>(++calls), 0.0 });
        }
    };

    class CountingIntegrator final : public Integrator
    {
    public:
        mutable int calls { 0 };
        std::string_view name() const override
        {
            return "counting_integrator";
        }
        std::shared_ptr<Integrator> clone() const override
        {
            return std::make_shared<CountingIntegrator>(*this);
        }
        void integrate_velocity(RigidBody& body, Real dt) const override
        {
            ++calls;
            SemiImplicitEulerIntegrator {}.integrate_velocity(body, dt);
        }
        void integrate_position(RigidBody& body, Real dt) const override
        {
            ++calls;
            SemiImplicitEulerIntegrator {}.integrate_position(body, dt);
        }
    };

    class UnorderedBroadPhase final : public BroadPhase
    {
    public:
        mutable int calls { 0 };
        std::string_view name() const override
        {
            return "unordered_broad_phase";
        }
        std::shared_ptr<BroadPhase> clone() const override
        {
            return std::make_shared<UnorderedBroadPhase>(*this);
        }
        void find_pairs(const std::vector<BroadPhaseProxy>& proxies, std::vector<BroadPhasePair>& pairs) const override
        {
            ++calls;
            BruteForceBroadPhase {}.find_pairs(proxies, pairs);
            for (auto& pair : pairs)
            {
                std::swap(pair.first, pair.second);
                std::swap(pair.first_collider, pair.second_collider);
            }
            std::reverse(pairs.begin(), pairs.end());
            if (!pairs.empty())
            {
                pairs.push_back(pairs.front());
            }
            pairs.push_back({}); // Invalid plugin output cannot reach the narrow phase.
        }
    };

    class CountingNarrowPhase final : public NarrowPhase
    {
    public:
        mutable int calls { 0 };
        std::string_view name() const override
        {
            return "counting_narrow_phase";
        }
        std::shared_ptr<NarrowPhase> clone() const override
        {
            return std::make_shared<CountingNarrowPhase>(*this);
        }
        bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const override
        {
            ++calls;
            manifold.first = query.first;
            manifold.second = query.second;
            manifold.first_collider = query.first_collider;
            manifold.second_collider = query.second_collider;
            manifold.point_count = 1;
            manifold.points[0].normal_impulse_n_s = 3.0;
            return true;
        }
    };

    class CountingSolver final : public ContactSolver
    {
    public:
        int calls { 0 };
        std::string_view name() const override
        {
            return "counting_solver";
        }
        std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<CountingSolver>(*this);
        }
        void prepare(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            ++calls;
        }
        void solve_velocity(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            ++calls;
        }
        void solve_position(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            ++calls;
        }
    };

    class CountingConstraint final : public Constraint
    {
    public:
        BodyId first;
        BodyId second;
        int calls { 0 };
        std::string_view name() const override
        {
            return "counting_constraint";
        }
        std::shared_ptr<Constraint> clone() const override
        {
            return std::make_shared<CountingConstraint>(*this);
        }
        BodyId first_body() const override
        {
            return first;
        }
        BodyId second_body() const override
        {
            return second;
        }
        void prepare(World&, Real) override
        {
            ++calls;
        }
        void solve_velocity(World&, Real) override
        {
            ++calls;
        }
        bool solve_position(World&, Real) override
        {
            ++calls;
            return true;
        }
    };

    class UnsupportedForce : public ForceGenerator
    {
    public:
        std::string_view name() const override
        {
            return "unsupported_force";
        }
        void apply(RigidBody&, const ForceContext&) override
        {
        }
    };

    class AliasingForce final : public UnsupportedForce, public std::enable_shared_from_this<AliasingForce>
    {
    public:
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::const_pointer_cast<AliasingForce>(shared_from_this());
        }
    };

    class LimitedCloneForce final : public UnsupportedForce
    {
    public:
        int remaining { 1 };
        std::shared_ptr<ForceGenerator> clone() const override
        {
            if (remaining == 0)
            {
                throw std::logic_error("clone unavailable");
            }
            auto result = std::make_shared<LimitedCloneForce>(*this);
            --result->remaining;
            return result;
        }
    };

    RIGIDBODIES_TEST("snapshot restores body state settings time and exact identifier allocation")
    {
        World world;
        auto definition = ball();
        definition.linear_velocity_m_s = { 2.0, 3.0 };
        definition.angular_velocity_rad_s = 0.75;
        const auto id = world.create_body(definition, "subject");
        const auto removed = world.create_body(ball(), "removed");
        world.destroy_body(removed);
        world.find_body(id)->override_mass(7.0);
        world.set_potential_energy_reference_height(1.25);
        world.step(0.01);
        world.find_body(id)->apply_force_at_center({ 4.0, 5.0 });
        world.find_body(id)->apply_torque(6.0);
        const auto before = *world.find_body(id);
        const auto checkpoint = world.snapshot();
        const auto expected_next = world.create_body(ball(), "new");
        world.clear();
        auto changed = world.settings();
        changed.gravity_m_s2 = {};
        changed.solver.velocity_iterations = 2;
        world.set_settings(changed);
        world.set_potential_energy_reference_height(-99.0);
        world.restore(checkpoint);

        RIGIDBODIES_EXPECT(world.is_valid(id) && !world.is_valid(removed), "live and stale handles restored");
        const auto* restored = world.find_body(id);
        RIGIDBODIES_EXPECT(restored->position_m() == before.position_m(), "body placement restored exactly");
        RIGIDBODIES_EXPECT(restored->linear_velocity_m_s() == before.linear_velocity_m_s(), "velocity restored exactly");
        RIGIDBODIES_EXPECT(restored->accumulated_force_n() == Vec2 { 4.0, 5.0 }, "pending forces restored");
        RIGIDBODIES_EXPECT_NEAR(restored->accumulated_torque_n_m(), 6.0, 0.0, "pending torque restored");
        RIGIDBODIES_EXPECT_NEAR(restored->mass_properties().mass_kg, 7.0, 0.0, "mass override preserved");
        RIGIDBODIES_EXPECT_NEAR(world.potential_energy_reference_height_m(), 1.25, 0.0, "reference height restored");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().elapsed_time_s, 0.01, 0.0, "elapsed simulation time restored");
        RIGIDBODIES_EXPECT(world.statistics().step_index == 1, "step counter restored");
        RIGIDBODIES_EXPECT(world.settings().solver.velocity_iterations == 8, "solver settings restored");
        RIGIDBODIES_EXPECT(world.settings().gravity_m_s2 == Vec2 { 0.0, -standard_gravity_m_s2 }, "gravity restored");
        RIGIDBODIES_EXPECT(world.create_body(ball(), "new") == expected_next, "free slots and generation allocator restored");
    }

    RIGIDBODIES_TEST("snapshot detaches shapes generators and repeated restored worlds")
    {
        World world;
        auto definition = ball();
        const auto shape = std::dynamic_pointer_cast<CircleShape>(definition.colliders[0].shape);
        const auto first = world.create_body(definition, "first");
        const auto second = world.create_body(definition, "second");
        auto attractor = std::make_shared<PointAttractor>(Vec2 { 2.0, 3.0 }, 4.0);
        attractor->set_enabled(false);
        world.add_force_generator(attractor);
        const auto checkpoint = world.snapshot();
        *shape = CircleShape(9.0);
        attractor->set_stiffness(99.0);
        attractor->set_enabled(true);
        world.restore(checkpoint);
        const auto first_shape = world.find_body(first)->colliders()[0].shape;
        const auto second_shape = world.find_body(second)->colliders()[0].shape;
        RIGIDBODIES_EXPECT(first_shape == second_shape, "internal shared shape topology preserved");
        RIGIDBODIES_EXPECT(first_shape != shape, "external mutable shape detached");
        RIGIDBODIES_EXPECT_NEAR(first_shape->bounding_radius(), 0.5, 0.0, "saved geometry unaffected by external assignment");
        auto restored_attractor = std::dynamic_pointer_cast<PointAttractor>(world.force_generators()[1]);
        RIGIDBODIES_EXPECT(restored_attractor != attractor && !restored_attractor->is_enabled(), "generator and enabled state detached");
        RIGIDBODIES_EXPECT_NEAR(restored_attractor->stiffness_n_m(), 4.0, 0.0, "generator parameters restored");
        restored_attractor->set_stiffness(88.0);
        World other;
        other.restore(checkpoint);
        RIGIDBODIES_EXPECT_NEAR(std::dynamic_pointer_cast<PointAttractor>(other.force_generators()[1])->stiffness_n_m(), 4.0, 0.0, "restored world cannot mutate checkpoint");
        RIGIDBODIES_EXPECT(other.find_body(first)->colliders()[0].shape != first_shape, "restored worlds share no mutable geometry");
    }

    RIGIDBODIES_TEST("snapshot preserves state of every pluggable phase constraints and contact caches")
    {
        World world;
        auto force = std::make_shared<CountingForce>();
        auto integrator = std::make_shared<CountingIntegrator>();
        auto broad = std::make_shared<UnorderedBroadPhase>();
        auto narrow = std::make_shared<CountingNarrowPhase>();
        auto solver = std::make_shared<CountingSolver>();
        auto constraint = std::make_shared<CountingConstraint>();
        constraint->first = world.create_body(ball(), "a");
        constraint->second = world.create_body(ball(), "b");
        world.add_force_generator(force);
        world.add_force_generator(force);
        world.set_integrator(integrator);
        world.set_broad_phase(broad);
        world.set_narrow_phase(narrow);
        world.set_contact_solver(solver);
        world.add_constraint(constraint);
        world.step(0.01);
        constraint->set_enabled(false);
        const auto checkpoint = world.snapshot();
        world.step(0.01);
        world.restore(checkpoint);
        RIGIDBODIES_EXPECT(std::dynamic_pointer_cast<CountingForce>(world.force_generators()[1])->calls == 4, "stateful generator restored");
        RIGIDBODIES_EXPECT(world.force_generators()[1] == world.force_generators()[2], "repeated registrations preserve shared state");
        RIGIDBODIES_EXPECT(world.force_generators()[1] != force, "generator clone independent");
        RIGIDBODIES_EXPECT(dynamic_cast<const CountingIntegrator&>(world.integrator()).calls == 4, "integrator state restored");
        RIGIDBODIES_EXPECT(dynamic_cast<const UnorderedBroadPhase&>(world.broad_phase()).calls == 1, "broad phase state restored");
        RIGIDBODIES_EXPECT(dynamic_cast<const CountingNarrowPhase&>(world.narrow_phase()).calls == 1, "narrow phase state restored");
        RIGIDBODIES_EXPECT(dynamic_cast<const CountingSolver&>(world.contact_solver()).calls == 12, "solver state restored");
        auto restored = std::dynamic_pointer_cast<CountingConstraint>(world.constraints()[0]);
        RIGIDBODIES_EXPECT(restored->calls == 12 && !restored->is_enabled(), "constraint state restored");
        RIGIDBODIES_EXPECT(restored->first == constraint->first && restored->second == constraint->second, "constraint handles preserved");
        RIGIDBODIES_EXPECT(restored != constraint, "constraint clone independent");
        RIGIDBODIES_EXPECT(world.broad_phase_pairs().size() == 1 && world.manifolds().size() == 1, "collision state restored");
        RIGIDBODIES_EXPECT_NEAR(world.manifolds()[0].points[0].normal_impulse_n_s, 3.0, 0.0, "contact impulse cache restored");
        RIGIDBODIES_EXPECT(world.statistics().contact_point_count == 1, "contact statistics restored");
    }

    RIGIDBODIES_TEST("snapshot replay reproduces exact trajectories and accumulated statistics")
    {
        World world;
        const auto id = world.create_body(ball(), "subject");
        world.add_force_generator(std::make_shared<CountingForce>());
        for (int step = 0; step < 20; ++step)
        {
            world.step(1.0 / 120.0);
        }
        const auto checkpoint = world.snapshot();
        for (int step = 0; step < 40; ++step)
        {
            world.step(1.0 / 120.0);
        }
        const auto expected = *world.find_body(id);
        const auto statistics = world.statistics();
        world.restore(checkpoint);
        for (int step = 0; step < 40; ++step)
        {
            world.step(1.0 / 120.0);
        }
        const auto* actual = world.find_body(id);
        RIGIDBODIES_EXPECT(actual->position_m() == expected.position_m(), "positions match bit for bit");
        RIGIDBODIES_EXPECT(actual->linear_velocity_m_s() == expected.linear_velocity_m_s(), "velocities match bit for bit");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_kinetic_energy_j, statistics.total_kinetic_energy_j, 0.0, "energy matches exactly");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().elapsed_time_s, statistics.elapsed_time_s, 0.0, "replay time matches exactly");
        RIGIDBODIES_EXPECT(world.statistics().step_index == statistics.step_index, "replay steps match");
    }

    RIGIDBODIES_TEST("stable keys govern body force and pair order independently of allocation history")
    {
        World first;
        World second;
        // Keep the deliberately coincident bodies overlapping while testing pair normalization.
        first.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        second.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        first.create_body(ball("c"), "c");
        first.create_body(ball("a"), "a");
        first.create_body(ball("b"), "b");
        const auto deleted = second.create_body(ball("discarded"), "discarded");
        second.create_body(ball("b"), "b");
        second.destroy_body(deleted);
        second.create_body(ball("c"), "c");
        second.create_body(ball("a"), "a");
        first.add_force_generator(std::make_shared<CountingForce>());
        second.add_force_generator(std::make_shared<CountingForce>());
        second.set_broad_phase(std::make_shared<UnorderedBroadPhase>());
        const auto first_ids = first.body_ids();
        const auto second_ids = second.body_ids();
        for (int step = 0; step < 10; ++step)
        {
            first.step(0.001);
            second.step(0.001);
        }
        for (std::size_t index = 0; index < first_ids.size(); ++index)
        {
            const auto* left = first.find_body(first_ids[index]);
            const auto* right = second.find_body(second_ids[index]);
            RIGIDBODIES_EXPECT(left->name() == std::string(1, static_cast<char>('a' + index)), "iteration follows stable keys");
            RIGIDBODIES_EXPECT(left->name() == right->name() && left->position_m() == right->position_m(), "creation history does not change motion");
        }
        const auto& left_pairs = first.broad_phase_pairs();
        const auto& right_pairs = second.broad_phase_pairs();
        RIGIDBODIES_EXPECT(left_pairs.size() == 3 && right_pairs.size() == 3, "duplicates and invalid plugin pairs removed");
        for (std::size_t index = 0; index < left_pairs.size(); ++index)
        {
            RIGIDBODIES_EXPECT(first.find_body(left_pairs[index].first)->name() == second.find_body(right_pairs[index].first)->name(), "first pair endpoint canonical");
            RIGIDBODIES_EXPECT(first.find_body(left_pairs[index].second)->name() == second.find_body(right_pairs[index].second)->name(), "second pair endpoint canonical");
        }
        RIGIDBODIES_EXPECT_NEAR(first.statistics().total_kinetic_energy_j, second.statistics().total_kinetic_energy_j, 0.0, "reduction order canonical");
    }

    RIGIDBODIES_TEST("duplicate stable keys fail cleanly and destroyed keys can be reused")
    {
        World world;
        const auto first = world.create_body(ball(), "subject");
        RIGIDBODIES_EXPECT(rejects([&]
                               {
                                   world.create_body(ball(), "subject");
                               }),
            "duplicate key rejected");
        RIGIDBODIES_EXPECT(world.body_ids().size() == 1 && world.is_valid(first), "failed insertion leaves world intact");
        world.destroy_body(first);
        const auto replacement = world.create_body(ball(), "subject");
        RIGIDBODIES_EXPECT(world.is_valid(replacement) && !world.is_valid(first), "destroyed key reusable with fresh handle");
    }

    RIGIDBODIES_TEST("unkeyed bodies keep creation order when slots are recycled and picking favors newest")
    {
        World world;
        const auto oldest = world.create_body(ball());
        const auto middle = world.create_body(ball());
        world.destroy_body(oldest);
        const auto newest = world.create_body(ball());
        RIGIDBODIES_EXPECT(world.body_ids()[0] == middle && world.body_ids()[1] == newest, "slot reuse does not reorder existing bodies");
        RIGIDBODIES_EXPECT(world.find_body_at_point({}) == newest, "newest overlapping body picked");
    }

    RIGIDBODIES_TEST("body visitors may destroy the current body or clear the world")
    {
        World world;
        for (const auto* key : { "a", "b", "c" })
        {
            world.create_body(ball(key), key);
        }
        std::vector<BodyId> visited;
        world.for_each_body([&](BodyId id, RigidBody&)
            {
                visited.push_back(id);
                world.destroy_body(id);
            });
        RIGIDBODIES_EXPECT(visited.size() == 3 && world.body_ids().empty(), "destroying current body does not skip later bodies");
        for (const auto* key : { "a", "b", "c" })
        {
            world.create_body(ball(key), key);
        }
        visited.clear();
        world.for_each_body([&](BodyId id, RigidBody&)
            {
                visited.push_back(id);
                world.clear();
            });
        RIGIDBODIES_EXPECT(visited.size() == 1 && world.body_ids().empty(), "cleared bodies are skipped without dereference");
    }

    RIGIDBODIES_TEST("body visitors skip replacements and newly created bodies")
    {
        World world;
        const auto first = world.create_body(ball("a"), "a");
        const auto removed = world.create_body(ball("b"), "b");
        const auto last = world.create_body(ball("c"), "c");
        BodyId replacement;
        std::vector<BodyId> visited;
        world.for_each_body([&](BodyId id, RigidBody&)
            {
                visited.push_back(id);
                if (id == first)
                {
                    world.destroy_body(removed);
                    replacement = world.create_body(ball("replacement"), "b");
                    // Force both the slot storage and order vector to grow during traversal.
                    for (int index = 0; index < 32; ++index)
                    {
                        world.create_body(ball());
                    }
                }
            });
        RIGIDBODIES_EXPECT(replacement.index == removed.index && !(replacement == removed), "replacement reuses slot with new generation");
        RIGIDBODIES_EXPECT(visited.size() == 2 && visited[0] == first && visited[1] == last,
            "captured generation skips replacement and appended bodies while retaining future live body");
    }

    RIGIDBODIES_TEST("const body visitors tolerate structural mutation through another world reference")
    {
        World world;
        const auto first = world.create_body(ball("a"), "a");
        const auto removed = world.create_body(ball("b"), "b");
        const auto last = world.create_body(ball("c"), "c");
        const World& view = world;
        std::vector<BodyId> visited;
        view.for_each_body([&](BodyId id, const RigidBody&)
            {
                visited.push_back(id);
                if (id == first)
                {
                    world.destroy_body(id);
                    world.destroy_body(removed);
                    world.create_body(ball("replacement"), "b");
                }
                else
                {
                    world.clear();
                }
            });
        RIGIDBODIES_EXPECT(visited.size() == 2 && visited[0] == first && visited[1] == last,
            "const traversal revalidates captured identifiers after external mutations");
        RIGIDBODIES_EXPECT(world.body_ids().empty(), "last callback can clear remaining bodies");
    }

    RIGIDBODIES_TEST("unsupported and aliasing component clones fail explicitly")
    {
        World world;
        const auto id = world.create_body(ball());
        auto unsupported = std::make_shared<UnsupportedForce>();
        world.add_force_generator(unsupported);
        RIGIDBODIES_EXPECT(rejects([&]
                               {
                                   (void)world.snapshot();
                               }),
            "custom force must support snapshots explicitly");
        world.remove_force_generator(unsupported);
        world.add_force_generator(std::make_shared<AliasingForce>());
        RIGIDBODIES_EXPECT(rejects([&]
                               {
                                   (void)world.snapshot();
                               }),
            "clone cannot return original mutable object");
        RIGIDBODIES_EXPECT(world.is_valid(id) && world.body_ids().size() == 1, "failed capture leaves world intact");
    }

    RIGIDBODIES_TEST("failed restore leaves target intact")
    {
        World source;
        source.add_force_generator(std::make_shared<LimitedCloneForce>());
        const auto checkpoint = source.snapshot();
        World target;
        const auto id = target.create_body(ball());
        target.step(0.01);
        const auto position = target.find_body(id)->position_m();
        RIGIDBODIES_EXPECT(rejects([&]
                               {
                                   target.restore(checkpoint);
                               }),
            "clone failure reported");
        RIGIDBODIES_EXPECT(target.is_valid(id) && target.find_body(id)->position_m() == position, "target unchanged after clone failure");
        RIGIDBODIES_EXPECT(rejects([&]
                               {
                                   target.restore(WorldSnapshot {});
                               }),
            "empty snapshot rejected");
        RIGIDBODIES_EXPECT(target.statistics().step_index == 1, "time unchanged after failures");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
