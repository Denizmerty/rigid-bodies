#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <map>
#include <string>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyId ball(World& world, std::string key, Vec2 position, BodyType type = BodyType::dynamic_body, bool sensor = false)
    {
        BodyDefinition definition;
        definition.name = key;
        definition.position_m = position;
        definition.type = type;
        Collider collider;
        collider.shape = make_circle(0.1);
        collider.is_sensor = sensor;
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, std::move(key));
        if (type == BodyType::dynamic_body)
            world.find_body(id)->override_mass(1.0);
        return id;
    }

    World quiet_world()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.quiet_duration_s = 0.03;
        return World(settings);
    }

    const SimulationIsland& containing(const World& world, BodyId id)
    {
        const auto& islands = world.simulation_islands();
        const auto found = std::find_if(islands.begin(), islands.end(), [id](const auto& island)
            {
                return std::find(island.bodies.begin(), island.bodies.end(), id) != island.bodies.end();
            });
        if (found == islands.end())
            throw std::runtime_error("Expected body is missing from simulation islands");
        return *found;
    }

    std::shared_ptr<JointConstraint> link(World& world, BodyId first, BodyId second, Real length, std::string key = {})
    {
        DistanceJointDefinition definition;
        definition.first = first;
        definition.second = second;
        definition.length_m = length;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, std::move(key));
        return joint;
    }

    RIGIDBODIES_TEST("separate bodies touching a shared floor form separate dynamic islands")
    {
        auto world = quiet_world();
        BodyDefinition floor;
        floor.type = BodyType::static_body;
        floor.position_m = { 0.0, -0.1 };
        Collider collider;
        collider.shape = make_box(10.0, 0.2);
        floor.colliders.push_back(collider);
        const auto floor_id = world.create_body(floor, "floor");
        const auto right = ball(world, "right", { 2.0, 0.1 });
        const auto left = ball(world, "left", { -2.0, 0.1 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.manifolds().size() == 2, "each ball has a solid floor contact");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 2, "a shared static floor does not merge otherwise independent dynamics");
        RIGIDBODIES_EXPECT(containing(world, left).bodies == std::vector<BodyId> { left }, "left island contains only its dynamic body");
        RIGIDBODIES_EXPECT(containing(world, right).bodies == std::vector<BodyId> { right }, "right island contains only its dynamic body");
        for (const auto& island : world.simulation_islands())
        {
            RIGIDBODIES_EXPECT(island.manifold_indices.size() == 1 && !island.serial_fallback, "each built-in solver island owns its single contact");
            RIGIDBODIES_EXPECT(std::find(island.bodies.begin(), island.bodies.end(), floor_id) == island.bodies.end(), "shared boundaries are not dynamic island members");
        }
        RIGIDBODIES_EXPECT(world.statistics().simulation_island_count == 2 && world.statistics().largest_island_body_count == 1, "statistics describe the actual partition");
    }

    RIGIDBODIES_TEST("enabled springs and joints connect islands while disabled links do not")
    {
        auto world = quiet_world();
        const auto a = ball(world, "a", { 0.0, 0.0 });
        const auto b = ball(world, "b", { 1.0, 0.0 });
        const auto c = ball(world, "c", { 2.0, 0.0 });
        const auto d = ball(world, "d", { 3.0, 0.0 });
        const auto e = ball(world, "e", { 4.0, 0.0 });
        const auto joint = link(world, a, b, 1.0, "enabled");
        const auto disabled_joint = link(world, d, e, 1.0, "disabled");
        disabled_joint->set_enabled(false);
        LinearSpringDefinition spring;
        spring.first = b;
        spring.second = c;
        const auto enabled_spring = world.create_spring(spring);
        spring.first = c;
        spring.second = d;
        spring.enabled = false;
        (void)world.create_spring(spring);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 3, "active pair relationships form one triple and two singletons");
        RIGIDBODIES_EXPECT(containing(world, a).bodies == std::vector<BodyId> { a, b, c }, "joint and spring connectivity combine transitively");
        RIGIDBODIES_EXPECT(containing(world, a).constraint_indices.size() == 1, "only the active built-in joint is scheduled");
        RIGIDBODIES_EXPECT(containing(world, d).bodies.size() == 1 && containing(world, e).bodies.size() == 1, "disabled links do not couple islands");
        RIGIDBODIES_EXPECT(world.set_spring_enabled(enabled_spring, false), "spring can be switched off");
        joint->set_enabled(false);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 5, "partition is rebuilt when link topology changes");
    }

    RIGIDBODIES_TEST("independent islands sleep and wake without disturbing one another")
    {
        auto world = quiet_world();
        const auto a = ball(world, "a", { -3.0, 0.0 });
        const auto b = ball(world, "b", { -2.0, 0.0 });
        const auto c = ball(world, "c", { 3.0, 0.0 });
        (void)link(world, a, b, 1.0);
        world.find_body(c)->set_linear_velocity({ 0.5, 0.0 });
        for (int step = 0; step < 8; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!containing(world, a).awake && containing(world, c).awake, "a quiet connected group sleeps while a separate moving group stays active");
        RIGIDBODIES_EXPECT(world.statistics().sleeping_island_count == 1 && world.statistics().active_island_count == 1, "sleep statistics count independently scheduled groups");
        world.find_body(c)->set_linear_velocity({});
        for (int step = 0; step < 8; ++step)
            world.step(0.01);
        const auto c_position = world.find_body(c)->position_m();
        world.find_body(a)->apply_linear_impulse({ 0.1, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(a)->is_awake() && world.find_body(b)->is_awake(), "one member's external impulse wakes its whole island");
        RIGIDBODIES_EXPECT(!world.find_body(c)->is_awake() && world.find_body(c)->position_m() == c_position, "the unrelated sleeping island stays motionless");
    }

    RIGIDBODIES_TEST("sensor contacts remain observable without merging solver islands")
    {
        auto world = quiet_world();
        const auto first = ball(world, "first", {}, BodyType::dynamic_body, true);
        const auto second = ball(world, "second", { 0.1, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.manifolds().size() == 1 && world.manifolds().front().is_sensor, "sensor overlap remains a collision observation");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 2, "sensor overlap does not make a physical solver connection");
        RIGIDBODIES_EXPECT(containing(world, first).manifold_indices.empty() && containing(world, second).manifold_indices.empty(), "sensor manifolds receive no contact solve work");
        RIGIDBODIES_EXPECT(!world.contact_events().empty(), "sensor begin events survive island partitioning");
    }

    RIGIDBODIES_TEST("canonical island order and snapshot replay survive different allocation histories")
    {
        const auto populate = [](World& world, bool reverse)
        {
            std::map<std::string, BodyId> ids;
            const std::vector<std::string> order = reverse ? std::vector<std::string> { "d", "c", "b", "a" } : std::vector<std::string> { "a", "b", "c", "d" };
            if (reverse)
            {
                const auto discarded = ball(world, "discarded", { 20.0, 20.0 });
                world.destroy_body(discarded);
            }
            for (const auto& key : order)
                ids[key] = ball(world, key, { static_cast<Real>(key.front() - 'a'), 0.0 });
            (void)link(world, ids["a"], ids["b"], 1.0, "ab");
            (void)link(world, ids["c"], ids["d"], 1.0, "cd");
            world.find_body(ids["a"])->set_linear_velocity({ 0.4, 0.0 });
            return ids;
        };
        auto first = quiet_world(), second = quiet_world();
        const auto a = populate(first, false), b = populate(second, true);
        first.step(0.01);
        second.step(0.01);
        RIGIDBODIES_EXPECT(first.simulation_islands().size() == 2 && second.simulation_islands().size() == 2, "equivalent worlds produce equivalent partitions");
        for (std::size_t index = 0; index < 2; ++index)
        {
            const auto& left = first.simulation_islands()[index].bodies;
            const auto& right = second.simulation_islands()[index].bodies;
            RIGIDBODIES_EXPECT(left.size() == right.size(), "equivalent canonical islands have matching sizes");
            for (std::size_t member = 0; member < left.size(); ++member)
                RIGIDBODIES_EXPECT(first.find_body(left[member])->name() == second.find_body(right[member])->name(), "island and member order follows stable keys rather than dense layout");
        }
        const auto snapshot = first.snapshot();
        for (int step = 0; step < 4; ++step)
        {
            first.step(0.01);
            second.step(0.01);
        }
        for (const auto& item : a)
            RIGIDBODIES_EXPECT(first.find_body(item.second)->position_m() == second.find_body(b.at(item.first))->position_m(), "independent allocation history does not change solved trajectories");
        const auto expected = *first.find_body(a.at("a"));
        first.restore(snapshot);
        for (int step = 0; step < 4; ++step)
            first.step(0.01);
        RIGIDBODIES_EXPECT(first.find_body(a.at("a"))->position_m() == expected.position_m() && first.find_body(a.at("a"))->linear_velocity_m_s() == expected.linear_velocity_m_s(), "snapshot rebuilds island caches and replays exactly");
    }

    class RecordingSolver final : public ContactSolver
    {
    public:
        explicit RecordingSolver(std::vector<std::string>& log, ConstraintPtr enable_on_prepare = {})
            : log_(log), enable_on_prepare_(std::move(enable_on_prepare))
        {
        }
        std::string_view name() const override
        {
            return "recording_solver";
        }
        void prepare(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            RIGIDBODIES_EXPECT(world.body_ids().size() == 4, "custom solver receives the complete world");
            log_.push_back("solver prepare");
            if (enable_on_prepare_)
                enable_on_prepare_->set_enabled(true);
        }
        void solve_velocity(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            log_.push_back("solver velocity");
        }
        void solve_position(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            log_.push_back("solver position");
        }

    private:
        std::vector<std::string>& log_;
        ConstraintPtr enable_on_prepare_;
    };

    class RecordingConstraint final : public Constraint
    {
    public:
        RecordingConstraint(BodyId first, BodyId second, std::string key, std::vector<std::string>& log, ConstraintPtr enable_on_prepare = {})
            : first_(first), second_(second), key_(std::move(key)), log_(log), enable_on_prepare_(std::move(enable_on_prepare))
        {
        }
        std::string_view name() const override
        {
            return key_;
        }
        BodyId first_body() const override
        {
            return first_;
        }
        BodyId second_body() const override
        {
            return second_;
        }
        void prepare(World&, Real) override
        {
            log_.push_back(key_ + " prepare");
            if (enable_on_prepare_)
                enable_on_prepare_->set_enabled(true);
        }
        void solve_velocity(World&, Real) override
        {
            log_.push_back(key_ + " velocity");
        }
        void finalize_velocity(World&, Real) override
        {
            log_.push_back(key_ + " finalize");
        }
        bool solve_position(World&, Real) override
        {
            log_.push_back(key_ + " position");
            return true;
        }

    private:
        BodyId first_, second_;
        std::string key_;
        std::vector<std::string>& log_;
        ConstraintPtr enable_on_prepare_;
    };

    RIGIDBODIES_TEST("custom solvers and constraints retain complete world callback order without cloning")
    {
        auto world = quiet_world();
        auto settings = world.settings();
        settings.solver.velocity_iterations = 2;
        settings.solver.position_iterations = 1;
        world.set_settings(settings);
        const auto a = ball(world, "a", {}), b = ball(world, "b", { 1.0, 0.0 });
        const auto c = ball(world, "c", { 3.0, 0.0 }), d = ball(world, "d", { 4.0, 0.0 });
        std::vector<std::string> events;
        world.set_contact_solver(std::make_shared<RecordingSolver>(events));
        world.add_constraint(std::make_shared<RecordingConstraint>(c, d, "second", events), "b");
        world.add_constraint(std::make_shared<RecordingConstraint>(a, b, "first", events), "a");
        world.step(0.01);
        const std::vector<std::string> expected {
            "solver prepare", "first prepare", "second prepare", "solver velocity", "first velocity", "second velocity", "solver velocity", "first velocity", "second velocity", "first finalize", "second finalize", "solver position", "first position", "second position"
        };
        RIGIDBODIES_EXPECT(events == expected, "extensions retain the original prepare/iteration/finalize/position traversal");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 1 && world.simulation_islands().front().serial_fallback && world.simulation_islands().front().bodies.size() == 4, "unknown world-mutating extensions use one explicit serial fallback group");
    }

    RIGIDBODIES_TEST("custom constraints trigger fallback even with the built in contact solver")
    {
        auto world = quiet_world();
        const auto a = ball(world, "a", {}), b = ball(world, "b", { 1.0, 0.0 });
        const auto independent = ball(world, "c", { 5.0, 0.0 });
        std::vector<std::string> events;
        world.add_constraint(std::make_shared<RecordingConstraint>(a, b, "custom", events));
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 1 && containing(world, independent).serial_fallback, "unknown constraint dependencies prevent unsafe partitioned callback execution");
        RIGIDBODIES_EXPECT(events.front() == "custom prepare" && events.back() == "custom position", "a noncloneable custom constraint remains usable");
    }

    RIGIDBODIES_TEST("custom solver preparation can enable a constraint for every subsequent phase")
    {
        auto world = quiet_world();
        auto settings = world.settings();
        settings.solver.velocity_iterations = 2;
        settings.solver.position_iterations = 1;
        world.set_settings(settings);
        const auto a = ball(world, "a", {}), b = ball(world, "b", { 1.0, 0.0 });
        (void)ball(world, "c", { 3.0, 0.0 });
        (void)ball(world, "d", { 4.0, 0.0 });
        std::vector<std::string> events;
        auto initially_disabled = std::make_shared<RecordingConstraint>(a, b, "enabled", events);
        initially_disabled->set_enabled(false);
        world.add_constraint(initially_disabled);
        world.set_contact_solver(std::make_shared<RecordingSolver>(events, initially_disabled));
        world.step(0.01);
        const std::vector<std::string> expected {
            "solver prepare", "enabled prepare", "solver velocity", "enabled velocity", "solver velocity", "enabled velocity", "enabled finalize", "solver position", "enabled position"
        };
        RIGIDBODIES_EXPECT(initially_disabled->is_enabled() && events == expected, "fallback rechecks activation after the contact solver prepares instead of freezing its initial active list");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 1 && world.simulation_islands().front().serial_fallback, "the custom solver retains whole-world extension semantics");
    }

    RIGIDBODIES_TEST("custom constraint preparation can enable a later canonical constraint immediately")
    {
        auto world = quiet_world();
        auto settings = world.settings();
        settings.solver.velocity_iterations = 2;
        settings.solver.position_iterations = 1;
        world.set_settings(settings);
        const auto a = ball(world, "a", {}), b = ball(world, "b", { 1.0, 0.0 });
        const auto c = ball(world, "c", { 3.0, 0.0 }), d = ball(world, "d", { 4.0, 0.0 });
        std::vector<std::string> events;
        auto later = std::make_shared<RecordingConstraint>(c, d, "second", events);
        later->set_enabled(false);
        world.add_constraint(later, "b");
        world.add_constraint(std::make_shared<RecordingConstraint>(a, b, "first", events, later), "a");
        world.step(0.01);
        const std::vector<std::string> expected {
            "first prepare", "second prepare", "first velocity", "second velocity", "first velocity", "second velocity", "first finalize", "second finalize", "first position", "second position"
        };
        RIGIDBODIES_EXPECT(later->is_enabled() && events == expected, "a canonical callback can activate a later callback in the same prepare traversal and all solve passes");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == 1 && containing(world, d).serial_fallback, "newly active hidden dependencies stay inside the complete-world fallback");
    }

    RIGIDBODIES_TEST("profiling is optional diagnostic state and snapshots reset wall clock measurements")
    {
        auto world = quiet_world();
        const auto id = ball(world, "body", {});
        world.find_body(id)->set_linear_velocity({ 1.0, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(!world.profiling_enabled() && world.profile().total_s == 0.0 && world.profile().forces_s == 0.0 && world.profile().wake_s == 0.0, "default operation leaves timing counters at zero");
        world.set_profiling_enabled(true);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.profile().total_s > 0.0 && world.profile().step_index == world.statistics().step_index, "opted-in timing describes the completed step");
        RIGIDBODIES_EXPECT(world.profile().total_s >= world.profile().forces_s && world.profile().total_s >= world.profile().velocity_solve_s, "phase measurements fit within the complete step");
        const auto snapshot = world.snapshot();
        world.step(0.01);
        const auto position = world.find_body(id)->position_m();
        world.set_profiling_enabled(false);
        RIGIDBODIES_EXPECT(world.profile().total_s == 0.0, "switching timing off clears the last measurement");
        world.restore(snapshot);
        RIGIDBODIES_EXPECT(world.profiling_enabled() && world.profile().total_s == 0.0, "snapshot restores the runtime preference without replaying wall clock measurements");
        RIGIDBODIES_EXPECT(world.simulation_islands().size() == world.statistics().simulation_island_count && containing(world, id).bodies == std::vector<BodyId> { id }, "snapshot retains coherent membership and statistics from its completed step");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(id)->position_m() == position, "timing does not alter deterministic simulation replay");
        world.set_profiling_enabled(false);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.profile().total_s == 0.0 && world.profile().islands_s == 0.0 && world.profile().sleep_s == 0.0, "disabled timers remain dormant on subsequent steps");
    }

    RIGIDBODIES_TEST("reducing timestep cannot break a sleeping joint using an old cached impulse")
    {
        WorldSettings settings;
        settings.sleep.quiet_duration_s = 0.05;
        World world(settings);
        const auto anchor = ball(world, "anchor", {}, BodyType::static_body);
        const auto hanging = ball(world, "hanging", { 0.0, -1.0 });
        DistanceJointDefinition definition;
        definition.first = anchor;
        definition.second = hanging;
        definition.length_m = 1.0;
        definition.break_force_n = 50.0;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint);
        for (int step = 0; step < 100; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(hanging)->is_awake() && !joint->is_broken(), "gravity supported below the break threshold can settle");
        world.step(0.001);
        RIGIDBODIES_EXPECT(!joint->is_broken() && world.constraint_break_events().empty(), "a sleeping joint cannot convert an old support impulse into a fresh breaking force");
        RIGIDBODIES_EXPECT(!world.find_body(hanging)->is_awake(), "changing only the step interval does not disturb the sleeping island");
    }

    RIGIDBODIES_TEST("nonzero kinematic boundary speed below sleep thresholds activates contact and joint islands")
    {
        for (int mode = 0; mode < 2; ++mode)
        {
            auto world = quiet_world();
            const auto boundary = ball(world, "boundary", {}, BodyType::kinematic_body);
            const auto sleeper = ball(world, "sleeper", { 0.0, mode == 0 ? 0.2 : 1.0 });
            if (mode == 1)
                (void)link(world, boundary, sleeper, 1.0);
            world.find_body(boundary)->set_linear_velocity({ 0.0001, 0.0 });
            world.step(0.01); // Consume the explicit edit flag while retaining prescribed motion.
            world.find_body(sleeper)->set_awake(false);
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(sleeper)->is_awake(), "any moving kinematic boundary must reach sleeping solver work, even below quiet thresholds");
            RIGIDBODIES_EXPECT(containing(world, sleeper).awake, "contact and joint scheduling both observe the moving boundary");
        }
    }

    RIGIDBODIES_TEST("joint endpoints becoming nondynamic deactivate old rows before a smaller timestep")
    {
        for (const auto type : { BodyType::static_body, BodyType::kinematic_body })
        {
            World world;
            const auto anchor = ball(world, "anchor", {}, BodyType::static_body);
            const auto hanging = ball(world, "hanging", { 0.0, -1.0 });
            DistanceJointDefinition definition;
            definition.first = anchor;
            definition.second = hanging;
            definition.length_m = 1.0;
            definition.break_force_n = 50.0;
            auto joint = std::make_shared<JointConstraint>(definition);
            world.add_constraint(joint);
            world.step(0.01);
            RIGIDBODIES_EXPECT(joint->velocity_rows() && !joint->velocity_rows()->empty(), "the dynamic endpoint first generates a support row");
            world.find_body(hanging)->set_type(type);
            world.step(0.001);
            RIGIDBODIES_EXPECT(!joint->is_broken() && world.constraint_break_events().empty(), "a joint excluded from all dynamic islands cannot finalize stale support impulses");
            RIGIDBODIES_EXPECT(joint->velocity_rows() == nullptr, "stationary nondynamic endpoints deactivate the old solver rows");
            RIGIDBODIES_EXPECT(world.simulation_islands().empty(), "nondynamic boundaries alone need no dynamic island");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
