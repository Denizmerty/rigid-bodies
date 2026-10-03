#include <rigidbodies/physics/constraint_graph.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    World mechanism(bool graph = true, bool sleep = false)
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.constraint_graph_enabled = graph;
        settings.solver.velocity_iterations = 1;
        settings.solver.position_iterations = 0;
        settings.sleep.enabled = sleep;
        settings.sleep.quiet_duration_s = 0.05;
        World world(settings);
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        return world;
    }

    BodyId body(World& world, Vec2 position, std::string key, BodyType type = BodyType::dynamic_body)
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.type = type;
        Collider collider;
        collider.shape = make_circle(0.05);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, std::move(key));
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    ConstraintPtr rod(World& world, BodyId a, BodyId b, std::string key, Real length = 1.0)
    {
        DistanceJointDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.length_m = length;
        auto constraint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(constraint, std::move(key));
        return constraint;
    }

    class DiagnosticRows final : public Constraint
    {
    public:
        std::vector<ConstraintRow> rows;
        std::string_view name() const override
        {
            return "diagnostic_rows";
        }
        BodyId first_body() const override
        {
            return rows.front().first;
        }
        BodyId second_body() const override
        {
            return rows.front().second;
        }
        void prepare(World&, Real) override
        {
        }
        void solve_velocity(World&, Real) override
        {
        }
        std::vector<ConstraintRow>* velocity_rows() override
        {
            return &rows;
        }
    };

    std::array<BodyId, 9> chain(World& world, bool reverse = false)
    {
        std::array<BodyId, 9> ids;
        for (std::size_t ordinal = 0; ordinal < ids.size(); ++ordinal)
        {
            const auto i = reverse ? ids.size() - ordinal - 1 : ordinal;
            ids[i] = body(world, { static_cast<Real>(i), 0.0 }, "body/" + std::to_string(i), i == 0 ? BodyType::static_body : BodyType::dynamic_body);
        }
        for (std::size_t ordinal = 1; ordinal < ids.size(); ++ordinal)
        {
            const auto i = reverse ? ids.size() - ordinal : ordinal;
            rod(world, ids[i - 1], ids[i], "joint/" + std::to_string(i));
        }
        world.find_body(ids.back())->set_linear_velocity({ 8.0, 0.0 });
        return ids;
    }

    RIGIDBODIES_TEST("one coupled graph pass propagates an end impulse through the whole anchored chain")
    {
        auto graph = mechanism(true);
        auto sequential = mechanism(false);
        const auto graph_ids = chain(graph), sequential_ids = chain(sequential);
        graph.step(0.01);
        sequential.step(0.01);
        Real graph_error = 0.0, sequential_error = 0.0;
        for (std::size_t i = 1; i < graph_ids.size(); ++i)
        {
            graph_error += std::abs(graph.find_body(graph_ids[i])->linear_velocity_m_s().x);
            sequential_error += std::abs(sequential.find_body(sequential_ids[i])->linear_velocity_m_s().x);
        }
        RIGIDBODIES_EXPECT(graph_error < 1.0e-6 && sequential_error > 1.0, "coupled elimination removes chain residual in one pass");
        RIGIDBODIES_EXPECT(graph.statistics().constraint_island_count == 1 && graph.statistics().constraint_row_count == 8, "all chain rows form one island");
        RIGIDBODIES_EXPECT(graph.statistics().constraint_velocity_residual < sequential.statistics().constraint_velocity_residual * 1.0e-6, "measured convergence improves over the same sequential iteration budget");
    }

    RIGIDBODIES_TEST("graph diagnostics measure complementarity at impulse bounds and equality residual on free rows")
    {
        auto world = mechanism();
        const auto a = body(world, {}, "a", BodyType::static_body), b = body(world, {}, "b");
        auto constraint = std::make_shared<DiagnosticRows>();
        ConstraintRow row;
        row.first = a;
        row.second = b;
        row.linear_second = { 1.0, 0.0 };
        row.target_velocity = 3.0;
        constraint->rows.push_back(row);
        world.add_constraint(constraint);
        const auto residual = [&]
        {
            return measure_constraint_graph(world, world.constraints()).velocity_residual;
        };
        auto& equation = constraint->rows.front();
        RIGIDBODIES_EXPECT_NEAR(residual(), 3.0, 0.0, "unbounded equality exposes its full speed mismatch");
        equation.lower_impulse = 0.0;
        equation.target_velocity = -3.0;
        RIGIDBODIES_EXPECT_NEAR(residual(), 0.0, 0.0, "a satisfied lower-bound complementarity condition has no solver residual");
        equation.target_velocity = 3.0;
        RIGIDBODIES_EXPECT_NEAR(residual(), 3.0, 0.0, "a lower-bound row that must push retains its unmet positive residual");
        equation.lower_impulse = -std::numeric_limits<Real>::infinity();
        equation.upper_impulse = 0.0;
        RIGIDBODIES_EXPECT_NEAR(residual(), 0.0, 0.0, "a satisfied upper-bound complementarity condition has no solver residual");
        equation.target_velocity = -3.0;
        RIGIDBODIES_EXPECT_NEAR(residual(), 3.0, 0.0, "an upper-bound row that must pull retains its unmet negative residual");
        equation.lower_impulse = equation.upper_impulse;
        RIGIDBODIES_EXPECT_NEAR(residual(), 0.0, 0.0, "a fixed impulse has no available correction direction");
    }

    RIGIDBODIES_TEST("stable constraint and body keys remove registration and slot history from graph replay")
    {
        auto first = mechanism(), second = mechanism();
        const auto discarded = body(second, {}, "discarded");
        second.destroy_body(discarded);
        const auto a = chain(first), b = chain(second, true);
        for (int step = 0; step < 25; ++step)
        {
            first.step(0.01);
            second.step(0.01);
        }
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            RIGIDBODIES_EXPECT(first.find_body(a[i])->position_m() == second.find_body(b[i])->position_m(), "canonical graph placement is bit-identical");
            RIGIDBODIES_EXPECT(first.find_body(a[i])->linear_velocity_m_s() == second.find_body(b[i])->linear_velocity_m_s(), "canonical graph velocity is bit-identical");
        }
        RIGIDBODIES_EXPECT(second.constraint_key(second.constraint_by_key("joint/1")) == "joint/1", "key lookup round-trips despite reverse registration");
    }

    RIGIDBODIES_TEST("redundant rows and a closed rigid loop remain finite while preserving momentum")
    {
        auto world = mechanism();
        const auto a = body(world, { 0.0, 0.0 }, "a");
        const auto b = body(world, { 1.0, 0.0 }, "b");
        const auto c = body(world, { 0.5, std::sqrt(0.75) }, "c");
        rod(world, a, b, "ab");
        rod(world, b, c, "bc");
        rod(world, c, a, "ca");
        rod(world, a, b, "redundant_ab");
        world.find_body(a)->set_linear_velocity({ 3.0, 0.0 });
        world.step(0.001);
        RIGIDBODIES_EXPECT(world.motion_limit_events().empty(), "redundant closed loops need no safety clipping");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_linear_momentum_kg_m_s.x, 3.0, 1.0e-12, "internal graph impulses conserve linear momentum");
        RIGIDBODIES_EXPECT(world.statistics().constraint_velocity_residual < 1.0e-6, "loop and duplicate row velocity constraints converge");
    }

    RIGIDBODIES_TEST("a common fixed anchor does not merge independent dynamic islands")
    {
        auto world = mechanism();
        const auto anchor = body(world, {}, "anchor", BodyType::static_body);
        const auto a = body(world, { 1.0, 0.0 }, "a");
        const auto b = body(world, { -1.0, 0.0 }, "b");
        rod(world, anchor, a, "a");
        rod(world, anchor, b, "b");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.statistics().constraint_island_count == 2, "fixed support is a boundary for each independent mechanism");
    }

    RIGIDBODIES_TEST("duplicate registration and keys reject without changing the graph")
    {
        auto world = mechanism();
        const auto a = body(world, {}, "a"), b = body(world, { 1.0, 0.0 }, "b");
        const auto joint = rod(world, a, b, "rod");
        bool duplicate = false, key = false;
        try
        {
            world.add_constraint(joint);
        }
        catch (const std::invalid_argument&)
        {
            duplicate = true;
        }
        try
        {
            world.add_constraint(joint->clone(), "rod");
        }
        catch (const std::invalid_argument&)
        {
            key = true;
        }
        RIGIDBODIES_EXPECT(duplicate && key && world.constraints().size() == 1, "row caches cannot alias through duplicate registrations");
        world.destroy_body(a);
        RIGIDBODIES_EXPECT(world.constraints().empty() && !world.constraint_by_key("rod"), "deleting an endpoint removes both connection and key");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.statistics().constraint_row_count == 0, "removed connection has no dangling graph rows");
    }

    RIGIDBODIES_TEST("graph warmstart caches and key metadata restore independently for exact replay")
    {
        auto world = mechanism();
        const auto ids = chain(world);
        world.step(0.01);
        const auto original = world.constraint_by_key("joint/1");
        const auto checkpoint = world.snapshot();
        for (int i = 0; i < 10; ++i)
            world.step(0.01);
        const auto position = world.find_body(ids.back())->position_m();
        const auto velocity = world.find_body(ids.back())->linear_velocity_m_s();
        const auto residual = world.statistics().constraint_velocity_residual;
        world.restore(checkpoint);
        RIGIDBODIES_EXPECT(world.constraint_by_key("joint/1") != original, "restored row cache has independent ownership");
        for (int i = 0; i < 10; ++i)
            world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(ids.back())->position_m() == position && world.find_body(ids.back())->linear_velocity_m_s() == velocity && world.statistics().constraint_velocity_residual == residual, "graph replay includes warmstart and exact numerical state");
    }

    RIGIDBODIES_TEST("load carrying joint islands sleep and a motor edit wakes their entire connected group")
    {
        auto world = mechanism(true, true);
        auto settings = world.settings();
        settings.gravity_m_s2 = { 0.0, -10.0 };
        world.set_settings(settings);
        const auto anchor = body(world, {}, "anchor", BodyType::static_body);
        const auto a = body(world, { 0.0, -1.0 }, "a");
        const auto b = body(world, { 0.0, -2.0 }, "b");
        RevoluteJointDefinition pivot;
        pivot.first = anchor;
        pivot.second = a;
        pivot.local_anchor_second_m = { 0.0, 1.0 };
        const auto motor = std::make_shared<JointConstraint>(pivot);
        world.add_constraint(motor, "pivot");
        rod(world, a, b, "rod");
        for (int i = 0; i < 30; ++i)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(a)->is_awake() && !world.find_body(b)->is_awake(), "static joint support balances gravity and permits group sleep");
        pivot.motor_enabled = true;
        pivot.motor_speed_rad_s = 1.0;
        pivot.maximum_motor_torque_n_m = 10.0;
        motor->set_definition(pivot);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(a)->is_awake() && world.find_body(b)->is_awake(), "motor revision wakes neighboring sleeping constraints before solve");
        RIGIDBODIES_EXPECT(std::abs(world.find_body(a)->angular_velocity_rad_s()) > 0.0, "enabled motor drives the awake mechanism");
    }

    RIGIDBODIES_TEST("breaking creates one event and later substeps apply no broken row impulses")
    {
        auto world = mechanism();
        const auto a = body(world, {}, "a", BodyType::static_body), b = body(world, { 1.0, 0.0 }, "b");
        DistanceJointDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.break_force_n = 5.0;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, "breakable");
        world.find_body(b)->set_linear_velocity({ 2.0, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(joint->is_broken() && world.constraint_break_events().size() == 1, "threshold crossing marks the joint and records one event");
        RIGIDBODIES_EXPECT(world.constraint_break_events()[0].force_n > 5.0 && world.statistics().broken_constraint_count == 1, "break event retains the measured overload");
        world.find_body(b)->set_linear_velocity({ 2.0, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->linear_velocity_m_s().x, 2.0, 0.0, "broken joints do not solve or warmstart later impulses");
        RIGIDBODIES_EXPECT(world.constraint_break_events().empty() && world.statistics().constraint_row_count == 0, "event stream reports transitions once");
    }

    RIGIDBODIES_TEST("rewiring a sleeping joint wakes its former supported endpoint before it falls")
    {
        auto world = mechanism(true, true);
        auto settings = world.settings();
        settings.gravity_m_s2 = { 0.0, -10.0 };
        world.set_settings(settings);
        const auto anchor = body(world, {}, "anchor", BodyType::static_body);
        const auto old_body = body(world, { 0.0, -1.0 }, "old");
        const auto replacement = body(world, { 0.0, -1.0 }, "replacement", BodyType::static_body);
        const auto joint = std::dynamic_pointer_cast<JointConstraint>(rod(world, anchor, old_body, "joint"));
        for (int i = 0; i < 30; ++i)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(old_body)->is_awake(), "joint carries a sleeping load before rewiring");
        auto changed = std::get<DistanceJointDefinition>(joint->definition());
        changed.second = replacement;
        joint->set_definition(changed);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(old_body)->is_awake() && world.find_body(old_body)->linear_velocity_m_s().y < 0.0, "former endpoint wakes and loses its support in the same substep");
        RIGIDBODIES_EXPECT(world.statistics().constraint_row_count == 0, "new fixed-fixed connection has no active dynamic graph rows");
    }

    RIGIDBODIES_TEST("invalid edited endpoints cannot reuse stale prepared rows or dereference destroyed bodies")
    {
        auto world = mechanism();
        const auto a = body(world, {}, "a", BodyType::static_body), b = body(world, { 1.0, 0.0 }, "b");
        const auto joint = std::dynamic_pointer_cast<JointConstraint>(rod(world, a, b, "rod"));
        world.step(0.01);
        auto changed = std::get<DistanceJointDefinition>(joint->definition());
        changed.second = { std::numeric_limits<std::uint32_t>::max(), 1 };
        joint->set_definition(changed);
        world.find_body(b)->set_linear_velocity({ 1.0, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->linear_velocity_m_s().x, 1.0, 0.0, "stale endpoint skips prepared rows rather than constraining the old body");
        RIGIDBODIES_EXPECT(world.statistics().active_constraint_count == 0 && world.statistics().constraint_row_count == 0, "invalid edited connection is inactive");
    }

    RIGIDBODIES_TEST("connected collision exclusions apply to contact detection and the postintegration CCD guard")
    {
        auto world = mechanism();
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        auto settings = world.settings();
        settings.solver.velocity_iterations = 0;
        settings.solver.warm_starting = false;
        world.set_settings(settings);
        const auto a = body(world, {}, "a", BodyType::static_body), b = body(world, { -1.0, 0.0 }, "b");
        const auto joint = rod(world, a, b, "rod");
        world.find_body(b)->set_linear_velocity({ 20.0, 0.0 });
        world.step(0.1);
        RIGIDBODIES_EXPECT(world.broad_phase_pairs().empty() && world.manifolds().empty(), "excluded connected colliders never enter narrow phase");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->position_m().x, 1.0, 1.0e-12, "excluded connected bodies cross without hidden CCD clipping");
        joint->set_enabled(false);
        world.find_body(b)->set_position({ -1.0, 0.0 });
        world.find_body(b)->set_linear_velocity({ 20.0, 0.0 });
        world.step(0.1);
        RIGIDBODIES_EXPECT(!world.broad_phase_pairs().empty() && world.statistics().ccd_clamped_body_count > 0, "disabling the connection restores collision filtering and CCD consistently");
    }

    RIGIDBODIES_TEST("bounded motor rows coupled to an active limit stay finite and respect impulse caps")
    {
        auto world = mechanism();
        const auto a = body(world, {}, "a", BodyType::static_body), b = body(world, {}, "b");
        RevoluteJointDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.motor_enabled = true;
        definition.motor_speed_rad_s = 10.0;
        definition.maximum_motor_torque_n_m = 1.0;
        definition.limits_enabled = true;
        definition.lower_angle_rad = 0.0;
        definition.upper_angle_rad = 0.0;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, "motor");
        for (int i = 0; i < 10; ++i)
            world.step(0.01);
        RIGIDBODIES_EXPECT(std::abs(world.find_body(b)->orientation_rad()) < 1.0e-6, "dependent motor and limit rows enforce the locked angle");
        RIGIDBODIES_EXPECT(std::abs(joint->report(world).motor_torque_n_m) <= 1.0 + 1.0e-9, "motor impulse remains within the substep cap");
        RIGIDBODIES_EXPECT(world.motion_limit_events().empty(), "redundant bounded row solve remains finite");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
