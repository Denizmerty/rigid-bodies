#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    World make_world(math::Vec2 gravity = {})
    {
        WorldSettings settings;
        settings.gravity_m_s2 = gravity;
        settings.sleep.enabled = false;
        World world { settings };
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        return world;
    }

    BodyId add_body(World& world, math::Vec2 position, BodyType type = BodyType::dynamic_body, Real mass = 1.0)
    {
        BodyDefinition definition;
        definition.type = type;
        definition.position_m = position;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(mass);
        return id;
    }

    template <typename Definition>
    std::shared_ptr<JointConstraint> connect(World& world, const Definition& definition)
    {
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint);
        return joint;
    }

    void advance(World& world, int count, Real dt = 1.0 / 240.0)
    {
        for (int step = 0; step < count; ++step)
            world.step(dt);
        RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "joint experiment stays inside numerical bounds");
    }

    Real angular_momentum(const World& world)
    {
        Real result = 0.0;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                result += math::cross(body.world_center_of_mass_m(), body.linear_momentum_kg_m_s()) + body.angular_momentum_about_center_kg_m2_s();
            });
        return result;
    }

    template <typename Action>
    void rejects(Action action)
    {
        bool rejected = false;
        try
        {
            action();
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "invalid joint input is explicitly rejected");
    }

    RIGIDBODIES_TEST("a vertical distance joint carries its body's weight with a measured reaction")
    {
        auto world = make_world({ 0.0, -9.0 });
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.0, -1.0 });
        const auto joint = connect(world, definition);
        advance(world, 100);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).distance_m, 1.0, 1.0e-12, "distance remains exactly supported");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).reaction_force_n.y, 9.0, 1.0e-10, "impulse divided by step duration reports weight support");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).reaction_torque_n_m, 0.0, 1.0e-12, "a centered rod transfers no couple");
    }

    RIGIDBODIES_TEST("a pendulum preserves rod length while its free angle responds to gravity")
    {
        auto world = make_world({ 0.0, -9.0 });
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.6, -0.8 });
        const auto joint = connect(world, definition);
        advance(world, 240);
        RIGIDBODIES_EXPECT(joint->report(world).position_error_m < 0.0004, "rod drift is corrected without restricting swing");
        RIGIDBODIES_EXPECT(world.find_body(definition.second)->position_m().x < -0.4, "pendulum swings to the other side");
    }

    RIGIDBODIES_TEST("distance impulses conserve momentum across a large mass ratio")
    {
        auto world = make_world();
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::dynamic_body, 0.01);
        definition.second = add_body(world, { 1.0, 0.0 }, BodyType::dynamic_body, 100.0);
        world.find_body(definition.first)->set_linear_velocity({ 4.0, 0.0 });
        world.find_body(definition.second)->set_linear_velocity({ -1.0, 0.0 });
        connect(world, definition);
        world.step(0.01);
        const auto expected = (0.04 - 100.0) / 100.01;
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.first)->linear_velocity_m_s().x, expected, 1.0e-8, "small body follows the common axial velocity within graph regularization tolerance");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->linear_velocity_m_s().x, expected, 1.0e-12, "large body receives its equal opposite impulse");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_linear_momentum_kg_m_s.x, -99.96, 1.0e-10, "pair momentum is conserved");
    }

    RIGIDBODIES_TEST("off-center hinge impulses preserve angular momentum and allow relative rotation")
    {
        auto world = make_world();
        RevoluteJointDefinition definition;
        definition.first = add_body(world, { -0.5, 0.0 });
        definition.second = add_body(world, { 0.5, 0.0 });
        definition.local_anchor_first_m = { 0.5, 0.0 };
        definition.local_anchor_second_m = { -0.5, 0.0 };
        world.find_body(definition.first)->set_linear_velocity({ 0.0, 1.0 });
        world.find_body(definition.second)->set_linear_velocity({ 0.0, -1.0 });
        const auto joint = connect(world, definition);
        const auto before = angular_momentum(world);
        joint->prepare(world, 0.01);
        for (int pass = 0; pass < 20; ++pass)
            joint->solve_velocity(world, 0.01);
        joint->finalize_velocity(world, 0.01);
        RIGIDBODIES_EXPECT_NEAR(angular_momentum(world), before, 1.0e-12, "anchor moments account for orbital and spin momentum");
        const auto first_velocity = world.find_body(definition.first)->velocity_at_world_point({});
        const auto second_velocity = world.find_body(definition.second)->velocity_at_world_point({});
        RIGIDBODIES_EXPECT_NEAR(math::distance(first_velocity, second_velocity), 0.0, 1.0e-12, "hinge anchor velocities coincide");
        RIGIDBODIES_EXPECT(std::abs(world.find_body(definition.first)->angular_velocity_rad_s()) > 0.01, "the free rotational degree of freedom remains available");
    }

    RIGIDBODIES_TEST("a prismatic guide follows its rotated first-body axis and locks relative angle")
    {
        auto world = make_world();
        PrismaticJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.0, 0.3 });
        definition.local_axis_first = { 7.0, 0.0 };
        world.find_body(definition.first)->set_orientation(math::pi / 2.0);
        world.find_body(definition.second)->set_orientation(math::pi / 2.0);
        world.find_body(definition.second)->set_linear_velocity({ 4.0, 2.0 });
        world.find_body(definition.second)->set_angular_velocity(3.0);
        const auto joint = connect(world, definition);
        advance(world, 24);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->position_m().x, 0.0, 1.0e-12, "guide rejects motion perpendicular to its rotated axis");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).translation_m, 0.5, 1.0e-12, "motion along the guide remains free");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).angle_rad, 0.0, 1.0e-12, "slider relative orientation is fixed");
    }

    RIGIDBODIES_TEST("a weld maintains both coincident anchors and an unwrapped reference angle")
    {
        auto world = make_world({ 0.0, -9.0 });
        WeldJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 1.0, 0.0 });
        world.find_body(definition.first)->set_orientation(6.0 * math::pi);
        world.find_body(definition.second)->set_orientation(8.0 * math::pi + 0.7);
        definition.reference_angle_rad = 2.0 * math::pi + 0.7;
        definition.local_anchor_first_m = { 1.0, 0.0 };
        const auto joint = connect(world, definition);
        advance(world, 120);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).distance_m, 0.0, 1.0e-10, "weld anchor placement remains fixed");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).angle_rad, 0.0, 1.0e-10, "full revolutions in the reference are retained");
    }

    RIGIDBODIES_TEST("motor forces and torques respect their per-step physical caps")
    {
        auto world = make_world();
        RevoluteJointDefinition rotation;
        rotation.first = add_body(world, {}, BodyType::static_body);
        rotation.second = add_body(world, {});
        rotation.motor_enabled = true;
        rotation.motor_speed_rad_s = 5.0;
        rotation.maximum_motor_torque_n_m = 0.05;
        const auto hinge = connect(world, rotation);
        PrismaticJointDefinition translation;
        translation.first = add_body(world, { 2.0, 0.0 }, BodyType::static_body);
        translation.second = add_body(world, { 2.0, 0.0 });
        translation.motor_enabled = true;
        translation.motor_speed_m_s = 2.0;
        translation.maximum_motor_force_n = 1.0;
        const auto slider = connect(world, translation);
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(hinge->report(world).motor_torque_n_m, 0.05, 1.0e-12, "motor angular impulse is capped at torque times dt");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(rotation.second)->angular_velocity_rad_s(), 0.1, 1.0e-12, "torque cap produces the expected angular acceleration");
        RIGIDBODIES_EXPECT_NEAR(slider->report(world).motor_force_n, 1.0, 1.0e-12, "motor linear impulse is capped at force times dt");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(translation.second)->linear_velocity_m_s().x, 0.01, 1.0e-12, "force cap produces the expected acceleration");
    }

    RIGIDBODIES_TEST("revolute motors stop at both unwrapped angular limits")
    {
        auto world = make_world();
        RevoluteJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        definition.reference_angle_rad = 4.0 * math::pi;
        world.find_body(definition.second)->set_orientation(definition.reference_angle_rad);
        definition.motor_enabled = definition.limits_enabled = true;
        definition.motor_speed_rad_s = 2.0;
        definition.maximum_motor_torque_n_m = 0.5;
        definition.lower_angle_rad = -0.4;
        definition.upper_angle_rad = 0.3;
        const auto joint = connect(world, definition);
        advance(world, 120);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).angle_rad, 0.3, 0.0002, "positive motor reaches but does not cross upper limit");
        definition.motor_speed_rad_s = -2.0;
        joint->set_definition(definition);
        advance(world, 120);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).angle_rad, -0.4, 0.0002, "negative motor reaches lower limit after reversal");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->angular_velocity_rad_s(), 0.0, 1.0e-8, "active limit arrests motor-driven spin");
    }

    RIGIDBODIES_TEST("prismatic motor obeys both stops and permits travel away from a stop")
    {
        auto world = make_world();
        PrismaticJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        definition.motor_enabled = definition.limits_enabled = true;
        definition.motor_speed_m_s = 3.0;
        definition.maximum_motor_force_n = 20.0;
        definition.lower_translation_m = -0.4;
        definition.upper_translation_m = 0.3;
        const auto joint = connect(world, definition);
        advance(world, 120);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).translation_m, 0.3, 0.0002, "slider reaches the upper stop");
        definition.motor_speed_m_s = -3.0;
        joint->set_definition(definition);
        advance(world, 120);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).translation_m, -0.4, 0.0002, "slider leaves the upper stop and reaches the lower stop");
    }

    RIGIDBODIES_TEST("equal joint limits act as locked coordinates even against a motor")
    {
        auto world = make_world();
        RevoluteJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        definition.limits_enabled = definition.motor_enabled = true;
        definition.motor_speed_rad_s = 2.0;
        definition.maximum_motor_torque_n_m = 0.5;
        definition.lower_angle_rad = definition.upper_angle_rad = 0.0;
        const auto joint = connect(world, definition);
        advance(world, 60);
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).angle_rad, 0.0, 1.0e-10, "equal bounds eliminate the angular degree of freedom");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->angular_velocity_rad_s(), 0.0, 1.0e-10, "motor cannot push through equal limits");
    }

    RIGIDBODIES_TEST("zero distance and collapsed rods remain finite and recover their geometry")
    {
        for (const auto target : { 0.0, 1.0 })
        {
            auto world = make_world();
            DistanceJointDefinition definition;
            definition.first = add_body(world, {}, BodyType::static_body);
            definition.second = add_body(world, {});
            definition.length_m = target;
            const auto joint = connect(world, definition);
            advance(world, 120);
            RIGIDBODIES_EXPECT_NEAR(joint->report(world).distance_m, target, 0.0001, "degenerate initial anchor placement has a defined finite correction");
            RIGIDBODIES_EXPECT(math::is_finite(world.find_body(definition.second)->position_m()), "no zero-length normalization leaks NaN");
        }
    }

    RIGIDBODIES_TEST("force threshold breaks a loaded rod and stops subsequent support")
    {
        auto world = make_world({ 0.0, -9.0 });
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.0, -1.0 });
        definition.break_force_n = 8.0;
        const auto joint = connect(world, definition);
        world.step(0.01);
        RIGIDBODIES_EXPECT(joint->is_broken(), "support load above threshold records a persistent broken state");
        RIGIDBODIES_EXPECT_NEAR(joint->broken_force_n(), 9.0, 1.0e-8, "failure retains the load that caused it within graph regularization tolerance");
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->linear_velocity_m_s().y, -0.09, 1.0e-10, "broken joint does not apply a cached support impulse");
        RIGIDBODIES_EXPECT(!joint->report(world).enabled && joint->report(world).broken, "inspection reports broken separately from enabled property");
    }

    RIGIDBODIES_TEST("torque thresholds observe constraint couples rather than anchor force moments")
    {
        auto world = make_world();
        WeldJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        definition.break_torque_n_m = 0.4;
        const auto joint = connect(world, definition);
        world.find_body(definition.second)->apply_torque(0.5);
        world.step(0.01);
        RIGIDBODIES_EXPECT(joint->is_broken(), "a pure reaction couple can break a weld");
        RIGIDBODIES_EXPECT_NEAR(joint->broken_torque_n_m(), 0.5, 1.0e-10, "failure torque measures torque impulse over dt");
    }

    RIGIDBODIES_TEST("threshold equality holds and definitions explicitly repair a broken joint")
    {
        auto world = make_world({ 0.0, -8.0 });
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.0, -1.0 });
        definition.break_force_n = 8.0;
        const auto joint = connect(world, definition);
        world.step(0.125);
        RIGIDBODIES_EXPECT(!joint->is_broken(), "only loads strictly above the threshold break a joint");
        definition.break_force_n = 0.0;
        joint->set_definition(definition);
        world.step(0.125);
        RIGIDBODIES_EXPECT(joint->is_broken(), "zero threshold breaks under a nonzero load");
        definition.break_force_n = std::numeric_limits<Real>::infinity();
        joint->set_definition(definition);
        RIGIDBODIES_EXPECT(!joint->is_broken(), "replacement definition is an explicit repair and cache reset");
        world.step(0.125);
        RIGIDBODIES_EXPECT(!joint->is_broken(), "infinite threshold disables breaking");
    }

    RIGIDBODIES_TEST("a translated prismatic guide can break from bending at its first anchor")
    {
        auto world = make_world();
        PrismaticJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 2.0, 0.0 });
        definition.break_torque_n_m = 5.0;
        const auto joint = connect(world, definition);
        world.find_body(definition.second)->apply_force_at_center({ 0.0, 3.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(joint->is_broken(), "transverse guide force produces a first-anchor bending failure");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).reaction_torque_n_m, 0.0, 1.0e-10, "the slider end itself transmits no pure couple in this geometry");
        RIGIDBODIES_EXPECT_NEAR(joint->report(world).reaction_torque_first_n_m, 6.0, 1.0e-6, "support couple equals transverse force times extension");
        RIGIDBODIES_EXPECT_NEAR(joint->broken_torque_n_m(), 6.0, 1.0e-6, "break criterion considers both anchor couples");
    }

    RIGIDBODIES_TEST("joint snapshots clone motor impulses and replay independently")
    {
        auto world = make_world();
        RevoluteJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        definition.motor_enabled = true;
        definition.motor_speed_rad_s = 4.0;
        definition.maximum_motor_torque_n_m = 0.02;
        const auto original = connect(world, definition);
        advance(world, 9);
        const auto snapshot = world.snapshot();
        advance(world, 37);
        const auto angle = world.find_body(definition.second)->orientation_rad();
        const auto speed = world.find_body(definition.second)->angular_velocity_rad_s();
        const auto reaction = original->report(world).motor_torque_n_m;
        original->set_enabled(false);
        world.restore(snapshot);
        const auto restored = std::dynamic_pointer_cast<JointConstraint>(world.constraints().front());
        RIGIDBODIES_EXPECT(restored.get() != original.get() && restored->is_enabled(), "snapshot owns an independent joint and flags");
        advance(world, 37);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->orientation_rad(), angle, 0.0, "restored cached joint replays position exactly");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->angular_velocity_rad_s(), speed, 0.0, "restored cached joint replays velocity exactly");
        RIGIDBODIES_EXPECT_NEAR(restored->report(world).motor_torque_n_m, reaction, 0.0, "restored motor reaction is exact");
    }

    RIGIDBODIES_TEST("stale endpoints and two infinite-mass bodies are harmless to direct joint calls")
    {
        auto world = make_world();
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 1.0, 0.0 }, BodyType::kinematic_body);
        const auto joint = connect(world, definition);
        advance(world, 5);
        world.destroy_body(definition.second);
        joint->prepare(world, 0.01);
        joint->solve_velocity(world, 0.01);
        joint->finalize_velocity(world, 0.01);
        RIGIDBODIES_EXPECT(joint->solve_position(world, 0.01), "missing endpoints need no correction");
        RIGIDBODIES_EXPECT(!joint->report(world).enabled, "report does not dereference a stale endpoint");
    }

    RIGIDBODIES_TEST("invalid joint definitions reject before replacing a live motor and its caches")
    {
        auto world = make_world();
        PrismaticJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, {});
        const auto joint = connect(world, definition);
        const auto revision = joint->revision();
        auto invalid = definition;
        invalid.local_axis_first = {};
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        invalid = definition;
        invalid.motor_speed_m_s = std::numeric_limits<Real>::infinity();
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        invalid = definition;
        invalid.lower_translation_m = 1.0;
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        invalid = definition;
        invalid.break_force_n = -1.0;
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        invalid = definition;
        invalid.local_anchor_second_m.x = std::numeric_limits<Real>::quiet_NaN();
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        invalid = definition;
        invalid.second = invalid.first;
        rejects([&]
            {
                joint->set_definition(invalid);
            });
        RIGIDBODIES_EXPECT(joint->revision() == revision, "failed edits do not change revision or definition");
        RIGIDBODIES_EXPECT(std::get<PrismaticJointDefinition>(joint->definition()).local_axis_first == math::Vec2 { 1.0, 0.0 }, "original valid guide remains intact");
    }

    RIGIDBODIES_TEST("changing step size rescales joint warm starts instead of changing support force")
    {
        auto world = make_world({ 0.0, -9.0 });
        DistanceJointDefinition definition;
        definition.first = add_body(world, {}, BodyType::static_body);
        definition.second = add_body(world, { 0.0, -1.0 });
        const auto joint = connect(world, definition);
        for (const auto dt : { 0.01, 0.02, 0.005, 0.03, 0.01 })
        {
            world.step(dt);
            RIGIDBODIES_EXPECT_NEAR(joint->report(world).reaction_force_n.y, 9.0, 1.0e-8, "warm-start impulses remain loads integrated over this step duration");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(definition.second)->linear_velocity_m_s().y, 0.0, 1.0e-9, "support remains stationary across step-size changes within graph regularization tolerance");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
