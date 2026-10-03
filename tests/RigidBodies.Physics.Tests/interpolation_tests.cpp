#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <limits>

namespace
{

    using namespace rigidbodies::physics;
    using namespace rigidbodies::math;

    BodyDefinition moving_disc()
    {
        BodyDefinition definition;
        definition.position_m = { 2.0, 3.0 };
        definition.linear_velocity_m_s = { 4.0, -2.0 };
        Collider collider;
        collider.shape = make_circle(0.25);
        definition.colliders.push_back(collider);
        return definition;
    }

    WorldSettings zero_gravity()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        return settings;
    }

    void expect_same_pose(const Transform2& actual, const Transform2& expected)
    {
        RIGIDBODIES_EXPECT_NEAR(actual.translation.x, expected.translation.x, 1.0e-12, "body-frame x matches");
        RIGIDBODIES_EXPECT_NEAR(actual.translation.y, expected.translation.y, 1.0e-12, "body-frame y matches");
        RIGIDBODIES_EXPECT_NEAR(actual.rotation.sine, expected.rotation.sine, 1.0e-12, "rotation sine matches");
        RIGIDBODIES_EXPECT_NEAR(actual.rotation.cosine, expected.rotation.cosine, 1.0e-12, "rotation cosine matches");
    }

    RIGIDBODIES_TEST("new bodies render at their initial pose at every interpolation fraction")
    {
        auto definition = moving_disc();
        definition.orientation_rad = 1.2;
        const RigidBody body { definition };
        expect_same_pose(body.previous_transform(), body.transform());
        for (const Real alpha : { 0.0, 0.3, 1.0 })
        {
            expect_same_pose(body.interpolated_transform(alpha), body.transform());
        }
    }

    RIGIDBODIES_TEST("world steps preserve the preceding pose and interpolation never changes physics")
    {
        World world { zero_gravity() };
        const auto id = world.create_body(moving_disc());
        world.step(0.5);
        const auto* body = world.find_body(id);

        RIGIDBODIES_EXPECT_NEAR(body->previous_transform().translation.x, 2.0, 1.0e-12, "step starts at creation pose");
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.25).translation.x, 2.5, 1.0e-12, "rendered x lies between steps");
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.25).translation.y, 2.75, 1.0e-12, "rendered y lies between steps");
        RIGIDBODIES_EXPECT_NEAR(body->position_m().x, 4.0, 1.0e-12, "physics retains its final x");
        RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 4.0, 1.0e-12, "rendering does not alter velocity");

        world.step(0.5);
        RIGIDBODIES_EXPECT_NEAR(body->previous_transform().translation.x, 4.0, 1.0e-12, "the next step refreshes history");
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.5).translation.x, 5.0, 1.0e-12, "only the most recent step is interpolated");
        world.step(0.0);
        RIGIDBODIES_EXPECT_NEAR(body->previous_transform().translation.x, 4.0, 1.0e-12, "rejected steps preserve history");
    }

    RIGIDBODIES_TEST("interpolation clamps fractions and handles a non-finite fraction")
    {
        RigidBody body { moving_disc() };
        body.set_simulated_pose({ 12.0, 5.0 }, 1.0);
        expect_same_pose(body.interpolated_transform(-0.5), body.previous_transform());
        expect_same_pose(body.interpolated_transform(1.5), body.transform());
        expect_same_pose(body.interpolated_transform(std::numeric_limits<Real>::quiet_NaN()), body.transform());
        expect_same_pose(body.interpolated_transform(std::numeric_limits<Real>::infinity()), body.transform());
    }

    RIGIDBODIES_TEST("rotation crosses the wrapped angle boundary in its simulated direction")
    {
        World world { zero_gravity() };
        auto definition = moving_disc();
        definition.orientation_rad = 0.9 * pi;
        definition.angular_velocity_rad_s = 0.2 * pi;
        const auto id = world.create_body(definition);
        world.step(1.0);

        const auto half = world.find_body(id)->interpolated_transform(0.5);
        RIGIDBODIES_EXPECT_NEAR(half.rotation.cosine, -1.0, 1.0e-12, "midpoint passes through pi, not zero");
        RIGIDBODIES_EXPECT_NEAR(half.rotation.sine, 0.0, 1.0e-12, "crossing the angle branch is smooth");
    }

    RIGIDBODIES_TEST("interpolation retains multiple complete turns and negative spin")
    {
        for (const auto direction : { -1.0, 1.0 })
        {
            World world { zero_gravity() };
            auto definition = moving_disc();
            definition.angular_velocity_rad_s = direction * 5.0 * pi;
            const auto id = world.create_body(definition);
            world.step(1.0);

            const auto half = world.find_body(id)->interpolated_transform(0.5);
            RIGIDBODIES_EXPECT_NEAR(half.rotation.cosine, 0.0, 1.0e-12, "halfway follows the unwrapped angular displacement");
            RIGIDBODIES_EXPECT_NEAR(half.rotation.sine, direction, 1.0e-12, "multiple-turn interpolation retains spin direction");
        }
    }

    RIGIDBODIES_TEST("an offset mass centre follows a straight path while its body frame rotates")
    {
        World world { zero_gravity() };
        auto definition = moving_disc();
        definition.colliders.front().local_transform.translation = { 1.5, 0.5 };
        definition.angular_velocity_rad_s = pi;
        const auto id = world.create_body(definition);
        const auto* body = world.find_body(id);
        const auto initial_center = body->world_center_of_mass_m();
        world.step(1.0);

        const auto half = body->interpolated_transform(0.5);
        const auto half_center = transform_point(half, body->mass_properties().center_of_mass_m);
        RIGIDBODIES_EXPECT_NEAR(half_center.x, initial_center.x + 2.0, 1.0e-12, "centre moves halfway along the physical x path");
        RIGIDBODIES_EXPECT_NEAR(half_center.y, initial_center.y - 1.0, 1.0e-12, "centre moves halfway along the physical y path");
        RIGIDBODIES_EXPECT_NEAR(half.rotation.sine, 1.0, 1.0e-12, "body rotates halfway around its centre");
        expect_same_pose(body->interpolated_transform(0.0), body->previous_transform());
        expect_same_pose(body->interpolated_transform(1.0), body->transform());
    }

    RIGIDBODIES_TEST("teleport setters synchronize both pose endpoints immediately")
    {
        RigidBody body { moving_disc() };
        body.set_simulated_pose({ 12.0, 5.0 }, 2.0);
        body.set_position({ -4.0, 8.0 });
        expect_same_pose(body.interpolated_transform(0.0), body.transform());
        expect_same_pose(body.interpolated_transform(0.5), body.transform());

        body.set_simulated_pose({ 12.0, 5.0 }, 2.0);
        body.set_orientation(-7.0);
        expect_same_pose(body.interpolated_transform(0.0), body.transform());
        expect_same_pose(body.interpolated_transform(0.5), body.transform());

        body.set_simulated_pose({ 12.0, 5.0 }, 2.0);
        body.set_transform(Transform2::from_angle({ 1.0, -3.0 }, 0.6));
        expect_same_pose(body.interpolated_transform(0.0), body.transform());
        expect_same_pose(body.interpolated_transform(0.5), body.transform());
    }

    RIGIDBODIES_TEST("several integration substeps retain one visual starting pose")
    {
        auto definition = moving_disc();
        definition.angular_velocity_rad_s = 4.0 * pi;
        RigidBody body { definition };
        const auto before = body.transform();
        body.capture_previous_transform();
        SemiImplicitEulerIntegrator integrator;
        for (int index = 0; index < 4; ++index)
        {
            integrator.integrate_position(body, 0.25);
        }

        expect_same_pose(body.previous_transform(), before);
        RIGIDBODIES_EXPECT_NEAR(body.interpolated_transform(0.25).translation.x, 3.0, 1.0e-12, "translation spans all substeps");
        RIGIDBODIES_EXPECT_NEAR(body.interpolated_transform(0.25).rotation.cosine, -1.0, 1.0e-12, "rotation spans all substeps");
    }

    RIGIDBODIES_TEST("render placement can be used consistently for bounds and body picking")
    {
        RigidBody body { moving_disc() };
        body.set_simulated_pose({ 10.0, 3.0 }, 0.0);
        const auto placement = body.interpolated_transform(0.25);
        const auto bounds = body.compute_bounds(placement);

        RIGIDBODIES_EXPECT_NEAR(bounds.minimum.x, 3.75, 1.0e-12, "bounds follow the rendered disc");
        RIGIDBODIES_EXPECT_NEAR(bounds.maximum.x, 4.25, 1.0e-12, "bounds keep the original radius");
        RIGIDBODIES_EXPECT(body.contains_world_point({ 4.0, 3.0 }, placement), "the visible disc is pickable");
        RIGIDBODIES_EXPECT(!body.contains_world_point({ 4.0, 3.0 }), "render picking does not move the simulated collider");
        RIGIDBODIES_EXPECT(body.contains_world_point({ 10.0, 3.0 }), "physics picking still uses the current pose");
    }

    RIGIDBODIES_TEST("static bodies stay fixed and kinematic bodies interpolate their prescribed motion")
    {
        World world { zero_gravity() };
        auto definition = moving_disc();
        definition.type = BodyType::static_body;
        const auto static_id = world.create_body(definition);
        definition.type = BodyType::kinematic_body;
        const auto kinematic_id = world.create_body(definition);
        world.step(0.5);

        const auto* fixed = world.find_body(static_id);
        expect_same_pose(fixed->interpolated_transform(0.5), fixed->transform());
        RIGIDBODIES_EXPECT_NEAR(world.find_body(kinematic_id)->interpolated_transform(0.5).translation.x, 3.0, 1.0e-12, "kinematic motion is interpolated too");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
