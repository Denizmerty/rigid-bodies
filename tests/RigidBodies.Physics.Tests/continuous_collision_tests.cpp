#include <rigidbodies/physics/continuous_collision.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <array>
#include <cmath>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Transform2;
    using rigidbodies::math::Vec2;

    ShapeSweep translation(const Shape& shape, Vec2 start, Vec2 displacement)
    {
        ShapeSweep result;
        result.shape = &shape;
        result.linear_displacement_m = displacement;
        result.transform_at_fraction = [start, displacement](Real fraction)
        {
            return Transform2::from_angle(start + displacement * fraction, 0.0);
        };
        return result;
    }

    WorldSettings settings()
    {
        WorldSettings result;
        result.gravity_m_s2 = {};
        result.sleep.enabled = false;
        return result;
    }

    BodyDefinition body(ShapePtr shape, BodyType type, Vec2 position = {}, Vec2 velocity = {})
    {
        BodyDefinition result;
        result.type = type;
        result.position_m = position;
        result.linear_velocity_m_s = velocity;
        Collider collider;
        collider.shape = std::move(shape);
        result.colliders.push_back(std::move(collider));
        return result;
    }

    std::array<IntegratorPtr, 3> integrators()
    {
        return { std::make_shared<SemiImplicitEulerIntegrator>(), std::make_shared<VelocityVerletIntegrator>(), std::make_shared<RungeKutta4Integrator>() };
    }

    void configure_guard_only(World& world, IntegratorPtr integrator)
    {
        world.set_integrator(std::move(integrator));
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_contact_solver(std::make_shared<NullContactSolver>());
    }

    RIGIDBODIES_TEST("conservative advancement finds the analytic circle versus thin wall impact")
    {
        const CircleShape circle { 0.1 };
        const SegmentShape wall { { 0.0, -2.0 }, { 0.0, 2.0 } };
        const auto moving = translation(circle, { -1.0, 0.0 }, { 2.0, 0.0 });
        const auto fixed = translation(wall, {}, {});
        const auto hit = sweep_shapes(moving, fixed);
        RIGIDBODIES_EXPECT(hit.hit && hit.converged, "an endpoint-disjoint sweep finds the intervening wall");
        RIGIDBODIES_EXPECT_NEAR(hit.fraction, 0.45, 1.0e-6, "first impact occurs one radius before the wall");
        const auto reverse = sweep_shapes(fixed, moving);
        RIGIDBODIES_EXPECT_NEAR(reverse.fraction, hit.fraction, 1.0e-9, "sweep ordering preserves the impact time");
        RIGIDBODIES_EXPECT(rigidbodies::math::dot(reverse.distance.normal, hit.distance.normal) < -0.999,
            "reversing the sweep reverses its contact normal");
    }

    RIGIDBODIES_TEST("parallel translation above a thin segment remains a near miss")
    {
        const CircleShape circle { 0.1 };
        const SegmentShape floor { { -2.0, 0.0 }, { 2.0, 0.0 } };
        const auto result = sweep_shapes(translation(circle, { -1.0, 0.2 }, { 2.0, 0.0 }), translation(floor, {}, {}));
        RIGIDBODIES_EXPECT(!result.hit && result.converged, "parallel motion cannot cross the separating support plane");
        RIGIDBODIES_EXPECT(result.iterations <= 2, "parallel near misses have a bounded early exit");
    }

    RIGIDBODIES_TEST("a full turn sweep finds a collision hidden by identical endpoint transforms")
    {
        const auto bar = ConvexPolygonShape::box(2.0, 0.05);
        const CircleShape obstacle { 0.05 };
        ShapeSweep rotating;
        rotating.shape = &bar;
        rotating.nonlinear_travel_m = rigidbodies::math::two_pi * bar.bounding_radius();
        rotating.transform_at_fraction = [](Real fraction)
        {
            return Transform2::from_angle({}, rigidbodies::math::two_pi * fraction);
        };
        const auto result = sweep_shapes(rotating, translation(obstacle, { 0.0, 0.8 }, {}));
        RIGIDBODIES_EXPECT(result.hit && result.converged, "angular travel exposes the collision between matching endpoints");
        RIGIDBODIES_EXPECT(result.fraction > 0.1 && result.fraction < 0.25, "the first impact occurs before a quarter turn");
    }

    RIGIDBODIES_TEST("sweep iteration exhaustion returns a conservative candidate")
    {
        const CircleShape circle { 0.1 };
        const SegmentShape wall { { 0.0, -2.0 }, { 0.0, 2.0 } };
        const auto result = sweep_shapes(translation(circle, { -1.0, 0.0 }, { 2.0, 0.0 }), translation(wall, {}, {}), 1.0e-6, 1);
        RIGIDBODIES_EXPECT(result.hit && !result.converged && result.iterations == 1, "the finite budget is observable");
        RIGIDBODIES_EXPECT(result.fraction <= 0.45, "exhaustion never advances past the unresolved crossing");
    }

    RIGIDBODIES_TEST("all integrators stop a fast circle before an infinitely thin static wall")
    {
        for (auto integrator : integrators())
        {
            World world { settings() };
            configure_guard_only(world, std::move(integrator));
            const auto moving = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
            (void)world.create_body(body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body));
            world.step(0.02);
            const auto& result = *world.find_body(moving);
            RIGIDBODIES_EXPECT(result.position_m().x <= -0.05 + 2.0e-6, "the actual integrated endpoint cannot tunnel through a wall");
            RIGIDBODIES_EXPECT(result.position_m().x > -0.06, "the guard retains safe progress to the impact");
            RIGIDBODIES_EXPECT_NEAR(result.previous_transform().translation.x, -1.0, 1.0e-12, "clamping preserves the visual interpolation start");
            RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 1, "the actual endpoint clamp is reported once");
        }
    }

    RIGIDBODIES_TEST("all integrators preserve ordering during a fast dynamic versus dynamic crossing")
    {
        for (auto integrator : integrators())
        {
            World world { settings() };
            configure_guard_only(world, std::move(integrator));
            const auto left = world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
            const auto right = world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { 1.0, 0.0 }, { -100.0, 0.0 }));
            world.step(0.02);
            const auto separation = world.find_body(right)->position_m().x - world.find_body(left)->position_m().x;
            RIGIDBODIES_EXPECT(separation >= 0.2 - 2.0e-6 && separation < 0.21, "both endpoints stop at their common impact");
            RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 2, "both moving participants are reported");
        }
    }

    RIGIDBODIES_TEST("World retains unwrapped angular travel when guarding a full rotation")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        auto definition = body(make_box(2.0, 0.05), BodyType::dynamic_body);
        definition.angular_velocity_rad_s = rigidbodies::math::two_pi / 0.1;
        const auto rotating = world.create_body(definition);
        (void)world.create_body(body(make_circle(0.05), BodyType::static_body, { 0.0, 0.8 }));
        world.step(0.1);
        const auto angle = world.find_body(rotating)->orientation_rad();
        RIGIDBODIES_EXPECT(angle > 0.5 && angle < rigidbodies::math::half_pi, "a matching endpoint orientation cannot conceal the swept collision");
        RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 1, "rotational clamping is observable");
    }

    RIGIDBODIES_TEST("clamping one trajectory rechecks the dynamic chain behind it")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        const auto rear = world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { -2.0, 0.0 }, { 100.0, 0.0 }));
        const auto front = world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
        (void)world.create_body(body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body));
        world.step(0.03);
        const auto rear_x = world.find_body(rear)->position_m().x;
        const auto front_x = world.find_body(front)->position_m().x;
        RIGIDBODIES_EXPECT(front_x <= -0.1 + 2.0e-6, "the front body stops before the wall");
        RIGIDBODIES_EXPECT(front_x - rear_x >= 0.2 - 2.0e-6, "the following body respects the changed leading trajectory");
        RIGIDBODIES_EXPECT(rear_x > -2.0 && front_x > -1.0, "a resolved chain preserves useful forward progress");
    }

    RIGIDBODIES_TEST("sensors and filtered colliders do not clamp otherwise free motion")
    {
        for (int mode = 0; mode < 2; ++mode)
        {
            World world { settings() };
            configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
            const auto moving = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
            auto wall = body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body);
            if (mode == 0)
                wall.colliders[0].is_sensor = true;
            else
                wall.colliders[0].filter.mask = 0;
            (void)world.create_body(wall);
            world.step(0.02);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->position_m().x, 1.0, 1.0e-10, "nonresponding geometry permits complete travel");
            RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 0, "no blocking clamp is reported");
        }
    }

    RIGIDBODIES_TEST("an exact endpoint landing retains its integrated pose without a tolerance clamp")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        const auto moving = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, { -1.0, 0.0 }, { 9.5, 0.0 }));
        (void)world.create_body(body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body));
        world.step(0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->position_m().x, -0.05, 1.0e-12, "an ordinary endpoint contact keeps the exact integration result");
        RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 0, "a sub-tolerance advancement gap is not a new endpoint clamp");
    }

    RIGIDBODIES_TEST("circle spin does not exhaust a geometrically parallel sweep")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        auto definition = body(make_circle(0.1), BodyType::dynamic_body, { 0.0, 0.101 }, { 10.0, 0.0 });
        definition.angular_velocity_rad_s = 100.0;
        const auto spinning = world.create_body(definition);
        (void)world.create_body(body(make_segment({ -100.0, 0.0 }, { 100.0, 0.0 }), BodyType::static_body));
        world.step(0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(spinning)->position_m().x, 1.0, 1.0e-10, "spinning an unchanged circular boundary cannot create a false swept collision above the floor");
        RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count == 0,
            "pure circle spin contributes no nonlinear geometric travel bound");
    }

    RIGIDBODIES_TEST("offset circles retain their centre-orbit rotational collision bound")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        auto definition = body(make_circle(0.05, { 0.8, 0.0 }), BodyType::dynamic_body);
        auto other_circle = definition.colliders.front();
        other_circle.shape = make_circle(0.05, { -0.8, 0.0 });
        definition.colliders.push_back(other_circle);
        definition.angular_velocity_rad_s = rigidbodies::math::two_pi / 0.1;
        const auto compound = world.create_body(definition);
        (void)world.create_body(body(make_circle(0.05), BodyType::static_body, { 0.0, 0.8 }));
        world.step(0.1);
        const auto angle = world.find_body(compound)->orientation_rad();
        RIGIDBODIES_EXPECT(angle > 0.1 && angle < rigidbodies::math::half_pi,
            "a compound circle's centre orbit still stops at an intervening obstacle");
        RIGIDBODIES_EXPECT(world.statistics().ccd_clamped_body_count == 1, "the compound's rotational clamp is reported");
    }

    RIGIDBODIES_TEST("null narrow phase preserves its explicit free motion behavior")
    {
        World world { settings() };
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        const auto moving = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
        (void)world.create_body(body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body));
        world.step(0.02);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->position_m().x, 1.0, 1.0e-10, "a disabled narrow phase also disables the endpoint guard");
    }

    RIGIDBODIES_TEST("an exhausted World sweep budget is reported and prevents tunneling")
    {
        auto limited = settings();
        limited.collision.maximum_sweep_iterations = 1;
        World world { limited };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        const auto moving = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, { -1.0, 0.0 }, { 100.0, 0.0 }));
        (void)world.create_body(body(make_segment({ 0.0, -2.0 }, { 0.0, 2.0 }), BodyType::static_body));
        world.step(0.02);
        RIGIDBODIES_EXPECT(world.find_body(moving)->position_m().x <= -0.05 + 2.0e-6, "budget exhaustion stops at or before the wall");
        RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count > 0, "budget exhaustion is visible to callers");
    }

    RIGIDBODIES_TEST("endpoint protection leaves attached kinematic trajectories prescribed")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        const auto prescribed = world.create_body(body(make_circle(0.1), BodyType::kinematic_body, { -1.0, 0.0 }));
        LinearMotion path;
        path.origin_m = { -1.0, 0.0 };
        path.velocity_m_s = { 10.0, 0.0 };
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(prescribed, KinematicMotion { path }), "a valid path attaches");
        (void)world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { 0.0, 0.0 }));
        world.step(0.2);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(prescribed)->position_m().x, 1.0, 1.0e-10, "the guard never rewrites a prescribed path endpoint");
        RIGIDBODIES_EXPECT(world.kinematic_motion(prescribed) != nullptr, "the motion remains attached");
    }

    RIGIDBODIES_TEST("a full-turn kinematic arm responds to a stationary dynamic target")
    {
        World world { settings() };
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
        const auto arm = world.create_body(body(make_box(2.0, 0.05), BodyType::kinematic_body));
        LinearMotion path;
        path.angular_velocity_rad_s = rigidbodies::math::two_pi / 0.1;
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(arm, KinematicMotion { path }), "the spinning arm path attaches");
        const Vec2 initial_target { 0.0, 0.8 };
        const auto target = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, initial_target));
        world.step(0.1);
        const auto& moved = *world.find_body(target);
        RIGIDBODIES_EXPECT(rigidbodies::math::distance(moved.position_m(), initial_target) > 0.05,
            "motion to the impact interval pushes the target even when the arm's complete-turn endpoint matches its start");
        RIGIDBODIES_EXPECT(moved.linear_velocity_m_s().x < -1.0, "the counterclockwise arm pushes the target to the left");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(arm)->orientation_rad(), rigidbodies::math::two_pi, 1.0e-10, "the prescribed arm completes its unwrapped full turn");
        RIGIDBODIES_EXPECT(world.kinematic_motion(arm) != nullptr, "collision response preserves the prescribed arm path");
    }

    RIGIDBODIES_TEST("a closed circular kinematic trajectory transfers motion before returning to its start")
    {
        World world { settings() };
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
        const auto orbiting = world.create_body(body(make_circle(0.05), BodyType::kinematic_body, { 0.8, 0.0 }));
        CircularMotion path;
        path.radius_m = 0.8;
        path.angular_speed_rad_s = rigidbodies::math::two_pi / 0.1;
        path.orient_to_path = false;
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(orbiting, KinematicMotion { path }), "the circular path attaches");
        const Vec2 initial_target { 0.0, 0.8 };
        const auto target = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, initial_target));
        world.step(0.1);
        RIGIDBODIES_EXPECT(rigidbodies::math::distance(world.find_body(target)->position_m(), initial_target) > 0.05,
            "the target responds to the intervening collision even when the driver returns to its start");
        RIGIDBODIES_EXPECT(world.find_body(target)->linear_velocity_m_s().x < -1.0, "the orbital impact pushes along its counterclockwise direction");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(orbiting)->position_m().x, 0.8, 1.0e-10, "the circular path completes its prescribed horizontal endpoint");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(orbiting)->position_m().y, 0.0, 1.0e-10, "the circular path completes its prescribed vertical endpoint");
    }

    RIGIDBODIES_TEST("a spinning kinematic arm responds through an initial persistence gap or exact contact")
    {
        for (const auto gap : { 0.0, 0.001 })
        {
            World world { settings() };
            world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
            world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
            const auto arm = world.create_body(body(make_box(2.0, 0.05), BodyType::kinematic_body));
            LinearMotion path;
            path.angular_velocity_rad_s = rigidbodies::math::two_pi / 0.1;
            RIGIDBODIES_EXPECT(world.set_kinematic_motion(arm, KinematicMotion { path }), "the spinning path attaches");
            const Vec2 initial { 0.8, 0.075 + gap };
            const auto target = world.create_body(body(make_circle(0.05), BodyType::dynamic_body, initial));
            world.step(0.1);
            const auto moved_y = world.find_body(target)->position_m().y;
            RIGIDBODIES_EXPECT(moved_y >= initial.y - 1.0e-10, "the responding target does not move into the approaching arm");
            if (moved_y <= initial.y + 0.05)
                RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count > 0,
                    "a difficult full-turn departure may stop conservatively only with an explicit iteration-budget report");
            RIGIDBODIES_EXPECT(world.find_body(target)->linear_velocity_m_s().y > 1.0,
                "an initial persistence manifold cannot cancel a closed path's collision response");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(arm)->orientation_rad(), rigidbodies::math::two_pi, 1.0e-10, "the arm's prescribed endpoint remains exact");
        }
    }

    RIGIDBODIES_TEST("zero amplitude and zero radius paths retain finite collision bounds for extreme finite times")
    {
        for (int mode = 0; mode < 2; ++mode)
        {
            World world { settings() };
            configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
            const auto fixed_path = world.create_body(body(make_circle(0.1), BodyType::kinematic_body));
            if (mode == 0)
            {
                HarmonicMotion path;
                path.frequency_hz = 1.0e100;
                RIGIDBODIES_EXPECT(world.set_kinematic_motion(fixed_path, KinematicMotion { path }), "zero-amplitude harmonic path attaches");
            }
            else
            {
                CircularMotion path;
                path.radius_m = 0.0;
                path.angular_speed_rad_s = 1.0e100;
                path.orient_to_path = false;
                RIGIDBODIES_EXPECT(world.set_kinematic_motion(fixed_path, KinematicMotion { path }), "zero-radius fixed-orientation circular path attaches");
            }
            (void)world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { 0.15, 0.0 }));
            world.step(1.0e300);
            RIGIDBODIES_EXPECT(!world.manifolds().empty(), "a stationary path's bounds remain finite so an existing overlap is still detected");
            RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count == 0, "zero motion does not require an unresolved sweep fallback");
            RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(world.find_body(fixed_path)->position_m()), 0.0, 1.0e-12, "extreme finite parameters do not move a zero-amplitude path");
        }
    }

    RIGIDBODIES_TEST("unrepresentable periodic travel stops conservatively with a visible bounded fallback")
    {
        World world { settings() };
        configure_guard_only(world, std::make_shared<SemiImplicitEulerIntegrator>());
        const auto oscillator = world.create_body(body(make_circle(0.1), BodyType::kinematic_body));
        HarmonicMotion path;
        path.translation_amplitude_m = { 0.1, 0.0 };
        path.frequency_hz = 10.0;
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(oscillator, KinematicMotion { path }), "a bounded-amplitude oscillator attaches");
        const auto target = world.create_body(body(make_circle(0.1), BodyType::dynamic_body, { 2.0, 0.0 }));
        world.step(1.0e307);
        RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count > 0, "a travel bound beyond the finite budget is explicitly reported");
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(world.find_body(target)->position_m()), "the conservative fallback preserves finite dynamic placement");
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(world.find_body(oscillator)->position_m()), "bounded periodic endpoint sampling remains finite");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
