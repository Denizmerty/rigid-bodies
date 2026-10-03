#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    WorldSettings settings()
    {
        WorldSettings result;
        result.gravity_m_s2 = {};
        result.sleep.enabled = false;
        return result;
    }

    BodyDefinition circle(Vec2 position, BodyType type = BodyType::dynamic_body, bool sensor = false)
    {
        BodyDefinition result;
        result.position_m = position;
        result.type = type;
        Collider collider;
        collider.shape = make_circle(0.5);
        collider.is_sensor = sensor;
        collider.material.restitution = 0.0;
        collider.material.static_friction = 0.0;
        collider.material.kinetic_friction = 0.0;
        result.colliders.push_back(collider);
        return result;
    }

    BodyDefinition box(Vec2 position, BodyType type = BodyType::dynamic_body)
    {
        auto result = circle(position, type);
        result.colliders[0].shape = make_box(1.0, 1.0);
        return result;
    }

    bool event_for(const std::vector<CollisionEvent>& events, CollisionEventKind kind, BodyId id)
    {
        return std::any_of(events.begin(), events.end(), [&](const auto& event)
            {
                return event.kind == kind && (event.pair.first == id || event.pair.second == id);
            });
    }

    RIGIDBODIES_TEST("the default world resolves real contacts with the spatial tree and impulse solver")
    {
        World world { settings() };
        auto left = circle({ -0.45, 0.0 });
        auto right = circle({ 0.45, 0.0 });
        left.linear_velocity_m_s = { 1.0, 0.0 };
        right.linear_velocity_m_s = { -1.0, 0.0 };
        const auto a = world.create_body(left, "a");
        const auto b = world.create_body(right, "b");
        world.step(0.01);
        RIGIDBODIES_EXPECT(dynamic_cast<const DynamicTreeBroadPhase*>(&world.broad_phase()) != nullptr, "persistent tree is the default");
        RIGIDBODIES_EXPECT(world.manifolds().size() == 1, "intersecting circles create an actual contact");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->linear_velocity_m_s().x, 0.0, 1.0e-12, "inelastic equal-mass impact stops the first body");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->linear_velocity_m_s().x, 0.0, 1.0e-12, "equal opposite momentum stops the second");
        RIGIDBODIES_EXPECT(world.find_body(b)->position_m().x - world.find_body(a)->position_m().x > 0.9, "split correction removes overlap");
    }

    RIGIDBODIES_TEST("broad phase overlap events and touching events have distinct lifetimes")
    {
        auto environment = settings();
        environment.collision.continuous = false;
        World world { environment };
        const auto a = world.create_body(circle({ 0.0, 0.0 }, BodyType::static_body, true), "a");
        const auto b = world.create_body(circle({ 0.9, 0.9 }, BodyType::static_body), "b");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.pair_events().size() == 1 && world.contact_events().empty(), "overlapping bounds do not imply geometric contact");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.pair_events().empty(), "an ongoing candidate is not reported as a new pair");
        world.find_body(b)->set_position({ 0.5, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.contact_events().size() == 1 && world.contact_events()[0].is_sensor, "touching a sensor begins an overlap event");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.contact_events().empty(), "persistent contacts do not repeat begin events");
        world.find_body(b)->set_position({ 4.0, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(event_for(world.pair_events(), CollisionEventKind::end, a), "leaving the broad bounds ends the candidate");
        RIGIDBODIES_EXPECT(event_for(world.contact_events(), CollisionEventKind::end, b), "leaving the shape ends sensor overlap");
    }

    RIGIDBODIES_TEST("sensors and excluded colliders never exchange impulses")
    {
        for (const bool sensor : { false, true })
        {
            World world { settings() };
            auto obstacle = box({ 0.0, 0.0 }, BodyType::static_body);
            obstacle.colliders[0].is_sensor = sensor;
            if (!sensor)
                obstacle.colliders[0].filter.mask = 0;
            world.create_body(obstacle, "obstacle");
            auto moving = circle({ 0.5, 0.0 });
            moving.linear_velocity_m_s = { -1.0, 0.0 };
            const auto id = world.create_body(moving, "subject");
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->linear_velocity_m_s().x, -1.0, 0.0, "a non-solid pair cannot change momentum");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->position_m().x, 0.49, 1.0e-12, "a non-solid pair cannot change placement");
            RIGIDBODIES_EXPECT(sensor ? !world.contact_events().empty() : world.contact_events().empty(), "only accepted sensor pairs report contact events");
        }
    }

    RIGIDBODIES_TEST("destroyed contacts emit one end event with the old generation and snapshots preserve it")
    {
        World world { settings() };
        world.create_body(circle({}, BodyType::static_body, true), "sensor");
        const auto old = world.create_body(circle({ 0.5, 0.0 }), "subject");
        world.step(0.01);
        world.destroy_body(old);
        const auto replacement = world.create_body(circle({ 0.5, 0.0 }), "subject");
        const auto checkpoint = world.snapshot();
        world.step(0.01);
        RIGIDBODIES_EXPECT(event_for(world.contact_events(), CollisionEventKind::end, old), "end refers to the destroyed generation");
        RIGIDBODIES_EXPECT(event_for(world.contact_events(), CollisionEventKind::begin, replacement), "replacement is a new contact");
        const auto count = world.contact_events().size();
        world.restore(checkpoint);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.contact_events().size() == count && event_for(world.contact_events(), CollisionEventKind::end, old), "pending end events replay exactly");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.contact_events().empty(), "end events are not replayed on following steps");
        world.clear();
        RIGIDBODIES_EXPECT(world.contact_events().empty() && world.pair_events().empty(), "a bulk reset clears event history");
    }

    RIGIDBODIES_TEST("contact caches survive snapshots and scale with a changed physical timestep")
    {
        auto environment = settings();
        environment.gravity_m_s2 = { 0.0, -10.0 };
        World world { environment };
        world.create_body(box({ 0.0, -0.5 }, BodyType::static_body), "floor");
        const auto id = world.create_body(box({ 0.0, 0.5 }), "subject");
        world.find_body(id)->override_mass(1.0);
        for (int step = 0; step < 60; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(world.statistics().persistent_contact_point_count == 2, "both face contact features retain independent cache entries");
        const auto checkpoint = world.snapshot();
        world.step(0.02);
        Real normal_impulse = 0.0;
        for (const auto& manifold : world.manifolds())
            for (std::size_t index = 0; index < manifold.point_count; ++index)
                normal_impulse += manifold.points[index].normal_impulse_n_s;
        RIGIDBODIES_EXPECT_NEAR(normal_impulse, 0.2, 1.0e-8, "cached force adapts to doubled step duration");
        const auto expected = *world.find_body(id);
        const auto expected_cache = world.manifolds()[0].points[0].normal_impulse_n_s;
        world.restore(checkpoint);
        world.step(0.02);
        RIGIDBODIES_EXPECT(world.find_body(id)->position_m() == expected.position_m() && world.find_body(id)->linear_velocity_m_s() == expected.linear_velocity_m_s(), "body state replays bit for bit");
        RIGIDBODIES_EXPECT_NEAR(world.manifolds()[0].points[0].normal_impulse_n_s, expected_cache, 0.0, "contact warm cache also replays exactly");
    }

    RIGIDBODIES_TEST("stable body keys give identical solved motion across reversed creation histories")
    {
        auto environment = settings();
        environment.gravity_m_s2 = { 0.0, -10.0 };
        World first { environment };
        World second { environment };
        const auto populate = [](World& world, bool reverse)
        {
            if (!reverse)
                world.create_body(box({ 0.0, -0.5 }, BodyType::static_body), "floor");
            for (int index = 0; index < 3; ++index)
            {
                const auto layer = reverse ? 2 - index : index;
                world.create_body(box({ 0.0, 0.51 + static_cast<Real>(layer) * 1.01 }), "box_" + std::to_string(layer));
            }
            if (reverse)
                world.create_body(box({ 0.0, -0.5 }, BodyType::static_body), "floor");
        };
        populate(first, false);
        populate(second, true);
        for (int step = 0; step < 240; ++step)
        {
            first.step(1.0 / 120.0);
            second.step(1.0 / 120.0);
        }
        const auto a = first.body_ids();
        const auto b = second.body_ids();
        for (std::size_t index = 0; index < a.size(); ++index)
        {
            RIGIDBODIES_EXPECT(first.find_body(a[index])->position_m() == second.find_body(b[index])->position_m(), "canonical pair and solve order gives identical placement");
            RIGIDBODIES_EXPECT(first.find_body(a[index])->linear_velocity_m_s() == second.find_body(b[index])->linear_velocity_m_s(), "canonical order gives identical velocity");
        }
    }

    RIGIDBODIES_TEST("explicit null collision implementations still provide an isolated free motion world")
    {
        World world { settings() };
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        world.set_contact_solver(std::make_shared<NullContactSolver>());
        world.create_body(box({}, BodyType::static_body), "obstacle");
        auto definition = circle({ 0.5, 0.0 });
        definition.linear_velocity_m_s = { -2.0, 0.0 };
        const auto id = world.create_body(definition, "subject");
        world.step(0.5);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->position_m().x, -0.5, 1.0e-12, "null narrow phase opts out of discrete and continuous resolution");
        RIGIDBODIES_EXPECT(world.manifolds().empty(), "null seam stays available for isolated integrator experiments");
    }

    RIGIDBODIES_TEST("replacing collision components releases sleeping bodies from obsolete support")
    {
        for (const bool replace_narrow_phase : { false, true })
        {
            WorldSettings environment;
            environment.gravity_m_s2 = { 0.0, -10.0 };
            environment.sleep.quiet_duration_s = 0.1;
            World world { environment };
            world.create_body(box({ 0.0, -0.5 }, BodyType::static_body), "floor");
            const auto id = world.create_body(box({ 0.0, 0.5 }), "subject");
            world.find_body(id)->override_mass(1.0);
            for (int step = 0; step < 120; ++step)
                world.step(0.01);
            RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake(), "the old response has settled the subject into supported sleep");
            RIGIDBODIES_EXPECT(!world.manifolds().empty() && world.manifolds()[0].points[0].normal_impulse_n_s > 0.0,
                "the sleeping support carries a cached normal impulse");
            const auto prior_y = world.find_body(id)->position_m().y;
            if (replace_narrow_phase)
                world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
            else
                world.set_contact_solver(std::make_shared<NullContactSolver>());
            RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "changing collision behavior immediately wakes settled bodies");
            for (const auto& manifold : world.manifolds())
                for (std::size_t index = 0; index < manifold.point_count; ++index)
                    RIGIDBODIES_EXPECT_NEAR(manifold.points[index].normal_impulse_n_s, 0.0, 0.0, "a replacement cannot inherit the previous solver's support impulse");
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->linear_velocity_m_s().y, -0.1, 1.0e-12, "unchanged gravity resumes immediately after removing collision response");
            RIGIDBODIES_EXPECT(world.find_body(id)->position_m().y < prior_y,
                "the released body moves instead of remaining asleep above its former support");
            if (replace_narrow_phase)
            {
                RIGIDBODIES_EXPECT(world.manifolds().empty(), "null narrow phase removes the old contact geometry");
                RIGIDBODIES_EXPECT(event_for(world.contact_events(), CollisionEventKind::end, id),
                    "the removed contact still emits its end event on the next step");
            }
            for (int step = 0; step < 10; ++step)
                world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(id)->is_awake() && world.find_body(id)->linear_velocity_m_s().y < -1.0,
                "obsolete support cannot put the falling body back to sleep");
        }
    }

    RIGIDBODIES_TEST("world restitution mixing settings select the actual contact material")
    {
        for (const auto policy : { MaterialMixing::minimum, MaterialMixing::maximum, MaterialMixing::arithmetic_mean, MaterialMixing::geometric_mean })
        {
            auto environment = settings();
            environment.collision.restitution_mixing = policy;
            World world { environment };
            auto floor = circle({}, BodyType::static_body, true);
            floor.colliders[0].material.restitution = 0.2;
            auto ball = circle({ 0.9, 0.0 });
            ball.colliders[0].material.restitution = 0.8;
            world.create_body(floor, "floor");
            world.create_body(ball, "subject");
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.manifolds()[0].material.restitution, mix_material_values(0.2, 0.8, policy), 1.0e-14, "world policy reaches the solver's contact record");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
