#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyId body(World& world, BodyType type, Vec2 position, Vec2 velocity = {}, Real mass = 1.0, bool fixed = false)
    {
        BodyDefinition definition;
        definition.type = type;
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        definition.fixed_rotation = fixed;
        Collider collider;
        collider.shape = make_box(1.0, 1.0);
        collider.material.restitution = 0.0;
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(mass);
        return id;
    }

    ContactManifold contact(BodyId first, BodyId second, Vec2 normal, Vec2 point, Real separation = 0.0)
    {
        ContactManifold manifold;
        manifold.first = first;
        manifold.second = second;
        manifold.normal = normal;
        manifold.points[0].world_position_m = point;
        manifold.points[0].separation_m = separation;
        manifold.point_count = 1;
        return manifold;
    }

    void velocity_solve(World& world, std::vector<ContactManifold>& contacts, const SolverSettings& settings = {}, int iterations = 12, Real dt = 0.01)
    {
        SequentialImpulseContactSolver solver;
        solver.prepare(world, contacts, settings, dt);
        for (int index = 0; index < iterations; ++index)
            solver.solve_velocity(world, contacts, settings, dt);
    }

    RIGIDBODIES_TEST("equal mass elastic and inelastic contact impulses match analytical momentum and energy")
    {
        for (const auto restitution : { 0.0, 1.0 })
        {
            World world;
            const auto first = body(world, BodyType::dynamic_body, { -0.5, 0.0 }, { 2.0, 0.0 });
            const auto second = body(world, BodyType::dynamic_body, { 0.5, 0.0 }, { -1.0, 0.0 });
            std::vector<ContactManifold> contacts { contact(first, second, { 1.0, 0.0 }, {}) };
            contacts[0].material.restitution = restitution;
            velocity_solve(world, contacts);
            const auto* a = world.find_body(first);
            const auto* b = world.find_body(second);
            RIGIDBODIES_EXPECT_NEAR(a->linear_velocity_m_s().x, 0.5 - 1.5 * restitution, 1.0e-12, "first velocity follows impulse law");
            RIGIDBODIES_EXPECT_NEAR(b->linear_velocity_m_s().x, 0.5 + 1.5 * restitution, 1.0e-12, "second velocity follows impulse law");
            RIGIDBODIES_EXPECT_NEAR((a->linear_momentum_kg_m_s() + b->linear_momentum_kg_m_s()).x, 1.0, 1.0e-12, "linear momentum conserved");
            RIGIDBODIES_EXPECT_NEAR(a->kinetic_energy_j() + b->kinetic_energy_j(), restitution == 1.0 ? 2.5 : 0.25, 1.0e-12, "elastic energy conserved and inelastic relative energy removed");
        }
    }

    RIGIDBODIES_TEST("unequal masses and offset contacts use translational and rotational effective mass")
    {
        World world;
        const auto first = body(world, BodyType::dynamic_body, { -0.5, 0.0 }, { 3.0, 0.0 }, 2.0);
        const auto second = body(world, BodyType::dynamic_body, { 0.5, 0.0 });
        std::vector<ContactManifold> contacts { contact(first, second, { 1.0, 0.0 }, {}) };
        contacts[0].material.restitution = 0.5;
        velocity_solve(world, contacts);
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].normal_impulse_n_s, 3.0, 1.0e-12, "unequal mass normal impulse");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->linear_velocity_m_s().x, 1.5, 1.0e-12, "heavy body's velocity change");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(second)->linear_velocity_m_s().x, 3.0, 1.0e-12, "light body's velocity change");

        World offset_world;
        const auto wall = body(offset_world, BodyType::static_body, {});
        const auto moving = body(offset_world, BodyType::dynamic_body, { 0.5, 0.0 }, { -2.0, 0.0 });
        auto* moving_body = offset_world.find_body(moving);
        const auto expected_impulse = 2.0 / (moving_body->inverse_mass() + 0.25 * moving_body->inverse_inertia());
        std::vector<ContactManifold> offset_contacts { contact(wall, moving, { 1.0, 0.0 }, { 0.0, 0.5 }) };
        velocity_solve(offset_world, offset_contacts);
        RIGIDBODIES_EXPECT_NEAR(offset_contacts[0].points[0].normal_impulse_n_s, expected_impulse, 1.0e-12, "lever arm reduces effective normal mass");
        RIGIDBODIES_EXPECT_NEAR(moving_body->angular_velocity_rad_s(), -0.5 * moving_body->inverse_inertia() * expected_impulse, 1.0e-12, "off-center impact generates angular impulse");
        RIGIDBODIES_EXPECT_NEAR(moving_body->velocity_at_world_point({ 0.0, 0.5 }).x, 0.0, 1.0e-12, "contact normal velocity stops");
    }

    RIGIDBODIES_TEST("two point normal block solve supports a flat box without artificial spin")
    {
        World world;
        const auto ground = body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -2.0 });
        auto manifold = contact(ground, box, { 0.0, 1.0 }, { -0.5, 0.0 });
        manifold.point_count = 2;
        manifold.points[1].world_position_m = { 0.5, 0.0 };
        std::vector<ContactManifold> contacts { manifold };
        velocity_solve(world, contacts, {}, 1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, 0.0, 1.0e-12, "two contact points cancel downward motion in one block solve");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->angular_velocity_rad_s(), 0.0, 1.0e-12, "balanced support introduces no rotation");
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].normal_impulse_n_s, 1.0, 1.0e-12, "left point carries half the support");
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[1].normal_impulse_n_s, 1.0, 1.0e-12, "right point carries half the support");
    }

    RIGIDBODIES_TEST("restitution threshold uses fixed incoming velocity and repeated iterations do not erase bounce")
    {
        for (const auto speed : { 0.5, 2.0 })
        {
            World world;
            const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
            const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -speed });
            std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, {}) };
            contacts[0].material.restitution = 1.0;
            velocity_solve(world, contacts, {}, 30);
            RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].pre_solve_normal_velocity_m_s, -speed, 0.0, "incoming normal speed captured once");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, speed > 1.0 ? speed : 0.0, 1.0e-12, "only sufficiently fast impacts rebound");
        }
    }

    RIGIDBODIES_TEST("Coulomb friction sticks below static capacity and slides at the kinetic normal-coupled limit")
    {
        for (const auto speed : { 0.2, 2.0 })
        {
            World world;
            const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
            const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { speed, -1.0 }, 1.0, true);
            std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, {}) };
            contacts[0].material.static_friction = 0.5;
            contacts[0].material.kinetic_friction = 0.25;
            velocity_solve(world, contacts);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().x, speed < 0.5 ? 0.0 : speed - 0.25, 1.0e-12, "tangent solve selects static or kinetic response");
            RIGIDBODIES_EXPECT(std::abs(contacts[0].points[0].tangent_impulse_n_s) <= 0.5 * contacts[0].points[0].normal_impulse_n_s, "friction is bounded by normal support");
        }
        for (const auto angle : { 0.2, 0.8 })
        {
            World world;
            const Vec2 normal { -std::sin(angle), std::cos(angle) };
            const auto floor = body(world, BodyType::static_body, -normal * 0.5);
            const auto box = body(world, BodyType::dynamic_body, normal * 0.5, { 0.0, -0.1 }, 1.0, true);
            std::vector<ContactManifold> contacts { contact(floor, box, normal, {}) };
            contacts[0].material.static_friction = 0.5;
            contacts[0].material.kinetic_friction = 0.25;
            velocity_solve(world, contacts);
            const auto tangential = rigidbodies::math::dot(world.find_body(box)->linear_velocity_m_s(), rigidbodies::math::perpendicular(normal));
            const auto expected = angle < 0.3 ? 0.0 : 0.1 * (std::sin(angle) - 0.25 * std::cos(angle));
            RIGIDBODIES_EXPECT_NEAR(tangential, expected, 1.0e-12, "incline sticks below friction angle and slides above it");
        }
    }

    RIGIDBODIES_TEST("rolling and spinning resistance consume bounded angular impulse without reversing spin")
    {
        for (const auto resistance : { Vec2 { 0.1, 0.0 }, Vec2 { 0.0, 0.2 }, Vec2 { 0.1, 0.2 }, Vec2 { 3.0, 3.0 } })
        {
            World world;
            const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
            const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -1.0 });
            world.find_body(box)->set_angular_velocity(10.0);
            std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, {}) };
            contacts[0].material.rolling_friction_m = resistance.x;
            contacts[0].material.spinning_friction_m = resistance.y;
            velocity_solve(world, contacts);
            const auto inverse_inertia = world.find_body(box)->inverse_inertia();
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->angular_velocity_rad_s(), std::max(0.0, 10.0 - inverse_inertia * (resistance.x + resistance.y)), 1.0e-12, "angular resistance adds its two normal-coupled capacities");
            RIGIDBODIES_EXPECT(std::abs(contacts[0].points[0].rolling_impulse_n_m_s) <= resistance.x + 1.0e-12, "rolling impulse within its own bound");
            RIGIDBODIES_EXPECT(std::abs(contacts[0].points[0].spinning_impulse_n_m_s) <= resistance.y + 1.0e-12, "spinning impulse within its own bound");
        }
    }

    RIGIDBODIES_TEST("anisotropic friction axes follow both body and collider rotation")
    {
        for (int rotation_case = 0; rotation_case < 3; ++rotation_case)
        {
            World world;
            const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
            BodyDefinition definition;
            definition.position_m = { 0.0, 0.5 };
            definition.linear_velocity_m_s = { 2.0, -1.0 };
            definition.fixed_rotation = true;
            definition.orientation_rad = rotation_case == 1 ? rigidbodies::math::pi * 0.5 : 0.0;
            Collider collider;
            collider.shape = make_box(1.0, 1.0);
            collider.local_transform.rotation = rigidbodies::math::Rotation2 { rotation_case == 2 ? rigidbodies::math::pi * 0.5 : 0.0 };
            definition.colliders.push_back(collider);
            const auto box = world.create_body(definition);
            world.find_body(box)->override_mass(1.0);
            Material ground_material;
            ground_material.static_friction = 0.5;
            ground_material.kinetic_friction = 0.25;
            auto moving_material = ground_material;
            moving_material.friction_anisotropy_ratio = 4.0;
            std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, {}) };
            contacts[0].material = combine_materials(ground_material, moving_material);
            velocity_solve(world, contacts);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().x, rotation_case == 0 ? 1.5 : 1.75, 1.0e-12, "parallel friction scales with local preferred axis and perpendicular friction stays unchanged");
        }
    }

    RIGIDBODIES_TEST("speculative contact only limits closing speed until actual touch")
    {
        for (const auto speed : { 1.0, 5.0 })
        {
            World world;
            const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
            const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.7 }, { 2.0, -speed }, 1.0, true);
            std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, { 0.0, 0.1 }, 0.2) };
            contacts[0].is_speculative = true;
            contacts[0].material = { 1.0, 1.0, 1.0 };
            contacts[0].points[0].speculative_velocity_bias_m_s = 0.5;
            velocity_solve(world, contacts, {}, 12, 0.1);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, speed < 1.5 ? -speed : -1.5, 1.0e-12, "closing speed respects gap over step with trajectory bias");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().x, 2.0, 1.0e-12, "positive gap creates neither premature friction nor bounce");
            RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].tangent_impulse_n_s, 0.0, 0.0, "speculative friction cache remains empty");
            RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].pending_impact_speed_m_s, speed < 1.5 ? 0.0 : speed, 0.0, "only an active speculative speed cap retains incoming impact speed");
        }
    }

    RIGIDBODIES_TEST("speculative speed caps preserve restitution at first touch and consume the saved impact exactly once")
    {
        World world;
        const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.7 }, { 0.0, -5.0 }, 1.0, true);
        std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, { 0.0, 0.1 }, 0.2) };
        contacts[0].is_speculative = true;
        contacts[0].material.restitution = 0.8;
        velocity_solve(world, contacts, {}, 12, 0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, -2.0, 1.0e-12, "speculative contact allows travel to the surface");
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].pending_impact_speed_m_s, 5.0, 0.0, "physical incoming speed survives speed cap");
        world.find_body(box)->set_position({ 0.0, 0.5 });
        contacts[0].points[0].world_position_m = {};
        contacts[0].points[0].separation_m = 5.0e-10;
        contacts[0].is_speculative = false;
        velocity_solve(world, contacts, {}, 12, 0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, 4.0, 1.0e-12, "touching response uses restitution times original impact speed");
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].pending_impact_speed_m_s, 0.0, 0.0, "saved impact consumed on touch");
        velocity_solve(world, contacts, {}, 12, 0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, 4.0, 1.0e-12, "resting cache cannot cause a second rebound");
    }

    RIGIDBODIES_TEST("transported speculative anchors avoid spurious normal impact torque")
    {
        World world;
        const auto wall = body(world, BodyType::static_body, {});
        const auto moving = body(world, BodyType::dynamic_body, { 1.0, 1.0 }, { -5.0, 0.0 });
        std::vector<ContactManifold> contacts { contact(wall, moving, { 1.0, 0.0 }, { 0.25, 0.5 }, 0.5) };
        auto& point = contacts[0].points[0];
        point.body_anchors_valid = true;
        point.local_anchor_first_m = {};
        point.local_anchor_second_m = { -0.5, 0.0 };
        contacts[0].is_speculative = true;
        velocity_solve(world, contacts, {}, 12, 0.2);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->linear_velocity_m_s().x, -2.5, 1.0e-12, "speculative constraint acts at transported normal feature");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->angular_velocity_rad_s(), 0.0, 1.0e-12, "tangential witness displacement does not invent a torque lever arm");
    }

    RIGIDBODIES_TEST("frictional velocity response conserves angular momentum for overlapping dynamic pairs")
    {
        World world;
        const auto first = body(world, BodyType::dynamic_body, { -0.4, 0.0 }, { 1.0, 1.0 });
        const auto second = body(world, BodyType::dynamic_body, { 0.4, 0.0 }, { -1.0, -1.0 });
        const auto angular_momentum = [&]()
        {
            Real sum = 0.0;
            for (const auto id : { first, second })
            {
                const auto* moving = world.find_body(id);
                sum += rigidbodies::math::cross(moving->world_center_of_mass_m(), moving->linear_momentum_kg_m_s()) + moving->angular_momentum_about_center_kg_m2_s();
            }
            return sum;
        };
        const auto before = angular_momentum();
        std::vector<ContactManifold> contacts { contact(first, second, { 1.0, 0.0 }, {}, -0.2) };
        contacts[0].material.static_friction = 1.0;
        contacts[0].material.kinetic_friction = 0.8;
        velocity_solve(world, contacts);
        RIGIDBODIES_EXPECT_NEAR(angular_momentum(), before, 1.0e-12, "equal and opposite friction impulses share a common application point");
    }

    RIGIDBODIES_TEST("position correction removes overlap without adding kinetic energy or corrupting interpolation history")
    {
        World world;
        const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.3 }, {}, 1.0, true);
        std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, { 0.0, -0.1 }, -0.2) };
        SequentialImpulseContactSolver solver;
        SolverSettings settings;
        solver.prepare(world, contacts, settings, 0.01);
        for (int index = 0; index < 10; ++index)
            solver.solve_position(world, contacts, settings, 0.01);
        RIGIDBODIES_EXPECT(world.find_body(box)->position_m().y > 0.47, "body moves out of initial overlap");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->kinetic_energy_j(), 0.0, 0.0, "pseudo-position impulses do not become kinetic energy");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->previous_transform().translation.y, 0.3, 0.0, "position solve preserves interpolation start pose");
    }

    RIGIDBODIES_TEST("warm start can be disabled and sleeping contacts retain support without receiving impulses")
    {
        World world;
        const auto floor = body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto box = body(world, BodyType::dynamic_body, { 0.0, 0.5 });
        std::vector<ContactManifold> contacts { contact(floor, box, { 0.0, 1.0 }, {}) };
        contacts[0].points[0].normal_impulse_n_s = 0.5;
        SequentialImpulseContactSolver solver;
        SolverSettings settings;
        solver.prepare(world, contacts, settings, 0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, 0.5, 0.0, "warm start applies prior support impulse");
        world.find_body(box)->set_linear_velocity({});
        settings.warm_starting = false;
        solver.prepare(world, contacts, settings, 0.01);
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].normal_impulse_n_s, 0.0, 0.0, "disabled warm start clears cached impulse");
        contacts[0].points[0].normal_impulse_n_s = 3.0;
        world.find_body(box)->set_awake(false);
        settings.warm_starting = true;
        solver.prepare(world, contacts, settings, 0.01);
        solver.solve_velocity(world, contacts, settings, 0.01);
        RIGIDBODIES_EXPECT(!world.find_body(box)->is_awake(), "resting support does not wake sleeping body");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(box)->linear_velocity_m_s().y, 0.0, 0.0, "cached support is not reapplied to a sleeping body");
        RIGIDBODIES_EXPECT_NEAR(contacts[0].points[0].normal_impulse_n_s, 3.0, 0.0, "support cache is retained for sleeping contact");
    }

    RIGIDBODIES_TEST("prepared solver clones contain no references into the source world and sensor contacts do not solve")
    {
        World world;
        const auto first = body(world, BodyType::dynamic_body, { -0.5, 0.0 }, { 2.0, 0.0 });
        const auto second = body(world, BodyType::dynamic_body, { 0.5, 0.0 }, { -1.0, 0.0 });
        std::vector<ContactManifold> contacts { contact(first, second, { 1.0, 0.0 }, {}) };
        SequentialImpulseContactSolver solver;
        solver.prepare(world, contacts, {}, 0.01);
        auto copy = solver.clone();
        auto copied_contacts = contacts;
        World copied_world;
        copied_world.restore(world.snapshot());
        copy->solve_velocity(copied_world, copied_contacts, {}, 0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->linear_velocity_m_s().x, 2.0, 0.0, "clone solve cannot modify source world");
        solver.solve_velocity(world, contacts, {}, 0.01);
        RIGIDBODIES_EXPECT(world.find_body(first)->linear_velocity_m_s() == copied_world.find_body(first)->linear_velocity_m_s(), "clone produces identical response in restored world");
        contacts[0].is_sensor = true;
        world.find_body(first)->set_linear_velocity({ 2.0, 0.0 });
        world.find_body(second)->set_linear_velocity({ -1.0, 0.0 });
        velocity_solve(world, contacts);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->linear_velocity_m_s().x, 2.0, 0.0, "sensor manifold never applies warm start or response");
    }

    RIGIDBODIES_TEST("friction and restitution mixing policies are independent and feature IDs preserve indices above 255")
    {
        Material first;
        Material second;
        first.restitution = 0.2;
        second.restitution = 0.8;
        first.static_friction = 0.25;
        second.static_friction = 1.0;
        first.kinetic_friction = 0.1;
        second.kinetic_friction = 0.4;
        const auto defaults = combine_materials(first, second);
        RIGIDBODIES_EXPECT_NEAR(defaults.restitution, 0.8, 0.0, "default maximum restitution preserved");
        RIGIDBODIES_EXPECT_NEAR(defaults.static_friction, 0.5, 1.0e-12, "default geometric friction preserved");
        const auto changed = combine_materials(first, second, MaterialMixing::minimum, MaterialMixing::arithmetic_mean);
        RIGIDBODIES_EXPECT_NEAR(changed.restitution, 0.5, 1.0e-12, "restitution policy independently selectable");
        RIGIDBODIES_EXPECT_NEAR(changed.static_friction, 0.25, 0.0, "friction policy independently selectable");
        RIGIDBODIES_EXPECT_NEAR(mix_material_values(1.0e300, 1.0e300, MaterialMixing::geometric_mean), 1.0e300, 1.0e285, "geometric mixing avoids avoidable intermediate overflow");
        RIGIDBODIES_EXPECT_NEAR(mix_material_values(std::numeric_limits<Real>::quiet_NaN(), 1.0, MaterialMixing::geometric_mean), 0.0, 0.0, "invalid material value cannot contaminate solver");
        ContactFeatureId high;
        high.incoming_edge = 300;
        ContactFeatureId low;
        low.incoming_edge = 44;
        RIGIDBODIES_EXPECT(high.key() != low.key(), "large polygon feature indices do not alias eight-bit values");
    }

    RIGIDBODIES_TEST("contact block solver maintains a four box stack over a sustained run")
    {
        WorldSettings settings;
        settings.sleep.enabled = false;
        settings.solver.velocity_iterations = 16;
        settings.solver.position_iterations = 6;
        World world { settings };
        world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
        world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
        BodyDefinition floor;
        floor.type = BodyType::static_body;
        floor.position_m = { 0.0, -0.5 };
        Collider floor_collider;
        floor_collider.shape = make_box(10.0, 1.0);
        floor_collider.material.restitution = 0.0;
        floor.colliders.push_back(floor_collider);
        world.create_body(floor, "floor");
        std::vector<BodyId> boxes;
        boxes.reserve(4);
        for (int index = 0; index < 4; ++index)
            boxes.push_back(body(world, BodyType::dynamic_body, { 0.0, 0.5 + index }));
        for (int step = 0; step < 1200; ++step)
            world.step(1.0 / 120.0);
        for (std::size_t index = 0; index < boxes.size(); ++index)
        {
            const auto* box = world.find_body(boxes[index]);
            RIGIDBODIES_EXPECT_NEAR(box->position_m().y, 0.5 + static_cast<Real>(index), 0.06, "stack retains its supported height");
            RIGIDBODIES_EXPECT_NEAR(box->position_m().x, 0.0, 0.05, "stack does not drift laterally");
            RIGIDBODIES_EXPECT(rigidbodies::math::length(box->linear_velocity_m_s()) < 0.05, "resting stack velocity remains small");
            RIGIDBODIES_EXPECT(std::abs(box->orientation_rad()) < 0.05, "block normal solve avoids accumulating tilt");
        }
    }

    RIGIDBODIES_TEST("a physical ramp holds a shallow block and lets a steep block slide")
    {
        for (const auto angle : { 0.2, 0.8 })
        {
            WorldSettings settings;
            settings.sleep.enabled = false;
            settings.solver.velocity_iterations = 12;
            settings.solver.position_iterations = 6;
            World world { settings };
            world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
            world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
            const Vec2 normal { std::sin(angle), std::cos(angle) };
            const Vec2 downhill { std::cos(angle), -std::sin(angle) };
            Material surface;
            surface.restitution = 0.0;
            surface.static_friction = 0.5;
            surface.kinetic_friction = 0.25;
            BodyDefinition ramp;
            ramp.type = BodyType::static_body;
            ramp.orientation_rad = -angle;
            Collider ramp_collider;
            ramp_collider.shape = make_box(8.0, 0.2);
            ramp_collider.material = surface;
            ramp.colliders.push_back(ramp_collider);
            world.create_body(ramp);
            BodyDefinition block;
            block.position_m = normal * 0.35;
            block.orientation_rad = -angle;
            block.fixed_rotation = true;
            Collider block_collider;
            block_collider.shape = make_box(0.5, 0.5);
            block_collider.material = surface;
            block.colliders.push_back(block_collider);
            const auto id = world.create_body(block);
            world.find_body(id)->override_mass(1.0);
            for (int step = 0; step < 120; ++step)
                world.step(1.0 / 120.0);
            const auto* result = world.find_body(id);
            const auto displacement = rigidbodies::math::dot(result->position_m(), downhill);
            if (angle < 0.5)
                RIGIDBODIES_EXPECT_NEAR(displacement, 0.0, 0.02, "static friction keeps shallow block in place");
            else
                RIGIDBODIES_EXPECT(displacement > 1.0, "block exceeds static capacity and slides down steep ramp");
            RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::dot(result->position_m(), normal), 0.35, 0.03, "normal response keeps block on the ramp surface");
        }
    }

    RIGIDBODIES_TEST("connected resting stacks sleep together and wake together while independent floor neighbors remain separate")
    {
        for (const bool warm_starting : { false, true })
        {
            WorldSettings settings;
            settings.sleep.quiet_duration_s = 0.2;
            settings.solver.velocity_iterations = 16;
            settings.solver.position_iterations = 6;
            settings.solver.warm_starting = warm_starting;
            World world { settings };
            world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
            world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
            BodyDefinition floor;
            floor.type = BodyType::static_body;
            floor.position_m = { 0.0, -0.5 };
            Collider collider;
            collider.shape = make_box(10.0, 1.0);
            collider.material.restitution = 0.0;
            floor.colliders.push_back(collider);
            world.create_body(floor);
            std::vector<BodyId> boxes;
            boxes.reserve(3);
            for (int index = 0; index < 3; ++index)
                boxes.push_back(body(world, BodyType::dynamic_body, { 0.0, 0.5 + index }));
            const auto independent = body(world, BodyType::dynamic_body, { 3.0, 0.5 });
            world.find_body(independent)->set_sleep_enabled(false);
            for (int step = 0; step < 12; ++step)
                world.step(1.0 / 120.0);
            world.find_body(boxes.back())->wake(); // Deliberately stagger the members' quiet timers.
            for (int step = 0; step < 360; ++step)
                world.step(1.0 / 120.0);
            for (const auto id : boxes)
                RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake(), "all supported stack members settle into one sleeping state");
            RIGIDBODIES_EXPECT(world.find_body(independent)->is_awake(), "shared static floor does not merge independent sleeping groups");
            RIGIDBODIES_EXPECT(world.statistics().sleeping_body_count == 3, "sleeping statistics account for the whole resting stack");
            world.find_body(boxes.back())->apply_linear_impulse({ 0.0, 1.0 });
            world.step(1.0 / 120.0);
            for (const auto id : boxes)
                RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "an external top impulse wakes the complete contact group before response");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
