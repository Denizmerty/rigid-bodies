#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/time_stepper.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition falling_ball(Real radius_m = 0.05, const Material& material = materials::steel())
    {
        BodyDefinition definition;
        definition.name = "ball";
        Collider collider;
        collider.shape = make_circle(radius_m);
        collider.material = material;
        collider.depth_m = 2.0 * radius_m;
        definition.colliders.push_back(std::move(collider));
        return definition;
    }

    RIGIDBODIES_TEST("a disc reports the closed-form mass and inertia for its material")
    {
        const Real radius_m = 0.05;
        const Real depth_m = 0.1;
        const auto material = materials::steel();

        Collider collider;
        collider.shape = make_circle(radius_m);
        collider.material = material;
        collider.depth_m = depth_m;

        const auto properties = collider.compute_mass_properties();
        const auto expected_mass = rigidbodies::math::pi * radius_m * radius_m * depth_m * material.density_kg_m3;
        RIGIDBODIES_EXPECT_NEAR(properties.mass_kg, expected_mass, 1.0e-12, "mass follows from area, depth, and density");
        RIGIDBODIES_EXPECT_NEAR(properties.inertia_kg_m2, 0.5 * expected_mass * radius_m * radius_m, 1.0e-12, "a uniform disc has m r squared over two");
    }

    RIGIDBODIES_TEST("a box reports the closed-form inertia about its centre")
    {
        const Real width_m = 0.4;
        const Real height_m = 0.2;
        const Real depth_m = 0.05;
        const auto material = materials::oak_wood();

        Collider collider;
        collider.shape = make_box(width_m, height_m);
        collider.material = material;
        collider.depth_m = depth_m;

        const auto properties = collider.compute_mass_properties();
        const auto expected_mass = width_m * height_m * depth_m * material.density_kg_m3;
        const auto expected_inertia = expected_mass * (width_m * width_m + height_m * height_m) / 12.0;
        RIGIDBODIES_EXPECT_NEAR(properties.mass_kg, expected_mass, 1.0e-12, "mass follows from the rectangle area");
        RIGIDBODIES_EXPECT_NEAR(properties.inertia_kg_m2, expected_inertia, 1.0e-12, "rectangle inertia matches the closed form");
    }

    RIGIDBODIES_TEST("a compound body places its centre of mass towards the heavier part")
    {
        BodyDefinition definition;
        definition.name = "hammer";

        Collider handle;
        handle.shape = make_box(0.3, 0.03);
        handle.material = materials::oak_wood();

        Collider head;
        head.shape = make_box(0.1, 0.05);
        head.material = materials::steel();
        head.local_transform = rigidbodies::math::Transform2::from_angle({ 0.16, 0.0 }, 0.0);

        definition.colliders.push_back(handle);
        definition.colliders.push_back(head);

        const RigidBody body { definition };
        RIGIDBODIES_EXPECT(body.mass_properties().mass_kg > 0.0, "the assembled body has mass");
        RIGIDBODIES_EXPECT(body.mass_properties().center_of_mass_m.x > 0.1, "the centre of mass sits near the steel head");
    }

    RIGIDBODIES_TEST("the parallel-axis shift adds the offset term")
    {
        const Real inertia = 0.02;
        const Real mass = 1.5;
        const Real offset = 0.4;
        RIGIDBODIES_EXPECT_NEAR(shift_inertia(inertia, mass, offset), inertia + mass * offset * offset, 1.0e-12, "inertia grows by m d squared");
    }

    RIGIDBODIES_TEST("free fall reaches the speed and drop that constant acceleration predicts")
    {
        WorldSettings settings;
        World world { settings };
        // Drag would make the comparison against the closed form meaningless, so the world runs
        // with gravity alone.
        const auto body_id = world.create_body(falling_ball());

        const Real step = 1.0 / 1000.0;
        const int steps = 1000;
        for (int index = 0; index < steps; ++index)
        {
            world.step(step);
        }

        const auto* body = world.find_body(body_id);
        RIGIDBODIES_EXPECT(body != nullptr, "the body outlives the stepping");

        const auto elapsed = step * static_cast<Real>(steps);
        const auto expected_speed = standard_gravity_m_s2 * elapsed;
        RIGIDBODIES_EXPECT_NEAR(-body->linear_velocity_m_s().y, expected_speed, 1.0e-9, "speed after one second of free fall");

        // Semi-implicit Euler advances position with the already-updated velocity, so the drop is
        // the analytic value plus half a step of that velocity. The offset is first order in the
        // step size and shrinks as the step does.
        const auto expected_drop = 0.5 * standard_gravity_m_s2 * elapsed * elapsed + 0.5 * step * expected_speed;
        RIGIDBODIES_EXPECT_NEAR(-body->position_m().y, expected_drop, 1.0e-6, "drop after one second of free fall");
    }

    RIGIDBODIES_TEST("mass does not change how fast an object falls")
    {
        World heavy_world;
        World light_world;
        const auto heavy = heavy_world.create_body(falling_ball(0.05, materials::steel()));
        const auto light = light_world.create_body(falling_ball(0.05, materials::expanded_polystyrene()));

        for (int index = 0; index < 240; ++index)
        {
            heavy_world.step(1.0 / 240.0);
            light_world.step(1.0 / 240.0);
        }

        const auto heavy_drop = heavy_world.find_body(heavy)->position_m().y;
        const auto light_drop = light_world.find_body(light)->position_m().y;
        RIGIDBODIES_EXPECT_NEAR(heavy_drop, light_drop, 1.0e-12, "both bodies fall together under gravity alone");
    }

    RIGIDBODIES_TEST("a static body ignores forces and impulses")
    {
        World world;
        auto definition = falling_ball();
        definition.type = BodyType::static_body;
        const auto id = world.create_body(definition);

        auto* body = world.find_body(id);
        body->apply_force_at_center({ 100.0, 100.0 });
        body->apply_linear_impulse({ 50.0, 50.0 });
        world.step(1.0 / 120.0);

        RIGIDBODIES_EXPECT(body->inverse_mass() == 0.0, "a static body has no inverse mass");
        RIGIDBODIES_EXPECT_NEAR(rigidbodies::math::length(body->linear_velocity_m_s()), 0.0, 1.0e-12, "a static body does not move");
    }

    RIGIDBODIES_TEST("an off-centre force produces both acceleration and spin")
    {
        World world;
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        world.set_settings(settings);

        BodyDefinition definition;
        definition.name = "plate";
        Collider collider;
        collider.shape = make_box(0.4, 0.4);
        collider.material = materials::aluminium();
        definition.colliders.push_back(collider);

        const auto id = world.create_body(definition);
        auto* body = world.find_body(id);
        body->apply_force_at_world_point({ 0.0, 10.0 }, { 0.15, 0.0 });

        RIGIDBODIES_EXPECT(body->accumulated_torque_n_m() > 0.0, "a force applied off the centre produces torque");

        world.step(1.0 / 240.0);
        RIGIDBODIES_EXPECT(body->linear_velocity_m_s().y > 0.0, "the body accelerates along the force");
        RIGIDBODIES_EXPECT(body->angular_velocity_rad_s() > 0.0, "the body also begins to turn");
    }

    RIGIDBODIES_TEST("a free body rotates about its centre of mass rather than its frame origin")
    {
        World world;
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        world.set_settings(settings);

        BodyDefinition definition;
        definition.name = "offset_plate";
        definition.angular_velocity_rad_s = 1.0;

        Collider collider;
        collider.shape = make_box(0.2, 0.2);
        collider.material = materials::aluminium();
        // Placing the only collider away from the body origin puts the centre of mass off the
        // origin as well, which is the case that distinguishes the two conventions.
        collider.local_transform = rigidbodies::math::Transform2::from_angle({ 0.5, 0.0 }, 0.0);
        definition.colliders.push_back(collider);

        const auto id = world.create_body(definition);
        const auto* body = world.find_body(id);
        const auto initial_center = body->world_center_of_mass_m();

        for (int index = 0; index < 120; ++index)
        {
            world.step(1.0 / 240.0);
        }

        const auto moved = rigidbodies::math::distance(body->world_center_of_mass_m(), initial_center);
        RIGIDBODIES_EXPECT_NEAR(moved, 0.0, 1.0e-9, "a spinning body with no linear velocity keeps its centre of mass still");
        RIGIDBODIES_EXPECT(rigidbodies::math::length(body->position_m()) > 1.0e-3, "the frame origin does move, because it orbits that centre");
    }

    RIGIDBODIES_TEST("drag opposes motion and never reverses it")
    {
        World world;
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        world.set_settings(settings);
        world.add_force_generator(std::make_shared<AerodynamicDrag>());

        auto definition = falling_ball(0.05, materials::expanded_polystyrene());
        definition.linear_velocity_m_s = { 20.0, 0.0 };
        const auto id = world.create_body(definition);
        const auto* body = world.find_body(id);

        Real previous_speed = rigidbodies::math::length(body->linear_velocity_m_s());
        for (int index = 0; index < 600; ++index)
        {
            world.step(1.0 / 600.0);
            const auto speed = rigidbodies::math::length(body->linear_velocity_m_s());
            RIGIDBODIES_EXPECT(speed <= previous_speed + 1.0e-12, "speed never increases under drag alone");
            RIGIDBODIES_EXPECT(body->linear_velocity_m_s().x > 0.0, "drag slows the body without reversing it");
            previous_speed = speed;
        }

        RIGIDBODIES_EXPECT(previous_speed < 20.0, "the body has measurably slowed");
    }

    RIGIDBODIES_TEST("body identifiers become invalid once the body is destroyed")
    {
        World world;
        const auto id = world.create_body(falling_ball());
        RIGIDBODIES_EXPECT(world.is_valid(id), "a freshly created identifier is valid");
        RIGIDBODIES_EXPECT(world.destroy_body(id), "the body is destroyed");
        RIGIDBODIES_EXPECT(!world.is_valid(id), "the identifier does not survive destruction");
        RIGIDBODIES_EXPECT(world.find_body(id) == nullptr, "a stale identifier resolves to nothing");

        // The freed slot is reused, and the generation counter is what keeps the old identifier
        // from addressing the new occupant.
        const auto replacement = world.create_body(falling_ball());
        RIGIDBODIES_EXPECT(replacement.index == id.index, "the slot is reused");
        RIGIDBODIES_EXPECT(!world.is_valid(id), "the stale identifier still resolves to nothing");
        RIGIDBODIES_EXPECT(world.is_valid(replacement), "the new identifier is valid");
    }

    RIGIDBODIES_TEST("the broad phase reports overlapping pairs and skips separated ones")
    {
        World world;
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        world.set_settings(settings);

        auto near_left = falling_ball(0.1);
        near_left.position_m = { 0.0, 0.0 };
        auto near_right = falling_ball(0.1);
        near_right.position_m = { 0.15, 0.0 };
        auto far_away = falling_ball(0.1);
        far_away.position_m = { 10.0, 0.0 };

        world.create_body(near_left);
        world.create_body(near_right);
        world.create_body(far_away);
        world.step(1.0 / 240.0);

        RIGIDBODIES_EXPECT(world.broad_phase_pairs().size() == 1, "only the overlapping pair is reported");
    }

    RIGIDBODIES_TEST("two immovable bodies are never paired")
    {
        World world;
        auto first = falling_ball(0.1);
        first.type = BodyType::static_body;
        auto second = falling_ball(0.1);
        second.type = BodyType::static_body;
        second.position_m = { 0.05, 0.0 };

        world.create_body(first);
        world.create_body(second);
        world.step(1.0 / 240.0);

        RIGIDBODIES_EXPECT(world.broad_phase_pairs().empty(), "a static pair cannot resolve, so it is not reported");
    }

    RIGIDBODIES_TEST("collision filters exclude pairs that share a negative group")
    {
        CollisionFilter first;
        CollisionFilter second;
        RIGIDBODIES_EXPECT(should_collide(first, second), "default filters collide");

        first.group = -1;
        second.group = -1;
        RIGIDBODIES_EXPECT(!should_collide(first, second), "a shared negative group suppresses the pair");

        first.group = 0;
        second.group = 0;
        first.mask = 0;
        RIGIDBODIES_EXPECT(!should_collide(first, second), "an empty mask accepts nothing");
    }

    RIGIDBODIES_TEST("material mixing keeps kinetic friction at or below static friction")
    {
        const auto combined = combine_materials(materials::rubber(), materials::glass());
        RIGIDBODIES_EXPECT(combined.kinetic_friction <= combined.static_friction, "sliding never resists more than resting");
        RIGIDBODIES_EXPECT(combined.restitution <= 1.0, "restitution stays within the physical range");
        RIGIDBODIES_EXPECT(combined.restitution >= 0.0, "restitution stays within the physical range");
    }

    RIGIDBODIES_TEST("the fixed stepper converts frame time into whole steps")
    {
        TimeStepper stepper { 1.0 / 100.0 };
        RIGIDBODIES_EXPECT(stepper.advance(0.005) == 0, "less than one step produces none");
        RIGIDBODIES_EXPECT(stepper.advance(0.006) == 1, "the leftover carries into the next call");
        RIGIDBODIES_EXPECT(stepper.advance(0.025) == 2, "a longer frame produces several steps");
    }

    RIGIDBODIES_TEST("a long stall cannot make the stepper fall permanently behind")
    {
        TimeStepper stepper { 1.0 / 100.0 };
        stepper.set_maximum_frame_time(0.25);
        RIGIDBODIES_EXPECT(stepper.advance(10.0) == 25, "the frame time is clamped before it is accumulated");
    }

    RIGIDBODIES_TEST("pausing halts the simulation and a single step still advances it")
    {
        TimeStepper stepper { 1.0 / 100.0 };
        stepper.set_paused(true);
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "a paused stepper produces nothing");

        stepper.request_single_step();
        RIGIDBODIES_EXPECT(stepper.advance(0.0) == 1, "a requested step is taken even while paused");
        RIGIDBODIES_EXPECT(stepper.advance(1.0) == 0, "the request is consumed once");
    }

    RIGIDBODIES_TEST("time scale slows the simulation without changing the step size")
    {
        TimeStepper stepper { 1.0 / 100.0 };
        stepper.set_time_scale(0.5);
        RIGIDBODIES_EXPECT(stepper.advance(0.02) == 1, "half speed yields half as many steps");
        RIGIDBODIES_EXPECT_NEAR(stepper.fixed_step_s(), 1.0 / 100.0, 1.0e-12, "the step size itself is unchanged");
    }

    RIGIDBODIES_TEST("every catalogued scenario loads and populates a world")
    {
        for (const auto& description : available_scenarios())
        {
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, description.id), "the scenario loads");
            RIGIDBODIES_EXPECT(world.body_ids().size() > 0, "the scenario creates bodies");

            world.step(1.0 / 240.0);
            RIGIDBODIES_EXPECT(world.statistics().body_count > 0, "statistics report the loaded bodies");
        }
    }

    RIGIDBODIES_TEST("an unknown scenario identifier leaves the world untouched")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, default_scenario_id()), "the default scenario loads");
        const auto before = world.body_ids().size();

        RIGIDBODIES_EXPECT(!load_scenario(world, "not_a_scenario"), "an unknown identifier is rejected");
        RIGIDBODIES_EXPECT(world.body_ids().size() == before, "the world is unchanged");
    }

    RIGIDBODIES_TEST("total energy is conserved for a body in free fall")
    {
        World world;
        world.set_potential_energy_reference_height(-100.0);
        const auto id = world.create_body(falling_ball());
        RIGIDBODIES_EXPECT(world.is_valid(id), "the body exists");

        world.step(1.0 / 1000.0);
        const auto initial = world.statistics().total_kinetic_energy_j + world.statistics().total_potential_energy_j;

        for (int index = 0; index < 500; ++index)
        {
            world.step(1.0 / 1000.0);
        }

        const auto final_total = world.statistics().total_kinetic_energy_j + world.statistics().total_potential_energy_j;
        // Semi-implicit Euler does not conserve energy exactly, but the drift over half a second
        // stays far below a tenth of a percent at this step size.
        RIGIDBODIES_EXPECT_NEAR(final_total, initial, std::abs(initial) * 1.0e-3, "energy is conserved to within integration error");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
