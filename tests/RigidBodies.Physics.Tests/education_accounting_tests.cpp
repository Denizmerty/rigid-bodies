#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    WorldSettings settings()
    {
        WorldSettings result;
        result.gravity_m_s2 = {};
        result.sleep.enabled = false;
        result.solver.restitution_threshold_m_s = 0.0;
        result.collision.continuous = false;
        return result;
    }

    BodyId body(World& world, BodyType type, Vec2 position, Vec2 velocity = {}, Real mass = 1.0, Real restitution = 0.0, Real friction = 0.0, bool fixed = false, bool sensor = false)
    {
        BodyDefinition definition;
        definition.type = type;
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        definition.fixed_rotation = fixed;
        definition.name = "Test body";
        Collider collider;
        collider.is_sensor = sensor;
        collider.shape = make_circle(0.5);
        collider.material.restitution = restitution;
        collider.material.static_friction = friction;
        collider.material.kinetic_friction = friction;
        collider.material.rolling_friction_m = 0.0;
        collider.material.spinning_friction_m = 0.0;
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition);
        world.find_body(id)->override_mass(mass);
        return id;
    }

    RIGIDBODIES_TEST("body energy gives stored springs to the moving bodies and sums to the world")
    {
        auto environment = settings();
        environment.gravity_m_s2 = { 3.0, -4.0 };
        World world(environment);
        world.set_potential_energy_reference_height(1.0);
        const auto anchor = body(world, BodyType::static_body, {});
        const auto moving = body(world, BodyType::dynamic_body, { 2.0, 3.0 }, { 3.0, 4.0 }, 2.0);
        world.find_body(moving)->set_angular_velocity(2.0);
        LinearSpringDefinition spring;
        spring.first = anchor;
        spring.second = moving;
        spring.rest_length_m = 1.0;
        spring.stiffness_n_m = 10.0;
        const auto spring_id = world.create_spring(spring);
        const auto a = *measure_body_energy(world, anchor);
        const auto b = *measure_body_energy(world, moving);
        const auto total = measure_world_energy(world);
        RIGIDBODIES_EXPECT_NEAR(b.translational_kinetic_j, 25.0, 1.0e-12, "translation uses the COM speed");
        RIGIDBODIES_EXPECT_NEAR(b.rotational_kinetic_j, 0.5, 1.0e-12, "rotation uses inertia about the COM");
        // Height is read at the middle of the step, half a step back along the velocity.
        const auto lag = world.energy_measurement_lag_s();
        RIGIDBODIES_EXPECT_NEAR(lag, 0.5 / 120.0, 1.0e-15, "semi-implicit Euler readings trail the position by half the conventional step");
        RIGIDBODIES_EXPECT_NEAR(b.gravitational_potential_j, 4.0 + 2.0 * lag * (3.0 * 3.0 - 4.0 * 4.0), 1.0e-12, "gravity projects along the tilted field from the reference");
        // Stored energy is read the same way, less the lag times the rate it is being stored at.
        const auto report = *world.spring_report(spring_id);
        const auto stored = report.potential_energy_j - lag * spring.stiffness_n_m * report.extension_m * report.axial_speed_m_s;
        RIGIDBODIES_EXPECT(std::abs(report.axial_speed_m_s) > 1.0, "the spring is stretching, so the reading differs from k x^2 / 2");
        RIGIDBODIES_EXPECT_NEAR(a.spring_potential_j, 0.0, 0.0, "a static anchor stores no energy of its own");
        RIGIDBODIES_EXPECT_NEAR(b.spring_potential_j, stored, 1.0e-12, "the moving end owns all the energy of a spring tied to a fixed anchor");
        RIGIDBODIES_EXPECT_NEAR(a.mechanical_j() + b.mechanical_j(), total.mechanical_j(), 1.0e-12, "per-body sums count each spring once");
        RIGIDBODIES_EXPECT(!measure_body_energy(world, {}), "invalid identifiers have no reading");

        // Between two moving bodies the stored energy is shared, still counted once in all.
        const auto partner = body(world, BodyType::dynamic_body, { -2.0, 1.0 }, { 0.5, 0.0 }, 1.0);
        LinearSpringDefinition coupling;
        coupling.first = moving;
        coupling.second = partner;
        coupling.rest_length_m = 2.0;
        coupling.stiffness_n_m = 6.0;
        const auto coupling_id = world.create_spring(coupling);
        const auto coupling_report = *world.spring_report(coupling_id);
        const auto coupling_stored = coupling_report.potential_energy_j - lag * coupling.stiffness_n_m * coupling_report.extension_m * coupling_report.axial_speed_m_s;
        RIGIDBODIES_EXPECT(coupling_stored > 0.1, "the coupling spring is stretched");
        const auto shared = *measure_body_energy(world, partner);
        RIGIDBODIES_EXPECT_NEAR(shared.spring_potential_j, coupling_stored * 0.5, 1.0e-12, "two moving ends share a spring's energy equally");
        const auto both = measure_body_energy(world, anchor)->mechanical_j() + measure_body_energy(world, moving)->mechanical_j() + shared.mechanical_j();
        RIGIDBODIES_EXPECT_NEAR(both, measure_world_energy(world).mechanical_j(), 1.0e-12, "every spring is still counted once across all bodies");
    }

    RIGIDBODIES_TEST("elastic partial and inelastic collisions report analytic momentum and normal energy loss")
    {
        for (const auto restitution : { 0.0, 0.5, 1.0 })
        {
            World world(settings());
            const auto first = body(world, BodyType::dynamic_body, { -0.5, 0.0 }, { 3.0, 0.0 }, 2.0, restitution);
            const auto second = body(world, BodyType::dynamic_body, { 0.5, 0.0 }, {}, 1.0, restitution);
            world.step(0.01);
            const auto expected_loss = 3.0 * (1.0 - restitution * restitution);
            RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), expected_loss, 1.0e-12, "normal loss is half reduced mass times relative speed squared times one minus e squared");
            RIGIDBODIES_EXPECT_NEAR(world.collision_energy().friction_loss_j(), 0.0, 1.0e-12, "frictionless collision has no friction loss");
            RIGIDBODIES_EXPECT(world.impact_reports().size() == 1, "one actual pair impact is reported");
            const auto& report = world.impact_reports().front();
            RIGIDBODIES_EXPECT(report.first_before.id == first && report.second_before.id == second, "report retains both body identifiers");
            RIGIDBODIES_EXPECT_NEAR(report.first_before.velocity_m_s.x, 3.0, 0.0, "incoming state is captured before warm starting");
            RIGIDBODIES_EXPECT_NEAR(report.first_after.velocity_m_s.x, 2.0 - restitution, 1.0e-12, "heavy body's outgoing speed matches the analytic solution");
            RIGIDBODIES_EXPECT_NEAR(report.second_after.velocity_m_s.x, 2.0 * (1.0 + restitution), 1.0e-12, "light body's outgoing speed matches the analytic solution");
            RIGIDBODIES_EXPECT_NEAR(report.impulse_on_second_n_s.x, 2.0 * (1.0 + restitution), 1.0e-12, "impulse equals the momentum change");
            RIGIDBODIES_EXPECT_NEAR(report.first_after.momentum_kg_m_s.x + report.second_after.momentum_kg_m_s.x, 6.0, 1.0e-12, "system momentum is conserved");
        }
    }

    RIGIDBODIES_TEST("gravity potential follows enabled global and per-body registrations")
    {
        auto environment = settings();
        environment.gravity_m_s2 = { 0.0, -10.0 };
        World world(environment);
        const auto moving = body(world, BodyType::dynamic_body, { 0.0, 3.0 }, {}, 2.0);
        const auto global = world.force_generators().front();
        RIGIDBODIES_EXPECT_NEAR(measure_body_energy(world, moving)->gravitational_potential_j, 60.0, 0.0, "one active field gives mgh");
        global->set_enabled(false);
        world.notify_body_properties_changed(moving);
        RIGIDBODIES_EXPECT_NEAR(measure_body_energy(world, moving)->gravitational_potential_j, 0.0, 0.0, "turning gravity off removes its potential from the energy ledger");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_potential_energy_j, 0.0, 0.0, "legacy statistics use the same effective field");
        const auto local = std::make_shared<UniformGravity>();
        world.add_force_generator(moving, local);
        world.add_force_generator(moving, local);
        RIGIDBODIES_EXPECT_NEAR(measure_world_energy(world).gravitational_potential_j, 120.0, 0.0, "duplicate local registrations are additive like their forces");
        global->set_enabled(true);
        world.find_body(moving)->set_gravity_scale(0.5);
        world.notify_body_properties_changed(moving);
        RIGIDBODIES_EXPECT_NEAR(measure_body_energy(world, moving)->gravitational_potential_j, 90.0, 0.0, "per-body scale multiplies every enabled gravity registration");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_potential_energy_j, 90.0, 0.0, "world and body readings agree immediately after notification");
        local->set_enabled(false);
        RIGIDBODIES_EXPECT_NEAR(measure_world_energy(world).gravitational_potential_j, 30.0, 0.0, "disabling an attached generator removes all its registrations' potential");
    }

    RIGIDBODIES_TEST("tangent friction and normal restitution losses are independently measured")
    {
        World world(settings());
        body(world, BodyType::static_body, { 0.0, -0.5 }, {}, 1.0, 0.0, 0.25);
        const auto moving = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 2.0, -1.0 }, 1.0, 0.0, 0.25, true);
        const auto before = world.find_body(moving)->kinetic_energy_j();
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 0.5, 1.0e-12, "normal collision removes vertical kinetic energy");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().friction_loss_j(), 0.46875, 1.0e-12, "kinetic friction removes the measured horizontal energy");
        RIGIDBODIES_EXPECT_NEAR(before - world.find_body(moving)->kinetic_energy_j(), world.collision_energy().restitution_loss_j() + world.collision_energy().friction_loss_j(), 1.0e-12, "the channels sum to the exact kinetic change");
    }

    RIGIDBODIES_TEST("warm start credits are signed and solver iterations do not accumulate fictitious loss")
    {
        for (const auto iterations : { 1, 32 })
        {
            auto environment = settings();
            environment.solver.velocity_iterations = iterations;
            World world(environment);
            body(world, BodyType::static_body, { 0.0, -0.5 });
            const auto moving = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -2.0 });
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 2.0, 1.0e-12, "the first stopping collision loses two joules");
            for (int index = 0; index < 6; ++index)
                world.step(index % 2 ? 0.005 : 0.02);
            RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 2.0, 1.0e-12, "warm-start and correction cancel even when timestep scales the cached impulse");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->kinetic_energy_j(), 0.0, 1.0e-12, "resting body gains no energy");
            RIGIDBODIES_EXPECT(world.impact_reports().empty(), "resting support is not repeatedly reported as a new collision");
        }
    }

    RIGIDBODIES_TEST("driven surface external work is separated from friction dissipation")
    {
        World world(settings());
        body(world, BodyType::kinematic_body, { 0.0, -0.5 }, { 2.0, 0.0 }, 1.0, 0.0, 0.5);
        const auto moving = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -1.0 }, 1.0, 0.0, 0.5, true);
        world.step(0.01);
        const auto& accounting = world.collision_energy();
        RIGIDBODIES_EXPECT_NEAR(accounting.friction_external_work_j, 1.0, 1.0e-12, "the motor sustaining surface speed supplies impulse times surface velocity");
        RIGIDBODIES_EXPECT_NEAR(accounting.friction_energy_change_j, 0.125, 1.0e-12, "the dynamic body gains horizontal kinetic energy");
        RIGIDBODIES_EXPECT_NEAR(accounting.friction_loss_j(), 0.875, 1.0e-12, "supplied work minus gain is dissipated at the sliding contact");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->linear_velocity_m_s().x, 0.5, 1.0e-12, "friction accelerates the body towards the moving surface");
    }

    RIGIDBODIES_TEST("initial resting support under gravity is not an impact but slow genuine approach is")
    {
        for (const auto incoming_speed : { 0.0, 1.0e-5, 1.0 })
        {
            auto environment = settings();
            environment.gravity_m_s2 = { 0.0, -10.0 };
            environment.solver.restitution_threshold_m_s = 1.0;
            World world(environment);
            body(world, BodyType::static_body, { 0.0, -0.5 });
            body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -incoming_speed });
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.impact_reports().size() == (incoming_speed > 0.0 ? 1U : 0U), "only approach already present before support force integration triggers inspection");
            if (incoming_speed > 0.0)
                RIGIDBODIES_EXPECT(world.collision_energy().restitution_loss_j() > 0.0, "suppressing a support explanation never hides its actual impulse work");
            else
                RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 0.0, 1.0e-12, "a support that only cancels this step's weight removes no energy");
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.impact_reports().empty(), "continuing gravity support stays quiet on later steps");
        }
    }

    RIGIDBODIES_TEST("sensors and a null solver contribute no impact or collision energy")
    {
        for (const auto sensor : { true, false })
        {
            World world(settings());
            body(world, BodyType::static_body, { -0.5, 0.0 }, {}, 1.0, 0.0, 0.0, false, sensor);
            body(world, BodyType::dynamic_body, { 0.5, 0.0 }, { -2.0, 0.0 });
            if (!sensor)
                world.set_contact_solver(std::make_shared<NullContactSolver>());
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.impact_reports().empty(), "an overlap without an applied response is not an impact");
            RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 0.0, 0.0, "no solver impulse means no measured loss");
        }
    }

    RIGIDBODIES_TEST("angular friction includes spin work and reported torque impulses")
    {
        World world(settings());
        const auto first = body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto second = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -1.0 });
        world.find_body(second)->set_angular_velocity(4.0);
        ContactManifold manifold;
        manifold.first = first;
        manifold.second = second;
        manifold.normal = { 0.0, 1.0 };
        manifold.point_count = 1;
        manifold.material.rolling_friction_m = 0.2;
        manifold.material.spinning_friction_m = 0.1;
        std::vector<ContactManifold> contacts { manifold };
        const auto before = world.find_body(second)->kinetic_energy_j();
        SequentialImpulseContactSolver solver;
        solver.prepare(world, contacts, settings().solver, 0.01);
        for (int index = 0; index < 8; ++index)
            solver.solve_velocity(world, contacts, settings().solver, 0.01);
        const auto& result = contacts.front();
        RIGIDBODIES_EXPECT_NEAR(result.energy.friction_loss_j(), 0.84, 1.0e-12, "bounded angular resistance removes the expected spin energy");
        RIGIDBODIES_EXPECT_NEAR(result.applied_angular_impulse_second_n_m_s, -0.3, 1.0e-12, "rolling and spinning angular impulse are both reported");
        RIGIDBODIES_EXPECT_NEAR(before - world.find_body(second)->kinetic_energy_j(), result.energy.restitution_loss_j() + result.energy.friction_loss_j(), 1.0e-12, "normal and angular work telescope to the actual kinetic loss");
    }

    RIGIDBODIES_TEST("snapshot restore replays collision balances and captured impact state")
    {
        World world(settings());
        body(world, BodyType::static_body, { -0.5, 0.0 });
        body(world, BodyType::dynamic_body, { 0.5, 0.0 }, { -2.0, 0.0 });
        const auto initial = world.snapshot();
        world.step(0.01);
        const auto impact = world.snapshot();
        const auto loss = world.collision_energy().restitution_loss_j();
        world.step(0.01);
        world.restore(impact);
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), loss, 0.0, "restoring retains accumulated measured work");
        RIGIDBODIES_EXPECT(world.impact_reports().size() == 1, "the snapshot retains the last impact explanation");
        world.restore(initial);
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), loss, 0.0, "replaying from initial state reproduces work exactly");
        world.clear();
        RIGIDBODIES_EXPECT(world.impact_reports().empty(), "clear removes captured reports");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 0.0, 0.0, "clear resets the accounting baseline");
    }

    RIGIDBODIES_TEST("CCD restitution accounts speculative loss and later returned energy once")
    {
        auto environment = settings();
        environment.collision.continuous = true;
        World world(environment);
        body(world, BodyType::static_body, { 0.0, 0.0 }, {}, 1.0, 1.0);
        const auto moving = body(world, BodyType::dynamic_body, { -2.0, 0.0 }, { 30.0, 0.0 }, 1.0, 1.0);
        const auto initial_energy = world.find_body(moving)->kinetic_energy_j();
        bool reported = false;
        for (int step = 0; step < 12; ++step)
        {
            world.step(0.05);
            for (const auto& report : world.impact_reports())
            {
                RIGIDBODIES_EXPECT(!reported, "the speculative and touching phases make one impact episode");
                reported = true;
                RIGIDBODIES_EXPECT(report.involved_speculation, "the report explains that CCD contributed pre-contact impulses");
                RIGIDBODIES_EXPECT_NEAR(report.second_before.velocity_m_s.x, 30.0, 1.0e-10, "incoming state precedes the first speculative speed cap");
            }
        }
        RIGIDBODIES_EXPECT(reported, "a fast collision eventually emits an actual touching impact");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->kinetic_energy_j(), initial_energy, 1.0e-6, "elastic CCD returns the saved impact energy");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 0.0, 1.0e-6, "temporary speed-cap loss is credited on restitution");
    }

    RIGIDBODIES_TEST("body property notifications clear stale response and keep other world state")
    {
        World world(settings());
        body(world, BodyType::static_body, { 0.0, -0.5 });
        const auto moving = body(world, BodyType::dynamic_body, { 0.0, 0.5 }, { 0.0, -2.0 });
        world.step(0.01);
        const auto loss = world.collision_energy().restitution_loss_j();
        world.find_body(moving)->override_mass(2.0);
        world.find_body(moving)->set_linear_velocity({ 0.0, 3.0 });
        RIGIDBODIES_EXPECT(world.notify_body_properties_changed(moving), "a live body edit is accepted");
        RIGIDBODIES_EXPECT(world.manifolds().empty(), "the old mass's cached impulses are removed");
        RIGIDBODIES_EXPECT_NEAR(world.statistics().total_kinetic_energy_j, 9.0, 1.0e-12, "paused property edits refresh statistics immediately");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), loss, 0.0, "editing a body does not rewrite earlier losses");
        RIGIDBODIES_EXPECT(!world.notify_body_properties_changed({}), "stale handles are rejected");
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->linear_velocity_m_s().y, 3.0, 1.0e-12, "old warm-start impulses cannot alter the edited velocity");
    }

    RIGIDBODIES_TEST("snapshot retains an unfinished speculative impact episode")
    {
        auto environment = settings();
        environment.collision.continuous = true;
        World world(environment);
        body(world, BodyType::static_body, {}, {}, 1.0, 0.5);
        const auto moving = body(world, BodyType::dynamic_body, { -2.0, 0.0 }, { 30.0, 0.0 }, 1.0, 0.5);
        world.step(0.05);
        RIGIDBODIES_EXPECT(world.impact_reports().empty(), "the initial speed cap is not yet a touching impact");
        const auto saved = world.snapshot();
        world.step(0.05);
        RIGIDBODIES_EXPECT(world.impact_reports().size() == 1, "the touching response completes the saved episode");
        const auto expected = world.impact_reports().front();
        world.restore(saved);
        world.step(0.05);
        const auto& replay = world.impact_reports().front();
        RIGIDBODIES_EXPECT(replay.spans_multiple_steps, "the explanation marks a multi-step CCD episode");
        RIGIDBODIES_EXPECT_NEAR(replay.second_before.velocity_m_s.x, 30.0, 0.0, "restore keeps the original uncapped incoming velocity");
        RIGIDBODIES_EXPECT_NEAR(replay.energy.restitution_loss_j(), expected.energy.restitution_loss_j(), 0.0, "replay includes precisely the same pre-contact work");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->linear_velocity_m_s().x, -15.0, 1.0e-10, "partial restitution retains its original incoming speed");
    }

    RIGIDBODIES_TEST("compound collider contacts produce one body pair explanation without counting work twice")
    {
        World world(settings());
        BodyDefinition ground;
        ground.type = BodyType::static_body;
        ground.position_m = { 0.0, -0.25 };
        Collider surface;
        surface.shape = make_box(4.0, 0.5);
        surface.material.restitution = 0.0;
        surface.material.static_friction = surface.material.kinetic_friction = 0.0;
        ground.colliders.push_back(surface);
        world.create_body(ground);
        BodyDefinition compound;
        compound.position_m = { 0.0, 0.5 };
        compound.linear_velocity_m_s = { 0.0, -2.0 };
        compound.fixed_rotation = true;
        for (const auto x : { -1.0, 1.0 })
        {
            auto collider = surface;
            collider.shape = make_circle(0.5);
            collider.local_transform.translation.x = x;
            compound.colliders.push_back(collider);
        }
        const auto id = world.create_body(compound);
        world.find_body(id)->override_mass(2.0);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.manifolds().size() == 2, "both authored collider contacts participate in response");
        RIGIDBODIES_EXPECT(world.impact_reports().size() == 1, "the inspector reports one physical body pair");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 4.0, 1.0e-12, "each manifold contributes its actual incremental work once");
        RIGIDBODIES_EXPECT_NEAR(world.impact_reports().front().impulse_on_second_n_s.y, 4.0, 1.0e-12, "all collider impulses are aggregated into the body momentum change");
    }

    RIGIDBODIES_TEST("impact retention is bounded while every collision still contributes energy")
    {
        auto environment = settings();
        environment.limits.maximum_position_m = 1000.0;
        World world(environment);
        for (std::size_t index = 0; index < maximum_impact_reports_per_step + 3; ++index)
        {
            const auto x = static_cast<Real>(index) * 3.0;
            body(world, BodyType::static_body, { x, -0.5 });
            body(world, BodyType::dynamic_body, { x, 0.5 }, { 0.0, -2.0 });
        }
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.impact_reports().size() == maximum_impact_reports_per_step, "only the bounded canonical prefix is retained");
        RIGIDBODIES_EXPECT(world.dropped_impact_report_count() == 3, "the overflow is explicitly reported");
        RIGIDBODIES_EXPECT_NEAR(world.collision_energy().restitution_loss_j(), 2.0 * static_cast<Real>(maximum_impact_reports_per_step + 3), 1.0e-10, "report limits never discard energy accounting");
    }

    RIGIDBODIES_TEST("property edits invalidate joint warm starts without changing or repairing the link")
    {
        for (const auto break_link : { false, true })
        {
            auto environment = settings();
            environment.gravity_m_s2 = { 0.0, -10.0 };
            World world(environment);
            const auto anchor = body(world, BodyType::static_body, {});
            const auto moving = body(world, BodyType::dynamic_body, { 0.0, 2.0 });
            DistanceJointDefinition definition;
            definition.first = anchor;
            definition.second = moving;
            definition.length_m = 2.0;
            if (break_link)
                definition.break_force_n = 1.0;
            auto joint = std::make_shared<JointConstraint>(definition);
            world.add_constraint(joint, "preserved_link");
            world.step(0.01);
            RIGIDBODIES_EXPECT(joint->is_broken() == break_link, "the requested weak link breaks under gravity");
            const auto prior_break_force = joint->broken_force_n();
            world.find_body(moving)->override_mass(2.0);
            world.notify_body_properties_changed(moving);
            RIGIDBODIES_EXPECT(joint->velocity_rows() == nullptr, "property edits discard every old warm-start row");
            RIGIDBODIES_EXPECT(joint->is_broken() == break_link, "invalidating a cache never repairs a broken link");
            RIGIDBODIES_EXPECT(world.constraint_by_key("preserved_link") == joint, "registration and component identity survive editing");
            RIGIDBODIES_EXPECT_NEAR(std::get<DistanceJointDefinition>(joint->definition()).length_m, 2.0, 0.0, "the relationship's definition stays unchanged");
            if (break_link)
                RIGIDBODIES_EXPECT_NEAR(joint->broken_force_n(), prior_break_force, 0.0, "the load that broke the link remains available");
            environment.gravity_m_s2 = {};
            environment.solver.velocity_iterations = 0;
            world.set_settings(environment);
            const auto velocity_before_prepare = world.find_body(moving)->linear_velocity_m_s();
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(moving)->linear_velocity_m_s().y, velocity_before_prepare.y, 0.0, "preparation cannot inject a stale impulse after the mass edit");
        }
    }

    constexpr Real scenario_step_s = 1.0 / 120.0;

    // Mechanical energy now, with what the budget says left and entered the moving bodies.
    struct BudgetReading
    {
        Real mechanical_j {}, lost_j {}, added_j {};
    };

    BudgetReading read_budget(const World& world)
    {
        const auto budget = measure_energy_budget(world);
        return { measure_world_energy(world).mechanical_j(), budget.lost_j(), budget.added_j() };
    }

    RIGIDBODIES_TEST("mechanical energy plus every loss stays at the starting energy plus every addition")
    {
        for (const auto* id : { "free_fall", "collision_comparison", "restitution_drop", "spring_damping", "ramp", "friction_comparison", "stable_stack", "magnus_effect", "distance_chain", "revolute_drive", "prismatic_drive", "targeted_force" })
        {
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, id), "bundled scenario loads");
            const auto start_j = read_budget(world).mechanical_j;
            auto scale_j = std::abs(start_j);
            Real worst_j = 0.0;
            for (int step = 1; step <= 480; ++step)
            {
                world.step(scenario_step_s);
                const auto reading = read_budget(world);
                scale_j = std::max({ scale_j, std::abs(reading.mechanical_j), measure_world_energy(world).kinetic_j(), std::abs(reading.lost_j), std::abs(reading.added_j) });
                worst_j = std::max(worst_j, std::abs(reading.mechanical_j + reading.lost_j - reading.added_j - start_j));
                worst_j = std::max(worst_j, std::abs(measure_energy_budget(world).start_energy_j - start_j));
            }
            RIGIDBODIES_EXPECT(worst_j <= 2.0e-4 * scale_j + 1.0e-9, std::string(id) + ": every joule is either still mechanical or booked to an interaction");
            RIGIDBODIES_EXPECT(std::abs(world.energy_ledger().unattributed_j) <= 2.0e-4 * scale_j + 1.0e-9, std::string(id) + ": nothing significant is left unexplained");
        }
    }

    RIGIDBODIES_TEST("free flight keeps mechanical energy level until the first contact")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "restitution_drop"), "bundled scenario loads");
        const auto start_j = measure_world_energy(world).mechanical_j();
        int flight_steps = 0;
        for (; flight_steps < 600; ++flight_steps)
        {
            world.step(scenario_step_s);
            if (!world.manifolds().empty())
                break;
            RIGIDBODIES_EXPECT_NEAR(measure_world_energy(world).mechanical_j(), start_j, 1.0e-9 * std::max(1.0, std::abs(start_j)), "gravity alone neither adds nor removes mechanical energy");
        }
        RIGIDBODIES_EXPECT(flight_steps > 20, "the ball falls freely for a while before touching");
    }

    RIGIDBODIES_TEST("a block resting on a ramp books almost no loss and none once it has settled")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "ramp"), "bundled scenario loads");
        for (int step = 0; step < 120; ++step)
            world.step(scenario_step_s);
        const auto settled = measure_energy_budget(world);
        for (int step = 0; step < 480; ++step)
            world.step(scenario_step_s);
        const auto later = measure_energy_budget(world);
        RIGIDBODIES_EXPECT(later.lost_in_impacts_j + later.lost_to_friction_j < 0.05, "settling into the ramp costs a few hundredths of a joule, not a phantom impact loss");
        RIGIDBODIES_EXPECT_NEAR(later.lost_j(), settled.lost_j(), 1.0e-3, "a resting block keeps its budget");
    }

    RIGIDBODIES_TEST("listed impacts account for the scene's loss in impacts")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "restitution_drop"), "bundled scenario loads");
        Real listed_j = 0.0;
        for (int step = 0; step < 480; ++step)
        {
            world.step(scenario_step_s);
            for (const auto& report : world.impact_reports())
                listed_j += report.energy.restitution_loss_j();
        }
        const auto total_j = world.collision_energy().restitution_loss_j();
        RIGIDBODIES_EXPECT(total_j > 1.0, "the bouncing ball loses energy in its impacts");
        RIGIDBODIES_EXPECT_NEAR(listed_j, total_j, 0.02 * total_j, "resting contact adds almost nothing beyond the listed impacts");
    }

    RIGIDBODIES_TEST("dampers and air drag book the energy they remove")
    {
        World springs;
        RIGIDBODIES_EXPECT(load_scenario(springs, "spring_damping"), "bundled scenario loads");
        const auto spring_start_j = measure_world_energy(springs).mechanical_j();
        for (int step = 0; step < 480; ++step)
            springs.step(scenario_step_s);
        const auto spring_budget = measure_energy_budget(springs);
        RIGIDBODIES_EXPECT(spring_budget.lost_in_dampers_j > 0.25 * spring_start_j, "the damped spring removes a large share of the energy");
        RIGIDBODIES_EXPECT_NEAR(spring_budget.lost_in_impacts_j + spring_budget.lost_to_friction_j + spring_budget.lost_to_air_j, 0.0, 1.0e-9, "nothing else removes energy from the springs");

        World air;
        RIGIDBODIES_EXPECT(load_scenario(air, "magnus_effect"), "bundled scenario loads");
        for (int step = 0; step < 240; ++step)
            air.step(scenario_step_s);
        RIGIDBODIES_EXPECT(measure_energy_budget(air).lost_to_air_j > 0.01, "drag on a spinning ball removes energy");
    }

    RIGIDBODIES_TEST("a mass on a spring from a fixed anchor owns the oscillator's energy")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "spring_damping"), "bundled scenario loads");
        struct Oscillator
        {
            BodyId mass;
            bool damped {};
            Real start_j {}, previous_j {}, highest_j {}, lowest_j {};
        };
        std::vector<Oscillator> oscillators;
        for (const auto id : world.spring_ids())
            if (const auto* spring = std::get_if<LinearSpringDefinition>(world.spring_definition(id)))
                for (const auto end : { spring->first, spring->second })
                    if (world.find_body(end)->type() == BodyType::dynamic_body)
                    {
                        const auto start_j = measure_body_energy(world, end)->mechanical_j();
                        oscillators.push_back({ end, spring->damping_n_s_m > 0.0, start_j, start_j, start_j, start_j });
                    }
        RIGIDBODIES_EXPECT(oscillators.size() == 2, "the scene hangs a damped and an undamped mass from fixed anchors");
        for (const auto& oscillator : oscillators)
            RIGIDBODIES_EXPECT(oscillator.start_j > 1.0, "the stretched spring's whole energy belongs to its mass, not half of it");
        for (int step = 0; step < 240; ++step)
        {
            world.step(scenario_step_s);
            for (auto& oscillator : oscillators)
            {
                const auto now_j = measure_body_energy(world, oscillator.mass)->mechanical_j();
                if (oscillator.damped)
                    RIGIDBODIES_EXPECT(now_j <= oscillator.previous_j + 1.0e-6 * oscillator.start_j, "a damped oscillator's energy never rises");
                oscillator.previous_j = now_j;
                oscillator.highest_j = std::max(oscillator.highest_j, now_j);
                oscillator.lowest_j = std::min(oscillator.lowest_j, now_j);
            }
        }
        for (const auto& oscillator : oscillators)
            if (oscillator.damped)
                RIGIDBODIES_EXPECT(oscillator.previous_j < 0.5 * oscillator.start_j, "the damper removes most of the energy");
            else
                RIGIDBODIES_EXPECT(oscillator.highest_j - oscillator.lowest_j <= 1.0e-3 * oscillator.start_j, "an undamped oscillator keeps a level energy");
    }

    RIGIDBODIES_TEST("a motor's work stays booked when a travel stop takes the energy back")
    {
        for (const auto* id : { "prismatic_drive", "revolute_drive" })
        {
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, id), "bundled scenario loads");
            Real added_before_stop_j = 0.0;
            for (int step = 0; step < 600; ++step)
            {
                world.step(scenario_step_s);
                const auto budget = measure_energy_budget(world);
                if (budget.lost_in_joints_j < 0.01)
                    added_before_stop_j = budget.added_by_drives_j;
            }
            const auto budget = measure_energy_budget(world);
            RIGIDBODIES_EXPECT(budget.has_drives && budget.has_joints, std::string(id) + ": the scene has a motor on a joint with stops");
            RIGIDBODIES_EXPECT(added_before_stop_j > 0.05, std::string(id) + ": the motor puts energy in on its way to the stop");
            RIGIDBODIES_EXPECT(budget.added_by_drives_j >= 0.95 * added_before_stop_j, std::string(id) + ": the motor's work stays booked once the stop has caught the motion");
            RIGIDBODIES_EXPECT(budget.lost_in_joints_j > 0.05, std::string(id) + ": the stop's absorption is booked as a loss in the joint");
            RIGIDBODIES_EXPECT(measure_world_energy(world).kinetic_j() < 1.0e-3, std::string(id) + ": the driven part rests against its stop");
        }
        World slider;
        RIGIDBODIES_EXPECT(load_scenario(slider, "prismatic_drive"), "bundled scenario loads");
        for (int step = 0; step < 600; ++step)
            slider.step(scenario_step_s);
        const auto budget = measure_energy_budget(slider);
        // On a level rail the carriage's height never changes: the stop takes back exactly the
        // kinetic energy the motor gave it.
        RIGIDBODIES_EXPECT_NEAR(budget.lost_in_joints_j, budget.added_by_drives_j, 0.05 * budget.added_by_drives_j, "the stop absorbs what the motor added on a level rail");
    }

    RIGIDBODIES_TEST("whole-scene momentum is read from the bodies before the first step")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "fast_projectile"), "bundled scenario loads");
        Vec2 expected;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                if (body.type() == BodyType::dynamic_body)
                    expected += body.linear_velocity_m_s() * body.mass_properties().mass_kg;
            });
        RIGIDBODIES_EXPECT(std::abs(expected.x) > 10.0, "the projectile starts moving fast");
        const auto measured = measure_world_momentum_kg_m_s(world);
        RIGIDBODIES_EXPECT_NEAR(measured.x, expected.x, 1.0e-9, "momentum x is the moving bodies' m v before anything has stepped");
        RIGIDBODIES_EXPECT_NEAR(measured.y, expected.y, 1.0e-9, "momentum y is the moving bodies' m v before anything has stepped");
    }

    RIGIDBODIES_TEST("an edit made while paused is booked at once and the starting energy stays put")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "collision_comparison"), "bundled scenario loads");
        const auto start_j = measure_world_energy(world).mechanical_j();
        RIGIDBODIES_EXPECT_NEAR(measure_energy_budget(world).start_energy_j, start_j, 0.0, "before the first step the start is the energy now");
        for (int step = 0; step < 30; ++step)
            world.step(scenario_step_s);
        BodyId moving;
        world.for_each_body([&](BodyId id, const RigidBody& body)
            {
                if (!moving.is_valid() && body.type() == BodyType::dynamic_body)
                    moving = id;
            });
        auto* body = world.find_body(moving);
        const auto before_j = measure_world_energy(world).mechanical_j();
        body->set_linear_velocity(body->linear_velocity_m_s() + Vec2 { 2.0, 0.0 });
        const auto edit_j = measure_world_energy(world).mechanical_j() - before_j;
        RIGIDBODIES_EXPECT(std::abs(edit_j) > 0.1, "the edit changes the scene's energy");
        const auto paused = measure_energy_budget(world);
        RIGIDBODIES_EXPECT_NEAR(paused.added_by_changes_j, edit_j, 1.0e-9, "the paused edit is booked as a change straight away");
        RIGIDBODIES_EXPECT_NEAR(paused.start_energy_j, start_j, 1.0e-6 * std::abs(start_j) + 1.0e-9, "the starting energy does not move with the edit");
        world.step(scenario_step_s);
        const auto resumed = measure_energy_budget(world);
        RIGIDBODIES_EXPECT_NEAR(resumed.added_by_changes_j, edit_j, 1.0e-9, "the next step books the same change once");
        RIGIDBODIES_EXPECT_NEAR(resumed.start_energy_j, start_j, 1.0e-6 * std::abs(start_j) + 1.0e-9, "the starting energy stays put after the step");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
