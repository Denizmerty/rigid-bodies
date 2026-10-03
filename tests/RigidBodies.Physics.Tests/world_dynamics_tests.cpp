#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition disc(BodyType type = BodyType::dynamic_body, Vec2 position = {})
    {
        BodyDefinition definition;
        definition.type = type;
        definition.position_m = position;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        return definition;
    }

    WorldSettings quiet_settings()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.quiet_duration_s = 0.05;
        return settings;
    }

    void settle(World& world)
    {
        for (int step = 0; step < 20; ++step)
        {
            world.step(0.01);
        }
    }

    bool reported(const World& world, BodyId id, MotionLimitKind kind)
    {
        return std::any_of(world.motion_limit_events().begin(), world.motion_limit_events().end(), [id, kind](const MotionLimitEvent& event)
            {
                return event.body == id && event.kind == kind;
            });
    }

    void expect_finite(const RigidBody& body)
    {
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(body.position_m()), "body position is finite after recovery");
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(body.orientation_rad()), "body orientation is finite after recovery");
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(body.linear_velocity_m_s()), "body velocity is finite after recovery");
        RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(body.angular_velocity_rad_s()), "body spin is finite after recovery");
    }

    class Link final : public Constraint
    {
    public:
        Link(BodyId first, BodyId second) : first_(first), second_(second)
        {
        }
        std::shared_ptr<Constraint> clone() const override
        {
            return std::make_shared<Link>(*this);
        }
        std::string_view name() const override
        {
            return "test_link";
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
        }
        void solve_velocity(World&, Real) override
        {
        }
        bool solve_position(World&, Real) override
        {
            return true;
        }

    private:
        BodyId first_;
        BodyId second_;
    };

    class SupportingContact final : public NarrowPhase
    {
    public:
        std::shared_ptr<NarrowPhase> clone() const override
        {
            return std::make_shared<SupportingContact>(*this);
        }
        std::string_view name() const override
        {
            return "test_supporting_contact";
        }
        bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const override
        {
            manifold.first = query.first;
            manifold.second = query.second;
            manifold.first_collider = query.first_collider;
            manifold.second_collider = query.second_collider;
            manifold.normal = { 0.0, 1.0 };
            manifold.is_sensor = query.is_sensor;
            manifold.point_count = 1;
            manifold.points[0].normal_impulse_n_s = 1.0;
            return true;
        }
    };

    class SupportingSolver final : public ContactSolver
    {
    public:
        std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<SupportingSolver>(*this);
        }
        std::string_view name() const override
        {
            return "test_supporting_solver";
        }
        void prepare(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
        }
        void solve_velocity(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.for_each_body([](BodyId, RigidBody& body)
                {
                    if (body.type() == BodyType::dynamic_body)
                    {
                        // Solvers commonly use public setters and impulse methods. These internal
                        // corrections must not look like a fresh external wake request every pass.
                        body.set_linear_velocity({});
                        body.set_angular_velocity(0.0);
                    }
                });
        }
        void solve_position(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.for_each_body([](BodyId, RigidBody& body)
                {
                    if (body.type() == BodyType::dynamic_body)
                    {
                        body.set_position(body.position_m());
                    }
                });
        }
    };

    class InvalidOutputSolver final : public ContactSolver
    {
    public:
        std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<InvalidOutputSolver>(*this);
        }
        std::string_view name() const override
        {
            return "test_invalid_solver";
        }
        void prepare(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
        }
        void solve_velocity(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.for_each_body([](BodyId, RigidBody& body)
                {
                    body.set_simulated_velocity({ std::numeric_limits<Real>::infinity(), 0.0 }, std::numeric_limits<Real>::quiet_NaN());
                });
        }
        void solve_position(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.for_each_body([](BodyId, RigidBody& body)
                {
                    body.set_simulated_pose({ std::numeric_limits<Real>::quiet_NaN(), 0.0 }, std::numeric_limits<Real>::infinity());
                });
        }
    };

    class TransformCorrectionSolver final : public ContactSolver
    {
    public:
        std::shared_ptr<ContactSolver> clone() const override
        {
            return std::make_shared<TransformCorrectionSolver>(*this);
        }
        std::string_view name() const override
        {
            return "test_transform_correction";
        }
        void prepare(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
        }
        void solve_velocity(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
        }
        void solve_position(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.for_each_body([](BodyId, RigidBody& body)
                {
                    body.set_position(body.position_m() + Vec2 { 0.1, 0.0 });
                    body.set_orientation(body.orientation_rad() + 0.2);
                });
        }
    };

    RIGIDBODIES_TEST("a quiet unsupported body sleeps while a falling body stays awake")
    {
        World quiet { quiet_settings() };
        const auto quiet_id = quiet.create_body(disc());
        quiet.step(0.01);
        RIGIDBODIES_EXPECT(quiet.find_body(quiet_id)->is_awake(), "sleep waits for the quiet interval");
        settle(quiet);
        RIGIDBODIES_EXPECT(!quiet.find_body(quiet_id)->is_awake(), "a stationary unloaded body can sleep without contact");
        RIGIDBODIES_EXPECT(quiet.statistics().sleeping_body_count == 1, "statistics include sleeping dynamic bodies");
        const auto position = quiet.find_body(quiet_id)->position_m();
        settle(quiet);
        RIGIDBODIES_EXPECT(quiet.find_body(quiet_id)->position_m() == position, "a sleeping body remains stationary");

        auto falling_settings = quiet_settings();
        falling_settings.gravity_m_s2 = { 0.0, -standard_gravity_m_s2 };
        // Deliberately put speed below the sleep threshold throughout the run. Acceleration,
        // rather than velocity alone, must keep unsupported falling bodies awake.
        falling_settings.sleep.linear_speed_m_s = 100.0;
        World falling { falling_settings };
        const auto falling_id = falling.create_body(disc());
        settle(falling);
        RIGIDBODIES_EXPECT(falling.find_body(falling_id)->is_awake(), "unsupported acceleration prevents sleep");
        RIGIDBODIES_EXPECT(falling.find_body(falling_id)->linear_velocity_m_s().y < -1.0, "gravity continues to accelerate the falling body");
    }

    RIGIDBODIES_TEST("world and body sleep opt-outs keep quiet dynamics active")
    {
        World world { quiet_settings() };
        auto definition = disc();
        definition.sleep_enabled = false;
        const auto id = world.create_body(definition);
        settle(world);
        RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "a body's sleep opt-out is respected");
        world.find_body(id)->set_sleep_enabled(true);
        settle(world);
        RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake(), "enabling sleep allows the quiet body to rest");
        auto settings = world.settings();
        settings.sleep.enabled = false;
        world.set_settings(settings);
        settle(world);
        RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "disabling world sleep wakes and keeps existing bodies active");
    }

    RIGIDBODIES_TEST("new fixed-rotation and static bodies enforce their initial velocity restrictions")
    {
        auto fixed_definition = disc();
        fixed_definition.fixed_rotation = true;
        fixed_definition.linear_velocity_m_s = { 1.0, 2.0 };
        fixed_definition.angular_velocity_rad_s = 5.0;
        const RigidBody fixed { fixed_definition };
        RIGIDBODIES_EXPECT(fixed.linear_velocity_m_s() == fixed_definition.linear_velocity_m_s && fixed.angular_velocity_rad_s() == 0.0,
            "fixed rotation suppresses initial spin before any integrator stage reads it");
        fixed_definition.type = BodyType::static_body;
        const RigidBody stationary { fixed_definition };
        RIGIDBODIES_EXPECT(stationary.linear_velocity_m_s() == Vec2 {} && stationary.angular_velocity_rad_s() == 0.0,
            "a standalone static body starts with zero motion");
    }

    RIGIDBODIES_TEST("external impulses forces edits and field changes wake sleeping dynamics")
    {
        World world { quiet_settings() };
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        settle(world);
        body->apply_linear_impulse({ 0.1, 0.0 });
        RIGIDBODIES_EXPECT(body->is_awake(), "a direct impulse wakes immediately");
        body->set_awake(false);
        body->apply_force_at_center({ 1.0, 0.0 });
        RIGIDBODIES_EXPECT(body->is_awake(), "a direct force wakes immediately");
        body->clear_accumulators();
        body->set_awake(false);
        body->set_position({ 1.0, 0.0 });
        RIGIDBODIES_EXPECT(body->is_awake(), "editing position wakes immediately");
        body->set_awake(false);
        auto changed = world.settings();
        changed.gravity_m_s2 = { 0.0, -1.0 };
        world.set_settings(changed);
        world.step(0.01);
        RIGIDBODIES_EXPECT(body->is_awake() && body->linear_velocity_m_s().y < 0.0, "changing a field wakes and accelerates a sleeping body");
    }

    RIGIDBODIES_TEST("solver corrections allow a supported body to finish its quiet interval")
    {
        auto settings = quiet_settings();
        settings.gravity_m_s2 = { 0.0, -standard_gravity_m_s2 };
        World world { settings };
        world.create_body(disc(BodyType::static_body, { 0.0, -0.15 }), "floor");
        const auto id = world.create_body(disc(), "subject");
        world.set_narrow_phase(std::make_shared<SupportingContact>());
        world.set_contact_solver(std::make_shared<SupportingSolver>());
        settle(world);
        RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake(), "persistent support and internal solver updates do not restart the quiet timer");
    }

    RIGIDBODIES_TEST("touching bodies in free fall and sensor contacts do not provide sleep support")
    {
        auto settings = quiet_settings();
        settings.gravity_m_s2 = { 0.0, -standard_gravity_m_s2 };
        settings.sleep.linear_speed_m_s = 100.0;
        World falling { settings };
        const auto first = falling.create_body(disc(), "first");
        const auto second = falling.create_body(disc(BodyType::dynamic_body, { 0.0, 0.15 }), "second");
        falling.set_narrow_phase(std::make_shared<SupportingContact>());
        settle(falling);
        RIGIDBODIES_EXPECT(falling.find_body(first)->is_awake() && falling.find_body(second)->is_awake(),
            "an unanchored contact pair cannot suspend both bodies under gravity");

        World sensors { settings };
        auto floor = disc(BodyType::static_body, { 0.0, -0.15 });
        floor.colliders[0].is_sensor = true;
        sensors.create_body(floor, "floor");
        const auto subject = sensors.create_body(disc(), "subject");
        sensors.set_narrow_phase(std::make_shared<SupportingContact>());
        sensors.set_contact_solver(std::make_shared<SupportingSolver>());
        settle(sensors);
        RIGIDBODIES_EXPECT(sensors.find_body(subject)->is_awake(), "sensor manifolds cannot support a loaded body even if a solver zeros its velocity");
    }

    RIGIDBODIES_TEST("public transform setters used by a position solver preserve interpolation endpoints")
    {
        World world { quiet_settings() };
        const auto id = world.create_body(disc());
        world.set_contact_solver(std::make_shared<TransformCorrectionSolver>());
        world.step(0.01);
        const auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(body->previous_transform().translation.x, 0.0, 0.0, "solver corrections retain the start-of-step position");
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.5).translation.x, 0.5 * body->position_m().x, 1.0e-14, "interpolation includes the complete solver position correction");
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.5).rotation.angle(), 0.5 * body->orientation_rad(), 1.0e-14, "interpolation includes the complete solver angle correction");
    }

    RIGIDBODIES_TEST("moving a static collider next to a sleeper wakes it")
    {
        World world { quiet_settings() };
        const auto sleeper = world.create_body(disc());
        const auto obstacle = world.create_body(disc(BodyType::static_body, { 2.0, 0.0 }));
        settle(world);
        RIGIDBODIES_EXPECT(!world.find_body(sleeper)->is_awake(), "the distant obstacle does not prevent sleep");
        world.find_body(obstacle)->set_position({ 0.15, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(sleeper)->is_awake(), "editing a nearby static collider wakes a sleeper");
    }

    RIGIDBODIES_TEST("moving a static support away wakes bodies that depended on its old contact")
    {
        auto settings = quiet_settings();
        settings.gravity_m_s2 = { 0.0, -standard_gravity_m_s2 };
        World world { settings };
        const auto floor = world.create_body(disc(BodyType::static_body, { 0.0, -0.15 }), "floor");
        const auto subject = world.create_body(disc(), "subject");
        world.set_narrow_phase(std::make_shared<SupportingContact>());
        world.set_contact_solver(std::make_shared<SupportingSolver>());
        settle(world);
        RIGIDBODIES_EXPECT(!world.find_body(subject)->is_awake(), "the supported body starts asleep");
        world.set_contact_solver(std::make_shared<NullContactSolver>());
        world.find_body(floor)->set_position({ 10.0, -0.15 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(subject)->is_awake() && world.find_body(subject)->linear_velocity_m_s().y < 0.0,
            "losing the previous contact wakes the body and gravity resumes");
    }

    RIGIDBODIES_TEST("destroying a solid support wakes its old neighbors while sensor removal does not")
    {
        for (int sensor = 0; sensor < 2; ++sensor)
        {
            auto settings = quiet_settings();
            if (sensor == 0)
                settings.gravity_m_s2 = { 0.0, -standard_gravity_m_s2 };
            World world { settings };
            auto definition = disc(BodyType::static_body, { 0.0, -0.15 });
            definition.colliders[0].is_sensor = sensor != 0;
            const auto floor = world.create_body(definition, "floor");
            const auto subject = world.create_body(disc(), "subject");
            world.set_narrow_phase(std::make_shared<SupportingContact>());
            if (sensor == 0)
                world.set_contact_solver(std::make_shared<SupportingSolver>());
            else
                world.set_contact_solver(std::make_shared<NullContactSolver>());
            settle(world);
            RIGIDBODIES_EXPECT(!world.find_body(subject)->is_awake(), "the subject begins asleep");
            world.destroy_body(floor);
            RIGIDBODIES_EXPECT(world.find_body(subject)->is_awake() == (sensor == 0),
                "only removal of an actual solid contact wakes its neighbor");
            // Verify the removal's own wake before changing response models, which also wakes
            // bodies. The sensor case already uses the null model and needs no transition.
            if (sensor == 0)
                world.set_contact_solver(std::make_shared<NullContactSolver>());
            world.step(0.01);
            if (sensor == 0)
            {
                RIGIDBODIES_EXPECT(world.find_body(subject)->linear_velocity_m_s().y < 0.0, "gravity resumes after its support is destroyed");
            }
            else
                RIGIDBODIES_EXPECT(!world.find_body(subject)->is_awake(), "sensor removal also preserves sleep through the next step");
        }
    }

    RIGIDBODIES_TEST("swept kinematic neighbors wake sleepers while sensors and excluded filters do not")
    {
        for (int mode = 0; mode < 3; ++mode)
        {
            World world { quiet_settings() };
            const auto sleeper = world.create_body(disc());
            auto definition = disc(BodyType::kinematic_body, { -0.3, 0.0 });
            definition.colliders[0].is_sensor = mode == 1;
            if (mode == 2)
            {
                definition.colliders[0].filter.mask = 0;
            }
            const auto mover = world.create_body(definition);
            settle(world);
            RIGIDBODIES_EXPECT(!world.find_body(sleeper)->is_awake(), "a distant stationary neighbor permits sleep");
            LinearMotion path;
            path.origin_m = { -0.3, 0.0 };
            path.velocity_m_s = { 60.0, 0.0 };
            RIGIDBODIES_EXPECT(world.set_kinematic_motion(mover, KinematicMotion { path }), "the moving path attaches");
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(sleeper)->is_awake() == (mode == 0),
                "only a nonsensor collision-eligible swept neighbor wakes the sleeper");
        }
    }

    RIGIDBODIES_TEST("wake requests propagate through enabled constraint chains only")
    {
        World world { quiet_settings() };
        const auto first = world.create_body(disc(BodyType::dynamic_body, { -3.0, 0.0 }));
        const auto second = world.create_body(disc());
        const auto third = world.create_body(disc(BodyType::dynamic_body, { 3.0, 0.0 }));
        const auto excluded = world.create_body(disc(BodyType::dynamic_body, { 6.0, 0.0 }));
        world.add_constraint(std::make_shared<Link>(first, second));
        world.add_constraint(std::make_shared<Link>(second, third));
        auto disabled = std::make_shared<Link>(third, excluded);
        disabled->set_enabled(false);
        world.add_constraint(disabled);
        settle(world);
        RIGIDBODIES_EXPECT(world.statistics().sleeping_body_count == 4, "all quiet linked bodies can initially sleep");
        world.find_body(first)->apply_linear_impulse({ 0.1, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(first)->is_awake() && world.find_body(second)->is_awake() && world.find_body(third)->is_awake(),
            "waking one end reaches the full enabled constraint chain");
        RIGIDBODIES_EXPECT(!world.find_body(excluded)->is_awake(), "a disabled constraint does not propagate wake requests");
    }

    RIGIDBODIES_TEST("waking covers curved paths and periodic paths starting at a turning point")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_linear_speed_m_s = 300.0;
        World circular { settings };
        const auto sleeper = circular.create_body(disc(BodyType::dynamic_body, { -0.3, 0.0 }));
        const auto mover = circular.create_body(disc(BodyType::kinematic_body, { 0.3, 0.0 }));
        settle(circular);
        CircularMotion orbit;
        orbit.radius_m = 0.3;
        orbit.angular_speed_rad_s = rigidbodies::math::two_pi / 0.01;
        orbit.orient_to_path = false;
        RIGIDBODIES_EXPECT(circular.set_kinematic_motion(mover, KinematicMotion { orbit }), "the circular path attaches");
        circular.step(0.01);
        RIGIDBODIES_EXPECT(circular.find_body(sleeper)->is_awake(), "a full revolution wakes bodies on its arc even when both path endpoints coincide");

        World harmonic { quiet_settings() };
        const auto subject = harmonic.create_body(disc());
        const auto piston = harmonic.create_body(disc(BodyType::kinematic_body, { 0.3, 0.0 }));
        HarmonicMotion oscillation;
        oscillation.translation_amplitude_m = { 0.3, 0.0 };
        oscillation.phase_rad = rigidbodies::math::half_pi;
        oscillation.frequency_hz = 50.0;
        RIGIDBODIES_EXPECT(harmonic.set_kinematic_motion(piston, KinematicMotion { oscillation }), "the harmonic path attaches");
        harmonic.step(0.02); // Complete a period so attachment's external-edit flag has cleared.
        harmonic.find_body(subject)->set_awake(false);
        harmonic.step(0.01);
        RIGIDBODIES_EXPECT(harmonic.find_body(subject)->is_awake(), "a zero-velocity turning point still has prescribed motion during the next step");
    }

    RIGIDBODIES_TEST("wake sweeps cover motion across the full bounded world")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_position_m = 1.0;
        settings.sleep.quiet_duration_s = 0.5;
        World world { settings };
        const auto sleeper = world.create_body(disc(BodyType::dynamic_body, { 0.9, 0.0 }));
        const auto mover = world.create_body(disc(BodyType::kinematic_body, { -0.9, 0.0 }));
        world.find_body(sleeper)->set_awake(false);
        world.find_body(mover)->set_linear_velocity({ 20.0, 0.0 });
        world.step(0.1);
        RIGIDBODIES_EXPECT(world.find_body(sleeper)->is_awake(), "a bounded sweep spans both sides of the origin rather than only one world radius");
    }

    RIGIDBODIES_TEST("world paths use attachment time and preserve exact centre-of-mass samples for offset bodies")
    {
        World world { quiet_settings() };
        world.step(0.25);
        auto definition = disc(BodyType::kinematic_body);
        definition.colliders[0].local_transform = rigidbodies::math::Transform2::from_angle({ 0.3, -0.2 }, 0.0);
        const auto id = world.create_body(definition);
        HarmonicMotion path;
        path.origin_m = { 2.0, 3.0 };
        path.translation_amplitude_m = { 0.4, 0.7 };
        path.rotation_amplitude_rad = 0.5;
        path.frequency_hz = 0.5;
        path.phase_rad = 0.25;
        const KinematicMotion motion { path };
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(id, motion), "kinematic bodies accept paths");
        world.step(0.1);
        const auto expected = motion.sample(0.1);
        const auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, expected.position_m.x, 1.0e-13, "path translation refers to the centre of mass");
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().y, expected.position_m.y, 1.0e-13, "local mass offset is removed from the body origin");
        RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), expected.orientation_rad, 1.0e-13, "world time before attachment does not advance the path");
        RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, expected.linear_velocity_m_s.x, 1.0e-13, "the world retains the analytic path derivative");
        RIGIDBODIES_EXPECT_NEAR(body->angular_velocity_rad_s(), expected.angular_velocity_rad_s, 1.0e-13, "the world retains the analytic angular derivative");
        const auto checkpoint = world.snapshot();
        world.step(0.1);
        const auto next_position = world.find_body(id)->position_m();
        const auto next_velocity = world.find_body(id)->linear_velocity_m_s();
        world.restore(checkpoint);
        world.step(0.1);
        RIGIDBODIES_EXPECT(world.find_body(id)->position_m() == next_position && world.find_body(id)->linear_velocity_m_s() == next_velocity,
            "snapshot replay restores the path and its relative time exactly");
    }

    RIGIDBODIES_TEST("clearing a path retains velocity and stale identifiers cannot attach to replacement bodies")
    {
        World world { quiet_settings() };
        const auto dynamic = world.create_body(disc());
        const auto id = world.create_body(disc(BodyType::kinematic_body));
        LinearMotion path;
        path.velocity_m_s = { 1.0, 0.0 };
        const KinematicMotion motion { path };
        RIGIDBODIES_EXPECT(!world.set_kinematic_motion(dynamic, motion), "dynamic bodies cannot be driven by a kinematic path");
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(id, motion), "the path attaches to its kinematic body");
        world.step(0.1);
        const auto position = world.find_body(id)->position_m();
        RIGIDBODIES_EXPECT(world.clear_kinematic_motion(id), "the path can be detached");
        RIGIDBODIES_EXPECT(world.kinematic_motion(id) == nullptr, "clearing removes the path");
        world.step(0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->position_m().x, position.x + 0.1, 1.0e-13, "free kinematic motion continues with the last prescribed velocity");
        world.destroy_body(id);
        const auto replacement = world.create_body(disc(BodyType::kinematic_body));
        RIGIDBODIES_EXPECT(replacement.index == id.index, "the stale-id case reuses the original slot");
        RIGIDBODIES_EXPECT(!world.set_kinematic_motion(id, motion) && !world.clear_kinematic_motion(id), "stale identifiers cannot alter replacement paths");
        RIGIDBODIES_EXPECT(world.kinematic_motion(replacement) == nullptr, "the replacement inherits no previous motion");
    }

    RIGIDBODIES_TEST("an invalid initial drive is rejected without replacing the existing path or pose")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_linear_speed_m_s = 5.0;
        settings.limits.maximum_angular_speed_rad_s = 5.0;
        settings.limits.maximum_orientation_rad = 5.0;
        World world { settings };
        const auto id = world.create_body(disc(BodyType::kinematic_body));
        LinearMotion valid;
        valid.origin_m = { 1.0, 2.0 };
        valid.velocity_m_s = { 1.0, 0.0 };
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(id, KinematicMotion { valid }), "the original drive is valid");
        const auto position = world.find_body(id)->position_m();
        LinearMotion invalid;
        invalid.origin_m = { 20.0, 30.0 };
        invalid.velocity_m_s = { 1000.0, 0.0 };
        invalid.initial_orientation_rad = 6.0;
        invalid.angular_velocity_rad_s = 6.0;
        RIGIDBODIES_EXPECT(!world.set_kinematic_motion(id, KinematicMotion { invalid }), "a drive exceeding speed and angular bounds is rejected");
        RIGIDBODIES_EXPECT(world.find_body(id)->position_m() == position && world.find_body(id)->linear_velocity_m_s() == valid.velocity_m_s,
            "rejection changes neither placement nor velocity");
        RIGIDBODIES_EXPECT(world.kinematic_motion(id) && world.kinematic_motion(id)->sample(0.0).position_m == valid.origin_m,
            "the prior drive remains installed");
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::kinematic_path) && reported(world, id, MotionLimitKind::linear_velocity) &&
                reported(world, id, MotionLimitKind::orientation) && reported(world, id, MotionLimitKind::angular_velocity),
            "the path and every rejected quantity are identified");
    }

    RIGIDBODIES_TEST("a drive stops at its last valid pose when a later position or derivative exceeds limits")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_position_m = 0.5;
        World linear { settings };
        const auto linear_id = linear.create_body(disc(BodyType::kinematic_body));
        LinearMotion path;
        path.velocity_m_s = { 1.0, 0.0 };
        RIGIDBODIES_EXPECT(linear.set_kinematic_motion(linear_id, KinematicMotion { path }), "the initially bounded path attaches");
        linear.step(0.25);
        const auto previous = linear.find_body(linear_id)->position_m();
        linear.step(0.5);
        RIGIDBODIES_EXPECT(linear.kinematic_motion(linear_id) == nullptr, "the path is detached once its next sample is out of range");
        RIGIDBODIES_EXPECT(linear.find_body(linear_id)->position_m() == previous && linear.find_body(linear_id)->linear_velocity_m_s() == Vec2 {},
            "an invalid sample stops the drive without teleporting to the boundary");
        RIGIDBODIES_EXPECT(reported(linear, linear_id, MotionLimitKind::position) && reported(linear, linear_id, MotionLimitKind::kinematic_path),
            "future position rejection is reported");

        settings.limits.maximum_position_m = 100.0;
        settings.limits.maximum_linear_speed_m_s = 5.0;
        World harmonic { settings };
        const auto harmonic_id = harmonic.create_body(disc(BodyType::kinematic_body));
        HarmonicMotion oscillator;
        oscillator.translation_amplitude_m = { 1.0, 0.0 };
        oscillator.phase_rad = rigidbodies::math::half_pi;
        RIGIDBODIES_EXPECT(harmonic.set_kinematic_motion(harmonic_id, KinematicMotion { oscillator }), "a zero-speed turning point initially fits the limit");
        const auto initial = harmonic.find_body(harmonic_id)->position_m();
        harmonic.step(0.25);
        RIGIDBODIES_EXPECT(harmonic.kinematic_motion(harmonic_id) == nullptr && harmonic.find_body(harmonic_id)->position_m() == initial,
            "later excessive analytic speed rejects the whole sample instead of clipping only its reported velocity");
        RIGIDBODIES_EXPECT(reported(harmonic, harmonic_id, MotionLimitKind::linear_velocity) && reported(harmonic, harmonic_id, MotionLimitKind::kinematic_path),
            "the derivative limit explains why the drive stopped");
    }

    RIGIDBODIES_TEST("path bounds validate the reconstructed body frame of an offset centre of mass")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_position_m = 0.5;
        World world { settings };
        auto definition = disc(BodyType::kinematic_body);
        definition.colliders[0].local_transform = rigidbodies::math::Transform2::from_angle({ 1.0, 0.0 }, 0.0);
        const auto id = world.create_body(definition);
        RIGIDBODIES_EXPECT(!world.set_kinematic_motion(id, KinematicMotion {}),
            "a valid centre-of-mass coordinate cannot hide an out-of-bounds reconstructed frame origin");
        RIGIDBODIES_EXPECT(world.find_body(id)->position_m() == Vec2 {} && world.kinematic_motion(id) == nullptr,
            "the rejected attachment preserves the body's existing frame");
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::position) && reported(world, id, MotionLimitKind::kinematic_path),
            "offset-frame rejection is visible in diagnostics");
    }

    RIGIDBODIES_TEST("finite runaway poses and velocities are bounded and reported")
    {
        auto settings = quiet_settings();
        settings.limits.maximum_position_m = 2.0;
        settings.limits.maximum_linear_speed_m_s = 3.0;
        settings.limits.maximum_angular_speed_rad_s = 4.0;
        settings.limits.maximum_orientation_rad = 5.0;
        World world { settings };
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->set_position({ 1000.0, -1000.0 });
        body->set_orientation(100.0);
        body->set_linear_velocity({ 300.0, 400.0 });
        body->set_angular_velocity(100.0);
        world.step(0.01);
        expect_finite(*body);
        RIGIDBODIES_EXPECT(std::abs(body->position_m().x) <= 2.0 && std::abs(body->position_m().y) <= 2.0, "both position components stay in the configured world range");
        RIGIDBODIES_EXPECT(rigidbodies::math::length(body->linear_velocity_m_s()) <= 3.0 + 1.0e-12, "linear speed stays under its bound");
        RIGIDBODIES_EXPECT(std::abs(body->angular_velocity_rad_s()) <= 4.0, "angular speed stays under its bound");
        RIGIDBODIES_EXPECT(std::abs(body->orientation_rad()) <= 5.0, "unwrapped orientation stays under its bound");
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::position) && reported(world, id, MotionLimitKind::linear_velocity), "translation recovery is visible in diagnostics");
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::orientation) && reported(world, id, MotionLimitKind::angular_velocity), "rotation recovery is visible in diagnostics");
        RIGIDBODIES_EXPECT(world.statistics().last_step_limit_event_count == world.motion_limit_events().size(), "the per-step counter matches reported events");
    }

    RIGIDBODIES_TEST("nonfinite body state and applied loads recover before contaminating statistics")
    {
        World world { quiet_settings() };
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->set_simulated_pose({ std::numeric_limits<Real>::quiet_NaN(), 0.0 }, std::numeric_limits<Real>::infinity());
        body->set_simulated_velocity({ std::numeric_limits<Real>::infinity(), 0.0 }, std::numeric_limits<Real>::quiet_NaN());
        body->apply_force_at_center({ std::numeric_limits<Real>::infinity(), 0.0 });
        body->apply_torque(std::numeric_limits<Real>::quiet_NaN());
        world.step(0.01);
        expect_finite(*body);
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::non_finite_load), "nonfinite loads produce an explicit report");
        RIGIDBODIES_EXPECT(std::isfinite(world.statistics().total_kinetic_energy_j) && std::isfinite(world.statistics().total_potential_energy_j),
            "recovered state does not poison energy statistics");
    }

    RIGIDBODIES_TEST("invalid custom solver output is recovered and snapshot diagnostics remain reproducible")
    {
        World world { quiet_settings() };
        const auto id = world.create_body(disc());
        world.set_contact_solver(std::make_shared<InvalidOutputSolver>());
        world.step(0.01);
        expect_finite(*world.find_body(id));
        RIGIDBODIES_EXPECT(reported(world, id, MotionLimitKind::linear_velocity) && reported(world, id, MotionLimitKind::position),
            "invalid solver velocities and poses are both reported");
        const auto count = world.statistics().limit_event_count;
        const auto events = world.motion_limit_events();
        RIGIDBODIES_EXPECT(count > 0 && !events.empty(), "the diagnostic history records the recovery");
        const auto checkpoint = world.snapshot();
        world.set_contact_solver(std::make_shared<NullContactSolver>());
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.motion_limit_events().empty() && world.statistics().last_step_limit_event_count == 0,
            "a clean step clears only the current-step report");
        RIGIDBODIES_EXPECT(world.statistics().limit_event_count == count, "the cumulative report survives a clean step");
        world.restore(checkpoint);
        RIGIDBODIES_EXPECT(world.statistics().limit_event_count == count && world.motion_limit_events().size() == events.size(),
            "snapshot restores both cumulative and per-step reports");
        for (std::size_t index = 0; index < events.size(); ++index)
        {
            RIGIDBODIES_EXPECT(world.motion_limit_events()[index].body == events[index].body && world.motion_limit_events()[index].kind == events[index].kind,
                "restored diagnostic details preserve their body and reason");
        }
    }

    RIGIDBODIES_TEST("snapshots preserve sleeping state and quiet-time progress")
    {
        World world { quiet_settings() };
        const auto id = world.create_body(disc());
        world.step(0.01);
        world.step(0.01);
        const auto quiet_time = world.find_body(id)->quiet_time_s();
        RIGIDBODIES_EXPECT(quiet_time > 0.0, "the body has begun its quiet interval");
        const auto partial = world.snapshot();
        settle(world);
        const auto sleeping = world.snapshot();
        world.find_body(id)->apply_linear_impulse({ 1.0, 0.0 });
        world.restore(sleeping);
        RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake() && world.statistics().sleeping_body_count == 1, "a sleeping checkpoint restores as sleeping");
        world.restore(partial);
        RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "an earlier awake checkpoint restores as awake");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->quiet_time_s(), quiet_time, 0.0, "partial quiet time restores exactly");
        settle(world);
        RIGIDBODIES_EXPECT(!world.find_body(id)->is_awake(), "restored quiet progress can complete normally");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
