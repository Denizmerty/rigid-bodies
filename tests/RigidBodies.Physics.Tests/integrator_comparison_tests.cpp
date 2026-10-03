#include <rigidbodies/physics/energy_drift.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>
#include <memory>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition disc()
    {
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.5);
        definition.colliders.push_back(collider);
        return definition;
    }

    World make_world(IntegratorPtr method)
    {
        World world;
        world.force_generators().front()->set_enabled(false);
        world.set_integrator(std::move(method));
        return world;
    }

    BodyId add_body(World& world, const BodyDefinition& definition = disc())
    {
        const auto id = world.create_body(definition, "subject");
        world.find_body(id)->override_mass(1.0);
        world.find_body(id)->set_sleep_enabled(false);
        return id;
    }

    class LinearDrag final : public ForceGenerator
    {
    public:
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<LinearDrag>(*this);
        }
        std::string_view name() const override
        {
            return "linear_drag";
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center(body.linear_velocity_m_s() * -body.mass_properties().mass_kg);
        }
    };

    class TimeForce final : public ForceGenerator
    {
    public:
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<TimeForce>(*this);
        }
        std::string_view name() const override
        {
            return "time_force";
        }
        void apply(RigidBody& body, const ForceContext& context) override
        {
            body.apply_force_at_center(Vec2 { context.elapsed_time_s, 2.0 * context.elapsed_time_s } * body.mass_properties().mass_kg);
        }
    };

    class VelocityCorrection final : public ContactSolver
    {
    public:
        BodyId target;
        Vec2 position_at_prepare;
        Vec2 velocity_at_prepare;
        std::string_view name() const override
        {
            return "velocity_correction";
        }
        void prepare(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            position_at_prepare = world.find_body(target)->world_center_of_mass_m();
            velocity_at_prepare = world.find_body(target)->linear_velocity_m_s();
        }
        void solve_velocity(World& world, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
            world.find_body(target)->set_linear_velocity({ 5.0, 0.0 });
            world.find_body(target)->set_angular_velocity(2.0);
        }
        void solve_position(World&, std::vector<ContactManifold>&, const SolverSettings&, Real) override
        {
        }
    };

    Real oscillator_error(IntegratorPtr method, Real dt, int count)
    {
        auto world = make_world(std::move(method));
        auto definition = disc();
        definition.position_m = { 1.0, 0.0 };
        const auto id = add_body(world, definition);
        world.add_force_generator(id, std::make_shared<PointAttractor>(Vec2 {}, 4.0));
        for (int index = 0; index < count; ++index)
        {
            world.step(dt);
        }
        const auto time = dt * count;
        return std::hypot(world.find_body(id)->world_center_of_mass_m().x - std::cos(2.0 * time),
            world.find_body(id)->linear_velocity_m_s().x + 2.0 * std::sin(2.0 * time));
    }

    Real drag_error(IntegratorPtr method, Real dt, int count)
    {
        auto world = make_world(std::move(method));
        auto definition = disc();
        definition.linear_velocity_m_s = { 1.0, 0.0 };
        const auto id = add_body(world, definition);
        world.add_force_generator(id, std::make_shared<LinearDrag>());
        for (int index = 0; index < count; ++index)
        {
            world.step(dt);
        }
        return std::abs(world.find_body(id)->linear_velocity_m_s().x - std::exp(-dt * count));
    }

    RIGIDBODIES_TEST("staged methods integrate constant force and torque exactly around an offset center of mass")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(method);
            auto definition = disc();
            definition.position_m = { 1.0, 2.0 };
            definition.orientation_rad = 0.3;
            definition.linear_velocity_m_s = { 3.0, -1.0 };
            definition.angular_velocity_rad_s = 0.4;
            definition.colliders.front().local_transform.translation = { 0.25, 0.0 };
            const auto id = add_body(world, definition);
            auto* body = world.find_body(id);
            body->override_mass(2.0);
            const auto initial_center = body->world_center_of_mass_m();
            const auto angular_acceleration = 4.0 * body->inverse_inertia();
            body->apply_force_at_center({ 4.0, -6.0 }, "external");
            body->apply_torque(4.0, "external");
            world.step(0.2);
            const auto expected_center = initial_center + Vec2 { 3.0, -1.0 } * 0.2 + Vec2 { 2.0, -3.0 } * 0.02;
            RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, expected_center.x, 1.0e-12, "x integrates acceleration with half dt squared");
            RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().y, expected_center.y, 1.0e-12, "y integrates acceleration with half dt squared");
            RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), 0.3 + 0.4 * 0.2 + angular_acceleration * 0.02, 1.0e-12, "angular acceleration advances orientation");
            RIGIDBODIES_EXPECT_NEAR(body->angular_velocity_rad_s(), 0.4 + angular_acceleration * 0.2, 1.0e-12, "torque advances spin");
            RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 3.4, 1.0e-12, "pending direct forces remain present in every stage");
            RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 {}, "pending force clears after completed step");
            RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 4.0, -6.0 }, "stage probes do not overwrite retained live force diagnostics");
        }
    }

    RIGIDBODIES_TEST("harmonic motion converges at first second and fourth order under step halving")
    {
        const auto euler_ratio = oscillator_error(std::make_shared<SemiImplicitEulerIntegrator>(), 0.1, 10) /
            oscillator_error(std::make_shared<SemiImplicitEulerIntegrator>(), 0.05, 20);
        const auto verlet_ratio = oscillator_error(std::make_shared<VelocityVerletIntegrator>(), 0.1, 10) /
            oscillator_error(std::make_shared<VelocityVerletIntegrator>(), 0.05, 20);
        const auto rk_ratio = oscillator_error(std::make_shared<RungeKutta4Integrator>(), 0.1, 10) /
            oscillator_error(std::make_shared<RungeKutta4Integrator>(), 0.05, 20);
        RIGIDBODIES_EXPECT(euler_ratio > 1.8 && euler_ratio < 2.2, "Euler error halves with step");
        RIGIDBODIES_EXPECT(verlet_ratio > 3.8 && verlet_ratio < 4.2, "Verlet error quarters with step");
        RIGIDBODIES_EXPECT(rk_ratio > 14.0 && rk_ratio < 18.0, "RK4 error reduces sixteenfold with step");
    }

    RIGIDBODIES_TEST("staged force sampling uses trial velocities for drag")
    {
        const auto verlet_ratio = drag_error(std::make_shared<VelocityVerletIntegrator>(), 0.1, 10) /
            drag_error(std::make_shared<VelocityVerletIntegrator>(), 0.05, 20);
        const auto rk_error = drag_error(std::make_shared<RungeKutta4Integrator>(), 0.1, 10);
        const auto rk_ratio = rk_error / drag_error(std::make_shared<RungeKutta4Integrator>(), 0.05, 20);
        RIGIDBODIES_EXPECT(verlet_ratio > 3.8 && verlet_ratio < 4.3, "Verlet velocity predictor preserves second order for drag");
        RIGIDBODIES_EXPECT(rk_error < 4.0e-7 && rk_ratio > 14.0 && rk_ratio < 18.0, "RK4 evaluates velocity dependent force at every stage");
    }

    RIGIDBODIES_TEST("RK4 samples elapsed time at intermediate stages")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto id = add_body(world);
        world.add_force_generator(std::make_shared<TimeForce>());
        for (int index = 0; index < 4; ++index)
        {
            world.step(0.25);
        }
        const auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 0.5, 1.0e-12, "linear acceleration in time integrates exactly");
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, 1.0 / 6.0, 1.0e-12, "cubic trajectory integrates exactly");
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().y, 1.0 / 3.0, 1.0e-12, "both force components receive stage times");
    }

    RIGIDBODIES_TEST("RK4 damping converges to continuous linear and angular decay")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        auto definition = disc();
        definition.linear_velocity_m_s = { 2.0, 0.0 };
        definition.angular_velocity_rad_s = 0.7;
        definition.linear_damping = 0.3;
        definition.angular_damping = 0.4;
        const auto id = add_body(world, definition);
        for (int index = 0; index < 10; ++index)
        {
            world.step(0.1);
        }
        const auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 2.0 * std::exp(-0.3), 2.0e-8, "stage damping integrates continuous velocity decay");
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, 2.0 * (1.0 - std::exp(-0.3)) / 0.3, 6.0e-8, "position uses stage velocities under damping");
        RIGIDBODIES_EXPECT_NEAR(body->angular_velocity_rad_s(), 0.7 * std::exp(-0.4), 1.0e-8, "angular damping integrated at every stage");
        RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), 0.7 * (1.0 - std::exp(-0.4)) / 0.4, 3.0e-8, "orientation uses damped stage angular velocities");
    }

    RIGIDBODIES_TEST("solver velocity corrections affect this step without erasing staged force displacement")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(method);
            const auto id = add_body(world);
            const auto solver = std::make_shared<VelocityCorrection>();
            solver->target = id;
            world.set_contact_solver(solver);
            world.find_body(id)->apply_force_at_center({ 4.0, 0.0 });
            world.step(0.25);
            const auto* body = world.find_body(id);
            RIGIDBODIES_EXPECT(solver->position_at_prepare == Vec2 {}, "collision solve occurs before predicted placement is committed");
            RIGIDBODIES_EXPECT_NEAR(solver->velocity_at_prepare.x, 1.0, 1.0e-12, "solver sees force integrated velocity");
            RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, 1.125, 1.0e-12, "solver delta velocity adds to unconstrained predicted displacement");
            RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), 0.5, 1.0e-12, "solver angular correction affects current placement");
            RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 5.0, 1.0e-12, "solver corrected velocity retained");
        }
    }

    RIGIDBODIES_TEST("staged prediction handles kinematic static fixed rotation and invalid steps")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto definition = disc();
            definition.type = BodyType::kinematic_body;
            definition.linear_velocity_m_s = { 3.0, -2.0 };
            definition.angular_velocity_rad_s = 0.4;
            RigidBody kinematic { definition };
            const auto moved = method->predict_motion(kinematic, 0.5, {});
            RIGIDBODIES_EXPECT(moved.center_position_m == Vec2 { 1.5, -1.0 }, "kinematic motion needs no force probes");
            RIGIDBODIES_EXPECT_NEAR(moved.orientation_rad, 0.2, 0.0, "kinematic spin remains prescribed");
            definition.type = BodyType::static_body;
            definition.linear_velocity_m_s = {};
            definition.angular_velocity_rad_s = 0.0;
            RigidBody stationary { definition };
            RIGIDBODIES_EXPECT(method->predict_motion(stationary, 0.5, {}).center_position_m == Vec2 {}, "static pose does not change");
            auto world = make_world(method);
            definition.type = BodyType::dynamic_body;
            definition.fixed_rotation = true;
            const auto id = add_body(world, definition);
            world.find_body(id)->apply_torque(10.0);
            world.step(0.1);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->orientation_rad(), 0.0, 0.0, "fixed rotation resists staged torque");
            const auto unchanged = method->predict_motion(*world.find_body(id), std::numeric_limits<Real>::quiet_NaN(), {});
            RIGIDBODIES_EXPECT(unchanged.center_position_m == world.find_body(id)->world_center_of_mass_m(), "invalid prediction step does not sample forces or move");
        }
    }

    RIGIDBODIES_TEST("staged speed caps cannot reverse displacement from positive force and torque")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(method);
            auto settings = world.settings();
            settings.limits.maximum_position_m = 100.0;
            settings.limits.maximum_linear_speed_m_s = 100.0;
            settings.limits.maximum_angular_speed_rad_s = 100.0;
            settings.limits.maximum_orientation_rad = 200.0;
            world.set_settings(settings);
            const auto id = add_body(world);
            auto* body = world.find_body(id);
            body->apply_force_at_center({ 1000.0, 0.0 });
            body->apply_torque(1000.0);
            world.step(1.0);
            RIGIDBODIES_EXPECT_NEAR(body->position_m().x, 100.0, 1.0e-12, "positive acceleration reaches the positive position cap");
            RIGIDBODIES_EXPECT_NEAR(body->orientation_rad(), 200.0, 1.0e-12, "positive torque reaches the positive orientation cap");
            RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 100.0, 1.0e-12, "linear speed remains capped in the force direction");
            RIGIDBODIES_EXPECT_NEAR(body->angular_velocity_rad_s(), 100.0, 1.0e-12, "angular speed remains capped in the torque direction");
            for (const auto kind : { MotionLimitKind::position, MotionLimitKind::orientation, MotionLimitKind::linear_velocity, MotionLimitKind::angular_velocity })
            {
                bool reported = false;
                for (const auto& event : world.motion_limit_events())
                {
                    reported = reported || (event.body == id && event.kind == kind);
                }
                RIGIDBODIES_EXPECT(reported, "each numerical cap is reported instead of masquerading as a solver correction");
            }
        }
    }

    RIGIDBODIES_TEST("energy drift tool compares identical initial conditions and reports all sampled drift")
    {
        EnergyDriftSettings settings;
        settings.duration_s = 10.0;
        settings.time_step_s = 0.05;
        const auto reports = compare_harmonic_energy_drift(settings);
        RIGIDBODIES_EXPECT(reports.size() == 3, "all built in methods compared");
        for (const auto& report : reports)
        {
            RIGIDBODIES_EXPECT(report.step_count == 200, "each method takes the same number of steps");
            RIGIDBODIES_EXPECT_NEAR(report.elapsed_time_s, settings.duration_s, 1.0e-12, "each method runs the same duration");
            RIGIDBODIES_EXPECT_NEAR(report.initial_energy_j, 2.0, 0.0, "spring potential counted in identical initial energy");
            RIGIDBODIES_EXPECT_NEAR(report.final_drift_j, report.final_energy_j - 2.0, 0.0, "signed final drift reported");
            RIGIDBODIES_EXPECT(report.maximum_absolute_drift_j >= std::abs(report.final_drift_j), "maximum drift covers final sample");
            RIGIDBODIES_EXPECT_NEAR(report.maximum_relative_drift, report.maximum_absolute_drift_j / 2.0, 0.0, "relative drift normalized by initial energy");
        }
        RIGIDBODIES_EXPECT(reports[1].maximum_absolute_drift_j < reports[0].maximum_absolute_drift_j * 0.1, "Verlet bounds harmonic energy drift more closely than Euler at this step");
        RIGIDBODIES_EXPECT(reports[2].maximum_absolute_drift_j < reports[1].maximum_absolute_drift_j * 0.01, "RK4 produces smaller finite horizon error for this smooth oscillator");
        settings.duration_s = 0.23;
        settings.time_step_s = 0.1;
        settings.initial_displacement_m = 0.0;
        const auto zero = measure_harmonic_energy_drift(RungeKutta4Integrator {}, settings);
        RIGIDBODIES_EXPECT(zero.step_count == 3, "partial final step reaches requested duration");
        RIGIDBODIES_EXPECT_NEAR(zero.elapsed_time_s, 0.23, 1.0e-15, "partial final step does not overshoot");
        RIGIDBODIES_EXPECT_NEAR(zero.maximum_relative_drift, 0.0, 0.0, "stationary zero energy state has defined relative drift");
    }

    RIGIDBODIES_TEST("staged methods clone independently and reject the legacy split interface")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            const auto copy = method->clone();
            RIGIDBODIES_EXPECT(copy != method && copy->name() == method->name() && copy->requires_force_evaluation(), "clone keeps method without shared mutable identity");
            RigidBody body { disc() };
            bool rejected = false;
            try
            {
                method->integrate_velocity(body, 0.1);
            }
            catch (const std::logic_error&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "legacy split call cannot silently substitute a different algorithm");
        }
        EnergyDriftSettings invalid;
        invalid.time_step_s = 0.0;
        bool rejected = false;
        try
        {
            (void)compare_harmonic_energy_drift(invalid);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "invalid tool configuration fails explicitly");
    }

    RIGIDBODIES_TEST("energy comparison rejects clamped motion and excessive work instead of reporting misleading drift")
    {
        EnergyDriftSettings settings;
        settings.duration_s = 0.1;
        settings.initial_displacement_m = 150.0;
        bool initial_rejected = false;
        try
        {
            (void)measure_harmonic_energy_drift(VelocityVerletIntegrator {}, settings);
        }
        catch (const std::overflow_error&)
        {
            initial_rejected = true;
        }
        RIGIDBODIES_EXPECT(initial_rejected, "initial motion limit clamp cannot change the comparison's stated initial condition");
        settings.initial_displacement_m = 1.0;
        settings.stiffness_n_m = 100000.0;
        settings.time_step_s = 0.1;
        bool step_rejected = false;
        try
        {
            (void)measure_harmonic_energy_drift(SemiImplicitEulerIntegrator {}, settings);
        }
        catch (const std::overflow_error&)
        {
            step_rejected = true;
        }
        RIGIDBODIES_EXPECT(step_rejected, "unstable run reaching velocity limits fails instead of measuring clamped energy");
        settings = {};
        settings.time_step_s = 1.0e-9;
        bool work_rejected = false;
        try
        {
            (void)compare_harmonic_energy_drift(settings);
        }
        catch (const std::invalid_argument&)
        {
            work_rejected = true;
        }
        RIGIDBODIES_EXPECT(work_rejected, "accidental tiny step cannot launch an unbounded comparison");
    }

    RIGIDBODIES_TEST("energy comparison does not append a rounding artifact substep to an integral duration")
    {
        const auto report = measure_harmonic_energy_drift(SemiImplicitEulerIntegrator {});
        RIGIDBODIES_EXPECT(report.step_count == 7200, "60 seconds at 120 Hz takes exactly 7200 steps");
        RIGIDBODIES_EXPECT_NEAR(report.elapsed_time_s, 60.0, 1.0e-9, "final duration differs only by accumulated clock roundoff");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
