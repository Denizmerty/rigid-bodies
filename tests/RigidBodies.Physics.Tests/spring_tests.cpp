#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    World make_world(IntegratorPtr method = std::make_shared<SemiImplicitEulerIntegrator>(), bool sleeping = false)
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = sleeping;
        settings.sleep.quiet_duration_s = 0.05;
        World world { settings };
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        world.set_integrator(std::move(method));
        return world;
    }

    BodyId add_body(World& world, Vec2 position, std::string key, BodyType type = BodyType::dynamic_body)
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.type = type;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, std::move(key));
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    std::vector<IntegratorPtr> methods()
    {
        return { std::make_shared<SemiImplicitEulerIntegrator>(), std::make_shared<VelocityVerletIntegrator>(), std::make_shared<RungeKutta4Integrator>() };
    }

    Real energy(const World& world, SpringId spring)
    {
        Real result = world.spring_report(spring)->potential_energy_j;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                result += body.kinetic_energy_j();
            });
        return result;
    }

    class OrderedForce final : public ForceGenerator
    {
    public:
        int calls {};
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<OrderedForce>(*this);
        }
        std::string_view name() const override
        {
            return "ordered_probe";
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            ++calls;
            body.apply_force_at_center({ static_cast<Real>(calls), 0.0 });
        }
    };

    class UnsupportedStagedIntegrator final : public Integrator
    {
    public:
        std::string_view name() const override
        {
            return "unsupported_pair_probe";
        }
        bool requires_force_evaluation() const override
        {
            return true;
        }
        void integrate_velocity(RigidBody&, Real) const override
        {
        }
        void integrate_position(RigidBody&, Real) const override
        {
        }
    };

    RIGIDBODIES_TEST("linear endpoint forces include equal opposite loads and conserving anchor torques")
    {
        auto world = make_world();
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 3.0, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.local_anchor_first_m = { 0.0, 1.0 };
        definition.local_anchor_second_m = { 0.0, -1.0 };
        definition.rest_length_m = 1.0;
        definition.stiffness_n_m = 2.0;
        const auto spring = world.create_spring(definition, "offset");
        const auto report = *world.spring_report(spring);
        const auto orbital = rigidbodies::math::cross(world.find_body(a)->world_center_of_mass_m(), report.force_on_first_n) +
            rigidbodies::math::cross(world.find_body(b)->world_center_of_mass_m(), -report.force_on_first_n);
        RIGIDBODIES_EXPECT_NEAR(orbital + report.torque_on_first_n_m + report.torque_on_second_n_m, 0.0, 1.0e-12, "internal axial loads conserve total angular momentum about the origin");
        world.step(0.001);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->applied_force_n().x, report.force_on_first_n.x, 1.0e-12, "first endpoint retains its spring load");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->applied_force_n().x, -report.force_on_first_n.x, 1.0e-12, "second endpoint receives the opposite force");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->applied_torque_n_m(), report.torque_on_first_n_m, 1.0e-12, "off-center force produces endpoint torque");
        const auto& channels = world.find_body(a)->applied_force_channels();
        bool named = false;
        for (const auto& channel : channels)
            named = named || channel.name == "spring/offset";
        RIGIDBODIES_EXPECT(named, "spring loads have a stable named force channel");
    }

    RIGIDBODIES_TEST("all integrators conserve pair momentum and bound conservative spring energy")
    {
        const auto integrators = methods();
        for (std::size_t index = 0; index < integrators.size(); ++index)
        {
            auto world = make_world(integrators[index]);
            const auto a = add_body(world, { -0.75, 0.0 }, "a");
            const auto b = add_body(world, { 0.75, 0.0 }, "b");
            LinearSpringDefinition definition;
            definition.first = a;
            definition.second = b;
            definition.stiffness_n_m = 2.0;
            const auto spring = world.create_spring(definition);
            const auto initial = energy(world, spring);
            Real largest_error = 0.0;
            for (int step = 0; step < 500; ++step)
            {
                world.step(0.01);
                largest_error = std::max(largest_error, std::abs(energy(world, spring) - initial));
                RIGIDBODIES_EXPECT_NEAR(world.statistics().total_linear_momentum_kg_m_s.x, 0.0, 1.0e-12, "paired stage forces retain zero total momentum");
            }
            const auto tolerance = index == 0 ? 0.003 : index == 1 ? 0.00003
                                                                   : 1.0e-8;
            RIGIDBODIES_EXPECT(largest_error < tolerance, "elastic spring energy follows the selected integrator's expected bound");
            RIGIDBODIES_EXPECT_NEAR(world.statistics().total_spring_potential_energy_j, world.spring_report(spring)->potential_energy_j, 1.0e-14, "statistics include the present elastic energy separately from gravity");
        }
    }

    RIGIDBODIES_TEST("coupled Verlet and RK4 attain their orders for the moving two-body oscillator")
    {
        const auto error = [](IntegratorPtr method, Real dt, int steps)
        {
            auto world = make_world(std::move(method));
            const auto a = add_body(world, { -0.75, 0.0 }, "a");
            const auto b = add_body(world, { 0.75, 0.0 }, "b");
            LinearSpringDefinition definition;
            definition.first = a;
            definition.second = b;
            definition.stiffness_n_m = 2.0;
            world.create_spring(definition);
            for (int step = 0; step < steps; ++step)
                world.step(dt);
            const auto expected = 0.5 + 0.25 * std::cos(2.0 * dt * steps);
            return std::abs(world.find_body(b)->position_m().x - expected);
        };
        for (int method = 0; method < 2; ++method)
        {
            const auto make = [&]() -> IntegratorPtr
            {
                if (method == 0)
                    return std::make_shared<VelocityVerletIntegrator>();
                return std::make_shared<RungeKutta4Integrator>();
            };
            const auto ratio = error(make(), 0.05, 26) / error(make(), 0.025, 52);
            RIGIDBODIES_EXPECT(ratio > (method == 0 ? 3.8 : 14.0), "halving the step reveals second or fourth order for jointly moving endpoints");
        }
    }

    RIGIDBODIES_TEST("axial damping removes energy without damping tangential motion")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto a = add_body(world, { -0.75, 0.0 }, "a");
        const auto b = add_body(world, { 0.75, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.stiffness_n_m = 2.0;
        definition.damping_n_s_m = 0.7;
        const auto spring = world.create_spring(definition);
        const auto initial = energy(world, spring);
        Real previous = initial;
        for (int step = 0; step < 500; ++step)
        {
            world.step(0.01);
            const auto current = energy(world, spring);
            RIGIDBODIES_EXPECT(current <= previous + 1.0e-10, "positive axial damping cannot add mechanical energy");
            previous = current;
        }
        RIGIDBODIES_EXPECT(previous < initial * 0.002, "the damped pair settles close to rest length");
        world.find_body(a)->set_position({ -0.5, 0.0 });
        world.find_body(b)->set_position({ 0.5, 0.0 });
        world.find_body(a)->set_linear_velocity({ 0.0, 1.0 });
        world.find_body(b)->set_linear_velocity({ 0.0, -1.0 });
        const auto report = *world.spring_report(spring);
        RIGIDBODIES_EXPECT_NEAR(report.dissipated_power_w, 0.0, 1.0e-14, "transverse relative motion is not axial damping");
    }

    RIGIDBODIES_TEST("angular springs retain multiple turns and oppose relative angular velocity")
    {
        auto world = make_world();
        const auto a = add_body(world, {}, "a", BodyType::static_body);
        const auto b = add_body(world, { 2.0, 0.0 }, "b");
        world.find_body(b)->set_orientation(rigidbodies::math::two_pi + 0.2);
        world.find_body(b)->set_angular_velocity(0.5);
        AngularSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.rest_angle_rad = 0.2;
        definition.stiffness_n_m_rad = 0.01;
        definition.damping_n_m_s_rad = 0.02;
        const auto spring = world.create_spring(definition, "torsion");
        const auto report = *world.spring_report(spring);
        RIGIDBODIES_EXPECT_NEAR(report.angular_displacement_rad, rigidbodies::math::two_pi, 1.0e-14, "torsion uses the unwrapped relative angle");
        RIGIDBODIES_EXPECT_NEAR(report.torque_on_first_n_m + report.torque_on_second_n_m, 0.0, 0.0, "torques are equal and opposite");
        RIGIDBODIES_EXPECT_NEAR(report.dissipated_power_w, 0.005, 1.0e-14, "angular damping power is coefficient times relative speed squared");
        world.step(0.001);
        RIGIDBODIES_EXPECT(world.find_body(b)->angular_velocity_rad_s() < 0.5, "torsion and damping oppose the positive relative spin");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->orientation_rad(), 0.0, 0.0, "a fixed support does not turn under its reaction torque");
    }

    RIGIDBODIES_TEST("spring identifiers clean up stale endpoints and snapshots replay independent state")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.5, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        const auto spring = world.create_spring(definition, "spring");
        world.step(0.01);
        const auto checkpoint = world.snapshot();
        world.step(0.01);
        const auto expected = world.find_body(b)->position_m();
        world.set_spring_enabled(spring, false);
        RIGIDBODIES_EXPECT(world.spring_report(spring)->potential_energy_j == 0.0, "disabled spring reports no active stored energy");
        world.restore(checkpoint);
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(b)->position_m() == expected && world.spring_report(spring)->enabled, "snapshot restores definitions, forces and exact replay");
        world.destroy_body(a);
        RIGIDBODIES_EXPECT(!world.is_valid(spring) && world.spring_ids().empty(), "destroying an endpoint removes its spring");
        const auto replacement = add_body(world, {}, "replacement");
        definition.first = replacement;
        const auto next = world.create_spring(definition, "spring");
        RIGIDBODIES_EXPECT(next != spring && !world.remove_spring(spring), "a stale spring handle cannot remove its recycled slot");
        world.clear();
        RIGIDBODIES_EXPECT(!world.spring_report(next) && world.spring_ids().empty(), "world reset clears pair interactions");
    }

    RIGIDBODIES_TEST("creating springs from queried definitions survives slot vector growth")
    {
        auto world = make_world();
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.5, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.local_anchor_first_m = { 0.1, 0.2 };
        definition.local_anchor_second_m = { -0.3, 0.4 };
        definition.rest_length_m = 0.75;
        definition.stiffness_n_m = 3.25;
        definition.damping_n_s_m = 0.625;
        const auto source = world.create_spring(definition, "source");
        // Fetch a fresh view for each call; the call itself is responsible for preserving aliased
        // input while growing its owning vector. Repeated appends exercise multiple reallocations.
        for (int index = 0; index < 64; ++index)
        {
            const auto copy = world.create_spring(*world.spring_definition(source), "copy_" + std::to_string(index));
            const auto& actual = std::get<LinearSpringDefinition>(*world.spring_definition(copy));
            RIGIDBODIES_EXPECT(copy != source && actual.first == a && actual.second == b, "duplicated endpoint identifiers survive allocation");
            RIGIDBODIES_EXPECT(actual.local_anchor_first_m == definition.local_anchor_first_m && actual.local_anchor_second_m == definition.local_anchor_second_m,
                "aliased local anchors are copied before their storage can move");
            RIGIDBODIES_EXPECT(actual.rest_length_m == definition.rest_length_m && actual.stiffness_n_m == definition.stiffness_n_m &&
                    actual.damping_n_s_m == definition.damping_n_s_m && actual.enabled,
                "all requested spring coefficients survive slot growth");
        }
        RIGIDBODIES_EXPECT(world.spring_ids().size() == 65 && world.is_valid(source), "the source and every independent copy remain registered");
    }

    RIGIDBODIES_TEST("resting spring groups sleep coherently and wake through a distant endpoint")
    {
        auto world = make_world(std::make_shared<SemiImplicitEulerIntegrator>(), true);
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.0, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        const auto spring = world.create_spring(definition);
        for (int step = 0; step < 30; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(a)->is_awake() && !world.find_body(b)->is_awake(), "separated spring neighbors settle together");
        world.find_body(b)->apply_linear_impulse({ 0.1, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(a)->is_awake() && world.find_body(b)->is_awake(), "an impulse wakes the pair before force integration");
        world.find_body(a)->set_awake(false);
        world.find_body(b)->set_awake(false);
        world.set_spring_enabled(spring, false);
        RIGIDBODIES_EXPECT(world.find_body(a)->is_awake() && world.find_body(b)->is_awake(), "editing a spring wakes both endpoints");
    }

    RIGIDBODIES_TEST("invalid spring definitions leave the world unchanged")
    {
        auto world = make_world();
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.0, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.stiffness_n_m = -1.0;
        bool rejected = false;
        try
        {
            world.create_spring(definition);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && world.spring_ids().empty(), "negative stiffness cannot become an active interaction");
        definition.stiffness_n_m = 1.0;
        definition.local_anchor_first_m.x = std::numeric_limits<Real>::infinity();
        rejected = false;
        try
        {
            world.create_spring(definition);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && world.spring_ids().empty(), "nonfinite anchors are rejected before mutation");
    }

    RIGIDBODIES_TEST("coupled probes preserve shared generator identity order and live call counts")
    {
        for (const auto& method : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(method);
            const auto a = add_body(world, {}, "a");
            const auto b = add_body(world, { 1.0, 0.0 }, "b");
            LinearSpringDefinition definition;
            definition.first = a;
            definition.second = b;
            definition.stiffness_n_m = 0.0;
            world.create_spring(definition);
            const auto generator = std::make_shared<OrderedForce>();
            world.add_force_generator(generator);
            world.add_force_generator(a, generator);
            world.step(0.1);
            RIGIDBODIES_EXPECT(generator->calls == 3, "stage probes do not advance live generator state");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->linear_velocity_m_s().x, 0.4, 1.0e-13, "the first global and third shared local registration are replayed in every stage");
            RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->linear_velocity_m_s().x, 0.2, 1.0e-13, "the second canonical body sees the second shared global call");
            const auto snapshot = world.snapshot();
            world.step(0.1);
            const auto expected = world.find_body(a)->position_m();
            world.restore(snapshot);
            world.step(0.1);
            RIGIDBODIES_EXPECT(world.find_body(a)->position_m() == expected, "shared generator state and spring stages replay exactly");
        }
    }

    RIGIDBODIES_TEST("a prescribed moving anchor is sampled at every coupled stage time")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto anchor = add_body(world, {}, "anchor", BodyType::kinematic_body);
        const auto subject = add_body(world, { 1.0, 0.0 }, "subject");
        LinearMotion path;
        path.velocity_m_s = { 1.0, 0.0 };
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(anchor, KinematicMotion { path }), "the prescribed anchor path attaches");
        LinearSpringDefinition definition;
        definition.first = anchor;
        definition.second = subject;
        definition.stiffness_n_m = 4.0;
        world.create_spring(definition);
        for (int step = 0; step < 40; ++step)
            world.step(0.025);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(subject)->position_m().x, 2.0 - 0.5 * std::sin(2.0), 5.0e-8, "joint stage sampling follows the analytical moving-support oscillator");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(anchor)->position_m().x, 1.0, 1.0e-12, "spring reaction cannot perturb the prescribed anchor");
    }

    RIGIDBODIES_TEST("coupled angular springs preserve total spin and conservative energy")
    {
        auto world = make_world(std::make_shared<VelocityVerletIntegrator>());
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.0, 0.0 }, "b");
        world.find_body(a)->set_orientation(-0.2);
        world.find_body(b)->set_orientation(0.2);
        AngularSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        definition.stiffness_n_m_rad = 0.01;
        const auto spring = world.create_spring(definition);
        const auto initial = energy(world, spring);
        for (int step = 0; step < 300; ++step)
        {
            world.step(0.01);
            RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->angular_velocity_rad_s() + world.find_body(b)->angular_velocity_rad_s(),
                0.0,
                1.0e-12,
                "equal inertia endpoints exchange opposite angular momentum");
            RIGIDBODIES_EXPECT(std::abs(energy(world, spring) - initial) < initial * 0.00011,
                "torsional Verlet energy stays within its second-order oscillation bound");
        }
    }

    RIGIDBODIES_TEST("damped springs remain asleep after their final small movement")
    {
        auto world = make_world(std::make_shared<SemiImplicitEulerIntegrator>(), true);
        const auto anchor = add_body(world, {}, "anchor", BodyType::static_body);
        const auto subject = add_body(world, { 1.2, 0.0 }, "subject");
        LinearSpringDefinition definition;
        definition.first = anchor;
        definition.second = subject;
        definition.stiffness_n_m = 4.0;
        definition.damping_n_s_m = 4.0;
        world.create_spring(definition);
        for (int step = 0; step < 1500; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(subject)->is_awake(), "the damped supported oscillator enters sleep");
        const auto position = world.find_body(subject)->position_m();
        for (int step = 0; step < 100; ++step)
            world.step(0.01);
        RIGIDBODIES_EXPECT(!world.find_body(subject)->is_awake() && world.find_body(subject)->position_m() == position,
            "its own final damping or sub-threshold spring load does not repeatedly wake it");
        world.find_body(anchor)->set_position({ 0.1, 0.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.find_body(subject)->is_awake(), "editing a separated fixed anchor wakes its dependent sleeper");
    }

    RIGIDBODIES_TEST("unsupported custom staged integration rejects coupled forces before mutation")
    {
        auto world = make_world(std::make_shared<UnsupportedStagedIntegrator>());
        const auto a = add_body(world, {}, "a");
        const auto b = add_body(world, { 1.5, 0.0 }, "b");
        LinearSpringDefinition definition;
        definition.first = a;
        definition.second = b;
        world.create_spring(definition);
        const auto force = std::make_shared<OrderedForce>();
        world.add_force_generator(force);
        world.find_body(a)->apply_force_at_center({ 3.0, 0.0 });
        bool rejected = false;
        try
        {
            world.step(0.1);
        }
        catch (const std::logic_error&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && force->calls == 0 && world.statistics().step_index == 0,
            "unsupported coupled prediction is diagnosed before advancing time or force state");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(a)->accumulated_force_n().x, 3.0, 0.0, "pending manual loads survive rejection unchanged");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(b)->position_m().x, 1.5, 0.0, "rejection preserves body placement");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
