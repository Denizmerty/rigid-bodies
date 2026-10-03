#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    class CountingForce final : public ForceGenerator
    {
    public:
        int calls {};
        std::string_view name() const override
        {
            return "counter";
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<CountingForce>(*this);
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center({ static_cast<Real>(++calls), 0.0 });
        }
    };

    class UnclonableForce final : public ForceGenerator
    {
    public:
        int calls {};
        std::string_view name() const override
        {
            return "unclonable";
        }
        void apply(RigidBody&, const ForceContext&) override
        {
            ++calls;
        }
    };

    class SelfDisablingForce final : public ForceGenerator
    {
    public:
        int calls {};
        std::string_view name() const override
        {
            return "one_shot";
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<SelfDisablingForce>(*this);
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            ++calls;
            body.apply_force_at_center({ 2.0, 0.0 });
            set_enabled(false);
        }
    };

    World make_world(const IntegratorPtr& integrator)
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        World world { settings };
        // Coincident bodies exercise force registration traversal without collision impulses.
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        world.set_integrator(integrator);
        return world;
    }

    BodyId add_body(World& world, const std::string& key)
    {
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        const auto id = world.create_body(definition, key);
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    RIGIDBODIES_TEST("staged probes preserve shared force registration traversal without advancing live state")
    {
        for (const auto& integrator : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(integrator);
            const auto second = add_body(world, "b");
            const auto first = add_body(world, "a");
            const auto force = std::make_shared<CountingForce>();
            world.add_force_generator(force);
            world.add_force_generator(first, force);
            world.add_force_generator(second, force);
            world.find_body(first)->apply_force_at_center({ 2.0, 0.0 }, "manual");
            world.step(0.1);
            RIGIDBODIES_EXPECT(force->calls == 4, "only the live canonical traversal advances persistent generator state");
            for (const auto id : { first, second })
            {
                RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->linear_velocity_m_s().x, 0.6, 1.0e-12, "each stage replays aliased global and attached registrations");
                RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->position_m().x, 0.03, 1.0e-12, "constant staged load integrates position exactly");
                RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->applied_force_n().x, 6.0, 0.0, "visible load remains the live initial sample");
            }
            RIGIDBODIES_EXPECT(world.find_body(first)->applied_force_channels().size() == 3, "gravity counter and manual channels survive probes");
            const auto checkpoint = world.snapshot();
            world.step(0.1);
            const auto expected = *world.find_body(first);
            world.restore(checkpoint);
            world.step(0.1);
            RIGIDBODIES_EXPECT(world.find_body(first)->position_m() == expected.position_m(), "staged replay restores exact component and body state");
            RIGIDBODIES_EXPECT(world.find_body(first)->linear_velocity_m_s() == expected.linear_velocity_m_s(), "staged velocity replay is exact");
        }
    }

    RIGIDBODIES_TEST("staged integrators reject unsafe force probes before advancing live generators")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto id = add_body(world, "a");
        const auto force = std::make_shared<UnclonableForce>();
        world.add_force_generator(id, force);
        bool rejected = false;
        try
        {
            world.step(0.1);
        }
        catch (const std::logic_error&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected && force->calls == 0, "missing independent clones cannot silently mutate generator state in stages");
        RIGIDBODIES_EXPECT(world.statistics().step_index == 0 && world.find_body(id)->position_m() == Vec2 {}, "failed setup does not integrate or advance time");
    }

    RIGIDBODIES_TEST("stage replay matches global enabled-state checks when a generator disables itself")
    {
        for (const auto& integrator : { IntegratorPtr { std::make_shared<VelocityVerletIntegrator>() }, IntegratorPtr { std::make_shared<RungeKutta4Integrator>() } })
        {
            auto world = make_world(integrator);
            const auto first = add_body(world, "a");
            const auto second = add_body(world, "b");
            const auto force = std::make_shared<SelfDisablingForce>();
            world.add_force_generator(force);
            world.add_force_generator(first, force);
            world.add_force_generator(second, force);
            const auto checkpoint = world.snapshot();
            world.step(0.1);
            RIGIDBODIES_EXPECT(force->calls == 2 && !force->is_enabled(), "live global registration finishes both bodies and later local registrations stay disabled");
            for (const auto id : { first, second })
            {
                const auto* body = world.find_body(id);
                RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 0.2, 1.0e-12, "every stage observes the same one-shot load for both bodies");
                RIGIDBODIES_EXPECT_NEAR(body->position_m().x, 0.01, 1.0e-12, "one-shot load produces exact constant-acceleration displacement");
            }
            const auto expected_position = world.find_body(second)->position_m();
            world.restore(checkpoint);
            world.step(0.1);
            const auto restored_force = std::dynamic_pointer_cast<SelfDisablingForce>(world.force_generators()[1]);
            RIGIDBODIES_EXPECT(restored_force && restored_force->calls == 2 && !restored_force->is_enabled(), "snapshot restores enabled state and alias-preserving traversal");
            RIGIDBODIES_EXPECT(world.find_body(second)->position_m() == expected_position, "replay of a self-disabling generator is deterministic");
        }
    }

    RIGIDBODIES_TEST("staged predictions keep the first transform throughout visual substeps")
    {
        auto world = make_world(std::make_shared<RungeKutta4Integrator>());
        const auto id = add_body(world, "a");
        world.find_body(id)->set_linear_velocity({ 1.0, 0.0 });
        world.step(0.1);
        world.step(0.1, false);
        const auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(body->interpolated_transform(0.5).translation.x, 0.1, 1.0e-12, "render blends complete presentation interval");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
