#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <memory>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition ball()
    {
        BodyDefinition definition;
        Collider collider;
        collider.shape = make_circle(0.5);
        definition.colliders.push_back(collider);
        return definition;
    }

    World unforced_world()
    {
        World world;
        world.remove_force_generator(world.force_generators().front());
        // These coincident test bodies isolate force attachment from contact response.
        world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        return world;
    }

    BodyId add_ball(World& world, const std::string& key)
    {
        const auto id = world.create_body(ball(), key);
        world.find_body(id)->override_mass(1.0);
        return id;
    }

    class ConstantForce final : public ForceGenerator
    {
    public:
        explicit ConstantForce(Vec2 force) : force_n(force)
        {
        }
        Vec2 force_n;
        std::string_view name() const override
        {
            return "constant";
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<ConstantForce>(*this);
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center(force_n);
        }
    };

    class CountingForce final : public ForceGenerator
    {
    public:
        int calls { 0 };
        std::string_view name() const override
        {
            return "counting";
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

    // Encodes invocation order in the accumulated x force without external mutable state.
    class OrderedForce final : public ForceGenerator
    {
    public:
        explicit OrderedForce(Real digit) : digit_(digit)
        {
        }
        std::string_view name() const override
        {
            return "ordered";
        }
        std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<OrderedForce>(*this);
        }
        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center({ body.accumulated_force_n().x * 9.0 + digit_, 0.0 });
        }

    private:
        Real digit_;
    };

    class UnsupportedForce final : public ForceGenerator
    {
    public:
        std::string_view name() const override
        {
            return "unsupported";
        }
        void apply(RigidBody&, const ForceContext&) override
        {
        }
    };

    RIGIDBODIES_TEST("body forces affect only their target and combine with global forces")
    {
        auto world = unforced_world();
        const auto first = add_ball(world, "a");
        const auto second = add_ball(world, "b");
        const auto local = std::make_shared<ConstantForce>(Vec2 { 4.0, 0.0 });
        world.add_force_generator(std::make_shared<ConstantForce>(Vec2 { 0.0, 6.0 }));
        RIGIDBODIES_EXPECT(world.add_force_generator(first, local), "valid attachment succeeds");
        world.step(0.5);
        RIGIDBODIES_EXPECT(world.find_body(first)->linear_velocity_m_s() == Vec2 { 2.0, 3.0 }, "target receives both forces");
        RIGIDBODIES_EXPECT(world.find_body(second)->linear_velocity_m_s() == Vec2 { 0.0, 3.0 }, "other body receives global force only");
        RIGIDBODIES_EXPECT(world.find_body(first)->applied_force_n() == Vec2 { 4.0, 6.0 }, "last integrated force retained");
        RIGIDBODIES_EXPECT(world.find_body(first)->accumulated_force_n() == Vec2 {}, "integrated force does not leak into next step");
    }

    RIGIDBODIES_TEST("invalid and stale body handles cannot attach detach or inspect replacement forces")
    {
        auto world = unforced_world();
        const auto stale = add_ball(world, "a");
        const auto force = std::make_shared<ConstantForce>(Vec2 { 3.0, 0.0 });
        world.add_force_generator(stale, force);
        world.destroy_body(stale);
        const auto replacement = add_ball(world, "a");
        RIGIDBODIES_EXPECT(stale.index == replacement.index, "test exercises a recycled slot");
        RIGIDBODIES_EXPECT(!world.add_force_generator(stale, force), "stale attachment rejected");
        RIGIDBODIES_EXPECT(!world.add_force_generator(BodyId {}, force), "empty handle rejected");
        RIGIDBODIES_EXPECT(!world.add_force_generator(BodyId { 9999, 1 }, force), "out of range handle rejected");
        RIGIDBODIES_EXPECT(!world.add_force_generator(replacement, {}), "null force rejected");
        RIGIDBODIES_EXPECT(world.force_generators(stale).empty() && world.force_generators(replacement).empty(), "replacement inherits no attachments");
        world.add_force_generator(replacement, force);
        RIGIDBODIES_EXPECT(!world.remove_force_generator(stale, force), "stale removal rejected");
        RIGIDBODIES_EXPECT(!world.remove_force_generator(replacement, {}), "null removal rejected");
        RIGIDBODIES_EXPECT(world.force_generators(stale).empty() && world.force_generators(replacement).size() == 1, "stale lookup cannot expose replacement");
    }

    RIGIDBODIES_TEST("duplicate attachments are additive disable together and remove only from the chosen list")
    {
        auto world = unforced_world();
        const auto first = add_ball(world, "a");
        const auto second = add_ball(world, "b");
        const auto force = std::make_shared<ConstantForce>(Vec2 { 2.0, 0.0 });
        world.add_force_generator(force);
        world.add_force_generator(first, force);
        world.add_force_generator(first, force);
        world.add_force_generator(second, force);
        world.step(0.25);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->applied_force_n().x, 6.0, 0.0, "each registration contributes once");
        force->set_enabled(false);
        world.step(0.25);
        RIGIDBODIES_EXPECT(world.find_body(first)->applied_force_n() == Vec2 {}, "disabled shared generator contributes nowhere");
        RIGIDBODIES_EXPECT(world.remove_force_generator(first, force), "removal reports existing attachments");
        RIGIDBODIES_EXPECT(!world.remove_force_generator(first, force), "second removal reports no attachment");
        RIGIDBODIES_EXPECT(world.force_generators(first).empty(), "all duplicate body registrations erased");
        RIGIDBODIES_EXPECT(world.force_generators().size() == 1 && world.force_generators(second).size() == 1, "global and other body lists remain attached");
        force->set_enabled(true);
        world.step(0.25);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->applied_force_n().x, 2.0, 0.0, "detached body receives global force only");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(second)->applied_force_n().x, 4.0, 0.0, "other body's attachment still active");
        world.remove_force_generator(force);
        RIGIDBODIES_EXPECT(world.force_generators(second).size() == 1, "global removal preserves body attachment");
    }

    RIGIDBODIES_TEST("destroy and clear release body owned generators while preserving global configuration")
    {
        auto world = unforced_world();
        const auto first = add_ball(world, "a");
        std::weak_ptr<ForceGenerator> destroyed_force;
        {
            const auto force = std::make_shared<ConstantForce>(Vec2 { 1.0, 0.0 });
            destroyed_force = force;
            world.add_force_generator(first, force);
        }
        RIGIDBODIES_EXPECT(!destroyed_force.expired(), "body owns attached generator");
        world.destroy_body(first);
        RIGIDBODIES_EXPECT(destroyed_force.expired(), "destroy releases generator");
        const auto second = add_ball(world, "b");
        std::weak_ptr<ForceGenerator> cleared_force;
        {
            const auto force = std::make_shared<ConstantForce>(Vec2 { 1.0, 0.0 });
            cleared_force = force;
            world.add_force_generator(second, force);
        }
        world.add_force_generator(std::make_shared<UniformGravity>());
        world.clear();
        const auto replacement = add_ball(world, "b");
        RIGIDBODIES_EXPECT(cleared_force.expired(), "clear releases body generator");
        RIGIDBODIES_EXPECT(world.force_generators(second).empty() && world.force_generators(replacement).empty(), "clear invalidates old attachments");
        RIGIDBODIES_EXPECT(!world.add_force_generator(second, std::make_shared<UniformGravity>()), "cleared handle remains invalid after slot reuse");
        RIGIDBODIES_EXPECT(world.force_generators().size() == 1, "clear preserves global configuration");
    }

    RIGIDBODIES_TEST("generator removal accepts references obtained directly from the registration list")
    {
        auto world = unforced_world();
        const auto id = add_ball(world, "a");
        const auto first = std::make_shared<ConstantForce>(Vec2 { 1.0, 0.0 });
        const auto second = std::make_shared<ConstantForce>(Vec2 { 0.0, 1.0 });
        for (const auto& force : { first, second, first })
        {
            world.add_force_generator(force);
            world.add_force_generator(id, force);
        }
        world.remove_force_generator(world.force_generators().front());
        world.remove_force_generator(id, world.force_generators(id).front());
        RIGIDBODIES_EXPECT(world.force_generators().size() == 1 && world.force_generators().front() == second,
            "global compaction preserves comparison target");
        RIGIDBODIES_EXPECT(world.force_generators(id).size() == 1 && world.force_generators(id).front() == second,
            "body compaction preserves comparison target");
    }

    RIGIDBODIES_TEST("global force phase precedes body phase and both traverse canonical bodies")
    {
        auto world = unforced_world();
        const auto second = add_ball(world, "b");
        const auto first = add_ball(world, "a");
        const auto force = std::make_shared<CountingForce>();
        world.add_force_generator(force);
        world.add_force_generator(first, force);
        world.add_force_generator(first, force);
        world.add_force_generator(second, force);
        world.step(0.01);
        RIGIDBODIES_EXPECT(force->calls == 5, "shared generator called once per registration and affected body");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(first)->applied_force_n().x, 8.0, 0.0, "a receives global call one then local calls three and four");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(second)->applied_force_n().x, 7.0, 0.0, "b receives global call two then local call five");
    }

    RIGIDBODIES_TEST("global and body force lists preserve registration order")
    {
        auto world = unforced_world();
        const auto id = add_ball(world, "a");
        world.add_force_generator(std::make_shared<OrderedForce>(1.0));
        world.add_force_generator(std::make_shared<OrderedForce>(2.0));
        world.add_force_generator(id, std::make_shared<OrderedForce>(3.0));
        world.add_force_generator(id, std::make_shared<OrderedForce>(4.0));
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->applied_force_n().x, 1234.0, 0.0, "global then local registration order retained");
    }

    RIGIDBODIES_TEST("snapshot shares one detached generator clone across global and body lists and replays its state")
    {
        auto world = unforced_world();
        const auto first = add_ball(world, "a");
        const auto second = add_ball(world, "b");
        const auto force = std::make_shared<CountingForce>();
        world.add_force_generator(force);
        world.add_force_generator(first, force);
        world.add_force_generator(second, force);
        world.add_force_generator(second, force);
        world.step(0.01);
        const auto checkpoint = world.snapshot();
        for (int index = 0; index < 7; ++index)
        {
            world.step(0.01);
        }
        const auto expected_first = *world.find_body(first);
        const auto expected_second = *world.find_body(second);
        const auto expected_calls = force->calls;
        force->set_enabled(false);
        force->calls = 999;
        world.clear();
        world.restore(checkpoint);
        const auto restored = std::dynamic_pointer_cast<CountingForce>(world.force_generators().front());
        RIGIDBODIES_EXPECT(restored != force && restored->is_enabled() && restored->calls == 5, "checkpoint detached external state");
        RIGIDBODIES_EXPECT(world.force_generators(first).front() == restored, "global and first body share clone");
        RIGIDBODIES_EXPECT(world.force_generators(second).size() == 2 && world.force_generators(second)[0] == restored && world.force_generators(second)[1] == restored,
            "all body lists and duplicate attachments share the same clone");
        for (int index = 0; index < 7; ++index)
        {
            world.step(0.01);
        }
        RIGIDBODIES_EXPECT(restored->calls == expected_calls, "stateful generator resumes saved call count");
        RIGIDBODIES_EXPECT(world.find_body(first)->position_m() == expected_first.position_m() && world.find_body(second)->position_m() == expected_second.position_m(),
            "replayed positions match exactly");
        RIGIDBODIES_EXPECT(world.find_body(first)->linear_velocity_m_s() == expected_first.linear_velocity_m_s() && world.find_body(second)->linear_velocity_m_s() == expected_second.linear_velocity_m_s(),
            "replayed velocities match exactly");
        World other;
        other.restore(checkpoint);
        const auto other_force = std::dynamic_pointer_cast<CountingForce>(other.force_generators(first).front());
        RIGIDBODIES_EXPECT(other_force != restored && other_force->calls == 5, "restored worlds remain independent from one another and snapshot");
    }

    RIGIDBODIES_TEST("snapshot restores body only generator parameters and disabled state")
    {
        auto world = unforced_world();
        const auto id = add_ball(world, "a");
        const auto force = std::make_shared<ConstantForce>(Vec2 { 2.0, 3.0 });
        force->set_enabled(false);
        world.add_force_generator(id, force);
        const auto checkpoint = world.snapshot();
        force->force_n = { 99.0, 100.0 };
        force->set_enabled(true);
        world.remove_force_generator(id, force);
        world.restore(checkpoint);
        const auto restored = std::dynamic_pointer_cast<ConstantForce>(world.force_generators(id).front());
        RIGIDBODIES_EXPECT(restored != force && !restored->is_enabled(), "attachment and saved enable state restored");
        RIGIDBODIES_EXPECT(restored->force_n == Vec2 { 2.0, 3.0 }, "parameters detached from external mutations");
        RIGIDBODIES_EXPECT(world.force_generators().empty(), "body only generator remains body scoped");
    }

    RIGIDBODIES_TEST("gravity scale rejects nonfinite values without losing signed finite support")
    {
        auto definition = ball();
        definition.gravity_scale = -2.0;
        RigidBody body { definition };
        RIGIDBODIES_EXPECT_NEAR(body.gravity_scale(), -2.0, 0.0, "negative scales preserve reverse-gravity physics");
        body.set_gravity_scale(0.0);
        body.set_awake(false);
        for (const auto invalid : { std::numeric_limits<Real>::infinity(), -std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
        {
            bool rejected = false;
            try
            {
                body.set_gravity_scale(invalid);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected && body.gravity_scale() == 0.0 && !body.is_awake(), "invalid edits preserve scale and sleep state");
            definition.gravity_scale = invalid;
            rejected = false;
            try
            {
                (void)RigidBody { definition };
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "invalid definitions are rejected before creating a body");
        }
        body.set_gravity_scale(-0.5);
        RIGIDBODIES_EXPECT(body.is_awake() && body.gravity_scale() == -0.5, "valid scale changes continue to wake the body");
    }

    RIGIDBODIES_TEST("unsupported body generator snapshot fails without changing live world")
    {
        auto world = unforced_world();
        const auto id = add_ball(world, "a");
        const auto force = std::make_shared<UnsupportedForce>();
        world.add_force_generator(id, force);
        bool rejected = false;
        try
        {
            (void)world.snapshot();
        }
        catch (const std::logic_error&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "body generators must honor snapshot clone contract");
        RIGIDBODIES_EXPECT(world.is_valid(id) && world.force_generators(id).front() == force, "failed capture leaves attachments intact");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
