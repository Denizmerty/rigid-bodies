#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <limits>
#include <stdexcept>

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    BodyDefinition disc()
    {
        BodyDefinition definition;
        definition.position_m = { 4.0, -3.0 };
        Collider collider;
        collider.shape = make_circle(0.5);
        collider.local_transform.translation = { 0.25, 0.0 };
        definition.colliders.push_back(collider);
        return definition;
    }

    void disable_gravity(World& world)
    {
        world.force_generators().front()->set_enabled(false);
    }

    const ForceChannelContribution& channel(const RigidBody& body, std::string_view name)
    {
        for (const auto& contribution : body.applied_force_channels())
        {
            if (contribution.name == name)
            {
                return contribution;
            }
        }
        RIGIDBODIES_FAIL("expected force channel is missing");
    }

    class MixedForce final : public ForceGenerator
    {
    public:
        std::string_view name() const override
        {
            return "generator";
        }

        void apply(RigidBody& body, const ForceContext&) override
        {
            body.apply_force_at_center({ 3.0, 4.0 });
            body.apply_torque(2.0);
            body.apply_force_at_center({ -1.0, 0.0 }, "override");
        }
    };

    class ThrowingForce final : public ForceGenerator
    {
    public:
        std::string_view name() const override
        {
            return "throwing";
        }

        void apply(RigidBody&, const ForceContext&) override
        {
            throw std::runtime_error("force evaluation failed");
        }
    };

    RIGIDBODIES_TEST("direct force channels group loads and retain off-center torque after integration")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->override_mass(2.0);
        body->apply_force_at_center({ 2.0, 3.0 }, "zeta");
        body->apply_force_at_world_point({ 0.0, 5.0 }, body->world_center_of_mass_m() + Vec2 { 2.0, 0.0 }, "alpha");
        body->apply_force_at_center({ -1.0, 1.0 }, "zeta");
        body->apply_torque(4.0, "alpha");
        body->apply_force_at_center({ 6.0, 0.0 });
        body->apply_torque(-2.0);
        RIGIDBODIES_EXPECT(body->applied_force_channels().empty(), "queued loads have not been integrated yet");
        world.step(0.25);

        RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 {}, "world clears pending force after integration");
        RIGIDBODIES_EXPECT_NEAR(body->accumulated_torque_n_m(), 0.0, 0.0, "world clears pending torque");
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 7.0, 9.0 }, "retained total includes all three channels");
        RIGIDBODIES_EXPECT_NEAR(body->applied_torque_n_m(), 12.0, 0.0, "retained torque includes force lever arm and direct torques");
        RIGIDBODIES_EXPECT(body->linear_velocity_m_s() == Vec2 { 0.875, 1.125 }, "retained load agrees with actual acceleration");
        RIGIDBODIES_EXPECT(body->applied_force_channels().size() == 3, "same named calls merge");
        RIGIDBODIES_EXPECT(body->applied_force_channels()[0].name == "alpha" && body->applied_force_channels()[1].name == "external" && body->applied_force_channels()[2].name == "zeta",
            "channels have deterministic lexical ordering");
        RIGIDBODIES_EXPECT(channel(*body, "alpha").force_n == Vec2 { 0.0, 5.0 }, "off-center force retained in its channel");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "alpha").torque_n_m, 14.0, 0.0, "channel includes offset and direct torque");
        RIGIDBODIES_EXPECT(channel(*body, "zeta").force_n == Vec2 { 1.0, 4.0 }, "channel contains summed force");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "external").torque_n_m, -2.0, 0.0, "unnamed direct torque uses external channel");
    }

    RIGIDBODIES_TEST("retained loads track the latest substep and clear only after a valid unloaded step")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->apply_force_at_center({ 1.0, 2.0 }, "first");
        world.step(0.01);
        body->apply_force_at_center({ 3.0, 4.0 }, "second");
        world.step(0.0);
        world.step(std::numeric_limits<Real>::quiet_NaN());
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 1.0, 2.0 }, "invalid steps preserve previous diagnostics");
        RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 { 3.0, 4.0 }, "invalid steps preserve pending loads");
        world.step(0.01, false);
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 3.0, 4.0 }, "last substep replaces rather than averages applied load");
        RIGIDBODIES_EXPECT(body->applied_force_channels().size() == 1 && channel(*body, "second").force_n == Vec2 { 3.0, 4.0 },
            "channels from the preceding substep do not leak");
        body->clear_accumulators();
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 3.0, 4.0 }, "clearing pending loads leaves retained diagnostics visible");
        world.step(0.01);
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 {} && body->applied_force_channels().empty(), "unloaded valid step clears retained force and channels");
        RIGIDBODIES_EXPECT_NEAR(body->applied_torque_n_m(), 0.0, 0.0, "unloaded valid step clears retained torque");
    }

    RIGIDBODIES_TEST("generator context supplies channel names and explicit names override it")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        world.add_force_generator(std::make_shared<MixedForce>());
        world.step(0.01);
        auto* body = world.find_body(id);
        RIGIDBODIES_EXPECT(channel(*body, "generator").force_n == Vec2 { 3.0, 4.0 }, "unnamed generator load uses generator name");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "generator").torque_n_m, 2.0, 0.0, "unnamed generator torque shares the generator channel");
        RIGIDBODIES_EXPECT(channel(*body, "override").force_n == Vec2 { -1.0, 0.0 }, "explicit name overrides generator context");
        body->apply_force_at_center({ 7.0, 8.0 });
        world.step(0.01);
        RIGIDBODIES_EXPECT(channel(*body, "external").force_n == Vec2 { 7.0, 8.0 }, "generator context is cleared on normal return");
    }

    RIGIDBODIES_TEST("throwing force generators do not leave a stale channel context")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto throwing = std::make_shared<ThrowingForce>();
        world.add_force_generator(throwing);
        bool threw = false;
        try
        {
            world.step(0.01);
        }
        catch (const std::runtime_error&)
        {
            threw = true;
        }
        RIGIDBODIES_EXPECT(threw, "generator exception reaches the caller");
        world.remove_force_generator(throwing);
        auto* body = world.find_body(id);
        body->apply_torque(5.0);
        world.step(0.01);
        RIGIDBODIES_EXPECT(body->applied_force_channels().size() == 1, "failed generator leaves no phantom channel");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "external").torque_n_m, 5.0, 0.0, "exception cleanup resets default channel to external");
    }

    RIGIDBODIES_TEST("snapshots independently preserve pending and retained named loads")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->apply_force_at_center({ 2.0, 3.0 }, "completed");
        body->apply_torque(4.0, "completed");
        world.step(0.01);
        body->apply_force_at_center({ 5.0, 6.0 }, "pending");
        body->apply_torque(7.0, "pending");
        const auto saved = world.snapshot();
        world.step(0.01);
        world.restore(saved);
        body = world.find_body(id);
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 2.0, 3.0 }, "saved retained total restored");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "completed").torque_n_m, 4.0, 0.0, "saved retained channel restored");
        RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 { 5.0, 6.0 }, "pending total restored separately");
        world.step(0.01);
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 { 5.0, 6.0 }, "restored pending total integrates next");
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "pending").torque_n_m, 7.0, 0.0, "restored pending channel retains its own name and torque");
        world.restore(saved);
        body = world.find_body(id);
        RIGIDBODIES_EXPECT_NEAR(channel(*body, "completed").torque_n_m, 4.0, 0.0, "stepping restored world cannot mutate saved diagnostics");
    }

    RIGIDBODIES_TEST("non-dynamic transitions discard loads and fixed rotation preserves applied torque diagnostics")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->set_fixed_rotation(true);
        body->apply_torque(9.0, "motor");
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(body->angular_velocity_rad_s(), 0.0, 0.0, "fixed rotation prevents angular acceleration");
        RIGIDBODIES_EXPECT_NEAR(body->applied_torque_n_m(), 9.0, 0.0, "diagnostics retain the applied torque even with fixed rotation");
        for (const auto type : { BodyType::static_body, BodyType::kinematic_body })
        {
            body->apply_force_at_center({ 1.0, 2.0 }, "queued");
            body->set_type(type);
            body->apply_force_at_center({ 3.0, 4.0 }, "ignored");
            body->apply_torque(5.0, "ignored");
            RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 {}, "non-dynamic transition discards queued forces and ignores new ones");
            RIGIDBODIES_EXPECT_NEAR(body->accumulated_torque_n_m(), 0.0, 0.0, "non-dynamic body carries no pending torque");
            RIGIDBODIES_EXPECT(body->applied_force_channels().empty(), "non-dynamic body does not show stale channels");
            RIGIDBODIES_EXPECT_NEAR(body->applied_torque_n_m(), 0.0, 0.0, "non-dynamic body does not show stale torque");
            body->set_type(BodyType::dynamic_body);
            world.step(0.01);
            RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 {}, "switching back to dynamic does not resurrect stale forces");
        }
    }

    RIGIDBODIES_TEST("impulses affect motion without being mistaken for force channels")
    {
        World world;
        disable_gravity(world);
        const auto id = world.create_body(disc());
        auto* body = world.find_body(id);
        body->override_mass(2.0);
        body->apply_linear_impulse({ 4.0, 0.0 });
        body->apply_angular_impulse(3.0);
        world.step(0.01);
        RIGIDBODIES_EXPECT_NEAR(body->linear_velocity_m_s().x, 2.0, 0.0, "linear impulse affects velocity");
        RIGIDBODIES_EXPECT(body->angular_velocity_rad_s() > 0.0, "angular impulse affects velocity");
        RIGIDBODIES_EXPECT(body->applied_force_n() == Vec2 {} && body->applied_force_channels().empty(), "impulses are not reported as forces");
        RIGIDBODIES_EXPECT_NEAR(body->applied_torque_n_m(), 0.0, 0.0, "angular impulse is not reported as torque");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
