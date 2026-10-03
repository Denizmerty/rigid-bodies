#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    class WakeLink final : public Constraint
    {
    public:
        WakeLink(BodyId first, BodyId second) : first_(first), second_(second)
        {
        }
        std::shared_ptr<Constraint> clone() const override
        {
            return std::make_shared<WakeLink>(*this);
        }
        std::string_view name() const override
        {
            return "wake_link";
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
        BodyId first_, second_;
    };

    World quiet_world()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.quiet_duration_s = 0.03;
        return World(settings);
    }

    BodyId body(World& world, Vec2 position, BodyType type = BodyType::dynamic_body)
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.type = type;
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(collider);
        return world.create_body(definition);
    }

    void settle(World& world)
    {
        for (int step = 0; step < 8; ++step)
            world.step(0.01);
    }

    RIGIDBODIES_TEST("wake graph propagates from the reverse end of a long constraint chain")
    {
        auto world = quiet_world();
        std::vector<BodyId> chain;
        for (int index = 0; index < 129; ++index)
        {
            chain.push_back(body(world, { index * 0.5 - 32.0, 0.0 }));
            if (chain.size() > 1)
                world.add_constraint(std::make_shared<WakeLink>(chain[chain.size() - 2], chain.back()));
        }
        const auto isolated = body(world, { 0.0, 10.0 });
        settle(world);
        RIGIDBODIES_EXPECT(world.statistics().sleeping_body_count == chain.size() + 1, "long quiet chain and disconnected body initially sleep");
        world.find_body(chain.back())->apply_linear_impulse({ 0.1, 0.0 });
        world.step(0.01);
        for (const auto id : chain)
            RIGIDBODIES_EXPECT(world.find_body(id)->is_awake(), "one queued propagation reaches every dynamic endpoint regardless of canonical direction");
        RIGIDBODIES_EXPECT(!world.find_body(isolated)->is_awake(), "graph traversal leaves an unconnected sleeper untouched");
    }

    RIGIDBODIES_TEST("wake graph does not relay dynamic edits through shared static or stationary kinematic boundaries")
    {
        for (const auto type : { BodyType::static_body, BodyType::kinematic_body })
        {
            auto world = quiet_world();
            const auto boundary = body(world, { 0.0, 0.0 }, type);
            const auto left = body(world, { -3.0, 0.0 });
            const auto right = body(world, { 3.0, 0.0 });
            world.add_constraint(std::make_shared<WakeLink>(boundary, left));
            world.add_constraint(std::make_shared<WakeLink>(boundary, right));
            settle(world);
            RIGIDBODIES_EXPECT(!world.find_body(left)->is_awake() && !world.find_body(right)->is_awake(), "both groups initially sleep at a shared fixed boundary");
            world.find_body(left)->apply_linear_impulse({ 0.1, 0.0 });
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(left)->is_awake() && !world.find_body(right)->is_awake(), "a dynamic seed cannot wake the other group through the boundary");
            world.find_body(boundary)->set_position({ 0.0, 1.0 });
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(right)->is_awake(), "an explicitly edited boundary itself seeds both linked groups");
        }
    }

    RIGIDBODIES_TEST("wake graph retains collider filters after dense body deletion and slot reuse")
    {
        for (int mode = 0; mode < 3; ++mode)
        {
            auto world = quiet_world();
            const auto removed = body(world, { -5.0, 0.0 });
            const auto sleeper = body(world, {});
            const auto unrelated = body(world, { 5.0, 0.0 });
            world.destroy_body(removed);
            BodyDefinition definition;
            definition.type = BodyType::kinematic_body;
            definition.position_m = { -1.0, 0.0 };
            Collider collider;
            collider.shape = make_circle(0.1);
            collider.is_sensor = mode == 1;
            if (mode == 2)
                collider.filter.mask = 0;
            definition.colliders.push_back(collider);
            const auto mover = world.create_body(definition);
            RIGIDBODIES_EXPECT(mover.index == removed.index && !(mover == removed), "candidate graph sees the recycled generation");
            settle(world);
            world.find_body(mover)->set_linear_velocity({ 100.0, 0.0 });
            world.step(0.01);
            RIGIDBODIES_EXPECT(world.find_body(sleeper)->is_awake() == (mode == 0), "only an eligible solid swept candidate propagates wake");
            RIGIDBODIES_EXPECT(!world.find_body(unrelated)->is_awake(), "compacted storage does not misroute the wake to another handle");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
