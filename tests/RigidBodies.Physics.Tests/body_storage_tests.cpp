#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    static_assert(!std::is_copy_constructible_v<World> && !std::is_copy_assignable_v<World>, "World copies must use independent snapshots");
    static_assert(std::is_nothrow_move_constructible_v<World> && std::is_nothrow_move_assignable_v<World>, "Transactions publish by moving a complete world");

    BodyDefinition ball(std::string name, Real x = 0.0)
    {
        BodyDefinition definition;
        definition.name = std::move(name);
        definition.position_m = { x, 0.0 };
        Collider collider;
        collider.shape = make_circle(0.1);
        definition.colliders.push_back(std::move(collider));
        return definition;
    }

    void expect_packed(const World& world)
    {
        const auto packed = world.contiguous_bodies();
        const auto ids = world.body_ids();
        RIGIDBODIES_EXPECT(packed.size() == ids.size(), "packed storage contains every live body and no vacant slots");
        RIGIDBODIES_EXPECT(world.body_storage_capacity() >= packed.size(), "capacity covers the live contiguous range");
        std::vector<const RigidBody*> addressed;
        for (const auto id : ids)
        {
            const auto* body = world.find_body(id);
            const auto entry = std::find_if(packed.begin(), packed.end(), [body](const auto& candidate)
                {
                    return &candidate == body;
                });
            RIGIDBODIES_EXPECT(entry != packed.end(), "every generation handle resolves into the same contiguous array");
            RIGIDBODIES_EXPECT(std::find(addressed.begin(), addressed.end(), body) == addressed.end(), "distinct identifiers address distinct entries");
            addressed.push_back(body);
        }
    }

    RIGIDBODIES_TEST("packed body growth preserves handles and all body state")
    {
        World world;
        std::vector<BodyId> ids;
        for (int index = 0; index < 257; ++index)
        {
            ids.push_back(world.create_body(ball(std::to_string(index), index * 0.25)));
            world.find_body(ids.back())->override_mass(index + 1.0);
            world.find_body(ids.back())->apply_force_at_center({ index * 0.5, 3.0 }, "pending");
        }
        expect_packed(world);
        for (std::size_t index = 0; index < ids.size(); ++index)
        {
            const auto* body = world.find_body(ids[index]);
            RIGIDBODIES_EXPECT(body && body->name() == std::to_string(index), "growth preserves the body associated with each stable handle");
            RIGIDBODIES_EXPECT_NEAR(body->mass_properties().mass_kg, index + 1.0, 0.0, "relocation retains mass overrides");
            RIGIDBODIES_EXPECT(body->accumulated_force_n() == Vec2 { index * 0.5, 3.0 }, "relocation retains pending loads");
        }
        RIGIDBODIES_EXPECT(world.body_ids() == ids, "packed allocation does not change creation ordering");
    }

    RIGIDBODIES_TEST("swap removal repairs handles and retains moved body attachments")
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        World world(settings);
        const auto first = world.create_body(ball("first", -5.0), "c");
        const auto hole = world.create_body(ball("hole", 0.0), "b");
        auto driven = ball("driven", 5.0);
        driven.type = BodyType::kinematic_body;
        const auto moved = world.create_body(driven, "a");
        const auto force = std::make_shared<PointAttractor>(Vec2 { 5.0, 0.0 }, 2.0);
        RIGIDBODIES_EXPECT(world.add_force_generator(moved, force), "attachment accepted");
        RIGIDBODIES_EXPECT(world.set_kinematic_motion(moved, KinematicMotion(LinearMotion { { 5.0, 0.0 }, 0.0, { 1.0, 0.0 }, 0.0 })), "drive accepted");
        RIGIDBODIES_EXPECT(world.destroy_body(hole), "middle body removed");
        expect_packed(world);
        RIGIDBODIES_EXPECT(!world.is_valid(hole) && world.find_body(hole) == nullptr, "removed generation stays invalid after compaction");
        RIGIDBODIES_EXPECT(world.find_body(moved)->name() == "driven", "moved body is still addressed by its old handle");
        RIGIDBODIES_EXPECT(world.force_generators(moved).size() == 1 && world.force_generators(moved).front() == force, "attachments remain keyed to the moved handle");
        RIGIDBODIES_EXPECT(world.kinematic_motion(moved) != nullptr, "kinematic drive remains keyed to its body");
        const auto replacement = world.create_body(ball("replacement", 0.0), "d");
        RIGIDBODIES_EXPECT(replacement.index == hole.index && !(replacement == hole), "vacant slot receives a fresh generation");
        RIGIDBODIES_EXPECT(world.force_generators(replacement).empty() && world.kinematic_motion(replacement) == nullptr, "recycled handles do not inherit attachments");
        RIGIDBODIES_EXPECT(world.body_ids() == std::vector<BodyId> { moved, first, replacement }, "canonical keys remain independent of physical storage order");
        world.step(0.1);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(moved)->position_m().x, 5.1, 1.0e-12, "the relocated kinematic body advances its own path");
    }

    RIGIDBODIES_TEST("repeated arbitrary compaction and slot reuse never aliases stale generations")
    {
        World world;
        std::vector<BodyId> live, stale;
        for (int index = 0; index < 31; ++index)
            live.push_back(world.create_body(ball(std::to_string(index))));
        for (std::size_t iteration = 0; iteration < 127; ++iteration)
        {
            const auto index = (iteration * 13) % live.size();
            const auto removed = live[index];
            RIGIDBODIES_EXPECT(world.destroy_body(removed), "arbitrary live entry is removed");
            stale.push_back(removed);
            live.erase(live.begin() + static_cast<std::ptrdiff_t>(index));
            live.push_back(world.create_body(ball("new " + std::to_string(iteration))));
            expect_packed(world);
            RIGIDBODIES_EXPECT(world.body_ids() == live, "swap removal does not alter canonical survivor order");
            for (const auto id : stale)
                RIGIDBODIES_EXPECT(!world.is_valid(id) && !world.destroy_body(id), "all abandoned generations remain rejected");
        }
    }

    RIGIDBODIES_TEST("snapshots restore packed layout holes and subsequent allocation exactly")
    {
        World world;
        const auto first = world.create_body(ball("first"));
        const auto removed = world.create_body(ball("removed"));
        const auto moved = world.create_body(ball("moved"));
        world.destroy_body(removed);
        world.find_body(moved)->set_linear_velocity({ 2.0, 3.0 });
        const auto snapshot = world.snapshot();
        const auto next = world.create_body(ball("future"));
        world.destroy_body(first);
        world.clear();
        world.restore(snapshot);
        expect_packed(world);
        RIGIDBODIES_EXPECT(world.body_ids() == std::vector<BodyId> { first, moved }, "restore retains canonical identifiers around vacant slots");
        RIGIDBODIES_EXPECT(!world.is_valid(removed) && !world.is_valid(next), "snapshot rejects generations from the abandoned future");
        RIGIDBODIES_EXPECT(world.find_body(moved)->linear_velocity_m_s() == Vec2 { 2.0, 3.0 }, "restored dense mapping reaches the correct saved body");
        World independent;
        independent.restore(snapshot);
        RIGIDBODIES_EXPECT(independent.find_body(moved)->colliders()[0].shape != world.find_body(moved)->colliders()[0].shape, "independent packed restores still deep clone geometry");
        world.find_body(moved)->set_name("edited");
        RIGIDBODIES_EXPECT(independent.find_body(moved)->name() == "moved", "restored packed bodies do not share mutable state");
        RIGIDBODIES_EXPECT(world.create_body(ball("future")) == next, "snapshot preserves the free slot and generation sequence");
    }

    RIGIDBODIES_TEST("failed body validation and duplicate keys leave packed allocation unchanged")
    {
        World world;
        const auto original = world.create_body(ball("original"), "unique");
        const auto checkpoint = world.snapshot();
        World expected;
        expected.restore(checkpoint);
        const auto expected_next = expected.create_body(ball("next"));
        auto invalid = ball("invalid");
        invalid.gravity_scale = std::numeric_limits<Real>::quiet_NaN();
        for (int index = 0; index < 2; ++index)
        {
            bool rejected = false;
            try
            {
                (void)world.create_body(index == 0 ? invalid : ball("duplicate"), index == 0 ? "invalid" : "unique");
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "invalid body creation is rejected before publishing an entry");
            expect_packed(world);
            RIGIDBODIES_EXPECT(world.body_ids() == std::vector<BodyId> { original }, "failure preserves live handles");
        }
        RIGIDBODIES_EXPECT(world.create_body(ball("next")) == expected_next, "failed creation consumes neither a generation nor a slot");
    }

    RIGIDBODIES_TEST("authored edits retain packed mapping after unrelated compaction")
    {
        World world;
        const auto spare = world.create_body(ball("spare", -10.0));
        Outline outline;
        outline.closed = true;
        for (const Vec2 point : { Vec2 { 0.0, 0.0 }, Vec2 { 1.0, 0.0 }, Vec2 { 1.0, 1.0 }, Vec2 { 0.0, 1.0 } })
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        AuthoredPartDefinition part;
        part.shape = build_authored_shape(outline).shape;
        BodyDefinition definition;
        definition.name = "editable";
        const auto authored = create_authored_body(world, definition, { part }, "editable");
        const auto other = world.create_body(ball("other", 10.0));
        world.destroy_body(spare);
        outline.nodes[1].position_m.x = 2.0;
        outline.nodes[2].position_m.x = 2.0;
        edit_authored_part(world, authored, 0, build_authored_shape(outline).shape);
        expect_packed(world);
        RIGIDBODIES_EXPECT(world.is_valid(authored) && world.find_body(authored)->name() == "editable", "transaction preserves the edited generation handle");
        RIGIDBODIES_EXPECT(world.find_body(other)->name() == "other", "transaction preserves the independently relocated body");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(authored)->compute_bounds().extents().x, 2.0, 1.0e-12, "replacement reaches the intended dense body");
    }

    RIGIDBODIES_TEST("clear drops packed contents while preserving stale handle rejection")
    {
        World world;
        const auto first = world.create_body(ball("first"));
        const auto second = world.create_body(ball("second"));
        world.clear();
        expect_packed(world);
        RIGIDBODIES_EXPECT(world.contiguous_bodies().empty() && !world.is_valid(first) && !world.is_valid(second), "clear releases all live entries");
        const auto replacement = world.create_body(ball("replacement"));
        RIGIDBODIES_EXPECT(!(replacement == first) && !(replacement == second), "clear preserves the generation allocator");
        expect_packed(world);
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
