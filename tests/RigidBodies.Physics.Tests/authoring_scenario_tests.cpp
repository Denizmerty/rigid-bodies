#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;
    using namespace rigidbodies::physics;

    BodyId named_body(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == name)
                return id;
        RIGIDBODIES_FAIL("workshop contains the named example");
    }

    RIGIDBODIES_TEST("shape workshop supplies concave curved and multi-material editable examples")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "shape_workshop"), "workshop is registered");
        RIGIDBODIES_EXPECT(world.body_ids().size() == 4 && !world.compute_bounds().is_empty(), "three examples and a floor provide framing bounds");
        const auto* bracket = world.find_body(named_body(world, "Concave bracket"));
        RIGIDBODIES_EXPECT(authored_parts(*bracket).size() == 1 && bracket->colliders().size() > 1, "concave outline remains one editable logical part across convex cells");
        const auto curve = authored_parts(*world.find_body(named_body(world, "Curved profile"))).front()->shape;
        RIGIDBODIES_EXPECT(curve->source.nodes.size() == 4 && curve->source.nodes.front().outgoing_edge == OutlineEdgeKind::cubic, "original cubic control nodes survive compilation");
        RIGIDBODIES_EXPECT(curve->render_outline.size() > curve->collision_outline.size(), "independent error tolerances give the visible curve finer detail");
        const auto parts = authored_parts(*world.find_body(named_body(world, "Two material tool")));
        RIGIDBODIES_EXPECT(parts.size() == 2 && parts[0]->material.name != parts[1]->material.name, "compound retains independent wood and steel properties");
    }

    RIGIDBODIES_TEST("workshop shapes collide with the floor and retain their compiled geometry")
    {
        World world;
        load_scenario(world, "shape_workshop");
        const auto bracket = named_body(world, "Concave bracket");
        const auto curve = named_body(world, "Curved profile");
        const auto tool = named_body(world, "Two material tool");
        const auto cached = authored_parts(*world.find_body(bracket)).front()->shape;
        bool made_contact = false;
        for (int frame = 0; frame < 300; ++frame)
        {
            world.step(1.0 / 120.0);
            for (const auto id : { bracket, curve, tool })
            {
                const auto* body = world.find_body(id);
                RIGIDBODIES_EXPECT(math::is_finite(body->position_m()) && math::is_finite(body->linear_velocity_m_s()) && math::is_finite(body->orientation_rad()), "authored bodies remain finite under gravity and collision");
                RIGIDBODIES_EXPECT(body->world_center_of_mass_m().y > -1.5, "authored bodies do not fall through the floor");
            }
            made_contact = made_contact || !world.manifolds().empty();
        }
        RIGIDBODIES_EXPECT(made_contact, "the examples exercise collision response");
        RIGIDBODIES_EXPECT(authored_parts(*world.find_body(bracket)).front()->shape == cached, "stepping never rebuilds authored geometry");
    }

    RIGIDBODIES_TEST("workshop separation returns logical materials and resetting restores the compound")
    {
        World world;
        load_scenario(world, "shape_workshop");
        const auto tool = named_body(world, "Two material tool");
        const auto checkpoint = world.snapshot();
        const auto mass = world.find_body(tool)->mass_properties().mass_kg;
        const auto originals = authored_parts(*world.find_body(tool));
        const auto split = separate_authored_body(world, tool);
        RIGIDBODIES_EXPECT(split.size() == 2 && !world.is_valid(tool), "separation produces two logical pieces and removes the assembly");
        Real separated_mass = 0.0;
        for (std::size_t index = 0; index < split.size(); ++index)
        {
            const auto* body = world.find_body(split[index]);
            separated_mass += body->mass_properties().mass_kg;
            const auto parts = authored_parts(*body);
            RIGIDBODIES_EXPECT(parts.size() == 1 && parts.front()->material.name == originals[index]->material.name, "each separated part keeps its material");
        }
        RIGIDBODIES_EXPECT_NEAR(separated_mass, mass, 1.0e-10, "touching nonoverlapping pieces preserve total mass");
        world.restore(checkpoint);
        RIGIDBODIES_EXPECT(world.is_valid(tool) && authored_parts(*world.find_body(tool)).size() == 2, "snapshot resets the authored assembly");
        RIGIDBODIES_EXPECT(world.body_ids().size() == 4, "reset removes separated children");
    }

    RIGIDBODIES_TEST("workshop snapshot replay reproduces collision trajectories with shared immutable outlines")
    {
        World world;
        load_scenario(world, "shape_workshop");
        const auto checkpoint = world.snapshot();
        const auto ids = world.body_ids();
        for (int frame = 0; frame < 150; ++frame)
            world.step(1.0 / 120.0);
        std::vector<math::Vec2> positions;
        std::vector<Real> angles;
        for (const auto id : ids)
        {
            positions.push_back(world.find_body(id)->position_m());
            angles.push_back(world.find_body(id)->orientation_rad());
        }
        world.restore(checkpoint);
        for (int frame = 0; frame < 150; ++frame)
            world.step(1.0 / 120.0);
        for (std::size_t index = 0; index < ids.size(); ++index)
        {
            RIGIDBODIES_EXPECT(world.find_body(ids[index])->position_m() == positions[index], "restored authored geometry replays positions exactly");
            RIGIDBODIES_EXPECT(world.find_body(ids[index])->orientation_rad() == angles[index], "restored authored geometry replays rotations exactly");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
