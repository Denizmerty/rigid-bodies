#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/body_properties.hpp>
#include <rigidbodies/physics/joint.hpp>

#include "test_framework.hpp"

#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    World empty_world()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        return World { settings };
    }

    BodyId add_body(World& world, BodyType type = BodyType::dynamic_body, math::Vec2 position = {})
    {
        BodyDefinition definition;
        definition.type = type;
        definition.position_m = position;
        definition.name = "editable";
        Collider collider;
        collider.shape = make_box(0.4, 0.2);
        collider.material = materials::oak_wood();
        definition.colliders.push_back(collider);
        return world.create_body(definition);
    }

    AuthoredPartDefinition authored_part(bool concave, Real density, math::Vec2 position = {})
    {
        Outline outline;
        outline.closed = true;
        const std::vector<math::Vec2> points = concave ? std::vector<math::Vec2> { { 0.0, 0.0 }, { 0.4, 0.0 }, { 0.4, 0.1 }, { 0.1, 0.1 }, { 0.1, 0.4 }, { 0.0, 0.4 } }
                                                       : std::vector<math::Vec2> { { -0.1, -0.1 }, { 0.1, -0.1 }, { 0.1, 0.1 }, { -0.1, 0.1 } };
        for (const auto point : points)
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        const auto built = build_authored_shape(outline);
        RIGIDBODIES_EXPECT(built.succeeded(), "valid test outline is available");
        AuthoredPartDefinition result;
        result.name = concave ? "Concave" : "Square";
        result.shape = built.shape;
        result.local_transform = math::Transform2::from_angle(position, 0.0);
        result.material = materials::oak_wood();
        result.density_override_kg_m3 = density;
        result.drag_coefficient_override = 0.42;
        return result;
    }

    template <typename Action>
    bool rejected(Action action)
    {
        try
        {
            action();
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }

    RIGIDBODIES_TEST("mass editing scales inertia preserves identity and retains attached physics")
    {
        auto world = empty_world();
        const auto id = add_body(world, BodyType::dynamic_body, { 1.0, 2.0 });
        const auto anchor_id = add_body(world, BodyType::static_body, { 1.0, 3.0 });
        auto generator = std::make_shared<PointAttractor>(math::Vec2 {}, 2.0);
        world.add_force_generator(id, generator);
        DistanceJointDefinition joint_definition;
        joint_definition.first = anchor_id;
        joint_definition.second = id;
        joint_definition.length_m = 1.0;
        auto joint = std::make_shared<JointConstraint>(joint_definition);
        world.add_constraint(joint, "retained_link");
        auto* body = world.find_body(id);
        body->set_linear_velocity({ 2.0, -1.0 });
        body->set_angular_velocity(0.4);
        const auto original = *body;
        BodyPropertyEdit edit;
        edit.mass_kg = original.mass_properties().mass_kg * 3.0;
        edit_body_properties(world, id, edit);
        RIGIDBODIES_EXPECT(world.find_body(id) == body, "property edits retain the body identity and live pointer");
        RIGIDBODIES_EXPECT_NEAR(body->mass_properties().inertia_kg_m2, original.mass_properties().inertia_kg_m2 * 3.0, 1.0e-13, "inertia scales with mass for unchanged geometry");
        RIGIDBODIES_EXPECT(body->has_mass_override() && body->position_m() == original.position_m() && body->orientation_rad() == original.orientation_rad(),
            "explicit mass override does not teleport or rotate the body");
        RIGIDBODIES_EXPECT(body->linear_velocity_m_s() == original.linear_velocity_m_s() && body->angular_velocity_rad_s() == original.angular_velocity_rad_s(),
            "property edits retain current velocities rather than silently applying conservation to external intervention");
        RIGIDBODIES_EXPECT(world.force_generators(id).size() == 1 && world.force_generators(id).front() == generator && world.constraints().front() == joint,
            "attachments remain live rather than being replaced with detached clones");
        RIGIDBODIES_EXPECT(body->interpolated_transform(0.0).translation == body->transform().translation,
            "paused edits synchronize render interpolation endpoints");
    }

    RIGIDBODIES_TEST("density-derived mass can be restored after a manual override")
    {
        auto world = empty_world();
        const auto id = add_body(world);
        const auto mass = world.find_body(id)->mass_properties();
        BodyPropertyEdit edit;
        edit.mass_kg = mass.mass_kg * 4.0;
        edit_body_properties(world, id, edit);
        edit = {};
        edit.use_density_mass = true;
        edit_body_properties(world, id, edit);
        const auto& body = *world.find_body(id);
        RIGIDBODIES_EXPECT(!body.has_mass_override(), "restoring density mode clears override state");
        RIGIDBODIES_EXPECT_NEAR(body.mass_properties().mass_kg, mass.mass_kg, 1.0e-14, "geometry and density restore initial mass");
        RIGIDBODIES_EXPECT_NEAR(body.mass_properties().inertia_kg_m2, mass.inertia_kg_m2, 1.0e-14, "original mass distribution restores inertia too");
    }

    RIGIDBODIES_TEST("material edits preserve authored grouping and geometry caches while freezing old snapshots")
    {
        auto world = empty_world();
        const auto id = create_authored_body(world, {}, { authored_part(true, 100.0), authored_part(false, 200.0, { 0.8, 0.0 }) }, "authored");
        const auto original_parts = authored_parts(*world.find_body(id));
        const auto original_mass = world.find_body(id)->mass_properties().mass_kg;
        const auto original_cells = world.find_body(id)->colliders().size();
        RIGIDBODIES_EXPECT(original_cells > 2, "concave logical part contains multiple collision cells");
        const auto snapshot = world.snapshot();
        BodyPropertyEdit edit;
        edit.material = materials::steel();
        edit_body_properties(world, id, edit);
        const auto changed_parts = authored_parts(*world.find_body(id));
        RIGIDBODIES_EXPECT(changed_parts.size() == 2 && world.find_body(id)->colliders().size() == original_cells, "editing a material does not turn convex cells into logical parts");
        for (std::size_t index = 0; index < changed_parts.size(); ++index)
        {
            const auto& changed = changed_parts[index];
            RIGIDBODIES_EXPECT(changed != original_parts[index] && changed->shape == original_parts[index]->shape,
                "each logical descriptor is replaced once while immutable tessellation is shared");
            RIGIDBODIES_EXPECT(changed->material.name == materials::steel().name && !changed->density_override_kg_m3 && !changed->drag_coefficient_override,
                "new material replaces authored overrides that would otherwise hide its effects");
            RIGIDBODIES_EXPECT(original_parts[index]->density_override_kg_m3.has_value() && original_parts[index]->material.name == materials::oak_wood().name,
                "previous immutable descriptors remain unchanged");
        }
        for (const auto& collider : world.find_body(id)->colliders())
        {
            RIGIDBODIES_EXPECT(collider.material.name == materials::steel().name && !collider.density_override_kg_m3 && !collider.drag_coefficient_override,
                "every convex collision cell uses the edited surface and density");
            RIGIDBODIES_EXPECT(collider.authored_part == changed_parts[0] || collider.authored_part == changed_parts[1],
                "convex cells still refer to one descriptor per logical source");
        }
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->mass_properties().mass_kg, (0.07 + 0.04) * 0.05 * materials::steel().density_kg_m3, 1.0e-10, "edited body mass is actually recomputed from new density");
        world.restore(snapshot);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(id)->mass_properties().mass_kg, original_mass, 0.0, "snapshot restores the old material's actual mass");
        const auto restored_parts = authored_parts(*world.find_body(id));
        RIGIDBODIES_EXPECT(restored_parts.size() == 2 && restored_parts[0]->material.name == materials::oak_wood().name && restored_parts[0]->density_override_kg_m3 == 100.0,
            "snapshot restores original authored provenance and per-part density");
    }

    RIGIDBODIES_TEST("changed centre of mass preserves the existing rigid velocity field")
    {
        auto world = empty_world();
        const auto id = create_authored_body(world, {}, { authored_part(false, 100.0, { -0.5, 0.0 }), authored_part(false, 900.0, { 0.5, 0.0 }) });
        auto* body = world.find_body(id);
        body->set_linear_velocity({ 1.0, 2.0 });
        body->set_angular_velocity(3.0);
        const auto before = *body;
        BodyPropertyEdit edit;
        edit.material = materials::rubber();
        edit_body_properties(world, id, edit);
        RIGIDBODIES_EXPECT_NEAR(body->world_center_of_mass_m().x, 0.0, 1.0e-12, "uniform replacement density centres the mass between equal parts");
        const auto expected_velocity = before.velocity_at_world_point(body->world_center_of_mass_m());
        RIGIDBODIES_EXPECT(body->linear_velocity_m_s() == expected_velocity && body->angular_velocity_rad_s() == before.angular_velocity_rad_s(),
            "new COM velocity samples the original rigid velocity field");
        RIGIDBODIES_EXPECT_NEAR(body->velocity_at_world_point({ -0.5, 0.0 }).y, before.velocity_at_world_point({ -0.5, 0.0 }).y, 1.0e-12, "a material change does not invent velocity at an unchanged material point");
    }

    RIGIDBODIES_TEST("property edit rejection is atomic across material mass and velocity fields")
    {
        auto world = empty_world();
        const auto id = add_body(world);
        auto* body = world.find_body(id);
        const auto original = *body;
        for (int problem = 0; problem < 5; ++problem)
        {
            BodyPropertyEdit edit;
            edit.material = materials::steel();
            edit.mass_kg = 2.0;
            edit.linear_velocity_m_s = math::Vec2 { 1.0, 2.0 };
            if (problem == 0)
                edit.mass_kg = -1.0;
            if (problem == 1)
                edit.material->restitution = 1.01;
            if (problem == 2)
                edit.material->density_kg_m3 = std::numeric_limits<Real>::quiet_NaN();
            if (problem == 3)
                edit.linear_velocity_m_s = math::Vec2 { 101.0, 0.0 };
            if (problem == 4)
                edit.angular_velocity_rad_s = std::numeric_limits<Real>::infinity();
            RIGIDBODIES_EXPECT(rejected([&]()
                                   {
                                       edit_body_properties(world, id, edit);
                                   }),
                "invalid member rejects the complete proposed edit");
            RIGIDBODIES_EXPECT(body->mass_properties().mass_kg == original.mass_properties().mass_kg && body->mass_properties().inertia_kg_m2 == original.mass_properties().inertia_kg_m2 &&
                    body->colliders().front().material.name == original.colliders().front().material.name && body->linear_velocity_m_s() == original.linear_velocity_m_s(),
                "failure does not leak any previously valid part of a compound edit");
        }
        BodyPropertyEdit massless;
        massless.material = materials::steel();
        massless.material->density_kg_m3 = 0.0;
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, id, massless);
                               }),
            "a dynamic body cannot lose all mass or inertia");
        RIGIDBODIES_EXPECT(body->colliders().front().material.name == original.colliders().front().material.name,
            "failure after candidate mass rebuild remains atomic");
    }

    RIGIDBODIES_TEST("velocity editing validates vector magnitude spin limits and fixed rotation")
    {
        auto world = empty_world();
        auto settings = world.settings();
        settings.limits.maximum_linear_speed_m_s = 5.0;
        settings.limits.maximum_angular_speed_rad_s = 3.0;
        world.set_settings(settings);
        const auto id = add_body(world);
        BodyPropertyEdit edit;
        edit.linear_velocity_m_s = math::Vec2 { 3.0, 4.0 };
        edit.angular_velocity_rad_s = -3.0;
        edit_body_properties(world, id, edit);
        RIGIDBODIES_EXPECT(world.find_body(id)->linear_velocity_m_s() == math::Vec2 { 3.0, 4.0 } && world.find_body(id)->angular_velocity_rad_s() == -3.0,
            "speed limits admit finite values on the boundary");
        edit.linear_velocity_m_s = math::Vec2 { 4.0, 4.0 };
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, id, edit);
                               }),
            "each component below the limit does not admit an excessive vector magnitude");
        edit.linear_velocity_m_s.reset();
        edit.angular_velocity_rad_s = 3.001;
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, id, edit);
                               }),
            "spin magnitude is bounded independently");
        world.find_body(id)->set_fixed_rotation(true);
        edit.angular_velocity_rad_s = 0.1;
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, id, edit);
                               }),
            "fixed rotation cannot silently ignore requested spin");
        edit.angular_velocity_rad_s = 0.0;
        edit_body_properties(world, id, edit);
        RIGIDBODIES_EXPECT(world.find_body(id)->angular_velocity_rad_s() == 0.0, "zero spin remains legal for fixed rotation");
    }

    RIGIDBODIES_TEST("static surfaces accept materials while unsupported modes and stale identifiers reject edits")
    {
        auto world = empty_world();
        const auto static_id = add_body(world, BodyType::static_body);
        BodyPropertyEdit edit;
        edit.material = materials::rubber();
        edit_body_properties(world, static_id, edit);
        RIGIDBODIES_EXPECT(world.find_body(static_id)->colliders().front().material.name == materials::rubber().name && world.find_body(static_id)->inverse_mass() == 0.0,
            "a fixed support can change contact material while remaining immovable");
        edit.mass_kg = 2.0;
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, static_id, edit);
                               }),
            "static bodies do not accept dynamic mass edits");
        edit = {};
        edit.linear_velocity_m_s = math::Vec2 {};
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, static_id, edit);
                               }),
            "static bodies do not accept a velocity command even when zero");
        const auto kinematic_id = add_body(world, BodyType::kinematic_body);
        edit = {};
        edit.material = materials::steel();
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, kinematic_id, edit);
                               }),
            "prescribed state is not edited through dynamic controls");
        const auto stale_id = add_body(world);
        world.destroy_body(stale_id);
        const auto replacement_id = add_body(world);
        RIGIDBODIES_EXPECT(rejected([&]()
                               {
                                   edit_body_properties(world, stale_id, edit);
                               }),
            "slot reuse cannot redirect a stale body selection");
        RIGIDBODIES_EXPECT(world.find_body(replacement_id)->colliders().front().material.name == materials::oak_wood().name,
            "rejected stale selection leaves the replacement body untouched");
    }

    RIGIDBODIES_TEST("successful surface edit invalidates warm contacts before the next solve")
    {
        World world;
        const auto ground = add_body(world, BodyType::static_body, { 0.0, -0.1 });
        const auto block = add_body(world, BodyType::dynamic_body, { 0.0, 0.1 });
        for (int index = 0; index < 60; ++index)
            world.step(1.0 / 120.0);
        RIGIDBODIES_EXPECT(!world.manifolds().empty(), "supported block has cached contact impulses before editing");
        BodyPropertyEdit edit;
        edit.material = materials::rubber();
        edit_body_properties(world, ground, edit);
        for (const auto& manifold : world.manifolds())
            RIGIDBODIES_EXPECT(!(manifold.first == ground) && !(manifold.second == ground), "stale pair material and warm impulses are removed immediately");
        world.step(1.0 / 120.0);
        RIGIDBODIES_EXPECT(!world.manifolds().empty() && math::is_finite(world.find_body(block)->linear_velocity_m_s()),
            "contact regenerates safely against the updated material");
        RIGIDBODIES_EXPECT(world.statistics().last_step_limit_event_count == 0, "surface edits do not require motion clamps");
    }
} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
