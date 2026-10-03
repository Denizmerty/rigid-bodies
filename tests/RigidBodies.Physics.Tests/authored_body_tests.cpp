#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    std::shared_ptr<const AuthoredShape> profile(bool concave = false, Real width = 1.0)
    {
        Outline outline;
        outline.closed = true;
        const std::vector<Vec2> points = concave ? std::vector<Vec2> { { 0, 0 }, { 2, 0 }, { 2, 1 }, { 1, 1 }, { 1, 2 }, { 0, 2 } }
                                                 : std::vector<Vec2> { { 0, 0 }, { width, 0 }, { width, 1 }, { 0, 1 } };
        for (const auto point : points)
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        const auto result = build_authored_shape(outline);
        if (!result.succeeded())
            throw std::runtime_error("The test profile could not be authored");
        return result.shape;
    }

    AuthoredPartDefinition part(bool concave = false, Real density = 10.0)
    {
        AuthoredPartDefinition value;
        value.shape = profile(concave);
        value.material.density_kg_m3 = density;
        value.material.name = density > 10.0 ? "Dense material" : "Light material";
        value.depth_m = 0.1;
        return value;
    }

    World empty_world()
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        return World(settings);
    }

    BodyId add(World& world, Vec2 position, const AuthoredPartDefinition& value = part(), std::string key = {})
    {
        BodyDefinition definition;
        definition.name = key;
        definition.position_m = position;
        return create_authored_body(world, definition, { value }, std::move(key));
    }

    template <typename Action>
    bool rejected(Action&& action)
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

    Real total_angular_momentum(const World& world)
    {
        Real result = 0.0;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                result += body.angular_momentum_about_center_kg_m2_s() + rigidbodies::math::cross(body.world_center_of_mass_m(), body.linear_momentum_kg_m_s());
            });
        return result;
    }

    Vec2 total_momentum(const World& world)
    {
        Vec2 result;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                result += body.linear_momentum_kg_m_s();
            });
        return result;
    }

    std::vector<Vec2> visible_vertices(const RigidBody& body)
    {
        std::vector<Vec2> points;
        for (const auto& logical : authored_parts(body))
            for (const auto vertex : logical->shape->render_outline)
                points.push_back(rigidbodies::math::transform_point(rigidbodies::math::concatenate(body.transform(), logical->local_transform), vertex));
        return points;
    }

    void same_vertices(const std::vector<Vec2>& first, const std::vector<Vec2>& second)
    {
        RIGIDBODIES_EXPECT(first.size() == second.size(), "authored boundary vertex count is preserved");
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            RIGIDBODIES_EXPECT_NEAR(first[index].x, second[index].x, 1.0e-12, "world boundary x placement is preserved");
            RIGIDBODIES_EXPECT_NEAR(first[index].y, second[index].y, 1.0e-12, "world boundary y placement is preserved");
        }
    }

    RIGIDBODIES_TEST("concave collision cells retain one immutable authored part and exact composite mass")
    {
        auto world = empty_world();
        const auto id = add(world, {}, part(true));
        const auto& body = *world.find_body(id);
        const auto logical = authored_parts(body);
        RIGIDBODIES_EXPECT(body.colliders().size() > 1 && logical.size() == 1, "a concave source decomposes without becoming several logical parts");
        for (const auto& collider : body.colliders())
        {
            RIGIDBODIES_EXPECT(collider.authored_part == logical.front(), "all convex cells share exactly one immutable provenance descriptor");
            RIGIDBODIES_EXPECT(collider.material.name == "Light material", "material belongs to every collision cell");
        }
        RIGIDBODIES_EXPECT_NEAR(body.mass_properties().mass_kg, 3.0, 1.0e-12, "convex decomposition preserves concave profile mass");
        RIGIDBODIES_EXPECT_NEAR(body.mass_properties().center_of_mass_m.x, 5.0 / 6.0, 1.0e-12, "compound mass uses the exact concave centroid");
        RIGIDBODIES_EXPECT(!body.contains_world_point({ 1.5, 1.5 }), "the concavity remains absent from collision and picking geometry");
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)separate_authored_body(world, id);
                               }),
            "one logical part cannot split into convex cells");
    }

    RIGIDBODIES_TEST("committing a source creates a detached immutable cache instead of trusting fabricated cells")
    {
        auto world = empty_world();
        auto mutable_shape = std::make_shared<AuthoredShape>(*profile());
        mutable_shape->convex_parts.clear();
        auto definition = part();
        definition.shape = mutable_shape;
        const auto id = add(world, {}, definition);
        const auto frozen = authored_parts(*world.find_body(id)).front()->shape;
        RIGIDBODIES_EXPECT(frozen.get() != mutable_shape.get() && !frozen->convex_parts.empty(), "source and options rebuild authoritative collision buffers");
        mutable_shape->source.nodes[0].position_m = { 1000.0, 1000.0 };
        RIGIDBODIES_EXPECT(frozen->source.nodes[0].position_m == Vec2 {}, "later caller mutation cannot alter live or snapshot provenance");
    }

    RIGIDBODIES_TEST("editing preserves identity frame material settings and the old rigid velocity field")
    {
        auto world = empty_world();
        auto value = part();
        value.local_transform = rigidbodies::math::Transform2::from_angle({ 0.3, -0.2 }, 0.4);
        value.filter.group = -7;
        value.density_override_kg_m3 = 20.0;
        value.drag_coefficient_override = 0.8;
        const auto id = add(world, { 1.0, 2.0 }, value, "edited");
        world.find_body(id)->set_orientation(0.7);
        world.find_body(id)->set_linear_velocity({ 2.0, -1.0 });
        world.find_body(id)->set_angular_velocity(3.0);
        world.find_body(id)->set_gravity_scale(0.5);
        const auto before = *world.find_body(id);
        edit_authored_part(world, id, 0, profile(false, 2.0));
        const auto& after = *world.find_body(id);
        const auto logical = authored_parts(after).front();
        RIGIDBODIES_EXPECT(after.position_m() == before.position_m() && after.orientation_rad() == before.orientation_rad() && after.name() == before.name(), "editing preserves live identity and body frame");
        RIGIDBODIES_EXPECT(logical->material.name == value.material.name && logical->depth_m == value.depth_m && logical->density_override_kg_m3 == value.density_override_kg_m3 && logical->drag_coefficient_override == value.drag_coefficient_override && logical->filter.group == -7, "authored part material and contact properties survive source editing");
        const auto expected = before.velocity_at_world_point(after.world_center_of_mass_m());
        RIGIDBODIES_EXPECT(after.linear_velocity_m_s() == expected && after.angular_velocity_rad_s() == 3.0 && after.gravity_scale() == 0.5, "the changed COM follows the same rigid velocity field");
        RIGIDBODIES_EXPECT_NEAR(after.mass_properties().mass_kg, 4.0, 1.0e-12, "edited mass follows geometry and retained part density");
    }

    RIGIDBODIES_TEST("assembly preserves world geometry materials and total linear and angular momentum")
    {
        auto world = empty_world();
        const auto a = add(world, { -2.0, 0.0 }, part(true), "a");
        const auto b = add(world, { 2.0, 1.0 }, part(false, 30.0), "b");
        world.find_body(a)->set_orientation(0.2);
        world.find_body(b)->set_orientation(-0.3);
        world.find_body(a)->set_linear_velocity({ 2.0, 1.0 });
        world.find_body(b)->set_linear_velocity({ -1.0, 3.0 });
        world.find_body(a)->set_angular_velocity(0.7);
        world.find_body(b)->set_angular_velocity(-0.4);
        auto vertices = visible_vertices(*world.find_body(a));
        const auto second_vertices = visible_vertices(*world.find_body(b));
        vertices.insert(vertices.end(), second_vertices.begin(), second_vertices.end());
        const auto momentum = total_momentum(world);
        const auto angular = total_angular_momentum(world);
        const auto original_part = authored_parts(*world.find_body(a)).front();
        const auto assembled = assemble_authored_bodies(world, { b, a }, "Assembly", "combined");
        RIGIDBODIES_EXPECT(!world.is_valid(a) && !world.is_valid(b) && world.is_valid(assembled), "assembly replaces source IDs only after it succeeds");
        const auto& body = *world.find_body(assembled);
        same_vertices(vertices, visible_vertices(body));
        const auto logical = authored_parts(body);
        RIGIDBODIES_EXPECT(logical.size() == 2 && body.colliders().size() > logical.size(), "assembly retains two logical sources despite concave decomposition");
        RIGIDBODIES_EXPECT(logical[0] != original_part && logical[0]->shape == original_part->shape, "assembly relocates an independent descriptor and reuses immutable geometry");
        RIGIDBODIES_EXPECT(logical[0]->material.name != logical[1]->material.name, "individual materials are retained");
        RIGIDBODIES_EXPECT_NEAR(body.mass_properties().mass_kg, 6.0, 1.0e-11, "separated material volumes add exactly");
        RIGIDBODIES_EXPECT_NEAR(total_momentum(world).x, momentum.x, 1.0e-11, "assembly conserves horizontal momentum");
        RIGIDBODIES_EXPECT_NEAR(total_momentum(world).y, momentum.y, 1.0e-11, "assembly conserves vertical momentum");
        RIGIDBODIES_EXPECT_NEAR(total_angular_momentum(world), angular, 1.0e-10, "assembly conserves spin plus orbital angular momentum");
    }

    RIGIDBODIES_TEST("separation restores logical parts with exact world placements and rigid velocities")
    {
        auto world = empty_world();
        const auto a = add(world, { -2.0, 0.0 }, part(true), "a");
        const auto b = add(world, { 2.0, 0.0 }, part(false, 30.0), "b");
        const auto assembled = assemble_authored_bodies(world, { a, b });
        world.find_body(assembled)->set_linear_velocity({ 1.0, 2.0 });
        world.find_body(assembled)->set_angular_velocity(2.0);
        const auto source = *world.find_body(assembled);
        const auto vertices = visible_vertices(source);
        const auto momentum = total_momentum(world);
        const auto angular = total_angular_momentum(world);
        const auto separated = separate_authored_body(world, assembled);
        RIGIDBODIES_EXPECT(separated.size() == 2 && !world.is_valid(assembled), "separation follows authored parts rather than collision cell count");
        std::vector<Vec2> after;
        for (const auto id : separated)
        {
            const auto& body = *world.find_body(id);
            RIGIDBODIES_EXPECT(authored_parts(body).size() == 1, "every child has one editable source");
            const auto expected = source.velocity_at_world_point(body.world_center_of_mass_m());
            RIGIDBODIES_EXPECT(body.linear_velocity_m_s() == expected && body.angular_velocity_rad_s() == source.angular_velocity_rad_s(), "each part inherits the rigid velocity field at its own COM");
            const auto child_vertices = visible_vertices(body);
            after.insert(after.end(), child_vertices.begin(), child_vertices.end());
        }
        same_vertices(vertices, after);
        RIGIDBODIES_EXPECT_NEAR(total_momentum(world).x, momentum.x, 1.0e-11, "non-overlapping separation preserves x momentum");
        RIGIDBODIES_EXPECT_NEAR(total_momentum(world).y, momentum.y, 1.0e-11, "non-overlapping separation preserves y momentum");
        RIGIDBODIES_EXPECT_NEAR(total_angular_momentum(world), angular, 1.0e-10, "non-overlapping separation preserves angular momentum");
    }

    RIGIDBODIES_TEST("overlap keeps ordered volume ownership while assembly preserves incoming momentum")
    {
        auto world = empty_world();
        const auto a = add(world, {}, part(false, 10.0), "a");
        const auto b = add(world, {}, part(false, 30.0), "b");
        world.find_body(a)->set_linear_velocity({ 1.0, 0.0 });
        world.find_body(b)->set_linear_velocity({ 1.0, 0.0 });
        const auto assembled = assemble_authored_bodies(world, { b, a });
        RIGIDBODIES_EXPECT_NEAR(world.find_body(assembled)->mass_properties().mass_kg, 1.0, 1.0e-12, "the earlier canonical material owns shared volume once");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(assembled)->linear_velocity_m_s().x, 4.0, 1.0e-12, "geometry-defined merged mass still carries all incoming momentum");
        const auto children = separate_authored_body(world, assembled);
        RIGIDBODIES_EXPECT_NEAR(world.find_body(children[0])->mass_properties().mass_kg + world.find_body(children[1])->mass_properties().mass_kg, 4.0, 1.0e-12, "separate logical parts regain their own complete material volume");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(children[0])->linear_velocity_m_s().x, 4.0, 0.0, "separation preserves the parent rigid velocity, including overlap ownership semantics");
    }

    RIGIDBODIES_TEST("snapshots preserve immutable provenance and independently restore source edits and allocator history")
    {
        auto world = empty_world();
        const auto id = add(world, {}, part(true), "shape");
        const auto logical = authored_parts(*world.find_body(id)).front();
        const auto collision_shape = world.find_body(id)->colliders().front().shape;
        const auto checkpoint = world.snapshot();
        edit_authored_part(world, id, 0, profile());
        const auto future = add(world, { 3.0, 0.0 });
        world.restore(checkpoint);
        const auto restored = authored_parts(*world.find_body(id));
        RIGIDBODIES_EXPECT(restored.size() == 1 && restored[0] == logical && world.find_body(id)->colliders().size() > 1, "snapshot shares immutable provenance and restores the original concave source");
        for (const auto& collider : world.find_body(id)->colliders())
            RIGIDBODIES_EXPECT(collider.authored_part == logical, "restored convex cells retain their common logical identity");
        RIGIDBODIES_EXPECT(world.find_body(id)->colliders().front().shape != collision_shape, "mutable shape components follow the existing independent snapshot contract");
        const auto replay = add(world, { 3.0, 0.0 });
        RIGIDBODIES_EXPECT(replay == future, "body edit transactions preserve deterministic identifier allocation");
    }

    RIGIDBODIES_TEST("invalid outlines materials and stable keys leave the live body and allocation unchanged")
    {
        auto world = empty_world();
        const auto id = add(world, {}, part(), "live");
        const auto* original = world.find_body(id);
        const auto logical = authored_parts(*original).front();
        auto invalid = std::make_shared<AuthoredShape>(*profile());
        invalid->source.closed = false;
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   edit_authored_part(world, id, 0, invalid);
                               }),
            "invalid outline fails before any body replacement");
        auto invalid_part = part();
        invalid_part.depth_m = std::numeric_limits<Real>::quiet_NaN();
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)add(world, {}, invalid_part);
                               }),
            "non-finite material volume fails transactionally");
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)add(world, {}, part(), "live");
                               }),
            "duplicate live keys fail without publishing a staged world");
        RIGIDBODIES_EXPECT(world.find_body(id) == original && authored_parts(*original).front() == logical && world.body_ids().size() == 1, "failed operations leave live pointers and immutable geometry untouched");
        const auto second = add(world, { 2.0, 0.0 });
        RIGIDBODIES_EXPECT(second.index == id.index + 1 && second.generation == id.generation + 1, "rejected staging does not consume identifiers");
    }

    RIGIDBODIES_TEST("mass overrides pending loads incompatible settings and stale selections reject assembly atomically")
    {
        auto world = empty_world();
        const auto a = add(world, {}, part(), "a"), b = add(world, { 2.0, 0.0 }, part(), "b");
        world.find_body(a)->override_mass(1.0);
        RIGIDBODIES_EXPECT(world.find_body(a)->has_mass_override() && rejected([&]
                                                                          {
                                                                              (void)assemble_authored_bodies(world, { a, b });
                                                                          }),
            "even equal explicit overrides retain their authored-mass ambiguity");
        world.find_body(a)->rebuild_mass_properties();
        world.find_body(a)->apply_force_at_center({ 1.0, 0.0 });
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, b });
                               }),
            "pending manual loads cannot silently disappear");
        world.find_body(a)->clear_accumulators();
        world.find_body(b)->set_gravity_scale(0.5);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, b });
                               }),
            "incompatible gravity settings are not silently averaged");
        world.find_body(b)->set_gravity_scale(1.0);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, a });
                               }),
            "duplicate body selections cannot consume one body twice");
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, BodyId { b.index, b.generation + 1 } });
                               }),
            "stale body IDs cannot address a recycled slot");
        RIGIDBODIES_EXPECT(world.is_valid(a) && world.is_valid(b) && world.body_ids().size() == 2, "all failed assembly attempts preserve both bodies");
    }

    RIGIDBODIES_TEST("spring joint and local force attachments must be detached before assembly or separation")
    {
        auto world = empty_world();
        const auto a = add(world, {}, part(), "a"), b = add(world, { 2.0, 0.0 }, part(), "b");
        LinearSpringDefinition spring;
        spring.first = a;
        spring.second = b;
        const auto spring_id = world.create_spring(spring);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, b });
                               }) &&
                world.is_valid(spring_id),
            "attached springs survive rejected authoring");
        world.remove_spring(spring_id);
        DistanceJointDefinition definition;
        definition.first = a;
        definition.second = b;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, "joint");
        joint->set_enabled(false);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, b });
                               }) &&
                world.constraint_by_key("joint") == joint,
            "disabled joints remain explicit attachments and are not silently deleted");
        world.remove_constraint(joint);
        auto local = std::make_shared<UniformGravity>();
        world.add_force_generator(a, local);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)assemble_authored_bodies(world, { a, b });
                               }),
            "local forces require an explicit reassignment decision");
        world.remove_force_generator(a, local);
        const auto merged = assemble_authored_bodies(world, { a, b });
        world.add_force_generator(merged, local);
        RIGIDBODIES_EXPECT(rejected([&]
                               {
                                   (void)separate_authored_body(world, merged);
                               }) &&
                world.is_valid(merged),
            "separation also preserves attachments on rejection");
    }

    class UncloneableForce final : public ForceGenerator
    {
    public:
        int calls {};
        std::string_view name() const override
        {
            return "uncloneable_authored_probe";
        }
        void apply(RigidBody&, const ForceContext&) override
        {
            ++calls;
        }
    };

    RIGIDBODIES_TEST("body edit transactions preserve global component identities without requiring plugin snapshots")
    {
        auto world = empty_world();
        auto custom = std::make_shared<UncloneableForce>();
        world.add_force_generator(custom);
        const auto gravity = world.force_generators().front();
        const auto a = add(world, {}, part(), "a"), b = add(world, { 2.0, 0.0 }, part(), "b");
        edit_authored_part(world, a, 0, profile(false, 0.8));
        const auto merged = assemble_authored_bodies(world, { a, b });
        const auto separated = separate_authored_body(world, merged);
        RIGIDBODIES_EXPECT(separated.size() == 2 && world.force_generators().front() == gravity && world.force_generators().back() == custom, "session force pointers remain attached to the same live instances");
        RIGIDBODIES_EXPECT(custom->calls == 0, "staged mutations never run mutable plugin callbacks");
        world.step(0.01);
        RIGIDBODIES_EXPECT(custom->calls == 2, "preserved generator advances normally after publication");
    }

    RIGIDBODIES_TEST("successful outline edits invalidate contact caches while preserving local joint identifiers")
    {
        auto world = empty_world();
        const auto id = add(world, {}, part(), "edited");
        BodyDefinition floor;
        floor.type = BodyType::static_body;
        floor.position_m = { 0.5, -0.05 };
        Collider collider;
        collider.shape = make_box(3.0, 0.2);
        floor.colliders.push_back(collider);
        const auto support = world.create_body(floor, "support");
        world.step(0.01);
        RIGIDBODIES_EXPECT(!world.manifolds().empty(), "fixture establishes cached solid contact");
        DistanceJointDefinition definition;
        definition.first = support;
        definition.second = id;
        auto joint = std::make_shared<JointConstraint>(definition);
        world.add_constraint(joint, "retained");
        edit_authored_part(world, id, 0, profile(false, 0.5));
        RIGIDBODIES_EXPECT(world.manifolds().empty() && world.broad_phase_pairs().empty() && world.statistics().contact_point_count == 0, "edited collider geometry cannot reuse stale contact indices or impulses");
        RIGIDBODIES_EXPECT(world.constraint_by_key("retained") == joint && joint->second_body() == id, "in-place editing preserves local-frame joint attachments and body ID");
        world.step(0.01);
        RIGIDBODIES_EXPECT(world.motion_limit_events().empty(), "collision and constraint phases rebuild safely after editing");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
