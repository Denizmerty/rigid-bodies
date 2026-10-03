#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/shape_document.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;
    using namespace rigidbodies::physics;
    using Json = content::Json;

    ScenarioMetadata metadata()
    {
        return { "saved-example", "Saved example", "A portable edited arrangement", { "energy", "motion" }, { "free_fall" }, {}, {}, {}, {}, 0, 4, false };
    }
    ScenarioDocument capture(const World& world)
    {
        ScenarioDocument result;
        std::string error;
        RIGIDBODIES_EXPECT(capture_scenario_document(world, metadata(), result, error), error);
        return result;
    }
    World restore(const ScenarioDocument& document)
    {
        std::string text, error;
        ScenarioDocument parsed;
        World result;
        RIGIDBODIES_EXPECT(write_scenario_document(document, text, error), error);
        RIGIDBODIES_EXPECT(parse_scenario_document(text, parsed, error), error);
        RIGIDBODIES_EXPECT(populate_world(parsed, result, error), error);
        return result;
    }
    BodyDefinition ball(std::string name = "Ball")
    {
        BodyDefinition definition;
        definition.name = std::move(name);
        definition.position_m = { 1, 3 };
        Collider collider;
        collider.shape = make_circle(0.3);
        definition.colliders.push_back(collider);
        return definition;
    }

    std::shared_ptr<const AuthoredShape> profile(Real width)
    {
        Outline outline;
        outline.closed = true;
        for (const auto point : { math::Vec2 { 0, 0 }, math::Vec2 { width, 0 }, math::Vec2 { width, 1 }, math::Vec2 { 0, 1 } })
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        const auto built = build_authored_shape(outline);
        RIGIDBODIES_EXPECT(built.succeeded(), "Authored fixture builds");
        return built.shape;
    }

    RIGIDBODIES_TEST("imported shape provenance follows logical parts through edits assembly separation and reload")
    {
        World world;
        ShapeDocument imported;
        std::string error;
        RIGIDBODIES_EXPECT(capture_shape_document(*profile(1), "Imported bracket", "Workshop source", imported, error), error);
        imported.root["version"]["minor"] = 7;
        imported.root["creator_extension"] = Json::Object { { "name", "Ada" } };
        AuthoredPartDefinition imported_part;
        imported_part.shape = imported.shape;
        imported_part.source_document = std::make_shared<const ShapeDocument>(imported);
        BodyDefinition placement;
        const auto first = create_authored_body(world, placement, { imported_part });
        AuthoredPartDefinition ordinary_part;
        ordinary_part.shape = profile(0.5);
        placement.position_m = { 3, 0 };
        const auto second = create_authored_body(world, placement, { ordinary_part });
        edit_authored_part(world, first, 0, profile(2));
        const auto assembly = assemble_authored_bodies(world, { first, second });
        const auto assembled_parts = authored_parts(*world.find_body(assembly));
        RIGIDBODIES_EXPECT(assembled_parts[0]->source_document && !assembled_parts[1]->source_document, "Assembly retains provenance only on its original logical part");
        const auto snapshot = world.snapshot();
        world.restore(snapshot);
        const auto separated = separate_authored_body(world, assembly);
        RIGIDBODIES_EXPECT(authored_parts(*world.find_body(separated[0])).front()->source_document != nullptr, "Separation retains provenance after snapshot restore");
        const auto saved = capture(world);
        auto loaded = restore(saved);
        const auto restored_part = authored_parts(*loaded.find_body(loaded.body_ids()[0])).front();
        RIGIDBODIES_EXPECT(restored_part->source_document->title == "Imported bracket" && restored_part->source_document->summary == "Workshop source", "Original descriptive metadata survives arrangement reload");
        RIGIDBODIES_EXPECT(restored_part->source_document->root.at("creator_extension").at("name").as_string() == "Ada", "Unknown imported shape metadata survives");
        RIGIDBODIES_EXPECT(restored_part->source_document->root.at("version").at("minor").as_number() == 7, "Future shape revision is retained");
        RIGIDBODIES_EXPECT_NEAR(restored_part->source_document->shape->source.nodes[1].position_m.x, 2, 0, "Embedded document captures current edited geometry");
        RIGIDBODIES_EXPECT(authored_parts(*loaded.find_body(loaded.body_ids()[1])).front()->source_document == nullptr, "Ordinary shapes do not acquire imported provenance");
    }

    RIGIDBODIES_TEST("embedded shape documents validate transactionally and outer geometry remains authoritative")
    {
        World world;
        ShapeDocument imported;
        std::string error;
        RIGIDBODIES_EXPECT(capture_shape_document(*profile(1), "Imported shape", "", imported, error), error);
        AuthoredPartDefinition part;
        part.shape = profile(2);
        part.source_document = std::make_shared<const ShapeDocument>(imported);
        const auto id = create_authored_body(world, {}, { part });
        const auto original = capture(world);
        auto source = original;
        auto& serialized_part = source.root["world"]["bodies"].as_array()[0]["parts"].as_array()[0];
        serialized_part["source_document"] = imported.root;
        auto loaded = restore(source);
        const auto restored_part = authored_parts(*loaded.find_body(loaded.body_ids()[0])).front();
        RIGIDBODIES_EXPECT_NEAR(restored_part->shape->source.nodes[1].position_m.x, 2, 0, "Provenance geometry cannot override the arrangement's authored geometry");
        auto previous = original;
        previous.root["world"]["bodies"].as_array()[0]["parts"].as_array()[0]["source_document"]["old_part_extension"] = "obsolete";
        ScenarioDocument replaced;
        RIGIDBODIES_EXPECT(capture_scenario_document(loaded, metadata(), replaced, error, &previous), error);
        RIGIDBODIES_EXPECT(replaced.root.at("world").at("bodies").as_array()[0].at("parts").as_array()[0].at("source_document").find("old_part_extension") == nullptr, "Provenance belongs to the live logical part rather than previous array positions");
        serialized_part["source_document"]["version"]["major"] = 2;
        RIGIDBODIES_EXPECT(!populate_world(source, world, error) && world.is_valid(id), "Unsupported embedded shape revision leaves the destination untouched");
        serialized_part["source_document"] = nullptr;
        RIGIDBODIES_EXPECT(populate_world(source, loaded, error), error);
        ScenarioDocument recaptured;
        RIGIDBODIES_EXPECT(capture_scenario_document(loaded, metadata(), recaptured, error, &original), error);
        RIGIDBODIES_EXPECT(recaptured.root.at("world").at("bodies").as_array()[0].at("parts").as_array()[0].at("source_document").is_null(), "Cleared provenance cannot reappear through unknown-field inheritance");
    }

    RIGIDBODIES_TEST("all catalogue scenarios survive portable document roundtrips")
    {
        RIGIDBODIES_EXPECT(!available_scenarios().empty() && find_scenario("empty_lab") != nullptr, "All shipped lessons are available");
        for (const auto& description : available_scenarios())
        {
            RIGIDBODIES_EXPECT(!description.id.empty() && !description.title.empty() && !description.summary.empty(), "Every lesson has catalogue identity and learner-facing copy");
            RIGIDBODIES_EXPECT(!description.concepts.empty() && !description.tags.empty(), "Every lesson is discoverable by concept and tag");
            RIGIDBODIES_EXPECT(!description.collection.empty() && !description.level.empty() && !description.hook.empty(), "Every lesson has collection, level, and hook metadata");
            RIGIDBODIES_EXPECT(description.collection_order > 0 && description.suggested_order > 0, "Every lesson has positive collection and suggested ordering");
            World original;
            RIGIDBODIES_EXPECT(load_scenario(original, description.id), "Scenario loads");
            const auto document = capture(original);
            auto copy = restore(document);
            const auto original_ids = original.body_ids(), copy_ids = copy.body_ids();
            RIGIDBODIES_EXPECT(original_ids.size() == copy_ids.size(), "Body count survives");
            RIGIDBODIES_EXPECT(original.spring_ids().size() == copy.spring_ids().size(), "Spring count survives");
            RIGIDBODIES_EXPECT(original.constraints().size() == copy.constraints().size(), "Joint count survives");
            RIGIDBODIES_EXPECT(original.force_generators().size() == copy.force_generators().size(), "Global forces survive");
            for (std::size_t index = 0; index < original_ids.size(); ++index)
            {
                const auto& first = *original.find_body(original_ids[index]);
                const auto& second = *copy.find_body(copy_ids[index]);
                RIGIDBODIES_EXPECT(first.name() == second.name() && first.type() == second.type(), "Body names and canonical order survive");
                RIGIDBODIES_EXPECT_NEAR(first.position_m().x, second.position_m().x, 1e-12, "Position survives");
                RIGIDBODIES_EXPECT_NEAR(first.mass_properties().mass_kg, second.mass_properties().mass_kg, 1e-10, "Mass distribution survives");
                RIGIDBODIES_EXPECT(first.has_mass_override() == second.has_mass_override(), "Explicit mass overrides survive");
                RIGIDBODIES_EXPECT(first.colliders().size() == second.colliders().size(), "Convex collision cells survive");
                RIGIDBODIES_EXPECT(authored_parts(first).size() == authored_parts(second).size(), "Logical editable parts survive");
                RIGIDBODIES_EXPECT(original.force_generators(original_ids[index]).size() == copy.force_generators(copy_ids[index]).size(), "Body force attachments survive");
            }
            for (int frame = 0; frame < 4; ++frame)
            {
                original.step(1.0 / 120);
                copy.step(1.0 / 120);
            }
            for (std::size_t index = 0; index < original_ids.size(); ++index)
            {
                const auto& first = *original.find_body(original_ids[index]);
                const auto& second = *copy.find_body(copy_ids[index]);
                RIGIDBODIES_EXPECT_NEAR(first.position_m().x, second.position_m().x, 1e-8, "Simulation resumes with matching horizontal motion");
                RIGIDBODIES_EXPECT_NEAR(first.position_m().y, second.position_m().y, 1e-8, "Simulation resumes with matching vertical motion");
            }
        }
    }

    RIGIDBODIES_TEST("invalid schema versions references and variants leave worlds and documents untouched")
    {
        World original;
        original.create_body(ball());
        const auto valid = capture(original);
        const auto check = [&](const ScenarioDocument& invalid)
        {
            World target;
            const auto old = target.create_body(ball("Sentinel"));
            target.step(0.01);
            std::string error;
            RIGIDBODIES_EXPECT(!populate_world(invalid, target, error) && !error.empty(), "Invalid document fails clearly");
            RIGIDBODIES_EXPECT(target.is_valid(old) && target.find_body(old)->name() == "Sentinel", "Existing target is unchanged");
            RIGIDBODIES_EXPECT(target.statistics().step_index == 1, "Existing elapsed simulation survives failure");
            ScenarioDocument parsed = valid;
            RIGIDBODIES_EXPECT(!parse_scenario_document(content::write_json(invalid.root), parsed, error), "Parser validates semantic data");
            RIGIDBODIES_EXPECT(parsed.metadata.title == valid.metadata.title, "Failed parse does not publish partial data");
        };
        auto invalid = valid;
        invalid.root["version"]["major"] = 2;
        check(invalid);
        invalid = valid;
        invalid.root["version"]["minor"] = -1;
        check(invalid);
        invalid = valid;
        invalid.root["requires"] = Json::Array { "future_solver" };
        check(invalid);
        invalid = valid;
        invalid.root["required_features"] = Json::Array { "future_solver" };
        check(invalid);
        invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["type"] = "unknown";
        check(invalid);
        invalid = valid;
        invalid.root["world"]["bodies"].as_array().push_back(invalid.root["world"]["bodies"].as_array()[0]);
        check(invalid);
        invalid = valid;
        invalid.root["world"]["springs"] = Json::Array { Json::Object { { "type", "linear" }, { "first", "missing" }, { "second", "also-missing" } } };
        check(invalid);
        invalid = valid;
        invalid.root["world"]["settings"]["air_density_kg_m3"] = -1;
        check(invalid);
        invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["parts"].as_array()[0]["material"]["restitution"] = 2;
        check(invalid);
        invalid = valid;
        invalid.root["world"]["forces"] = Json::Array { Json::Object { { "type", "custom-force" } } };
        check(invalid);
    }

    RIGIDBODIES_TEST("loading invalidates abandoned body and spring handles")
    {
        World world;
        const auto first = world.create_body(ball("First"));
        const auto second = world.create_body(ball("Second"));
        LinearSpringDefinition spring;
        spring.first = first;
        spring.second = second;
        const auto old_spring = world.create_spring(spring);
        const auto document = capture(world);
        std::string error;
        RIGIDBODIES_EXPECT(populate_world(document, world, error), error);
        RIGIDBODIES_EXPECT(!world.is_valid(first) && !world.is_valid(second) && !world.is_valid(old_spring), "A loaded arrangement cannot resurrect stale handles");
        RIGIDBODIES_EXPECT(world.body_ids().size() == 2 && world.spring_ids().size() == 1, "Replacement components exist under fresh handles");
    }

    RIGIDBODIES_TEST("unrepresentable geometry and out of bounds motion are rejected without clamping")
    {
        World world;
        const auto id = world.create_body(ball());
        const auto valid = capture(world);
        std::string error;
        auto invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["position_m"] = Json::Array { 101, 0 };
        RIGIDBODIES_EXPECT(!populate_world(invalid, world, error), "Out of range positions fail rather than being silently clamped");
        invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["linear_velocity_m_s"] = Json::Array { 100, 100 };
        RIGIDBODIES_EXPECT(!populate_world(invalid, world, error), "Velocity magnitude respects document limits");
        invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["parts"].as_array()[0]["shape"]["radius_m"] = 1e308;
        RIGIDBODIES_EXPECT(!populate_world(invalid, world, error), "Finite inputs with overflowing derived geometry fail");
        invalid = valid;
        invalid.root["world"]["bodies"].as_array()[0]["parts"].as_array()[0]["depth_m"] = 1e308;
        RIGIDBODIES_EXPECT(!populate_world(invalid, world, error), "Material mass multiplication cannot overflow silently");
        RIGIDBODIES_EXPECT(world.is_valid(id) && world.find_body(id)->name() == "Ball", "The destination survives every invalid load");
    }

    RIGIDBODIES_TEST("future additive fields survive editing by stable body identity")
    {
        World initial;
        initial.create_body(ball("First"));
        initial.create_body(ball("Second"));
        auto source = capture(initial);
        source.root["version"]["minor"] = 19;
        source.root["future_annotation"] = Json::Object { { "label", "preserve me" } };
        source.root["metadata"]["future_author"] = "Ada";
        source.root["world"]["settings"]["future_setting"] = 3;
        auto& bodies = source.root["world"]["bodies"].as_array();
        bodies[0]["future_tag"] = "removed";
        bodies[1]["future_tag"] = "survivor";
        auto edited = restore(source);
        const auto ids = edited.body_ids();
        edited.destroy_body(ids.front());
        edited.find_body(ids.back())->set_name("Renamed");
        edited.create_body(ball("Third"));
        ScenarioDocument saved;
        std::string error;
        RIGIDBODIES_EXPECT(capture_scenario_document(edited, metadata(), saved, error, &source), error);
        RIGIDBODIES_EXPECT(saved.root.at("version").at("minor").as_number() == 19, "Future minor is not downgraded");
        RIGIDBODIES_EXPECT(saved.root.at("future_annotation").at("label").as_string() == "preserve me", "Root extension survives");
        RIGIDBODIES_EXPECT(saved.root.at("metadata").at("future_author").as_string() == "Ada", "Metadata extension survives");
        RIGIDBODIES_EXPECT(saved.root.at("world").at("settings").at("future_setting").as_number() == 3, "Settings extension survives");
        const auto& saved_bodies = saved.root.at("world").at("bodies").as_array();
        RIGIDBODIES_EXPECT(saved_bodies[0].at("future_tag").as_string() == "survivor", "Deletion cannot shift extension onto a different body");
        RIGIDBODIES_EXPECT(saved_bodies[1].find("future_tag") == nullptr, "New body does not inherit a deleted body's extension");
        const auto snapshot = edited.snapshot();
        edited.restore(snapshot);
        auto recaptured = capture(edited);
        RIGIDBODIES_EXPECT(recaptured.root.at("world").at("bodies").as_array()[0].at("id").as_string() == saved_bodies[0].at("id").as_string(), "Undo snapshots retain document identity");
    }

    RIGIDBODIES_TEST("saving a moving prescribed path retains its current phase")
    {
        World original;
        auto definition = ball();
        definition.type = BodyType::kinematic_body;
        const auto id = original.create_body(definition);
        HarmonicMotion path;
        path.origin_m = { 2, 3 };
        path.translation_amplitude_m = { 1, 0.5 };
        path.rotation_amplitude_rad = 0.2;
        path.frequency_hz = 0.6;
        path.phase_rad = 0.3;
        RIGIDBODIES_EXPECT(original.set_kinematic_motion(id, KinematicMotion(path)), "Path is attached");
        for (int frame = 0; frame < 47; ++frame)
            original.step(1.0 / 120);
        auto copy = restore(capture(original));
        const auto restored = copy.body_ids().front();
        RIGIDBODIES_EXPECT(copy.statistics().step_index == 0, "Arrangement starts a new simulation clock");
        for (int frame = 0; frame < 30; ++frame)
        {
            original.step(1.0 / 120);
            copy.step(1.0 / 120);
            RIGIDBODIES_EXPECT_NEAR(original.find_body(id)->position_m().x, copy.find_body(restored)->position_m().x, 1e-12, "Retained phase advances continuously");
            RIGIDBODIES_EXPECT_NEAR(original.find_body(id)->angular_velocity_rad_s(), copy.find_body(restored)->angular_velocity_rad_s(), 1e-12, "Analytic derivative retains phase");
        }
    }

    RIGIDBODIES_TEST("portable arrangement retains component settings filters overrides and pending loads")
    {
        WorldSettings settings;
        settings.gravity_m_s2 = { 2, -4 };
        settings.air_velocity_m_s = { 3, 1 };
        settings.air_density_kg_m3 = 0.8;
        settings.constraint_graph_enabled = false;
        settings.solver.warm_starting = false;
        settings.sleep.enabled = false;
        settings.collision.friction_mixing = MaterialMixing::minimum;
        settings.collision.restitution_mixing = MaterialMixing::arithmetic_mean;
        World original(settings);
        original.set_potential_energy_reference_height(2.5);
        original.set_integrator(std::make_shared<RungeKutta4Integrator>());
        original.set_broad_phase(std::make_shared<BruteForceBroadPhase>());
        original.set_narrow_phase(std::make_shared<NullNarrowPhase>());
        original.set_contact_solver(std::make_shared<NullContactSolver>());
        auto definition = ball();
        definition.linear_damping = 0.1;
        definition.angular_damping = 0.3;
        definition.gravity_scale = -2;
        auto& collider = definition.colliders.front();
        collider.filter = { 0x80000000U, 0xffffffffU, -4 };
        collider.shell_thickness_m = 0.04;
        collider.density_override_kg_m3 = 17;
        collider.drag_coefficient_override = 0.1;
        collider.material.friction_axis_local = { 0, 1 };
        collider.material.friction_anisotropy_ratio = 0.2;
        const auto id = original.create_body(definition, "original-key");
        auto* body = original.find_body(id);
        body->override_mass(2.3);
        body->apply_force_at_center({ 3, 5 });
        body->apply_torque(0.7);
        auto attractor = std::make_shared<PointAttractor>(math::Vec2 { 1, 2 }, 0.7);
        attractor->set_enabled(false);
        original.add_force_generator(id, attractor);
        AerodynamicSettings drag;
        drag.maximum_lift_coefficient = 0.6;
        drag.reynolds_correction = false;
        original.add_force_generator(std::make_shared<AerodynamicDrag>(drag));
        auto copy = restore(capture(original));
        const auto copy_id = copy.body_ids().front();
        const auto& restored = *copy.find_body(copy_id);
        RIGIDBODIES_EXPECT(copy.integrator().name() == original.integrator().name() && copy.broad_phase().name() == original.broad_phase().name() && copy.narrow_phase().name() == original.narrow_phase().name() && copy.contact_solver().name() == original.contact_solver().name(), "Selected algorithms survive");
        RIGIDBODIES_EXPECT_NEAR(copy.settings().air_velocity_m_s.x, 3, 1e-12, "Wind survives");
        RIGIDBODIES_EXPECT_NEAR(copy.potential_energy_reference_height_m(), 2.5, 1e-12, "Energy reference survives");
        RIGIDBODIES_EXPECT(copy.settings().collision.friction_mixing == MaterialMixing::minimum && !copy.settings().constraint_graph_enabled && !copy.settings().solver.warm_starting, "Policies survive");
        RIGIDBODIES_EXPECT(restored.colliders().front().filter.category == 0x80000000U && restored.colliders().front().filter.mask == 0xffffffffU && restored.colliders().front().filter.group == -4, "Full-width collision masks survive");
        RIGIDBODIES_EXPECT_NEAR(*restored.colliders().front().shell_thickness_m, 0.04, 1e-12, "Hollow geometry survives");
        RIGIDBODIES_EXPECT_NEAR(restored.mass_properties().mass_kg, 2.3, 1e-12, "Explicit mass survives");
        RIGIDBODIES_EXPECT_NEAR(restored.accumulated_force_n().y, 5, 1e-12, "Pending force is not lost");
        RIGIDBODIES_EXPECT_NEAR(restored.accumulated_torque_n_m(), 0.7, 1e-12, "Pending torque is not lost");
        RIGIDBODIES_EXPECT(!copy.force_generators(copy_id).front()->is_enabled(), "Disabled attachment remains disabled");
        RIGIDBODIES_EXPECT_NEAR(dynamic_cast<const AerodynamicDrag&>(*copy.force_generators().back()).settings().maximum_lift_coefficient, 0.6, 1e-12, "Aerodynamic model settings survive");
    }

    RIGIDBODIES_TEST("broken joints and disabled springs retain their state")
    {
        World original;
        const auto first = original.create_body(ball("First"));
        const auto second = original.create_body(ball("Second"));
        RevoluteJointDefinition hinge;
        hinge.first = first;
        hinge.second = second;
        hinge.motor_enabled = true;
        hinge.maximum_motor_torque_n_m = 2;
        hinge.motor_speed_rad_s = 3;
        hinge.limits_enabled = true;
        hinge.lower_angle_rad = -1;
        hinge.upper_angle_rad = 1;
        hinge.break_force_n = 5;
        original.add_constraint(std::make_shared<JointConstraint>(hinge), "hinge");
        AngularSpringDefinition spring;
        spring.first = first;
        spring.second = second;
        spring.enabled = false;
        spring.rest_angle_rad = 7;
        spring.stiffness_n_m_rad = 4;
        original.create_spring(spring, "torsion");
        auto document = capture(original);
        auto& joint = document.root["world"]["joints"].as_array()[0];
        joint["broken"] = true;
        joint["enabled"] = false;
        auto copy = restore(document);
        const auto& restored = *copy.constraint_by_key("hinge");
        RIGIDBODIES_EXPECT(restored.is_broken() && !restored.is_enabled(), "A saved broken link is not repaired");
        const auto& definition = std::get<RevoluteJointDefinition>(dynamic_cast<const JointConstraint&>(restored).definition());
        RIGIDBODIES_EXPECT(definition.motor_enabled && definition.limits_enabled && definition.break_force_n == 5, "Joint limits motor and break threshold survive");
        const auto& restored_spring = std::get<AngularSpringDefinition>(*copy.spring_definition(copy.spring_ids().front()));
        RIGIDBODIES_EXPECT(!restored_spring.enabled && restored_spring.rest_angle_rad == 7, "Disabled unwrapped torsional spring survives");
        const auto recaptured = capture(copy);
        RIGIDBODIES_EXPECT(recaptured.root.at("world").at("joints").as_array()[0].at("broken").as_bool(), "Broken state survives repeated saves");
    }

    class CustomForce final : public ForceGenerator
    {
    public:
        std::string_view name() const override
        {
            return "Unsupported custom force";
        }
        void apply(RigidBody&, const ForceContext&) override
        {
        }
    };
    RIGIDBODIES_TEST("unsupported custom components fail saving without dropping simulation content")
    {
        World world;
        world.create_body(ball());
        auto result = capture(world);
        const auto original_title = result.metadata.title;
        world.add_force_generator(std::make_shared<CustomForce>());
        std::string error;
        RIGIDBODIES_EXPECT(!capture_scenario_document(world, metadata(), result, error), "Unknown custom force is rejected");
        RIGIDBODIES_EXPECT(error.find("custom force generator") != std::string::npos, "Error identifies unsupported component");
        RIGIDBODIES_EXPECT(result.metadata.title == original_title && world.force_generators().size() == 2, "Neither previous result nor live world changes");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
