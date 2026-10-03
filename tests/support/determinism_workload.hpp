#pragma once

#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>

namespace rigidbodies::testing::determinism
{
    using namespace physics;
    using content::Json;
    using math::Vec2;

    inline Json vector(Vec2 value)
    {
        return Json::Array { value.x, value.y };
    }

    inline std::string identity(const World& world, BodyId id)
    {
        // Retain the slot generation as well as the workload's stable name: stale/reused handles must not
        // become indistinguishable. Missing endpoints still have a useful stable diagnostic.
        const auto* body = world.find_body(id);
        return (body != nullptr ? body->name() : "missing") + ":" + std::to_string(id.index) + ":" + std::to_string(id.generation);
    }

    inline Json pair_identity(const World& world, const BroadPhasePair& pair)
    {
        return Json::Array { identity(world, pair.first), pair.first_collider, identity(world, pair.second), pair.second_collider };
    }

    inline Json events(const World& world, const std::vector<CollisionEvent>& source)
    {
        Json::Array result;
        for (const auto& event : source)
            result.emplace_back(Json::Object { { "pair", pair_identity(world, event.pair) },
                { "kind", event.kind == CollisionEventKind::begin ? "begin" : "end" },
                { "sensor", event.is_sensor } });
        return result;
    }

    inline Json record(const World& world)
    {
        Json::Array bodies, motion, pairs, contacts, contact_values, islands, joints, joint_values;
        for (const auto id : world.body_ids())
        {
            const auto& body = *world.find_body(id);
            bodies.emplace_back(Json::Object { { "id", identity(world, id) }, { "awake", body.is_awake() }, { "type", static_cast<int>(body.type()) }, { "colliders", body.colliders().size() } });
            motion.emplace_back(Json::Object { { "position_m", vector(body.position_m()) },
                { "angle_rad", body.orientation_rad() },
                { "velocity_m_s", vector(body.linear_velocity_m_s()) },
                { "angular_velocity_rad_s", body.angular_velocity_rad_s() },
                { "mass_kg", body.mass_properties().mass_kg },
                { "kinetic_energy_j", body.kinetic_energy_j() },
                { "force_n", vector(body.applied_force_n()) },
                { "torque_n_m", body.applied_torque_n_m() } });
        }
        for (const auto& pair : world.broad_phase_pairs())
            pairs.push_back(pair_identity(world, pair));
        for (const auto& manifold : world.manifolds())
        {
            Json::Array features, points;
            for (std::size_t index = 0; index < manifold.point_count; ++index)
            {
                const auto& point = manifold.points[index];
                features.emplace_back(std::to_string(point.feature.key())); // All 64 bits, never a JSON double.
                points.emplace_back(Json::Object { { "position_m", vector(point.world_position_m) },
                    { "separation_m", point.separation_m },
                    { "normal_impulse_n_s", point.normal_impulse_n_s },
                    { "tangent_impulse_n_s", point.tangent_impulse_n_s },
                    { "rolling_impulse_n_m_s", point.rolling_impulse_n_m_s },
                    { "spinning_impulse_n_m_s", point.spinning_impulse_n_m_s } });
            }
            contacts.emplace_back(Json::Object {
                { "pair", pair_identity(world, { manifold.first, manifold.second, manifold.first_collider, manifold.second_collider }) },
                { "sensor", manifold.is_sensor },
                { "speculative", manifold.is_speculative },
                { "features", features } });
            contact_values.emplace_back(Json::Object { { "normal", vector(manifold.normal) }, { "points", points } });
        }
        for (const auto& island : world.simulation_islands())
        {
            Json::Array members;
            for (const auto id : island.bodies)
                members.emplace_back(identity(world, id));
            islands.emplace_back(Json::Object { { "members", members }, { "awake", island.awake }, { "manifolds", island.manifold_indices.size() }, { "constraints", island.constraint_indices.size() } });
        }
        for (const auto& constraint : world.constraints())
        {
            joints.emplace_back(Json::Object { { "key", world.constraint_key(constraint) },
                { "first", identity(world, constraint->first_body()) },
                { "second", identity(world, constraint->second_body()) },
                { "enabled", constraint->is_enabled() },
                { "broken", constraint->is_broken() } });
            const auto* joint = dynamic_cast<const JointConstraint*>(constraint.get());
            if (joint == nullptr)
                throw std::logic_error("determinism workload expects built-in joints");
            const auto report = joint->report(world);
            joint_values.emplace_back(Json::Object { { "force_n", vector(report.reaction_force_n) },
                { "torque_n_m", report.reaction_torque_n_m },
                { "position_error_m", report.position_error_m },
                { "angular_error_rad", report.angular_error_rad } });
        }
        const auto& statistics = world.statistics();
        return Json::Object { { "step", statistics.step_index },
            { "topology", Json::Object { { "bodies", bodies }, { "pairs", pairs }, { "contacts", contacts }, { "pair_events", events(world, world.pair_events()) }, { "contact_events", events(world, world.contact_events()) }, { "joints", joints }, { "islands", islands }, { "ccd_clamped", statistics.ccd_clamped_body_count }, { "sweep_limits", statistics.sweep_iteration_limit_count }, { "limit_events", statistics.last_step_limit_event_count } } },
            { "values", Json::Object { { "bodies", motion }, { "contacts", contact_values }, { "joints", joint_values }, { "elapsed_s", statistics.elapsed_time_s }, { "energy_j", statistics.total_kinetic_energy_j }, { "momentum_kg_m_s", vector(statistics.total_linear_momentum_kg_m_s) } } } };
    }

    inline BodyDefinition body(ShapePtr shape, Vec2 position, Vec2 velocity = {}, BodyType type = BodyType::dynamic_body)
    {
        BodyDefinition definition;
        definition.position_m = position;
        definition.linear_velocity_m_s = velocity;
        definition.type = type;
        definition.sleep_enabled = false;
        Collider collider;
        collider.shape = std::move(shape);
        collider.material.restitution = 0.5;
        collider.material.static_friction = 0.0;
        collider.material.kinetic_friction = 0.0;
        definition.colliders.push_back(std::move(collider));
        return definition;
    }

    inline BodyId unit_body(World& world, BodyDefinition definition, const std::string& key)
    {
        definition.name = key;
        const auto id = world.create_body(definition, key);
        if (definition.type == BodyType::dynamic_body)
            world.find_body(id)->override_mass(1.0);
        return id;
    }

    struct Evidence
    {
        std::size_t broad_workers {}, narrow_workers {}, contacts {}, speculative {}, events {}, snapshot_replays {};
    };

    inline Json run_case(const std::string& name, const std::string& contract, World& world, Real dt, int steps,
        Evidence& evidence, const std::function<void(World&, int)>& mutation = {})
    {
        Json::Array records;
        WorldSnapshot checkpoint;
        for (int step = 0; step < steps; ++step)
        {
            if (step == steps / 2)
                checkpoint = world.snapshot();
            if (mutation)
                mutation(world, step);
            world.step(dt);
            records.push_back(record(world));
            evidence.broad_workers = std::max(evidence.broad_workers, world.profile().broad_phase_workers);
            evidence.narrow_workers = std::max(evidence.narrow_workers, world.profile().narrow_phase_workers);
            evidence.contacts += world.manifolds().size();
            evidence.speculative += world.statistics().speculative_manifold_count;
            evidence.events += world.contact_events().size();
        }
        world.restore(checkpoint);
        for (int step = steps / 2; step < steps; ++step)
        {
            if (mutation)
                mutation(world, step);
            world.step(dt);
            if (content::write_json(record(world)) != content::write_json(records[static_cast<std::size_t>(step)]))
                throw std::runtime_error(name + " snapshot replay differs at step " + std::to_string(step + 1));
        }
        ++evidence.snapshot_replays;
        return Json::Object { { "id", name }, { "numeric_contract", contract }, { "dt_s", dt }, { "steps", steps }, { "records", records } };
    }

    inline Json trace(std::size_t workers, Evidence& evidence)
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = false;
        Json::Array scenarios;

        // This restricted subset is dyadic throughout: no rotation, gravity, transcendental
        // profile, or solver regularization. It is checked exactly between supported platforms.
        auto exact_settings = settings;
        exact_settings.collision.continuous = false;
        exact_settings.collision.contact_margin_m = 0.0;
        exact_settings.solver.position_iterations = 0;
        exact_settings.solver.restitution_threshold_m_s = 0.0;
        World exact(exact_settings);
        exact.set_parallel_settings({ workers, 1 });
        for (int lane = 0; lane < 4; ++lane)
        {
            for (int side = 0; side < 2; ++side)
            {
                auto definition = body(make_circle(0.25), { side == 0 ? -1.0 : 1.0, static_cast<Real>(lane) * 4.0 }, { side == 0 ? 2.0 : -2.0, 0.0 });
                definition.fixed_rotation = true;
                definition.colliders[0].material.restitution = 1.0;
                (void)unit_body(exact, definition, "impact-" + std::to_string(lane) + "-" + std::to_string(side));
            }
        }
        scenarios.push_back(run_case("dyadic_elastic_impacts", "exact", exact, 1.0 / 128.0, 72, evidence));

        World storage(exact_settings);
        storage.set_parallel_settings({ workers, 1 });
        for (int index = 0; index < 8; ++index)
            (void)unit_body(storage, body(make_circle(0.25), { static_cast<Real>(index) * 4.0, 0.0 }, { 0.125, 0.25 }), "transport-" + std::to_string(index));
        const auto mutation = [](World& world, int step)
        {
            if (step != 4 && step != 20)
                return;
            const auto ids = world.body_ids();
            const auto removed = ids[2];
            if (!world.destroy_body(removed))
                throw std::logic_error("storage trace failed to remove live body");
            const auto created = unit_body(world, body(make_circle(0.25), { -8.0, static_cast<Real>(step) }, { 0.5, -0.125 }), "replacement-" + std::to_string(step));
            if (created.index != removed.index || created.generation == removed.generation || world.find_body(removed) != nullptr)
                throw std::logic_error("storage trace failed generation reuse contract");
        };
        scenarios.push_back(run_case("dyadic_storage_replay", "exact", storage, 1.0 / 128.0, 32, evidence, mutation));

        World contacts(settings);
        contacts.set_parallel_settings({ workers, 1 });
        for (int lane = 0; lane < 8; ++lane)
        {
            const auto y = static_cast<Real>(lane) * 3.0;
            (void)unit_body(contacts, body(make_circle(0.35), { -1.0, y }, { 2.0, 0.0 }), "left-" + std::to_string(lane));
            (void)unit_body(contacts, body(make_circle(0.35), { 1.0, y }, { -2.0, 0.0 }), "right-" + std::to_string(lane));
            auto sensor = body(make_box(1.1, 1.0), { 0.0, y }, {}, BodyType::static_body);
            sensor.colliders[0].is_sensor = true;
            (void)unit_body(contacts, sensor, "sensor-" + std::to_string(lane));
        }
        scenarios.push_back(run_case("contacts_and_sensors", "bounded", contacts, 1.0 / 120.0, 56, evidence));

        auto joint_settings = settings;
        joint_settings.gravity_m_s2 = { 0.0, -2.0 };
        World joints(joint_settings);
        joints.set_parallel_settings({ workers, 1 });
        for (int index = 0; index < 3; ++index)
        {
            const auto x = static_cast<Real>(index) * 6.0;
            const auto anchor = unit_body(joints, body(make_circle(0.1), { x, 2.0 }, {}, BodyType::static_body), "anchor-" + std::to_string(index));
            auto moving = body(make_circle(0.2), { x + 2.0, 2.0 }, { 0.0, 0.4 });
            moving.linear_damping = 0.07;
            const auto weight = unit_body(joints, moving, "weight-" + std::to_string(index));
            DistanceJointDefinition joint;
            joint.first = anchor;
            joint.second = weight;
            joint.length_m = 2.0;
            joints.add_constraint(std::make_shared<JointConstraint>(joint), "distance-" + std::to_string(index));
        }
        scenarios.push_back(run_case("joints_and_forces", "bounded", joints, 1.0 / 120.0, 48, evidence));

        World ccd(settings);
        ccd.set_parallel_settings({ workers, 1 });
        for (int lane = 0; lane < 6; ++lane)
        {
            const auto y = static_cast<Real>(lane) * 3.0;
            auto bullet = body(make_circle(0.05), { -0.5, y }, { 60.0, 0.0 });
            bullet.colliders[0].material.restitution = 0.0;
            (void)unit_body(ccd, bullet, "bullet-" + std::to_string(lane));
            auto wall = body(make_box(0.03, 1.5), { 0.0, y }, {}, BodyType::static_body);
            wall.colliders[0].material.restitution = 0.0;
            (void)unit_body(ccd, wall, "wall-" + std::to_string(lane));
        }
        scenarios.push_back(run_case("continuous_collision", "bounded", ccd, 1.0 / 60.0, 12, evidence));
        return Json::Object { { "format", "rigid-bodies-determinism" }, { "schema_version", 1 }, { "workload_revision", 1 }, { "seed", "fixed-dyadic-and-mechanics-v1" }, { "scenarios", scenarios } };
    }
}
