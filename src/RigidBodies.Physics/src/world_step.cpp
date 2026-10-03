#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/constraint_graph.hpp>

#include <algorithm>
#include <stdexcept>
#include <typeinfo>
#include <unordered_map>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {
        template <typename Component>
        std::shared_ptr<Component> clone_component(const std::shared_ptr<Component>& source)
        {
            auto result = source->clone();
            const auto* cloned = result.get();
            const auto* original = source.get();
            if (!cloned || cloned == original || typeid(*cloned) != typeid(*original))
            {
                throw std::logic_error("Snapshot clones must be independent objects of the original type");
            }
            return result;
        }

        template <typename Component>
        std::vector<std::shared_ptr<Component>> clone_components(const std::vector<std::shared_ptr<Component>>& sources,
            std::unordered_map<const Component*, std::shared_ptr<Component>>& clones)
        {
            std::vector<std::shared_ptr<Component>> result;
            result.reserve(sources.size());
            for (const auto& source : sources)
            {
                auto& copy = clones[source.get()];
                if (!copy)
                {
                    copy = clone_component(source);
                }
                result.push_back(copy);
            }
            return result;
        }

    }

    void World::step(Real time_step_s, bool capture_previous_transform)
    {
        if (!math::is_finite(time_step_s) || time_step_s <= 0.0 ||
            !math::is_finite(statistics_.elapsed_time_s + time_step_s))
        {
            return;
        }

        const auto coupled = integrator_->requires_force_evaluation() && !ordered_spring_slots_.empty();
        if (coupled && !integrator_->supports_coupled_prediction())
            throw std::logic_error("This staged integrator cannot advance coupled spring forces");
        profile_ = {};
        profile_.step_index = statistics_.step_index + 1;
        ScopedProfileTimer total_timer(profiling_enabled_, profile_.total_s);
        motion_limit_events_.clear();
        constraint_break_events_.clear();
        statistics_.last_step_limit_event_count = 0;
        enforce_motion_limits();
        synchronize_constraints();

        if (capture_previous_transform)
        {
            for_each_body([](BodyId, RigidBody& body)
                {
                    body.capture_previous_transform();
                });
        }

        ScopedProfileTimer force_timer(profiling_enabled_, profile_.forces_s);
        // Staged methods evaluate independent copies of the start-of-step force registrations.
        // Preserve aliases and canonical traversal even for generators with mutable caches. The
        // live generators advance just once; probes cannot change their state or visible loads.
        struct ForceState
        {
            std::vector<RigidBody> bodies;
            std::vector<ForceGeneratorPtr> global;
            std::vector<std::vector<ForceGeneratorPtr>> local;
        };
        const auto ids = body_ids();
        std::vector<IntegratedMotion> accounting_starts(slots_.size());
        for (const auto id : ids)
        {
            const auto& body = *find_body(id);
            accounting_starts[id.index] = { body.world_center_of_mass_m(), body.orientation_rad(), body.linear_velocity_m_s(), body.angular_velocity_rad_s() };
        }
        begin_energy_ledger(time_step_s, accounting_starts);
        std::shared_ptr<ForceState> frozen;
        if (integrator_->requires_force_evaluation())
        {
            frozen = std::make_shared<ForceState>();
            std::unordered_map<const ForceGenerator*, ForceGeneratorPtr> copies;
            frozen->global = clone_components(force_generators_, copies);
            frozen->bodies.reserve(ids.size());
            frozen->local.reserve(ids.size());
            for (const auto id : ids)
            {
                frozen->bodies.push_back(*find_body(id));
                frozen->local.push_back(clone_components(slots_[id.index].force_generators, copies));
            }
        }

        apply_forces(time_step_s);
        enforce_motion_limits();
        force_timer.stop();
        {
            ScopedProfileTimer timer(profiling_enabled_, profile_.wake_s);
            wake_connected_bodies(time_step_s);
        }
        ScopedProfileTimer velocity_timer(profiling_enabled_, profile_.velocity_integration_s);

        std::vector<std::optional<IntegratedMotion>> predictions(slots_.size());
        std::vector<std::optional<KinematicState>> paths(slots_.size());
        const CoupledForceEvaluator sample_forces = [&](std::vector<RigidBody>& probes, Real offset)
        {
            for (std::size_t index = 0; index < probes.size(); ++index)
            {
                auto& probe = probes[index];
                probe.accumulated_force_n_ = frozen->bodies[index].accumulated_force_n_;
                probe.accumulated_torque_n_m_ = frozen->bodies[index].accumulated_torque_n_m_;
                probe.accumulated_force_channels_ = frozen->bodies[index].accumulated_force_channels_;
                const auto& slot = slots_[ids[index].index];
                if (slot.motion && probe.type() == BodyType::kinematic_body)
                {
                    const auto motion = slot.motion->sample(statistics_.elapsed_time_s + offset - slot.motion_start_time_s);
                    probe.set_simulated_pose(motion.position_m - math::rotate(math::Rotation2 { motion.orientation_rad }, probe.mass_properties().center_of_mass_m), motion.orientation_rad);
                    probe.set_simulated_velocity(motion.linear_velocity_m_s, motion.angular_velocity_rad_s);
                }
            }
            std::unordered_map<const ForceGenerator*, ForceGeneratorPtr> copies;
            const auto global = clone_components(frozen->global, copies);
            std::vector<std::vector<ForceGeneratorPtr>> local;
            local.reserve(frozen->local.size());
            for (const auto& registrations : frozen->local)
                local.push_back(clone_components(registrations, copies));
            const ForceContext context { settings_.gravity_m_s2, settings_.air_density_kg_m3, time_step_s, statistics_.elapsed_time_s + offset, settings_.air_velocity_m_s, settings_.air_dynamic_viscosity_pa_s };
            const auto apply = [&](RigidBody& probe, const ForceGeneratorPtr& generator)
            {
                probe.active_force_channel_ = generator->name();
                generator->apply(probe, context);
                probe.active_force_channel_.clear();
            };
            for (const auto& generator : global)
                if (generator->is_enabled())
                    for (auto& probe : probes)
                        apply(probe, generator);
            for (std::size_t index = 0; index < probes.size(); ++index)
                for (const auto& generator : local[index])
                    if (generator->is_enabled())
                        apply(probes[index], generator);
            std::vector<RigidBody*> pointers;
            pointers.reserve(probes.size());
            for (auto& probe : probes)
                pointers.push_back(&probe);
            apply_springs(ids, pointers);
            std::vector<ForceSample> result;
            result.reserve(probes.size());
            for (std::size_t index = 0; index < probes.size(); ++index)
            {
                const auto& probe = probes[index];
                if (!math::is_finite(probe.accumulated_force_n()) || !math::is_finite(probe.accumulated_torque_n_m()))
                {
                    report_motion_limit(ids[index], MotionLimitKind::non_finite_load);
                    result.push_back({});
                }
                else
                    result.push_back({ probe.accumulated_force_n(), probe.accumulated_torque_n_m() });
            }
            return result;
        };
        for (std::size_t ordinal = 0; ordinal < ids.size(); ++ordinal)
        {
            const auto id = ids[ordinal];
            auto& body = *find_body(id);
            auto& slot = slots_[id.index];
            body.retain_applied_forces();
            if (slot.motion && body.type() != BodyType::kinematic_body)
                slot.motion.reset();
            if (slot.motion)
            {
                try
                {
                    const auto motion = slot.motion->sample(statistics_.elapsed_time_s + time_step_s - slot.motion_start_time_s);
                    if (accept_kinematic_state(id, motion))
                    {
                        paths[id.index] = motion;
                        body.set_simulated_velocity(motion.linear_velocity_m_s, motion.angular_velocity_rad_s);
                    }
                    else
                    {
                        slot.motion.reset();
                        body.set_simulated_velocity({}, 0.0);
                    }
                }
                catch (const std::exception&)
                {
                    report_motion_limit(id, MotionLimitKind::kinematic_path);
                    slot.motion.reset();
                    body.set_simulated_velocity({}, 0.0);
                }
            }
            else if (frozen && !coupled && body.is_awake())
            {
                const ForceEvaluator evaluate = [&, ordinal](const RigidBody& trial, Real offset)
                {
                    auto probes = frozen->bodies;
                    probes[ordinal] = trial;
                    return sample_forces(probes, offset)[ordinal];
                };
                predictions[id.index] = integrator_->predict_motion(body, time_step_s, evaluate);
                const auto& predicted = *predictions[id.index];
                body.set_simulated_velocity(predicted.linear_velocity_m_s, predicted.angular_velocity_rad_s);
            }
            else if (!frozen && body.is_awake())
                integrator_->integrate_velocity(body, time_step_s);
        }

        if (coupled)
        {
            std::vector<RigidBody> starting_bodies;
            for (std::size_t index = 0; index < ids.size(); ++index)
            {
                starting_bodies.push_back(*find_body(ids[index]));
                if (paths[ids[index].index])
                    starting_bodies.back().set_simulated_velocity(frozen->bodies[index].linear_velocity_m_s(), frozen->bodies[index].angular_velocity_rad_s());
            }
            const auto result = integrator_->predict_coupled(starting_bodies, time_step_s, sample_forces);
            if (result.size() != ids.size())
                throw std::logic_error("Coupled integrator result size must match the world body count");
            for (std::size_t index = 0; index < ids.size(); ++index)
            {
                auto& body = *find_body(ids[index]);
                if (body.type() != BodyType::dynamic_body || !body.is_awake())
                    continue;
                predictions[ids[index].index] = result[index];
                body.set_simulated_velocity(result[index].linear_velocity_m_s, result[index].angular_velocity_rad_s);
            }
        }

        enforce_motion_limits();
        // A safety cap is not a contact impulse. Compare solver output to the bounded input,
        // otherwise clipping a large predicted velocity could reverse predicted displacement.
        std::vector<math::Vec2> velocities_before_solve(slots_.size());
        std::vector<Real> angular_velocities_before_solve(slots_.size());
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                velocities_before_solve[id.index] = body.linear_velocity_m_s();
                angular_velocities_before_solve[id.index] = body.angular_velocity_rad_s();
            });
        begin_ledger_velocity_solve(accounting_starts);
        std::vector<IntegratedMotion> collision_targets(slots_.size());
        std::vector<IntegratedMotion> collision_starts(slots_.size());
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                auto& target = collision_targets[id.index];
                target = { body.world_center_of_mass_m(), body.orientation_rad(), body.linear_velocity_m_s(), body.angular_velocity_rad_s() };
                collision_starts[id.index] = target;
                if (paths[id.index])
                {
                    const auto& path = *paths[id.index];
                    target = { path.position_m, path.orientation_rad, path.linear_velocity_m_s, path.angular_velocity_rad_s };
                }
                else if (predictions[id.index])
                    target = *predictions[id.index];
                else if (body.is_awake())
                {
                    target.center_position_m += body.linear_velocity_m_s() * time_step_s;
                    target.orientation_rad += body.angular_velocity_rad_s() * time_step_s;
                }
                if (!math::is_finite(target.center_position_m))
                    target.center_position_m = body.world_center_of_mass_m();
                if (!math::is_finite(target.orientation_rad))
                    target.orientation_rad = body.orientation_rad();
                target.center_position_m.x = math::clamp(target.center_position_m.x, -settings_.limits.maximum_position_m, settings_.limits.maximum_position_m);
                target.center_position_m.y = math::clamp(target.center_position_m.y, -settings_.limits.maximum_position_m, settings_.limits.maximum_position_m);
                target.orientation_rad = math::clamp(target.orientation_rad, -settings_.limits.maximum_orientation_rad, settings_.limits.maximum_orientation_rad);
            });
        velocity_timer.stop();
        detect_collisions(time_step_s, collision_targets);
        build_simulation_islands();
        {
            ScopedProfileTimer timer(profiling_enabled_, profile_.accounting_s);
            begin_collision_accounting(accounting_starts);
        }

        set_solver_update(true);
        try
        {
            solve_island_velocities(time_step_s);
            ScopedProfileTimer accounting_timer(profiling_enabled_, profile_.accounting_s);
            measure_contact_work(accounting_starts);
            finish_collision_accounting(time_step_s);
            const auto graph_statistics = measure_constraint_graph(*this, constraints_);
            statistics_.constraint_island_count = graph_statistics.island_count;
            statistics_.constraint_row_count = graph_statistics.row_count;
            statistics_.constraint_velocity_residual = graph_statistics.velocity_residual;
            for (auto& constraint : constraints_)
                if (constraint_active(*constraint))
                {
                    constraint->finalize_velocity(*this, time_step_s);
                    if (constraint->is_broken())
                    {
                        constraint_break_events_.push_back({ constraint->first_body(), constraint->second_body(), std::string(constraint->name()), constraint->broken_force_n(), constraint->broken_torque_n_m() });
                        find_body(constraint->first_body())->wake();
                        find_body(constraint->second_body())->wake();
                    }
                }

            accounting_timer.stop();
            ScopedProfileTimer position_timer(profiling_enabled_, profile_.position_integration_s);
            enforce_motion_limits();
            finish_ledger_velocity_solve(accounting_starts);

            for_each_body([&](BodyId id, RigidBody& body)
                {
                    if (id.index < paths.size() && paths[id.index])
                    {
                        const auto& path = *paths[id.index];
                        body.set_simulated_pose(path.position_m - math::rotate(math::Rotation2 { path.orientation_rad }, body.mass_properties().center_of_mass_m), path.orientation_rad);
                    }
                    else if (id.index < predictions.size() && predictions[id.index])
                    {
                        const auto& predicted = *predictions[id.index];
                        const auto center = predicted.center_position_m + (body.linear_velocity_m_s() - velocities_before_solve[id.index]) * time_step_s;
                        const auto angle = predicted.orientation_rad + (body.angular_velocity_rad_s() - angular_velocities_before_solve[id.index]) * time_step_s;
                        body.set_simulated_pose(center - math::rotate(math::Rotation2 { angle }, body.mass_properties().center_of_mass_m), angle);
                    }
                    else if (body.is_awake())
                    {
                        if (frozen)
                            SemiImplicitEulerIntegrator {}.integrate_position(body, time_step_s);
                        else
                            integrator_->integrate_position(body, time_step_s);
                    }
                });

            enforce_motion_limits();
            record_ledger_integrated_positions();

            position_timer.stop();
            {
                ScopedProfileTimer timer(profiling_enabled_, profile_.ccd_s);
                constrain_swept_motion(time_step_s, collision_starts);
            }
            solve_island_positions(time_step_s);

            enforce_motion_limits();
        }
        catch (...)
        {
            set_solver_update(false);
            throw;
        }
        set_solver_update(false);
        {
            ScopedProfileTimer timer(profiling_enabled_, profile_.sleep_s);
            update_sleep(time_step_s);
            finish_island_solving();
        }
        {
            ScopedProfileTimer timer(profiling_enabled_, profile_.accounting_s);
            finish_energy_ledger(time_step_s, accounting_starts);
        }

        for_each_body([](BodyId, RigidBody& body)
            {
                body.clear_accumulators();
            });

        statistics_.elapsed_time_s += time_step_s;
        ++statistics_.step_index;
        refresh_statistics();
    }

} // namespace rigidbodies::physics
