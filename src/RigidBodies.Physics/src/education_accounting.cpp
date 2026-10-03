#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <type_traits>
#include <variant>

namespace rigidbodies::physics
{
    namespace
    {
        // Potential energy at the middle of the last step: the position less the lag times the
        // velocity, which for semi-implicit Euler is exactly where the step's motion was centred.
        EnergyBreakdown body_energy(const World& world, BodyId id, const RigidBody& body, Real lag_s)
        {
            EnergyBreakdown result;
            if (body.type() == BodyType::dynamic_body)
            {
                result.translational_kinetic_j = body.translational_kinetic_energy_j();
                result.rotational_kinetic_j = body.rotational_kinetic_energy_j();
                const auto height = body.world_center_of_mass_m() - body.linear_velocity_m_s() * lag_s - math::Vec2 { 0.0, world.potential_energy_reference_height_m() };
                result.gravitational_potential_j = -body.mass_properties().mass_kg * math::dot(effective_uniform_gravity_m_s2(world, id), height);
            }
            return result;
        }

        // Stored energy less the lag times the rate at which the spring is storing it: the same
        // mid-step reading as for height, k x^2 / 2 - lag k x dx/dt for a linear spring.
        Real spring_energy(const SpringDefinition& definition, const SpringReport& report, Real lag_s)
        {
            if (!report.enabled)
                return 0.0;
            const auto storing_w = std::visit([&](const auto& spring)
                {
                    using Definition = std::decay_t<decltype(spring)>;
                    if constexpr (std::is_same_v<Definition, LinearSpringDefinition>)
                        return spring.stiffness_n_m * report.extension_m * report.axial_speed_m_s;
                    else
                        return spring.stiffness_n_m_rad * report.angular_displacement_rad * report.relative_angular_speed_rad_s;
                },
                definition);
            return report.potential_energy_j - lag_s * storing_w;
        }

        Real world_mechanical_energy(const World& world)
        {
            return measure_world_energy(world).mechanical_j();
        }

        CollisionBodyState collision_state(BodyId id, const RigidBody& body)
        {
            CollisionBodyState result;
            result.id = id;
            result.name = body.name();
            result.type = body.type();
            result.mass_kg = body.mass_properties().mass_kg;
            result.inertia_kg_m2 = body.mass_properties().inertia_kg_m2;
            result.center_m = body.world_center_of_mass_m();
            result.velocity_m_s = body.linear_velocity_m_s();
            result.angular_velocity_rad_s = body.angular_velocity_rad_s();
            if (body.type() == BodyType::dynamic_body)
            {
                result.momentum_kg_m_s = body.linear_momentum_kg_m_s();
                result.angular_momentum_kg_m2_s = body.angular_momentum_about_center_kg_m2_s();
                result.kinetic_energy_j = body.kinetic_energy_j();
            }
            return result;
        }
    }

    CollisionEnergyAccounting& CollisionEnergyAccounting::operator+=(const CollisionEnergyAccounting& other)
    {
        normal_energy_change_j += other.normal_energy_change_j;
        friction_energy_change_j += other.friction_energy_change_j;
        normal_external_work_j += other.normal_external_work_j;
        friction_external_work_j += other.friction_external_work_j;
        return *this;
    }

    EnergyLedger& EnergyLedger::operator+=(const EnergyLedger& other)
    {
        air_work_j += other.air_work_j;
        damper_work_j += other.damper_work_j;
        joint_work_j += other.joint_work_j;
        motor_work_j += other.motor_work_j;
        applied_work_j += other.applied_work_j;
        contact_correction_work_j += other.contact_correction_work_j;
        edit_work_j += other.edit_work_j;
        unattributed_j += other.unattributed_j;
        return *this;
    }

    std::optional<EnergyBreakdown> measure_body_energy(const World& world, BodyId id)
    {
        const auto* body = world.find_body(id);
        if (!body)
            return std::nullopt;
        const auto lag = world.energy_measurement_lag_s();
        auto result = body_energy(world, id, *body, lag);
        const auto moves = [&](BodyId end)
        {
            const auto* found = world.find_body(end);
            return found && found->type() == BodyType::dynamic_body;
        };
        for (const auto spring_id : world.spring_ids())
            if (const auto report = world.spring_report(spring_id))
                if (report->first == id || report->second == id)
                {
                    const auto this_end_moves = moves(id);
                    const auto other_end_moves = moves(report->first == id ? report->second : report->first);
                    const auto share = this_end_moves == other_end_moves ? 0.5 : this_end_moves ? 1.0
                                                                                                : 0.0;
                    result.spring_potential_j += spring_energy(*world.spring_definition(spring_id), *report, lag) * share;
                }
        return result;
    }

    EnergyBreakdown measure_world_energy(const World& world)
    {
        EnergyBreakdown result;
        const auto lag = world.energy_measurement_lag_s();
        world.for_each_body([&](BodyId id, const RigidBody& body)
            {
                const auto part = body_energy(world, id, body, lag);
                result.translational_kinetic_j += part.translational_kinetic_j;
                result.rotational_kinetic_j += part.rotational_kinetic_j;
                result.gravitational_potential_j += part.gravitational_potential_j;
            });
        for (const auto id : world.spring_ids())
            if (const auto report = world.spring_report(id))
                result.spring_potential_j += spring_energy(*world.spring_definition(id), *report, lag);
        return result;
    }

    math::Vec2 measure_world_momentum_kg_m_s(const World& world)
    {
        math::Vec2 result;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                if (body.type() == BodyType::dynamic_body)
                    result += body.linear_momentum_kg_m_s();
            });
        return result;
    }

    EnergyBudget measure_energy_budget(const World& world)
    {
        const auto& contacts = world.collision_energy();
        const auto& ledger = world.energy_ledger();
        EnergyBudget result;
        result.lost_in_impacts_j = contacts.restitution_loss_j() - ledger.contact_correction_work_j;
        result.lost_to_friction_j = contacts.friction_loss_j();
        result.lost_to_air_j = -ledger.air_work_j;
        result.lost_in_dampers_j = -ledger.damper_work_j;
        result.lost_in_joints_j = ledger.motor_work_j - ledger.joint_work_j;
        result.added_by_drives_j = contacts.external_work_j() + ledger.motor_work_j;
        result.added_by_forces_j = ledger.applied_work_j;
        // An edit made since the last step is booked when the next step begins; counting it now
        // keeps the budget balanced while the experiment is paused.
        const auto now_j = world_mechanical_energy(world);
        const auto end_j = world.ledger_end_energy_j();
        result.added_by_changes_j = ledger.edit_work_j + (end_j ? now_j - *end_j : 0.0);
        const auto booked_j = contacts.normal_energy_change_j + contacts.friction_energy_change_j + ledger.air_work_j + ledger.damper_work_j + ledger.joint_work_j +
            ledger.applied_work_j + ledger.contact_correction_work_j + ledger.edit_work_j + ledger.unattributed_j;
        result.start_energy_j = end_j ? *end_j - booked_j : now_j;

        const auto classify = [&](const std::vector<ForceGeneratorPtr>& generators)
        {
            for (const auto& generator : generators)
            {
                if (!generator || !generator->is_enabled() || dynamic_cast<const UniformGravity*>(generator.get()))
                    continue;
                (dynamic_cast<const AerodynamicDrag*>(generator.get()) ? result.has_air : result.has_applied_forces) = true;
            }
        };
        classify(world.force_generators());
        world.for_each_body([&](BodyId id, const RigidBody& body)
            {
                classify(world.force_generators(id));
                result.has_drives = result.has_drives || body.type() == BodyType::kinematic_body;
                result.has_air = result.has_air || (body.type() == BodyType::dynamic_body && (body.linear_damping() > 0.0 || body.angular_damping() > 0.0));
            });
        for (const auto id : world.spring_ids())
            std::visit([&](const auto& spring)
                {
                    using Definition = std::decay_t<decltype(spring)>;
                    if constexpr (std::is_same_v<Definition, LinearSpringDefinition>)
                        result.has_dampers = result.has_dampers || (spring.enabled && spring.damping_n_s_m > 0.0);
                    else
                        result.has_dampers = result.has_dampers || (spring.enabled && spring.damping_n_m_s_rad > 0.0);
                },
                *world.spring_definition(id));
        for (const auto& constraint : world.constraints())
        {
            const auto* joint = dynamic_cast<const JointConstraint*>(constraint.get());
            if (!joint || joint->is_broken())
                continue;
            result.has_joints = true;
            std::visit([&](const auto& definition)
                {
                    using Definition = std::decay_t<decltype(definition)>;
                    if constexpr (std::is_same_v<Definition, RevoluteJointDefinition> || std::is_same_v<Definition, PrismaticJointDefinition>)
                        result.has_drives = result.has_drives || definition.motor_enabled;
                },
                joint->definition());
        }
        return result;
    }

    bool impact_in_progress(const World& world)
    {
        // Slower approaches never rebound, so a support hovering just above a surface is not one.
        const auto rebound_speed = world.settings().solver.restitution_threshold_m_s;
        return std::any_of(world.manifolds().begin(), world.manifolds().end(), [&](const ContactManifold& manifold)
            {
                for (std::size_t index = 0; index < manifold.point_count && index < manifold.points.size(); ++index)
                    if (manifold.points[index].pending_impact_speed_m_s > rebound_speed)
                        return true;
                return false;
            });
    }

    math::Vec2 effective_uniform_gravity_m_s2(const World& world, BodyId id)
    {
        const auto* body = world.find_body(id);
        if (!body || body->type() != BodyType::dynamic_body)
            return {};
        Real count = 0.0;
        const auto add = [&](const auto& generators)
        {
            for (const auto& generator : generators)
                if (generator && generator->is_enabled() && dynamic_cast<const UniformGravity*>(generator.get()))
                    count += 1.0;
        };
        add(world.force_generators());
        add(world.force_generators(id));
        return world.settings().gravity_m_s2 * (count * body->gravity_scale());
    }

    const CollisionEnergyAccounting& World::collision_energy() const
    {
        return collision_energy_;
    }
    const CollisionEnergyAccounting& World::last_step_collision_energy() const
    {
        return last_step_collision_energy_;
    }
    const std::vector<CollisionImpactReport>& World::impact_reports() const
    {
        return impact_reports_;
    }
    std::size_t World::dropped_impact_report_count() const
    {
        return dropped_impact_report_count_;
    }
    const EnergyLedger& World::energy_ledger() const
    {
        return energy_ledger_;
    }

    std::optional<Real> World::ledger_end_energy_j() const
    {
        return ledger_has_end_energy_ ? std::optional<Real> { ledger_end_energy_j_ } : std::nullopt;
    }

    Real World::energy_measurement_lag_s() const
    {
        // Staged integrators already move a body along its mean velocity over the step.
        return integrator_ && !integrator_->requires_force_evaluation() ? 0.5 * energy_time_step_s_ : 0.0;
    }

    void World::reset_energy_ledger()
    {
        energy_ledger_ = {};
        ledger_has_end_energy_ = false;
        ledger_end_energy_j_ = 0.0;
    }

    void World::begin_energy_ledger(Real time_step_s, const std::vector<IntegratedMotion>& starts)
    {
        auto& step = ledger_step_;
        step.joint_work_j = 0.0;
        step.motor_work_j = 0.0;
        energy_time_step_s_ = time_step_s;
        step.start_energy_j = world_mechanical_energy(*this);
        if (ledger_has_end_energy_)
            energy_ledger_.edit_work_j += step.start_energy_j - ledger_end_energy_j_;

        step.dampers.clear();
        for (const auto index : ordered_spring_slots_)
        {
            const auto& definition = *spring_slots_[index].definition;
            std::visit([&](const auto& spring)
                {
                    using Definition = std::decay_t<decltype(spring)>;
                    const auto* first = find_body(spring.first);
                    const auto* second = find_body(spring.second);
                    if (!spring.enabled || !first || !second)
                        return;
                    LedgerDamper damper;
                    damper.first = spring.first;
                    damper.second = spring.second;
                    const auto report = evaluate_spring(definition, *first, *second);
                    if constexpr (std::is_same_v<Definition, LinearSpringDefinition>)
                    {
                        if (spring.damping_n_s_m <= 0.0)
                            return;
                        const auto span = report.second_anchor_m - report.first_anchor_m;
                        const auto length = math::length(span);
                        damper.axis = length > math::geometric_epsilon ? span / length : math::Vec2 {};
                        damper.first_lever_m = report.first_anchor_m - starts[spring.first.index].center_position_m;
                        damper.second_lever_m = report.second_anchor_m - starts[spring.second.index].center_position_m;
                        damper.load = spring.damping_n_s_m * report.axial_speed_m_s;
                    }
                    else
                    {
                        if (spring.damping_n_m_s_rad <= 0.0)
                            return;
                        damper.angular = true;
                        damper.load = spring.damping_n_m_s_rad * report.relative_angular_speed_rad_s;
                    }
                    step.dampers.push_back(damper);
                },
                definition);
        }
    }

    void World::begin_ledger_velocity_solve(const std::vector<IntegratedMotion>& starts)
    {
        auto& step = ledger_step_;
        step.force_changes.assign(slots_.size(), {});
        step.integrated.assign(slots_.size(), false);
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                if (body.type() != BodyType::dynamic_body)
                    return;
                const auto& start = starts[id.index];
                step.force_changes[id.index] = { body.linear_velocity_m_s() - start.linear_velocity_m_s, body.angular_velocity_rad_s() - start.angular_velocity_rad_s };
                step.integrated[id.index] = body.is_awake();
            });
    }

    // Each manifold's normal and friction impulses do work against the mean of each body's
    // velocity at the start and at the end of the step. Unlike the change across each impulse,
    // this does not depend on the order the solver applied them in, and a support that only
    // cancels the weight a resting body gained this step does no work.
    void World::measure_contact_work(const std::vector<IntegratedMotion>& starts)
    {
        const auto mean = [&](BodyId id, const RigidBody& body)
        {
            if (body.type() != BodyType::dynamic_body)
                return LedgerVelocity {};
            const auto& start = starts[id.index];
            return LedgerVelocity { (start.linear_velocity_m_s + body.linear_velocity_m_s()) * 0.5, 0.5 * (start.angular_velocity_rad_s + body.angular_velocity_rad_s()) };
        };
        for (auto& manifold : manifolds_)
        {
            const auto* first = find_body(manifold.first);
            const auto* second = find_body(manifold.second);
            if (manifold.is_sensor || !first || !second)
                continue;
            const auto first_mean = mean(manifold.first, *first);
            const auto second_mean = mean(manifold.second, *second);
            const auto work = [&](const math::Vec2& on_second, Real angular_first, Real angular_second)
            {
                return math::dot(on_second, second_mean.linear_m_s - first_mean.linear_m_s) + angular_first * first_mean.angular_rad_s + angular_second * second_mean.angular_rad_s;
            };
            const auto normal = manifold.normal * manifold.applied_normal_impulse_n_s;
            manifold.energy.normal_energy_change_j = work(normal, manifold.applied_normal_angular_impulse_first_n_m_s, manifold.applied_normal_angular_impulse_second_n_m_s);
            manifold.energy.friction_energy_change_j = work(manifold.applied_impulse_on_second_n_s - normal,
                manifold.applied_angular_impulse_first_n_m_s - manifold.applied_normal_angular_impulse_first_n_m_s,
                manifold.applied_angular_impulse_second_n_m_s - manifold.applied_normal_angular_impulse_second_n_m_s);
        }
    }

    // Joint and motor impulses are whatever the velocity solve changed beyond the contact
    // impulses, measured the same way.
    void World::finish_ledger_velocity_solve(const std::vector<IntegratedMotion>& starts)
    {
        auto& step = ledger_step_;
        step.solved_velocities.assign(slots_.size(), {});
        auto& contacts = step.contact_impulses;
        contacts.assign(slots_.size(), {});
        for (const auto& manifold : manifolds_)
        {
            if (manifold.is_sensor || manifold.first.index >= contacts.size() || manifold.second.index >= contacts.size())
                continue;
            auto& first = contacts[manifold.first.index];
            auto& second = contacts[manifold.second.index];
            first.linear_m_s -= manifold.applied_impulse_on_second_n_s;
            first.angular_rad_s += manifold.applied_angular_impulse_first_n_m_s;
            second.linear_m_s += manifold.applied_impulse_on_second_n_s;
            second.angular_rad_s += manifold.applied_angular_impulse_second_n_m_s;
        }
        step.joint_work_j = 0.0;
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                step.solved_velocities[id.index] = { body.linear_velocity_m_s(), body.angular_velocity_rad_s() };
                if (body.type() != BodyType::dynamic_body || step.force_changes.size() != slots_.size())
                    return;
                const auto& start = starts[id.index];
                const auto& change = step.force_changes[id.index];
                const auto& contact = contacts[id.index];
                const auto inertia = body.inverse_inertia() > 0.0 ? body.mass_properties().inertia_kg_m2 : 0.0;
                const auto joint_linear = (body.linear_velocity_m_s() - start.linear_velocity_m_s - change.linear_m_s) * body.mass_properties().mass_kg - contact.linear_m_s;
                const auto joint_angular = inertia > 0.0 ? (body.angular_velocity_rad_s() - start.angular_velocity_rad_s - change.angular_rad_s) * inertia - contact.angular_rad_s : 0.0;
                step.joint_work_j += 0.5 * (math::dot(joint_linear, start.linear_velocity_m_s + body.linear_velocity_m_s()) + joint_angular * (start.angular_velocity_rad_s + body.angular_velocity_rad_s()));
            });
        // A motor's part is its own row's impulse against the same mean velocities, so a travel
        // stop that takes the energy back is booked to the joint rather than netted with it.
        step.motor_work_j = 0.0;
        const auto mean = [&](BodyId id)
        {
            const auto* body = find_body(id);
            if (!body || body->type() != BodyType::dynamic_body || id.index >= starts.size())
                return LedgerVelocity {};
            const auto& start = starts[id.index];
            return LedgerVelocity { (start.linear_velocity_m_s + body->linear_velocity_m_s()) * 0.5, 0.5 * (start.angular_velocity_rad_s + body->angular_velocity_rad_s()) };
        };
        for (const auto& constraint : constraints_)
        {
            const auto* joint = constraint_active(*constraint) ? dynamic_cast<const JointConstraint*>(constraint.get()) : nullptr;
            const auto* row = joint ? joint->solved_motor_row() : nullptr;
            if (!row)
                continue;
            const auto first = mean(row->first);
            const auto second = mean(row->second);
            step.motor_work_j += row->impulse * (math::dot(row->linear_first, first.linear_m_s) + row->angular_first * first.angular_rad_s + math::dot(row->linear_second, second.linear_m_s) + row->angular_second * second.angular_rad_s);
        }
    }

    void World::record_ledger_integrated_positions()
    {
        auto& centers = ledger_step_.integrated_centers_m;
        centers.assign(slots_.size(), {});
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                centers[id.index] = body.world_center_of_mass_m();
            });
    }

    void World::finish_energy_ledger(Real time_step_s, const std::vector<IntegratedMotion>& starts)
    {
        auto& step = ledger_step_;
        if (step.solved_velocities.size() != slots_.size() || step.integrated_centers_m.size() != slots_.size() || step.force_changes.size() != slots_.size())
            return;
        EnergyLedger work;
        work.joint_work_j = step.joint_work_j;
        work.motor_work_j = step.motor_work_j;
        auto& jointed = step.jointed;
        jointed.assign(slots_.size(), false);
        for (const auto& constraint : constraints_)
            if (constraint_active(*constraint))
                for (const auto id : { constraint->first_body(), constraint->second_body() })
                    if (id.index < jointed.size())
                        jointed[id.index] = true;
        static const auto gravity_channel = UniformGravity {}.name();
        // AerodynamicDrag reports its spin resistance and Magnus lift as channels of their own.
        static const auto air_channel = AerodynamicDrag {}.name();
        const auto air = [](std::string_view name)
        {
            return name == air_channel || name == "angular_drag" || name == "magnus";
        };
        const auto staged = integrator_->requires_force_evaluation();
        // Loads act on bodies that were awake when the step integrated them.
        const auto mean = [&](BodyId id, const RigidBody& body)
        {
            if (body.type() != BodyType::dynamic_body || !step.integrated[id.index])
                return LedgerVelocity {};
            const auto& start = starts[id.index];
            const auto& solved = step.solved_velocities[id.index];
            return LedgerVelocity { (start.linear_velocity_m_s + solved.linear_m_s) * 0.5, 0.5 * (start.angular_velocity_rad_s + solved.angular_rad_s) };
        };
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                if (body.type() != BodyType::dynamic_body)
                    return;
                const auto average = mean(id, body);
                Real load_work_j = 0.0;
                for (const auto& channel : body.applied_force_channels())
                {
                    const auto channel_work_j = (math::dot(channel.force_n, average.linear_m_s) + channel.torque_n_m * average.angular_rad_s) * time_step_s;
                    load_work_j += channel_work_j;
                    if (channel.name == gravity_channel || channel.name.rfind("spring/", 0) == 0)
                        continue;
                    (air(channel.name) ? work.air_work_j : work.applied_work_j) += channel_work_j;
                }
                // Semi-implicit Euler applies a body's velocity damping after summing its loads.
                if (!staged && step.integrated[id.index])
                {
                    const auto& change = step.force_changes[id.index];
                    const auto inertia = body.inverse_inertia() > 0.0 ? body.mass_properties().inertia_kg_m2 : 0.0;
                    work.air_work_j += body.mass_properties().mass_kg * math::dot(change.linear_m_s, average.linear_m_s) + inertia * change.angular_rad_s * average.angular_rad_s - load_work_j;
                }
                // Position correction and continuous collision move a body without changing its
                // velocity; the height it gains or loses is work done by its contacts or joints.
                const auto moved = body.world_center_of_mass_m() - step.integrated_centers_m[id.index];
                const auto lifted_j = -body.mass_properties().mass_kg * math::dot(effective_uniform_gravity_m_s2(*this, id), moved);
                (jointed[id.index] ? work.joint_work_j : work.contact_correction_work_j) += lifted_j;
            });
        for (const auto& damper : step.dampers)
        {
            const auto* first = find_body(damper.first);
            const auto* second = find_body(damper.second);
            if (!first || !second)
                continue;
            const auto first_mean = mean(damper.first, *first);
            const auto second_mean = mean(damper.second, *second);
            if (damper.angular)
                work.damper_work_j += damper.load * (first_mean.angular_rad_s - second_mean.angular_rad_s) * time_step_s;
            else
            {
                const auto first_point = first_mean.linear_m_s + math::cross(first_mean.angular_rad_s, damper.first_lever_m);
                const auto second_point = second_mean.linear_m_s + math::cross(second_mean.angular_rad_s, damper.second_lever_m);
                work.damper_work_j += damper.load * math::dot(damper.axis, first_point - second_point) * time_step_s;
            }
        }

        const auto end_energy_j = world_mechanical_energy(*this);
        const auto contact_j = last_step_collision_energy_.normal_energy_change_j + last_step_collision_energy_.friction_energy_change_j;
        work.unattributed_j = end_energy_j - step.start_energy_j - contact_j - work.air_work_j - work.damper_work_j - work.joint_work_j -
            work.applied_work_j - work.contact_correction_work_j;
        energy_ledger_ += work;
        ledger_end_energy_j_ = end_energy_j;
        ledger_has_end_energy_ = true;
    }

    bool World::notify_body_properties_changed(BodyId id)
    {
        auto* body = find_body(id);
        if (!body)
            return false;
        body->wake();
        body->motion_edited_ = true;
        wake_connected_bodies(0.0);
        remove_collision_state(id);
        for (const auto& constraint : constraints_)
            if (constraint->first_body() == id || constraint->second_body() == id)
                constraint->invalidate_cached_impulses();
        proxies_.erase(std::remove_if(proxies_.begin(), proxies_.end(), [id](const auto& proxy)
                           {
                               return proxy.body == id;
                           }),
            proxies_.end());
        enforce_motion_limits();
        refresh_statistics();
        return true;
    }

    void World::begin_collision_accounting(const std::vector<IntegratedMotion>& starts)
    {
        impact_reports_.clear();
        last_step_collision_energy_ = {};
        dropped_impact_report_count_ = 0;
        for (auto& episode : impact_episodes_)
            episode.observed = false;
        for (auto& manifold : manifolds_)
        {
            // A custom solver that does not instrument impulses must not reuse old readings.
            manifold.energy = {};
            manifold.applied_impulse_on_second_n_s = {};
            manifold.applied_angular_impulse_first_n_m_s = manifold.applied_angular_impulse_second_n_m_s = 0.0;
            manifold.applied_normal_impulse_n_s = manifold.applied_tangent_impulse_n_s = 0.0;
            manifold.applied_normal_angular_impulse_first_n_m_s = manifold.applied_normal_angular_impulse_second_n_m_s = 0.0;
            if (manifold.is_sensor || manifold.is_empty())
                continue;
            auto episode = std::find_if(impact_episodes_.begin(), impact_episodes_.end(), [&](const auto& value)
                {
                    return value.report.first_before.id == manifold.first && value.report.second_before.id == manifold.second;
                });
            if (episode == impact_episodes_.end())
            {
                ImpactEpisode value;
                value.report.first_before = collision_state(manifold.first, *find_body(manifold.first));
                value.report.second_before = collision_state(manifold.second, *find_body(manifold.second));
                value.report.step_index = statistics_.step_index + 1;
                value.report.normal = manifold.normal;
                value.report.point_m = manifold.points[0].world_position_m;
                impact_episodes_.push_back(std::move(value));
                episode = impact_episodes_.end() - 1;
            }
            else if (!episode->reported && !episode->has_impulse && !episode->observed)
            {
                // A margin contact may exist long before it transfers momentum. Its incoming
                // state belongs to the first responding step, not the first broad-phase visit.
                episode->report.first_before = collision_state(manifold.first, *find_body(manifold.first));
                episode->report.second_before = collision_state(manifold.second, *find_body(manifold.second));
                episode->report.step_index = statistics_.step_index + 1;
                episode->report.normal = manifold.normal;
                episode->report.point_m = manifold.points[0].world_position_m;
            }
            if (!episode->reported && !episode->has_impulse)
            {
                // Existing zero-speed support acquiring g*dt during force integration is not a
                // new impact. Preserve even very slow pre-existing approach, and any closure
                // from a real gap, without tying inspection to the restitution speed threshold.
                const auto& first_start = starts[manifold.first.index];
                const auto& second_start = starts[manifold.second.index];
                for (std::size_t index = 0; index < manifold.point_count; ++index)
                {
                    const auto& point = manifold.points[index];
                    const auto first_velocity = first_start.linear_velocity_m_s + math::cross(first_start.angular_velocity_rad_s, point.world_position_m - first_start.center_position_m);
                    const auto second_velocity = second_start.linear_velocity_m_s + math::cross(second_start.angular_velocity_rad_s, point.world_position_m - second_start.center_position_m);
                    episode->impact_motion = episode->impact_motion || point.separation_m > math::geometric_epsilon || math::dot(second_velocity - first_velocity, manifold.normal) < -1.0e-10;
                }
            }
            episode->observed = true;
        }
        impact_episodes_.erase(std::remove_if(impact_episodes_.begin(), impact_episodes_.end(), [](const auto& value)
                                   {
                                       return !value.observed;
                                   }),
            impact_episodes_.end());
    }

    void World::finish_collision_accounting(Real time_step_s)
    {
        for (auto& episode : impact_episodes_)
        {
            auto& report = episode.report;
            bool touching = false;
            bool impulse_applied = false;
            for (const auto& manifold : manifolds_)
            {
                if (manifold.is_sensor || !(manifold.first == report.first_before.id) || !(manifold.second == report.second_before.id))
                    continue;
                last_step_collision_energy_ += manifold.energy;
                if (episode.reported)
                    continue;
                report.energy += manifold.energy;
                report.impulse_on_second_n_s += manifold.applied_impulse_on_second_n_s;
                report.angular_impulse_first_n_m_s += manifold.applied_angular_impulse_first_n_m_s;
                report.angular_impulse_second_n_m_s += manifold.applied_angular_impulse_second_n_m_s;
                report.normal_impulse_n_s += manifold.applied_normal_impulse_n_s;
                report.tangent_impulse_n_s += manifold.applied_tangent_impulse_n_s;
                report.involved_speculation = report.involved_speculation || manifold.is_speculative;
                for (std::size_t index = 0; index < manifold.point_count; ++index)
                    touching = touching || manifold.points[index].separation_m <= math::geometric_epsilon;
                impulse_applied = impulse_applied || manifold.applied_normal_impulse_n_s > 1.0e-10;
            }
            episode.has_impulse = episode.has_impulse || impulse_applied;
            if (touching && impulse_applied && !episode.impact_motion)
                episode.reported = true; // Suppress ongoing support for this contact episode.
            if (episode.reported || !touching || !impulse_applied)
                continue;
            report.first_after = collision_state(report.first_before.id, *find_body(report.first_before.id));
            report.second_after = collision_state(report.second_before.id, *find_body(report.second_before.id));
            report.spans_multiple_steps = report.step_index != statistics_.step_index + 1;
            report.step_index = statistics_.step_index + 1;
            report.elapsed_time_s = statistics_.elapsed_time_s + time_step_s;
            report.coupled_constraints = std::any_of(constraints_.begin(), constraints_.end(), [&](const auto& constraint)
                {
                    return constraint_active(*constraint) && (constraint->first_body() == report.first_before.id || constraint->second_body() == report.first_before.id || constraint->first_body() == report.second_before.id || constraint->second_body() == report.second_before.id);
                });
            report.coupled_contacts = std::any_of(manifolds_.begin(), manifolds_.end(), [&](const auto& other)
                {
                    return !other.is_sensor && !(other.first == report.first_before.id && other.second == report.second_before.id) &&
                        (other.first == report.first_before.id || other.second == report.first_before.id || other.first == report.second_before.id || other.second == report.second_before.id) &&
                        (math::length_squared(other.applied_impulse_on_second_n_s) > 1.0e-20 || other.applied_angular_impulse_first_n_m_s != 0.0 || other.applied_angular_impulse_second_n_m_s != 0.0);
                });
            episode.reported = true;
            if (impact_reports_.size() < maximum_impact_reports_per_step)
                impact_reports_.push_back(report);
            else
                ++dropped_impact_report_count_;
        }
        collision_energy_ += last_step_collision_energy_;
    }
}
