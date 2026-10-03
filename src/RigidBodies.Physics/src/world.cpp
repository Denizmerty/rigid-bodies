#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/constraint_graph.hpp>

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {
        // Growth is geometric, while allocation finishes before publishing a new identifier.
        // RigidBody's value members must remain safely relocatable for packed storage.
        static_assert(std::is_nothrow_move_constructible_v<RigidBody>);
        static_assert(std::is_nothrow_move_assignable_v<RigidBody>);

        template <typename Value>
        void reserve_additional(std::vector<Value>& values, std::size_t count = 1)
        {
            const auto required = values.size() + count;
            if (required > values.capacity())
                values.reserve(std::max(required, values.capacity() + values.capacity() / 2 + 1));
        }

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

        template <typename Component>
        std::vector<std::shared_ptr<Component>> clone_components(const std::vector<std::shared_ptr<Component>>& sources)
        {
            std::unordered_map<const Component*, std::shared_ptr<Component>> clones;
            return clone_components(sources, clones);
        }

        void validate_settings(const WorldSettings& value)
        {
            const auto nonnegative = [](Real number)
            {
                return math::is_finite(number) && number >= 0.0;
            };
            const auto positive = [&](Real number)
            {
                return nonnegative(number) && number > 0.0;
            };
            const auto& sleep = value.sleep;
            const auto& limits = value.limits;
            const auto& collision = value.collision;
            if (!math::is_finite(value.gravity_m_s2) || !nonnegative(value.air_density_kg_m3) ||
                !math::is_finite(value.air_velocity_m_s) || !positive(value.air_dynamic_viscosity_pa_s) ||
                !nonnegative(value.broad_phase_margin_m) || !nonnegative(sleep.linear_speed_m_s) ||
                !nonnegative(sleep.angular_speed_rad_s) || !nonnegative(sleep.linear_acceleration_m_s2) ||
                !nonnegative(sleep.angular_acceleration_rad_s2) || !positive(sleep.quiet_duration_s) ||
                !positive(limits.maximum_position_m) || !positive(limits.maximum_linear_speed_m_s) ||
                !positive(limits.maximum_angular_speed_rad_s) || !positive(limits.maximum_orientation_rad) ||
                !nonnegative(collision.contact_margin_m) || !positive(collision.matching_tolerance_m) ||
                !positive(collision.sweep_tolerance_m) || collision.maximum_sweep_iterations < 1 || collision.maximum_sweep_iterations > 1024 ||
                value.solver.velocity_iterations < 0 || value.solver.velocity_iterations > 512 ||
                value.solver.position_iterations < 0 || value.solver.position_iterations > 512 ||
                !nonnegative(value.solver.linear_slop_m) || !nonnegative(value.solver.restitution_threshold_m_s) ||
                !nonnegative(value.solver.position_correction_fraction) || value.solver.position_correction_fraction > 1.0 ||
                !positive(value.solver.maximum_position_correction_m))
            {
                throw std::invalid_argument("World environment, sleep thresholds and motion limits must be finite and valid");
            }
        }
    }

    World::World() : World(WorldSettings {})
    {
    }

    World::World(const WorldSettings& settings) : settings_(settings),
                                                  integrator_(std::make_shared<SemiImplicitEulerIntegrator>()),
                                                  broad_phase_(std::make_shared<DynamicTreeBroadPhase>()),
                                                  narrow_phase_(std::make_shared<CollisionNarrowPhase>()),
                                                  contact_solver_(std::make_shared<SequentialImpulseContactSolver>())
    {
        validate_settings(settings);
        // Gravity is present by default because a playground that starts with nothing falling is
        // harder to make sense of than one that does. Scenarios are free to disable it.
        force_generators_.push_back(std::make_shared<UniformGravity>());
    }

    const WorldSettings& World::settings() const
    {
        return settings_;
    }

    std::string_view World::body_document_id(BodyId id) const
    {
        return is_valid(id) ? std::string_view(slots_[id.index].document_id) : std::string_view {};
    }

    void World::set_settings(const WorldSettings& value)
    {
        validate_settings(value);
        settings_ = value;
        for_each_body([](BodyId, RigidBody& body)
            {
                body.wake();
            });
        enforce_motion_limits();
    }

    Real World::potential_energy_reference_height_m() const
    {
        return potential_energy_reference_height_m_;
    }

    void World::set_potential_energy_reference_height(Real value)
    {
        potential_energy_reference_height_m_ = value;
    }

    BodyId World::create_body(const BodyDefinition& definition, std::string stable_key)
    {
        if (!stable_key.empty())
        {
            for (const auto index : ordered_slots_)
            {
                if (slots_[index].stable_key == stable_key)
                {
                    throw std::invalid_argument("A live body already uses this stable key");
                }
            }
        }

        RigidBody body(definition);
        reserve_additional(bodies_);
        reserve_additional(dense_slots_);
        reserve_additional(ordered_slots_);
        if (free_slots_.empty())
            reserve_additional(slots_);
        std::uint32_t index = 0;
        if (!free_slots_.empty())
        {
            index = free_slots_.back();
            free_slots_.pop_back();
        }
        else
        {
            index = static_cast<std::uint32_t>(slots_.size());
            slots_.emplace_back();
        }

        auto& slot = slots_[index];
        slot.dense_index = bodies_.size();
        bodies_.push_back(std::move(body));
        dense_slots_.push_back(index);
        slot.stable_key = std::move(stable_key);
        slot.document_id.clear();
        slot.creation_order = next_creation_order_++;
        slot.generation = next_generation_++;
        if (next_generation_ == 0)
        {
            // Generation zero marks an invalid identifier, so the counter skips it on wrap.
            next_generation_ = 1;
        }

        const auto position = std::lower_bound(ordered_slots_.begin(), ordered_slots_.end(), index, [this](std::uint32_t first, std::uint32_t second)
            {
                return precedes(first, second);
            });
        ordered_slots_.insert(position, index);
        enforce_motion_limits();
        return { index, slot.generation };
    }

    bool World::precedes(std::uint32_t first, std::uint32_t second) const
    {
        const auto& left = slots_[first];
        const auto& right = slots_[second];
        if (left.stable_key.empty() != right.stable_key.empty())
        {
            return !left.stable_key.empty();
        }
        if (left.stable_key != right.stable_key)
        {
            return left.stable_key < right.stable_key;
        }
        return left.creation_order < right.creation_order;
    }

    bool World::destroy_body(BodyId id)
    {
        if (!is_valid(id))
        {
            return false;
        }

        // Reserve every removal queue before changing live state. Compaction itself consists
        // only of noexcept moves, so failures cannot leave a dangling slot-to-body mapping.
        reserve_additional(free_slots_);
        const auto springs = spring_ids();
        reserve_additional(free_spring_slots_, springs.size());
        reserve_additional(pending_pair_events_, active_pairs_.size());
        reserve_additional(pending_contact_events_, active_contacts_.size());
        for (const auto spring_id : springs)
        {
            const auto& definition = *spring_definition(spring_id);
            if (std::visit([&](const auto& spring)
                    {
                        return spring.first == id || spring.second == id;
                    },
                    definition))
                remove_spring(spring_id);
        }
        const auto wake_neighbor = [&](BodyId neighbor)
        {
            if (auto* body = find_body(neighbor))
                body->wake();
        };
        // Removing a support changes the dynamics even when no geometry moves into a sleeper.
        // Preserve that wake request before the destroyed identifier becomes invalid.
        for (const auto& manifold : manifolds_)
        {
            if (manifold.is_sensor || manifold.is_empty())
                continue;
            if (manifold.first == id)
                wake_neighbor(manifold.second);
            else if (manifold.second == id)
                wake_neighbor(manifold.first);
        }
        for (const auto& constraint : constraints_)
        {
            if (!constraint || !constraint->is_enabled())
                continue;
            if (constraint->first_body() == id)
                wake_neighbor(constraint->second_body());
            else if (constraint->second_body() == id)
                wake_neighbor(constraint->first_body());
        }
        for (std::size_t index = constraints_.size(); index > 0; --index)
        {
            const auto& constraint = constraints_[index - 1];
            if (constraint->first_body() == id || constraint->second_body() == id)
            {
                constraints_.erase(constraints_.begin() + static_cast<std::ptrdiff_t>(index - 1));
                constraint_registrations_.erase(constraint_registrations_.begin() + static_cast<std::ptrdiff_t>(index - 1));
            }
        }
        remove_collision_state(id);
        ordered_slots_.erase(std::remove(ordered_slots_.begin(), ordered_slots_.end(), id.index), ordered_slots_.end());
        auto& slot = slots_[id.index];
        const auto dense_index = slot.dense_index;
        if (dense_index + 1 != bodies_.size())
        {
            bodies_[dense_index] = std::move(bodies_.back());
            const auto moved_slot = dense_slots_.back();
            dense_slots_[dense_index] = moved_slot;
            slots_[moved_slot].dense_index = dense_index;
        }
        bodies_.pop_back();
        dense_slots_.pop_back();
        slot.dense_index = vacant_body_index;
        slot.generation = 0;
        slot.stable_key.clear();
        slot.document_id.clear();
        slot.force_generators.clear();
        slot.applied_spring_load = {};
        slot.motion.reset();
        free_slots_.push_back(id.index);
        return true;
    }

    void World::clear()
    {
        bodies_.clear();
        dense_slots_.clear();
        slots_.clear();
        free_slots_.clear();
        ordered_slots_.clear();
        proxies_.clear();
        pairs_.clear();
        manifolds_.clear();
        active_pairs_.clear();
        active_contacts_.clear();
        pair_events_.clear();
        contact_events_.clear();
        pending_pair_events_.clear();
        pending_contact_events_.clear();
        previous_contact_step_s_ = 0.0;
        constraints_.clear();
        constraint_registrations_.clear();
        constraint_break_events_.clear();
        spring_slots_.clear();
        free_spring_slots_.clear();
        ordered_spring_slots_.clear();
        motion_limit_events_.clear();
        collision_energy_ = {};
        last_step_collision_energy_ = {};
        impact_reports_.clear();
        impact_episodes_.clear();
        dropped_impact_report_count_ = 0;
        reset_energy_ledger();
        profile_ = {};
        islands_.clear();
        island_work_.clear();
        island_by_slot_.clear();
        statistics_ = {};
    }

    std::unique_ptr<World> World::clone(bool independent_components) const
    {
        auto copy = std::make_unique<World>(settings_);
        copy->potential_energy_reference_height_m_ = potential_energy_reference_height_m_;
        copy->bodies_ = bodies_;
        copy->dense_slots_ = dense_slots_;
        copy->slots_.resize(slots_.size());
        std::unordered_map<const Shape*, ShapePtr> shapes;
        std::unordered_map<const ForceGenerator*, ForceGeneratorPtr> force_generators;
        copy->force_generators_ = independent_components ? clone_components(force_generators_, force_generators) : force_generators_;
        for (std::size_t index = 0; index < slots_.size(); ++index)
        {
            const auto& source = slots_[index];
            auto& target = copy->slots_[index];
            target.dense_index = source.dense_index;
            target.generation = source.generation;
            target.stable_key = source.stable_key;
            target.document_id = source.document_id;
            target.creation_order = source.creation_order;
            target.motion = source.motion;
            target.motion_start_time_s = source.motion_start_time_s;
            target.force_generators = independent_components ? clone_components(source.force_generators, force_generators) : source.force_generators;
            target.applied_spring_load = source.applied_spring_load;
            if (source.dense_index != vacant_body_index)
            {
                for (auto& collider : copy->bodies_[target.dense_index].colliders_)
                {
                    if (independent_components && collider.shape)
                    {
                        auto& shape = shapes[collider.shape.get()];
                        if (!shape)
                        {
                            shape = clone_component(collider.shape);
                        }
                        collider.shape = shape;
                    }
                }
            }
        }
        copy->free_slots_ = free_slots_;
        copy->ordered_slots_ = ordered_slots_;
        copy->next_generation_ = next_generation_;
        copy->next_creation_order_ = next_creation_order_;
        copy->constraints_ = independent_components ? clone_components(constraints_) : constraints_;
        copy->constraint_registrations_ = constraint_registrations_;
        copy->constraint_break_events_ = constraint_break_events_;
        copy->spring_slots_ = spring_slots_;
        copy->free_spring_slots_ = free_spring_slots_;
        copy->ordered_spring_slots_ = ordered_spring_slots_;
        copy->next_spring_generation_ = next_spring_generation_;
        copy->next_spring_creation_order_ = next_spring_creation_order_;
        copy->integrator_ = independent_components ? clone_component(integrator_) : integrator_;
        copy->broad_phase_ = independent_components ? clone_component(broad_phase_) : broad_phase_;
        copy->narrow_phase_ = independent_components ? clone_component(narrow_phase_) : narrow_phase_;
        copy->contact_solver_ = independent_components ? clone_component(contact_solver_) : contact_solver_;
        copy->parallel_settings_ = parallel_settings_;
        copy->profiling_enabled_ = profiling_enabled_;
        copy->islands_ = islands_;
        copy->island_by_slot_ = island_by_slot_;
        copy->proxies_ = proxies_;
        copy->pairs_ = pairs_;
        copy->manifolds_ = manifolds_;
        copy->active_pairs_ = active_pairs_;
        copy->active_contacts_ = active_contacts_;
        copy->pair_events_ = pair_events_;
        copy->contact_events_ = contact_events_;
        copy->pending_pair_events_ = pending_pair_events_;
        copy->pending_contact_events_ = pending_contact_events_;
        copy->previous_contact_step_s_ = previous_contact_step_s_;
        copy->statistics_ = statistics_;
        copy->motion_limit_events_ = motion_limit_events_;
        copy->collision_energy_ = collision_energy_;
        copy->last_step_collision_energy_ = last_step_collision_energy_;
        copy->impact_reports_ = impact_reports_;
        copy->impact_episodes_ = impact_episodes_;
        copy->dropped_impact_report_count_ = dropped_impact_report_count_;
        copy->energy_ledger_ = energy_ledger_;
        copy->ledger_end_energy_j_ = ledger_end_energy_j_;
        copy->ledger_has_end_energy_ = ledger_has_end_energy_;
        copy->energy_time_step_s_ = energy_time_step_s_;
        return copy;
    }

    WorldSnapshot World::snapshot() const
    {
        WorldSnapshot result;
        result.state_ = clone();
        return result;
    }

    void World::restore(const WorldSnapshot& snapshot)
    {
        if (!snapshot.state_)
        {
            throw std::invalid_argument("Cannot restore an empty world snapshot");
        }
        auto restored = snapshot.state_->clone();
        *this = std::move(*restored);
    }

    void World::restart_timeline()
    {
        const auto elapsed = statistics_.elapsed_time_s;
        for (auto& slot : slots_)
            if (slot.dense_index != vacant_body_index && slot.motion)
                slot.motion_start_time_s -= elapsed;
        pair_events_.clear();
        contact_events_.clear();
        pending_pair_events_.clear();
        pending_contact_events_.clear();
        constraint_break_events_.clear();
        motion_limit_events_.clear();
        collision_energy_ = {};
        last_step_collision_energy_ = {};
        impact_reports_.clear();
        impact_episodes_.clear();
        dropped_impact_report_count_ = 0;
        reset_energy_ledger();
        profile_ = {};
        statistics_ = {};
        refresh_statistics();
    }

    RigidBody* World::find_body(BodyId id)
    {
        return is_valid(id) ? &bodies_[slots_[id.index].dense_index] : nullptr;
    }

    const RigidBody* World::find_body(BodyId id) const
    {
        return is_valid(id) ? &bodies_[slots_[id.index].dense_index] : nullptr;
    }

    bool World::is_valid(BodyId id) const
    {
        if (!id.is_valid() || id.index >= slots_.size())
        {
            return false;
        }
        const auto& slot = slots_[id.index];
        return slot.dense_index != vacant_body_index && slot.generation == id.generation;
    }

    math::Span<const RigidBody> World::contiguous_bodies() const
    {
        return bodies_;
    }

    std::size_t World::body_storage_capacity() const
    {
        return bodies_.capacity();
    }

    std::vector<BodyId> World::body_ids() const
    {
        std::vector<BodyId> identifiers;
        identifiers.reserve(ordered_slots_.size());
        for (const auto index : ordered_slots_)
        {
            const auto& slot = slots_[index];
            identifiers.push_back({ index, slot.generation });
        }
        return identifiers;
    }

    void World::for_each_body(const std::function<void(BodyId, RigidBody&)>& visitor)
    {
        const auto identifiers = body_ids();
        for (const auto id : identifiers)
        {
            if (auto* body = find_body(id))
            {
                visitor(id, *body);
            }
        }
    }

    void World::for_each_body(const std::function<void(BodyId, const RigidBody&)>& visitor) const
    {
        const auto identifiers = body_ids();
        for (const auto id : identifiers)
        {
            if (const auto* body = find_body(id))
            {
                visitor(id, *body);
            }
        }
    }

    void World::add_force_generator(ForceGeneratorPtr generator)
    {
        if (generator)
        {
            force_generators_.push_back(std::move(generator));
        }
    }

    void World::remove_force_generator(const ForceGeneratorPtr& generator)
    {
        // Callers may pass a reference obtained from force_generators(); compaction must not
        // overwrite the value against which later registrations are compared.
        // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
        const auto target = generator;
        force_generators_.erase(std::remove(force_generators_.begin(), force_generators_.end(), target), force_generators_.end());
    }

    const std::vector<ForceGeneratorPtr>& World::force_generators() const
    {
        return force_generators_;
    }

    bool World::add_force_generator(BodyId body, ForceGeneratorPtr generator)
    {
        if (!is_valid(body) || !generator)
        {
            return false;
        }
        slots_[body.index].force_generators.push_back(std::move(generator));
        return true;
    }

    bool World::remove_force_generator(BodyId body, const ForceGeneratorPtr& generator)
    {
        if (!is_valid(body) || !generator)
        {
            return false;
        }
        auto& generators = slots_[body.index].force_generators;
        const auto previous_count = generators.size();
        // Keep an owning snapshot when generator aliases an element of generators.
        // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
        const auto target = generator;
        generators.erase(std::remove(generators.begin(), generators.end(), target), generators.end());
        return generators.size() != previous_count;
    }

    const std::vector<ForceGeneratorPtr>& World::force_generators(BodyId body) const
    {
        static const std::vector<ForceGeneratorPtr> empty;
        return is_valid(body) ? slots_[body.index].force_generators : empty;
    }

    void World::add_constraint(ConstraintPtr constraint, std::string stable_key)
    {
        if (!constraint)
            return;
        if (!is_valid(constraint->first_body()) || !is_valid(constraint->second_body()) || constraint->first_body() == constraint->second_body())
            throw std::invalid_argument("Constraint endpoints must be distinct live bodies");
        if (std::find(constraints_.begin(), constraints_.end(), constraint) != constraints_.end())
            throw std::invalid_argument("The constraint is already registered");
        if (!stable_key.empty() && constraint_by_key(stable_key))
            throw std::invalid_argument("A live constraint already uses this stable key");
        std::size_t index = constraints_.size();
        if (!stable_key.empty())
            for (std::size_t candidate = 0; candidate < constraint_registrations_.size(); ++candidate)
                if (constraint_registrations_[candidate].stable_key.empty() || stable_key < constraint_registrations_[candidate].stable_key)
                {
                    index = candidate;
                    break;
                }
        constraints_.reserve(constraints_.size() + 1);
        constraint_registrations_.reserve(constraint_registrations_.size() + 1);
        constraint_registrations_.insert(constraint_registrations_.begin() + static_cast<std::ptrdiff_t>(index), { std::move(stable_key), constraint->revision(), constraint->first_body(), constraint->second_body() });
        const auto first = constraint->first_body(), second = constraint->second_body();
        constraints_.insert(constraints_.begin() + static_cast<std::ptrdiff_t>(index), std::move(constraint));
        find_body(first)->wake();
        find_body(second)->wake();
    }

    void World::remove_constraint(const ConstraintPtr& constraint)
    {
        const auto found = std::find(constraints_.begin(), constraints_.end(), constraint);
        if (found == constraints_.end())
            return;
        const auto index = static_cast<std::size_t>(found - constraints_.begin());
        if (auto* first = find_body((*found)->first_body()))
            first->wake();
        if (auto* second = find_body((*found)->second_body()))
            second->wake();
        constraints_.erase(found);
        constraint_registrations_.erase(constraint_registrations_.begin() + static_cast<std::ptrdiff_t>(index));
    }

    const std::vector<ConstraintPtr>& World::constraints() const
    {
        return constraints_;
    }

    ConstraintPtr World::constraint_by_key(std::string_view key) const
    {
        if (!key.empty())
            for (std::size_t index = 0; index < constraints_.size(); ++index)
                if (constraint_registrations_[index].stable_key == key)
                    return constraints_[index];
        return {};
    }

    const std::string& World::constraint_key(const ConstraintPtr& constraint) const
    {
        static const std::string empty;
        const auto found = std::find(constraints_.begin(), constraints_.end(), constraint);
        return found == constraints_.end() ? empty : constraint_registrations_[static_cast<std::size_t>(found - constraints_.begin())].stable_key;
    }

    const std::vector<ConstraintBreakEvent>& World::constraint_break_events() const
    {
        return constraint_break_events_;
    }

    bool World::constraint_active(const Constraint& constraint) const
    {
        return constraint.is_enabled() && !constraint.is_broken() &&
            is_valid(constraint.first_body()) && is_valid(constraint.second_body());
    }

    bool World::bodies_can_collide(BodyId first, BodyId second) const
    {
        for (const auto& constraint : constraints_)
            if (constraint_active(*constraint) && !constraint->collide_connected() &&
                ((constraint->first_body() == first && constraint->second_body() == second) ||
                    (constraint->first_body() == second && constraint->second_body() == first)))
                return false;
        return true;
    }

    void World::synchronize_constraints()
    {
        for (std::size_t index = 0; index < constraints_.size(); ++index)
        {
            const auto& constraint = *constraints_[index];
            auto& registration = constraint_registrations_[index];
            if (registration.observed_revision != constraint.revision() ||
                (constraint_active(constraint) && constraint.has_active_drive()))
            {
                for (const auto id : { registration.first, registration.second, constraint.first_body(), constraint.second_body() })
                    if (auto* body = find_body(id))
                    {
                        body->wake();
                        body->motion_edited_ = true;
                    }
                registration.observed_revision = constraint.revision();
                registration.first = constraint.first_body();
                registration.second = constraint.second_body();
            }
        }
    }

    void World::set_integrator(IntegratorPtr integrator)
    {
        if (integrator)
        {
            integrator_ = std::move(integrator);
        }
    }

    const Integrator& World::integrator() const
    {
        return *integrator_;
    }

    void World::set_broad_phase(BroadPhasePtr broad_phase)
    {
        if (broad_phase)
        {
            broad_phase_ = std::move(broad_phase);
        }
    }

    const BroadPhase& World::broad_phase() const
    {
        return *broad_phase_;
    }

    void World::set_narrow_phase(NarrowPhasePtr narrow_phase)
    {
        if (narrow_phase)
        {
            narrow_phase_ = std::move(narrow_phase);
            // Geometry supplied by the previous implementation cannot establish support or
            // warm a contact from its replacement. Keep event history for next-step end events.
            manifolds_.clear();
            previous_contact_step_s_ = 0.0;
            for_each_body([](BodyId, RigidBody& body)
                {
                    body.wake();
                });
        }
    }

    const NarrowPhase& World::narrow_phase() const
    {
        return *narrow_phase_;
    }

    void World::set_contact_solver(ContactSolverPtr solver)
    {
        if (solver)
        {
            contact_solver_ = std::move(solver);
            // A different response model may remove support entirely. Invalidate its predecessor's
            // impulses and wake settled bodies so unchanged gravity can act under the new model.
            for (auto& manifold : manifolds_)
            {
                for (auto& point : manifold.points)
                {
                    point.normal_impulse_n_s = 0.0;
                    point.tangent_impulse_n_s = 0.0;
                    point.rolling_impulse_n_m_s = 0.0;
                    point.spinning_impulse_n_m_s = 0.0;
                    point.pending_impact_speed_m_s = 0.0;
                }
            }
            previous_contact_step_s_ = 0.0;
            for_each_body([](BodyId, RigidBody& body)
                {
                    body.wake();
                });
        }
    }

    const ContactSolver& World::contact_solver() const
    {
        return *contact_solver_;
    }

    const WorldStatistics& World::statistics() const
    {
        return statistics_;
    }

    const std::vector<ContactManifold>& World::manifolds() const
    {
        return manifolds_;
    }

    const std::vector<BroadPhasePair>& World::broad_phase_pairs() const
    {
        return pairs_;
    }

    math::Aabb World::compute_bounds() const
    {
        math::Aabb bounds;
        for_each_body([&bounds](BodyId, const RigidBody& body)
            {
                bounds.expand(body.compute_bounds());
            });
        return bounds;
    }

    BodyId World::find_body_at_point(const math::Vec2& world_point_m) const
    {
        BodyId found;
        // Rendering follows canonical body order, so the last hit is the visible topmost body.
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                if (body.contains_world_point(world_point_m))
                {
                    found = id;
                }
            });
        return found;
    }

    void World::apply_forces(Real time_step_s)
    {
        ForceContext context;
        context.gravity_m_s2 = settings_.gravity_m_s2;
        context.air_density_kg_m3 = settings_.air_density_kg_m3;
        context.time_step_s = time_step_s;
        context.elapsed_time_s = statistics_.elapsed_time_s;
        context.air_velocity_m_s = settings_.air_velocity_m_s;
        context.air_dynamic_viscosity_pa_s = settings_.air_dynamic_viscosity_pa_s;

        const auto apply = [&context](RigidBody& body, const ForceGeneratorPtr& generator)
        {
            // A failing custom generator must not label later manual forces as its own.
            struct ChannelScope
            {
                std::string& channel;
                ~ChannelScope()
                {
                    channel.clear();
                }
            } scope { body.active_force_channel_ };
            body.active_force_channel_ = generator->name();
            generator->apply(body, context);
        };

        // Preserve global registration order and canonical body traversal. Body attachments run
        // afterwards, in canonical body order and then each body's registration order.
        for (const auto& generator : force_generators_)
        {
            if (!generator || !generator->is_enabled())
            {
                continue;
            }
            for_each_body([&](BodyId, RigidBody& body)
                {
                    apply(body, generator);
                });
        }
        for_each_body([&](BodyId id, RigidBody& body)
            {
                for (const auto& generator : slots_[id.index].force_generators)
                {
                    if (generator && generator->is_enabled())
                    {
                        apply(body, generator);
                    }
                }
            });
        const auto ids = body_ids();
        std::vector<RigidBody*> bodies;
        std::vector<ForceSample> before_springs;
        bodies.reserve(ids.size());
        for (const auto id : ids)
        {
            auto& body = *find_body(id);
            const auto& prior_spring = slots_[id.index].applied_spring_load;
            // A sleeping spring group retains its final settled configuration. Its own final
            // sub-threshold movement or damping cannot be interpreted as a new external load.
            // Endpoint edits, moving drives and spring edits wake through their explicit paths.
            if (!body.is_awake() &&
                (math::length(body.accumulated_force_n() - (body.applied_force_n() - prior_spring.force_n)) > 1.0e-12 ||
                    std::abs(body.accumulated_torque_n_m() - (body.applied_torque_n_m() - prior_spring.torque_n_m)) > 1.0e-12))
                body.wake();
            bodies.push_back(&body);
            before_springs.push_back({ body.accumulated_force_n(), body.accumulated_torque_n_m() });
        }
        apply_springs(ids, bodies);
        for (std::size_t index = 0; index < ids.size(); ++index)
            slots_[ids[index].index].applied_spring_load = {
                bodies[index]->accumulated_force_n() - before_springs[index].force_n,
                bodies[index]->accumulated_torque_n_m() - before_springs[index].torque_n_m
            };
    }

    void World::refresh_statistics()
    {
        statistics_.body_count = 0;
        statistics_.dynamic_body_count = 0;
        statistics_.sleeping_body_count = 0;
        statistics_.total_linear_momentum_kg_m_s = {};
        statistics_.total_kinetic_energy_j = 0.0;
        statistics_.total_potential_energy_j = 0.0;
        statistics_.total_spring_potential_energy_j = 0.0;
        statistics_.active_constraint_count = 0;
        statistics_.broken_constraint_count = 0;
        for (const auto& constraint : constraints_)
        {
            statistics_.active_constraint_count += constraint_active(*constraint) ? 1 : 0;
            statistics_.broken_constraint_count += constraint->is_broken() ? 1 : 0;
        }

        for_each_body([&](BodyId id, const RigidBody& body)
            {
                ++statistics_.body_count;
                if (body.type() != BodyType::dynamic_body)
                {
                    return;
                }

                ++statistics_.dynamic_body_count;
                if (!body.is_awake())
                    ++statistics_.sleeping_body_count;
                statistics_.total_linear_momentum_kg_m_s += body.linear_momentum_kg_m_s();
                statistics_.total_kinetic_energy_j += body.kinetic_energy_j();

                // Height is measured against the gravity direction rather than against the vertical
                // axis, so the figure stays meaningful when a scenario tilts the field.
                const auto center = body.world_center_of_mass_m();
                statistics_.total_potential_energy_j -= body.mass_properties().mass_kg *
                    math::dot(center - math::Vec2 { 0.0, potential_energy_reference_height_m_ }, effective_uniform_gravity_m_s2(*this, id));
            });

        for (const auto id : spring_ids())
            if (const auto report = spring_report(id))
                statistics_.total_spring_potential_energy_j += report->potential_energy_j;
        statistics_.broad_phase_pair_count = pairs_.size();
        statistics_.manifold_count = manifolds_.size();
        statistics_.contact_point_count = 0;
        for (const auto& manifold : manifolds_)
        {
            statistics_.contact_point_count += manifold.point_count;
        }
    }

} // namespace rigidbodies::physics
