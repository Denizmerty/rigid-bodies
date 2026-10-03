#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/constraint_graph.hpp>
#include <rigidbodies/physics/joint.hpp>

#include <algorithm>
#include <limits>
#include <numeric>

namespace rigidbodies::physics
{
    const WorldProfile& World::profile() const
    {
        return profile_;
    }
    bool World::profiling_enabled() const
    {
        return profiling_enabled_;
    }
    void World::set_profiling_enabled(bool enabled)
    {
        profiling_enabled_ = enabled;
        profile_ = {};
    }
    const std::vector<SimulationIsland>& World::simulation_islands() const
    {
        return islands_;
    }

    void World::build_simulation_islands()
    {
        ScopedProfileTimer timer(profiling_enabled_, profile_.islands_s);
        const auto absent = std::numeric_limits<std::uint32_t>::max();
        islands_.clear();
        island_work_.clear();
        island_by_slot_.assign(slots_.size(), absent);
        const auto ids = body_ids();
        const auto dynamic = [&](BodyId id)
        {
            const auto* body = find_body(id);
            return body && body->type() == BodyType::dynamic_body;
        };
        // Extensions receive World&, so only the built-in endpoint-local implementations
        // can promise that different islands have no hidden dependencies.
        const auto fallback = (!dynamic_cast<SequentialImpulseContactSolver*>(contact_solver_.get()) &&
                                  !dynamic_cast<NullContactSolver*>(contact_solver_.get())) ||
            std::any_of(constraints_.begin(), constraints_.end(), [&](const auto& constraint)
                {
                    return constraint_active(*constraint) && !dynamic_cast<JointConstraint*>(constraint.get());
                });
        if (fallback)
        {
            islands_.emplace_back();
            auto& island = islands_.back();
            island.serial_fallback = true;
            island.awake = true;
            for (const auto id : ids)
                if (dynamic(id))
                {
                    island.bodies.push_back(id);
                    island_by_slot_[id.index] = 0;
                }
            island.manifold_indices.resize(manifolds_.size());
            std::iota(island.manifold_indices.begin(), island.manifold_indices.end(), std::size_t {});
            for (std::size_t i = 0; i < constraints_.size(); ++i)
                if (constraint_active(*constraints_[i]))
                    island.constraint_indices.push_back(i);
            return;
        }
        std::vector<std::uint32_t> parents(slots_.size());
        std::iota(parents.begin(), parents.end(), std::uint32_t {});
        const auto root = [&](std::uint32_t index)
        {
            while (parents[index] != index)
            {
                parents[index] = parents[parents[index]];
                index = parents[index];
            }
            return index;
        };
        const auto join = [&](BodyId first, BodyId second)
        {
            if (!dynamic(first) || !dynamic(second))
                return;
            const auto a = root(first.index), b = root(second.index);
            parents[std::max(a, b)] = std::min(a, b);
        };
        for (const auto& contact : manifolds_)
            if (!contact.is_sensor && !contact.is_empty())
                join(contact.first, contact.second);
        for (const auto& constraint : constraints_)
            if (constraint_active(*constraint))
                join(constraint->first_body(), constraint->second_body());
        for (const auto index : ordered_spring_slots_)
            std::visit([&](const auto& spring)
                {
                    if (spring.enabled)
                        join(spring.first, spring.second);
                },
                *spring_slots_[index].definition);
        std::vector<std::uint32_t> island_by_root(slots_.size(), absent);
        for (const auto id : ids)
        {
            if (!dynamic(id))
                continue;
            auto& index = island_by_root[root(id.index)];
            if (index == absent)
            {
                index = static_cast<std::uint32_t>(islands_.size());
                islands_.emplace_back();
            }
            island_by_slot_[id.index] = index;
            islands_[index].bodies.push_back(id);
            islands_[index].awake = islands_[index].awake || find_body(id)->is_awake();
        }
        const auto island_for = [&](BodyId first, BodyId second)
        {
            return dynamic(first) ? island_by_slot_[first.index] : dynamic(second) ? island_by_slot_[second.index]
                                                                                   : absent;
        };
        const auto moving_boundary = [&](BodyId id)
        {
            const auto* body = find_body(id);
            return body && body->type() == BodyType::kinematic_body &&
                (math::length_squared(body->linear_velocity_m_s()) > 0.0 || body->angular_velocity_rad_s() != 0.0);
        };
        for (std::size_t i = 0; i < manifolds_.size(); ++i)
        {
            const auto& contact = manifolds_[i];
            if (contact.is_sensor || contact.is_empty())
                continue;
            const auto index = island_for(contact.first, contact.second);
            if (index != absent)
            {
                islands_[index].manifold_indices.push_back(i);
                islands_[index].awake = islands_[index].awake || moving_boundary(contact.first) || moving_boundary(contact.second);
            }
        }
        for (std::size_t i = 0; i < constraints_.size(); ++i)
            if (constraint_active(*constraints_[i]))
            {
                const auto index = island_for(constraints_[i]->first_body(), constraints_[i]->second_body());
                if (index != absent)
                {
                    islands_[index].constraint_indices.push_back(i);
                    islands_[index].awake = islands_[index].awake || moving_boundary(constraints_[i]->first_body()) || moving_boundary(constraints_[i]->second_body());
                }
            }
        // One member waking activates the whole group before warm starting.
        for (const auto& island : islands_)
            if (island.awake)
                for (const auto id : island.bodies)
                    if (!find_body(id)->is_awake())
                        find_body(id)->wake();
    }

    void World::solve_island_velocities(Real dt)
    {
        ScopedProfileTimer timer(profiling_enabled_, profile_.velocity_solve_s);
        if (!islands_.empty() && islands_.front().serial_fallback)
        {
            // Custom callbacks can enable other registrations or replace the solver. Read
            // the live world collections at each original phase instead of snapshotting work.
            contact_solver_->prepare(*this, manifolds_, settings_.solver, dt);
            for (const auto& constraint : constraints_)
                if (constraint_active(*constraint))
                    constraint->prepare(*this, dt);
            for (int iteration = 0; iteration < settings_.solver.velocity_iterations; ++iteration)
            {
                contact_solver_->solve_velocity(*this, manifolds_, settings_.solver, dt);
                for (const auto& constraint : constraints_)
                    if (constraint_active(*constraint) && (!settings_.constraint_graph_enabled || !constraint->velocity_rows()))
                        constraint->solve_velocity(*this, dt);
                if (settings_.constraint_graph_enabled)
                    (void)solve_constraint_graph(*this, constraints_);
            }
            return;
        }
        island_work_.resize(islands_.size());
        // A joint can lose its last dynamic endpoint after an in-place body-type edit.
        // It still needs prepare() to discard its previous active rows and reaction cache.
        for (const auto& constraint : constraints_)
            if (constraint_active(*constraint) &&
                find_body(constraint->first_body())->type() != BodyType::dynamic_body &&
                find_body(constraint->second_body())->type() != BodyType::dynamic_body)
                constraint->prepare(*this, dt);
        for (std::size_t index = 0; index < islands_.size(); ++index)
        {
            const auto& island = islands_[index];
            auto& work = island_work_[index];
            if (!island.awake)
            {
                // Deactivate cached joint rows/reactions even though no solve is needed.
                // Otherwise a later smaller dt could turn an old impulse into a false break.
                for (const auto i : island.constraint_indices)
                    constraints_[i]->prepare(*this, dt);
                continue;
            }
            if (island.manifold_indices.empty() && island.constraint_indices.empty())
                continue;
            work.solver = contact_solver_->clone();
            for (const auto i : island.manifold_indices)
                work.contacts.push_back(manifolds_[i]);
            auto& contacts = work.contacts;
            for (const auto i : island.constraint_indices)
                work.constraints.push_back(constraints_[i]);
            work.solver->prepare(*this, contacts, settings_.solver, dt);
            for (const auto& constraint : work.constraints)
                if (constraint_active(*constraint))
                    constraint->prepare(*this, dt);
            for (int iteration = 0; iteration < settings_.solver.velocity_iterations; ++iteration)
            {
                work.solver->solve_velocity(*this, contacts, settings_.solver, dt);
                for (const auto& constraint : work.constraints)
                    if (constraint_active(*constraint) && (!settings_.constraint_graph_enabled || !constraint->velocity_rows()))
                        constraint->solve_velocity(*this, dt);
                if (settings_.constraint_graph_enabled && !work.constraints.empty())
                    (void)solve_constraint_graph(*this, work.constraints);
            }
            for (std::size_t i = 0; i < work.contacts.size(); ++i)
                manifolds_[island.manifold_indices[i]] = work.contacts[i];
        }
    }

    void World::solve_island_positions(Real dt)
    {
        ScopedProfileTimer timer(profiling_enabled_, profile_.position_solve_s);
        if (!islands_.empty() && islands_.front().serial_fallback)
        {
            for (int iteration = 0; iteration < settings_.solver.position_iterations; ++iteration)
            {
                contact_solver_->solve_position(*this, manifolds_, settings_.solver, dt);
                for (const auto& constraint : constraints_)
                    if (constraint_active(*constraint))
                        constraint->solve_position(*this, dt);
            }
            return;
        }
        for (std::size_t index = 0; index < islands_.size(); ++index)
        {
            const auto& island = islands_[index];
            auto& work = island_work_[index];
            if (!work.solver)
                continue;
            for (std::size_t i = 0; i < work.contacts.size(); ++i)
                work.contacts[i] = manifolds_[island.manifold_indices[i]];
            auto& contacts = work.contacts;
            for (int iteration = 0; iteration < settings_.solver.position_iterations; ++iteration)
            {
                work.solver->solve_position(*this, contacts, settings_.solver, dt);
                for (const auto& constraint : work.constraints)
                    if (constraint_active(*constraint))
                        constraint->solve_position(*this, dt);
            }
            for (std::size_t i = 0; i < work.contacts.size(); ++i)
                manifolds_[island.manifold_indices[i]] = work.contacts[i];
        }
    }

    void World::finish_island_solving()
    {
        statistics_.simulation_island_count = islands_.size();
        statistics_.active_island_count = statistics_.sleeping_island_count = statistics_.largest_island_body_count = 0;
        for (auto& island : islands_)
        {
            island.awake = island.serial_fallback || std::any_of(island.bodies.begin(), island.bodies.end(), [&](BodyId id)
                                                         {
                                                             const auto* body = find_body(id);
                                                             return body && body->is_awake();
                                                         });
            if (island.awake)
                ++statistics_.active_island_count;
            else
                ++statistics_.sleeping_island_count;
            statistics_.largest_island_body_count = std::max(statistics_.largest_island_body_count, island.bodies.size());
        }
        island_work_.clear();
    }
}
