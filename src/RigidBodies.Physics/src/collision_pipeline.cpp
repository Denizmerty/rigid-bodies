#include <rigidbodies/physics/constraint.hpp>
#include <rigidbodies/physics/contact.hpp>
#include <rigidbodies/physics/contact_solver.hpp>
#include <rigidbodies/physics/narrow_phase.hpp>

namespace rigidbodies::physics
{

    std::uint64_t ContactFeatureId::key() const
    {
        return static_cast<std::uint64_t>(incoming_edge) | (static_cast<std::uint64_t>(outgoing_edge) << 16) | (static_cast<std::uint64_t>(incoming_vertex) << 32) |
            (static_cast<std::uint64_t>(outgoing_vertex) << 48);
    }

    bool Constraint::solve_position(World&, Real)
    {
        // A velocity-only constraint reports itself satisfied so that the position pass can treat
        // every constraint uniformly.
        return true;
    }

    std::string_view NullNarrowPhase::name() const
    {
        return "null_narrow_phase";
    }

    bool NullNarrowPhase::collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const
    {
        manifold = {};
        manifold.first = query.first;
        manifold.second = query.second;
        manifold.first_collider = query.first_collider;
        manifold.second_collider = query.second_collider;
        manifold.material = query.material;
        manifold.is_sensor = query.is_sensor;
        return false;
    }

    std::string_view NullContactSolver::name() const
    {
        return "null_contact_solver";
    }

    void NullContactSolver::prepare(World&, std::vector<ContactManifold>&, const SolverSettings&, Real)
    {
    }

    void NullContactSolver::solve_velocity(World&, std::vector<ContactManifold>&, const SolverSettings&, Real)
    {
    }

    void NullContactSolver::solve_position(World&, std::vector<ContactManifold>&, const SolverSettings&, Real)
    {
    }

} // namespace rigidbodies::physics
