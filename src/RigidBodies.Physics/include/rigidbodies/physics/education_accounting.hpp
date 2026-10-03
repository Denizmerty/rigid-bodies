#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <optional>
#include <string>

namespace rigidbodies::physics
{
    class World;

    struct EnergyBreakdown
    {
        Real translational_kinetic_j {};
        Real rotational_kinetic_j {};
        Real gravitational_potential_j {};
        Real spring_potential_j {};
        [[nodiscard]] Real kinetic_j() const
        {
            return translational_kinetic_j + rotational_kinetic_j;
        }
        [[nodiscard]] Real mechanical_j() const
        {
            return kinetic_j() + gravitational_potential_j + spring_potential_j;
        }
    };

    // Kinetic/gravity energy belongs to dynamic bodies. A spring's stored energy belongs to the
    // moving bodies it joins: all of it to a dynamic body tied to a fixed or driven one, which
    // stores nothing, and half to each end when both ends move (or neither does). Summing every
    // body therefore counts each spring once.
    //
    // Potential energy is read at the middle of the last step rather than at its end, which
    // moving bodies reach with the step's final velocity: semi-implicit Euler conserves that
    // reading exactly under uniform gravity and linear springs, so free flight keeps a level
    // mechanical energy instead of losing m (g dt)^2 / 2 each step. Resting bodies read m g h
    // and k x^2 / 2 exactly. World::energy_measurement_lag_s() gives the offset.
    [[nodiscard]] std::optional<EnergyBreakdown> measure_body_energy(const World& world, BodyId body);
    [[nodiscard]] EnergyBreakdown measure_world_energy(const World& world);
    // Linear momentum of the dynamic bodies as they move now, including before the first step
    // and after edits made while paused, which WorldStatistics only catches up with on a step.
    [[nodiscard]] math::Vec2 measure_world_momentum_kg_m_s(const World& world);
    // Acceleration contributed by enabled built-in UniformGravity registrations, including
    // additive body attachments and the body's gravity scale. Other force laws are not inferred.
    [[nodiscard]] math::Vec2 effective_uniform_gravity_m_s2(const World& world, BodyId body);

    // Work done on the dynamic bodies since the timeline started by everything that is not
    // stored as potential energy, in joules; positive work adds mechanical energy. Contact
    // impulses are kept in CollisionEnergyAccounting. Together they close the budget:
    //     mechanical now = mechanical at the start + contact work + every field below,
    // apart from unattributed_j, which collects what no interaction explains (bodies falling
    // asleep, speed limits, the time-step error of staged integrators and of springs that turn).
    struct EnergyLedger
    {
        // Aerodynamic drag, lift and spin resistance, and per-body velocity damping.
        Real air_work_j {};
        // The viscous part of linear and angular springs.
        Real damper_work_j {};
        // Joint impulses, motors and the joints' position correction.
        Real joint_work_j {};
        // The part of joint_work_j done by joint motors; the rest is done by the joints' links,
        // travel stops and position correction.
        Real motor_work_j {};
        // Other applied loads: attractors, the pointer and custom force generators.
        Real applied_work_j {};
        // Pushing overlapping bodies apart and limiting fast bodies to their first contact.
        Real contact_correction_work_j {};
        // Changes made between steps: edits, throws, moved objects and changed settings.
        Real edit_work_j {};
        Real unattributed_j {};
        EnergyLedger& operator+=(const EnergyLedger& other);
    };

    // Energy lost to each kind of interaction, as an experiment presents it: positive values
    // removed energy from the moving bodies and positive additions put energy in. Contact
    // position correction counts with impacts.
    struct EnergyBudget
    {
        // Joints lose energy in their links, at their travel stops and in position correction;
        // what their motors put in is an addition.
        Real lost_in_impacts_j {}, lost_to_friction_j {}, lost_to_air_j {}, lost_in_dampers_j {}, lost_in_joints_j {};
        // Work by motors and driven objects; by attractors, the pointer and other applied loads;
        // and changes made between steps, including edits the next step has yet to book.
        Real added_by_drives_j {}, added_by_forces_j {}, added_by_changes_j {};
        // Mechanical energy when the timeline started. It equals mechanical energy now plus every
        // loss less every addition, apart from what EnergyLedger::unattributed_j leaves unexplained.
        Real start_energy_j {};
        // Which interactions the scene contains, so a budget can list them before they act.
        bool has_air { false }, has_dampers { false }, has_joints { false }, has_drives { false }, has_applied_forces { false };
        [[nodiscard]] Real lost_j() const
        {
            return lost_in_impacts_j + lost_to_friction_j + lost_to_air_j + lost_in_dampers_j + lost_in_joints_j;
        }
        [[nodiscard]] Real added_j() const
        {
            return added_by_drives_j + added_by_forces_j + added_by_changes_j;
        }
    };
    [[nodiscard]] EnergyBudget measure_energy_budget(const World& world);

    // True while a contact has stopped a body short of a surface and the rebound it is owed is
    // applied on the next step. Energy read in between counts the impact's loss before its return.
    [[nodiscard]] bool impact_in_progress(const World& world);

    struct CollisionEnergyAccounting
    {
        Real normal_energy_change_j {};
        Real friction_energy_change_j {};
        Real normal_external_work_j {};
        Real friction_external_work_j {};
        // Signed balances, not sums of positive iteration losses: warm starts and speculative
        // velocity caps can return energy later. Normal loss includes restitution suppression.
        // A solver on its own books the kinetic change across each impulse; World::step instead
        // measures each kind's impulses against every body's mean velocity over the step.
        [[nodiscard]] Real restitution_loss_j() const
        {
            return normal_external_work_j - normal_energy_change_j;
        }
        [[nodiscard]] Real friction_loss_j() const
        {
            return friction_external_work_j - friction_energy_change_j;
        }
        [[nodiscard]] Real external_work_j() const
        {
            return normal_external_work_j + friction_external_work_j;
        }
        CollisionEnergyAccounting& operator+=(const CollisionEnergyAccounting& other);
    };

    struct CollisionBodyState
    {
        BodyId id;
        std::string name;
        BodyType type { BodyType::static_body };
        Real mass_kg {}, inertia_kg_m2 {};
        math::Vec2 center_m {}, velocity_m_s {}, momentum_kg_m_s {};
        Real angular_velocity_rad_s {}, angular_momentum_kg_m2_s {}, kinetic_energy_j {};
    };

    struct CollisionImpactReport
    {
        CollisionBodyState first_before, second_before, first_after, second_after;
        math::Vec2 normal {}, point_m {}, impulse_on_second_n_s {};
        Real angular_impulse_first_n_m_s {}, angular_impulse_second_n_m_s {};
        Real normal_impulse_n_s {}, tangent_impulse_n_s {};
        CollisionEnergyAccounting energy;
        Real elapsed_time_s {};
        std::uint64_t step_index {};
        bool involved_speculation { false };
        bool spans_multiple_steps { false };
        bool coupled_constraints { false };
        bool coupled_contacts { false };
    };

    inline constexpr std::size_t maximum_impact_reports_per_step = 64;
}
