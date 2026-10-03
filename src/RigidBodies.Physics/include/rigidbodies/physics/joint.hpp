#pragma once

#include <rigidbodies/physics/constraint.hpp>

#include <limits>
#include <variant>
#include <vector>

namespace rigidbodies::physics
{
    struct JointEndpoints
    {
        BodyId first;
        BodyId second;
        math::Vec2 local_anchor_first_m {};
        math::Vec2 local_anchor_second_m {};
        bool collide_connected { false };
        Real break_force_n { std::numeric_limits<Real>::infinity() };
        // Either anchor's pure couple may break the joint. A translating guide can transmit
        // different couples at its ends because its anchors are separated.
        Real break_torque_n_m { std::numeric_limits<Real>::infinity() };
    };

    struct DistanceJointDefinition : JointEndpoints
    {
        Real length_m { 1.0 };
    };

    struct RevoluteJointDefinition : JointEndpoints
    {
        Real reference_angle_rad {};
        bool limits_enabled { false };
        Real lower_angle_rad {};
        Real upper_angle_rad {};
        bool motor_enabled { false };
        Real motor_speed_rad_s {};
        Real maximum_motor_torque_n_m {};
    };

    struct PrismaticJointDefinition : JointEndpoints
    {
        math::Vec2 local_axis_first { 1.0, 0.0 };
        Real reference_angle_rad {};
        bool limits_enabled { false };
        Real lower_translation_m {};
        Real upper_translation_m {};
        bool motor_enabled { false };
        Real motor_speed_m_s {};
        Real maximum_motor_force_n {};
    };

    struct WeldJointDefinition : JointEndpoints
    {
        Real reference_angle_rad {};
    };

    using JointDefinition = std::variant<DistanceJointDefinition, RevoluteJointDefinition, PrismaticJointDefinition, WeldJointDefinition>;

    enum class JointKind
    {
        distance,
        revolute,
        prismatic,
        weld
    };

    struct JointReport
    {
        JointKind kind { JointKind::distance };
        BodyId first;
        BodyId second;
        math::Vec2 first_anchor_m {};
        math::Vec2 second_anchor_m {};
        math::Vec2 axis {};
        // Reaction on the second body, averaged over the latest solved substep. Torque is the
        // couple at its anchor; the COM torque additionally includes anchor cross force.
        math::Vec2 reaction_force_n {};
        Real reaction_torque_n_m {};
        // First-anchor couple is separately available for guides with separated anchors.
        Real reaction_torque_first_n_m {};
        Real distance_m {};
        Real translation_m {};
        Real angle_rad {};
        Real position_error_m {};
        Real angular_error_rad {};
        Real motor_force_n {};
        Real motor_torque_n_m {};
        bool enabled { true };
        bool broken { false };
        bool limits_enabled { false };
        bool motor_enabled { false };
        Real lower_limit {};
        Real upper_limit {};
    };

    // Definitions are checked before a joint or its caches are changed. Angles are unwrapped,
    // local anchors use body frames, and prismatic axes are normalized internally.
    void validate_joint(const JointDefinition& definition);

    class JointConstraint final : public Constraint
    {
    public:
        explicit JointConstraint(JointDefinition definition);
        [[nodiscard]] std::shared_ptr<Constraint> clone() const override;
        [[nodiscard]] std::string_view name() const override;
        [[nodiscard]] BodyId first_body() const override;
        [[nodiscard]] BodyId second_body() const override;
        [[nodiscard]] bool collide_connected() const override;
        [[nodiscard]] bool is_broken() const override;
        [[nodiscard]] bool has_active_drive() const override;
        [[nodiscard]] bool supports_sleeping_load() const override;
        [[nodiscard]] std::uint64_t revision() const override;
        [[nodiscard]] Real broken_force_n() const override;
        [[nodiscard]] Real broken_torque_n_m() const override;
        [[nodiscard]] const JointDefinition& definition() const;
        void set_definition(JointDefinition definition);
        [[nodiscard]] JointReport report(const World& world) const;
        // The motor's row as the last velocity solve left it, its impulse summed over the step;
        // null while the joint has no running motor. It is read to measure the motor's work.
        [[nodiscard]] const ConstraintRow* solved_motor_row() const;

        void prepare(World& world, Real time_step_s) override;
        void solve_velocity(World& world, Real time_step_s) override;
        bool solve_position(World& world, Real time_step_s) override;
        [[nodiscard]] std::vector<ConstraintRow>* velocity_rows() override;
        void finalize_velocity(World& world, Real time_step_s) override;
        void invalidate_cached_impulses() noexcept override;

    private:
        friend struct ScenarioDocumentAccess;
        JointDefinition definition_;
        std::vector<ConstraintRow> rows_;
        std::vector<unsigned> row_keys_;
        std::vector<bool> motor_rows_;
        math::Vec2 reaction_force_n_ {};
        Real reaction_torque_n_m_ {};
        Real reaction_torque_first_n_m_ {};
        Real motor_force_n_ {};
        Real motor_torque_n_m_ {};
        Real previous_time_step_s_ {};
        std::uint64_t definition_revision_ {};
        std::uint64_t prepared_revision_ {};
        bool broken_ { false };
        bool active_ { false };
    };
}
