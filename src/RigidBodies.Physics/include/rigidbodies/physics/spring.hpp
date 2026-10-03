#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <variant>

namespace rigidbodies::physics
{
    struct SpringId
    {
        std::uint32_t index {};
        std::uint32_t generation {};
        [[nodiscard]] bool is_valid() const
        {
            return generation != 0;
        }
        friend bool operator==(SpringId a, SpringId b)
        {
            return a.index == b.index && a.generation == b.generation;
        }
        friend bool operator!=(SpringId a, SpringId b)
        {
            return !(a == b);
        }
    };

    struct LinearSpringDefinition
    {
        BodyId first;
        BodyId second;
        math::Vec2 local_anchor_first_m {};
        math::Vec2 local_anchor_second_m {};
        Real rest_length_m { 1.0 };
        Real stiffness_n_m { 10.0 };
        Real damping_n_s_m { 0.0 };
        bool enabled { true };
    };

    struct AngularSpringDefinition
    {
        BodyId first;
        BodyId second;
        // Uses unwrapped (second angle - first angle - rest angle). A torsional spring can
        // store multiple turns; crossing +/-pi does not reset its stored energy.
        Real rest_angle_rad { 0.0 };
        Real stiffness_n_m_rad { 1.0 };
        Real damping_n_m_s_rad { 0.0 };
        bool enabled { true };
    };

    using SpringDefinition = std::variant<LinearSpringDefinition, AngularSpringDefinition>;
    enum class SpringKind
    {
        linear,
        angular
    };

    // An instantaneous reading of the present endpoint state, not the previous step's retained
    // applied load. Forces/torques include damping; stored potential contains elasticity only.
    struct SpringReport
    {
        SpringKind kind { SpringKind::linear };
        BodyId first;
        BodyId second;
        bool enabled { true };
        math::Vec2 first_anchor_m {};
        math::Vec2 second_anchor_m {};
        math::Vec2 force_on_first_n {};
        Real torque_on_first_n_m {};
        Real torque_on_second_n_m {};
        Real extension_m {};
        Real axial_speed_m_s {};
        Real angular_displacement_rad {};
        Real relative_angular_speed_rad_s {};
        Real potential_energy_j {};
        Real dissipated_power_w {};
    };

    // Finite nonnegative stiffness, damping and rest length are required. Endpoints must be
    // distinct valid IDs; World additionally verifies that both generations are still alive.
    void validate_spring(const SpringDefinition& definition);
    [[nodiscard]] SpringReport evaluate_spring(const SpringDefinition& definition, const RigidBody& first, const RigidBody& second);
} // namespace rigidbodies::physics
