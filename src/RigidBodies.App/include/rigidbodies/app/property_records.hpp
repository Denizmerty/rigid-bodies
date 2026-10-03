#pragma once

#include <rigidbodies/physics/material.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/scenario_document.hpp>

#include <string>
#include <variant>
#include <vector>

namespace rigidbodies::app
{
    struct LabSettings
    {
        std::string integrator_id { "semi_implicit_euler" };
        double fixed_step_s { 1.0 / 120.0 };
        int velocity_iterations { 8 };
        int position_iterations { 3 };
        double linear_slop_m { 0.005 };
        double position_correction_fraction { 0.2 };
        double restitution_threshold_m_s { 1.0 };
        double maximum_position_correction_m { 0.2 };
        bool warm_starting { true };
        bool constraint_graph { true };
    };

    struct BodyParameterRecord
    {
        physics::BodyId body;
        double mass_kg {}, gravity_scale {};
        bool mass_from_density {};
        std::vector<physics::Material> part_materials;
        std::vector<physics::AuthoredPartPtr> part_geometry;
    };
    struct JointParameterRecord
    {
        std::string joint_key;
        bool motor_enabled {}, limits_enabled {};
        double motor_speed {};
    };
    struct SpringParameterRecord
    {
        std::string spring_key;
        double stiffness {}, damping {};
    };
    struct EnvironmentRecord
    {
        math::Vec2 gravity_m_s2 {}, wind_m_s {};
        bool gravity_enabled {}, drag_enabled {}, angular_drag_enabled {}, magnus_enabled {};
        double gravity_direction_degrees {}, air_density {}, viscosity_pa_s {}, zero_height_m {};
        physics::MaterialMixing bounce_rule { physics::MaterialMixing::maximum };
        bool catch_fast_objects {};
    };
    struct LabSettingsRecord
    {
        LabSettings settings;
    };

    using PropertyRecord = std::variant<BodyParameterRecord, JointParameterRecord, SpringParameterRecord, EnvironmentRecord, LabSettingsRecord>;
}
