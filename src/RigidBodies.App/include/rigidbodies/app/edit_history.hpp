#pragma once
#include <rigidbodies/app/shape_editor.hpp>
#include <rigidbodies/app/run_recorder.hpp>
#include <rigidbodies/app/property_records.hpp>
#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/physics/time_stepper.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <optional>

namespace rigidbodies::app
{
    struct SetupSnapshot
    {
        physics::WorldSnapshot world;
        double gravity_direction_degrees { -90.0 };
    };

    // Frozen world evidence and edit-related session state. Display preferences stay independent.
    struct SessionEditState
    {
        physics::WorldSnapshot world {};
        SetupSnapshot setup {};
        SetupSnapshot original {};
        LabSettings lab {}, default_lab {};
        physics::TimeStepper stepper {};
        RunRecorder runs {};
        ShapeEditor shape_editor {};
        std::vector<physics::BodyId> selected_bodies {};
        physics::BodyId selection {};
        physics::BodyId edited_shape_body {};
        std::size_t authored_part_index {};
        std::size_t edited_part_index {};
        std::size_t new_shape_material_index {};
        physics::Material shape_material {};
        std::string shape_edit_target {};
        ui::PauseState pause_reason {};
        bool next_impact_armed {};
        std::optional<ui::Prediction> pending_prediction;
        std::optional<ui::SelectedConnection> selected_connection;
        std::string scenario_id {};
        std::shared_ptr<const physics::ScenarioDocument> scenario_document;
        double gravity_direction_degrees {};
        bool state_setup_toast_shown {};
    };
    struct SessionEdit
    {
        SessionEditState before, after;
        std::string label;
        ui::EditCategory category { ui::EditCategory::state };
        std::vector<PropertyRecord> before_records, after_records;
        ui::UiCommand coalescing_command;
        double committed_wall_time_s { 0.0 };
    };
    inline constexpr std::size_t maximum_session_edits = 64;
}
