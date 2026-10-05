#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/ui/control_spec.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rigidbodies::ui
{
    struct SetupChange
    {
        std::string key, control_key;
        physics::BodyId body;
        std::string label, original_text, current_text;
        EditCategory category { EditCategory::parameter };
    };

    struct InterventionMarker
    {
        double time_s {};
        std::string label;
    };

    struct RunSeries
    {
        std::vector<float> time_s;
        std::vector<float> scene;
        // The energy budget's other rows for the whole scene, six per sample: lost to air, in
        // dampers and in joints, then added by drives, by applied forces and by changes.
        std::vector<float> ledger;
        std::vector<float> objects;
        std::vector<physics::BodyId> object_ids;
        // Special relativity, three per sample: probe clock τ (s), lab − probe t − τ (s), γ − 1. Empty for Newtonian runs.
        std::vector<float> relativity;
    };

    struct Prediction
    {
        std::string option, note;
    };

    enum class RunAggregator : std::uint8_t
    {
        at_end,
        maximum,
        minimum,
        at_first_impact,
        at_time
    };

    struct PinnedValue
    {
        std::string key, quantity;
        physics::BodyId body;
        RunAggregator aggregator { RunAggregator::at_end };
        double time_s {};
    };

    struct RunRecord
    {
        int number {};
        std::vector<SetupChange> changes_from_original, changes_from_previous;
        std::optional<Prediction> prediction;
        double duration_s {}, plot_start_s {};
        std::size_t impact_count {};
        std::optional<double> first_impact_s;
        bool changed_during_run {}, starred {};
        RunSeries series;
        std::vector<InterventionMarker> markers;
        std::vector<std::optional<double>> pinned_results;
        std::uint64_t retention_order {};
    };

    struct UndoHistoryItem
    {
        std::string label;
        EditCategory category { EditCategory::parameter };
        double time_s {};
    };

    struct ObjectItem
    {
        physics::BodyId id;
        std::string document_id, display_name, kind, role, caption;
        bool moving {}, selected {};
    };

    struct SelectedConnection
    {
        std::string key, kind;
    };

    enum class PauseReason : std::uint8_t
    {
        none,
        drawing,
        new_object,
        impact,
        after_undo,
        error,
        in_background,
        finished
    };

    enum class HoldReason : std::uint8_t
    {
        none,
        dragging,
        adjusting
    };

    struct PauseState
    {
        PauseReason reason { PauseReason::none };
        std::size_t impact_number {};
    };

    struct DraftModel
    {
        bool active {}, closed {}, valid {}, editing_existing {};
        std::string target_name, diagnostic, material_name;
        std::size_t node_count {}, drawing_vertex_count {}, collision_vertex_count {}, piece_count {};
    };

    struct ImpactItem
    {
        std::size_t index {};
        double time_s {};
        physics::BodyId first, second;
        std::string first_name, second_name;
        double impulse_n_s {};
        math::Vec2 contact_point_m {};
        math::Vec2 first_velocity_before_m_s {}, first_velocity_after_m_s {};
        math::Vec2 second_velocity_before_m_s {}, second_velocity_after_m_s {};
        math::Vec2 impulse_on_second_n_s {};
        double restitution_loss_j {}, friction_loss_j {};
        bool selected {};
        math::Vec2 total_momentum_before_kg_m_s {}, total_momentum_after_kg_m_s {};
        physics::BodyType first_type { physics::BodyType::dynamic_body }, second_type { physics::BodyType::dynamic_body };
        math::Vec2 first_momentum_before_kg_m_s {}, first_momentum_after_kg_m_s {};
        math::Vec2 second_momentum_before_kg_m_s {}, second_momentum_after_kg_m_s {};
        // Kinetic energy of both objects, so the loss can be read against what they had.
        double kinetic_before_j {}, kinetic_after_j {};
        // What else acted on the objects while they touched: gravity, and other contacts or
        // joints. The card names it beside the contact's own impulse and loss.
        bool weighted {}, coupled {};

        // Both bodies move freely, so the pair's momentum is conserved through the impact.
        [[nodiscard]] bool between_free_bodies() const
        {
            return first_type == physics::BodyType::dynamic_body && second_type == physics::BodyType::dynamic_body;
        }
    };

    struct SetupFileItem
    {
        std::string path, title, based_on, date;
        std::uint64_t serial {};
    };
}
