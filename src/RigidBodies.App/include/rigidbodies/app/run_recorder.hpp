#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/ui/run_types.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>

namespace rigidbodies::app
{
    class RunRecorder
    {
    public:
        RunRecorder();

        void set_experiment(std::string_view experiment_id);
        void begin_run(const physics::World& world, std::vector<ui::SetupChange> changes_from_original,
            std::vector<ui::SetupChange> changes_from_previous, std::optional<ui::Prediction> prediction = {});
        void record_sample(const physics::World& world, double time_s);
        void add_marker(double time_s, std::string label);
        void note_impact(double time_s);
        void mark_changed_during_run();
        void cut_at(double time_s);
        void clear_graph(double time_s);
        void close_run(bool keep, bool make_previous);

        [[nodiscard]] const ui::RunRecord* current() const;
        [[nodiscard]] const ui::RunRecord* previous() const;
        [[nodiscard]] math::Span<const ui::RunRecord> kept(std::string_view experiment_id) const;
        [[nodiscard]] math::Span<const ui::PinnedValue> pinned(std::string_view experiment_id) const;
        [[nodiscard]] std::size_t memory_bytes() const;
        [[nodiscard]] std::size_t starred_count() const;

        bool star(int run_number, bool starred);
        void clear_unstarred();
        bool pin(ui::PinnedValue value);
        bool unpin(std::string_view key);

        [[nodiscard]] const std::string& experiment_id() const;

    private:
        struct ExperimentRuns
        {
            std::vector<ui::RunRecord> kept;
            std::vector<ui::PinnedValue> pinned;
            int next_number { 1 };
            int previous_number {};
            std::uint64_t next_pinned_number { 1 };
        };
        struct State
        {
            std::map<std::string, ExperimentRuns, std::less<>> experiments;
            std::optional<ui::RunRecord> current;
            std::string experiment_id;
            std::uint64_t next_retention_order { 1 };
        };

        void ensure_unique();
        void apply_retention();
        void update_pinned_results(ExperimentRuns& experiment, ui::RunRecord& run) const;
        std::shared_ptr<State> state_;
    };
}
