#include <rigidbodies/app/run_recorder.hpp>

#include <rigidbodies/physics/education_accounting.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rigidbodies::app
{
    namespace
    {
        constexpr std::size_t scene_channels = 8;
        constexpr std::size_t ledger_channels = 6;
        constexpr std::size_t object_channels = 14;
        constexpr std::size_t maximum_objects = 32;
        constexpr std::size_t maximum_samples = 2400;
        constexpr std::size_t maximum_runs_per_experiment = 10;
        constexpr std::size_t maximum_starred_runs = 8;
        constexpr std::size_t maximum_pinned_values = 12;
        constexpr std::size_t full_run_bytes = 4444800;
        constexpr std::size_t memory_budget_bytes = 48u * 1024u * 1024u;

        std::optional<std::size_t> scene_channel(std::string_view quantity)
        {
            if (quantity == "kinetic_moving")
                return 0;
            if (quantity == "kinetic_spinning")
                return 1;
            if (quantity == "potential_height")
                return 2;
            if (quantity == "potential_springs")
                return 3;
            if (quantity == "lost_impacts")
                return 4;
            if (quantity == "lost_friction")
                return 5;
            if (quantity == "momentum_x")
                return 6;
            if (quantity == "momentum_y")
                return 7;
            return {};
        }

        std::optional<std::size_t> object_channel(std::string_view quantity)
        {
            if (quantity == "kinetic_moving")
                return 0;
            if (quantity == "kinetic_spinning")
                return 1;
            if (quantity == "potential_height")
                return 2;
            if (quantity == "potential_springs")
                return 3;
            if (quantity == "mechanical")
                return 4;
            if (quantity == "momentum_x")
                return 5;
            if (quantity == "momentum_y")
                return 6;
            if (quantity == "speed")
                return 7;
            if (quantity == "velocity_x")
                return 8;
            if (quantity == "velocity_y")
                return 9;
            if (quantity == "height")
                return 10;
            if (quantity == "spin")
                return 11;
            if (quantity == "position_x")
                return 12;
            if (quantity == "rotation")
                return 13;
            return {};
        }

        void erase_first_sample(ui::RunSeries& series)
        {
            if (series.time_s.empty())
                return;
            series.time_s.erase(series.time_s.begin());
            series.scene.erase(series.scene.begin(), series.scene.begin() + static_cast<std::ptrdiff_t>(scene_channels));
            series.ledger.erase(series.ledger.begin(), series.ledger.begin() + static_cast<std::ptrdiff_t>(std::min(ledger_channels, series.ledger.size())));
            const auto count = object_channels * series.object_ids.size();
            series.objects.erase(series.objects.begin(), series.objects.begin() + static_cast<std::ptrdiff_t>(count));
        }

        std::size_t samples_bytes(const ui::RunRecord& run)
        {
            return (run.series.time_s.capacity() + run.series.scene.capacity() + run.series.ledger.capacity() + run.series.objects.capacity()) * sizeof(float);
        }
    }

    RunRecorder::RunRecorder() : state_(std::make_shared<State>())
    {
    }

    void RunRecorder::ensure_unique()
    {
        if (state_.use_count() != 1)
            state_ = std::make_shared<State>(*state_);
    }

    void RunRecorder::set_experiment(std::string_view experiment_id)
    {
        ensure_unique();
        state_->experiment_id = std::string(experiment_id);
        (void)state_->experiments[state_->experiment_id];
        state_->current.reset();
    }

    void RunRecorder::begin_run(const physics::World& world, std::vector<ui::SetupChange> changes_from_original,
        std::vector<ui::SetupChange> changes_from_previous, std::optional<ui::Prediction> prediction)
    {
        if (state_->current || state_->experiment_id.empty())
            return;
        ensure_unique();
        auto& experiment = state_->experiments[state_->experiment_id];
        ui::RunRecord run;
        run.number = experiment.next_number++;
        run.changes_from_original = std::move(changes_from_original);
        run.changes_from_previous = std::move(changes_from_previous);
        run.prediction = std::move(prediction);
        for (const auto id : world.body_ids())
            if (const auto* body = world.find_body(id); body && body->type() == physics::BodyType::dynamic_body && run.series.object_ids.size() < maximum_objects)
                run.series.object_ids.push_back(id);
        run.series.time_s.reserve(maximum_samples);
        run.series.scene.reserve(maximum_samples * scene_channels);
        run.series.ledger.reserve(maximum_samples * ledger_channels);
        run.series.objects.reserve(maximum_samples * object_channels * run.series.object_ids.size());
        state_->current = std::move(run);
    }

    void RunRecorder::record_sample(const physics::World& world, double time_s)
    {
        if (!state_->current || !std::isfinite(time_s))
            return;
        ensure_unique();
        auto& run = *state_->current;
        if (!run.series.time_s.empty() && time_s + 1.0e-10 < run.series.time_s.back())
            cut_at(time_s);
        // Keep the cadence decision in double precision. The public series deliberately stores
        // floats, but consulting its rounded last timestamp here would intermittently drop an
        // otherwise exact 40 Hz sample late in a run.
        if (!run.series.time_s.empty() && time_s - run.duration_s < 0.025 - 1.0e-10)
            return;
        // A sample taken between a contact stopping a body and its rebound would show the impact's
        // loss before the energy it returns, a dip no student can explain. Wait for the next step.
        if (!run.series.time_s.empty() && time_s - run.duration_s < 0.05 - 1.0e-10 && physics::impact_in_progress(world))
            return;
        // Stamp the regular 40 Hz cadence, rather than the first solver substep after it. This
        // keeps graphs and aggregates deterministic when the fixed step is not a divisor of 25 ms.
        const auto sample_time_s = run.series.time_s.empty() ? time_s : run.duration_s + 0.025;
        while (!run.series.time_s.empty() && (run.series.time_s.size() >= maximum_samples || sample_time_s - run.series.time_s.front() > 60.0 + 1.0e-10))
            erase_first_sample(run.series);

        const auto scene = physics::measure_world_energy(world);
        run.series.time_s.push_back(static_cast<float>(sample_time_s));
        run.series.scene.push_back(static_cast<float>(scene.translational_kinetic_j));
        run.series.scene.push_back(static_cast<float>(scene.rotational_kinetic_j));
        run.series.scene.push_back(static_cast<float>(scene.gravitational_potential_j));
        run.series.scene.push_back(static_cast<float>(scene.spring_potential_j));
        const auto budget = physics::measure_energy_budget(world);
        run.series.scene.push_back(static_cast<float>(budget.lost_in_impacts_j));
        run.series.scene.push_back(static_cast<float>(budget.lost_to_friction_j));
        const auto momentum_kg_m_s = physics::measure_world_momentum_kg_m_s(world);
        run.series.scene.push_back(static_cast<float>(momentum_kg_m_s.x));
        run.series.scene.push_back(static_cast<float>(momentum_kg_m_s.y));
        for (const auto value : { budget.lost_to_air_j, budget.lost_in_dampers_j, budget.lost_in_joints_j, budget.added_by_drives_j, budget.added_by_forces_j, budget.added_by_changes_j })
            run.series.ledger.push_back(static_cast<float>(value));
        for (const auto id : run.series.object_ids)
        {
            const auto* body = world.find_body(id);
            const auto energy = body ? physics::measure_body_energy(world, id) : std::optional<physics::EnergyBreakdown> {};
            const auto nan = std::numeric_limits<float>::quiet_NaN();
            if (!body || !energy)
            {
                for (std::size_t channel = 0; channel < object_channels; ++channel)
                    run.series.objects.push_back(nan);
                continue;
            }
            const auto momentum = body->linear_momentum_kg_m_s();
            const auto velocity = body->linear_velocity_m_s();
            run.series.objects.push_back(static_cast<float>(energy->translational_kinetic_j));
            run.series.objects.push_back(static_cast<float>(energy->rotational_kinetic_j));
            run.series.objects.push_back(static_cast<float>(energy->gravitational_potential_j));
            run.series.objects.push_back(static_cast<float>(energy->spring_potential_j));
            run.series.objects.push_back(static_cast<float>(energy->mechanical_j()));
            run.series.objects.push_back(static_cast<float>(momentum.x));
            run.series.objects.push_back(static_cast<float>(momentum.y));
            run.series.objects.push_back(static_cast<float>(math::length(velocity)));
            run.series.objects.push_back(static_cast<float>(velocity.x));
            run.series.objects.push_back(static_cast<float>(velocity.y));
            run.series.objects.push_back(static_cast<float>(body->world_center_of_mass_m().y));
            run.series.objects.push_back(static_cast<float>(body->angular_velocity_rad_s()));
            run.series.objects.push_back(static_cast<float>(body->world_center_of_mass_m().x));
            run.series.objects.push_back(static_cast<float>(body->orientation_rad()));
        }
        run.duration_s = sample_time_s;
    }

    void RunRecorder::add_marker(double time_s, std::string label)
    {
        if (!state_->current)
            return;
        ensure_unique();
        auto& markers = state_->current->markers;
        if (markers.size() == 32)
            markers.erase(markers.begin());
        markers.push_back({ time_s, std::move(label) });
    }

    void RunRecorder::note_impact(double time_s)
    {
        if (!state_->current)
            return;
        ensure_unique();
        if (!state_->current->first_impact_s)
            state_->current->first_impact_s = std::max(0.0, time_s);
        ++state_->current->impact_count;
    }

    void RunRecorder::mark_changed_during_run()
    {
        if (!state_->current)
            return;
        ensure_unique();
        state_->current->changed_during_run = true;
    }

    void RunRecorder::cut_at(double time_s)
    {
        if (!state_->current)
            return;
        ensure_unique();
        auto& run = *state_->current;
        const auto first = std::upper_bound(run.series.time_s.begin(), run.series.time_s.end(), static_cast<float>(time_s));
        const auto kept_count = static_cast<std::size_t>(std::distance(run.series.time_s.begin(), first));
        run.series.time_s.resize(kept_count);
        run.series.scene.resize(kept_count * scene_channels);
        run.series.ledger.resize(kept_count * ledger_channels);
        run.series.objects.resize(kept_count * object_channels * run.series.object_ids.size());
        run.markers.erase(std::remove_if(run.markers.begin(), run.markers.end(), [&](const auto& marker)
                              {
                                  return marker.time_s > time_s;
                              }),
            run.markers.end());
        run.duration_s = time_s;
        run.changed_during_run = true;
    }

    void RunRecorder::clear_graph(double time_s)
    {
        if (!state_->current)
            return;
        ensure_unique();
        state_->current->plot_start_s = std::max(0.0, time_s);
    }

    void RunRecorder::close_run(bool keep, bool make_previous)
    {
        if (!state_->current)
            return;
        ensure_unique();
        auto run = std::move(*state_->current);
        state_->current.reset();
        if (!keep || run.duration_s < 0.5)
            return;
        auto& experiment = state_->experiments[state_->experiment_id];
        run.retention_order = state_->next_retention_order++;
        update_pinned_results(experiment, run);
        const auto number = run.number;
        experiment.kept.push_back(std::move(run));
        if (make_previous)
            experiment.previous_number = number;
        apply_retention();
    }

    const ui::RunRecord* RunRecorder::current() const
    {
        return state_->current ? &*state_->current : nullptr;
    }

    const ui::RunRecord* RunRecorder::previous() const
    {
        const auto found = state_->experiments.find(state_->experiment_id);
        if (found == state_->experiments.end() || found->second.previous_number == 0)
            return nullptr;
        for (const auto& run : found->second.kept)
            if (run.number == found->second.previous_number)
                return &run;
        return nullptr;
    }

    math::Span<const ui::RunRecord> RunRecorder::kept(std::string_view experiment_id) const
    {
        const auto found = state_->experiments.find(experiment_id);
        return found == state_->experiments.end() ? math::Span<const ui::RunRecord> {} : math::Span<const ui::RunRecord>(found->second.kept);
    }

    math::Span<const ui::PinnedValue> RunRecorder::pinned(std::string_view experiment_id) const
    {
        const auto found = state_->experiments.find(experiment_id);
        return found == state_->experiments.end() ? math::Span<const ui::PinnedValue> {} : math::Span<const ui::PinnedValue>(found->second.pinned);
    }

    std::size_t RunRecorder::memory_bytes() const
    {
        std::size_t result = state_->current ? samples_bytes(*state_->current) : 0;
        for (const auto& [id, experiment] : state_->experiments)
        {
            (void)id;
            for (const auto& run : experiment.kept)
                result += samples_bytes(run);
        }
        return result;
    }

    std::size_t RunRecorder::starred_count() const
    {
        std::size_t result = 0;
        for (const auto& [id, experiment] : state_->experiments)
        {
            (void)id;
            result += static_cast<std::size_t>(std::count_if(experiment.kept.begin(), experiment.kept.end(), [](const auto& run)
                {
                    return run.starred;
                }));
        }
        return result;
    }

    bool RunRecorder::star(int run_number, bool starred)
    {
        ensure_unique();
        auto& runs = state_->experiments[state_->experiment_id].kept;
        const auto found = std::find_if(runs.begin(), runs.end(), [&](const auto& run)
            {
                return run.number == run_number;
            });
        if (found == runs.end() || (starred && !found->starred && starred_count() >= maximum_starred_runs))
            return false;
        found->starred = starred;
        return true;
    }

    void RunRecorder::clear_unstarred()
    {
        ensure_unique();
        auto& runs = state_->experiments[state_->experiment_id].kept;
        runs.erase(std::remove_if(runs.begin(), runs.end(), [](const auto& run)
                       {
                           return !run.starred;
                       }),
            runs.end());
    }

    bool RunRecorder::pin(ui::PinnedValue value)
    {
        if ((value.body.is_valid() ? !object_channel(value.quantity) : value.quantity != "mechanical" && !scene_channel(value.quantity)) ||
            (value.aggregator == ui::RunAggregator::at_time && (!std::isfinite(value.time_s) || value.time_s < 0.0 || value.time_s > 60.0)))
            return false;
        ensure_unique();
        auto& experiment = state_->experiments[state_->experiment_id];
        if (!value.key.empty() && std::any_of(experiment.pinned.begin(), experiment.pinned.end(), [&](const auto& existing)
                                      {
                                          return existing.key == value.key;
                                      }))
            return false;
        if (experiment.pinned.size() >= maximum_pinned_values)
            return false;
        if (value.key.empty())
            do
                value.key = value.quantity + ":" + std::to_string(experiment.next_pinned_number++);
            while (std::any_of(experiment.pinned.begin(), experiment.pinned.end(), [&](const auto& existing)
                {
                    return existing.key == value.key;
                }));
        experiment.pinned.push_back(std::move(value));
        for (auto& run : experiment.kept)
            update_pinned_results(experiment, run);
        return true;
    }

    bool RunRecorder::unpin(std::string_view key)
    {
        ensure_unique();
        auto& experiment = state_->experiments[state_->experiment_id];
        const auto before = experiment.pinned.size();
        experiment.pinned.erase(std::remove_if(experiment.pinned.begin(), experiment.pinned.end(), [&](const auto& value)
                                    {
                                        return value.key == key;
                                    }),
            experiment.pinned.end());
        if (experiment.pinned.size() == before)
            return false;
        for (auto& run : experiment.kept)
            update_pinned_results(experiment, run);
        return true;
    }

    void RunRecorder::update_pinned_results(ExperimentRuns& experiment, ui::RunRecord& run) const
    {
        run.pinned_results.clear();
        for (const auto& pinned : experiment.pinned)
        {
            const auto samples = run.series.time_s.size();
            const float* values = nullptr;
            std::size_t stride = 0;
            const bool scene_mechanical = !pinned.body.is_valid() && pinned.quantity == "mechanical";
            if (!pinned.body.is_valid())
            {
                const auto channel = scene_channel(pinned.quantity);
                if (channel)
                {
                    values = run.series.scene.data() + *channel;
                    stride = scene_channels;
                }
            }
            else
            {
                const auto object = std::find(run.series.object_ids.begin(), run.series.object_ids.end(), pinned.body);
                const auto channel = object_channel(pinned.quantity);
                if (object != run.series.object_ids.end() && channel)
                {
                    const auto object_index = static_cast<std::size_t>(std::distance(run.series.object_ids.begin(), object));
                    values = run.series.objects.data() + object_index * object_channels + *channel;
                    stride = run.series.object_ids.size() * object_channels;
                }
            }
            if ((!values && !scene_mechanical) || samples == 0)
            {
                run.pinned_results.push_back({});
                continue;
            }
            const auto sample_at = [&](std::size_t index) -> double
            {
                if (!scene_mechanical)
                    return static_cast<double>(values[index * stride]);
                const auto base = index * scene_channels;
                return static_cast<double>(run.series.scene[base]) + run.series.scene[base + 1] + run.series.scene[base + 2] + run.series.scene[base + 3];
            };
            std::optional<double> value;
            if (pinned.aggregator == ui::RunAggregator::at_end)
                value = sample_at(samples - 1);
            else if (pinned.aggregator == ui::RunAggregator::at_time || pinned.aggregator == ui::RunAggregator::at_first_impact)
            {
                const auto requested_time = pinned.aggregator == ui::RunAggregator::at_first_impact ? run.first_impact_s : std::optional<double> { pinned.time_s };
                if (!requested_time || static_cast<float>(*requested_time) < run.series.time_s.front() || static_cast<float>(*requested_time) > run.series.time_s.back())
                {
                    run.pinned_results.push_back({});
                    continue;
                }
                const auto found = std::lower_bound(run.series.time_s.begin(), run.series.time_s.end(), static_cast<float>(*requested_time));
                const auto index = static_cast<std::size_t>(std::distance(run.series.time_s.begin(), found));
                value = sample_at(std::min(index, samples - 1));
                if (pinned.aggregator == ui::RunAggregator::at_time && index > 0 && index < samples && *requested_time < run.series.time_s[index])
                {
                    const auto before = sample_at(index - 1);
                    const auto span = static_cast<double>(run.series.time_s[index] - run.series.time_s[index - 1]);
                    if (span > 0.0)
                        value = before + (*value - before) * ((*requested_time - run.series.time_s[index - 1]) / span);
                }
            }
            else
            {
                double aggregate = pinned.aggregator == ui::RunAggregator::minimum ? std::numeric_limits<double>::infinity() : -std::numeric_limits<double>::infinity();
                for (std::size_t index = 0; index < samples; ++index)
                {
                    const auto sample = sample_at(index);
                    if (!std::isfinite(sample))
                        continue;
                    aggregate = pinned.aggregator == ui::RunAggregator::minimum ? std::min(aggregate, sample) : std::max(aggregate, sample);
                }
                if (std::isfinite(aggregate))
                    value = aggregate;
            }
            run.pinned_results.push_back(value && std::isfinite(*value) ? value : std::nullopt);
        }
    }

    void RunRecorder::apply_retention()
    {
        auto& current = state_->experiments[state_->experiment_id];
        while (current.kept.size() > maximum_runs_per_experiment)
        {
            const auto found = std::find_if(current.kept.begin(), current.kept.end(), [](const auto& run)
                {
                    return !run.starred;
                });
            if (found == current.kept.end())
                break;
            current.kept.erase(found);
        }
        while (memory_bytes() + full_run_bytes > memory_budget_bytes)
        {
            ExperimentRuns* owner = nullptr;
            std::vector<ui::RunRecord>::iterator oldest;
            auto oldest_order = std::numeric_limits<std::uint64_t>::max();
            for (auto& [id, experiment] : state_->experiments)
            {
                (void)id;
                const auto found = std::min_element(experiment.kept.begin(), experiment.kept.end(), [](const auto& left, const auto& right)
                    {
                        if (left.starred != right.starred)
                            return !left.starred;
                        return left.retention_order < right.retention_order;
                    });
                if (found != experiment.kept.end() && !found->starred && found->retention_order < oldest_order)
                {
                    owner = &experiment;
                    oldest = found;
                    oldest_order = found->retention_order;
                }
            }
            if (!owner)
                break;
            owner->kept.erase(oldest);
        }
    }

    const std::string& RunRecorder::experiment_id() const
    {
        return state_->experiment_id;
    }
}
