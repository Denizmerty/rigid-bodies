#include <rigidbodies/physics/benchmark.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/scenario_document.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        using content::Json;

        std::string digest(std::string_view text)
        {
            std::uint64_t hash = 14695981039346656037ULL;
            for (const unsigned char byte : text)
            {
                hash ^= byte;
                hash *= 1099511628211ULL;
            }
            std::ostringstream result;
            result << std::hex << std::setw(16) << std::setfill('0') << hash;
            return result.str();
        }

        const BenchmarkWorkload& find_workload(std::string_view id)
        {
            const auto& workloads = benchmark_workloads();
            const auto found = std::find_if(workloads.begin(), workloads.end(), [&](const auto& item)
                {
                    return item.id == id;
                });
            if (found == workloads.end())
                throw std::invalid_argument("Unknown benchmark workload: " + std::string(id));
            return *found;
        }

        void validate_settings(const BenchmarkSettings& settings)
        {
            if (settings.warmup_steps > 100000 || settings.sample_steps == 0 || settings.sample_steps > 100000 ||
                settings.repetitions == 0 || settings.repetitions > 100 || settings.worker_count > 32 ||
                !std::isfinite(settings.time_step_s) || settings.time_step_s <= 0 || settings.time_step_s > 0.05 ||
                settings.repetitions * (settings.warmup_steps + settings.sample_steps) > 1000000)
                throw std::invalid_argument("Benchmark settings exceed supported step, repetition, worker, or timestep bounds");
        }

        Real next_unit(std::uint32_t& state)
        {
            // Specified integer arithmetic avoids implementation-dependent random distributions.
            state = state * 1664525U + 1013904223U;
            return static_cast<Real>(state >> 8U) / 16777216.0;
        }

        BodyId create_shape(World& world, std::string name, math::Vec2 position, bool box, BodyType type = BodyType::dynamic_body)
        {
            BodyDefinition definition;
            definition.name = name;
            definition.position_m = position;
            definition.type = type;
            Collider collider;
            collider.shape = box ? std::static_pointer_cast<Shape>(std::make_shared<ConvexPolygonShape>(ConvexPolygonShape::box(0.4, 0.4))) : std::static_pointer_cast<Shape>(std::make_shared<CircleShape>(0.2));
            collider.material = materials::oak_wood();
            definition.colliders.push_back(std::move(collider));
            return world.create_body(definition, std::move(name));
        }

        std::string state_digest(const World& world)
        {
            std::ostringstream state;
            state.imbue(std::locale::classic());
            state << std::hexfloat;
            for (const auto id : world.body_ids())
            {
                const auto& body = *world.find_body(id);
                state << body.position_m().x << ',' << body.position_m().y << ',' << body.orientation_rad() << ','
                      << body.linear_velocity_m_s().x << ',' << body.linear_velocity_m_s().y << ','
                      << body.angular_velocity_rad_s() << ',' << body.is_awake() << ';';
            }
            return digest(state.str());
        }

        std::string fingerprint(const BenchmarkWorkload& workload)
        {
            std::string source = "RigidBodies benchmark workload revision 1:" + workload.id;
            if (!workload.scenario_id.empty())
            {
                const auto* document = scenario_document_for_id(workload.scenario_id);
                if (!document)
                    throw std::runtime_error("Benchmark scenario is missing: " + workload.scenario_id);
                source += content::write_json(document->root);
            }
            return digest(source);
        }

        Json timing_json(const BenchmarkTiming& timing)
        {
            return Json::Object { { "median_ms", timing.median_ms }, { "p95_ms", timing.p95_ms }, { "minimum_ms", timing.minimum_ms }, { "maximum_ms", timing.maximum_ms } };
        }

        Real nonnegative(const Json& value)
        {
            const auto number = value.as_number();
            if (!std::isfinite(number) || number < 0)
                throw std::invalid_argument("Expected a finite nonnegative benchmark number");
            return number;
        }

        std::size_t count(const Json& value, std::size_t maximum = 1000000)
        {
            const auto number = nonnegative(value);
            if (number > static_cast<Real>(maximum) || std::floor(number) != number)
                throw std::invalid_argument("Expected a bounded benchmark count");
            return static_cast<std::size_t>(number);
        }

        BenchmarkTiming read_timing(const Json& value)
        {
            BenchmarkTiming timing { nonnegative(value.at("median_ms")), nonnegative(value.at("p95_ms")), nonnegative(value.at("minimum_ms")), nonnegative(value.at("maximum_ms")) };
            if (timing.minimum_ms > timing.median_ms || timing.median_ms > timing.p95_ms || timing.p95_ms > timing.maximum_ms)
                throw std::invalid_argument("Benchmark timing percentiles are out of order");
            return timing;
        }

        bool matching_settings(const BenchmarkSettings& left, const BenchmarkSettings& right)
        {
            return left.warmup_steps == right.warmup_steps && left.sample_steps == right.sample_steps &&
                left.repetitions == right.repetitions && left.time_step_s == right.time_step_s &&
                left.seed == right.seed && left.worker_count == right.worker_count && left.profiling == right.profiling;
        }
    }

    const std::vector<BenchmarkWorkload>& benchmark_workloads()
    {
        static const std::vector<BenchmarkWorkload> workloads {
            { "scene_stack", "Resting stack from the catalogue, including contact solving", "stable_stack" },
            { "scene_aerodynamics", "Distributed aerodynamic forces from the catalogue", "asymmetric_aerodynamics" },
            { "scene_authored", "Drawn shapes from the catalogue, split into convex collision pieces", "shape_workshop" },
            { "sparse_128", "128 separated moving circles and boxes", {} },
            { "sparse_512", "512 separated moving circles and boxes", {} },
            { "dense_256", "256 touching moving circles and boxes", {} },
            { "stacks_128", "16 independent stacks sharing one static floor", {} },
            { "mechanisms_96", "24 independent four-link distance chains", {} },
            { "sleeping_512", "512 sleeping bodies sharing one static floor", {} }
        };
        return workloads;
    }

    void populate_benchmark_world(World& world, std::string_view workload_id, std::uint32_t seed)
    {
        const auto& workload = find_workload(workload_id);
        if (!workload.scenario_id.empty())
        {
            if (!load_scenario(world, workload.scenario_id))
                throw std::runtime_error("Could not load benchmark scenario: " + workload.scenario_id);
            return;
        }
        const World initial;
        world.restore(initial.snapshot());
        auto settings = WorldSettings {};
        settings.gravity_m_s2 = {};
        settings.sleep.enabled = workload.id == "sleeping_512" || workload.id == "stacks_128";
        settings.collision.continuous = false;
        settings.limits.maximum_position_m = 1000;
        world.set_settings(settings);
        // A new world already has uniform gravity. Explicitly reset registrations so repeated
        // population is identical even when the previous workload used other forces.
        const auto forces = world.force_generators();
        for (const auto& force : forces)
            world.remove_force_generator(force);
        world.add_force_generator(std::make_shared<UniformGravity>());

        if (workload.id == "mechanisms_96")
        {
            settings.gravity_m_s2 = { 0, -standard_gravity_m_s2 };
            world.set_settings(settings);
            for (int mechanism = 0; mechanism < 24; ++mechanism)
            {
                const auto origin = math::Vec2 { (mechanism % 8) * 4.0, (mechanism / 8) * 5.0 };
                auto previous = create_shape(world, "anchor_" + std::to_string(mechanism), origin, false, BodyType::static_body);
                for (int link = 0; link < 4; ++link)
                {
                    const auto name = "link_" + std::to_string(mechanism) + "_" + std::to_string(link);
                    const auto current = create_shape(world, name, origin + math::Vec2 { (link + 1) * 0.6, 0 }, true);
                    DistanceJointDefinition joint;
                    joint.first = previous;
                    joint.second = current;
                    joint.length_m = 0.6;
                    world.add_constraint(std::make_shared<JointConstraint>(joint), name);
                    previous = current;
                }
            }
            return;
        }
        const bool dense = workload.id == "dense_256";
        const bool stacks = workload.id == "stacks_128";
        const bool sleeping = workload.id == "sleeping_512";
        const int bodies = dense ? 256 : (workload.id == "sparse_512" || sleeping ? 512 : 128);
        if (stacks || sleeping)
        {
            BodyDefinition floor;
            floor.name = "Shared floor";
            floor.type = BodyType::static_body;
            Collider collider;
            collider.shape = std::make_shared<SegmentShape>(math::Vec2 { -1, 0 }, math::Vec2 { 110, 0 });
            floor.colliders.push_back(collider);
            world.create_body(floor, "floor");
            settings.gravity_m_s2 = stacks ? math::Vec2 { 0, -standard_gravity_m_s2 } : math::Vec2 {};
            world.set_settings(settings);
        }
        for (int index = 0; index < bodies; ++index)
        {
            math::Vec2 position;
            if (stacks)
                position = { (index / 8) * 2.0, 0.2 + (index % 8) * 0.402 };
            else if (sleeping)
                position = { (index % 128) * 0.8, 0.2 + (index / 128) * 0.4 };
            else
            {
                const Real spacing = dense ? 0.399 : 1.2;
                position = { (index % 32) * spacing, (index / 32) * spacing };
            }
            const auto id = create_shape(world, "body_" + std::to_string(index), position, stacks || sleeping || index % 2 == 0);
            auto* body = world.find_body(id);
            if (sleeping)
                body->set_awake(false);
            else if (!stacks)
                body->set_linear_velocity({ (next_unit(seed) - 0.5) * 0.04, (next_unit(seed) - 0.5) * 0.04 });
        }
        if (sleeping)
        {
            // Seed contact and edit-revision caches outside the measured interval. Zero gravity
            // and zero velocity make these touching groups an exact stationary sleep workload.
            world.step(1.0 / 120.0);
            for (const auto id : world.body_ids())
                if (world.find_body(id)->type() == BodyType::dynamic_body)
                    world.find_body(id)->set_awake(false);
        }
    }

    BenchmarkTiming summarize_benchmark_samples(std::vector<Real> samples_ms)
    {
        if (samples_ms.empty() || std::any_of(samples_ms.begin(), samples_ms.end(), [](Real value)
                                      {
                                          return !std::isfinite(value) || value < 0;
                                      }))
            throw std::invalid_argument("Benchmark samples must be finite, nonnegative and nonempty");
        std::sort(samples_ms.begin(), samples_ms.end());
        const auto percentile = [&](Real fraction)
        {
            // Nearest-rank percentiles: a single sample is a valid, bounded smoke measurement.
            return samples_ms[static_cast<std::size_t>(std::ceil(fraction * static_cast<Real>(samples_ms.size()))) - 1];
        };
        return { percentile(0.5), percentile(0.95), samples_ms.front(), samples_ms.back() };
    }

    BenchmarkResult measure_benchmark(std::string_view workload_id, const BenchmarkSettings& settings)
    {
        validate_settings(settings);
        const auto& workload = find_workload(workload_id);
        BenchmarkResult result { workload.id, fingerprint(workload), {} };
        for (std::size_t repetition = 0; repetition < settings.repetitions; ++repetition)
        {
            World world;
            populate_benchmark_world(world, workload.id, settings.seed);
#ifndef RIGIDBODIES_BENCHMARK_BASELINE
            world.set_parallel_settings({ settings.worker_count, 64 });
            world.set_profiling_enabled(settings.profiling);
#endif
            std::vector<Real> times;
            times.reserve(settings.sample_steps);
            std::map<std::string, std::vector<Real>, std::less<>> phase_times;
            BenchmarkRun run;
            for (std::size_t step = 0; step < settings.warmup_steps + settings.sample_steps; ++step)
            {
                const auto start = std::chrono::steady_clock::now();
                world.step(settings.time_step_s);
                const auto elapsed = std::chrono::duration<Real, std::milli>(std::chrono::steady_clock::now() - start).count();
                if (world.statistics().last_step_limit_event_count != 0)
                    throw std::runtime_error("Benchmark hit a motion limit: " + workload.id);
                if (step < settings.warmup_steps)
                    continue;
                times.push_back(elapsed);
                const auto& statistics = world.statistics();
                run.bodies = statistics.body_count;
                run.maximum_pairs = std::max(run.maximum_pairs, statistics.broad_phase_pair_count);
                run.maximum_contacts = std::max(run.maximum_contacts, statistics.contact_point_count);
                run.maximum_constraints = std::max(run.maximum_constraints, statistics.active_constraint_count);
                run.sleeping_bodies = statistics.sleeping_body_count;
#ifndef RIGIDBODIES_BENCHMARK_BASELINE
                run.maximum_islands = std::max(run.maximum_islands, statistics.simulation_island_count);
                run.sleeping_islands = statistics.sleeping_island_count;
                if (settings.profiling)
                {
                    const auto& profile = world.profile();
                    run.broad_phase_workers = std::max(run.broad_phase_workers, profile.broad_phase_workers);
                    run.narrow_phase_workers = std::max(run.narrow_phase_workers, profile.narrow_phase_workers);
                    phase_times["forces"].push_back(profile.forces_s * 1000);
                    phase_times["wake"].push_back(profile.wake_s * 1000);
                    phase_times["velocity_integration"].push_back(profile.velocity_integration_s * 1000);
                    phase_times["broad_phase"].push_back(profile.broad_phase_s * 1000);
                    phase_times["narrow_phase"].push_back(profile.narrow_phase_s * 1000);
                    phase_times["islands"].push_back(profile.islands_s * 1000);
                    phase_times["velocity_solve"].push_back(profile.velocity_solve_s * 1000);
                    phase_times["position_integration"].push_back(profile.position_integration_s * 1000);
                    phase_times["ccd"].push_back(profile.ccd_s * 1000);
                    phase_times["position_solve"].push_back(profile.position_solve_s * 1000);
                    phase_times["sleep"].push_back(profile.sleep_s * 1000);
                    phase_times["accounting"].push_back(profile.accounting_s * 1000);
                }
#endif
            }
            run.step = summarize_benchmark_samples(std::move(times));
            for (auto& phase : phase_times)
                run.phases.emplace(phase.first, summarize_benchmark_samples(std::move(phase.second)));
            run.state_digest = state_digest(world);
            if (!result.runs.empty() && result.runs.front().state_digest != run.state_digest)
                throw std::runtime_error("Benchmark repetitions produced different final body states: " + workload.id);
            result.runs.push_back(std::move(run));
        }
        return result;
    }

    std::string write_benchmark_report(const BenchmarkReport& report)
    {
        validate_settings(report.settings);
        const auto& settings = report.settings;
        Json root = Json::Object { { "format", "rigid-bodies-benchmark" }, { "version", content::document_version() }, { "suite_revision", 1 }, { "settings", Json::Object { { "warmup_steps", settings.warmup_steps }, { "sample_steps", settings.sample_steps }, { "repetitions", settings.repetitions }, { "time_step_s", settings.time_step_s }, { "seed", settings.seed }, { "worker_count", settings.worker_count }, { "profiling", settings.profiling } } } };
        Json::Object metadata;
        for (const auto& field : report.metadata)
            metadata.emplace(field.first, field.second);
        root["metadata"] = std::move(metadata);
        Json::Array results;
        for (const auto& result : report.results)
        {
            Json::Array runs;
            for (const auto& run : result.runs)
            {
                Json::Object phases;
                for (const auto& phase : run.phases)
                    phases.emplace(phase.first, timing_json(phase.second));
                runs.emplace_back(Json::Object { { "step", timing_json(run.step) }, { "phases", std::move(phases) }, { "bodies", run.bodies }, { "maximum_pairs", run.maximum_pairs }, { "maximum_contacts", run.maximum_contacts }, { "maximum_constraints", run.maximum_constraints }, { "sleeping_bodies", run.sleeping_bodies }, { "state_digest", run.state_digest } });
                auto& recorded = runs.back();
                recorded["maximum_islands"] = run.maximum_islands;
                recorded["sleeping_islands"] = run.sleeping_islands;
                recorded["broad_phase_workers"] = run.broad_phase_workers;
                recorded["narrow_phase_workers"] = run.narrow_phase_workers;
            }
            results.emplace_back(Json::Object { { "workload", result.workload }, { "content_fingerprint", result.content_fingerprint }, { "runs", std::move(runs) } });
        }
        root["results"] = std::move(results);
        return content::write_json(root);
    }

    bool parse_benchmark_report(std::string_view text, BenchmarkReport& report, std::string& error)
    {
        try
        {
            Json root;
            if (!content::parse_json(text, root, error) || !content::validate_document_header(root, "rigid-bodies-benchmark", error))
                return false;
            if (count(root.at("suite_revision")) != 1)
                throw std::invalid_argument("Unsupported benchmark suite revision");
            BenchmarkReport parsed;
            const auto& settings = root.at("settings");
            parsed.settings.warmup_steps = count(settings.at("warmup_steps"));
            parsed.settings.sample_steps = count(settings.at("sample_steps"));
            parsed.settings.repetitions = count(settings.at("repetitions"));
            parsed.settings.time_step_s = nonnegative(settings.at("time_step_s"));
            parsed.settings.seed = static_cast<std::uint32_t>(count(settings.at("seed"), std::numeric_limits<std::uint32_t>::max()));
            parsed.settings.worker_count = count(settings.at("worker_count"));
            parsed.settings.profiling = settings.at("profiling").as_bool();
            validate_settings(parsed.settings);
            for (const auto& field : root.at("metadata").as_object())
                parsed.metadata.emplace(field.first, field.second.as_string());
            for (const auto* required : { "machine", "platform", "architecture", "cpu", "hardware_threads", "compiler", "build", "project_version" })
                if (parsed.metadata.find(required) == parsed.metadata.end() || parsed.metadata.at(required).empty())
                    throw std::invalid_argument(std::string("Missing benchmark metadata: ") + required);
            std::set<std::string> unique;
            for (const auto& value : root.at("results").as_array())
            {
                BenchmarkResult result;
                result.workload = value.at("workload").as_string();
                result.content_fingerprint = value.at("content_fingerprint").as_string();
                if (result.workload.empty() || result.content_fingerprint.empty() || !unique.insert(result.workload).second)
                    throw std::invalid_argument("Empty or duplicate benchmark workload");
                for (const auto& recorded : value.at("runs").as_array())
                {
                    BenchmarkRun run;
                    run.step = read_timing(recorded.at("step"));
                    for (const auto& phase : recorded.at("phases").as_object())
                        run.phases.emplace(phase.first, read_timing(phase.second));
                    run.bodies = count(recorded.at("bodies"));
                    run.maximum_pairs = count(recorded.at("maximum_pairs"), 100000000);
                    run.maximum_contacts = count(recorded.at("maximum_contacts"), 100000000);
                    run.maximum_constraints = count(recorded.at("maximum_constraints"));
                    run.sleeping_bodies = count(recorded.at("sleeping_bodies"));
                    if (const auto* count_value = recorded.find("maximum_islands"))
                        run.maximum_islands = count(*count_value);
                    if (const auto* count_value = recorded.find("sleeping_islands"))
                        run.sleeping_islands = count(*count_value);
                    if (const auto* count_value = recorded.find("broad_phase_workers"))
                        run.broad_phase_workers = count(*count_value, 32);
                    if (const auto* count_value = recorded.find("narrow_phase_workers"))
                        run.narrow_phase_workers = count(*count_value, 32);
                    run.state_digest = recorded.at("state_digest").as_string();
                    if (run.sleeping_bodies > run.bodies || run.state_digest.empty())
                        throw std::invalid_argument("Invalid benchmark body count or state digest");
                    result.runs.push_back(std::move(run));
                }
                if (result.runs.size() != parsed.settings.repetitions)
                    throw std::invalid_argument("Benchmark repetition count does not match settings");
                parsed.results.push_back(std::move(result));
            }
            if (parsed.results.empty() || parsed.results.size() > 1000)
                throw std::invalid_argument("Benchmark report must contain a bounded nonempty workload list");
            report = std::move(parsed);
            error.clear();
            return true;
        }
        catch (const std::exception& failure)
        {
            error = failure.what();
            return false;
        }
    }

    BenchmarkComparison compare_benchmarks(const BenchmarkReport& baseline, const BenchmarkReport& candidate, const BenchmarkComparisonSettings& settings)
    {
        for (const auto threshold : { settings.relative_threshold, settings.absolute_threshold_ms, settings.noise_multiplier, settings.maximum_relative_spread })
            if (!std::isfinite(threshold) || threshold < 0)
                throw std::invalid_argument("Benchmark comparison thresholds must be finite and nonnegative");
        BenchmarkComparison comparison;
        if (!matching_settings(baseline.settings, candidate.settings))
        {
            comparison.skipped.push_back("Warmup, samples, repetitions, timestep, seed, workers, or profiling settings differ");
            return comparison;
        }
        for (const auto* key : { "machine", "platform", "architecture", "cpu", "hardware_threads", "compiler", "build", "project_version" })
        {
            const auto left = baseline.metadata.find(key), right = candidate.metadata.find(key);
            if (left == baseline.metadata.end() || right == candidate.metadata.end() || left->second != right->second)
            {
                comparison.skipped.push_back(std::string("Machine/build fingerprint differs: ") + key);
                return comparison;
            }
        }
        comparison.compatible = true;
        for (const auto& result : candidate.results)
        {
            const auto previous = std::find_if(baseline.results.begin(), baseline.results.end(), [&](const auto& item)
                {
                    return item.workload == result.workload;
                });
            if (previous == baseline.results.end() || previous->content_fingerprint != result.content_fingerprint)
            {
                comparison.skipped.push_back(result.workload + ": workload or content fingerprint differs");
                continue;
            }
            if (previous->runs.size() < 3 || result.runs.size() < 3)
            {
                comparison.skipped.push_back(result.workload + ": use at least three repetitions to check for a regression");
                continue;
            }
            bool compared = false;
            for (const bool tail : { false, true })
            {
                std::vector<Real> old_values, new_values;
                for (const auto& run : previous->runs)
                    old_values.push_back(tail ? run.step.p95_ms : run.step.median_ms);
                for (const auto& run : result.runs)
                    new_values.push_back(tail ? run.step.p95_ms : run.step.median_ms);
                const auto old_summary = summarize_benchmark_samples(old_values);
                const auto new_summary = summarize_benchmark_samples(new_values);
                const auto metric = tail ? " p95" : " median";
                if (old_summary.maximum_ms - old_summary.minimum_ms > std::max(settings.absolute_threshold_ms, old_summary.median_ms * settings.maximum_relative_spread) ||
                    new_summary.maximum_ms - new_summary.minimum_ms > std::max(settings.absolute_threshold_ms, new_summary.median_ms * settings.maximum_relative_spread))
                {
                    comparison.skipped.push_back(result.workload + metric + ": variation between repetitions exceeds the allowed spread");
                    continue;
                }
                compared = true;
                for (auto& value : old_values)
                    value = std::abs(value - old_summary.median_ms);
                const auto allowance = std::max({ settings.absolute_threshold_ms, settings.relative_threshold * old_summary.median_ms, settings.noise_multiplier * summarize_benchmark_samples(old_values).median_ms });
                if (new_summary.minimum_ms > old_summary.maximum_ms + allowance)
                {
                    std::ostringstream message;
                    message << result.workload << metric << ": " << old_summary.median_ms << " -> " << new_summary.median_ms
                            << " ms (allowance " << allowance << " ms; every repetition slower)";
                    comparison.regressions.push_back(message.str());
                }
            }
            if (compared)
                ++comparison.compared_workloads;
        }
        return comparison;
    }
}
