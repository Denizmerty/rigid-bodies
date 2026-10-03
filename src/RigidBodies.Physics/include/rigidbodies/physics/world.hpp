#pragma once

#include <rigidbodies/physics/broad_phase.hpp>
#include <rigidbodies/physics/constraint.hpp>
#include <rigidbodies/physics/contact_solver.hpp>
#include <rigidbodies/physics/force_generator.hpp>
#include <rigidbodies/physics/integrator.hpp>
#include <rigidbodies/physics/kinematic_motion.hpp>
#include <rigidbodies/physics/narrow_phase.hpp>
#include <rigidbodies/physics/parallel_executor.hpp>
#include <rigidbodies/physics/simulation_island.hpp>
#include <rigidbodies/physics/spring.hpp>
#include <rigidbodies/physics/world_profile.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rigidbodies::physics
{

    class World;
    struct ShapeSweep;

    // An immutable checkpoint. Copies may share this private frozen state; each restore clones it
    // so subsequent edits and simulation cannot change the checkpoint or another restored world.
    class WorldSnapshot
    {
    public:
        WorldSnapshot() = default;
        [[nodiscard]] bool is_valid() const
        {
            return state_ != nullptr;
        }

    private:
        friend class World;
        std::shared_ptr<const World> state_;
    };

    struct SleepSettings
    {
        bool enabled { true };
        Real linear_speed_m_s { 0.02 };
        Real angular_speed_rad_s { 0.02 };
        Real linear_acceleration_m_s2 { 0.02 };
        Real angular_acceleration_rad_s2 { 0.02 };
        Real quiet_duration_s { 0.5 };
    };

    struct MotionLimits
    {
        Real maximum_position_m { 100.0 };
        Real maximum_linear_speed_m_s { 100.0 };
        Real maximum_angular_speed_rad_s { 100.0 };
        Real maximum_orientation_rad { 1.0e6 };
    };

    enum class MotionLimitKind
    {
        position,
        linear_velocity,
        angular_velocity,
        orientation,
        non_finite_load,
        kinematic_path
    };

    struct MotionLimitEvent
    {
        BodyId body;
        MotionLimitKind kind;
    };

    enum class CollisionEventKind
    {
        begin,
        end
    };

    struct CollisionEvent
    {
        BroadPhasePair pair;
        CollisionEventKind kind { CollisionEventKind::begin };
        bool is_sensor { false };
    };

    struct ConstraintBreakEvent
    {
        BodyId first, second;
        std::string name;
        Real force_n {}, torque_n_m {};
    };

    struct CollisionSettings
    {
        bool continuous { true };
        Real contact_margin_m { 0.002 };
        Real matching_tolerance_m { 0.02 };
        Real sweep_tolerance_m { 1.0e-6 };
        int maximum_sweep_iterations { 64 };
        MaterialMixing friction_mixing { MaterialMixing::geometric_mean };
        MaterialMixing restitution_mixing { MaterialMixing::maximum };
    };

    // Everything about the simulated environment that is not a property of an individual body.
    struct WorldSettings
    {
        math::Vec2 gravity_m_s2 { 0.0, -standard_gravity_m_s2 };
        Real air_density_kg_m3 { default_air_density_kg_m3 };
        SolverSettings solver {};
        SleepSettings sleep {};
        MotionLimits limits {};
        CollisionSettings collision {};

        // Margin added to broad-phase bounds so that a body moving slowly does not cross a
        // boundary between one step and the next, in metres.
        Real broad_phase_margin_m { 0.01 };
        math::Vec2 air_velocity_m_s {};
        Real air_dynamic_viscosity_pa_s { 1.81e-5 };
        bool constraint_graph_enabled { true };
    };

    // Per-step figures worth showing. They exist so that the interface and the tests can observe
    // the simulation without reaching into it, and because conserved quantities are themselves a
    // subject the playground is meant to illustrate.
    struct WorldStatistics
    {
        std::size_t body_count { 0 };
        std::size_t dynamic_body_count { 0 };
        std::size_t sleeping_body_count { 0 };
        std::uint64_t limit_event_count { 0 };
        std::size_t last_step_limit_event_count { 0 };
        std::size_t broad_phase_pair_count { 0 };
        std::size_t manifold_count { 0 };
        std::size_t contact_point_count { 0 };
        std::size_t persistent_contact_point_count { 0 };
        std::size_t speculative_manifold_count { 0 };
        std::size_t sweep_iteration_limit_count { 0 };
        std::size_t ccd_clamped_body_count { 0 };
        std::size_t active_constraint_count {};
        std::size_t broken_constraint_count {};
        std::size_t constraint_island_count {};
        std::size_t simulation_island_count {};
        std::size_t active_island_count {};
        std::size_t sleeping_island_count {};
        std::size_t largest_island_body_count {};
        std::size_t constraint_row_count {};
        Real constraint_velocity_residual {};

        math::Vec2 total_linear_momentum_kg_m_s {};
        Real total_kinetic_energy_j { 0.0 };

        // Gravitational potential energy measured against the world reference height, so that the
        // sum with kinetic energy is the quantity a conservation demonstration tracks.
        Real total_potential_energy_j { 0.0 };
        Real total_spring_potential_energy_j { 0.0 };

        Real elapsed_time_s { 0.0 };
        std::uint64_t step_index { 0 };
    };

    // Owns the bodies and advances them. The world depends on nothing outside the physics and
    // mathematics modules, so a scenario can be built, stepped, and measured with no window, no
    // renderer, and no interface present. That property is what keeps the simulation testable, and
    // it is the reason the dependency only ever points in this direction.
    class World
    {
    public:
        World();
        explicit World(const WorldSettings& settings);
        World(const World&) = delete;
        World& operator=(const World&) = delete;
        World(World&&) noexcept = default;
        World& operator=(World&&) noexcept = default;

        [[nodiscard]] const WorldSettings& settings() const;
        void set_settings(const WorldSettings& value);

        // Height treated as zero gravitational potential, in metres.
        [[nodiscard]] Real potential_energy_reference_height_m() const;
        void set_potential_energy_reference_height(Real value);

        // Unique, nonempty stable keys define canonical simulation order independently of slot
        // allocation and creation history. Unkeyed bodies follow keyed bodies in creation order.
        // Reusing a live key is an error; display names remain free to change or repeat.
        BodyId create_body(const BodyDefinition& definition, std::string stable_key = {});
        bool destroy_body(BodyId id);
        void clear();

        // Paths prescribe the centre-of-mass trajectory from attachment time. Only kinematic
        // bodies accept them; clearing a path stops the drive and preserves its current velocity.
        bool set_kinematic_motion(BodyId id, const KinematicMotion& motion);
        bool clear_kinematic_motion(BodyId id);
        [[nodiscard]] const KinematicMotion* kinematic_motion(BodyId id) const;

        // Preserve all bodies, components, settings, solver caches, time, and identifier allocation.
        // Unsupported component clones throw before the target world is changed.
        // Restore invalidates pointers/references to live contents. Discard handles from the
        // abandoned future timeline too: replay deliberately reuses its original identifiers.
        [[nodiscard]] WorldSnapshot snapshot() const;
        void restore(const WorldSnapshot& snapshot);

        // Preserve the current arrangement as a new t = 0 state. Stable identifiers and authored
        // structure remain intact; run-only counters, reports, and events are cleared.
        void restart_timeline();

        // Body identifiers remain stable across storage growth and compaction. Body pointers,
        // references and contiguous views must be reacquired after create/destroy, clear,
        // restore, or authored-body transactions.
        [[nodiscard]] RigidBody* find_body(BodyId id);
        [[nodiscard]] const RigidBody* find_body(BodyId id) const;
        [[nodiscard]] bool is_valid(BodyId id) const;

        // Packed live storage has no vacant entries. Its order is an implementation detail;
        // use body_ids()/for_each_body() for canonical simulation and presentation order.
        [[nodiscard]] math::Span<const RigidBody> contiguous_bodies() const;
        [[nodiscard]] std::size_t body_storage_capacity() const;

        [[nodiscard]] std::vector<BodyId> body_ids() const;
        // Persistent identity carried by a loaded content document. Newly created bodies have
        // no document identity until saved and reloaded; invalid handles return an empty view.
        [[nodiscard]] std::string_view body_document_id(BodyId id) const;
        // Capture identifiers before visiting: callbacks may create or destroy bodies. New bodies
        // are visited only on a later traversal, and destroyed/replaced bodies are skipped.
        // A callback must not use its body reference after a structural world mutation.
        void for_each_body(const std::function<void(BodyId, RigidBody&)>& visitor);
        void for_each_body(const std::function<void(BodyId, const RigidBody&)>& visitor) const;

        // Registrations are additive: registering the same generator twice applies it twice.
        // Removal erases every matching registration in the selected global or body list.
        void add_force_generator(ForceGeneratorPtr generator);
        void remove_force_generator(const ForceGeneratorPtr& generator);
        [[nodiscard]] const std::vector<ForceGeneratorPtr>& force_generators() const;

        // Body attachments follow the complete identifier, never a recycled slot. Invalid handles
        // and null generators are rejected; removal returns whether any registration was erased.
        // Invalid handles expose an empty list. Destroying or clearing bodies drops attachments.
        bool add_force_generator(BodyId body, ForceGeneratorPtr generator);
        bool remove_force_generator(BodyId body, const ForceGeneratorPtr& generator);
        [[nodiscard]] const std::vector<ForceGeneratorPtr>& force_generators(BodyId body) const;

        // Springs are pair forces, evaluated after the one-body generators in stable-key then
        // creation order. Destroying either endpoint removes the spring. Invalid IDs return false
        // or an empty query; invalid definitions throw without changing the world.
        SpringId create_spring(const SpringDefinition& definition, std::string stable_key = {});
        bool remove_spring(SpringId id);
        bool set_spring(SpringId id, const SpringDefinition& definition);
        bool set_spring_enabled(SpringId id, bool enabled);
        [[nodiscard]] bool is_valid(SpringId id) const;
        [[nodiscard]] std::vector<SpringId> spring_ids() const;
        [[nodiscard]] const SpringDefinition* spring_definition(SpringId id) const;
        [[nodiscard]] std::optional<SpringReport> spring_report(SpringId id) const;
        [[nodiscard]] SpringId spring_by_key(std::string_view key) const;
        [[nodiscard]] std::string_view spring_key(SpringId id) const;

        // Stable keys establish canonical row order. Duplicate pointers and live keys are
        // rejected; removing either endpoint removes its constraints and wakes its neighbors.
        void add_constraint(ConstraintPtr constraint, std::string stable_key = {});
        void remove_constraint(const ConstraintPtr& constraint);
        [[nodiscard]] const std::vector<ConstraintPtr>& constraints() const;
        [[nodiscard]] ConstraintPtr constraint_by_key(std::string_view key) const;
        [[nodiscard]] const std::string& constraint_key(const ConstraintPtr& constraint) const;
        [[nodiscard]] const std::vector<ConstraintBreakEvent>& constraint_break_events() const;
        [[nodiscard]] bool bodies_can_collide(BodyId first, BodyId second) const;

        void set_integrator(IntegratorPtr integrator);
        [[nodiscard]] const Integrator& integrator() const;

        void set_broad_phase(BroadPhasePtr broad_phase);
        [[nodiscard]] const BroadPhase& broad_phase() const;

        void set_narrow_phase(NarrowPhasePtr narrow_phase);
        [[nodiscard]] const NarrowPhase& narrow_phase() const;

        void set_contact_solver(ContactSolverPtr solver);
        [[nodiscard]] const ContactSolver& contact_solver() const;

        void set_parallel_settings(const ParallelSettings& settings);
        [[nodiscard]] const ParallelSettings& parallel_settings() const;

        [[nodiscard]] const WorldProfile& profile() const;
        [[nodiscard]] bool profiling_enabled() const;
        void set_profiling_enabled(bool enabled);
        [[nodiscard]] const std::vector<SimulationIsland>& simulation_islands() const;

        // Advances the simulation by exactly one step. Callers pace themselves with a
        // TimeStepper rather than passing a frame duration here, because a fixed step is what
        // makes a run reproducible.
        // For several physical substeps within one presentation tick, capture only the first.
        void step(Real time_step_s, bool capture_previous_transform = true);

        [[nodiscard]] const WorldStatistics& statistics() const;
        [[nodiscard]] const std::vector<ContactManifold>& manifolds() const;
        [[nodiscard]] const std::vector<BroadPhasePair>& broad_phase_pairs() const;
        [[nodiscard]] const std::vector<MotionLimitEvent>& motion_limit_events() const;
        // Events describe changes at the most recent step's collision phase. Pair events refer
        // to broad-phase candidates; contact events require touching shapes (sensors included).
        // Destroyed bodies queue end events for the next step, retaining their original IDs.
        [[nodiscard]] const std::vector<CollisionEvent>& pair_events() const;
        [[nodiscard]] const std::vector<CollisionEvent>& contact_events() const;

        [[nodiscard]] const CollisionEnergyAccounting& collision_energy() const;
        [[nodiscard]] const CollisionEnergyAccounting& last_step_collision_energy() const;
        [[nodiscard]] const std::vector<CollisionImpactReport>& impact_reports() const;
        [[nodiscard]] std::size_t dropped_impact_report_count() const;
        // Work by everything except gravity, stored spring energy and contact impulses since the
        // timeline started. It is observed only: keeping it never changes the motion.
        [[nodiscard]] const EnergyLedger& energy_ledger() const;
        // Mechanical energy at the end of the latest step, which the ledger has accounted for;
        // empty before the first step. The next step books any change since as edit work.
        [[nodiscard]] std::optional<Real> ledger_end_energy_j() const;
        // Time by which potential energy readings trail the stepped positions: half of the last
        // step for semi-implicit Euler, nothing for staged integrators.
        [[nodiscard]] Real energy_measurement_lag_s() const;
        // Use after a validated in-place mass/material/velocity edit. Handles and attachments
        // remain valid; stale contact impulses are discarded and neighbors are awakened.
        bool notify_body_properties_changed(BodyId id);

        // Bounds enclosing every body, used to frame the camera on a scenario.
        [[nodiscard]] math::Aabb compute_bounds() const;

        // Topmost body containing the point, for selection in the interface. Returns an invalid
        // identifier when the point is empty.
        [[nodiscard]] BodyId find_body_at_point(const math::Vec2& world_point_m) const;

    private:
        friend struct AuthoredBodyAccess;
        friend struct ScenarioDocumentAccess;
        struct ConstraintRegistration
        {
            std::string stable_key;
            std::uint64_t observed_revision {};
            BodyId first, second;
        };
        struct SpringSlot
        {
            std::optional<SpringDefinition> definition;
            std::uint32_t generation {};
            std::string stable_key;
            std::uint64_t creation_order {};
        };
        struct BodySlot
        {
            std::size_t dense_index { vacant_body_index };
            std::uint32_t generation { 0 };
            std::string stable_key;
            std::string document_id;
            std::uint64_t creation_order { 0 };
            std::vector<ForceGeneratorPtr> force_generators;
            ForceSample applied_spring_load;
            std::optional<KinematicMotion> motion;
            Real motion_start_time_s { 0.0 };
        };

        [[nodiscard]] std::unique_ptr<World> clone(bool independent_components = true) const;
        [[nodiscard]] ParallelExecutor& parallel_executor();
        [[nodiscard]] bool precedes(std::uint32_t first, std::uint32_t second) const;
        void apply_forces(Real time_step_s);
        void apply_springs(const std::vector<BodyId>& ids, const std::vector<RigidBody*>& bodies) const;
        void wake_spring_endpoints(const SpringDefinition& definition);
        [[nodiscard]] ShapeSweep make_sweep(BodyId id, const Collider& collider, Real time_step_s, const IntegratedMotion& target, const IntegratedMotion* start = nullptr) const;
        void constrain_swept_motion(Real time_step_s, const std::vector<IntegratedMotion>& starts);
        void build_proxies(Real time_step_s, const std::vector<IntegratedMotion>& targets);
        void detect_collisions(Real time_step_s, const std::vector<IntegratedMotion>& targets);
        void remove_collision_state(BodyId id);
        void refresh_statistics();
        void begin_collision_accounting(const std::vector<IntegratedMotion>& starts);
        void finish_collision_accounting(Real time_step_s);
        void begin_energy_ledger(Real time_step_s, const std::vector<IntegratedMotion>& starts);
        void begin_ledger_velocity_solve(const std::vector<IntegratedMotion>& starts);
        void measure_contact_work(const std::vector<IntegratedMotion>& starts);
        void finish_ledger_velocity_solve(const std::vector<IntegratedMotion>& starts);
        void record_ledger_integrated_positions();
        void finish_energy_ledger(Real time_step_s, const std::vector<IntegratedMotion>& starts);
        void reset_energy_ledger();
        void enforce_motion_limits();
        [[nodiscard]] bool accept_kinematic_state(BodyId id, const KinematicState& state);
        void report_motion_limit(BodyId id, MotionLimitKind kind);
        void wake_connected_bodies(Real time_step_s);
        void update_sleep(Real time_step_s);
        void set_solver_update(bool value);
        void synchronize_constraints();
        [[nodiscard]] bool constraint_active(const Constraint& constraint) const;
        void build_simulation_islands();
        void solve_island_velocities(Real time_step_s);
        void solve_island_positions(Real time_step_s);
        void finish_island_solving();

        WorldSettings settings_;
        Real potential_energy_reference_height_m_ { 0.0 };

        static constexpr std::size_t vacant_body_index = static_cast<std::size_t>(-1);
        std::vector<RigidBody> bodies_;
        std::vector<std::uint32_t> dense_slots_;
        std::vector<BodySlot> slots_;
        std::vector<std::uint32_t> free_slots_;
        std::vector<std::uint32_t> ordered_slots_;
        std::uint32_t next_generation_ { 1 };
        std::uint64_t next_creation_order_ { 0 };

        std::vector<ForceGeneratorPtr> force_generators_;
        std::vector<ConstraintPtr> constraints_;
        std::vector<ConstraintRegistration> constraint_registrations_;
        std::vector<ConstraintBreakEvent> constraint_break_events_;
        std::vector<SpringSlot> spring_slots_;
        std::vector<std::uint32_t> free_spring_slots_;
        std::vector<std::uint32_t> ordered_spring_slots_;
        std::uint32_t next_spring_generation_ { 1 };
        std::uint64_t next_spring_creation_order_ {};

        IntegratorPtr integrator_;
        BroadPhasePtr broad_phase_;
        NarrowPhasePtr narrow_phase_;
        ContactSolverPtr contact_solver_;
        ParallelSettings parallel_settings_;
        std::shared_ptr<ParallelExecutor> parallel_executor_;
        WorldProfile profile_ {};
        bool profiling_enabled_ { false };
        std::vector<SimulationIsland> islands_;
        std::vector<std::uint32_t> island_by_slot_;
        struct IslandSolverWork
        {
            ContactSolverPtr solver;
            std::vector<ContactManifold> contacts;
            std::vector<ConstraintPtr> constraints;
        };
        std::vector<IslandSolverWork> island_work_;

        std::vector<BroadPhaseProxy> proxies_;
        std::vector<BroadPhasePair> pairs_;
        std::vector<ContactManifold> manifolds_;
        std::vector<CollisionEvent> active_pairs_;
        std::vector<CollisionEvent> active_contacts_;
        std::vector<CollisionEvent> pair_events_;
        std::vector<CollisionEvent> contact_events_;
        std::vector<CollisionEvent> pending_pair_events_;
        std::vector<CollisionEvent> pending_contact_events_;
        Real previous_contact_step_s_ { 0.0 };
        std::vector<MotionLimitEvent> motion_limit_events_;

        struct ImpactEpisode
        {
            CollisionImpactReport report;
            bool reported { false };
            bool observed { false };
            bool has_impulse { false };
            bool impact_motion { false };
        };
        CollisionEnergyAccounting collision_energy_, last_step_collision_energy_;
        std::vector<CollisionImpactReport> impact_reports_;
        std::vector<ImpactEpisode> impact_episodes_;
        std::size_t dropped_impact_report_count_ {};

        // A spring's viscous load at the start of a step, kept to measure its work afterwards.
        struct LedgerDamper
        {
            BodyId first, second;
            math::Vec2 axis {}, first_lever_m {}, second_lever_m {};
            Real load {};
            bool angular { false };
        };
        // A body's velocity, or its change over part of a step.
        struct LedgerVelocity
        {
            math::Vec2 linear_m_s {};
            Real angular_rad_s {};
        };
        // Per-step measurements; only the totals they produce outlive the step.
        struct LedgerStep
        {
            std::vector<LedgerVelocity> force_changes, contact_impulses, solved_velocities;
            // Whether the step's loads were integrated into each body, which sleeping bodies skip.
            std::vector<bool> integrated, jointed;
            std::vector<math::Vec2> integrated_centers_m;
            std::vector<LedgerDamper> dampers;
            Real start_energy_j {}, joint_work_j {}, motor_work_j {};
        };
        EnergyLedger energy_ledger_;
        LedgerStep ledger_step_;
        Real ledger_end_energy_j_ {};
        bool ledger_has_end_energy_ { false };
        // Until the first step, readings assume the TimeStepper's default fixed step.
        Real energy_time_step_s_ { 1.0 / 120.0 };

        WorldStatistics statistics_;
    };

} // namespace rigidbodies::physics
