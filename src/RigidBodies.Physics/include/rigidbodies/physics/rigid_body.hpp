#pragma once

#include <rigidbodies/physics/collider.hpp>
#include <rigidbodies/physics/mass_properties.hpp>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    // Stable reference to a body owned by a world. The generation counter detects stale identifiers
    // when a slot is reused, including selections held across a scenario change.
    struct BodyId
    {
        std::uint32_t index { 0 };
        std::uint32_t generation { 0 };

        [[nodiscard]] bool is_valid() const
        {
            return generation != 0;
        }
    };

    [[nodiscard]] inline bool operator==(const BodyId& left, const BodyId& right)
    {
        return left.index == right.index && left.generation == right.generation;
    }

    enum class BodyType
    {
        // Never moves and has infinite effective mass. Floors, walls, and ramps.
        static_body,

        // Moves along a prescribed path and is unaffected by forces or impacts. Useful for
        // driven demonstrations such as a conveyor or a swinging arm on a fixed schedule.
        kinematic_body,

        // Fully simulated: responds to forces, torques, impacts, and constraints.
        dynamic_body
    };

    // Everything needed to create a body, shared by scenarios, serialisation, and authoring tools.
    struct BodyDefinition
    {
        std::string name;
        BodyType type { BodyType::dynamic_body };
        math::Vec2 position_m {};
        Real orientation_rad {};
        math::Vec2 linear_velocity_m_s {};
        Real angular_velocity_rad_s {};

        // Velocity is scaled by (1 - damping * dt) each step. Damping is a stabilising
        // convenience, not a physical model; genuine air resistance is a force generator.
        Real linear_damping { 0.0 };
        Real angular_damping { 0.0 };

        // Per-body multiplier on world gravity, so a scenario can show a single object falling
        // differently without changing the world.
        Real gravity_scale { 1.0 };

        // Prevents the solver from changing the orientation, which is occasionally wanted for a
        // demonstration that isolates linear motion.
        bool fixed_rotation { false };
        bool sleep_enabled { true };

        std::vector<Collider> colliders;
    };

    // Loads from a single named source, summed over one physics substep. Torques are about the
    // centre of mass; contact and constraint impulses are reported separately by their solvers.
    struct ForceChannelContribution
    {
        std::string name;
        math::Vec2 force_n {};
        Real torque_n_m {};
    };

    // A body carries the state that a Newtonian simulation needs and nothing else: a placement, a
    // velocity, an accumulated load, and the mass distribution that relates the two. It holds no
    // reference to rendering, to the interface, or to the world that owns it.
    //
    // Position is the placement of the body frame. Velocity always refers to the centre of mass,
    // which is where the equations of motion are separable, and the two are related through the
    // local centre of mass held in the mass properties.
    class RigidBody
    {
    public:
        explicit RigidBody(const BodyDefinition& definition);

        [[nodiscard]] const std::string& name() const;
        void set_name(std::string value);

        [[nodiscard]] BodyType type() const;
        void set_type(BodyType value);

        [[nodiscard]] bool is_awake() const;
        void wake();
        void set_awake(bool value);
        [[nodiscard]] bool is_sleep_enabled() const;
        void set_sleep_enabled(bool value);
        [[nodiscard]] Real quiet_time_s() const;

        [[nodiscard]] const math::Transform2& transform() const;

        // Explicit placement changes are teleports: both interpolation endpoints follow the new
        // pose so a paused edit or reset is visible immediately.
        void set_transform(const math::Transform2& value);

        [[nodiscard]] const math::Transform2& previous_transform() const;

        // Called once at the start of a fixed simulation step, before any substeps or corrections.
        void capture_previous_transform();

        // Integrators and position solvers preserve the step's starting pose. The angle remains
        // unwrapped, so interpolation follows the actual rotation even beyond half a revolution.
        void set_simulated_pose(const math::Vec2& position_m, Real orientation_rad);
        // Integration updates do not count as an external wake request.
        void set_simulated_velocity(const math::Vec2& linear_velocity_m_s, Real angular_velocity_rad_s);

        // Interpolates the centre of mass and unwrapped angle, then reconstructs the body frame.
        // Fractions outside [0, 1] are clamped; a non-finite fraction returns the current pose.
        [[nodiscard]] math::Transform2 interpolated_transform(Real alpha) const;
        [[nodiscard]] Real interpolated_orientation_rad(Real alpha) const;

        [[nodiscard]] const math::Vec2& position_m() const;
        void set_position(const math::Vec2& value);

        [[nodiscard]] Real orientation_rad() const;
        void set_orientation(Real radians);

        [[nodiscard]] const math::Vec2& linear_velocity_m_s() const;
        void set_linear_velocity(const math::Vec2& value);

        [[nodiscard]] Real angular_velocity_rad_s() const;
        void set_angular_velocity(Real value);

        [[nodiscard]] math::Vec2 world_center_of_mass_m() const;

        [[nodiscard]] const MassProperties& mass_properties() const;

        // Zero for static and kinematic bodies, which behave as though infinitely massive.
        [[nodiscard]] Real inverse_mass() const;
        [[nodiscard]] Real inverse_inertia() const;

        [[nodiscard]] Real linear_damping() const;
        void set_linear_damping(Real value);

        [[nodiscard]] Real angular_damping() const;
        void set_angular_damping(Real value);

        [[nodiscard]] Real gravity_scale() const;
        void set_gravity_scale(Real value);

        [[nodiscard]] bool has_fixed_rotation() const;
        void set_fixed_rotation(bool value);

        [[nodiscard]] const std::vector<Collider>& colliders() const;
        void add_collider(Collider collider);
        void clear_colliders();

        // Recomputes mass, centre of mass, and inertia from the attached colliders. Called
        // automatically whenever the collider set or the body type changes.
        void rebuild_mass_properties();

        // Overrides the computed mass while keeping the shape-derived distribution, for scenarios
        // that want a familiar object at a stated mass rather than at its material density.
        void override_mass(Real mass_kg);
        [[nodiscard]] bool has_mass_override() const;

        [[nodiscard]] math::Aabb compute_bounds() const;
        [[nodiscard]] math::Aabb compute_bounds(const math::Transform2& placement) const;

        // Pending load, cleared by the world at the end of every physics substep.
        [[nodiscard]] const math::Vec2& accumulated_force_n() const;
        [[nodiscard]] Real accumulated_torque_n_m() const;
        void clear_accumulators();

        // Retained loads from the last completed physics substep, available after the pending
        // accumulators have been cleared. Channels are grouped and ordered by name. These values
        // exclude contact/constraint impulses and are replaced even when the next load is zero.
        [[nodiscard]] const math::Vec2& applied_force_n() const;
        [[nodiscard]] Real applied_torque_n_m() const;
        [[nodiscard]] const std::vector<ForceChannelContribution>& applied_force_channels() const;

        // An empty channel uses the currently executing generator's name, or "external" for a
        // direct call. Explicit channel names let a generator expose several independent loads.
        void apply_force_at_center(const math::Vec2& force_n, std::string_view channel = {});

        // A force applied away from the centre of mass produces a torque as well as an
        // acceleration. Keeping that in one call is what makes the difference visible in the
        // playground rather than something a scenario has to remember to add.
        void apply_force_at_world_point(const math::Vec2& force_n, const math::Vec2& world_point_m, std::string_view channel = {});

        void apply_torque(Real torque_n_m, std::string_view channel = {});

        void apply_linear_impulse(const math::Vec2& impulse_n_s);
        void apply_impulse_at_world_point(const math::Vec2& impulse_n_s, const math::Vec2& world_point_m);
        void apply_angular_impulse(Real impulse_n_m_s);

        // Velocity of the material point of this body that currently coincides with a world point.
        [[nodiscard]] math::Vec2 velocity_at_world_point(const math::Vec2& world_point_m) const;

        [[nodiscard]] math::Vec2 linear_momentum_kg_m_s() const;
        [[nodiscard]] Real angular_momentum_about_center_kg_m2_s() const;
        [[nodiscard]] Real translational_kinetic_energy_j() const;
        [[nodiscard]] Real rotational_kinetic_energy_j() const;
        [[nodiscard]] Real kinetic_energy_j() const;

        [[nodiscard]] bool contains_world_point(const math::Vec2& world_point_m) const;
        [[nodiscard]] bool contains_world_point(const math::Vec2& world_point_m, const math::Transform2& placement) const;

    private:
        // A world snapshot deep-copies collider geometry while preserving cached body state.
        friend class World;

        void refresh_inverse_mass();
        void accumulate_load(const math::Vec2& force_n, Real torque_n_m, std::string_view channel);
        void retain_applied_forces();

        std::string name_;
        BodyType type_ { BodyType::dynamic_body };

        math::Transform2 transform_;
        Real orientation_rad_ {};
        math::Transform2 previous_transform_;
        Real previous_orientation_rad_ {};

        math::Vec2 linear_velocity_m_s_ {};
        Real angular_velocity_rad_s_ {};

        math::Vec2 accumulated_force_n_ {};
        Real accumulated_torque_n_m_ {};
        std::vector<ForceChannelContribution> accumulated_force_channels_;

        math::Vec2 applied_force_n_ {};
        Real applied_torque_n_m_ {};
        std::vector<ForceChannelContribution> applied_force_channels_;
        std::string active_force_channel_;

        MassProperties mass_properties_;
        bool mass_overridden_ { false };
        Real inverse_mass_ {};
        Real inverse_inertia_ {};

        Real linear_damping_ {};
        Real angular_damping_ {};
        Real gravity_scale_ { 1.0 };
        bool fixed_rotation_ { false };
        bool sleep_enabled_ { true };
        bool awake_ { true };
        bool solver_update_ { false };
        bool motion_edited_ { true };
        Real quiet_time_s_ { 0.0 };

        std::vector<Collider> colliders_;
    };

} // namespace rigidbodies::physics
