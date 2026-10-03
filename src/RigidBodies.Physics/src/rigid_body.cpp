#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/compound_mass.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics
{

    RigidBody::RigidBody(const BodyDefinition& definition) : name_(definition.name),
                                                             type_(definition.type),
                                                             transform_(math::Transform2::from_angle(definition.position_m, definition.orientation_rad)),
                                                             orientation_rad_(definition.orientation_rad),
                                                             linear_velocity_m_s_(definition.linear_velocity_m_s),
                                                             angular_velocity_rad_s_(definition.angular_velocity_rad_s),
                                                             linear_damping_(std::max(definition.linear_damping, 0.0)),
                                                             angular_damping_(std::max(definition.angular_damping, 0.0)),
                                                             gravity_scale_(definition.gravity_scale),
                                                             fixed_rotation_(definition.fixed_rotation),
                                                             sleep_enabled_(definition.sleep_enabled),
                                                             colliders_(definition.colliders)
    {
        if (!math::is_finite(definition.gravity_scale))
            throw std::invalid_argument("Gravity scale must be finite");
        rebuild_mass_properties();
        set_simulated_velocity(definition.linear_velocity_m_s, definition.angular_velocity_rad_s);
    }

    const std::string& RigidBody::name() const
    {
        return name_;
    }

    void RigidBody::set_name(std::string value)
    {
        name_ = std::move(value);
    }

    BodyType RigidBody::type() const
    {
        return type_;
    }

    void RigidBody::set_type(BodyType value)
    {
        type_ = value;
        wake();
        motion_edited_ = true;
        if (type_ != BodyType::dynamic_body)
        {
            linear_velocity_m_s_ = {};
            angular_velocity_rad_s_ = 0.0;
            clear_accumulators();
            applied_force_n_ = {};
            applied_torque_n_m_ = 0.0;
            applied_force_channels_.clear();
        }
        refresh_inverse_mass();
    }

    bool RigidBody::is_awake() const
    {
        return type_ == BodyType::kinematic_body || (type_ == BodyType::dynamic_body && awake_);
    }

    void RigidBody::wake()
    {
        if (type_ == BodyType::dynamic_body)
        {
            if (!solver_update_)
                quiet_time_s_ = 0.0;
            awake_ = true;
            if (!solver_update_)
                motion_edited_ = true;
        }
    }

    void RigidBody::set_awake(bool value)
    {
        if (value || !sleep_enabled_)
        {
            wake();
        }
        else if (type_ == BodyType::dynamic_body)
        {
            awake_ = false;
            linear_velocity_m_s_ = {};
            angular_velocity_rad_s_ = 0.0;
            clear_accumulators();
        }
    }

    bool RigidBody::is_sleep_enabled() const
    {
        return sleep_enabled_;
    }
    void RigidBody::set_sleep_enabled(bool value)
    {
        sleep_enabled_ = value;
        if (!value)
            wake();
    }
    Real RigidBody::quiet_time_s() const
    {
        return quiet_time_s_;
    }

    const math::Transform2& RigidBody::transform() const
    {
        return transform_;
    }

    void RigidBody::set_transform(const math::Transform2& value)
    {
        wake();
        if (!solver_update_)
            motion_edited_ = true;
        transform_ = value;
        orientation_rad_ = value.rotation.angle();
        if (!solver_update_)
            capture_previous_transform();
    }

    const math::Transform2& RigidBody::previous_transform() const
    {
        return previous_transform_;
    }

    void RigidBody::capture_previous_transform()
    {
        previous_transform_ = transform_;
        previous_orientation_rad_ = orientation_rad_;
    }

    void RigidBody::set_simulated_pose(const math::Vec2& position_m, Real orientation_rad)
    {
        orientation_rad_ = orientation_rad;
        transform_ = math::Transform2::from_angle(position_m, orientation_rad);
    }

    void RigidBody::set_simulated_velocity(const math::Vec2& linear_velocity_m_s, Real angular_velocity_rad_s)
    {
        linear_velocity_m_s_ = type_ == BodyType::static_body ? math::Vec2 {} : linear_velocity_m_s;
        angular_velocity_rad_s_ = type_ == BodyType::static_body || fixed_rotation_ ? 0.0 : angular_velocity_rad_s;
    }

    math::Transform2 RigidBody::interpolated_transform(Real alpha) const
    {
        if (!math::is_finite(alpha) || alpha >= 1.0)
        {
            return transform_;
        }
        if (alpha <= 0.0)
        {
            return previous_transform_;
        }

        const auto local_center = mass_properties_.center_of_mass_m;
        const auto previous_center = math::transform_point(previous_transform_, local_center);
        const auto current_center = math::transform_point(transform_, local_center);
        const auto center = previous_center + (current_center - previous_center) * alpha;
        const auto angle = interpolated_orientation_rad(alpha);
        const math::Rotation2 rotation { angle };
        return { center - math::rotate(rotation, local_center), rotation };
    }

    Real RigidBody::interpolated_orientation_rad(Real alpha) const
    {
        if (!math::is_finite(alpha) || alpha >= 1.0)
            return orientation_rad_;
        if (alpha <= 0.0)
            return previous_orientation_rad_;
        return previous_orientation_rad_ + (orientation_rad_ - previous_orientation_rad_) * alpha;
    }

    const math::Vec2& RigidBody::position_m() const
    {
        return transform_.translation;
    }

    void RigidBody::set_position(const math::Vec2& value)
    {
        wake();
        if (!solver_update_)
            motion_edited_ = true;
        transform_.translation = value;
        if (!solver_update_)
            capture_previous_transform();
    }

    Real RigidBody::orientation_rad() const
    {
        return orientation_rad_;
    }

    void RigidBody::set_orientation(Real radians)
    {
        wake();
        if (!solver_update_)
            motion_edited_ = true;
        // The unwrapped angle is kept so that a read-out can report accumulated revolutions; the
        // cached rotation only ever needs the wrapped value.
        orientation_rad_ = radians;
        transform_.rotation = math::Rotation2 { radians };
        if (!solver_update_)
            capture_previous_transform();
    }

    const math::Vec2& RigidBody::linear_velocity_m_s() const
    {
        return linear_velocity_m_s_;
    }

    void RigidBody::set_linear_velocity(const math::Vec2& value)
    {
        if (type_ == BodyType::static_body)
        {
            return;
        }
        linear_velocity_m_s_ = value;
        wake();
    }

    Real RigidBody::angular_velocity_rad_s() const
    {
        return angular_velocity_rad_s_;
    }

    void RigidBody::set_angular_velocity(Real value)
    {
        if (type_ == BodyType::static_body || fixed_rotation_)
        {
            return;
        }
        angular_velocity_rad_s_ = value;
        wake();
    }

    math::Vec2 RigidBody::world_center_of_mass_m() const
    {
        return math::transform_point(transform_, mass_properties_.center_of_mass_m);
    }

    const MassProperties& RigidBody::mass_properties() const
    {
        return mass_properties_;
    }

    Real RigidBody::inverse_mass() const
    {
        return inverse_mass_;
    }

    Real RigidBody::inverse_inertia() const
    {
        return inverse_inertia_;
    }

    Real RigidBody::linear_damping() const
    {
        return linear_damping_;
    }

    void RigidBody::set_linear_damping(Real value)
    {
        linear_damping_ = std::max(value, 0.0);
        wake();
    }

    Real RigidBody::angular_damping() const
    {
        return angular_damping_;
    }

    void RigidBody::set_angular_damping(Real value)
    {
        angular_damping_ = std::max(value, 0.0);
        wake();
    }

    Real RigidBody::gravity_scale() const
    {
        return gravity_scale_;
    }

    void RigidBody::set_gravity_scale(Real value)
    {
        if (!math::is_finite(value))
            throw std::invalid_argument("Gravity scale must be finite");
        gravity_scale_ = value;
        wake();
    }

    bool RigidBody::has_fixed_rotation() const
    {
        return fixed_rotation_;
    }

    void RigidBody::set_fixed_rotation(bool value)
    {
        fixed_rotation_ = value;
        wake();
        if (fixed_rotation_)
        {
            angular_velocity_rad_s_ = 0.0;
        }
        refresh_inverse_mass();
    }

    const std::vector<Collider>& RigidBody::colliders() const
    {
        return colliders_;
    }

    void RigidBody::add_collider(Collider collider)
    {
        colliders_.push_back(std::move(collider));
        rebuild_mass_properties();
    }

    void RigidBody::clear_colliders()
    {
        colliders_.clear();
        rebuild_mass_properties();
    }

    void RigidBody::rebuild_mass_properties()
    {
        mass_properties_ = compute_compound_mass_properties(colliders_);
        mass_overridden_ = false;
        wake();
        motion_edited_ = true;
        refresh_inverse_mass();
        // A changed mass distribution invalidates the previous centre-of-mass path. This also
        // initializes interpolation to the creation pose when the constructor builds the mass.
        capture_previous_transform();
    }

    void RigidBody::override_mass(Real mass_kg)
    {
        if (!math::is_finite(mass_kg) || mass_kg < 0.0)
        {
            return;
        }

        if (mass_properties_.mass_kg > 0.0)
        {
            // Inertia scales with mass for an unchanged distribution, so the shape-derived ratio
            // is preserved rather than recomputed.
            const auto ratio = mass_kg / mass_properties_.mass_kg;
            mass_properties_.inertia_kg_m2 *= ratio;
        }
        mass_properties_.mass_kg = mass_kg;
        mass_overridden_ = true;
        wake();
        refresh_inverse_mass();
    }

    bool RigidBody::has_mass_override() const
    {
        return mass_overridden_;
    }

    math::Aabb RigidBody::compute_bounds() const
    {
        return compute_bounds(transform_);
    }

    math::Aabb RigidBody::compute_bounds(const math::Transform2& placement) const
    {
        math::Aabb bounds;
        for (const auto& collider : colliders_)
        {
            bounds.expand(collider.compute_bounds(placement));
        }
        return bounds;
    }

    const math::Vec2& RigidBody::accumulated_force_n() const
    {
        return accumulated_force_n_;
    }

    Real RigidBody::accumulated_torque_n_m() const
    {
        return accumulated_torque_n_m_;
    }

    void RigidBody::clear_accumulators()
    {
        accumulated_force_n_ = {};
        accumulated_torque_n_m_ = 0.0;
        accumulated_force_channels_.clear();
    }

    const math::Vec2& RigidBody::applied_force_n() const
    {
        return applied_force_n_;
    }

    Real RigidBody::applied_torque_n_m() const
    {
        return applied_torque_n_m_;
    }

    const std::vector<ForceChannelContribution>& RigidBody::applied_force_channels() const
    {
        return applied_force_channels_;
    }

    void RigidBody::accumulate_load(const math::Vec2& force_n, Real torque_n_m, std::string_view channel)
    {
        if (active_force_channel_.empty() && (force_n.x != 0.0 || force_n.y != 0.0 || torque_n_m != 0.0))
            wake();
        if (channel.empty())
        {
            channel = active_force_channel_.empty() ? std::string_view { "external" } : std::string_view { active_force_channel_ };
        }

        const auto found = std::lower_bound(accumulated_force_channels_.begin(), accumulated_force_channels_.end(), channel, [](const ForceChannelContribution& contribution, std::string_view name)
            {
                return std::string_view { contribution.name } < name;
            });
        if (found != accumulated_force_channels_.end() && found->name == channel)
        {
            found->force_n += force_n;
            found->torque_n_m += torque_n_m;
        }
        else
        {
            accumulated_force_channels_.insert(found, { std::string { channel }, force_n, torque_n_m });
        }

        accumulated_force_n_ += force_n;
        accumulated_torque_n_m_ += torque_n_m;
    }

    void RigidBody::retain_applied_forces()
    {
        applied_force_channels_ = accumulated_force_channels_;
        applied_force_n_ = accumulated_force_n_;
        applied_torque_n_m_ = accumulated_torque_n_m_;
    }

    void RigidBody::apply_force_at_center(const math::Vec2& force_n, std::string_view channel)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        accumulate_load(force_n, 0.0, channel);
    }

    void RigidBody::apply_force_at_world_point(const math::Vec2& force_n, const math::Vec2& world_point_m, std::string_view channel)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        accumulate_load(force_n, math::cross(world_point_m - world_center_of_mass_m(), force_n), channel);
    }

    void RigidBody::apply_torque(Real torque_n_m, std::string_view channel)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        accumulate_load({}, torque_n_m, channel);
    }

    void RigidBody::apply_linear_impulse(const math::Vec2& impulse_n_s)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        linear_velocity_m_s_ += impulse_n_s * inverse_mass_;
        if (impulse_n_s.x != 0.0 || impulse_n_s.y != 0.0)
            wake();
    }

    void RigidBody::apply_impulse_at_world_point(const math::Vec2& impulse_n_s, const math::Vec2& world_point_m)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        linear_velocity_m_s_ += impulse_n_s * inverse_mass_;
        angular_velocity_rad_s_ += inverse_inertia_ * math::cross(world_point_m - world_center_of_mass_m(), impulse_n_s);
        if (impulse_n_s.x != 0.0 || impulse_n_s.y != 0.0)
            wake();
    }

    void RigidBody::apply_angular_impulse(Real impulse_n_m_s)
    {
        if (type_ != BodyType::dynamic_body)
        {
            return;
        }
        angular_velocity_rad_s_ += inverse_inertia_ * impulse_n_m_s;
        if (impulse_n_m_s != 0.0)
            wake();
    }

    math::Vec2 RigidBody::velocity_at_world_point(const math::Vec2& world_point_m) const
    {
        return linear_velocity_m_s_ + math::cross(angular_velocity_rad_s_, world_point_m - world_center_of_mass_m());
    }

    math::Vec2 RigidBody::linear_momentum_kg_m_s() const
    {
        return linear_velocity_m_s_ * mass_properties_.mass_kg;
    }

    Real RigidBody::angular_momentum_about_center_kg_m2_s() const
    {
        return mass_properties_.inertia_kg_m2 * angular_velocity_rad_s_;
    }

    Real RigidBody::translational_kinetic_energy_j() const
    {
        return 0.5 * mass_properties_.mass_kg * math::length_squared(linear_velocity_m_s_);
    }

    Real RigidBody::rotational_kinetic_energy_j() const
    {
        return 0.5 * mass_properties_.inertia_kg_m2 * angular_velocity_rad_s_ * angular_velocity_rad_s_;
    }

    Real RigidBody::kinetic_energy_j() const
    {
        return translational_kinetic_energy_j() + rotational_kinetic_energy_j();
    }

    bool RigidBody::contains_world_point(const math::Vec2& world_point_m) const
    {
        return contains_world_point(world_point_m, transform_);
    }

    bool RigidBody::contains_world_point(const math::Vec2& world_point_m, const math::Transform2& placement) const
    {
        for (const auto& collider : colliders_)
        {
            if (!collider.shape)
            {
                continue;
            }
            const auto shape_transform = math::concatenate(placement, collider.local_transform);
            if (collider.shape->contains_local_point(math::inverse_transform_point(shape_transform, world_point_m)))
            {
                return true;
            }
        }
        return false;
    }

    void RigidBody::refresh_inverse_mass()
    {
        // Static and kinematic bodies are treated as infinitely massive: a zero inverse mass makes
        // every impulse expression fall out to no change without a special case in the solver.
        if (type_ != BodyType::dynamic_body || mass_properties_.mass_kg <= 0.0)
        {
            inverse_mass_ = 0.0;
            inverse_inertia_ = 0.0;
            return;
        }

        inverse_mass_ = 1.0 / mass_properties_.mass_kg;
        inverse_inertia_ = (fixed_rotation_ || mass_properties_.inertia_kg_m2 <= 0.0) ? 0.0 : 1.0 / mass_properties_.inertia_kg_m2;
    }

} // namespace rigidbodies::physics
