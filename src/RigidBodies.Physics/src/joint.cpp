#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace rigidbodies::physics
{
    namespace
    {
        constexpr Real angular_tolerance = 1.0e-4;

        const JointEndpoints& endpoints(const JointDefinition& definition)
        {
            return std::visit([](const auto& value) -> const JointEndpoints&
                {
                    return value;
                },
                definition);
        }

        void require(bool condition, const char* message)
        {
            if (!condition)
                throw std::invalid_argument(message);
        }

        struct Geometry
        {
            math::Vec2 first_anchor;
            math::Vec2 second_anchor;
            math::Vec2 first_radius;
            math::Vec2 second_radius;
            math::Vec2 delta;
            Real length {};
        };

        Geometry geometry(const JointEndpoints& definition, const RigidBody& first, const RigidBody& second)
        {
            Geometry result;
            result.first_anchor = math::transform_point(first.transform(), definition.local_anchor_first_m);
            result.second_anchor = math::transform_point(second.transform(), definition.local_anchor_second_m);
            result.first_radius = result.first_anchor - first.world_center_of_mass_m();
            result.second_radius = result.second_anchor - second.world_center_of_mass_m();
            result.delta = result.second_anchor - result.first_anchor;
            result.length = std::hypot(result.delta.x, result.delta.y);
            if (!math::is_finite(result.first_anchor) || !math::is_finite(result.second_anchor) ||
                !math::is_finite(result.first_radius) || !math::is_finite(result.second_radius) ||
                !math::is_finite(result.delta) || !math::is_finite(result.length))
                throw std::overflow_error("Joint anchor geometry cannot be represented");
            return result;
        }

        struct Equation
        {
            ConstraintRow row;
            unsigned key {};
            Real error {};
            bool angular { false };
            bool motor { false };
        };

        ConstraintRow linear_row(const JointEndpoints& definition, const Geometry& placement, const math::Vec2& direction, bool axis_rotates = false)
        {
            ConstraintRow row;
            row.first = definition.first;
            row.second = definition.second;
            row.linear_first = -direction;
            row.linear_second = direction;
            row.angular_first = -math::cross(placement.first_radius + (axis_rotates ? placement.delta : math::Vec2 {}), direction);
            row.angular_second = math::cross(placement.second_radius, direction);
            return row;
        }

        ConstraintRow angular_row(const JointEndpoints& definition)
        {
            ConstraintRow row;
            row.first = definition.first;
            row.second = definition.second;
            row.angular_first = -1.0;
            row.angular_second = 1.0;
            return row;
        }

        void append_motor(std::vector<Equation>& rows, ConstraintRow row, Real speed, Real strength, Real dt, bool angular)
        {
            row.target_velocity = speed;
            row.lower_impulse = -strength * dt;
            row.upper_impulse = strength * dt;
            if (!math::is_finite(row.upper_impulse))
                throw std::overflow_error("Joint motor impulse cannot be represented");
            rows.push_back({ row, 4, 0.0, angular, true });
        }

        void append_limits(std::vector<Equation>& rows, const ConstraintRow& row, Real coordinate, Real lower, Real upper, Real dt, bool angular, bool positions)
        {
            if (lower == upper)
            {
                rows.push_back({ row, 5, coordinate - lower, angular });
                return;
            }
            auto low = row;
            low.lower_impulse = 0.0;
            // Within the interval a row only stops motion that would cross its boundary during
            // this step. Outside it, velocity is stopped and placement correction removes drift.
            low.target_velocity = positions ? 0.0 : std::min(0.0, (lower - coordinate) / dt);
            if (!positions || coordinate < lower)
                rows.push_back({ low, 5, std::min(0.0, coordinate - lower), angular });
            auto high = row;
            high.upper_impulse = 0.0;
            high.target_velocity = positions ? 0.0 : std::max(0.0, (upper - coordinate) / dt);
            if (!positions || coordinate > upper)
                rows.push_back({ high, 6, std::max(0.0, coordinate - upper), angular });
        }

        std::vector<Equation> equations(const JointDefinition& definition, const RigidBody& first, const RigidBody& second, Real dt, bool positions)
        {
            const auto& common = endpoints(definition);
            const auto placement = geometry(common, first, second);
            std::vector<Equation> result;
            result.reserve(6);
            const auto append_anchor = [&]
            {
                result.push_back({ linear_row(common, placement, { 1.0, 0.0 }), 0, placement.delta.x });
                result.push_back({ linear_row(common, placement, { 0.0, 1.0 }), 1, placement.delta.y });
            };
            std::visit([&](const auto& value)
                {
                    using Definition = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Definition, DistanceJointDefinition>)
                    {
                        if (value.length_m == 0.0)
                            append_anchor();
                        else
                        {
                            // A collapsed nonzero rod has no unique axis. Choosing the first
                            // body's x direction gives it a deterministic, finite way to recover.
                            const auto axis = placement.length > math::geometric_epsilon ? placement.delta / placement.length : math::rotate(first.transform().rotation, { 1.0, 0.0 });
                            result.push_back({ linear_row(common, placement, axis), 0, placement.length - value.length_m });
                        }
                    }
                    else
                    {
                        const auto angle = second.orientation_rad() - first.orientation_rad() - value.reference_angle_rad;
                        if constexpr (std::is_same_v<Definition, PrismaticJointDefinition>)
                        {
                            const auto axis = math::rotate(first.transform().rotation, value.local_axis_first / std::hypot(value.local_axis_first.x, value.local_axis_first.y));
                            const auto normal = math::perpendicular(axis);
                            result.push_back({ linear_row(common, placement, normal, true), 0, math::dot(placement.delta, normal) });
                            result.push_back({ angular_row(common), 2, angle, true });
                            const auto axis_row = linear_row(common, placement, axis, true);
                            if (!positions && value.motor_enabled && value.maximum_motor_force_n > 0.0)
                                append_motor(result, axis_row, value.motor_speed_m_s, value.maximum_motor_force_n, dt, false);
                            if (value.limits_enabled)
                                append_limits(result, axis_row, math::dot(placement.delta, axis), value.lower_translation_m, value.upper_translation_m, dt, false, positions);
                        }
                        else
                        {
                            append_anchor();
                            if constexpr (std::is_same_v<Definition, WeldJointDefinition>)
                                result.push_back({ angular_row(common), 2, angle, true });
                            else
                            {
                                if (!positions && value.motor_enabled && value.maximum_motor_torque_n_m > 0.0)
                                    append_motor(result, angular_row(common), value.motor_speed_rad_s, value.maximum_motor_torque_n_m, dt, true);
                                if (value.limits_enabled)
                                    append_limits(result, angular_row(common), angle, value.lower_angle_rad, value.upper_angle_rad, dt, true, positions);
                            }
                        }
                    }
                },
                definition);
            for (const auto& equation : result)
            {
                const auto& row = equation.row;
                if (!math::is_finite(row.linear_first) || !math::is_finite(row.linear_second) || !math::is_finite(row.angular_first) ||
                    !math::is_finite(row.angular_second) || !math::is_finite(row.target_velocity) || !math::is_finite(equation.error))
                    throw std::overflow_error("Joint equation cannot be represented");
            }
            return result;
        }

        Real inverse_effective_mass(const ConstraintRow& row, const RigidBody& first, const RigidBody& second)
        {
            return first.inverse_mass() * math::length_squared(row.linear_first) + second.inverse_mass() * math::length_squared(row.linear_second) +
                first.inverse_inertia() * row.angular_first * row.angular_first + second.inverse_inertia() * row.angular_second * row.angular_second;
        }

        void apply_velocity_impulse(const ConstraintRow& row, RigidBody& first, RigidBody& second, Real impulse)
        {
            first.set_simulated_velocity(first.linear_velocity_m_s() + row.linear_first * (first.inverse_mass() * impulse),
                first.angular_velocity_rad_s() + row.angular_first * first.inverse_inertia() * impulse);
            second.set_simulated_velocity(second.linear_velocity_m_s() + row.linear_second * (second.inverse_mass() * impulse),
                second.angular_velocity_rad_s() + row.angular_second * second.inverse_inertia() * impulse);
        }

        void apply_position_impulse(RigidBody& body, const math::Vec2& linear, Real angular, Real impulse)
        {
            if (body.type() != BodyType::dynamic_body || !body.is_awake())
                return;
            const auto center = body.world_center_of_mass_m() + linear * (body.inverse_mass() * impulse);
            const auto angle = body.orientation_rad() + angular * body.inverse_inertia() * impulse;
            body.set_simulated_pose(center - math::rotate(math::Rotation2 { angle }, body.mass_properties().center_of_mass_m), angle);
        }

        bool awake_dynamic(const RigidBody& body)
        {
            return body.type() == BodyType::dynamic_body && body.is_awake();
        }

        bool moving_kinematic(const RigidBody& body)
        {
            return body.type() == BodyType::kinematic_body && (math::length_squared(body.linear_velocity_m_s()) > 0.0 || body.angular_velocity_rad_s() != 0.0);
        }
    }

    void validate_joint(const JointDefinition& definition)
    {
        std::visit([](const auto& value)
            {
                require(value.first.is_valid() && value.second.is_valid() && !(value.first == value.second), "Joint endpoints must be two distinct valid identifiers");
                require(math::is_finite(value.local_anchor_first_m) && math::is_finite(value.local_anchor_second_m), "Joint anchors must be finite");
                const auto valid_threshold = [](Real threshold)
                {
                    return !std::isnan(threshold) && threshold >= 0.0;
                };
                require(valid_threshold(value.break_force_n) && valid_threshold(value.break_torque_n_m), "Joint breaking thresholds must be nonnegative finite values or positive infinity");
                using Definition = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<Definition, DistanceJointDefinition>)
                    require(math::is_finite(value.length_m) && value.length_m >= 0.0, "Joint distance must be finite and nonnegative");
                else
                {
                    require(math::is_finite(value.reference_angle_rad), "Joint reference angle must be finite");
                    if constexpr (std::is_same_v<Definition, RevoluteJointDefinition>)
                    {
                        require(math::is_finite(value.lower_angle_rad) && math::is_finite(value.upper_angle_rad) && value.lower_angle_rad <= value.upper_angle_rad, "Joint angular limits must be finite and ordered");
                        require(math::is_finite(value.motor_speed_rad_s) && math::is_finite(value.maximum_motor_torque_n_m) && value.maximum_motor_torque_n_m >= 0.0, "Joint motor speed and nonnegative torque must be finite");
                    }
                    if constexpr (std::is_same_v<Definition, PrismaticJointDefinition>)
                    {
                        const auto axis_length = std::hypot(value.local_axis_first.x, value.local_axis_first.y);
                        require(math::is_finite(value.local_axis_first) && math::is_finite(axis_length) && axis_length > math::geometric_epsilon, "Prismatic joint axis must have finite nonzero length");
                        require(math::is_finite(value.lower_translation_m) && math::is_finite(value.upper_translation_m) && value.lower_translation_m <= value.upper_translation_m, "Joint translation limits must be finite and ordered");
                        require(math::is_finite(value.motor_speed_m_s) && math::is_finite(value.maximum_motor_force_n) && value.maximum_motor_force_n >= 0.0, "Joint motor speed and nonnegative force must be finite");
                    }
                }
            },
            definition);
    }

    JointConstraint::JointConstraint(JointDefinition definition) : definition_(definition)
    {
        validate_joint(definition_);
    }

    std::shared_ptr<Constraint> JointConstraint::clone() const
    {
        return std::make_shared<JointConstraint>(*this);
    }

    std::string_view JointConstraint::name() const
    {
        constexpr std::string_view names[] { "distance_joint", "revolute_joint", "prismatic_joint", "weld_joint" };
        return names[definition_.index()];
    }

    BodyId JointConstraint::first_body() const
    {
        return endpoints(definition_).first;
    }
    BodyId JointConstraint::second_body() const
    {
        return endpoints(definition_).second;
    }
    bool JointConstraint::collide_connected() const
    {
        return endpoints(definition_).collide_connected;
    }
    bool JointConstraint::is_broken() const
    {
        return broken_;
    }
    bool JointConstraint::supports_sleeping_load() const
    {
        return true;
    }
    std::uint64_t JointConstraint::revision() const
    {
        return Constraint::revision() + definition_revision_;
    }
    Real JointConstraint::broken_force_n() const
    {
        return math::length(reaction_force_n_);
    }
    Real JointConstraint::broken_torque_n_m() const
    {
        return std::max(std::abs(reaction_torque_n_m_), std::abs(reaction_torque_first_n_m_));
    }
    const JointDefinition& JointConstraint::definition() const
    {
        return definition_;
    }

    bool JointConstraint::has_active_drive() const
    {
        return is_enabled() && !broken_ && std::visit([](const auto& value)
                                               {
                                                   using Definition = std::decay_t<decltype(value)>;
                                                   if constexpr (std::is_same_v<Definition, RevoluteJointDefinition>)
                                                       return value.motor_enabled && value.maximum_motor_torque_n_m > 0.0 && value.motor_speed_rad_s != 0.0;
                                                   else if constexpr (std::is_same_v<Definition, PrismaticJointDefinition>)
                                                       return value.motor_enabled && value.maximum_motor_force_n > 0.0 && value.motor_speed_m_s != 0.0;
                                                   else
                                                       return false;
                                               },
                                               definition_);
    }

    void JointConstraint::set_definition(JointDefinition definition)
    {
        validate_joint(definition);
        definition_ = definition;
        rows_.clear();
        row_keys_.clear();
        motor_rows_.clear();
        reaction_force_n_ = {};
        reaction_torque_n_m_ = reaction_torque_first_n_m_ = motor_force_n_ = motor_torque_n_m_ = previous_time_step_s_ = 0.0;
        broken_ = active_ = false;
        ++definition_revision_;
    }

    void JointConstraint::invalidate_cached_impulses() noexcept
    {
        rows_.clear();
        row_keys_.clear();
        motor_rows_.clear();
        previous_time_step_s_ = 0.0;
        active_ = false;
        if (!broken_)
        {
            reaction_force_n_ = {};
            reaction_torque_n_m_ = reaction_torque_first_n_m_ = motor_force_n_ = motor_torque_n_m_ = 0.0;
        }
        ++definition_revision_;
    }

    JointReport JointConstraint::report(const World& world) const
    {
        JointReport result;
        result.kind = static_cast<JointKind>(definition_.index());
        result.first = first_body();
        result.second = second_body();
        result.enabled = is_enabled() && !broken_;
        result.broken = broken_;
        result.reaction_force_n = reaction_force_n_;
        result.reaction_torque_n_m = reaction_torque_n_m_;
        result.reaction_torque_first_n_m = reaction_torque_first_n_m_;
        result.motor_force_n = motor_force_n_;
        result.motor_torque_n_m = motor_torque_n_m_;
        const auto* first = world.find_body(result.first);
        const auto* second = world.find_body(result.second);
        if (!first || !second)
        {
            result.enabled = false;
            return result;
        }
        const auto placement = geometry(endpoints(definition_), *first, *second);
        result.first_anchor_m = placement.first_anchor;
        result.second_anchor_m = placement.second_anchor;
        result.distance_m = placement.length;
        std::visit([&](const auto& value)
            {
                using Definition = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<Definition, DistanceJointDefinition>)
                {
                    result.position_error_m = std::abs(placement.length - value.length_m);
                    result.axis = placement.length > math::geometric_epsilon ? placement.delta / placement.length : math::rotate(first->transform().rotation, { 1.0, 0.0 });
                }
                else
                {
                    result.angle_rad = second->orientation_rad() - first->orientation_rad() - value.reference_angle_rad;
                    if constexpr (std::is_same_v<Definition, PrismaticJointDefinition>)
                    {
                        result.axis = math::rotate(first->transform().rotation, value.local_axis_first / std::hypot(value.local_axis_first.x, value.local_axis_first.y));
                        result.translation_m = math::dot(placement.delta, result.axis);
                        result.position_error_m = std::abs(math::cross(result.axis, placement.delta));
                        result.angular_error_rad = std::abs(result.angle_rad);
                        result.limits_enabled = value.limits_enabled;
                        result.motor_enabled = value.motor_enabled;
                        result.lower_limit = value.lower_translation_m;
                        result.upper_limit = value.upper_translation_m;
                        if (value.limits_enabled)
                            result.position_error_m = std::max(result.position_error_m, std::max(value.lower_translation_m - result.translation_m, result.translation_m - value.upper_translation_m));
                    }
                    else
                    {
                        result.position_error_m = placement.length;
                        if constexpr (std::is_same_v<Definition, WeldJointDefinition>)
                            result.angular_error_rad = std::abs(result.angle_rad);
                        else
                        {
                            result.limits_enabled = value.limits_enabled;
                            result.motor_enabled = value.motor_enabled;
                            result.lower_limit = value.lower_angle_rad;
                            result.upper_limit = value.upper_angle_rad;
                            if (value.limits_enabled)
                                result.angular_error_rad = std::max(0.0, std::max(value.lower_angle_rad - result.angle_rad, result.angle_rad - value.upper_angle_rad));
                        }
                    }
                }
            },
            definition_);
        return result;
    }

    const ConstraintRow* JointConstraint::solved_motor_row() const
    {
        if (!active_ || broken_)
            return nullptr;
        for (std::size_t index = 0; index < rows_.size() && index < motor_rows_.size(); ++index)
            if (motor_rows_[index])
                return &rows_[index];
        return nullptr;
    }

    void JointConstraint::prepare(World& world, Real dt)
    {
        active_ = false;
        if (!is_enabled() || broken_ || !math::is_finite(dt) || dt <= 0.0)
            return;
        auto* first = world.find_body(first_body());
        auto* second = world.find_body(second_body());
        if (!first || !second || (!awake_dynamic(*first) && !awake_dynamic(*second) && !moving_kinematic(*first) && !moving_kinematic(*second)))
            return;
        auto prepared = equations(definition_, *first, *second, dt, false);
        const auto old_rows = rows_;
        const auto old_keys = row_keys_;
        rows_.clear();
        row_keys_.clear();
        motor_rows_.clear();
        const auto reuse = world.settings().solver.warm_starting && previous_time_step_s_ > 0.0 && prepared_revision_ == revision();
        const auto scale = reuse ? dt / previous_time_step_s_ : 0.0;
        for (auto& equation : prepared)
        {
            const auto found = std::find(old_keys.begin(), old_keys.end(), equation.key);
            if (reuse && found != old_keys.end())
                equation.row.impulse = math::clamp(old_rows[static_cast<std::size_t>(found - old_keys.begin())].impulse * scale, equation.row.lower_impulse, equation.row.upper_impulse);
            const auto inverse_mass = inverse_effective_mass(equation.row, *first, *second);
            if (!math::is_finite(inverse_mass) || !math::is_finite(equation.row.impulse))
                throw std::overflow_error("Joint effective mass or cached impulse cannot be represented");
            if (inverse_mass <= 0.0)
                equation.row.impulse = 0.0;
            rows_.push_back(equation.row);
            row_keys_.push_back(equation.key);
            motor_rows_.push_back(equation.motor);
        }
        if (first->type() == BodyType::dynamic_body && !first->is_awake())
            first->wake();
        if (second->type() == BodyType::dynamic_body && !second->is_awake())
            second->wake();
        for (const auto& row : rows_)
            apply_velocity_impulse(row, *first, *second, row.impulse);
        previous_time_step_s_ = dt;
        prepared_revision_ = revision();
        active_ = true;
    }

    void JointConstraint::solve_velocity(World& world, Real)
    {
        if (!active_ || !is_enabled() || broken_)
            return;
        auto* first = world.find_body(first_body());
        auto* second = world.find_body(second_body());
        if (!first || !second)
            return;
        for (auto& row : rows_)
        {
            const auto inverse_mass = inverse_effective_mass(row, *first, *second);
            if (inverse_mass <= 0.0)
                continue;
            const auto speed = math::dot(row.linear_first, first->linear_velocity_m_s()) + row.angular_first * first->angular_velocity_rad_s() +
                math::dot(row.linear_second, second->linear_velocity_m_s()) + row.angular_second * second->angular_velocity_rad_s();
            const auto impulse = math::clamp(row.impulse + (row.target_velocity - speed) / inverse_mass, row.lower_impulse, row.upper_impulse);
            const auto change = impulse - row.impulse;
            row.impulse = impulse;
            apply_velocity_impulse(row, *first, *second, change);
        }
    }

    std::vector<ConstraintRow>* JointConstraint::velocity_rows()
    {
        return active_ && is_enabled() && !broken_ ? &rows_ : nullptr;
    }

    void JointConstraint::finalize_velocity(World& world, Real dt)
    {
        if (!active_ || !is_enabled() || broken_ || dt <= 0.0 || !math::is_finite(dt))
            return;
        const auto* first = world.find_body(first_body());
        const auto* second = world.find_body(second_body());
        if (!first || !second)
            return;
        const auto first_anchor = math::transform_point(first->transform(), endpoints(definition_).local_anchor_first_m);
        const auto first_radius = first_anchor - first->world_center_of_mass_m();
        const auto anchor = math::transform_point(second->transform(), endpoints(definition_).local_anchor_second_m);
        const auto radius = anchor - second->world_center_of_mass_m();
        reaction_force_n_ = {};
        reaction_torque_n_m_ = reaction_torque_first_n_m_ = motor_force_n_ = motor_torque_n_m_ = 0.0;
        for (std::size_t index = 0; index < rows_.size(); ++index)
        {
            const auto& row = rows_[index];
            const auto load = row.impulse / dt;
            reaction_force_n_ += row.linear_second * load;
            const auto couple = (row.angular_second - math::cross(radius, row.linear_second)) * load;
            reaction_torque_n_m_ += couple;
            reaction_torque_first_n_m_ += (row.angular_first - math::cross(first_radius, row.linear_first)) * load;
            if (motor_rows_[index])
            {
                if (definition_.index() == 1)
                    motor_torque_n_m_ += load;
                else
                    motor_force_n_ += load;
            }
        }
        const auto& common = endpoints(definition_);
        if (std::hypot(reaction_force_n_.x, reaction_force_n_.y) > common.break_force_n || broken_torque_n_m() > common.break_torque_n_m)
        {
            broken_ = true;
            active_ = false;
            ++definition_revision_;
            if (auto* awake_first = world.find_body(first_body()))
                awake_first->wake();
            if (auto* other = world.find_body(second_body()))
                other->wake();
        }
    }

    bool JointConstraint::solve_position(World& world, Real dt)
    {
        if (!active_ || !is_enabled() || broken_)
            return true;
        auto* first = world.find_body(first_body());
        auto* second = world.find_body(second_body());
        if (!first || !second)
            return true;
        bool solved = true;
        const auto linear_tolerance = std::min(world.settings().solver.linear_slop_m, 1.0e-4);
        const auto prepared = equations(definition_, *first, *second, dt, true);
        // Rebuild each scalar equation after the preceding correction: changing a body's angle
        // also moves its anchors and its prismatic axis.
        for (const auto& initial : prepared)
        {
            const auto current = equations(definition_, *first, *second, dt, true);
            const auto found = std::find_if(current.begin(), current.end(), [&](const Equation& value)
                {
                    return value.key == initial.key;
                });
            if (found == current.end())
                continue;
            const auto& equation = *found;
            const auto tolerance = equation.angular ? angular_tolerance : linear_tolerance;
            solved = solved && std::abs(equation.error) <= tolerance;
            const auto inverse_mass = inverse_effective_mass(equation.row, *first, *second);
            if (inverse_mass <= 0.0)
                continue;
            const auto maximum = equation.angular ? 0.2 : world.settings().solver.maximum_position_correction_m;
            const auto correction = math::clamp(equation.error, -maximum, maximum) * world.settings().solver.position_correction_fraction;
            const auto impulse = math::clamp(-correction / inverse_mass, equation.row.lower_impulse, equation.row.upper_impulse);
            apply_position_impulse(*first, equation.row.linear_first, equation.row.angular_first, impulse);
            apply_position_impulse(*second, equation.row.linear_second, equation.row.angular_second, impulse);
        }
        return solved;
    }
}
