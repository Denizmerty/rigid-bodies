#include <rigidbodies/physics/spring.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace rigidbodies::physics
{
    void validate_spring(const SpringDefinition& definition)
    {
        const auto nonnegative = [](Real value)
        {
            return math::is_finite(value) && value >= 0.0;
        };
        std::visit([&](const auto& spring)
            {
                if (!spring.first.is_valid() || !spring.second.is_valid() || spring.first == spring.second)
                    throw std::invalid_argument("A spring requires two distinct valid body identifiers");
                using Definition = std::decay_t<decltype(spring)>;
                if constexpr (std::is_same_v<Definition, LinearSpringDefinition>)
                {
                    if (!math::is_finite(spring.local_anchor_first_m) || !math::is_finite(spring.local_anchor_second_m) ||
                        !nonnegative(spring.rest_length_m) || !nonnegative(spring.stiffness_n_m) || !nonnegative(spring.damping_n_s_m))
                        throw std::invalid_argument("Linear spring parameters must be finite with nonnegative rest length, stiffness and damping");
                }
                else if (!math::is_finite(spring.rest_angle_rad) || !nonnegative(spring.stiffness_n_m_rad) || !nonnegative(spring.damping_n_m_s_rad))
                    throw std::invalid_argument("Angular spring parameters must be finite with nonnegative stiffness and damping");
            },
            definition);
    }

    SpringReport evaluate_spring(const SpringDefinition& definition, const RigidBody& first, const RigidBody& second)
    {
        SpringReport report;
        std::visit([&](const auto& spring)
            {
                report.first = spring.first;
                report.second = spring.second;
                report.enabled = spring.enabled;
                using Definition = std::decay_t<decltype(spring)>;
                if constexpr (std::is_same_v<Definition, LinearSpringDefinition>)
                {
                    report.first_anchor_m = math::transform_point(first.transform(), spring.local_anchor_first_m);
                    report.second_anchor_m = math::transform_point(second.transform(), spring.local_anchor_second_m);
                    const auto displacement = report.second_anchor_m - report.first_anchor_m;
                    const auto length = math::length(displacement);
                    // Coincident anchors have no distinguished axial direction. Apply no force
                    // at that singular point instead of inventing an orientation-dependent kick.
                    const auto axis = length > math::geometric_epsilon ? displacement / length : math::Vec2 {};
                    report.extension_m = length - spring.rest_length_m;
                    report.axial_speed_m_s = math::dot(second.velocity_at_world_point(report.second_anchor_m) -
                            first.velocity_at_world_point(report.first_anchor_m),
                        axis);
                    if (!spring.enabled)
                        return;
                    report.force_on_first_n = axis * (spring.stiffness_n_m * report.extension_m + spring.damping_n_s_m * report.axial_speed_m_s);
                    report.torque_on_first_n_m = math::cross(report.first_anchor_m - first.world_center_of_mass_m(), report.force_on_first_n);
                    report.torque_on_second_n_m = math::cross(report.second_anchor_m - second.world_center_of_mass_m(), -report.force_on_first_n);
                    report.potential_energy_j = 0.5 * spring.stiffness_n_m * report.extension_m * report.extension_m;
                    report.dissipated_power_w = spring.damping_n_s_m * report.axial_speed_m_s * report.axial_speed_m_s;
                }
                else
                {
                    report.kind = SpringKind::angular;
                    report.first_anchor_m = first.world_center_of_mass_m();
                    report.second_anchor_m = second.world_center_of_mass_m();
                    report.angular_displacement_rad = second.orientation_rad() - first.orientation_rad() - spring.rest_angle_rad;
                    report.relative_angular_speed_rad_s = second.angular_velocity_rad_s() - first.angular_velocity_rad_s();
                    if (!spring.enabled)
                        return;
                    report.torque_on_first_n_m = spring.stiffness_n_m_rad * report.angular_displacement_rad +
                        spring.damping_n_m_s_rad * report.relative_angular_speed_rad_s;
                    report.torque_on_second_n_m = -report.torque_on_first_n_m;
                    report.potential_energy_j = 0.5 * spring.stiffness_n_m_rad * report.angular_displacement_rad * report.angular_displacement_rad;
                    report.dissipated_power_w = spring.damping_n_m_s_rad * report.relative_angular_speed_rad_s * report.relative_angular_speed_rad_s;
                }
            },
            definition);
        return report;
    }

    void World::wake_spring_endpoints(const SpringDefinition& definition)
    {
        std::visit([&](const auto& spring)
            {
                if (auto* first = find_body(spring.first))
                    first->wake();
                if (auto* second = find_body(spring.second))
                    second->wake();
            },
            definition);
    }

    SpringId World::create_spring(const SpringDefinition& definition, std::string stable_key)
    {
        // Callers may duplicate a definition obtained from spring_definition(). Growing the slot
        // vector must not invalidate that input before its parameters and endpoints are copied.
        const auto requested = definition;
        validate_spring(requested);
        std::visit([&](const auto& spring)
            {
                if (!is_valid(spring.first) || !is_valid(spring.second))
                    throw std::invalid_argument("Spring endpoints must belong to the live world");
            },
            requested);
        if (!stable_key.empty())
            for (const auto index : ordered_spring_slots_)
                if (spring_slots_[index].stable_key == stable_key)
                    throw std::invalid_argument("A live spring already uses this stable key");
        ordered_spring_slots_.reserve(ordered_spring_slots_.size() + 1);
        std::uint32_t index;
        if (free_spring_slots_.empty())
        {
            index = static_cast<std::uint32_t>(spring_slots_.size());
            spring_slots_.emplace_back();
        }
        else
        {
            index = free_spring_slots_.back();
            free_spring_slots_.pop_back();
        }
        auto& slot = spring_slots_[index];
        slot.definition = requested;
        slot.stable_key = std::move(stable_key);
        slot.generation = next_spring_generation_++;
        if (next_spring_generation_ == 0)
            next_spring_generation_ = 1;
        slot.creation_order = next_spring_creation_order_++;
        const auto precedes = [&](std::uint32_t a, std::uint32_t b)
        {
            const auto& first = spring_slots_[a];
            const auto& second = spring_slots_[b];
            if (first.stable_key.empty() != second.stable_key.empty())
                return !first.stable_key.empty();
            if (first.stable_key != second.stable_key)
                return first.stable_key < second.stable_key;
            return first.creation_order < second.creation_order;
        };
        ordered_spring_slots_.insert(std::lower_bound(ordered_spring_slots_.begin(), ordered_spring_slots_.end(), index, precedes), index);
        wake_spring_endpoints(requested);
        return { index, slot.generation };
    }

    bool World::is_valid(SpringId id) const
    {
        return id.is_valid() && id.index < spring_slots_.size() && spring_slots_[id.index].definition && spring_slots_[id.index].generation == id.generation;
    }

    bool World::remove_spring(SpringId id)
    {
        if (!is_valid(id))
            return false;
        free_spring_slots_.push_back(id.index);
        auto& slot = spring_slots_[id.index];
        wake_spring_endpoints(*slot.definition);
        slot.definition.reset();
        slot.generation = 0;
        slot.stable_key.clear();
        ordered_spring_slots_.erase(std::remove(ordered_spring_slots_.begin(), ordered_spring_slots_.end(), id.index), ordered_spring_slots_.end());
        return true;
    }

    bool World::set_spring(SpringId id, const SpringDefinition& definition)
    {
        if (!is_valid(id))
            return false;
        validate_spring(definition);
        std::visit([&](const auto& spring)
            {
                if (!is_valid(spring.first) || !is_valid(spring.second))
                    throw std::invalid_argument("Spring endpoints must belong to the live world");
            },
            definition);
        auto& slot = spring_slots_[id.index];
        wake_spring_endpoints(*slot.definition);
        slot.definition = definition;
        wake_spring_endpoints(definition);
        return true;
    }

    bool World::set_spring_enabled(SpringId id, bool enabled)
    {
        if (!is_valid(id))
            return false;
        auto definition = *spring_slots_[id.index].definition;
        std::visit([&](auto& spring)
            {
                spring.enabled = enabled;
            },
            definition);
        return set_spring(id, definition);
    }

    std::vector<SpringId> World::spring_ids() const
    {
        std::vector<SpringId> result;
        result.reserve(ordered_spring_slots_.size());
        for (const auto index : ordered_spring_slots_)
            result.push_back({ index, spring_slots_[index].generation });
        return result;
    }

    const SpringDefinition* World::spring_definition(SpringId id) const
    {
        return is_valid(id) ? &*spring_slots_[id.index].definition : nullptr;
    }

    SpringId World::spring_by_key(std::string_view key) const
    {
        if (key.empty())
            return {};
        for (const auto index : ordered_spring_slots_)
            if (spring_slots_[index].stable_key == key)
                return { index, spring_slots_[index].generation };
        return {};
    }

    std::string_view World::spring_key(SpringId id) const
    {
        return is_valid(id) ? std::string_view { spring_slots_[id.index].stable_key } : std::string_view {};
    }

    std::optional<SpringReport> World::spring_report(SpringId id) const
    {
        const auto* definition = spring_definition(id);
        if (!definition)
            return std::nullopt;
        return std::visit([&](const auto& spring) -> std::optional<SpringReport>
            {
                const auto* first = find_body(spring.first);
                const auto* second = find_body(spring.second);
                if (!first || !second)
                    return std::nullopt;
                return evaluate_spring(*definition, *first, *second);
            },
            *definition);
    }

    void World::apply_springs(const std::vector<BodyId>& ids, const std::vector<RigidBody*>& bodies) const
    {
        for (const auto index : ordered_spring_slots_)
        {
            const auto& slot = spring_slots_[index];
            std::visit([&](const auto& spring)
                {
                    if (!spring.enabled)
                        return;
                    const auto a = std::find(ids.begin(), ids.end(), spring.first);
                    const auto b = std::find(ids.begin(), ids.end(), spring.second);
                    if (a == ids.end() || b == ids.end())
                        return;
                    auto& first = *bodies[static_cast<std::size_t>(a - ids.begin())];
                    auto& second = *bodies[static_cast<std::size_t>(b - ids.begin())];
                    const auto report = evaluate_spring(*slot.definition, first, second);
                    const auto channel = "spring/" + (slot.stable_key.empty() ? std::to_string(index) + ":" + std::to_string(slot.generation) : slot.stable_key);
                    struct ChannelScope
                    {
                        std::string& first;
                        std::string& second;
                        ~ChannelScope()
                        {
                            first.clear();
                            second.clear();
                        }
                    } scope { first.active_force_channel_, second.active_force_channel_ };
                    first.active_force_channel_ = channel;
                    second.active_force_channel_ = channel;
                    first.apply_force_at_center(report.force_on_first_n);
                    second.apply_force_at_center(-report.force_on_first_n);
                    first.apply_torque(report.torque_on_first_n_m);
                    second.apply_torque(report.torque_on_second_n_m);
                },
                *slot.definition);
        }
    }
} // namespace rigidbodies::physics
