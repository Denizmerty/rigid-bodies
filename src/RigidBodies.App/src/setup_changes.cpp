#include <rigidbodies/app/setup_changes.hpp>

#include <rigidbodies/core/text_format.hpp>

#include <algorithm>
#include <cmath>
#include <map>

namespace rigidbodies::app
{
    namespace
    {
        std::string identity(const physics::World& world, physics::BodyId id)
        {
            const auto document_id = world.body_document_id(id);
            if (!document_id.empty())
                return std::string(document_id);
            return core::substitute("runtime_{}_{}", id.index, id.generation);
        }

        std::string material_name(const physics::RigidBody& body)
        {
            if (body.colliders().empty())
                return {};
            const auto& first = body.colliders().front().material.name;
            return std::all_of(body.colliders().begin(), body.colliders().end(), [&](const auto& collider)
                       {
                           return collider.material.name == first;
                       })
                ? first
                : "Mixed";
        }

        void add(std::vector<ui::SetupChange>& result, std::string key, std::string control, physics::BodyId body,
            std::string label, std::string before, std::string after, ui::EditCategory category)
        {
            result.push_back({ std::move(key), std::move(control), body, std::move(label), std::move(before), std::move(after), category });
        }
    }

    std::vector<ui::SetupChange> compute_setup_changes(const physics::World& setup, const physics::World& original)
    {
        std::vector<ui::SetupChange> result;
        std::map<std::string, physics::BodyId, std::less<>> setup_ids, original_ids;
        for (const auto id : setup.body_ids())
            setup_ids[identity(setup, id)] = id;
        for (const auto id : original.body_ids())
            original_ids[identity(original, id)] = id;

        for (const auto& [key, id] : setup_ids)
        {
            const auto* current = setup.find_body(id);
            const auto found = original_ids.find(key);
            if (found == original_ids.end())
            {
                add(result, "body:" + key + ":added", "world.objects.row", id, "Added " + current->name(), "—", current->name(), ui::EditCategory::structure);
                continue;
            }
            const auto* before = original.find_body(found->second);
            const auto name = current->name();
            if (std::abs(current->mass_properties().mass_kg - before->mass_properties().mass_kg) > 1.0e-9)
                add(result, "body:" + key + ":mass", "object.properties.mass", id, "Mass of " + name, core::substitute("{} kg", core::fixed(before->mass_properties().mass_kg, 2)), core::substitute("{} kg", core::fixed(current->mass_properties().mass_kg, 2)), ui::EditCategory::parameter);
            const auto current_material = material_name(*current), before_material = material_name(*before);
            if (current_material != before_material)
                add(result, "body:" + key + ":material", "object.properties.material", id, "Material of " + name, before_material, current_material, ui::EditCategory::parameter);
            if (std::abs(current->gravity_scale() - before->gravity_scale()) > 1.0e-9)
                add(result, "body:" + key + ":gravity_scale", "object.properties.gravity_scale", id, "Gravity scale of " + name, core::substitute("{}×", core::fixed(before->gravity_scale(), 2)), core::substitute("{}×", core::fixed(current->gravity_scale(), 2)), ui::EditCategory::parameter);
            if (math::distance(current->position_m(), before->position_m()) > 1.0e-9)
                add(result, "body:" + key + ":position", "object.motion.position_x", id, "Position of " + name, core::substitute("({}, {}) m", core::fixed(before->position_m().x, 2), core::fixed(before->position_m().y, 2)), core::substitute("({}, {}) m", core::fixed(current->position_m().x, 2), core::fixed(current->position_m().y, 2)), ui::EditCategory::state);
            if (math::distance(current->linear_velocity_m_s(), before->linear_velocity_m_s()) > 1.0e-9)
                add(result, "body:" + key + ":velocity", "object.motion.velocity_x", id, "Velocity of " + name, core::substitute("({}, {}) m/s", core::fixed(before->linear_velocity_m_s().x, 2), core::fixed(before->linear_velocity_m_s().y, 2)), core::substitute("({}, {}) m/s", core::fixed(current->linear_velocity_m_s().x, 2), core::fixed(current->linear_velocity_m_s().y, 2)), ui::EditCategory::state);
        }
        for (const auto& [key, id] : original_ids)
            if (setup_ids.find(key) == setup_ids.end())
                if (const auto* body = original.find_body(id))
                    add(result, "body:" + key + ":removed", "world.objects.row", id, "Removed " + body->name(), body->name(), "—", ui::EditCategory::structure);

        const auto& current = setup.settings();
        const auto& before = original.settings();
        const auto current_g = std::hypot(current.gravity_m_s2.x, current.gravity_m_s2.y);
        const auto before_g = std::hypot(before.gravity_m_s2.x, before.gravity_m_s2.y);
        if (std::abs(current_g - before_g) > 1.0e-9)
            add(result, "world:gravity", "world.gravity.strength", {}, "Gravity", core::substitute("{} m/s²", core::fixed(before_g, 2)), core::substitute("{} m/s²", core::fixed(current_g, 2)), ui::EditCategory::parameter);
        if (current.collision.restitution_mixing != before.collision.restitution_mixing)
            add(result, "world:bounce_rule", "world.collisions.bounce_rule", {}, "Bounce rule", "Original", "Changed", ui::EditCategory::parameter);
        if (current.collision.continuous != before.collision.continuous)
            add(result, "world:continuous", "world.collisions.catch_fast", {}, "Catch fast objects", before.collision.continuous ? "On" : "Off", current.collision.continuous ? "On" : "Off", ui::EditCategory::parameter);
        if (std::abs(setup.potential_energy_reference_height_m() - original.potential_energy_reference_height_m()) > 1.0e-9)
            add(result, "world:zero_height", "world.gravity.zero_height", {}, "Zero height", core::substitute("{} m", core::fixed(original.potential_energy_reference_height_m(), 2)), core::substitute("{} m", core::fixed(setup.potential_energy_reference_height_m(), 2)), ui::EditCategory::parameter);

        return result;
    }
}
