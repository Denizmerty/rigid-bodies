#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/body_properties.hpp>
#include <rigidbodies/physics/education.hpp>
#include <rigidbodies/physics/education_accounting.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rigidbodies::app
{
    void SimulationSession::reset_measurements()
    {
        inspected_impacts_.clear();
        impact_report_index_ = 0;
        notifier_.clear_source("property");
    }

    bool SimulationSession::collect_impacts()
    {
        if (world_.impact_reports().empty())
            return false;
        bool moving_pair = false;
        // Impacts keep the names the stage and Inspector showed when they happened, so the list
        // still reads correctly after an object is renamed or removed.
        const auto names = object_names();
        const auto display_name = [&](physics::BodyId id)
        {
            const auto found = std::find_if(names.begin(), names.end(), [&](const auto& entry)
                {
                    return entry.first == id;
                });
            return found == names.end() ? std::string { "Object" } : found->second;
        };
        for (const auto& source : world_.impact_reports())
        {
            auto impact = source;
            impact.first_before.name = display_name(impact.first_before.id);
            impact.first_after.name = impact.first_before.name;
            impact.second_before.name = display_name(impact.second_before.id);
            impact.second_after.name = impact.second_before.name;
            inspected_impacts_.push_back(impact);
            run_recorder_.note_impact(impact.elapsed_time_s);
            const auto relative = impact.second_before.velocity_m_s - impact.first_before.velocity_m_s;
            moving_pair = moving_pair || -math::dot(relative, impact.normal) >= 0.10;
            if (inspected_impacts_.size() > 64)
                inspected_impacts_.erase(inspected_impacts_.begin());
        }
        impact_report_index_ = inspected_impacts_.size() - 1;
        const bool should_pause = moving_pair && (pause_on_impact_ || next_impact_armed_);
        if (should_pause)
        {
            next_impact_armed_ = false;
            pause_reason_ = { ui::PauseReason::impact, impact_report_index_ + 1 };
            ui::NotificationAction action;
            action.label = "Inspect";
            action.reveal_key = "measure.collisions.list";
            action.reveal_instance = std::to_string(impact_report_index_);
            notify(ui::Severity::info, "Paused at an impact.", "impact", action);
        }
        return should_pause;
    }

    bool SimulationSession::apply_education_command(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        switch (command.kind)
        {
        case K::set_vector_auto_scale:
            scene_settings_.vector_scales.automatic = command.flag;
            return true;
        case K::set_vector_scale:
            if (std::isfinite(command.value) && command.value > 0.0 && command.value <= 1.0e6)
            {
                auto& scales = scene_settings_.vector_scales;
                const auto value = static_cast<float>(command.value);
                if (command.id == "velocity")
                    scales.velocity = value;
                if (command.id == "acceleration")
                    scales.acceleration = value;
                if (command.id == "force")
                    scales.force = value;
                if (command.id == "momentum")
                    scales.momentum = value;
            }
            return true;
        case K::set_vector_components:
            if (command.id == "none")
                scene_settings_.vector_components = render::VectorComponents::none;
            else if (command.id == "world")
                scene_settings_.vector_components = render::VectorComponents::world_axes;
            else if (command.id == "chosen")
                scene_settings_.vector_components = render::VectorComponents::custom_axes;
            else if (command.id == "contact")
                scene_settings_.vector_components = render::VectorComponents::contact_axes;
            return true;
        case K::set_component_angle_degrees:
            if (std::isfinite(command.value))
                scene_settings_.component_angle_rad = math::degrees_to_radians(std::remainder(command.value, 360.0));
            return true;
        case K::set_display_units:
            if (command.id == "si")
                scene_settings_.display_units = core::DisplayUnits::si;
            else if (command.id == "centimetre_gram")
                scene_settings_.display_units = core::DisplayUnits::centimetre_gram;
            return true;
        case K::clear_energy_history:
            run_recorder_.clear_graph(world_.statistics().elapsed_time_s);
            return true;
        case K::set_pause_on_impact:
            pause_on_impact_ = command.flag;
            return true;
        case K::pause_at_next_impact:
            next_impact_armed_ = command.flag;
            if (command.flag)
                stepper_.set_paused(false);
            return true;
        case K::set_prediction:
            if (command.id.empty() && command.detail.empty())
                pending_prediction_.reset();
            else
                pending_prediction_ = ui::Prediction { command.id, command.detail };
            return true;
        case K::select_impact:
            if (!inspected_impacts_.empty())
            {
                const auto requested = command.value < 0.0 ? std::size_t { 0 } : static_cast<std::size_t>(command.value);
                impact_report_index_ = std::min(requested, inspected_impacts_.size() - 1);
            }
            return true;
        case K::compare_collisions:
        {
            collision_comparison_error_.clear();
            try
            {
                collision_comparison_ = physics::compare_head_on_collisions(collision_comparison_settings_);
            }
            catch (const std::exception& error)
            {
                collision_comparison_.clear();
                collision_comparison_error_ = error.what();
            }
            ui::NotificationAction reveal;
            reveal.label = "View results";
            reveal.reveal_key = "measure.theory.collisions_run";
            notify(ui::Severity::success, "Collision comparison complete.", "comparison", reveal);
            return true;
        }
        case K::set_selected_mass:
        case K::use_selected_density_mass:
        case K::set_selected_material:
        case K::set_selected_velocity_x:
        case K::set_selected_velocity_y:
        case K::set_selected_velocity:
        case K::set_selected_angular_velocity:
        {
            try
            {
                if (shape_editor_.active())
                    throw std::invalid_argument("Apply or discard the outline before editing properties.");
                const auto id = command.body.is_valid() ? command.body : selection();
                const auto* body = world_.find_body(id);
                if (!body)
                    throw std::invalid_argument("Select an object that is still in the setup.");
                physics::BodyPropertyEdit edit;
                if (command.kind == K::set_selected_mass)
                    edit.mass_kg = command.value;
                if (command.kind == K::use_selected_density_mass)
                    edit.use_density_mass = true;
                if (command.kind == K::set_selected_material)
                {
                    const auto names = physics::materials::catalogue_names();
                    if (std::find(names.begin(), names.end(), command.id) == names.end())
                        throw std::invalid_argument("Choose a known material.");
                    edit.material = physics::materials::by_name(command.id);
                }
                if (command.kind == K::set_selected_velocity_x || command.kind == K::set_selected_velocity_y || command.kind == K::set_selected_velocity)
                {
                    auto velocity = body->linear_velocity_m_s();
                    if (command.kind == K::set_selected_velocity)
                        velocity = { command.value, command.value_y };
                    else if (command.kind == K::set_selected_velocity_x)
                        velocity.x = command.value;
                    else
                        velocity.y = command.value;
                    edit.linear_velocity_m_s = velocity;
                }
                if (command.kind == K::set_selected_angular_velocity)
                    edit.angular_velocity_rad_s = command.value;
                // Validate the whole selection in an isolated world before replacing live state.
                // A mixed static/dynamic or stale selection must never receive half an edit.
                const auto targets = command.body.is_valid() ? std::vector<physics::BodyId> { command.body } : selected_bodies_;
                if (targets.size() > 1)
                {
                    physics::World candidate;
                    candidate.restore(world_.snapshot());
                    for (const auto selected : targets)
                    {
                        auto per_body = edit;
                        if (edit.linear_velocity_m_s)
                        {
                            const auto* target = candidate.find_body(selected);
                            if (!target)
                                throw std::invalid_argument("One of the selected objects has been removed.");
                            auto velocity = target->linear_velocity_m_s();
                            if (command.kind == K::set_selected_velocity)
                                velocity = { command.value, command.value_y };
                            else if (command.kind == K::set_selected_velocity_x)
                                velocity.x = command.value;
                            else
                                velocity.y = command.value;
                            per_body.linear_velocity_m_s = velocity;
                        }
                        physics::edit_body_properties(candidate, selected, per_body);
                    }
                    const auto runtime_parallel = world_.parallel_settings();
                    const auto runtime_profile = world_.profiling_enabled();
                    world_.restore(candidate.snapshot());
                    world_.set_parallel_settings(runtime_parallel);
                    world_.set_profiling_enabled(runtime_profile);
                    refresh_force_generators();
                }
                else
                    physics::edit_body_properties(world_, id, edit);
                mark_edit_changed();
                synchronize_render_history();
                if (!applying_preview_)
                {
                    scene_renderer_.clear_trajectories();
                    notify(ui::Severity::success,
                        targets.size() > 1 ? "Property applied to all selected bodies." : edit.material ? "Material applied to every part. Mass now uses the material's density."
                                                                                                        : "Property updated.",
                        "property");
                }
            }
            catch (const std::exception& error)
            {
                notify(ui::Severity::error, error.what(), "property");
            }
            return true;
        }
        default:
            return false;
        }
    }

    void SimulationSession::populate_education_model(ui::UiModel& model) const
    {
        model.pause_on_impact = pause_on_impact_;
        model.next_impact_armed = next_impact_armed_;
        model.pause_reason = pause_reason_;
        model.prediction = pending_prediction_;
        for (std::size_t index = 0; index < inspected_impacts_.size(); ++index)
        {
            const auto& impact = inspected_impacts_[index];
            auto item = ui::ImpactItem { index, impact.elapsed_time_s, impact.first_before.id, impact.second_before.id, impact.first_before.name, impact.second_before.name, impact.normal_impulse_n_s, impact.point_m, impact.first_before.velocity_m_s, impact.first_after.velocity_m_s, impact.second_before.velocity_m_s, impact.second_after.velocity_m_s, impact.impulse_on_second_n_s, impact.energy.restitution_loss_j(), impact.energy.friction_loss_j(), index == impact_report_index_ };
            item.total_momentum_before_kg_m_s = impact.first_before.momentum_kg_m_s + impact.second_before.momentum_kg_m_s;
            item.total_momentum_after_kg_m_s = impact.first_after.momentum_kg_m_s + impact.second_after.momentum_kg_m_s;
            item.first_type = impact.first_before.type;
            item.second_type = impact.second_before.type;
            item.first_momentum_before_kg_m_s = impact.first_before.momentum_kg_m_s;
            item.first_momentum_after_kg_m_s = impact.first_after.momentum_kg_m_s;
            item.second_momentum_before_kg_m_s = impact.second_before.momentum_kg_m_s;
            item.second_momentum_after_kg_m_s = impact.second_after.momentum_kg_m_s;
            item.kinetic_before_j = impact.first_before.kinetic_energy_j + impact.second_before.kinetic_energy_j;
            item.kinetic_after_j = impact.first_after.kinetic_energy_j + impact.second_after.kinetic_energy_j;
            item.coupled = impact.coupled_constraints || impact.coupled_contacts;
            if (item.selected)
                for (const auto id : { impact.first_before.id, impact.second_before.id })
                    item.weighted = item.weighted || math::length_squared(physics::effective_uniform_gravity_m_s2(world_, id)) > 0.0;
            model.impacts.push_back(std::move(item));
        }
        if (pause_reason_.reason == ui::PauseReason::impact && !inspected_impacts_.empty())
        {
            const auto& impact = inspected_impacts_[impact_report_index_ % inspected_impacts_.size()];
            ui::Banner banner;
            banner.serial = impact.step_index;
            // The body that moves is named first, as in the Collisions list.
            const auto fixed_first = impact.first_before.type == physics::BodyType::static_body && impact.second_before.type != physics::BodyType::static_body;
            const auto& named_first = fixed_first ? impact.second_before.name : impact.first_before.name;
            const auto& named_second = fixed_first ? impact.first_before.name : impact.second_before.name;
            banner.text = core::substitute("Impact {} · {} ↔ {} · t {}", impact_report_index_ + 1, named_first, named_second, core::format_quantity(impact.elapsed_time_s, core::DisplayQuantity::time, model.display_units));
            ui::NotificationAction inspect;
            inspect.label = "Inspect";
            inspect.command = ui::UiCommand { ui::UiCommandKind::select_impact, {}, static_cast<double>(impact_report_index_) };
            inspect.reveal_key = "measure.collisions.list";
            ui::NotificationAction resume;
            resume.label = "Continue";
            resume.command = ui::UiCommand { ui::UiCommandKind::toggle_pause };
            banner.actions = { std::move(inspect), std::move(resume) };
            model.banner = std::move(banner);
        }
        model.collision_comparison_settings = collision_comparison_settings_;
        model.collision_comparison = collision_comparison_;
        if (!collision_comparison_error_.empty())
            model.inline_notices.push_back({ "measure.collision.comparison", ui::Severity::error, collision_comparison_error_ });
        model.display_units = scene_settings_.display_units;
        model.vector_components = scene_settings_.vector_components;
        model.component_angle_rad = scene_settings_.component_angle_rad;
        model.oscillation_period_note = "Requires one anchored spring.";
        model.ramp_angle_note = "Select a body with a material.";
        const auto* body = world_.find_body(selection());
        if (!body)
            return;
        if (!body->colliders().empty())
        {
            const auto& material = body->colliders().front().material;
            if (material.friction_anisotropy_ratio == 1.0)
            {
                model.critical_ramp_angle_rad = physics::critical_ramp_angle_rad(material.static_friction);
                model.ramp_angle_note = "Same material; no tipping.";
            }
            else
                model.ramp_angle_note = "Anisotropic: angle depends on direction.";
        }
        for (const auto& contact : world_.manifolds())
        {
            if (contact.is_sensor || contact.is_speculative || contact.point_count == 0 || (!(contact.first == selection()) && !(contact.second == selection())))
                continue;
            const auto other = contact.first == selection() ? contact.second : contact.first;
            const auto* support = world_.find_body(other);
            if (!support || support->type() != physics::BodyType::static_body)
                continue;
            if (contact.material.first_direction.ratio == 1.0 && contact.material.second_direction.ratio == 1.0)
            {
                model.critical_ramp_angle_rad = physics::critical_ramp_angle_rad(contact.material.static_friction);
                model.ramp_angle_note = "Current contact; no tipping.";
            }
            else
            {
                model.critical_ramp_angle_rad.reset();
                model.ramp_angle_note = "Directional contact: no single angle.";
            }
            break;
        }
        std::size_t springs = 0;
        const physics::LinearSpringDefinition* spring = nullptr;
        for (const auto id : world_.spring_ids())
        {
            const auto* definition = world_.spring_definition(id);
            const auto touches = std::visit([&](const auto& value)
                {
                    return value.enabled && (value.first == selection() || value.second == selection());
                },
                *definition);
            if (touches)
            {
                ++springs;
                spring = std::get_if<physics::LinearSpringDefinition>(definition);
            }
        }
        if (springs == 1 && spring && body->type() == physics::BodyType::dynamic_body && body->linear_damping() == 0.0)
        {
            const auto other = spring->first == selection() ? spring->second : spring->first;
            const auto* anchor = world_.find_body(other);
            const auto local = spring->first == selection() ? spring->local_anchor_first_m : spring->local_anchor_second_m;
            if (anchor && anchor->type() == physics::BodyType::static_body && math::distance(local, body->mass_properties().center_of_mass_m) < 1.0e-9)
            {
                model.oscillation_period_s = physics::oscillation_period_s(body->mass_properties().mass_kg, spring->stiffness_n_m, spring->damping_n_s_m);
                model.oscillation_period_note = model.oscillation_period_s ? "Prediction for motion along the spring only." : "No oscillation: overdamped.";
            }
        }
    }
}
