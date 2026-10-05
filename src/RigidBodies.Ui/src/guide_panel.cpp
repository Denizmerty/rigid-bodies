#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/joint.hpp>

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <variant>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand guide_action(UiCommandKind kind)
        {
            UiCommand result;
            result.kind = kind;
            return result;
        }
        UiCommand guide_state(std::string_view key, std::string_view value)
        {
            auto result = guide_action(UiCommandKind::none);
            result.detail = "state:" + std::string(key) + "=" + std::string(value);
            return result;
        }
        physics::BodyId annotated_body(const UiModel& model, std::string_view document_id)
        {
            const auto found = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& value)
                {
                    return value.document_id == document_id;
                });
            return found == model.objects.end() ? physics::BodyId {} : found->id;
        }
        const physics::JointConstraint* joint_by_key(const UiModel& model, std::string_view key)
        {
            if (!model.world || key.empty())
                return nullptr;
            return dynamic_cast<const physics::JointConstraint*>(model.world->constraint_by_key(std::string(key)).get());
        }

        // The lesson's motion fields are its starting conditions, which Back to start replays, so
        // they read the setup rather than the run in progress. An object made during the run has
        // no setup of its own and reads its live state.
        const physics::RigidBody* starting_body(const UiModel& model, physics::BodyId id)
        {
            if (const auto* body = model.setup_world ? model.setup_world->find_body(id) : nullptr)
                return body;
            return model.world ? model.world->find_body(id) : nullptr;
        }

        bool starting_motion_control(std::string_view key)
        {
            return key == "object.motion.velocity_x" || key == "object.motion.velocity_y" || key == "object.motion.spin" || key == "object.motion.reverse_spin";
        }

        double number_value(const UiModel& model, std::string_view key, physics::BodyId id, std::string_view instance)
        {
            const auto* body = model.world ? model.world->find_body(id) : nullptr;
            const auto* start = starting_body(model, id);
            if (key == "object.properties.mass" && body)
                return body->mass_properties().mass_kg;
            if (key == "object.properties.gravity_scale" && body)
                return body->gravity_scale();
            if (key == "object.motion.velocity_x" && start)
                return start->linear_velocity_m_s().x;
            if (key == "object.motion.velocity_y" && start)
                return start->linear_velocity_m_s().y;
            if (key == "object.motion.spin" && start)
                return start->angular_velocity_rad_s();
            if (key == "world.gravity.strength" && model.world)
                return math::length(model.world->settings().gravity_m_s2);
            if (key == "world.gravity.tilt")
                return model.gravity_direction_degrees + 90.0;
            if (key == "world.gravity.zero_height" && model.world)
                return model.world->potential_energy_reference_height_m();
            if (key == "world.relativity.speed" && model.relativity)
                return model.relativity->speed_fraction;
            if (key == "joint.motor.angular_speed" || key == "joint.motor.linear_speed")
                if (const auto* joint = joint_by_key(model, instance))
                    return std::visit([](const auto& value) -> double
                        {
                            using Definition = std::decay_t<decltype(value)>;
                            if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition>)
                                return value.motor_speed_rad_s;
                            else if constexpr (std::is_same_v<Definition, physics::PrismaticJointDefinition>)
                                return value.motor_speed_m_s;
                            else
                                return 0.0;
                        },
                        joint->definition());
            if ((key == "spring.linear.stiffness" || key == "spring.linear.damping") && model.world)
                if (const auto* spring = model.world->spring_definition(model.world->spring_by_key(std::string(instance))))
                    if (const auto* linear = std::get_if<physics::LinearSpringDefinition>(spring))
                        return key == "spring.linear.stiffness" ? linear->stiffness_n_m : linear->damping_n_s_m;
            return 0.0;
        }

        bool switch_value(const UiModel& model, std::string_view key, std::string_view instance)
        {
            if (!model.world)
                return false;
            const auto& settings = model.world->settings();
            if (key == "world.gravity.enabled")
                return math::length_squared(settings.gravity_m_s2) > 0.0;
            if (key == "world.air.resistance")
                return model.drag_enabled;
            if (key == "world.air.spin_slowing")
                return model.angular_drag_enabled;
            if (key == "world.air.spin_lift")
                return model.magnus_enabled;
            if (key == "world.collisions.catch_fast")
                return settings.collision.continuous;
            if (key == "world.advanced.reuse_impulses")
                return settings.solver.warm_starting;
            if (key == "joint.motor.enabled" || key == "joint.stops.enabled")
                if (const auto* joint = joint_by_key(model, instance))
                    return std::visit([&](const auto& value)
                        {
                            using Definition = std::decay_t<decltype(value)>;
                            if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition> || std::is_same_v<Definition, physics::PrismaticJointDefinition>)
                                return key == "joint.motor.enabled" ? value.motor_enabled : value.limits_enabled;
                            else
                                return false;
                        },
                        joint->definition());
            return false;
        }

        std::string choice_value(const UiModel& model, std::string_view key, physics::BodyId id)
        {
            if (!model.world)
                return {};
            if (key == "world.gravity.preset")
                return std::string(gravity_preset_id(model));
            if (key == "world.relativity.preset" && model.relativity)
                return std::string(relativity_preset_id(model.relativity->speed_fraction));
            if (key == "object.properties.material")
            {
                const auto* body = model.world->find_body(id);
                return body && !body->colliders().empty() ? body->colliders().front().material.name : std::string {};
            }
            if (key == "world.collisions.bounce_rule")
                switch (model.world->settings().collision.restitution_mixing)
                {
                case physics::MaterialMixing::maximum:
                    return "maximum";
                case physics::MaterialMixing::minimum:
                    return "minimum";
                case physics::MaterialMixing::arithmetic_mean:
                    return "arithmetic_mean";
                case physics::MaterialMixing::geometric_mean:
                    return "geometric_mean";
                }
            if (key == "world.advanced.integration_method")
                return std::string(model.world->integrator().name());
            if (key == "world.advanced.time_step")
                return core::fixed(model.fixed_step_s, 6);
            return {};
        }
    }

    std::string_view gravity_preset_id(const UiModel& model)
    {
        if (!model.world)
            return {};
        // A planet preset is shown only for its own strength pointing straight down; the
        // tolerance admits a scenario that stores a rounded value such as 9.81.
        const auto strength = math::length(model.world->settings().gravity_m_s2);
        if (strength <= 0.0)
            return {};
        if (std::abs(std::remainder(model.gravity_direction_degrees + 90.0, 360.0)) > 0.05)
            return "custom";
        constexpr double tolerance_m_s2 = 0.005;
        if (std::abs(strength - physics::standard_gravity_m_s2) < tolerance_m_s2)
            return "earth";
        if (std::abs(strength - 1.62) < tolerance_m_s2)
            return "moon";
        if (std::abs(strength - 3.73) < tolerance_m_s2)
            return "mars";
        return "custom";
    }

    bool prediction_pending(const UiModel& model)
    {
        return model.ask_predictions && model.scenario_content && !model.scenario_content->guide.predict.prompt.empty() && !model.prediction && model.runs.empty();
    }

    bool guide_control_fits_row(std::string_view control)
    {
        const auto* spec = find_control_spec(control);
        if (!spec)
            return false;
        switch (spec->kind)
        {
        case ControlKind::number:
        case ControlKind::stepper:
        case ControlKind::switch_control:
        case ControlKind::checkbox:
        case ControlKind::segmented:
        case ControlKind::select:
            return true;
        case ControlKind::action:
            return control == "object.motion.reverse_spin" || control == "joint.motor.reverse";
        default:
            return false;
        }
    }

    bool guide_control_row(const UiModel& model, PanelBuilder& builder, std::string_view control, std::string_view body_document_id, std::string_view instance, std::string_view label, std::string_view surface)
    {
        const auto* spec = find_control_spec(control);
        // A file's guide may name a control of the other kind of experiment, which would do nothing here.
        if (!spec || !guide_control_fits_row(control) || !control_available(model, control))
            return false;
        const auto body = annotated_body(model, body_document_id);
        UiCommand command;
        command.kind = spec->command;
        command.id = std::string(spec->command_id);
        command.body = body;
        if (!instance.empty())
            command.id = std::string(instance);
        if (starting_motion_control(control))
            command.detail = "setup";
        const auto name = label.empty() ? std::string(spec->label) : std::string(label);
        if (!body_document_id.empty() && !body.is_valid())
        {
            builder.action_row(name, {}, "This object has been removed. Choose Restore original to bring it back.");
            return true;
        }
        const auto row_instance = std::string(surface) + (body_document_id.empty() ? std::string {} : "_" + std::string(body_document_id)) + (instance.empty() ? std::string {} : "_" + std::string(instance));
        switch (spec->kind)
        {
        case ControlKind::number:
        case ControlKind::stepper:
            builder.number_row(*spec, number_value(model, control, body, instance), command, row_instance);
            break;
        case ControlKind::switch_control:
        case ControlKind::checkbox:
        {
            const auto on = switch_value(model, control, instance);
            command.flag = !on;
            builder.switch_row(*spec, on, command);
            break;
        }
        case ControlKind::segmented:
            builder.segmented_row(*spec, choice_value(model, control, body), command);
            break;
        case ControlKind::select:
        {
            const auto current = choice_value(model, control, body);
            builder.select_row(*spec, current, command, core::humanise_identifier(current));
            break;
        }
        default:
            if (control == "object.motion.reverse_spin")
            {
                const auto* target = starting_body(model, body);
                command.value = target ? -target->angular_velocity_rad_s() : 0.0;
                builder.action_row(control, name, command, target && target->angular_velocity_rad_s() != 0.0 ? "" : "It starts without spin.");
            }
            else
                builder.action_row(control, name, command);
            builder.present_last(presentation(icons::replay));
            break;
        }
        builder.instance_last(row_instance);
        if (!label.empty())
            builder.label_last(label);
        return true;
    }

    std::string_view GuidePanel::id() const
    {
        return "guide";
    }
    std::string_view GuidePanel::title() const
    {
        return "Guide";
    }
    RegionId GuidePanel::region() const
    {
        return RegionId::guide_panel;
    }

    void GuidePanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.scenario_content || model.scenario_content->guide.empty())
            return;
        const auto& content = *model.scenario_content;
        const auto& guide = content.guide;
        builder.title(content.title);
        auto collapse = guide_action(UiCommandKind::none);
        collapse.detail = "view:guide";
        builder.action_row("view.guide_collapse", "Collapse Guide", collapse);
        builder.present_last(presentation(icons::collapse_left, "G").icon_label_only().in_header());
        builder.value_row(content.collection.empty() ? "Experiment" : content.collection, core::substitute("{} step{}", guide.steps.size(), guide.steps.size() == 1 ? "" : "s"));
        if (builder.section("guide.about", "About", true))
        {
            // The hook is the Library's one-line teaser; the summary already says it in full.
            builder.paragraph(content.summary.empty() ? content.hook : content.summary);
            builder.begin_group("concepts");
            for (const auto& concept : content.concepts)
                builder.label(concept);
            builder.end_group();
        }
        if (!guide.focus.empty() && builder.section("guide.watch", "What to watch", true))
            builder.paragraph(guide.focus);
        const auto predicting_again = builder.view_value("guide.predict_again", "false") == "true";
        if (model.ask_predictions && !guide.predict.prompt.empty() && (model.runs.empty() || predicting_again) && builder.section("guide.predict", model.runs.empty() ? "Predict" : "Predict again", true))
        {
            builder.paragraph(guide.predict.prompt);
            for (const auto& option : guide.predict.options)
            {
                auto prediction = guide_action(UiCommandKind::set_prediction);
                prediction.id = option;
                builder.choice_row(option, model.prediction && model.prediction->option == option, prediction);
            }
            auto skip = guide_action(UiCommandKind::set_prediction);
            builder.action_row("Skip", skip);
            builder.present_last(presentation().quiet());
        }
        else if (model.ask_predictions && !guide.predict.prompt.empty() && !model.runs.empty())
        {
            const auto predicted = std::find_if(model.runs.begin(), model.runs.end(), [](const auto& run)
                {
                    return run.prediction.has_value();
                });
            if (predicted != model.runs.end())
                builder.paragraph("You predicted: " + (!predicted->prediction->option.empty() ? predicted->prediction->option : predicted->prediction->note));
            builder.action_row("Predict again", guide_state("guide.predict_again", "true"));
            builder.present_last(presentation(icons::replay));
        }
        if (!guide.variables.empty() && builder.section("guide.change", "Change one thing", true))
        {
            for (const auto& variable : guide.variables)
                if (!control_available(model, variable.control))
                    builder.paragraph("This control does not apply to this experiment.");
                else if (!guide_control_row(model, builder, variable.control, variable.body, variable.instance, variable.label, "guide"))
                    builder.paragraph("This control is unavailable in this version.");
        }
        if (!guide.steps.empty() && builder.section("guide.steps", "Try this", true))
        {
            for (std::size_t index = 0; index < guide.steps.size(); ++index)
            {
                const auto& step = guide.steps[index];
                const auto step_key = "guide.step." + std::string(model.scenario_id) + "." + std::to_string(index);
                const auto done = builder.view_value(step_key, "open") == "done";
                builder.checkbox_row(ControlSpec { "guide.steps.tick", step.text, "Guide > Steps", ControlKind::checkbox, UiCommandKind::none }, done, guide_state(step_key, done ? "open" : "done"));
                const auto reveal_key = !step.control.empty() ? step.control : step.open;
                if (!reveal_key.empty())
                {
                    UiCommand show;
                    show.kind = !step.instance.empty() ? UiCommandKind::select_connection : !step.body.empty() ? UiCommandKind::select_body
                                                                                                               : UiCommandKind::none;
                    show.body = annotated_body(model, step.body);
                    show.id = !step.instance.empty() ? step.instance : std::string {};
                    show.detail = "reveal:" + reveal_key;
                    builder.action_row("guide.steps.show_me", "Show me", show, control_available(model, reveal_key) ? "" : "This control does not apply to this experiment.");
                    builder.present_last(presentation(icons::frame_subject));
                }
            }
        }
        if (!guide.expect.empty() && builder.section("guide.expect", "What should happen", false))
        {
            const auto revealed = builder.view_value("guide.expect_revealed", "false") == "true";
            if ((!model.current_run || model.current_run->duration_s < 0.5) && model.runs.empty() && !revealed)
            {
                builder.paragraph("Run the experiment once before checking the explanation.");
                builder.action_row("guide.expect.reveal", "Reveal anyway", guide_state("guide.expect_revealed", "true"));
                builder.present_last(presentation(icons::show).quiet());
            }
            else
                builder.paragraph(guide.expect);
        }
        if (!guide.limits.empty() && builder.section("guide.limits", "Model limits", false))
            builder.paragraph(guide.limits);
    }
}
