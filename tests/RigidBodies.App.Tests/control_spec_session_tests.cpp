#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/spring.hpp>
#include <rigidbodies/ui/control_spec.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <optional>

namespace
{
    using namespace rigidbodies;

    physics::BodyId select_free_body(app::SimulationSession& session)
    {
        for (const auto id : session.world().body_ids())
            if (const auto* body = session.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
            {
                session.set_selection(id);
                return id;
            }
        RIGIDBODIES_FAIL("fixture has a free object");
    }

    void configure_session(app::SimulationSession& session, const ui::ControlSpec& spec)
    {
        session.configure({});
        const auto relativity = spec.key.rfind("world.relativity.", 0) == 0;
        const auto scenario = spec.key == "joint.motor.angular_speed" ? "revolute_drive"
            : spec.key == "joint.motor.linear_speed"                  ? "prismatic_drive"
            : spec.key.rfind("spring.", 0) == 0                       ? "spring_damping"
            : relativity                                              ? "chasing_light"
                                                                      : "free_fall";
        RIGIDBODIES_EXPECT(session.load_scenario(scenario), "household validation scenario loads");
        // The probe's controls need no selection: it is not a World body.
        if (relativity)
        {
            RIGIDBODIES_EXPECT(session.relativity_active(), "the relativity fixture installs its probe");
            return;
        }
        (void)select_free_body(session);
        if (spec.key.rfind("draw.", 0) == 0)
        {
            ui::UiCommand draw;
            draw.kind = ui::UiCommandKind::start_new_shape;
            session.apply(draw);
            if (spec.command == ui::UiCommandKind::set_shape_node_position)
            {
                ui::UiEvent point;
                point.kind = ui::UiEventKind::pointer_down;
                point.pointer_px = session.camera().world_to_screen({});
                session.handle_scene_event(point, false);
                point.kind = ui::UiEventKind::pointer_up;
                session.handle_scene_event(point, false);
            }
        }
    }

    std::optional<double> applied_value(const ui::ControlSpec& spec, const app::SimulationSession& session)
    {
        const auto model = session.build_model();
        const auto* body = session.world().find_body(session.selection());
        if (spec.key == "bar.speed.custom")
            return model.time_scale;
        if (spec.key == "draw.node.position_x")
            return model.shape_node_world_m.x;
        if (spec.key == "draw.node.position_y")
            return model.shape_node_world_m.y;
        if (spec.key == "camera.scale.height")
            return model.view_height_m;
        if (spec.key == "world.relativity.speed" && model.relativity)
            return model.relativity->speed_fraction;
        if (spec.key == "draw.precision.collision")
            return model.shape_collision_tolerance_m;
        if (spec.key == "draw.precision.drawing")
            return model.shape_render_tolerance_m;
        if (spec.key == "draw.precision.hollows")
            return model.shape_concavity_tolerance_m;
        if (spec.key == "draw.precision.simplify")
            return model.shape_simplification_tolerance_m;
        if (spec.key == "draw.precision.vertex_budget")
            return static_cast<double>(model.shape_vertex_budget);
        if (spec.key == "object.motion.spin" && body)
            return body->angular_velocity_rad_s();
        if (spec.key == "object.motion.position_x" && body)
            return body->position_m().x;
        if (spec.key == "object.motion.position_y" && body)
            return body->position_m().y;
        if (spec.key == "object.motion.orientation" && body)
            return math::radians_to_degrees(body->orientation_rad());
        if (spec.key == "object.motion.velocity_x" && body)
            return body->linear_velocity_m_s().x;
        if (spec.key == "object.motion.velocity_y" && body)
            return body->linear_velocity_m_s().y;
        if (spec.key == "object.properties.gravity_scale" && body)
            return body->gravity_scale();
        if (spec.key == "object.properties.mass" && body)
            return body->mass_properties().mass_kg;
        if (spec.key == "prefs.appearance.text_size")
            return model.ui_scale;
        if (spec.key == "prefs.controls.zoom_speed")
            return model.camera_zoom_sensitivity;
        if (spec.key == "show.arrows.direction")
            return math::radians_to_degrees(model.component_angle_rad);
        if (spec.key == "show.arrows.scale")
            return model.vector_scales.velocity;
        if (spec.key == "world.advanced.time_step_custom")
            return model.fixed_step_s;
        if (spec.key == "world.air.density")
            return session.world().settings().air_density_kg_m3;
        if (spec.key == "world.air.viscosity")
            return session.world().settings().air_dynamic_viscosity_pa_s;
        if (spec.key == "world.air.wind_x")
            return session.world().settings().air_velocity_m_s.x;
        if (spec.key == "world.air.wind_y")
            return session.world().settings().air_velocity_m_s.y;
        if (spec.key == "world.gravity.strength")
            return math::length(session.world().settings().gravity_m_s2);
        if (spec.key == "world.gravity.tilt")
            return model.gravity_direction_degrees;
        if (spec.key == "world.gravity.zero_height")
            return session.world().potential_energy_reference_height_m();
        if (spec.key == "joint.motor.angular_speed" || spec.key == "joint.motor.linear_speed")
        {
            const auto key = spec.key == "joint.motor.angular_speed" ? "driven_hinge" : "driven_slider";
            const auto joint = std::dynamic_pointer_cast<const physics::JointConstraint>(session.world().constraint_by_key(key));
            if (!joint)
                return std::nullopt;
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
        }
        if (spec.key.rfind("spring.", 0) == 0)
        {
            const auto angular = spec.key.rfind("spring.angular.", 0) == 0;
            const auto* definition = session.world().spring_definition(session.world().spring_by_key(angular ? "torsional" : "damped"));
            if (!definition)
                return std::nullopt;
            const auto damping = spec.key.size() >= 7 && spec.key.compare(spec.key.size() - 7, 7, "damping") == 0;
            return std::visit([&](const auto& value) -> double
                {
                    using Definition = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Definition, physics::LinearSpringDefinition>)
                        return damping ? value.damping_n_s_m : value.stiffness_n_m;
                    else
                        return damping ? value.damping_n_m_s_rad : value.stiffness_n_m_rad;
                },
                *definition);
        }
        const auto& settings = model.visual_settings;
        if (spec.command == ui::UiCommandKind::set_visual_budget)
        {
            if (spec.command_id == "motion_body_budget")
                return static_cast<double>(settings.motion_body_budget);
            if (spec.command_id == "directional_blur_budget")
                return static_cast<double>(settings.directional_blur_budget);
            if (spec.command_id == "shading_body_budget")
                return static_cast<double>(settings.shading_body_budget);
            if (spec.command_id == "contact_shadow_budget")
                return static_cast<double>(settings.contact_shadow_budget);
            if (spec.command_id == "impact_flash_budget")
                return static_cast<double>(settings.impact_flash_budget);
            if (spec.command_id == "spark_budget")
                return static_cast<double>(settings.spark_budget);
            if (spec.command_id == "dust_budget")
                return static_cast<double>(settings.dust_budget);
            if (spec.command_id == "deformation_budget")
                return static_cast<double>(settings.deformation_budget);
        }
        return std::nullopt;
    }

    ui::UiCommand command_for(const ui::ControlSpec& spec, double value)
    {
        ui::UiCommand command;
        command.kind = spec.command;
        command.id = std::string(spec.command_id);
        if (spec.key == "show.arrows.scale")
            command.id = "velocity";
        if (spec.key == "object.motion.position_x" || spec.key == "draw.node.position_x")
            command.detail = "x";
        if (spec.key == "object.motion.position_y" || spec.key == "draw.node.position_y")
            command.detail = "y";
        if (spec.key == "joint.motor.angular_speed")
            command.id = "driven_hinge";
        if (spec.key == "joint.motor.linear_speed")
            command.id = "driven_slider";
        if (spec.key.rfind("spring.angular.", 0) == 0)
            command.id = "torsional";
        if (spec.key.rfind("spring.linear.", 0) == 0)
            command.id = "damped";
        if (spec.key.rfind("spring.", 0) == 0)
            command.detail = spec.key.size() >= 7 && spec.key.compare(spec.key.size() - 7, 7, "damping") == 0 ? "damping" : "stiffness";
        command.value = value;
        command.phase = ui::UiEditPhase::commit;
        return command;
    }

    bool error_message(const ui::UiModel& model)
    {
        const auto looks_bad = [](const std::string& text)
        {
            return text.find("must") != std::string::npos || text.find("invalid") != std::string::npos || text.find("between") != std::string::npos || text.find("Could not") != std::string::npos;
        };
        return std::any_of(model.notifications.begin(), model.notifications.end(), [&](const ui::Notification& item)
                   {
                       return looks_bad(item.text);
                   }) ||
            std::any_of(model.inline_notices.begin(), model.inline_notices.end(), [&](const ui::InlineNotice& item)
                {
                    return looks_bad(item.text);
                });
    }

    RIGIDBODIES_TEST("every registered number endpoint is accepted and observable by the session")
    {
        for (const auto& spec : ui::control_specs())
        {
            if (spec.kind != ui::ControlKind::number && spec.kind != ui::ControlKind::stepper)
                continue;
            app::SimulationSession session;
            configure_session(session, spec);
            for (const auto endpoint : { spec.number.minimum, spec.number.maximum })
            {
                session.apply(command_for(spec, endpoint));
                const auto actual = applied_value(spec, session);
                RIGIDBODIES_EXPECT(actual.has_value(), "number spec has a session readback: " + std::string(spec.key));
                RIGIDBODIES_EXPECT_NEAR(*actual, endpoint, std::max(1.0e-9, std::abs(endpoint) * 1.0e-6), "session accepts endpoint for " + std::string(spec.key));
                RIGIDBODIES_EXPECT(!error_message(session.build_model()), "endpoint produces no error message for " + std::string(spec.key));
            }
        }
    }

    RIGIDBODIES_TEST("session bounds reject or clamp one step outside without an undo entry")
    {
        for (const auto& spec : ui::control_specs())
        {
            if (spec.kind != ui::ControlKind::number && spec.kind != ui::ControlKind::stepper)
                continue;
            for (const auto lower : { true, false })
            {
                if ((lower && !spec.number.session_minimum) || (!lower && !spec.number.session_maximum))
                    continue;
                app::SimulationSession session;
                configure_session(session, spec);
                const auto before = session.build_model();
                const auto outside = lower ? spec.number.minimum - spec.number.step : spec.number.maximum + spec.number.step;
                session.apply(command_for(spec, outside));
                const auto actual = applied_value(spec, session);
                RIGIDBODIES_EXPECT(actual.has_value(), "bounded number spec has a readback");
                RIGIDBODIES_EXPECT(*actual >= spec.number.minimum && *actual <= spec.number.maximum, "session rejects or clamps outside bound for " + std::string(spec.key));
                const auto after = session.build_model();
                RIGIDBODIES_EXPECT(after.can_undo == before.can_undo && after.undo_label == before.undo_label, "invalid bound creates no undo entry for " + std::string(spec.key));
            }
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
