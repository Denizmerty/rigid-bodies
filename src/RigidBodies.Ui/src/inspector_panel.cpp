#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/ui/icons.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <type_traits>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand request(UiCommandKind kind, physics::BodyId body = {})
        {
            UiCommand result;
            result.kind = kind;
            result.body = body;
            return result;
        }
        UiCommand flag(UiCommandKind kind, bool value)
        {
            auto result = request(kind);
            result.flag = value;
            return result;
        }
        const ControlSpec& spec(std::string_view key)
        {
            return *find_control_spec(key);
        }
        std::string body_name(const UiModel& model, physics::BodyId id, const physics::RigidBody& body)
        {
            const auto found = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& item)
                {
                    return item.id == id;
                });
            if (found != model.objects.end())
                return found->display_name;
            if (!body.name().empty())
                return core::humanise_identifier(body.name());
            std::size_t index = 1;
            if (model.world)
                for (const auto type : { physics::BodyType::dynamic_body, physics::BodyType::kinematic_body, physics::BodyType::static_body })
                    for (const auto candidate : model.world->body_ids())
                        if (const auto* current = model.world->find_body(candidate); current && current->type() == type && current->name().empty())
                        {
                            if (candidate == id)
                                return "Object " + std::to_string(index);
                            ++index;
                        }
            return "Object";
        }
        std::string_view material_id(const physics::RigidBody& body)
        {
            return body.colliders().empty() ? std::string_view {} : std::string_view(body.colliders().front().material.name);
        }

        // A selection edits every selected object at once, so each row shows the value they share
        // or reads Mixed; it never shows a value that none of them has. Rows that cannot apply to
        // every selected object are left out rather than offered and then refused.
        void several_objects(const UiModel& model, PanelBuilder& builder)
        {
            std::vector<const physics::RigidBody*> bodies;
            for (const auto id : model.selected_bodies)
                if (const auto* body = model.world->find_body(id))
                    bodies.push_back(body);
            // The same header as one object's: a count in place of the name, then their kinds.
            builder.begin_group("object-name");
            builder.heading(core::substitute("{} objects", model.selected_bodies.size()));
            if (!bodies.empty())
            {
                std::string kinds;
                for (const auto& [type, name] : { std::pair { physics::BodyType::dynamic_body, "free" }, std::pair { physics::BodyType::kinematic_body, "driven" }, std::pair { physics::BodyType::static_body, "fixed" } })
                {
                    const auto count = std::count_if(bodies.begin(), bodies.end(), [type = type](const auto* body)
                        {
                            return body->type() == type;
                        });
                    if (count > 0)
                        kinds += (kinds.empty() ? "" : ", ") + core::substitute("{} {}", count, name);
                }
                const auto moving = std::any_of(bodies.begin(), bodies.end(), [](const auto* body)
                    {
                        return body->type() != physics::BodyType::static_body && body_moving(*body);
                    });
                builder.label(core::substitute("{} \xC2\xB7 {}", kinds, moving ? "moving" : "resting"));
            }
            builder.end_group();
            builder.action_row("Frame selection", request(UiCommandKind::frame_selection));
            builder.present_last(presentation(icons::crosshair, "Shift+F"));
            if (bodies.empty())
                return;
            const auto all_free = std::all_of(bodies.begin(), bodies.end(), [](const auto* body)
                {
                    return body->type() == physics::BodyType::dynamic_body;
                });
            const auto any_driven = std::any_of(bodies.begin(), bodies.end(), [](const auto* body)
                {
                    return body->type() == physics::BodyType::kinematic_body;
                });
            const auto shared_number = [&](auto read) -> std::optional<double>
            {
                const auto first = read(*bodies.front());
                for (const auto* body : bodies)
                {
                    const auto value = read(*body);
                    if (std::abs(value - first) > 1.0e-6 * std::max({ std::abs(value), std::abs(first), 1.0e-9 }))
                        return std::nullopt;
                }
                return first;
            };
            builder.paragraph(all_free ? "Changes apply to all selected objects. Mixed means they have different values for that property."
                                       : "To change mass or gravity scale, select only free objects. This selection includes fixed or driven objects.");
            if (all_free)
            {
                const auto mass = shared_number([](const physics::RigidBody& body)
                    {
                        return body.mass_properties().mass_kg;
                    });
                if (mass)
                    builder.number_row(spec("object.properties.mass"), *mass, request(UiCommandKind::set_selected_mass));
                else
                    builder.mixed_number_row(spec("object.properties.mass"), request(UiCommandKind::set_selected_mass));
            }
            if (!any_driven)
            {
                const auto material = material_id(*bodies.front());
                const auto same_material = std::all_of(bodies.begin(), bodies.end(), [&](const auto* body)
                    {
                        return material_id(*body) == material;
                    });
                if (same_material)
                    builder.select_row(spec("object.properties.material"), material, request(UiCommandKind::set_selected_material), core::humanise_identifier(material));
                else
                    builder.mixed_select_row(spec("object.properties.material"), request(UiCommandKind::set_selected_material));
            }
            if (all_free)
            {
                const auto scale = shared_number([](const physics::RigidBody& body)
                    {
                        return body.gravity_scale();
                    });
                if (scale)
                    builder.number_row(spec("object.properties.gravity_scale"), *scale, request(UiCommandKind::set_selected_gravity_scale));
                else
                    builder.mixed_number_row(spec("object.properties.gravity_scale"), request(UiCommandKind::set_selected_gravity_scale));
            }
            const auto any_moving = std::any_of(bodies.begin(), bodies.end(), [](const auto* body)
                {
                    return body->type() != physics::BodyType::static_body && body_moving(*body);
                });
            builder.action_row("Stop motion", request(UiCommandKind::stop_selected_motion), any_moving ? "" : "None of these objects is moving.");
            builder.present_last(presentation(icons::grab));
            builder.action_row("Combine", request(UiCommandKind::assemble_selected_bodies));
            builder.present_last(presentation(icons::shape, "Ctrl+G"));
            builder.action_row("Delete selected objects", request(UiCommandKind::delete_selected_body));
            builder.present_last(presentation(icons::remove, "Del").danger());
            builder.action_row("Clear selection", request(UiCommandKind::clear_selection));
            builder.present_last(presentation(icons::close, "Esc"));
        }

        void object_list(const UiModel& model, PanelBuilder& builder)
        {
            for (const auto& object : model.objects)
            {
                if (object.role == "marker")
                    continue;
                ListItemContent item { object.display_name, object.kind == "free" ? "Free" : object.kind == "driven" ? "Driven"
                                                                                                                     : "Fixed",
                    {},
                    {} };
                auto select = request(UiCommandKind::select_body, object.id);
                auto frame = request(UiCommandKind::frame_subject, object.id);
                auto extend = request(UiCommandKind::select_bodies);
                extend.bodies = model.selected_bodies;
                if (const auto selected = std::find(extend.bodies.begin(), extend.bodies.end(), object.id); selected == extend.bodies.end())
                    extend.bodies.push_back(object.id);
                else
                    extend.bodies.erase(selected);
                builder.list_item("object.list.row", object.document_id.empty() ? std::to_string(object.id.index) : object.document_id, item, select, frame, {}, extend);
                builder.select_last(std::find(model.selected_bodies.begin(), model.selected_bodies.end(), object.id) != model.selected_bodies.end());
            }
        }

        // Markers are reference lines, not objects: they are named but cannot be selected, so they
        // are listed as text. A caption every marker shares is said once rather than per marker.
        void marker_list(const UiModel& model, PanelBuilder& builder)
        {
            const ObjectItem* first = nullptr;
            bool shared = true;
            for (const auto& object : model.objects)
                if (object.role == "marker")
                {
                    if (!first)
                        first = &object;
                    else if (object.caption != first->caption)
                        shared = false;
                }
            if (!first)
                return;
            if (shared && !first->caption.empty())
                builder.paragraph(first->caption);
            builder.begin_group("markers");
            for (const auto& object : model.objects)
                if (object.role == "marker")
                {
                    builder.label(object.display_name);
                    if (!shared && !object.caption.empty())
                        builder.paragraph(object.caption);
                }
            builder.end_group();
        }
        void world_target(const UiModel& model, PanelBuilder& builder)
        {
            if (!model.world)
            {
                builder.paragraph("No experiment is open.");
                return;
            }
            const auto& settings = model.world->settings();
            const auto gravity_on = math::length_squared(settings.gravity_m_s2) > 0.0;
            if (builder.section("world.objects", "Objects", true))
            {
                object_list(model, builder);
                builder.action_row("Select all free objects", request(UiCommandKind::select_all));
                builder.present_last(presentation(icons::select_all, "Ctrl+A"));
                builder.action_row("Draw shape", request(UiCommandKind::start_new_shape));
                builder.present_last(presentation(icons::draw, "D"));
            }
            if (std::any_of(model.objects.begin(), model.objects.end(), [](const auto& value)
                    {
                        return value.role == "marker";
                    }) &&
                builder.section("world.markers", "Markers", false))
                marker_list(model, builder);
            if (builder.section("world.gravity", "Gravity", true))
            {
                builder.switch_row(spec("world.gravity.enabled"), gravity_on, flag(UiCommandKind::set_gravity_enabled, !gravity_on));
                builder.segmented_row(spec("world.gravity.preset"), gravity_preset_id(model), request(UiCommandKind::set_gravity_preset));
                builder.number_row(spec("world.gravity.strength"), math::length(settings.gravity_m_s2), request(UiCommandKind::set_gravity_magnitude));
                builder.number_row(spec("world.gravity.tilt"), model.gravity_direction_degrees + 90.0, request(UiCommandKind::set_gravity_angle_degrees));
                builder.number_row(spec("world.gravity.zero_height"), model.world->potential_energy_reference_height_m(), request(UiCommandKind::set_energy_reference_height));
            }
            if (builder.section("world.air", "Air", false))
            {
                builder.switch_row(spec("world.air.resistance"), model.drag_enabled, flag(UiCommandKind::set_drag_enabled, !model.drag_enabled));
                builder.switch_row(spec("world.air.spin_slowing"), model.angular_drag_enabled, flag(UiCommandKind::set_angular_drag_enabled, !model.angular_drag_enabled));
                builder.switch_row(spec("world.air.spin_lift"), model.magnus_enabled, flag(UiCommandKind::set_magnus_enabled, !model.magnus_enabled));
                auto wind_x = request(UiCommandKind::set_environment_parameter);
                wind_x.id = "air_velocity_x_m_s";
                auto wind_y = request(UiCommandKind::set_environment_parameter);
                wind_y.id = "air_velocity_y_m_s";
                auto density = request(UiCommandKind::set_environment_parameter);
                density.id = "air_density_kg_m3";
                auto viscosity = request(UiCommandKind::set_environment_parameter);
                viscosity.id = "air_dynamic_viscosity_pa_s";
                builder.number_row(spec("world.air.wind_x"), model.air_velocity_m_s.x, wind_x);
                builder.number_row(spec("world.air.wind_y"), model.air_velocity_m_s.y, wind_y);
                builder.number_row(spec("world.air.density"), settings.air_density_kg_m3, density);
                builder.number_row(spec("world.air.viscosity"), model.air_dynamic_viscosity_pa_s, viscosity);
            }
            if (builder.section("world.collisions", "Collisions", false))
            {
                const auto mixing = settings.collision.restitution_mixing == physics::MaterialMixing::maximum ? "maximum" : settings.collision.restitution_mixing == physics::MaterialMixing::minimum ? "minimum"
                    : settings.collision.restitution_mixing == physics::MaterialMixing::arithmetic_mean                                                                                               ? "arithmetic_mean"
                                                                                                                                                                                                      : "geometric_mean";
                builder.select_row(spec("world.collisions.bounce_rule"), mixing, request(UiCommandKind::set_restitution_mixing));
                builder.switch_row(spec("world.collisions.catch_fast"), settings.collision.continuous, flag(UiCommandKind::set_continuous_collision, !settings.collision.continuous));
                builder.action_row("Compare bounce rules", request(UiCommandKind::compare_restitution));
                builder.present_last(presentation(icons::table));
            }
            if (builder.section("world.advanced", "Advanced", false))
            {
                builder.select_row(spec("world.advanced.integration_method"), model.world->integrator().name(), request(UiCommandKind::set_integrator));
                builder.select_row(spec("world.advanced.time_step"), core::fixed(model.fixed_step_s, 6), request(UiCommandKind::set_fixed_step));
                builder.switch_row(spec("world.advanced.reuse_impulses"), settings.solver.warm_starting, flag(UiCommandKind::set_warm_starting, !settings.solver.warm_starting));
                builder.switch_row(spec("world.advanced.solve_joints_together"), settings.constraint_graph_enabled, flag(UiCommandKind::set_constraint_graph, !settings.constraint_graph_enabled));
                builder.action_row("Compare methods", request(UiCommandKind::compare_integrators));
                builder.present_last(presentation(icons::table));
            }
            if (builder.section("world.changes", "Changes", !model.changes.empty()))
            {
                if (model.changes.empty() && model.lab_changes.empty())
                    builder.paragraph("This setup matches the original.");
                for (const auto& change : model.changes)
                {
                    auto revert = request(UiCommandKind::revert_change);
                    revert.id = change.key;
                    builder.action_row(change.label + ": " + change.original_text + " → " + change.current_text, revert);
                    builder.present_last(presentation(icons::revert));
                }
                if (!model.lab_changes.empty())
                    builder.heading("Lab settings");
                for (const auto& change : model.lab_changes)
                    builder.value_row(change.label, change.current_text);
                if (!model.changes.empty())
                {
                    builder.action_row("Restore original", request(UiCommandKind::restore_original));
                    builder.present_last(presentation(icons::reset));
                }
                if (!model.lab_changes.empty())
                {
                    builder.action_row("Revert lab settings", request(UiCommandKind::revert_lab_settings));
                    builder.present_last(presentation(icons::revert));
                }
            }
            if (builder.section("world.statistics", "Statistics", false))
            {
                const auto& stats = model.world->statistics();
                builder.live_value_row("Objects", std::to_string(stats.body_count));
                builder.live_value_row("Contacts", std::to_string(stats.contact_point_count));
                builder.live_value_row("Sleeping", std::to_string(stats.sleeping_body_count));
            }
        }

        void connection_target(const UiModel& model, PanelBuilder& builder, const SelectedConnection& selected)
        {
            if (!model.world)
                return;
            builder.heading(selected.kind == "spring" ? "Spring" : "Joint");
            builder.value_row("Key", selected.key);
            if (selected.kind == "joint")
            {
                const auto joint = std::dynamic_pointer_cast<const physics::JointConstraint>(model.world->constraint_by_key(selected.key));
                if (!joint)
                {
                    builder.paragraph("This joint is no longer in the setup.");
                    return;
                }
                builder.value_row("State", joint->is_broken() ? "Broken" : "Connected");
                const auto first = model.world->find_body(joint->first_body());
                const auto second = model.world->find_body(joint->second_body());
                builder.value_row("Objects", core::substitute("{} ↔ {}", first ? body_name(model, joint->first_body(), *first) : "Missing", second ? body_name(model, joint->second_body(), *second) : "Missing"));
                const auto definition = joint->definition();
                std::visit([&](const auto& value)
                    {
                        using Definition = std::decay_t<decltype(value)>;
                        if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition> || std::is_same_v<Definition, physics::PrismaticJointDefinition>)
                        {
                            auto motor = request(UiCommandKind::set_joint_motor_enabled);
                            motor.id = selected.key;
                            motor.flag = !value.motor_enabled;
                            builder.switch_row(spec("joint.motor.enabled"), value.motor_enabled, motor);
                            auto speed = request(UiCommandKind::set_joint_motor_speed);
                            speed.id = selected.key;
                            if constexpr (std::is_same_v<Definition, physics::RevoluteJointDefinition>)
                                builder.number_row(spec("joint.motor.angular_speed"), value.motor_speed_rad_s, speed);
                            else
                                builder.number_row(spec("joint.motor.linear_speed"), value.motor_speed_m_s, speed);
                            auto reverse = request(UiCommandKind::reverse_joint_motor);
                            reverse.id = selected.key;
                            builder.action_row("Reverse motor", reverse);
                            builder.present_last(presentation(icons::replay));
                            auto limits = request(UiCommandKind::set_joint_limits_enabled);
                            limits.id = selected.key;
                            limits.flag = !value.limits_enabled;
                            builder.switch_row(spec("joint.stops.enabled"), value.limits_enabled, limits);
                        }
                    },
                    definition);
            }
            else
            {
                const auto spring = model.world->spring_by_key(selected.key);
                const auto* definition = model.world->spring_definition(spring);
                if (!definition)
                {
                    builder.paragraph("This spring is no longer in the setup.");
                    return;
                }
                std::visit([&](const auto& value)
                    {
                        using Definition = std::decay_t<decltype(value)>;
                        auto stiffness = request(UiCommandKind::set_spring_parameter);
                        stiffness.id = selected.key;
                        stiffness.detail = "stiffness";
                        auto damping = request(UiCommandKind::set_spring_parameter);
                        damping.id = selected.key;
                        damping.detail = "damping";
                        if constexpr (std::is_same_v<Definition, physics::LinearSpringDefinition>)
                        {
                            builder.number_row(spec("spring.linear.stiffness"), value.stiffness_n_m, stiffness);
                            builder.number_row(spec("spring.linear.damping"), value.damping_n_s_m, damping);
                            builder.live_value_row("Length", core::format_quantity(value.rest_length_m, core::DisplayQuantity::length, model.display_units));
                        }
                        else
                        {
                            builder.number_row(spec("spring.angular.stiffness"), value.stiffness_n_m_rad, stiffness);
                            builder.number_row(spec("spring.angular.damping"), value.damping_n_m_s_rad, damping);
                        }
                    },
                    *definition);
            }
            builder.action_row("Restore original", request(UiCommandKind::restore_original));
            builder.present_last(presentation(icons::reset));
            builder.action_row("Back to object", request(UiCommandKind::select_body, model.selection));
            builder.present_last(presentation(icons::previous_step));
        }
    }

    bool body_moving(const physics::RigidBody& body)
    {
        return math::length_squared(body.linear_velocity_m_s()) > 1.0e-6 || std::abs(body.angular_velocity_rad_s()) > 1.0e-3;
    }

    std::string_view InspectorPanel::id() const
    {
        return "inspector";
    }
    std::string_view InspectorPanel::title() const
    {
        return "Inspector";
    }
    RegionId InspectorPanel::region() const
    {
        return RegionId::inspector;
    }

    void InspectorPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title("Inspector");
        {
            UiCommand hide;
            hide.detail = "view:inspector";
            builder.action_row("view.inspector_collapse", "Hide Inspector", hide);
            builder.present_last(presentation(icons::expand_right, "I").icon_label_only().in_header());
        }
        if (model.selected_connection)
        {
            connection_target(model, builder, *model.selected_connection);
            return;
        }
        static constexpr OptionSpec targets[] { { "selection", "Selection", {}, {} }, { "world", "World", {}, {} } };
        const auto target = builder.tabs("inspector.target", targets, model.selection.is_valid() ? "selection" : "world");
        if (target == "world" || !model.selection.is_valid())
        {
            world_target(model, builder);
            return;
        }
        if (!model.world)
            return;
        if (model.selected_bodies.size() > 1)
        {
            several_objects(model, builder);
            return;
        }
        const auto* body = model.world->find_body(model.selection);
        if (!body)
            return;
        // Name, kind and motion form one header so the first property sits high in the panel.
        builder.begin_group("object-name");
        builder.heading(body_name(model, model.selection, *body));
        const auto kind = body->type() == physics::BodyType::dynamic_body ? "Free object" : body->type() == physics::BodyType::kinematic_body ? "Driven object"
                                                                                                                                              : "Fixed object";
        builder.label(core::substitute("{} \xC2\xB7 {}", kind, body_moving(*body) ? "moving" : "resting"));
        builder.end_group();
        builder.action_row("Frame selection", request(UiCommandKind::frame_selection));
        builder.present_last(presentation(icons::crosshair, "Shift+F"));
        static constexpr OptionSpec tabs[] { { "properties", "Body", {}, {} }, { "motion", "Motion", {}, {} }, { "forces", "Forces", {}, {} }, { "joints", "Joints", {}, {} }, { "shape", "Shape", {}, {} } };
        const auto tab = builder.tabs("inspector.object", tabs, "properties");
        const auto targeted = [&](UiCommandKind kind)
        {
            return request(kind, model.selection);
        };
        if (tab == "properties")
        {
            builder.number_row(spec("object.properties.mass"), body->mass_properties().mass_kg, targeted(UiCommandKind::set_selected_mass));
            builder.action_row("Use material density", targeted(UiCommandKind::use_selected_density_mass));
            builder.present_last(presentation(icons::mass));
            const auto material = material_id(*body);
            builder.select_row(spec("object.properties.material"), material, targeted(UiCommandKind::set_selected_material), core::humanise_identifier(material));
            builder.number_row(spec("object.properties.gravity_scale"), body->gravity_scale(), targeted(UiCommandKind::set_selected_gravity_scale));
            builder.live_value_row("Inertia", core::format_quantity(body->mass_properties().inertia_kg_m2, core::DisplayQuantity::inertia, model.display_units));
            builder.present_last(presentation(icons::measure));
            if (!body->colliders().empty())
            {
                // The coefficients come with the material, so they are listed under its name as
                // figures. Bars would put coefficients with different ranges on one hidden scale.
                const auto& material_data = body->colliders().front().material;
                const auto& material_spec = spec("object.properties.material");
                const auto listed = std::find_if(material_spec.options.begin(), material_spec.options.end(), [&](const OptionSpec& option)
                    {
                        return option.id == material;
                    });
                const auto material_name = listed != material_spec.options.end() ? std::string(listed->label) : core::humanise_identifier(material);
                builder.heading(material_name.empty() ? std::string { "Material properties" } : core::substitute("Material properties ({})", material_name));
                builder.value_row("Bounciness", core::fixed(material_data.restitution, 2));
                builder.value_row("Grip at rest", core::fixed(material_data.static_friction, 2));
                builder.value_row("Grip while sliding", core::fixed(material_data.kinetic_friction, 2));
                builder.value_row("Drag coefficient", core::fixed(body->colliders().front().effective_drag_coefficient(), 2));
            }
        }
        else if (tab == "motion")
        {
            auto x = targeted(UiCommandKind::set_selected_position);
            x.detail = "x";
            x.value_y = body->position_m().y;
            auto y = targeted(UiCommandKind::set_selected_position);
            y.detail = "y";
            y.value_y = body->position_m().x;
            builder.number_row(spec("object.motion.position_x"), body->position_m().x, x);
            builder.number_row(spec("object.motion.position_y"), body->position_m().y, y);
            builder.number_row(spec("object.motion.orientation"), math::radians_to_degrees(body->orientation_rad()), targeted(UiCommandKind::set_selected_orientation));
            builder.number_row(spec("object.motion.velocity_x"), body->linear_velocity_m_s().x, targeted(UiCommandKind::set_selected_velocity_x));
            builder.number_row(spec("object.motion.velocity_y"), body->linear_velocity_m_s().y, targeted(UiCommandKind::set_selected_velocity_y));
            builder.number_row(spec("object.motion.spin"), body->angular_velocity_rad_s(), targeted(UiCommandKind::set_selected_angular_velocity));
            auto reverse = targeted(UiCommandKind::set_selected_angular_velocity);
            reverse.value = -body->angular_velocity_rad_s();
            builder.action_row("object.motion.reverse_spin", "Reverse spin", reverse, body->angular_velocity_rad_s() == 0.0 ? "Not spinning." : "");
            builder.present_last(presentation(icons::replay));
            builder.action_row("Stop motion", targeted(UiCommandKind::stop_selected_motion), body->type() != physics::BodyType::static_body && body_moving(*body) ? "" : "Already at rest.");
            builder.present_last(presentation(icons::grab));
            builder.live_value_row("Speed", core::format_quantity(math::length(body->linear_velocity_m_s()), core::DisplayQuantity::velocity, model.display_units));
            builder.present_last(presentation(icons::measure));
        }
        else if (tab == "forces")
        {
            builder.live_value_row("Net force", core::format_quantity(math::length(body->applied_force_n()), core::DisplayQuantity::force, model.display_units));
            builder.present_last(presentation(icons::measure));
            for (const auto& channel : body->applied_force_channels())
                builder.live_value_row(channel.name.empty() ? "Force" : channel.name,
                    core::format_quantity(math::length(channel.force_n), core::DisplayQuantity::force, model.display_units));
        }
        else if (tab == "joints")
        {
            bool any = false;
            for (const auto& constraint : model.world->constraints())
                if (const auto joint = std::dynamic_pointer_cast<const physics::JointConstraint>(constraint); joint && (joint->first_body() == model.selection || joint->second_body() == model.selection))
                {
                    any = true;
                    const auto key = model.world->constraint_key(constraint);
                    ListItemContent item { key.empty() ? "Joint" : key, joint->is_broken() ? "Broken" : "Connected", {}, {} };
                    auto select = request(UiCommandKind::select_connection);
                    select.id = key;
                    select.detail = "joint";
                    builder.list_item("joint.card", key, item, select);
                }
            for (const auto spring_id : model.world->spring_ids())
                if (const auto* spring = model.world->spring_definition(spring_id); spring)
                    std::visit([&](const auto& value)
                        {
                            if (value.first == model.selection || value.second == model.selection)
                            {
                                any = true;
                                const auto key = std::string(model.world->spring_key(spring_id));
                                ListItemContent item { key.empty() ? "Spring" : key, "Connected", {}, {} };
                                auto select = request(UiCommandKind::select_connection);
                                select.id = key;
                                select.detail = "spring";
                                builder.list_item("spring.card", key.empty() ? std::to_string(spring_id.index) : key, item, select);
                            }
                        },
                        *spring);
            if (!any)
                builder.paragraph("No joints or springs connect this object.");
        }
        else
        {
            for (std::size_t index = 0; index < model.authored_part_count; ++index)
            {
                const auto label = index == model.authored_part_index && !model.authored_part_name.empty()
                    ? model.authored_part_name
                    : "Part " + std::to_string(index + 1);
                ListItemContent item { label, index == model.authored_part_index ? "Selected" : "", {}, {} };
                auto select = targeted(UiCommandKind::select_authored_part);
                select.value = static_cast<double>(index);
                builder.list_item("object.shape.parts", std::to_string(index), item, select);
            }
            builder.action_row("Edit shape", targeted(UiCommandKind::edit_selected_shape));
            builder.present_last(presentation(icons::draw));
            builder.action_row("Separate parts", targeted(UiCommandKind::split_selected_body));
            builder.present_last(presentation(icons::shape, "Ctrl+Shift+G"));
            builder.action_row("Export shape…", targeted(UiCommandKind::export_shape));
            builder.present_last(presentation(icons::export_file));
        }
        builder.separator();
        builder.action_row("Keep as starting state", request(UiCommandKind::keep_state_as_setup), model.elapsed_time_s <= 0.0 ? "Already at the starting setup." : "");
        builder.present_last(presentation(icons::pin));
        builder.action_row("Delete", targeted(UiCommandKind::delete_selected_body));
        builder.present_last(presentation(icons::remove, "Del").danger());
        builder.action_row("Clear selection", request(UiCommandKind::clear_selection));
        builder.present_last(presentation(icons::close, "Esc"));
    }
}
