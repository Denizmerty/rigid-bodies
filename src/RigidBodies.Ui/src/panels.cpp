#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/core/display_units.hpp>

#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/project_identity.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/physics/joint.hpp>

#include <cmath>
#include <algorithm>
#include <map>
#include <tuple>
#include <cctype>
#include <type_traits>
#include <stdexcept>
#include <filesystem>

namespace rigidbodies::ui
{
    namespace
    {

        UiCommand command(UiCommandKind kind)
        {
            UiCommand result;
            result.kind = kind;
            return result;
        }

        UiCommand command(UiCommandKind kind, std::string_view id)
        {
            UiCommand result;
            result.kind = kind;
            result.id = std::string { id };
            return result;
        }

        UiCommand command(UiCommandKind kind, double value)
        {
            UiCommand result;
            result.kind = kind;
            result.value = value;
            return result;
        }

        UiCommand flag_command(UiCommandKind kind, bool enabled)
        {
            auto result = command(kind);
            result.flag = enabled;
            return result;
        }

        const ControlSpec& spec(std::string_view key)
        {
            const auto* result = find_control_spec(key);
            if (!result)
                throw std::logic_error("missing control specification: " + std::string(key));
            return *result;
        }

        std::string quantity(double value, core::DisplayQuantity kind, const UiModel& model, int precision = -1)
        {
            return core::format_quantity(value, kind, model.display_units, precision);
        }

        std::string vector_quantity(const math::Vec2& value, core::DisplayQuantity kind, const UiModel& model)
        {
            return core::substitute("{}, {}\xC2\xA0{}", core::format_value(value.x, kind, model.display_units), core::format_value(value.y, kind, model.display_units), core::display_unit(kind, model.display_units));
        }

        std::string playback_speed_label(double value)
        {
            auto text = core::fixed(value, 6);
            while (!text.empty() && text.back() == '0')
                text.pop_back();
            if (!text.empty() && text.back() == '.')
                text.pop_back();
            return text + "×";
        }

    } // namespace

    std::string_view SimulationControlsPanel::id() const
    {
        return "simulation_controls";
    }

    std::string_view SimulationControlsPanel::title() const
    {
        return "Simulation";
    }

    RegionId SimulationControlsPanel::region() const
    {
        return RegionId::command_bar;
    }

    void SimulationControlsPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        builder.action_row(model.paused ? "Play" : "Pause", command(UiCommandKind::toggle_pause));
        builder.action_row("Step", command(UiCommandKind::single_step));
        builder.action_row("Back to start", command(UiCommandKind::reset_scenario));
        builder.select_row(spec("bar.speed.choice"), core::fixed(model.time_scale, 6), command(UiCommandKind::set_time_scale), playback_speed_label(model.time_scale));
        if (model.world != nullptr)
            builder.select_row(spec("world.advanced.integration_method"), model.world->integrator().name(), command(UiCommandKind::set_integrator));
        builder.select_row(spec("world.advanced.time_step"), core::fixed(model.fixed_step_s, 6), command(UiCommandKind::set_fixed_step));
        builder.number_row(spec("world.advanced.time_step_custom"), model.fixed_step_s, command(UiCommandKind::set_fixed_step));
        builder.number_row(spec("camera.scale.height"), model.view_height_m, command(UiCommandKind::set_view_height));
        builder.action_row("Frame subject", command(UiCommandKind::frame_subject));
        builder.action_row("Frame everything", command(UiCommandKind::frame_all));
        builder.action_row("Frame selection", command(UiCommandKind::frame_selection), model.selected_bodies.empty() ? "Select an object first." : "");
        {
            auto follow = command(UiCommandKind::set_preference, "view.follow_selection");
            follow.detail = "view.follow_selection";
            follow.flag = !model.follow_selection;
            builder.toggle_row("menu.view.follow_selection", "Follow selection", model.follow_selection, follow);
        }
        builder.live_value_row("Elapsed", now_text(model));
        if (model.discarded_time_s > 0.0)
        {
            builder.live_value_row("Time skipped", elapsed_text(model, model.discarded_time_s));
        }
        builder.value_row("Time step", core::substitute("{}\xC2\xA0ms", core::format_value(model.fixed_step_s * 1000.0, core::DisplayQuantity::coefficient, model.display_units)));
        if (model.world != nullptr)
        {
            const auto& statistics = model.world->statistics();
            builder.value_row("Sleeping", std::to_string(statistics.sleeping_body_count));
            if (statistics.limit_event_count > 0)
            {
                builder.value_row("Limit warnings", std::to_string(statistics.limit_event_count), Color { 1.0f, 0.64f, 0.34f, 1.0f });
            }
            if (statistics.last_step_limit_event_count > 0)
            {
                builder.label("Numeric limit reached.");
            }
        }
    }

    std::string_view ScenarioPanel::id() const
    {
        return "scenarios";
    }

    std::string_view ScenarioPanel::title() const
    {
        return "Scenarios";
    }

    RegionId ScenarioPanel::region() const
    {
        return RegionId::library_sheet;
    }

    void ScenarioPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        builder.action_row("Save arrangement...", command(UiCommandKind::save_arrangement));
        builder.action_row("Open arrangement...", command(UiCommandKind::open_arrangement));
        builder.action_row("Import shape...", command(UiCommandKind::import_shape));
        if (model.can_export_shape)
            builder.action_row("Export authored part...", command(UiCommandKind::export_shape));
        builder.separator();

        const auto scenarios = physics::available_scenarios();
        if (scenarios.empty())
        {
            builder.label("No scenarios available.");
            return;
        }
        for (std::size_t index = 0; index < scenarios.size(); ++index)
        {
            const auto& description = scenarios[index];
            const auto selected = description.id == model.scenario_id;
            builder.choice_row(description.title, selected, command(UiCommandKind::load_scenario, description.id));
        }
        builder.separator();
        if (!model.scenario_title.empty())
            builder.heading(model.scenario_title);
        if (model.scenario_suggested_order > 0)
            builder.value_row("Suggested order", std::to_string(model.scenario_suggested_order));
        if (!model.scenario_concepts.empty())
        {
            builder.label("Concepts");
            for (const auto& concept : model.scenario_concepts)
                builder.paragraph(concept);
        }
        if (!model.scenario_prerequisites.empty())
        {
            builder.label("Try first");
            for (const auto& prerequisite : model.scenario_prerequisites)
            {
                const auto* description = physics::find_scenario(prerequisite);
                if (description)
                    builder.action_row(description->title, command(UiCommandKind::load_scenario, prerequisite));
                else
                    builder.paragraph(prerequisite);
            }
        }
    }

    std::string_view VisualizationPanel::id() const
    {
        return "visualization";
    }

    std::string_view VisualizationPanel::title() const
    {
        return "Show";
    }

    RegionId VisualizationPanel::region() const
    {
        return RegionId::show_popover;
    }

    void VisualizationPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        // An untouched experiment shows its recommended view, so that preset reads as chosen.
        const auto selected_preset = model.layers.bits() == model.recommended_layers.bits() ? "recommended" : model.layers.bits() == render::LayerMask::all().bits() ? "all"
            : model.layers.bits() == render::LayerMask::none().bits()                                                                                                ? "none"
                                                                                                                                                                     : "custom";
        // Custom carries the current mix, so choosing it keeps the layers exactly as they are.
        builder.segmented_row(spec("show.presets.preset"), selected_preset, command(UiCommandKind::set_layer_mask, static_cast<double>(model.layers.bits())));
        // Layers are grouped by what they explain; the solver's own diagnostics stay folded away.
        // A relativity experiment draws its own stage, so only the grid, the rail and its labels
        // remain to show or hide.
        const auto relativity = model.relativity.has_value();
        const auto layer_switches = [&](std::string_view location)
        {
            for (const auto& description : render::layer_catalogue())
            {
                if (description.id == "selection")
                    continue;
                const auto& layer_spec = spec("show.layer." + std::string(description.id));
                if (layer_spec.location != location)
                    continue;
                auto set = command(UiCommandKind::set_layer, description.id);
                set.flag = !model.layers.is_enabled(description.layer);
                builder.switch_row(layer_spec, model.layers.is_enabled(description.layer), set);
                if (relativity && description.id != "grid" && description.id != "bodies" && description.id != "labels")
                    builder.disable_last("Not used in relativity experiments.");
            }
        };
        for (const auto& [heading, location] : std::array<std::pair<std::string_view, std::string_view>, 4> { { { "Objects", "Show › Objects" }, { "Motion", "Show › Motion" }, { "Forces and contact", "Show › Forces and contact" }, { "Reference", "Show › Reference" } } })
        {
            builder.heading(heading);
            layer_switches(location);
        }
        if (builder.section("show.internals", "Engine internals", false))
            layer_switches("Show › Engine internals");
        if (!relativity && builder.section("show.arrows", "Arrows", true))
        {
            builder.segmented_row(spec("show.arrows.scope"), model.visual_settings.vectors_selected_only ? "selected" : "all", command(UiCommandKind::set_vector_scope));
            builder.switch_row(spec("show.arrows.auto_length"), model.vector_scales.automatic, flag_command(UiCommandKind::set_vector_auto_scale, !model.vector_scales.automatic));
            // Manual scales matter only while arrow lengths are not chosen automatically.
            if (!model.vector_scales.automatic)
                for (const auto& [id, label, value] : std::array<std::tuple<std::string_view, std::string_view, double>, 4> { { { "velocity", "Velocity arrows", model.vector_scales.velocity }, { "acceleration", "Acceleration arrows", model.vector_scales.acceleration }, { "force", "Force arrows", model.vector_scales.force }, { "momentum", "Momentum arrows", model.vector_scales.momentum } } })
                {
                    builder.number_row(spec("show.arrows.scale"), value, command(UiCommandKind::set_vector_scale, id), id);
                    builder.label_last(label);
                }
            // Force arrows are refitted to the scene's weights so a light ball and a heavy crate
            // both read; say so rather than leave the requested factor looking ignored.
            const auto drawn = model.drawn_force_scale;
            if (!model.vector_scales.automatic && model.layers.is_enabled(render::VisualizationLayer::force_vectors) && std::isfinite(drawn) && drawn > 0.0 && std::abs(drawn - model.vector_scales.force) > 0.01 * model.vector_scales.force)
                builder.paragraph("Force arrows share a scale based on the objects' weights. The key beside the scale bar shows the force represented by its sample arrow.");
            const auto split = model.vector_components == render::VectorComponents::world_axes ? "world" : model.vector_components == render::VectorComponents::custom_axes ? "chosen"
                : model.vector_components == render::VectorComponents::contact_axes                                                                                         ? "contact"
                                                                                                                                                                            : "none";
            // The split modes have long names, so they stack as a list rather than share one line.
            builder.radio_list_row(spec("show.arrows.split"), split, command(UiCommandKind::set_vector_components));
            if (model.vector_components == render::VectorComponents::custom_axes)
            {
                builder.number_row(spec("show.arrows.direction"), math::radians_to_degrees(model.component_angle_rad), command(UiCommandKind::set_component_angle_degrees));
                auto pick_surface = command(UiCommandKind::none);
                pick_surface.detail = "view:pick_surface";
                builder.action_row("show.arrows.pick", "Pick surface", pick_surface);
                builder.present_last(presentation(icons::crosshair));
            }
        }
        if (builder.section("show.view", "View", true))
        {
            builder.number_row(spec("camera.scale.height"), model.view_height_m, command(UiCommandKind::set_view_height));
            builder.action_row("Frame selection", command(UiCommandKind::frame_selection), model.selected_bodies.empty() ? "Select an object first." : "");
            builder.present_last(presentation(icons::crosshair, "Shift+F"));
            builder.segmented_row(spec("prefs.units.system"), model.display_units == core::DisplayUnits::si ? "si" : "centimetre_gram", command(UiCommandKind::set_display_units));
        }
    }

    std::string_view PerformancePanel::id() const
    {
        return "performance";
    }

    std::string_view PerformancePanel::title() const
    {
        return "Performance";
    }

    RegionId PerformancePanel::region() const
    {
        return RegionId::performance_overlay;
    }

    void PerformancePanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!builder.view_surface_open("performance.overlay", false))
            return;
        builder.readout("stage.performance.frame_time", "Frame", core::substitute("{}\xC2\xA0ms", core::format_value(model.performance.frame_time_s * 1000.0, core::DisplayQuantity::coefficient, model.display_units)), RowTone::normal, true);
        builder.readout("stage.performance.fps", "FPS", core::fixed(model.performance.frames_per_second, 0), RowTone::normal, true);
        builder.readout("stage.performance.substeps", "Substeps", std::to_string(model.performance.substeps), RowTone::normal, true);
        builder.readout("stage.performance.renderer", "Renderer", model.performance.renderer, RowTone::normal, true);
        builder.readout("stage.performance.backend", "Interface", model.performance.interface_backend, RowTone::normal, true);
        builder.readout("stage.performance.pointer", "Pointer", vector_quantity(model.performance.pointer_world_m, core::DisplayQuantity::length, model), RowTone::normal, true);
        builder.readout("stage.performance.view_height", "View height", quantity(model.performance.view_height_m, core::DisplayQuantity::length, model), RowTone::normal, true);
    }

    std::string_view InteractionPanel::id() const
    {
        return "interaction";
    }
    std::string_view InteractionPanel::title() const
    {
        return "Tools & appearance";
    }
    RegionId InteractionPanel::region() const
    {
        return RegionId::command_bar;
    }
    void InteractionPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        builder.value_row("Selected bodies", std::to_string(model.selected_bodies.size()));
        builder.segmented_row(spec("tools.mode"), model.interaction_mode, command(UiCommandKind::set_interaction_mode));
        builder.paragraph("Shift-click adds or removes a body. Drag in Move or Throw mode; Pull applies a spring force while running. Esc cancels the gesture.");
        if (model.can_undo)
            builder.action_row("Undo: " + model.undo_label + " (Ctrl+Z)", command(UiCommandKind::undo));
        else
            builder.label("Undo (Ctrl+Z): no edits yet");
        if (model.can_redo)
            builder.action_row("Redo: " + model.redo_label + " (Ctrl+Shift+Z)", command(UiCommandKind::redo));
        builder.segmented_row(spec("prefs.appearance.theme"), model.theme_id, command(UiCommandKind::set_theme));
        builder.number_row(spec("prefs.appearance.text_size"), model.ui_scale, command(UiCommandKind::set_ui_scale));
        builder.number_row(spec("prefs.controls.zoom_speed"), model.camera_zoom_sensitivity, command(UiCommandKind::set_camera_zoom_sensitivity));
        if (builder.section("tools.reference", "Keyboard reference", false))
        {
            for (const auto& binding : model.keyboard_reference)
                builder.value_row(binding.chord, binding.description);
            builder.paragraph("Middle- or right-drag pans. Wheel zooms at the pointer. Tab and Shift+Tab move between controls; Enter activates them. Space always plays or pauses.");
        }
        if (model.selected_bodies.size() > 1)
            builder.paragraph("Changes apply to all selected bodies. If any body cannot accept a change, none are changed. Undo and redo pause the simulation so you can inspect the result.");
        builder.action_row("Quit", command(UiCommandKind::quit));
    }

    std::string_view RenderingPanel::id() const
    {
        return "rendering";
    }
    std::string_view RenderingPanel::title() const
    {
        return "Appearance & capture";
    }
    RegionId RenderingPanel::region() const
    {
        return RegionId::modal;
    }
    void RenderingPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        const auto& visual = model.visual_settings;
        const auto toggle = [&](const char* label, const char* id, bool enabled)
        {
            auto request = command(UiCommandKind::set_visual_effect, id);
            request.flag = !enabled;
            builder.toggle_row(label, enabled, request);
        };
        toggle("Material shading", "material_shading", visual.material_shading);
        toggle("Contact shadows", "contact_shadows", visual.contact_shadows);
        toggle("Background depth", "depth_background", visual.depth_background);
        toggle("Motion trails", "motion_trails", visual.motion_trails);
        toggle("Directional blur", "directional_blur", visual.directional_blur);
        toggle("Impact flashes", "impact_flashes", visual.impact_flashes);
        toggle("Impact sparks", "impact_sparks", visual.impact_sparks);
        toggle("Impact dust", "impact_dust", visual.impact_dust);
        toggle("Soft impact cues", "soft_deformation", visual.soft_deformation);
        toggle("Smooth transitions", "transitions", visual.transitions);
        builder.heading("Effect limits");
        const auto budget = [&](const char* label, const char* id, std::size_t value, std::size_t maximum)
        {
            auto request = command(UiCommandKind::set_visual_budget, id);
            request.value = static_cast<double>(value >= maximum ? 8 : std::min(maximum, std::max<std::size_t>(8, value * 2)));
            builder.action_row(core::substitute("{}: {} (change)", label, value), request);
        };
        budget("Trail bodies", "motion_body_budget", visual.motion_body_budget, 128);
        budget("Blur bodies", "directional_blur_budget", visual.directional_blur_budget, 128);
        budget("Shaded bodies", "shading_body_budget", visual.shading_body_budget, 4096);
        budget("Shadow contacts", "contact_shadow_budget", visual.contact_shadow_budget, 192);
        budget("Flashes", "impact_flash_budget", visual.impact_flash_budget, 48);
        budget("Sparks", "spark_budget", visual.spark_budget, 256);
        budget("Dust particles", "dust_budget", visual.dust_budget, 128);
        budget("Soft cues", "deformation_budget", visual.deformation_budget, 64);
        builder.separator();
        builder.action_row("Save scene image", command(UiCommandKind::export_still));
        builder.action_row(model.capturing_frames ? "Stop image sequence" : "Record image sequence", command(UiCommandKind::toggle_frame_capture));
        builder.paragraph("PNG files go to captures in your RigidBodies data folder. Sequences stop at 120 images or 240 MiB and record at most 10 per second.");
    }

    std::string_view CommandBarPanel::id() const
    {
        return "command_bar";
    }
    std::string_view CommandBarPanel::title() const
    {
        return "Command bar";
    }
    RegionId CommandBarPanel::region() const
    {
        return RegionId::command_bar;
    }
    void CommandBarPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto view = [&](std::string_view name)
        {
            auto result = command(UiCommandKind::none);
            result.detail = "view:" + std::string(name);
            return result;
        };

        // Shedding order when the window narrows: passive text before the labels of primary
        // actions. Library and Show labels, the run-state chip (Play/Pause already shows it), the
        // Guide/Inspector/Measure labels; then the title gives up width, shortened with an
        // ellipsis, before the tool labels and the Add label go; the title and the speed leave
        // together, then the Play label; then Undo/Redo, Library and Show, and finally the panel
        // toggles move into the menu. Transport, tools and the menu itself never leave the bar.
        builder.begin_group("identity");
        builder.title(model.scenario_title.empty() ? "Rigid Bodies" : model.scenario_title);
        builder.present_last(presentation().shrink_at(4).overflow_at(7));
        std::string state = model.run_state == RunState::ready ? "Ready" : model.run_state == RunState::running ? "Running"
            : model.run_state == RunState::held                                                                 ? "Held"
                                                                                                                : "Paused";
        auto tone = model.run_state == RunState::running ? RowTone::positive : model.run_state == RunState::ready ? RowTone::info
                                                                                                                  : RowTone::normal;
        if (model.run_state == RunState::paused)
        {
            switch (model.pause_reason.reason)
            {
            case PauseReason::drawing:
                state += " · drawing";
                break;
            case PauseReason::new_object:
                state += " · new object";
                break;
            case PauseReason::impact:
                state += " · impact " + std::to_string(model.pause_reason.impact_number);
                tone = RowTone::warning;
                break;
            case PauseReason::after_undo:
                state += " · after undo";
                break;
            case PauseReason::error:
                state += " · error";
                tone = RowTone::danger;
                break;
            case PauseReason::in_background:
                state += " · in background";
                break;
            case PauseReason::finished:
                state += " · finished";
                break;
            default:
                break;
            }
        }
        else if (model.run_state == RunState::held)
            tone = RowTone::warning;
        builder.readout("bar.state", "State", state, tone, true);
        builder.present_last(presentation().overflow_at(2));
        builder.end_group();

        builder.begin_group("transport");
        auto back = command(UiCommandKind::reset_scenario);
        back.flag = false;
        builder.action_row("Back to start", back, model.elapsed_time_s <= 0.0 ? "Already at the starting setup." : "");
        builder.present_last(presentation(icons::back_to_start, "R").icon_label_only());
        const auto running = model.run_state == RunState::running;
        builder.action_row(running ? "Pause" : "Play", command(UiCommandKind::toggle_pause));
        builder.present_last(presentation(running ? icons::pause : icons::play, "Space").primary().hide_label_at(8));
        builder.action_row("Step", command(UiCommandKind::single_step));
        builder.present_last(presentation(icons::step, ".").icon_label_only());
        builder.select_row(spec("bar.speed.choice"), core::fixed(model.time_scale, 6), command(UiCommandKind::set_time_scale), playback_speed_label(model.time_scale));
        builder.present_last(presentation(icons::speed).overflow_at(7));
        auto many = command(UiCommandKind::step_many);
        many.value = 10.0;
        builder.action_row("Step 10 frames", many);
        builder.present_last(presentation(icons::step_many, "Shift+.").always_overflow());
        auto replay = command(UiCommandKind::reset_scenario);
        replay.flag = true;
        replay.id = "replay";
        builder.action_row("Replay from start", replay);
        builder.present_last(presentation(icons::replay, "Shift+R").always_overflow());
        auto pause_next = command(UiCommandKind::pause_at_next_impact);
        pause_next.flag = !model.next_impact_armed;
        builder.action_row(model.next_impact_armed ? "Cancel play until next impact" : "Play until next impact", pause_next, model.relativity ? "Nothing collides in this experiment." : "");
        builder.present_last(presentation(icons::impact, "Shift+Space").always_overflow());
        if (model.next_impact_armed)
        {
            builder.readout("bar.next_impact", "Pause", "Next impact", RowTone::warning);
            builder.present_last(presentation(icons::impact).overflow_at(6));
        }
        builder.action_row("Keep as starting state", command(UiCommandKind::keep_state_as_setup), model.elapsed_time_s <= 0.0 ? "Already at the starting setup." : "");
        builder.present_last(presentation(icons::pin).always_overflow());
        builder.end_group();

        builder.begin_group("tools");
        builder.segmented_row(spec("tools.mode"), model.interaction_mode, command(UiCommandKind::set_interaction_mode));
        builder.present_last(presentation().hide_label_at(5));
        if (model.relativity)
            builder.disable_last("Throw and Pull act on Newtonian objects. Change the probe's speed instead.");
        builder.end_group();

        builder.begin_group("history");
        // History labels read as sentences: "Undo change scenario".
        const auto history_label = [](std::string_view verb, const std::string& label)
        {
            if (label.empty())
                return std::string(verb);
            auto text = std::string(verb) + " " + label;
            if (std::isupper(static_cast<unsigned char>(text[verb.size() + 1])) && !(label.size() > 1 && std::isupper(static_cast<unsigned char>(label[1]))))
                text[verb.size() + 1] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[verb.size() + 1])));
            return text;
        };
        builder.action_row(model.can_undo ? history_label("Undo", model.undo_label) : "Undo", command(UiCommandKind::undo), model.can_undo ? "" : "Nothing to undo.");
        builder.present_last(presentation(icons::undo, "Ctrl+Z").icon_label_only().overflow_at(9));
        builder.action_row(model.can_redo ? history_label("Redo", model.redo_label) : "Redo", command(UiCommandKind::redo), model.can_redo ? "" : "Nothing to redo.");
        builder.present_last(presentation(icons::redo, "Ctrl+Shift+Z").icon_label_only().overflow_at(9));
        builder.end_group();

        builder.spacer(0.0);

        builder.begin_group("workspace");
        builder.action_row("Add", view("add"), model.relativity ? "Objects cannot be added to a relativity experiment." : "");
        builder.present_last(presentation(icons::add, "Shift+A").hide_label_at(6).overflow_at(13));
        builder.select_last(builder.view_transient_open("add_menu"));
        builder.action_row("Library", view("library"));
        builder.present_last(presentation(icons::library, "L").hide_label_at(1).overflow_at(10));
        builder.select_last(builder.view_sheet_open("library"));
        // Panel toggles read pressed while their panel can be seen; the Library sheet slides over
        // whichever docked panel it covers.
        const auto has_guide = model.scenario_content && !model.scenario_content->guide.empty();
        builder.action_row("Guide", view("guide"), has_guide ? "" : "This experiment has no guide.");
        builder.present_last(presentation(icons::guide, "G").hide_label_at(3).overflow_at(12));
        builder.select_last(has_guide && builder.view_region_present(RegionId::guide_panel) && !builder.view_region_under_library(RegionId::guide_panel));
        builder.action_row("Inspector", view("inspector"));
        builder.present_last(presentation(icons::sidebar_right, "I").hide_label_at(3).overflow_at(12));
        builder.select_last(builder.view_region_present(RegionId::inspector) && !builder.view_region_under_library(RegionId::inspector));
        builder.action_row("Show", view("show"));
        builder.present_last(presentation(icons::show, "S").hide_label_at(1).overflow_at(10));
        builder.select_last(builder.view_transient_open("show"));
        const auto impacts = listed_impact_count(model, builder.view_value("measure.collisions.filter", "all"));
        const auto impact_badge = impacts == 0 ? std::string {} : std::to_string(std::min<std::size_t>(99, impacts)) + (impacts > 99 ? "+" : "");
        builder.action_row("Measure", view("measure"));
        builder.present_last(presentation(icons::measure, "M").with_badge(impact_badge, core::substitute("{} {} recorded", impact_badge, impacts == 1 ? "impact" : "impacts")).hide_label_at(3).overflow_at(13));
        builder.select_last(builder.view_surface_open("measure.open", false));
        builder.end_group();

        builder.begin_group("menu");
        builder.action_row("Menu", view("menu"));
        builder.present_last(presentation(icons::menu, "F10").icon_label_only());
        builder.select_last(builder.view_transient_open("main_menu"));
        builder.end_group();
    }

    std::string_view StatusLinePanel::id() const
    {
        return "status_line";
    }
    std::string_view StatusLinePanel::title() const
    {
        return "Status";
    }
    RegionId StatusLinePanel::region() const
    {
        return RegionId::status_line;
    }
    void StatusLinePanel::build(const UiModel& model, PanelBuilder& builder)
    {
        // The status line answers what is happening and what is selected. Persistent settings
        // (units, view height) live in Show and Preferences rather than becoming a second toolbar.
        builder.begin_group("status");
        if (builder.view_pick_surface_armed())
        {
            builder.readout("status.pick_surface", "Pick", "Click a surface to set the direction", RowTone::info);
            builder.present_last(presentation(icons::crosshair));
        }
        // A relativity experiment's clock is the lab's, in nanoseconds of lab time; the line shows
        // no names, so the reading carries its own, as the view height does.
        builder.readout("bar.time.readout", model.relativity ? "Lab time" : "Time", model.relativity ? "Lab time " + now_text(model) : now_text(model), RowTone::normal, true);
        builder.present_last(presentation(icons::timer));
        if (!model.selected_bodies.empty())
        {
            // The count names itself: a bare "6 objects" reads as the size of the scene. Where the
            // Inspector cannot sit beside the stage (a docked Guide in a medium window, or a narrow
            // window), the count also opens it, so a selection always leads to its properties.
            const auto selected = core::substitute("{} selected", model.selected_bodies.size());
            const auto inspector_hidden = builder.view_region_present(RegionId::stage) && !builder.view_region_present(RegionId::inspector) && !builder.view_region_present(RegionId::inspector_rail);
            if (!inspector_hidden)
            {
                builder.readout("status.selection", "Selected", selected, RowTone::normal);
                builder.present_last(presentation(icons::tool_select).overflow_at(2));
            }
            else
            {
                auto inspect = command(UiCommandKind::none);
                inspect.detail = "view:inspector";
                builder.action_row("status.selection_inspect", selected + " \xC2\xB7 Inspect", inspect);
                builder.present_last(presentation(icons::sidebar_right, "I").overflow_at(2));
            }
        }
        for (const auto& chip : model.status_chips)
        {
            builder.action_row(chip.text, chip.action.value_or(command(UiCommandKind::none)));
            builder.present_last(presentation(chip.severity == Severity::error ? icons::error : chip.severity == Severity::warning ? icons::warning
                    : chip.severity == Severity::success                                                                           ? icons::success
                                                                                                                                   : icons::info)
                    .overflow_at(3));
        }
        if (!model.lab_changes.empty() && !model.relativity)
        {
            const auto suffix = model.lab_changes.size() > 1 ? core::substitute(" and {} more", model.lab_changes.size() - 1) : std::string {};
            builder.action_row("Lab: " + model.lab_changes.front().current_text + suffix, command(UiCommandKind::revert_lab_settings));
            builder.present_last(presentation(icons::revert).overflow_at(3));
            builder.select_last(true);
        }
        builder.end_group();
        builder.spacer(0.0);
        builder.begin_group("view");
        // The read-out names what it measures, and the framing buttons keep their names until the
        // line runs short of room.
        builder.readout("status.view_height", "View height", "View height " + quantity(model.view_height_m, core::DisplayQuantity::length, model), RowTone::muted);
        builder.present_last(presentation(icons::ruler).overflow_at(2));
        builder.action_row("Frame subject", command(UiCommandKind::frame_subject));
        builder.present_last(presentation(icons::frame_subject, "F").hide_label_at(1));
        builder.action_row("Frame everything", command(UiCommandKind::frame_all));
        builder.present_last(presentation(icons::frame).hide_label_at(1));
        builder.end_group();
    }

    std::string_view LibraryPanel::id() const
    {
        return "library";
    }
    std::string_view LibraryPanel::title() const
    {
        return "Library";
    }
    RegionId LibraryPanel::region() const
    {
        return RegionId::library_sheet;
    }
    void LibraryPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title("Library");
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("library.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        const auto search = std::string(builder.view_value("library.search"));
        const auto collection = std::string(builder.view_value("library.collection", "all"));
        const auto unvisited_only = builder.view_value("library.visited", "all") == "unvisited";
        const auto sort = std::string(builder.view_value("library.sort", "collections"));
        const auto state_command = [](std::string_view key, std::string_view value)
        {
            auto result = command(UiCommandKind::none);
            result.detail = "state:" + std::string(key) + "=" + std::string(value);
            return result;
        };
        builder.text_field("library.top.search", "Search experiments", search, "Title, concept, description or collection", "library.search");
        builder.present_last(presentation(icons::search));
        std::vector<std::string> collections;
        for (const auto& experiment : model.catalogue)
            if (!experiment.collection.empty() && std::find(collections.begin(), collections.end(), experiment.collection) == collections.end())
                collections.push_back(experiment.collection);
        collection_ids_.clear();
        collection_labels_.clear();
        collection_options_.clear();
        collection_ids_.reserve(collections.size() + 1);
        collection_labels_.reserve(collections.size() + 1);
        collection_ids_.push_back("all");
        collection_labels_.push_back("All collections");
        for (const auto& name : collections)
        {
            collection_ids_.push_back(name);
            collection_labels_.push_back(core::humanise_identifier(name));
        }
        for (std::size_t index = 0; index < collection_ids_.size(); ++index)
            collection_options_.push_back({ collection_ids_[index], collection_labels_[index], {}, {} });
        builder.begin_group("filters");
        ControlSpec collection_spec;
        collection_spec.key = "library.top.collection";
        collection_spec.label = "Collection";
        collection_spec.kind = ControlKind::select;
        collection_spec.options = collection_options_;
        auto choose_collection = command(UiCommandKind::none);
        choose_collection.detail = "state-id:library.collection";
        builder.select_row(collection_spec, collection, choose_collection);
        static constexpr OptionSpec sort_options[] { { "collections", "By collection", {}, {} }, { "suggested", "Suggested order", {}, {} } };
        ControlSpec sort_spec;
        sort_spec.key = "library.top.sort";
        sort_spec.label = "Sort";
        sort_spec.kind = ControlKind::select;
        sort_spec.options = sort_options;
        auto choose_sort = command(UiCommandKind::none);
        choose_sort.detail = "state-id:library.sort";
        builder.select_row(sort_spec, sort, choose_sort);
        builder.action_row("library.top.visited", "Unopened only", state_command("library.visited", unvisited_only ? "all" : "unvisited"));
        builder.select_last(unvisited_only);
        if (!search.empty() || collection != "all" || unvisited_only)
        {
            auto clear = command(UiCommandKind::none);
            clear.detail = "library-reset-filters";
            builder.action_row("library.top.clear_filters", "Clear filters", clear);
            builder.present_last(presentation(icons::close).quiet());
        }
        builder.end_group();
        if (builder.section("library.setups", "My setups and files", false))
        {
            for (const auto& file : builder.view_setup_files())
            {
                const auto exists = std::filesystem::exists(std::filesystem::u8path(file.path));
                auto open = command(UiCommandKind::open_arrangement, file.path);
                auto remove = command(UiCommandKind::none, file.path);
                remove.detail = "remove-setup";
                ListItemContent content { file.title, "Based on " + (file.based_on.empty() ? std::string { "an experiment" } : file.based_on) + " · " + file.date, "setup", {} };
                builder.list_item("library.setups.row", file.path, content, open, exists ? UiCommand {} : remove, exists ? "" : "File not found");
                if (!exists)
                {
                    builder.action_row("Remove " + file.title, remove);
                    builder.present_last(presentation(icons::remove).danger());
                }
            }
            builder.action_row("library.setups.open_file", "Open setup…", command(UiCommandKind::open_arrangement));
            builder.present_last(presentation(icons::open, "Ctrl+O"));
            builder.action_row("Save setup", command(UiCommandKind::save_arrangement));
            builder.present_last(presentation(icons::save, "Ctrl+S"));
            builder.action_row("Import shape", command(UiCommandKind::import_shape), model.relativity ? "Shapes cannot be imported into a relativity experiment." : "");
            builder.present_last(presentation(icons::open, "Ctrl+I"));
        }
        const auto folded = [](std::string_view value)
        {
            std::string result(value);
            std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c)
                {
                    return static_cast<char>(std::tolower(c));
                });
            return result;
        };
        const auto needle = folded(search);
        std::vector<std::string> words;
        for (std::size_t start = 0; start < needle.size();)
        {
            while (start < needle.size() && std::isspace(static_cast<unsigned char>(needle[start])))
                ++start;
            auto end = start;
            while (end < needle.size() && !std::isspace(static_cast<unsigned char>(needle[end])))
                ++end;
            if (end > start)
                words.push_back(needle.substr(start, end - start));
            start = end;
        }
        std::vector<const ExperimentCard*> matches;
        for (const auto& experiment : model.catalogue)
        {
            auto haystack = folded(experiment.title + " " + experiment.hook + " " + experiment.summary + " " + experiment.collection + " " + experiment.level);
            for (const auto& concept : experiment.concepts)
                haystack += " " + folded(concept);
            for (const auto& tag : experiment.tags)
                haystack += " " + folded(tag);
            if (std::any_of(words.begin(), words.end(), [&](const auto& word)
                    {
                        return haystack.find(word) == std::string::npos;
                    }) ||
                (collection != "all" && experiment.collection != collection) || (unvisited_only && builder.view_visited(experiment.id)))
                continue;
            matches.push_back(&experiment);
        }
        // Collections follow the suggested path (the collection holding the earliest suggested
        // experiment comes first) and keep their experiments together in their own order.
        std::map<std::string, int> collection_rank;
        for (const auto& experiment : model.catalogue)
        {
            const auto found = collection_rank.find(experiment.collection);
            if (found == collection_rank.end() || experiment.suggested_order < found->second)
                collection_rank[experiment.collection] = experiment.suggested_order;
        }
        std::stable_sort(matches.begin(), matches.end(), [&](const ExperimentCard* a, const ExperimentCard* b)
            {
                if (sort == "suggested")
                    return a->suggested_order < b->suggested_order;
                const auto rank_a = std::make_tuple(collection_rank[a->collection], a->collection, a->collection_order, a->suggested_order);
                const auto rank_b = std::make_tuple(collection_rank[b->collection], b->collection, b->collection_order, b->suggested_order);
                return rank_a < rank_b;
            });
        builder.readout("library.results.count", "Experiments", std::to_string(matches.size()) + " of " + std::to_string(model.catalogue.size()));
        if (matches.empty())
        {
            builder.paragraph(words.empty() ? "No experiments match these filters. Clear filters to see the full library." : "No experiments match your search and filters. Try fewer words or clear filters.");
            return;
        }
        const auto current = std::find_if(matches.begin(), matches.end(), [&](const auto* item)
            {
                return item->id == model.scenario_id;
            });
        const auto default_selection = current == matches.end() ? matches.front()->id : (*current)->id;
        auto selected_id = std::string(builder.view_value("library.selected", default_selection));
        if (std::none_of(matches.begin(), matches.end(), [&](const auto* item)
                {
                    return item->id == selected_id;
                }))
            selected_id = default_selection;
        std::string previous_collection;
        for (const auto* experiment : matches)
        {
            if (sort == "collections" && experiment->collection != previous_collection)
            {
                builder.heading(core::humanise_identifier(experiment->collection));
                previous_collection = experiment->collection;
            }
            const auto is_selected = experiment->id == selected_id;
            std::vector<std::string> badges;
            if (experiment->id == model.scenario_id)
                badges.push_back("Current");
            badges.push_back(experiment->level);
            // Sorted by collection, the heading above already names it.
            if (experiment->special_relativity && sort != "collections")
                badges.push_back("Special relativity");
            // The selected card opens into its own description: the full summary and every
            // concept, where the others show the one-line hook and the first few.
            for (std::size_t index = 0; index < std::min<std::size_t>(is_selected ? experiment->concepts.size() : 3, experiment->concepts.size()); ++index)
                badges.push_back(experiment->concepts[index]);
            if (std::find(experiment->tags.begin(), experiment->tags.end(), "simulator") != experiment->tags.end())
                badges.push_back("How the simulator works");
            if (experiment->id != model.scenario_id && builder.view_visited(experiment->id))
                badges.push_back("Visited");
            ListItemContent content { experiment->title, is_selected && !experiment->summary.empty() ? experiment->summary : experiment->hook, experiment->level, std::move(badges) };
            // The selected card carries its own actions, so opening it is part of the card.
            if (is_selected)
            {
                auto open = command(UiCommandKind::load_scenario, experiment->id);
                if (experiment->id == model.scenario_id)
                {
                    open = command(UiCommandKind::none);
                    open.detail = "view:close";
                }
                // Returning to the open experiment only closes the Library, so it points back
                // rather than offering to start a run.
                const auto current_experiment = experiment->id == model.scenario_id;
                content.actions.push_back({ current_experiment ? "Back to experiment" : "Open experiment", std::string(current_experiment ? icons::previous_step : icons::play), open, true });
                for (const auto& prerequisite : experiment->builds_on)
                {
                    const auto found = std::find_if(model.catalogue.begin(), model.catalogue.end(), [&](const auto& item)
                        {
                            return item.id == prerequisite;
                        });
                    auto show = command(UiCommandKind::none, prerequisite);
                    show.detail = "library-show";
                    content.actions.push_back({ "Builds on " + (found != model.catalogue.end() ? found->title : core::humanise_identifier(prerequisite)), std::string(icons::guide), show, false });
                }
            }
            auto select = state_command("library.selected", experiment->id);
            builder.list_item("library.cards.card", experiment->id, content, select, command(UiCommandKind::load_scenario, experiment->id));
            builder.select_last(is_selected);
        }
    }

    std::string_view MainMenuPanel::id() const
    {
        return "main_menu";
    }
    std::string_view MainMenuPanel::title() const
    {
        return "Main menu";
    }
    RegionId MainMenuPanel::region() const
    {
        return RegionId::show_popover;
    }
    void MainMenuPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto view = [&](std::string_view name)
        {
            auto result = command(UiCommandKind::none);
            result.detail = "view:" + std::string(name);
            return result;
        };
        builder.heading("File");
        builder.action_row("menu.file.open_setup", "Open setup…", command(UiCommandKind::open_arrangement));
        builder.present_last(presentation(icons::open, "Ctrl+O"));
        builder.action_row("menu.file.save_setup", "Save setup", command(UiCommandKind::save_arrangement));
        builder.present_last(presentation(icons::save, "Ctrl+S"));
        auto save_as = command(UiCommandKind::save_arrangement);
        save_as.flag = true;
        builder.action_row("menu.file.save_setup_as", "Save setup as…", save_as);
        builder.present_last(presentation(icons::export_file, "Ctrl+Shift+S"));
        builder.action_row("Restore original", command(UiCommandKind::restore_original));
        builder.present_last(presentation(icons::reset));
        auto revert_setup = command(UiCommandKind::revert_change, "setup");
        builder.action_row("Revert setup changes", revert_setup);
        builder.present_last(presentation(icons::revert));
        builder.heading("View");
        builder.action_row("menu.view.present", "Present", view("present"));
        builder.present_last(presentation(icons::present, "F5"));
        builder.action_row("menu.view.search", "Command search…", view("search"));
        builder.present_last(presentation(icons::search, "Ctrl+K"));
        builder.action_row("menu.view.performance_overlay", "Performance overlay", view("performance"));
        builder.present_last(presentation(icons::speed));
        builder.select_last(builder.view_surface_open("performance.overlay", false));
        builder.action_row("Frame subject", command(UiCommandKind::frame_subject));
        builder.present_last(presentation(icons::frame_subject, "F"));
        builder.action_row("Frame everything", command(UiCommandKind::frame_all));
        builder.present_last(presentation(icons::frame));
        builder.action_row("Frame selection", command(UiCommandKind::frame_selection), model.selected_bodies.empty() ? "Select an object first." : "");
        builder.present_last(presentation(icons::crosshair, "Shift+F"));
        builder.heading("Capture");
        builder.action_row("Save image", command(UiCommandKind::export_still));
        builder.present_last(presentation(icons::image));
        builder.action_row(model.capturing_frames ? "Stop image sequence" : "Record image sequence", command(UiCommandKind::toggle_frame_capture));
        builder.present_last(presentation(model.capturing_frames ? icons::stop : icons::record));
        builder.select_last(model.capturing_frames);
        builder.action_row("Open captures folder", command(UiCommandKind::open_captures_folder));
        builder.present_last(presentation(icons::open));
        builder.heading("Help");
        builder.action_row("Keyboard shortcuts", view("shortcuts"));
        builder.present_last(presentation(icons::keyboard, "F1"));
        builder.action_row("Preferences…", view("preferences"));
        builder.present_last(presentation(icons::settings, "Ctrl+,"));
        builder.action_row("About Rigid Bodies", view("about"));
        builder.present_last(presentation(icons::info));
        builder.action_row("Quit", command(UiCommandKind::quit));
        builder.present_last(presentation(icons::quit, "Ctrl+Q"));
    }

    std::string_view PreferencesPanel::id() const
    {
        return "preferences";
    }
    std::string_view PreferencesPanel::title() const
    {
        return "Preferences";
    }
    RegionId PreferencesPanel::region() const
    {
        return RegionId::modal;
    }
    void PreferencesPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title("Preferences");
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("prefs.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        const auto preference = [](std::string_view key, bool flag = false)
        {
            auto result = command(UiCommandKind::set_preference, key);
            result.detail = std::string(key);
            result.flag = flag;
            return result;
        };
        builder.heading("Appearance");
        builder.segmented_row(spec("prefs.appearance.theme"), model.theme_id, command(UiCommandKind::set_theme));
        builder.number_row(spec("prefs.appearance.text_size"), model.ui_scale, command(UiCommandKind::set_ui_scale));
        builder.heading("Accessibility");
        builder.switch_row(spec("prefs.accessibility.reduce_motion"), model.reduce_motion, preference("prefs.accessibility.reduce_motion", !model.reduce_motion));
        builder.heading("Playback");
        builder.switch_row(spec("prefs.playback.pause_in_background"), model.pause_in_background, preference("prefs.playback.pause_in_background", !model.pause_in_background));
        builder.heading("Experiments");
        {
            auto reset = command(UiCommandKind::none);
            reset.detail = "reset-hints";
            builder.action_row("prefs.experiments.reset_hints", "Show first-run hints again", reset);
            builder.present_last(presentation(icons::hint));
        }
        builder.select_row(spec("prefs.experiments.on_start"), model.experiments_on_start, preference("prefs.experiments.on_start"));
        builder.switch_row(spec("prefs.experiments.open_running"), model.open_experiments_running, preference("prefs.experiments.open_running", !model.open_experiments_running));
        builder.switch_row(spec("prefs.experiments.keep_lab_settings"), model.keep_lab_settings, preference("prefs.experiments.keep_lab_settings", !model.keep_lab_settings));
        builder.switch_row(spec("prefs.experiments.recommended_view"), model.recommended_view, preference("prefs.experiments.recommended_view", !model.recommended_view));
        builder.switch_row(spec("prefs.experiments.ask_predictions"), model.ask_predictions, preference("prefs.experiments.ask_predictions", !model.ask_predictions));
        builder.heading("Visual effects");
        // The highlighted quality is the preset the switches and limits below match; any other mix
        // reads Custom.
        builder.segmented_row(spec("prefs.effects.quality"), model.effects_quality, preference("prefs.effects.quality"));
        if (model.reduce_motion)
            builder.label("Reduce motion keeps blur, impact flashes, sparks, dust and smooth transitions off.");
        const auto& visual = model.visual_settings;
        const auto effect = [&](const char* key, const char* id, bool enabled, bool motion = false)
        {
            auto request = command(UiCommandKind::set_visual_effect, id);
            request.flag = !enabled;
            builder.switch_row(spec(key), enabled, request);
            if (motion && model.reduce_motion)
                builder.disable_last("Off while Reduce motion is on");
        };
        effect("prefs.effects.material_shading", "material_shading", visual.material_shading);
        effect("prefs.effects.contact_shadows", "contact_shadows", visual.contact_shadows);
        effect("prefs.effects.depth_background", "depth_background", visual.depth_background);
        effect("prefs.effects.motion_trails", "motion_trails", visual.motion_trails);
        effect("prefs.effects.directional_blur", "directional_blur", visual.directional_blur, true);
        effect("prefs.effects.impact_flashes", "impact_flashes", visual.impact_flashes, true);
        effect("prefs.effects.impact_sparks", "impact_sparks", visual.impact_sparks, true);
        effect("prefs.effects.impact_dust", "impact_dust", visual.impact_dust, true);
        effect("prefs.effects.soft_deformation", "soft_deformation", visual.soft_deformation);
        effect("prefs.effects.transitions", "transitions", visual.transitions, true);
        builder.heading("Effect limits");
        const auto limit = [&](const char* key, const char* id, std::size_t value)
        {
            auto request = command(UiCommandKind::set_visual_budget, id);
            builder.number_row(spec(key), static_cast<double>(value), request);
        };
        limit("prefs.limits.motion_body_budget", "motion_body_budget", visual.motion_body_budget);
        limit("prefs.limits.directional_blur_budget", "directional_blur_budget", visual.directional_blur_budget);
        limit("prefs.limits.shading_body_budget", "shading_body_budget", visual.shading_body_budget);
        limit("prefs.limits.contact_shadow_budget", "contact_shadow_budget", visual.contact_shadow_budget);
        limit("prefs.limits.impact_flash_budget", "impact_flash_budget", visual.impact_flash_budget);
        limit("prefs.limits.spark_budget", "spark_budget", visual.spark_budget);
        limit("prefs.limits.dust_budget", "dust_budget", visual.dust_budget);
        limit("prefs.limits.deformation_budget", "deformation_budget", visual.deformation_budget);
        builder.action_row("Restore effects defaults", preference("prefs.effects.restore_defaults"));
        builder.present_last(presentation(icons::reset));
        builder.heading("Capture");
        builder.segmented_row(spec("prefs.capture.area"), model.capture_area, preference("prefs.capture.area"));
        builder.action_row("Open captures folder", command(UiCommandKind::open_captures_folder));
        builder.heading("Controls");
        {
            auto performance = command(UiCommandKind::none);
            performance.detail = "view:performance";
            builder.action_row("prefs.performance.overlay", "Toggle Performance overlay", performance);
        }
        builder.number_row(spec("prefs.controls.zoom_speed"), model.camera_zoom_sensitivity, command(UiCommandKind::set_camera_zoom_sensitivity));
    }

    std::string_view ShortcutsPanel::id() const
    {
        return "shortcuts";
    }
    std::string_view ShortcutsPanel::title() const
    {
        return "Keyboard shortcuts";
    }
    RegionId ShortcutsPanel::region() const
    {
        return RegionId::modal;
    }
    void ShortcutsPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title("Keyboard shortcuts");
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("shortcuts.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        // Bindings are listed in table order; group them so each category appears once.
        std::vector<const KeyReference*> ordered;
        ordered.reserve(model.keyboard_reference.size());
        for (const auto& binding : model.keyboard_reference)
            ordered.push_back(&binding);
        std::stable_sort(ordered.begin(), ordered.end(), [](const KeyReference* a, const KeyReference* b)
            {
                return a->category < b->category;
            });
        KeyCategory previous = KeyCategory::help;
        bool first = true;
        for (const auto* entry : ordered)
        {
            const auto& binding = *entry;
            if (first || binding.category != previous)
                builder.heading(binding.category == KeyCategory::playback ? "Playback" : binding.category == KeyCategory::tools ? "Tools"
                        : binding.category == KeyCategory::view                                                                 ? "View"
                        : binding.category == KeyCategory::surfaces                                                             ? "Panels"
                        : binding.category == KeyCategory::editing                                                              ? "Editing"
                        : binding.category == KeyCategory::draw                                                                 ? "Draw"
                        : binding.category == KeyCategory::files                                                                ? "Files"
                                                                                                                                : "Help");
            builder.value_row(binding.description, binding.chord);
            previous = binding.category;
            first = false;
        }
        builder.paragraph("Keyboard shortcuts are off while you type in a text field. Esc closes the topmost sheet or menu.");
    }

    std::string_view AboutPanel::id() const
    {
        return "about";
    }
    std::string_view AboutPanel::title() const
    {
        return "About";
    }
    RegionId AboutPanel::region() const
    {
        return RegionId::modal;
    }
    void AboutPanel::build(const UiModel&, PanelBuilder& builder)
    {
        builder.title("Rigid Bodies");
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("about.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        builder.paragraph("An interactive 2D playground for exploring how rigid bodies move.");
        builder.value_row("Version", rigidbodies::project_version);
        builder.value_row("Author", rigidbodies::project_author);
        builder.value_row("Contact", rigidbodies::project_contact);
        builder.value_row("License", rigidbodies::project_license);
        builder.paragraph(rigidbodies::project_copyright);
        builder.heading("Credits");
        builder.value_row("Interface type", "Inter, SIL Open Font License");
        builder.value_row("Icons", "Phosphor Icons, MIT License");
    }

    std::string_view ConfirmationPanel::id() const
    {
        return "confirmation";
    }
    std::string_view ConfirmationPanel::title() const
    {
        return "Confirmation";
    }
    RegionId ConfirmationPanel::region() const
    {
        return RegionId::modal;
    }
    void ConfirmationPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.confirmation)
            return;
        builder.title(model.confirmation->title);
        builder.paragraph(model.confirmation->text);
        builder.action_row(model.confirmation->save_label, model.confirmation->save);
        builder.action_row(model.confirmation->confirm_label, model.confirmation->confirm);
        builder.action_row("Cancel", model.confirmation->cancel);
    }

    std::string_view PlaybackSpeedPanel::id() const
    {
        return "playback_speed";
    }

    std::string_view PlaybackSpeedPanel::title() const
    {
        return "Playback speed";
    }

    RegionId PlaybackSpeedPanel::region() const
    {
        return RegionId::show_popover;
    }

    void PlaybackSpeedPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        builder.title(title());
        UiCommand close;
        close.detail = "view:close";
        builder.action_row("playback.close", "Done", close);
        builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        // A relativity experiment plays in slow motion: a world second is a lab nanosecond.
        builder.paragraph(model.relativity ? "1× plays one nanosecond of lab time each second, a billion times slower than real time. Choose 0.05× to 4×; slower playback makes the clocks easier to compare."
                                           : "1× is real time. Choose 0.05× to 4×; slower playback makes motion easier to inspect.");
        builder.number_row(spec("bar.speed.custom"), model.time_scale, command(UiCommandKind::set_time_scale));
        builder.action_row("playback.normal", "Reset to 1×", command(UiCommandKind::set_time_scale, 1.0));
        builder.readout("playback.elapsed", "Each real second advances", elapsed_text(model, model.time_scale) + (model.relativity ? " of lab time" : ""));
    }

    std::vector<std::unique_ptr<Panel>> create_default_panels()
    {
        std::vector<std::unique_ptr<Panel>> panels;
        panels.push_back(std::make_unique<CommandBarPanel>());
        panels.push_back(std::make_unique<PlaybackSpeedPanel>());
        panels.push_back(std::make_unique<StatusLinePanel>());
        panels.push_back(std::make_unique<LibraryPanel>());
        panels.push_back(std::make_unique<MainMenuPanel>());
        panels.push_back(std::make_unique<PreferencesPanel>());
        panels.push_back(std::make_unique<SaveDetailsPanel>());
        panels.push_back(std::make_unique<ShortcutsPanel>());
        panels.push_back(std::make_unique<AboutPanel>());
        panels.push_back(std::make_unique<ConfirmationPanel>());
        panels.push_back(std::make_unique<VisualizationPanel>());
        panels.push_back(std::make_unique<DrawBarPanel>());
        panels.push_back(std::make_unique<DrawOptionsPanel>());
        panels.push_back(std::make_unique<GuidePanel>());
        panels.push_back(std::make_unique<InspectorPanel>());
        panels.push_back(std::make_unique<MeasurePanel>());
        panels.push_back(std::make_unique<BannerPanel>());
        panels.push_back(std::make_unique<PresentPanel>());
        panels.push_back(std::make_unique<PresentCaptionPanel>());
        panels.push_back(std::make_unique<HoverCardPanel>());
        panels.push_back(std::make_unique<ContextMenuPanel>());
        panels.push_back(std::make_unique<CommandSearchPanel>());
        panels.push_back(std::make_unique<HintsPanel>());
        panels.push_back(std::make_unique<AddMenuPanel>());
        panels.push_back(std::make_unique<PerformancePanel>());

        return panels;
    }

} // namespace rigidbodies::ui
