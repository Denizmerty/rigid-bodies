#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <algorithm>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand command(UiCommandKind kind, physics::BodyId body = {})
        {
            UiCommand result;
            result.kind = kind;
            result.body = body;
            return result;
        }

        // The lab clock as the status line and the Present strip show it, beside its run.
        bool relativity_time_readout(std::string_view key)
        {
            return key == "bar.time.readout" || key == "present.time";
        }

        // The run channel a relativity reading is recorded in, or empty for a reading that is not
        // recorded over a run.
        std::string_view relativity_channel(std::string_view key)
        {
            if (key == "measure.relativity.lab_clock" || relativity_time_readout(key))
                return "lab_clock";
            if (key == "measure.relativity.probe_clock")
                return "probe_clock";
            if (key == "measure.relativity.clock_gap")
                return "clock_gap";
            if (key == "measure.relativity.gamma")
                return "lorentz";
            return {};
        }

        // A reading that is the same at every speed: the probe's rest mass and rest energy, and c.
        bool relativity_constant(std::string_view key)
        {
            return key == "world.relativity.rest_mass" || key == "world.relativity.light_speed" || key == "measure.relativity.rest_mass" || key == "measure.relativity.rest_energy";
        }

        // A relativity reading: the clocks are graphed and recorded over a run; the rest follow
        // the speed alone, which the Relativity plot already shows, or do not change at all.
        void relativity_value_actions(PanelBuilder& builder, std::string_view key, std::string_view value)
        {
            const auto channel = relativity_channel(key);
            std::string_view plot_reason;
            if (channel.empty())
                plot_reason = relativity_constant(key) ? "This value is the same at every speed." : "This value depends only on the speed. See the Relativity graph.";
            // The Graph adds this clock to the ones it plots, then shows them.
            UiCommand plot;
            plot.detail = "plot-clock:" + std::string(channel);
            builder.action_row("context.readout.plot", "Plot over time", plot, plot_reason);
            builder.present_last(presentation(icons::measure));
            UiCommand pin = command(UiCommandKind::pin_run_value);
            pin.id = std::string(channel);
            builder.action_row("context.readout.pin", "Add to runs table…", pin, channel.empty() ? "Only values recorded over time can be added." : "");
            builder.present_last(presentation(icons::table));
            UiCommand copy;
            copy.detail = "copy-value:" + std::string(value);
            builder.action_row("context.readout.copy", "Copy value", copy, value.empty() ? "No value is available to copy." : "");
            builder.present_last(presentation(icons::copy));
            UiCommand revert = command(UiCommandKind::revert_change);
            revert.id = std::string(key);
            builder.action_row("context.readout.revert", "Revert to original", revert, key == "world.relativity.speed" ? "" : "Only a setting can be reverted.");
            builder.present_last(presentation(icons::revert));
        }
    }

    std::string_view ContextMenuPanel::id() const
    {
        return "context_menu";
    }
    std::string_view ContextMenuPanel::title() const
    {
        return "Actions";
    }
    RegionId ContextMenuPanel::region() const
    {
        return RegionId::show_popover;
    }

    void ContextMenuPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto ui_kind = builder.view_value("context.kind", "stage");
        if (ui_kind == "readout")
        {
            const auto key = builder.view_value("context.key");
            const auto value = builder.view_value("context.value");
            builder.title("Value actions");
            if (model.relativity && relativity_time_readout(key))
            {
                // The same lab clock as Measure's, so the same actions; its copy is the time alone,
                // without the name the line shows beside it.
                relativity_value_actions(builder, key, now_text(model));
                return;
            }
            if (model.relativity && (key.rfind("measure.relativity.", 0) == 0 || key.rfind("world.relativity.", 0) == 0))
            {
                relativity_value_actions(builder, key, value);
                return;
            }
            UiCommand plot;
            plot.detail = "reveal:measure.graph.scope";
            plot.id = std::string(key);
            builder.action_row("context.readout.plot", "Plot over time", plot);
            builder.present_last(presentation(icons::measure));
            // A read-out of the status line, the Present strip or a panel is neither recorded over a
            // run nor a setting, so the runs table and Revert have nothing to act on.
            UiCommand pin = command(UiCommandKind::pin_run_value);
            pin.id = std::string(key);
            builder.action_row("context.readout.pin", "Add to runs table…", pin, "Only values recorded over time can be added.");
            builder.present_last(presentation(icons::table));
            UiCommand copy;
            copy.detail = "copy-value:" + std::string(value);
            builder.action_row("context.readout.copy", "Copy value", copy, value.empty() ? "No value is available to copy." : "");
            builder.present_last(presentation(icons::copy));
            UiCommand revert = command(UiCommandKind::revert_change);
            revert.id = std::string(key);
            builder.action_row("context.readout.revert", "Revert to original", revert, "Only a setting can be reverted.");
            builder.present_last(presentation(icons::revert));
            return;
        }
        if (ui_kind == "graph")
        {
            builder.title("Graph actions");
            // The Relativity plot draws curves, not a recording: it offers the other speed range.
            if (builder.view_value("context.key") == "measure.relativity.plot")
            {
                const auto near = builder.view_value("measure.relativity.range", "full") == "near";
                UiCommand range;
                range.detail = std::string("state:measure.relativity.range=") + (near ? "full" : "near");
                builder.action_row("context.graph.range", near ? "Show up to c" : "Show near c", range);
                builder.present_last(presentation(icons::curve));
                return;
            }
            builder.action_row("context.graph.clear", "Clear graph", command(UiCommandKind::clear_energy_history));
            builder.present_last(presentation(icons::remove));
            UiCommand hide;
            hide.detail = "state:measure.graph.previous_run=off";
            builder.action_row("context.graph.previous", "Hide previous run", hide);
            builder.present_last(presentation(icons::hide));
            return;
        }
        if (!model.context_menu_request)
            return;
        const auto& target = *model.context_menu_request;
        const auto locked = builder.view_present() && builder.view_present_locked();
        const auto reason = locked ? "Locked in Present mode. Use the lock button to unlock." : "";
        // The probe is a relativity experiment's only object.
        const auto add_reason = model.relativity ? "Objects cannot be added to a relativity experiment." : reason;
        const auto draw_reason = model.relativity ? "Shapes cannot be drawn in a relativity experiment." : reason;
        if (target.kind == StageTargetKind::object)
        {
            // The menu is headed by what it acts on.
            const auto object = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& item)
                {
                    return item.id == target.body;
                });
            builder.title(object != model.objects.end() ? object->display_name : std::string { "Object" });
            builder.action_row("context.object.inspect", "Inspect", command(UiCommandKind::select_body, target.body));
            builder.present_last(presentation(icons::sidebar));
            // Framing and deleting act on the selection, so they wait until this object is the
            // selection rather than acting on another one.
            const auto selected = model.selected_bodies.size() == 1 && model.selection == target.body;
            const auto unselected = selected ? "" : "Inspect this object first to select it.";
            builder.action_row("context.object.frame", "Frame selection", command(UiCommandKind::frame_selection, target.body), unselected);
            builder.present_last(presentation(icons::crosshair, "Shift+F"));
            const auto moving = object != model.objects.end() && object->moving && object->kind != "fixed";
            builder.action_row("context.object.stop", "Stop motion", command(UiCommandKind::stop_selected_motion, target.body), moving ? "" : "Already at rest.");
            builder.present_last(presentation(icons::grab));
            builder.action_row("context.object.delete", "Delete", command(UiCommandKind::delete_selected_body, target.body), locked ? reason : unselected);
            builder.present_last(presentation(icons::remove, "Del").danger());
        }
        else if (target.kind == StageTargetKind::connection)
        {
            builder.title("Connection");
            auto select = command(UiCommandKind::select_connection);
            select.id = target.id;
            select.detail = target.detail;
            builder.action_row("context.connection.inspect", "Inspect connection", select);
            builder.present_last(presentation(icons::connection));
            builder.action_row("context.connection.reverse", "Reverse motor", command(UiCommandKind::reverse_joint_motor));
            builder.present_last(presentation(icons::replay));
        }
        else if (target.kind == StageTargetKind::impact)
        {
            builder.title("Impact");
            auto inspect = command(UiCommandKind::select_impact);
            try
            {
                inspect.value = std::stod(target.id);
            }
            catch (...)
            {
                inspect.value = 0.0;
            }
            inspect.detail = "reveal:measure.collisions.list";
            builder.action_row("context.impact.inspect", "Inspect impact", inspect);
            builder.present_last(presentation(icons::impact));
        }
        else
        {
            builder.title("Stage");
            for (const auto* id : { "ball", "box", "plank" })
            {
                auto add = command(UiCommandKind::add_object);
                add.id = id;
                builder.action_row(std::string("context.empty.add_") + id, std::string("Add ") + id + " here", add, add_reason);
                builder.present_last(presentation(std::string_view(id) == "ball" ? icons::ball : std::string_view(id) == "box" ? icons::box
                                                                                                                               : icons::plank));
            }
            auto draw = command(UiCommandKind::start_new_shape);
            draw.value = target.screen_position_px.x;
            draw.value_y = target.screen_position_px.y;
            builder.action_row("context.empty.draw", "Draw shape here", draw, draw_reason);
            builder.present_last(presentation(icons::draw));
            builder.action_row("context.empty.frame_all", "Frame everything", command(UiCommandKind::frame_all));
            builder.present_last(presentation(icons::frame));
        }
    }
}
