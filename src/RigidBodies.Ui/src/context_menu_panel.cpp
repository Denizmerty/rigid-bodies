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
            UiCommand plot;
            plot.detail = "reveal:measure.graph.scope";
            plot.id = std::string(key);
            builder.action_row("context.readout.plot", "Plot over time", plot);
            builder.present_last(presentation(icons::measure));
            UiCommand pin = command(UiCommandKind::pin_run_value);
            pin.id = std::string(key);
            builder.action_row("context.readout.pin", "Add to runs table…", pin, key.empty() ? "Only quantities that can be graphed can be added." : "");
            builder.present_last(presentation(icons::table));
            UiCommand copy;
            copy.detail = "copy-value:" + std::string(value);
            builder.action_row("context.readout.copy", "Copy value", copy, value.empty() ? "No value is available to copy." : "");
            builder.present_last(presentation(icons::copy));
            UiCommand revert = command(UiCommandKind::revert_change);
            revert.id = std::string(key);
            builder.action_row("context.readout.revert", "Revert to original", revert);
            builder.present_last(presentation(icons::revert));
            return;
        }
        if (ui_kind == "graph")
        {
            builder.title("Graph actions");
            builder.action_row("context.graph.clear", "Clear graph", command(UiCommandKind::clear_energy_history));
            builder.present_last(presentation(icons::remove));
            UiCommand hide;
            hide.detail = "state:measure.graph.previous_run=false";
            builder.action_row("context.graph.previous", "Hide previous run", hide);
            builder.present_last(presentation(icons::hide));
            return;
        }
        if (!model.context_menu_request)
            return;
        const auto& target = *model.context_menu_request;
        const auto locked = builder.view_present() && builder.view_present_locked();
        const auto reason = locked ? "Locked in Present mode. Use the lock button to unlock." : "";
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
                builder.action_row(std::string("context.empty.add_") + id, std::string("Add ") + id + " here", add, reason);
                builder.present_last(presentation(std::string_view(id) == "ball" ? icons::ball : std::string_view(id) == "box" ? icons::box
                                                                                                                               : icons::plank));
            }
            auto draw = command(UiCommandKind::start_new_shape);
            draw.value = target.screen_position_px.x;
            draw.value_y = target.screen_position_px.y;
            builder.action_row("context.empty.draw", "Draw shape here", draw, reason);
            builder.present_last(presentation(icons::draw));
            builder.action_row("context.empty.frame_all", "Frame everything", command(UiCommandKind::frame_all));
            builder.present_last(presentation(icons::frame));
        }
    }
}
