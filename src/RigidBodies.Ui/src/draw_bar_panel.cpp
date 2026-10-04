#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/text_format.hpp>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand draw(UiCommandKind kind)
        {
            UiCommand result;
            result.kind = kind;
            return result;
        }
        UiCommand draw_flag(UiCommandKind kind, bool value)
        {
            auto result = draw(kind);
            result.flag = value;
            return result;
        }
        const ControlSpec& spec(std::string_view key)
        {
            return *find_control_spec(key);
        }
        UiCommand view(std::string_view name)
        {
            UiCommand result;
            result.detail = "view:" + std::string(name);
            return result;
        }

        void snapping_rows(const UiModel& model, PanelBuilder& builder, std::string_view instance)
        {
            builder.switch_row(spec("draw.snap.grid"), model.shape_snap_grid, draw_flag(UiCommandKind::set_shape_snap_grid, !model.shape_snap_grid));
            builder.instance_last(instance);
            builder.select_row(spec("draw.snap.grid_spacing"), core::fixed(model.shape_grid_spacing_m, 2), draw(UiCommandKind::set_shape_grid_spacing));
            builder.instance_last(instance);
            builder.switch_row(spec("draw.snap.points"), model.shape_snap_vertices, draw_flag(UiCommandKind::set_shape_snap_vertices, !model.shape_snap_vertices));
            builder.instance_last(instance);
            builder.switch_row(spec("draw.snap.angles"), model.shape_snap_angles, draw_flag(UiCommandKind::set_shape_snap_angles, !model.shape_snap_angles));
            builder.instance_last(instance);
        }
    }

    std::string_view DrawBarPanel::id() const
    {
        return "draw_bar";
    }
    std::string_view DrawBarPanel::title() const
    {
        return "Draw";
    }
    RegionId DrawBarPanel::region() const
    {
        return RegionId::draw_bar;
    }

    // The bar keeps the commit actions at every width. Snapping and material shed to icons and
    // then leave the bar; the Options popover always offers them, together with precision.
    void DrawBarPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.draft.active && !model.shape_editor_active)
            return;
        builder.begin_group("identity");
        builder.label(model.draft.target_name.empty() ? "New shape" : model.draft.target_name);
        builder.present_last(presentation().overflow_at(3));
        builder.select_row(spec("draw.bar.material"), model.draft.material_name.empty() ? model.shape_material_name : model.draft.material_name, draw(UiCommandKind::set_shape_material));
        if (!model.shape_can_change_material)
            builder.disable_last("Apply the outline, then change material in Selection › Properties.");
        builder.present_last(presentation().overflow_at(5));
        builder.end_group();

        // Snapping toggles are pressed-state buttons so they can shed their labels like any tool.
        builder.begin_group("snap");
        builder.action_row("draw.snap.grid", "Grid", draw_flag(UiCommandKind::set_shape_snap_grid, !model.shape_snap_grid));
        builder.present_last(presentation(icons::grid).hide_label_at(1).overflow_at(6));
        builder.select_last(model.shape_snap_grid);
        builder.select_row(spec("draw.snap.grid_spacing"), core::fixed(model.shape_grid_spacing_m, 2), draw(UiCommandKind::set_shape_grid_spacing));
        builder.present_last(presentation().overflow_at(4));
        builder.action_row("draw.snap.points", "Points", draw_flag(UiCommandKind::set_shape_snap_vertices, !model.shape_snap_vertices));
        builder.present_last(presentation(icons::snap_points).hide_label_at(1).overflow_at(6));
        builder.select_last(model.shape_snap_vertices);
        builder.action_row("draw.snap.angles", "15° angles", draw_flag(UiCommandKind::set_shape_snap_angles, !model.shape_snap_angles));
        builder.present_last(presentation(icons::angle).hide_label_at(1).overflow_at(6));
        builder.select_last(model.shape_snap_angles);
        builder.end_group();

        if (model.shape_node_selected)
        {
            builder.begin_group("node");
            builder.segmented_row(spec("draw.node.edge"), model.shape_selected_edge_cubic ? "curved" : "straight", draw(UiCommandKind::set_shape_edge));
            builder.present_last(presentation().hide_label_at(2).overflow_at(8));
            builder.segmented_row(spec("draw.node.join"), model.shape_continuity.empty() ? "corner" : model.shape_continuity, draw(UiCommandKind::set_shape_continuity));
            builder.present_last(presentation().overflow_at(8));
            builder.action_row("Insert node", draw(UiCommandKind::insert_shape_node));
            builder.present_last(presentation(icons::add_circle).hide_label_at(1));
            builder.action_row("Delete node", draw(UiCommandKind::remove_shape_node));
            builder.present_last(presentation(icons::remove, "Del").hide_label_at(1).danger());
            builder.end_group();
        }

        builder.spacer(0.0);
        builder.begin_group("commit");
        if (!model.draft.diagnostic.empty())
        {
            // The instruction for finishing the outline never leaves the bar; only a closed outline
            // that cannot become a body is a warning.
            const auto problem = model.draft.closed && !model.draft.valid;
            builder.readout("draw.diagnostic", "Shape", model.draft.diagnostic, problem ? RowTone::warning : RowTone::info);
            builder.present_last(presentation(problem ? icons::warning : icons::info));
        }
        builder.action_row("draw.options.open", "Options", view("draw_options"));
        builder.present_last(presentation(icons::sliders).hide_label_at(1));
        builder.select_last(builder.view_transient_open("draw_options"));
        if (!model.draft.closed)
        {
            // The outline's next step keeps its name as long as Discard keeps its own.
            builder.action_row("Close shape", draw(UiCommandKind::close_shape_outline));
            builder.present_last(presentation(icons::shape, "Enter").hide_label_at(9));
        }
        builder.action_row("Discard", draw(UiCommandKind::cancel_shape_outline));
        builder.present_last(presentation(icons::close, "Esc").hide_label_at(9).danger());
        builder.action_row("Apply", draw(UiCommandKind::commit_shape_outline), model.draft.valid ? "" : model.draft.diagnostic);
        builder.present_last(presentation(icons::success).primary().hide_label_at(10));
        builder.end_group();
    }

    std::string_view DrawOptionsPanel::id() const
    {
        return "draw_options";
    }
    std::string_view DrawOptionsPanel::title() const
    {
        return "Drawing options";
    }
    RegionId DrawOptionsPanel::region() const
    {
        return RegionId::show_popover;
    }

    void DrawOptionsPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.draft.active && !model.shape_editor_active)
            return;
        builder.title(title());
        builder.select_row(spec("draw.bar.material"), model.draft.material_name.empty() ? model.shape_material_name : model.draft.material_name, draw(UiCommandKind::set_shape_material));
        if (!model.shape_can_change_material)
            builder.disable_last("Apply the outline, then change material in Selection › Properties.");
        builder.instance_last("options");
        builder.heading("Snapping");
        snapping_rows(model, builder, "options");
        // Tolerances tune how outlines become geometry; they stay folded away so the popover
        // leads with material and snapping.
        if (builder.section("draw.precision", "Precision", false))
        {
            builder.number_row(spec("draw.precision.drawing"), model.shape_render_tolerance_m, draw(UiCommandKind::set_shape_render_tolerance));
            builder.number_row(spec("draw.precision.collision"), model.shape_collision_tolerance_m, draw(UiCommandKind::set_shape_collision_tolerance));
            builder.number_row(spec("draw.precision.simplify"), model.shape_simplification_tolerance_m, draw(UiCommandKind::set_shape_simplification_tolerance));
            builder.number_row(spec("draw.precision.hollows"), model.shape_concavity_tolerance_m, draw(UiCommandKind::set_shape_concavity_tolerance));
            builder.stepper_row(spec("draw.precision.vertex_budget"), static_cast<double>(model.shape_vertex_budget), draw(UiCommandKind::set_shape_vertex_budget));
        }
        if (model.shape_node_selected)
        {
            builder.heading("Selected point");
            auto position = draw(UiCommandKind::set_shape_node_position);
            position.id = std::to_string(model.shape_selected_node);
            position.detail = "x";
            builder.number_row(spec("draw.node.position_x"), model.shape_node_world_m.x, position);
            position.detail = "y";
            builder.number_row(spec("draw.node.position_y"), model.shape_node_world_m.y, position);
            builder.label("World coordinates. On canvas: Alt+arrows nudges 1 cm; Shift+Alt+arrows nudges 10 cm.");
            builder.segmented_row(spec("draw.node.edge"), model.shape_selected_edge_cubic ? "curved" : "straight", draw(UiCommandKind::set_shape_edge));
            builder.instance_last("options");
            builder.segmented_row(spec("draw.node.join"), model.shape_continuity.empty() ? "corner" : model.shape_continuity, draw(UiCommandKind::set_shape_continuity));
            builder.instance_last("options");
        }
        builder.separator();
        // Importing opens a file, so it shows the open-file glyph; the polygon stays with closing
        // the outline being drawn.
        builder.action_row("Import shape…", draw(UiCommandKind::import_shape));
        builder.present_last(presentation(icons::open, "Ctrl+I"));
    }
}
