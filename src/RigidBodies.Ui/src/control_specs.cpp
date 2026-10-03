#include <rigidbodies/ui/control_spec.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <vector>

namespace rigidbodies::ui
{
    namespace
    {
        using K = UiCommandKind;
        using Q = core::DisplayQuantity;

        // Option ids are the physics catalogue names, so a body's material selects its own entry
        // and a choice sends a name the session knows.
        constexpr OptionSpec materials[] = {
            { "oak_wood", "Oak", "750 kg/m3 · bounciness 0.30 · grip 0.55", {} },
            { "steel", "Steel", "7850 kg/m3 · bounciness 0.55 · grip 0.70", {} },
            { "aluminium", "Aluminium", "2700 kg/m3 · bounciness 0.45 · grip 0.60", {} },
            { "rubber", "Rubber", "1100 kg/m3 · bounciness 0.80 · grip 1.10", {} },
            { "glass", "Glass", "2500 kg/m3 · bounciness 0.60 · grip 0.50", {} },
            { "expanded_polystyrene", "Foam", "Expanded polystyrene · 25 kg/m3 · bounciness 0.20 · grip 0.55", {} },
        };
        constexpr OptionSpec integrators[] = {
            { "semi_implicit_euler", "Semi-implicit Euler", {}, {} },
            { "velocity_verlet", "Velocity Verlet", {}, {} },
            { "runge_kutta_4", "Runge–Kutta 4", {}, {} },
        };
        constexpr OptionSpec bounce_rules[] = {
            { "maximum", "The bouncier surface", "Uses the higher bounciness of the two", {} },
            { "minimum", "The less bouncy surface", "Uses the lower bounciness of the two", {} },
            { "arithmetic_mean", "Average", "Halfway between the two", {} },
            { "geometric_mean", "Geometric average", "sqrt(e1 x e2)", {} },
        };
        constexpr OptionSpec gravity_presets[] = {
            { "earth", "Earth", "9.81 m/s2", {} },
            { "moon", "Moon", "1.62 m/s2", {} },
            { "mars", "Mars", "3.73 m/s2", {} },
            { "custom", "Custom", {}, {} },
        };
        constexpr OptionSpec units[] = { { "si", "SI", {}, {} }, { "centimetre_gram", "CGS", {}, {} } };
        constexpr OptionSpec themes[] = { { "workbench_dark", "Dark", {}, "moon" }, { "workbench_light", "Light", {}, "sun" }, { "workbench_projector", "Projector", "High contrast for classrooms", "presentation-chart" } };
        constexpr OptionSpec tools[] = { { "select", "Select & move", "Select, drag and edit objects (V)", "cursor" }, { "throw", "Throw", "Fling objects with the pointer (T)", "hand" }, { "pull", "Pull", "Attach a temporary spring to an object (P)", "magnet" } };
        constexpr OptionSpec start_choices[] = { { "last", "Last experiment", {}, {} }, { "library", "Library", {}, {} }, { "empty", "Empty lab", {}, {} } };
        constexpr OptionSpec quality_choices[] = { { "low", "Low", {}, {} }, { "standard", "Standard", {}, {} }, { "high", "High", {}, {} }, { "custom", "Custom", {}, {} } };
        constexpr OptionSpec capture_choices[] = { { "stage", "Stage only", {}, {} }, { "window", "Whole window", {}, {} } };
        constexpr OptionSpec vector_scope[] = { { "all", "All objects", {}, {} }, { "selected", "Selected object", {}, {} } };
        constexpr OptionSpec vector_split[] = { { "none", "Off", {}, {} }, { "world", "Horizontal & vertical", {}, {} }, { "chosen", "Along a direction", {}, {} }, { "contact", "Along & across contact", {}, {} } };
        constexpr OptionSpec view_presets[] = { { "recommended", "Recommended", "The overlays this experiment was designed with", {} }, { "none", "Clean", "Objects without overlays", {} }, { "all", "All", "Every layer, including the engine's diagnostics", {} }, { "custom", "Custom", "Your own mix of the layers below", {} } };
        constexpr OptionSpec edge[] = { { "straight", "Straight", {}, "line-segment" }, { "curved", "Curved", {}, "bezier-curve" } };
        constexpr OptionSpec joins[] = { { "corner", "Corner", {}, {} }, { "smooth", "Smooth", {}, {} }, { "symmetric", "Symmetric", {}, {} } };
        constexpr OptionSpec grid_spacing[] = { { "0.01", "0.01 m", {}, {} }, { "0.02", "0.02 m", {}, {} }, { "0.05", "0.05 m", {}, {} }, { "0.10", "0.10 m", {}, {} }, { "0.25", "0.25 m", {}, {} }, { "0.50", "0.50 m", {}, {} } };
        constexpr OptionSpec speeds[] = { { "0.1", "0.1×", {}, {} }, { "0.25", "0.25×", {}, {} }, { "0.5", "0.5×", {}, {} }, { "1", "1×", {}, {} }, { "2", "2×", {}, {} }, { "custom", "Custom…", {}, {} } };
        constexpr OptionSpec time_steps[] = { { "0.0166666667", "1/60 s", {}, {} }, { "0.0083333333", "1/120 s", {}, {} }, { "0.0041666667", "1/240 s", {}, {} }, { "0.0020833333", "1/480 s", {}, {} }, { "custom", "Custom", {}, {} } };

        constexpr double gravity_detents[] = { 1.62, 3.73, 9.80665 };
        constexpr double one_detent[] = { 1.0 };
        constexpr double text_detents[] = { 0.75, 1.0, 1.25, 1.5, 1.75, 2.0 };

        ControlSpec plain(std::string_view key, std::string_view label, std::string_view location, ControlKind kind, K command, EditCategory category = EditCategory::not_an_edit, math::Span<const OptionSpec> options = {}, std::string_view command_id = {})
        {
            ControlSpec result;
            result.key = key;
            result.label = label;
            result.location = location;
            result.kind = kind;
            result.command = command;
            result.command_id = command_id;
            result.options = options;
            result.category = category;
            return result;
        }

        ControlSpec number(std::string_view key, std::string_view label, std::string_view location, K command, double minimum, double maximum, double step, Q quantity, EditCategory category, bool slider = false, NumberScale scale = NumberScale::linear, double soft_minimum = 0.0, double soft_maximum = 0.0, math::Span<const double> detents = {}, bool session_minimum = false, bool session_maximum = false, bool dial = false, std::string_view command_id = {})
        {
            auto result = plain(key, label, location, ControlKind::number, command, category, {}, command_id);
            result.number.minimum = minimum;
            result.number.maximum = maximum;
            result.number.soft_minimum = slider && soft_maximum > soft_minimum ? soft_minimum : minimum;
            result.number.soft_maximum = slider && soft_maximum > soft_minimum ? soft_maximum : maximum;
            result.number.step = step;
            result.number.scale = scale;
            result.number.quantity = quantity;
            result.number.detents = detents;
            result.number.slider = slider;
            result.number.dial = dial;
            result.number.coarse = scale == NumberScale::logarithmic ? 2.0 : 10.0;
            result.number.fine = scale == NumberScale::logarithmic ? 1.01 : 0.1;
            result.number.session_minimum = session_minimum;
            result.number.session_maximum = session_maximum;
            return result;
        }

        const std::vector<ControlSpec>& registry()
        {
            static const std::vector<ControlSpec> table = []
            {
                std::vector<ControlSpec> value {
                    plain("bar.speed.choice", "Playback speed", "Toolbar", ControlKind::select, K::set_time_scale, EditCategory::not_an_edit, speeds),
                    number("camera.scale.height", "View height", "Show › View", K::set_view_height, 0.05, 500.0, 0.01, Q::length, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, true, true),
                    plain("camera.frame.everything", "Frame everything", "Menu › View", ControlKind::action, K::frame_all),
                    plain("camera.frame.selection", "Frame selection", "Menu › View", ControlKind::action, K::frame_selection),
                    plain("camera.frame.subject", "Frame subject", "Menu › View", ControlKind::action, K::frame_subject),
                    plain("draw.actions.apply", "Apply", "Draw bar", ControlKind::action, K::commit_shape_outline, EditCategory::draft),
                    plain("draw.actions.close", "Close shape", "Draw bar", ControlKind::action, K::close_shape_outline, EditCategory::draft),
                    plain("draw.actions.discard", "Discard", "Draw bar", ControlKind::action, K::cancel_shape_outline, EditCategory::draft),
                    plain("draw.bar.material", "Material", "Draw bar", ControlKind::select, K::set_shape_material, EditCategory::draft, materials),
                    plain("draw.node.edge", "Edge", "Draw bar", ControlKind::segmented, K::set_shape_edge, EditCategory::draft, edge),
                    plain("draw.node.insert", "Insert", "Draw bar", ControlKind::action, K::insert_shape_node, EditCategory::draft),
                    plain("draw.node.join", "Join", "Draw bar", ControlKind::segmented, K::set_shape_continuity, EditCategory::draft, joins),
                    plain("draw.node.remove", "Remove", "Draw bar", ControlKind::action, K::remove_shape_node, EditCategory::draft),
                    number("draw.precision.collision", "Collision precision", "Drawing options › Precision", K::set_shape_collision_tolerance, 0.001, 0.05, 0.001, Q::fine_length, EditCategory::draft, true),
                    number("draw.precision.drawing", "Drawing precision", "Drawing options › Precision", K::set_shape_render_tolerance, 0.0005, 0.02, 0.0005, Q::fine_length, EditCategory::draft, true),
                    number("draw.precision.hollows", "Keep hollows deeper than", "Drawing options › Precision", K::set_shape_concavity_tolerance, 0.0, 0.2, 0.005, Q::fine_length, EditCategory::draft, true),
                    number("draw.precision.simplify", "Simplify outline", "Drawing options › Precision", K::set_shape_simplification_tolerance, 0.0, 0.02, 0.0005, Q::fine_length, EditCategory::draft, true),
                    []
                    {
                        auto s = number("draw.precision.vertex_budget", "Vertex budget", "Drawing options › Precision", K::set_shape_vertex_budget, 32, 512, 32, Q::count, EditCategory::draft, false);
                        s.kind = ControlKind::stepper;
                        s.number.session_maximum = true;
                        return s;
                    }(),
                    plain("draw.snap.angles", "15° angles", "Drawing options › Snapping", ControlKind::switch_control, K::set_shape_snap_angles, EditCategory::draft),
                    plain("draw.snap.grid", "Grid", "Drawing options › Snapping", ControlKind::switch_control, K::set_shape_snap_grid, EditCategory::draft),
                    plain("draw.snap.grid_spacing", "Grid spacing", "Drawing options › Snapping", ControlKind::select, K::set_shape_grid_spacing, EditCategory::draft, grid_spacing),
                    plain("draw.snap.points", "Points", "Drawing options › Snapping", ControlKind::switch_control, K::set_shape_snap_vertices, EditCategory::draft),
                    plain("joint.motor.enabled", "Motor", "Inspector › Joint", ControlKind::switch_control, K::set_joint_motor_enabled, EditCategory::parameter),
                    plain("joint.card", "Joint", "Selection › Joints", ControlKind::list, K::select_connection),
                    number("joint.motor.angular_speed", "Motor speed", "Inspector › Joint", K::set_joint_motor_speed, -50, 50, 0.1, Q::angular_velocity, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, true, true),
                    number("joint.motor.linear_speed", "Motor speed", "Inspector › Joint", K::set_joint_motor_speed, -10, 10, 0.01, Q::velocity, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, true, true),
                    plain("joint.motor.reverse", "Reverse", "Inspector › Joint", ControlKind::action, K::reverse_joint_motor, EditCategory::parameter),
                    plain("joint.stops.enabled", "Stops", "Inspector › Joint", ControlKind::switch_control, K::set_joint_limits_enabled, EditCategory::parameter),
                    plain("library.cards.card", "Experiment", "Library", ControlKind::list, K::load_scenario),
                    plain("library.top.import", "Import shape", "Library", ControlKind::action, K::import_shape),
                    plain("library.top.open", "Open setup", "Library", ControlKind::action, K::open_arrangement),
                    plain("library.top.save", "Save setup", "Library", ControlKind::action, K::save_arrangement),
                    plain("measure.collisions.pause_each", "Pause at each impact", "Measure › Collisions", ControlKind::switch_control, K::set_pause_on_impact),
                    plain("measure.collisions.pause_next", "Pause at next impact", "Measure › Collisions", ControlKind::action, K::pause_at_next_impact),
                    plain("measure.collisions.list", "Impacts", "Measure › Collisions", ControlKind::list, K::select_impact),
                    plain("measure.collisions.filter", "Show", "Measure › Collisions", ControlKind::segmented, K::none),
                    plain("measure.energy.scope", "Scope", "Measure › Energy", ControlKind::segmented, K::none),
                    plain("measure.graph.clear", "Clear graph", "Measure › Graph", ControlKind::action, K::clear_energy_history),
                    plain("measure.graph.quantities", "Quantities", "Measure › Graph", ControlKind::checklist, K::none),
                    plain("measure.graph.previous_run", "Previous run", "Measure › Graph", ControlKind::checkbox, K::none),
                    plain("measure.graph.compare_with", "Compare with", "Measure › Graph", ControlKind::select, K::none),
                    plain("measure.graph.scope", "Scope", "Measure › Graph", ControlKind::select, K::none),
                    plain("measure.graph.window", "Window", "Measure › Graph", ControlKind::segmented, K::none),
                    plain("measure.runs.row", "Run", "Measure › Runs", ControlKind::list, K::star_run),
                    plain("measure.runs.clear", "Clear runs", "Measure › Runs", ControlKind::action, K::clear_runs),
                    plain("measure.runs.add_value", "Add value", "Measure › Runs", ControlKind::action, K::pin_run_value),
                    plain("measure.runs.add", "Add", "Measure › Runs › Add value", ControlKind::action, K::pin_run_value),
                    plain("measure.runs.add_aggregator", "Measure", "Measure › Runs › Add value", ControlKind::select, K::none),
                    plain("measure.runs.add_object", "Object", "Measure › Runs › Add value", ControlKind::select, K::none),
                    plain("measure.runs.add_quantity", "Quantity", "Measure › Runs › Add value", ControlKind::select, K::none),
                    plain("measure.runs.compare_a", "Run A", "Measure › Runs", ControlKind::select, K::none),
                    plain("measure.runs.compare_b", "Run B", "Measure › Runs", ControlKind::select, K::none),
                    plain("measure.runs.clear_apply", "Clear unstarred runs", "Measure › Runs", ControlKind::action, K::clear_runs),
                    plain("measure.runs.remove_value", "Remove value", "Measure › Runs", ControlKind::action, K::unpin_run_value),
                    plain("measure.theory.bounce_run", "Run bounce rules", "Measure › Theory checks", ControlKind::action, K::compare_restitution),
                    plain("measure.theory.collisions_run", "Run collisions", "Measure › Theory checks", ControlKind::action, K::compare_collisions),
                    plain("measure.theory.integration_run", "Run integration", "Measure › Theory checks", ControlKind::action, K::compare_integrators),
                    plain("menu.capture.record", "Record image sequence", "Menu › Capture", ControlKind::action, K::toggle_frame_capture),
                    plain("menu.capture.save_image", "Save image", "Menu › Capture", ControlKind::action, K::export_still),
                    plain("object.empty.select_all", "Select all free objects", "World › Objects", ControlKind::action, K::select_all),
                    plain("object.header.delete", "Delete", "Selection", ControlKind::action, K::delete_selected_body, EditCategory::structure),
                    plain("object.list.row", "Object", "World › Objects", ControlKind::list, K::select_body),
                    plain("object.several.combine", "Combine", "Selection", ControlKind::action, K::assemble_selected_bodies, EditCategory::structure_pose),
                    plain("object.motion.stop", "Stop motion", "Selection › Motion", ControlKind::action, K::stop_selected_motion, EditCategory::state),
                    number("object.motion.position_x", "Position x", "Selection › Motion", K::set_selected_position, -100, 100, 0.01, Q::length, EditCategory::state, false, NumberScale::linear, 0, 0, {}, true, true),
                    number("object.motion.position_y", "Position y", "Selection › Motion", K::set_selected_position, -100, 100, 0.01, Q::length, EditCategory::state, false, NumberScale::linear, 0, 0, {}, true, true),
                    plain("object.motion.reverse_spin", "Reverse spin", "Selection › Motion", ControlKind::action, K::set_selected_angular_velocity, EditCategory::state),
                    plain("object.forces.channels", "Applied forces", "Selection › Forces", ControlKind::list, K::none, EditCategory::state),
                    number("object.motion.orientation", "Orientation", "Selection › Motion", K::set_selected_orientation, -180, 180, 0.1, Q::angle, EditCategory::state, false, NumberScale::linear, 0, 0, {}, false, false, true),
                    number("object.motion.spin", "Spin", "Selection › Motion", K::set_selected_angular_velocity, -100, 100, 0.1, Q::angular_velocity, EditCategory::state, false, NumberScale::linear, 0, 0, {}, true, true),
                    number("object.motion.velocity_x", "Velocity x", "Selection › Motion", K::set_selected_velocity_x, -100, 100, 0.1, Q::velocity, EditCategory::state, false, NumberScale::linear, 0, 0, {}, true, true),
                    number("object.motion.velocity_y", "Velocity y", "Selection › Motion", K::set_selected_velocity_y, -100, 100, 0.1, Q::velocity, EditCategory::state, false, NumberScale::linear, 0, 0, {}, true, true),
                    number("object.properties.gravity_scale", "Gravity scale", "Selection › Body", K::set_selected_gravity_scale, 0, 4, 0.05, Q::multiplier, EditCategory::parameter, true, NumberScale::linear, 0, 4, one_detent),
                    number("object.properties.mass", "Mass", "Selection › Body", K::set_selected_mass, 0.001, 1000, 1.1, Q::mass, EditCategory::parameter, true, NumberScale::logarithmic, 0.01, 100),
                    plain("object.properties.material", "Material", "Selection › Body", ControlKind::select, K::set_selected_material, EditCategory::parameter, materials),
                    plain("object.properties.use_density", "Use material density", "Selection › Body", ControlKind::action, K::use_selected_density_mass, EditCategory::parameter),
                    plain("object.shape.edit", "Edit shape", "Selection › Shape", ControlKind::action, K::edit_selected_shape, EditCategory::structure),
                    plain("object.shape.export", "Export shape", "Selection › Shape", ControlKind::action, K::export_shape),
                    plain("object.shape.parts", "Parts", "Selection › Shape", ControlKind::list, K::select_authored_part),
                    plain("object.shape.separate", "Separate parts", "Selection › Shape", ControlKind::action, K::split_selected_body, EditCategory::structure),
                    number("prefs.appearance.text_size", "Text size", "Preferences › Appearance", K::set_ui_scale, 0.75, 2.0, 0.05, Q::scale, EditCategory::not_an_edit, true, NumberScale::linear, 0.75, 2.0, text_detents, true, true),
                    plain("prefs.appearance.theme", "Theme", "Preferences › Appearance", ControlKind::segmented, K::set_theme, EditCategory::not_an_edit, themes),
                    plain("prefs.accessibility.reduce_motion", "Reduce motion", "Preferences › Accessibility", ControlKind::switch_control, K::set_preference),
                    plain("prefs.capture.area", "Capture area", "Preferences › Capture", ControlKind::segmented, K::set_preference, EditCategory::not_an_edit, capture_choices),
                    plain("prefs.effects.quality", "Effects quality", "Preferences › Visual effects", ControlKind::segmented, K::set_preference, EditCategory::not_an_edit, quality_choices),
                    plain("prefs.experiments.keep_lab_settings", "Keep lab settings when changing experiment", "Preferences › Experiments", ControlKind::switch_control, K::set_preference),
                    plain("prefs.experiments.recommended_view", "Use recommended view", "Preferences › Experiments", ControlKind::switch_control, K::set_preference),
                    plain("prefs.experiments.ask_predictions", "Ask for predictions", "Preferences › Experiments", ControlKind::switch_control, K::set_preference),
                    plain("prefs.experiments.open_running", "Open experiments running", "Preferences › Experiments", ControlKind::switch_control, K::set_preference),
                    plain("prefs.experiments.on_start", "On start", "Preferences › Experiments", ControlKind::select, K::set_preference, EditCategory::not_an_edit, start_choices),
                    plain("prefs.playback.pause_in_background", "Pause when the window is in the background", "Preferences › Playback", ControlKind::switch_control, K::set_preference),
                    plain("prefs.effects.contact_shadows", "Contact shadows", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "contact_shadows"),
                    plain("prefs.effects.depth_background", "Background depth", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "depth_background"),
                    plain("prefs.effects.directional_blur", "Directional blur", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "directional_blur"),
                    plain("prefs.effects.impact_dust", "Impact dust", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "impact_dust"),
                    plain("prefs.effects.impact_flashes", "Impact flashes", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "impact_flashes"),
                    plain("prefs.effects.impact_sparks", "Impact sparks", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "impact_sparks"),
                    plain("prefs.effects.material_shading", "Material shading", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "material_shading"),
                    plain("prefs.effects.motion_trails", "Motion trails", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "motion_trails"),
                    plain("prefs.effects.soft_deformation", "Soft impact cues", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "soft_deformation"),
                    plain("prefs.effects.transitions", "Smooth transitions", "Preferences › Visual effects", ControlKind::switch_control, K::set_visual_effect, EditCategory::not_an_edit, {}, "transitions"),
                    number("prefs.controls.zoom_speed", "Wheel zoom speed", "Preferences › Controls", K::set_camera_zoom_sensitivity, 1.02, 1.5, 0.01, Q::multiplier, EditCategory::not_an_edit, true, NumberScale::linear, 1.02, 1.5, {}, false, true),
                    []
                    {
                        auto s = number("prefs.limits.contact_shadow_budget", "Contact shadows", "Preferences › Effect limits", K::set_visual_budget, 8, 192, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "contact_shadow_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.deformation_budget", "Soft impact cues", "Preferences › Effect limits", K::set_visual_budget, 8, 64, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "deformation_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.directional_blur_budget", "Objects with blur", "Preferences › Effect limits", K::set_visual_budget, 8, 128, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "directional_blur_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.dust_budget", "Dust particles", "Preferences › Effect limits", K::set_visual_budget, 8, 128, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "dust_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.impact_flash_budget", "Impact flashes", "Preferences › Effect limits", K::set_visual_budget, 8, 48, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "impact_flash_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.motion_body_budget", "Objects with trails", "Preferences › Effect limits", K::set_visual_budget, 8, 128, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "motion_body_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.shading_body_budget", "Shaded objects", "Preferences › Effect limits", K::set_visual_budget, 8, 4096, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "shading_body_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    []
                    {
                        auto s = number("prefs.limits.spark_budget", "Sparks", "Preferences › Effect limits", K::set_visual_budget, 8, 256, 8, Q::count, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, true, false, "spark_budget");
                        s.kind = ControlKind::stepper;
                        return s;
                    }(),
                    plain("prefs.units.system", "Units", "Show › View", ControlKind::segmented, K::set_display_units, EditCategory::not_an_edit, units),
                    number("show.arrows.direction", "Direction", "Show › Arrows", K::set_component_angle_degrees, -180, 180, 1, Q::angle, EditCategory::not_an_edit, false, NumberScale::linear, 0, 0, {}, false, false, true),
                    plain("show.arrows.auto_length", "Automatic arrow length", "Show › Arrows", ControlKind::switch_control, K::set_vector_auto_scale),
                    number("show.arrows.scale", "Arrow scale", "Show › Arrows", K::set_vector_scale, 0.1, 200, 1.1, Q::multiplier, EditCategory::not_an_edit, true, NumberScale::logarithmic),
                    plain("show.arrows.split", "Split arrows", "Show › Arrows", ControlKind::radio_list, K::set_vector_components, EditCategory::not_an_edit, vector_split),
                    plain("show.arrows.scope", "Arrow scope", "Show › Arrows", ControlKind::segmented, K::set_vector_scope, EditCategory::not_an_edit, vector_scope),
                    plain("show.layer.acceleration", "Acceleration", "Show › Motion", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "acceleration"),
                    plain("show.layer.angular_velocity", "Spin", "Show › Motion", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "angular_velocity"),
                    plain("show.layer.bodies", "Objects", "Show › Objects", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "bodies"),
                    plain("show.layer.bounding_boxes", "Bounding boxes", "Show › Engine internals", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "bounding_boxes"),
                    plain("show.layer.broad_phase_pairs", "Candidate pairs", "Show › Engine internals", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "broad_phase_pairs"),
                    plain("show.layer.center_of_mass", "Centre of mass", "Show › Reference", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "center_of_mass"),
                    plain("show.layer.constraints", "Joints and springs", "Show › Forces and contact", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "constraints"),
                    plain("show.layer.contact_normals", "Contact normals", "Show › Forces and contact", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "contact_normals"),
                    plain("show.layer.contact_points", "Contact points", "Show › Forces and contact", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "contact_points"),
                    plain("show.layer.force", "Forces", "Show › Forces and contact", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "force"),
                    plain("show.layer.grid", "Grid", "Show › Reference", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "grid"),
                    plain("show.layer.labels", "Labels", "Show › Objects", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "labels"),
                    plain("show.layer.momentum", "Momentum", "Show › Motion", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "momentum"),
                    plain("show.layer.outlines", "Outlines", "Show › Objects", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "outlines"),
                    plain("show.layer.trajectories", "Trajectories", "Show › Motion", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "trajectories"),
                    plain("show.layer.velocity", "Velocity", "Show › Motion", ControlKind::switch_control, K::set_layer, EditCategory::not_an_edit, {}, "velocity"),
                    plain("show.presets.preset", "Presets", "Show", ControlKind::segmented, K::set_layer_mask, EditCategory::not_an_edit, view_presets),
                    plain("tools.mode", "Tools", "Toolbar", ControlKind::segmented, K::set_interaction_mode, EditCategory::not_an_edit, tools),
                    plain("tools.quit", "Quit", "Menu", ControlKind::action, K::quit),
                    plain("tools.redo", "Redo", "Toolbar", ControlKind::action, K::redo),
                    plain("tools.undo", "Undo", "Toolbar", ControlKind::action, K::undo),
                    number("spring.angular.damping", "Damping", "Inspector › Spring", K::set_spring_parameter, 0, 10000, 0.01, Q::twist_damping, EditCategory::parameter, true, NumberScale::logarithmic, 0.001, 1000),
                    plain("spring.card", "Spring", "Selection › Joints", ControlKind::list, K::select_connection),
                    number("spring.angular.stiffness", "Stiffness", "Inspector › Spring", K::set_spring_parameter, 0, 100000, 0.01, Q::twist_stiffness, EditCategory::parameter, true, NumberScale::logarithmic, 0.001, 10000),
                    number("spring.linear.damping", "Damping", "Inspector › Spring", K::set_spring_parameter, 0, 10000, 0.01, Q::damping, EditCategory::parameter, true, NumberScale::logarithmic, 0.001, 1000),
                    number("spring.linear.stiffness", "Stiffness", "Inspector › Spring", K::set_spring_parameter, 0, 100000, 0.01, Q::stiffness, EditCategory::parameter, true, NumberScale::logarithmic, 0.001, 10000),
                    plain("transport.back_to_start", "Back to start", "Toolbar", ControlKind::action, K::reset_scenario),
                    plain("transport.play", "Play", "Toolbar", ControlKind::action, K::toggle_pause),
                    plain("transport.step", "Step", "Toolbar", ControlKind::action, K::single_step),
                    plain("guide.predict.option", "Prediction", "Guide › Predict", ControlKind::radio_list, K::set_prediction),
                    plain("world.advanced.integration_method", "Integration method", "World › Advanced", ControlKind::select, K::set_integrator, EditCategory::lab, integrators),
                    plain("world.advanced.reuse_impulses", "Reuse contact impulses", "World › Advanced", ControlKind::switch_control, K::set_warm_starting, EditCategory::lab),
                    plain("world.advanced.solve_joints_together", "Solve joints together", "World › Advanced", ControlKind::switch_control, K::set_constraint_graph, EditCategory::lab),
                    plain("world.advanced.time_step", "Time step", "World › Advanced", ControlKind::select, K::set_fixed_step, EditCategory::lab, time_steps),
                    number("world.advanced.time_step_custom", "Custom time step", "World › Advanced", K::set_fixed_step, 0.0001, 0.02, 0.0001, Q::time, EditCategory::lab, false, NumberScale::linear, 0, 0, {}, true, true),
                    plain("world.air.resistance", "Air resistance", "World › Air", ControlKind::switch_control, K::set_drag_enabled, EditCategory::parameter),
                    number("world.air.density", "Air density", "World › Air", K::set_environment_parameter, 0, 10, 0.001, Q::air_density, EditCategory::parameter, true, NumberScale::linear, 0, 3, {}, false, false, false, "air_density_kg_m3"),
                    plain("world.air.spin_lift", "Spin lift", "World › Air", ControlKind::switch_control, K::set_magnus_enabled, EditCategory::parameter),
                    plain("world.air.spin_slowing", "Air slows spinning", "World › Air", ControlKind::switch_control, K::set_angular_drag_enabled, EditCategory::parameter),
                    number("world.air.viscosity", "Viscosity", "World › Air", K::set_environment_parameter, 1e-6, 1e-3, 1.1, Q::viscosity, EditCategory::parameter, false, NumberScale::logarithmic, 0, 0, {}, false, false, false, "air_dynamic_viscosity_pa_s"),
                    number("world.air.wind_x", "Wind x", "World › Air", K::set_environment_parameter, -30, 30, 0.1, Q::velocity, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, false, false, false, "air_velocity_x_m_s"),
                    number("world.air.wind_y", "Wind y", "World › Air", K::set_environment_parameter, -30, 30, 0.1, Q::velocity, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, false, false, false, "air_velocity_y_m_s"),
                    plain("world.collisions.bounce_rule", "Bounce rule", "World › Collisions", ControlKind::select, K::set_restitution_mixing, EditCategory::parameter, bounce_rules),
                    plain("world.collisions.catch_fast", "Catch fast objects", "World › Collisions", ControlKind::switch_control, K::set_continuous_collision, EditCategory::parameter),
                    plain("world.collisions.compare", "Compare bounce rules", "World › Collisions", ControlKind::action, K::compare_restitution),
                    plain("world.gravity.enabled", "Gravity", "World › Gravity", ControlKind::switch_control, K::set_gravity_enabled, EditCategory::parameter),
                    plain("world.gravity.preset", "Preset", "World › Gravity", ControlKind::segmented, K::set_gravity_preset, EditCategory::parameter, gravity_presets),
                    number("world.gravity.strength", "Strength", "World › Gravity", K::set_gravity_magnitude, 0, 100, 0.01, Q::acceleration, EditCategory::parameter, true, NumberScale::linear, 0, 30, gravity_detents),
                    number("world.gravity.tilt", "Tilt from straight down", "World › Gravity", K::set_gravity_angle_degrees, -180, 180, 0.1, Q::angle, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, false, false, true),
                    number("world.gravity.zero_height", "Zero height", "World › Gravity", K::set_energy_reference_height, -100, 100, 0.01, Q::length, EditCategory::parameter, false, NumberScale::linear, 0, 0, {}, true, true),
                };
                std::sort(value.begin(), value.end(), [](const auto& a, const auto& b)
                    {
                        return a.key < b.key;
                    });
                return value;
            }();
            return table;
        }
    }

    const ControlSpec* find_control_spec(std::string_view key)
    {
        const auto& values = registry();
        const auto found = std::lower_bound(values.begin(), values.end(), key, [](const ControlSpec& spec, std::string_view wanted)
            {
                return spec.key < wanted;
            });
        return found != values.end() && found->key == key ? &*found : nullptr;
    }

    math::Span<const ControlSpec> control_specs()
    {
        return registry();
    }

    bool valid_control_key(std::string_view key)
    {
        int dots = 0;
        bool segment_start = true;
        if (key.empty() || !std::islower(static_cast<unsigned char>(key.front())))
            return false;
        for (const auto c : key)
        {
            if (c == '.')
            {
                if (segment_start || ++dots > 2)
                    return false;
                segment_start = true;
            }
            else if (!(std::islower(static_cast<unsigned char>(c)) || std::isdigit(static_cast<unsigned char>(c)) || c == '_'))
                return false;
            else
                segment_start = false;
        }
        return !segment_start && dots >= 1;
    }

    std::string element_id(std::string_view host, std::string_view key, std::string_view instance)
    {
        std::string result(host);
        result.push_back('-');
        for (const auto c : key)
            result.push_back(c == '.' ? '-' : c);
        // Child parts are suffixed "--part"; a distinct instance separator keeps a second copy of a
        // control (key@instance) from ever matching another element's part.
        if (!instance.empty())
        {
            result += "@";
            for (const auto c : instance)
                result.push_back((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' ? c : '_');
        }
        return result;
    }

    std::string control_key(const UiCommand& command)
    {
        using K = UiCommandKind;
        switch (command.kind)
        {
        case K::toggle_pause:
            return "transport.play";
        case K::single_step:
            return "transport.step";
        case K::reset_scenario:
            return "transport.back_to_start";
        case K::set_time_scale:
            return "bar.speed.choice";
        case K::frame_subject:
            return "camera.frame.subject";
        case K::frame_all:
            return "camera.frame.everything";
        case K::frame_selection:
            return "camera.frame.selection";
        case K::set_selected_mass:
            return "object.properties.mass";
        case K::use_selected_density_mass:
            return "object.properties.use_density";
        case K::set_selected_material:
            return "object.properties.material";
        case K::set_selected_gravity_scale:
            return "object.properties.gravity_scale";
        case K::set_selected_velocity_x:
            return "object.motion.velocity_x";
        case K::set_selected_velocity_y:
            return "object.motion.velocity_y";
        case K::set_selected_angular_velocity:
            return "object.motion.spin";
        case K::set_selected_position:
            return command.detail == "y" ? "object.motion.position_y" : "object.motion.position_x";
        case K::set_selected_orientation:
            return "object.motion.orientation";
        case K::stop_selected_motion:
            return "object.motion.stop";
        case K::set_joint_motor_enabled:
            return "joint.motor.enabled";
        case K::set_joint_motor_speed:
            return "joint.motor.angular_speed";
        case K::reverse_joint_motor:
            return "joint.motor.reverse";
        case K::set_joint_limits_enabled:
            return "joint.stops.enabled";
        case K::set_gravity_enabled:
            return "world.gravity.enabled";
        case K::set_gravity_magnitude:
            return "world.gravity.strength";
        case K::set_gravity_angle_degrees:
            return "world.gravity.tilt";
        case K::set_gravity_preset:
            return "world.gravity.preset";
        case K::set_drag_enabled:
            return "world.air.resistance";
        case K::set_angular_drag_enabled:
            return "world.air.spin_slowing";
        case K::set_magnus_enabled:
            return "world.air.spin_lift";
        case K::set_continuous_collision:
            return "world.collisions.catch_fast";
        case K::set_warm_starting:
            return "world.advanced.reuse_impulses";
        case K::set_constraint_graph:
            return "world.advanced.solve_joints_together";
        case K::set_fixed_step:
            return "world.advanced.time_step";
        case K::set_vector_auto_scale:
            return "show.arrows.auto_length";
        case K::set_vector_scale:
            return "show.arrows.scale";
        case K::set_vector_scope:
            return "show.arrows.scope";
        case K::set_component_angle_degrees:
            return "show.arrows.direction";
        case K::set_layer_mask:
            return "show.presets.preset";
        case K::set_layer:
            return "show.layer.item";
        case K::set_display_units:
            return "prefs.units.system";
        case K::set_theme:
            return "prefs.appearance.theme";
        case K::set_ui_scale:
            return "prefs.appearance.text_size";
        case K::set_camera_zoom_sensitivity:
            return "prefs.controls.zoom_speed";
        case K::set_pause_on_impact:
            return "measure.collisions.pause_each";
        case K::pause_at_next_impact:
            return "measure.collisions.pause_next";
        case K::clear_energy_history:
            return "measure.graph.clear";
        case K::compare_integrators:
            return "measure.theory.integration_run";
        case K::compare_restitution:
            return "measure.theory.bounce_run";
        case K::compare_collisions:
            return "measure.theory.collisions_run";
        case K::select_impact:
            return "measure.collisions.list";
        case K::select_bodies:
            return "object.list.row";
        case K::select_connection:
            return command.detail == "spring" ? "spring.card" : "joint.card";
        case K::assemble_selected_bodies:
            return "object.several.combine";
        case K::set_spring_parameter:
            return command.detail == "damping" ? "spring.linear.damping" : "spring.linear.stiffness";
        case K::set_energy_reference_height:
            return "world.gravity.zero_height";
        case K::set_prediction:
            return "guide.predict.option";
        case K::star_run:
            return "measure.runs.row";
        case K::clear_runs:
            return "measure.runs.clear";
        case K::pin_run_value:
            return "measure.runs.add_value";
        case K::unpin_run_value:
            return "measure.runs.remove_value";
        case K::select_authored_part:
            return "object.shape.parts";
        case K::start_new_shape:
            return "tools.draw";
        case K::edit_selected_shape:
            return "object.shape.edit";
        case K::export_shape:
            return "object.shape.export";
        case K::split_selected_body:
            return "object.shape.separate";
        case K::set_shape_snap_grid:
            return "draw.snap.grid";
        case K::set_shape_snap_vertices:
            return "draw.snap.points";
        case K::set_shape_snap_angles:
            return "draw.snap.angles";
        case K::set_shape_grid_spacing:
            return "draw.snap.grid_spacing";
        case K::set_shape_vertex_budget:
            return "draw.precision.vertex_budget";
        case K::set_shape_render_tolerance:
            return "draw.precision.drawing";
        case K::set_shape_collision_tolerance:
            return "draw.precision.collision";
        case K::set_shape_simplification_tolerance:
            return "draw.precision.simplify";
        case K::set_shape_concavity_tolerance:
            return "draw.precision.hollows";
        case K::set_shape_edge:
            return "draw.node.edge";
        case K::set_shape_continuity:
            return "draw.node.join";
        case K::set_shape_material:
            return "draw.bar.material";
        case K::insert_shape_node:
            return "draw.node.insert";
        case K::remove_shape_node:
            return "draw.node.remove";
        case K::close_shape_outline:
            return "draw.actions.close";
        case K::commit_shape_outline:
            return "draw.actions.apply";
        case K::cancel_shape_outline:
            return "draw.actions.discard";
        case K::undo:
            return "bar.history.undo";
        case K::redo:
            return "bar.history.redo";
        case K::step_many:
            return "transport.step_many";
        case K::open_captures_folder:
            return "menu.capture.open_folder";
        case K::set_preference:
            return command.id;
        case K::restore_original:
            return "world.original.restore";
        case K::keep_state_as_setup:
            return "object.menu.keep_as_start";
        case K::revert_change:
            return "changes.item.revert";
        case K::revert_lab_settings:
            return "status.lab.revert";
        case K::set_interaction_mode:
            return command.id.empty() ? "tools.select" : std::string("tools.") + command.id;
        case K::load_scenario:
            return "library.cards.card";
        case K::none:
            if (command.detail.rfind("view:", 0) == 0)
                return "view." + command.detail.substr(5);
            return {};
        default:
            return "legacy.command.c" + std::to_string(static_cast<int>(command.kind));
        }
    }
}
