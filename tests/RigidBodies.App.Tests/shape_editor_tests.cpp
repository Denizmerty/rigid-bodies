#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/input_router.hpp>
#include <rigidbodies/app/stage_overlay_drawing.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace
{
    using namespace rigidbodies;
    using K = ui::UiCommandKind;

    void command(app::SimulationSession& session, K kind, double value = 0.0, bool flag = false, std::string id = {})
    {
        ui::UiCommand action;
        action.kind = kind;
        action.value = value;
        action.flag = flag;
        action.id = std::move(id);
        session.apply(action);
    }

    void setup(app::SimulationSession& session)
    {
        session.world().clear();
        auto settings = session.world().settings();
        settings.gravity_m_s2 = {};
        session.world().set_settings(settings);
        session.set_viewport({ 1000, 800 });
        session.camera().set_center({});
        session.camera().set_view_height(4.0);
    }

    bool pointer(app::SimulationSession& session, ui::UiEventKind kind, math::Vec2 world,
        bool consumed = false, ui::PointerButton button = ui::PointerButton::primary)
    {
        ui::UiEvent event;
        event.kind = kind;
        event.pointer_px = session.camera().world_to_screen(world);
        event.button = button;
        return session.handle_scene_event(event, consumed);
    }

    void click(app::SimulationSession& session, math::Vec2 world, bool consumed = false)
    {
        pointer(session, ui::UiEventKind::pointer_down, world, consumed);
        pointer(session, ui::UiEventKind::pointer_up, world, consumed);
    }

    bool key(app::SimulationSession& session, ui::UiKey value, bool consumed = false)
    {
        ui::UiEvent event;
        event.kind = ui::UiEventKind::key_down;
        event.key = value;
        return session.handle_scene_event(event, consumed);
    }

    void precise(app::SimulationSession& session)
    {
        command(session, K::set_shape_snap_grid, 0.0, false);
        command(session, K::set_shape_snap_vertices, 0.0, false);
    }

    void draw_square(app::SimulationSession& session, math::Vec2 offset = {})
    {
        command(session, K::start_new_shape);
        precise(session);
        for (const auto& position : { math::Vec2 { -0.5, -0.5 }, math::Vec2 { 0.5, -0.5 }, math::Vec2 { 0.5, 0.5 }, math::Vec2 { -0.5, 0.5 } })
            click(session, position + offset);
        key(session, ui::UiKey::enter);
    }

    std::shared_ptr<const physics::AuthoredShape> square_shape()
    {
        physics::Outline outline;
        outline.closed = true;
        for (const auto& position : { math::Vec2 { -0.5, -0.5 }, math::Vec2 { 0.5, -0.5 }, math::Vec2 { 0.5, 0.5 }, math::Vec2 { -0.5, 0.5 } })
        {
            physics::OutlineNode node;
            node.position_m = position;
            outline.nodes.push_back(node);
        }
        const auto result = physics::build_authored_shape(outline);
        RIGIDBODIES_EXPECT(result.succeeded(), "fixture square builds");
        return result.shape;
    }

    physics::BodyId authored_body(app::SimulationSession& session, math::Vec2 position = {}, physics::Material material = physics::materials::oak_wood())
    {
        physics::BodyDefinition definition;
        definition.position_m = position;
        definition.name = "Authored fixture";
        physics::AuthoredPartDefinition part;
        part.shape = square_shape();
        part.material = std::move(material);
        part.name = "Fixture part";
        return physics::create_authored_body(session.world(), definition, { part });
    }

    RIGIDBODIES_TEST("point drawing closes and commits one material-aware body then resumes the session")
    {
        app::SimulationSession session;
        setup(session);
        session.stepper().set_paused(false);
        draw_square(session);
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "draft editing pauses physics");
        RIGIDBODIES_EXPECT(session.world().body_ids().empty(), "draft points never create partial bodies");
        const auto before = session.world().statistics().elapsed_time_s;
        command(session, K::single_step);
        session.advance(1.0);
        RIGIDBODIES_EXPECT_NEAR(session.world().statistics().elapsed_time_s, before + session.stepper().fixed_step_s(), 1.0e-12, "Step advances the real simulation while a draft remains open");
        command(session, K::set_shape_material, 0.0, false, "steel");
        RIGIDBODIES_EXPECT(session.build_model().shape_material_name == "steel", "new shape material can be chosen");
        command(session, K::commit_shape_outline);
        RIGIDBODIES_EXPECT(!session.shape_editor().active() && session.stepper().is_paused(), "Apply never starts time");
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && session.world().is_valid(session.selection()), "commit creates and selects one body");
        const auto parts = physics::authored_parts(*session.world().find_body(session.selection()));
        RIGIDBODIES_EXPECT(parts.size() == 1 && parts.front()->material.name == "steel", "chosen material survives construction");
        RIGIDBODIES_EXPECT_NEAR(session.world().find_body(session.selection())->mass_properties().mass_kg, 7850.0 * 0.05, 1.0e-8, "authored area and material density determine mass");
    }

    RIGIDBODIES_TEST("pause intent survives cancellation and can be changed while editing")
    {
        app::SimulationSession session;
        setup(session);
        session.stepper().set_paused(true);
        draw_square(session);
        command(session, K::cancel_shape_outline);
        RIGIDBODIES_EXPECT(session.stepper().is_paused(), "original paused state is restored");
        command(session, K::start_new_shape);
        command(session, K::toggle_pause);
        RIGIDBODIES_EXPECT(!session.stepper().is_paused(), "Play starts the real simulation while the draft stays open");
        const auto before = session.world().statistics().elapsed_time_s;
        session.advance(0.05);
        RIGIDBODIES_EXPECT(session.world().statistics().elapsed_time_s > before && session.shape_editor().active(), "time runs without discarding or moving the draft");
        ui::UiEvent escape;
        escape.kind = ui::UiEventKind::key_down;
        escape.key = ui::UiKey::escape;
        app::InputContext context;
        context.draw_active = true;
        const auto routed = app::route_event(escape, {}, context);
        RIGIDBODIES_EXPECT(routed.escape == app::EscapeStep::draw_discard, "the input router assigns an empty draft to the Draw discard step");
        command(session, K::cancel_shape_outline);
        RIGIDBODIES_EXPECT(!session.stepper().is_paused() && !session.shape_editor().active(), "Discard leaves the current play state unchanged");
        RIGIDBODIES_EXPECT(session.world().body_ids().empty(), "cancel leaves no body");
    }

    RIGIDBODIES_TEST("scene Enter closes a three-point outline and Space dispatches play or pause once")
    {
        app::SimulationSession session;
        setup(session);
        command(session, K::start_new_shape);
        precise(session);
        click(session, { -0.5, -0.5 });
        click(session, { 0.5, -0.5 });
        click(session, { 0.0, 0.5 });
        RIGIDBODIES_EXPECT(!session.shape_editor().outline().closed, "three points start as an open outline");
        RIGIDBODIES_EXPECT(key(session, ui::UiKey::enter), "scene Enter is consumed by the draft editor");
        RIGIDBODIES_EXPECT(session.shape_editor().outline().closed, "scene Enter closes an outline with three points");

        command(session, K::cancel_shape_outline);
        session.stepper().set_paused(true);
        const auto play_pause = app::command_for_action(app::action_for_key(ui::UiKey::space, {}));
        RIGIDBODIES_EXPECT(play_pause.kind == K::toggle_pause, "the global Space binding dispatches Play or Pause");
        session.apply(play_pause);
        RIGIDBODIES_EXPECT(!session.stepper().is_paused(), "one Space command toggles the post-draft play intent exactly once");
    }

    RIGIDBODIES_TEST("invalid self intersections block commit without losing the editable draft")
    {
        app::SimulationSession session;
        setup(session);
        command(session, K::start_new_shape);
        precise(session);
        for (const auto& point : { math::Vec2 { 0.0, 0.0 }, math::Vec2 { 1.0, 1.0 }, math::Vec2 { 0.0, 1.0 }, math::Vec2 { 1.0, 0.0 } })
            click(session, point);
        key(session, ui::UiKey::enter);
        command(session, K::commit_shape_outline);
        const auto invalid = session.build_model();
        RIGIDBODIES_EXPECT(invalid.shape_editor_active && !invalid.shape_can_commit && invalid.shape_node_count == 4, "invalid geometry stays in the editor");
        RIGIDBODIES_EXPECT(!invalid.shape_diagnostic.empty() && session.world().body_ids().empty(), "invalid geometry has a diagnostic and no live collider");
        RIGIDBODIES_EXPECT(key(session, ui::UiKey::delete_key), "Delete removes only the selected draft vertex");
        RIGIDBODIES_EXPECT(session.build_model().shape_node_count == 3 && session.build_model().shape_can_commit, "removing the crossing corner repairs the draft");
        command(session, K::commit_shape_outline);
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1, "repaired draft commits normally");
    }

    RIGIDBODIES_TEST("panel clicks never add nodes and dragging releases reliably above a panel")
    {
        app::SimulationSession session;
        setup(session);
        draw_square(session);
        const auto count = session.shape_editor().outline().nodes.size();
        click(session, { 1.2, 1.2 }, true);
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.size() == count, "consumed panel press does not leak into the draft");
        pointer(session, ui::UiEventKind::pointer_down, { -0.5, -0.5 });
        RIGIDBODIES_EXPECT(session.shape_editor().has_pointer_capture(), "node press starts a drag");
        pointer(session, ui::UiEventKind::pointer_move, { -0.7, -0.6 }, true);
        pointer(session, ui::UiEventKind::pointer_up, { -0.7, -0.6 }, true);
        RIGIDBODIES_EXPECT(!session.shape_editor().has_pointer_capture(), "panel release clears the drag capture");
        const auto released = session.shape_editor().outline().nodes.front().position_m;
        pointer(session, ui::UiEventKind::pointer_move, { -1.0, -1.0 });
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.front().position_m == released, "later hover cannot keep moving the released node");
    }

    RIGIDBODIES_TEST("Delete cannot remove the selected body while a draft owns the keyboard")
    {
        app::SimulationSession session;
        setup(session);
        const auto id = authored_body(session);
        session.set_selection(id);
        command(session, K::edit_selected_shape);
        precise(session);
        click(session, { -0.5, -0.5 });
        RIGIDBODIES_EXPECT(key(session, ui::UiKey::delete_key), "the neutral Delete event is consumed before its app action");
        command(session, K::delete_selected_body);
        RIGIDBODIES_EXPECT(session.world().is_valid(id), "even a direct delete-body command cannot destroy an edited body");
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.size() == 3, "Delete was not dispatched to a second selected draft node");
        key(session, ui::UiKey::escape);
        RIGIDBODIES_EXPECT(physics::authored_parts(*session.world().find_body(id)).front()->shape->source.nodes.size() == 4, "cancel restores the unchanged committed outline");
    }

    RIGIDBODIES_TEST("grid snapping has a controlled spacing and can be disabled")
    {
        app::SimulationSession session;
        setup(session);
        command(session, K::start_new_shape);
        command(session, K::set_shape_snap_vertices, 0.0, false);
        command(session, K::set_shape_grid_spacing, 0.25);
        click(session, { 0.13, 0.12 });
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.front().position_m == math::Vec2(0.25, 0.0), "point rounds to the requested world grid");
        command(session, K::set_shape_snap_grid, 0.0, false);
        click(session, { 0.61, 0.43 });
        const auto free = session.shape_editor().outline().nodes.back().position_m;
        RIGIDBODIES_EXPECT_NEAR(free.x, 0.61, 1.0e-12, "grid toggle permits free x placement");
        RIGIDBODIES_EXPECT_NEAR(free.y, 0.43, 1.0e-12, "grid toggle permits free y placement");
    }

    RIGIDBODIES_TEST("existing world vertices take precedence over nearby grid positions")
    {
        app::SimulationSession session;
        setup(session);
        const auto id = authored_body(session, { -0.97, 0.02 });
        session.set_selection(id);
        command(session, K::start_new_shape);
        click(session, { -1.45, 0.54 });
        const auto point = session.shape_editor().outline().nodes.front().position_m;
        RIGIDBODIES_EXPECT_NEAR(point.x, -1.47, 1.0e-12, "nearby authored vertex supplies exact x");
        RIGIDBODIES_EXPECT_NEAR(point.y, 0.52, 1.0e-12, "nearby authored vertex supplies exact y");
        command(session, K::set_shape_snap_vertices, 0.0, false);
        click(session, { -0.45, 0.54 });
        RIGIDBODIES_EXPECT_NEAR(session.shape_editor().outline().nodes.back().position_m.y, 0.5, 1.0e-12, "vertex toggle falls back to grid snapping");
    }

    RIGIDBODIES_TEST("angle snapping uses fifteen degree increments and remains independently toggleable")
    {
        app::SimulationSession session;
        setup(session);
        command(session, K::start_new_shape);
        precise(session);
        command(session, K::set_shape_snap_angles, 0.0, true);
        click(session, {});
        click(session, { 0.96, 0.28 });
        const auto point = session.shape_editor().outline().nodes.back().position_m;
        RIGIDBODIES_EXPECT_NEAR(std::atan2(point.y, point.x), math::pi / 12.0, 1.0e-12, "edge direction snaps to fifteen degrees");
        command(session, K::set_shape_snap_angles, 0.0, false);
        click(session, { 1.31, 0.65 });
        RIGIDBODIES_EXPECT_NEAR(session.shape_editor().outline().nodes.back().position_m.x, 1.31, 1.0e-12, "angle snap can be disabled without affecting other modes");
    }

    RIGIDBODIES_TEST("closed edges can be selected inserted converted to cubic and removed")
    {
        app::SimulationSession session;
        setup(session);
        draw_square(session);
        click(session, { 0.0, -0.5 });
        RIGIDBODIES_EXPECT(session.shape_editor().selected_node() == std::optional<std::size_t> { 0 }, "edge hit selects its start node");
        command(session, K::insert_shape_node);
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.size() == 5, "insert creates one node after the selected edge start");
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes[1].position_m == math::Vec2(0.0, -0.5), "line insertion uses the midpoint");
        command(session, K::set_shape_edge, 0.0, false, "curved");
        RIGIDBODIES_EXPECT(session.build_model().shape_selected_edge_cubic, "selected outgoing edge becomes cubic");
        RIGIDBODIES_EXPECT(math::length(session.shape_editor().outline().nodes[1].outgoing_handle_m) > 0.0, "conversion creates usable handles");
        command(session, K::set_shape_edge, 0.0, false, "straight");
        command(session, K::remove_shape_node);
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.size() == 4 && session.build_model().shape_can_commit, "removing inserted node restores a valid quadrilateral");
    }

    RIGIDBODIES_TEST("dragging both cubic handles obeys mirrored aligned and corner continuity")
    {
        app::SimulationSession session;
        setup(session);
        draw_square(session);
        click(session, { -0.5, 0.5 });
        command(session, K::set_shape_edge, 0.0, false, "curved");
        click(session, { -0.5, -0.5 });
        command(session, K::set_shape_edge, 0.0, false, "curved");
        command(session, K::set_shape_continuity, 0.0, false, "symmetric");
        const auto drag_handle = [&](bool incoming, math::Vec2 offset)
        {
            const auto node = session.shape_editor().outline().nodes.front();
            const auto original = node.position_m + (incoming ? node.incoming_handle_m : node.outgoing_handle_m);
            pointer(session, ui::UiEventKind::pointer_down, original);
            pointer(session, ui::UiEventKind::pointer_move, node.position_m + offset);
            pointer(session, ui::UiEventKind::pointer_up, node.position_m + offset);
        };
        drag_handle(false, { 0.3, -0.2 });
        auto node = session.shape_editor().outline().nodes.front();
        RIGIDBODIES_EXPECT_NEAR(math::length(node.incoming_handle_m + node.outgoing_handle_m), 0.0, 1.0e-12, "mirrored drag keeps opposite equal handles");
        RIGIDBODIES_EXPECT_NEAR(node.outgoing_handle_m.y, -0.2, 1.0e-12, "outgoing handle follows the pointer");
        command(session, K::set_shape_continuity, 0.0, false, "smooth");
        const auto incoming_length = math::length(session.shape_editor().outline().nodes.front().incoming_handle_m);
        drag_handle(false, { 0.45, 0.1 });
        node = session.shape_editor().outline().nodes.front();
        RIGIDBODIES_EXPECT_NEAR(math::cross(node.incoming_handle_m, node.outgoing_handle_m), 0.0, 1.0e-12, "aligned drag preserves a common tangent line");
        RIGIDBODIES_EXPECT_NEAR(math::length(node.incoming_handle_m), incoming_length, 1.0e-12, "aligned opposite handle preserves its own length");
        command(session, K::set_shape_continuity, 0.0, false, "corner");
        const auto outgoing = session.shape_editor().outline().nodes.front().outgoing_handle_m;
        drag_handle(true, { -0.2, 0.3 });
        node = session.shape_editor().outline().nodes.front();
        RIGIDBODIES_EXPECT(node.outgoing_handle_m == outgoing, "corner incoming edit leaves outgoing handle independent");
        RIGIDBODIES_EXPECT_NEAR(node.incoming_handle_m.y, 0.3, 1.0e-12, "incoming handle follows the pointer");
    }

    RIGIDBODIES_TEST("collision budgets and distinct tolerances validate without discarding source geometry")
    {
        app::SimulationSession session;
        setup(session);
        draw_square(session);
        command(session, K::set_shape_render_tolerance, 0.001);
        command(session, K::set_shape_collision_tolerance, 0.02);
        command(session, K::set_shape_simplification_tolerance, 0.003);
        command(session, K::set_shape_concavity_tolerance, 0.01);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT_NEAR(model.shape_render_tolerance_m, 0.001, 0.0, "render tolerance is independent");
        RIGIDBODIES_EXPECT_NEAR(model.shape_collision_tolerance_m, 0.02, 0.0, "collision tolerance is independent");
        RIGIDBODIES_EXPECT_NEAR(model.shape_simplification_tolerance_m, 0.003, 0.0, "simplification has its own error limit");
        RIGIDBODIES_EXPECT_NEAR(model.shape_concavity_tolerance_m, 0.01, 0.0, "concavity tolerance is explicit");
        command(session, K::set_shape_vertex_budget, 3.0);
        command(session, K::commit_shape_outline);
        RIGIDBODIES_EXPECT(session.shape_editor().active() && !session.build_model().shape_can_commit, "insufficient budget blocks commit");
        RIGIDBODIES_EXPECT(session.shape_editor().outline().nodes.size() == 4, "budget failure preserves the authored nodes");
        command(session, K::set_shape_vertex_budget, 128.0);
        RIGIDBODIES_EXPECT(session.build_model().shape_can_commit, "raising budget repairs build without redrawing");
        command(session, K::set_shape_vertex_budget, 513.0);
        RIGIDBODIES_EXPECT(session.shape_editor().options().max_collision_vertices == 128 && !session.shape_editor().diagnostic().empty(), "budget beyond pipeline limits is rejected before changing settings");
        command(session, K::set_shape_collision_tolerance, std::numeric_limits<double>::quiet_NaN());
        RIGIDBODIES_EXPECT_NEAR(session.shape_editor().options().collision_tolerance_m, 0.02, 0.0, "invalid numeric input does not poison draft settings");
    }

    RIGIDBODIES_TEST("editing a rotated authored part preserves placement material depth and body identity")
    {
        app::SimulationSession session;
        setup(session);
        physics::BodyDefinition body;
        body.position_m = { -0.8, 0.2 };
        body.orientation_rad = 0.3;
        physics::AuthoredPartDefinition part;
        part.shape = square_shape();
        part.local_transform = math::Transform2::from_angle({ 0.4, 0.2 }, 0.4);
        part.material = physics::materials::rubber();
        part.depth_m = 0.12;
        part.name = "Rubber part";
        const auto id = physics::create_authored_body(session.world(), body, { part });
        session.set_selection(id);
        const auto placement = math::concatenate(session.world().find_body(id)->transform(), part.local_transform);
        command(session, K::edit_selected_shape);
        precise(session);
        command(session, K::set_shape_material, 0.0, false, "steel");
        RIGIDBODIES_EXPECT(!session.build_model().shape_can_change_material && session.build_model().shape_material_name == "rubber", "editing keeps the original material");
        pointer(session, ui::UiEventKind::pointer_down, math::transform_point(placement, { -0.5, -0.5 }));
        pointer(session, ui::UiEventKind::pointer_move, math::transform_point(placement, { -0.7, -0.5 }));
        pointer(session, ui::UiEventKind::pointer_up, math::transform_point(placement, { -0.7, -0.5 }));
        command(session, K::commit_shape_outline);
        RIGIDBODIES_EXPECT(!session.shape_editor().active() && session.selection() == id, "edit keeps the stable body identity");
        const auto changed = physics::authored_parts(*session.world().find_body(id)).front();
        RIGIDBODIES_EXPECT_NEAR(changed->shape->source.nodes.front().position_m.x, -0.7, 1.0e-12, "drag is stored in original part-local coordinates");
        RIGIDBODIES_EXPECT(changed->local_transform.translation == part.local_transform.translation && changed->material.name == "rubber", "local placement and material survive edit");
        RIGIDBODIES_EXPECT_NEAR(changed->depth_m, 0.12, 0.0, "physical depth survives edit");
    }

    RIGIDBODIES_TEST("compound part selection edits one logical outline without splitting collision cells")
    {
        app::SimulationSession session;
        setup(session);
        physics::AuthoredPartDefinition first;
        first.shape = square_shape();
        first.local_transform.translation = { -0.75, 0.0 };
        first.name = "Left wood";
        auto second = first;
        second.local_transform.translation = { 0.75, 0.0 };
        second.material = physics::materials::steel();
        second.name = "Right steel";
        const auto id = physics::create_authored_body(session.world(), {}, { first, second });
        session.set_selection(id);
        command(session, K::select_authored_part, 1.0);
        RIGIDBODIES_EXPECT(session.build_model().authored_part_index == 1 && session.build_model().authored_material_name == "steel", "part picker advances in logical insertion order");
        command(session, K::edit_selected_shape);
        precise(session);
        pointer(session, ui::UiEventKind::pointer_down, { 1.25, 0.5 });
        pointer(session, ui::UiEventKind::pointer_move, { 1.35, 0.6 });
        pointer(session, ui::UiEventKind::pointer_up, { 1.35, 0.6 });
        command(session, K::commit_shape_outline);
        const auto parts = physics::authored_parts(*session.world().find_body(id));
        RIGIDBODIES_EXPECT(parts.size() == 2 && session.world().body_ids().size() == 1, "edited assembly remains one body with two logical parts");
        RIGIDBODIES_EXPECT(parts.front()->shape->source.nodes[2].position_m == math::Vec2(0.5, 0.5), "the other part retains its geometry");
        RIGIDBODIES_EXPECT_NEAR(parts.back()->shape->source.nodes[2].position_m.x, 0.6, 1.0e-12, "selected part receives the geometry edit");
    }

    RIGIDBODIES_TEST("editing a compound part can snap to another part on the same body")
    {
        app::SimulationSession session;
        setup(session);
        physics::AuthoredPartDefinition first;
        first.shape = square_shape();
        first.local_transform.translation = { -0.75, 0.0 };
        auto second = first;
        second.local_transform.translation = { 0.75, 0.0 };
        const auto id = physics::create_authored_body(session.world(), {}, { first, second });
        session.set_selection(id);
        command(session, K::edit_selected_shape);
        command(session, K::set_shape_snap_grid, 0.0, false);
        command(session, K::set_shape_snap_vertices, 0.0, true);
        pointer(session, ui::UiEventKind::pointer_down, { -1.25, -0.5 });
        pointer(session, ui::UiEventKind::pointer_move, { 0.27, -0.48 });
        pointer(session, ui::UiEventKind::pointer_up, { 0.27, -0.48 });
        const auto changed = session.shape_editor().outline().nodes.front().position_m;
        RIGIDBODIES_EXPECT_NEAR(changed.x, 1.0, 1.0e-12, "other part's exact world vertex becomes edited part-local x");
        RIGIDBODIES_EXPECT_NEAR(changed.y, -0.5, 1.0e-12, "same-body part vertex remains a valid snapping target");
        const auto original = physics::authored_parts(*session.world().find_body(id)).front()->shape->source.nodes.front().position_m;
        RIGIDBODIES_EXPECT(original == math::Vec2(-0.5, -0.5), "snapping a draft does not change committed compound geometry");
    }

    RIGIDBODIES_TEST("multi-selection assembly and splitting preserve authored parts and material identity")
    {
        app::SimulationSession session;
        setup(session);
        const auto first = authored_body(session, { -0.8, 0.0 });
        const auto second = authored_body(session, { 0.8, 0.0 }, physics::materials::steel());
        session.set_selections({ first, second });
        RIGIDBODIES_EXPECT(session.build_model().can_assemble_shapes, "two selected bodies can be assembled");
        command(session, K::assemble_selected_bodies);
        const auto combined = session.selection();
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && session.build_model().authored_part_count == 2, "two bodies combine into two logical parts on one body");
        RIGIDBODIES_EXPECT(!session.world().is_valid(first) && !session.world().is_valid(second), "source bodies are replaced by the assembly");
        const auto parts = physics::authored_parts(*session.world().find_body(combined));
        RIGIDBODIES_EXPECT(parts.front()->material.name == "oak_wood" && parts.back()->material.name == "steel", "assembly retains each material");
        command(session, K::split_selected_body);
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 2 && !session.world().is_valid(combined), "split restores two complete logical bodies");
        for (const auto id : session.world().body_ids())
            RIGIDBODIES_EXPECT(physics::authored_parts(*session.world().find_body(id)).size() == 1, "collision cells never become separate split bodies");
    }

    RIGIDBODIES_TEST("invalid assembly selections fail without losing bodies")
    {
        app::SimulationSession session;
        setup(session);
        const auto first = authored_body(session);
        session.set_selection(first);
        command(session, K::assemble_selected_bodies);
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && !session.build_model().notifications.empty(), "single-body assembly reports an actionable error");
        const auto second = authored_body(session, { 2.0, 0.0 });
        session.world().destroy_body(first);
        session.set_selection(second);
        command(session, K::assemble_selected_bodies);
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == 1 && session.world().is_valid(second), "an invalid multi-selection cannot consume the selected body");
    }

    RIGIDBODIES_TEST("Back to start is disabled at Ready while a successful load discards drafts")
    {
        app::SimulationSession session;
        setup(session);
        RIGIDBODIES_EXPECT(session.load_scenario(physics::default_scenario_id()), "default scenario exists");
        const auto initial_count = session.world().body_ids().size();
        draw_square(session);
        pointer(session, ui::UiEventKind::pointer_down, { -0.5, -0.5 });
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.shape_editor().active(), "Back to start is a no-op while already Ready");
        RIGIDBODIES_EXPECT(session.world().body_ids().size() == initial_count, "the draft has not changed world geometry");
        RIGIDBODIES_EXPECT(!session.load_scenario("missing_shape_scene") && session.shape_editor().active(), "failed scene lookup leaves current draft intact");
        RIGIDBODIES_EXPECT(session.load_scenario("revolute_drive") && session.shape_editor().active() && session.build_model().confirmation.has_value(), "scene switch asks before discarding the draft");
        ui::UiCommand confirm;
        confirm.kind = K::load_scenario;
        confirm.id = "revolute_drive";
        confirm.flag = true;
        session.apply(confirm);
        RIGIDBODIES_EXPECT(!session.shape_editor().active() && !session.shape_editor().has_pointer_capture(), "confirming the scene switch discards the draft and capture");
    }

    RIGIDBODIES_TEST("editor overlay uses cached geometry and cyan collision cells without mutating the world")
    {
        app::SimulationSession session;
        setup(session);
        draw_square(session);
        const auto cached = session.shape_editor().build().shape;
        render::DrawList list;
        session.scene_settings().layers = render::LayerMask::none();
        session.render(list);
        bool fill = false;
        bool collision_outline = false;
        for (const auto& draw : list.commands())
        {
            // The fill is one seamless mesh with exactly one triangle per cached render triangle.
            fill = fill || (draw.kind == render::DrawCommandKind::indexed_mesh && draw.mesh && draw.mesh->indices.size() == cached->render_triangles.size() * 3);
            collision_outline = collision_outline || (draw.kind == render::DrawCommandKind::polygon_outline && draw.color.blue == session.scene_settings().theme.velocity.blue && draw.color.green == session.scene_settings().theme.velocity.green);
        }
        RIGIDBODIES_EXPECT(fill && collision_outline, "preview distinguishes filled visual shape from cyan collision cells");
        for (const auto& vertex : list.vertices())
            RIGIDBODIES_EXPECT(math::is_finite(vertex), "editor preview vertices remain finite");
        (void)session.build_model();
        session.render(list);
        RIGIDBODIES_EXPECT(session.shape_editor().build().shape == cached, "unchanged drawing and model reuse one authored geometry cache");
        RIGIDBODIES_EXPECT(session.world().body_ids().empty(), "preview leaves the simulation world unchanged");
    }

    RIGIDBODIES_TEST("view panning also releases over interface space and editor wheel zoom stays available")
    {
        app::SimulationSession session;
        setup(session);
        command(session, K::start_new_shape);
        pointer(session, ui::UiEventKind::pointer_down, {}, false, ui::PointerButton::middle);
        ui::UiEvent move;
        move.kind = ui::UiEventKind::pointer_move;
        move.pointer_delta_px = { 40.0, -20.0 };
        session.handle_scene_event(move, true);
        pointer(session, ui::UiEventKind::pointer_up, {}, true, ui::PointerButton::middle);
        const auto center = session.camera().center_m();
        session.handle_scene_event(move, false);
        RIGIDBODIES_EXPECT(session.camera().center_m() == center, "releasing middle drag over a panel ends camera capture");
        ui::UiEvent wheel;
        wheel.kind = ui::UiEventKind::wheel;
        wheel.wheel_delta = 1.0;
        wheel.pointer_px = { 500.0, 400.0 };
        const auto height = session.camera().view_height_m();
        session.handle_scene_event(wheel, true);
        RIGIDBODIES_EXPECT_NEAR(session.camera().view_height_m(), height, 0.0, "panel wheel never zooms the underlying scene");
        session.handle_scene_event(wheel);
        RIGIDBODIES_EXPECT(session.camera().view_height_m() < height, "scene wheel zoom remains available during drafting");
    }

    RIGIDBODIES_TEST("right drag pans only after four logical pixels and Space has no drag modifier effect")
    {
        app::SimulationSession session;
        setup(session);
        const auto original_center = session.camera().center_m();
        const auto send = [&](ui::UiEventKind kind, ui::PointerButton button, math::Vec2 position, math::Vec2 delta = {}, double scale = 1.0)
        {
            ui::UiEvent event;
            event.kind = kind;
            event.button = button;
            event.pointer_px = position;
            event.pointer_delta_px = delta;
            event.logical_pixel_scale = scale;
            return session.handle_scene_event(event);
        };

        RIGIDBODIES_EXPECT(send(ui::UiEventKind::pointer_down, ui::PointerButton::secondary, { 100.0, 100.0 }), "a secondary press starts threshold tracking");
        RIGIDBODIES_EXPECT(send(ui::UiEventKind::pointer_move, ui::PointerButton::secondary, { 104.5, 100.0 }, { 4.5, 0.0 }, 1.5), "a three-logical-pixel secondary move remains captured");
        send(ui::UiEventKind::pointer_up, ui::PointerButton::secondary, { 104.5, 100.0 }, {}, 1.5);
        RIGIDBODIES_EXPECT(session.camera().center_m() == original_center, "a right drag of three logical pixels does not pan");

        send(ui::UiEventKind::pointer_down, ui::PointerButton::secondary, { 100.0, 100.0 });
        send(ui::UiEventKind::pointer_move, ui::PointerButton::secondary, { 106.0, 100.0 }, { 6.0, 0.0 }, 1.5);
        send(ui::UiEventKind::pointer_up, ui::PointerButton::secondary, { 106.0, 100.0 }, {}, 1.5);
        RIGIDBODIES_EXPECT(!(session.camera().center_m() == original_center), "a right drag of four logical pixels pans");

        const auto after_right_pan = session.camera().center_m();
        send(ui::UiEventKind::pointer_down, ui::PointerButton::primary, { 700.0, 500.0 });
        send(ui::UiEventKind::pointer_move, ui::PointerButton::primary, { 740.0, 500.0 }, { 40.0, 0.0 });
        send(ui::UiEventKind::pointer_up, ui::PointerButton::primary, { 740.0, 500.0 });
        RIGIDBODIES_EXPECT(session.camera().center_m() == after_right_pan, "a left drag on empty space does not inherit a Space pan modifier");
    }

    const render::DrawCommand* drawn_text(const render::DrawList& list, std::string_view text)
    {
        for (const auto& draw : list.commands())
            if (draw.kind == render::DrawCommandKind::text && std::string_view(list.text_buffer()).substr(draw.text_offset, draw.text_length) == text)
                return &draw;
        return nullptr;
    }

    RIGIDBODIES_TEST("invalid outlines are flagged on the stage with a short reason that clears once repaired")
    {
        app::SimulationSession session;
        setup(session);
        session.scene_settings().layers = render::LayerMask::none();
        command(session, K::start_new_shape);
        precise(session);
        for (const auto& point : { math::Vec2 { 0.0, 0.0 }, math::Vec2 { 1.0, 1.0 }, math::Vec2 { 0.0, 1.0 }, math::Vec2 { 1.0, 0.0 } })
            click(session, point);
        key(session, ui::UiKey::enter);
        render::DrawList list;
        session.render(list);
        const auto* reason = drawn_text(list, "Edges cross");
        RIGIDBODIES_EXPECT(reason != nullptr && reason->text_background.has_value(), "a crossing outline names its problem on a readable plate");
        RIGIDBODIES_EXPECT(reason->color.red > reason->color.blue && reason->color.red > reason->color.green, "the reason uses the danger colour rather than the editing accent");
        key(session, ui::UiKey::delete_key);
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Edges cross") == nullptr && session.shape_editor().build().succeeded(), "a repaired outline drops the warning");
    }

    RIGIDBODIES_TEST("the first node offers a closing cue only where a click would close the outline")
    {
        app::SimulationSession session;
        setup(session);
        session.scene_settings().layers = render::LayerMask::none();
        command(session, K::start_new_shape);
        precise(session);
        for (const auto& point : { math::Vec2 { -0.5, -0.5 }, math::Vec2 { 0.5, -0.5 }, math::Vec2 { 0.5, 0.5 } })
            click(session, point);
        render::DrawList list;
        pointer(session, ui::UiEventKind::pointer_move, { 0.0, 0.6 });
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Close shape") == nullptr, "no closing cue away from the first node");
        const math::Vec2 near_first { -0.5 + 0.03, -0.5 + 0.02 };
        pointer(session, ui::UiEventKind::pointer_move, near_first);
        session.render(list);
        RIGIDBODIES_EXPECT(drawn_text(list, "Close shape") != nullptr, "hovering the first node of a closable outline shows the cue");
        click(session, near_first);
        RIGIDBODIES_EXPECT(session.shape_editor().outline().closed && session.shape_editor().outline().nodes.size() == 3, "clicking where the cue appeared closes the outline");
    }

    RIGIDBODIES_TEST("an open draft that can close previews its closing edge and a faint fill")
    {
        app::SimulationSession session;
        setup(session);
        session.scene_settings().layers = render::LayerMask::none();
        command(session, K::start_new_shape);
        precise(session);
        const auto count_meshes = [&](const render::DrawList& list, bool fills)
        {
            std::size_t found = 0;
            for (const auto& draw : list.commands())
                if (draw.layer == app::overlay::overlay_layer && draw.kind == render::DrawCommandKind::indexed_mesh && draw.mesh && !draw.mesh->indices.empty())
                {
                    // Fills list each triangle's corners in turn; dashes are four-corner quads.
                    const auto fill = draw.mesh->vertices.size() == draw.mesh->indices.size();
                    found += fill == fills ? 1 : 0;
                }
            return found;
        };
        click(session, { -0.5, -0.5 });
        click(session, { 0.5, -0.5 });
        pointer(session, ui::UiEventKind::pointer_move, { 0.0, 0.9 });
        render::DrawList list;
        session.render(list);
        RIGIDBODIES_EXPECT(count_meshes(list, true) == 0 && count_meshes(list, false) == 1, "two nodes show only the line towards the pointer");
        click(session, { 0.5, 0.5 });
        pointer(session, ui::UiEventKind::pointer_move, { 0.0, 0.9 });
        session.render(list);
        RIGIDBODIES_EXPECT(count_meshes(list, true) == 1, "three nodes preview the shape Enter would create");
        RIGIDBODIES_EXPECT(count_meshes(list, false) == 2, "the implied closing edge is drawn beside the line towards the pointer");
        RIGIDBODIES_EXPECT(!session.shape_editor().outline().closed, "previewing does not close the draft");
    }

    RIGIDBODIES_TEST("editor handles and their pointer targets keep their proportions at every display scale")
    {
        for (const auto scale : { 1.0, 1.5, 2.0 })
        {
            app::SimulationSession session;
            setup(session);
            session.scene_settings().layers = render::LayerMask::none();
            session.scene_settings().display_scale = static_cast<float>(scale);
            draw_square(session);
            pointer(session, ui::UiEventKind::pointer_move, { 0.0, 1.5 });
            render::DrawList list;
            session.render(list);
            std::size_t handles = 0;
            for (const auto& draw : list.commands())
                if (draw.kind == render::DrawCommandKind::rectangle_fill && draw.vertex_count == 2)
                {
                    const auto size = list.vertices()[draw.vertex_offset + 1] - list.vertices()[draw.vertex_offset];
                    handles += std::abs(size.x - 8.0 * scale) < 1.0e-9 && std::abs(size.y - 8.0 * scale) < 1.0e-9 ? 1 : 0;
                }
            RIGIDBODIES_EXPECT(handles == 4, "every vertex handle is eight logical pixels square");

            const auto press = [&](math::Vec2 offset_px)
            {
                ui::UiEvent event;
                event.kind = ui::UiEventKind::pointer_down;
                event.pointer_px = session.camera().world_to_screen({ -0.5, -0.5 }) + offset_px * scale;
                event.logical_pixel_scale = scale;
                session.handle_scene_event(event);
                event.kind = ui::UiEventKind::pointer_up;
                session.handle_scene_event(event);
            };
            press({ -11.0, 0.0 });
            RIGIDBODIES_EXPECT(session.shape_editor().selected_node() == std::optional<std::size_t> { 0 }, "a press within twelve logical pixels selects the node");
            press({ -14.0, 0.0 });
            RIGIDBODIES_EXPECT(!session.shape_editor().selected_node().has_value(), "a press beyond the target and the edge band selects nothing");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
