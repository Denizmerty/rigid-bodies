#include "learner_interface_harness.hpp"

#include <rigidbodies/physics/shape.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/measure_tabs.hpp>

#include <algorithm>
#include <array>
#include <set>

namespace
{
    using namespace rigidbodies;
    using testing::LearnerInterface;

    std::string at(render::ViewportSize size, float scale)
    {
        return " at " + std::to_string(size.width) + "x" + std::to_string(size.height) + " and " + std::to_string(scale) + "x";
    }

    bool inside(const ui::Rect& inner, const ui::Rect& outer, double tolerance = 1.0)
    {
        return inner.minimum.x >= outer.minimum.x - tolerance && inner.minimum.y >= outer.minimum.y - tolerance && inner.maximum.x <= outer.maximum.x + tolerance && inner.maximum.y <= outer.maximum.y + tolerance;
    }

    ui::Rect region_bounds(const LearnerInterface& harness, ui::RegionId id)
    {
        const auto* region = harness.interface.layout().find(id);
        RIGIDBODIES_EXPECT(region != nullptr, "the layout publishes the region");
        return region->bounds;
    }

    std::string control(std::string_view key, std::string_view instance = {})
    {
        return ui::element_id("legacy", key, instance);
    }

    // The vertical extent of an open dropdown below its field, found the way a pointer finds it.
    std::optional<std::pair<double, double>> dropdown_extent(const LearnerInterface& harness, const std::string& field)
    {
        const auto box = harness.visible_bounds(field);
        if (!box)
            return std::nullopt;
        const auto x = (box->minimum.x + box->maximum.x) * 0.5;
        double top = -1.0, bottom = -1.0;
        // Start past the field's own rounded border, which can reach the next pixel row.
        for (auto y = std::ceil(box->maximum.y) + 2.0; y < static_cast<double>(harness.viewport().height); y += 1.0)
        {
            const auto inside_list = harness.document().element_at({ x, y }) == field;
            if (inside_list && top < 0.0)
                top = y;
            if (!inside_list && top >= 0.0)
            {
                bottom = y;
                break;
            }
        }
        if (top < 0.0 || bottom <= top)
            return std::nullopt;
        return std::pair { top, bottom };
    }

    RIGIDBODIES_TEST("a dropdown is chosen with the pointer, and a press elsewhere only closes it")
    {
        LearnerInterface harness;
        const auto field = control("bar.speed.choice") + "--field";
        const auto& options = ui::find_control_spec("bar.speed.choice")->options;
        const auto choose = [&](std::size_t index)
        {
            RIGIDBODIES_EXPECT(harness.click_element(field), "the speed choice is displayed");
            harness.frames_for(2);
            const auto list = dropdown_extent(harness, field);
            RIGIDBODIES_EXPECT(list.has_value(), "pressing the speed choice opens its list");
            const auto x = (harness.visible_bounds(field)->minimum.x + harness.visible_bounds(field)->maximum.x) * 0.5;
            const auto row_height = (list->second - list->first) / static_cast<double>(options.size());
            const ui::Vec2 option { x, list->first + row_height * (static_cast<double>(index) + 0.5) };
            RIGIDBODIES_EXPECT(harness.interface.surface_at(option), "the open list is an interface surface, not the stage");
            harness.click(option);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!dropdown_extent(harness, field), "choosing an option closes the list");
        };

        choose(2);
        RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, std::stod(std::string(options[2].id)), 1e-12, "the chosen speed becomes the playback speed");
        choose(options.size() - 1);
        RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, std::stod(std::string(options[2].id)), 1e-12, "opening Custom keeps the current speed until an edit is confirmed");
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-playback_speed") && harness.document().focused_element() == control("bar.speed.custom") + "--field", "Custom opens the exact speed editor ready for typing");
        harness.key(ui::UiKey::escape);
        harness.frame();
        harness.key(ui::UiKey::escape);
        harness.frame();

        physics::BodyId target;
        for (const auto id : harness.session.world().body_ids())
            if (const auto* body = harness.session.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                target = id;
        const auto on_body = harness.session.camera().world_to_screen(harness.session.world().find_body(target)->world_center_of_mass_m());
        RIGIDBODIES_EXPECT(harness.click_element(field), "the speed choice can be opened again");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(dropdown_extent(harness, field).has_value(), "the list is open");
        harness.click(on_body);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!dropdown_extent(harness, field), "a press elsewhere closes the list");
        RIGIDBODIES_EXPECT(harness.session.build_model().selected_bodies.empty(), "the press that closes the list does not reach the object beneath it");
        harness.click(on_body);
        RIGIDBODIES_EXPECT(harness.session.selection() == target, "with the list closed the stage takes presses again");
        const auto paused = harness.session.build_model().paused;
        harness.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(harness.session.build_model().paused != paused, "Space reaches the simulation afterwards");
    }

    RIGIDBODIES_TEST("custom playback speed validates exact values and remains discoverable at every width")
    {
        for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 1600, 900 }, 1.0f }, { { 560, 320 }, 1.0f }, { { 1120, 640 }, 2.0f } })
        {
            LearnerInterface harness({}, size, scale);
            const auto undo_before = harness.session.build_model().undo_label;
            // This is the same reveal path used by a Command search result, even when the speed
            // dropdown has moved into the toolbar's overflow menu.
            harness.interface.reveal("bar.speed.choice");
            harness.frames_for(2);
            const auto field = control("bar.speed.custom") + "--field";
            RIGIDBODIES_EXPECT(harness.document().focused_element() == field && harness.visible_bounds(field), "the editor opens and takes keyboard focus" + at(size, scale));
            harness.key(ui::UiKey::a, { false, true, false });
            harness.text("0.75");
            harness.key(ui::UiKey::enter);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, 0.75, 1e-12, "an exact custom playback multiplier reaches the session");
            RIGIDBODIES_EXPECT(harness.session.build_model().undo_label == undo_before, "changing playback speed does not add an experiment edit to undo history");
            harness.key(ui::UiKey::a, { false, true, false });
            harness.text("8");
            harness.key(ui::UiKey::enter);
            harness.frame();
            RIGIDBODIES_EXPECT(harness.document().element_has_class(field, "is-invalid"), "an out-of-range speed shows a validation error");
            RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, 0.75, 1e-12, "invalid input preserves the last accepted speed");
            harness.key(ui::UiKey::escape);
            harness.frame();
            RIGIDBODIES_EXPECT(harness.document().element_visible("panel-playback_speed"), "Escape cancels the input before closing the editor");
            RIGIDBODIES_EXPECT(harness.click_element(control("playback.normal")), "the real-time reset is reachable");
            harness.frames_for(2);
            RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, 1.0, 1e-12, "reset returns playback to real time");
            harness.key(ui::UiKey::escape);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.document().element_visible("panel-playback_speed"), "Escape closes the editor once editing has ended");
        }
    }

    RIGIDBODIES_TEST("custom speed scrubbing stays live and cancels without changing edit history")
    {
        for (const bool lose_focus : { false, true })
        {
            LearnerInterface harness;
            harness.interface.reveal("bar.speed.custom");
            harness.frames_for(2);
            harness.session.stepper().set_paused(false);
            const auto original = harness.session.build_model().time_scale;
            const auto history_size = harness.session.build_model().undo_history.size();
            const auto label = harness.visible_bounds(control("bar.speed.custom") + "--label");
            RIGIDBODIES_EXPECT(label.has_value(), "the custom speed supports label scrubbing");
            const auto origin = (label->minimum + label->maximum) * 0.5;
            const auto moved = origin + ui::Vec2 { 24, 0 };
            harness.pointer(ui::UiEventKind::pointer_down, origin);
            harness.pointer(ui::UiEventKind::pointer_move, moved);
            RIGIDBODIES_EXPECT(harness.session.build_model().time_scale != original, "the speed changes during the preview");
            const auto time_before = harness.session.world().statistics().elapsed_time_s;
            harness.frame(0.1);
            RIGIDBODIES_EXPECT(harness.session.world().statistics().elapsed_time_s > time_before && harness.session.build_model().held_reason.empty(),
                "speed previews keep playback running instead of holding physics time");
            if (lose_focus)
            {
                ui::UiEvent lost;
                lost.kind = ui::UiEventKind::focus_lost;
                harness.dispatch(lost);
            }
            else
                harness.key(ui::UiKey::escape);
            RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, original, 1.0e-12, "cancel restores the speed before scrubbing");
            RIGIDBODIES_EXPECT(!harness.interface.pointer_captured() && !harness.session.stepper().is_paused() &&
                    harness.session.build_model().undo_history.size() == history_size,
                "cancel releases the pointer without pausing or creating edit history");
            harness.pointer(ui::UiEventKind::pointer_up, moved);
            RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, original, 1.0e-12, "physical release cannot commit the cancelled speed");
        }
    }

    RIGIDBODIES_TEST("a custom speed stays visible and Custom can be chosen again")
    {
        LearnerInterface harness;
        ui::UiCommand speed;
        speed.kind = ui::UiCommandKind::set_time_scale;
        speed.value = 0.875;
        harness.apply_command(speed);
        harness.frames_for(2);
        const auto field = control("bar.speed.choice") + "--field";
        RIGIDBODIES_EXPECT(harness.document().element_value(field) == "0.875000", "the selected option carries the actual custom speed instead of only saying Custom");
        RIGIDBODIES_EXPECT(harness.click_element(field), "the speed dropdown opens");
        harness.frames_for(2);
        const auto list = dropdown_extent(harness, field);
        RIGIDBODIES_EXPECT(list.has_value(), "the dropdown is open for a custom value");
        const auto bounds = *harness.visible_bounds(field);
        const auto count = ui::find_control_spec("bar.speed.choice")->options.size() + 1;
        const auto row_height = (list->second - list->first) / static_cast<double>(count);
        harness.click({ (bounds.minimum.x + bounds.maximum.x) * 0.5, list->second - row_height * 0.5 });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-playback_speed"), "Custom remains actionable when the active speed is already custom");
        RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().time_scale, 0.875, 1e-12, "reopening the editor preserves the exact active multiplier");
        RIGIDBODIES_EXPECT(harness.document().element_value(control("bar.speed.custom") + "--field") == "0.875\xC3\x97", "the editor displays the full custom multiplier");
        harness.key(ui::UiKey::escape);
        harness.frame();
        harness.key(ui::UiKey::escape);
        speed.value = 2.0001;
        harness.apply_command(speed);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().element_value(field) == "2.000100", "a custom value near a preset is still shown as its actual speed");
    }

    RIGIDBODIES_TEST("the folded playback Custom action opens its editor from the menu")
    {
        LearnerInterface harness({}, { 560, 600 });
        RIGIDBODIES_EXPECT(harness.document().element_has_class(control("bar.speed.choice"), "is-overflowed"), "speed has moved into the menu on a narrow toolbar");
        harness.interface.request(ui::ViewRequest::open_main_menu);
        harness.frames_for(2);
        std::string custom;
        for (const auto& id : harness.document().child_ids("menu-overflow"))
            if (harness.document().element_attribute(id, "aria-label") == "Custom…")
                custom = id;
        RIGIDBODIES_EXPECT(!custom.empty() && harness.click_element(custom), "the overflow Custom action can be pressed");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("main_menu") && harness.interface.view_state().transient_open("playback_speed"), "the menu gives way to the speed editor");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("bar.speed.custom") + "--field", "the exact speed field is ready for typing");
    }

    physics::BodyId first_free_object(const LearnerInterface& harness)
    {
        for (const auto id : harness.session.world().body_ids())
            if (const auto* body = harness.session.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                return id;
        return {};
    }

    bool focus_in_panel(const LearnerInterface& harness, std::string_view panel)
    {
        const auto focus = harness.document().focused_element();
        const auto rows = harness.document().child_ids(panel);
        return std::any_of(rows.begin(), rows.end(), [&](const std::string& row)
            {
                return focus == row || focus.rfind(row + "--", 0) == 0;
            });
    }

    RIGIDBODIES_TEST("dialog keyboard navigation stays on its surface and restores the invoking control")
    {
        LearnerInterface harness;
        harness.key(ui::UiKey::tab);
        const auto origin = harness.document().focused_element();
        RIGIDBODIES_EXPECT(!origin.empty(), "the workspace has a keyboard-focused control");
        harness.key(ui::UiKey::comma, { false, true, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(focus_in_panel(harness, "panel-preferences"), "Preferences takes focus from the workspace");
        for (int index = 0; index < 80; ++index)
        {
            harness.key(ui::UiKey::tab, { index >= 40, false, false });
            RIGIDBODIES_EXPECT(focus_in_panel(harness, "panel-preferences"), "Tab and Shift+Tab never enter controls behind the sheet");
        }
        harness.key(ui::UiKey::f6);
        RIGIDBODIES_EXPECT(focus_in_panel(harness, "panel-preferences"), "F6 also stays in the active dialog");
        // Finish any number-field edit before closing the sheet.
        if (harness.interface.focus_owner() == ui::FocusOwner::text_field)
            harness.key(ui::UiKey::escape);
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("preferences") && harness.document().focused_element() == origin, "closing the dialog returns to its invoking control");

        harness.key(ui::UiKey::f10);
        harness.frames_for(2);
        for (int index = 0; index < 50; ++index)
        {
            harness.key(ui::UiKey::tab);
            RIGIDBODIES_EXPECT(focus_in_panel(harness, "panel-main_menu") || focus_in_panel(harness, "menu-overflow"), "menu traversal includes its overflow actions but not workspace controls");
        }
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().focused_element() == origin, "a menu also restores the invoking keyboard control");
    }

    RIGIDBODIES_TEST("F6 visits Guide Inspector Measure and the scene in both directions")
    {
        LearnerInterface harness({}, { 2560, 1440 });
        harness.interface.view_state().set_surface_open("guide.expanded", true);
        harness.interface.view_state().set_surface_open("inspector.open", true);
        harness.interface.view_state().set_surface_open("measure.open", true);
        harness.frames_for(2);
        for (const auto panel : { "panel-command_bar", "panel-guide", "panel-inspector", "panel-measure" })
        {
            harness.key(ui::UiKey::f6);
            RIGIDBODIES_EXPECT(focus_in_panel(harness, panel), std::string("F6 reaches ") + panel);
        }
        harness.key(ui::UiKey::f6);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene, "the last F6 returns keyboard control to the stage");
        for (const auto panel : { "panel-measure", "panel-inspector", "panel-guide", "panel-command_bar" })
        {
            harness.key(ui::UiKey::f6, { true, false, false });
            RIGIDBODIES_EXPECT(focus_in_panel(harness, panel), std::string("Shift+F6 reaches ") + panel);
        }
    }

    RIGIDBODIES_TEST("Space activates focused controls and dialogs protect the scene from destructive keys")
    {
        LearnerInterface harness;
        const auto target = first_free_object(harness);
        ui::UiCommand select { ui::UiCommandKind::select_body };
        select.body = target;
        harness.apply_command(select);
        harness.key(ui::UiKey::comma, { false, true, false });
        harness.frames_for(2);
        const auto paused = harness.session.stepper().is_paused();
        harness.key(ui::UiKey::delete_key);
        harness.key(ui::UiKey::r);
        RIGIDBODIES_EXPECT(harness.session.world().is_valid(target) && harness.session.selection() == target, "Delete and reset do not alter the selection behind Preferences");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("prefs.close"), "Preferences starts on its close action");
        harness.key(ui::UiKey::space);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("preferences") && harness.session.stepper().is_paused() == paused, "Space closes the focused sheet action without changing playback");

        ui::UiCommand mass { ui::UiCommandKind::set_selected_mass };
        mass.body = target;
        mass.value = harness.session.world().find_body(target)->mass_properties().mass_kg + 1.0;
        harness.apply_command(mass);
        harness.apply_command(ui::UiCommand { ui::UiCommandKind::quit });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.session.build_model().confirmation.has_value(), "dirty setup asks before quitting");
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.session.build_model().confirmation && !harness.session.should_quit(), "confirmation initially focuses Cancel, so Enter never discards work accidentally");
        harness.apply_command(ui::UiCommand { ui::UiCommandKind::quit });
        harness.frames_for(2);
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.session.build_model().confirmation && harness.session.selection() == target, "Escape cancels the confirmation without clearing the underlying selection");
    }

    RIGIDBODIES_TEST("a confirmation takes priority over Command search and returns to its query on Cancel")
    {
        LearnerInterface harness;
        const auto target = first_free_object(harness);
        ui::UiCommand mass { ui::UiCommandKind::set_selected_mass };
        mass.body = target;
        mass.value = harness.session.world().find_body(target)->mass_properties().mass_kg + 1.0;
        harness.apply_command(mass);
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        harness.text("gravity");
        harness.key(ui::UiKey::q, { false, true, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-confirmation") && !harness.document().element_visible("panel-command_search"), "the confirmation is visible above the still-open palette");
        for (int index = 0; index < 8; ++index)
        {
            harness.key(ui::UiKey::tab);
            RIGIDBODIES_EXPECT(focus_in_panel(harness, "panel-confirmation"), "confirmation owns keyboard traversal");
        }
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.session.build_model().confirmation && harness.interface.view_state().sheet_open("command_search"), "Cancel dismisses only the confirmation");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("search.field.query") + "--field" && harness.interface.view_state().value("search.query") == "gravity", "the palette query and focus are restored");
    }

    RIGIDBODIES_TEST("Command search arrows select results while keeping the query editable")
    {
        LearnerInterface harness;
        harness.interface.view_state().open_transient("playback_speed");
        harness.frames_for(2);
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("command_search") && harness.interface.view_state().transient_open("playback_speed"), "Escape closes the visible search sheet before its underlying popover");
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        const auto query = control("search.field.query") + "--field";
        harness.text("gravity");
        harness.frames_for(2);
        std::vector<std::string> results;
        for (const auto& id : harness.document().control_ids())
            if (harness.document().element_attribute(id, "data-control-key") == std::optional<std::string> { "search.result" })
                results.push_back(id);
        RIGIDBODIES_EXPECT(results.size() > 1, "the query offers multiple results");
        harness.key(ui::UiKey::arrow_down);
        harness.key(ui::UiKey::arrow_down);
        RIGIDBODIES_EXPECT(harness.document().focused_element() == query && harness.document().element_attribute(query, "aria-activedescendant") == std::optional<std::string> { results[1] }, "Down highlights the second result without stealing the query's caret");
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().element_has_class(results[1], "is-selected"), "the selected result remains visible across frame refreshes");
        for (const auto theme_id : { "workbench_dark", "workbench_light", "workbench_projector" })
        {
            ui::UiCommand theme { ui::UiCommandKind::set_theme };
            theme.id = theme_id;
            harness.apply_command(theme);
            harness.frames_for(2);
            const auto selected_color = harness.document().element_color(results[1], "background-color");
            const auto unselected_color = harness.document().element_color(results[0], "background-color");
            RIGIDBODIES_EXPECT(selected_color && unselected_color && selected_color->alpha > unselected_color->alpha + 0.05f, "the active result has a visible highlight in every theme");
        }
        harness.key(ui::UiKey::arrow_up);
        RIGIDBODIES_EXPECT(harness.document().element_attribute(query, "aria-activedescendant") == std::optional<std::string> { results[0] }, "Up selects the previous result");
        harness.text(" strength");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.document().element_attribute(query, "aria-activedescendant"), "editing the query clears stale result selection");
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("command_search") && harness.document().focused_element() == control("world.gravity.strength") + "--field", "Enter reveals the setting matched by the refined query");
    }

    RIGIDBODIES_TEST("a press on the stage ends editing in a field and commits the typed value")
    {
        LearnerInterface harness;
        const auto target = first_free_object(harness);
        ui::UiCommand select;
        select.kind = ui::UiCommandKind::select_body;
        select.body = target;
        harness.apply_command(select);
        harness.frames_for(2);
        const auto mass = control("object.properties.mass") + "--field";
        RIGIDBODIES_EXPECT(harness.click_element(mass), "the Mass field is displayed");
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::text_field, "pressing the Mass field starts editing it");
        harness.key(ui::UiKey::a, { false, true, false });
        harness.text("2.5");

        // An empty spot well below the objects and above the floor.
        ui::Vec2 empty;
        for (const auto y : { 0.45, 0.5, 0.55, 0.6 })
        {
            empty = harness.stage_point(0.15, y);
            if (!harness.interface.surface_at(empty))
                break;
        }
        harness.click(empty);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene && !harness.interface.wants_text_input(), "the press on the stage ends editing");
        RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, 2.5, 1e-9, "leaving the field commits the typed mass to the object it belonged to");
        RIGIDBODIES_EXPECT(!harness.document().element_visible(mass), "with nothing selected the Mass field is gone rather than left behind");
        const auto paused = harness.session.build_model().paused;
        harness.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(harness.session.build_model().paused != paused, "Space reaches the simulation again");

        // Pressing another object keeps a Mass field on screen, now for that object.
        physics::BodyId other;
        for (const auto id : harness.session.world().body_ids())
            if (const auto* body = harness.session.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body && !(id == target))
                other = id;
        harness.apply_command(select);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(mass), "the Mass field is displayed again");
        harness.frame();
        harness.click(harness.session.camera().world_to_screen(harness.session.world().find_body(other)->world_center_of_mass_m()));
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.session.selection() == other && harness.document().element_visible(mass), "the press selects the other object");
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene, "the press on the other object also ends editing");

        harness.apply_command(select);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(mass), "the Mass field is displayed for the first object");
        harness.frame();
        harness.text("9");
        ui::UiCommand clear;
        clear.kind = ui::UiCommandKind::clear_selection;
        harness.apply_command(clear);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene && !harness.document().element_visible(mass), "a field whose object is deselected does not linger with the keys");
        RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, 2.5, 1e-9, "an unconfirmed edit for a deselected object is dropped");
    }

    RIGIDBODIES_TEST("sheets stack: only the top one is displayed and Escape reveals the one beneath")
    {
        LearnerInterface harness;
        harness.key(ui::UiKey::l);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-library"), "L opens the Library");
        harness.key(ui::UiKey::comma, { false, true, false });
        harness.frame();
        harness.key(ui::UiKey::f1);
        harness.frame();
        const auto& sheets = harness.interface.view_state().sheets();
        RIGIDBODIES_EXPECT(sheets.size() == 3 && sheets.back() == "shortcuts", "three sheets are open with Shortcuts on top");
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-shortcuts") && !harness.document().element_visible("panel-library") && !harness.document().element_visible("panel-preferences"), "only the top sheet is displayed");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-preferences") && !harness.document().element_visible("panel-shortcuts"), "Escape closes the top sheet and reveals Preferences");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-library") && !harness.document().element_visible("panel-preferences"), "the next Escape reveals the Library");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().sheets().empty() && !harness.document().element_visible("panel-library"), "the last Escape closes the Library");
    }

    RIGIDBODIES_TEST("a keyboard context menu closed within the same frame stays closed")
    {
        LearnerInterface harness;
        const ui::KeyModifiers shift { true, false, false };
        harness.key(ui::UiKey::f10, shift);
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu"), "Shift+F10 opens the context menu at once");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu") && harness.interface.view_state().value("context.kind") == "stage", "the menu stays open on the stage target");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("context_menu"), "Escape closes it");

        harness.key(ui::UiKey::f10, shift);
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("context_menu"), "a menu closed before the next frame is not reopened by its own request");

        harness.click(harness.stage_point(0.5, 0.4), ui::PointerButton::secondary);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu"), "a right-click still opens the menu on the next frame");
    }

    RIGIDBODIES_TEST("a control hidden with its surface stops taking keys from the scene")
    {
        LearnerInterface harness;
        ui::UiCommand select;
        select.kind = ui::UiCommandKind::select_body;
        for (const auto id : harness.session.world().body_ids())
            if (const auto* body = harness.session.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                select.body = id;
        harness.apply_command(select);
        harness.frames_for(2);
        const auto mass = control("object.properties.mass") + "--field";
        RIGIDBODIES_EXPECT(harness.click_element(mass), "the Mass field is displayed");
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::text_field, "pressing the Mass field starts editing it");

        harness.key(ui::UiKey::f5);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode && !harness.document().element_visible(mass), "Present hides the inspector");
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene && !harness.interface.wants_text_input(), "the hidden field no longer holds keyboard focus");
        auto paused = harness.session.build_model().paused;
        harness.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(harness.session.build_model().paused != paused, "Space reaches the simulation in Present");
        harness.key(ui::UiKey::f5);
        harness.frames_for(2);

        harness.key(ui::UiKey::f10);
        harness.frame();
        harness.key(ui::UiKey::tab);
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("main_menu") && harness.interface.focus_owner() == ui::FocusOwner::keyboard_control, "Tab moves keyboard focus while the main menu is open");
        harness.click(harness.stage_point(0.3, 0.5));
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("main_menu"), "a press on the stage closes the menu");
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() != ui::FocusOwner::text_field && harness.document().focused_element().empty() == (harness.interface.focus_owner() == ui::FocusOwner::scene), "focus never rests on a hidden menu item");
        paused = harness.session.build_model().paused;
        harness.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(harness.session.build_model().paused != paused, "Space reaches the simulation after the menu closes");
    }

    RIGIDBODIES_TEST("the Quick start hint rests inside the bottom of the stage and Got it dismisses it")
    {
        for (const auto& [size, display, text] : { std::tuple { render::ViewportSize { 1600, 900 }, 1.5f, 1.0 }, std::tuple { render::ViewportSize { 1280, 720 }, 1.0f, 1.0 }, std::tuple { render::ViewportSize { 640, 480 }, 2.25f, 2.0 } })
        {
            core::ApplicationConfig config;
            config.interface_settings.interface_scale = text;
            LearnerInterface harness(config, size, display);
            const auto where = at(size, display) + " with " + std::to_string(text) + "x text";
            RIGIDBODIES_EXPECT(harness.interface.view_state().next_hint() == "play", "a new learner is offered the Play hint" + where);
            const auto stage = harness.interface.layout().stage;
            const auto card = harness.visible_bounds("region-hint");
            RIGIDBODIES_EXPECT(card.has_value(), "the hint card is displayed" + where);
            RIGIDBODIES_EXPECT(card->minimum.y >= stage.minimum.y - 1.0 && card->minimum.x >= stage.minimum.x - 1.0 && card->maximum.x <= stage.maximum.x + 1.0, "the hint card never leaves the stage for the command bar" + where);
            if (card->height() + 40.0 * harness.interface.scale() <= stage.height())
                RIGIDBODIES_EXPECT_NEAR(card->maximum.y, stage.maximum.y - 40.0 * harness.interface.scale(), 1.0, "the hint card rests just above the bottom of the stage" + where);
            const auto play = harness.visible_bounds(control("transport.play"));
            RIGIDBODIES_EXPECT(play && harness.document().element_at((play->minimum + play->maximum) * 0.5).rfind(control("transport.play"), 0) == 0, "the hint never covers Play" + where);

            RIGIDBODIES_EXPECT(harness.click_element(control("hints.dismiss")), "Got it can be pressed" + where);
            harness.frame();
            RIGIDBODIES_EXPECT(harness.interface.view_state().hint_dismissed("play"), "Got it dismisses the hint" + where);
        }
    }

    RIGIDBODIES_TEST("Escape and the guide arrows reach Present while no control holds keyboard focus")
    {
        LearnerInterface harness;
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene, "nothing has keyboard focus when the application opens");
        RIGIDBODIES_EXPECT(harness.click_element(control("transport.play")), "Play is displayed");
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::scene, "a pointer press on a control leaves keys with the scene");

        harness.key(ui::UiKey::f5);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode, "F5 enters Present");
        harness.key(ui::UiKey::arrow_right);
        harness.key(ui::UiKey::arrow_right);
        harness.key(ui::UiKey::arrow_left);
        RIGIDBODIES_EXPECT_NEAR(harness.interface.view_state().number("present.guide_step", 0.0), 1.0, 0.0, "the arrow keys step through the guide in Present");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.interface.view_state().present().mode, "the last Escape leaves Present");

        harness.key(ui::UiKey::f5);
        harness.frame();
        harness.key(ui::UiKey::tab);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::keyboard_control, "Tab gives a control keyboard focus");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode && harness.interface.focus_owner() == ui::FocusOwner::scene, "the first Escape only leaves keyboard mode");
        harness.key(ui::UiKey::escape);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.interface.view_state().present().mode, "the next Escape leaves Present");
    }

    RIGIDBODIES_TEST("the Present strip replaces the command bar in the real document")
    {
        for (const auto& [size, scale] : { std::pair { render::ViewportSize { 1600, 900 }, 1.0f }, std::pair { render::ViewportSize { 1600, 900 }, 1.5f }, std::pair { render::ViewportSize { 640, 480 }, 1.25f } })
        {
            LearnerInterface harness({}, size, scale);
            const auto where = at(size, scale);
            harness.interface.request(ui::ViewRequest::toggle_present);
            harness.frames_for(2);
            const auto strip = region_bounds(harness, ui::RegionId::present_strip);
            RIGIDBODIES_EXPECT(strip.minimum.y == 0.0 && strip.width() == static_cast<double>(size.width), "the Present strip spans the top of the window" + where);
            const auto card = harness.visible_bounds("panel-present");
            RIGIDBODIES_EXPECT(card.has_value(), "the Present card is displayed" + where);
            RIGIDBODIES_EXPECT(inside(*card, strip), "the Present card lies inside the Present strip" + where);
            RIGIDBODIES_EXPECT(!harness.document().element_visible("panel-command_bar") && !harness.interface.layout().find(ui::RegionId::command_bar), "the command bar is not displayed in Present" + where);
            const auto play = harness.visible_bounds(control("present.transport.play"));
            RIGIDBODIES_EXPECT(play && inside(*play, strip), "the strip's Play control is laid out inside it" + where);

            harness.interface.request(ui::ViewRequest::toggle_present);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.document().element_visible("panel-present") && harness.document().element_visible("panel-command_bar"), "leaving Present restores the command bar" + where);
        }
    }

    RIGIDBODIES_TEST("an active draft displays the draw bar card inside its region at every width")
    {
        for (const auto& [size, scale, mode] : { std::tuple { render::ViewportSize { 1600, 900 }, 1.0f, ui::LayoutMode::wide }, std::tuple { render::ViewportSize { 1600, 900 }, 1.5f, ui::LayoutMode::medium }, std::tuple { render::ViewportSize { 1280, 720 }, 2.0f, ui::LayoutMode::narrow } })
        {
            LearnerInterface harness({}, size, scale);
            const auto where = at(size, scale);
            RIGIDBODIES_EXPECT(harness.interface.layout().mode == mode, "the viewport selects the intended layout mode" + where);
            RIGIDBODIES_EXPECT(!harness.interface.layout().find(ui::RegionId::draw_bar) && !harness.document().element_visible("panel-draw_bar"), "no draw bar is shown before drawing" + where);
            ui::UiCommand draw;
            draw.kind = ui::UiCommandKind::start_new_shape;
            harness.apply_command(draw);
            harness.frames_for(2);
            const auto bar = region_bounds(harness, ui::RegionId::draw_bar);
            const auto command_bar = region_bounds(harness, ui::RegionId::command_bar);
            RIGIDBODIES_EXPECT(bar.minimum.y >= command_bar.maximum.y - 1.0 && bar.maximum.y <= harness.interface.layout().stage.minimum.y + 1.0, "the draw bar sits between the command bar and the stage" + where);
            const auto card = harness.visible_bounds("panel-draw_bar");
            RIGIDBODIES_EXPECT(card.has_value(), "the draw bar card is displayed" + where);
            RIGIDBODIES_EXPECT(inside(*card, bar), "the draw bar card lies inside the draw bar region" + where);
            const auto discard = harness.visible_bounds(control("draw.actions.discard"));
            RIGIDBODIES_EXPECT(discard.has_value(), "Discard is displayed on the draw bar" + where);

            // The draw bar sheds no controls, so a narrow window ends drawing with Escape instead.
            if (mode == ui::LayoutMode::narrow)
                harness.key(ui::UiKey::escape);
            else
            {
                RIGIDBODIES_EXPECT(inside(*discard, bar), "Discard lies on the draw bar" + where);
                RIGIDBODIES_EXPECT(harness.click_element(control("draw.actions.discard")), "Discard can be pressed" + where);
            }
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.session.build_model().shape_editor_active && !harness.interface.layout().find(ui::RegionId::draw_bar) && !harness.document().element_visible("panel-draw_bar"), "Discard ends drawing and removes the draw bar" + where);
        }
    }

    RIGIDBODIES_TEST("a collapsed Guide shows only its rail, and the rail expands the Guide")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.0f);
        RIGIDBODIES_EXPECT(harness.interface.layout().mode == ui::LayoutMode::wide, "1600 logical pixels is the wide layout");
        RIGIDBODIES_EXPECT(harness.session.build_model().scenario_content && !harness.session.build_model().scenario_content->guide.empty(), "the startup experiment has a guide");
        RIGIDBODIES_EXPECT(harness.interface.view_state().surface_open("guide.expanded", false) && harness.document().element_visible("panel-guide"), "a first visit opens the guide");

        harness.key(ui::UiKey::g);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().surface_open("guide.expanded", true), "G collapses the guide");
        RIGIDBODIES_EXPECT(!harness.interface.layout().find(ui::RegionId::guide_panel), "a collapsed guide has no docked panel region");
        const auto rail_region = region_bounds(harness, ui::RegionId::guide_rail);
        RIGIDBODIES_EXPECT(!harness.document().element_visible("panel-guide"), "the guide card is not displayed while collapsed");
        const auto rail = harness.visible_bounds("rail-guide");
        RIGIDBODIES_EXPECT(rail.has_value(), "the rail's expand control is displayed");
        // The control fills the rail, whose region clips the overhang of its top padding.
        const auto rail_centre = (rail->minimum + rail->maximum) * 0.5;
        RIGIDBODIES_EXPECT(rail->minimum.x >= rail_region.minimum.x - 1.0 && rail->maximum.x <= rail_region.maximum.x + 1.0 && std::abs(rail->minimum.y - rail_region.minimum.y) <= 1.0 && rail_region.contains(rail_centre), "the expand control lies in the guide rail");
        RIGIDBODIES_EXPECT(harness.interface.surface_at(rail_centre), "the rail is an interface surface, not the stage");

        RIGIDBODIES_EXPECT(harness.click_element("rail-guide"), "the rail can be pressed");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().surface_open("guide.expanded", false), "pressing the rail expands the guide");
        RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::guide_panel) && !harness.interface.layout().find(ui::RegionId::guide_rail), "the expanded guide is docked again");
        const auto card = harness.visible_bounds("panel-guide");
        const auto dock = region_bounds(harness, ui::RegionId::guide_panel);
        // The guide scrolls inside its dock, so only the card's top and width are bounded by it.
        RIGIDBODIES_EXPECT(card && card->minimum.x >= dock.minimum.x - 1.0 && card->maximum.x <= dock.maximum.x + 1.0 && dock.contains(card->minimum + ui::Vec2 { 1.0, 1.0 }), "the guide card is displayed in its dock");
    }

    struct BarRow
    {
        std::string id, kind, key;
        bool folded { false }, label_hidden { false };
    };

    std::vector<BarRow> command_bar_rows(const LearnerInterface& harness)
    {
        std::vector<BarRow> rows;
        const auto& backend = harness.document();
        for (const auto& id : backend.child_ids("panel-command_bar"))
        {
            BarRow row;
            row.id = id;
            row.kind = backend.element_attribute(id, "data-row-kind").value_or("");
            row.key = backend.element_attribute(id, "data-control-key").value_or("");
            row.folded = backend.element_has_class(id, "is-overflowed");
            row.label_hidden = backend.element_has_class(id, "label-hidden");
            rows.push_back(std::move(row));
        }
        return rows;
    }

    bool is_command(const BarRow& row)
    {
        return row.kind == "action" || row.kind == "select" || row.kind == "segmented";
    }

    RIGIDBODIES_TEST("the command bar sheds labels, then folds rows into the menu, and never overflows")
    {
        LearnerInterface harness({}, { 1920, 900 }, 1.0f);
        std::set<std::string> always_folded, labelled_when_wide;
        for (const auto& row : command_bar_rows(harness))
        {
            if (row.folded)
                always_folded.insert(row.id);
            if (!row.label_hidden)
                labelled_when_wide.insert(row.id);
        }
        for (const auto* key : { "transport.play", "tools.mode", "view.library", "view.measure" })
        {
            const auto id = control(key);
            RIGIDBODIES_EXPECT(harness.visible_bounds(id).has_value() && labelled_when_wide.count(id) == 1, std::string("a wide bar shows ") + key + " with its label");
        }

        std::set<std::string> previous_folded = always_folded, previous_hidden;
        int widest_hidden_label = -1, widest_fold = -1;
        bool labels_before_folds = false;
        for (const auto width : { 1920, 1600, 1440, 1280, 1180, 1080, 980, 900, 820, 740, 660, 580, 500, 440, 400, 360 })
        {
            const render::ViewportSize size { width, 800 };
            harness.resize(size, 1.0f);
            harness.frames_for(2);
            const auto where = at(size, 1.0f);
            const auto bar = region_bounds(harness, ui::RegionId::command_bar);
            std::set<std::string> folded, hidden;
            for (const auto& row : command_bar_rows(harness))
            {
                if (row.folded)
                    folded.insert(row.id);
                else
                {
                    if (row.label_hidden && labelled_when_wide.count(row.id))
                        hidden.insert(row.id);
                    if (const auto bounds = harness.visible_bounds(row.id))
                        RIGIDBODIES_EXPECT(inside(*bounds, bar), "every displayed row stays inside the command bar: " + row.id + where);
                }
            }
            for (const auto& id : previous_folded)
                RIGIDBODIES_EXPECT(folded.count(id) == 1, "a narrower bar never unfolds a row: " + id + where);
            for (const auto& id : previous_hidden)
                RIGIDBODIES_EXPECT(hidden.count(id) == 1 || folded.count(id) == 1, "a narrower bar never restores a hidden label: " + id + where);
            const auto newly_folded = folded.size() > always_folded.size();
            if (!hidden.empty() && widest_hidden_label < 0)
                widest_hidden_label = width;
            if (newly_folded && widest_fold < 0)
                widest_fold = width;
            labels_before_folds |= !hidden.empty() && !newly_folded;
            for (const auto* key : { "transport.play", "view.menu" })
            {
                const auto bounds = harness.visible_bounds(control(key));
                RIGIDBODIES_EXPECT(bounds.has_value() && inside(*bounds, bar), std::string("the bar always shows ") + key + where);
            }

            // Every command the bar folded away is offered at the top of the main menu.
            harness.interface.request(ui::ViewRequest::open_main_menu);
            harness.frames_for(2);
            std::set<std::string> offered;
            for (const auto& id : harness.document().child_ids("menu-overflow"))
                if (id.rfind("menu-overflow--", 0) == 0 && harness.visible_bounds(id))
                    offered.insert(harness.document().element_attribute(id, "aria-label").value_or(""));
            for (const auto& row : command_bar_rows(harness))
            {
                if (!row.folded || !is_command(row))
                    continue;
                if (row.kind == "action")
                {
                    const auto label = harness.document().element_attribute(row.id, "aria-label").value_or("");
                    RIGIDBODIES_EXPECT(!label.empty() && offered.count(label) == 1, "the menu offers the folded command " + label + where);
                }
                else if (const auto* spec = ui::find_control_spec(row.key))
                    for (const auto& option : spec->options)
                        RIGIDBODIES_EXPECT(offered.count(std::string(option.label)) == 1, "the menu offers every option of the folded " + row.key + where);
            }
            harness.interface.request(ui::ViewRequest::close_top_surface);
            harness.frames_for(1);
            previous_folded = std::move(folded);
            previous_hidden = std::move(hidden);
        }
        RIGIDBODIES_EXPECT(widest_fold > 0 && widest_hidden_label > widest_fold && labels_before_folds, "labels hide at wider widths than any row folds");
    }

    RIGIDBODIES_TEST("Reduce Motion configured at startup suppresses motion effects from the first frame")
    {
        core::ApplicationConfig config;
        config.interface_settings.reduce_motion = true;
        config.effects.transitions = config.effects.impact_flashes = config.effects.directional_blur = config.effects.impact_sparks = config.effects.impact_dust = true;
        config.effects.motion_trails = config.effects.soft_deformation = true;
        app::SimulationSession session;
        session.configure(config);
        const auto suppressed = [&]
        {
            const auto& settings = session.scene_settings();
            return !settings.transitions && !settings.impact_flashes && !settings.directional_blur && !settings.impact_sparks && !settings.impact_dust;
        };
        RIGIDBODIES_EXPECT(session.build_model().reduce_motion && suppressed(), "decorative motion is off straight after configure");
        RIGIDBODIES_EXPECT(session.scene_settings().motion_trails && session.scene_settings().soft_deformation, "trails and deformation, which show the physics, stay on");

        for (const auto* effect : { "transitions", "impact_flashes", "directional_blur", "impact_sparks", "impact_dust" })
        {
            ui::UiCommand enable;
            enable.kind = ui::UiCommandKind::set_visual_effect;
            enable.id = effect;
            enable.flag = true;
            session.apply(enable);
        }
        ui::UiCommand restore;
        restore.kind = ui::UiCommandKind::set_preference;
        restore.detail = "prefs.effects.restore_defaults";
        session.apply(restore);
        RIGIDBODIES_EXPECT(suppressed(), "enabling an effect or restoring defaults keeps it off while motion is reduced");

        ui::UiCommand off;
        off.kind = ui::UiCommandKind::set_preference;
        off.detail = "prefs.accessibility.reduce_motion";
        off.flag = false;
        session.apply(off);
        const render::SceneRenderSettings defaults;
        const auto& settings = session.scene_settings();
        RIGIDBODIES_EXPECT(!session.build_model().reduce_motion && settings.transitions == defaults.transitions && settings.impact_flashes == defaults.impact_flashes && settings.directional_blur == defaults.directional_blur && settings.impact_sparks == defaults.impact_sparks && settings.impact_dust == defaults.impact_dust, "turning Reduce Motion off restores the default effects");
    }

    RIGIDBODIES_TEST("the document carries the reduce-motion class exactly while motion is reduced")
    {
        core::ApplicationConfig config;
        config.interface_settings.reduce_motion = true;
        LearnerInterface harness(config);
        RIGIDBODIES_EXPECT(harness.document().document_has_class("reduce-motion"), "a reduced-motion startup gates document animations from the first frame");
        ui::UiCommand off;
        off.kind = ui::UiCommandKind::set_preference;
        off.detail = "prefs.accessibility.reduce_motion";
        off.flag = false;
        harness.apply_command(off);
        harness.frame();
        RIGIDBODIES_EXPECT(!harness.document().document_has_class("reduce-motion"), "turning the preference off removes the class");
        off.flag = true;
        harness.apply_command(off);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().document_has_class("reduce-motion"), "turning it on again restores the class");
    }

    ui::UiCommand effects_preference(std::string_view key, std::string_view id = {}, bool flag = false)
    {
        ui::UiCommand preference;
        preference.kind = ui::UiCommandKind::set_preference;
        preference.detail = std::string(key);
        preference.id = std::string(id);
        preference.flag = flag;
        return preference;
    }

    RIGIDBODIES_TEST("Effects quality presets set the effect switches and limits, and an edited mix reads Custom")
    {
        app::SimulationSession session;
        session.configure(core::ApplicationConfig {});
        const auto& settings = session.scene_settings();
        RIGIDBODIES_EXPECT(session.build_model().effects_quality == "standard", "the first-run effects read Standard");

        session.apply(effects_preference("prefs.effects.quality", "low"));
        RIGIDBODIES_EXPECT(session.build_model().effects_quality == "low", "choosing Low highlights Low");
        RIGIDBODIES_EXPECT(settings.material_shading && settings.motion_trails && settings.soft_deformation && settings.impact_flashes && !settings.contact_shadows && !settings.depth_background && !settings.directional_blur && !settings.impact_sparks && !settings.impact_dust, "Low keeps the effects that explain the physics and drops the decorative ones");
        RIGIDBODIES_EXPECT(settings.shading_body_budget < 128 && settings.spark_budget < 128 && settings.motion_body_budget < 64, "Low lowers the effect limits");

        session.apply(effects_preference("prefs.effects.quality", "high"));
        RIGIDBODIES_EXPECT(session.build_model().effects_quality == "high" && settings.contact_shadows && settings.directional_blur && settings.impact_dust && settings.shading_body_budget > 128 && settings.spark_budget > 128, "High turns every effect on and raises the limits");

        ui::UiCommand blur;
        blur.kind = ui::UiCommandKind::set_visual_effect;
        blur.id = "directional_blur";
        blur.flag = false;
        session.apply(blur);
        RIGIDBODIES_EXPECT(session.build_model().effects_quality == "custom", "a switch changed by hand makes the mix Custom");
        session.apply(effects_preference("prefs.effects.quality", "custom"));
        RIGIDBODIES_EXPECT(!settings.directional_blur && session.build_model().effects_quality == "custom", "choosing Custom keeps the hand-made mix");

        session.apply(effects_preference("prefs.effects.restore_defaults"));
        const core::ApplicationConfig defaults;
        RIGIDBODIES_EXPECT(session.build_model().effects_quality == "standard" && settings.directional_blur && settings.shading_body_budget == static_cast<std::size_t>(defaults.effects.shading_body_budget) && settings.impact_flash_budget == static_cast<std::size_t>(defaults.effects.impact_flash_budget), "restoring the defaults returns to Standard, the first-run state");
    }

    RIGIDBODIES_TEST("Reduce Motion keeps the chosen quality, shows the switches it holds as unavailable, and gives them back")
    {
        core::ApplicationConfig config;
        config.interface_settings.reduce_motion = true;
        LearnerInterface harness(config);
        RIGIDBODIES_EXPECT(harness.session.build_model().effects_quality == "standard", "the switches Reduce Motion holds do not turn Standard into Custom");
        harness.interface.view_state().open_sheet("preferences");
        harness.frames_for(2);
        for (const auto* key : { "prefs.effects.directional_blur", "prefs.effects.impact_flashes", "prefs.effects.impact_sparks", "prefs.effects.impact_dust", "prefs.effects.transitions" })
            RIGIDBODIES_EXPECT(harness.document().element_has_class(control(key), "is-disabled"), std::string("Reduce Motion shows the switch it holds as unavailable: ") + key);
        RIGIDBODIES_EXPECT(!harness.document().element_has_class(control("prefs.effects.motion_trails"), "is-disabled"), "a switch Reduce Motion leaves alone stays available");

        harness.apply_command(effects_preference("prefs.effects.quality", "low"));
        harness.apply_command(effects_preference("prefs.accessibility.reduce_motion", {}, false));
        harness.frames_for(2);
        const auto& settings = harness.session.scene_settings();
        RIGIDBODIES_EXPECT(harness.session.build_model().effects_quality == "low", "the quality chosen while motion was reduced stays chosen");
        RIGIDBODIES_EXPECT(settings.transitions && settings.impact_flashes && !settings.directional_blur && !settings.impact_sparks && !settings.impact_dust, "turning Reduce Motion off gives back the switches the chosen quality sets");
        RIGIDBODIES_EXPECT(!harness.document().element_has_class(control("prefs.effects.impact_flashes"), "is-disabled"), "the switches are available again");
    }

    RIGIDBODIES_TEST("the Guide's starting-motion field shows and edits the setup while a run goes on")
    {
        LearnerInterface harness;
        RIGIDBODIES_EXPECT(harness.session.load_scenario("collision_comparison"), "the collision scene loads");
        harness.frames_for(2);
        physics::BodyId disc;
        for (const auto& object : harness.session.build_model().objects)
            if (object.document_id == "body-0-1")
                disc = object.id;
        RIGIDBODIES_EXPECT(disc.is_valid(), "the elastic left disc is in the scene");
        const auto field = control("object.motion.velocity_x", "guide_body-0-1") + "--field";
        const auto starting = harness.document().element_value(field);
        RIGIDBODIES_EXPECT(starting.has_value() && !starting->empty(), "the Guide shows the disc's incoming speed");
        const auto starting_speed = harness.session.world().find_body(disc)->linear_velocity_m_s().x;

        ui::UiCommand play;
        play.kind = ui::UiCommandKind::toggle_pause;
        harness.apply_command(play);
        harness.frames_for(100);
        harness.apply_command(play);
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.session.world().find_body(disc)->linear_velocity_m_s().x < 0.0, "the disc has rebounded");
        RIGIDBODIES_EXPECT(harness.document().element_value(field) == starting, "after the rebound the field still shows the starting speed, not the live one");

        const auto elapsed = harness.session.build_model().elapsed_time_s;
        const auto live_velocity = harness.session.world().find_body(disc)->linear_velocity_m_s();
        ui::UiCommand edit;
        edit.kind = ui::UiCommandKind::set_selected_velocity_x;
        edit.body = disc;
        edit.value = starting_speed + 1.0;
        edit.detail = "setup";
        harness.apply_command(edit);
        harness.frames_for(1);
        const auto setup_speed = [&]
        {
            const auto model = harness.session.build_model();
            const auto* body = model.setup_world ? model.setup_world->find_body(disc) : nullptr;
            return body ? body->linear_velocity_m_s().x : 0.0;
        };
        const auto live_after = harness.session.world().find_body(disc)->linear_velocity_m_s();
        RIGIDBODIES_EXPECT(live_after.x == live_velocity.x && live_after.y == live_velocity.y && harness.session.build_model().elapsed_time_s == elapsed, "the run in progress is left as it was");
        RIGIDBODIES_EXPECT(setup_speed() == starting_speed + 1.0 && harness.document().element_value(field) != starting, "the setup, and the field, take the new starting speed");

        ui::UiCommand undo;
        undo.kind = ui::UiCommandKind::undo;
        harness.apply_command(undo);
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(setup_speed() == starting_speed && harness.session.build_model().elapsed_time_s == elapsed, "undo restores the starting speed without rewinding the run");

        ui::UiCommand redo;
        redo.kind = ui::UiCommandKind::redo;
        harness.apply_command(redo);
        ui::UiCommand back;
        back.kind = ui::UiCommandKind::reset_scenario;
        harness.apply_command(back);
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.session.world().find_body(disc)->linear_velocity_m_s().x == starting_speed + 1.0, "Back to start replays the new starting speed");
    }

    RIGIDBODIES_TEST("where the Inspector cannot dock beside the Guide, the status line's selection count opens it")
    {
        LearnerInterface harness({}, { 800, 600 }, 1.0f);
        RIGIDBODIES_EXPECT(harness.session.load_scenario("ramp"), "the ramp loads");
        harness.frames_for(2);
        for (const auto& object : harness.session.build_model().objects)
            if (object.kind == "free")
            {
                harness.session.set_selection(object.id);
                break;
            }
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::guide_panel) != nullptr && harness.interface.layout().find(ui::RegionId::inspector) == nullptr && harness.interface.layout().find(ui::RegionId::inspector_rail) == nullptr, "the Guide keeps the dock and the Inspector has no rail");
        const auto inspect = control("status.selection_inspect");
        RIGIDBODIES_EXPECT(harness.visible_bounds(inspect).has_value(), "the status line offers the Inspector for the selection");
        RIGIDBODIES_EXPECT(harness.click_element(inspect), "the offer can be pressed");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::inspector) != nullptr, "pressing it brings the Inspector into view");
    }

    RIGIDBODIES_TEST("Present on a short window sets the lesson card compactly")
    {
        for (const auto& [scale, short_window] : { std::pair<float, bool> { 1.5f, true }, std::pair<float, bool> { 1.0f, false } })
        {
            LearnerInterface harness({}, { 1280, 720 }, scale);
            harness.interface.request(ui::ViewRequest::toggle_present);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.document().document_has_class("present-short") == short_window, short_window ? "1280 x 720 at 150 % is short, so the card is compact" : "1280 x 720 at 100 % keeps the full-size card");
            const auto* caption = harness.interface.layout().find(ui::RegionId::present_caption);
            RIGIDBODIES_EXPECT(caption != nullptr && caption->bounds.height() <= 0.35 * 720.0 + 0.5, "the caption box leaves the demonstration two thirds of the window");
        }
    }

    RIGIDBODIES_TEST("Open World focuses World's own Gravity, not the copy the Guide offers")
    {
        LearnerInterface harness;
        RIGIDBODIES_EXPECT(harness.session.load_scenario("targeted_force"), "the targeted force scene loads");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("world.gravity.enabled", "guide")).has_value(), "the Guide offers its own Gravity switch");
        harness.interface.request(ui::ViewRequest::open_world);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("world.gravity.enabled"), "World's Gravity row takes the focus");
    }

    RIGIDBODIES_TEST("the impact count on Measure never makes the command bar shed or move its controls")
    {
        LearnerInterface harness({}, { 1024, 768 }, 1.0f);
        RIGIDBODIES_EXPECT(harness.session.load_scenario("free_fall"), "free fall loads");
        harness.frames_for(2);
        const auto snapshot = [&]
        {
            std::vector<std::string> state;
            for (const auto& row : command_bar_rows(harness))
            {
                const auto bounds = harness.visible_bounds(row.id);
                state.push_back(row.id + (row.folded ? " folded" : "") + (row.label_hidden ? " icon" : "") + (bounds ? " at " + std::to_string(static_cast<int>(bounds->minimum.x)) : std::string {}));
            }
            return state;
        };
        const auto before = snapshot();
        ui::UiCommand play;
        play.kind = ui::UiCommandKind::toggle_pause;
        harness.apply_command(play);
        harness.frames_for(150);
        RIGIDBODIES_EXPECT(!harness.session.build_model().impacts.empty() && harness.document().element_has_class(control("view.measure"), "has-badge"), "a recorded impact puts a count on Measure");
        RIGIDBODIES_EXPECT(snapshot() == before, "every bar control keeps its place, label and fold when the count appears");
    }

    const ui::StageHandle* find_handle(const ui::UiModel& model, std::string_view id)
    {
        const auto found = std::find_if(model.handles.begin(), model.handles.end(), [&](const auto& handle)
            {
                return handle.id == id;
            });
        return found == model.handles.end() ? nullptr : &*found;
    }

    // Vertex handles are squares; each outline gives one node's half size.
    std::vector<float> node_radii(const render::DrawList& list)
    {
        std::vector<float> radii;
        for (const auto& command : list.commands())
            if (command.kind == render::DrawCommandKind::rectangle_outline && command.vertex_count >= 2)
            {
                const auto& first = list.vertices()[command.vertex_offset];
                const auto& second = list.vertices()[command.vertex_offset + 1];
                radii.push_back(static_cast<float>(std::abs(second.x - first.x) * 0.5));
            }
        std::sort(radii.begin(), radii.end());
        return radii;
    }

    RIGIDBODIES_TEST("stage handles and draft nodes scale linearly with display scale times text size")
    {
        double reference_handle = 0.0, reference_inset = 0.0;
        float reference_node = 0.0f;
        for (const auto& [display, text] : { std::pair { 1.0f, 1.0 }, std::pair { 2.0f, 1.0 }, std::pair { 1.0f, 1.5 }, std::pair { 1.5f, 2.0 } })
        {
            core::ApplicationConfig config;
            config.interface_settings.interface_scale = text;
            app::SimulationSession session;
            session.configure(config);
            session.set_viewport({ 1600, 900 });
            session.scene_settings().display_scale = display;
            const auto factor = static_cast<double>(display) * text;
            const auto model = session.build_model();
            const auto* gravity = find_handle(model, "gravity");
            RIGIDBODIES_EXPECT(gravity != nullptr, "the gravity handle is published");
            if (reference_handle == 0.0)
                reference_handle = gravity->screen_bounds.width;
            RIGIDBODIES_EXPECT_NEAR(gravity->screen_bounds.width, reference_handle * factor, 1e-9, "the gravity handle grows with display scale times text size");
            const auto inset = 1600.0 - (gravity->screen_bounds.left + gravity->screen_bounds.width * 0.5);
            if (reference_inset == 0.0)
                reference_inset = inset;
            RIGIDBODIES_EXPECT_NEAR(inset, reference_inset * factor, 1e-9, "the gravity handle keeps its scaled inset from the corner");

            // An empty world leaves the draft's vertex handles as the only square outlines in the frame.
            session.world().clear();
            ui::UiCommand draw;
            draw.kind = ui::UiCommandKind::start_new_shape;
            session.apply(draw);
            for (const auto& point : { math::Vec2 { 700.0, 400.0 }, math::Vec2 { 900.0, 420.0 } })
                for (const auto kind : { ui::UiEventKind::pointer_down, ui::UiEventKind::pointer_up })
                {
                    ui::UiEvent press;
                    press.kind = kind;
                    press.pointer_px = point;
                    press.logical_pixel_scale = factor;
                    (void)session.handle_scene_event(press);
                }
            render::DrawList list;
            session.render(list);
            const auto radii = node_radii(list);
            RIGIDBODIES_EXPECT(!radii.empty(), "the draft draws its nodes");
            if (reference_node == 0.0f)
                reference_node = radii.front();
            RIGIDBODIES_EXPECT_NEAR(radii.front(), reference_node * factor, 1e-4, "draft nodes grow with display scale times text size");
        }
    }

    RIGIDBODIES_TEST("the shape editor sizes nodes and handles from its view scale")
    {
        app::ShapeEditor editor;
        physics::Outline outline;
        for (const auto& point : { math::Vec2 { 0.0, 0.0 }, math::Vec2 { 1.0, 0.0 }, math::Vec2 { 0.5, 1.0 } })
        {
            physics::OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        editor.begin(outline);
        render::Camera2D camera;
        camera.set_viewport({ 800, 600 });
        const render::Theme theme;
        std::vector<float> reference;
        for (const auto scale : { 1.0, 2.5 })
        {
            editor.set_view_scale(scale);
            render::DrawList list;
            editor.draw(camera, theme, list);
            const auto radii = node_radii(list);
            RIGIDBODIES_EXPECT(radii.size() == 3, "each node draws one square handle");
            if (reference.empty())
                reference = radii;
            for (std::size_t index = 0; index < radii.size(); ++index)
                RIGIDBODIES_EXPECT_NEAR(radii[index], reference[index] * scale, 1e-4, "node handles scale with the view scale");
        }
    }

    RIGIDBODIES_TEST("a context menu opens at the pointer and stays inside the window near every corner")
    {
        for (const auto& [size, scale] : { std::pair { render::ViewportSize { 1600, 900 }, 1.5f }, std::pair { render::ViewportSize { 800, 600 }, 1.0f } })
        {
            LearnerInterface harness({}, size, scale);
            // With both side surfaces and the Quick start hint put away the stage reaches the
            // window's side edges and nothing covers its corners.
            harness.interface.view_state().set_surface_open("inspector.open", false);
            harness.interface.view_state().set_surface_open("guide.expanded", false);
            harness.interface.view_state().set_surface_open("hints.enabled", false);
            harness.frames_for(2);
            const ui::Rect window { { 0.0, 0.0 }, { static_cast<double>(size.width), static_cast<double>(size.height) } };
            RIGIDBODIES_EXPECT(window.maximum.x - harness.interface.layout().stage.maximum.x < 1.0, "the stage reaches the right edge" + at(size, scale));
            for (const auto& [x, y] : { std::pair { 0.02, 0.03 }, std::pair { 0.98, 0.03 }, std::pair { 0.02, 0.9 }, std::pair { 0.98, 0.9 }, std::pair { 0.5, 0.5 } })
            {
                const auto point = harness.stage_point(x, y);
                const auto where = at(size, scale) + " for a press at " + std::to_string(static_cast<int>(point.x)) + "," + std::to_string(static_cast<int>(point.y));
                harness.click(point, ui::PointerButton::secondary);
                harness.frames_for(2);
                RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu"), "a right-click on the stage opens the context menu" + where);
                const auto menu = harness.visible_bounds("panel-context_menu");
                RIGIDBODIES_EXPECT(menu.has_value(), "the context menu is displayed" + where);
                RIGIDBODIES_EXPECT(inside(*menu, window), "the context menu lies inside the window" + where);
                // The popover surface, not the card within its padding, is what opens at the pointer.
                const auto surface = harness.visible_bounds("region-popovers");
                RIGIDBODIES_EXPECT(surface && inside(*surface, window) && inside(*menu, *surface), "the popover surface holds the menu inside the window" + where);
                const auto slack = 8.0 * scale;
                RIGIDBODIES_EXPECT(point.x >= surface->minimum.x - slack && point.x <= surface->maximum.x + slack, "the menu spans the pointer horizontally" + where);
                const auto below = std::abs(surface->minimum.y - point.y) <= slack;
                const auto above = std::abs(surface->maximum.y - point.y) <= slack;
                RIGIDBODIES_EXPECT(below || above, "the menu opens just below the pointer or flips just above it" + where);
                if (x < 0.5 && y < 0.5)
                    RIGIDBODIES_EXPECT(below && std::abs(surface->minimum.x - point.x) <= slack, "away from the edges the menu opens at the pointer" + where);
                if (point.x + surface->width() > window.maximum.x)
                    RIGIDBODIES_EXPECT(surface->minimum.x < point.x, "near the right edge the menu shifts left to stay in the window" + where);
                if (point.y + surface->height() > window.maximum.y)
                    RIGIDBODIES_EXPECT(above, "near the bottom edge the menu flips above the pointer" + where);
                harness.key(ui::UiKey::escape);
                harness.frame();
                RIGIDBODIES_EXPECT(!harness.interface.view_state().transient_open("context_menu"), "Escape closes the context menu" + where);
            }
        }
    }

    RIGIDBODIES_TEST("a context menu stays beside its object when selecting the object hands the dock to the Inspector")
    {
        for (const auto& [size, scale] : { std::pair { render::ViewportSize { 1600, 900 }, 1.5f }, std::pair { render::ViewportSize { 1280, 720 }, 1.0f } })
        {
            LearnerInterface harness({}, size, scale);
            harness.interface.view_state().set_surface_open("hints.enabled", false);
            RIGIDBODIES_EXPECT(harness.session.load_scenario("ramp"), "the ramp lesson loads");
            harness.frames_for(2);
            const auto where = at(size, scale);
            RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::guide_panel) != nullptr, "the Guide owns the dock before anything is selected" + where);
            physics::BodyId block;
            harness.session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.name() == "block")
                        block = id;
                });
            RIGIDBODIES_EXPECT(block.is_valid(), "the lesson has its block");
            const auto centre = [&]()
            {
                return harness.session.camera().world_to_screen(harness.session.world().find_body(block)->world_center_of_mass_m());
            };
            const auto pressed = centre();
            harness.click(pressed, ui::PointerButton::secondary);
            harness.frames_for(3);
            RIGIDBODIES_EXPECT(harness.session.selection() == block && harness.interface.layout().find(ui::RegionId::inspector) != nullptr, "the right-click selects the block and the Inspector takes the dock" + where);
            RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu"), "the context menu is open" + where);
            const auto now = centre();
            RIGIDBODIES_EXPECT(std::abs(now.x - pressed.x) > 40.0 * scale, "the stage is reframed for the new dock while the menu is open" + where);
            const auto surface = harness.visible_bounds("region-popovers");
            const auto slack = 8.0 * scale;
            RIGIDBODIES_EXPECT(surface && now.x >= surface->minimum.x - slack && now.x <= surface->maximum.x + slack, "the menu still spans its block horizontally" + where);
            RIGIDBODIES_EXPECT(surface && (std::abs(surface->minimum.y - now.y) <= slack || std::abs(surface->maximum.y - now.y) <= slack), "the menu still opens just below or above its block" + where);
        }
    }

    RIGIDBODIES_TEST("a hover card stands clear of the other objects and no stage label shows past its edge")
    {
        for (const auto text_scale : { 1.0, 2.0 })
        {
            core::ApplicationConfig config;
            config.interface_settings.interface_scale = text_scale;
            const render::ViewportSize size { 1920, 1080 };
            LearnerInterface harness(config, size, 1.0f);
            harness.interface.view_state().set_surface_open("hints.enabled", false);
            RIGIDBODIES_EXPECT(harness.session.load_scenario("collision_comparison"), "the collision lesson loads");
            harness.frames_for(2);
            const auto where = " with text at " + std::to_string(text_scale) + "x";
            const auto screen_box = [&](const physics::RigidBody& body)
            {
                const auto bounds = body.compute_bounds();
                const auto a = harness.session.camera().world_to_screen(bounds.minimum), b = harness.session.camera().world_to_screen(bounds.maximum);
                return ui::Rect { math::min_components(a, b), math::max_components(a, b) };
            };
            const auto overlaps = [](const ui::Rect& first, const ui::Rect& second)
            {
                return first.minimum.x < second.maximum.x && second.minimum.x < first.maximum.x && first.minimum.y < second.maximum.y && second.minimum.y < first.maximum.y;
            };
            physics::BodyId hovered;
            harness.session.world().for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.name() == "Elastic left")
                        hovered = id;
                });
            RIGIDBODIES_EXPECT(hovered.is_valid(), "the lesson has its elastic left disc");
            harness.pointer(ui::UiEventKind::pointer_move, harness.session.camera().world_to_screen(harness.session.world().find_body(hovered)->world_center_of_mass_m()));
            harness.frames_for(50);
            const auto card = harness.interface.hover_card_bounds();
            RIGIDBODIES_EXPECT(card.has_value() && card->width() > 0.0, "the hover card is shown" + where);
            harness.session.world().for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    RIGIDBODIES_EXPECT(!overlaps(*card, screen_box(body)), "the card covers neither its own object nor its neighbours " + std::string(body.name()) + where);
                });
            for (const auto& command : harness.scene_list.commands())
                if (command.kind == render::DrawCommandKind::rounded_rectangle_fill && command.vertex_count == 2)
                {
                    const ui::Rect plate { harness.scene_list.vertices()[command.vertex_offset], harness.scene_list.vertices()[command.vertex_offset + 1] };
                    RIGIDBODIES_EXPECT(!overlaps(*card, plate), "no stage label plate lies under the card" + where);
                }
        }
    }

    RIGIDBODIES_TEST("at common laptop sizes the Guide owns the dock when an experiment opens, until an object is selected")
    {
        for (const auto& [size, scale] : { std::pair { render::ViewportSize { 1366, 768 }, 1.0f }, std::pair { render::ViewportSize { 1536, 864 }, 1.25f }, std::pair { render::ViewportSize { 1920, 1080 }, 1.5f } })
        {
            LearnerInterface harness({}, size, scale);
            const auto where = at(size, scale);
            const auto& layout = harness.interface.layout();
            RIGIDBODIES_EXPECT(layout.mode == ui::LayoutMode::medium, "a laptop window is the medium layout" + where);
            RIGIDBODIES_EXPECT(layout.find(ui::RegionId::guide_panel) && harness.document().element_visible("panel-guide"), "the opened experiment shows its Guide" + where);
            RIGIDBODIES_EXPECT(!layout.find(ui::RegionId::inspector) && layout.find(ui::RegionId::inspector_rail), "the Inspector waits on its rail" + where);

            ui::UiCommand select;
            select.kind = ui::UiCommandKind::select_body;
            select.body = first_free_object(harness);
            harness.apply_command(select);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::inspector) && harness.interface.layout().find(ui::RegionId::guide_rail), "selecting an object hands the dock to the Inspector" + where);

            harness.key(ui::UiKey::g);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::guide_panel) && !harness.interface.layout().find(ui::RegionId::inspector), "asking for the Guide brings it back" + where);

            ui::UiCommand open;
            open.kind = ui::UiCommandKind::load_scenario;
            open.id = "ramp";
            harness.key(ui::UiKey::i);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::inspector) != nullptr, "asking for the Inspector brings it forward" + where);
            harness.apply_command(open);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::guide_panel) != nullptr, "opening another experiment shows its Guide again" + where);
        }
    }

    RIGIDBODIES_TEST("a long experiment title is shortened in the command bar rather than removed")
    {
        const auto title_row = [](const LearnerInterface& harness) -> std::string
        {
            for (const auto& id : harness.document().child_ids("panel-command_bar"))
                if (harness.document().element_attribute(id, "data-row-kind").value_or("") == "title")
                    return id;
            return {};
        };
        LearnerInterface harness({}, { 1600, 900 }, 1.5f);
        ui::UiCommand open;
        open.kind = ui::UiCommandKind::load_scenario;
        open.id = "mass_distribution";
        harness.apply_command(open);
        harness.frames_for(3);
        const auto full = harness.session.build_model().scenario_title;
        const auto title = title_row(harness);
        RIGIDBODIES_EXPECT(!title.empty() && harness.document().element_visible(title), "the title stays in the bar at 1600 by 900 and 1.5x");
        const auto shown = harness.document().element_text(title);
        RIGIDBODIES_EXPECT(shown.size() > 3 && shown.compare(shown.size() - 3, 3, "\xE2\x80\xA6") == 0 && full.rfind(shown.substr(0, shown.size() - 3), 0) == 0, "the title is shortened with an ellipsis: " + shown);
        RIGIDBODIES_EXPECT(harness.document().element_attribute(title, "data-tooltip").value_or("") == full, "the shortened title shows its full text as a tooltip");
        for (const auto& id : harness.document().child_ids("panel-command_bar"))
            if (harness.document().element_attribute(id, "data-control-key").value_or("") == "tools.mode")
                RIGIDBODIES_EXPECT(!harness.document().element_has_class(id, "label-hidden"), "the title gives up width before the tool labels go");

        harness.resize({ 1920, 1080 }, 1.0f);
        harness.frames_for(3);
        RIGIDBODIES_EXPECT(harness.document().element_text(title_row(harness)) == full && !harness.document().element_attribute(title_row(harness), "data-tooltip"), "with room again the full title returns");
    }

    RIGIDBODIES_TEST("Measure hides from a control at the end of its tab row")
    {
        for (const auto& [size, scale] : { std::pair { render::ViewportSize { 1600, 900 }, 1.0f }, std::pair { render::ViewportSize { 700, 900 }, 1.0f } })
        {
            LearnerInterface harness({}, size, scale);
            const auto where = at(size, scale);
            harness.key(ui::UiKey::m);
            harness.frames_for(2);
            const auto drawer = region_bounds(harness, ui::RegionId::measure_drawer);
            const auto id = harness.document().element_for_key("legacy", "view.measure_collapse");
            RIGIDBODIES_EXPECT(id.has_value(), "Measure has a hide control" + where);
            if (!id)
                continue;
            const auto close = harness.visible_bounds(*id);
            RIGIDBODIES_EXPECT(close && inside(*close, drawer), "the hide control lies on the Measure drawer" + where);
            RIGIDBODIES_EXPECT(close && close->maximum.x > drawer.maximum.x - 60.0 * scale && close->minimum.y < drawer.minimum.y + 60.0 * scale, "the hide control sits at the end of the tab row" + where);
            RIGIDBODIES_EXPECT(harness.click_element(*id), "the hide control can be pressed" + where);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.interface.view_state().surface_open("measure.open", false), "pressing it hides Measure" + where);
        }
    }

    RIGIDBODIES_TEST("the selected Library card holds its own action")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.0f);
        harness.key(ui::UiKey::l);
        harness.frames_for(3);
        const auto scenario = harness.session.build_model().scenario_id;
        const auto card = harness.document().element_for_key("legacy", "library.cards.card", scenario);
        RIGIDBODIES_EXPECT(card.has_value(), "the current experiment has a card");
        if (!card)
            return;
        const auto card_bounds = harness.visible_bounds(*card);
        const auto action = harness.visible_bounds(*card + "--action_0");
        RIGIDBODIES_EXPECT(card_bounds && action && inside(*action, *card_bounds), "Back to experiment lies inside the selected card's highlight");
        RIGIDBODIES_EXPECT(harness.click_element(*card + "--action_0"), "the card's action can be pressed");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("library"), "Back to experiment closes the Library");
    }

    ui::Vec2 centre(const ui::Rect& box)
    {
        return { (box.minimum.x + box.maximum.x) * 0.5, (box.minimum.y + box.maximum.y) * 0.5 };
    }

    bool overlaps(const ui::Rect& a, const ui::Rect& b)
    {
        return a.minimum.x < b.maximum.x && b.minimum.x < a.maximum.x && a.minimum.y < b.maximum.y && b.minimum.y < a.maximum.y;
    }

    RIGIDBODIES_TEST("as the window's title bar the strip keeps its toolbar clear of the app mark and the window controls")
    {
        LearnerInterface harness({}, { 1920, 900 }, 1.0f);
        harness.window_frame.controls = true;
        harness.frames_for(2);
        // Down to the window's minimum size, 560 by 320 dp, at every text size.
        for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 1920, 900 }, 1.0f }, { { 1600, 900 }, 1.5f }, { { 1280, 800 }, 1.0f }, { { 1080, 800 }, 1.0f }, { { 900, 700 }, 1.0f }, { { 760, 600 }, 1.0f }, { { 640, 480 }, 1.0f }, { { 560, 320 }, 1.0f }, { { 840, 480 }, 1.5f }, { { 1120, 640 }, 2.0f } })
        {
            harness.resize(size, scale);
            harness.frames_for(2);
            const auto where = at(size, scale);
            const auto title_bar = region_bounds(harness, ui::RegionId::title_bar);
            const auto brand = region_bounds(harness, ui::RegionId::window_brand);
            const auto controls = region_bounds(harness, ui::RegionId::window_controls);
            const auto bar = region_bounds(harness, ui::RegionId::command_bar);
            for (const auto* id : { "window-minimize", "window-maximize", "window-close" })
            {
                const auto bounds = harness.visible_bounds(id);
                // The controls stop 1 dp short of the strip's hairline.
                RIGIDBODIES_EXPECT(bounds && inside(*bounds, controls) && bounds->height() >= title_bar.height() - 1.0 * scale - 1.0, std::string("the window control ") + id + " fills the strip's height inside its region" + where);
            }
            const auto close = harness.visible_bounds("window-close");
            RIGIDBODIES_EXPECT(close && std::abs(close->maximum.x - size.width) <= 1.0, "the close button reaches the window's corner" + where);
            for (const auto& row : command_bar_rows(harness))
                if (const auto bounds = harness.visible_bounds(row.id); bounds && !row.folded)
                    RIGIDBODIES_EXPECT(inside(*bounds, bar) && !overlaps(*bounds, brand) && !overlaps(*bounds, controls), "the toolbar keeps clear of the mark and the controls: " + row.id + where);
            for (const auto* key : { "transport.play", "view.menu" })
            {
                const auto bounds = harness.visible_bounds(control(key));
                RIGIDBODIES_EXPECT(bounds.has_value() && inside(*bounds, bar), std::string("the bar still shows ") + key + where);
            }
        }
        harness.interface.request(ui::ViewRequest::toggle_present);
        for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 1600, 900 }, 1.5f }, { { 1280, 800 }, 1.0f }, { { 900, 700 }, 1.0f }, { { 640, 480 }, 1.0f }, { { 560, 320 }, 1.0f }, { { 1120, 640 }, 2.0f } })
        {
            harness.resize(size, scale);
            harness.frames_for(2);
            const auto where = at(size, scale);
            const auto strip = region_bounds(harness, ui::RegionId::present_strip);
            const auto controls = region_bounds(harness, ui::RegionId::window_controls);
            RIGIDBODIES_EXPECT(std::abs(strip.maximum.x - controls.minimum.x) <= 1.0 && harness.visible_bounds("window-close").has_value(), "Present's strip ends where the window controls begin" + where);
            for (const auto& id : harness.document().child_ids("panel-present"))
                if (const auto bounds = harness.visible_bounds(id); bounds && !harness.document().element_has_class(id, "is-overflowed"))
                    RIGIDBODIES_EXPECT(inside(*bounds, strip) && !overlaps(*bounds, controls), "every Present control stays in its strip, clear of the window controls: " + id + where);
            RIGIDBODIES_EXPECT(harness.visible_bounds(control("present.transport.exit")).has_value(), "Exit stays in the strip" + where);
            if (size.width >= 1280)
                RIGIDBODIES_EXPECT(!harness.document().element_has_class(control("present.transport.exit"), "label-hidden"), "with room to spare Exit keeps its label beside the window's close button" + where);
        }
    }

    RIGIDBODIES_TEST("the title bar tells the platform which points move the window and which are controls")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.5f);
        harness.window_frame.controls = true;
        harness.frames_for(2);
        const auto part = [&](ui::Vec2 point)
        {
            return harness.interface.window_part(point);
        };
        using P = ui::WindowPart;
        const std::array<std::pair<const char*, P>, 3> buttons { std::pair { "window-minimize", P::minimize }, std::pair { "window-maximize", P::maximize }, std::pair { "window-close", P::close } };
        for (const auto& [id, expected] : buttons)
        {
            const auto bounds = harness.visible_bounds(id);
            RIGIDBODIES_EXPECT(bounds && part(centre(*bounds)) == expected, std::string("the platform hears ") + id + " as its caption button");
        }
        const auto controls = region_bounds(harness, ui::RegionId::window_controls);
        RIGIDBODIES_EXPECT(part({ controls.minimum.x + 2.0, controls.minimum.y + 20.0 }) == P::title_bar, "the gap before the divider moves the window");
        RIGIDBODIES_EXPECT(part(centre(region_bounds(harness, ui::RegionId::window_brand))) == P::title_bar, "the app mark moves the window");
        std::optional<ui::Rect> title, spacer;
        for (const auto& row : command_bar_rows(harness))
        {
            if (row.kind == "title" && !row.folded)
                title = harness.visible_bounds(row.id);
            if (row.kind == "spacer")
                spacer = harness.document().element_bounds(row.id);
        }
        RIGIDBODIES_EXPECT(title && part(centre(*title)) == P::title_bar, "the scenario's title moves the window");
        RIGIDBODIES_EXPECT(spacer && part({ centre(*spacer).x, region_bounds(harness, ui::RegionId::command_bar).minimum.y + 20.0 }) == P::title_bar, "the stretch between the groups moves the window");
        const auto play = harness.visible_bounds(control("transport.play"));
        RIGIDBODIES_EXPECT(play && part(centre(*play)) == P::client, "Play is a control");
        RIGIDBODIES_EXPECT(play && part({ centre(*play).x, region_bounds(harness, ui::RegionId::title_bar).minimum.y + 1.0 }) == P::client, "so is its whole column, so a press just above it never drags or maximizes the window");
        RIGIDBODIES_EXPECT(part(harness.stage_point(0.5, 0.5)) == P::client, "the stage is the interface");
        harness.interface.request(ui::ViewRequest::open_main_menu);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(title && part(centre(*title)) == P::client, "with a menu open a press on the strip closes the menu instead");
        harness.interface.request(ui::ViewRequest::close_top_surface);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(title && part(centre(*title)) == P::title_bar, "and the title moves the window again once it closes");
        // A press on the bare title bar closes an open menu, as on a native title bar.
        for (const auto request : { ui::ViewRequest::open_main_menu, ui::ViewRequest::toggle_show })
        {
            harness.interface.request(request);
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(harness.interface.menu_open(), "the menu is open");
            RIGIDBODIES_EXPECT(title && harness.interface.dismiss_menus_at(centre(*title)), "a press on the title closes it");
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.interface.menu_open() && title && part(centre(*title)) == P::title_bar, "and the title moves the window again");
        }
        // A fixed-size window's maximize button is greyed out and is title bar.
        harness.window_frame.resizable = false;
        harness.frames_for(2);
        const auto maximize = harness.visible_bounds("window-maximize");
        RIGIDBODIES_EXPECT(maximize && harness.document().element_has_class("window-maximize", "is-disabled") && part(centre(*maximize)) == P::title_bar, "a fixed-size window cannot be maximized from its title bar");
        const auto commands = harness.document().emitted_commands();
        RIGIDBODIES_EXPECT(std::none_of(commands.begin(), commands.end(), [](const ui::UiCommand& command)
                               {
                                   return command.kind == ui::UiCommandKind::toggle_maximize_window;
                               }),
            "nor send the maximize command");
        harness.window_frame.resizable = true;
        harness.window_frame.controls = false;
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(title && part(centre(*title)) == P::client && !harness.visible_bounds("window-close"), "under the platform's own title bar the strip is all interface");
    }

    RIGIDBODIES_TEST("a title shortened to fit can still be hovered for its full name")
    {
        LearnerInterface harness({}, { 1920, 1080 }, 1.0f);
        harness.window_frame.controls = true;
        harness.frames_for(2);
        const auto title_row = [&]() -> std::string
        {
            for (const auto& row : command_bar_rows(harness))
                if (row.kind == "title" && !row.folded)
                    return row.id;
            return {};
        };
        harness.session.load_scenario("mass_distribution");
        harness.frames_for(2);
        auto title = title_row();
        RIGIDBODIES_EXPECT(!title.empty() && !harness.document().element_attribute(title, "data-tooltip"), "a wide window shows the whole title");
        if (const auto bounds = harness.visible_bounds(title))
            RIGIDBODIES_EXPECT(harness.interface.window_part(centre(*bounds)) == ui::WindowPart::title_bar, "a title shown in full moves the window");
        harness.resize({ 1600, 900 }, 1.5f);
        harness.frames_for(2);
        title = title_row();
        const auto bounds = title.empty() ? std::nullopt : harness.visible_bounds(title);
        RIGIDBODIES_EXPECT(bounds && harness.document().element_attribute(title, "data-tooltip").has_value(), "a narrower window shortens the title and keeps its full name as a tooltip");
        RIGIDBODIES_EXPECT(bounds && harness.interface.window_part(centre(*bounds)) == ui::WindowPart::client, "so the shortened title can be hovered rather than dragged");
    }

    RIGIDBODIES_TEST("menus and cards open below the window's own title bar, never under its controls")
    {
        LearnerInterface harness({}, { 560, 320 }, 1.0f);
        harness.window_frame.controls = true;
        // Nothing covers the stage's right edge, as in the context menu placement test.
        harness.interface.view_state().set_surface_open("inspector.open", false);
        harness.interface.view_state().set_surface_open("guide.expanded", false);
        harness.interface.view_state().set_surface_open("hints.enabled", false);
        harness.frames_for(2);
        harness.click(harness.stage_point(0.97, 0.45), ui::PointerButton::secondary);
        harness.frames_for(2);
        const auto title_bar = region_bounds(harness, ui::RegionId::title_bar);
        const auto menu = harness.visible_bounds("region-popovers");
        RIGIDBODIES_EXPECT(harness.interface.view_state().transient_open("context_menu") && menu.has_value(), "a right click near the window's right edge opens the stage menu");
        RIGIDBODIES_EXPECT(menu && menu->minimum.y >= title_bar.maximum.y - 0.5, "the menu flips up only as far as the title bar's foot");
    }

    RIGIDBODIES_TEST("a window control's tooltip shows on hover and hides while it is pressed")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.0f);
        harness.window_frame.controls = true;
        harness.window_frame.hot = ui::WindowPart::close;
        harness.frames_for(45);
        RIGIDBODIES_EXPECT(harness.visible_bounds("tooltip").has_value() && harness.document().element_text("tooltip-text") == "Close", "resting on Close names it");
        harness.window_frame.pressed = ui::WindowPart::close;
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.visible_bounds("tooltip").has_value(), "pressing it hides the tooltip");
        harness.window_frame.pressed = ui::WindowPart::client;
        harness.frames_for(45);
        RIGIDBODIES_EXPECT(!harness.visible_bounds("tooltip").has_value(), "and it stays hidden until the pointer moves to another control");
    }

    RIGIDBODIES_TEST("the window controls send the window's commands and show the window's state")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.0f);
        harness.window_frame.controls = true;
        harness.frames_for(2);
        const auto serial = harness.session.build_model().change_serial;
        RIGIDBODIES_EXPECT(harness.click_element("window-minimize"), "minimize can be pressed");
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.click_element("window-maximize"), "maximize can be pressed");
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.window_requests == std::vector<ui::UiCommandKind> { ui::UiCommandKind::minimize_window, ui::UiCommandKind::toggle_maximize_window }, "each control asks for its own change to the window (" + std::to_string(harness.window_requests.size()) + " asked)");
        RIGIDBODIES_EXPECT(harness.session.build_model().change_serial == serial, "working the window edits nothing");
        RIGIDBODIES_EXPECT(harness.click_element("window-close"), "close can be pressed");
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.session.should_quit(), "close quits the way Quit does");
        RIGIDBODIES_EXPECT(harness.document().element_attribute("window-maximize", "data-tooltip").value_or("") == "Maximize", "an ordinary window offers to maximize");
        harness.window_frame.maximized = true;
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.document().element_attribute("window-maximize", "data-tooltip").value_or("") == "Restore down", "a maximized window offers to restore");
        harness.window_frame.active = false;
        harness.window_frame.hot = ui::WindowPart::close;
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.document().document_has_class("window-inactive"), "an inactive window dims its title bar");
        RIGIDBODIES_EXPECT(harness.document().element_has_class("window-close", "is-hot") && !harness.document().element_has_class("window-minimize", "is-hot"), "the control the platform reports under the pointer is highlighted");
        harness.window_frame.pressed = ui::WindowPart::close;
        harness.frames_for(1);
        RIGIDBODIES_EXPECT(harness.document().element_has_class("window-close", "is-pressed"), "and shows its press");
        harness.window_frame = {};
        harness.window_frame.controls = true;
        // A large setup rebuilds its panels only every two seconds; the window's state still shows
        // on the very next frame.
        for (int index = 0; index < 30; ++index)
        {
            physics::BodyDefinition definition;
            definition.type = physics::BodyType::dynamic_body;
            definition.position_m = { -3.0 + 0.2 * index, 2.0 };
            physics::Collider collider;
            collider.shape = physics::make_circle(0.05);
            definition.colliders.push_back(collider);
            harness.session.world().create_body(definition);
        }
        harness.frames_for(3);
        harness.window_frame.maximized = true;
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().element_attribute("window-maximize", "data-tooltip").value_or("") == "Restore down", "a large setup shows a maximize at once");
    }

    RIGIDBODIES_TEST("every displayed toolbar control remains a client hit across window sizes")
    {
        LearnerInterface harness;
        harness.window_frame.controls = true;
        for (const auto present : { false, true })
        {
            if (present)
                harness.interface.request(ui::ViewRequest::toggle_present);
            for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 1920, 1080 }, 1.0f }, { { 1600, 900 }, 1.5f }, { { 900, 700 }, 1.0f }, { { 560, 320 }, 1.0f }, { { 1120, 640 }, 2.0f } })
            {
                harness.resize(size, scale);
                harness.frames_for(2);
                const auto bar = region_bounds(harness, present ? ui::RegionId::present_strip : ui::RegionId::command_bar);
                for (const auto& [id, point] : harness.visible_controls())
                    if (bar.contains(point))
                    {
                        const auto hit = harness.document().element_at(point);
                        const auto where = " for " + id + at(size, scale);
                        RIGIDBODIES_EXPECT(hit == id || hit.rfind(id + "--", 0) == 0, "the visible control receives the pointer instead of " + hit + where);
                        RIGIDBODIES_EXPECT(harness.interface.window_part(point) == ui::WindowPart::client, "the visible control never starts a window drag" + where);
                    }
            }
        }
    }

    RIGIDBODIES_TEST("Library search accepts separate words and explicit filters can be cleared together")
    {
        LearnerInterface harness;
        harness.interface.request(ui::ViewRequest::open_library);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("library.top.search") + "--field"), "the Library search accepts typing");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("library.top.search") + "--field" && harness.interface.focus_owner() == ui::FocusOwner::text_field, "clicking Library search retains text focus after pointer release");
        harness.text("  FALL  FREE  ");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.search") == "  FALL  FREE  ", "typing updates the query that filters the Library");
        RIGIDBODIES_EXPECT(harness.document().element_visible(control("library.cards.card", "free_fall")), "words match in either order, ignoring case and surrounding spaces");
        RIGIDBODIES_EXPECT(!harness.document().element_visible(control("library.cards.card", "revolute_drive")), "experiments that lack the search words are removed");
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("library.results.count")).has_value(), "search reports the number of matching experiments");
        RIGIDBODIES_EXPECT(harness.click_element(control("library.top.clear_filters")), "search can be cleared without selecting and deleting the text");
        harness.frames_for(2);

        std::vector<std::string> collections;
        const auto catalogue = harness.session.build_model().catalogue;
        for (const auto& experiment : catalogue)
            if (!experiment.collection.empty() && std::find(collections.begin(), collections.end(), experiment.collection) == collections.end())
                collections.push_back(experiment.collection);
        const auto field = control("library.top.collection") + "--field";
        RIGIDBODIES_EXPECT(harness.click_element(field), "Collection opens an explicit list");
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::transient,
            "clicking Collection opens its native options; focus: " + harness.document().focused_element());
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().focused_element() == field, "the dropdown keeps keyboard focus on its select instead of a decorative child");
        for (std::size_t index = 0; index < collections.size(); ++index)
            harness.key(ui::UiKey::arrow_down);
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.collection") == collections.back(),
            "choosing the final collection applies that category directly; expected: " + collections.back() + "; received: " + std::string(harness.interface.view_state().value("library.collection")) + "; focus: " + harness.document().focused_element() + "; field: " + harness.document().element_value(field).value_or("missing"));
        RIGIDBODIES_EXPECT(harness.click_element(control("library.top.sort") + "--field"), "Sort opens an explicit list");
        harness.frames_for(2);
        harness.key(ui::UiKey::arrow_down);
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.sort") == "suggested", "Suggested order can be selected directly");

        for (const auto& experiment : catalogue)
            harness.interface.view_state().mark_visited(experiment.id);
        RIGIDBODIES_EXPECT(harness.click_element(control("library.top.visited")), "Unopened only can be enabled");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().document_text().find("No experiments match these filters") != std::string::npos, "an empty filter result explains the cause without blaming an empty search");
        RIGIDBODIES_EXPECT(harness.click_element(control("library.top.clear_filters")), "Clear filters is reachable even with no results");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.collection") == "all" && harness.interface.view_state().value("library.visited") == "all" && harness.interface.view_state().value("library.search").empty(), "the action resets every filtering constraint");
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.sort") == "suggested", "clearing filters preserves the chosen sort order");
        RIGIDBODIES_EXPECT(harness.document().element_visible(control("library.cards.card", "free_fall")), "experiments reappear after the filters clear");
    }

    RIGIDBODIES_TEST("typing in Command search filters real results and Enter opens the matching setting")
    {
        LearnerInterface harness;
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("search.field.query") + "--field"), "Command search has an editable query");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("search.field.query") + "--field" && harness.interface.focus_owner() == ui::FocusOwner::text_field, "clicking Command search retains text focus after pointer release");
        harness.text("Gravity strength");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("search.query") == "Gravity strength", "typed text reaches the query used by command matching");
        RIGIDBODIES_EXPECT(harness.document().document_text().find("Gravity strength") != std::string::npos && harness.document().document_text().find("Suggestions") == std::string::npos, "the query replaces suggestions with matching results");
        harness.key(ui::UiKey::enter);
        harness.frames_for(3);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("command_search"), "Enter activates the first matching result");
        RIGIDBODIES_EXPECT(harness.document().element_visible(control("world.gravity.strength") + "--field"), "the requested World setting is revealed");
    }

    RIGIDBODIES_TEST("Library prerequisite links reveal their target through active filters")
    {
        LearnerInterface harness;
        const auto catalogue = harness.session.build_model().catalogue;
        const auto source = std::find_if(catalogue.begin(), catalogue.end(), [](const auto& experiment)
            {
                return !experiment.builds_on.empty();
            });
        RIGIDBODIES_EXPECT(source != catalogue.end(), "the catalogue contains a prerequisite link");
        const auto prerequisite = source->builds_on.front();
        harness.interface.view_state().set_value("library.search", source->title);
        harness.interface.view_state().set_value("library.collection", source->collection);
        harness.interface.view_state().set_value("library.visited", "unvisited");
        harness.interface.view_state().set_value("library.selected", source->id);
        harness.interface.view_state().mark_visited(prerequisite);
        harness.interface.request(ui::ViewRequest::open_library);
        harness.frames_for(2);
        const auto link = control("library.cards.card", source->id) + "--action_1";
        RIGIDBODIES_EXPECT(harness.click_element(link), "the selected card offers its prerequisite");
        harness.frames_for(3);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("library.selected") == prerequisite, "the link selects the prerequisite experiment");
        RIGIDBODIES_EXPECT(harness.document().element_visible(control("library.cards.card", prerequisite)), "the prerequisite is visible even when the previous search and Unopened filter excluded it");
    }

    RIGIDBODIES_TEST("empty notification space does not cover controls in short windows")
    {
        for (const auto& [size, scale] : { std::pair<render::ViewportSize, float> { { 560, 320 }, 1.0f }, { { 1120, 640 }, 2.0f } })
        {
            LearnerInterface harness({}, size, scale);
            ui::UiCommand all_layers;
            all_layers.kind = ui::UiCommandKind::set_layer_mask;
            all_layers.id = "all";
            harness.apply_command(all_layers);
            harness.interface.request(ui::ViewRequest::toggle_show);
            harness.frames_for(2);
            const auto popover = harness.visible_bounds("region-popovers");
            const auto toast_space = harness.document().element_bounds("toast-stack");
            RIGIDBODIES_EXPECT(popover && toast_space, "the menu and notification placement are available");
            std::size_t checked = 0;
            for (const auto& [id, point] : harness.visible_controls())
                if (popover->contains(point) && toast_space->contains(point))
                {
                    ++checked;
                    const auto hit = harness.document().element_at(point);
                    RIGIDBODIES_EXPECT(hit == id || hit.rfind(id + "--", 0) == 0, "empty notification space leaves " + id + " clickable instead of hitting " + hit + at(size, scale));
                }
            RIGIDBODIES_EXPECT(checked > 0, "the regression checks menu controls beneath the empty notification area" + at(size, scale));
            RIGIDBODIES_EXPECT(harness.visible_bounds("popover-shadow").has_value(), "the open menu displays its decorative shadow");
            const ui::Vec2 shadow_edge { popover->minimum.x + 10.0 * scale, popover->minimum.y - 2.0 * scale };
            RIGIDBODIES_EXPECT(harness.document().element_at(shadow_edge) != "popover-shadow", "the menu shadow does not take pointer input");
            RIGIDBODIES_EXPECT(harness.click_element(control("show.presets.preset") + "--option_recommended"), "the recommended preset can be pressed");
            harness.frames_for(2);
            const auto model = harness.session.build_model();
            RIGIDBODIES_EXPECT(model.layers.bits() == model.recommended_layers.bits(), "the press restores the recommended layers" + at(size, scale));

            harness.interface.request(ui::ViewRequest::close_top_surface);
            harness.session.notify(ui::Severity::info, "A notification still has working controls", "toast-hit-test");
            harness.frames_for(2);
            const auto notifications = harness.session.build_model().notifications;
            RIGIDBODIES_EXPECT(!notifications.empty(), "a real notification is displayed");
            const auto serial = notifications.back().serial;
            RIGIDBODIES_EXPECT(harness.click_element("toast-" + std::to_string(serial) + "--dismiss"), "the notification's dismiss button can be pressed");
            harness.frames_for(2);
            RIGIDBODIES_EXPECT(!harness.visible_bounds("toast-" + std::to_string(serial)), "the real notification consumes its dismiss press");
        }
    }

    RIGIDBODIES_TEST("cached large scenes retain floating menu and notification input coverage")
    {
        LearnerInterface harness;
        for (int index = 0; index < 30; ++index)
        {
            physics::BodyDefinition body;
            body.position_m = { 100.0 + index, 100.0 };
            harness.session.world().create_body(body);
        }
        harness.session.notify(ui::Severity::info, "A notification over the stage", "coverage-test");
        harness.interface.request(ui::ViewRequest::open_main_menu);
        harness.frames_for(2);
        const auto menu = harness.visible_bounds("region-popovers");
        const auto notifications = harness.session.build_model().notifications;
        RIGIDBODIES_EXPECT(menu && !notifications.empty(), "both floating surfaces are displayed");
        const auto toast = harness.visible_bounds("toast-" + std::to_string(notifications.back().serial));
        RIGIDBODIES_EXPECT(toast.has_value(), "the notification has a visible card");
        const auto toast_space = harness.document().element_bounds("toast-stack");
        RIGIDBODIES_EXPECT(toast_space.has_value(), "the notification placement area is available");
        const ui::Vec2 below_toast { toast_space->minimum.x + 10.0, toast_space->maximum.y - 10.0 };
        RIGIDBODIES_EXPECT(!toast->contains(below_toast), "the probe lies in the unused notification area");
        for (int frame = 0; frame < 3; ++frame)
        {
            harness.frame();
            RIGIDBODIES_EXPECT(harness.document().covers(centre(*menu)).value_or(false), "a cached frame still routes menu presses to the interface");
            RIGIDBODIES_EXPECT(harness.document().covers(centre(*toast)).value_or(false), "a cached frame still routes notification presses to the interface");
            RIGIDBODIES_EXPECT(!harness.document().covers(below_toast).value_or(true), "unused space below a notification keeps receiving stage input");
        }
    }

    RIGIDBODIES_TEST("Save as and quick Save preserve the selected title content and mode")
    {
        LearnerInterface harness;
        harness.key(ui::UiKey::s, { false, true, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("save.title") + "--field"), "Save opens an editable title");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == control("save.title") + "--field" && harness.interface.focus_owner() == ui::FocusOwner::text_field, "clicking the title retains text focus after pointer release");
        harness.key(ui::UiKey::a, { false, true, false });
        harness.text("My saved experiment");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("save.mode.current")), "the current moment can be selected");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("save.include_guide")), "the guide can be excluded");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("save.apply")), "the configured setup can be saved");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.saved_setup.empty() && harness.session.current_setup_file().current_moment,
            "the save keeps the chosen current-state mode");
        RIGIDBODIES_EXPECT(harness.session.current_setup_file().title == "My saved experiment",
            "the initial Save uses the edited title, received: " + harness.session.current_setup_file().title);
        RIGIDBODIES_EXPECT(!harness.session.current_setup_file().include_guide,
            "the initial Save uses the unchecked guide option");
        const auto body = harness.session.world().body_ids().back();
        const auto moved = harness.session.world().find_body(body)->position_m() + math::Vec2 { 1, 1 };
        harness.session.world().find_body(body)->set_position(moved);
        harness.key(ui::UiKey::s, { false, true, false });
        harness.frames_for(2);
        physics::content::Json saved;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(harness.saved_setup, saved, error), error);
        RIGIDBODIES_EXPECT(saved.at("metadata").at("title").as_string() == "My saved experiment" && !saved.find("guide"),
            "Ctrl+S retains the entered title and excluded guide");
        app::SimulationSession reopened;
        RIGIDBODIES_EXPECT(reopened.open_arrangement(harness.saved_setup, error), error);
        RIGIDBODIES_EXPECT(reopened.world().find_body(reopened.world().body_ids().back())->position_m() == moved,
            "Ctrl+S updates the saved current moment rather than reverting to the starting pose");
        RIGIDBODIES_EXPECT(!harness.interface.view_state().setup_files().empty() &&
                harness.interface.view_state().setup_files().front().title == "My saved experiment",
            "the recent setup keeps its custom name");
        harness.key(ui::UiKey::s, { true, true, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("save.title") == "My saved experiment (copy)" &&
                harness.interface.view_state().value("save.mode") == "current" &&
                harness.interface.view_state().value("save.include_guide") == "false",
            "Save as starts a copy with the current setup's existing choices");
    }

    RIGIDBODIES_TEST("Escape reverts a Preferences field before closing its sheet")
    {
        LearnerInterface harness;
        harness.interface.reveal("prefs.appearance.text_size");
        harness.frames_for(2);
        const auto row = control("prefs.appearance.text_size");
        const auto field = row + "--field";
        RIGIDBODIES_EXPECT(harness.click_element(field), "Text size can be edited inside Preferences");
        const auto original_scale = harness.session.build_model().ui_scale;
        harness.key(ui::UiKey::a, { false, true, false });
        harness.text("999");
        harness.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(harness.document().element_has_class(field, "is-invalid"), "an out-of-range size is visibly rejected");
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().element_visible("panel-preferences"), "the first Escape only ends the field edit");
        RIGIDBODIES_EXPECT(harness.document().element_text(row + "--field-error").empty(), "cancellation removes the obsolete validation message");
        RIGIDBODIES_EXPECT_NEAR(harness.session.build_model().ui_scale, original_scale, 1.0e-12, "cancel never applies the invalid value");
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.document().element_visible("panel-preferences"), "the next Escape closes Preferences");
    }

    RIGIDBODIES_TEST("keyboard dropdowns open and dismiss without changing or closing their sheet")
    {
        LearnerInterface harness;
        harness.interface.reveal("prefs.experiments.on_start");
        harness.frames_for(2);
        const auto field = control("prefs.experiments.on_start") + "--field";
        const auto original = harness.session.build_model().experiments_on_start;
        RIGIDBODIES_EXPECT(harness.document().focused_element() == field && harness.interface.focus_owner() == ui::FocusOwner::keyboard_control, "a closed select is a keyboard control rather than a text field");
        RIGIDBODIES_EXPECT(!harness.interface.wants_text_input(), "selects never request text entry");
        harness.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() == ui::FocusOwner::transient, "Enter opens the focused dropdown");
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.focus_owner() != ui::FocusOwner::transient && harness.document().element_visible("panel-preferences"), "Escape closes only the dropdown");
        RIGIDBODIES_EXPECT(harness.session.build_model().experiments_on_start == original && harness.document().element_value(field) == original, "dismissal preserves the selected option");
    }

    RIGIDBODIES_TEST("property drags cancel completely on Escape or focus loss")
    {
        for (const auto suffix : { "--label", "--slider" })
            for (const auto lose_focus : { false, true })
            {
                LearnerInterface harness;
                const auto target = first_free_object(harness);
                ui::UiCommand select { ui::UiCommandKind::select_body };
                select.body = target;
                harness.apply_command(select);
                harness.frames_for(2);
                const auto original_mass = harness.session.world().find_body(target)->mass_properties().mass_kg;
                const auto box = harness.visible_bounds(control("object.properties.mass") + suffix);
                RIGIDBODIES_EXPECT(box.has_value(), "the numeric drag target is visible");
                auto point = centre(*box);
                harness.pointer(ui::UiEventKind::pointer_down, point);
                point.x += std::string_view(suffix) == "--label" ? 24.0 : box->width() * 0.2;
                harness.pointer(ui::UiEventKind::pointer_move, point);
                harness.frame(0.0);
                RIGIDBODIES_EXPECT(harness.interface.pointer_captured() && !harness.session.build_model().held_reason.empty(), "a real property preview is in progress");
                if (lose_focus)
                {
                    ui::UiEvent lost;
                    lost.kind = ui::UiEventKind::focus_lost;
                    harness.dispatch(lost);
                }
                else
                    harness.key(ui::UiKey::escape);
                RIGIDBODIES_EXPECT(!harness.interface.pointer_captured() && harness.session.build_model().held_reason.empty(), "cancellation releases capture and the simulation's edit hold");
                RIGIDBODIES_EXPECT(harness.session.selection() == target, "cancelling a property drag preserves the selection");
                RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, original_mass, 1.0e-12, "cancellation restores the original property");
                point.x += 12.0;
                harness.pointer(ui::UiEventKind::pointer_move, point);
                harness.pointer(ui::UiEventKind::pointer_up, point);
                harness.frame(0.0);
                RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, original_mass, 1.0e-12, "later movement and physical release cannot restart or commit the cancelled drag");
            }
    }

    RIGIDBODIES_TEST("scrubbing back to the starting point restores the original property")
    {
        LearnerInterface harness;
        const auto target = first_free_object(harness);
        ui::UiCommand select { ui::UiCommandKind::select_body };
        select.body = target;
        harness.apply_command(select);
        harness.frames_for(2);
        const auto original_mass = harness.session.world().find_body(target)->mass_properties().mass_kg;
        const auto box = harness.visible_bounds(control("object.properties.mass") + "--label");
        RIGIDBODIES_EXPECT(box.has_value(), "the mass scrub label is visible");
        const auto origin = centre(*box);
        const auto moved = origin + ui::Vec2 { 24.0, 0.0 };
        harness.pointer(ui::UiEventKind::pointer_down, origin);
        harness.pointer(ui::UiEventKind::pointer_move, moved);
        harness.frame(0.0);
        RIGIDBODIES_EXPECT(harness.session.world().find_body(target)->mass_properties().mass_kg != original_mass, "scrubbing away previews a different mass");
        harness.pointer(ui::UiEventKind::pointer_up, moved, ui::PointerButton::secondary);
        RIGIDBODIES_EXPECT(harness.interface.pointer_captured(), "releasing another button does not finish the primary-button scrub");
        harness.pointer(ui::UiEventKind::pointer_move, origin);
        harness.pointer(ui::UiEventKind::pointer_up, origin);
        RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, original_mass, 1.0e-12, "returning to the origin and releasing keeps the initial mass");
        RIGIDBODIES_EXPECT(harness.session.build_model().held_reason.empty() && !harness.interface.pointer_captured(), "release ends the adjustment cleanly");
    }

    RIGIDBODIES_TEST("commands found by keyboard shortcut remain executable from Recent")
    {
        LearnerInterface harness;
        const auto target = first_free_object(harness);
        const auto original_mass = harness.session.world().find_body(target)->mass_properties().mass_kg;
        ui::UiCommand mass { ui::UiCommandKind::set_selected_mass };
        mass.body = target;
        mass.value = original_mass + 1.0;
        harness.apply_command(mass);
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        harness.text("Ctrl+Z");
        harness.frames_for(2);
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, original_mass, 1.0e-12, "Enter executes the Undo found by typing");
        RIGIDBODIES_EXPECT(!harness.interface.view_state().sheet_open("command_search"), "executing the command closes the palette");
        const auto& history = harness.interface.view_state().search_recent();
        RIGIDBODIES_EXPECT(!history.empty() && history.front() == "key:Undo", "a shortcut search remembers the keyboard identity");

        mass.value = original_mass + 2.0;
        harness.apply_command(mass);
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        harness.key(ui::UiKey::a, { false, true, false });
        harness.key(ui::UiKey::backspace);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("search.query").empty(), "an empty query shows Recent");
        const auto recent = harness.document().commands_for_key("search.result");
        RIGIDBODIES_EXPECT(std::any_of(recent.begin(), recent.end(), [](const auto& command)
                               {
                                   return command.kind == ui::UiCommandKind::undo;
                               }),
            "the recent keyboard identity resolves to its executable command");
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT_NEAR(harness.session.world().find_body(target)->mass_properties().mass_kg, original_mass, 1.0e-12, "Enter executes the same Undo from Recent");
    }

    RIGIDBODIES_TEST("revealing a draft point coordinate supports precise units cancellation and undo")
    {
        LearnerInterface harness({}, { 1600, 1000 });
        ui::UiCommand units { ui::UiCommandKind::set_display_units };
        units.id = "centimetre_gram";
        harness.apply_command(units);
        harness.apply_command({ ui::UiCommandKind::start_new_shape });
        for (const auto kind : { ui::UiCommandKind::set_shape_snap_grid, ui::UiCommandKind::set_shape_snap_vertices, ui::UiCommandKind::set_shape_snap_angles })
            harness.apply_command({ kind });
        harness.frames_for(2);
        for (const auto point : { harness.stage_point(0.42, 0.4), harness.stage_point(0.54, 0.4), harness.stage_point(0.5, 0.6) })
        {
            harness.click(point);
            harness.frames_for(2);
        }
        const auto before = harness.session.shape_editor().outline();
        const auto selected = harness.session.shape_editor().selected_node();
        RIGIDBODIES_EXPECT(before.nodes.size() == 3 && selected.has_value(), "three stage clicks create a selected draft point");
        harness.interface.close_menus();
        harness.interface.reveal("draw.node.position_x");
        harness.frames_for(3);
        const auto field = control("draw.node.position_x") + "--field";
        RIGIDBODIES_EXPECT(harness.document().element_visible(field) && harness.document().focused_element() == field, "revealing the coordinate opens Drawing options and focuses its exact field");
        const auto coordinate = harness.session.build_model().shape_node_world_m.x;
        RIGIDBODIES_EXPECT(harness.session.build_model().display_units == core::DisplayUnits::centimetre_gram, "the draft uses the chosen CGS units");
        RIGIDBODIES_EXPECT_NEAR(std::stod(harness.document().element_value(field).value_or("nan")), coordinate * 100.0, 0.01, "the field shows centimetres while the unit suffix is outside the editable text");
        harness.key(ui::UiKey::a, { false, true, false });
        harness.text("125 cm");
        harness.key(ui::UiKey::enter);
        harness.frames_for(2);
        const auto& editor = harness.session.shape_editor();
        const auto world = math::transform_point(editor.placement(), editor.outline().nodes[*selected].position_m);
        RIGIDBODIES_EXPECT_NEAR(world.x, 1.25, 1.0e-10, "the numeric edit moves the selected point to exactly 1.25 metres");
        for (std::size_t index = 0; index < before.nodes.size(); ++index)
            if (index != *selected)
                RIGIDBODIES_EXPECT(editor.outline().nodes[index].position_m == before.nodes[index].position_m, "editing a point never translates the other draft nodes");
        harness.key(ui::UiKey::a, { false, true, false });
        harness.text("999 m");
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT_NEAR(math::transform_point(editor.placement(), editor.outline().nodes[*selected].position_m).x, 1.25, 1.0e-10, "Escape discards unfinished coordinate text without altering the committed draft");
        harness.key(ui::UiKey::escape);
        harness.frames_for(2);
        harness.key(ui::UiKey::z, { false, true, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.session.shape_editor().active() && harness.session.shape_editor().outline().nodes.size() == before.nodes.size(), "Undo keeps the draft open and its nodes intact");
        RIGIDBODIES_EXPECT(harness.session.shape_editor().outline().nodes[*selected].position_m == before.nodes[*selected].position_m, "Undo restores the precise point edit as one draft operation");
    }

    RIGIDBODIES_TEST("a single saved run opens its own graph while a new run keeps recording")
    {
        LearnerInterface harness({}, { 1600, 1200 });
        harness.session.stepper().set_paused(false);
        harness.frames_for(45, 0.025);
        harness.session.reset_scenario();
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.session.build_model().runs.size() == 1, "Back to start keeps one recording");
        const auto saved_number = harness.session.build_model().runs.front().number;
        harness.session.stepper().set_paused(false);
        harness.frames_for(12, 0.025);
        harness.interface.view_state().set_surface_open("measure.open", true);
        harness.interface.view_state().set_active_tab("measure.header.tabs", "runs");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("measure.runs.inspect_graph")), "one saved run already offers Show this run in graph");
        harness.frames_for(2);
        constexpr std::string_view tabs[] { "energy", "graph", "collisions", "runs", "theory", "relativity" };
        RIGIDBODIES_EXPECT(harness.interface.view_state().active_tab("measure.header.tabs", tabs, "energy") == "graph" &&
                harness.interface.view_state().value("measure.graph.review_run") == std::to_string(saved_number),
            "the real action opens the selected saved run rather than merely overlaying it on the live window");
        RIGIDBODIES_EXPECT(!harness.session.stepper().is_paused() && harness.session.build_model().current_run && harness.session.build_model().current_run->number != saved_number,
            "reviewing saved measurements leaves the new recording running");
        RIGIDBODIES_EXPECT(harness.click_element(control("measure.graph.live")), "Back to live graph is directly reachable");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("measure.graph.review_run").empty() && !harness.session.stepper().is_paused(), "returning to live clears review without changing playback");
    }

    RIGIDBODIES_TEST("a pointer leaving the interface for the title bar leaves nothing hovered")
    {
        LearnerInterface harness({}, { 1600, 900 }, 1.0f);
        harness.window_frame.controls = true;
        harness.frames_for(2);
        const auto play = harness.visible_bounds(control("transport.play"));
        RIGIDBODIES_EXPECT(play.has_value(), "Play is shown");
        if (!play)
            return;
        harness.pointer(ui::UiEventKind::pointer_move, centre(*play));
        harness.frames_for(45);
        RIGIDBODIES_EXPECT(harness.visible_bounds("tooltip").has_value(), "resting on Play shows its tooltip");
        ui::UiEvent leave;
        leave.kind = ui::UiEventKind::pointer_leave;
        leave.pointer_px = centre(*play);
        harness.dispatch(leave);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.visible_bounds("tooltip").has_value(), "the tooltip goes when the pointer leaves for the title bar");
    }

    double probe_speed(const LearnerInterface& harness)
    {
        const auto model = harness.session.build_model();
        return model.relativity ? model.relativity->speed_fraction : -1.0;
    }

    void open_chasing_light(LearnerInterface& harness)
    {
        RIGIDBODIES_EXPECT(harness.session.load_scenario("chasing_light"), "Chasing light loads");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.session.relativity_active(), "the relativity experiment is open");
    }

    RIGIDBODIES_TEST("Shift+Up raises the probe speed in Present under the lock")
    {
        LearnerInterface harness;
        open_chasing_light(harness);
        harness.key(ui::UiKey::f5);
        harness.frame();
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode && harness.interface.view_state().present().lock, "Present opens locked");
        const auto& spec = ui::find_control_spec("world.relativity.speed")->number;
        harness.key(ui::UiKey::arrow_up, { true, false, false });
        harness.frame();
        RIGIDBODIES_EXPECT(probe_speed(harness) == 0.5, "Shift+Up goes from rest to the next preset, 0.5 c, under the lock");
        harness.key(ui::UiKey::arrow_up, { true, false, false });
        harness.frame();
        RIGIDBODIES_EXPECT(probe_speed(harness) == 0.9, "Shift+Up again goes to 0.9 c");
        harness.key(ui::UiKey::arrow_up);
        harness.frame();
        RIGIDBODIES_EXPECT(probe_speed(harness) == ui::keyboard_step(spec, 0.9, 1, ui::StepSize::normal) && probe_speed(harness) > 0.9 && probe_speed(harness) < 0.99, "Up raises the speed a little");
        harness.key(ui::UiKey::arrow_down, { true, false, false });
        harness.frame();
        RIGIDBODIES_EXPECT(probe_speed(harness) == 0.9, "Shift+Down returns to the previous preset");
        RIGIDBODIES_EXPECT_NEAR(harness.interface.view_state().number("present.guide_step", 0.0), 0.0, 0.0, "the speed keys leave the lesson step where it was");
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode && harness.interface.view_state().present().lock, "the presenter stays in locked Present");
        const auto strip_time = harness.document().element_text(control("present.time") + "--value");
        RIGIDBODIES_EXPECT(strip_time.find("Lab time") == 0 && strip_time.find("ns") != std::string::npos, "the strip's clock reads lab time in nanoseconds: " + strip_time);
    }

    RIGIDBODIES_TEST("Present on a 1600 x 900 window at 150 % settles with both clocks and the race on the stage")
    {
        // The client area of that window. Present first frames the stage under the caption's whole
        // box and measures the caption a frame later; the stage must then show the largest tier
        // that fits, not the one the first frame forced on it.
        LearnerInterface harness({}, { 1578, 889 }, 1.5f);
        open_chasing_light(harness);
        harness.key(ui::UiKey::f5);
        harness.frames_for(6);
        RIGIDBODIES_EXPECT(harness.interface.view_state().present().mode, "Present is on");
        const auto& stage = harness.session.scene_renderer().relativity_stage_layout();
        RIGIDBODIES_EXPECT(stage.has_value(), "the stage has been drawn");
        if (!stage)
            return;
        const auto focus = harness.interface.layout().focus;
        RIGIDBODIES_EXPECT(std::abs(stage->text_pixel_scale - 1.5 * 1.4) <= 1.0e-6, "Present draws stage text 1.4 times larger");
        RIGIDBODIES_EXPECT(stage->tier == render::fitting_relativity_tier(focus.width(), focus.height(), stage->text_pixel_scale, harness.session.scene_settings().display_units), "the stage settles on the largest tier that fits");
        RIGIDBODIES_EXPECT(stage->tier == render::RelativityStageTier::compact && stage->clock_radius >= 16.0 * stage->text_pixel_scale, "that is the compact tier, with both clock faces");
        bool race = false;
        for (const auto& command : harness.scene_list.commands())
            race = race || (command.kind == render::DrawCommandKind::text && std::string_view { harness.scene_list.text_buffer() }.substr(command.text_offset, command.text_length).rfind("Light ", 0) == 0);
        RIGIDBODIES_EXPECT(race, "and the race plate");
        harness.key(ui::UiKey::f5);
        harness.frames_for(3);
        RIGIDBODIES_EXPECT(!harness.interface.view_state().present().mode && stage && stage->tier == render::RelativityStageTier::full, "leaving Present returns to the full tier");
    }

    RIGIDBODIES_TEST("the Present caption carries the speed presets")
    {
        LearnerInterface harness;
        open_chasing_light(harness);
        harness.key(ui::UiKey::f5);
        harness.frame();
        harness.key(ui::UiKey::arrow_right);
        harness.frames_for(2);
        const auto chips = control("world.relativity.preset", "present");
        RIGIDBODIES_EXPECT(harness.visible_bounds(chips).has_value(), "step 2 offers the speed presets on the lesson card");
        RIGIDBODIES_EXPECT(harness.click_element(chips + "--option_row_0_9"), "the 0.9 c chip can be pressed");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(probe_speed(harness) == 0.9 && harness.interface.view_state().present().mode, "the chip sets 0.9 c without leaving Present");
        harness.key(ui::UiKey::arrow_right);
        harness.key(ui::UiKey::arrow_right);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("world.relativity.speed", "present") + "--field").has_value(), "step 4 offers the speed field");
        harness.key(ui::UiKey::arrow_right);
        harness.key(ui::UiKey::arrow_right);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("present.strip.show_me")), "the last step offers to show the comparison");
        harness.frames_for(2);
        constexpr std::string_view tabs[] { "relativity", "graph", "runs" };
        RIGIDBODIES_EXPECT(!harness.interface.view_state().present().mode && harness.interface.view_state().surface_open("measure.open", false) && harness.interface.view_state().active_tab("measure.header.tabs", tabs, "graph") == "relativity", "Exit and show me leaves Present on the Relativity tab");
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("measure.relativity.curve")).has_value(), "the curve chooser is shown");
    }

    RIGIDBODIES_TEST("Show me for the curve opens the Relativity tab")
    {
        LearnerInterface harness({}, { 1600, 1200 });
        open_chasing_light(harness);
        // Fold the Guide's earlier sections so its steps are in view.
        for (const auto* section : { "guide.about", "guide.watch", "guide.predict", "guide.change" })
            harness.interface.view_state().set_section_open(section, false);
        harness.interface.view_state().set_surface_open("measure.open", false);
        harness.interface.view_state().set_active_tab("measure.header.tabs", "graph");
        harness.frames_for(2);
        // Steps 2 to 6 each have Show me; the last one opens the comparison of the curves.
        RIGIDBODIES_EXPECT(harness.click_element(control("guide.steps.show_me", "5")), "the last step's Show me is displayed");
        harness.frames_for(2);
        constexpr std::string_view tabs[] { "relativity", "graph", "runs" };
        RIGIDBODIES_EXPECT(harness.interface.view_state().surface_open("measure.open", false) && harness.interface.view_state().active_tab("measure.header.tabs", tabs, "graph") == "relativity", "Show me opens Measure on the Relativity tab");
        RIGIDBODIES_EXPECT(harness.document().focused_element().rfind(control("measure.relativity.curve"), 0) == 0, "the curve chooser takes the focus");
    }

    RIGIDBODIES_TEST("W opens Probe speed and a probe click does not open Measure")
    {
        LearnerInterface harness;
        open_chasing_light(harness);
        harness.interface.view_state().set_surface_open("measure.open", true);
        harness.interface.view_state().set_active_tab("measure.header.tabs", "relativity");
        harness.frames_for(2);
        harness.key(ui::UiKey::w);
        harness.frames_for(2);
        const auto speed = control("world.relativity.speed");
        RIGIDBODIES_EXPECT(harness.document().focused_element().rfind(speed + "-", 0) == 0, "W focuses the World page's Probe speed, not the Relativity tab's copy: " + harness.document().focused_element());

        harness.interface.view_state().set_surface_open("measure.open", false);
        harness.interface.view_state().set_surface_open("inspector.open", false);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(!harness.interface.layout().find(ui::RegionId::inspector), "the Inspector is closed before the click");
        const auto& stage = harness.session.scene_renderer().relativity_stage_layout();
        RIGIDBODIES_EXPECT(stage.has_value(), "the stage has been drawn");
        if (!stage)
            return;
        harness.click(stage->probe_center);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.layout().find(ui::RegionId::inspector) && harness.document().focused_element().rfind(speed + "-", 0) == 0, "a click on the probe opens Probe speed");
        RIGIDBODIES_EXPECT(!harness.interface.view_state().surface_open("measure.open", false), "the probe click leaves Measure closed");
    }

    RIGIDBODIES_TEST("dragging the speed slider keeps the clocks running")
    {
        LearnerInterface harness;
        open_chasing_light(harness);
        harness.session.stepper().set_paused(false);
        harness.frames_for(10);
        const auto slider = harness.visible_bounds(control("world.relativity.speed") + "--slider");
        RIGIDBODIES_EXPECT(slider.has_value(), "the World page shows the speed slider");
        if (!slider)
            return;
        auto point = ui::Vec2 { slider->minimum.x + slider->width() * 0.1, (slider->minimum.y + slider->maximum.y) * 0.5 };
        harness.pointer(ui::UiEventKind::pointer_down, point);
        point.x = slider->minimum.x + slider->width() * 0.45;
        harness.pointer(ui::UiEventKind::pointer_move, point);
        harness.frame();
        const auto during = harness.session.build_model();
        RIGIDBODIES_EXPECT(harness.interface.pointer_captured() && during.held_reason.empty() && during.run_state == ui::RunState::running, "a speed drag holds nothing: the run goes on under the hand");
        RIGIDBODIES_EXPECT(during.relativity && during.relativity->speed_fraction > 0.0, "the drag previews a speed");
        harness.frames_for(20);
        const auto later = harness.session.build_model();
        RIGIDBODIES_EXPECT(later.relativity && later.relativity->lab_time_s > during.relativity->lab_time_s && later.relativity->proper_time_s > during.relativity->proper_time_s, "both clocks keep running while the slider is held");
        harness.pointer(ui::UiEventKind::pointer_up, point);
        harness.frame();
        const auto after = harness.session.build_model();
        RIGIDBODIES_EXPECT(!harness.interface.pointer_captured() && after.held_reason.empty() && after.run_state == ui::RunState::running, "release commits and the run goes on");
        RIGIDBODIES_EXPECT(after.can_undo && after.undo_label == "Change probe speed" && after.relativity && after.relativity->speed_fraction == during.relativity->speed_fraction, "the drag is one undo entry keeping the previewed speed");
    }

    RIGIDBODIES_TEST("a focused speed field keeps its arrows")
    {
        LearnerInterface harness;
        open_chasing_light(harness);
        const auto field = control("world.relativity.speed") + "--field";
        RIGIDBODIES_EXPECT(harness.click_element(field), "the speed field is displayed");
        harness.frame();
        RIGIDBODIES_EXPECT(harness.document().focused_element() == field, "the press focuses the field");
        const auto& spec = ui::find_control_spec("world.relativity.speed")->number;
        harness.key(ui::UiKey::arrow_up);
        harness.frames_for(2);
        const auto nudged = ui::keyboard_step(spec, 0.0, 1, ui::StepSize::normal);
        RIGIDBODIES_EXPECT(probe_speed(harness) == nudged, "Up in the field takes one rapidity step, once");
        harness.key(ui::UiKey::arrow_up, { true, false, false });
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(probe_speed(harness) == 0.5, "Shift+Up in the field climbs to the next preset, once");
        harness.key(ui::UiKey::arrow_down);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(probe_speed(harness) == ui::keyboard_step(spec, 0.5, -1, ui::StepSize::normal), "Down in the field takes one step down");
        RIGIDBODIES_EXPECT(harness.document().focused_element() == field, "the field keeps the focus");
    }

    RIGIDBODIES_TEST("a relativity run is pinned, kept and reviewed in lab nanoseconds")
    {
        LearnerInterface harness({}, { 1600, 1200 });
        open_chasing_light(harness);
        ui::UiCommand speed;
        speed.kind = ui::UiCommandKind::set_relativity_speed;
        speed.value = 0.6;
        harness.apply_command(speed);
        harness.interface.view_state().set_surface_open("measure.open", true);
        harness.interface.view_state().set_active_tab("measure.header.tabs", "runs");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.click_element(control("measure.runs.add_value")), "the Runs tab offers Add value");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("measure.runs.add_quantity") + "--field").has_value() && !harness.visible_bounds(control("measure.runs.add_object") + "--field").has_value(), "a clock is chosen without an object");
        RIGIDBODIES_EXPECT(harness.click_element(control("measure.runs.add", "probe_clock")), "Add is pressed with the default, the probe clock at the end");
        harness.frames_for(2);
        const auto pinned = harness.session.build_model().pinned_values;
        RIGIDBODIES_EXPECT(pinned.size() == 1 && pinned.front().quantity == "probe_clock" && pinned.front().aggregator == ui::RunAggregator::at_end, "the probe clock is pinned for every run");

        harness.session.stepper().set_paused(false);
        harness.frames_for(200, 0.025);
        harness.session.reset_scenario();
        harness.frames_for(2);
        const auto runs = harness.session.build_model().runs;
        RIGIDBODIES_EXPECT(runs.size() == 1 && runs.front().pinned_results.size() == 1 && runs.front().pinned_results.front(), "Back to start keeps the run with its probe clock");
        if (runs.size() != 1 || runs.front().pinned_results.empty() || !runs.front().pinned_results.front())
            return;
        const auto lab_ns = runs.front().duration_s;
        RIGIDBODIES_EXPECT_NEAR(*runs.front().pinned_results.front(), 0.8 * lab_ns * 1.0e-9, 1.0e-15, "the probe clock ran at 0.8 of the lab rate");
        const auto run_row = harness.document().element_text(control("measure.runs.row", "1"));
        RIGIDBODIES_EXPECT(run_row.find(core::format_quantity(lab_ns * 1.0e-9, core::DisplayQuantity::fine_time, core::DisplayUnits::si)) != std::string::npos, "the kept run's length reads in lab nanoseconds: " + run_row);
        const auto value = core::format_quantity(*runs.front().pinned_results.front(), core::DisplayQuantity::fine_time, core::DisplayUnits::si);
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("measure.runs.remove_value." + pinned.front().key, pinned.front().key)).has_value(), "the pinned value is listed with its Remove action");
        const auto text = harness.document().document_text();
        RIGIDBODIES_EXPECT(text.find("At end") != std::string::npos && text.find(value) != std::string::npos, "the pinned probe clock reads in nanoseconds: " + value);

        RIGIDBODIES_EXPECT(harness.click_element(control("measure.runs.inspect_graph")), "the kept run offers Show this run in graph");
        harness.frames_for(2);
        constexpr std::string_view tabs[] { "relativity", "graph", "runs" };
        RIGIDBODIES_EXPECT(harness.interface.view_state().active_tab("measure.header.tabs", tabs, "relativity") == "graph" && harness.interface.view_state().value("measure.graph.review_run") == "1", "the run opens on the Graph tab for review");
        RIGIDBODIES_EXPECT(harness.visible_bounds(control("measure.graph.clocks") + "--field").has_value() && !harness.visible_bounds(control("measure.graph.scope") + "--field").has_value(), "the review offers the clocks and no object scope");
        const auto summary = harness.document().element_attribute(control("measure.graph.plot") + "--plot", "aria-label").value_or("");
        RIGIDBODIES_EXPECT(summary.find("against Lab time (ns) for Probe and lab clocks") != std::string::npos && summary.find("Probe clock \xCF\x84 (Run 1)") != std::string::npos, "the reviewed graph plots the run's clocks against lab time: " + summary);
        RIGIDBODIES_EXPECT(harness.click_element(control("measure.graph.live")), "Back to live graph is offered");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("measure.graph.review_run").empty(), "the graph returns to the live run");

        // A clock reading's context menu, in a session whose stage was never right-clicked: Add to
        // runs table refuses the probe clock pinned above, and Plot over time lands on the clocks
        // the Graph offers. The readings sit below the speed controls, so the drawer is scrolled
        // to them as a learner would.
        harness.interface.view_state().set_active_tab("measure.header.tabs", "relativity");
        harness.frames_for(2);
        const auto drawer = region_bounds(harness, ui::RegionId::measure_drawer);
        auto reading = harness.visible_bounds(control("measure.relativity.probe_clock"));
        for (int attempt = 0; attempt < 20 && reading && reading->maximum.y > drawer.maximum.y; ++attempt)
        {
            harness.wheel((drawer.minimum + drawer.maximum) * 0.5, -3.0);
            harness.frame();
            reading = harness.visible_bounds(control("measure.relativity.probe_clock"));
        }
        RIGIDBODIES_EXPECT(reading && reading->minimum.y >= drawer.minimum.y && reading->maximum.y <= drawer.maximum.y, "the probe clock is scrolled into view");
        if (!reading)
            return;
        const auto pointer = (reading->minimum + reading->maximum) * 0.5;
        // The drawer's content can move after a menu action, so each right-click finds the reading
        // where it is now.
        const auto open_menu = [&]
        {
            const auto now = harness.visible_bounds(control("measure.relativity.probe_clock"));
            if (!now)
                return false;
            harness.click((now->minimum + now->maximum) * 0.5, ui::PointerButton::secondary);
            harness.frames_for(2);
            return harness.interface.view_state().transient_open("context_menu") && harness.interface.view_state().value("context.kind") == "readout" && harness.interface.view_state().value("context.key") == "measure.relativity.probe_clock";
        };
        RIGIDBODIES_EXPECT(open_menu(), "a right-click on the probe clock opens its value actions");
        const auto pin_row = harness.visible_bounds(control("context.readout.pin", "probe_clock"));
        RIGIDBODIES_EXPECT(pin_row && std::abs(pin_row->minimum.x - pointer.x) < 260.0 && std::abs(pin_row->minimum.y - pointer.y) < 300.0, "the menu opens at the pointer");
        RIGIDBODIES_EXPECT(harness.click_element(control("context.readout.pin", "probe_clock")), "the menu offers Add to runs table");
        harness.frames_for(2);
        const auto repeated = harness.session.build_model();
        RIGIDBODIES_EXPECT(repeated.pinned_values.size() == 1 && std::any_of(repeated.notifications.begin(), repeated.notifications.end(), [](const auto& notification)
                                                                     {
                                                                         return notification.text == "This value is already pinned.";
                                                                     }),
            "the probe clock already pinned through Add value is not pinned again");
        RIGIDBODIES_EXPECT(open_menu(), "the value actions open again");
        RIGIDBODIES_EXPECT(harness.click_element(control("context.readout.plot")), "the menu offers Plot over time");
        harness.frames_for(3);
        RIGIDBODIES_EXPECT(harness.interface.view_state().active_tab("measure.header.tabs", tabs, "relativity") == "graph" && harness.document().focused_element().rfind(control("measure.graph.clocks"), 0) == 0, "Plot over time opens the Graph tab on its clocks: " + harness.document().focused_element());

        // Plot over time on a reading the Graph does not show yet adds it to the clocks it plots.
        harness.interface.view_state().set_active_tab("measure.header.tabs", "relativity");
        harness.frames_for(2);
        auto gamma = harness.visible_bounds(control("measure.relativity.gamma"));
        for (int attempt = 0; attempt < 20 && gamma && (gamma->maximum.y > drawer.maximum.y || gamma->minimum.y < drawer.minimum.y); ++attempt)
        {
            harness.wheel((drawer.minimum + drawer.maximum) * 0.5, gamma->maximum.y > drawer.maximum.y ? -3.0 : 3.0);
            harness.frame();
            gamma = harness.visible_bounds(control("measure.relativity.gamma"));
        }
        RIGIDBODIES_EXPECT(gamma.has_value(), "the Lorentz factor reading is shown");
        if (!gamma)
            return;
        harness.click((gamma->minimum + gamma->maximum) * 0.5, ui::PointerButton::secondary);
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.interface.view_state().value("context.key") == "measure.relativity.gamma" && harness.click_element(control("context.readout.plot")), "the Lorentz factor offers Plot over time");
        harness.frames_for(3);
        const auto plotted = harness.interface.view_state().checklist("measure.graph.clocks", ui::default_graph_clocks());
        RIGIDBODIES_EXPECT(harness.interface.view_state().active_tab("measure.header.tabs", tabs, "relativity") == "graph" && std::find(plotted.begin(), plotted.end(), "lorentz") != plotted.end() && std::find(plotted.begin(), plotted.end(), "probe_clock") != plotted.end(), "the Graph plots the Lorentz factor beside the clocks it already showed");
    }

    RIGIDBODIES_TEST("a reading found by Command search is scrolled into view on its tab")
    {
        LearnerInterface harness({}, { 1280, 800 });
        open_chasing_light(harness);
        harness.interface.view_state().set_surface_open("measure.open", false);
        harness.interface.view_state().set_active_tab("measure.header.tabs", "graph");
        harness.frames_for(2);
        harness.interface.request(ui::ViewRequest::open_command_search);
        harness.frames_for(2);
        harness.text("proper time");
        harness.frames_for(2);
        harness.key(ui::UiKey::enter);
        harness.frames_for(3);
        constexpr std::string_view tabs[] { "relativity", "graph", "runs" };
        RIGIDBODIES_EXPECT(harness.interface.view_state().surface_open("measure.open", false) && harness.interface.view_state().active_tab("measure.header.tabs", tabs, "graph") == "relativity", "the reading's tab opens");
        const auto drawer = region_bounds(harness, ui::RegionId::measure_drawer);
        const auto reading = harness.visible_bounds(control("measure.relativity.probe_clock"));
        RIGIDBODIES_EXPECT(reading && reading->minimum.y >= drawer.minimum.y - 1.0e-6 && reading->maximum.y <= drawer.maximum.y + 1.0e-6, "the probe clock is scrolled into the drawer");
    }

    RIGIDBODIES_TEST("the Tools chips say why they are disabled in a relativity experiment")
    {
        LearnerInterface harness;
        harness.frames_for(2);
        const auto chip = [](std::string_view option)
        {
            return control("tools.mode") + "--option_" + std::string(option);
        };
        RIGIDBODIES_EXPECT(harness.document().element_attribute(chip("throw"), "data-tooltip") == std::optional<std::string> { "Fling objects with the pointer (T)" }, "a Newtonian chip describes its tool");
        open_chasing_light(harness);
        const std::string reason = "Throw and Pull act on Newtonian objects. Change the probe's speed instead.";
        for (const auto* option : { "select", "throw", "pull" })
            RIGIDBODIES_EXPECT(harness.document().element_attribute(chip(option), "data-tooltip") == std::optional<std::string> { reason }, std::string("a disabled chip gives the row's reason, not its tool: ") + option);
        RIGIDBODIES_EXPECT(harness.session.load_scenario("free_fall"), "Free fall loads");
        harness.frames_for(2);
        RIGIDBODIES_EXPECT(harness.document().element_attribute(chip("throw"), "data-tooltip") == std::optional<std::string> { "Fling objects with the pointer (T)" }, "back in a Newtonian experiment the chip describes its tool again");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
