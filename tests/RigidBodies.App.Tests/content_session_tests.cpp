#include <rigidbodies/app/content_files.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/shape_document.hpp>
#include "test_framework.hpp"

#include <chrono>
#include <fstream>

namespace
{
    using namespace rigidbodies;
    using K = ui::UiCommandKind;
    void command(app::SimulationSession& session, K kind)
    {
        ui::UiCommand c;
        c.kind = kind;
        session.apply(c);
    }
    std::string save(const app::SimulationSession& session)
    {
        std::string text, error;
        RIGIDBODIES_EXPECT(session.save_arrangement(text, error), error);
        return text;
    }
    std::string shape_file()
    {
        physics::Outline outline;
        outline.closed = true;
        for (const auto position : { math::Vec2 { 0, 0 }, math::Vec2 { 1, 0 }, math::Vec2 { 0, 1 } })
        {
            physics::OutlineNode node;
            node.position_m = position;
            outline.nodes.push_back(node);
        }
        const auto built = physics::build_authored_shape(outline);
        RIGIDBODIES_EXPECT(built.succeeded(), "fixture builds");
        physics::ShapeDocument document;
        std::string error;
        RIGIDBODIES_EXPECT(physics::capture_shape_document(*built.shape, "Triangle", "Import fixture", document, error), error);
        return physics::write_shape_document(document);
    }
    struct TemporaryDirectory
    {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("rigid-bodies-content-tests-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        TemporaryDirectory()
        {
            std::filesystem::create_directory(path);
        }
        ~TemporaryDirectory()
        {
            std::error_code error;
            std::filesystem::remove_all(path, error);
        }
    };

    RIGIDBODIES_TEST("opening an arrangement restores edited state and establishes its reset baseline")
    {
        app::SimulationSession source, target;
        RIGIDBODIES_EXPECT(source.load_scenario("shape_workshop"), "fixture loads");
        const auto id = source.world().body_ids().back();
        source.world().find_body(id)->set_position({ 9, 8 });
        source.world().find_body(id)->set_linear_velocity({ 1, 2 });
        auto settings = source.world().settings();
        settings.gravity_m_s2 = { 3, -4 };
        source.world().set_settings(settings);
        source.stepper().set_fixed_step(0.005);
        source.stepper().set_substep_count(3);
        source.stepper().set_time_scale(0.5);
        source.camera().set_center({ 2, 3 });
        source.camera().set_view_height(6);
        const auto text = save(source);
        std::string error;
        RIGIDBODIES_EXPECT(target.open_arrangement(text, error), error);
        RIGIDBODIES_EXPECT(target.scenario_id() == "shape_workshop" && target.world().body_ids().size() == source.world().body_ids().size(), "physical arrangement and document identity survive the roundtrip");
        RIGIDBODIES_EXPECT(target.stepper().is_paused(), "opening leaves a reviewable paused arrangement");
        RIGIDBODIES_EXPECT(target.stepper().substep_count() != 3 && target.stepper().time_scale() == 0.5 && target.camera().center_m() == math::Vec2 { 2, 3 }, "only playback speed and presentation view are imported from the file");
        const auto loaded = target.world().body_ids().back();
        target.stepper().set_paused(false);
        target.advance(0.02);
        target.world().find_body(loaded)->set_position({ -2, -3 });
        target.reset_scenario();
        RIGIDBODIES_EXPECT_NEAR(target.world().find_body(loaded)->position_m().x, 9, 1e-12, "reset uses imported positions");
        RIGIDBODIES_EXPECT(target.world().settings().gravity_m_s2 == math::Vec2 { 3, -4 }, "reset uses imported environment");
    }

    RIGIDBODIES_TEST("save details distinguish starting setup from current moment and control teaching content")
    {
        app::SimulationSession source;
        RIGIDBODIES_EXPECT(source.load_scenario("free_fall"), "save-details fixture loads");
        const auto body = source.world().body_ids().back();
        source.set_selection(body);
        ui::UiCommand position;
        position.kind = K::set_selected_position;
        position.body = body;
        position.detail = "x";
        position.value = 3.0;
        position.value_y = source.world().find_body(body)->position_m().y;
        source.apply(position);
        const auto setup_position = source.world().find_body(body)->position_m();
        source.world().find_body(body)->set_linear_velocity({ 2.0, 0.0 });
        source.stepper().set_paused(false);
        source.advance(0.5);
        const auto current_position = source.world().find_body(body)->position_m();
        RIGIDBODIES_EXPECT(!(current_position == setup_position), "fixture has distinct starting and live poses");

        std::string starting_text, current_text, error;
        RIGIDBODIES_EXPECT(source.save_arrangement(starting_text, error, "My Drop", false, false), error);
        RIGIDBODIES_EXPECT(!source.stepper().is_paused(), "saving the starting setup never pauses playback");
        RIGIDBODIES_EXPECT(source.save_arrangement(current_text, error, "My Drop Now", true, true), error);
        RIGIDBODIES_EXPECT(!source.stepper().is_paused(), "saving the current moment never pauses playback");

        physics::content::Json starting_json, current_json;
        RIGIDBODIES_EXPECT(physics::content::parse_json(starting_text, starting_json, error), error);
        RIGIDBODIES_EXPECT(physics::content::parse_json(current_text, current_json, error), error);
        RIGIDBODIES_EXPECT(starting_json.at("metadata").at("title").as_string() == "My Drop" &&
                starting_json.at("metadata").at("id").as_string() == "my_drop_setup" &&
                starting_json.at("metadata").at("based_on").as_string() == "free_fall",
            "a titled setup records its identity and source");
        RIGIDBODIES_EXPECT(!starting_json.find("guide") && current_json.find("guide"), "Include guide controls whether teaching content is saved");

        app::SimulationSession starting, current;
        RIGIDBODIES_EXPECT(starting.open_arrangement(starting_text, error), error);
        RIGIDBODIES_EXPECT(current.open_arrangement(current_text, error), error);
        const auto starting_body = starting.world().body_ids().back();
        const auto current_body = current.world().body_ids().back();
        RIGIDBODIES_EXPECT(starting.world().find_body(starting_body)->position_m() == setup_position, "Starting setup round-trips the Ready baseline");
        RIGIDBODIES_EXPECT(current.world().find_body(current_body)->position_m() == current_position, "Current moment round-trips the live pose");
        RIGIDBODIES_EXPECT(starting.stepper().is_paused() && current.stepper().is_paused(), "opened setup files land Ready");
    }

    RIGIDBODIES_TEST("opening and undoing files restores metadata world and original reset baseline")
    {
        app::SimulationSession a, b;
        a.load_scenario("free_fall");
        b.load_scenario("revolute_drive");
        const auto original_content = a.build_model().scenario_content;
        const auto replacement_content = b.build_model().scenario_content;
        const auto original = save(a), replacement = save(b);
        std::string error;
        RIGIDBODIES_EXPECT(a.open_arrangement(replacement, error), error);
        command(a, K::undo);
        RIGIDBODIES_EXPECT(a.scenario_id() == "free_fall" && a.world().body_ids().size() == 4, "undo restores document identity and world");
        RIGIDBODIES_EXPECT(a.build_model().scenario_content->id == original_content->id &&
                a.build_model().scenario_content->guide.focus == original_content->guide.focus,
            "undo restores the opened document's guide and annotation source");
        a.reset_scenario();
        RIGIDBODIES_EXPECT(a.scenario_id() == "free_fall" && a.world().body_ids().size() == 4, "undo restored old baseline");
        RIGIDBODIES_EXPECT(a.open_arrangement(replacement, error), error);
        command(a, K::undo);
        command(a, K::redo);
        RIGIDBODIES_EXPECT(a.scenario_id() == "revolute_drive" && a.world().body_ids().size() == b.world().body_ids().size(), "redo reinstates imported world and source");
        RIGIDBODIES_EXPECT(a.build_model().scenario_content->id == replacement_content->id &&
                a.build_model().scenario_content->guide.focus == replacement_content->guide.focus,
            "redo restores the imported guide and annotation source");
    }

    RIGIDBODIES_TEST("quick save preserves the chosen title guide option and starting or current state")
    {
        for (const bool current_moment : { false, true })
        {
            app::SimulationSession session;
            RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "save fixture opens");
            const auto body = session.world().body_ids().back();
            const auto starting_position = session.world().find_body(body)->position_m();
            session.world().find_body(body)->set_position(starting_position + math::Vec2 { 2, 1 });
            const auto current_position = session.world().find_body(body)->position_m();
            session.note_setup_file("my-fall.json", "My custom fall", "free_fall", current_moment, false);
            std::string text, error;
            RIGIDBODIES_EXPECT(session.save_current_arrangement(text, error), error);
            physics::content::Json saved;
            RIGIDBODIES_EXPECT(physics::content::parse_json(text, saved, error), error);
            RIGIDBODIES_EXPECT(saved.at("metadata").at("title").as_string() == "My custom fall" && !saved.find("guide"),
                "quick save retains the custom title and excluded guide");
            app::SimulationSession reopened;
            RIGIDBODIES_EXPECT(reopened.open_arrangement(text, error), error);
            RIGIDBODIES_EXPECT(reopened.world().find_body(reopened.world().body_ids().back())->position_m() ==
                    (current_moment ? current_position : starting_position),
                "quick save honors the selected starting or current state");
        }
    }

    RIGIDBODIES_TEST("opening and undoing setups restores the matching save destination")
    {
        app::SimulationSession session, replacement;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall") && replacement.load_scenario("revolute_drive"), "fixtures open");
        session.note_setup_file("first.json", "My first setup", "free_fall", true, false);
        const auto second = save(replacement);
        std::string error;
        RIGIDBODIES_EXPECT(session.open_arrangement(second, error, "second.json"), error);
        RIGIDBODIES_EXPECT(session.current_setup_path() == "second.json", "open associates its source file");
        session.note_setup_file("second-copy.json", "Second copy", "revolute_drive", false, false);
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.current_setup_path() == "first.json" && session.current_setup_file().title == "My first setup" &&
                session.current_setup_file().current_moment && !session.current_setup_file().include_guide,
            "undo restores the previous document's destination and save choices");
        command(session, K::redo);
        RIGIDBODIES_EXPECT(session.current_setup_path() == "second-copy.json" && session.current_setup_file().title == "Second copy",
            "redo restores the document's most recent Save as destination");
        RIGIDBODIES_EXPECT(!session.open_arrangement("broken JSON", error, "invalid.json") && session.current_setup_path() == "second-copy.json",
            "failed open never redirects the next save");
        RIGIDBODIES_EXPECT(session.open_arrangement(second, error) && session.current_setup_path().empty(),
            "opening content without a source path never inherits another document's destination");
    }

    RIGIDBODIES_TEST("saving a setup keeps its association across edits resets and scenario history")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
        session.set_selection(session.world().body_ids().back());
        command(session, K::delete_selected_body);
        session.note_setup_file("edited.json", "Edited fall", "free_fall");
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.current_setup_path() == "edited.json", "undoing an edit made before saving keeps the current file");
        command(session, K::redo);
        session.reset_scenario();
        RIGIDBODIES_EXPECT(session.current_setup_path() == "edited.json", "Back to start keeps the document's save destination");
        ui::UiCommand open;
        open.kind = K::load_scenario;
        open.id = "revolute_drive";
        open.flag = true;
        session.apply(open);
        RIGIDBODIES_EXPECT(session.current_setup_path().empty(), "a new built-in experiment needs its own save destination");
        std::string text, error;
        RIGIDBODIES_EXPECT(!session.save_current_arrangement(text, error) && !error.empty(), "quick save cannot reuse the old file");
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.current_setup_path() == "edited.json", "undoing the experiment change restores the saved setup destination");
    }

    RIGIDBODIES_TEST("opened setup quick saves keep custom titles and excluded teaching content")
    {
        app::SimulationSession source, reopened;
        RIGIDBODIES_EXPECT(source.load_scenario("free_fall"), "fixture opens");
        std::string text, error;
        RIGIDBODIES_EXPECT(source.save_arrangement(text, error, "A guide-free setup", false, false), error);
        RIGIDBODIES_EXPECT(reopened.open_arrangement(text, error, "guide-free.json"), error);
        RIGIDBODIES_EXPECT(reopened.save_current_arrangement(text, error), error);
        physics::content::Json saved;
        RIGIDBODIES_EXPECT(physics::content::parse_json(text, saved, error), error);
        RIGIDBODIES_EXPECT(saved.at("metadata").at("title").as_string() == "A guide-free setup" && !saved.find("guide"),
            "reopening and saving does not rename the setup or reintroduce the guide");
    }

    RIGIDBODIES_TEST("saving marks the edited setup clean without erasing its lesson comparison")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
        session.set_selection(session.world().body_ids().back());
        command(session, K::delete_selected_body);
        const auto changes = session.build_model().changes.size();
        RIGIDBODIES_EXPECT(changes > 0, "deleting an original object changes the lesson setup");
        session.note_setup_file("saved.json", "Edited fall", "free_fall");
        RIGIDBODIES_EXPECT(session.build_model().changes.size() == changes, "saving preserves the comparison with the original experiment");
        command(session, K::quit);
        RIGIDBODIES_EXPECT(session.should_quit() && !session.build_model().confirmation, "a successfully saved setup quits without a loss warning");
    }

    RIGIDBODIES_TEST("later edits and undo redo compare against the most recent saved setup")
    {
        for (const int history_steps : { 0, 1, 2 })
        {
            app::SimulationSession session;
            RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
            session.note_setup_file("saved.json", "Saved fall", "free_fall");
            session.set_selection(session.world().body_ids().back());
            ui::UiCommand rotate { K::set_selected_orientation };
            rotate.value = 37.0;
            session.apply(rotate);
            if (history_steps >= 1)
                command(session, K::undo);
            if (history_steps == 2)
                command(session, K::redo);
            command(session, K::quit);
            RIGIDBODIES_EXPECT(session.should_quit() == (history_steps == 1), "only undoing back to the saved orientation is clean");
            RIGIDBODIES_EXPECT(session.build_model().confirmation.has_value() == (history_steps != 1), "redo restores the unsaved edit warning");
        }
    }

    RIGIDBODIES_TEST("unsaved original object edits are protected when leaving a fresh experiment")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
        session.set_selection(session.world().body_ids().back());
        ui::UiCommand rotate { K::set_selected_orientation };
        rotate.value = 37.0;
        session.apply(rotate);
        ui::UiCommand leave { K::load_scenario };
        leave.id = "revolute_drive";
        session.apply(leave);
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall" && session.build_model().confirmation.has_value(), "editing only an original object's orientation asks before replacing the setup");
        RIGIDBODIES_EXPECT(session.build_model().confirmation->save_label == "Save and leave…", "the save action explains that it will continue to the next experiment");
        command(session, K::quit);
        RIGIDBODIES_EXPECT(!session.should_quit() && session.build_model().confirmation->save_label == "Save and quit…", "a newer quit request replaces the pending leave request");
        const auto quit = session.take_pending_departure_for_save();
        RIGIDBODIES_EXPECT(quit && quit->command.kind == K::quit && !session.build_model().confirmation, "Save continues the departure shown by the confirmation");
        command(session, K::quit);
        session.apply(leave);
        const auto load = session.take_pending_departure_for_save();
        RIGIDBODIES_EXPECT(load && load->command.kind == K::load_scenario && load->command.id == leave.id, "a newer leave request likewise replaces a pending quit");
    }

    RIGIDBODIES_TEST("saving a current moment stays clean as the simulation continues")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
        session.stepper().set_paused(false);
        session.advance(0.1);
        std::string text, error;
        RIGIDBODIES_EXPECT(session.save_arrangement(text, error, "Moving fall", true, true), error);
        session.note_setup_file("moving.json", "Moving fall", "free_fall", true, true);
        session.advance(0.2);
        command(session, K::quit);
        RIGIDBODIES_EXPECT(session.should_quit(), "normal playback after a save does not create a new unsaved setup edit");
    }

    RIGIDBODIES_TEST("save departures continue only after completion and never discard an open draft")
    {
        for (const bool leave_scene : { false, true })
        {
            app::SimulationSession session;
            RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
            session.set_selection(session.world().body_ids().back());
            command(session, K::delete_selected_body);
            ui::UiCommand requested { leave_scene ? K::load_scenario : K::quit };
            requested.id = "revolute_drive";
            session.apply(requested);
            const auto cancelled = session.take_pending_departure_for_save();
            RIGIDBODIES_EXPECT(cancelled && !session.should_quit() && session.scenario_id() == "free_fall", "opening Save details does not yet quit or leave");
            RIGIDBODIES_EXPECT(!session.take_pending_departure_for_save(), "cancelling or failing Save leaves no hidden pending departure for a later save");
            session.note_setup_file("saved.json", "Edited fall", "free_fall");
            RIGIDBODIES_EXPECT(!session.should_quit() && session.scenario_id() == "free_fall", "an unrelated later save does not perform the cancelled departure");
            session.complete_saved_departure(*cancelled);
            RIGIDBODIES_EXPECT(leave_scene ? session.scenario_id() == "revolute_drive" : session.should_quit(), "the application can continue the requested departure after its successful write");
        }
        app::SimulationSession draft;
        RIGIDBODIES_EXPECT(draft.load_scenario("free_fall"), "draft fixture opens");
        command(draft, K::start_new_shape);
        command(draft, K::quit);
        RIGIDBODIES_EXPECT(draft.build_model().confirmation && draft.build_model().confirmation->save_label == "Save setup…", "saving a setup never promises to save an unfinished outline");
        const auto pending = draft.take_pending_departure_for_save();
        RIGIDBODIES_EXPECT(pending.has_value(), "draft quit can open Save details");
        draft.note_setup_file("draft-base.json", "Committed setup", "free_fall");
        draft.complete_saved_departure(*pending);
        RIGIDBODIES_EXPECT(!draft.should_quit() && draft.shape_editor().active(), "saving the committed setup preserves the unsaved shape draft");
    }

    RIGIDBODIES_TEST("an opened authored setup is clean until an actual setup edit")
    {
        app::SimulationSession source, opened;
        RIGIDBODIES_EXPECT(source.load_scenario("empty_lab"), "fixture opens");
        std::string error;
        RIGIDBODIES_EXPECT(source.import_shape(shape_file(), error), error);
        RIGIDBODIES_EXPECT(opened.open_arrangement(save(source), error, "authored.json"), error);
        ui::UiCommand leave { K::load_scenario };
        leave.id = "free_fall";
        opened.apply(leave);
        RIGIDBODIES_EXPECT(opened.scenario_id() == "free_fall" && !opened.build_model().confirmation, "saved authored objects do not by themselves trigger a loss warning");
    }

    RIGIDBODIES_TEST("malformed incompatible and wrong-kind files preserve scene selection and history")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        session.set_selection(session.world().body_ids().back());
        const auto selected = session.selection();
        const auto original = save(session);
        const auto undo_label = session.build_model().undo_label;
        physics::content::Json root;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(original, root, error), error);
        root["version"]["major"] = 99;
        for (const auto& bad : { std::string { "{\"broken\":" }, physics::content::write_json(root), shape_file() })
        {
            RIGIDBODIES_EXPECT(!session.open_arrangement(bad, error) && !error.empty(), "invalid file reports a diagnostic");
            RIGIDBODIES_EXPECT(save(session) == original && session.selection() == selected, "failure preserves exact arrangement and selection");
            RIGIDBODIES_EXPECT(session.build_model().undo_label == undo_label, "failure does not create an undo edit");
        }
    }

    RIGIDBODIES_TEST("arrangement history preserves custom guide content outside the scenario catalogue")
    {
        app::SimulationSession session;
        RIGIDBODIES_EXPECT(session.load_scenario("free_fall"), "fixture opens");
        physics::content::Json custom;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(save(session), custom, error), error);
        custom["metadata"]["id"] = "custom_fall_setup";
        custom["metadata"]["title"] = "My falling objects";
        custom["guide"]["focus"] = "Compare my custom objects.";
        RIGIDBODIES_EXPECT(session.open_arrangement(physics::content::write_json(custom), error), error);
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.build_model().scenario_content->id == "free_fall", "undo reinstates the original experiment content");
        command(session, K::redo);
        const auto restored = session.build_model().scenario_content;
        RIGIDBODIES_EXPECT(restored && restored->id == "custom_fall_setup" && restored->title == "My falling objects" &&
                restored->guide.focus == "Compare my custom objects.",
            "redo uses the saved document's custom identity and teaching content");
    }

    RIGIDBODIES_TEST("future additive metadata survives opening editing saving and undo")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        physics::content::Json root;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(save(session), root, error), error);
        root["version"]["minor"] = 7;
        root["future_annotation"] = "keep this";
        RIGIDBODIES_EXPECT(session.open_arrangement(physics::content::write_json(root), error), error);
        session.world().find_body(session.world().body_ids().back())->set_linear_velocity({ 5, 6 });
        physics::content::Json saved;
        RIGIDBODIES_EXPECT(physics::content::parse_json(save(session), saved, error), error);
        RIGIDBODIES_EXPECT(saved.at("future_annotation").as_string() == "keep this", "unknown source annotation retained");
        RIGIDBODIES_EXPECT(saved.at("version").at("minor").as_number() == 7, "future minor retained");
        RIGIDBODIES_EXPECT(!session.build_model().scenario_concepts.empty(), "lesson metadata is available to the interface");
    }

    RIGIDBODIES_TEST("shape import creates a selectable authored body and is one reversible edit")
    {
        app::SimulationSession session;
        session.camera().set_center({ 4, 5 });
        std::string error, exported;
        RIGIDBODIES_EXPECT(session.import_shape(shape_file(), error), error);
        const auto id = session.selection();
        const auto* body = session.world().find_body(id);
        RIGIDBODIES_EXPECT(body && body->position_m() == math::Vec2 { 4, 5 }, "import at camera centre");
        RIGIDBODIES_EXPECT(physics::authored_parts(*body).size() == 1 && session.stepper().is_paused(), "logical authoring source kept and simulation paused");
        RIGIDBODIES_EXPECT(session.export_shape(exported, error), error);
        physics::ShapeDocument document;
        RIGIDBODIES_EXPECT(physics::parse_shape_document(exported, document, error), error);
        RIGIDBODIES_EXPECT(document.title == "Triangle" && document.shape->source.nodes.size() == 3, "standalone export retains title and editable nodes");
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.world().body_ids().empty(), "one undo removes import");
        command(session, K::redo);
        RIGIDBODIES_EXPECT(session.world().is_valid(id) && session.selection() == id, "redo restores body and selection");
    }

    RIGIDBODIES_TEST("shape export requires an authored selection and invalid import does not mutate")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        session.set_selection(session.world().body_ids().back());
        std::string text = "unchanged", error;
        const auto original = save(session);
        RIGIDBODIES_EXPECT(!session.export_shape(text, error) && text == "unchanged", "ordinary primitive cannot pretend to be an authored outline");
        RIGIDBODIES_EXPECT(!session.import_shape(original, error), "scenario is not a shape");
        RIGIDBODIES_EXPECT(!session.import_shape("{}", error), "invalid shape rejected");
        RIGIDBODIES_EXPECT(save(session) == original, "failed imports preserve world");
    }

    RIGIDBODIES_TEST("standalone shape extensions survive import export and undo redo")
    {
        app::SimulationSession session;
        physics::content::Json root;
        std::string error, text;
        RIGIDBODIES_EXPECT(physics::content::parse_json(shape_file(), root, error), error);
        root["version"]["minor"] = 4;
        root["future_note"] = "preserve";
        RIGIDBODIES_EXPECT(session.import_shape(physics::content::write_json(root), error), error);
        const auto arrangement = save(session);
        RIGIDBODIES_EXPECT(session.open_arrangement(arrangement, error), error);
        session.set_selection(session.world().body_ids().front());
        session.reset_scenario();
        session.set_selection(session.world().body_ids().front());
        command(session, K::undo);
        command(session, K::redo);
        session.set_selection(session.world().body_ids().front());
        RIGIDBODIES_EXPECT(session.export_shape(text, error), error);
        physics::content::Json exported;
        RIGIDBODIES_EXPECT(physics::content::parse_json(text, exported, error), error);
        RIGIDBODIES_EXPECT(exported.at("future_note").as_string() == "preserve" && exported.at("version").at("minor").as_number() == 4, "source envelope retained through history");
    }

    RIGIDBODIES_TEST("invalid saved playback settings cannot partially replace a session")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        const auto before = save(session);
        physics::content::Json root;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(before, root, error), error);
        root["playback"]["substeps"] = 1.5;
        RIGIDBODIES_EXPECT(session.open_arrangement(physics::content::write_json(root), error), "engine-only playback fields are ignored on open");
        root["presentation"]["view"]["center_x_m"] = 1.0e20;
        RIGIDBODIES_EXPECT(!session.open_arrangement(physics::content::write_json(root), error), "unrenderable grid centre rejected");
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall", "invalid view leaves the open world intact");
    }

    RIGIDBODIES_TEST("out-of-world shape import preserves pacing and undo history")
    {
        app::SimulationSession session;
        session.camera().set_center({ 101, 0 });
        session.stepper().set_paused(false);
        const auto before = save(session);
        std::string error;
        RIGIDBODIES_EXPECT(!session.import_shape(shape_file(), error), "placement outside world bounds rejected");
        RIGIDBODIES_EXPECT(save(session) == before && !session.stepper().is_paused() && !session.build_model().can_undo, "failure has no session side effects");
    }

    RIGIDBODIES_TEST("a stale speed preview cancellation cannot change a replaced or reset document")
    {
        app::SimulationSession source;
        source.load_scenario("revolute_drive");
        source.stepper().set_time_scale(2.0);
        const auto target = save(source);
        for (const auto operation : { "file", "experiment", "reset" })
        {
            app::SimulationSession session;
            session.load_scenario("free_fall");
            ui::UiCommand speed { K::set_time_scale };
            speed.phase = ui::UiEditPhase::preview;
            speed.value = 0.5;
            session.apply(speed);
            std::string error;
            if (std::string_view(operation) == "file")
                RIGIDBODIES_EXPECT(session.open_arrangement(target, error), error);
            else if (std::string_view(operation) == "experiment")
                RIGIDBODIES_EXPECT(session.load_scenario("revolute_drive"), "the replacement experiment opens");
            else
            {
                // Back to start is intentionally a no-op before time has advanced.
                command(session, K::single_step);
                session.advance(0.0);
                session.reset_scenario();
            }
            const auto expected_speed = session.stepper().time_scale();
            speed.phase = ui::UiEditPhase::cancel;
            session.apply(speed);
            RIGIDBODIES_EXPECT_NEAR(session.stepper().time_scale(), expected_speed, 0.0, "a delayed cancellation from the old interaction cannot replace the new document's playback speed");
            RIGIDBODIES_EXPECT_NEAR(expected_speed, std::string_view(operation) == "file" ? 2.0 : 0.5, 0.0, "file speed is restored and regular reset or experiment opening keeps the transport speed");
        }
    }

    RIGIDBODIES_TEST("save completion records the captured setup rather than newer edits")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        session.set_selection(session.world().body_ids().back());
        ui::UiCommand rotate { K::set_selected_orientation };
        rotate.value = 20.0;
        session.apply(rotate);
        command(session, K::quit);
        const auto departure = session.take_pending_departure_for_save();
        std::string error;
        const auto snapshot = session.capture_setup_save(error, "Captured fall", false, false);
        RIGIDBODIES_EXPECT(snapshot && departure, error);
        session.set_wall_time(2.0);
        rotate.value = 55.0;
        session.apply(rotate);
        RIGIDBODIES_EXPECT(!session.complete_setup_save(*snapshot, "captured.json"), "newer edits are not marked saved by an older write");
        session.complete_saved_departure(*departure);
        RIGIDBODIES_EXPECT(!session.should_quit(), "Save and quit does not discard the newer edit");
        command(session, K::undo);
        command(session, K::quit);
        RIGIDBODIES_EXPECT(session.should_quit(), "undoing to the exact captured setup returns to its saved baseline");
    }

    RIGIDBODIES_TEST("save completion remains attached to its original document across scene switches")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        std::string error;
        const auto snapshot = session.capture_setup_save(error, "Earlier document", false, true);
        RIGIDBODIES_EXPECT(snapshot.has_value(), error);
        RIGIDBODIES_EXPECT(session.load_scenario("revolute_drive"), "another clean experiment opens");
        session.note_setup_file("newer.json", "Newer document", "revolute_drive");
        RIGIDBODIES_EXPECT(!session.complete_setup_save(*snapshot, "earlier.json"), "a background save cannot claim the newly opened document");
        RIGIDBODIES_EXPECT(session.current_setup_path() == "newer.json" && session.current_setup_file().title == "Newer document", "the current save destination is preserved");
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall" && session.current_setup_path() == "earlier.json", "undoing the scene switch recovers the earlier document's completed save");
    }

    RIGIDBODIES_TEST("an older save completion cannot replace a newer successful destination")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        std::string error;
        const auto earlier = session.capture_setup_save(error, "Earlier", false, true);
        const auto newer = session.capture_setup_save(error, "Newer", true, false);
        RIGIDBODIES_EXPECT(earlier && newer, error);
        RIGIDBODIES_EXPECT(session.complete_setup_save(*newer, "newer.json"), "the newer request completes");
        RIGIDBODIES_EXPECT(!session.complete_setup_save(*earlier, "earlier.json"), "the stale completion is recognized");
        RIGIDBODIES_EXPECT(session.current_setup_path() == "newer.json" && session.current_setup_file().title == "Newer" && !session.current_setup_file().include_guide, "quick Save continues using the latest completed choices");
    }

    RIGIDBODIES_TEST("a stale save snapshot cannot overwrite a newer completed file through a path alias")
    {
        TemporaryDirectory directory;
        const auto path = directory.path / "saved.json";
        app::SimulationSession session;
        session.load_scenario("free_fall");
        std::string error, written;
        const auto earlier = session.capture_setup_save(error, "Earlier", false, true);
        session.set_selection(session.world().body_ids().back());
        command(session, K::delete_selected_body);
        const auto newer = session.capture_setup_save(error, "Newer", false, true);
        RIGIDBODIES_EXPECT(earlier && newer, error);
        RIGIDBODIES_EXPECT(app::write_content_file(path, newer->text, error), error);
        RIGIDBODIES_EXPECT(session.complete_setup_save(*newer, path.u8string()), "the newer file establishes the saved baseline");
        const auto alias = directory.path / "." / "saved.json";
        RIGIDBODIES_EXPECT(!session.can_write_setup_save(*earlier, alias.u8string(), error) && !error.empty(), "an older native dialog cannot overwrite the newer file through an equivalent path");
        RIGIDBODIES_EXPECT(app::read_content_file(path, written, error) && written == newer->text, "the newer saved bytes remain intact");
        RIGIDBODIES_EXPECT(session.can_write_setup_save(*earlier, (directory.path / "older-copy.json").u8string(), error), "saving the old snapshot to a separate file is still allowed");
    }

    RIGIDBODIES_TEST("a later departure request invalidates an earlier Save and quit continuation")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        session.set_selection(session.world().body_ids().back());
        command(session, K::delete_selected_body);
        command(session, K::quit);
        const auto departure = session.take_pending_departure_for_save();
        std::string error;
        const auto snapshot = session.capture_setup_save(error, "Saved fall", false, true);
        RIGIDBODIES_EXPECT(snapshot && departure, error);
        ui::UiCommand leave { K::load_scenario };
        leave.id = "revolute_drive";
        session.apply(leave);
        RIGIDBODIES_EXPECT(session.complete_setup_save(*snapshot, "saved.json"), "the original document is saved");
        session.complete_saved_departure(*departure);
        RIGIDBODIES_EXPECT(!session.should_quit() && session.build_model().confirmation && session.build_model().confirmation->confirm.id == "revolute_drive", "finishing the older save cannot revive a superseded quit request");
    }

    RIGIDBODIES_TEST("opening a file protects unsaved edits and deliberate discard remains undoable")
    {
        app::SimulationSession session, source;
        session.load_scenario("free_fall");
        source.load_scenario("revolute_drive");
        session.note_setup_file("old.json", "Original file", "free_fall");
        session.set_selection(session.world().body_ids().back());
        command(session, K::delete_selected_body);
        const auto original_ids = session.world().body_ids();
        const auto history = session.build_model().undo_history.size();
        std::string error;
        const auto target = save(source);
        RIGIDBODIES_EXPECT(session.request_open_arrangement(target, error, "new.json"), error);
        auto confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation && confirmation->save_label == "Save and open…" && session.current_setup_path() == "old.json", "Open asks how to handle the current edited document");
        session.apply(confirmation->cancel);
        RIGIDBODIES_EXPECT(!session.build_model().confirmation && session.world().body_ids() == original_ids && session.build_model().undo_history.size() == history, "Cancel preserves both edits and history");
        RIGIDBODIES_EXPECT(session.request_open_arrangement(target, error, "new.json"), error);
        confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation.has_value(), "discard is an explicit second decision");
        session.apply(confirmation->confirm);
        RIGIDBODIES_EXPECT(session.scenario_id() == "revolute_drive" && session.current_setup_path() == "new.json", "Discard and open uses the staged file");
        command(session, K::undo);
        RIGIDBODIES_EXPECT(session.scenario_id() == "free_fall" && session.world().body_ids() == original_ids && session.current_setup_path() == "old.json", "the previous edited document remains recoverable through Undo");
    }

    RIGIDBODIES_TEST("Save and open continues only when the captured editable setup is still current")
    {
        for (const bool edit_during_save : { false, true })
        {
            app::SimulationSession session, source;
            session.load_scenario("free_fall");
            source.load_scenario("revolute_drive");
            session.set_selection(session.world().body_ids().back());
            ui::UiCommand rotate { K::set_selected_orientation };
            rotate.value = 25.0;
            session.apply(rotate);
            std::string error;
            RIGIDBODIES_EXPECT(session.request_open_arrangement(save(source), error, "target.json"), error);
            const auto departure = session.take_pending_departure_for_save();
            const auto snapshot = session.capture_setup_save(error, "Saved source", false, true);
            RIGIDBODIES_EXPECT(departure && snapshot, error);
            if (edit_during_save)
            {
                rotate.value = 70.0;
                session.apply(rotate);
            }
            const auto current = session.complete_setup_save(*snapshot, "source.json");
            RIGIDBODIES_EXPECT(current != edit_during_save, "the completion identifies whether newer edits remain");
            session.complete_saved_departure(*departure);
            RIGIDBODIES_EXPECT(session.scenario_id() == (edit_during_save ? "free_fall" : "revolute_drive"), "only a completed save of the current setup continues opening");
        }
    }

    RIGIDBODIES_TEST("invalid files are rejected before asking to discard a setup or draft")
    {
        app::SimulationSession session, source;
        session.load_scenario("free_fall");
        source.load_scenario("revolute_drive");
        command(session, K::start_new_shape);
        const auto history = session.build_model().undo_history.size();
        physics::content::Json target;
        std::string error;
        RIGIDBODIES_EXPECT(physics::content::parse_json(save(source), target, error), error);
        target["presentation"]["view"]["center_x_m"] = 1.0e20;
        for (const auto& invalid : { std::string("not JSON"), physics::content::write_json(target) })
        {
            RIGIDBODIES_EXPECT(!session.request_open_arrangement(invalid, error, "invalid.json"), "invalid input fails preflight");
            RIGIDBODIES_EXPECT(!error.empty() && !session.build_model().confirmation && session.shape_editor().active() && session.scenario_id() == "free_fall" && session.build_model().undo_history.size() == history, "failed Open leaves the existing draft and history untouched");
        }
        RIGIDBODIES_EXPECT(session.request_open_arrangement(save(source), error, "valid.json"), error);
        const auto confirmation = session.build_model().confirmation;
        RIGIDBODIES_EXPECT(confirmation && confirmation->save_label == "Save setup…", "a valid file still protects the unsaved draft");
        session.apply(confirmation->cancel);
        RIGIDBODIES_EXPECT(session.shape_editor().active(), "cancelling Open keeps the draft editable");
    }

    RIGIDBODIES_TEST("suggested document names preserve titles while removing invalid filename characters")
    {
        RIGIDBODIES_EXPECT(app::suggested_content_filename("  My / falling: setup... ") == "My  falling  setup.rbscenario.json", "titles become readable filename suggestions");
        RIGIDBODIES_EXPECT(app::suggested_content_filename("CON") == "_CON.rbscenario.json", "reserved Windows names are made usable");
        RIGIDBODIES_EXPECT(app::suggested_content_filename("... ", true) == "My shape.rbshape.json", "an empty title has a useful shape fallback");
        RIGIDBODIES_EXPECT(app::suggested_content_filename("\xC5\x9E"
                                                           "ekil",
                               true) == "\xC5\x9E"
                                        "ekil.rbshape.json",
            "Unicode names remain intact");
        RIGIDBODIES_EXPECT(app::content_save_destination("experiment").filename() == "experiment.rbscenario.json", "extensionless setup names receive their document suffix");
        RIGIDBODIES_EXPECT(app::content_save_destination("custom.json", true).filename() == "custom.json", "an explicit extension is respected");
    }

    RIGIDBODIES_TEST("content files support Unicode paths and complete replacement without leftover staging")
    {
        TemporaryDirectory directory;
        const auto path = directory.path / std::filesystem::u8path("\xC5\x9F"
                                                                   "ekil.rbshape.json");
        std::string error, text;
        const auto first = shape_file();
        RIGIDBODIES_EXPECT(app::write_content_file(path, first, error), error);
        RIGIDBODIES_EXPECT(app::read_content_file(path, text, error) && text == first, "UTF8 path complete roundtrip");
        RIGIDBODIES_EXPECT(app::write_content_file(path, "replacement", error), error);
        RIGIDBODIES_EXPECT(app::read_content_file(path, text, error) && text == "replacement", "replace existing file");
        std::size_t count = 0;
        for (const auto& entry : std::filesystem::directory_iterator(directory.path))
        {
            (void)entry;
            ++count;
        }
        RIGIDBODIES_EXPECT(count == 1, "no temporary file or folder left after success");
    }

    RIGIDBODIES_TEST("failed file reads and oversized saves leave existing data untouched")
    {
        TemporaryDirectory directory;
        const auto path = directory.path / "existing.json";
        std::string error, text = "kept";
        RIGIDBODIES_EXPECT(app::write_content_file(path, "original", error), error);
        RIGIDBODIES_EXPECT(!app::read_content_file(directory.path / "missing", text, error) && text == "kept", "failed read retains output");
        const std::string oversized(app::maximum_content_file_bytes + 1, ' ');
        RIGIDBODIES_EXPECT(!app::write_content_file(path, oversized, error), "oversized save fails before replacing");
        RIGIDBODIES_EXPECT(app::read_content_file(path, text, error) && text == "original", "old document retained");
        RIGIDBODIES_EXPECT(!app::write_content_file(directory.path, "document", error), "directory is not overwritten");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
