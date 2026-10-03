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
        const auto original = save(a), replacement = save(b);
        std::string error;
        RIGIDBODIES_EXPECT(a.open_arrangement(replacement, error), error);
        command(a, K::undo);
        RIGIDBODIES_EXPECT(a.scenario_id() == "free_fall" && a.world().body_ids().size() == 4, "undo restores document identity and world");
        a.reset_scenario();
        RIGIDBODIES_EXPECT(a.scenario_id() == "free_fall" && a.world().body_ids().size() == 4, "undo restored old baseline");
        RIGIDBODIES_EXPECT(a.open_arrangement(replacement, error), error);
        command(a, K::undo);
        command(a, K::redo);
        RIGIDBODIES_EXPECT(a.scenario_id() == "revolute_drive" && a.world().body_ids().size() == b.world().body_ids().size(), "redo reinstates imported world and source");
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
