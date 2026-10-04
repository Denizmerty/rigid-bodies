#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/app/content_files.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/shape_document.hpp>

#include <algorithm>
#include <exception>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace rigidbodies::app
{
    namespace
    {
        double document_number(const physics::content::Json& object, const char* key, double fallback, double minimum, double maximum, bool integer = false)
        {
            if (!object.is_object())
                throw std::invalid_argument("Playback and view settings must be objects.");
            const auto* value = object.find(key);
            if (!value)
                return fallback;
            if (!value->is_number() || value->as_number() < minimum || value->as_number() > maximum ||
                (integer && std::floor(value->as_number()) != value->as_number()))
                throw std::invalid_argument(std::string { "Invalid saved setting: " } + key);
            return value->as_number();
        }

        bool read_saved_presentation(const physics::ScenarioDocument& document, physics::TimeStepper& pacing,
            render::Camera2D& view, std::string& error)
        {
            try
            {
                if (const auto* playback = document.root.find("playback"))
                {
                    const auto saved_speed = document_number(*playback, "time_scale", pacing.time_scale(), -std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                    pacing.set_time_scale(std::clamp(saved_speed, 0.05, 4.0));
                }
                const auto* saved_view = document.root.find("view");
                if (const auto* presentation = document.root.find("presentation"); presentation && presentation->is_object())
                    if (const auto* preferred = presentation->find("view"))
                        saved_view = preferred;
                if (saved_view)
                {
                    view.set_center({ document_number(*saved_view, "center_x_m", view.center_m().x, -1.0e9, 1.0e9),
                        document_number(*saved_view, "center_y_m", view.center_m().y, -1.0e9, 1.0e9) });
                    view.set_view_height(document_number(*saved_view, "height_m", view.view_height_m(), view.minimum_view_height_m(), view.maximum_view_height_m()));
                }
            }
            catch (const std::exception& exception)
            {
                error = exception.what();
                return false;
            }
            return true;
        }
    }
    void SimulationSession::set_content_message(std::string message)
    {
        notify(ui::Severity::info, std::move(message), "content");
    }

    void SimulationSession::notify(ui::Severity severity, std::string message, std::string source,
        std::optional<ui::NotificationAction> action, bool persistent)
    {
        (void)notifier_.post(severity, std::move(message), std::move(source), std::move(action), persistent);
    }

    bool SimulationSession::save_arrangement(std::string& text, std::string& error) const
    {
        return save_arrangement(text, error, {}, true, true);
    }

    bool SimulationSession::save_arrangement(std::string& text, std::string& error, std::string_view title,
        bool current_moment, bool include_guide) const
    {
        if (!math::is_finite(camera_.center_m()) || std::abs(camera_.center_m().x) > 1.0e9 || std::abs(camera_.center_m().y) > 1.0e9)
        {
            error = "Move the view within one billion metres of the origin before saving.";
            return false;
        }
        physics::ScenarioMetadata metadata;
        if (scenario_document_)
            metadata = scenario_document_->metadata;
        else if (const auto* description = physics::find_scenario(scenario_id_))
            metadata = { description->id, description->title, description->summary, description->concepts, description->prerequisites, description->tags, description->collection, description->level, description->hook, description->collection_order, description->suggested_order, description->lab };
        else
            metadata = { "my_arrangement", "My arrangement", "An arrangement saved from the playground.", {}, {}, {}, "Saved setups", "", "", 0, 0, true };
        auto based_on = scenario_document_ ? scenario_document_->metadata.id : scenario_id_;
        if (scenario_document_)
            if (const auto* source_metadata = scenario_document_->root.find("metadata"); source_metadata && source_metadata->is_object())
                if (const auto* source = source_metadata->find("based_on"); source && source->is_string() && !source->as_string().empty())
                    based_on = source->as_string();
        if (!title.empty())
        {
            metadata.title = std::string(title);
            std::string slug;
            for (const auto ch : title)
            {
                const auto byte = static_cast<unsigned char>(ch);
                if (std::isalnum(byte))
                    slug.push_back(static_cast<char>(std::tolower(byte)));
                else if (!slug.empty() && slug.back() != '_')
                    slug.push_back('_');
            }
            while (!slug.empty() && slug.back() == '_')
                slug.pop_back();
            metadata.id = (slug.empty() ? std::string { "my_setup" } : slug) + "_setup";
            metadata.collection = "make_your_own";
            metadata.lab = true;
        }
        physics::ScenarioDocument document;
        const auto& saved_world = current_moment ? world_ : setup_view_;
        if (!physics::capture_scenario_document(saved_world, metadata, document, error, scenario_document_ ? &*scenario_document_ : nullptr))
            return false;
        if (!title.empty())
        {
            if (auto* saved_metadata = document.root.find("metadata"); saved_metadata && saved_metadata->is_object())
                (*saved_metadata)["based_on"] = based_on;
        }
        if (!include_guide && document.root.is_object())
            document.root.as_object().erase("guide");
        auto& playback = document.root["playback"];
        if (!playback.is_object())
            playback = physics::content::Json::Object {};
        playback["fixed_step_s"] = stepper_.fixed_step_s();
        playback["substeps"] = stepper_.substep_count();
        playback["time_scale"] = stepper_.time_scale();
        playback["maximum_substeps_per_frame"] = stepper_.maximum_substeps_per_frame();
        playback["maximum_frame_time_s"] = stepper_.maximum_frame_time_s();
        auto& presentation = document.root["presentation"];
        if (!presentation.is_object())
            presentation = physics::content::Json::Object {};
        physics::content::Json::Array layers;
        for (const auto& layer : render::layer_catalogue())
            if (scene_settings_.layers.is_enabled(layer.layer))
                layers.emplace_back(std::string { layer.id });
        presentation["layers"] = std::move(layers);
        auto& view = presentation["view"];
        if (!view.is_object())
            view = physics::content::Json::Object {};
        view["center_x_m"] = camera_.center_m().x;
        view["center_y_m"] = camera_.center_m().y;
        view["height_m"] = camera_.view_height_m();
        return physics::write_scenario_document(document, text, error);
    }

    bool SimulationSession::save_current_arrangement(std::string& text, std::string& error) const
    {
        if (setup_file_->path.empty())
        {
            error = "Choose a destination with Save as before saving this setup.";
            return false;
        }
        return save_arrangement(text, error, setup_file_->title, setup_file_->current_moment, setup_file_->include_guide);
    }

    std::string SimulationSession::setup_fingerprint() const
    {
        physics::ScenarioMetadata metadata;
        metadata.id = "saved_setup";
        metadata.title = "Saved setup";
        physics::ScenarioDocument document;
        std::string text, error;
        if (!physics::capture_scenario_document(setup_view_, metadata, document, error) ||
            !physics::write_scenario_document(document, text, error))
            return {};
        return text;
    }

    std::optional<SetupSaveSnapshot> SimulationSession::capture_setup_save(std::string& error,
        std::string_view title, bool current_moment, bool include_guide) const
    {
        SetupSaveSnapshot snapshot;
        if (!save_arrangement(snapshot.text, error, title, current_moment, include_guide))
            return std::nullopt;
        snapshot.document = setup_file_;
        snapshot.file = *setup_file_;
        snapshot.file.title = title.empty() ? (scenario_document_ ? scenario_document_->metadata.title : "My setup") : std::string(title);
        snapshot.file.based_on = experiment_content_ && !experiment_content_->based_on.empty() ? experiment_content_->based_on : scenario_id_;
        snapshot.file.current_moment = current_moment;
        snapshot.file.include_guide = include_guide;
        snapshot.file.saved_setup_fingerprint = setup_fingerprint();
        snapshot.serial = ++setup_file_->save_request_serial;
        return snapshot;
    }

    std::optional<SetupSaveSnapshot> SimulationSession::capture_current_setup_save(std::string& error) const
    {
        if (setup_file_->path.empty())
        {
            error = "Choose a destination with Save as before saving this setup.";
            return std::nullopt;
        }
        return capture_setup_save(error, setup_file_->title, setup_file_->current_moment, setup_file_->include_guide);
    }

    bool SimulationSession::can_write_setup_save(const SetupSaveSnapshot& snapshot, std::string_view path, std::string& error) const
    {
        if (!snapshot.document)
        {
            error = "The setup save is no longer available.";
            return false;
        }
        if (snapshot.serial < snapshot.document->save_completion_serial &&
            same_content_file(std::filesystem::u8path(path), std::filesystem::u8path(snapshot.document->path)))
        {
            error = "A newer version was already saved to this file. Save again or choose a different file name.";
            return false;
        }
        error.clear();
        return true;
    }

    bool SimulationSession::complete_setup_save(const SetupSaveSnapshot& snapshot, std::string path)
    {
        if (!snapshot.document)
            return false;
        auto saved = snapshot.file;
        saved.path = std::move(path);
        publish_setup_file(saved);
        // A newer quick Save may have completed while an older native dialog was still open.
        if (snapshot.serial < snapshot.document->save_completion_serial)
            return false;
        saved.save_request_serial = snapshot.document->save_request_serial;
        saved.save_completion_serial = snapshot.serial;
        *snapshot.document = std::move(saved);
        return setup_file_ == snapshot.document && !has_setup_changes();
    }

    std::optional<SetupDeparture> SimulationSession::take_pending_departure_for_save()
    {
        std::optional<SetupDeparture> departure;
        if (pending_quit_confirmation_)
            departure = SetupDeparture { ui::UiCommand { ui::UiCommandKind::quit } };
        else if (pending_setup_open_)
            departure = SetupDeparture { ui::UiCommand { ui::UiCommandKind::open_arrangement }, pending_setup_open_ };
        else if (!pending_leave_scenario_.empty())
        {
            departure = SetupDeparture { ui::UiCommand { ui::UiCommandKind::load_scenario } };
            departure->command.id = pending_leave_scenario_;
        }
        if (departure)
        {
            departure->command.flag = true;
            departure->document = setup_file_;
            departure->serial = departure_serial_;
        }
        pending_quit_confirmation_ = false;
        pending_leave_scenario_.clear();
        pending_setup_open_.reset();
        return departure;
    }

    void SimulationSession::complete_saved_departure(const SetupDeparture& departure)
    {
        // A setup file contains committed shapes; saving must never silently discard a draft.
        if (departure.serial != departure_serial_ || departure.document != setup_file_ || shape_editor_.active() || has_setup_changes())
            return;
        if (departure.opening)
        {
            std::string error;
            if (!open_arrangement(departure.opening->text, error, departure.opening->path))
                notify(ui::Severity::error, error, "content");
        }
        else if (departure.command.kind == ui::UiCommandKind::quit || departure.command.kind == ui::UiCommandKind::load_scenario)
            apply(departure.command);
    }

    void SimulationSession::note_setup_file(std::string path, std::string title, std::string based_on,
        bool current_moment, bool include_guide)
    {
        *setup_file_ = { std::move(path), std::move(title), std::move(based_on), current_moment, include_guide, setup_fingerprint() };
        publish_setup_file(*setup_file_);
    }

    void SimulationSession::publish_setup_file(const SetupFileAssociation& file)
    {
        const auto now = std::time(nullptr);
        std::tm local {};
#if defined(_WIN32)
        localtime_s(&local, &now);
#else
        localtime_r(&now, &local);
#endif
        std::ostringstream date;
        date << std::put_time(&local, "%Y-%m-%d %H:%M");
        last_setup_file_ = ui::SetupFileInfo { ++setup_file_serial_, file.title, file.based_on, date.str(), file.path };
    }

    bool SimulationSession::request_open_arrangement(std::string text, std::string& error, std::string path)
    {
        physics::ScenarioDocument document;
        auto pacing = stepper_;
        auto view = camera_;
        physics::World validation_world;
        if (!physics::parse_scenario_document(text, document, error) ||
            !read_saved_presentation(document, pacing, view, error) ||
            !physics::populate_world(document, validation_world, error))
            return false;
        ++departure_serial_;
        pending_leave_scenario_.clear();
        pending_quit_confirmation_ = false;
        pending_setup_open_.reset();
        if (shape_editor_.active() || has_setup_changes())
        {
            pending_setup_open_ = std::make_shared<PendingSetupOpen>(PendingSetupOpen { std::move(text), std::move(path), document.metadata.title });
            ++change_serial_;
            error.clear();
            return true;
        }
        return open_arrangement(text, error, std::move(path));
    }

    bool SimulationSession::open_arrangement(std::string_view text, std::string& error, std::string path)
    {
        const auto carried_lab = lab_;
        const auto had_setup = setup_.world.is_valid();
        physics::ScenarioDocument document;
        if (!physics::parse_scenario_document(text, document, error))
            return false;

        auto pacing = stepper_;
        auto view = camera_;
        if (!read_saved_presentation(document, pacing, view, error))
            return false;

        if (interaction_.active)
            cancel_interaction();
        if (pending_edit_)
            commit_edit();
        begin_edit("Open arrangement");
        try
        {
            close_current_run(false);
            finish_shape_editor();
            if (!physics::populate_world(document, world_, error))
                throw std::runtime_error(error);
            scenario_document_ = std::make_shared<physics::ScenarioDocument>(std::move(document));
            experiment_content_ = parse_experiment_content(*scenario_document_);
            scenario_id_ = scenario_document_->metadata.id;
            setup_file_ = std::make_shared<SetupFileAssociation>();
            run_recorder_.set_experiment(scenario_id_);
            notifier_.clear_source("shape");
            scene_settings_.selection = {};
            selected_bodies_.clear();
            refresh_force_generators();
            stepper_ = pacing;
            stepper_.reset();
            stepper_.set_paused(true);
            single_step_pending_ = false;
            default_lab_ = capture_lab_settings();
            original_ = { world_.snapshot(), gravity_direction_degrees_ };
            apply_lab_settings(had_setup && keep_lab_settings_ ? carried_lab : default_lab_);
            setup_ = { world_.snapshot(), gravity_direction_degrees_ };
            rebuild_snapshot_views();
            setup_file_->saved_setup_fingerprint = setup_fingerprint();
            reset_measurements();
            synchronize_render_history();
            const auto* presentation = scenario_document_->root.find("presentation");
            if (scenario_document_->root.find("view") || (presentation && presentation->is_object() && presentation->find("view")))
                camera_ = view;
            else
                frame_subject();
            mark_edit_changed();
            commit_edit();
            if (!path.empty())
                note_setup_file(std::move(path), scenario_document_->metadata.title, experiment_content_->based_on, false, scenario_document_->root.find("guide") != nullptr);
            pending_setup_open_.reset();
            pending_leave_scenario_.clear();
            pending_quit_confirmation_ = false;
            speed_preview_start_.reset();
            ++departure_serial_;
            notify(ui::Severity::success, "Arrangement opened.", "content");
            return true;
        }
        catch (const std::exception& exception)
        {
            cancel_edit();
            error = exception.what();
            return false;
        }
    }

    bool SimulationSession::export_shape(std::string& text, std::string& error) const
    {
        std::shared_ptr<const physics::AuthoredShape> shape;
        const physics::ShapeDocument* source = nullptr;
        std::string title;
        if (shape_editor_.active())
        {
            const auto result = shape_editor_.build();
            if (!result.succeeded())
            {
                error = "Finish a valid outline before exporting the draft.";
                return false;
            }
            shape = result.shape;
            title = "Authored shape";
            if (const auto* body = world_.find_body(edited_shape_body_))
            {
                const auto parts = physics::authored_parts(*body);
                if (edited_part_index_ < parts.size())
                {
                    source = parts[edited_part_index_]->source_document.get();
                    title = parts[edited_part_index_]->name.empty() ? body->name() : parts[edited_part_index_]->name;
                }
            }
        }
        else if (const auto* body = world_.find_body(selection()))
        {
            const auto parts = physics::authored_parts(*body);
            if (!parts.empty())
            {
                const auto& part = parts[std::min(authored_part_index_, parts.size() - 1)];
                shape = part->shape;
                title = part->name.empty() ? body->name() : part->name;
                source = part->source_document.get();
            }
        }
        if (!shape)
        {
            error = "Select an authored part or draw a valid outline to export a shape.";
            return false;
        }
        physics::ShapeDocument document;
        return physics::capture_shape_document(*shape, title, source ? source->summary : "An authored outline from the playground.", document, error, source) &&
            physics::write_shape_document(document, text, error);
    }

    bool SimulationSession::import_shape(std::string_view text, std::string& error)
    {
        physics::ShapeDocument document;
        if (!physics::parse_shape_document(text, document, error))
            return false;
        physics::AuthoredPartDefinition part;
        part.shape = document.shape;
        part.name = document.title;
        part.source_document = std::make_shared<physics::ShapeDocument>(document);
        physics::BodyDefinition body;
        body.name = document.title.empty() ? "Imported shape" : document.title;
        body.position_m = camera_.center_m();
        // Validate placement against this world's bounds before ending a draft or gesture.
        try
        {
            physics::World validation;
            validation.restore(world_.snapshot());
            physics::create_authored_body(validation, body, { part });
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
        if (interaction_.active)
            cancel_interaction();
        if (pending_edit_)
            commit_edit();
        begin_edit("Import shape");
        try
        {
            finish_shape_editor();
            const auto id = physics::create_authored_body(world_, body, { part });
            set_selection(id);
            authored_part_index_ = 0;
            if (setup_.world.is_valid())
            {
                if (world_.statistics().elapsed_time_s <= 0.0)
                {
                    setup_ = { world_.snapshot(), gravity_direction_degrees_ };
                    rebuild_snapshot_views();
                }
                else
                    merge_live_structure_into_setup();
            }
            const auto was_running = !stepper_.is_paused();
            if (was_running)
            {
                stepper_.set_paused(true);
                pause_reason_ = { ui::PauseReason::new_object, 0 };
            }
            single_step_pending_ = false;
            stepper_.cancel_single_step_request();
            mark_edit_changed();
            commit_edit();
            notify(ui::Severity::success, "Shape imported at the centre of the view.", "content");
            return true;
        }
        catch (const std::exception& exception)
        {
            cancel_edit();
            error = exception.what();
            return false;
        }
    }
}
