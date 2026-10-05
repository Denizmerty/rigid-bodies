#include <rigidbodies/app/application.hpp>
#include <rigidbodies/app/content_files.hpp>

#include <SDL3/SDL.h>
#include <mutex>

namespace rigidbodies::app
{
    // SDL may deliver the callback on a worker thread, including after Application has closed.
    // Its separately owned mailbox never holds a pointer to the session or window.
    struct ContentFileDialog
    {
        std::mutex mutex;
        bool ready { false };
        std::string path, error, text, default_location;
        std::optional<SetupSaveSnapshot> setup_save;
        std::optional<SetupDeparture> continuation;
        ui::UiCommandKind operation { ui::UiCommandKind::none };
        static void SDLCALL complete(void* userdata, const char* const* paths, int)
        {
            std::unique_ptr<std::shared_ptr<ContentFileDialog>> owner(static_cast<std::shared_ptr<ContentFileDialog>*>(userdata));
            auto& state = **owner;
            std::lock_guard<std::mutex> lock(state.mutex);
            if (!paths)
                state.error = SDL_GetError();
            else if (*paths)
                state.path = *paths;
            state.ready = true;
        }
    };

    bool Application::begin_content_dialog(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        const bool saving = command.kind == K::save_arrangement || command.kind == K::export_shape;
        const bool shape = command.kind == K::import_shape || command.kind == K::export_shape;
        if (!saving && command.kind != K::open_arrangement && command.kind != K::import_shape)
            return false;
        // A relativity experiment has no Newtonian objects to add a shape to: the session refuses
        // the command with its notice, and no file dialog opens.
        if (command.kind == K::import_shape && session_.relativity_active())
        {
            session_.apply(command);
            return true;
        }
        if (content_dialog_)
            return true;
        auto state = std::make_shared<ContentFileDialog>();
        state->operation = command.kind;
        if (command.kind == K::save_arrangement)
            state->setup_save = session_.capture_setup_save(state->error, command.detail, command.value >= 0.5, command.flag);
        if (saving && !(shape ? session_.export_shape(state->text, state->error) : state->setup_save.has_value()))
        {
            save_continuation_.reset();
            session_.notify(ui::Severity::error, state->error, "content");
            return true;
        }
        if (state->setup_save)
        {
            state->continuation = std::move(save_continuation_);
            save_continuation_.reset();
            interface_.request(ui::ViewRequest::close_top_surface);
        }
        session_.notify(ui::Severity::info, "Choose a file.", "content");
        paths_.ensure_user_data_root();
        auto folder = last_content_folder_;
        if (folder.empty() && !session_.current_setup_path().empty())
            folder = std::filesystem::u8path(session_.current_setup_path()).parent_path();
        std::error_code code;
        if (folder.empty() || !std::filesystem::is_directory(folder, code))
            folder = paths_.user_data_root();
        auto title = command.detail;
        if (shape && title.empty())
            if (const auto* selected = session_.world().find_body(session_.selection()))
                title = selected->name();
        state->default_location = (saving ? folder / std::filesystem::u8path(suggested_content_filename(title, shape)) : folder).u8string();
        content_dialog_ = state;
        static const SDL_DialogFileFilter scenario_filters[] { { "Rigid Bodies arrangement (JSON)", "json" }, { "All files", "*" } };
        static const SDL_DialogFileFilter shape_filters[] { { "Rigid Bodies shape (JSON)", "json" }, { "All files", "*" } };
        auto* owner = new std::shared_ptr<ContentFileDialog>(std::move(state));
        if (saving)
            SDL_ShowSaveFileDialog(ContentFileDialog::complete, owner, window_->native(), shape ? shape_filters : scenario_filters, 2, content_dialog_->default_location.c_str());
        else
            SDL_ShowOpenFileDialog(ContentFileDialog::complete, owner, window_->native(), shape ? shape_filters : scenario_filters, 2, content_dialog_->default_location.c_str(), false);
        return true;
    }

    void Application::finish_content_dialog()
    {
        if (!content_dialog_)
            return;
        auto state = content_dialog_;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if (!state->ready)
                return;
        }
        content_dialog_.reset();
        if (!state->error.empty())
        {
            session_.notify(ui::Severity::error, "File dialog failed: " + state->error, "content");
            return;
        }
        if (state->path.empty())
        {
            session_.notify(ui::Severity::info, "File operation cancelled.", "content");
            return;
        }
        using K = ui::UiCommandKind;
        const bool saving = state->operation == K::save_arrangement || state->operation == K::export_shape;
        const auto path = saving ? content_save_destination(std::filesystem::u8path(state->path), state->operation == K::export_shape) : std::filesystem::u8path(state->path);
        bool success = false;
        if (saving)
        {
            std::error_code code;
            if (state->setup_save && !session_.can_write_setup_save(*state->setup_save, path.u8string(), state->error))
                success = false;
            else if (path != std::filesystem::u8path(state->path) && std::filesystem::exists(path, code))
                state->error = "A file with the document extension already exists. Choose its full name to confirm replacing it.";
            else
                success = write_content_file(path, state->setup_save ? state->setup_save->text : state->text, state->error);
        }
        else if (read_content_file(path, state->text, state->error))
            success = state->operation == K::import_shape ? session_.import_shape(state->text, state->error) : session_.request_open_arrangement(std::move(state->text), state->error, path.u8string());
        if (!success)
        {
            session_.notify(ui::Severity::error, state->error, "content");
        }
        else if (state->operation == K::save_arrangement)
        {
            const auto current = session_.complete_setup_save(*state->setup_save, path.u8string());
            session_.notify(ui::Severity::success, "Saved " + path.filename().u8string() + (current ? "." : " as it was when the dialog opened."), "content");
            if (current && state->continuation)
                session_.complete_saved_departure(*state->continuation);
        }
        else if (saving)
            session_.notify(ui::Severity::success, "Saved " + path.filename().u8string() + ".", "content");
        if (success)
            last_content_folder_ = path.parent_path();
    }
}
