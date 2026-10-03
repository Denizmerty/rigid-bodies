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
        std::string path, error, text, default_location, title, based_on;
        bool current_moment { false }, include_guide { true };
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
        if (content_dialog_)
            return true;
        auto state = std::make_shared<ContentFileDialog>();
        state->operation = command.kind;
        state->title = command.detail;
        state->based_on = session_.scenario_id();
        state->current_moment = command.value >= 0.5;
        state->include_guide = command.flag;
        if (saving && !(shape ? session_.export_shape(state->text, state->error) : session_.save_arrangement(state->text, state->error, state->title, state->current_moment, state->include_guide)))
        {
            session_.notify(ui::Severity::error, state->error, "content");
            return true;
        }
        session_.notify(ui::Severity::info, "Choose a file.", "content");
        paths_.ensure_user_data_root();
        state->default_location = (saving ? paths_.user_data(shape ? "shape.rbshape.json" : "arrangement.rbscenario.json") : paths_.user_data_root()).u8string();
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
        const auto path = std::filesystem::u8path(state->path);
        const bool saving = state->operation == K::save_arrangement || state->operation == K::export_shape;
        bool success = false;
        if (saving)
            success = write_content_file(path, state->text, state->error);
        else if (read_content_file(path, state->text, state->error))
            success = state->operation == K::import_shape ? session_.import_shape(state->text, state->error) : session_.open_arrangement(state->text, state->error);
        if (!success)
            session_.notify(ui::Severity::error, state->error, "content");
        else if (state->operation == K::save_arrangement)
        {
            session_.note_setup_file(path.u8string(), state->title, state->based_on);
            session_.notify(ui::Severity::success, "Saved " + path.filename().u8string() + ".", "content");
        }
        else if (state->operation == K::open_arrangement)
        {
            const auto model = session_.build_model();
            const auto based_on = model.scenario_content ? model.scenario_content->based_on : std::string {};
            session_.note_setup_file(path.u8string(), model.scenario_title, based_on);
        }
        else if (saving)
            session_.notify(ui::Severity::success, "Saved " + path.filename().u8string() + ".", "content");
    }
}
