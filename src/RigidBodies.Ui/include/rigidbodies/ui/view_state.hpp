#pragma once

#include <rigidbodies/math/span.hpp>
#include <rigidbodies/ui/toast_presenter.hpp>
#include <rigidbodies/ui/ui_model.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::ui
{
    // Interface-owned state. It is deliberately absent from UiModel and session snapshots.
    class ViewState
    {
    public:
        [[nodiscard]] std::string_view active_tab(std::string_view key, math::Span<const std::string_view> available, std::string_view fallback) const;
        void set_active_tab(std::string_view key, std::string_view value);

        [[nodiscard]] bool section_open(std::string_view key, bool fallback) const;
        void set_section_open(std::string_view key, bool value);

        [[nodiscard]] std::vector<std::string> checklist(std::string_view key, math::Span<const std::string_view> defaults) const;
        void set_checklist(std::string_view key, std::vector<std::string> values);

        [[nodiscard]] bool surface_open(std::string_view key, bool fallback = false) const;
        void set_surface_open(std::string_view key, bool value);
        void toggle_surface(std::string_view key, bool fallback = false);
        [[nodiscard]] double number(std::string_view key, double fallback) const;
        void set_number(std::string_view key, double value);
        [[nodiscard]] std::string_view value(std::string_view key, std::string_view fallback = {}) const;
        void set_value(std::string_view key, std::string_view value);
        // Run comparison choices are session-only and scoped to the open experiment. The current
        // experiment is supplied before panels build so ordinary document controls can still use
        // the same value binding path.
        void set_experiment_context(std::string_view experiment);
        [[nodiscard]] bool visited(std::string_view experiment) const;
        void mark_visited(std::string_view experiment);
        void merge_setup_file(const SetupFileInfo& file);
        void remove_setup_file(std::string_view path);
        [[nodiscard]] const std::vector<SetupFileInfo>& setup_files() const
        {
            return setup_files_;
        }
        [[nodiscard]] const std::vector<std::string>& sheets() const
        {
            return sheets_;
        }
        [[nodiscard]] const std::vector<std::string>& transients() const
        {
            return transients_;
        }
        void open_sheet(std::string_view id);
        void close_sheet(std::string_view id);
        void open_transient(std::string_view id);
        void close_transient(std::string_view id);
        [[nodiscard]] bool sheet_open(std::string_view id) const;
        [[nodiscard]] bool transient_open(std::string_view id) const;

        struct PresentState
        {
            bool mode { false };
            bool lock { true };
            bool spotlight { false };
        };
        [[nodiscard]] PresentState& present()
        {
            return present_;
        }
        [[nodiscard]] const PresentState& present() const
        {
            return present_;
        }
        [[nodiscard]] bool pick_surface_armed() const
        {
            return pick_surface_armed_;
        }
        void set_pick_surface_armed(bool value)
        {
            pick_surface_armed_ = value;
        }
        [[nodiscard]] bool hint_dismissed(std::string_view id) const;
        void dismiss_hint(std::string_view id);
        void reset_hints();
        [[nodiscard]] std::string_view next_hint() const;
        void remember_search(std::string_view key);
        [[nodiscard]] const std::vector<std::string>& search_recent() const
        {
            return search_recent_;
        }

        [[nodiscard]] std::string serialize() const;
        // Returns diagnostics for malformed or unknown lines. A newer version leaves state intact.
        [[nodiscard]] std::vector<std::string> deserialize(std::string_view text);

        [[nodiscard]] ToastPresenter& toast_presenter()
        {
            return toast_presenter_;
        }
        [[nodiscard]] const ToastPresenter& toast_presenter() const
        {
            return toast_presenter_;
        }

    private:
        std::map<std::string, std::string, std::less<>> active_tabs_;
        std::map<std::string, bool, std::less<>> open_sections_;
        std::map<std::string, std::vector<std::string>, std::less<>> checklists_;
        std::map<std::string, bool, std::less<>> surfaces_;
        std::map<std::string, double, std::less<>> numbers_;
        std::map<std::string, std::string, std::less<>> values_;
        std::map<std::string, std::map<std::string, std::string, std::less<>>, std::less<>> experiment_values_;
        std::string experiment_context_;
        std::set<std::string, std::less<>> visited_;
        std::vector<SetupFileInfo> setup_files_;
        std::vector<std::string> sheets_, transients_;
        PresentState present_;
        bool pick_surface_armed_ { false };
        std::set<std::string, std::less<>> dismissed_hints_;
        std::vector<std::string> search_recent_;
        ToastPresenter toast_presenter_;
    };

    enum class ViewRequest : std::uint8_t
    {
        open_shortcuts,
        open_library,
        toggle_show,
        toggle_measure,
        open_world,
        toggle_guide,
        toggle_inspector_pin,
        open_main_menu,
        open_preferences,
        open_save_details,
        open_add_menu,
        open_about,
        toggle_present,
        open_command_search,
        open_context_menu,
        toggle_performance_overlay,
        arm_pick_surface,
        close_top_surface
    };
}
