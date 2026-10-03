#pragma once

#include <rigidbodies/render/draw_list.hpp>
#include <rigidbodies/render/render_device.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/geometry.hpp>
#include <rigidbodies/ui/layout.hpp>
#include <rigidbodies/ui/notifications.hpp>
#include <rigidbodies/ui/plot.hpp>
#include <rigidbodies/ui/ui_command.hpp>
#include <rigidbodies/ui/ui_model.hpp>
#include <rigidbodies/ui/view_state.hpp>

#include <optional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::ui
{

    using math::Vec2;
    using render::Color;
    using render::DrawList;
    using render::Theme;

    // A clickable region recorded during panel drawing and checked on the next pointer press.
    // Drawing and hit testing use the same bounds.
    struct Hotspot
    {
        Rect bounds;
        UiCommand command;
    };

    enum class PanelRowKind
    {
        title,
        heading,
        label,
        paragraph,
        separator,
        spacer,
        action,
        meter,
        number,
        stepper,
        select,
        radio_list,
        checklist,
        segmented,
        switch_control,
        checkbox,
        tabs,
        section,
        notice,
        readout,
        list_item,
        text_field,
        plot
    };

    struct ListItemAction
    {
        std::string label, icon;
        UiCommand command;
        bool primary { false };
    };

    struct ListItemContent
    {
        std::string title, secondary, icon;
        std::vector<std::string> badges;
        // Buttons inside the item, for an item that opens into its own detail: they belong to it
        // and sit within its highlight rather than beneath it.
        std::vector<ListItemAction> actions;
    };

    enum class RowEmphasis : std::uint8_t
    {
        normal,
        primary,
        quiet,
        danger
    };

    // How a row is presented by a document backend, separate from what it means. Toolbars and
    // strips shed detail in a declared order when space runs out: a positive label_step hides the
    // visible label at that fitting step (the label stays the accessible name and tooltip), a
    // positive shrink_step lets a title give up width from that step on, shortened with an
    // ellipsis (the full text becomes its tooltip), and a positive overflow_step moves the whole
    // row into the strip's overflow menu at that step. A negative overflow_step keeps the row in
    // the overflow menu at every width.
    struct RowPresentation
    {
        std::string icon, shortcut, badge;
        // What the badge counts, for its tooltip and accessible name; a bare number is ambiguous.
        std::string badge_description;
        RowEmphasis emphasis { RowEmphasis::normal };
        int label_step { 0 }, shrink_step { 0 }, overflow_step { 0 };
        bool icon_only { false }, header { false };
        // The icon follows the label, for an action that points onward such as Next.
        bool icon_after { false };

        RowPresentation& with_icon(std::string_view name)
        {
            icon = std::string(name);
            return *this;
        }
        RowPresentation& with_shortcut(std::string_view chord)
        {
            shortcut = std::string(chord);
            return *this;
        }
        RowPresentation& with_badge(std::string_view text, std::string_view description = {})
        {
            badge = std::string(text);
            badge_description = std::string(description);
            return *this;
        }
        RowPresentation& primary()
        {
            emphasis = RowEmphasis::primary;
            return *this;
        }
        RowPresentation& quiet()
        {
            emphasis = RowEmphasis::quiet;
            return *this;
        }
        RowPresentation& danger()
        {
            emphasis = RowEmphasis::danger;
            return *this;
        }
        RowPresentation& hide_label_at(int step)
        {
            label_step = step;
            return *this;
        }
        RowPresentation& shrink_at(int step)
        {
            shrink_step = step;
            return *this;
        }
        RowPresentation& overflow_at(int step)
        {
            overflow_step = step;
            return *this;
        }
        RowPresentation& always_overflow()
        {
            overflow_step = -1;
            return *this;
        }
        RowPresentation& icon_label_only()
        {
            icon_only = true;
            return *this;
        }
        RowPresentation& in_header()
        {
            header = true;
            return *this;
        }
        RowPresentation& icon_trailing()
        {
            icon_after = true;
            return *this;
        }
    };

    [[nodiscard]] inline RowPresentation presentation(std::string_view icon = {}, std::string_view shortcut = {})
    {
        RowPresentation result;
        result.icon = std::string(icon);
        result.shortcut = std::string(shortcut);
        return result;
    }
    // A backend-neutral semantic description, including the original command payload. The
    // document backend owns text measurement, wrapping, focus, scrolling, and hit testing.
    struct PanelRow
    {
        PanelRowKind kind;
        std::string key, instance;
        // Rows outlive Panel::build in the document backend. Own a copy so controls assembled
        // from scenario/run-specific options never leave a dangling specification pointer.
        std::shared_ptr<const ControlSpec> spec;
        std::string text, value;
        std::string hint, tooltip, disabled_reason;
        RowTone tone { RowTone::normal };
        double number_si { 0.0 };
        std::string selected_option;
        std::vector<std::string> selected_options;
        std::vector<OptionSpec> options;
        std::string group;
        std::shared_ptr<const PlotData> plot;
        UiCommand command;
        UiCommand alternate_command;
        UiCommand shift_command;
        std::optional<NotificationAction> primary_action, secondary_action;
        std::optional<ListItemContent> list_content;
        RowPresentation presentation;
        bool selected { false };
        bool live { false };
        // A control standing for several objects whose values differ: it shows no value of its
        // own, and an entry applies to all of them.
        bool mixed { false };
        double amount { 0.0 };
        // A meter drawn in plot series colour 1 to 3; zero keeps the accent colour.
        std::uint8_t series { 0 };
        explicit PanelRow(PanelRowKind row_kind, std::string_view label = {}) : kind(row_kind), text(label)
        {
        }
    };

    // Builds a panel from semantic rows and collects its clickable regions.
    //
    // The overlay draws these rows directly; the document backend uses them for its own layout.
    class PanelBuilder
    {
    public:
        PanelBuilder(const Theme& theme, const render::RenderDevice& device, float scale, const Rect& bounds, DrawList& list, std::vector<Hotspot>& hotspots, const ViewState* view = nullptr);

        void title(std::string_view text);
        void heading(std::string_view text);
        void label(std::string_view text);
        void paragraph(std::string_view text);
        void separator();
        void spacer(double pixels = 6.0);

        // A name on the left and a value on the right, the arrangement every read-out uses.
        void value_row(std::string_view name, std::string_view value);
        void value_row(std::string_view name, std::string_view value, const Color& value_color);
        void live_value_row(std::string_view name, std::string_view value);
        void live_value_row(std::string_view name, std::string_view value, const Color& value_color);

        // A row that can be clicked. Returns true when the pointer is over it, so that the caller
        // can show a hover state.
        bool action_row(std::string_view text, const UiCommand& command, std::string_view disabled_reason = {});
        bool action_row(std::string_view key, std::string_view text, const UiCommand& command, std::string_view disabled_reason = {});

        // A row showing an on or off state, which emits its command when clicked.
        bool toggle_row(std::string_view text, bool enabled, const UiCommand& command);
        bool toggle_row(std::string_view key, std::string_view text, bool enabled, const UiCommand& command);

        // A row marked as the current selection among several choices.
        bool choice_row(std::string_view text, bool selected, const UiCommand& command);
        bool choice_row(std::string_view key, std::string_view text, bool selected, const UiCommand& command);

        // A horizontal bar filled to a fraction, used for figures that are easier to compare than
        // to read.
        // The bar shows `fraction` (0 to 1); `value_text`, when given, is the figure printed beside
        // it, for quantities whose bar is drawn on another scale.
        void meter_row(std::string_view name, double fraction, const Color& color, std::string_view value_text = {});
        // A share of a whole, coloured like the plot series (1 to 3) of the quantity it stands
        // for. It refreshes with the live read-outs, so the bar and the figures beside it agree.
        void share_row(std::string_view name, double fraction, std::string_view value_text, std::uint8_t series);
        void readout(std::string_view key, std::string_view label, std::string_view text, RowTone tone = RowTone::normal, bool live = false, std::string_view instance = {});
        void number_row(const ControlSpec& spec, double value_si, const UiCommand& command_template, std::string_view instance = {});
        // A number control for a selection whose values differ: the field is empty and reads
        // Mixed until a value is entered for all of them.
        void mixed_number_row(const ControlSpec& spec, const UiCommand& command_template);
        // `unlisted_label` names a current value that is none of the specification's options
        // (an authored material, say), so the control never claims the first option instead.
        void select_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template, std::string_view unlisted_label = {});
        void mixed_select_row(const ControlSpec& spec, const UiCommand& command_template);
        void segmented_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template);
        void switch_row(const ControlSpec& spec, bool on, const UiCommand& command);
        void stepper_row(const ControlSpec& spec, double value_si, const UiCommand& command_template);
        void radio_list_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template);
        void checkbox_row(const ControlSpec& spec, bool on, const UiCommand& command);
        [[nodiscard]] std::vector<std::string> checklist(const ControlSpec& spec, math::Span<const std::string_view> default_ids);
        [[nodiscard]] std::string tabs(std::string_view key, math::Span<const OptionSpec> tabs, std::string_view default_tab);
        [[nodiscard]] bool section(std::string_view key, std::string_view title, bool default_open);
        void notice(std::string_view key, Severity severity, std::string_view text, const std::optional<NotificationAction>& primary = {}, const std::optional<NotificationAction>& secondary = {});
        void list_item(std::string_view key, std::string_view instance, const ListItemContent& content,
            const UiCommand& command, const UiCommand& alternate = {}, std::string_view disabled_reason = {}, const UiCommand& shift = {},
            std::string_view tooltip = {});
        // `placeholder` is shown inside the empty field.
        void text_field(std::string_view key, std::string_view label, std::string_view value, std::string_view placeholder = {});
        void plot(std::string_view key, const PlotData& data);
        void begin_group(std::string_view group);
        void end_group();

        // Adds presentation detail to the most recently recorded row. Backends without document
        // presentation ignore it, so panels may call it unconditionally.
        void present_last(const RowPresentation& presentation);
        // Marks the most recently recorded row as selected, for toolbar toggles whose pressed
        // state mirrors an open surface.
        void select_last(bool selected = true);
        // Makes the most recently recorded control unavailable, naming why; it keeps showing its
        // value but takes no input.
        void disable_last(std::string_view reason);
        // Distinguishes a control that a panel shows twice, or that another visible panel also
        // shows, so each copy keeps its own document element.
        void instance_last(std::string_view instance);
        // Renames the most recently recorded row, for one control spec shown once per quantity.
        void label_last(std::string_view text);
        [[nodiscard]] bool view_section_open(std::string_view key, bool fallback = false) const;
        [[nodiscard]] std::string_view view_value(std::string_view key, std::string_view fallback = {}) const;
        // The checklist selection without recording its row, for a row that must come first.
        [[nodiscard]] std::vector<std::string> view_checklist(std::string_view key, math::Span<const std::string_view> default_ids) const;
        [[nodiscard]] bool view_visited(std::string_view experiment) const;
        [[nodiscard]] math::Span<const SetupFileInfo> view_setup_files() const;
        [[nodiscard]] bool view_present() const;
        [[nodiscard]] bool view_present_locked() const;
        [[nodiscard]] bool view_present_spotlight() const;
        [[nodiscard]] bool view_pick_surface_armed() const;
        [[nodiscard]] std::string_view view_next_hint() const;
        [[nodiscard]] bool view_hint_dismissed(std::string_view id) const;
        [[nodiscard]] bool view_surface_open(std::string_view id, bool fallback = false) const;
        [[nodiscard]] bool view_sheet_open(std::string_view id) const;
        [[nodiscard]] bool view_transient_open(std::string_view id) const;
        [[nodiscard]] math::Span<const std::string> view_search_recent() const;
        // Whether the current layout actually presents a region, as opposed to the surface merely
        // being requested open (a medium window docks only one side surface at a time).
        void set_layout(const LayoutResult* layout)
        {
            layout_ = layout;
        }
        [[nodiscard]] bool view_region_present(RegionId id) const;
        // Logical width of a presented region, or zero when the layout does not present it.
        [[nodiscard]] double view_region_width(RegionId id) const;
        // Whether an open Library sheet hides a docked region, so a toggle for that region shows
        // what the learner actually sees.
        [[nodiscard]] bool view_region_under_library(RegionId id) const;

        [[nodiscard]] double cursor_y() const;
        [[nodiscard]] double remaining_height() const;

        void set_pointer(const Vec2& pointer_px);
        void record_rows(std::vector<PanelRow>& rows);

    private:
        [[nodiscard]] double line_height() const;
        [[nodiscard]] Rect next_row(double height);
        void draw_row_background(const Rect& row, bool hovered, bool active);

        const Theme& theme_;
        const render::RenderDevice& device_;
        float scale_ { 1.0f };
        Rect bounds_;
        DrawList& list_;
        std::vector<Hotspot>& hotspots_;
        Vec2 pointer_px_ { -1.0, -1.0 };
        double cursor_y_ { 0.0 };
        std::vector<PanelRow>* rows_ { nullptr };
        const ViewState* view_ { nullptr };
        const LayoutResult* layout_ { nullptr };
        std::string group_;
    };

    // One region of the interface. Panels know what to show and what to ask for, and nothing about
    // how the surrounding layout is arranged or how the drawing is carried out.
    class Panel
    {
    public:
        Panel() = default;
        Panel(const Panel&) = default;
        Panel(Panel&&) = default;
        Panel& operator=(const Panel&) = default;
        Panel& operator=(Panel&&) = default;
        virtual ~Panel() = default;

        [[nodiscard]] virtual std::string_view id() const = 0;
        [[nodiscard]] virtual std::string_view title() const = 0;
        [[nodiscard]] virtual RegionId region() const = 0;

        virtual void build(const UiModel& model, PanelBuilder& builder) = 0;

        [[nodiscard]] bool is_visible() const
        {
            return visible_;
        }

        void set_visible(bool value)
        {
            visible_ = value;
        }

    private:
        bool visible_ { true };
    };

} // namespace rigidbodies::ui
