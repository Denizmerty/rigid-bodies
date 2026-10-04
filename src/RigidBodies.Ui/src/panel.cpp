#include <rigidbodies/ui/panel.hpp>
#include <rigidbodies/ui/icons.hpp>

#include <algorithm>
#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <charconv>
#include <cmath>
#include <optional>

namespace rigidbodies::ui
{
    namespace
    {

        constexpr double row_padding = 4.0;
        constexpr double horizontal_padding = 10.0;

    } // namespace

    double Rect::width() const
    {
        return maximum.x - minimum.x;
    }

    double Rect::height() const
    {
        return maximum.y - minimum.y;
    }

    bool Rect::contains(const Vec2& point) const
    {
        return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y && point.y <= maximum.y;
    }

    Rect Rect::inset(double amount) const
    {
        return { { minimum.x + amount, minimum.y + amount }, { maximum.x - amount, maximum.y - amount } };
    }

    PanelBuilder::PanelBuilder(const Theme& theme, const render::RenderDevice& device, float scale, const Rect& bounds, DrawList& list, std::vector<Hotspot>& hotspots, const ViewState* view) : theme_(theme), device_(device), scale_(scale), bounds_(bounds), list_(list), hotspots_(hotspots), cursor_y_(bounds.minimum.y), view_(view)
    {
    }

    void PanelBuilder::set_pointer(const Vec2& pointer_px)
    {
        pointer_px_ = pointer_px;
    }

    void PanelBuilder::record_rows(std::vector<PanelRow>& rows)
    {
        rows_ = &rows;
    }

    bool PanelBuilder::view_present() const
    {
        return view_ && view_->present().mode;
    }
    bool PanelBuilder::view_present_locked() const
    {
        return view_ && view_->present().lock;
    }
    bool PanelBuilder::view_present_spotlight() const
    {
        return view_ && view_->present().spotlight;
    }
    bool PanelBuilder::view_pick_surface_armed() const
    {
        return view_ && view_->pick_surface_armed();
    }
    std::string_view PanelBuilder::view_next_hint() const
    {
        return view_ ? view_->next_hint() : std::string_view {};
    }
    bool PanelBuilder::view_hint_dismissed(std::string_view id) const
    {
        return view_ && view_->hint_dismissed(id);
    }
    bool PanelBuilder::view_surface_open(std::string_view id, bool fallback) const
    {
        return view_ && view_->surface_open(id, fallback);
    }
    bool PanelBuilder::view_sheet_open(std::string_view id) const
    {
        return view_ && view_->sheet_open(id);
    }
    bool PanelBuilder::view_transient_open(std::string_view id) const
    {
        return view_ && view_->transient_open(id);
    }
    math::Span<const std::string> PanelBuilder::view_search_recent() const
    {
        return view_ ? math::Span<const std::string>(view_->search_recent()) : math::Span<const std::string> {};
    }

    double PanelBuilder::line_height() const
    {
        return static_cast<double>(device_.text_line_height(scale_));
    }

    Rect PanelBuilder::next_row(double height)
    {
        Rect row { { bounds_.minimum.x, cursor_y_ }, { bounds_.maximum.x, cursor_y_ + height } };
        cursor_y_ += height;
        return row;
    }

    double PanelBuilder::cursor_y() const
    {
        return cursor_y_;
    }

    double PanelBuilder::remaining_height() const
    {
        return std::max(bounds_.maximum.y - cursor_y_, 0.0);
    }

    void PanelBuilder::title(std::string_view text)
    {
        if (rows_)
        {
            rows_->emplace_back(PanelRowKind::title, text);
            return;
        }
        const auto row = next_row(line_height() * 1.6 + row_padding);
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y + row_padding }, text, theme_.panel_title, scale_ * 1.35f);
        list_.add_line({ row.minimum.x + horizontal_padding, row.maximum.y - 2.0 }, { row.maximum.x - horizontal_padding, row.maximum.y - 2.0 }, theme_.panel_accent.with_alpha(0.5f));
    }

    void PanelBuilder::heading(std::string_view text)
    {
        if (rows_)
        {
            rows_->emplace_back(PanelRowKind::heading, text);
            rows_->back().group = group_;
            return;
        }
        spacer(4.0);
        const auto row = next_row(line_height() + row_padding);
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y + row_padding * 0.5 }, text, theme_.panel_accent, scale_);
    }

    void PanelBuilder::label(std::string_view text)
    {
        if (rows_)
        {
            rows_->emplace_back(PanelRowKind::label, text);
            rows_->back().group = group_;
            return;
        }
        const auto row = next_row(line_height() + row_padding);
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y + row_padding * 0.5 }, text, theme_.panel_text, scale_);
    }

    void PanelBuilder::paragraph(std::string_view text)
    {
        if (rows_)
        {
            rows_->emplace_back(PanelRowKind::paragraph, text);
            rows_->back().group = group_;
            return;
        }
        // The debug font is fixed width, so the number of characters that fit is exact rather than
        // an estimate, and the text is wrapped on word boundaries.
        const auto available = bounds_.width() - 2.0 * horizontal_padding;
        const auto character_width = static_cast<double>(device_.measure_text_width("M", scale_));
        const auto characters_per_line = character_width > 0.0 ? static_cast<std::size_t>(available / character_width) : text.size();
        if (characters_per_line == 0)
        {
            return;
        }

        std::vector<std::string> lines;
        std::size_t position = 0;
        while (position < text.size())
        {
            auto remaining = text.size() - position;
            auto take = std::min(characters_per_line, remaining);
            if (take < remaining)
            {
                const auto window = text.substr(position, take + 1);
                const auto space = window.find_last_of(' ');
                if (space != std::string_view::npos && space > 0)
                {
                    take = space;
                }
            }

            lines.emplace_back(text.substr(position, take));
            position += take;
            while (position < text.size() && text[position] == ' ')
            {
                ++position;
            }
        }

        const auto row_height = line_height() + row_padding;
        const auto capacity = row_height > 0.0 ? static_cast<std::size_t>(remaining_height() / row_height) : lines.size();
        if (capacity == 0)
            return;
        const auto visible = std::min(capacity, lines.size());
        if (visible < lines.size())
        {
            auto& last = lines[visible - 1];
            constexpr std::string_view ellipsis = "\xE2\x80\xA6";
            while (last.size() + ellipsis.size() + 1 > characters_per_line && !last.empty())
                last.pop_back();
            while (!last.empty() && last.back() == ' ')
                last.pop_back();
            if (!last.empty())
                last.push_back(' ');
            last += ellipsis;
        }
        for (std::size_t index = 0; index < visible; ++index)
            label(lines[index]);
    }

    void PanelBuilder::separator()
    {
        if (rows_)
        {
            rows_->emplace_back(PanelRowKind::separator);
            return;
        }
        const auto row = next_row(8.0);
        const auto middle = (row.minimum.y + row.maximum.y) * 0.5;
        list_.add_line({ row.minimum.x + horizontal_padding, middle }, { row.maximum.x - horizontal_padding, middle }, theme_.panel_border);
    }

    void PanelBuilder::spacer(double pixels)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::spacer };
            row.amount = pixels;
            rows_->push_back(std::move(row));
            return;
        }
        cursor_y_ += pixels;
    }

    void PanelBuilder::value_row(std::string_view name, std::string_view value)
    {
        value_row(name, value, theme_.panel_title);
    }

    void PanelBuilder::value_row(std::string_view name, std::string_view value, const Color& value_color)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::readout, name };
            row.value = value;
            row.group = group_;
            rows_->push_back(std::move(row));
            return;
        }
        const auto row = next_row(line_height() + row_padding);
        const auto baseline = row.minimum.y + row_padding * 0.5;
        list_.add_text({ row.minimum.x + horizontal_padding, baseline }, name, theme_.panel_muted, scale_);

        // The value is right-aligned so that a column of figures can be compared at a glance.
        const auto value_width = static_cast<double>(device_.measure_text_width(value, scale_));
        list_.add_text({ row.maximum.x - horizontal_padding - value_width, baseline }, value, value_color, scale_);
    }

    void PanelBuilder::live_value_row(std::string_view name, std::string_view value)
    {
        live_value_row(name, value, theme_.panel_title);
    }

    void PanelBuilder::live_value_row(std::string_view name, std::string_view value, const Color& value_color)
    {
        value_row(name, value, value_color);
        if (rows_ && !rows_->empty())
            rows_->back().live = true;
    }

    void PanelBuilder::draw_row_background(const Rect& row, bool hovered, bool active)
    {
        if (active)
        {
            list_.add_rectangle_fill(row.minimum, row.maximum, theme_.panel_accent.with_alpha(0.28f));
        }
        else if (hovered)
        {
            list_.add_rectangle_fill(row.minimum, row.maximum, theme_.panel_accent.with_alpha(0.14f));
        }
    }

    bool PanelBuilder::action_row(std::string_view text, const UiCommand& command, std::string_view disabled_reason)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::action, text };
            row.command = command;
            row.key = control_key(command);
            row.instance = command.id;
            row.disabled_reason = std::string(disabled_reason);
            row.group = group_;
            rows_->push_back(std::move(row));
            return false;
        }
        const auto row = next_row(line_height() + row_padding * 2.0);
        const auto hovered = row.contains(pointer_px_);
        draw_row_background(row, hovered, false);
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y + row_padding }, text, hovered ? theme_.panel_title : theme_.panel_text, scale_);
        if (disabled_reason.empty())
            hotspots_.push_back({ row, command });
        return hovered;
    }

    bool PanelBuilder::action_row(std::string_view key, std::string_view text, const UiCommand& command, std::string_view disabled_reason)
    {
        const auto hovered = action_row(text, command, disabled_reason);
        if (rows_ && !rows_->empty())
            rows_->back().key = std::string(key);
        return hovered;
    }

    bool PanelBuilder::toggle_row(std::string_view text, bool enabled, const UiCommand& command)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::switch_control, text };
            row.command = command;
            row.key = control_key(command);
            row.instance = command.id;
            row.selected = enabled;
            row.group = group_;
            rows_->push_back(std::move(row));
            return false;
        }
        const auto row = next_row(line_height() + row_padding * 2.0);
        const auto hovered = row.contains(pointer_px_);
        draw_row_background(row, hovered, false);

        const auto box_size = line_height();
        const Vec2 box_minimum { row.minimum.x + horizontal_padding, row.minimum.y + row_padding };
        const Vec2 box_maximum { box_minimum.x + box_size, box_minimum.y + box_size };
        list_.add_rectangle_outline(box_minimum, box_maximum, enabled ? theme_.panel_accent : theme_.panel_border, 1.5f);
        if (enabled)
        {
            list_.add_rectangle_fill({ box_minimum.x + 3.0, box_minimum.y + 3.0 }, { box_maximum.x - 3.0, box_maximum.y - 3.0 }, theme_.panel_accent);
        }

        list_.add_text({ box_maximum.x + horizontal_padding * 0.8, row.minimum.y + row_padding }, text, enabled ? theme_.panel_title : theme_.panel_text, scale_);
        hotspots_.push_back({ row, command });
        return hovered;
    }

    bool PanelBuilder::toggle_row(std::string_view key, std::string_view text, bool enabled, const UiCommand& command)
    {
        const auto hovered = toggle_row(text, enabled, command);
        if (rows_ && !rows_->empty())
            rows_->back().key = std::string(key);
        return hovered;
    }

    bool PanelBuilder::choice_row(std::string_view text, bool selected, const UiCommand& command)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::action, text };
            // A choice reads as an option: a ring that fills when chosen, never a text prefix.
            row.presentation = presentation(selected ? icons::check_circle : icons::ball);
            row.group = group_.empty() ? std::string { "choices" } : group_;
            row.command = command;
            row.key = control_key(command);
            row.instance = command.id;
            row.selected = selected;
            rows_->push_back(std::move(row));
            return false;
        }
        const auto row = next_row(line_height() + row_padding * 2.0);
        const auto hovered = row.contains(pointer_px_);
        draw_row_background(row, hovered, selected);

        if (selected)
        {
            list_.add_rectangle_fill({ row.minimum.x, row.minimum.y }, { row.minimum.x + 3.0, row.maximum.y }, theme_.panel_accent);
        }
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y + row_padding }, text, selected ? theme_.panel_title : theme_.panel_text, scale_);
        hotspots_.push_back({ row, command });
        return hovered;
    }

    bool PanelBuilder::choice_row(std::string_view key, std::string_view text, bool selected, const UiCommand& command)
    {
        const auto hovered = choice_row(text, selected, command);
        if (rows_ && !rows_->empty())
            rows_->back().key = std::string(key);
        return hovered;
    }

    void PanelBuilder::meter_row(std::string_view name, double fraction, const Color& color, std::string_view value_text)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::meter, name };
            row.amount = fraction;
            row.value = value_text.empty() ? core::format_value(math::clamp(fraction, 0.0, 1.0), core::DisplayQuantity::coefficient, core::DisplayUnits::si, 2) : std::string(value_text);
            row.hint = core::format_quantity(math::clamp(fraction, 0.0, 1.0) * 100.0, core::DisplayQuantity::percentage, core::DisplayUnits::si);
            row.group = group_;
            rows_->push_back(std::move(row));
            return;
        }
        const auto row = next_row(line_height() + row_padding * 2.5);
        list_.add_text({ row.minimum.x + horizontal_padding, row.minimum.y }, name, theme_.panel_muted, scale_);
        const auto displayed = value_text.empty() ? core::format_value(math::clamp(fraction, 0.0, 1.0), core::DisplayQuantity::coefficient, core::DisplayUnits::si, 2) : std::string(value_text);
        const auto displayed_width = device_.measure_text_width(displayed, scale_);
        list_.add_text({ row.maximum.x - horizontal_padding - displayed_width, row.minimum.y }, displayed, theme_.panel_text, scale_);

        const auto bar_top = row.minimum.y + line_height() + 2.0;
        const auto bar_bottom = bar_top + 5.0;
        const Vec2 track_minimum { row.minimum.x + horizontal_padding, bar_top };
        const Vec2 track_maximum { row.maximum.x - horizontal_padding, bar_bottom };
        list_.add_rectangle_fill(track_minimum, track_maximum, theme_.panel_border.with_alpha(0.6f));

        const auto clamped = math::clamp(fraction, 0.0, 1.0);
        if (clamped > 0.0)
        {
            list_.add_rectangle_fill(track_minimum, { track_minimum.x + (track_maximum.x - track_minimum.x) * clamped, track_maximum.y }, color);
        }
    }

    void PanelBuilder::share_row(std::string_view name, double fraction, std::string_view value_text, std::uint8_t series)
    {
        if (rows_)
        {
            PanelRow row { PanelRowKind::meter, name };
            row.amount = math::clamp(fraction, 0.0, 1.0);
            row.value = std::string(value_text);
            row.hint = core::format_quantity(row.amount * 100.0, core::DisplayQuantity::percentage, core::DisplayUnits::si);
            row.series = series;
            row.live = true;
            row.group = group_;
            rows_->push_back(std::move(row));
            return;
        }
        static constexpr Color series_colours[] { { 0.31f, 0.58f, 0.94f, 1.0f }, { 0.88f, 0.44f, 0.18f, 1.0f }, { 0.12f, 0.68f, 0.49f, 1.0f } };
        meter_row(name, fraction, series_colours[(std::max<std::uint8_t>(series, 1) - 1) % 3], value_text);
    }

    void PanelBuilder::readout(std::string_view key, std::string_view label, std::string_view text, RowTone tone, bool live, std::string_view instance)
    {
        if (!rows_)
        {
            live ? live_value_row(label, text) : value_row(label, text);
            return;
        }
        PanelRow row { PanelRowKind::readout, label };
        row.key = key;
        row.instance = instance;
        row.value = text;
        row.tone = tone;
        row.live = live;
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    namespace
    {
        std::optional<double> option_number(std::string_view text)
        {
            double value = 0.0;
            const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
            if (error != std::errc {} || end != text.data() + text.size() || !std::isfinite(value))
                return std::nullopt;
            return value;
        }

        // Numeric choices (time steps, speeds) are matched by value, since a current value printed
        // to a few decimals never spells an option id such as 0.0083333333 exactly. A value that is
        // no option selects the specification's own Custom entry when it has one.
        std::string matching_option(const ControlSpec& spec, std::string_view current)
        {
            for (const auto& option : spec.options)
                if (option.id == current)
                    return std::string(current);
            if (const auto value = option_number(current))
            {
                const auto tolerance = spec.key == "bar.speed.choice" ? 1.0e-10 : 1.0e-4;
                for (const auto& option : spec.options)
                    if (const auto candidate = option_number(option.id); candidate && std::abs(*candidate - *value) <= tolerance * std::max(std::abs(*candidate), 1.0e-9))
                        return std::string(option.id);
                for (const auto& option : spec.options)
                    if (option.id == "custom")
                        return std::string(option.id);
            }
            return std::string(current);
        }

        PanelRow control_row(PanelRowKind kind, const ControlSpec& spec, const UiCommand& command, std::string_view instance, std::string_view selected = {})
        {
            PanelRow row { kind, spec.label };
            row.key = spec.key;
            row.instance = instance;
            row.spec = std::make_shared<ControlSpec>(spec);
            row.command = command;
            row.selected_option = selected;
            return row;
        }
    }

    void PanelBuilder::number_row(const ControlSpec& spec, double value_si, const UiCommand& command_template, std::string_view instance)
    {
        if (!rows_)
        {
            auto command = command_template;
            command.value = value_si;
            action_row(std::string(spec.label) + ": " + core::format_quantity(value_si, spec.number.quantity, core::DisplayUnits::si) + "  [-] [+]", command);
            return;
        }
        auto row = control_row(PanelRowKind::number, spec, command_template, instance);
        row.number_si = value_si;
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::stepper_row(const ControlSpec& spec, double value_si, const UiCommand& command_template)
    {
        if (!rows_)
        {
            auto command = command_template;
            command.value = value_si;
            action_row(std::string(spec.label) + ": " + core::fixed(value_si, 2) + "  [-] [+]", command);
            return;
        }
        auto row = control_row(PanelRowKind::stepper, spec, command_template, {});
        row.number_si = value_si;
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::mixed_number_row(const ControlSpec& spec, const UiCommand& command_template)
    {
        if (!rows_)
        {
            action_row(std::string(spec.label) + ": Mixed", command_template);
            return;
        }
        auto row = control_row(PanelRowKind::number, spec, command_template, {});
        row.mixed = true;
        row.value = "Mixed";
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::select_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template, std::string_view unlisted_label)
    {
        if (!rows_)
        {
            action_row(std::string(spec.label) + ": " + std::string(current), command_template);
            return;
        }
        auto row = control_row(PanelRowKind::select, spec, command_template, {}, matching_option(spec, current));
        // Keep a custom value visible while leaving the Custom… action independently selectable.
        if (row.selected_option == "custom" && current != "custom" && !unlisted_label.empty())
            row.selected_option = std::string(current);
        const auto listed = std::any_of(spec.options.begin(), spec.options.end(), [&](const OptionSpec& option)
            {
                return option.id == row.selected_option;
            });
        if (!listed && !current.empty())
            row.value = unlisted_label.empty() ? std::string("Custom") : std::string(unlisted_label);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::mixed_select_row(const ControlSpec& spec, const UiCommand& command_template)
    {
        if (!rows_)
        {
            action_row(std::string(spec.label) + ": Mixed", command_template);
            return;
        }
        auto row = control_row(PanelRowKind::select, spec, command_template, {});
        row.mixed = true;
        row.value = "Mixed";
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::radio_list_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template)
    {
        if (!rows_)
        {
            action_row(std::string(spec.label) + ": " + std::string(current), command_template);
            return;
        }
        auto row = control_row(PanelRowKind::radio_list, spec, command_template, {}, current);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::segmented_row(const ControlSpec& spec, std::string_view current, const UiCommand& command_template)
    {
        if (!rows_)
        {
            action_row(std::string(spec.label) + ": " + std::string(current), command_template);
            return;
        }
        auto row = control_row(PanelRowKind::segmented, spec, command_template, {}, current);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::switch_row(const ControlSpec& spec, bool on, const UiCommand& command)
    {
        if (!rows_)
        {
            toggle_row(spec.label, on, command);
            return;
        }
        auto row = control_row(PanelRowKind::switch_control, spec, command, {});
        row.selected = on;
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::checkbox_row(const ControlSpec& spec, bool on, const UiCommand& command)
    {
        if (!rows_)
        {
            toggle_row(spec.label, on, command);
            return;
        }
        auto row = control_row(PanelRowKind::checkbox, spec, command, {});
        row.selected = on;
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    std::vector<std::string> PanelBuilder::checklist(const ControlSpec& spec, math::Span<const std::string_view> default_ids)
    {
        auto selected = view_ ? view_->checklist(spec.key, default_ids) : std::vector<std::string> { default_ids.begin(), default_ids.end() };
        if (rows_)
        {
            auto row = control_row(PanelRowKind::checklist, spec, {}, {});
            row.options.assign(spec.options.begin(), spec.options.end());
            row.selected_option = selected.empty() ? std::string {} : selected.front();
            row.selected_options = selected;
            row.group = group_;
            rows_->push_back(std::move(row));
        }
        return selected;
    }

    std::string PanelBuilder::tabs(std::string_view key, math::Span<const OptionSpec> tab_options, std::string_view default_tab)
    {
        std::vector<std::string_view> ids;
        ids.reserve(tab_options.size());
        for (const auto& option : tab_options)
            ids.push_back(option.id);
        const auto active = view_ ? view_->active_tab(key, ids, default_tab) : default_tab;
        if (rows_)
        {
            PanelRow row { PanelRowKind::tabs, key };
            row.key = key;
            row.selected_option = active;
            row.options.assign(tab_options.begin(), tab_options.end());
            row.group = group_;
            rows_->push_back(std::move(row));
            return rows_->back().selected_option;
        }
        return std::string(active);
    }

    bool PanelBuilder::section(std::string_view key, std::string_view title, bool default_open)
    {
        const auto open = view_ ? view_->section_open(key, default_open) : default_open;
        if (rows_)
        {
            PanelRow row { PanelRowKind::section, title };
            row.key = key;
            row.selected = open;
            row.group = group_;
            rows_->push_back(std::move(row));
        }
        return open;
    }

    void PanelBuilder::notice(std::string_view key, Severity severity, std::string_view text, const std::optional<NotificationAction>& primary, const std::optional<NotificationAction>& secondary)
    {
        if (!rows_)
        {
            label(text);
            return;
        }
        PanelRow row { PanelRowKind::notice, text };
        row.key = key;
        row.tone = severity == Severity::error ? RowTone::danger : severity == Severity::warning ? RowTone::warning
            : severity == Severity::success                                                      ? RowTone::positive
                                                                                                 : RowTone::info;
        row.primary_action = primary;
        row.secondary_action = primary ? secondary : std::optional<NotificationAction> {};
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::list_item(std::string_view key, std::string_view instance, const ListItemContent& content,
        const UiCommand& command, const UiCommand& alternate, std::string_view disabled_reason, const UiCommand& shift,
        std::string_view tooltip)
    {
        if (!rows_)
            return;
        PanelRow row { PanelRowKind::list_item, content.title };
        row.key = key;
        row.instance = instance;
        row.command = command;
        row.alternate_command = alternate;
        row.shift_command = shift;
        row.list_content = content;
        row.hint = content.secondary;
        row.tooltip = std::string(tooltip);
        row.disabled_reason = std::string(disabled_reason);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::text_field(std::string_view key, std::string_view label, std::string_view value, std::string_view placeholder, std::string_view view_key)
    {
        if (!rows_)
            return;
        PanelRow row { PanelRowKind::text_field, label };
        row.key = std::string(key);
        row.view_key = std::string(view_key.empty() ? key : view_key);
        row.value = std::string(value);
        row.hint = std::string(placeholder);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::plot(std::string_view key, const PlotData& data)
    {
        if (!rows_)
            return;
        PanelRow row { PanelRowKind::plot };
        row.key = key;
        row.plot = std::make_shared<const PlotData>(data);
        row.group = group_;
        rows_->push_back(std::move(row));
    }

    void PanelBuilder::begin_group(std::string_view group)
    {
        group_ = group;
    }

    void PanelBuilder::end_group()
    {
        group_.clear();
    }

    bool PanelBuilder::view_region_present(RegionId id) const
    {
        return layout_ && layout_->find(id) != nullptr;
    }

    double PanelBuilder::view_region_width(RegionId id) const
    {
        const auto* region = layout_ ? layout_->find(id) : nullptr;
        return region && scale_ > 0.0f ? region->bounds.width() / static_cast<double>(scale_) : 0.0;
    }

    bool PanelBuilder::view_region_under_library(RegionId id) const
    {
        if (!layout_ || !view_sheet_open("library"))
            return false;
        const auto* region = layout_->find(id);
        const auto* library = layout_->find(RegionId::library_sheet);
        return region && library && region->bounds.minimum.x < library->bounds.maximum.x && region->bounds.maximum.x > library->bounds.minimum.x && region->bounds.minimum.y < library->bounds.maximum.y && region->bounds.maximum.y > library->bounds.minimum.y;
    }

    void PanelBuilder::present_last(const RowPresentation& presentation)
    {
        if (rows_ && !rows_->empty())
            rows_->back().presentation = presentation;
    }

    void PanelBuilder::label_last(std::string_view text)
    {
        if (rows_ && !rows_->empty())
            rows_->back().text = std::string(text);
    }

    void PanelBuilder::instance_last(std::string_view instance)
    {
        if (rows_ && !rows_->empty())
            rows_->back().instance = std::string(instance);
    }

    void PanelBuilder::select_last(bool selected)
    {
        if (rows_ && !rows_->empty())
            rows_->back().selected = selected;
    }

    void PanelBuilder::disable_last(std::string_view reason)
    {
        if (rows_ && !rows_->empty())
            rows_->back().disabled_reason = std::string(reason);
    }

    bool PanelBuilder::view_section_open(std::string_view key, bool fallback) const
    {
        return view_ ? view_->section_open(key, fallback) : fallback;
    }

    std::string_view PanelBuilder::view_value(std::string_view key, std::string_view fallback) const
    {
        return view_ ? view_->value(key, fallback) : fallback;
    }

    std::vector<std::string> PanelBuilder::view_checklist(std::string_view key, math::Span<const std::string_view> default_ids) const
    {
        return view_ ? view_->checklist(key, default_ids) : std::vector<std::string> { default_ids.begin(), default_ids.end() };
    }

    bool PanelBuilder::view_visited(std::string_view experiment) const
    {
        return view_ && view_->visited(experiment);
    }

    math::Span<const SetupFileInfo> PanelBuilder::view_setup_files() const
    {
        return view_ ? math::Span<const SetupFileInfo>(view_->setup_files()) : math::Span<const SetupFileInfo> {};
    }

} // namespace rigidbodies::ui
