#include <rigidbodies/ui/overlay_backend.hpp>

#include <rigidbodies/core/text_format.hpp>

#include <algorithm>
#include <charconv>

namespace rigidbodies::ui
{
    namespace
    {
        std::string row_text(const PanelRow& row, core::DisplayUnits units)
        {
            switch (row.kind)
            {
            case PanelRowKind::title:
                return "# " + row.text;
            case PanelRowKind::heading:
                return "## " + row.text;
            case PanelRowKind::separator:
                return "----------------";
            case PanelRowKind::spacer:
                return {};
            case PanelRowKind::readout:
                return row.text + ": " + row.value;
            case PanelRowKind::number:
            case PanelRowKind::stepper:
                return row.text + ": " + (row.mixed ? row.value : row.spec ? core::format_quantity(row.number_si, row.spec->number.quantity, units)
                                                                           : core::fixed(row.number_si, 2)) +
                    "  [-] [+]";
            case PanelRowKind::switch_control:
            case PanelRowKind::checkbox:
                return std::string(row.selected ? "[x] " : "[ ] ") + row.text;
            case PanelRowKind::select:
                return row.text + ": " + (row.mixed || !row.value.empty() ? row.value : row.selected_option);
            case PanelRowKind::radio_list:
            case PanelRowKind::segmented:
                return row.text + ": " + row.selected_option;
            case PanelRowKind::tabs:
                return "Tabs: " + row.selected_option;
            case PanelRowKind::section:
                return std::string(row.selected ? "v " : "> ") + row.text;
            case PanelRowKind::meter:
                return row.text + ": " + row.value + " (" + row.hint + ")";
            case PanelRowKind::notice:
                return "! " + row.text;
            case PanelRowKind::list_item:
                return row.text + (row.hint.empty() ? "" : ": " + row.hint);
            case PanelRowKind::text_field:
                return row.text + ": " + row.value;
            case PanelRowKind::plot:
                return "[Graph]";
            default:
                return row.text;
            }
        }

        bool actionable(const PanelRow& row)
        {
            return row.command.kind != UiCommandKind::none || row.alternate_command.kind != UiCommandKind::none || row.shift_command.kind != UiCommandKind::none || row.kind == PanelRowKind::number || row.kind == PanelRowKind::stepper;
        }
    }

    std::string_view OverlayBackend::name() const
    {
        return "overlay";
    }

    bool OverlayBackend::initialize(const std::filesystem::path&)
    {
        hotspots_.clear();
        hotspot_rows_.clear();
        panel_regions_.clear();
        rows_.clear();
        return true;
    }

    void OverlayBackend::shutdown()
    {
        hotspots_.clear();
        hotspot_rows_.clear();
        panel_regions_.clear();
        rows_.clear();
    }

    double OverlayBackend::column_width() const
    {
        return column_width_;
    }
    void OverlayBackend::set_column_width(double value)
    {
        column_width_ = std::max(value, 120.0);
    }

    void OverlayBackend::draw_panel_frame(const Rect& bounds, const Theme& theme, DrawList& list) const
    {
        list.add_rectangle_fill(bounds.minimum, bounds.maximum, theme.panel_background);
        list.add_rectangle_outline(bounds.minimum, bounds.maximum, theme.panel_border);
    }

    void OverlayBackend::build(const UiFrameContext& context, DrawList& list)
    {
        list.clear();
        hotspots_.clear();
        hotspot_rows_.clear();
        panel_regions_.clear();
        rows_.clear();
        if (!context.model || !context.device || !context.theme)
            return;

        last_input_.viewport = context.viewport;
        last_input_.scale = context.scale;
        last_input_.overlay_backend = true;
        const auto layout = compute_layout(last_input_);
        const auto* region = layout.find(RegionId::overlay_list);
        if (!region)
            return;
        const auto bounds = region->bounds;
        draw_panel_frame(bounds, *context.theme, list);
        panel_regions_.push_back(bounds);

        DrawList scratch;
        std::vector<Hotspot> ignored;
        PanelBuilder builder { *context.theme, *context.device, context.scale, bounds, scratch, ignored, context.view };
        builder.record_rows(rows_);
        for (auto* panel : context.panels)
        {
            if (!panel || !panel->is_visible())
                continue;
            PanelRow heading { PanelRowKind::title, std::string(panel->title()) };
            heading.key = std::string(panel->id());
            rows_.push_back(std::move(heading));
            const auto first = rows_.size();
            panel->build(*context.model, builder);
            // The list has no nesting, so actions inside an item follow it as lines of their own.
            for (auto index = first; index < rows_.size(); ++index)
                if (rows_[index].list_content && !rows_[index].list_content->actions.empty())
                {
                    std::vector<PanelRow> actions;
                    for (const auto& action : rows_[index].list_content->actions)
                    {
                        PanelRow row { PanelRowKind::action, action.label };
                        row.command = action.command;
                        actions.push_back(std::move(row));
                    }
                    rows_.insert(rows_.begin() + static_cast<std::ptrdiff_t>(index) + 1, actions.begin(), actions.end());
                    index += actions.size();
                }
        }

        const double pad = 8.0 * context.scale;
        const double line = 22.0 * context.scale;
        content_height_ = rows_.size() * line + 2.0 * pad;
        const double max_scroll = std::max(0.0, content_height_ - bounds.height());
        scroll_offset_ = std::clamp(scroll_offset_, 0.0, max_scroll);
        double y = bounds.minimum.y + pad - scroll_offset_;
        for (std::size_t index = 0; index < rows_.size(); ++index, y += line)
        {
            const Rect row_bounds { { bounds.minimum.x + pad, y }, { bounds.maximum.x - pad, y + line } };
            if (row_bounds.maximum.y < bounds.minimum.y || row_bounds.minimum.y > bounds.maximum.y)
                continue;
            const auto text = active_number_ == index ? rows_[index].text + ": " + edit_buffer_ + "_" : row_text(rows_[index], context.model->display_units);
            list.add_text({ row_bounds.minimum.x + 3.0, row_bounds.minimum.y + 3.0 }, text, rows_[index].kind == PanelRowKind::title ? context.theme->panel_title : context.theme->panel_text, context.scale);
            if (actionable(rows_[index]) && rows_[index].disabled_reason.empty())
            {
                hotspots_.push_back({ row_bounds, rows_[index].command });
                hotspot_rows_.push_back(index);
            }
        }
    }

    bool OverlayBackend::handle_event(const UiEvent& event, std::vector<UiCommand>& commands)
    {
        if (event.kind == UiEventKind::wheel)
        {
            scroll_offset_ = std::max(0.0, scroll_offset_ - event.wheel_delta * 42.0 * event.logical_pixel_scale);
            return std::any_of(panel_regions_.begin(), panel_regions_.end(), [&](const Rect& r)
                {
                    return r.contains(event.pointer_px);
                });
        }
        if (event.kind == UiEventKind::key_down)
        {
            if (event.key == UiKey::page_down || event.key == UiKey::arrow_down)
            {
                scroll_offset_ += event.key == UiKey::page_down ? 360.0 : 28.0;
                return true;
            }
            if (event.key == UiKey::page_up || event.key == UiKey::arrow_up)
            {
                scroll_offset_ = std::max(0.0, scroll_offset_ - (event.key == UiKey::page_up ? 360.0 : 28.0));
                return true;
            }
            if (active_number_)
            {
                if (event.key == UiKey::escape)
                {
                    active_number_.reset();
                    edit_buffer_.clear();
                    return true;
                }
                if (event.key == UiKey::backspace && !edit_buffer_.empty())
                {
                    edit_buffer_.pop_back();
                    return true;
                }
                if (event.key == UiKey::enter)
                {
                    double value {};
                    const auto [end, error] = std::from_chars(edit_buffer_.data(), edit_buffer_.data() + edit_buffer_.size(), value);
                    if (error == std::errc {} && end == edit_buffer_.data() + edit_buffer_.size())
                    {
                        auto command = rows_[*active_number_].command;
                        command.value = value;
                        commands.push_back(std::move(command));
                    }
                    active_number_.reset();
                    edit_buffer_.clear();
                    return true;
                }
            }
        }
        if (event.kind == UiEventKind::text_input && active_number_)
        {
            for (char c : event.text)
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+')
                    edit_buffer_.push_back(c);
            return true;
        }
        if (event.kind == UiEventKind::pointer_move)
            pointer_px_ = event.pointer_px;
        if (event.kind == UiEventKind::pointer_down && event.button == PointerButton::primary)
        {
            pointer_px_ = event.pointer_px;
            for (std::size_t i = hotspots_.size(); i-- > 0;)
            {
                if (!hotspots_[i].bounds.contains(event.pointer_px))
                    continue;
                if (i >= hotspot_rows_.size())
                    continue;
                const auto r = hotspot_rows_[i];
                if (rows_[r].kind == PanelRowKind::number || rows_[r].kind == PanelRowKind::stepper)
                {
                    active_number_ = r;
                    edit_buffer_.clear();
                    return true;
                }
                commands.push_back(event.modifiers.shift && rows_[r].shift_command.kind != UiCommandKind::none ? rows_[r].shift_command : rows_[r].command);
                return true;
            }
        }
        if (event.kind == UiEventKind::pointer_down || event.kind == UiEventKind::pointer_up)
            return std::any_of(panel_regions_.begin(), panel_regions_.end(), [&](const Rect& r)
                {
                    return r.contains(event.pointer_px);
                });
        return false;
    }

    std::vector<UiCommand> OverlayBackend::emitted_commands() const
    {
        std::vector<UiCommand> result;
        for (const auto& row : rows_)
        {
            if (row.disabled_reason.empty() && row.command.kind != UiCommandKind::none)
                result.push_back(row.command);
            if (row.disabled_reason.empty() && row.alternate_command.kind != UiCommandKind::none)
                result.push_back(row.alternate_command);
            if (row.disabled_reason.empty() && row.shift_command.kind != UiCommandKind::none)
                result.push_back(row.shift_command);
        }
        return result;
    }
}
