#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand command(UiCommandKind kind)
        {
            UiCommand result;
            result.kind = kind;
            return result;
        }
    }

    std::string_view SaveDetailsPanel::id() const
    {
        return "save_details";
    }
    std::string_view SaveDetailsPanel::title() const
    {
        return "Save details";
    }
    RegionId SaveDetailsPanel::region() const
    {
        return RegionId::modal;
    }

    void SaveDetailsPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto state = [](std::string_view key, std::string_view value)
        {
            auto result = command(UiCommandKind::none);
            result.detail = "state:" + std::string(key) + "=" + std::string(value);
            return result;
        };
        const auto fallback_title = (model.scenario_title.empty() ? std::string { "My setup" } : model.scenario_title) + " (my version)";
        const auto title_text = std::string(builder.view_value("save.title", fallback_title));
        const auto mode = std::string(builder.view_value("save.mode", "starting"));
        const auto include_guide = builder.view_value("save.include_guide", "true") != "false";

        builder.title(title());
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("save_details.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        builder.text_field("save.title", "Title", title_text);
        builder.heading("Save");
        builder.choice_row("save.mode.starting", "The starting setup", mode == "starting", state("save.mode", "starting"));
        builder.choice_row("save.mode.current", "The current moment", mode == "current", state("save.mode", "current"));
        builder.toggle_row("save.include_guide", "Include the guide", include_guide, state("save.include_guide", include_guide ? "false" : "true"));

        auto save = command(UiCommandKind::save_arrangement);
        save.detail = title_text;
        save.value = mode == "current" ? 1.0 : 0.0;
        save.flag = include_guide;
        builder.action_row("save.apply", "Save setup…", save, title_text.empty() ? "Enter a title first." : "");
        auto cancel = command(UiCommandKind::none);
        cancel.detail = "view:close";
        builder.action_row("save.cancel", "Cancel", cancel);
    }
}
