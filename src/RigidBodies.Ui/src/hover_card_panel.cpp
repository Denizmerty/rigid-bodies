#include <rigidbodies/ui/panels.hpp>
namespace rigidbodies::ui
{
    std::string_view HoverCardPanel::id() const
    {
        return "hover_card";
    }
    std::string_view HoverCardPanel::title() const
    {
        return "Details";
    }
    RegionId HoverCardPanel::region() const
    {
        return RegionId::show_popover;
    }
    // The first line names the target and heads the card; the rest describe it.
    void HoverCardPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.hover || model.hover->card_lines.empty())
            return;
        builder.title(model.hover->card_lines.front());
        for (std::size_t index = 1; index < model.hover->card_lines.size(); ++index)
            builder.paragraph(model.hover->card_lines[index]);
    }
}
