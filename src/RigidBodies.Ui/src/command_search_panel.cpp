#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/command_search.hpp>
namespace rigidbodies::ui
{
    std::string_view CommandSearchPanel::id() const
    {
        return "command_search";
    }
    std::string_view CommandSearchPanel::title() const
    {
        return "Command search";
    }
    RegionId CommandSearchPanel::region() const
    {
        return RegionId::modal;
    }
    // Results are a list rather than buttons: the name leads, and where it lives and its shortcut
    // follow quietly on the right.
    void CommandSearchPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!builder.view_sheet_open("command_search"))
            return;
        builder.title("Command search");
        {
            UiCommand close;
            close.detail = "view:close";
            builder.action_row("search.close", "Close", close);
            builder.present_last(presentation(icons::close, "Esc").icon_label_only().in_header());
        }
        const auto query = builder.view_value("search.query", "");
        builder.text_field("search.field.query", "Search actions and settings", query, "Search actions and settings…");
        builder.present_last(presentation(icons::search));
        if (query.empty())
        {
            const auto listed = [&](const std::string_view key)
            {
                if (const auto* spec = find_control_spec(key))
                {
                    UiCommand command;
                    if (spec->kind == ControlKind::action)
                    {
                        command.kind = spec->command;
                        command.id = std::string(spec->command_id);
                        command.detail = "search-run:" + std::string(spec->key);
                    }
                    else
                        command.detail = "search-reveal:" + std::string(spec->key);
                    builder.action_row("search.result", std::string(spec->label), command);
                    builder.present_last(presentation({}, spec->location));
                }
            };
            const auto recent = builder.view_search_recent();
            builder.heading(recent.empty() ? "Suggestions" : "Recent");
            builder.begin_group("results");
            // Before anything has been searched, the settings a lesson most often changes show
            // what the palette can reach.
            if (recent.empty())
                for (const auto key : { "world.gravity.strength", "world.air.resistance", "show.arrows.auto_length", "prefs.units.system", "prefs.appearance.theme", "camera.frame.everything" })
                    listed(key);
            else
                for (const auto& key : recent)
                    listed(key);
            builder.end_group();
            builder.paragraph("Search for gravity, theme, step or units. Press Enter to run an action or open a setting.");
            return;
        }
        const auto results = search_commands(query, model.keyboard_reference);
        if (results.empty())
        {
            builder.paragraph("No action or setting matches that search.");
            return;
        }
        builder.begin_group("results");
        for (const auto& result : results)
        {
            auto command = result.command;
            if (command.detail.rfind("view:", 0) != 0 && command.detail.rfind("search-reveal:", 0) != 0)
                command.detail = "search-run:" + result.key;
            builder.action_row("search.result", result.label, command);
            builder.present_last(presentation({}, result.shortcut.empty() ? result.location : result.location + " \xC2\xB7 " + result.shortcut));
        }
        builder.end_group();
    }
}
