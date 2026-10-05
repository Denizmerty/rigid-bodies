#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/ui/command_search.hpp>

#include <algorithm>
#include <optional>
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
        builder.text_field("search.field.query", "Search actions and settings", query, "Search actions and settings…", "search.query");
        builder.present_last(presentation(icons::search));
        const auto list_result = [&](const CommandSearchResult& result)
        {
            auto command = result.command;
            if (command.detail.rfind("view:", 0) != 0 && command.detail.rfind("search-reveal:", 0) != 0)
                command.detail = "search-run:" + result.key;
            builder.action_row("search.result", result.label, command);
            builder.present_last(presentation({}, result.shortcut.empty() ? result.location : result.location + " \xC2\xB7 " + result.shortcut));
        };
        if (query.empty())
        {
            const auto resolve = [&](const std::string_view key) -> std::optional<CommandSearchResult>
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
                    return CommandSearchResult { spec, std::string(spec->label), std::string(spec->location), std::string(spec->key), command, std::string(spec->shortcut), 0 };
                }
                // Shortcut searches also produce keyboard identities. A later label search may
                // deduplicate that action against a ControlSpec, so accept its equivalent label.
                if (key.rfind("key:", 0) == 0)
                {
                    const auto results = search_commands(key.substr(4), model.keyboard_reference);
                    const auto found = std::find_if(results.begin(), results.end(), [&](const auto& result)
                        {
                            return result.key == key || result.label == key.substr(4);
                        });
                    if (found != results.end())
                    {
                        auto result = *found;
                        result.key = std::string(key);
                        return result;
                    }
                }
                return std::nullopt;
            };
            const auto recent = builder.view_search_recent();
            std::vector<CommandSearchResult> recent_results;
            for (const auto& key : recent)
                if (auto result = resolve(key); result && control_available(model, result->key))
                    recent_results.push_back(std::move(*result));
            builder.heading(recent_results.empty() ? "Suggestions" : "Recent");
            builder.begin_group("results");
            // Before anything has been searched, the settings a lesson most often changes show
            // what the palette can reach.
            // A relativity experiment suggests its own speed and plot instead of gravity and arrows.
            static constexpr std::string_view newtonian_suggestions[] { "world.gravity.strength", "world.air.resistance", "show.arrows.auto_length", "prefs.units.system", "prefs.appearance.theme", "camera.frame.everything" };
            static constexpr std::string_view relativity_suggestions[] { "world.relativity.speed", "world.relativity.preset", "measure.relativity.curve", "prefs.units.system", "prefs.appearance.theme", "camera.frame.everything" };
            if (recent_results.empty())
                for (const auto key : model.relativity ? math::Span<const std::string_view> { relativity_suggestions } : math::Span<const std::string_view> { newtonian_suggestions })
                {
                    if (const auto result = resolve(key))
                        list_result(*result);
                }
            else
                for (const auto& result : recent_results)
                    list_result(result);
            builder.end_group();
            builder.paragraph(model.relativity ? "Search for speed, Lorentz factor, clocks, theme or units. Press Enter to run an action or open a setting."
                                               : "Search for gravity, theme, step or units. Press Enter to run an action or open a setting.");
            return;
        }
        // Results are limited to the controls this experiment offers.
        auto results = search_commands(query, model.keyboard_reference);
        results.erase(std::remove_if(results.begin(), results.end(), [&](const CommandSearchResult& result)
                          {
                              return !control_available(model, result.key);
                          }),
            results.end());
        if (results.empty())
        {
            builder.paragraph("No action or setting matches that search.");
            return;
        }
        builder.begin_group("results");
        for (const auto& result : results)
            list_result(result);
        builder.end_group();
    }
}
