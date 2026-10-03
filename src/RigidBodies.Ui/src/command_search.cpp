#include <rigidbodies/ui/command_search.hpp>

#include <algorithm>
#include <cctype>

namespace rigidbodies::ui
{
    namespace
    {
        std::string lower(std::string_view value)
        {
            std::string result(value);
            std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c)
                {
                    return static_cast<char>(std::tolower(c));
                });
            return result;
        }

        int match_rank(std::string_view text, std::string_view query, int base)
        {
            const auto haystack = lower(text);
            const auto needle = lower(query);
            if (needle.empty())
                return base;
            if (haystack.rfind(needle, 0) == 0)
                return base;
            auto position = haystack.find(needle);
            while (position != std::string::npos)
            {
                if (position == 0 || !std::isalnum(static_cast<unsigned char>(haystack[position - 1])))
                    return base + 1;
                position = haystack.find(needle, position + 1);
            }
            return haystack.find(needle) != std::string::npos ? base + 2 : 1000;
        }

        bool contains_every_word(std::string_view text, std::string_view query)
        {
            const auto haystack = lower(text);
            const auto words = lower(query);
            std::size_t start = 0;
            bool found_word = false;
            while (start < words.size())
            {
                while (start < words.size() && std::isspace(static_cast<unsigned char>(words[start])))
                    ++start;
                auto end = start;
                while (end < words.size() && !std::isspace(static_cast<unsigned char>(words[end])))
                    ++end;
                if (end > start)
                {
                    found_word = true;
                    if (haystack.find(words.substr(start, end - start)) == std::string::npos)
                        return false;
                }
                start = end;
            }
            return found_word;
        }
    }

    std::vector<CommandSearchResult> search_commands(std::string_view query, math::Span<const KeyReference> keys)
    {
        std::vector<CommandSearchResult> results;
        for (const auto& spec : control_specs())
        {
            auto rank = match_rank(spec.label, query, 0);
            rank = std::min(rank, match_rank(spec.expert_term, query, 2));
            for (const auto synonym : spec.synonyms)
                rank = std::min(rank, match_rank(synonym, query, 2));
            rank = std::min(rank, match_rank(spec.location, query, 3));
            std::string combined = std::string(spec.label) + " " + std::string(spec.location) + " " + std::string(spec.expert_term);
            for (const auto synonym : spec.synonyms)
                combined += " " + std::string(synonym);
            if (contains_every_word(combined, query))
                rank = std::min(rank, 4);
            if (rank >= 1000)
                continue;
            std::string shortcut(spec.shortcut);
            if (shortcut.empty())
                for (const auto& key : keys)
                    if (lower(key.description).find(lower(spec.label)) != std::string::npos)
                    {
                        shortcut = key.chord;
                        break;
                    }
            UiCommand command;
            if (spec.kind == ControlKind::action)
            {
                command.kind = spec.command;
                command.id = std::string(spec.command_id);
            }
            else
                command.detail = "search-reveal:" + std::string(spec.key);
            results.push_back({ &spec, std::string(spec.label), std::string(spec.location), std::string(spec.key), command, std::move(shortcut), rank });
        }
        for (const auto& key : keys)
        {
            auto rank = std::min(match_rank(key.description, query, 0), match_rank(key.chord, query, 1));
            if (rank >= 1000)
                continue;
            // A command listed as a control already shows its shortcut; it is not listed twice.
            if (std::any_of(results.begin(), results.end(), [&](const CommandSearchResult& result)
                    {
                        return result.control && lower(result.label) == lower(key.description);
                    }))
                continue;
            UiCommand command;
            const auto description = lower(key.description);
            if (description == "play / pause")
                command.kind = UiCommandKind::toggle_pause;
            else if (description == "back to start")
                command.kind = UiCommandKind::reset_scenario;
            else if (description == "single step")
                command.kind = UiCommandKind::single_step;
            else if (description == "frame subject")
                command.kind = UiCommandKind::frame_subject;
            else if (description == "frame selection")
                command.kind = UiCommandKind::frame_selection;
            else if (description == "undo")
                command.kind = UiCommandKind::undo;
            else if (description == "redo")
                command.kind = UiCommandKind::redo;
            else if (description == "delete selection")
                command.kind = UiCommandKind::delete_selected_body;
            else if (description == "select all free objects")
                command.kind = UiCommandKind::select_all;
            else if (description == "draw shape")
                command.kind = UiCommandKind::start_new_shape;
            else if (description == "open library")
                command.detail = "view:library";
            else if (description == "open preferences")
                command.detail = "view:preferences";
            else if (description == "command search")
                command.detail = "view:search";
            else if (description == "toggle present")
                command.detail = "view:present";
            else if (description == "toggle show")
                command.detail = "view:show";
            else if (description == "toggle measure")
                command.detail = "view:measure";
            else if (description == "toggle guide")
                command.detail = "view:guide";
            else if (description == "open world")
                command.detail = "view:world";
            else if (description == "toggle inspector")
                command.detail = "view:inspector";
            else if (description == "keyboard shortcuts")
                command.detail = "view:shortcuts";
            else
                continue;
            if (command.detail.empty())
                command.detail = "search-run:key:" + key.description;
            else
                command.id = "search:key:" + key.description;
            results.push_back({ nullptr, key.description, "Keyboard", "key:" + key.description, command, key.chord, rank });
        }
        std::stable_sort(results.begin(), results.end(), [](const auto& left, const auto& right)
            {
                return left.rank < right.rank;
            });
        if (results.size() > 50)
            results.resize(50);
        return results;
    }
}
