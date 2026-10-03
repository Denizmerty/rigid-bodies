#include <rigidbodies/ui/view_state.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <sstream>

namespace rigidbodies::ui
{
    namespace
    {
        std::string trim(std::string_view value)
        {
            const auto first = value.find_first_not_of(" \t\r\n");
            if (first == std::string_view::npos)
                return {};
            const auto last = value.find_last_not_of(" \t\r\n");
            return std::string(value.substr(first, last - first + 1));
        }

        std::vector<std::string> split_list(std::string_view value)
        {
            std::vector<std::string> result;
            while (!value.empty())
            {
                const auto comma = value.find(',');
                auto item = trim(value.substr(0, comma));
                if (!item.empty())
                    result.push_back(std::move(item));
                if (comma == std::string_view::npos)
                    break;
                value.remove_prefix(comma + 1);
            }
            return result;
        }
    }

    std::string_view ViewState::active_tab(std::string_view key, math::Span<const std::string_view> available, std::string_view fallback) const
    {
        const auto found = active_tabs_.find(key);
        if (found != active_tabs_.end() && std::find(available.begin(), available.end(), found->second) != available.end())
            return found->second;
        return fallback;
    }

    void ViewState::set_active_tab(std::string_view key, std::string_view value)
    {
        active_tabs_[std::string(key)] = value;
    }

    bool ViewState::section_open(std::string_view key, bool fallback) const
    {
        const auto found = open_sections_.find(key);
        return found == open_sections_.end() ? fallback : found->second;
    }

    void ViewState::set_section_open(std::string_view key, bool value)
    {
        open_sections_[std::string(key)] = value;
    }

    std::vector<std::string> ViewState::checklist(std::string_view key, math::Span<const std::string_view> defaults) const
    {
        const auto found = checklists_.find(key);
        if (found != checklists_.end())
            return found->second;
        return { defaults.begin(), defaults.end() };
    }

    void ViewState::set_checklist(std::string_view key, std::vector<std::string> values)
    {
        checklists_[std::string(key)] = std::move(values);
    }

    bool ViewState::surface_open(std::string_view key, bool fallback) const
    {
        const auto found = surfaces_.find(key);
        return found == surfaces_.end() ? fallback : found->second;
    }

    void ViewState::set_surface_open(std::string_view key, bool value)
    {
        surfaces_[std::string(key)] = value;
    }

    void ViewState::toggle_surface(std::string_view key, bool fallback)
    {
        set_surface_open(key, !surface_open(key, fallback));
    }

    double ViewState::number(std::string_view key, double fallback) const
    {
        const auto found = numbers_.find(key);
        return found == numbers_.end() ? fallback : found->second;
    }

    void ViewState::set_number(std::string_view key, double value)
    {
        if (std::isfinite(value))
            numbers_[std::string(key)] = value;
    }

    std::string_view ViewState::value(std::string_view key, std::string_view fallback) const
    {
        if (key == "measure.runs.compare_a" || key == "measure.runs.compare_b" || key == "measure.graph.compare_with")
        {
            const auto experiment = experiment_values_.find(experiment_context_);
            if (experiment != experiment_values_.end())
            {
                const auto found = experiment->second.find(key);
                if (found != experiment->second.end())
                    return found->second;
            }
            return fallback;
        }
        const auto found = values_.find(key);
        return found == values_.end() ? fallback : std::string_view(found->second);
    }

    void ViewState::set_value(std::string_view key, std::string_view value)
    {
        if (key == "measure.runs.compare_a" || key == "measure.runs.compare_b" || key == "measure.graph.compare_with")
        {
            experiment_values_[experiment_context_][std::string(key)] = value;
            return;
        }
        values_[std::string(key)] = value;
    }

    void ViewState::set_experiment_context(std::string_view experiment)
    {
        experiment_context_ = experiment;
    }

    bool ViewState::visited(std::string_view experiment) const
    {
        return visited_.find(experiment) != visited_.end();
    }

    void ViewState::mark_visited(std::string_view experiment)
    {
        if (!experiment.empty())
            visited_.insert(std::string(experiment));
    }

    void ViewState::merge_setup_file(const SetupFileInfo& file)
    {
        if (file.path.empty())
            return;
        setup_files_.erase(std::remove_if(setup_files_.begin(), setup_files_.end(), [&](const auto& item)
                               {
                                   return item.path == file.path;
                               }),
            setup_files_.end());
        setup_files_.insert(setup_files_.begin(), file);
        if (setup_files_.size() > 8)
            setup_files_.resize(8);
    }

    void ViewState::remove_setup_file(std::string_view path)
    {
        setup_files_.erase(std::remove_if(setup_files_.begin(), setup_files_.end(), [&](const auto& file)
                               {
                                   return file.path == path;
                               }),
            setup_files_.end());
    }

    namespace
    {
        void add_unique(std::vector<std::string>& values, std::string_view value)
        {
            if (!value.empty() && std::find(values.begin(), values.end(), value) == values.end())
                values.emplace_back(value);
        }
        void erase_value(std::vector<std::string>& values, std::string_view value)
        {
            values.erase(std::remove(values.begin(), values.end(), value), values.end());
        }
    }

    void ViewState::open_sheet(std::string_view id)
    {
        add_unique(sheets_, id);
    }
    void ViewState::close_sheet(std::string_view id)
    {
        erase_value(sheets_, id);
    }
    void ViewState::open_transient(std::string_view id)
    {
        add_unique(transients_, id);
    }
    void ViewState::close_transient(std::string_view id)
    {
        erase_value(transients_, id);
    }
    bool ViewState::sheet_open(std::string_view id) const
    {
        return std::find(sheets_.begin(), sheets_.end(), id) != sheets_.end();
    }
    bool ViewState::transient_open(std::string_view id) const
    {
        return std::find(transients_.begin(), transients_.end(), id) != transients_.end();
    }

    bool ViewState::hint_dismissed(std::string_view id) const
    {
        return dismissed_hints_.find(id) != dismissed_hints_.end();
    }
    void ViewState::dismiss_hint(std::string_view id)
    {
        if (!id.empty())
            dismissed_hints_.insert(std::string(id));
    }
    void ViewState::reset_hints()
    {
        dismissed_hints_.clear();
    }
    std::string_view ViewState::next_hint() const
    {
        for (const auto id : { std::string_view { "play" }, std::string_view { "inspect" }, std::string_view { "library" } })
            if (!hint_dismissed(id))
                return id;
        return {};
    }
    void ViewState::remember_search(std::string_view key)
    {
        if (key.empty())
            return;
        search_recent_.erase(std::remove(search_recent_.begin(), search_recent_.end(), key), search_recent_.end());
        search_recent_.insert(search_recent_.begin(), std::string(key));
        if (search_recent_.size() > 8)
            search_recent_.resize(8);
    }

    std::string ViewState::serialize() const
    {
        std::ostringstream output;
        output << "interface_state.version = 2\n";
        for (const auto& [key, value] : active_tabs_)
            output << "tab." << key << " = " << value << '\n';
        for (const auto& [key, value] : open_sections_)
            output << "section." << key << " = " << (value ? "true" : "false") << '\n';
        for (const auto& [key, values] : checklists_)
        {
            output << "checklist." << key << " = ";
            for (std::size_t i = 0; i < values.size(); ++i)
                output << (i ? ", " : "") << values[i];
            output << '\n';
        }
        for (const auto& [key, value] : surfaces_)
            output << "surface." << key << " = " << (value ? "true" : "false") << '\n';
        for (const auto& [key, value] : numbers_)
            output << "number." << key << " = " << value << '\n';
        for (const auto& [key, value] : values_)
            output << "value." << key << " = " << value << '\n';
        for (const auto& value : visited_)
            output << "visited." << value << " = true\n";
        if (!dismissed_hints_.empty())
        {
            output << "hints.dismissed = ";
            bool first = true;
            for (const auto& value : dismissed_hints_)
            {
                output << (first ? "" : ", ") << value;
                first = false;
            }
            output << '\n';
        }
        const auto one_line = [](std::string value)
        {
            std::replace(value.begin(), value.end(), '\n', ' ');
            std::replace(value.begin(), value.end(), '\r', ' ');
            return value;
        };
        for (std::size_t index = 0; index < setup_files_.size(); ++index)
        {
            const auto prefix = "my_setups." + std::to_string(index) + ".";
            const auto& file = setup_files_[index];
            output << prefix << "serial = " << file.serial << '\n';
            output << prefix << "title = " << one_line(file.title) << '\n';
            output << prefix << "based_on = " << one_line(file.based_on) << '\n';
            output << prefix << "date = " << one_line(file.date) << '\n';
            output << prefix << "path = " << one_line(file.path) << '\n';
        }
        if (!sheets_.empty())
            output << "stack.sheets = " << [&]
            {
                std::string s;
                for (const auto& v : sheets_)
                {
                    if (!s.empty())
                        s += ", ";
                    s += v;
                }
                return s;
            }() << '\n';
        if (!transients_.empty())
            output << "stack.transients = " << [&]
            {
                std::string s;
                for (const auto& v : transients_)
                {
                    if (!s.empty())
                        s += ", ";
                    s += v;
                }
                return s;
            }() << '\n';
        return output.str();
    }

    std::vector<std::string> ViewState::deserialize(std::string_view text)
    {
        std::vector<std::string> diagnostics;
        std::map<std::string, std::string, std::less<>> tabs;
        std::map<std::string, bool, std::less<>> sections;
        std::map<std::string, std::vector<std::string>, std::less<>> lists;
        std::map<std::string, bool, std::less<>> surfaces;
        std::map<std::string, double, std::less<>> numbers;
        std::map<std::string, std::string, std::less<>> values;
        std::set<std::string, std::less<>> visited;
        std::set<std::string, std::less<>> dismissed_hints;
        std::map<std::size_t, SetupFileInfo> setup_files;
        std::vector<std::string> sheets, transients;
        bool version_seen = false;
        std::istringstream input { std::string(text) };
        std::string line;
        std::size_t line_number = 0;
        while (std::getline(input, line))
        {
            ++line_number;
            const auto cleaned = trim(line);
            if (cleaned.empty() || cleaned.front() == '#')
                continue;
            const auto equals = cleaned.find('=');
            if (equals == std::string::npos)
            {
                diagnostics.push_back("line " + std::to_string(line_number) + ": expected key = value");
                continue;
            }
            const auto key = trim(std::string_view(cleaned).substr(0, equals));
            const auto value = trim(std::string_view(cleaned).substr(equals + 1));
            if (key == "interface_state.version")
            {
                version_seen = true;
                if (value != "1" && value != "2")
                {
                    diagnostics.push_back("newer interface-state version ignored");
                    return diagnostics;
                }
            }
            else if (key.rfind("tab.", 0) == 0 && key.size() > 4 && !value.empty())
                tabs[key.substr(4)] = value;
            else if (key.rfind("section.", 0) == 0 && key.size() > 8 && (value == "true" || value == "false"))
                sections[key.substr(8)] = value == "true";
            else if (key.rfind("checklist.", 0) == 0 && key.size() > 10)
                lists[key.substr(10)] = split_list(value);
            else if (key.rfind("surface.", 0) == 0 && key.size() > 8 && (value == "true" || value == "false"))
                surfaces[key.substr(8)] = value == "true";
            else if (key.rfind("number.", 0) == 0 && key.size() > 7)
            {
                std::istringstream number_stream(value);
                double parsed {};
                if (number_stream >> parsed && number_stream.eof() && std::isfinite(parsed))
                    numbers[key.substr(7)] = parsed;
                else
                    diagnostics.push_back("line " + std::to_string(line_number) + ": malformed number " + key);
            }
            else if (key.rfind("value.", 0) == 0 && key.size() > 6)
                values[key.substr(6)] = value;
            else if (key.rfind("visited.", 0) == 0 && key.size() > 8 && value == "true")
                visited.insert(key.substr(8));
            else if (key == "hints.dismissed")
                for (auto& item : split_list(value))
                    dismissed_hints.insert(std::move(item));
            else if (key.rfind("my_setups.", 0) == 0)
            {
                const auto rest = std::string_view(key).substr(10);
                const auto dot = rest.find('.');
                if (dot == std::string_view::npos)
                {
                    diagnostics.push_back("line " + std::to_string(line_number) + ": malformed setup entry " + key);
                    continue;
                }
                std::size_t index {};
                const auto [end, error] = std::from_chars(rest.data(), rest.data() + dot, index);
                if (error != std::errc {} || end != rest.data() + dot || index >= 8)
                {
                    diagnostics.push_back("line " + std::to_string(line_number) + ": malformed setup entry " + key);
                    continue;
                }
                auto& file = setup_files[index];
                const auto field = rest.substr(dot + 1);
                if (field == "serial")
                {
                    const auto [serial_end, serial_error] = std::from_chars(value.data(), value.data() + value.size(), file.serial);
                    if (serial_error != std::errc {} || serial_end != value.data() + value.size())
                        diagnostics.push_back("line " + std::to_string(line_number) + ": malformed setup serial");
                }
                else if (field == "title")
                    file.title = value;
                else if (field == "based_on")
                    file.based_on = value;
                else if (field == "date")
                    file.date = value;
                else if (field == "path")
                    file.path = value;
                else
                    diagnostics.push_back("line " + std::to_string(line_number) + ": unknown setup field " + std::string(field));
            }
            else if (key == "stack.sheets")
                sheets = split_list(value);
            else if (key == "stack.transients")
                transients = split_list(value);
            else
                diagnostics.push_back("line " + std::to_string(line_number) + ": unknown or malformed key " + key);
        }
        if (!version_seen)
            diagnostics.push_back("interface_state.version is missing");
        active_tabs_ = std::move(tabs);
        open_sections_ = std::move(sections);
        checklists_ = std::move(lists);
        surfaces_ = std::move(surfaces);
        numbers_ = std::move(numbers);
        values_ = std::move(values);
        visited_ = std::move(visited);
        dismissed_hints_ = std::move(dismissed_hints);
        setup_files_.clear();
        for (auto& [index, file] : setup_files)
            if (!file.path.empty())
                setup_files_.push_back(std::move(file));
        if (setup_files_.size() > 8)
            setup_files_.resize(8);
        sheets_ = std::move(sheets);
        transients_ = std::move(transients);
        return diagnostics;
    }
}
