#include <rigidbodies/app/content_files.hpp>

#include <atomic>
#include <chrono>
#include <fstream>
#include <system_error>
#include <algorithm>
#include <cctype>
#include <cwctype>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace rigidbodies::app
{
    std::string suggested_content_filename(std::string_view title, bool shape)
    {
        std::string name;
        for (const auto value : title)
        {
            const auto byte = static_cast<unsigned char>(value);
            if (byte < 32 || std::string_view("<>:\"/\\|?*").find(value) != std::string_view::npos)
            {
                if (!name.empty() && name.back() != ' ')
                    name.push_back(' ');
            }
            else
                name.push_back(value);
        }
        const auto first = name.find_first_not_of(" .\t");
        name = first == std::string::npos ? std::string {} : name.substr(first);
        if (name.size() > 120)
        {
            auto end = std::size_t { 120 };
            while (end > 0 && (static_cast<unsigned char>(name[end]) & 0xc0) == 0x80)
                --end;
            name.resize(end);
        }
        const auto last = name.find_last_not_of(" .\t");
        name = last == std::string::npos ? std::string {} : name.substr(0, last + 1);
        if (name.empty())
            name = shape ? "My shape" : "My setup";
        auto stem = name.substr(0, name.find('.'));
        std::transform(stem.begin(), stem.end(), stem.begin(), [](unsigned char value)
            {
                return static_cast<char>(std::toupper(value));
            });
        if (stem == "CON" || stem == "PRN" || stem == "AUX" || stem == "NUL" ||
            (stem.size() == 4 && (stem.substr(0, 3) == "COM" || stem.substr(0, 3) == "LPT") && stem[3] >= '1' && stem[3] <= '9'))
            name.insert(name.begin(), '_');
        return name + (shape ? ".rbshape.json" : ".rbscenario.json");
    }

    std::filesystem::path content_save_destination(std::filesystem::path path, bool shape)
    {
        if (!path.empty() && !path.filename().empty() && !path.has_extension())
            path += shape ? ".rbshape.json" : ".rbscenario.json";
        return path;
    }

    bool same_content_file(const std::filesystem::path& first, const std::filesystem::path& second)
    {
        if (first.empty() || second.empty())
            return false;
        std::error_code code;
        if (std::filesystem::equivalent(first, second, code) && !code)
            return true;
        const auto normalized = [](const std::filesystem::path& path)
        {
            std::error_code error;
            auto result = std::filesystem::weakly_canonical(path, error);
            if (error)
            {
                error.clear();
                result = std::filesystem::absolute(path, error).lexically_normal();
                if (error)
                    result = path.lexically_normal();
            }
            return result;
        };
#if defined(_WIN32)
        auto left = normalized(first).native();
        auto right = normalized(second).native();
        std::transform(left.begin(), left.end(), left.begin(), [](wchar_t value)
            {
                return static_cast<wchar_t>(std::towlower(value));
            });
        std::transform(right.begin(), right.end(), right.begin(), [](wchar_t value)
            {
                return static_cast<wchar_t>(std::towlower(value));
            });
        return left == right;
#else
        return normalized(first) == normalized(second);
#endif
    }

    bool read_content_file(const std::filesystem::path& path, std::string& text, std::string& error)
    {
        std::error_code code;
        if (!std::filesystem::is_regular_file(path, code))
        {
            error = "Choose an existing document file.";
            return false;
        }
        const auto size = std::filesystem::file_size(path, code);
        if (code || size > maximum_content_file_bytes)
        {
            error = "The document cannot be read or exceeds the 16 MiB size limit.";
            return false;
        }
        std::ifstream input(path, std::ios::binary);
        std::string staged(static_cast<std::size_t>(size), '\0');
        if (!input || !input.read(staged.data(), static_cast<std::streamsize>(size)) || input.peek() != std::char_traits<char>::eof())
        {
            error = "The file could not be read completely. It may have changed while it was being opened.";
            return false;
        }
        text = std::move(staged);
        error.clear();
        return true;
    }

    bool write_content_file(const std::filesystem::path& path, std::string_view text, std::string& error)
    {
        if (path.empty() || path.filename().empty() || text.size() > maximum_content_file_bytes)
        {
            error = "Choose a file name and keep the document within the 16 MiB size limit.";
            return false;
        }
        std::error_code code;
        const auto target = std::filesystem::absolute(path, code);
        if (code)
        {
            error = "The destination path could not be resolved.";
            return false;
        }
        if (std::filesystem::exists(target, code) && !std::filesystem::is_regular_file(target, code))
        {
            error = "The destination is not a regular file.";
            return false;
        }
        static std::atomic<unsigned long long> sequence { 0 };
        std::filesystem::path staging;
        bool reserved = false;
        for (int attempt = 0; attempt < 32 && !reserved; ++attempt)
        {
            const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
            staging = target.parent_path() / (".rigid-bodies-save-" + std::to_string(ticks) + "-" + std::to_string(sequence++));
            code.clear();
            reserved = std::filesystem::create_directory(staging, code);
            if (code)
                break;
        }
        if (!reserved)
        {
            error = "Cannot create a temporary file in the destination folder.";
            return false;
        }
        const auto temporary = staging / "document.tmp";
        const auto cleanup = [&]
        {
            std::error_code ignored;
            std::filesystem::remove(temporary, ignored);
            std::filesystem::remove(staging, ignored);
        };
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        output.write(text.data(), static_cast<std::streamsize>(text.size()));
        output.flush();
        const auto written = output.good();
        output.close();
        if (!written || output.fail())
        {
            cleanup();
            error = "The document could not be written completely. The previous file was kept.";
            return false;
        }
#if defined(_WIN32)
        const auto replaced = MoveFileExW(temporary.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        std::filesystem::rename(temporary, target, code);
        const auto replaced = !code;
#endif
        cleanup();
        if (!replaced)
        {
            error = "The destination could not be replaced. The previous file was kept.";
            return false;
        }
        error.clear();
        return true;
    }
}
