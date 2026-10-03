#include <rigidbodies/app/content_files.hpp>

#include <atomic>
#include <chrono>
#include <fstream>
#include <system_error>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace rigidbodies::app
{
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
