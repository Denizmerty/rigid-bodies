#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace rigidbodies::app
{
    inline constexpr std::size_t maximum_content_file_bytes = 16 * 1024 * 1024;
    bool read_content_file(const std::filesystem::path& path, std::string& text, std::string& error);
    // Write completely to a reserved sibling before atomically replacing the destination.
    // Existing documents remain intact if staging or replacement fails.
    bool write_content_file(const std::filesystem::path& path, std::string_view text, std::string& error);
}
