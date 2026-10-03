#pragma once

#include <filesystem>
#include <optional>
#include <string_view>

namespace rigidbodies::core
{

    // Finds assets and configuration relative to the executable.
    //
    // The ordered search covers build-tree and installed layouts. It works regardless of the
    // working directory chosen by a shell or desktop shortcut.
    class ResourcePaths
    {
    public:
        // Discovers the roots from the executable location reported by the platform layer.
        static ResourcePaths discover(const std::filesystem::path& executable_directory);

        [[nodiscard]] const std::filesystem::path& executable_directory() const;
        [[nodiscard]] const std::filesystem::path& asset_root() const;
        [[nodiscard]] const std::filesystem::path& config_root() const;

        // Default location for user settings and saved files. Asset and configuration roots
        // are read-only during normal use.
        [[nodiscard]] const std::filesystem::path& user_data_root() const;

        [[nodiscard]] std::filesystem::path asset(std::string_view relative) const;
        [[nodiscard]] std::filesystem::path config(std::string_view relative) const;
        [[nodiscard]] std::filesystem::path user_data(std::string_view relative) const;

        // Returns the first of the two configuration candidates that exists: the per-user file
        // takes precedence over the one shipped with the application.
        [[nodiscard]] std::optional<std::filesystem::path> find_application_config() const;

        // Creates the user data directory if it is absent. Returns false when it could not be
        // created, in which case the application runs without persisting settings.
        bool ensure_user_data_root() const;

    private:
        std::filesystem::path executable_directory_;
        std::filesystem::path asset_root_;
        std::filesystem::path config_root_;
        std::filesystem::path user_data_root_;
    };

} // namespace rigidbodies::core
