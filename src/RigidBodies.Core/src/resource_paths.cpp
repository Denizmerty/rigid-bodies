#include <rigidbodies/core/resource_paths.hpp>

#include <rigidbodies/core/log.hpp>

#include <array>
#include <cstdlib>
#include <system_error>

namespace rigidbodies::core
{
    namespace
    {

        // Candidates are tried in order. The first entry covers a staged or installed layout, the
        // rest walk up towards a source tree so that a build-tree run finds the originals.
        constexpr std::array<std::string_view, 4> search_prefixes { ".", "..", "../..", "../../.." };

        std::filesystem::path find_directory(const std::filesystem::path& base, std::string_view name)
        {
            std::error_code error;
            for (const auto prefix : search_prefixes)
            {
                auto candidate = std::filesystem::weakly_canonical(base / prefix / name, error);
                if (!error && std::filesystem::is_directory(candidate, error))
                {
                    return candidate;
                }
            }
            return base / name;
        }

        std::filesystem::path platform_user_data_root()
        {
            std::error_code error;

#if defined(_WIN32)
            if (const auto* app_data = std::getenv("APPDATA"); app_data != nullptr)
            {
                return std::filesystem::path { app_data } / "RigidBodies";
            }
#elif defined(__APPLE__)
            if (const auto* home = std::getenv("HOME"); home != nullptr)
            {
                return std::filesystem::path { home } / "Library" / "Application Support" / "RigidBodies";
            }
#else
            if (const auto* config_home = std::getenv("XDG_CONFIG_HOME"); config_home != nullptr)
            {
                return std::filesystem::path { config_home } / "rigid-bodies";
            }
            if (const auto* home = std::getenv("HOME"); home != nullptr)
            {
                return std::filesystem::path { home } / ".config" / "rigid-bodies";
            }
#endif

            // With no home directory available the application still runs; it simply keeps its
            // settings next to itself.
            return std::filesystem::current_path(error) / "user";
        }

    } // namespace

    ResourcePaths ResourcePaths::discover(const std::filesystem::path& executable_directory)
    {
        ResourcePaths paths;
        paths.executable_directory_ = executable_directory;
        paths.asset_root_ = find_directory(executable_directory, "assets");
        paths.config_root_ = find_directory(executable_directory, "config");
        paths.user_data_root_ = platform_user_data_root();

        log_trace("asset root resolved to {}", paths.asset_root_.string());
        log_trace("configuration root resolved to {}", paths.config_root_.string());
        return paths;
    }

    const std::filesystem::path& ResourcePaths::executable_directory() const
    {
        return executable_directory_;
    }

    const std::filesystem::path& ResourcePaths::asset_root() const
    {
        return asset_root_;
    }

    const std::filesystem::path& ResourcePaths::config_root() const
    {
        return config_root_;
    }

    const std::filesystem::path& ResourcePaths::user_data_root() const
    {
        return user_data_root_;
    }

    std::filesystem::path ResourcePaths::asset(std::string_view relative) const
    {
        return asset_root_ / relative;
    }

    std::filesystem::path ResourcePaths::config(std::string_view relative) const
    {
        return config_root_ / relative;
    }

    std::filesystem::path ResourcePaths::user_data(std::string_view relative) const
    {
        return user_data_root_ / relative;
    }

    std::optional<std::filesystem::path> ResourcePaths::find_application_config() const
    {
        std::error_code error;

        auto user_copy = user_data("application.cfg");
        if (std::filesystem::is_regular_file(user_copy, error))
        {
            return user_copy;
        }

        auto shipped = config("application.cfg");
        if (std::filesystem::is_regular_file(shipped, error))
        {
            return shipped;
        }

        return std::nullopt;
    }

    bool ResourcePaths::ensure_user_data_root() const
    {
        std::error_code error;
        if (std::filesystem::is_directory(user_data_root_, error))
        {
            return true;
        }

        std::filesystem::create_directories(user_data_root_, error);
        if (error)
        {
            log_warning("could not create the user data directory at {}: {}", user_data_root_.string(), error.message());
            return false;
        }
        return true;
    }

} // namespace rigidbodies::core
