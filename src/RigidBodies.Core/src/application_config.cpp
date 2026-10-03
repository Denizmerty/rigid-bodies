#include <rigidbodies/core/application_config.hpp>

#include <algorithm>
#include <charconv>
#include <fstream>
#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#endif
#include <locale>
#include <ostream>
#include <sstream>

namespace rigidbodies::core
{
    namespace
    {

        std::string_view trim(std::string_view text)
        {
            const auto first = text.find_first_not_of(" \t\r\n");
            if (first == std::string_view::npos)
            {
                return {};
            }
            const auto last = text.find_last_not_of(" \t\r\n");
            return text.substr(first, last - first + 1);
        }

        bool parse_bool(std::string_view text, bool& target)
        {
            if (text == "true" || text == "yes" || text == "on" || text == "1")
            {
                target = true;
                return true;
            }
            if (text == "false" || text == "no" || text == "off" || text == "0")
            {
                target = false;
                return true;
            }
            return false;
        }

        bool parse_int(std::string_view text, int& target)
        {
            int value = 0;
            const auto* end = text.data() + text.size();
            const auto result = std::from_chars(text.data(), end, value);
            if (result.ec != std::errc {} || result.ptr != end)
            {
                return false;
            }
            target = value;
            return true;
        }

        bool parse_double(std::string_view text, double& target)
        {
            // Floating-point from_chars is not available everywhere this project is meant to
            // build, so the conversion goes through a stream with the classic locale, which keeps
            // the decimal separator independent of the machine the application runs on.
            std::istringstream stream { std::string { text } };
            stream.imbue(std::locale::classic());
            double value = 0.0;
            stream >> value;
            if (!stream || !stream.eof())
            {
                return false;
            }
            target = value;
            return true;
        }

        bool parse_log_level(std::string_view text, LogLevel& target)
        {
            if (text == "trace")
            {
                target = LogLevel::trace;
                return true;
            }
            if (text == "info")
            {
                target = LogLevel::info;
                return true;
            }
            if (text == "warning")
            {
                target = LogLevel::warning;
                return true;
            }
            if (text == "error")
            {
                target = LogLevel::error;
                return true;
            }
            return false;
        }

        // Returns false when the key is known but the value could not be read, so that the caller
        // can tell a malformed value from an unrecognised key.
        bool assign(ApplicationConfig& config, std::string_view key, std::string_view value, bool& key_recognised)
        {
            key_recognised = true;

            if (key == "config.version")
            {
                return parse_int(value, config.version) && config.version >= 1;
            }

            if (key == "window.title")
            {
                config.window.title = std::string { value };
                return true;
            }
            if (key == "window.width")
            {
                return parse_int(value, config.window.width);
            }
            if (key == "window.height")
            {
                return parse_int(value, config.window.height);
            }
            if (key == "window.resizable")
            {
                return parse_bool(value, config.window.resizable);
            }
            if (key == "window.start_maximised")
            {
                return parse_bool(value, config.window.start_maximised);
            }
            if (key == "window.vertical_sync")
            {
                return parse_bool(value, config.window.vertical_sync);
            }
            if (key == "window.custom_title_bar")
            {
                return parse_bool(value, config.window.custom_title_bar);
            }

            if (key == "simulation.steps_per_second")
            {
                return parse_double(value, config.simulation.steps_per_second);
            }
            if (key == "simulation.time_scale")
            {
                return parse_double(value, config.simulation.time_scale);
            }
            if (key == "simulation.substeps" || key == "simulation.maximum_substeps_per_frame")
            {
                int count = 0;
                if (!parse_int(value, count) || count < 1 || count > 4096)
                {
                    return false;
                }
                (key == "simulation.substeps" ? config.simulation.substeps : config.simulation.maximum_substeps_per_frame) = count;
                return true;
            }
            if (key == "simulation.start_paused")
            {
                return parse_bool(value, config.simulation.start_paused);
            }
            if (key == "simulation.default_view_height_m")
            {
                return parse_double(value, config.simulation.default_view_height_m);
            }
            if (key == "simulation.startup_scenario")
            {
                config.startup_scenario = std::string { value };
                return true;
            }

            if (key == "interface.backend")
            {
                config.interface_settings.backend = std::string { value };
                return true;
            }
            if (key == "interface.theme")
            {
                config.interface_settings.theme = std::string { value };
                return true;
            }
            if (key == "interface.scale")
            {
                double parsed {};
                if (!parse_double(value, parsed) || parsed < 0.75 || parsed > 2.0)
                    return false;
                config.interface_settings.interface_scale = parsed;
                return true;
            }
            if (key == "interface.reduce_motion")
                return parse_bool(value, config.interface_settings.reduce_motion);
            if (key == "interface.units")
            {
                if (value != "si" && value != "centimetre_gram")
                    return false;
                config.interface_settings.units = std::string(value);
                return true;
            }
            if (key == "interface.show_developer_overlay")
            {
                return parse_bool(value, config.interface_settings.show_developer_overlay);
            }
            if (key == "interface.performance_overlay")
                return parse_bool(value, config.interface_settings.performance_overlay);

            if (key == "visualization.velocity_vectors")
            {
                return parse_bool(value, config.visualization.show_velocity_vectors);
            }
            if (key == "visualization.force_vectors")
            {
                return parse_bool(value, config.visualization.show_force_vectors);
            }
            if (key == "visualization.contact_points")
            {
                return parse_bool(value, config.visualization.show_contact_points);
            }
            if (key == "visualization.center_of_mass")
            {
                return parse_bool(value, config.visualization.show_center_of_mass);
            }
            if (key == "visualization.trajectories")
            {
                return parse_bool(value, config.visualization.show_trajectories);
            }
            if (key == "visualization.bounding_boxes")
            {
                return parse_bool(value, config.visualization.show_bounding_boxes);
            }
            if (key == "visualization.grid")
            {
                return parse_bool(value, config.visualization.show_grid);
            }
            if (key == "visualization.layers")
            {
                config.visualization.layers = std::string(value);
                return true;
            }
            if (key == "visualization.arrow_scope")
            {
                if (value != "all" && value != "selected")
                    return false;
                config.visualization.arrow_scope = std::string(value);
                return true;
            }
            if (key == "visualization.arrow_length_automatic")
                return parse_bool(value, config.visualization.arrow_length_automatic);
            if (key == "visualization.split_arrows")
            {
                if (value != "none" && value != "world" && value != "chosen" && value != "contact")
                    return false;
                config.visualization.split_arrows = std::string(value);
                return true;
            }
            if (key == "visualization.direction_deg")
            {
                double parsed {};
                if (!parse_double(value, parsed) || parsed < -180.0 || parsed > 180.0)
                    return false;
                config.visualization.direction_degrees = parsed;
                return true;
            }
            const auto parse_scale = [&](double& target)
            {
                double parsed {};
                if (!parse_double(value, parsed) || parsed < 0.1 || parsed > 200.0)
                    return false;
                target = parsed;
                return true;
            };
            if (key == "visualization.scale.velocity")
                return parse_scale(config.visualization.velocity_scale);
            if (key == "visualization.scale.acceleration")
                return parse_scale(config.visualization.acceleration_scale);
            if (key == "visualization.scale.force")
                return parse_scale(config.visualization.force_scale);
            if (key == "visualization.scale.momentum")
                return parse_scale(config.visualization.momentum_scale);

            if (key == "controls.wheel_zoom_factor")
            {
                double parsed {};
                if (!parse_double(value, parsed) || parsed < 1.02 || parsed > 1.50)
                    return false;
                config.controls.wheel_zoom_factor = parsed;
                return true;
            }
            if (key == "controls.pause_in_background")
                return parse_bool(value, config.controls.pause_in_background);
            if (key == "experiments.on_start")
            {
                if (value != "last" && value != "startup")
                    return false;
                config.experiments.on_start = std::string(value);
                return true;
            }
            if (key == "experiments.keep_lab_settings")
                return parse_bool(value, config.experiments.keep_lab_settings);
            if (key == "experiments.recommended_view")
                return parse_bool(value, config.experiments.recommended_view);
            if (key == "experiments.ask_predictions")
                return parse_bool(value, config.experiments.ask_predictions);
            if (key == "effects.quality")
            {
                if (value != "low" && value != "standard" && value != "high" && value != "custom")
                    return false;
                config.effects.quality = std::string(value);
                return true;
            }
            const auto effect_bool = [&](bool& target)
            {
                return parse_bool(value, target);
            };
            if (key == "effects.material_shading")
                return effect_bool(config.effects.material_shading);
            if (key == "effects.contact_shadows")
                return effect_bool(config.effects.contact_shadows);
            if (key == "effects.depth_background")
                return effect_bool(config.effects.depth_background);
            if (key == "effects.motion_trails")
                return effect_bool(config.effects.motion_trails);
            if (key == "effects.directional_blur")
                return effect_bool(config.effects.directional_blur);
            if (key == "effects.impact_flashes")
                return effect_bool(config.effects.impact_flashes);
            if (key == "effects.impact_sparks")
                return effect_bool(config.effects.impact_sparks);
            if (key == "effects.impact_dust")
                return effect_bool(config.effects.impact_dust);
            if (key == "effects.soft_deformation")
                return effect_bool(config.effects.soft_deformation);
            if (key == "effects.transitions")
                return effect_bool(config.effects.transitions);
            const auto effect_limit = [&](int& target)
            {
                int parsed {};
                if (!parse_int(value, parsed) || parsed < 8)
                    return false;
                target = parsed;
                return true;
            };
            if (key == "effects.limit.shading_body_budget")
                return effect_limit(config.effects.shading_body_budget);
            if (key == "effects.limit.contact_shadow_budget")
                return effect_limit(config.effects.contact_shadow_budget);
            if (key == "effects.limit.motion_body_budget")
                return effect_limit(config.effects.motion_body_budget);
            if (key == "effects.limit.directional_blur_budget")
                return effect_limit(config.effects.directional_blur_budget);
            if (key == "effects.limit.impact_flash_budget")
                return effect_limit(config.effects.impact_flash_budget);
            if (key == "effects.limit.spark_budget")
                return effect_limit(config.effects.spark_budget);
            if (key == "effects.limit.dust_budget")
                return effect_limit(config.effects.dust_budget);
            if (key == "effects.limit.deformation_budget")
                return effect_limit(config.effects.deformation_budget);
            if (key == "capture.area")
            {
                if (value != "stage" && value != "window")
                    return false;
                config.capture.area = std::string(value);
                return true;
            }

            if (key == "logging.level")
            {
                return parse_log_level(value, config.log_level);
            }

            key_recognised = false;
            return false;
        }

        void write_bool(std::ostream& stream, std::string_view key, bool value)
        {
            stream << key << " = " << (value ? "true" : "false") << '\n';
        }

    } // namespace

    ConfigLoadResult load_application_config(const std::filesystem::path& path)
    {
        ConfigLoadResult result;

        std::ifstream file { path };
        if (!file)
        {
            // Absent configuration is an ordinary first run, not a failure.
            return result;
        }
        result.file_found = true;

        std::string line;
        std::size_t line_number = 0;
        while (std::getline(file, line))
        {
            ++line_number;
            const auto trimmed = trim(line);
            if (trimmed.empty() || trimmed.front() == '#')
            {
                continue;
            }

            const auto separator = trimmed.find('=');
            if (separator == std::string_view::npos)
            {
                result.issues.push_back({ line_number, "line is not a key and value separated by an equals sign" });
                continue;
            }

            const auto key = trim(trimmed.substr(0, separator));
            const auto value = trim(trimmed.substr(separator + 1));

            bool key_recognised = false;
            if (!assign(result.config, key, value, key_recognised))
            {
                if (key_recognised)
                {
                    result.issues.push_back({ line_number, std::string { "value for " } + std::string { key } + " could not be read" });
                }
                else
                {
                    result.issues.push_back({ line_number, std::string { "unrecognised key " } + std::string { key } });
                }
            }
        }

        return result;
    }

    bool save_application_config(const ApplicationConfig& config, const std::filesystem::path& path)
    {
        const auto temporary = path.string() + ".tmp";
        std::ofstream file { temporary, std::ios::trunc };
        if (!file)
        {
            return false;
        }

        file.imbue(std::locale::classic());
        file << "# Rigid Bodies application configuration.\n";
        file << "# Values absent from this file keep their built-in defaults.\n\n";

        file << "config.version = 2\n\n";

        file << "window.title = " << config.window.title << '\n';
        file << "window.width = " << config.window.width << '\n';
        file << "window.height = " << config.window.height << '\n';
        write_bool(file, "window.resizable", config.window.resizable);
        write_bool(file, "window.start_maximised", config.window.start_maximised);
        write_bool(file, "window.vertical_sync", config.window.vertical_sync);
        write_bool(file, "window.custom_title_bar", config.window.custom_title_bar);
        file << '\n';

        file << "simulation.steps_per_second = " << config.simulation.steps_per_second << '\n';
        file << "simulation.substeps = " << config.simulation.substeps << '\n';
        file << "simulation.maximum_substeps_per_frame = " << config.simulation.maximum_substeps_per_frame << '\n';
        file << "simulation.time_scale = " << config.simulation.time_scale << '\n';
        write_bool(file, "simulation.start_paused", config.simulation.start_paused);
        file << "simulation.default_view_height_m = " << config.simulation.default_view_height_m << '\n';
        file << "simulation.startup_scenario = " << config.startup_scenario << '\n';
        file << '\n';

        file << "interface.backend = " << config.interface_settings.backend << '\n';
        file << "interface.theme = " << config.interface_settings.theme << '\n';
        file << "interface.performance_overlay = " << (config.interface_settings.performance_overlay ? "true" : "false") << '\n';
        file << "interface.scale = " << config.interface_settings.interface_scale << '\n';
        write_bool(file, "interface.reduce_motion", config.interface_settings.reduce_motion);
        file << "interface.units = " << config.interface_settings.units << '\n';
        write_bool(file, "interface.show_developer_overlay", config.interface_settings.show_developer_overlay);
        file << '\n';

        write_bool(file, "visualization.velocity_vectors", config.visualization.show_velocity_vectors);
        write_bool(file, "visualization.force_vectors", config.visualization.show_force_vectors);
        write_bool(file, "visualization.contact_points", config.visualization.show_contact_points);
        write_bool(file, "visualization.center_of_mass", config.visualization.show_center_of_mass);
        write_bool(file, "visualization.trajectories", config.visualization.show_trajectories);
        write_bool(file, "visualization.bounding_boxes", config.visualization.show_bounding_boxes);
        write_bool(file, "visualization.grid", config.visualization.show_grid);
        file << "visualization.layers = " << config.visualization.layers << '\n';
        file << "visualization.arrow_scope = " << config.visualization.arrow_scope << '\n';
        write_bool(file, "visualization.arrow_length_automatic", config.visualization.arrow_length_automatic);
        file << "visualization.scale.velocity = " << config.visualization.velocity_scale << '\n';
        file << "visualization.scale.acceleration = " << config.visualization.acceleration_scale << '\n';
        file << "visualization.scale.force = " << config.visualization.force_scale << '\n';
        file << "visualization.scale.momentum = " << config.visualization.momentum_scale << '\n';
        file << "visualization.split_arrows = " << config.visualization.split_arrows << '\n';
        file << "visualization.direction_deg = " << config.visualization.direction_degrees << '\n';
        file << '\n';

        file << "controls.wheel_zoom_factor = " << config.controls.wheel_zoom_factor << '\n';
        write_bool(file, "controls.pause_in_background", config.controls.pause_in_background);
        file << "experiments.on_start = " << config.experiments.on_start << '\n';
        write_bool(file, "experiments.keep_lab_settings", config.experiments.keep_lab_settings);
        write_bool(file, "experiments.recommended_view", config.experiments.recommended_view);
        write_bool(file, "experiments.ask_predictions", config.experiments.ask_predictions);
        file << "effects.quality = " << config.effects.quality << '\n';
        write_bool(file, "effects.material_shading", config.effects.material_shading);
        write_bool(file, "effects.contact_shadows", config.effects.contact_shadows);
        write_bool(file, "effects.depth_background", config.effects.depth_background);
        write_bool(file, "effects.motion_trails", config.effects.motion_trails);
        write_bool(file, "effects.directional_blur", config.effects.directional_blur);
        write_bool(file, "effects.impact_flashes", config.effects.impact_flashes);
        write_bool(file, "effects.impact_sparks", config.effects.impact_sparks);
        write_bool(file, "effects.impact_dust", config.effects.impact_dust);
        write_bool(file, "effects.soft_deformation", config.effects.soft_deformation);
        write_bool(file, "effects.transitions", config.effects.transitions);
        file << "effects.limit.shading_body_budget = " << config.effects.shading_body_budget << '\n';
        file << "effects.limit.contact_shadow_budget = " << config.effects.contact_shadow_budget << '\n';
        file << "effects.limit.motion_body_budget = " << config.effects.motion_body_budget << '\n';
        file << "effects.limit.directional_blur_budget = " << config.effects.directional_blur_budget << '\n';
        file << "effects.limit.impact_flash_budget = " << config.effects.impact_flash_budget << '\n';
        file << "effects.limit.spark_budget = " << config.effects.spark_budget << '\n';
        file << "effects.limit.dust_budget = " << config.effects.dust_budget << '\n';
        file << "effects.limit.deformation_budget = " << config.effects.deformation_budget << '\n';
        file << "capture.area = " << config.capture.area << '\n';
        file << '\n';

        file << "logging.level = " << to_string(config.log_level) << '\n';

        const auto written = file.good();
        file.close();
        if (!written)
            return false;
#if defined(_WIN32)
        if (!MoveFileExW(std::filesystem::path(temporary).c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
            return false;
#else
        std::error_code error;
        std::filesystem::rename(temporary, path, error);
        if (error)
            return false;
#endif
        return true;
    }

} // namespace rigidbodies::core
