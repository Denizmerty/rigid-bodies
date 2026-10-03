#pragma once

#include <rigidbodies/core/log.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace rigidbodies::core
{

    struct WindowConfig
    {
        std::string title { "Rigid Bodies" };
        int width { 1600 };
        int height { 900 };
        bool resizable { true };
        bool start_maximised { false };

        // Waiting for the display refresh keeps the presented frame whole and keeps the process
        // from spinning the processor for frames nobody sees.
        bool vertical_sync { true };

        // The interface draws the window's title bar, where the platform allows it (Windows);
        // false keeps the platform's own title bar.
        bool custom_title_bar { true };
    };

    struct SimulationConfig
    {
        // Simulation steps per second. Faster stepping buys stability in contact-heavy scenes at a
        // proportional cost in time.
        double steps_per_second { 120.0 };
        int substeps { 1 };
        int maximum_substeps_per_frame { 128 };

        double time_scale { 1.0 };
        bool start_paused { true };

        // Metres of world visible across the window height when a scenario is first framed. The
        // default frames a workbench a viewer can relate to household objects.
        double default_view_height_m { 4.0 };
    };

    struct InterfaceConfig
    {
        // Selects the backend the interface layer drives. Recognised values are "overlay" and
        // "rmlui"; an unrecognised value falls back to the overlay with a warning.
        std::string backend { "rmlui" };

        std::string theme { "workbench_dark" };

        // Additional scale applied on top of the display scale, for viewers who want larger
        // controls than the system reports.
        double interface_scale { 1.0 };

        bool reduce_motion { false };
        std::string units { "si" };

        bool show_developer_overlay { false };
        bool performance_overlay { false };
    };

    struct VisualizationConfig
    {
        bool show_velocity_vectors { true };
        bool show_force_vectors { false };
        bool show_contact_points { true };
        bool show_center_of_mass { true };
        bool show_trajectories { false };
        bool show_bounding_boxes { false };
        bool show_grid { true };

        // Version 2 stores the complete learner view. An empty layer list means the legacy
        // booleans above are authoritative, preserving version-1 files.
        std::string layers;
        std::string arrow_scope { "all" };
        bool arrow_length_automatic { false };
        double velocity_scale { 24.0 }, acceleration_scale { 6.0 }, force_scale { 6.0 }, momentum_scale { 30.0 };
        std::string split_arrows { "none" };
        double direction_degrees { 0.0 };
    };

    struct ControlsConfig
    {
        double wheel_zoom_factor { 1.12 };
        bool pause_in_background { false };
    };

    struct ExperimentsConfig
    {
        std::string on_start { "last" };
        bool keep_lab_settings { true };
        bool recommended_view { true };
        bool ask_predictions { true };
    };

    struct EffectsConfig
    {
        std::string quality { "standard" };
        bool material_shading { true }, contact_shadows { true }, depth_background { true };
        bool motion_trails { true }, directional_blur { true }, impact_flashes { true };
        bool impact_sparks { true }, impact_dust { true }, soft_deformation { true }, transitions { true };
        int shading_body_budget { 128 }, contact_shadow_budget { 96 }, motion_body_budget { 64 };
        int directional_blur_budget { 64 }, impact_flash_budget { 32 }, spark_budget { 128 };
        int dust_budget { 64 }, deformation_budget { 32 };
    };

    struct CaptureConfig
    {
        std::string area { "stage" };
    };

    // Everything the application reads before it opens a window. Configuration is a plain value
    // with working defaults, so a missing or partly written file never prevents a start: unknown
    // keys are reported and skipped, and anything absent keeps its default.
    struct ApplicationConfig
    {
        int version { 2 };
        WindowConfig window;
        SimulationConfig simulation;
        InterfaceConfig interface_settings;
        VisualizationConfig visualization;
        ControlsConfig controls;
        ExperimentsConfig experiments;
        EffectsConfig effects;
        CaptureConfig capture;

        LogLevel log_level { LogLevel::info };

        std::string startup_scenario { "free_fall" };
    };

    struct ConfigIssue
    {
        std::size_t line_number { 0 };
        std::string text;
    };

    struct ConfigLoadResult
    {
        ApplicationConfig config;
        std::vector<ConfigIssue> issues;

        // False when the file was absent, in which case the defaults are returned unchanged.
        bool file_found { false };
    };

    // Reads the flat `section.key = value` format used by config/application.cfg. Blank lines and
    // lines beginning with a hash are ignored.
    [[nodiscard]] ConfigLoadResult load_application_config(const std::filesystem::path& path);

    // Writes the configuration back in the same format, preserving the documented key order.
    bool save_application_config(const ApplicationConfig& config, const std::filesystem::path& path);

} // namespace rigidbodies::core
