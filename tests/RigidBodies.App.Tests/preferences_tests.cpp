#include <rigidbodies/core/application_config.hpp>

#include "test_framework.hpp"

#include <filesystem>
#include <fstream>

RIGIDBODIES_TEST("every preference round trips through the per-user config")
{
    using namespace rigidbodies;
    const auto path = std::filesystem::current_path() / "preferences_test.cfg";
    struct Cleanup
    {
        std::filesystem::path path;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove(path, error);
        }
    } cleanup { path };
    core::ApplicationConfig source;
    source.interface_settings.theme = "workbench_light";
    source.interface_settings.interface_scale = 1.75;
    source.interface_settings.units = "centimetre_gram";
    source.interface_settings.reduce_motion = true;
    source.controls.wheel_zoom_factor = 1.31;
    source.controls.pause_in_background = true;
    source.experiments.on_start = "startup";
    source.experiments.keep_lab_settings = false;
    source.simulation.start_paused = false;
    source.effects.quality = "low";
    source.effects.material_shading = false;
    source.effects.spark_budget = 64;
    source.capture.area = "window";
    source.window.custom_title_bar = false;
    RIGIDBODIES_EXPECT(core::save_application_config(source, path), "config is saved atomically");
    const auto loaded = core::load_application_config(path);
    RIGIDBODIES_EXPECT(loaded.issues.empty(), "the saved file is accepted");
    const auto& value = loaded.config;
    RIGIDBODIES_EXPECT(value.interface_settings.theme == source.interface_settings.theme && value.interface_settings.interface_scale == source.interface_settings.interface_scale && value.interface_settings.units == source.interface_settings.units && value.interface_settings.reduce_motion, "appearance and units survive");
    RIGIDBODIES_EXPECT(value.controls.wheel_zoom_factor == source.controls.wheel_zoom_factor && value.controls.pause_in_background, "control preferences survive");
    RIGIDBODIES_EXPECT(value.experiments.on_start == "startup" && !value.experiments.keep_lab_settings && !value.simulation.start_paused, "experiment opening preferences survive");
    RIGIDBODIES_EXPECT(value.effects.quality == "low" && !value.effects.material_shading && value.effects.spark_budget == 64 && value.capture.area == "window", "effects and capture preferences survive");
    RIGIDBODIES_EXPECT(!value.window.custom_title_bar, "the choice of the platform's title bar survives");
}

RIGIDBODIES_TEST("malformed and unknown preference lines retain defaults and report one issue each")
{
    using namespace rigidbodies;
    const auto path = std::filesystem::current_path() / "preferences_bad.cfg";
    struct Cleanup
    {
        std::filesystem::path path;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove(path, error);
        }
    } cleanup { path };
    {
        std::ofstream file(path);
        file << "interface.scale = enormous\nunknown.future = value\ncontrols.pause_in_background = perhaps\n";
    }
    const auto loaded = core::load_application_config(path);
    RIGIDBODIES_EXPECT(loaded.issues.size() == 3, "each malformed or unknown line produces one diagnostic");
    RIGIDBODIES_EXPECT(loaded.config.interface_settings.interface_scale == 1.0 && !loaded.config.controls.pause_in_background, "bad values retain safe defaults");
}

int main()
{
    return rigidbodies::testing::run_all();
}
