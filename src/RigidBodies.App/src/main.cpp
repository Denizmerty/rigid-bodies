#include <rigidbodies/app/application.hpp>
#include <rigidbodies/core/log.hpp>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace
{

    // The working directory is whatever launched the program, so assets are located relative to
    // the executable instead. The platform reports where that is.
    std::filesystem::path executable_directory()
    {
        if (const auto* reported = SDL_GetBasePath(); reported != nullptr)
        {
            return std::filesystem::path { reported };
        }

        std::error_code error;
        return std::filesystem::current_path(error);
    }

    // --screenshot <file.png> [--size WxH] [--display-scale S] [--ui-scale S] [--theme id]
    //     [--scenario id] [--run-frames N] [--state a,b,c]
    std::optional<rigidbodies::app::ScreenshotOptions> parse_screenshot(int argc, char** argv)
    {
        std::optional<rigidbodies::app::ScreenshotOptions> options;
        for (int index = 1; index + 1 < argc; ++index)
        {
            const std::string_view name { argv[index] };
            const std::string value { argv[index + 1] };
            if (name == "--screenshot")
            {
                if (!options)
                    options.emplace();
                options->output = std::filesystem::u8path(value);
                ++index;
            }
        }
        if (!options)
            return std::nullopt;
        for (int index = 1; index + 1 < argc; ++index)
        {
            const std::string_view name { argv[index] };
            const std::string value { argv[index + 1] };
            try
            {
                if (name == "--size")
                {
                    const auto separator = value.find('x');
                    if (separator != std::string::npos)
                    {
                        options->width = std::stoi(value.substr(0, separator));
                        options->height = std::stoi(value.substr(separator + 1));
                    }
                }
                else if (name == "--display-scale")
                    options->display_scale = std::stof(value);
                else if (name == "--ui-scale")
                    options->ui_scale = std::stod(value);
                else if (name == "--theme")
                    options->theme = value;
                else if (name == "--scenario")
                    options->scenario = value;
                else if (name == "--run-frames")
                    options->run_frames = std::stoi(value);
                else if (name == "--state")
                {
                    for (std::size_t start = 0; start <= value.size();)
                    {
                        const auto end = std::min(value.find(',', start), value.size());
                        if (end > start)
                            options->states.push_back(value.substr(start, end - start));
                        start = end + 1;
                    }
                }
                else
                    continue;
                ++index;
            }
            catch (const std::exception&)
            {
                rigidbodies::core::log_warning("ignoring the malformed screenshot option {} {}", name, value);
                ++index;
            }
        }
        return options;
    }

} // namespace

int main(int argc, char** argv)
{
    try
    {
        rigidbodies::app::Application application;
        const bool hidden_smoke = argc == 2 && std::string_view { argv[1] } == "--render-smoke";
        const bool render_benchmark = argc == 2 && (std::string_view { argv[1] } == "--render-benchmark" || std::string_view { argv[1] } == "--render-benchmark-wide");
        const bool wide_benchmark = argc == 2 && std::string_view { argv[1] } == "--render-benchmark-wide";
        const auto screenshot = parse_screenshot(argc, argv);
        if (!application.initialize(executable_directory(), hidden_smoke, render_benchmark, wide_benchmark, screenshot ? &*screenshot : nullptr))
        {
            rigidbodies::core::log_error("the application could not start");
            return EXIT_FAILURE;
        }

        return application.run();
    }
    catch (const std::exception& error)
    {
        rigidbodies::core::log_error("the application stopped with an unhandled error: {}", error.what());
        return EXIT_FAILURE;
    }
}
