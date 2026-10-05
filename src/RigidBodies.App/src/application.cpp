#include <rigidbodies/app/application.hpp>
#include <rigidbodies/app/content_files.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <rigidbodies/render/gl_render_device.hpp>

#include <rigidbodies/project_identity.hpp>

#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <vector>
#include <fstream>
#include <sstream>

namespace
{
    // The stage the reader sees: below the toolbar, above the status line and any sheet across the
    // stage's foot.
    rigidbodies::render::ScreenRect visible_stage(const rigidbodies::ui::LayoutResult& layout)
    {
        auto bottom = layout.stage.maximum.y;
        for (const auto& region : layout.regions)
            if (region.id == rigidbodies::ui::RegionId::status_line || (region.id == rigidbodies::ui::RegionId::measure_drawer && region.presentation == rigidbodies::ui::RegionPresentation::sheet))
                bottom = std::min(bottom, region.bounds.minimum.y);
        return { layout.stage.minimum.x, layout.stage.minimum.y, layout.stage.width(), std::max(0.0, bottom - layout.stage.minimum.y) };
    }

    SDL_Cursor* create_glyph_cursor(rigidbodies::ui::CursorShape shape)
    {
        constexpr int extent = 24;
        auto* surface = SDL_CreateSurface(extent, extent, SDL_PIXELFORMAT_RGBA32);
        if (!surface)
            return nullptr;
        (void)SDL_FillSurfaceRect(surface, nullptr, SDL_MapSurfaceRGBA(surface, 0, 0, 0, 0));

        const auto pixel = [surface](int x, int y, Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255)
        {
            if (x >= 0 && x < extent && y >= 0 && y < extent)
                (void)SDL_WriteSurfacePixel(surface, x, y, r, g, b, a);
        };
        const auto line = [&pixel](int x0, int y0, int x1, int y1, int width, Uint8 r, Uint8 g, Uint8 b)
        {
            const int dx = std::abs(x1 - x0);
            const int sx = x0 < x1 ? 1 : -1;
            const int dy = -std::abs(y1 - y0);
            const int sy = y0 < y1 ? 1 : -1;
            int error = dx + dy;
            for (;;)
            {
                const int radius = width / 2;
                for (int py = -radius; py <= radius; ++py)
                    for (int px = -radius; px <= radius; ++px)
                        pixel(x0 + px, y0 + py, r, g, b);
                if (x0 == x1 && y0 == y1)
                    break;
                const int doubled = error * 2;
                if (doubled >= dy)
                {
                    error += dy;
                    x0 += sx;
                }
                if (doubled <= dx)
                {
                    error += dx;
                    y0 += sy;
                }
            }
        };
        const auto glyph_line = [&line](int x0, int y0, int x1, int y1, int width = 1)
        {
            line(x0, y0, x1, y1, width + 2, 13, 27, 42);
            line(x0, y0, x1, y1, width, 248, 251, 255);
        };

        int hot_x = 11;
        int hot_y = 10;
        if (shape == rigidbodies::ui::CursorShape::open_hand)
        {
            glyph_line(7, 4, 7, 12);
            glyph_line(10, 2, 10, 11);
            glyph_line(13, 3, 13, 11);
            glyph_line(16, 5, 16, 13);
            glyph_line(5, 10, 9, 16);
            glyph_line(16, 10, 16, 15);
            glyph_line(9, 17, 15, 17, 2);
            glyph_line(8, 13, 8, 17);
        }
        else if (shape == rigidbodies::ui::CursorShape::grab)
        {
            glyph_line(7, 7, 7, 14, 2);
            glyph_line(10, 6, 10, 13, 2);
            glyph_line(13, 7, 13, 13, 2);
            glyph_line(16, 8, 16, 14, 2);
            glyph_line(6, 13, 9, 18, 2);
            glyph_line(9, 18, 15, 18, 2);
            glyph_line(16, 13, 15, 18, 2);
        }
        else if (shape == rigidbodies::ui::CursorShape::pen)
        {
            hot_x = 4;
            hot_y = 20;
            glyph_line(5, 18, 17, 6, 3);
            glyph_line(15, 5, 18, 8, 2);
            glyph_line(4, 20, 7, 18, 2);
        }
        else
        {
            SDL_DestroySurface(surface);
            return nullptr;
        }

        auto* cursor = SDL_CreateColorCursor(surface, hot_x, hot_y);
        SDL_DestroySurface(surface);
        return cursor;
    }
}

namespace rigidbodies::app
{

    Application::Application() = default;

    Application::~Application()
    {
        shutdown();
    }

    bool Application::initialize(const std::filesystem::path& executable_directory, bool hidden_smoke,
        bool render_benchmark, bool wide_benchmark, const ScreenshotOptions* screenshot)
    {
        if (screenshot)
        {
            screenshot_ = *screenshot;
            display_scale_override_ = screenshot->display_scale;
        }
        hidden_smoke_ = hidden_smoke || render_benchmark || screenshot_.has_value();
        render_benchmark_ = render_benchmark;
        paths_ = core::ResourcePaths::discover(executable_directory);
        (void)paths_.ensure_user_data_root();
        frame_capture_.configure(paths_.user_data("captures"));
        std::string catalogue_error;
        if (!physics::initialize_scenario_catalogue(paths_.asset("scenarios"), catalogue_error))
        {
            core::log_error("the scenario catalogue could not be loaded: {}", catalogue_error);
            return false;
        }

        // A screenshot reads only the shipped configuration so the image never depends on, or
        // changes, the user's own preferences.
        const auto config_path = screenshot_ ? std::optional<std::filesystem::path> { paths_.config("application.cfg") } : paths_.find_application_config();
        if (config_path.has_value() && std::filesystem::is_regular_file(*config_path))
        {
            auto loaded = core::load_application_config(*config_path);
            config_ = loaded.config;
            for (const auto& issue : loaded.issues)
            {
                // Configuration problems are reported and stepped over rather than treated as
                // fatal, so that a stale or hand-edited file never stops the application opening.
                core::log_warning("{}:{}: {}", config_path->string(), issue.line_number, issue.text);
            }
            core::log_info("configuration read from {}", config_path->string());
        }
        else
        {
            core::log_info("no configuration file found; built-in defaults are in use");
        }

        core::set_active_log_level(config_.log_level);
        if (render_benchmark_)
        {
            config_.window.vertical_sync = false;
            if (wide_benchmark)
            {
                config_.window.width = 1920;
                config_.window.height = 1010;
                config_.interface_settings.interface_scale = 2.0 / 3.0;
            }
        }

        if (!Window::initialize_platform())
        {
            return false;
        }
        if (screenshot_)
        {
            // The drawable is corrected after creation, because whether a hidden window's pixels
            // follow the monitor density depends on the platform.
            config_.window.width = std::max(320, screenshot_->width);
            config_.window.height = std::max(240, screenshot_->height);
            config_.window.vertical_sync = false;
            config_.window.start_maximised = false;
            if (screenshot_->ui_scale > 0.0)
                config_.interface_settings.interface_scale = screenshot_->ui_scale;
            if (!screenshot_->theme.empty())
                config_.interface_settings.theme = screenshot_->theme;
        }

        // The interface draws the window's title bar where the platform allows it and the
        // configured interface can draw one; the overlay fallback draws no window controls.
        const auto custom_title_bar = config_.window.custom_title_bar && WindowChrome::supported() && ui::backend_kind_from_name(config_.interface_settings.backend) == ui::UiBackendKind::document;
        window_ = Window::create(config_.window, true, hidden_smoke_, custom_title_bar);
        if (window_)
            device_ = render::GlRenderDevice::create(window_->native(), config_.window.vertical_sync);
        if (!device_)
        {
            // An OpenGL window can constrain SDL's available renderer backends. Recreate it
            // without that flag before asking SDL to choose the portable fallback.
            window_.reset();
            core::log_warning("the modern graphics backend is unavailable; using the SDL fallback");
            window_ = Window::create(config_.window, false, hidden_smoke_, custom_title_bar);
            if (window_)
                device_ = render::SdlRenderDevice::create(window_->native(), config_.window.vertical_sync);
        }
        if (!device_)
        {
            return false;
        }
        if (screenshot_)
        {
            int window_width = 0, window_height = 0;
            SDL_GetWindowSize(window_->native(), &window_width, &window_height);
            const auto drawable = device_->drawable_size();
            if (drawable.width > 0 && drawable.height > 0 && (drawable.width != screenshot_->width || drawable.height != screenshot_->height))
            {
                SDL_SetWindowSize(window_->native(), static_cast<int>(std::lround(screenshot_->width * static_cast<double>(window_width) / drawable.width)), static_cast<int>(std::lround(screenshot_->height * static_cast<double>(window_height) / drawable.height)));
                SDL_SyncWindow(window_->native());
            }
        }
        window_->set_title(core::substitute("{} {}", rigidbodies::display_name, rigidbodies::project_version));
        if (!device_->load_font(paths_.asset_root() / "fonts" / "Inter-Medium.ttf"))
        {
            core::log_error("the packaged scene font could not be loaded");
            return false;
        }

#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        developer_overlay_ = std::make_unique<DeveloperOverlay>();
        auto* sdl_device = dynamic_cast<render::SdlRenderDevice*>(device_.get());
        auto* gl_device = dynamic_cast<render::GlRenderDevice*>(device_.get());
        const bool overlay_ready = gl_device ? developer_overlay_->initialize_opengl(window_->native(), gl_device->native_context()) : sdl_device && developer_overlay_->initialize(window_->native(), sdl_device->native_renderer());
        if (!overlay_ready)
            return false;
        developer_overlay_->set_visible((hidden_smoke_ && !render_benchmark_) || config_.interface_settings.show_developer_overlay);
#endif

        interface_.set_scale(display_scale() * static_cast<float>(config_.interface_settings.interface_scale));
        interface_.set_theme(render::theme_by_name(config_.interface_settings.theme.c_str()));
        interface_.view_state().set_surface_open("performance.overlay", config_.interface_settings.performance_overlay);
        if (!interface_.initialize(ui::backend_kind_from_name(config_.interface_settings.backend), paths_.asset_root()))
        {
            return false;
        }
        window_frame_.controls = custom_title_bar && interface_.draws_window_controls();
        if (custom_title_bar && !window_frame_.controls)
            window_->set_bordered(true);
        if (!hidden_smoke_)
        {
            if (window_frame_.controls)
                install_window_chrome();
            show_window_pending_ = custom_title_bar;
            watching_events_ = SDL_AddEventWatch(&Application::watch_events, this);
        }
        if (!screenshot_)
        {
            std::ifstream state_file(paths_.user_data("interface-state.cfg"));
            if (state_file)
            {
                std::ostringstream contents;
                contents << state_file.rdbuf();
                for (const auto& issue : interface_.view_state().deserialize(contents.str()))
                    core::log_warning("interface-state.cfg: {}", issue);
            }
        }
        interface_.set_clipboard_hooks([]
            {
                char* value = SDL_GetClipboardText();
                std::string result = value ? value : "";
                SDL_free(value);
                return result;
            },
            [](std::string_view value)
            {
                const std::string copy { value };
                (void)SDL_SetClipboardText(copy.c_str());
            });

        session_.configure(config_);
        if (render_benchmark_ && wide_benchmark)
            session_.ui_scale_ = 2.0 / 3.0;
        if (screenshot_ && !screenshot_->scenario.empty() && !session_.load_scenario(screenshot_->scenario))
            core::log_warning("screenshot: the scenario \"{}\" is not in the catalogue", screenshot_->scenario);
        if (render_benchmark_)
        {
            std::size_t dynamic_count = 0;
            for (const auto id : session_.world().body_ids())
                if (const auto* body = session_.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                    ++dynamic_count;
            for (; dynamic_count < 32; ++dynamic_count)
            {
                physics::BodyDefinition definition;
                definition.type = physics::BodyType::dynamic_body;
                definition.name = "Benchmark object " + std::to_string(dynamic_count + 1);
                definition.position_m = { -3.0 + static_cast<double>(dynamic_count % 8) * 0.8,
                    1.0 + static_cast<double>(dynamic_count / 8) * 0.8 };
                physics::Collider collider;
                collider.shape = physics::make_circle(0.18);
                collider.material = physics::materials::oak_wood();
                definition.colliders.push_back(collider);
                session_.world().create_body(definition);
            }
            for (const auto id : session_.world().body_ids())
                if (const auto* body = session_.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                {
                    session_.set_selection(id);
                    break;
                }
            ui::UiCommand everything;
            everything.kind = ui::UiCommandKind::set_layer_mask;
            everything.id = "all";
            session_.apply(everything);
            ui::UiCommand play;
            play.kind = ui::UiCommandKind::toggle_pause;
            session_.apply(play);
            interface_.view_state().set_surface_open("inspector.open", true);
            interface_.view_state().set_surface_open("measure.open", true);
            interface_.view_state().set_surface_open("performance.overlay", true);
            interface_.view_state().set_active_tab("measure.header.tabs", "graph");
            session_.notify(ui::Severity::info, "Interface benchmark", "benchmark");
        }
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        session_.world().set_profiling_enabled(true);
#endif
        session_.render_backend_name_ = std::string { device_->backend_name() };
        session_.interface_backend_name_ = std::string { interface_.backend_name() };

        sync_viewport();
        const auto initial_scale = display_scale() * static_cast<float>(config_.interface_settings.interface_scale);
        ui::LayoutInput initial_layout_input;
        initial_layout_input.viewport = device_->drawable_size();
        initial_layout_input.scale = initial_scale;
        const auto initial_layout = ui::compute_layout(initial_layout_input);
        session_.set_focus_rect({ initial_layout.focus.minimum.x, initial_layout.focus.minimum.y, initial_layout.focus.width(), initial_layout.focus.height() });
        session_.set_visible_stage_rect(visible_stage(initial_layout));
        session_.frame_subject();
        observed_preference_signature_ = preference_signature();

        core::log_info("{} {} ready", rigidbodies::display_name, rigidbodies::project_version);
        return true;
    }

    void Application::shutdown()
    {
        // The platform calls back into the interface through these, so they go first. The window
        // is hidden before its frame goes back to SDL, which would otherwise show the borders
        // around it as undrawn strips while the rest shuts down.
        if (watching_events_)
        {
            SDL_RemoveEventWatch(&Application::watch_events, this);
            watching_events_ = false;
        }
        if (window_chrome_ && window_)
            window_->hide();
        window_chrome_.reset();
        persist_preferences();
        for (auto*& cached : cursor_cache_)
            if (cached)
            {
                SDL_DestroyCursor(cached);
                cached = nullptr;
            }
        cursor_ = nullptr;
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        developer_overlay_.reset();
#endif
        interface_.shutdown();
        device_.reset();
        window_.reset();
        Window::shutdown_platform();
    }

    std::string Application::preference_signature() const
    {
        std::ostringstream signature;
        signature << session_.theme_id_ << '|' << session_.ui_scale_ << '|' << session_.reduce_motion_ << '|'
                  << static_cast<int>(session_.scene_settings_.display_units) << '|' << session_.camera_zoom_sensitivity_ << '|'
                  << session_.pause_in_background_ << '|' << session_.start_paused_ << '|' << session_.experiments_on_start_ << '|'
                  << session_.keep_lab_settings_ << '|' << session_.effects_quality_ << '|' << session_.capture_area_ << '|'
                  << session_.scene_settings_.material_shading << session_.scene_settings_.contact_shadows << session_.scene_settings_.depth_background
                  << session_.scene_settings_.motion_trails << session_.scene_settings_.directional_blur << session_.scene_settings_.impact_flashes
                  << session_.scene_settings_.impact_sparks << session_.scene_settings_.impact_dust << session_.scene_settings_.soft_deformation
                  << session_.scene_settings_.transitions << '|'
                  << session_.scene_settings_.shading_body_budget << '|' << session_.scene_settings_.contact_shadow_budget << '|'
                  << session_.scene_settings_.motion_body_budget << '|' << session_.scene_settings_.directional_blur_budget << '|'
                  << session_.scene_settings_.impact_flash_budget << '|' << session_.scene_settings_.spark_budget << '|'
                  << session_.scene_settings_.dust_budget << '|' << session_.scene_settings_.deformation_budget << '|'
                  << interface_.view_state().serialize();
        return signature.str();
    }

    void Application::persist_preferences()
    {
        if (render_benchmark_ || screenshot_)
            return;
        if (!paths_.user_data_root().empty() && paths_.ensure_user_data_root())
        {
            config_.interface_settings.theme = session_.theme_id_;
            config_.interface_settings.interface_scale = session_.ui_scale_;
            config_.interface_settings.reduce_motion = session_.reduce_motion_;
            config_.interface_settings.performance_overlay = interface_.view_state().surface_open("performance.overlay", false);
            config_.interface_settings.units = session_.scene_settings_.display_units == core::DisplayUnits::si ? "si" : "centimetre_gram";
            config_.controls.wheel_zoom_factor = session_.camera_zoom_sensitivity_;
            config_.controls.pause_in_background = session_.pause_in_background_;
            config_.simulation.start_paused = session_.start_paused_;
            config_.experiments.on_start = session_.experiments_on_start_;
            config_.experiments.keep_lab_settings = session_.keep_lab_settings_;
            config_.effects.quality = session_.effects_quality_;
            config_.capture.area = session_.capture_area_;
            config_.visualization.arrow_scope = session_.scene_settings_.vectors_selected_only ? "selected" : "all";
            config_.visualization.arrow_length_automatic = session_.scene_settings_.vector_scales.automatic;
            config_.visualization.velocity_scale = session_.scene_settings_.vector_scales.velocity;
            config_.visualization.acceleration_scale = session_.scene_settings_.vector_scales.acceleration;
            config_.visualization.force_scale = session_.scene_settings_.vector_scales.force;
            config_.visualization.momentum_scale = session_.scene_settings_.vector_scales.momentum;
            config_.visualization.split_arrows = session_.scene_settings_.vector_components == render::VectorComponents::world_axes ? "world" : session_.scene_settings_.vector_components == render::VectorComponents::custom_axes ? "chosen"
                : session_.scene_settings_.vector_components == render::VectorComponents::contact_axes                                                                                                                              ? "contact"
                                                                                                                                                                                                                                    : "none";
            config_.visualization.direction_degrees = math::radians_to_degrees(session_.scene_settings_.component_angle_rad);
            config_.effects.material_shading = session_.scene_settings_.material_shading;
            config_.effects.contact_shadows = session_.scene_settings_.contact_shadows;
            config_.effects.depth_background = session_.scene_settings_.depth_background;
            config_.effects.motion_trails = session_.scene_settings_.motion_trails;
            config_.effects.directional_blur = session_.scene_settings_.directional_blur;
            config_.effects.impact_flashes = session_.scene_settings_.impact_flashes;
            config_.effects.impact_sparks = session_.scene_settings_.impact_sparks;
            config_.effects.impact_dust = session_.scene_settings_.impact_dust;
            config_.effects.soft_deformation = session_.scene_settings_.soft_deformation;
            config_.effects.transitions = session_.scene_settings_.transitions;
            config_.effects.shading_body_budget = static_cast<int>(session_.scene_settings_.shading_body_budget);
            config_.effects.contact_shadow_budget = static_cast<int>(session_.scene_settings_.contact_shadow_budget);
            config_.effects.motion_body_budget = static_cast<int>(session_.scene_settings_.motion_body_budget);
            config_.effects.directional_blur_budget = static_cast<int>(session_.scene_settings_.directional_blur_budget);
            config_.effects.impact_flash_budget = static_cast<int>(session_.scene_settings_.impact_flash_budget);
            config_.effects.spark_budget = static_cast<int>(session_.scene_settings_.spark_budget);
            config_.effects.dust_budget = static_cast<int>(session_.scene_settings_.dust_budget);
            config_.effects.deformation_budget = static_cast<int>(session_.scene_settings_.deformation_budget);
            (void)core::save_application_config(config_, paths_.user_data("application.cfg"));
            std::ofstream state_file(paths_.user_data("interface-state.cfg"), std::ios::trunc);
            if (state_file)
                state_file << interface_.view_state().serialize();
        }
    }

    void Application::sync_viewport()
    {
        const auto size = device_->drawable_size();
        session_.set_viewport(size);
        int window_width = 0, window_height = 0;
        SDL_GetWindowSize(window_->native(), &window_width, &window_height);
        input_pixel_scale_ = { window_width > 0 ? static_cast<double>(size.width) / window_width : 1.0,
            window_height > 0 ? static_cast<double>(size.height) / window_height : 1.0 };
        interface_.set_scale(display_scale() * static_cast<float>(session_.build_model().ui_scale));
        session_.scene_settings().display_scale = display_scale();
    }

    int Application::run()
    {
        if (!window_ || !device_)
        {
            core::log_error("the application cannot run before it has been initialised");
            return 1;
        }

        running_ = true;
        if (screenshot_)
            return run_screenshot();
        frame_start_s_ = Window::elapsed_seconds();
        int smoke_frames = 0;
        double benchmark_start_s = 0.0;
        std::vector<double> benchmark_frame_ms;
        std::vector<double> benchmark_interface_ms;
        // Simulation, scene recording, device submission and presentation, in frame order.
        std::array<std::vector<double>, 4> benchmark_phase_ms;

        while (running_)
        {
            const auto now = Window::elapsed_seconds();
            // TimeStepper owns the stall limit and accounts for skipped simulation time.
            const auto frame_time = now - frame_start_s_;
            frame_start_s_ = now;
            last_frame_time_s_ = frame_time;

#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
            const auto input_start = Window::elapsed_seconds();
#endif
            process_events();
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
            frame_timings_.input_s = Window::elapsed_seconds() - input_start;
#endif
            if (!running_)
            {
                break;
            }

            update(frame_time);
            present();
            if (show_window_pending_)
            {
                show_window_pending_ = false;
                window_->show();
            }
            if (hidden_smoke_)
            {
                if (!device_->last_error().empty())
                {
                    core::log_error("hidden render smoke failed: {}", device_->last_error());
                    return 1;
                }
                ++smoke_frames;
                if (render_benchmark_ && smoke_frames == 10)
                    benchmark_start_s = Window::elapsed_seconds();
                if (render_benchmark_ && smoke_frames > 10)
                {
                    benchmark_frame_ms.push_back((Window::elapsed_seconds() - now) * 1000.0);
                    benchmark_interface_ms.push_back(previous_interface_s_ * 1000.0);
                    benchmark_phase_ms[0].push_back(previous_simulation_s_ * 1000.0);
                    benchmark_phase_ms[1].push_back(previous_scene_s_ * 1000.0);
                    benchmark_phase_ms[2].push_back(previous_submit_s_ * 1000.0);
                    benchmark_phase_ms[3].push_back(previous_present_s_ * 1000.0);
                }
                const auto benchmark_complete = render_benchmark_ && smoke_frames > 10 && Window::elapsed_seconds() - benchmark_start_s >= 30.0;
                if ((!render_benchmark_ && smoke_frames == 6) || benchmark_complete)
                {
                    if (render_benchmark_)
                    {
                        const auto mean = std::accumulate(benchmark_frame_ms.begin(), benchmark_frame_ms.end(), 0.0) / static_cast<double>(benchmark_frame_ms.size());
                        std::sort(benchmark_frame_ms.begin(), benchmark_frame_ms.end());
                        const auto percentile = benchmark_frame_ms[static_cast<std::size_t>(std::ceil(static_cast<double>(benchmark_frame_ms.size()) * 0.95)) - 1];
                        std::sort(benchmark_interface_ms.begin(), benchmark_interface_ms.end());
                        const auto interface_median = benchmark_interface_ms[benchmark_interface_ms.size() / 2];
                        const auto interface_percentile = benchmark_interface_ms[static_cast<std::size_t>(std::ceil(static_cast<double>(benchmark_interface_ms.size()) * 0.95)) - 1];
                        const auto extent = device_->drawable_size();
                        const auto* gl_device = dynamic_cast<render::GlRenderDevice*>(device_.get());
                        core::log_info("hidden render benchmark: {} frames after 10 warmup, {}x{}, {}x MSAA, vsync off, mean {} ms, p95 {} ms, interface median {} ms, interface p95 {} ms, backend {}", benchmark_frame_ms.size(), extent.width, extent.height, gl_device ? gl_device->multisample_count() : 1, mean, percentile, interface_median, interface_percentile, device_->backend_name());
                        for (auto& phase : benchmark_phase_ms)
                            std::sort(phase.begin(), phase.end());
                        const auto median = [](const std::vector<double>& sorted)
                        {
                            return sorted[sorted.size() / 2];
                        };
                        core::log_info("hidden render benchmark phase medians: simulation {} ms, scene {} ms, interface {} ms, submit {} ms, present {} ms", median(benchmark_phase_ms[0]), median(benchmark_phase_ms[1]), interface_median, median(benchmark_phase_ms[2]), median(benchmark_phase_ms[3]));
                    }
                    else
                        core::log_info("hidden render smoke completed six scene/interface frames on {}", device_->backend_name());
                    running_ = false;
                }
            }
        }

        return 0;
    }

    void Application::process_events()
    {
        // ImGui emits commands while constructing the preceding frame. Apply them before any
        // newer keyboard event, so that Undo reverses the latest visible edit.
        apply_pending_commands();
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            auto translated = input_.translate(event);
            bool developer_consumed = false;
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
            developer_consumed = developer_overlay_ && developer_overlay_->handle_event(event);
#endif

            if (translated.quit_requested)
            {
                running_ = false;
                return;
            }
            if (translated.close_requested)
            {
                request_close();
                continue;
            }

            if (translated.viewport_changed)
            {
                sync_viewport();
            }

            if (translated.ui_event.has_value())
            {
                auto& input = *translated.ui_event;
                if (input.kind == ui::UiEventKind::focus_lost)
                    session_.set_window_backgrounded(true);
                else if (input.kind == ui::UiEventKind::focus_gained)
                    session_.set_window_backgrounded(false);
                input.pointer_px.x *= input_pixel_scale_.x;
                input.pointer_px.y *= input_pixel_scale_.y;
                input.pointer_delta_px.x *= input_pixel_scale_.x;
                input.pointer_delta_px.y *= input_pixel_scale_.y;
                input.logical_pixel_scale = display_scale() * session_.build_model().ui_scale;
                const auto model = session_.build_model();
                InputContext context;
                context.focus = interface_.focus_owner();
                context.interface_escape = interface_.escape_target();
                context.interface_modal = !interface_.view_state().sheets().empty() || !interface_.view_state().transients().empty() || model.confirmation.has_value();
                context.developer_captured = developer_consumed;
                context.pointer_over_surface = interface_.surface_at(input.pointer_px);
                context.pointer_captured_by_interface = interface_.pointer_captured();
                context.gesture_active = session_.interaction_active() || session_.shape_editor().has_pointer_capture();
                context.draw_active = model.shape_editor_active;
                context.draft_node_selected = model.shape_node_selected;
                context.draft_nodes = model.shape_node_count;
                context.select_tool = session_.interaction_mode() == InteractionMode::select;
                context.has_selection = !model.selected_bodies.empty();
                context.keyboard_mode = context.focus == ui::FocusOwner::keyboard_control;
                context.open_list = context.focus == ui::FocusOwner::transient;
                context.present = interface_.view_state().present().mode;
                context.present_locked = interface_.view_state().present().lock;
                context.pick_surface_armed = interface_.view_state().pick_surface_armed();
                if (input.kind == ui::UiEventKind::pointer_down && !context.gesture_active && interface_.dismiss_menus_at(input.pointer_px))
                {
                    sync_text_input_and_cursor();
                    continue;
                }
                if (context.pick_surface_armed && input.kind == ui::UiEventKind::pointer_down && input.button == ui::PointerButton::primary && !context.pointer_over_surface)
                {
                    (void)session_.pick_surface_at(input.pointer_px);
                    interface_.view_state().set_pick_surface_armed(false);
                    sync_text_input_and_cursor();
                    continue;
                }
                const auto action = translated.action == AppAction::none ? std::optional<AppAction> {} : std::optional<AppAction> { translated.action };
                const auto decision = route_event(input, action, context);
                bool interface_consumed = false;
                if (decision.to_interface)
                    interface_consumed = interface_.handle_event(input);
                else if (decision.to_scene && input.kind == ui::UiEventKind::pointer_down)
                    interface_.release_focus();
                if (decision.to_scene)
                    (void)session_.handle_scene_event(input, interface_consumed);
                switch (decision.escape)
                {
                case EscapeStep::cancel_gesture:
                    session_.cancel_gesture();
                    interface_.view_state().set_pick_surface_armed(false);
                    break;
                case EscapeStep::draw_deselect_node:
                    session_.deselect_draft_node();
                    break;
                case EscapeStep::draw_confirm_discard:
                    interface_.view_state().set_section_open("draw.discard_prompt", true);
                    break;
                case EscapeStep::draw_discard:
                {
                    ui::UiCommand command;
                    command.kind = ui::UiCommandKind::cancel_shape_outline;
                    apply_command(command);
                    break;
                }
                case EscapeStep::tool_to_select:
                    handle_action(AppAction::select_mode);
                    break;
                case EscapeStep::clear_selection:
                {
                    ui::UiCommand command;
                    command.kind = ui::UiCommandKind::clear_selection;
                    apply_command(command);
                    break;
                }
                case EscapeStep::close_transient:
                    interface_.view_state().set_section_open("draw.discard_prompt", false);
                    interface_.request(ui::ViewRequest::close_top_surface);
                    break;
                case EscapeStep::close_sheet:
                    interface_.request(ui::ViewRequest::close_top_surface);
                    break;
                case EscapeStep::leave_present:
                    interface_.request(ui::ViewRequest::toggle_present);
                    break;
                default:
                    break;
                }
                if (decision.action)
                    handle_action(*decision.action);
                sync_text_input_and_cursor();
            }

            // Document controls emit during event handling. Do not leave an earlier button
            // click queued behind a later keyboard action in the same platform event batch.
            apply_pending_commands();
        }
        apply_window_control_clicks();
    }

    void Application::sync_text_input_and_cursor()
    {
        const auto wants_text = interface_.wants_text_input();
        if (wants_text != text_input_active_)
        {
            if (wants_text)
                (void)SDL_StartTextInput(window_->native());
            else
                (void)SDL_StopTextInput(window_->native());
            text_input_active_ = wants_text;
        }
        if (wants_text)
        {
            const auto area = interface_.text_input_area();
            SDL_Rect rectangle { static_cast<int>(area.minimum.x), static_cast<int>(area.minimum.y), std::max(1, static_cast<int>(area.width())), std::max(1, static_cast<int>(area.height())) };
            (void)SDL_SetTextInputArea(window_->native(), &rectangle, 0);
        }
        auto requested = interface_.cursor();
        if (requested == ui::CursorShape::arrow && !interface_.pointer_captured())
            requested = session_.scene_cursor();
        if (requested == cursor_shape_ && cursor_)
            return;
        const auto cursor_index = static_cast<std::size_t>(requested);
        if (cursor_index < cursor_cache_.size() && cursor_cache_[cursor_index])
        {
            cursor_ = cursor_cache_[cursor_index];
            SDL_SetCursor(cursor_);
            cursor_shape_ = requested;
            return;
        }
        cursor_ = nullptr;
        if (requested == ui::CursorShape::open_hand || requested == ui::CursorShape::grab || requested == ui::CursorShape::pen)
            cursor_ = create_glyph_cursor(requested);

        SDL_SystemCursor system = SDL_SYSTEM_CURSOR_DEFAULT;
        switch (requested)
        {
        case ui::CursorShape::pointer:
            system = SDL_SYSTEM_CURSOR_POINTER;
            break;
        case ui::CursorShape::text:
            system = SDL_SYSTEM_CURSOR_TEXT;
            break;
        case ui::CursorShape::crosshair:
            system = SDL_SYSTEM_CURSOR_CROSSHAIR;
            break;
        case ui::CursorShape::move:
            system = SDL_SYSTEM_CURSOR_MOVE;
            break;
        case ui::CursorShape::resize_ns:
            system = SDL_SYSTEM_CURSOR_NS_RESIZE;
            break;
        case ui::CursorShape::resize_ew:
            system = SDL_SYSTEM_CURSOR_EW_RESIZE;
            break;
        case ui::CursorShape::not_allowed:
            system = SDL_SYSTEM_CURSOR_NOT_ALLOWED;
            break;
        default:
            break;
        }
        if (!cursor_)
            cursor_ = SDL_CreateSystemCursor(system);
        if (cursor_)
        {
            if (cursor_index < cursor_cache_.size())
                cursor_cache_[cursor_index] = cursor_;
            SDL_SetCursor(cursor_);
        }
        cursor_shape_ = requested;
    }

    void Application::handle_action(AppAction action)
    {
        if (action == AppAction::toggle_help)
        {
            interface_.request(ui::ViewRequest::open_shortcuts);
            return;
        }
        switch (action)
        {
        case AppAction::open_library:
            interface_.request(ui::ViewRequest::open_library);
            return;
        case AppAction::toggle_show:
            interface_.request(ui::ViewRequest::toggle_show);
            return;
        case AppAction::toggle_measure:
            interface_.request(ui::ViewRequest::toggle_measure);
            return;
        case AppAction::open_world:
            interface_.request(ui::ViewRequest::open_world);
            return;
        case AppAction::toggle_guide:
            interface_.request(ui::ViewRequest::toggle_guide);
            return;
        case AppAction::toggle_inspector_pin:
            interface_.request(ui::ViewRequest::toggle_inspector_pin);
            return;
        case AppAction::open_main_menu:
            interface_.request(ui::ViewRequest::open_main_menu);
            return;
        case AppAction::open_preferences:
            interface_.request(ui::ViewRequest::open_preferences);
            return;
        case AppAction::open_add_menu:
            interface_.request(ui::ViewRequest::open_add_menu);
            return;
        case AppAction::toggle_present:
            interface_.request(ui::ViewRequest::toggle_present);
            return;
        case AppAction::open_command_search:
            interface_.request(ui::ViewRequest::open_command_search);
            return;
        case AppAction::open_context_menu:
            session_.request_context_menu_for_selection();
            interface_.request(ui::ViewRequest::open_context_menu);
            return;
        case AppAction::previous_guide_step:
        case AppAction::next_guide_step:
        {
            if (!interface_.view_state().present().mode)
                return;
            const auto delta = action == AppAction::next_guide_step ? 1.0 : -1.0;
            const auto model = session_.build_model();
            const auto steps = model.scenario_content ? model.scenario_content->guide.steps.size() : 0;
            const auto last = static_cast<double>(steps > 0 ? steps - 1 : 0);
            const auto next = std::clamp(interface_.view_state().number("present.guide_step", 0.0) + delta, 0.0, last);
            interface_.view_state().set_number("present.guide_step", next);
            interface_.view_state().set_value("present.guide_step", std::to_string(static_cast<int>(next)));
            return;
        }
        default:
            break;
        }
        if (action == AppAction::previous_object || action == AppAction::next_object)
        {
            const auto model = session_.build_model();
            std::vector<physics::BodyId> objects;
            for (const auto& object : model.objects)
                if (object.role != "marker")
                    objects.push_back(object.id);
            if (objects.empty())
                return;
            const auto found = std::find(objects.begin(), objects.end(), model.selection);
            std::size_t index = found == objects.end() ? (action == AppAction::previous_object ? objects.size() - 1 : 0)
                                                       : static_cast<std::size_t>(std::distance(objects.begin(), found));
            if (found != objects.end())
                index = action == AppAction::previous_object ? (index + objects.size() - 1) % objects.size() : (index + 1) % objects.size();
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::select_body;
            command.body = objects[index];
            apply_command(command);
            return;
        }
        if (action == AppAction::toggle_developer_overlay)
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::toggle_developer_overlay;
            apply_command(command);
            return;
        }
        apply_command(command_for_action(action));
    }

    void Application::apply_command(const ui::UiCommand& command)
    {
        if (command.kind == ui::UiCommandKind::minimize_window || command.kind == ui::UiCommandKind::toggle_maximize_window)
        {
            // Working the window closes any open menu, as a press anywhere else would.
            interface_.close_menus();
            if (command.kind == ui::UiCommandKind::minimize_window)
                window_->minimize();
            else
                window_->toggle_maximized();
            return;
        }
        if (command.kind == ui::UiCommandKind::open_arrangement && (command.flag || command.detail == "cancel"))
        {
            session_.apply(command);
            return;
        }
        if (command.kind == ui::UiCommandKind::open_arrangement && !command.id.empty())
        {
            std::string text, error;
            const auto path = std::filesystem::u8path(command.id);
            if (!read_content_file(path, text, error) || !session_.request_open_arrangement(std::move(text), error, path.u8string()))
                session_.notify(ui::Severity::error, error, "content");
            else
                last_content_folder_ = path.parent_path();
            return;
        }
        if (command.kind == ui::UiCommandKind::save_arrangement && command.detail.empty())
        {
            save_continuation_ = session_.take_pending_departure_for_save();
            if (command.flag || session_.current_setup_path().empty())
            {
                const auto model = session_.build_model();
                const auto& file = session_.current_setup_file();
                const auto title = file.title.empty() ? (model.scenario_title.empty() ? std::string { "My setup" } : model.scenario_title) : file.title;
                interface_.view_state().set_value("save.title", title + (file.path.empty() ? " (my version)" : " (copy)"));
                interface_.view_state().set_value("save.mode", file.current_moment ? "current" : "starting");
                interface_.view_state().set_value("save.include_guide", file.include_guide ? "true" : "false");
                interface_.request(ui::ViewRequest::open_save_details);
                return;
            }
            std::string error;
            const auto snapshot = session_.capture_current_setup_save(error);
            if (!snapshot || !session_.can_write_setup_save(*snapshot, snapshot->file.path, error) ||
                !write_content_file(std::filesystem::u8path(snapshot->file.path), snapshot->text, error))
            {
                save_continuation_.reset();
                session_.notify(ui::Severity::error, error, "content");
            }
            else
            {
                const auto current = session_.complete_setup_save(*snapshot, snapshot->file.path);
                last_content_folder_ = std::filesystem::u8path(snapshot->file.path).parent_path();
                session_.notify(ui::Severity::success, "Setup saved.", "content");
                const auto continuation = std::move(save_continuation_);
                save_continuation_.reset();
                if (current && continuation)
                    session_.complete_saved_departure(*continuation);
            }
            return;
        }
        if (begin_content_dialog(command))
            return;
        if (command.kind == ui::UiCommandKind::export_still)
        {
            frame_capture_.request_still();
            return;
        }
        if (command.kind == ui::UiCommandKind::toggle_frame_capture)
        {
            frame_capture_.toggle_sequence();
            return;
        }
        if (command.kind == ui::UiCommandKind::open_captures_folder)
        {
            const auto folder = paths_.user_data("captures");
            std::error_code error;
            std::filesystem::create_directories(folder, error);
            if (!error)
            {
                auto uri = std::string("file:///") + folder.generic_u8string();
                (void)SDL_OpenURL(uri.c_str());
            }
            return;
        }
        if (command.kind == ui::UiCommandKind::toggle_developer_overlay)
        {
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
            if (developer_overlay_)
            {
                ui::UiEvent lost_focus;
                lost_focus.kind = ui::UiEventKind::focus_lost;
                interface_.handle_event(lost_focus);
                session_.handle_scene_event(lost_focus, true);
                developer_overlay_->toggle();
            }
#endif
            return;
        }
        session_.apply(command);
    }

    void Application::apply_pending_commands()
    {
        finish_content_dialog();
        for (const auto& command : interface_.take_commands())
        {
            apply_command(command);
        }
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        if (developer_overlay_)
            for (const auto& command : developer_overlay_->take_commands())
                apply_command(command);
#endif
        // Closing Save details with Cancel or Escape must not make a later Save quit the app.
        if (save_continuation_ && !content_dialog_ && !interface_.view_state().sheet_open("save_details"))
            save_continuation_.reset();
    }

    void Application::update(double frame_time_s)
    {
        if (session_.should_quit())
        {
            running_ = false;
            return;
        }

        const auto model_scale = session_.build_model().ui_scale;
        const auto interface_scale = display_scale() * static_cast<float>(model_scale);
        interface_.set_scale(interface_scale);
        const auto focus = interface_.layout().focus;
        if (focus.width() > 0.0 && focus.height() > 0.0)
            session_.set_focus_rect({ focus.minimum.x, focus.minimum.y, focus.width(), focus.height() });
        session_.set_visible_stage_rect(visible_stage(interface_.layout()));
        session_.set_present_spotlight(interface_.view_state().present().mode && interface_.view_state().present().spotlight);
        session_.set_presenting(interface_.view_state().present().mode);
        // The interface shows a hover card only while no sheet or transient surface is open.
        session_.set_hover_cards_allowed(interface_.view_state().sheets().empty() && interface_.view_state().transients().empty());
        if (const auto card = interface_.hover_card_bounds())
            session_.set_hover_card_area(render::ScreenRect { card->minimum.x, card->minimum.y, card->width(), card->height() });
        else
            session_.set_hover_card_area(std::nullopt);
        session_.set_frame_time(frame_time_s);
        const auto wall_time_s = Window::elapsed_seconds();
        session_.set_wall_time(wall_time_s);
        const auto signature = preference_signature();
        if (signature != observed_preference_signature_)
        {
            observed_preference_signature_ = signature;
            preference_save_due_s_ = wall_time_s + 0.75;
        }
        if (preference_save_due_s_ >= 0.0 && wall_time_s >= preference_save_due_s_)
        {
            persist_preferences();
            preference_save_due_s_ = -1.0;
        }
        auto phase_start = Window::elapsed_seconds();
        session_.advance(frame_time_s);
        previous_simulation_s_ = Window::elapsed_seconds() - phase_start;
        phase_start = Window::elapsed_seconds();
        session_.render(scene_list_);
        previous_scene_s_ = Window::elapsed_seconds() - phase_start;
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        frame_timings_.simulation_s = previous_simulation_s_;
        frame_timings_.scene_s = previous_scene_s_;
#endif
        auto model = session_.build_model();
        model.capturing_frames = frame_capture_.recording();
        model.capture_frame_count = frame_capture_.frame_count();
        model.capture_frame_limit = frame_capture_.frame_limit();
        const auto capture_status = frame_capture_.status();
        if (capture_status != last_capture_status_)
        {
            session_.notify(ui::Severity::info, capture_status, "capture");
            last_capture_status_ = capture_status;
            model = session_.build_model();
            model.capturing_frames = frame_capture_.recording();
            model.capture_frame_count = frame_capture_.frame_count();
            model.capture_frame_limit = frame_capture_.frame_limit();
        }
        interface_.set_scale(display_scale() * static_cast<float>(model.ui_scale));
        interface_.set_theme(session_.scene_settings().theme);
        update_window_frame(model);
        interface_.set_window_frame(window_frame_);
        const auto interface_start = Window::elapsed_seconds();
        interface_.build(model, *device_, interface_list_, wall_time_s);
        previous_interface_s_ = Window::elapsed_seconds() - interface_start;
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        if (developer_overlay_)
            developer_overlay_->build(model, previous_frame_timings_);
        frame_timings_.interface_s = previous_interface_s_;
#endif
    }

    void Application::present()
    {
        // The session owns the live capture-area preference; config_ only catches up on the
        // debounced save, which must not delay a just-changed setting.
        const auto window_capture = session_.capture_area_ == "window";
        if (!window_capture)
        {
            const auto stage = interface_.layout().stage;
            frame_capture_.set_capture_rect(render::ScreenRect { stage.minimum.x, stage.minimum.y, stage.width(), stage.height() });
        }
        else
            frame_capture_.set_capture_rect(std::nullopt);
        frame_capture_.process(*device_, scene_list_, session_.scene_settings().theme.background, Window::elapsed_seconds(), window_capture ? &interface_list_ : nullptr);
        auto phase_start = Window::elapsed_seconds();
        device_->begin_frame(session_.scene_settings().theme.background);
        device_->submit(scene_list_);
        device_->submit(interface_list_);
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        if (developer_overlay_)
            developer_overlay_->render();
#endif
        previous_submit_s_ = Window::elapsed_seconds() - phase_start;
        phase_start = Window::elapsed_seconds();
        device_->end_frame();
        previous_present_s_ = Window::elapsed_seconds() - phase_start;
#if defined(RIGIDBODIES_DEVELOPER_OVERLAY)
        frame_timings_.submit_s = previous_submit_s_;
        frame_timings_.present_s = previous_present_s_;
        previous_frame_timings_ = frame_timings_;
#endif
    }

    float Application::display_scale() const
    {
        if (display_scale_override_ > 0.0f)
            return display_scale_override_;
        return window_ ? window_->display_scale() : 1.0f;
    }

    void Application::render_frame(double frame_time_s)
    {
        process_events();
        update(frame_time_s);
        present();
    }

    void Application::install_window_chrome()
    {
        // The platform asks about a point in the window's coordinates; the interface answers in
        // the drawable's pixels.
        window_chrome_ = WindowChrome::install(window_->native(), [this](const ui::Vec2& point)
            {
                return interface_.window_part({ point.x * input_pixel_scale_.x, point.y * input_pixel_scale_.y });
            });
        if (!window_chrome_)
        {
            core::log_warning("the interface could not take over the title bar; the window keeps the platform's");
            window_frame_.controls = false;
            window_->set_bordered(true);
        }
    }

    void Application::update_window_frame(const ui::UiModel& model)
    {
        // A high-contrast theme turned on while running hands the title bar back to the platform,
        // whose title bar follows the system colours; turned off again, the interface takes it
        // back. Neither happens from within the platform's own move or size loop.
        if ((window_chrome_ || title_bar_yielded_to_contrast_) && !drawing_live_frame_)
        {
            const auto now = Window::elapsed_seconds();
            if (now >= next_contrast_check_s_)
            {
                next_contrast_check_s_ = now + 1.0;
                const auto supported = WindowChrome::supported();
                if (window_chrome_ && !supported)
                {
                    window_chrome_.reset();
                    window_frame_.controls = false;
                    window_->set_bordered(true);
                    title_bar_yielded_to_contrast_ = true;
                }
                else if (!window_chrome_ && supported)
                {
                    title_bar_yielded_to_contrast_ = false;
                    window_->set_bordered(false);
                    window_frame_.controls = true;
                    install_window_chrome();
                    minimum_size_scale_ = 0.0f;
                }
            }
        }
        window_frame_.maximized = window_->maximized();
        window_frame_.resizable = window_->resizable();
        // A screenshot's hidden window never has focus; it is drawn as the window looks in use.
        window_frame_.active = screenshot_.has_value() || window_->active();
        window_frame_.hot = window_chrome_ ? window_chrome_->hot() : ui::WindowPart::client;
        window_frame_.pressed = window_chrome_ ? window_chrome_->pressed() : ui::WindowPart::client;
        if (!window_chrome_)
            return;
        window_chrome_->set_dark(model.theme_id == "workbench_dark");
        // The window shrinks only as far as its title bar still holds the mark, the transport, the
        // tools, the menu and the window controls, with room left to drag it by, but never needs
        // more than its display can show. The size is not changed from within the platform's move
        // loop, which a move to another monitor's scale would otherwise do.
        const auto scale = display_scale() * static_cast<float>(model.ui_scale);
        const auto display = window_->display_id();
        if ((scale != minimum_size_scale_ || display != minimum_size_display_) && !drawing_live_frame_)
        {
            minimum_size_scale_ = scale;
            minimum_size_display_ = display;
            auto width = static_cast<int>(std::lround(560.0 * scale / input_pixel_scale_.x));
            auto height = static_cast<int>(std::lround(320.0 * scale / input_pixel_scale_.y));
            if (const auto usable = window_->usable_display_size())
            {
                width = std::min(width, usable->width);
                height = std::min(height, usable->height);
            }
            window_chrome_->set_minimum_client_size(width, height);
        }
    }

    void Application::apply_window_control_clicks()
    {
        if (!window_chrome_)
            return;
        ui::UiCommand command;
        switch (window_chrome_->take_click())
        {
        case ui::WindowPart::minimize:
            command.kind = ui::UiCommandKind::minimize_window;
            break;
        case ui::WindowPart::maximize:
            command.kind = ui::UiCommandKind::toggle_maximize_window;
            break;
        case ui::WindowPart::close:
            command.kind = ui::UiCommandKind::quit;
            break;
        case ui::WindowPart::client:
        case ui::WindowPart::title_bar:
            return;
        }
        apply_command(command);
    }

    void Application::request_close()
    {
        // Closing the window asks what Quit asks; a minimized window comes forward to ask it.
        ui::UiCommand quit;
        quit.kind = ui::UiCommandKind::quit;
        apply_command(quit);
        if (!session_.should_quit())
            window_->restore_and_raise();
    }

    bool Application::watch_events(void* application, SDL_Event* event)
    {
        auto& self = *static_cast<Application*>(application);
        // Live exposures come only from within the platform's move, size and menu loops, which
        // run inside event polling; any other event drawing a frame would re-enter it.
        if (event->type == SDL_EVENT_WINDOW_EXPOSED && event->window.data1 == 1 && self.window_ && event->window.windowID == self.window_->id() && SDL_IsMainThread())
            self.draw_live_frame();
        return true;
    }

    void Application::draw_live_frame()
    {
        if (drawing_live_frame_ || !running_)
            return;
        drawing_live_frame_ = true;
        // The resize events wait in the queue until the loop ends, so the viewport follows the
        // window here.
        sync_viewport();
        const auto now = Window::elapsed_seconds();
        last_frame_time_s_ = now - frame_start_s_;
        frame_start_s_ = now;
        update(last_frame_time_s_);
        present();
        drawing_live_frame_ = false;
    }

    void Application::apply_screenshot_states()
    {
        const auto first_dynamic = [&]() -> physics::BodyId
        {
            for (const auto id : session_.world().body_ids())
                if (const auto* body = session_.world().find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                    return id;
            return {};
        };
        const auto stage_point = [&](double x, double y)
        {
            const auto stage = interface_.layout().stage;
            return math::Vec2 { stage.minimum.x + stage.width() * x, stage.minimum.y + stage.height() * y };
        };
        const auto scene_event = [&](ui::UiEventKind kind, const math::Vec2& point, ui::PointerButton button = ui::PointerButton::primary)
        {
            ui::UiEvent event;
            event.kind = kind;
            event.pointer_px = point;
            event.button = button;
            event.logical_pixel_scale = display_scale() * session_.build_model().ui_scale;
            session_.set_pointer_position(point);
            (void)session_.handle_scene_event(event, false);
        };
        for (const auto& state : screenshot_->states)
        {
            auto& view = interface_.view_state();
            if (state == "guide" || state == "noguide")
            {
                view.set_surface_open("guide.expanded", state == "guide");
                view.set_surface_open("guide.sheet", state == "guide");
                if (state == "guide")
                {
                    view.set_surface_open("inspector.sheet", false);
                    view.set_value("side.last_used", "inspector");
                }
            }
            else if (state == "inspector" || state == "noinspector")
            {
                view.set_surface_open("inspector.open", state == "inspector");
                view.set_surface_open("inspector.sheet", state == "inspector");
                if (state == "inspector")
                {
                    view.set_surface_open("guide.sheet", false);
                    view.set_value("side.last_used", "guide");
                }
            }
            else if (state == "measure" || state == "graph" || state == "collisions" || state == "runs" || state == "relativity")
            {
                view.set_surface_open("measure.open", true);
                if (state != "measure")
                    view.set_active_tab("measure.header.tabs", state);
            }
            else if (state.rfind("speed=", 0) == 0)
            {
                // The probe's speed as v/c, committed as the speed field would commit it.
                const auto text = state.substr(6);
                char* end = nullptr;
                const auto value = std::strtod(text.c_str(), &end);
                if (text.empty() || end != text.c_str() + text.size())
                    core::log_warning("screenshot: invalid speed \"{}\"", text);
                else
                {
                    ui::UiCommand speed;
                    speed.kind = ui::UiCommandKind::set_relativity_speed;
                    speed.value = value;
                    apply_command(speed);
                }
            }
            else if (state.rfind("curve=", 0) == 0 || state.rfind("range=", 0) == 0)
            {
                // The Relativity plot's choosers, which belong to the view.
                const auto curve = state.rfind("curve=", 0) == 0;
                const auto value = state.substr(6);
                const auto known = curve ? value == "energy" || value == "momentum" || value == "gamma" || value == "clock_rate" : value == "full" || value == "near";
                if (known)
                    view.set_value(curve ? "measure.relativity.curve" : "measure.relativity.range", value);
                else
                    core::log_warning("screenshot: unknown state \"{}\"", state);
            }
            else if (state == "world")
                interface_.request(ui::ViewRequest::open_world);
            else if (state == "library")
                interface_.request(ui::ViewRequest::open_library);
            else if (state == "preferences")
                interface_.request(ui::ViewRequest::open_preferences);
            else if (state == "shortcuts")
                interface_.request(ui::ViewRequest::open_shortcuts);
            else if (state == "about")
                interface_.request(ui::ViewRequest::open_about);
            else if (state == "search")
                interface_.request(ui::ViewRequest::open_command_search);
            else if (state == "playback_speed")
                interface_.reveal("bar.speed.custom");
            else if (state == "menu")
                interface_.request(ui::ViewRequest::open_main_menu);
            else if (state == "add")
                interface_.request(ui::ViewRequest::open_add_menu);
            else if (state == "show")
                interface_.request(ui::ViewRequest::toggle_show);
            else if (state == "present")
                interface_.request(ui::ViewRequest::toggle_present);
            else if (state == "spotlight")
                interface_.view_state().present().spotlight = true;
            else if (state == "unlock")
                interface_.view_state().present().lock = false;
            else if (state == "performance")
                interface_.request(ui::ViewRequest::toggle_performance_overlay);
            else if (state == "nohints")
            {
                for (const auto* hint : { "play", "inspect", "library", "projector_theme" })
                    view.dismiss_hint(hint);
            }
            else if (state == "select")
                session_.set_selection(first_dynamic());
            else if (state == "selectall")
            {
                ui::UiCommand all;
                all.kind = ui::UiCommandKind::select_all;
                apply_command(all);
            }
            else if (state == "context")
            {
                session_.set_selection(first_dynamic());
                session_.request_context_menu_for_selection();
            }
            else if (state == "hover")
            {
                if (const auto* body = session_.world().find_body(first_dynamic()))
                {
                    const auto point = session_.camera().world_to_screen(body->world_center_of_mass_m());
                    for (int frame = 0; frame < 45; ++frame)
                    {
                        scene_event(ui::UiEventKind::pointer_move, point);
                        render_frame(1.0 / 60.0);
                        SDL_Delay(16);
                    }
                }
            }
            else if (state == "draw")
            {
                ui::UiCommand draw;
                draw.kind = ui::UiCommandKind::start_new_shape;
                apply_command(draw);
                render_frame(1.0 / 60.0);
                for (const auto& [x, y] : { std::pair { 0.40, 0.35 }, std::pair { 0.62, 0.32 }, std::pair { 0.66, 0.58 } })
                {
                    scene_event(ui::UiEventKind::pointer_down, stage_point(x, y));
                    scene_event(ui::UiEventKind::pointer_up, stage_point(x, y));
                    render_frame(1.0 / 60.0);
                }
                scene_event(ui::UiEventKind::pointer_move, stage_point(0.47, 0.62));
            }
            else if (state == "draw-options")
                view.open_transient("draw_options");
            else if (state == "reduce-motion")
            {
                ui::UiCommand preference;
                preference.kind = ui::UiCommandKind::set_preference;
                preference.detail = "prefs.accessibility.reduce_motion";
                preference.flag = true;
                apply_command(preference);
            }
            else if (state == "run")
            {
                if (session_.build_model().paused)
                {
                    ui::UiCommand play;
                    play.kind = ui::UiCommandKind::toggle_pause;
                    apply_command(play);
                }
            }
            else if (state == "keep" || state.rfind("keep=", 0) == 0 || state.rfind("play=", 0) == 0)
            {
                // Plays N sixtieths of a second (two seconds by default). "keep" then goes Back to
                // start, which keeps them as a run, so the Runs table and the Graph's previous run
                // have data; "play" leaves the run going, so a later state changes it mid-run.
                const auto keep = state.rfind("play=", 0) != 0;
                const auto text = state == "keep" ? std::string("120") : state.substr(5);
                char* end = nullptr;
                const auto frames = std::strtol(text.c_str(), &end, 10);
                if (text.empty() || end != text.c_str() + text.size() || frames < 1 || frames > 36000)
                    core::log_warning("screenshot: invalid frame count \"{}\"", text);
                else
                {
                    if (session_.build_model().paused)
                    {
                        ui::UiCommand play;
                        play.kind = ui::UiCommandKind::toggle_pause;
                        apply_command(play);
                    }
                    for (long frame = 0; frame < frames; ++frame)
                        render_frame(1.0 / 60.0);
                    if (keep)
                    {
                        ui::UiCommand reset;
                        reset.kind = ui::UiCommandKind::reset_scenario;
                        apply_command(reset);
                    }
                }
            }
            else if (state.rfind("pin=", 0) == 0)
            {
                // A Runs table value, as Add value pins it: "pin=probe_clock" or "pin=lorentz:maximum".
                const auto text = state.substr(4);
                const auto colon = text.find(':');
                ui::UiCommand pin;
                pin.kind = ui::UiCommandKind::pin_run_value;
                pin.id = text.substr(0, colon);
                if (colon != std::string::npos)
                    pin.detail = text.substr(colon + 1);
                apply_command(pin);
            }
            else
                core::log_warning("screenshot: unknown state \"{}\"", state);
            render_frame(1.0 / 60.0);
        }
    }

    int Application::run_screenshot()
    {
        // Settle the first layout, open the requested surfaces, optionally advance the simulation
        // in fixed frames, then let interface animations finish before the capture.
        for (int frame = 0; frame < 4; ++frame)
            render_frame(1.0 / 60.0);
        apply_screenshot_states();
        if (screenshot_->run_frames > 0)
        {
            if (session_.build_model().paused)
            {
                ui::UiCommand play;
                play.kind = ui::UiCommandKind::toggle_pause;
                apply_command(play);
            }
            for (int frame = 0; frame < screenshot_->run_frames; ++frame)
                render_frame(1.0 / 60.0);
        }
        for (int frame = 0; frame < 16; ++frame)
        {
            render_frame(screenshot_->run_frames > 0 ? 1.0 / 60.0 : 0.0);
            SDL_Delay(20);
        }
        const auto error = FrameCapture::write_image(*device_, scene_list_, &interface_list_, session_.scene_settings().theme.background, screenshot_->output);
        if (!error.empty())
        {
            core::log_error("screenshot failed: {}", error);
            return 1;
        }
        const auto size = device_->drawable_size();
        core::log_info("screenshot written to {} ({}x{} at {}x display, {}x text)", screenshot_->output.u8string(), size.width, size.height, display_scale(), session_.build_model().ui_scale);
        return 0;
    }

} // namespace rigidbodies::app
