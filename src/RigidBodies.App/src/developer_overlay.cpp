#include <rigidbodies/app/developer_overlay.hpp>

#include <rigidbodies/core/log.hpp>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <imgui_impl_opengl3.h>
#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace rigidbodies::app
{
    namespace
    {
        double nonnegative_finite(double value)
        {
            return std::isfinite(value) ? std::max(0.0, value) : 0.0;
        }

        void scalar(DeveloperInspectorState& state, std::vector<ui::UiCommand>& commands,
            const char* label, double current, ui::UiCommandKind kind, const char* id = "",
            double step = 0.1, const char* format = "%.4f")
        {
            auto& edited = state.numeric_edits[label];
            if (state.active_numeric_edit != label)
                edited = current;
            ImGui::InputDouble(label, &edited, step, step * 10.0, format);
            if (ImGui::IsItemActive())
                state.active_numeric_edit = label;
            if (ImGui::IsItemDeactivatedAfterEdit())
            {
                if (std::isfinite(edited) && edited != current)
                {
                    ui::UiCommand command;
                    command.kind = kind;
                    command.id = id;
                    command.value = edited;
                    commands.push_back(std::move(command));
                }
            }
            if (ImGui::IsItemDeactivated())
                state.active_numeric_edit.clear();
        }

        void toggle(std::vector<ui::UiCommand>& commands, const char* label, bool current,
            ui::UiCommandKind kind, std::string id = {}, physics::BodyId body = {})
        {
            if (ImGui::Checkbox(label, &current))
            {
                ui::UiCommand command;
                command.kind = kind;
                command.flag = current;
                command.id = std::move(id);
                command.body = body;
                commands.push_back(std::move(command));
            }
        }

        void force_controls(const std::vector<physics::ForceGeneratorPtr>& generators,
            physics::BodyId body, std::vector<ui::UiCommand>& commands)
        {
            for (std::size_t index = 0; index < generators.size(); ++index)
            {
                if (!generators[index])
                    continue;
                const auto id = std::to_string(index);
                const auto label = std::string(generators[index]->name()) + "##" + id;
                toggle(commands, label.c_str(), generators[index]->is_enabled(), ui::UiCommandKind::set_force_generator_enabled, id, body);
            }
        }
    } // namespace

    std::array<double, 6> FramePhaseTimings::milliseconds() const
    {
        return { nonnegative_finite(input_s) * 1000.0, nonnegative_finite(simulation_s) * 1000.0, nonnegative_finite(scene_s) * 1000.0, nonnegative_finite(interface_s) * 1000.0, nonnegative_finite(submit_s) * 1000.0, nonnegative_finite(present_s) * 1000.0 };
    }

    double FramePhaseTimings::total_s() const
    {
        return nonnegative_finite(input_s) + nonnegative_finite(simulation_s) +
            nonnegative_finite(scene_s) + nonnegative_finite(interface_s) +
            nonnegative_finite(submit_s) + nonnegative_finite(present_s);
    }

    void draw_developer_inspector(const ui::UiModel& model, const FramePhaseTimings& timings,
        bool& visible, DeveloperInspectorState& state, std::vector<ui::UiCommand>& commands)
    {
        if (!visible)
            return;
        ImGui::SetNextWindowSize({ 460.0f, 620.0f }, ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Internal inspector (F12)", &visible))
        {
            ImGui::End();
            return;
        }
        ImGui::TextUnformatted("Development build | numerical controls use SI units");
        ImGui::TextWrapped("Number changes apply when you leave a field. You can undo simulation changes.");
        if (ImGui::CollapsingHeader("Previous frame CPU timings", ImGuiTreeNodeFlags_DefaultOpen))
        {
            constexpr std::array<const char*, 6> names { "Input", "Simulation", "Scene construction", "Interface construction", "Render submission", "Present / vertical sync" };
            const auto values = timings.milliseconds();
            if (ImGui::BeginTable("frame phases", 2, ImGuiTableFlags_SizingStretchProp))
            {
                for (std::size_t index = 0; index < names.size(); ++index)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TextUnformatted(names[index]);
                    ImGui::TableNextColumn();
                    ImGui::Text("%.3f ms", values[index]);
                }
                ImGui::EndTable();
            }
            ImGui::Text("Measured work %.3f ms | interval %.3f ms", timings.total_s() * 1000.0, nonnegative_finite(model.frame_time_s) * 1000.0);
            ImGui::TextWrapped("CPU wall time. Present includes any wait for display synchronization. GPU execution time is not measured.");
        }
        if (model.world)
        {
            const auto& world = *model.world;
            const auto& settings = world.settings();
            const auto& statistics = world.statistics();
            if (ImGui::CollapsingHeader("Physics step profile"))
            {
                toggle(commands, "Measure physics phases", world.profiling_enabled(), ui::UiCommandKind::set_physics_profiling);
                scalar(state, commands, "Workers (0 = automatic)", static_cast<double>(world.parallel_settings().worker_count), ui::UiCommandKind::set_physics_workers, "", 1.0, "%.0f");
                const auto& profile = world.profile();
                ImGui::Text("Last step %llu | %.3f ms", static_cast<unsigned long long>(profile.step_index), profile.total_s * 1000.0);
                ImGui::Text("Broad / narrow workers: %zu / %zu", profile.broad_phase_workers, profile.narrow_phase_workers);
                ImGui::Text("Islands %zu | awake %zu | sleeping %zu | largest %zu", statistics.simulation_island_count, statistics.active_island_count, statistics.sleeping_island_count, statistics.largest_island_body_count);
                ImGui::Text("Packed bodies %zu / %zu capacity", world.contiguous_bodies().size(), world.body_storage_capacity());
                if (world.profiling_enabled() && ImGui::BeginTable("physics phases", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    const std::array<std::pair<const char*, double>, 13> phases { { { "Forces", profile.forces_s }, { "Wake propagation", profile.wake_s }, { "Velocity integration", profile.velocity_integration_s }, { "Broad phase", profile.broad_phase_s }, { "Narrow phase", profile.narrow_phase_s }, { "Island construction", profile.islands_s }, { "Velocity solve", profile.velocity_solve_s }, { "Position integration", profile.position_integration_s }, { "CCD guard", profile.ccd_s }, { "Position solve", profile.position_solve_s }, { "Sleep", profile.sleep_s }, { "Accounting", profile.accounting_s }, { "Total step", profile.total_s } } };
                    for (const auto& phase : phases)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(phase.first);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.3f ms", phase.second * 1000.0);
                    }
                    ImGui::EndTable();
                }
                ImGui::TextWrapped("CPU wall time for the last physics substep. Parallel timings include waiting for workers. Runtime controls do not change scene content.");
            }
            if (ImGui::CollapsingHeader("Solver", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const auto& solver = settings.solver;
                constexpr auto kind = ui::UiCommandKind::set_solver_parameter;
                scalar(state, commands, "Velocity iterations", solver.velocity_iterations, kind, "velocity_iterations", 1.0, "%.0f");
                scalar(state, commands, "Position iterations", solver.position_iterations, kind, "position_iterations", 1.0, "%.0f");
                scalar(state, commands, "Linear slop (m)", solver.linear_slop_m, kind, "linear_slop_m", 0.001);
                scalar(state, commands, "Correction fraction", solver.position_correction_fraction, kind, "position_correction_fraction", 0.05);
                scalar(state, commands, "Restitution threshold (m/s)", solver.restitution_threshold_m_s, kind, "restitution_threshold_m_s");
                scalar(state, commands, "Maximum correction (m)", solver.maximum_position_correction_m, kind, "maximum_position_correction_m", 0.01);
                toggle(commands, "Warm starting", solver.warm_starting, ui::UiCommandKind::set_warm_starting);
                toggle(commands, "Continuous collision", settings.collision.continuous, ui::UiCommandKind::set_continuous_collision);
                toggle(commands, "Constraint graph", settings.constraint_graph_enabled, ui::UiCommandKind::set_constraint_graph);
                ImGui::Text("Bodies %zu | contacts %zu | rows %zu", statistics.body_count, statistics.contact_point_count, statistics.constraint_row_count);
                ImGui::Text("Constraint velocity residual %.6g", statistics.constraint_velocity_residual);
            }
            if (ImGui::CollapsingHeader("Forces and environment"))
            {
                scalar(state, commands, "Gravity magnitude (m/s^2)", math::length(settings.gravity_m_s2), ui::UiCommandKind::set_gravity_magnitude);
                scalar(state, commands, "Gravity direction (degrees)", model.gravity_direction_degrees, ui::UiCommandKind::set_gravity_angle_degrees, "", 5.0, "%.1f");
                constexpr auto kind = ui::UiCommandKind::set_environment_parameter;
                scalar(state, commands, "Air density (kg/m^3)", settings.air_density_kg_m3, kind, "air_density_kg_m3");
                scalar(state, commands, "Air velocity X (m/s)", settings.air_velocity_m_s.x, kind, "air_velocity_x_m_s");
                scalar(state, commands, "Air velocity Y (m/s)", settings.air_velocity_m_s.y, kind, "air_velocity_y_m_s");
                scalar(state, commands, "Air viscosity (Pa s)", settings.air_dynamic_viscosity_pa_s, kind, "air_dynamic_viscosity_pa_s", 1.0e-6, "%.8f");
                ImGui::SeparatorText("Global registrations");
                ImGui::PushID("global forces");
                force_controls(world.force_generators(), {}, commands);
                ImGui::PopID();
                if (world.is_valid(model.selection))
                {
                    ImGui::SeparatorText("Selected body registrations");
                    ImGui::PushID("body forces");
                    force_controls(world.force_generators(model.selection), model.selection, commands);
                    ImGui::PopID();
                }
                toggle(commands, "Angular air resistance", model.angular_drag_enabled, ui::UiCommandKind::set_angular_drag_enabled);
                toggle(commands, "Magnus lift", model.magnus_enabled, ui::UiCommandKind::set_magnus_enabled);
            }
        }
        if (ImGui::CollapsingHeader("Camera"))
        {
            scalar(state, commands, "Visible height (m)", model.view_height_m, ui::UiCommandKind::set_view_height);
            scalar(state, commands, "Wheel zoom sensitivity", model.camera_zoom_sensitivity, ui::UiCommandKind::set_camera_zoom_sensitivity, "", 0.05);
            if (ImGui::Button("Frame everything"))
                commands.push_back({ ui::UiCommandKind::frame_all });
            ImGui::SameLine();
            if (ImGui::Button("Frame selection"))
                commands.push_back({ ui::UiCommandKind::frame_selection });
        }
        ImGui::End();
    }

    DeveloperOverlay::~DeveloperOverlay()
    {
        shutdown();
    }

    bool DeveloperOverlay::initialize(SDL_Window* window, SDL_Renderer* renderer)
    {
        shutdown();
        if (!window || !renderer)
            return false;
        IMGUI_CHECKVERSION();
        context_ = ImGui::CreateContext();
        if (!context_)
            return false;
        ImGui::SetCurrentContext(context_);
        auto& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        ImGui::StyleColorsDark();
        platform_initialized_ = ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
        if (platform_initialized_)
            renderer_initialized_ = ImGui_ImplSDLRenderer3_Init(renderer);
        if (!platform_initialized_ || !renderer_initialized_)
        {
            core::log_error("Dear ImGui developer inspector could not initialize");
            shutdown();
            return false;
        }
        renderer_ = renderer;
        return true;
    }

    bool DeveloperOverlay::initialize_opengl(SDL_Window* window, void* context)
    {
        shutdown();
        if (!window || !context)
            return false;
        IMGUI_CHECKVERSION();
        context_ = ImGui::CreateContext();
        if (!context_)
            return false;
        ImGui::SetCurrentContext(context_);
        auto& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        ImGui::StyleColorsDark();
        open_gl_ = true;
        platform_initialized_ = ImGui_ImplSDL3_InitForOpenGL(window, context);
        if (platform_initialized_)
            renderer_initialized_ = ImGui_ImplOpenGL3_Init("#version 330 core");
        if (!platform_initialized_ || !renderer_initialized_)
        {
            core::log_error("Dear ImGui OpenGL inspector could not initialize");
            shutdown();
            return false;
        }
        return true;
    }

    void DeveloperOverlay::shutdown()
    {
        if (!context_)
            return;
        ImGui::SetCurrentContext(context_);
        if (renderer_initialized_)
        {
            if (open_gl_)
                ImGui_ImplOpenGL3_Shutdown();
            else
                ImGui_ImplSDLRenderer3_Shutdown();
        }
        if (platform_initialized_)
            ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext(context_);
        context_ = nullptr;
        renderer_ = nullptr;
        platform_initialized_ = renderer_initialized_ = frame_ready_ = false;
        open_gl_ = false;
        commands_.clear();
        state_ = {};
    }

    void DeveloperOverlay::toggle()
    {
        set_visible(!visible_);
    }

    void DeveloperOverlay::set_visible(bool visible)
    {
        visible_ = visible;
    }

    bool DeveloperOverlay::visible() const
    {
        return visible_;
    }

    bool DeveloperOverlay::handle_event(const SDL_Event& event)
    {
        if (!context_)
            return false;
        ImGui::SetCurrentContext(context_);
        ImGui_ImplSDL3_ProcessEvent(&event);
        const auto& io = ImGui::GetIO();
        const bool mouse = event.type == SDL_EVENT_MOUSE_MOTION || event.type == SDL_EVENT_MOUSE_BUTTON_DOWN ||
            event.type == SDL_EVENT_MOUSE_BUTTON_UP || event.type == SDL_EVENT_MOUSE_WHEEL;
        const bool keyboard = event.type == SDL_EVENT_KEY_DOWN || event.type == SDL_EVENT_KEY_UP || event.type == SDL_EVENT_TEXT_INPUT;
        return visible_ && ((mouse && io.WantCaptureMouse) || (keyboard && io.WantCaptureKeyboard));
    }

    void DeveloperOverlay::build(const ui::UiModel& model, const FramePhaseTimings& timings)
    {
        if (!context_)
            return;
        ImGui::SetCurrentContext(context_);
        if (open_gl_)
            ImGui_ImplOpenGL3_NewFrame();
        else
            ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        draw_developer_inspector(model, timings, visible_, state_, commands_);
        ImGui::Render();
        frame_ready_ = true;
    }

    void DeveloperOverlay::render()
    {
        if (!context_ || !frame_ready_)
            return;
        ImGui::SetCurrentContext(context_);
        if (open_gl_)
        {
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            frame_ready_ = false;
            return;
        }
        // ImGui uses window coordinates; the teaching renderer records device pixels. The SDL
        // backend scales scissors but expects the SDL renderer to scale the actual vertices.
        float previous_x = 1.0f, previous_y = 1.0f;
        SDL_GetRenderScale(renderer_, &previous_x, &previous_y);
        const auto scale = ImGui::GetDrawData()->FramebufferScale;
        SDL_SetRenderScale(renderer_, scale.x, scale.y);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer_);
        SDL_SetRenderScale(renderer_, previous_x, previous_y);
        frame_ready_ = false;
    }

    std::vector<ui::UiCommand> DeveloperOverlay::take_commands()
    {
        auto result = std::move(commands_);
        commands_.clear();
        return result;
    }
} // namespace rigidbodies::app
