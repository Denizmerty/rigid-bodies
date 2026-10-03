#pragma once

#include <rigidbodies/ui/ui_command.hpp>
#include <rigidbodies/ui/ui_model.hpp>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

struct SDL_Window;
struct SDL_Renderer;
union SDL_Event;
struct ImGuiContext;

namespace rigidbodies::app
{
    // Wall-clock CPU durations. Present includes any vertical-sync wait; these are not GPU times.
    struct FramePhaseTimings
    {
        double input_s {}, simulation_s {}, scene_s {}, interface_s {}, submit_s {}, present_s {};
        [[nodiscard]] std::array<double, 6> milliseconds() const;
        [[nodiscard]] double total_s() const;
    };

    struct DeveloperInspectorState
    {
        std::unordered_map<std::string, double> numeric_edits;
        std::string active_numeric_edit;
    };

    // Constructs the actual ImGui controls with an existing context. Keeping platform startup
    // separate lets tests exercise the inspector and its geometry without a window or renderer.
    void draw_developer_inspector(const ui::UiModel& model, const FramePhaseTimings& timings,
        bool& visible, DeveloperInspectorState& state, std::vector<ui::UiCommand>& commands);

    class DeveloperOverlay
    {
    public:
        DeveloperOverlay() = default;
        ~DeveloperOverlay();
        DeveloperOverlay(const DeveloperOverlay&) = delete;
        DeveloperOverlay& operator=(const DeveloperOverlay&) = delete;

        [[nodiscard]] bool initialize(SDL_Window* window, SDL_Renderer* renderer);
        [[nodiscard]] bool initialize_opengl(SDL_Window* window, void* context);
        void shutdown();
        void toggle();
        void set_visible(bool visible);
        [[nodiscard]] bool visible() const;
        [[nodiscard]] bool handle_event(const SDL_Event& event);
        void build(const ui::UiModel& model, const FramePhaseTimings& timings);
        void render();
        [[nodiscard]] std::vector<ui::UiCommand> take_commands();

    private:
        ImGuiContext* context_ {};
        SDL_Renderer* renderer_ {};
        bool platform_initialized_ {}, renderer_initialized_ {}, visible_ {}, frame_ready_ {};
        bool open_gl_ {};
        DeveloperInspectorState state_;
        std::vector<ui::UiCommand> commands_;
    };
} // namespace rigidbodies::app
