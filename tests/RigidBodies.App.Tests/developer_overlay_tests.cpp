#include <rigidbodies/app/developer_overlay.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include <imgui.h>
#include <imgui_internal.h>
#include <SDL3/SDL.h>

#include "test_framework.hpp"

#include <limits>

namespace
{
    using namespace rigidbodies;

    class HeadlessInspector
    {
    public:
        HeadlessInspector()
        {
            IMGUI_CHECKVERSION();
            context_ = ImGui::CreateContext();
            auto& io = ImGui::GetIO();
            io.IniFilename = nullptr;
            io.LogFilename = nullptr;
            io.DisplaySize = { 1280.0f, 800.0f };
            io.DeltaTime = 1.0f / 60.0f;
            unsigned char* pixels = nullptr;
            int width = 0, height = 0;
            io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
            RIGIDBODIES_EXPECT(pixels && width > 0 && height > 0, "real Dear ImGui font atlas is built without a platform");
        }
        ~HeadlessInspector()
        {
            ImGui::DestroyContext(context_);
        }
        int draw(const ui::UiModel& model, bool& visible)
        {
            ImGui::NewFrame();
            app::draw_developer_inspector(model, {}, visible, state, commands);
            ImGui::Render();
            return ImGui::GetDrawData()->TotalVtxCount;
        }
        app::DeveloperInspectorState state;
        std::vector<ui::UiCommand> commands;

    private:
        ImGuiContext* context_ {};
    };

    RIGIDBODIES_TEST("developer timing distinguishes six phases including display wait")
    {
        app::FramePhaseTimings timings { 0.001, 0.002, 0.003, 0.004, 0.005, 0.006 };
        const auto milliseconds = timings.milliseconds();
        for (std::size_t index = 0; index < milliseconds.size(); ++index)
            RIGIDBODIES_EXPECT_NEAR(milliseconds[index], static_cast<double>(index + 1), 1.0e-12, "each duration is converted independently");
        RIGIDBODIES_EXPECT_NEAR(timings.total_s(), 0.021, 1.0e-12, "present wait participates in measured work");
        timings.input_s = -1.0;
        timings.simulation_s = std::numeric_limits<double>::quiet_NaN();
        timings.present_s = std::numeric_limits<double>::infinity();
        RIGIDBODIES_EXPECT_NEAR(timings.total_s(), 0.012, 1.0e-12, "unavailable samples cannot poison the diagnostic display");
    }

    RIGIDBODIES_TEST("hidden developer inspector contributes no rendered content or commands")
    {
        HeadlessInspector inspector;
        ui::UiModel model;
        bool visible = false;
        RIGIDBODIES_EXPECT(inspector.draw(model, visible) == 0, "closed inspector emits no window");
        RIGIDBODIES_EXPECT(inspector.commands.empty(), "closed inspector cannot modify a scenario");
    }

    RIGIDBODIES_TEST("actual Dear ImGui controls render without changing the observed world")
    {
        HeadlessInspector inspector;
        physics::World world;
        RIGIDBODIES_EXPECT(physics::load_scenario(world, "collision_comparison"), "reference scene is available");
        const auto settings = world.settings();
        const auto ids = world.body_ids();
        ui::UiModel model;
        model.world = &world;
        bool visible = true;
        inspector.draw(model, visible);
        RIGIDBODIES_EXPECT(inspector.draw(model, visible) > 100, "real ImGui text and widgets produce geometry");
        RIGIDBODIES_EXPECT(inspector.commands.empty(), "merely inspecting does not emit edits");
        RIGIDBODIES_EXPECT(world.body_ids() == ids && world.statistics().step_index == 0, "reading the inspector does not step or replace bodies");
        RIGIDBODIES_EXPECT(world.settings().solver.velocity_iterations == settings.solver.velocity_iterations, "solver parameters are read only until a command is applied");
        visible = false;
        RIGIDBODIES_EXPECT(inspector.draw(model, visible) == 0, "closing an existing inspector removes its geometry");
    }

    RIGIDBODIES_TEST("developer adapter rejects missing native surfaces and tears down safely")
    {
        app::DeveloperOverlay overlay;
        RIGIDBODIES_EXPECT(!overlay.initialize(nullptr, nullptr), "no window is opened implicitly");
        overlay.set_visible(true);
        overlay.build({}, {});
        overlay.render();
        overlay.toggle();
        RIGIDBODIES_EXPECT(!overlay.visible(), "visibility remains an explicit operation");
        RIGIDBODIES_EXPECT(overlay.take_commands().empty(), "failed startup produces no commands");
        overlay.shutdown();
        overlay.shutdown();
    }

    RIGIDBODIES_TEST("activating a real developer control queues an edit instead of mutating physics")
    {
        HeadlessInspector inspector;
        physics::World world;
        ui::UiModel model;
        model.world = &world;
        bool visible = true;
        inspector.draw(model, visible);
        inspector.draw(model, visible);
        // The pinned ImGui navigation API activates the same widget as keyboard or pointer
        // input. No native window, screen coordinates, or alternate fake controls are involved.
        auto* window = ImGui::FindWindowByName("Internal inspector (F12)");
        RIGIDBODIES_EXPECT(window != nullptr, "inspector window exists");
        ImGui::ActivateItemByID(window->GetID("Warm starting"));
        inspector.draw(model, visible);
        RIGIDBODIES_EXPECT(inspector.commands.size() == 1, "one activation queues one command");
        RIGIDBODIES_EXPECT(inspector.commands.front().kind == ui::UiCommandKind::set_warm_starting &&
                !inspector.commands.front().flag,
            "command carries the requested solver setting");
        RIGIDBODIES_EXPECT(world.settings().solver.warm_starting, "physics remains untouched until the session applies the command");
    }

    RIGIDBODIES_TEST("physics profiling control routes through the real inspector")
    {
        HeadlessInspector inspector;
        physics::World world;
        ui::UiModel model;
        model.world = &world;
        bool visible = true;
        inspector.draw(model, visible);
        auto* window = ImGui::FindWindowByName("Internal inspector (F12)");
        RIGIDBODIES_EXPECT(window != nullptr, "inspector exists");
        ImGui::ActivateItemByID(window->GetID("Physics step profile"));
        inspector.draw(model, visible);
        inspector.draw(model, visible);
        ImGui::ActivateItemByID(window->GetID("Measure physics phases"));
        inspector.draw(model, visible);
        RIGIDBODIES_EXPECT(inspector.commands.size() == 1 && inspector.commands.front().kind == ui::UiCommandKind::set_physics_profiling && inspector.commands.front().flag, "profiling activation queues one runtime command");
        RIGIDBODIES_EXPECT(!world.profiling_enabled(), "inspector never mutates observed physics");
    }
    RIGIDBODIES_TEST("SDL dummy backend renders developer frames without leaking renderer scale")
    {
        // The dummy video driver has no native desktop surface. This exercises the actual
        // SDL3/SDLRenderer3 adapters while respecting the window-free validation workflow.
        SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "dummy");
        RIGIDBODIES_EXPECT(SDL_Init(SDL_INIT_VIDEO), "dummy platform initializes");
        struct PlatformScope
        {
            SDL_Window* window {};
            SDL_Renderer* renderer {};
            ~PlatformScope()
            {
                if (renderer)
                    SDL_DestroyRenderer(renderer);
                if (window)
                    SDL_DestroyWindow(window);
                SDL_Quit();
            }
        } platform;
        platform.window = SDL_CreateWindow("headless developer test", 960, 720, SDL_WINDOW_HIDDEN);
        RIGIDBODIES_EXPECT(platform.window != nullptr, "dummy window exists without displaying anything");
        platform.renderer = SDL_CreateRenderer(platform.window, "software");
        RIGIDBODIES_EXPECT(platform.renderer != nullptr, "software renderer exists");
        app::DeveloperOverlay overlay;
        RIGIDBODIES_EXPECT(overlay.initialize(platform.window, platform.renderer), "both official backends initialize");
        overlay.set_visible(true);
        overlay.build({}, {});
        overlay.build({}, {});
        SDL_SetRenderScale(platform.renderer, 1.25f, 1.5f);
        ImGui::GetDrawData()->FramebufferScale = { 2.0f, 2.0f };
        overlay.render();
        float scale_x = 0.0f, scale_y = 0.0f;
        SDL_GetRenderScale(platform.renderer, &scale_x, &scale_y);
        RIGIDBODIES_EXPECT_NEAR(scale_x, 1.25, 1.0e-6, "horizontal scale is restored for scene drawing");
        RIGIDBODIES_EXPECT_NEAR(scale_y, 1.5, 1.0e-6, "vertical scale is restored for document drawing");
        auto* pixels = SDL_RenderReadPixels(platform.renderer, nullptr);
        RIGIDBODIES_EXPECT(pixels != nullptr, "official ImGui renderer produces readable pixels");
        SDL_DestroySurface(pixels);
        overlay.shutdown();
    }
} // namespace

int main()
{
    return testing::run_all();
}
