#include <rigidbodies/render/gl_render_device.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <SDL3/SDL.h>
#include "test_framework.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>

namespace
{
    using namespace rigidbodies;
    using namespace rigidbodies::render;

    std::array<int, 4> pixel(const TexturePixels& image, int x, int y)
    {
        const auto offset = (static_cast<std::size_t>(y) * static_cast<std::size_t>(image.width) + static_cast<std::size_t>(x)) * 4;
        return { image.rgba[offset], image.rgba[offset + 1], image.rgba[offset + 2], image.rgba[offset + 3] };
    }

    struct SoftwareFixture
    {
        SDL_Surface* surface = SDL_CreateSurface(128, 96, SDL_PIXELFORMAT_RGBA32);
        RenderDevicePtr device = SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface));
        ~SoftwareFixture()
        {
            device.reset();
            SDL_DestroySurface(surface);
        }
    };

    struct GlFixture
    {
        SDL_Window* window {};
        RenderDevicePtr device;
        GlFixture()
        {
            const bool initialized = SDL_Init(SDL_INIT_VIDEO);
            if (initialized)
            {
                SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
                SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
                SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
                SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
                window = SDL_CreateWindow("Rigid Bodies hidden renderer test", 128, 96, SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
                if (window)
                    device = GlRenderDevice::create(window, false);
            }
            if (!device)
            {
                const auto* required = SDL_getenv("RIGIDBODIES_REQUIRE_OPENGL");
                const std::string reason = std::string { "hidden OpenGL device unavailable: " } + SDL_GetError();
                if (required && std::string_view { required } == "1")
                    RIGIDBODIES_FAIL(reason);
                std::cout << "SKIP hardware validation: " << reason << '\n';
            }
        }
        ~GlFixture()
        {
            device.reset();
            SDL_DestroyWindow(window);
            SDL_Quit();
        }
    };

    std::shared_ptr<IndexedMesh> unit_quad()
    {
        auto mesh = std::make_shared<IndexedMesh>();
        mesh->vertices = { { { 0, 0 }, { 0, 0 }, {} }, { { 1, 0 }, { 1, 0 }, {} }, { { 1, 1 }, { 1, 1 }, {} }, { { 0, 1 }, { 0, 1 }, {} } };
        mesh->indices = { 0, 1, 2, 0, 2, 3 };
        return mesh;
    }

    void exercise_target(RenderDevice& device)
    {
        device.begin_frame({ 0, 0, 0, 1 });
        const auto target = device.create_render_target({ 96, 64 }, 4);
        RIGIDBODIES_EXPECT(target.has_value(), "offscreen target allocates");
        RIGIDBODIES_EXPECT(target->samples >= 1 && target->samples <= 4, "actual sample count is explicit");
        RIGIDBODIES_EXPECT(device.begin_target(*target, { 0, 0, 1, 1 }), "target begins");
        RIGIDBODIES_EXPECT(device.drawable_size().width == 96 && device.drawable_size().height == 64, "target reports its own drawable extent");
        DrawList list;
        list.add_rectangle_fill({ 0, 0 }, { 96, 20 }, { 1, 0, 0, 1 });
        list.add_rectangle_fill({ 0, 44 }, { 96, 64 }, { 0, 1, 0, 1 });
        device.submit(list);
        const auto image = device.read_target(*target);
        RIGIDBODIES_EXPECT(image.has_value(), "target readback succeeds");
        RIGIDBODIES_EXPECT(pixel(*image, 30, 10) == std::array<int, 4> { 255, 0, 0, 255 }, "readback starts at the top row in RGBA order");
        RIGIDBODIES_EXPECT(pixel(*image, 30, 30) == std::array<int, 4> { 0, 0, 255, 255 }, "target clear survives outside the mesh");
        RIGIDBODIES_EXPECT(pixel(*image, 30, 55) == std::array<int, 4> { 0, 255, 0, 255 }, "readback lower rows are not inverted");
        device.resume_frame();
        RIGIDBODIES_EXPECT(device.drawable_size().width == 128, "window frame is restored");
        device.destroy_render_target(*target);
        RIGIDBODIES_EXPECT(!device.begin_target(*target, {}), "stale targets cannot be rebound");
        RIGIDBODIES_EXPECT(!device.read_target(*target).has_value(), "stale targets cannot be read");
        device.destroy_render_target(*target);
        RIGIDBODIES_EXPECT(!device.create_render_target({ 0, 64 }).has_value(), "invalid dimensions are rejected");
        device.end_frame();
    }

    void exercise_instances(RenderDevice& device)
    {
        const auto target = device.create_render_target({ 96, 64 }, 4);
        RIGIDBODIES_EXPECT(target.has_value(), "instance target allocates");
        RIGIDBODIES_EXPECT(device.begin_target(*target, { 0, 0, 1, 1 }), "instance target begins");
        DrawList list;
        list.add_instanced_mesh(unit_quad(), { { { 10, 10 }, { 20, 20 }, 0, { 1, 0, 0, 1 } }, { { 70, 10 }, { 20, 20 }, static_cast<float>(math::pi * 0.5), { 0, 1, 0, 0.5f } } });
        device.submit(list);
        const auto image = device.read_target(*target);
        RIGIDBODIES_EXPECT(image.has_value(), "instance readback succeeds");
        RIGIDBODIES_EXPECT(pixel(*image, 20, 20) == std::array<int, 4> { 255, 0, 0, 255 }, "first instance transforms and tints shared geometry");
        const auto blended = pixel(*image, 60, 20);
        RIGIDBODIES_EXPECT(std::abs(blended[1] - 128) <= 2 && std::abs(blended[2] - 127) <= 2, "rotated instance premultiplies its tint once");
        RIGIDBODIES_EXPECT(pixel(*image, 80, 20) == std::array<int, 4> { 0, 0, 255, 255 }, "instance rotation changes coverage");
        device.destroy_render_target(*target);
    }
}

RIGIDBODIES_TEST("SDL fallback targets roundtrip RGBA rows and validate resource lifetime")
{
    SoftwareFixture fixture;
    RIGIDBODIES_EXPECT(fixture.device != nullptr, "software renderer is available without a window");
    exercise_target(*fixture.device);
}

RIGIDBODIES_TEST("SDL fallback expands instanced geometry with rotation and alpha")
{
    SoftwareFixture fixture;
    exercise_instances(*fixture.device);
}

RIGIDBODIES_TEST("SDL fallback uses shared font meshes for proportional scene labels")
{
    SoftwareFixture fixture;
    RIGIDBODIES_EXPECT(fixture.device->load_font(std::filesystem::path { RIGIDBODIES_SOURCE_ASSETS } / "fonts/Inter-Medium.ttf"), "bundled face loads");
    RIGIDBODIES_EXPECT(fixture.device->measure_text_width("WWW", 1) > fixture.device->measure_text_width("iii", 1) * 2, "scene text uses proportional metrics");
    const auto target = fixture.device->create_render_target({ 128, 96 }, 1);
    RIGIDBODIES_EXPECT(fixture.device->begin_target(*target, { 0, 0, 0, 1 }), "font target begins");
    DrawList list;
    list.add_text({ 10, 10 }, "Mass 1.0 kg", { 1, 1, 1, 1 });
    fixture.device->submit(list);
    const auto image = fixture.device->read_target(*target);
    bool has_coverage = false, has_edge = false;
    for (std::size_t index = 0; index < image->rgba.size(); index += 4)
    {
        has_coverage = has_coverage || image->rgba[index] > 200;
        has_edge = has_edge || (image->rgba[index] > 0 && image->rgba[index] < 200);
    }
    RIGIDBODIES_EXPECT(has_coverage && has_edge, "font atlas produces visible antialiased glyph coverage");
    fixture.device->destroy_render_target(*target);
}

RIGIDBODIES_TEST("SDL software feathers retain hue as premultiplied coverage falls to zero")
{
    SoftwareFixture fixture;
    const auto target = fixture.device->create_render_target({ 32, 32 }, 1);
    RIGIDBODIES_EXPECT(fixture.device->begin_target(*target, { 0, 0, 0, 0 }), "transparent feather target begins");
    auto mesh = std::make_shared<IndexedMesh>();
    mesh->vertices = { { { 0, 0 }, {}, { 1, 0, 0, 1 } }, { { 30, 0 }, {}, { 1, 0, 0, 1 } }, { { 0, 30 }, {}, { 0, 0, 0, 0 } } };
    mesh->indices = { 0, 1, 2 };
    DrawList list;
    list.add_indexed_mesh(mesh);
    fixture.device->submit(list);
    const auto image = fixture.device->read_target(*target);
    const auto edge = pixel(*image, 4, 15);
    RIGIDBODIES_EXPECT(edge[3] > 30 && edge[3] < 220, "the sample has partial coverage");
    RIGIDBODIES_EXPECT(std::abs(edge[0] - edge[3]) <= 2, "red coverage is not darkened a second time");
    fixture.device->destroy_render_target(*target);
}

RIGIDBODIES_TEST("hidden OpenGL core shader target resolves and reads correct RGBA orientation")
{
    GlFixture fixture;
    if (!fixture.device)
        return;
    std::cout << "hardware backend: " << fixture.device->backend_name() << '\n';
    exercise_target(*fixture.device);
    std::cout << "screen MSAA samples: " << dynamic_cast<GlRenderDevice*>(fixture.device.get())->multisample_count() << '\n';
    RIGIDBODIES_EXPECT(dynamic_cast<GlRenderDevice*>(fixture.device.get())->multisample_count() >= 1, "screen framebuffer is available");
}

RIGIDBODIES_TEST("hidden OpenGL draws hardware instances with transforms and premultiplied tint")
{
    GlFixture fixture;
    if (!fixture.device)
        return;
    exercise_instances(*fixture.device);
}

RIGIDBODIES_TEST("hidden OpenGL samples glyph textures, clips batches, and measures repeated instance rendering")
{
    GlFixture fixture;
    if (!fixture.device)
        return;
    auto& device = *fixture.device;
    RIGIDBODIES_EXPECT(device.load_font(std::filesystem::path { RIGIDBODIES_SOURCE_ASSETS } / "fonts/Inter-Medium.ttf"), "GL scene font loads");
    const auto target = device.create_render_target({ 256, 192 }, 4);
    RIGIDBODIES_EXPECT(device.begin_target(*target, { 0, 0, 0, 1 }), "text target begins");
    auto quad = unit_quad();
    quad->clip = MeshClip { 20, 10, 20, 20 };
    DrawList list;
    list.add_instanced_mesh(quad, { { { 0, 0 }, { 80, 80 }, 0, { 1, 0, 0, 1 } } });
    list.add_text({ 10, 60 }, "Mass 1.0 kg", { 1, 1, 1, 1 });
    device.submit(list);
    const auto image = device.read_target(*target);
    RIGIDBODIES_EXPECT(pixel(*image, 25, 15)[0] == 255 && pixel(*image, 10, 15)[0] == 0 && pixel(*image, 25, 40)[0] == 0, "top-left clipping maps correctly to GL scissor coordinates");
    bool glyph = false, partial = false;
    for (int y = 60; y < 90; ++y)
        for (int x = 10; x < 128; ++x)
        {
            const auto value = pixel(*image, x, y)[0];
            glyph = glyph || value > 200;
            partial = partial || (value > 0 && value < 200);
        }
    RIGIDBODIES_EXPECT(glyph && partial, "GL texture sampling draws antialiased proportional labels");
    device.destroy_render_target(*target);
    const auto benchmark_target = device.create_render_target({ 1600, 900 }, 4);
    RIGIDBODIES_EXPECT(benchmark_target.has_value(), "full-size benchmark target allocates");
    std::vector<MeshInstance> instances;
    instances.reserve(512);
    for (int i = 0; i < 512; ++i)
        instances.push_back({ { static_cast<double>((i % 32) * 50), static_cast<double>((i / 32) * 56) }, { 45, 45 }, 0, { 0.2f, 0.7f, 0.9f, 0.7f } });
    list.clear();
    list.add_instanced_mesh(unit_quad(), instances);
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 120; ++frame)
    {
        RIGIDBODIES_EXPECT(device.begin_target(*benchmark_target, { 0, 0, 0, 1 }), "benchmark target remains available");
        device.submit(list);
    }
    RIGIDBODIES_EXPECT(device.read_target(*benchmark_target).has_value(), "benchmark synchronizes with GPU readback");
    const auto duration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    std::cout << "512 GPU instances, 120 frames at 1600x900 with " << benchmark_target->samples << "x MSAA: " << duration << " ms total including final readback (" << duration / 120.0 << " ms/frame)\n";
    device.destroy_render_target(*benchmark_target);
}

int main()
{
    return rigidbodies::testing::run_all();
}
