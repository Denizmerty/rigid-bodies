#include <rigidbodies/app/frame_capture.hpp>
#include <SDL3/SDL.h>
#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;
    class Device final : public render::RenderDevice
    {
    public:
        bool available { true }, readable { true };
        int created {}, destroyed {}, submitted {}, resumed {};
        render::ViewportSize drawable { 2, 2 };
        std::string error;
        std::string_view backend_name() const override
        {
            return "capture-test";
        }
        render::ViewportSize drawable_size() const override
        {
            return drawable;
        }
        float display_scale() const override
        {
            return 1;
        }
        void set_vertical_sync(bool) override
        {
        }
        void begin_frame(const render::Color&) override
        {
        }
        void submit(const render::DrawList&) override
        {
            ++submitted;
        }
        void end_frame() override
        {
        }
        float measure_text_width(std::string_view, float) const override
        {
            return 0;
        }
        float text_line_height(float) const override
        {
            return 14;
        }
        const std::string& last_error() const override
        {
            return error;
        }
        std::optional<render::RenderTarget> create_render_target(render::ViewportSize size, int samples) override
        {
            if (!available)
                return {};
            return render::RenderTarget { static_cast<std::uint64_t>(++created), size, samples };
        }
        bool begin_target(const render::RenderTarget&, const render::Color&) override
        {
            return true;
        }
        std::optional<render::TexturePixels> read_target(const render::RenderTarget&) override
        {
            if (!readable)
                return {};
            if (drawable.width == 2 && drawable.height == 2)
                return render::TexturePixels { 2, 2, { 128, 0, 0, 128, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255 } };
            // Opaque pixels that name their own position, so a crop can be located.
            render::TexturePixels pixels { drawable.width, drawable.height, {} };
            for (int y = 0; y < drawable.height; ++y)
                for (int x = 0; x < drawable.width; ++x)
                    pixels.rgba.insert(pixels.rgba.end(), { static_cast<std::uint8_t>(x * 16), static_cast<std::uint8_t>(y * 16), 0, 255 });
            return pixels;
        }
        void destroy_render_target(const render::RenderTarget&) override
        {
            ++destroyed;
        }
        void resume_frame() override
        {
            ++resumed;
        }
    };

    RIGIDBODIES_TEST("capture is explicit and writes valid straight-alpha PNG")
    {
        Device device;
        app::FrameCapture capture;
        const auto directory = std::filesystem::current_path() / "stage8-capture-tests/still";
        capture.configure(directory);
        render::DrawList list;
        capture.process(device, list, {}, 0);
        RIGIDBODIES_EXPECT(device.created == 0, "ordinary presentation does no capture work");
        capture.request_still();
        capture.process(device, list, {}, 0);
        RIGIDBODIES_EXPECT(device.created == 1 && device.destroyed == 1 && device.resumed == 1 && device.submitted == 1, "one recorded scene exported with balanced target lifetime");
        RIGIDBODIES_EXPECT(capture.status().find("Saved ") == 0, "successful path is visible to the user");
        auto* png = SDL_LoadPNG(capture.status().substr(6).c_str());
        RIGIDBODIES_EXPECT(png && png->w == 2 && png->h == 2, "saved PNG loads at the target dimensions");
        Uint8 red {}, green {}, blue {}, alpha {};
        const auto read = SDL_ReadSurfacePixel(png, 0, 0, &red, &green, &blue, &alpha);
        SDL_DestroySurface(png);
        RIGIDBODIES_EXPECT(read && red == 255 && alpha == 128 && green == 0 && blue == 0, "premultiplied target converted to straight PNG alpha");
        capture.process(device, list, {}, 1);
        RIGIDBODIES_EXPECT(device.created == 1, "still request consumes exactly once");
    }

    RIGIDBODIES_TEST("image sequences respect rate and frame budgets without catch-up")
    {
        Device device;
        app::FrameCapture capture;
        capture.configure(std::filesystem::current_path() / "stage8-capture-tests/sequence");
        render::DrawList list;
        capture.toggle_sequence();
        capture.process(device, list, {}, 0);
        capture.process(device, list, {}, .01);
        RIGIDBODIES_EXPECT(device.created == 1, "frames below the 100ms interval are skipped");
        capture.process(device, list, {}, 1000);
        RIGIDBODIES_EXPECT(device.created == 2, "a long pause records one frame without an unbounded backlog");
        for (int i = 1; i < 130; ++i)
            capture.process(device, list, {}, 1000.0 + i);
        RIGIDBODIES_EXPECT(device.created == 120 && !capture.recording(), "sequence stops at its hard frame budget");
        RIGIDBODIES_EXPECT(device.created == device.destroyed && device.created == device.resumed, "every sequence target is released and frame state restored");
    }

    struct SavedImage
    {
        int width { 0 }, height { 0 };
        Uint8 red { 0 }, green { 0 };
    };

    SavedImage load_saved(const std::string& path)
    {
        SavedImage image;
        if (auto* png = SDL_LoadPNG(path.c_str()))
        {
            image.width = png->w;
            image.height = png->h;
            Uint8 blue {}, alpha {};
            (void)SDL_ReadSurfacePixel(png, 0, 0, &image.red, &image.green, &blue, &alpha);
            SDL_DestroySurface(png);
        }
        return image;
    }

    RIGIDBODIES_TEST("a window capture replays the interface, a stage capture replays the scene cropped to the stage")
    {
        Device device;
        device.drawable = { 8, 6 };
        app::FrameCapture capture;
        capture.configure(std::filesystem::current_path() / "stage8-capture-tests/areas");
        render::DrawList scene, interface;

        capture.set_capture_rect(std::nullopt);
        capture.request_still();
        capture.process(device, scene, {}, 0, &interface);
        RIGIDBODIES_EXPECT(device.submitted == 2, "a window capture replays the scene and the interface");
        auto image = load_saved(capture.status().substr(6));
        RIGIDBODIES_EXPECT(image.width == 8 && image.height == 6, "a window capture keeps the whole drawable");

        capture.set_capture_rect(render::ScreenRect { 2.0, 1.0, 4.0, 3.0 });
        capture.request_still();
        capture.process(device, scene, {}, 1);
        RIGIDBODIES_EXPECT(device.submitted == 3, "a stage capture replays only the scene");
        image = load_saved(capture.status().substr(6));
        RIGIDBODIES_EXPECT(image.width == 4 && image.height == 3, "a stage capture is cropped to the stage rectangle");
        RIGIDBODIES_EXPECT(image.red == 2 * 16 && image.green == 1 * 16, "the crop starts at the stage's top-left pixel");

        capture.set_capture_rect(render::ScreenRect { 6.5, 4.5, 10.0, 10.0 });
        capture.request_still();
        capture.process(device, scene, {}, 2);
        image = load_saved(capture.status().substr(6));
        RIGIDBODIES_EXPECT(image.width == 2 && image.height == 2 && image.red == 6 * 16 && image.green == 4 * 16, "a stage reaching past the drawable is clamped to it");

        const auto still = std::filesystem::current_path() / "stage8-capture-tests/areas/screenshot.png";
        device.submitted = 0;
        RIGIDBODIES_EXPECT(app::FrameCapture::write_image(device, scene, &interface, {}, still).empty() && device.submitted == 2, "a screenshot replays the scene and the interface");
        device.submitted = 0;
        RIGIDBODIES_EXPECT(app::FrameCapture::write_image(device, scene, nullptr, {}, still).empty() && device.submitted == 1, "a scene-only screenshot replays only the scene");
        RIGIDBODIES_EXPECT(device.created == device.destroyed && device.created == device.resumed, "every capture target is released and the frame restored");
    }

    RIGIDBODIES_TEST("unavailable and failed render targets stop capture cleanly")
    {
        Device device;
        app::FrameCapture capture;
        capture.configure(std::filesystem::current_path() / "stage8-capture-tests/failure");
        render::DrawList list;
        device.available = false;
        capture.toggle_sequence();
        capture.process(device, list, {}, 0);
        RIGIDBODIES_EXPECT(!capture.recording() && capture.status().find("failed") != std::string::npos, "allocation failure is reported and stops recording");
        device.available = true;
        device.readable = false;
        capture.request_still();
        capture.process(device, list, {}, 1);
        RIGIDBODIES_EXPECT(device.created == 1 && device.destroyed == 1 && device.resumed == 1, "readback failure releases resources and restores frame state");
        capture.process(device, list, {}, 2);
        RIGIDBODIES_EXPECT(device.created == 1, "failed request does not retry every frame");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
