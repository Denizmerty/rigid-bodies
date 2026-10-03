#pragma once

#include <rigidbodies/render/camera2d.hpp>
#include <rigidbodies/render/draw_list.hpp>

#include <memory>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace rigidbodies::render
{
    struct RenderTarget
    {
        std::uint64_t id {};
        ViewportSize size;
        int samples { 1 };
        [[nodiscard]] explicit operator bool() const
        {
            return id != 0;
        }
    };

    // The seam between what the application wants drawn and the graphics interface that draws it.
    //
    // Everything above this class records a DrawList and knows nothing further. That is what keeps
    // the scene and interface layers portable, and it is what will allow a different backend to be
    // introduced without either layer changing: the backend is chosen once, where the device is
    // created.
    class RenderDevice
    {
    public:
        RenderDevice() = default;
        RenderDevice(const RenderDevice&) = delete;
        RenderDevice(RenderDevice&&) = delete;
        RenderDevice& operator=(const RenderDevice&) = delete;
        RenderDevice& operator=(RenderDevice&&) = delete;
        virtual ~RenderDevice() = default;

        [[nodiscard]] virtual std::string_view backend_name() const = 0;

        // Size of the drawable area in pixels, which is not the window size on a display with a
        // scale factor applied.
        [[nodiscard]] virtual ViewportSize drawable_size() const = 0;

        // Ratio between drawable pixels and the logical units the window reports, used to keep
        // line widths and text the same apparent size on any display.
        [[nodiscard]] virtual float display_scale() const = 0;

        virtual void set_vertical_sync(bool enabled) = 0;

        virtual void begin_frame(const Color& clear_color) = 0;

        // Replays a recorded list. Several lists may be submitted per frame, and they are drawn in
        // the order they are submitted, which is how the scene, the overlays, and the interface are
        // layered.
        virtual void submit(const DrawList& list) = 0;

        virtual void end_frame() = 0;

        // Targets use the same top-left pixel coordinates and premultiplied colors as a frame.
        // Readback is tightly packed RGBA8 in top-to-bottom row order. These defaults retain the
        // ability to use small command-only test devices with no graphics resource lifetime.
        [[nodiscard]] virtual std::optional<RenderTarget> create_render_target(ViewportSize, int = 4)
        {
            return std::nullopt;
        }
        virtual bool begin_target(const RenderTarget&, const Color&)
        {
            return false;
        }
        [[nodiscard]] virtual std::optional<TexturePixels> read_target(const RenderTarget&)
        {
            return std::nullopt;
        }
        virtual void destroy_render_target(const RenderTarget&)
        {
        }
        virtual void resume_frame()
        {
        }
        [[nodiscard]] virtual bool load_font(const std::filesystem::path&)
        {
            return false;
        }

        // Width in pixels the given text would occupy at the given scale, so that the interface can
        // lay out without drawing first.
        [[nodiscard]] virtual float measure_text_width(std::string_view text, float scale) const = 0;
        [[nodiscard]] virtual float text_line_height(float scale) const = 0;

        // Message describing why the last operation failed, empty when nothing has failed.
        [[nodiscard]] virtual const std::string& last_error() const = 0;
    };

    using RenderDevicePtr = std::unique_ptr<RenderDevice>;

} // namespace rigidbodies::render
