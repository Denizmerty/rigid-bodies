#pragma once

#include <rigidbodies/render/render_device.hpp>

struct SDL_Window;

namespace rigidbodies::render
{
    // OpenGL 3.3 core, acquired through SDL without a platform-specific link dependency.
    // A private MSAA framebuffer makes sample negotiation independent of the window format.
    class GlRenderDevice final : public RenderDevice
    {
    public:
        [[nodiscard]] static RenderDevicePtr create(SDL_Window* window, bool vertical_sync, int samples = 4);
        ~GlRenderDevice() override;
        [[nodiscard]] std::string_view backend_name() const override;
        [[nodiscard]] ViewportSize drawable_size() const override;
        [[nodiscard]] float display_scale() const override;
        void set_vertical_sync(bool enabled) override;
        void begin_frame(const Color& clear) override;
        void submit(const DrawList& list) override;
        void end_frame() override;
        [[nodiscard]] std::optional<RenderTarget> create_render_target(ViewportSize size, int samples = 4) override;
        bool begin_target(const RenderTarget& target, const Color& clear) override;
        [[nodiscard]] std::optional<TexturePixels> read_target(const RenderTarget& target) override;
        void destroy_render_target(const RenderTarget& target) override;
        void resume_frame() override;
        [[nodiscard]] bool load_font(const std::filesystem::path& path) override;
        [[nodiscard]] float measure_text_width(std::string_view text, float scale) const override;
        [[nodiscard]] float text_line_height(float scale) const override;
        [[nodiscard]] const std::string& last_error() const override;
        [[nodiscard]] void* native_context() const;
        [[nodiscard]] int multisample_count() const;

    private:
        struct Impl;
        explicit GlRenderDevice(std::unique_ptr<Impl> implementation);
        std::unique_ptr<Impl> impl_;
    };
} // namespace rigidbodies::render
