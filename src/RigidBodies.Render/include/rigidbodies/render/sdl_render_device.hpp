#pragma once

#include <rigidbodies/render/render_device.hpp>
#include <rigidbodies/render/font_atlas.hpp>
#include <unordered_map>

// Declared rather than included so that nothing above this header needs the SDL headers on its
// include path. The definitions come from SDL itself in the implementation file.
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

namespace rigidbodies::render
{

    // Device backed by the SDL renderer.
    //
    // The portable fallback when the shader device is unavailable. SDL chooses an accelerated
    // platform renderer, or this class adopts a software renderer for window-free image tests.
    // Both paths consume the same antialiased triangle compiler and proportional glyph atlas as
    // the OpenGL device, so fallback changes capabilities without changing scene commands.
    class SdlRenderDevice final : public RenderDevice
    {
    public:
        // Returns nothing when the renderer could not be created. The reason is written to the
        // log before the failure is reported.
        [[nodiscard]] static RenderDevicePtr create(SDL_Window* window, bool vertical_sync);
        // Adopts an SDL software renderer for image-based tests without creating a window.
        [[nodiscard]] static RenderDevicePtr adopt_software_renderer(SDL_Renderer* renderer);

        ~SdlRenderDevice() override;

        [[nodiscard]] std::string_view backend_name() const override;
        [[nodiscard]] ViewportSize drawable_size() const override;
        [[nodiscard]] float display_scale() const override;

        void set_vertical_sync(bool enabled) override;

        void begin_frame(const Color& clear_color) override;
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

        [[nodiscard]] SDL_Renderer* native_renderer() const;

    private:
        SdlRenderDevice(SDL_Window* window, SDL_Renderer* renderer);

        void draw_mesh(const IndexedMesh& mesh);
        [[nodiscard]] std::vector<IndexedMesh> build_text(Vec2 position, std::string_view text, Color color, float scale);

        SDL_Window* window_ { nullptr };
        SDL_Renderer* renderer_ { nullptr };
        std::string backend_name_ { "sdl_renderer" };
        std::string last_error_;

        FontAtlas font_;
        struct TargetEntry
        {
            RenderTarget handle;
            SDL_Texture* texture {};
        };
        std::unordered_map<std::uint64_t, TargetEntry> targets_;
        std::uint64_t next_target_ { 1 }, active_target_ {};
        bool reported_sample_fallback_ {};
        struct TextureEntry
        {
            std::weak_ptr<const TexturePixels> pixels;
            SDL_Texture* texture { nullptr };
        };
        std::unordered_map<const TexturePixels*, TextureEntry> textures_;
    };

} // namespace rigidbodies::render
