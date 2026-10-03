#include <rigidbodies/render/sdl_render_device.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/render/draw_compiler.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace rigidbodies::render
{
    namespace
    {

        SDL_FColor to_sdl(const Color& color)
        {
            return SDL_FColor { color.red, color.green, color.blue, color.alpha };
        }

        SDL_FPoint to_point(const Vec2& value)
        {
            return SDL_FPoint { static_cast<float>(value.x), static_cast<float>(value.y) };
        }

    } // namespace

    RenderDevicePtr SdlRenderDevice::create(SDL_Window* window, bool vertical_sync)
    {
        if (window == nullptr)
        {
            core::log_error("a render device cannot be created without a window");
            return nullptr;
        }

        // Passing no backend name lets SDL choose the best one available on the machine.
        auto* renderer = SDL_CreateRenderer(window, nullptr);
        if (renderer == nullptr)
        {
            core::log_error("could not create an SDL renderer: {}", SDL_GetError());
            return nullptr;
        }

        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        SDL_SetRenderVSync(renderer, vertical_sync ? 1 : SDL_RENDERER_VSYNC_DISABLED);

        RenderDevicePtr device { new SdlRenderDevice { window, renderer } };
        core::log_info("render device ready on the {} backend", device->backend_name());
        return device;
    }

    SdlRenderDevice::SdlRenderDevice(SDL_Window* window, SDL_Renderer* renderer) : window_(window), renderer_(renderer)
    {
        if (const auto* name = SDL_GetRendererName(renderer_); name != nullptr)
        {
            backend_name_ = name;
        }
    }

    RenderDevicePtr SdlRenderDevice::adopt_software_renderer(SDL_Renderer* renderer)
    {
        return renderer ? RenderDevicePtr { new SdlRenderDevice { nullptr, renderer } } : nullptr;
    }

    SdlRenderDevice::~SdlRenderDevice()
    {
        for (const auto& [id, target] : targets_)
        {
            (void)id;
            SDL_DestroyTexture(target.texture);
        }
        for (const auto& entry : textures_)
            SDL_DestroyTexture(entry.second.texture);
        if (renderer_ != nullptr)
        {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }
    }

    std::string_view SdlRenderDevice::backend_name() const
    {
        return backend_name_;
    }

    ViewportSize SdlRenderDevice::drawable_size() const
    {
        if (const auto target = targets_.find(active_target_); target != targets_.end())
            return target->second.handle.size;
        int width = 0;
        int height = 0;
        SDL_GetRenderOutputSize(renderer_, &width, &height);
        return { std::max(width, 1), std::max(height, 1) };
    }

    float SdlRenderDevice::display_scale() const
    {
        if (!window_)
            return 1.0f;
        const auto scale = SDL_GetWindowDisplayScale(window_);
        return scale > 0.0f ? scale : 1.0f;
    }

    void SdlRenderDevice::set_vertical_sync(bool enabled)
    {
        if (!SDL_SetRenderVSync(renderer_, enabled ? 1 : SDL_RENDERER_VSYNC_DISABLED))
        {
            last_error_ = SDL_GetError();
            core::log_warning("could not change the vertical sync setting: {}", last_error_);
        }
    }

    void SdlRenderDevice::begin_frame(const Color& clear_color)
    {
        resume_frame();
        for (auto entry = textures_.begin(); entry != textures_.end();)
        {
            if (entry->second.pixels.expired())
            {
                SDL_DestroyTexture(entry->second.texture);
                entry = textures_.erase(entry);
            }
            else
                ++entry;
        }
        SDL_SetRenderDrawColorFloat(renderer_, clear_color.red, clear_color.green, clear_color.blue, clear_color.alpha);
        SDL_RenderClear(renderer_);
    }

    void SdlRenderDevice::submit(const DrawList& list)
    {
        DrawCompileOptions options;
        options.feather = std::clamp(display_scale(), 1.0f, 4.0f);
        const auto compiled = compile_draw_list(list, [&](Vec2 position, std::string_view text, Color color, float scale)
            {
                return build_text(position, text, color, scale);
            },
            options);
        for (const auto& mesh : compiled.meshes)
            draw_mesh(mesh);
    }

    void SdlRenderDevice::end_frame()
    {
        SDL_RenderPresent(renderer_);
    }

    float SdlRenderDevice::measure_text_width(std::string_view text, float scale) const
    {
        return font_.ready() ? font_.measure(text, scale) : static_cast<float>(text.size()) * static_cast<float>(SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE) * scale;
    }

    float SdlRenderDevice::text_line_height(float scale) const
    {
        return font_.ready() ? font_.line_height(scale) : static_cast<float>(SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE) * scale;
    }

    const std::string& SdlRenderDevice::last_error() const
    {
        return last_error_;
    }

    SDL_Renderer* SdlRenderDevice::native_renderer() const
    {
        return renderer_;
    }

    bool SdlRenderDevice::load_font(const std::filesystem::path& path)
    {
        return font_.load(path);
    }

    std::vector<IndexedMesh> SdlRenderDevice::build_text(Vec2 position, std::string_view text, Color color, float scale)
    {
        if (font_.ready())
            return font_.build(position, text, color, scale);
        // The proportional face is a packaged asset. Keep readable diagnostics if that asset
        // was removed, and make the fallback font obey the same ordering and clip pipeline.
        if (text.empty() || text.size() > 4096)
            return {};
        const int width = static_cast<int>(text.size()) * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;
        constexpr int height = SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE;
        auto* surface = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_RGBA32);
        if (!surface)
            return {};
        auto* temporary = SDL_CreateSoftwareRenderer(surface);
        if (!temporary)
        {
            SDL_DestroySurface(surface);
            return {};
        }
        SDL_SetRenderDrawColor(temporary, 0, 0, 0, 0);
        SDL_RenderClear(temporary);
        SDL_SetRenderDrawColor(temporary, 255, 255, 255, 255);
        SDL_RenderDebugText(temporary, 0, 0, std::string { text }.c_str());
        SDL_RenderPresent(temporary);
        auto pixels = std::make_shared<TexturePixels>();
        pixels->width = width;
        pixels->height = height;
        pixels->rgba.resize(static_cast<std::size_t>(width) * height * 4);
        for (int y = 0; y < height; ++y)
            std::memcpy(pixels->rgba.data() + static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 4, static_cast<const std::uint8_t*>(surface->pixels) + y * surface->pitch, static_cast<std::size_t>(width) * 4);
        SDL_DestroyRenderer(temporary);
        SDL_DestroySurface(surface);
        color.red *= color.alpha;
        color.green *= color.alpha;
        color.blue *= color.alpha;
        IndexedMesh mesh;
        mesh.texture = pixels;
        const Vec2 size { width * scale, height * scale };
        mesh.vertices = { { position, { 0, 0 }, color }, { position + Vec2 { size.x, 0 }, { 1, 0 }, color }, { position + size, { 1, 1 }, color }, { position + Vec2 { 0, size.y }, { 0, 1 }, color } };
        mesh.indices = { 0, 1, 2, 0, 2, 3 };
        return { std::move(mesh) };
    }

    std::optional<RenderTarget> SdlRenderDevice::create_render_target(ViewportSize size, int samples)
    {
        if (size.width < 1 || size.height < 1 || size.width > 16384 || size.height > 16384)
        {
            last_error_ = "Render target dimensions are invalid";
            return std::nullopt;
        }
        auto* texture = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_TARGET, size.width, size.height);
        if (!texture)
        {
            last_error_ = SDL_GetError();
            return std::nullopt;
        }
        SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND_PREMULTIPLIED);
        if (samples > 1 && !reported_sample_fallback_)
        {
            core::log_warning("the SDL fallback uses geometric antialiasing; render targets support one sample");
            reported_sample_fallback_ = true;
        }
        RenderTarget handle { next_target_++, size, 1 };
        targets_.emplace(handle.id, TargetEntry { handle, texture });
        return handle;
    }

    bool SdlRenderDevice::begin_target(const RenderTarget& target, const Color& clear)
    {
        const auto found = targets_.find(target.id);
        if (found == targets_.end())
        {
            last_error_ = "Render target is no longer valid";
            return false;
        }
        if (!SDL_SetRenderTarget(renderer_, found->second.texture))
        {
            last_error_ = SDL_GetError();
            return false;
        }
        active_target_ = target.id;
        SDL_SetRenderScale(renderer_, 1, 1);
        SDL_SetRenderClipRect(renderer_, nullptr);
        SDL_SetRenderDrawColorFloat(renderer_, clear.red * clear.alpha, clear.green * clear.alpha, clear.blue * clear.alpha, clear.alpha);
        return SDL_RenderClear(renderer_);
    }

    std::optional<TexturePixels> SdlRenderDevice::read_target(const RenderTarget& target)
    {
        const auto found = targets_.find(target.id);
        if (found == targets_.end())
        {
            last_error_ = "Render target is no longer valid";
            return std::nullopt;
        }
        auto* previous = SDL_GetRenderTarget(renderer_);
        if (!SDL_SetRenderTarget(renderer_, found->second.texture))
        {
            last_error_ = SDL_GetError();
            return std::nullopt;
        }
        auto* raw = SDL_RenderReadPixels(renderer_, nullptr);
        SDL_SetRenderTarget(renderer_, previous);
        if (!raw)
        {
            last_error_ = SDL_GetError();
            return std::nullopt;
        }
        auto* surface = SDL_ConvertSurface(raw, SDL_PIXELFORMAT_RGBA32);
        SDL_DestroySurface(raw);
        if (!surface)
        {
            last_error_ = SDL_GetError();
            return std::nullopt;
        }
        TexturePixels pixels;
        pixels.width = surface->w;
        pixels.height = surface->h;
        const auto row = static_cast<std::size_t>(pixels.width) * 4;
        pixels.rgba.resize(row * static_cast<std::size_t>(pixels.height));
        for (int y = 0; y < pixels.height; ++y)
            std::memcpy(pixels.rgba.data() + static_cast<std::size_t>(y) * row, static_cast<const std::uint8_t*>(surface->pixels) + y * surface->pitch, row);
        SDL_DestroySurface(surface);
        return pixels;
    }

    void SdlRenderDevice::destroy_render_target(const RenderTarget& target)
    {
        const auto found = targets_.find(target.id);
        if (found == targets_.end())
            return;
        if (active_target_ == target.id)
            resume_frame();
        SDL_DestroyTexture(found->second.texture);
        targets_.erase(found);
    }

    void SdlRenderDevice::resume_frame()
    {
        SDL_SetRenderTarget(renderer_, nullptr);
        SDL_SetRenderScale(renderer_, 1, 1);
        SDL_SetRenderClipRect(renderer_, nullptr);
        active_target_ = 0;
    }

    void SdlRenderDevice::draw_mesh(const IndexedMesh& mesh)
    {
        if (!mesh.instances.empty())
        {
            IndexedMesh expanded = mesh;
            expanded.instances.clear();
            for (const auto& instance : mesh.instances)
            {
                for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
                {
                    const auto& original = mesh.vertices[i];
                    auto& vertex = expanded.vertices[i];
                    const Vec2 scaled { original.position.x * instance.scale.x, original.position.y * instance.scale.y };
                    const auto c = std::cos(instance.rotation), s = std::sin(instance.rotation);
                    vertex.position = { scaled.x * c - scaled.y * s + instance.translation.x, scaled.x * s + scaled.y * c + instance.translation.y };
                    vertex.color = { original.color.red * instance.tint.red * instance.tint.alpha, original.color.green * instance.tint.green * instance.tint.alpha, original.color.blue * instance.tint.blue * instance.tint.alpha, original.color.alpha * instance.tint.alpha };
                }
                draw_mesh(expanded);
            }
            return;
        }
        // SDL's software triangle path delegates to surface blits, which do not support the
        // premultiplied blend mode. Convert only that backend; accelerated devices retain the
        // RmlUi premultiplied texture and vertex data unchanged.
        const bool software = backend_name_ == "software";
        const auto blend = software ? SDL_BLENDMODE_BLEND : SDL_BLENDMODE_BLEND_PREMULTIPLIED;
        SDL_Texture* texture = nullptr;
        if (mesh.texture)
        {
            if (mesh.texture->width < 1 || mesh.texture->height < 1 || mesh.texture->width > 16384 || mesh.texture->height > 16384 || mesh.texture->rgba.size() != static_cast<std::size_t>(mesh.texture->width) * static_cast<std::size_t>(mesh.texture->height) * 4)
            {
                last_error_ = "Invalid RGBA texture payload";
                return;
            }
            auto found = textures_.find(mesh.texture.get());
            if (found != textures_.end() && found->second.pixels.expired())
            {
                SDL_DestroyTexture(found->second.texture);
                textures_.erase(found);
                found = textures_.end();
            }
            if (found == textures_.end())
            {
                texture = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, mesh.texture->width, mesh.texture->height);
                if (!texture)
                    return;
                auto pixels = mesh.texture->rgba;
                if (software)
                    for (std::size_t i = 0; i + 3 < pixels.size(); i += 4)
                        for (std::size_t channel = 0; channel < 3; ++channel)
                            pixels[i + channel] = pixels[i + 3] ? static_cast<std::uint8_t>(std::min(255, (static_cast<int>(pixels[i + channel]) * 255 + pixels[i + 3] / 2) / pixels[i + 3])) : 0;
                if (!SDL_UpdateTexture(texture, nullptr, pixels.data(), mesh.texture->width * 4))
                {
                    last_error_ = SDL_GetError();
                    SDL_DestroyTexture(texture);
                    return;
                }
                SDL_SetTextureScaleMode(texture, software ? SDL_SCALEMODE_NEAREST : SDL_SCALEMODE_LINEAR);
                SDL_SetTextureBlendMode(texture, blend);
                textures_.emplace(mesh.texture.get(), TextureEntry { mesh.texture, texture });
            }
            else
                texture = found->second.texture;
        }
        std::vector<SDL_Vertex> geometry;
        geometry.reserve(mesh.vertices.size());
        for (const auto& vertex : mesh.vertices)
        {
            auto color = vertex.color;
            if (software && color.alpha > 0.0f)
            {
                color.red = std::min(1.0f, color.red / color.alpha);
                color.green = std::min(1.0f, color.green / color.alpha);
                color.blue = std::min(1.0f, color.blue / color.alpha);
            }
            geometry.push_back({ to_point(vertex.position), to_sdl(color), to_point(vertex.uv) });
        }
        if (software && !texture)
        {
            // Straight-alpha interpolation needs a hue even at zero coverage. The compiler's
            // premultiplied feather naturally stores transparent black there; copying the hue
            // from the closest connected opaque endpoint avoids darkening the edge twice.
            std::vector<double> nearest(mesh.vertices.size(), std::numeric_limits<double>::max());
            for (std::size_t triangle = 0; triangle + 2 < mesh.indices.size(); triangle += 3)
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const auto index = static_cast<std::size_t>(mesh.indices[triangle + corner]);
                    if (geometry[index].color.a > 0)
                        continue;
                    for (std::size_t neighbor = 0; neighbor < 3; ++neighbor)
                    {
                        const auto other = static_cast<std::size_t>(mesh.indices[triangle + neighbor]);
                        if (geometry[other].color.a <= 0)
                            continue;
                        const auto delta = mesh.vertices[index].position - mesh.vertices[other].position;
                        const auto distance = delta.x * delta.x + delta.y * delta.y;
                        if (distance < nearest[index])
                        {
                            nearest[index] = distance;
                            geometry[index].color.r = geometry[other].color.r;
                            geometry[index].color.g = geometry[other].color.g;
                            geometry[index].color.b = geometry[other].color.b;
                        }
                    }
                }
        }
        if (mesh.clip)
        {
            const SDL_Rect clip { mesh.clip->x, mesh.clip->y, mesh.clip->width, mesh.clip->height };
            SDL_SetRenderClipRect(renderer_, &clip);
        }
        else
            SDL_SetRenderClipRect(renderer_, nullptr);
        SDL_SetRenderDrawBlendMode(renderer_, blend);
        if (software && !texture)
        {
            // SDL's software quad shortcut can mistake adjacent triangles in a rounded fan for
            // an axis-aligned rectangle. Submit those triangles individually so the actual
            // tessellation is preserved, including its thin curved-border triangles.
            for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
                if (!SDL_RenderGeometry(renderer_, nullptr, geometry.data(), static_cast<int>(geometry.size()), mesh.indices.data() + i, 3))
                    last_error_ = SDL_GetError();
        }
        else
        {
            if (!SDL_RenderGeometry(renderer_, texture, geometry.data(), static_cast<int>(geometry.size()), mesh.indices.data(), static_cast<int>(mesh.indices.size())))
                last_error_ = SDL_GetError();
        }
        SDL_SetRenderClipRect(renderer_, nullptr);
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
    }

} // namespace rigidbodies::render
