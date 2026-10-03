#include <rigidbodies/app/frame_capture.hpp>

#include <SDL3/SDL.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <vector>

namespace rigidbodies::app
{
    namespace
    {
        // PNG stores straight alpha; target readback follows the renderer's premultiplied contract.
        void unpremultiply(std::vector<std::uint8_t>& rgba)
        {
            for (std::size_t i = 0; i < rgba.size(); i += 4)
                for (std::size_t channel = 0; channel < 3; ++channel)
                    rgba[i + channel] = rgba[i + 3]
                        ? static_cast<std::uint8_t>(std::min(255, (static_cast<int>(rgba[i + channel]) * 255 + rgba[i + 3] / 2) / rgba[i + 3]))
                        : 0;
        }
    }

    std::string FrameCapture::write_image(render::RenderDevice& device, const render::DrawList& scene, const render::DrawList* interface_list,
        const render::Color& background, const std::filesystem::path& path)
    {
        const auto size = device.drawable_size();
        if (size.width <= 0 || size.height <= 0 || size.width > 8192 || size.height > 8192)
            return "the viewport is outside the capture limits.";
        const auto target = device.create_render_target(size, 4);
        if (!target)
            return "the render device could not allocate an export target.";
        struct TargetGuard
        {
            render::RenderDevice& device;
            render::RenderTarget target;
            ~TargetGuard()
            {
                device.resume_frame();
                device.destroy_render_target(target);
            }
        } guard { device, *target };
        if (!device.begin_target(*target, background))
            return "the export target could not be activated.";
        device.submit(scene);
        if (interface_list)
            device.submit(*interface_list);
        auto pixels = device.read_target(*target);
        if (!pixels || pixels->rgba.size() != static_cast<std::size_t>(size.width) * size.height * 4)
            return "the rendered image could not be read.";
        unpremultiply(pixels->rgba);
        auto* surface = SDL_CreateSurfaceFrom(size.width, size.height, SDL_PIXELFORMAT_RGBA32, pixels->rgba.data(), size.width * 4);
        if (!surface)
            return SDL_GetError();
        const auto saved = SDL_SavePNG(surface, path.u8string().c_str());
        SDL_DestroySurface(surface);
        return saved ? std::string {} : std::string(SDL_GetError());
    }

    void FrameCapture::configure(std::filesystem::path directory)
    {
        directory_ = std::move(directory);
    }
    void FrameCapture::request_still()
    {
        pending_ = true;
    }
    bool FrameCapture::recording() const
    {
        return recording_;
    }
    const std::string& FrameCapture::status() const
    {
        return status_;
    }

    std::filesystem::path FrameCapture::unique_path(const char* prefix, const char* extension)
    {
        const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::filesystem::path candidate;
        do
        {
            candidate = directory_ / (std::string(prefix) + std::to_string(stamp) + "-" + std::to_string(++serial_) + extension);
        }
        while (std::filesystem::exists(candidate));
        return candidate;
    }

    void FrameCapture::toggle_sequence()
    {
        recording_ = !recording_;
        sequence_directory_.clear();
        sequence_frames_ = sequence_bytes_ = 0;
        next_frame_s_ = 0;
        status_ = recording_ ? "Recording scene images..." : "Image sequence stopped.";
    }

    void FrameCapture::fail(std::string message)
    {
        status_ = "Capture failed: " + std::move(message);
        pending_ = recording_ = false;
    }

    void FrameCapture::process(render::RenderDevice& device, const render::DrawList& list, const render::Color& background, double now_s,
        const render::DrawList* interface_list)
    {
        if (!pending_ && (!recording_ || !std::isfinite(now_s) || now_s < next_frame_s_))
            return;
        if (directory_.empty())
        {
            fail("no output directory is configured.");
            return;
        }
        const auto size = device.drawable_size();
        if (size.width <= 0 || size.height <= 0 || size.width > 8192 || size.height > 8192 || static_cast<std::uint64_t>(size.width) * size.height > 16777216)
        {
            fail("the viewport exceeds the 16 megapixel capture limit.");
            return;
        }
        // Reserve the uncompressed payload plus PNG/container overhead before allocating or
        // writing another frame. A sequence may stop early; it never exceeds its disk ceiling.
        const auto frame_reserve = static_cast<std::size_t>(size.width) * size.height * 4 + 1024u * 1024u;
        if (recording_ && sequence_bytes_ + frame_reserve > 240u * 1024u * 1024u)
        {
            recording_ = false;
            status_ = "Image sequence complete: " + sequence_directory_.u8string();
            if (!pending_)
                return;
        }
        const auto target = device.create_render_target(size, 4);
        if (!target)
        {
            fail("the render device could not allocate an export target.");
            return;
        }
        struct TargetGuard
        {
            render::RenderDevice& device;
            render::RenderTarget target;
            ~TargetGuard()
            {
                device.resume_frame();
                device.destroy_render_target(target);
            }
        } guard { device, *target };
        if (!device.begin_target(*target, background))
        {
            fail("the export target could not be activated.");
            return;
        }
        device.submit(list);
        if (interface_list)
            device.submit(*interface_list);
        auto pixels = device.read_target(*target);
        if (!pixels || pixels->width != size.width || pixels->height != size.height || pixels->rgba.size() != static_cast<std::size_t>(size.width) * size.height * 4)
        {
            fail("the rendered image could not be read.");
            return;
        }
        int output_x = 0, output_y = 0, output_width = size.width, output_height = size.height;
        if (capture_rect_ && !capture_rect_->empty())
        {
            output_x = std::clamp(static_cast<int>(std::floor(capture_rect_->left)), 0, size.width - 1);
            output_y = std::clamp(static_cast<int>(std::floor(capture_rect_->top)), 0, size.height - 1);
            output_width = std::clamp(static_cast<int>(std::ceil(capture_rect_->width)), 1, size.width - output_x);
            output_height = std::clamp(static_cast<int>(std::ceil(capture_rect_->height)), 1, size.height - output_y);
        }
        std::vector<std::uint8_t> cropped;
        if (output_x != 0 || output_y != 0 || output_width != size.width || output_height != size.height)
        {
            cropped.resize(static_cast<std::size_t>(output_width) * output_height * 4);
            for (int y = 0; y < output_height; ++y)
                std::copy_n(pixels->rgba.begin() + (static_cast<std::size_t>(output_y + y) * size.width + output_x) * 4,
                    static_cast<std::size_t>(output_width) * 4,
                    cropped.begin() + static_cast<std::size_t>(y) * output_width * 4);
            pixels->rgba = std::move(cropped);
        }
        unpremultiply(pixels->rgba);
        auto* surface = SDL_CreateSurfaceFrom(output_width, output_height, SDL_PIXELFORMAT_RGBA32, pixels->rgba.data(), output_width * 4);
        if (!surface)
        {
            fail(SDL_GetError());
            return;
        }
        struct SurfaceGuard
        {
            SDL_Surface* surface;
            ~SurfaceGuard()
            {
                SDL_DestroySurface(surface);
            }
        } surface_guard { surface };
        try
        {
            std::filesystem::create_directories(directory_);
            const auto save = [&](const std::filesystem::path& path)
            {
                if (!SDL_SavePNG(surface, path.u8string().c_str()))
                    return false;
                status_ = "Saved " + path.u8string();
                return true;
            };
            if (pending_)
            {
                pending_ = false;
                if (!save(unique_path("scene-", ".png")))
                {
                    fail(SDL_GetError());
                    return;
                }
            }
            if (recording_ && std::isfinite(now_s) && now_s >= next_frame_s_)
            {
                if (sequence_directory_.empty())
                {
                    sequence_directory_ = unique_path("sequence-", "");
                    std::filesystem::create_directory(sequence_directory_);
                }
                std::ostringstream name;
                name << "frame-" << std::setw(4) << std::setfill('0') << sequence_frames_ + 1 << ".png";
                const auto path = sequence_directory_ / name.str();
                if (!save(path))
                {
                    fail(SDL_GetError());
                    return;
                }
                ++sequence_frames_;
                sequence_bytes_ += static_cast<std::size_t>(std::filesystem::file_size(path));
                next_frame_s_ = now_s + .1;
                if (sequence_frames_ >= 120 || sequence_bytes_ >= 240u * 1024u * 1024u)
                {
                    recording_ = false;
                    status_ = "Image sequence complete: " + sequence_directory_.u8string();
                }
            }
        }
        catch (const std::exception& error)
        {
            fail(error.what());
        }
    }
}
