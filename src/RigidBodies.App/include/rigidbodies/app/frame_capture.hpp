#pragma once

#include <rigidbodies/render/render_device.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <filesystem>

namespace rigidbodies::app
{
    // Explicit user-requested scene exports; bounded in time, count, dimensions, and disk use.
    // Replays the already recorded scene so exporting cannot advance or change the world.
    class FrameCapture
    {
    public:
        void configure(std::filesystem::path directory);
        void request_still();
        void toggle_sequence();
        // A "window" capture also replays the interface list over the scene; a "stage" capture
        // passes none and crops to the stage rectangle instead.
        void process(render::RenderDevice&, const render::DrawList&, const render::Color&, double now_s,
            const render::DrawList* interface_list = nullptr);
        // Renders the scene (and optionally the interface) once and writes it to path. Returns an
        // empty string on success, otherwise the reason it failed.
        [[nodiscard]] static std::string write_image(render::RenderDevice& device, const render::DrawList& scene, const render::DrawList* interface_list,
            const render::Color& background, const std::filesystem::path& path);
        [[nodiscard]] bool recording() const;
        [[nodiscard]] const std::string& status() const;
        void set_capture_rect(std::optional<render::ScreenRect> rect)
        {
            capture_rect_ = rect;
        }
        [[nodiscard]] std::size_t frame_count() const
        {
            return sequence_frames_;
        }
        [[nodiscard]] static constexpr std::size_t frame_limit()
        {
            return 120;
        }

    private:
        std::filesystem::path directory_, sequence_directory_;
        bool pending_ { false }, recording_ { false };
        std::size_t sequence_frames_ { 0 }, sequence_bytes_ { 0 };
        std::uint64_t serial_ { 0 };
        double next_frame_s_ { 0 };
        std::string status_;
        std::optional<render::ScreenRect> capture_rect_;
        std::filesystem::path unique_path(const char* prefix, const char* extension);
        void fail(std::string message);
    };
}
