#pragma once

#include <rigidbodies/render/draw_list.hpp>

#include <filesystem>
#include <memory>

namespace rigidbodies::render
{
    // Immutable, premultiplied glyph pages. The small bounded cache can discard a page while
    // recorded meshes still retain it; neither device uploads nor old frames become dangling.
    class FontAtlas
    {
    public:
        FontAtlas();
        ~FontAtlas();
        FontAtlas(const FontAtlas&) = delete;
        FontAtlas& operator=(const FontAtlas&) = delete;

        bool load(const std::filesystem::path& path);
        [[nodiscard]] bool ready() const;
        [[nodiscard]] std::vector<IndexedMesh> build(Vec2 position, std::string_view text, Color color, float scale = 1.0f) const;
        [[nodiscard]] float measure(std::string_view text, float scale = 1.0f) const;
        [[nodiscard]] float line_height(float scale = 1.0f) const;
        [[nodiscard]] std::size_t cached_bytes() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
