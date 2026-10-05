#include <rigidbodies/render/font_atlas.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>

namespace rigidbodies::render
{
    math::Span<const std::uint32_t> font_atlas_extra_code_points()
    {
        // Superscripts complete scientific notation such as "1.00 × 10⁶ dyn", and the thin space
        // keeps Core's digit groups apart ("299 792 458 m/s").
        static constexpr std::uint32_t codes[] { 0x394u, 0x3b1u, 0x3b2u, 0x3b3u, 0x3b8u, 0x3bcu, 0x3c0u, 0x3c4u, 0x3c9u, 0x2009u, 0x2013u, 0x2014u, 0x2022u, 0x2070u, 0x2074u, 0x2075u, 0x2076u, 0x2077u, 0x2078u, 0x2079u, 0x207bu, 0x2190u, 0x2192u, 0x2212u, 0x221au, 0x221eu, 0x2264u, 0x2265u };
        return codes;
    }

    namespace
    {
        constexpr std::size_t cache_budget = 32 * 1024 * 1024;
        constexpr std::size_t text_budget = 16384;

        std::vector<std::uint32_t> decode(std::string_view text)
        {
            std::vector<std::uint32_t> result;
            for (std::size_t i = 0; i < text.size() && result.size() < text_budget;)
            {
                const auto first = static_cast<unsigned char>(text[i++]);
                if (first < 128)
                {
                    result.push_back(first);
                    continue;
                }
                const int count = first >= 0xf0 && first <= 0xf4 ? 3 : first >= 0xe0 && first < 0xf0 ? 2
                    : first >= 0xc2 && first < 0xe0                                                  ? 1
                                                                                                     : 0;
                std::uint32_t value = first & (count == 3 ? 7u : count == 2 ? 15u
                                                                            : 31u);
                bool valid = count > 0 && i + static_cast<std::size_t>(count) <= text.size();
                for (int j = 0; valid && j < count; ++j)
                {
                    const auto next = static_cast<unsigned char>(text[i + static_cast<std::size_t>(j)]);
                    valid = (next & 0xc0u) == 0x80u;
                    value = (value << 6u) | (next & 0x3fu);
                }
                const auto minimum = count == 3 ? 0x10000u : count == 2 ? 0x800u
                                                                        : 0x80u;
                valid = valid && value >= minimum && value <= 0x10ffffu && !(value >= 0xd800u && value <= 0xdfffu);
                if (valid)
                    i += static_cast<std::size_t>(count);
                result.push_back(valid ? value : static_cast<std::uint32_t>('?'));
            }
            return result;
        }

        int font_size(float scale)
        {
            return static_cast<int>(std::lround(std::clamp(std::isfinite(scale) ? scale * 14.0f : 14.0f, 7.0f, 112.0f)));
        }

        bool digit(std::uint32_t code)
        {
            return code >= '0' && code <= '9';
        }
    }

    struct FontAtlas::Impl
    {
        struct Glyph
        {
            int x {}, y {}, width {}, height {}, left {}, top {};
            float advance {};
            FT_UInt index {};
        };
        struct Page
        {
            std::map<std::uint32_t, Glyph> glyphs;
            std::shared_ptr<const TexturePixels> pixels;
            float ascender {}, height {}, digit_advance {};
            std::uint64_t use {};
        };
        FT_Library library {};
        FT_Face face {};
        std::vector<unsigned char> font;
        std::map<int, Page> pages;
        std::uint64_t clock {};
        int face_size {};

        ~Impl()
        {
            if (face)
                FT_Done_Face(face);
            if (library)
                FT_Done_FreeType(library);
        }

        std::size_t bytes() const
        {
            std::size_t result = 0;
            for (const auto& entry : pages)
                result += entry.second.pixels->rgba.size();
            return result;
        }

        Page& page(int size)
        {
            // Kerning and glyph loading read the face's current size.
            if (face_size != size)
            {
                FT_Set_Pixel_Sizes(face, 0, static_cast<FT_UInt>(size));
                face_size = size;
            }
            const auto found = pages.find(size);
            if (found != pages.end())
            {
                found->second.use = ++clock;
                return found->second;
            }
            Page result;
            result.use = ++clock;
            result.ascender = static_cast<float>(face->size->metrics.ascender) / 64.0f;
            result.height = static_cast<float>(face->size->metrics.height) / 64.0f;
            std::vector<std::uint32_t> codes;
            for (std::uint32_t code = 32; code <= 255; ++code)
                codes.push_back(code);
            for (const auto code : font_atlas_extra_code_points())
                codes.push_back(code);
            auto pixels = std::make_shared<TexturePixels>();
            pixels->width = size > 56 ? 2048 : 1024;
            int x = 2, y = 2, row_height = 0;
            struct Bitmap
            {
                Glyph glyph;
                std::vector<unsigned char> coverage;
            };
            std::vector<Bitmap> bitmaps;
            for (auto code : codes)
            {
                const auto index = FT_Get_Char_Index(face, code);
                // Light hinting snaps only vertically, so advances stay the rounded design widths the
                // scene renderer reserves for labels, and glyph shapes keep their designed spacing.
                if (!index || FT_Load_Glyph(face, index, FT_LOAD_RENDER | FT_LOAD_TARGET_LIGHT))
                    continue;
                const auto& slot = *face->glyph;
                const auto& bitmap = slot.bitmap;
                Glyph glyph;
                glyph.width = static_cast<int>(bitmap.width);
                glyph.height = static_cast<int>(bitmap.rows);
                if (x + glyph.width + 2 > pixels->width)
                {
                    x = 2;
                    y += row_height + 2;
                    row_height = 0;
                }
                glyph.x = x;
                glyph.y = y;
                glyph.left = slot.bitmap_left;
                glyph.top = slot.bitmap_top;
                glyph.advance = static_cast<float>(slot.advance.x) / 64.0f;
                glyph.index = index;
                if (digit(code))
                    result.digit_advance = std::max(result.digit_advance, glyph.advance);
                Bitmap saved { glyph, {} };
                saved.coverage.resize(static_cast<std::size_t>(glyph.width * glyph.height));
                for (int row = 0; row < glyph.height; ++row)
                {
                    const auto* source = bitmap.buffer + (bitmap.pitch >= 0 ? row : glyph.height - 1 - row) * std::abs(bitmap.pitch);
                    for (int column = 0; column < glyph.width; ++column)
                        saved.coverage[static_cast<std::size_t>(row * glyph.width + column)] = bitmap.pixel_mode == FT_PIXEL_MODE_MONO
                            ? static_cast<unsigned char>((source[column / 8] & (0x80u >> (column % 8))) ? 255 : 0)
                            : source[column];
                }
                result.glyphs.emplace(code, glyph);
                bitmaps.push_back(std::move(saved));
                x += glyph.width + 2;
                row_height = std::max(row_height, glyph.height);
            }
            pixels->height = 1;
            while (pixels->height < y + row_height + 2)
                pixels->height *= 2;
            pixels->rgba.resize(static_cast<std::size_t>(pixels->width * pixels->height * 4), 0);
            for (const auto& bitmap : bitmaps)
                for (int row = 0; row < bitmap.glyph.height; ++row)
                    for (int column = 0; column < bitmap.glyph.width; ++column)
                    {
                        const auto destination = static_cast<std::size_t>(((bitmap.glyph.y + row) * pixels->width + bitmap.glyph.x + column) * 4);
                        const auto coverage = bitmap.coverage[static_cast<std::size_t>(row * bitmap.glyph.width + column)];
                        for (std::size_t channel = 0; channel < 4; ++channel)
                            pixels->rgba[destination + channel] = coverage;
                    }
            result.pixels = std::move(pixels);
            while (!pages.empty() && (pages.size() >= 8 || bytes() + result.pixels->rgba.size() > cache_budget))
            {
                const auto oldest = std::min_element(pages.begin(), pages.end(), [](const auto& a, const auto& b)
                    {
                        return a.second.use < b.second.use;
                    });
                pages.erase(oldest);
            }
            return pages.emplace(size, std::move(result)).first->second;
        }

        float layout(std::string_view text, int size, Vec2 origin, Color color, IndexedMesh* mesh)
        {
            auto& atlas = page(size);
            if (mesh)
                mesh->texture = atlas.pixels;
            float x = 0.0f, y = 0.0f, maximum = 0.0f;
            FT_UInt previous = 0;
            bool previous_digit = false;
            for (auto code : decode(text))
            {
                if (code == '\r')
                    continue;
                if (code == '\n')
                {
                    maximum = std::max(maximum, x);
                    x = 0;
                    y += atlas.height;
                    previous = 0;
                    previous_digit = false;
                    continue;
                }
                if (code == '\t')
                {
                    x += atlas.digit_advance * 4.0f;
                    previous = 0;
                    previous_digit = false;
                    continue;
                }
                auto found = atlas.glyphs.find(code);
                if (found == atlas.glyphs.end())
                    found = atlas.glyphs.find('?');
                if (found == atlas.glyphs.end())
                    continue;
                const auto& glyph = found->second;
                if (previous && !digit(code) && !previous_digit && FT_HAS_KERNING(face))
                {
                    FT_Vector kerning {};
                    FT_Get_Kerning(face, previous, glyph.index, FT_KERNING_DEFAULT, &kerning);
                    x += static_cast<float>(kerning.x) / 64.0f;
                }
                const auto advance = digit(code) ? atlas.digit_advance : glyph.advance;
                if (mesh && glyph.width > 0 && glyph.height > 0)
                {
                    const auto left = origin.x + x + glyph.left + (advance - glyph.advance) * 0.5f;
                    const auto top = origin.y + y + atlas.ascender - glyph.top;
                    const auto u0 = static_cast<double>(glyph.x) / atlas.pixels->width;
                    const auto v0 = static_cast<double>(glyph.y) / atlas.pixels->height;
                    const auto u1 = static_cast<double>(glyph.x + glyph.width) / atlas.pixels->width;
                    const auto v1 = static_cast<double>(glyph.y + glyph.height) / atlas.pixels->height;
                    const auto base = static_cast<int>(mesh->vertices.size());
                    mesh->vertices.push_back({ { left, top }, { u0, v0 }, color });
                    mesh->vertices.push_back({ { left + glyph.width, top }, { u1, v0 }, color });
                    mesh->vertices.push_back({ { left + glyph.width, top + glyph.height }, { u1, v1 }, color });
                    mesh->vertices.push_back({ { left, top + glyph.height }, { u0, v1 }, color });
                    for (int index : { 0, 1, 2, 0, 2, 3 })
                        mesh->indices.push_back(base + index);
                }
                x += advance;
                previous = glyph.index;
                previous_digit = digit(code);
            }
            return std::max(maximum, x);
        }
    };

    FontAtlas::FontAtlas() = default;
    FontAtlas::~FontAtlas() = default;

    bool FontAtlas::load(const std::filesystem::path& path)
    {
        auto next = std::make_unique<Impl>();
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file)
            return false;
        const auto length = file.tellg();
        if (length <= 0 || length > 16 * 1024 * 1024)
            return false;
        next->font.resize(static_cast<std::size_t>(length));
        file.seekg(0);
        file.read(reinterpret_cast<char*>(next->font.data()), static_cast<std::streamsize>(length));
        if (!file || FT_Init_FreeType(&next->library) || FT_New_Memory_Face(next->library, next->font.data(), static_cast<FT_Long>(next->font.size()), 0, &next->face))
            return false;
        next->page(14);
        impl_ = std::move(next);
        return true;
    }

    bool FontAtlas::ready() const
    {
        return impl_ != nullptr;
    }

    std::vector<IndexedMesh> FontAtlas::build(Vec2 position, std::string_view text, Color color, float scale) const
    {
        if (!impl_ || text.empty() || !std::isfinite(position.x) || !std::isfinite(position.y))
            return {};
        IndexedMesh mesh;
        const auto channel = [](float value)
        {
            return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f;
        };
        color.alpha = channel(color.alpha);
        color.red = channel(color.red) * color.alpha;
        color.green = channel(color.green) * color.alpha;
        color.blue = channel(color.blue) * color.alpha;
        impl_->layout(text, font_size(scale), position, color, &mesh);
        return mesh.indices.empty() ? std::vector<IndexedMesh> {} : std::vector<IndexedMesh> { std::move(mesh) };
    }

    float FontAtlas::measure(std::string_view text, float scale) const
    {
        return impl_ ? impl_->layout(text, font_size(scale), {}, {}, nullptr) : 0.0f;
    }

    float FontAtlas::line_height(float scale) const
    {
        return impl_ ? impl_->page(font_size(scale)).height : 0.0f;
    }

    std::size_t FontAtlas::cached_bytes() const
    {
        return impl_ ? impl_->bytes() : 0;
    }
}
