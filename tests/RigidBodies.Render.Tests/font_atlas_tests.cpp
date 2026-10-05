#include <rigidbodies/render/font_atlas.hpp>
#include "scene_style.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
    using namespace rigidbodies;
    const auto font_path = std::filesystem::path { RIGIDBODIES_SOURCE_ASSETS } / "fonts/Inter-Medium.ttf";

    RIGIDBODIES_TEST("proportional_type_uses_tabular_numbers")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "bundled proportional font loads");
        RIGIDBODIES_EXPECT(font.measure("WWW") > font.measure("iii") * 2.0f, "letter advances are proportional");
        RIGIDBODIES_EXPECT_NEAR(font.measure("111.11"), font.measure("888.88"), 0.001, "numeric columns use tabular digits");
        RIGIDBODIES_EXPECT(font.line_height(2) > font.line_height() * 1.8f, "display scaling selects larger rasterized glyphs");
        RIGIDBODIES_EXPECT_NEAR(font.measure("one\nlongest"), font.measure("longest"), 0.001, "multiline width is the widest line");
    }

    RIGIDBODIES_TEST("atlas_is_antialiased_premultiplied_and_records_geometry")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "font loads");
        const auto meshes = font.build({ 10, 20 }, "Force 12 N", { 1, .5f, .25f, .5f });
        RIGIDBODIES_EXPECT(meshes.size() == 1 && !meshes[0].indices.empty(), "one atlas batch contains the glyphs");
        const auto& mesh = meshes[0];
        RIGIDBODIES_EXPECT_NEAR(mesh.vertices[0].color.red, .5, 1e-6, "vertex color is premultiplied once");
        bool feather = false;
        for (std::size_t i = 0; i < mesh.texture->rgba.size(); i += 4)
        {
            const auto alpha = mesh.texture->rgba[i + 3];
            feather = feather || (alpha > 0 && alpha < 255);
            RIGIDBODIES_EXPECT(mesh.texture->rgba[i] == alpha && mesh.texture->rgba[i + 1] == alpha, "glyph coverage is premultiplied white");
        }
        RIGIDBODIES_EXPECT(feather, "glyph outlines contain antialiased coverage");
        for (const auto& vertex : mesh.vertices)
            RIGIDBODIES_EXPECT(vertex.uv.x >= 0 && vertex.uv.x <= 1 && vertex.uv.y >= 0 && vertex.uv.y <= 1, "atlas coordinates are bounded");
    }

    RIGIDBODIES_TEST("unicode_fallback_and_invalid_utf8_are_bounded")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "font loads");
        RIGIDBODIES_EXPECT(font.build({}, "\xc2\xb5 \xce\xb8 \xe2\x86\x92", {}).size() == 1, "scientific labels decode UTF-8");
        RIGIDBODIES_EXPECT_NEAR(font.measure("\xf0\x9f\x98\x80"), font.measure("?"), .001, "unsupported codepoints use one fallback glyph");
        RIGIDBODIES_EXPECT(font.measure("\xff\xc0\xaf") > 0, "malformed UTF-8 safely renders replacements");
        RIGIDBODIES_EXPECT(std::isfinite(font.measure("finite", std::numeric_limits<float>::infinity())), "invalid scale falls back to readable size");
        RIGIDBODIES_EXPECT(font.build({ std::numeric_limits<double>::infinity(), 0 }, "bad", {}).empty(), "nonfinite origin cannot poison geometry");
        const auto invalid = font.build({}, "safe", { std::numeric_limits<float>::quiet_NaN(), -1, 2, std::numeric_limits<float>::quiet_NaN() });
        RIGIDBODIES_EXPECT(!invalid.empty() && invalid[0].vertices[0].color.alpha == 0 && invalid[0].vertices[0].color.red == 0, "invalid colors cannot create nonfinite blend inputs");
    }

    // The scene renderer reserves label space from Inter Medium advances in font units; light hinting
    // keeps each glyph's advance at round(units × pixels / 2048).
    double design_advance(int units, float scale)
    {
        return std::round(units * static_cast<double>(std::lround(scale * 14.0f)) / 2048.0);
    }

    bool same_glyph(const render::IndexedMesh& a, const render::IndexedMesh& b)
    {
        return a.vertices.size() == b.vertices.size() && a.vertices[0].uv.x == b.vertices[0].uv.x && a.vertices[0].uv.y == b.vertices[0].uv.y && a.vertices[2].uv.x == b.vertices[2].uv.x && a.vertices[2].uv.y == b.vertices[2].uv.y;
    }

    RIGIDBODIES_TEST("gamma_and_tau_render_as_glyphs")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "font loads");
        const auto fallback = font.build({}, "?", {});
        RIGIDBODIES_EXPECT(fallback.size() == 1 && fallback[0].vertices.size() == 4, "the fallback is one glyph");
        // γ U+03B3 (1177 units), τ U+03C4 (984), µ U+00B5 (1229) and → U+2192 (1954).
        const std::pair<std::string_view, int> letters[] { { "\xce\xb3", 1177 }, { "\xcf\x84", 984 }, { "\xc2\xb5", 1229 }, { "\xe2\x86\x92", 1954 } };
        for (const auto& [letter, units] : letters)
        {
            const auto meshes = font.build({}, letter, {});
            RIGIDBODIES_EXPECT(meshes.size() == 1 && meshes[0].vertices.size() == 4, "the letter draws one glyph");
            RIGIDBODIES_EXPECT(!same_glyph(meshes[0], fallback[0]), "the letter is baked, not the '?' fallback");
            for (const auto scale : { 12.0f / 14.0f, 1.0f, 1.5f, 2.0f })
                RIGIDBODIES_EXPECT_NEAR(font.measure(letter, scale), design_advance(units, scale), 0.001, "the letter keeps the advance the scene renderer reserves");
        }
    }

    std::string utf8(std::uint32_t code)
    {
        std::string text;
        if (code < 0x80)
            text += static_cast<char>(code);
        else if (code < 0x800)
        {
            text += static_cast<char>(0xc0 | (code >> 6));
            text += static_cast<char>(0x80 | (code & 0x3f));
        }
        else
        {
            text += static_cast<char>(0xe0 | (code >> 12));
            text += static_cast<char>(0x80 | ((code >> 6) & 0x3f));
            text += static_cast<char>(0x80 | (code & 0x3f));
        }
        return text;
    }

    RIGIDBODIES_TEST("the scene renderer reserves each glyph's own advance, and every listed code point is baked")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "font loads");
        const auto fallback = font.build({}, "?", {});
        // Plates are sized from the scene renderer's table of advances, so it must be the font's.
        std::vector<std::uint32_t> codes;
        for (std::uint32_t code = 32; code < 127; ++code)
            codes.push_back(code);
        for (const auto code : { 0xa0u, 0x2009u, 0xb7u, 0xb2u, 0xb3u, 0xb9u, 0x2070u, 0x2074u, 0x2075u, 0x2076u, 0x2077u, 0x2078u, 0x2079u, 0x207bu, 0xb0u, 0xd7u, 0x2212u, 0x2013u, 0x2014u, 0xb5u, 0x3b3u, 0x3c4u, 0x3c9u, 0x2192u })
            codes.push_back(code);
        for (const auto code : codes)
            for (const auto scale : { 12.0f / 14.0f, 1.0f, 1.5f, 2.0f })
                RIGIDBODIES_EXPECT_NEAR(font.measure(utf8(code), scale), render::detail::text_width(utf8(code), scale), 0.001, "the reserved advance is the font's for code point " + std::to_string(code));
        // Every code point the atlas lists is a glyph of its own, never the '?' fallback.
        for (const auto code : render::font_atlas_extra_code_points())
        {
            const auto meshes = font.build({}, utf8(code), {});
            if (code == 0x2009u)
                RIGIDBODIES_EXPECT(meshes.empty() && font.measure(utf8(code)) > 0.0f, "the thin space is a blank advance");
            else
                RIGIDBODIES_EXPECT(meshes.size() == 1 && !same_glyph(meshes[0], fallback[0]), "a listed code point is baked: " + std::to_string(code));
        }
    }

    RIGIDBODIES_TEST("thin_space_is_blank_with_its_own_advance")
    {
        // Core groups digits with U+2009 ("299 792 458"). Every bundled face maps it, so the interface
        // and the stage show the same gap.
        const std::string thin = "\xE2\x80\x89";
        for (const auto* face : { "Inter-Regular.ttf", "Inter-Medium.ttf", "Inter-SemiBold.ttf" })
        {
            render::FontAtlas font;
            RIGIDBODIES_EXPECT(font.load(font_path.parent_path() / face), "face loads");
            for (const auto scale : { 12.0f / 14.0f, 1.0f, 1.5f, 2.0f })
            {
                const auto gap = design_advance(410, scale);
                RIGIDBODIES_EXPECT(gap > 0.0 && std::abs(font.measure("?", scale) - gap) > 0.5, "the gap differs from the '?' fallback");
                RIGIDBODIES_EXPECT_NEAR(font.measure("1" + thin + "2", scale), font.measure("12", scale) + gap, 0.001, "a thin space adds 0.2 em between two digit advances");
                RIGIDBODIES_EXPECT_NEAR(font.measure(thin, scale), gap, 0.001, "a thin space alone is 0.2 em wide");
            }
            RIGIDBODIES_EXPECT(font.build({}, thin, {}).empty(), "a thin space draws nothing");
            const auto grouped = font.build({}, "299" + thin + "792" + thin + "458", {});
            RIGIDBODIES_EXPECT(grouped.size() == 1 && grouped[0].vertices.size() == 9 * 4, "a grouped number draws only its nine digits");
        }
    }

    RIGIDBODIES_TEST("recorded_pages_survive_eviction_and_failed_reload")
    {
        render::FontAtlas font;
        RIGIDBODIES_EXPECT(font.load(font_path), "font loads");
        const auto original = font.build({}, "retained", {});
        const auto retained = original[0].texture;
        const auto first_bytes = retained->rgba;
        for (int size = 7; size <= 112; size += 7)
            (void)font.build({}, "new size", {}, static_cast<float>(size) / 14.0f);
        RIGIDBODIES_EXPECT(font.cached_bytes() <= 32u * 1024u * 1024u, "owned atlas cache stays within budget");
        RIGIDBODIES_EXPECT(retained->rgba == first_bytes, "eviction preserves immutable recorded pixels");
        RIGIDBODIES_EXPECT(!font.load(font_path.parent_path() / "missing.ttf") && font.ready(), "failed replacement preserves the working font");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
