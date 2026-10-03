#include <rigidbodies/render/font_atlas.hpp>
#include "test_framework.hpp"

#include <limits>

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
