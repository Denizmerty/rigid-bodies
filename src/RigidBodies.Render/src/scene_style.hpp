#pragma once

// Typography, stroke and mesh helpers shared by the scene renderer and the relativity stage, so
// stage text is measured and stroked the same way wherever it is drawn. Internal to Render.

#include <rigidbodies/render/scene_renderer.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>
#include <utility>

namespace rigidbodies::render::detail
{
    inline float pixel_scale(const SceneRenderSettings& settings)
    {
        return std::isfinite(settings.display_scale) ? std::clamp(settings.display_scale, 0.5f, 4.0f) : 1.0f;
    }

    // Stroke weights follow the house hierarchy (hairline 1, standard 1.5, emphasis 2
    // logical pixels), scaled for density and for the projector's heavier lines.
    inline float stroke(const SceneRenderSettings& settings, float logical)
    {
        const auto weight = std::isfinite(settings.theme.stroke_weight) ? std::clamp(settings.theme.stroke_weight, 0.5f, 3.0f) : 1.0f;
        return logical * pixel_scale(settings) * weight;
    }

    // Typography. Scene text uses Inter Medium through the device atlas, which rasterises
    // at round(14 * scale) pixels with hinted advances and tabular digits. Mirroring those
    // metrics lets the renderer reserve label space without a device round trip.

    inline constexpr std::array<std::uint16_t, 95> inter_medium_advances {
        546, 623, 1012, 1308, 1323, 2034, 1338, 641, 755, 755, 1066, 1367, 621, 947, 621, 757, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 1327, 621, 646, 1367, 1367, 1367, 1080, 2012, 1452, 1345, 1502, 1478, 1235, 1207, 1531, 1525, 558, 1178, 1408, 1158, 1869, 1549, 1570, 1314, 1574, 1327, 1323, 1337, 1516, 1452, 2054, 1435, 1426, 1312, 755, 757, 755, 976, 948, 690, 1163, 1266, 1182, 1266, 1203, 777, 1269, 1232, 516, 516, 1145, 516, 1819, 1232, 1237, 1266, 1266, 792, 1103, 697, 1232, 1177, 1698, 1141, 1178, 1145, 902, 708, 902, 1367
    };

    // Annotations are 12 logical pixels against the atlas' 14 pixel base.
    inline constexpr float label_text_ratio = 12.0f / 14.0f;

    // Density times the reader's text-size preference: what stage text and its plates scale by.
    inline float text_pixel_scale(const SceneRenderSettings& settings)
    {
        const auto preference = std::isfinite(settings.text_scale) ? std::clamp(settings.text_scale, 0.5f, 2.0f) : 1.0f;
        return pixel_scale(settings) * preference;
    }

    inline int font_pixels(float scale)
    {
        return static_cast<int>(std::lround(std::clamp(std::isfinite(scale) ? scale * 14.0f : 14.0f, 7.0f, 112.0f)));
    }

    inline std::uint16_t advance_units(std::uint32_t code)
    {
        if (code >= 32 && code < 127)
            return inter_medium_advances[code - 32];
        switch (code)
        {
        case 0xa0:
            return 546;
        case 0x2009:
            return 410;
        case 0xb7:
            return 621;
        // The superscripts are proportional, as the font draws them.
        case 0xb2:
            return 917;
        case 0xb3:
            return 931;
        case 0xb9:
            return 643;
        case 0x2070:
            return 966;
        case 0x2074:
            return 960;
        case 0x2075:
            return 924;
        case 0x2076:
        case 0x2079:
            return 930;
        case 0x2077:
            return 844;
        case 0x2078:
            return 943;
        case 0x207b:
            return 907;
        case 0xb0:
            return 936;
        case 0xd7:
        case 0x2212:
            return 1367;
        case 0x2013:
            return 1024;
        case 0x2014:
            return 2048;
        case 0xb5:
            return 1229;
        case 0x3b3:
            return 1177;
        case 0x3c4:
            return 984;
        case 0x3c9:
            return 1668;
        case 0x2192:
            return 1954;
        default:
            return 1240;
        }
    }

    inline double text_width(std::string_view text, float scale)
    {
        const auto pixels = static_cast<std::int64_t>(font_pixels(scale));
        double width = 0.0;
        for (std::size_t index = 0; index < text.size();)
        {
            const auto first = static_cast<unsigned char>(text[index++]);
            std::uint32_t code = first;
            int extra = first >= 0xf0 ? 3 : first >= 0xe0 ? 2
                : first >= 0xc0                           ? 1
                                                          : 0;
            if (extra > 0)
                code = first & (extra == 3 ? 0x07u : extra == 2 ? 0x0fu
                                                                : 0x1fu);
            for (; extra > 0 && index < text.size(); --extra)
                code = (code << 6u) | (static_cast<unsigned char>(text[index++]) & 0x3fu);
            // As FreeType scales an advance for light hinting: to 26.6 fixed point, rounded, then
            // rounded to whole pixels, so 1426 units at 28 pixels is 20, not 19.
            const auto fixed = (static_cast<std::int64_t>(advance_units(code)) * pixels * 2048 + 0x8000) >> 16;
            width += static_cast<double>((fixed + 32) >> 6);
        }
        return width;
    }

    struct LabelMetrics
    {
        float scale {};
        double size {}, ascender {}, cap_height {}, height {}, pad_x {}, gap {}, radius {};
    };

    // A house plate's metrics at a text pixel scale (density times the text-size preference).
    inline LabelMetrics label_metrics_at(float text_scale)
    {
        LabelMetrics result;
        const auto ds = static_cast<double>(text_scale);
        result.scale = text_scale * label_text_ratio;
        result.size = static_cast<double>(font_pixels(result.scale));
        result.ascender = std::ceil(result.size * 1984.0 / 2048.0);
        result.cap_height = result.size * 1490.0 / 2048.0;
        result.height = std::round(result.size + 7.0 * ds);
        result.pad_x = std::round(6.0 * ds);
        result.gap = 6.0 * ds;
        result.radius = 4.0 * ds;
        return result;
    }

    inline LabelMetrics label_metrics(const SceneRenderSettings& settings)
    {
        return label_metrics_at(text_pixel_scale(settings));
    }

    inline Color premultiplied(const Color& color)
    {
        const auto unit = [](float value)
        {
            return std::isfinite(value) ? std::clamp(value, 0.0f, 1.0f) : 0.0f;
        };
        const auto alpha = unit(color.alpha);
        return { unit(color.red) * alpha, unit(color.green) * alpha, unit(color.blue) * alpha, alpha };
    }

    // Mesh building. Hand-built meshes carry premultiplied colours and their own transparent rim,
    // so the compiler can skip its boundary weld for them.

    inline int add_vertex(IndexedMesh& mesh, const Vec2& position, const Color& premultiplied_color)
    {
        mesh.vertices.push_back({ position, {}, premultiplied_color });
        return static_cast<int>(mesh.vertices.size()) - 1;
    }

    inline void add_triangle(IndexedMesh& mesh, int a, int b, int c)
    {
        mesh.indices.insert(mesh.indices.end(), { a, b, c });
    }

    // A straight antialiased stroke with butt ends and colours graded between its ends.
    inline void append_soft_segment(IndexedMesh& mesh, const Vec2& a, const Vec2& b, double half_width, double feather, const Color& color_a, const Color& color_b)
    {
        const auto delta = b - a;
        if (math::length_squared(delta) < 1.0e-12 || !math::is_finite(a) || !math::is_finite(b))
            return;
        const auto normal = math::perpendicular(math::normalized(delta));
        const auto first = premultiplied(color_a), second = premultiplied(color_b);
        const Color clear { 0.0f, 0.0f, 0.0f, 0.0f };
        const auto base = static_cast<int>(mesh.vertices.size());
        for (const auto& [point, color] : { std::pair<Vec2, Color> { a, first }, std::pair<Vec2, Color> { b, second } })
        {
            add_vertex(mesh, point + normal * (half_width + feather), clear);
            add_vertex(mesh, point + normal * half_width, color);
            add_vertex(mesh, point - normal * half_width, color);
            add_vertex(mesh, point - normal * (half_width + feather), clear);
        }
        for (int strip = 0; strip < 3; ++strip)
        {
            add_triangle(mesh, base + strip, base + strip + 1, base + 4 + strip + 1);
            add_triangle(mesh, base + strip, base + 4 + strip + 1, base + 4 + strip);
        }
    }
}
