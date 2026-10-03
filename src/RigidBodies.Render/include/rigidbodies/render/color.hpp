#pragma once

#include <cstdint>

namespace rigidbodies::render
{

    // Straight-alpha colour with components in [0, 1].
    struct Color
    {
        float red { 1.0f };
        float green { 1.0f };
        float blue { 1.0f };
        float alpha { 1.0f };

        [[nodiscard]] static constexpr Color from_bytes(std::uint8_t red_value, std::uint8_t green_value, std::uint8_t blue_value, std::uint8_t alpha_value = 255)
        {
            return { static_cast<float>(red_value) / 255.0f, static_cast<float>(green_value) / 255.0f, static_cast<float>(blue_value) / 255.0f, static_cast<float>(alpha_value) / 255.0f };
        }

        [[nodiscard]] constexpr Color with_alpha(float value) const
        {
            return { red, green, blue, value };
        }

        [[nodiscard]] constexpr Color scaled(float factor) const
        {
            return { red * factor, green * factor, blue * factor, alpha };
        }
    };

    [[nodiscard]] inline constexpr Color mix(const Color& from, const Color& to, float fraction)
    {
        return { from.red + (to.red - from.red) * fraction,
            from.green + (to.green - from.green) * fraction,
            from.blue + (to.blue - from.blue) * fraction,
            from.alpha + (to.alpha - from.alpha) * fraction };
    }

    // The presentation palette. Colours are named for the role they play rather than for the hue
    // they happen to be, so that a second theme can replace the values without any drawing code
    // changing. Each quantity the playground visualises keeps one colour everywhere it appears, so
    // that a viewer learns the association once.
    struct Theme
    {
        // The stage: a calm canvas a step darker than the chrome, lit faintly from above.
        Color background { Color::from_bytes(19, 22, 26) };
        Color stage_highlight { Color::from_bytes(25, 28, 34) };
        Color stage_vignette { Color::from_bytes(0, 0, 0, 56) };
        Color grid_minor { Color::from_bytes(255, 255, 255, 8) };
        Color grid_major { Color::from_bytes(255, 255, 255, 18) };
        Color axis { Color::from_bytes(176, 190, 214, 74) };

        // Flat fills, used when material shading is off or over budget.
        Color body_fill { Color::from_bytes(92, 124, 172, 235) };
        Color body_outline { Color::from_bytes(156, 188, 232) };
        Color static_body_fill { Color::from_bytes(34, 38, 45) };
        Color static_body_outline { Color::from_bytes(66, 73, 84) };
        Color kinematic_body_fill { Color::from_bytes(84, 104, 108, 235) };
        Color kinematic_body_outline { Color::from_bytes(132, 205, 196) };
        // Fixed bodies read as architectural ground: a lit surface line over a hatched section.
        Color ground_surface { Color::from_bytes(122, 132, 148) };
        Color ground_hatch { Color::from_bytes(255, 255, 255, 15) };

        Color selection { Color::from_bytes(106, 165, 255) };
        Color center_of_mass { Color::from_bytes(238, 241, 245) };
        Color center_of_mass_ink { Color::from_bytes(18, 20, 24) };
        Color shadow { Color::from_bytes(0, 0, 0, 120) };

        // Physical quantities. Hue families follow the interface tones and stay apart under
        // common colour-vision deficiencies by lightness as well as hue; arrowheads and labels
        // differ too, so colour is never the only cue.
        Color velocity { Color::from_bytes(76, 205, 150) };
        Color acceleration { Color::from_bytes(92, 192, 232) };
        Color force { Color::from_bytes(246, 122, 94) };
        Color momentum { Color::from_bytes(184, 146, 248) };
        Color contact { Color::from_bytes(229, 178, 86) };
        Color bounds { Color::from_bytes(122, 134, 154, 170) };
        Color trajectory { Color::from_bytes(138, 176, 228, 190) };

        // Annotation plates on the stage: chrome at partial opacity, hairline edge.
        Color label_plate { Color::from_bytes(24, 27, 32, 218) };
        Color label_border { Color::from_bytes(255, 255, 255, 18) };
        Color label_text { Color::from_bytes(233, 236, 241) };
        Color label_muted { Color::from_bytes(169, 177, 189) };

        Color panel_background { Color::from_bytes(24, 27, 32, 235) };
        Color panel_border { Color::from_bytes(52, 57, 66) };
        Color panel_title { Color::from_bytes(233, 236, 241) };
        Color panel_text { Color::from_bytes(169, 177, 189) };
        Color panel_accent { Color::from_bytes(106, 165, 255) };
        Color panel_muted { Color::from_bytes(142, 151, 165) };

        // Multiplies stage stroke weights; the projector theme thickens lines for distance.
        float stroke_weight { 1.0f };
        // True when the stage is light, so shading and outlines pick their contrast direction.
        bool light_stage { false };
    };

    // Returns the named theme, or the default when the name is not recognised.
    [[nodiscard]] Theme theme_by_name(const char* name);

} // namespace rigidbodies::render
