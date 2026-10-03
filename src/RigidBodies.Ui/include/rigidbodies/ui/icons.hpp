#pragma once

#include <array>
#include <string>
#include <string_view>

namespace rigidbodies::ui::icons
{
    // Semantic names are shared by both backends. RmlUi maps them to the bundled Phosphor face;
    // the dependency-free overlay deliberately presents the readable fallback text.
    inline constexpr std::string_view menu = "list";
    inline constexpr std::string_view play = "play";
    inline constexpr std::string_view pause = "pause";
    inline constexpr std::string_view step = "skip-forward";
    inline constexpr std::string_view reset = "arrow-counter-clockwise";
    inline constexpr std::string_view undo = "arrow-u-up-left";
    inline constexpr std::string_view redo = "arrow-u-up-right";
    inline constexpr std::string_view library = "books";
    inline constexpr std::string_view show = "eye";
    inline constexpr std::string_view measure = "chart-line";
    inline constexpr std::string_view frame = "corners-out";
    inline constexpr std::string_view close = "x";
    inline constexpr std::string_view warning = "warning";
    inline constexpr std::string_view success = "check";
    inline constexpr std::string_view add = "plus";
    inline constexpr std::string_view world = "globe-simple";
    inline constexpr std::string_view guide = "graduation-cap";
    inline constexpr std::string_view overflow = "dots-three";
    inline constexpr std::string_view collapse_left = "caret-double-left";
    inline constexpr std::string_view expand_right = "caret-double-right";
    inline constexpr std::string_view previous_step = "caret-left";
    inline constexpr std::string_view next_step = "caret-right";
    inline constexpr std::string_view tool_select = "cursor";
    inline constexpr std::string_view tool_throw = "hand";
    inline constexpr std::string_view tool_pull = "magnet";
    inline constexpr std::string_view draw = "pencil-simple";
    inline constexpr std::string_view capture = "camera";
    inline constexpr std::string_view settings = "gear-six";
    inline constexpr std::string_view search = "magnifying-glass";
    inline constexpr std::string_view lock = "lock";
    inline constexpr std::string_view unlock = "lock-open";
    inline constexpr std::string_view spotlight = "flashlight";
    inline constexpr std::string_view replay = "arrow-clockwise";
    inline constexpr std::string_view back_to_start = "skip-back";
    inline constexpr std::string_view step_many = "fast-forward";
    inline constexpr std::string_view speed = "gauge";
    inline constexpr std::string_view info = "info";
    inline constexpr std::string_view remove = "trash-simple";
    inline constexpr std::string_view star = "star";
    inline constexpr std::string_view save = "floppy-disk";
    inline constexpr std::string_view open = "folder-open";
    inline constexpr std::string_view impact = "lightning";
    inline constexpr std::string_view present = "presentation-chart";
    inline constexpr std::string_view question = "question";
    inline constexpr std::string_view sidebar = "sidebar-simple";
    inline constexpr std::string_view frame_subject = "target";
    inline constexpr std::string_view caret_down = "caret-down";
    inline constexpr std::string_view caret_up = "caret-up";
    inline constexpr std::string_view minus = "minus";
    inline constexpr std::string_view image = "image";
    inline constexpr std::string_view record = "record";
    inline constexpr std::string_view quit = "sign-out";
    inline constexpr std::string_view copy = "copy";
    inline constexpr std::string_view pin = "push-pin";
    inline constexpr std::string_view stop = "stop";
    inline constexpr std::string_view select_all = "selection-all";
    inline constexpr std::string_view crosshair = "crosshair";
    inline constexpr std::string_view keyboard = "keyboard";
    inline constexpr std::string_view hide = "eye-slash";
    inline constexpr std::string_view check_circle = "check-circle";
    inline constexpr std::string_view warning_circle = "warning-circle";
    inline constexpr std::string_view hint = "lightbulb";
    inline constexpr std::string_view error = "x-circle";
    inline constexpr std::string_view export_file = "export";
    inline constexpr std::string_view sliders = "sliders-horizontal";
    inline constexpr std::string_view timer = "timer";
    inline constexpr std::string_view ruler = "ruler";
    inline constexpr std::string_view connection = "link";
    inline constexpr std::string_view curve = "bezier-curve";
    inline constexpr std::string_view theme = "circle-half";
    inline constexpr std::string_view text_size = "text-aa";
    inline constexpr std::string_view air = "wind";
    inline constexpr std::string_view mass = "scales";
    inline constexpr std::string_view checklist = "list-checks";
    inline constexpr std::string_view table = "table";
    inline constexpr std::string_view revert = "arrow-arc-left";
    inline constexpr std::string_view trajectory = "path";
    inline constexpr std::string_view shape = "polygon";
    inline constexpr std::string_view object = "cube";
    inline constexpr std::string_view add_circle = "plus-circle";
    inline constexpr std::string_view more_vertical = "dots-three-vertical";
    inline constexpr std::string_view effects = "sparkle";
    inline constexpr std::string_view display = "monitor";
    inline constexpr std::string_view grab = "hand-grabbing";
    inline constexpr std::string_view move = "arrows-out-cardinal";
    inline constexpr std::string_view light = "sun";
    inline constexpr std::string_view dark = "moon";
    inline constexpr std::string_view ball = "circle";
    inline constexpr std::string_view box = "square";
    // A project glyph derived from Phosphor's rectangle (scripts/add_icon_glyphs.py).
    inline constexpr std::string_view plank = "plank";
    inline constexpr std::string_view grid = "grid-four";
    inline constexpr std::string_view snap_points = "magnet-straight";
    inline constexpr std::string_view angle = "angle";
    inline constexpr std::string_view line = "line-segment";
    // The Inspector docks on the right, so its toggle shows a right-hand sidebar.
    inline constexpr std::string_view sidebar_right = "sidebar-right";

    struct Glyph
    {
        std::string_view name;
        char32_t codepoint;
    };
    inline constexpr std::array glyphs {
        Glyph { menu, 0xe2f0 }, Glyph { play, 0xe3d0 }, Glyph { pause, 0xe39e }, Glyph { step, 0xe5a6 }, Glyph { reset, 0xe038 }, Glyph { undo, 0xe08a }, Glyph { redo, 0xe08c }, Glyph { library, 0xe758 }, Glyph { show, 0xe220 }, Glyph { measure, 0xe154 }, Glyph { frame, 0xe1d0 }, Glyph { close, 0xe4f6 }, Glyph { warning, 0xe4e0 }, Glyph { success, 0xe182 }, Glyph { add, 0xe3d4 }, Glyph { world, 0xe28e }, Glyph { guide, 0xe62c }, Glyph { overflow, 0xe1fe }, Glyph { collapse_left, 0xe128 }, Glyph { expand_right, 0xe12a }, Glyph { previous_step, 0xe138 }, Glyph { next_step, 0xe13a }, Glyph { tool_select, 0xe1dc }, Glyph { tool_throw, 0xe298 }, Glyph { tool_pull, 0xe680 }, Glyph { draw, 0xe3b4 }, Glyph { capture, 0xe10e }, Glyph { settings, 0xe272 }, Glyph { search, 0xe30c }, Glyph { lock, 0xe2fa }, Glyph { unlock, 0xe306 }, Glyph { spotlight, 0xe246 }, Glyph { replay, 0xe036 }, Glyph { back_to_start, 0xe5a4 }, Glyph { step_many, 0xe6a6 }, Glyph { speed, 0xe628 }, Glyph { info, 0xe2ce }, Glyph { remove, 0xe4a8 }, Glyph { star, 0xe46a }, Glyph { save, 0xe248 }, Glyph { open, 0xe256 }, Glyph { impact, 0xe2de }, Glyph { present, 0xe656 }, Glyph { question, 0xe3e8 }, Glyph { sidebar, 0xec24 }, Glyph { frame_subject, 0xe47c }, Glyph { caret_down, 0xe136 }, Glyph { caret_up, 0xe13c }, Glyph { minus, 0xe32a }, Glyph { image, 0xe2ca }, Glyph { record, 0xe3ee }, Glyph { quit, 0xe42a }, Glyph { copy, 0xe1ca }, Glyph { pin, 0xe3e2 }, Glyph { stop, 0xe46c }, Glyph { select_all, 0xe746 }, Glyph { crosshair, 0xe1d6 }, Glyph { keyboard, 0xe2d8 }, Glyph { hide, 0xe224 }, Glyph { check_circle, 0xe184 }, Glyph { warning_circle, 0xe4e2 }, Glyph { hint, 0xe2dc }, Glyph { error, 0xe4f8 }, Glyph { export_file, 0xeaf0 }, Glyph { sliders, 0xe434 }, Glyph { timer, 0xe492 }, Glyph { ruler, 0xe6b8 }, Glyph { connection, 0xe2e2 }, Glyph { curve, 0xeb00 }, Glyph { theme, 0xe18c }, Glyph { text_size, 0xe6ee }, Glyph { air, 0xe5d2 }, Glyph { mass, 0xe750 }, Glyph { checklist, 0xeadc }, Glyph { table, 0xe476 }, Glyph { revert, 0xe014 }, Glyph { trajectory, 0xe39c }, Glyph { shape, 0xe6d0 }, Glyph { object, 0xe1da }, Glyph { add_circle, 0xe3d6 }, Glyph { more_vertical, 0xe208 }, Glyph { effects, 0xe6a2 }, Glyph { display, 0xe32e }, Glyph { grab, 0xe57c }, Glyph { move, 0xe0a4 }, Glyph { light, 0xe472 }, Glyph { dark, 0xe330 }, Glyph { ball, 0xe18a }, Glyph { box, 0xe45e }, Glyph { plank, 0xf101 }, Glyph { grid, 0xe296 }, Glyph { snap_points, 0xe682 }, Glyph { angle, 0xe7bc }, Glyph { line, 0xe6d2 }, Glyph { sidebar_right, 0xec24 }
    };

    // The UTF-8 text for a semantic icon name, or empty when the name is unknown. The document
    // backend writes this into an .icon element so the Phosphor face renders the glyph.
    [[nodiscard]] constexpr char32_t codepoint(std::string_view name)
    {
        for (const auto& glyph : glyphs)
            if (glyph.name == name)
                return glyph.codepoint;
        return 0;
    }

    [[nodiscard]] inline std::string utf8(std::string_view name)
    {
        const auto code = codepoint(name);
        if (code == 0)
            return {};
        std::string text;
        text.push_back(static_cast<char>(0xe0 | ((code >> 12) & 0x0f)));
        text.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3f)));
        text.push_back(static_cast<char>(0x80 | (code & 0x3f)));
        return text;
    }
}
