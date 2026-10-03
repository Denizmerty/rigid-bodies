#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <rigidbodies/render/color.hpp>
#include <rigidbodies/render/draw_list.hpp>
#include <rigidbodies/render/scene_renderer.hpp>

#include <string_view>
#include <vector>

namespace rigidbodies::app
{
    // The App draws handles, guides, value chips, the shape editor and the presentation spotlight
    // on top of the scene. They share one vocabulary so that they read as part of the same
    // product as the scene: every size below is in logical pixels and is multiplied by the
    // overlay scale (display density times text size), line weights follow one hairline /
    // standard / emphasis hierarchy, and colours come from the active scene theme. The theme has
    // no danger or raised-surface role, so those two follow the brightness of its stage.
    namespace overlay
    {
        inline constexpr double hairline = 1.0;
        inline constexpr double standard = 1.5;
        inline constexpr double emphasis = 2.0;
        inline constexpr double dash_on = 4.0;
        inline constexpr double dash_off = 3.0;
        inline constexpr double knob_radius = 5.0;
        inline constexpr double vertex_half_size = 4.0;
        // Point handles keep a 24 px target and lines a 20 px band, so every pointer target stays
        // at least 16 logical pixels across however small the drawn handle is.
        inline constexpr double handle_hit_radius = 12.0;
        inline constexpr double edge_hit_distance = 10.0;
        inline constexpr double snap_radius = 10.0;
        // 11.2 px Inter Medium on the scene font atlas, whose unit scale is 14 px.
        inline constexpr double label_text_scale = 0.8;
        inline constexpr double compass_radius = 17.0;
        inline constexpr double rotation_ring_minimum_radius = 24.0;
        inline constexpr double rotation_grip_radius = 4.0;
        // An arrow too short to clear the centre-of-mass symbol parks its knob this far out, so
        // the symbol and the body's natural grab point stay free while the knob stays inside
        // even the smallest rotation ring.
        inline constexpr double velocity_knob_park_distance = 16.0;
        // Present mode sets stage text, chips and handles about as much larger as the Present
        // strip sets its own type, so a room can read them.
        inline constexpr double present_scale = 1.4;
        // Stage overlays sort above every scene layer, and the spotlight dims them as well. The
        // drawing veil sits beneath the scale bar so the bar stays readable while drawing.
        inline constexpr int overlay_layer = 40;
        inline constexpr int spotlight_layer = 50;
        inline constexpr int veil_layer = render::instrument_layer - 1;
        // The scene draws vector arrows and stage labels on layer 20. The selection ring sorts
        // just beneath them, so a label's plate covers the ring instead of the ring cutting
        // through its text.
        inline constexpr int selection_ring_layer = 19;
    }

    struct OverlayPalette
    {
        render::Color accent, selection, surface, plate, text, muted, shadow, danger;
        bool dark { true };
    };
    [[nodiscard]] OverlayPalette overlay_palette(const render::Theme& theme);

    enum class HandleState
    {
        normal,
        hover,
        active
    };

    // Where a chip sits relative to its anchor. The anchor is the vertical centre of the text.
    enum class ChipAlign
    {
        left,
        centre,
        right
    };

    // Small drawing vocabulary built only from DrawList primitives. Dashed and tapered strokes are
    // emitted as one indexed mesh per path, so a long dashed outline costs one command rather
    // than one per dash.
    class OverlayPainter
    {
    public:
        OverlayPainter(render::DrawList& list, const render::Theme& theme, double scale);

        [[nodiscard]] double px(double logical) const;
        [[nodiscard]] float width(double logical) const;
        [[nodiscard]] const OverlayPalette& palette() const;
        [[nodiscard]] render::DrawList& list() const;

        // Widths and the dash pattern are logical pixels. Closed paths stretch the pattern
        // slightly so that whole dashes meet at the seam.
        void dashed_path(const std::vector<math::Vec2>& points, bool closed, const render::Color& color, double width_logical,
            double on_logical = overlay::dash_on, double off_logical = overlay::dash_off) const;
        void dashed_circle(const math::Vec2& centre, double radius_px, const render::Color& color, double width_logical,
            double on_logical = overlay::dash_on, double off_logical = overlay::dash_off) const;
        // Width and opacity fall linearly from the first point to the last.
        void tapered_dashes(const std::vector<math::Vec2>& points, const render::Color& color, double start_width_logical,
            double end_width_logical, float end_opacity, double on_logical, double off_logical) const;

        void halo(const math::Vec2& centre, double radius_px, const render::Color& color) const;
        void knob(const math::Vec2& centre, HandleState state, const render::Color& ring) const;
        // A solid dot, so the rotation grip never reads as the hollow velocity knob.
        void grip(const math::Vec2& centre, HandleState state, const render::Color& fill) const;
        void square_handle(const math::Vec2& centre, HandleState state, bool selected) const;
        void reticle(const math::Vec2& centre, const render::Color& color) const;

        void chip(const math::Vec2& anchor, std::string_view text, ChipAlign align, const render::Color& text_color) const;
        // An estimate from Inter's advance widths, good to a few pixels, for placing chips whose
        // plate is measured exactly from the glyphs when the frame is compiled.
        [[nodiscard]] double chip_text_width(std::string_view text) const;
        [[nodiscard]] double chip_height() const;

    private:
        render::DrawList* list_;
        OverlayPalette palette_;
        double scale_;
    };

    // Geometry shared by each overlay and the pointer hit test for the same handle, so what is
    // drawn and what responds to the pointer can never drift apart.
    [[nodiscard]] math::Vec2 gravity_compass_centre(const render::Camera2D& camera, double scale);
    // Screen-space vectors use the same pixels per unit as the scene's vector arrows.
    [[nodiscard]] math::Vec2 velocity_handle_tip(const render::Camera2D& camera, const math::Vec2& centre_m,
        const math::Vec2& velocity_m_s, double pixels_per_m_s);
    [[nodiscard]] math::Vec2 velocity_from_handle(const render::Camera2D& camera, const math::Vec2& centre_m,
        const math::Vec2& pointer_px, double pixels_per_m_s);
    // Where the idle velocity knob is drawn and grabbed: the arrow's tip, or parked along the
    // velocity (to the right at rest) when the tip lies too close to the centre of mass. Scale
    // is the overlay scale; a drag still puts the tip under the pointer.
    [[nodiscard]] math::Vec2 velocity_knob_position(const render::Camera2D& camera, const math::Vec2& centre_m,
        const math::Vec2& velocity_m_s, double pixels_per_m_s, double scale);
    // Measured in the body's own frame, so the ring keeps its size while the body turns.
    [[nodiscard]] double rotation_ring_radius(const render::Camera2D& camera, const physics::RigidBody& body, double scale);
    // The rotation knob marks the body's local up axis, like the rotate grip above a selection.
    [[nodiscard]] math::Vec2 rotation_knob_position(const math::Vec2& centre_px, double radius_px, double orientation_rad);
    [[nodiscard]] double orientation_from_knob(const math::Vec2& centre_px, const math::Vec2& pointer_px);

    // Shades the viewport outside a clear circle, with a smooth feathered edge between them.
    void add_spotlight(render::DrawList& list, const render::ViewportSize& viewport, const math::Vec2& centre_px,
        double clear_radius_px, double feather_px, const render::Color& shade);
}
