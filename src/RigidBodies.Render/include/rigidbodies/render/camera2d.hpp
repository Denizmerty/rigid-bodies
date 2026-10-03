#pragma once

#include <rigidbodies/math/aabb.hpp>
#include <rigidbodies/math/vec2.hpp>

namespace rigidbodies::render
{

    using math::Aabb;
    using math::Real;
    using math::Vec2;

    struct ViewportSize
    {
        int width { 1 };
        int height { 1 };

        [[nodiscard]] Real aspect_ratio() const;
    };

    struct ScreenRect
    {
        Real left {};
        Real top {};
        Real width {};
        Real height {};

        [[nodiscard]] Vec2 center() const;
        [[nodiscard]] bool empty() const;
    };

    // Converts world coordinates in metres (y up) to device coordinates in pixels (y down).
    //
    // The visible world height defines the view. The viewport height sets the pixel scale, and
    // its aspect ratio determines the visible world width.
    class Camera2D
    {
    public:
        Camera2D() = default;

        [[nodiscard]] const Vec2& center_m() const;
        void set_center(const Vec2& value);

        [[nodiscard]] Real view_height_m() const;
        void set_view_height(Real value);

        [[nodiscard]] const ViewportSize& viewport() const;
        void set_viewport(const ViewportSize& value);

        void set_focus_rect(const ScreenRect& rect);
        void set_focus_center_world(const Vec2& world_point_m);
        [[nodiscard]] const ScreenRect& focus_rect() const;

        // Minimum and maximum visible height, in metres, to keep zoom within usable bounds.
        [[nodiscard]] Real minimum_view_height_m() const;
        [[nodiscard]] Real maximum_view_height_m() const;
        void set_view_height_limits(Real minimum_m, Real maximum_m);

        // Pixels per metre implied by the current view height and viewport.
        [[nodiscard]] Real pixels_per_metre() const;

        [[nodiscard]] Vec2 world_to_screen(const Vec2& world_point_m) const;
        [[nodiscard]] Vec2 screen_to_world(const Vec2& screen_point_px) const;

        // Directions carry no origin, so only the scale and the axis flip apply.
        [[nodiscard]] Vec2 world_to_screen_direction(const Vec2& world_direction) const;
        [[nodiscard]] Vec2 screen_to_world_direction(const Vec2& screen_direction) const;

        [[nodiscard]] Real world_to_screen_length(Real world_length_m) const;
        [[nodiscard]] Real screen_to_world_length(Real screen_length_px) const;

        [[nodiscard]] Aabb visible_bounds_m() const;

        // Pans the camera by a drag measured in screen pixels.
        void pan_by_screen_delta(const Vec2& screen_delta_px);

        // Zooms about a fixed screen point, keeping the world point under the pointer in place.
        void zoom_about_screen_point(Real factor, const Vec2& screen_point_px);

        // Frames a region with a margin expressed as a fraction of its size.
        void frame_bounds(const Aabb& bounds_m, Real margin_fraction = 0.15);

    private:
        Vec2 center_m_ {};
        Real view_height_m_ { 4.0 };
        Real minimum_view_height_m_ { 0.05 };
        Real maximum_view_height_m_ { 500.0 };
        ViewportSize viewport_;
        ScreenRect focus_rect_;
    };

} // namespace rigidbodies::render
