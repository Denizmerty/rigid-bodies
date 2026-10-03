#include <rigidbodies/render/camera2d.hpp>

#include <algorithm>

namespace rigidbodies::render
{

    Real ViewportSize::aspect_ratio() const
    {
        return height > 0 ? static_cast<Real>(width) / static_cast<Real>(height) : 1.0;
    }

    Vec2 ScreenRect::center() const
    {
        return { left + width * 0.5, top + height * 0.5 };
    }

    bool ScreenRect::empty() const
    {
        return width <= 0.0 || height <= 0.0;
    }

    const Vec2& Camera2D::center_m() const
    {
        return center_m_;
    }

    void Camera2D::set_center(const Vec2& value)
    {
        if (math::is_finite(value))
        {
            center_m_ = value;
        }
    }

    Real Camera2D::view_height_m() const
    {
        return view_height_m_;
    }

    void Camera2D::set_view_height(Real value)
    {
        if (!math::is_finite(value) || value <= 0.0)
        {
            return;
        }
        view_height_m_ = math::clamp(value, minimum_view_height_m_, maximum_view_height_m_);
    }

    const ViewportSize& Camera2D::viewport() const
    {
        return viewport_;
    }

    void Camera2D::set_viewport(const ViewportSize& value)
    {
        // A zero-sized viewport arrives when the window is minimised. Clamping to one pixel keeps
        // the projection finite instead of propagating a division by zero through every transform.
        viewport_.width = std::max(value.width, 1);
        viewport_.height = std::max(value.height, 1);
    }

    void Camera2D::set_focus_rect(const ScreenRect& rect)
    {
        focus_rect_ = rect;
    }

    void Camera2D::set_focus_center_world(const Vec2& world_point_m)
    {
        if (!math::is_finite(world_point_m))
            return;
        const auto target = focus_rect_.empty() ? Vec2 { static_cast<Real>(viewport_.width) * 0.5, static_cast<Real>(viewport_.height) * 0.5 } : focus_rect_.center();
        center_m_ += world_point_m - screen_to_world(target);
    }

    const ScreenRect& Camera2D::focus_rect() const
    {
        return focus_rect_;
    }

    Real Camera2D::minimum_view_height_m() const
    {
        return minimum_view_height_m_;
    }

    Real Camera2D::maximum_view_height_m() const
    {
        return maximum_view_height_m_;
    }

    void Camera2D::set_view_height_limits(Real minimum_m, Real maximum_m)
    {
        if (!math::is_finite(minimum_m) || !math::is_finite(maximum_m) || minimum_m <= 0.0 || maximum_m <= minimum_m)
        {
            return;
        }
        minimum_view_height_m_ = minimum_m;
        maximum_view_height_m_ = maximum_m;
        view_height_m_ = math::clamp(view_height_m_, minimum_view_height_m_, maximum_view_height_m_);
    }

    Real Camera2D::pixels_per_metre() const
    {
        return static_cast<Real>(viewport_.height) / view_height_m_;
    }

    Vec2 Camera2D::world_to_screen(const Vec2& world_point_m) const
    {
        const auto scale = pixels_per_metre();
        const auto offset = world_point_m - center_m_;
        return { static_cast<Real>(viewport_.width) * 0.5 + offset.x * scale, static_cast<Real>(viewport_.height) * 0.5 - offset.y * scale };
    }

    Vec2 Camera2D::screen_to_world(const Vec2& screen_point_px) const
    {
        const auto scale = pixels_per_metre();
        const auto offset = Vec2 { screen_point_px.x - static_cast<Real>(viewport_.width) * 0.5, static_cast<Real>(viewport_.height) * 0.5 - screen_point_px.y };
        return center_m_ + offset / scale;
    }

    Vec2 Camera2D::world_to_screen_direction(const Vec2& world_direction) const
    {
        const auto scale = pixels_per_metre();
        return { world_direction.x * scale, -world_direction.y * scale };
    }

    Vec2 Camera2D::screen_to_world_direction(const Vec2& screen_direction) const
    {
        const auto scale = pixels_per_metre();
        return { screen_direction.x / scale, -screen_direction.y / scale };
    }

    Real Camera2D::world_to_screen_length(Real world_length_m) const
    {
        return world_length_m * pixels_per_metre();
    }

    Real Camera2D::screen_to_world_length(Real screen_length_px) const
    {
        return screen_length_px / pixels_per_metre();
    }

    Aabb Camera2D::visible_bounds_m() const
    {
        const auto half_height = view_height_m_ * 0.5;
        const auto half_width = half_height * viewport_.aspect_ratio();

        Aabb bounds;
        bounds.expand(center_m_ - Vec2 { half_width, half_height });
        bounds.expand(center_m_ + Vec2 { half_width, half_height });
        return bounds;
    }

    void Camera2D::pan_by_screen_delta(const Vec2& screen_delta_px)
    {
        // Dragging right should bring the scene right with the pointer, which means the camera
        // moves the opposite way.
        center_m_ -= screen_to_world_direction(screen_delta_px);
    }

    void Camera2D::zoom_about_screen_point(Real factor, const Vec2& screen_point_px)
    {
        if (!math::is_finite(factor) || factor <= 0.0)
        {
            return;
        }

        const auto anchor_before = screen_to_world(screen_point_px);
        set_view_height(view_height_m_ * factor);
        const auto anchor_after = screen_to_world(screen_point_px);
        center_m_ += anchor_before - anchor_after;
    }

    void Camera2D::frame_bounds(const Aabb& bounds_m, Real margin_fraction)
    {
        if (bounds_m.is_empty())
        {
            return;
        }

        const auto extents = bounds_m.extents();
        const auto margin = std::max(margin_fraction, 0.0);
        if (focus_rect_.empty())
        {
            // Preserve the original whole-viewport calculation exactly for callers that have not
            // published a focus rectangle.
            center_m_ = bounds_m.center();
            const auto required_height = extents.y * (1.0 + 2.0 * margin);
            const auto aspect = viewport_.aspect_ratio();
            const auto required_from_width = aspect > 0.0 ? extents.x * (1.0 + 2.0 * margin) / aspect : required_height;
            set_view_height(std::max({ required_height, required_from_width, minimum_view_height_m_ }));
            return;
        }

        const auto focus_aspect = focus_rect_.width / focus_rect_.height;
        const auto required_height = extents.y * (1.0 + 2.0 * margin);
        const auto required_from_width = focus_aspect > 0.0 ? extents.x * (1.0 + 2.0 * margin) / focus_aspect : required_height;
        const auto focus_height_m = std::max(required_height, required_from_width);
        set_view_height(std::max(focus_height_m * static_cast<Real>(viewport_.height) / focus_rect_.height, minimum_view_height_m_));

        const auto pixels_per_m = pixels_per_metre();
        const auto viewport_center = Vec2 { static_cast<Real>(viewport_.width) * 0.5, static_cast<Real>(viewport_.height) * 0.5 };
        const auto screen_offset = focus_rect_.center() - viewport_center;
        center_m_ = bounds_m.center() - Vec2 { screen_offset.x / pixels_per_m, -screen_offset.y / pixels_per_m };
    }

} // namespace rigidbodies::render
