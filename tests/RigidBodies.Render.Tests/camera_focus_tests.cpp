#include <rigidbodies/render/camera2d.hpp>

#include "test_framework.hpp"

#include <algorithm>

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("focus framing centres and fits bounds with its margin")
    {
        render::Camera2D camera;
        camera.set_viewport({ 1600, 900 });
        const render::ScreenRect focus { 504.0, 36.0, 592.0, 420.0 };
        camera.set_focus_rect(focus);
        math::Aabb bounds;
        bounds.expand({ -2.0, -1.0 });
        bounds.expand({ 2.0, 1.0 });
        camera.frame_bounds(bounds, 0.1);
        const auto centre = camera.world_to_screen(bounds.center());
        RIGIDBODIES_EXPECT_NEAR(centre.x, focus.center().x, 1.0e-10, "horizontal focus centre");
        RIGIDBODIES_EXPECT_NEAR(centre.y, focus.center().y, 1.0e-10, "vertical focus centre");
        const auto minimum = camera.world_to_screen(bounds.minimum);
        const auto maximum = camera.world_to_screen(bounds.maximum);
        RIGIDBODIES_EXPECT(minimum.x >= focus.left && maximum.x <= focus.left + focus.width, "horizontal fit");
        RIGIDBODIES_EXPECT(maximum.y >= focus.top && minimum.y <= focus.top + focus.height, "vertical fit");
    }

    RIGIDBODIES_TEST("an empty focus rectangle preserves whole viewport framing exactly")
    {
        render::Camera2D camera;
        camera.set_viewport({ 800, 600 });
        math::Aabb bounds;
        bounds.expand({ -3.0, -2.0 });
        bounds.expand({ 5.0, 4.0 });
        camera.frame_bounds(bounds, 0.15);
        RIGIDBODIES_EXPECT(camera.center_m() == bounds.center(), "old centre is bit exact");
        const auto expected = std::max(bounds.extents().y * 1.3, bounds.extents().x * 1.3 / (800.0 / 600.0));
        RIGIDBODIES_EXPECT(camera.view_height_m() == expected, "old fit is bit exact");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
