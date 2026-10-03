#pragma once

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/shape_authoring.hpp>
#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/ui/ui_command.hpp>
#include <rigidbodies/ui/ui_event.hpp>

#include <algorithm>
#include <optional>

namespace rigidbodies::app
{
    // A retained draft in its original part-local frame. Building and drawing a draft never
    // changes the World; the session owns the explicit, validated commit transaction.
    class ShapeEditor
    {
    public:
        void begin(physics::Outline outline = {}, math::Transform2 placement = {},
            std::optional<physics::ShapeAuthoringOptions> options = std::nullopt);
        void end();
        [[nodiscard]] bool active() const;
        [[nodiscard]] bool has_pointer_capture() const;
        void release_pointer()
        {
            drag_ = DragKind::none;
        }
        [[nodiscard]] const physics::Outline& outline() const;
        [[nodiscard]] const physics::ShapeAuthoringOptions& options() const;
        [[nodiscard]] const math::Transform2& placement() const;
        [[nodiscard]] std::optional<std::size_t> selected_node() const;
        void deselect_node()
        {
            selected_.reset();
            release_pointer();
        }
        [[nodiscard]] physics::ShapeBuildResult build() const;
        [[nodiscard]] std::string diagnostic() const;

        // interface_consumed blocks new scene interactions, but an existing drag still receives
        // its move/release events so releasing above a panel cannot leave a captured pointer.
        bool handle_event(const ui::UiEvent& event, const render::Camera2D& camera,
            const std::vector<math::Vec2>& existing_vertices, bool interface_consumed = false);
        bool apply(const ui::UiCommand& command);
        void draw(const render::Camera2D& camera, const render::Theme& theme, render::DrawList& list,
            core::DisplayUnits units = core::DisplayUnits::si) const;

        [[nodiscard]] bool snap_grid() const;
        [[nodiscard]] bool snap_vertices() const;
        [[nodiscard]] bool snap_angles() const;
        [[nodiscard]] double grid_spacing_m() const;

        // Handle sizes and snap tolerances follow the display's pixel density and the text-size
        // preference so the editor keeps its proportions on high-DPI monitors.
        void set_view_scale(double scale)
        {
            view_scale_ = std::clamp(scale, 0.25, 8.0);
        }

    private:
        enum class DragKind
        {
            none,
            node,
            incoming,
            outgoing
        };
        // Which rule placed the pointer, so the overlay can show the guide that explains it.
        enum class SnapKind
        {
            none,
            grid,
            angle,
            vertex
        };
        struct SnapResult
        {
            math::Vec2 point {};
            SnapKind kind { SnapKind::none };
            std::optional<math::Vec2> origin;
        };
        [[nodiscard]] SnapResult snap(const math::Vec2& world, const render::Camera2D& camera,
            const std::vector<math::Vec2>& existing_vertices, std::optional<std::size_t> moving_node,
            double scale, bool temporary_angle = false) const;
        void track_pointer(const SnapResult& snapped, const math::Vec2& pointer_px);
        [[nodiscard]] math::Vec2 curve_point(std::size_t edge, double t) const;
        [[nodiscard]] bool incoming_visible(std::size_t node) const;
        [[nodiscard]] bool outgoing_visible(std::size_t node) const;
        [[nodiscard]] bool edge_exists(std::size_t edge) const;

        physics::Outline outline_;
        physics::ShapeAuthoringOptions options_;
        math::Transform2 placement_;
        mutable physics::ShapeAuthoringCache cache_;
        // An open draft's outline as closing it would make it, previewed while it is drawn.
        mutable physics::Outline closing_preview_;
        mutable physics::ShapeAuthoringCache closing_cache_;
        std::optional<std::size_t> selected_;
        DragKind drag_ { DragKind::none };
        math::Vec2 pointer_world_ {};
        math::Vec2 press_screen_px_ {};
        math::Vec2 pointer_screen_px_ {};
        std::optional<math::Vec2> snap_origin_world_;
        SnapKind pointer_snap_ { SnapKind::none };
        bool pointer_known_ { false };
        bool smooth_node_pending_ { false };
        bool drag_threshold_pending_ { false }, close_pending_ { false }, pointer_shift_ { false };
        bool active_ { false };
        bool snap_grid_ { true };
        bool snap_vertices_ { true };
        bool snap_angles_ { false };
        double grid_spacing_m_ { 0.1 };
        double view_scale_ { 1.0 };
        std::string edit_message_;
    };
}
