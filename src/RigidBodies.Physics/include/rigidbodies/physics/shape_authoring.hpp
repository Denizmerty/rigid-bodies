#pragma once

#include <rigidbodies/math/polygon.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace rigidbodies::physics
{
    using math::Real;

    enum class OutlineEdgeKind
    {
        line,
        cubic
    };
    enum class OutlineContinuity
    {
        corner,
        aligned,
        mirrored
    };

    struct OutlineNode
    {
        math::Vec2 position_m {};
        // Handle offsets relative to position_m, so moving a node carries both handles with it.
        math::Vec2 incoming_handle_m {};
        math::Vec2 outgoing_handle_m {};
        OutlineEdgeKind outgoing_edge { OutlineEdgeKind::line };
        OutlineContinuity continuity { OutlineContinuity::corner };
    };

    struct Outline
    {
        std::vector<OutlineNode> nodes;
        bool closed { false };
    };

    struct ShapeAuthoringOptions
    {
        Real render_tolerance_m { 0.002 };
        Real collision_tolerance_m { 0.01 };
        Real simplification_tolerance_m { 0.002 };
        Real concavity_tolerance_m { 0.0 };
        std::size_t max_render_vertices { 512 };
        std::size_t max_collision_vertices { 128 };
        unsigned max_subdivision_depth { 16 };
        Real minimum_area_m2 { 1.0e-8 };
        Real duplicate_tolerance_m { 1.0e-7 };
    };

    enum class OutlineDiagnosticCode
    {
        open_outline,
        insufficient_vertices,
        non_finite,
        invalid_options,
        degenerate_edge,
        self_intersection,
        insufficient_area,
        vertex_budget,
        subdivision_budget,
        decomposition_failed
    };

    struct OutlineDiagnostic
    {
        OutlineDiagnosticCode code { OutlineDiagnosticCode::insufficient_vertices };
        std::string message;
        std::size_t edge_index {};
    };

    struct AuthoredShape
    {
        Outline source;
        ShapeAuthoringOptions options;
        std::vector<math::Vec2> render_outline;
        std::vector<std::array<math::Vec2, 3>> render_triangles;
        std::vector<math::Vec2> collision_outline;
        std::vector<std::vector<math::Vec2>> convex_parts;
        Real approximation_added_area_m2 {};
        // Conservative sum of collision tessellation, simplification, and decomposition error.
        Real max_collision_error_m {};
    };

    struct ShapeBuildResult
    {
        std::shared_ptr<const AuthoredShape> shape;
        std::vector<OutlineDiagnostic> diagnostics;
        [[nodiscard]] bool succeeded() const
        {
            return shape != nullptr;
        }
    };

    [[nodiscard]] ShapeBuildResult build_authored_shape(const Outline& outline, const ShapeAuthoringOptions& options = {});

    // A bounded, single-entry cache owned by an editor/document, never by the simulation loop.
    // Unchanged source and options return the same immutable result, including failure diagnostics.
    class ShapeAuthoringCache
    {
    public:
        [[nodiscard]] ShapeBuildResult build(const Outline& outline, const ShapeAuthoringOptions& options = {});
        void clear();
        [[nodiscard]] std::size_t build_count() const;

    private:
        Outline source_;
        ShapeAuthoringOptions options_;
        ShapeBuildResult result_;
        std::size_t build_count_ {};
        bool populated_ { false };
    };

    // Invalid indices return false (insertion returns nodes.size()). Non-finite edits throw before
    // mutation. Splitting a cubic uses de Casteljau and preserves its exact curve; an endpoint's
    // mirrored join becomes aligned when its split-side handle changes length.
    bool move_outline_node(Outline& outline, std::size_t index, const math::Vec2& position_m);
    [[nodiscard]] std::size_t insert_outline_node(Outline& outline, std::size_t edge_index, Real parameter = 0.5);
    bool remove_outline_node(Outline& outline, std::size_t index);
    bool set_outline_edge_kind(Outline& outline, std::size_t index, OutlineEdgeKind kind);
    bool set_outline_continuity(Outline& outline, std::size_t index, OutlineContinuity continuity);
    bool move_outline_handle(Outline& outline, std::size_t index, bool incoming, const math::Vec2& offset_m);
}
