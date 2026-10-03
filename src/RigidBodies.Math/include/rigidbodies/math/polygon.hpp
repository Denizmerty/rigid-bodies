#pragma once

#include <rigidbodies/math/aabb.hpp>
#include <rigidbodies/math/span.hpp>
#include <rigidbodies/math/vec2.hpp>

#include <vector>

namespace rigidbodies::math
{

    // Geometry helpers shared by collision shapes, mass-property calculation, and shape authoring.
    // They operate on a plain span of vertices so that callers are free
    // to keep their own storage, and they make no assumption about winding beyond what each
    // function documents.
    using VertexSpan = Span<const Vec2>;

    // Signed area of a closed polygon. Positive for counter-clockwise winding, negative for
    // clockwise, and near zero for degenerate outlines.
    [[nodiscard]] Real signed_area(VertexSpan vertices);

    [[nodiscard]] Vec2 centroid(VertexSpan vertices);

    // Second moment of area about the axis normal to the plane, taken about the polygon centroid
    // and expressed per unit area. Multiplying by an areal density yields rotational inertia.
    [[nodiscard]] Real second_moment_of_area(VertexSpan vertices);

    [[nodiscard]] Aabb compute_bounds(VertexSpan vertices);

    // Reports whether the outline is convex and free of repeated vertices. Concave outlines are
    // valid input to the authoring pipeline but have to be decomposed before collision use.
    [[nodiscard]] bool is_convex(VertexSpan vertices);

    // Returns a copy wound counter-clockwise, which is the orientation every collider expects.
    [[nodiscard]] std::vector<Vec2> ensure_counter_clockwise(VertexSpan vertices);

    // Farthest vertex along a direction, the primitive the separating-axis and clipping routines
    // are built on.
    [[nodiscard]] Vec2 support_point(VertexSpan vertices, const Vec2& direction);

    // Outward unit normals, one per edge, for a counter-clockwise outline.
    [[nodiscard]] std::vector<Vec2> edge_normals(VertexSpan vertices);

} // namespace rigidbodies::math
