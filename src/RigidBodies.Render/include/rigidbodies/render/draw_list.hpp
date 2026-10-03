#pragma once

#include <rigidbodies/math/vec2.hpp>
#include <rigidbodies/render/color.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::render
{

    using math::Vec2;

    // A recorded drawing command. Commands hold positions in the device coordinate space, in
    // pixels with the origin at the top-left corner, so that a list can be built by scene code
    // that has already projected through a camera and by interface code that works in pixels
    // directly, without either one knowing about the other.
    enum class DrawCommandKind
    {
        line,
        polyline,
        polygon_fill,
        polygon_outline,
        circle_fill,
        circle_outline,
        rectangle_fill,
        rectangle_outline,
        text,
        indexed_mesh,
        rounded_rectangle_fill,
        rounded_rectangle_outline,
        gradient_polygon_fill,
        gradient_circle_fill,
        shadow,
        rounded_rectangle_shadow,
        textured_quad,
        instanced_mesh,
        tapered_polyline
    };

    enum class StrokeJoin
    {
        miter,
        bevel,
        round
    };
    enum class StrokeCap
    {
        butt,
        square,
        round
    };

    struct TexturePixels
    {
        int width { 0 }, height { 0 };
        // Premultiplied RGBA8, matching the document renderer's glyph atlas.
        std::vector<std::uint8_t> rgba;
    };
    struct MeshVertex
    {
        Vec2 position, uv;
        // Premultiplied RGBA, including untextured geometry. DrawList helper inputs use
        // ordinary straight-alpha Color; the compiler performs this conversion once.
        Color color;
    };
    struct MeshClip
    {
        int x { 0 }, y { 0 }, width { 0 }, height { 0 };
    };
    struct MeshInstance
    {
        Vec2 translation;
        Vec2 scale { 1.0, 1.0 };
        float rotation { 0.0f };
        // A straight-alpha tint. Devices premultiply it before modulating mesh vertices.
        Color tint;
    };
    struct IndexedMesh
    {
        std::vector<MeshVertex> vertices;
        std::vector<int> indices;
        std::shared_ptr<const TexturePixels> texture;
        std::optional<MeshClip> clip;
        std::vector<MeshInstance> instances;
        // Set when every exposed edge already fades to zero alpha, as in shaded scene surfaces,
        // ribbons and soft shadows built by their producer. The compiler then skips its
        // boundary weld and adds no second antialiasing band.
        bool feathered { false };
        // Set by producers whose vertices are device pixels snapped to the pixel grid, as in
        // laid-out interface boxes. Axis-aligned edges on that grid are already crisp, so only
        // the remaining edges (rounded corners, diagonals) receive an antialiasing band.
        bool pixel_aligned { false };
    };

    // Head shapes for add_arrow. Distinct silhouettes keep vector quantities apart for viewers
    // who cannot rely on hue: filled for velocity-like quantities, open for rates of change,
    // doubled for accumulated quantities, and an outlined triangle for a load that shares its
    // hue with a filled one.
    enum class ArrowHead
    {
        filled,
        open,
        double_filled,
        hollow
    };

    struct DrawCommand
    {
        DrawCommandKind kind { DrawCommandKind::line };
        Color color;

        // Offset and count into the shared vertex buffer. Keeping the points in one contiguous
        // buffer is what lets a frame be built without an allocation per command.
        std::size_t vertex_offset { 0 };
        std::size_t vertex_count { 0 };

        // Line width in pixels, or the radius for the circle commands.
        float thickness { 1.0f };
        float radius { 0.0f };

        // Offset and length into the shared text buffer, for the text command.
        std::size_t text_offset { 0 };
        std::size_t text_length { 0 };
        float text_scale { 1.0f };
        std::optional<Color> text_background;
        float text_padding { 4.0f };
        std::shared_ptr<const IndexedMesh> mesh;
        std::shared_ptr<const TexturePixels> texture;
        Color gradient_end;
        Vec2 gradient_from, gradient_to, shadow_offset;
        Vec2 uv_min {}, uv_max { 1.0, 1.0 };
        float blur_radius { 8.0f };
        StrokeJoin join { StrokeJoin::round };
        StrokeCap cap { StrokeCap::round };
        bool closed { false };
        int layer { 0 };
        std::optional<MeshClip> clip;
        // Dash pattern in pixels along the path for stroked commands; zero draws a solid stroke.
        float dash_length { 0.0f };
        float gap_length { 0.0f };
    };

    // A frame of drawing work, recorded once and replayed by a device.
    //
    // Separating what to draw from how it is drawn is what keeps the scene and interface layers
    // free of any graphics API, and it is what will allow the device behind them to be replaced
    // without touching either one.
    class DrawList
    {
    public:
        void clear();

        [[nodiscard]] bool is_empty() const;
        [[nodiscard]] const std::vector<DrawCommand>& commands() const;
        [[nodiscard]] const std::vector<Vec2>& vertices() const;
        [[nodiscard]] const std::string& text_buffer() const;

        void add_line(const Vec2& from, const Vec2& to, const Color& color, float thickness = 1.0f);
        void add_polyline(const std::vector<Vec2>& points, const Color& color, float thickness = 1.0f, bool closed = false);

        // The polygon commands assume a convex outline, which every collider produced by the shape
        // layer satisfies. Concave outlines are decomposed before they reach a renderer.
        void add_polygon_fill(const std::vector<Vec2>& points, const Color& color);
        void add_polygon_outline(const std::vector<Vec2>& points, const Color& color, float thickness = 1.0f);

        void add_circle_fill(const Vec2& center, float radius, const Color& color);
        void add_circle_outline(const Vec2& center, float radius, const Color& color, float thickness = 1.0f);

        void add_rectangle_fill(const Vec2& minimum, const Vec2& maximum, const Color& color);
        void add_rectangle_outline(const Vec2& minimum, const Vec2& maximum, const Color& color, float thickness = 1.0f);

        void add_text(const Vec2& position, std::string_view text, const Color& color, float scale = 1.0f);
        // The compiler measures actual glyph ink, so proportional text, units and display
        // scaling always receive a matching contrast surface rather than a guessed width.
        void add_text_label(const Vec2& position, std::string_view text, const Color& color, const Color& background, float scale = 1.0f, float padding = 4.0f);
        void add_indexed_mesh(std::shared_ptr<const IndexedMesh> mesh);
        void add_instanced_mesh(const std::shared_ptr<const IndexedMesh>& mesh, std::vector<MeshInstance> instances);

        // Layer sorting is stable. Only adjacent compatible commands are batched, so transparent
        // surfaces retain painter order within their layer. Clipping nests by intersection.
        void set_layer(int layer);
        [[nodiscard]] int layer() const;
        void push_clip(MeshClip clip);
        void pop_clip();
        void add_path(const std::vector<Vec2>& points, const Color& color, float thickness, bool closed = false,
            StrokeJoin join = StrokeJoin::round, StrokeCap cap = StrokeCap::round);
        void add_rounded_rectangle_fill(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color);
        void add_rounded_rectangle_outline(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color, float thickness = 1.0f);
        void add_gradient_polygon_fill(const std::vector<Vec2>& points, const Color& start, const Color& end, const Vec2& gradient_from, const Vec2& gradient_to);
        void add_gradient_circle_fill(const Vec2& center, float radius, const Color& start, const Color& end, const Vec2& gradient_from, const Vec2& gradient_to);
        void add_shadow(const std::vector<Vec2>& points, const Color& color, float blur_radius = 8.0f, const Vec2& offset = { 0.0, 3.0 });
        void add_rounded_rectangle_shadow(const Vec2& minimum, const Vec2& maximum, float radius, const Color& color, float blur_radius = 8.0f, const Vec2& offset = { 0.0, 3.0 });
        void add_textured_quad(std::shared_ptr<const TexturePixels> texture, const Vec2& minimum, const Vec2& maximum, const Color& tint = {}, const Vec2& uv_min = {}, const Vec2& uv_max = { 1.0, 1.0 });

        // A line with a head at the far end, the shape every vector quantity is drawn with. The
        // tip lands exactly on `to`; the butt-capped shaft stops inside the notched head so the
        // point stays crisp. Short arrows shrink the head rather than turning into a blob.
        void add_arrow(const Vec2& from, const Vec2& to, const Color& color, float thickness = 1.0f, float head_length = 10.0f);
        // As above with a chosen head silhouette. A zero head_width uses the house proportion
        // (0.42 of the head length either side of the shaft). A positive shaft_dash draws the
        // shaft dotted, in dashes and gaps of that many pixels; the head stays solid.
        void add_arrow(const Vec2& from, const Vec2& to, const Color& color, float thickness, float head_length, ArrowHead head, float head_width = 0.0f, float shaft_dash = 0.0f);

        // A small cross, used for points that mark a location rather than an extent.
        void add_cross(const Vec2& center, float radius, const Color& color, float thickness = 1.0f);

        // A straight segment with an explicit end cap. Butt caps end exactly at the endpoints.
        void add_line(const Vec2& from, const Vec2& to, const Color& color, float thickness, StrokeCap cap);

        // Dashed strokes, compiled into one mesh per call. Lengths are device pixels measured
        // along the path from its first point; the house pattern is 4 on, 3 off logical pixels.
        void add_dashed_line(const Vec2& from, const Vec2& to, const Color& color, float thickness, float dash_length, float gap_length);
        void add_dashed_polyline(const std::vector<Vec2>& points, const Color& color, float thickness, float dash_length, float gap_length, bool closed = false);

        // An open stroke whose width and colour interpolate by arc length from the first point
        // to the last, with butt ends. Used for tapered motion ribbons and fading trails; either
        // width may be zero.
        void add_tapered_polyline(const std::vector<Vec2>& points, const Color& start_color, const Color& end_color, float start_thickness, float end_thickness);

        // A soft circular shadow: a full-strength disc that falls linearly to zero over
        // blur_radius beyond its edge. Pair with add_shadow for polygonal outlines.
        void add_circle_shadow(const Vec2& center, float radius, const Color& color, float blur_radius = 8.0f, const Vec2& offset = { 0.0, 3.0 });

    private:
        std::size_t push_vertices(const Vec2* points, std::size_t count);
        void push_command(DrawCommand command);

        std::vector<DrawCommand> commands_;
        std::vector<Vec2> vertices_;
        std::string text_buffer_;
        int layer_ { 0 };
        std::vector<MeshClip> clips_;
    };

} // namespace rigidbodies::render
