#pragma once

#include <rigidbodies/render/draw_list.hpp>

#include <functional>

namespace rigidbodies::render
{
    struct DrawCompileOptions
    {
        float feather { 1.0f };
        // The target resolves several coverage samples per pixel. Pixel-aligned interface meshes
        // then rely on those samples for their curved edges and take no feather band at all, so
        // corners keep the same weight as the hard straight edges beside them.
        bool multisampled { false };
        float curve_tolerance { 0.25f };
        std::size_t max_batch_vertices { 65536 };
    };

    struct DrawCompileStats
    {
        std::size_t command_count { 0 }, mesh_count { 0 }, triangle_count { 0 };
    };

    struct CompiledDrawList
    {
        std::vector<IndexedMesh> meshes;
        DrawCompileStats stats;
    };

    using TextMeshBuilder = std::function<std::vector<IndexedMesh>(Vec2, std::string_view, Color, float)>;

    // Maximum arc sagitta in device pixels, bounded to 4096 segments for pathological input.
    [[nodiscard]] int curve_segment_count(float radius, float tolerance = 0.25f, float arc_radians = static_cast<float>(math::two_pi));

    // Shared by every graphics backend. Input helper colours are straight alpha; generated
    // vertices and supplied IndexedMesh data use premultiplied alpha. Stable layer ordering and
    // adjacent-only batching preserve the order of translucent surfaces.
    [[nodiscard]] CompiledDrawList compile_draw_list(const DrawList& list, const TextMeshBuilder& text_builder = {}, const DrawCompileOptions& options = {});
}
