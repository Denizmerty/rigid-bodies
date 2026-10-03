#pragma once

#include <rigidbodies/math/span.hpp>

#include <cstdint>
#include <string_view>

namespace rigidbodies::render
{

    // What the scene renderer may draw on top of the objects themselves.
    //
    // Each layer is one physical quantity made visible. They are separate bits rather than a single
    // verbosity setting because the point of the playground is to let a viewer isolate one quantity
    // at a time: showing velocity and force together explains something different from showing
    // either alone, and both differ from showing everything at once.
    enum class VisualizationLayer : std::uint32_t
    {
        grid = 1u << 0,
        bodies = 1u << 1,
        outlines = 1u << 2,
        center_of_mass = 1u << 3,
        velocity_vectors = 1u << 4,
        acceleration_vectors = 1u << 5,
        force_vectors = 1u << 6,
        momentum_vectors = 1u << 7,
        angular_velocity = 1u << 8,
        contact_points = 1u << 9,
        contact_normals = 1u << 10,
        trajectories = 1u << 11,
        bounding_boxes = 1u << 12,
        broad_phase_pairs = 1u << 13,
        constraints = 1u << 14,
        labels = 1u << 15,
        selection = 1u << 16
    };

    struct LayerDescription
    {
        VisualizationLayer layer;
        std::string_view id;
        std::string_view title;

        // One line explaining what the layer shows, used as the tooltip beside its toggle.
        std::string_view summary;
    };

    [[nodiscard]] math::Span<const LayerDescription> layer_catalogue();

    [[nodiscard]] const LayerDescription* find_layer(std::string_view id);

    // A set of enabled layers.
    class LayerMask
    {
    public:
        LayerMask() = default;
        explicit LayerMask(std::uint32_t bits);

        [[nodiscard]] static LayerMask defaults();
        [[nodiscard]] static LayerMask all();
        [[nodiscard]] static LayerMask none();

        [[nodiscard]] bool is_enabled(VisualizationLayer layer) const;
        void set(VisualizationLayer layer, bool enabled);
        void toggle(VisualizationLayer layer);

        [[nodiscard]] std::uint32_t bits() const;

    private:
        std::uint32_t bits_ { 0 };
    };

} // namespace rigidbodies::render
