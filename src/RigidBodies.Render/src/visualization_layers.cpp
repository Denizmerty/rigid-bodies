#include <rigidbodies/render/visualization_layers.hpp>

#include <array>

namespace rigidbodies::render
{
    namespace
    {

        constexpr std::array<LayerDescription, 16> layer_table {
            LayerDescription { VisualizationLayer::grid, "grid", "Grid", "A distance grid labelled in your chosen units." },
            LayerDescription { VisualizationLayer::bodies, "bodies", "Objects", "The filled shape of every object." },
            LayerDescription { VisualizationLayer::outlines, "outlines", "Outlines", "Outlines of drawn parts and basic shapes. The shape editor also shows the pieces used for collision detection." },
            LayerDescription { VisualizationLayer::center_of_mass, "center_of_mass", "Centre of mass", "The point an object rotates about when nothing holds it." },
            LayerDescription { VisualizationLayer::velocity_vectors, "velocity", "Velocity", "Velocity at the centre of mass, with its value, units and optional signed components." },
            LayerDescription { VisualizationLayer::acceleration_vectors, "acceleration", "Acceleration", "Acceleration from the last applied external forces; excludes damping and contact impulses." },
            LayerDescription { VisualizationLayer::force_vectors, "force", "Forces", "Applied forces at each centre of mass (weight, drag, springs, pulls) and, with an open head, the contact force from supports and collisions. Joint forces show on their connections; a key beside the scale bar gives the arrow scale." },
            LayerDescription { VisualizationLayer::momentum_vectors, "momentum", "Momentum", "Mass times velocity, with its value shown beside the arrow. Automatic scaling uses the same scale for every object." },
            LayerDescription { VisualizationLayer::angular_velocity, "angular_velocity", "Spin", "The rate and direction of rotation." },
            LayerDescription { VisualizationLayer::contact_points, "contact_points", "Contact points", "Where two objects touch." },
            LayerDescription { VisualizationLayer::contact_normals, "contact_normals", "Contact normals", "The direction along which a contact pushes." },
            LayerDescription { VisualizationLayer::trajectories, "trajectories", "Trajectories", "The recent path of each object." },
            LayerDescription { VisualizationLayer::bounding_boxes, "bounding_boxes", "Bounding boxes", "The bounding boxes used to find possible collisions." },
            LayerDescription { VisualizationLayer::broad_phase_pairs, "broad_phase_pairs", "Candidate pairs", "Objects with overlapping bounding boxes that need a detailed collision check." },
            LayerDescription { VisualizationLayer::constraints, "constraints", "Connections", "Springs and joints show anchors, rods, rails, stops, motors, reactions, and broken links." },
            LayerDescription { VisualizationLayer::labels, "labels", "Labels", "Object names and measurements." }
        };

        constexpr std::uint32_t default_bits = static_cast<std::uint32_t>(VisualizationLayer::grid) | static_cast<std::uint32_t>(VisualizationLayer::bodies) |
            static_cast<std::uint32_t>(VisualizationLayer::outlines) | static_cast<std::uint32_t>(VisualizationLayer::center_of_mass) |
            static_cast<std::uint32_t>(VisualizationLayer::velocity_vectors) | static_cast<std::uint32_t>(VisualizationLayer::contact_points) |
            static_cast<std::uint32_t>(VisualizationLayer::constraints);

    } // namespace

    math::Span<const LayerDescription> layer_catalogue()
    {
        return layer_table;
    }

    const LayerDescription* find_layer(std::string_view id)
    {
        for (const auto& description : layer_table)
        {
            if (description.id == id)
            {
                return &description;
            }
        }
        return nullptr;
    }

    LayerMask::LayerMask(std::uint32_t bits) : bits_(bits)
    {
    }

    LayerMask LayerMask::defaults()
    {
        return LayerMask { default_bits };
    }

    LayerMask LayerMask::all()
    {
        std::uint32_t bits = 0;
        for (const auto& description : layer_table)
        {
            bits |= static_cast<std::uint32_t>(description.layer);
        }
        return LayerMask { bits };
    }

    LayerMask LayerMask::none()
    {
        return LayerMask { 0 };
    }

    bool LayerMask::is_enabled(VisualizationLayer layer) const
    {
        return (bits_ & static_cast<std::uint32_t>(layer)) != 0;
    }

    void LayerMask::set(VisualizationLayer layer, bool enabled)
    {
        if (enabled)
        {
            bits_ |= static_cast<std::uint32_t>(layer);
        }
        else
        {
            bits_ &= ~static_cast<std::uint32_t>(layer);
        }
    }

    void LayerMask::toggle(VisualizationLayer layer)
    {
        bits_ ^= static_cast<std::uint32_t>(layer);
    }

    std::uint32_t LayerMask::bits() const
    {
        return bits_;
    }

} // namespace rigidbodies::render
