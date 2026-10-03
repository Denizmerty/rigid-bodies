#pragma once

#include <rigidbodies/physics/contact.hpp>

#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    // The pair of colliders the narrow phase is asked about, already placed in world space by the
    // caller so that the test itself is purely geometric.
    struct NarrowPhaseQuery
    {
        BodyId first;
        BodyId second;
        std::size_t first_collider { 0 };
        std::size_t second_collider { 0 };

        const Shape* first_shape { nullptr };
        const Shape* second_shape { nullptr };

        math::Transform2 first_transform;
        math::Transform2 second_transform;

        ContactMaterial material {};
        bool is_sensor { false };
        Real contact_margin_m { 0.0 };
    };

    // Turns a candidate pair into the manifold through which the two shapes exchange impulse.
    //
    // Implementations produce manifolds through a shared interface so collision algorithms remain
    // interchangeable.
    class NarrowPhase
    {
    public:
        NarrowPhase() = default;
        NarrowPhase(const NarrowPhase&) = default;
        NarrowPhase(NarrowPhase&&) = default;
        NarrowPhase& operator=(const NarrowPhase&) = default;
        NarrowPhase& operator=(NarrowPhase&&) = default;
        virtual ~NarrowPhase() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<NarrowPhase> clone() const
        {
            throw std::logic_error("NarrowPhase does not support snapshots");
        }

        [[nodiscard]] virtual std::string_view name() const = 0;

        // Returns false when the pair does not touch, in which case the manifold is left empty.
        virtual bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const = 0;
    };

    using NarrowPhasePtr = std::shared_ptr<NarrowPhase>;

    class CollisionNarrowPhase final : public NarrowPhase
    {
    public:
        [[nodiscard]] std::shared_ptr<NarrowPhase> clone() const override
        {
            return std::make_shared<CollisionNarrowPhase>(*this);
        }
        [[nodiscard]] std::string_view name() const override;
        bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const override;
    };

    // Null implementation that reports no contact for every pair.
    class NullNarrowPhase final : public NarrowPhase
    {
    public:
        [[nodiscard]] std::shared_ptr<NarrowPhase> clone() const override
        {
            return std::make_shared<NullNarrowPhase>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        bool collide(const NarrowPhaseQuery& query, ContactManifold& manifold) const override;
    };

} // namespace rigidbodies::physics
