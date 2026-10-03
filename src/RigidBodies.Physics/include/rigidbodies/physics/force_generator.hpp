#pragma once

#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/aerodynamic.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rigidbodies::physics
{

    class World;

    // Environment values a force generator may need. Passing them in rather than reaching into the
    // world keeps generators testable in isolation and keeps the set of things they can depend on
    // visible at a glance.
    struct ForceContext
    {
        math::Vec2 gravity_m_s2 { 0.0, -standard_gravity_m_s2 };
        Real air_density_kg_m3 { default_air_density_kg_m3 };
        Real time_step_s {};
        Real elapsed_time_s {};
        math::Vec2 air_velocity_m_s {};
        Real air_dynamic_viscosity_pa_s { 1.81e-5 };
    };

    // A one-body source of force or torque applied before integration. Generators stay separate
    // from pair interactions such as springs and from contact constraints, while sharing the same
    // accumulated-load integration path and independently switchable interface.
    class ForceGenerator
    {
    public:
        ForceGenerator() = default;
        ForceGenerator(const ForceGenerator&) = default;
        ForceGenerator(ForceGenerator&&) = default;
        ForceGenerator& operator=(const ForceGenerator&) = default;
        ForceGenerator& operator=(ForceGenerator&&) = default;
        virtual ~ForceGenerator() = default;

        // Return an independent copy of all mutable state, including caches and external state.
        // Custom implementations that cannot do so must reject snapshots explicitly.
        [[nodiscard]] virtual std::shared_ptr<ForceGenerator> clone() const
        {
            throw std::logic_error("ForceGenerator does not support snapshots");
        }

        // Name shown in the interface and used when a scenario refers to the generator.
        [[nodiscard]] virtual std::string_view name() const = 0;

        // Called once per body per step, before integration. Implementations accumulate onto the
        // body and change nothing else.
        virtual void apply(RigidBody& body, const ForceContext& context) = 0;

        [[nodiscard]] bool is_enabled() const
        {
            return enabled_;
        }

        void set_enabled(bool value)
        {
            enabled_ = value;
        }

    private:
        bool enabled_ { true };
    };

    using ForceGeneratorPtr = std::shared_ptr<ForceGenerator>;

    // Uniform gravitational field. Household-scale scenarios treat gravity as a constant
    // acceleration rather than as an inverse-square attraction between bodies.
    class UniformGravity final : public ForceGenerator
    {
    public:
        [[nodiscard]] std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<UniformGravity>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void apply(RigidBody& body, const ForceContext& context) override;
    };

    // Profile-dependent, distributed drag with an optional Reynolds bridge, angular resistance,
    // and Magnus lift. See aerodynamic.hpp for equations, units and explicit model limitations.
    class AerodynamicDrag final : public ForceGenerator
    {
    public:
        explicit AerodynamicDrag(const AerodynamicSettings& settings = {});
        [[nodiscard]] std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<AerodynamicDrag>(*this);
        }

        [[nodiscard]] std::string_view name() const override;
        void apply(RigidBody& body, const ForceContext& context) override;
        [[nodiscard]] const AerodynamicSettings& settings() const;
        void set_settings(const AerodynamicSettings& settings);

    private:
        AerodynamicSettings settings_;
    };

    // Attracts every affected body towards a fixed world point with a linear spring law.
    class PointAttractor final : public ForceGenerator
    {
    public:
        [[nodiscard]] std::shared_ptr<ForceGenerator> clone() const override
        {
            return std::make_shared<PointAttractor>(*this);
        }

        PointAttractor(const math::Vec2& world_position_m, Real stiffness_n_m);

        [[nodiscard]] std::string_view name() const override;
        void apply(RigidBody& body, const ForceContext& context) override;

        [[nodiscard]] const math::Vec2& world_position_m() const;
        void set_world_position(const math::Vec2& value);

        [[nodiscard]] Real stiffness_n_m() const;
        void set_stiffness(Real value);

    private:
        math::Vec2 world_position_m_ {};
        Real stiffness_n_m_ {};
    };

} // namespace rigidbodies::physics
