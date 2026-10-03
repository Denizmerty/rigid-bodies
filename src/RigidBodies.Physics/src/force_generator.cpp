#include <rigidbodies/physics/force_generator.hpp>

#include <algorithm>

namespace rigidbodies::physics
{

    std::string_view UniformGravity::name() const
    {
        return "uniform_gravity";
    }

    void UniformGravity::apply(RigidBody& body, const ForceContext& context)
    {
        if (body.type() != BodyType::dynamic_body)
        {
            return;
        }

        // Weight acts at the centre of mass, so it produces no torque. This is the reason a
        // freely falling object keeps whatever spin it started with.
        body.apply_force_at_center(context.gravity_m_s2 * body.mass_properties().mass_kg * body.gravity_scale());
    }

    std::string_view AerodynamicDrag::name() const
    {
        return "aerodynamic_drag";
    }

    AerodynamicDrag::AerodynamicDrag(const AerodynamicSettings& settings)
    {
        set_settings(settings);
    }

    const AerodynamicSettings& AerodynamicDrag::settings() const
    {
        return settings_;
    }

    void AerodynamicDrag::set_settings(const AerodynamicSettings& settings)
    {
        validate_aerodynamic_settings(settings);
        settings_ = settings;
    }

    void AerodynamicDrag::apply(RigidBody& body, const ForceContext& context)
    {
        if (body.type() != BodyType::dynamic_body)
        {
            return;
        }

        const auto report = evaluate_aerodynamics(body, context, settings_);
        if (!(report.drag_force_n == math::Vec2 {}) || report.drag_torque_n_m != 0.0)
        {
            body.apply_force_at_center(report.drag_force_n, "aerodynamic_drag");
            body.apply_torque(report.drag_torque_n_m, "aerodynamic_drag");
        }
        if (report.angular_drag_torque_n_m != 0.0)
            body.apply_torque(report.angular_drag_torque_n_m, "angular_drag");
        if (!(report.magnus_force_n == math::Vec2 {}) || report.magnus_torque_n_m != 0.0)
        {
            body.apply_force_at_center(report.magnus_force_n, "magnus");
            body.apply_torque(report.magnus_torque_n_m, "magnus");
        }
    }

    PointAttractor::PointAttractor(const math::Vec2& world_position_m, Real stiffness_n_m) : world_position_m_(world_position_m), stiffness_n_m_(stiffness_n_m)
    {
    }

    std::string_view PointAttractor::name() const
    {
        return "point_attractor";
    }

    void PointAttractor::apply(RigidBody& body, const ForceContext&)
    {
        if (body.type() != BodyType::dynamic_body)
        {
            return;
        }

        const auto offset = world_position_m_ - body.world_center_of_mass_m();
        body.apply_force_at_center(offset * stiffness_n_m_);
    }

    const math::Vec2& PointAttractor::world_position_m() const
    {
        return world_position_m_;
    }

    void PointAttractor::set_world_position(const math::Vec2& value)
    {
        world_position_m_ = value;
    }

    Real PointAttractor::stiffness_n_m() const
    {
        return stiffness_n_m_;
    }

    void PointAttractor::set_stiffness(Real value)
    {
        stiffness_n_m_ = std::max(value, 0.0);
    }

} // namespace rigidbodies::physics
