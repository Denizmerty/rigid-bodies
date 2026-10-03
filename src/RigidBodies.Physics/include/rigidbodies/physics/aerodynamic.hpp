#pragma once

#include <rigidbodies/physics/rigid_body.hpp>

#include <optional>

namespace rigidbodies::physics
{
    struct ForceContext;

    // An explicit educational slab model, not a calibrated CFD or flight model. Independent
    // collider outlines add without aerodynamic shielding, including overlapping compound parts.
    // Hollow mass profiles retain their outer aerodynamic envelope, just as collision does.
    // The reference laws follow NASA Glenn's educational descriptions:
    // https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/
    // https://www.grc.nasa.gov/www/k-12/airplane/viscosity.html
    // https://www.grc.nasa.gov/www/k-12/airplane/cyl.html
    // Our coefficients and low-Re bridge are declared approximations, not NASA correlations.
    struct AerodynamicSettings
    {
        bool estimate_outline_coefficient { true };
        bool reynolds_correction { true };
        bool angular_drag { true };
        bool magnus_lift { true };
        Real angular_drag_coefficient { 0.02 };
        Real magnus_efficiency { 0.2 };
        Real maximum_lift_coefficient { 2.0 };
    };

    enum class DragCoefficientSource
    {
        outline,
        material,
        collider_override
    };

    struct AerodynamicProfile
    {
        Real projected_area_m2 {};
        // Reference length for Re: the larger span along/across the flow (diameter for circles).
        Real characteristic_length_m {};
        math::Vec2 center_of_pressure_m {};
        math::Vec2 geometric_center_m {};
        Real base_drag_coefficient {};
        DragCoefficientSource coefficient_source { DragCoefficientSource::material };
        // Integral depth * |position - body COM|^3 ds over the outer boundary, in m^5.
        // Edges use eight-point Gauss quadrature, offset circles 64 angular samples.
        Real rotational_area_m5 {};
        Real equivalent_radius_m {};
    };

    struct AerodynamicReport
    {
        math::Vec2 drag_force_n {};
        math::Vec2 magnus_force_n {};
        Real drag_torque_n_m {};
        Real angular_drag_torque_n_m {};
        Real magnus_torque_n_m {};
        Real projected_area_m2 {};
        // Area-weighted part diagnostics at each geometric centre's relative velocity.
        Real effective_drag_coefficient {};
        Real reynolds_number {};
        Real relative_speed_m_s {};
    };

    struct TerminalSpeedEstimate
    {
        Real speed_m_s {};
        math::Vec2 relative_velocity_m_s {};
        math::Vec2 world_velocity_m_s {};
        Real weight_n {};
        Real drag_force_n {};
        Real projected_area_m2 {};
    };

    // Invalid/non-finite parameters throw invalid_argument; unrepresentable derived values throw
    // overflow_error. No force is silently capped by this model.
    void validate_aerodynamic_settings(const AerodynamicSettings& settings);

    // Exact directional projection for circles, convex polygons and two-sided segments. The
    // nonzero finite direction points along body motion relative to air. Zero returns no profile.
    // Cd precedence: explicit collider override, outline estimate when requested, material fallback.
    // Outline Cd0: circular cylinder 1.2, two-sided segment 1.8, convex polygon 0.8 + 0.6 B,
    // where B is windward projected-edge-weighted squared normal alignment with the flow.
    [[nodiscard]] AerodynamicProfile project_aerodynamic_profile(const Collider& collider,
        const math::Transform2& body_transform, const math::Vec2& direction,
        const math::Vec2& world_center_of_mass_m, const AerodynamicSettings& settings = {});

    // Re = rho |v| L / mu. The optional low-speed bridge is Cd = Cd0 (1 + 24/Re),
    // giving linear viscous plus quadratic inertial drag without pretending to model drag crises,
    // compressibility, wakes, finite-span end effects, or a measured arbitrary-shape correlation.
    // At zero speed Cd is reported as Cd0 and the force is exactly zero.
    [[nodiscard]] Real aerodynamic_drag_coefficient(Real base_coefficient, Real reynolds_number,
        const AerodynamicSettings& settings = {});

    // Surface samples use their own rigid-body velocity relative to the air. Every drag sample
    // has non-positive relative power; Magnus force is perpendicular to its application-point
    // velocity. Angular skin drag is -0.5 rho Cd0 Cw rotational_area * omega |omega|.
    // Magnus circulation is efficiency * 2 pi equivalent_radius^2 omega, capped by the stated
    // lift coefficient and applied at each part's geometric centre. Sensors do not load the body.
    // Authored convex cells share one cached fine outline, evaluated once per logical part. Its
    // re-entrant windward edges are normalized to frontal span; collision seams never add loads.
    [[nodiscard]] AerodynamicReport evaluate_aerodynamics(const RigidBody& body,
        const ForceContext& context, const AerodynamicSettings& settings = {});

    // Fixed current orientation, no spin/lift, constant wind, no contacts or other applied forces.
    // Solves the model's summed quadratic-plus-linear drag against mass * effective gravity.
    // Returns nullopt for a non-dynamic/massless body or no finite positive-weight drag balance.
    // Zero effective gravity has a zero relative terminal-speed estimate (drifting with the wind).
    [[nodiscard]] std::optional<TerminalSpeedEstimate> estimate_terminal_speed(const RigidBody& body,
        const ForceContext& context, const AerodynamicSettings& settings = {});
}
