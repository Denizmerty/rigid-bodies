#include <rigidbodies/physics/aerodynamic.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/force_generator.hpp>
#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <limits>
#include <random>

namespace
{
    using namespace rigidbodies;
    using namespace rigidbodies::physics;

    Collider part(ShapePtr shape, Real depth = 0.2)
    {
        Collider result;
        result.shape = std::move(shape);
        result.depth_m = depth;
        result.material.density_kg_m3 = 10.0;
        return result;
    }

    RigidBody body_for(std::vector<Collider> colliders, math::Vec2 velocity = {}, Real spin = 0.0)
    {
        BodyDefinition definition;
        definition.colliders = std::move(colliders);
        definition.linear_velocity_m_s = velocity;
        definition.angular_velocity_rad_s = spin;
        return RigidBody { definition };
    }

    AerodynamicSettings quadratic_only()
    {
        AerodynamicSettings result;
        result.reynolds_correction = false;
        result.angular_drag = false;
        result.magnus_lift = false;
        return result;
    }

    template <class Operation>
    void expect_invalid(const Operation& operation)
    {
        bool rejected = false;
        try
        {
            operation();
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        RIGIDBODIES_EXPECT(rejected, "invalid aerodynamic input is rejected explicitly");
    }

    RIGIDBODIES_TEST("directional profiles use exact transformed circle polygon and segment projections")
    {
        const auto box = part(make_box(2.0, 0.4), 0.3);
        const auto horizontal = project_aerodynamic_profile(box, {}, { 1.0, 0.0 }, {});
        const auto vertical = project_aerodynamic_profile(box, {}, { 0.0, 1.0 }, {});
        RIGIDBODIES_EXPECT_NEAR(horizontal.projected_area_m2, 0.12, 1.0e-14, "frontal width across horizontal flow times slab depth");
        RIGIDBODIES_EXPECT_NEAR(vertical.projected_area_m2, 0.6, 1.0e-14, "broad face presents five times the area");
        RIGIDBODIES_EXPECT_NEAR(horizontal.center_of_pressure_m.x, 1.0, 1.0e-14, "pressure centroid lies on the windward edge");
        auto rotated = box;
        rotated.local_transform = math::Transform2::from_angle({ 1.0, 2.0 }, math::pi * 0.25);
        const auto profile = project_aerodynamic_profile(rotated, math::Transform2::from_angle({ 4.0, -2.0 }, math::pi * 0.25), { 1.0, 0.0 }, {});
        RIGIDBODIES_EXPECT_NEAR(profile.projected_area_m2, 0.6, 1.0e-14, "body and collider rotations compose before projection");
        const auto disk = part(make_circle(0.5, { 8.0, -3.0 }), 0.3);
        RIGIDBODIES_EXPECT_NEAR(project_aerodynamic_profile(disk, {}, { 0.3, -0.4 }, {}).projected_area_m2, 0.3, 1.0e-14, "circle area uses diameter rather than distance from the local origin");
        const auto segment = part(make_segment({ -1.0, 0.0 }, { 1.0, 0.0 }), 0.3);
        RIGIDBODIES_EXPECT_NEAR(project_aerodynamic_profile(segment, {}, { 1.0, 0.0 }, {}).projected_area_m2, 0.0, 0.0, "edge-on segment has no projected area");
        RIGIDBODIES_EXPECT_NEAR(project_aerodynamic_profile(segment, {}, { 0.0, -1.0 }, {}).projected_area_m2, 0.6, 1.0e-14, "segment is exposed from either side");
    }

    RIGIDBODIES_TEST("outline drag estimates have explicit override and material fallback precedence")
    {
        auto collider = part(make_box(2.0, 0.4));
        collider.material.drag_coefficient = 0.37;
        auto profile = project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, {});
        RIGIDBODIES_EXPECT(profile.coefficient_source == DragCoefficientSource::outline, "known outline selects the geometric estimate by default");
        RIGIDBODIES_EXPECT_NEAR(profile.base_drag_coefficient, 1.4, 1.0e-14, "flat windward face uses the documented bluff estimate");
        const auto diagonal = project_aerodynamic_profile(collider, {}, { 1.0, 1.0 }, {});
        RIGIDBODIES_EXPECT(diagonal.base_drag_coefficient < profile.base_drag_coefficient, "inclined outline changes estimated bluffness");
        auto settings = quadratic_only();
        settings.estimate_outline_coefficient = false;
        profile = project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, {}, settings);
        RIGIDBODIES_EXPECT(profile.coefficient_source == DragCoefficientSource::material, "explicitly disabling estimation falls back to material");
        RIGIDBODIES_EXPECT_NEAR(profile.base_drag_coefficient, 0.37, 0.0, "fallback value is preserved");
        collider.drag_coefficient_override = 0.72;
        profile = project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, {});
        RIGIDBODIES_EXPECT(profile.coefficient_source == DragCoefficientSource::collider_override, "collider override takes priority over outline estimation");
        RIGIDBODIES_EXPECT_NEAR(profile.base_drag_coefficient, 0.72, 0.0, "author-supplied base coefficient is retained");
    }

    RIGIDBODIES_TEST("distributed translational drag matches the analytical quadratic area law")
    {
        ForceContext context;
        context.air_density_kg_m3 = 1.2;
        auto settings = quadratic_only();
        for (const auto& shape : { make_circle(0.4), make_box(0.8, 0.6) })
        {
            auto collider = part(shape, 0.25);
            collider.drag_coefficient_override = 0.9;
            auto body = body_for({ collider }, { 3.0, 0.0 });
            const auto profile = project_aerodynamic_profile(collider, body.transform(), { 1.0, 0.0 }, body.world_center_of_mass_m(), settings);
            const auto expected = 0.5 * context.air_density_kg_m3 * 0.9 * profile.projected_area_m2 * 9.0;
            const auto first = evaluate_aerodynamics(body, context, settings);
            RIGIDBODIES_EXPECT_NEAR(first.drag_force_n.x, -expected, 1.0e-12, "surface integration recovers drag equation");
            RIGIDBODIES_EXPECT_NEAR(first.drag_force_n.y, 0.0, 1.0e-12, "symmetric profile has no spurious lateral drag");
            RIGIDBODIES_EXPECT_NEAR(first.drag_torque_n_m, 0.0, 1.0e-12, "symmetric pressure distribution has zero torque");
            body.set_linear_velocity({ 6.0, 0.0 });
            RIGIDBODIES_EXPECT_NEAR(evaluate_aerodynamics(body, context, settings).drag_force_n.x, first.drag_force_n.x * 4.0, 1.0e-11, "doubling speed quadruples inertial drag");
            context.air_density_kg_m3 *= 2.0;
            RIGIDBODIES_EXPECT_NEAR(evaluate_aerodynamics(body, context, settings).drag_force_n.x, first.drag_force_n.x * 8.0, 1.0e-11, "doubling density doubles drag");
            context.air_density_kg_m3 *= 0.5;
        }
    }

    RIGIDBODIES_TEST("Reynolds bridge gives finite low-speed linear drag and approaches the inertial coefficient")
    {
        ForceContext context;
        context.air_density_kg_m3 = 1.0;
        context.air_dynamic_viscosity_pa_s = 0.1;
        auto collider = part(make_circle(0.5));
        collider.drag_coefficient_override = 1.0;
        auto body = body_for({ collider }, { 0.1, 0.0 });
        AerodynamicSettings settings;
        settings.magnus_lift = false;
        const auto report = evaluate_aerodynamics(body, context, settings);
        RIGIDBODIES_EXPECT_NEAR(report.reynolds_number, 1.0, 1.0e-14, "reported Reynolds number uses density speed diameter and dynamic viscosity");
        RIGIDBODIES_EXPECT_NEAR(report.effective_drag_coefficient, 25.0, 1.0e-12, "low-Re bridge follows its stated formula");
        RIGIDBODIES_EXPECT_NEAR(report.drag_force_n.x, -0.025, 1.0e-13, "coefficient and area recover expected force");
        RIGIDBODIES_EXPECT_NEAR(aerodynamic_drag_coefficient(1.0, 1.0e12), 1.0, 3.0e-11, "large Reynolds approaches the base coefficient");
        body.set_linear_velocity({ 1.0e-10, 0.0 });
        const auto slow = evaluate_aerodynamics(body, context, settings).drag_force_n.x;
        body.set_linear_velocity({ 2.0e-10, 0.0 });
        RIGIDBODIES_EXPECT_NEAR(evaluate_aerodynamics(body, context, settings).drag_force_n.x, 2.0 * slow, 1.0e-19, "very low speed force is linear rather than singular");
        body.set_linear_velocity({});
        RIGIDBODIES_EXPECT(evaluate_aerodynamics(body, context, settings).drag_force_n == math::Vec2 {}, "rest has exactly zero drag");
    }

    RIGIDBODIES_TEST("angular skin drag is quadratic in spin and uses distance from the actual centre of mass")
    {
        auto collider = part(make_circle(0.4), 0.25);
        collider.drag_coefficient_override = 1.0;
        auto body = body_for({ collider }, {}, 3.0);
        ForceContext context;
        context.air_density_kg_m3 = 1.2;
        auto settings = quadratic_only();
        settings.angular_drag = true;
        settings.angular_drag_coefficient = 0.1;
        const auto integral = math::two_pi * std::pow(0.4, 4) * 0.25;
        const auto profile = project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, {});
        RIGIDBODIES_EXPECT_NEAR(profile.rotational_area_m5, integral, 1.0e-14, "centered disk boundary moment has a closed form");
        const auto first = evaluate_aerodynamics(body, context, settings);
        RIGIDBODIES_EXPECT(first.drag_force_n == math::Vec2 {}, "pure spin does not create translation for a centered circle");
        RIGIDBODIES_EXPECT_NEAR(first.angular_drag_torque_n_m, -0.5 * 1.2 * 0.1 * integral * 9.0, 1.0e-13, "angular skin resistance follows its dimensional formula");
        body.set_angular_velocity(-6.0);
        RIGIDBODIES_EXPECT_NEAR(evaluate_aerodynamics(body, context, settings).angular_drag_torque_n_m, -4.0 * first.angular_drag_torque_n_m, 1.0e-12, "spin reversal reverses resistance and twice the speed gives four times the torque");
        const auto displaced_com = project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, { 1.0, 0.0 });
        RIGIDBODIES_EXPECT(displaced_com.rotational_area_m5 > profile.rotational_area_m5 * 10.0, "moving mass centre away increases aerodynamic rotational resistance");
    }

    RIGIDBODIES_TEST("distributed drag dissipates relative kinetic energy for asymmetric moving spinning compounds")
    {
        auto left = part(make_box(0.4, 0.7));
        left.local_transform = math::Transform2::from_angle({ -0.4, 0.2 }, 0.3);
        auto right = part(make_circle(0.25));
        right.local_transform.translation = { 0.7, -0.3 };
        right.density_override_kg_m3 = 60.0;
        auto body = body_for({ left, right });
        ForceContext context;
        context.air_velocity_m_s = { 1.0, -0.4 };
        std::mt19937 random { 8751 };
        std::uniform_real_distribution<Real> distribution { -8.0, 8.0 };
        for (int sample = 0; sample < 120; ++sample)
        {
            body.set_linear_velocity({ distribution(random), distribution(random) });
            body.set_angular_velocity(distribution(random));
            body.set_orientation(distribution(random));
            const auto report = evaluate_aerodynamics(body, context);
            const auto relative = body.linear_velocity_m_s() - context.air_velocity_m_s;
            const auto drag_power = math::dot(report.drag_force_n, relative) + (report.drag_torque_n_m + report.angular_drag_torque_n_m) * body.angular_velocity_rad_s();
            const auto magnus_power = math::dot(report.magnus_force_n, relative) + report.magnus_torque_n_m * body.angular_velocity_rad_s();
            RIGIDBODIES_EXPECT(drag_power <= 1.0e-10, "each distributed aerodynamic drag contribution removes relative mechanical energy");
            RIGIDBODIES_EXPECT_NEAR(magnus_power, 0.0, 1.0e-10, "distributed Magnus forces do no work at their moving application points");
        }
    }

    RIGIDBODIES_TEST("Magnus circulation reverses with spin remains perpendicular and honors its lift cap")
    {
        auto body = body_for({ part(make_circle(0.2), 0.3) }, { 4.0, 0.0 }, 2.0);
        ForceContext context;
        context.air_density_kg_m3 = 1.0;
        auto settings = quadratic_only();
        settings.magnus_lift = true;
        settings.magnus_efficiency = 0.2;
        const auto expected = 1.0 * math::two_pi * 0.04 * 2.0 * 0.2 * 0.3 * 4.0;
        const auto positive = evaluate_aerodynamics(body, context, settings);
        RIGIDBODIES_EXPECT_NEAR(positive.magnus_force_n.y, expected, 1.0e-13, "uncapped lift equals density times circulation span and speed");
        RIGIDBODIES_EXPECT_NEAR(positive.magnus_force_n.x, 0.0, 0.0, "Magnus is perpendicular to relative translation");
        body.set_angular_velocity(-2.0);
        RIGIDBODIES_EXPECT_NEAR(evaluate_aerodynamics(body, context, settings).magnus_force_n.y, -expected, 1.0e-13, "spin reversal reverses the trajectory curvature");
        body.set_angular_velocity(1000.0);
        const auto capped = evaluate_aerodynamics(body, context, settings);
        RIGIDBODIES_EXPECT_NEAR(capped.magnus_force_n.y, 0.5 * 1.0 * 16.0 * 0.12 * settings.maximum_lift_coefficient, 1.0e-12, "large spin is bounded by the explicit lift coefficient");
        settings.magnus_lift = false;
        RIGIDBODIES_EXPECT(evaluate_aerodynamics(body, context, settings).magnus_force_n == math::Vec2 {}, "lift can be disabled independently");
    }

    RIGIDBODIES_TEST("asymmetric pressure generates a torque matching its profile centre and changes with mirrored geometry")
    {
        const std::vector<math::Vec2> triangle { { -1.0, -0.5 }, { 1.0, -0.5 }, { -1.0, 0.5 } };
        auto collider = part(std::make_shared<ConvexPolygonShape>(triangle));
        collider.drag_coefficient_override = 1.0;
        auto body = body_for({ collider }, { 3.0, 0.0 });
        const auto settings = quadratic_only();
        const auto profile = project_aerodynamic_profile(collider, body.transform(), { 1.0, 0.0 }, body.world_center_of_mass_m(), settings);
        const auto report = evaluate_aerodynamics(body, {}, settings);
        RIGIDBODIES_EXPECT(std::abs(report.drag_torque_n_m) > 0.01, "asymmetric windward outline produces a visible aerodynamic torque");
        RIGIDBODIES_EXPECT_NEAR(report.drag_torque_n_m, math::cross(profile.center_of_pressure_m - body.world_center_of_mass_m(), report.drag_force_n), 1.0e-12, "integrated torque agrees with the pressure centroid for uniform translation");
        body.set_orientation(math::pi);
        body.set_linear_velocity({ -3.0, 0.0 });
        const auto rotated = evaluate_aerodynamics(body, {}, settings);
        RIGIDBODIES_EXPECT_NEAR(rotated.drag_torque_n_m, report.drag_torque_n_m, 1.0e-12, "rotating the whole experiment preserves scalar torque");
    }

    RIGIDBODIES_TEST("terminal-speed estimate solves the same Reynolds-dependent drag law in signed gravity and wind")
    {
        auto collider = part(make_box(0.4, 0.1), 0.2);
        collider.drag_coefficient_override = 0.8;
        auto body = body_for({ collider });
        body.override_mass(0.025);
        ForceContext context;
        context.air_velocity_m_s = { 2.0, 1.0 };
        context.air_dynamic_viscosity_pa_s = 0.002;
        for (const auto scale : { 1.0, -0.4 })
        {
            body.set_gravity_scale(scale);
            const auto estimate = estimate_terminal_speed(body, context);
            RIGIDBODIES_EXPECT(estimate && estimate->speed_m_s > 0.0, "positive drag creates a finite terminal speed");
            body.set_linear_velocity(estimate->world_velocity_m_s);
            const auto report = evaluate_aerodynamics(body, context);
            const auto weight = context.gravity_m_s2 * (body.mass_properties().mass_kg * scale);
            RIGIDBODIES_EXPECT_NEAR(report.drag_force_n.x + weight.x, 0.0, 1.0e-12, "air-relative terminal motion has no horizontal residual");
            RIGIDBODIES_EXPECT_NEAR(report.drag_force_n.y + weight.y, 0.0, 1.0e-12, "estimated terminal speed balances actual evaluated drag against effective gravity");
            RIGIDBODIES_EXPECT_NEAR(estimate->drag_force_n, estimate->weight_n, 1.0e-12, "report includes the solved force balance");
        }
        body.set_gravity_scale(1.0);
        auto settings = quadratic_only();
        const auto estimate = estimate_terminal_speed(body, context, settings);
        const auto expected = std::sqrt(2.0 * 0.025 * standard_gravity_m_s2 / (context.air_density_kg_m3 * 0.8 * 0.08));
        RIGIDBODIES_EXPECT_NEAR(estimate->speed_m_s, expected, 1.0e-12, "constant coefficient terminal speed has its closed form");
    }

    RIGIDBODIES_TEST("zero gravity vacuum absent profiles and sensors have explicit terminal and loading behavior")
    {
        auto collider = part(make_circle(0.2));
        auto body = body_for({ collider }, { 2.0, 1.0 }, 3.0);
        ForceContext context;
        context.air_density_kg_m3 = 0.0;
        RIGIDBODIES_EXPECT(!estimate_terminal_speed(body, context), "positive weight in a vacuum has no finite terminal speed");
        const auto vacuum = evaluate_aerodynamics(body, context);
        RIGIDBODIES_EXPECT(vacuum.drag_force_n == math::Vec2 {} && vacuum.magnus_force_n == math::Vec2 {} && vacuum.angular_drag_torque_n_m == 0.0, "vacuum has no aerodynamic loads");
        context.gravity_m_s2 = {};
        context.air_velocity_m_s = { 2.0, -1.0 };
        const auto no_gravity = estimate_terminal_speed(body, context);
        RIGIDBODIES_EXPECT(no_gravity && no_gravity->speed_m_s == 0.0 && no_gravity->world_velocity_m_s == context.air_velocity_m_s, "zero gravity explicitly reports zero relative drift speed");
        collider.is_sensor = true;
        body = body_for({ collider }, { 2.0, 1.0 }, 3.0);
        context = {};
        RIGIDBODIES_EXPECT(!estimate_terminal_speed(body, context), "sensor outlines do not supply a drag balance");
        const auto sensor = evaluate_aerodynamics(body, context);
        RIGIDBODIES_EXPECT(sensor.drag_force_n == math::Vec2 {} && sensor.angular_drag_torque_n_m == 0.0, "sensors do not load their parent");
        RIGIDBODIES_EXPECT(project_aerodynamic_profile(collider, {}, {}, {}).projected_area_m2 == 0.0, "zero projection direction returns no profile");
    }

    RIGIDBODIES_TEST("aerodynamic settings and nonfinite flow parameters fail explicitly")
    {
        const auto body = body_for({ part(make_circle(0.2)) }, { 1.0, 0.0 });
        for (const auto invalid : { -1.0, std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
        {
            auto settings = AerodynamicSettings {};
            settings.magnus_efficiency = invalid;
            expect_invalid([&]
                {
                    AerodynamicDrag generator { settings };
                });
            auto context = ForceContext {};
            context.air_dynamic_viscosity_pa_s = invalid;
            expect_invalid([&]
                {
                    (void)evaluate_aerodynamics(body, context);
                });
            context = {};
            context.air_density_kg_m3 = invalid;
            expect_invalid([&]
                {
                    (void)estimate_terminal_speed(body, context);
                });
        }
        auto context = ForceContext {};
        context.air_dynamic_viscosity_pa_s = 0.0;
        expect_invalid([&]
            {
                (void)evaluate_aerodynamics(body, context);
            });
        auto collider = part(make_circle(0.2));
        collider.drag_coefficient_override = -0.1;
        expect_invalid([&]
            {
                (void)project_aerodynamic_profile(collider, {}, { 1.0, 0.0 }, {});
            });
    }

    RIGIDBODIES_TEST("aerodynamic generator emits independent load channels and clones its settings in world snapshots")
    {
        WorldSettings settings;
        settings.gravity_m_s2 = {};
        settings.air_velocity_m_s = { 0.2, -0.1 };
        World world { settings };
        auto drag = std::make_shared<AerodynamicDrag>();
        world.add_force_generator(drag);
        BodyDefinition definition;
        definition.colliders.push_back(part(make_circle(0.2)));
        definition.linear_velocity_m_s = { 2.0, 0.0 };
        definition.angular_velocity_rad_s = 3.0;
        const auto id = world.create_body(definition);
        const auto snapshot = world.snapshot();
        world.step(0.001);
        const auto* body = world.find_body(id);
        bool translation = false, angular = false, magnus = false;
        for (const auto& channel : body->applied_force_channels())
        {
            translation = translation || channel.name == "aerodynamic_drag";
            angular = angular || channel.name == "angular_drag";
            magnus = magnus || channel.name == "magnus";
        }
        RIGIDBODIES_EXPECT(translation && angular && magnus, "independent pressure rotational and lift contributions are retained");
        const auto velocity = body->linear_velocity_m_s();
        auto changed = drag->settings();
        changed.magnus_lift = false;
        changed.angular_drag_coefficient = 0.9;
        drag->set_settings(changed);
        world.restore(snapshot);
        const AerodynamicDrag* restored = nullptr;
        for (const auto& generator : world.force_generators())
            if (const auto* candidate = dynamic_cast<const AerodynamicDrag*>(generator.get()))
                restored = candidate;
        RIGIDBODIES_EXPECT(restored && restored != drag.get() && restored->settings().magnus_lift && restored->settings().angular_drag_coefficient == 0.02, "snapshot restores an independent aerodynamic configuration");
        world.step(0.001);
        RIGIDBODIES_EXPECT(world.find_body(id)->linear_velocity_m_s() == velocity, "snapshot replays the same aerodynamic step exactly");
    }
    std::shared_ptr<const AuthoredShape> authored_profile(const std::vector<math::Vec2>& points)
    {
        Outline outline;
        outline.closed = true;
        for (const auto point : points)
        {
            OutlineNode node;
            node.position_m = point;
            outline.nodes.push_back(node);
        }
        const auto result = build_authored_shape(outline);
        if (!result.succeeded())
            throw std::runtime_error("Aerodynamic test outline failed validation");
        return result.shape;
    }

    std::vector<Collider> authored_cells(const AuthoredPartPtr& logical, bool triangles)
    {
        std::vector<Collider> cells;
        const auto add = [&](const std::vector<math::Vec2>& vertices)
        {
            auto collider = part(std::make_shared<ConvexPolygonShape>(vertices), logical->depth_m);
            collider.local_transform = logical->local_transform;
            collider.material = logical->material;
            collider.authored_part = logical;
            cells.push_back(std::move(collider));
        };
        if (triangles)
            for (const auto& triangle : logical->shape->render_triangles)
                add({ triangle.begin(), triangle.end() });
        else
            for (const auto& polygon : logical->shape->convex_parts)
                add(polygon);
        return cells;
    }

    RIGIDBODIES_TEST("authored profile aerodynamics depend on the logical boundary rather than convex collision seams")
    {
        AuthoredPartDefinition source;
        source.shape = authored_profile({ { 0, 0 }, { 2, 0 }, { 2, 1 }, { 1, 1 }, { 1, 2 }, { 0, 2 } });
        source.depth_m = 0.2;
        source.local_transform = math::Transform2::from_angle({ 0.4, -0.2 }, 0.3);
        auto logical = std::make_shared<const AuthoredPartDefinition>(source);
        const auto merged_cells = authored_cells(logical, false), triangle_cells = authored_cells(logical, true);
        RIGIDBODIES_EXPECT(merged_cells.size() != triangle_cells.size(), "fixture has two distinct exact decompositions of the same source");
        auto merged = body_for(merged_cells, { 2.0, -1.0 }, 3.0);
        auto triangulated = body_for(triangle_cells, { 2.0, -1.0 }, 3.0);
        ForceContext context;
        context.air_velocity_m_s = { -0.3, 0.2 };
        const auto first = evaluate_aerodynamics(merged, context), second = evaluate_aerodynamics(triangulated, context);
        RIGIDBODIES_EXPECT_NEAR(first.drag_force_n.x, second.drag_force_n.x, 1.0e-10, "pressure drag ignores arbitrary internal convex edges");
        RIGIDBODIES_EXPECT_NEAR(first.drag_force_n.y, second.drag_force_n.y, 1.0e-10, "pressure force is independent of triangulation");
        RIGIDBODIES_EXPECT_NEAR(first.drag_torque_n_m, second.drag_torque_n_m, 1.0e-10, "pressure torque uses the common authored boundary");
        RIGIDBODIES_EXPECT_NEAR(first.angular_drag_torque_n_m, second.angular_drag_torque_n_m, 1.0e-10, "angular drag integrates the exterior perimeter once");
        RIGIDBODIES_EXPECT_NEAR(first.magnus_force_n.x, second.magnus_force_n.x, 1.0e-10, "Magnus lift uses one geometric centre and area per logical part");
        RIGIDBODIES_EXPECT_NEAR(first.magnus_torque_n_m, second.magnus_torque_n_m, 1.0e-10, "Magnus application point is independent of collision cells");
        RIGIDBODIES_EXPECT_NEAR(first.projected_area_m2, second.projected_area_m2, 1.0e-12, "frontal area is counted once per logical part");
        const auto terminal_first = estimate_terminal_speed(merged, context), terminal_second = estimate_terminal_speed(triangulated, context);
        RIGIDBODIES_EXPECT(terminal_first && terminal_second, "both decompositions have a terminal estimate");
        RIGIDBODIES_EXPECT_NEAR(terminal_first->speed_m_s, terminal_second->speed_m_s, 1.0e-10, "terminal speed shares logical-part aggregation");
    }

    RIGIDBODIES_TEST("reentrant authored boundaries preserve terminal balance and dissipative pressure work")
    {
        AuthoredPartDefinition source;
        source.shape = authored_profile({ { 0, 0 }, { 3, 0 }, { 3, 3 }, { 2, 3 }, { 2, 1 }, { 1, 1 }, { 1, 3 }, { 0, 3 } });
        source.depth_m = 0.2;
        auto logical = std::make_shared<const AuthoredPartDefinition>(source);
        auto body = body_for(authored_cells(logical, false));
        ForceContext context;
        context.gravity_m_s2 = { 3.0, 0.0 };
        context.air_velocity_m_s = { 0.2, -0.1 };
        const auto terminal = estimate_terminal_speed(body, context);
        RIGIDBODIES_EXPECT(terminal.has_value(), "reentrant authored geometry has a finite terminal balance");
        body.set_linear_velocity(terminal->world_velocity_m_s);
        const auto drag = evaluate_aerodynamics(body, context);
        RIGIDBODIES_EXPECT_NEAR(-drag.drag_force_n.x, terminal->weight_n, terminal->weight_n * 1.0e-10, "normalized windward pressure matches frontal-area terminal balance");
        body.set_angular_velocity(2.0);
        const auto rotating = evaluate_aerodynamics(body, context);
        const auto pressure_power = math::dot(rotating.drag_force_n, body.linear_velocity_m_s() - context.air_velocity_m_s) + rotating.drag_torque_n_m * body.angular_velocity_rad_s();
        RIGIDBODIES_EXPECT(pressure_power <= 0.0 && rotating.angular_drag_torque_n_m < 0.0, "the reentrant outline pressure and skin drag remain dissipative under rotation");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
