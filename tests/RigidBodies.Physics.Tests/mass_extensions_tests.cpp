#include <rigidbodies/physics/compound_mass.hpp>
#include <rigidbodies/physics/rigid_body.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{

    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;
    constexpr Real pi = rigidbodies::math::pi;

    Collider part(ShapePtr shape, Real density = 1.0, Real depth = 1.0, Vec2 position = {}, Real angle = 0.0)
    {
        Collider result;
        result.shape = std::move(shape);
        result.density_override_kg_m3 = density;
        result.depth_m = depth;
        result.local_transform = rigidbodies::math::Transform2::from_angle(position, angle);
        return result;
    }

    void expect_properties(const MassProperties& actual, Real mass, Vec2 center, Real inertia, Real relative_tolerance = 1.0e-11)
    {
        RIGIDBODIES_EXPECT(actual.is_valid(), "mass properties stay finite and non-negative");
        RIGIDBODIES_EXPECT_NEAR(actual.mass_kg, mass, relative_tolerance * std::max(1.0, mass), "mass matches the independent closed form");
        RIGIDBODIES_EXPECT_NEAR(actual.center_of_mass_m.x, center.x, relative_tolerance * std::max(1.0, std::abs(center.x)), "centre x matches the closed form");
        RIGIDBODIES_EXPECT_NEAR(actual.center_of_mass_m.y, center.y, relative_tolerance * std::max(1.0, std::abs(center.y)), "centre y matches the closed form");
        RIGIDBODIES_EXPECT_NEAR(actual.inertia_kg_m2, inertia, relative_tolerance * std::max(1.0, inertia), "centroidal inertia matches the independent closed form");
    }

    struct Lens
    {
        Real area;
        Real polar_about_origin;
    };

    Lens unit_circle_lens(Real half_separation)
    {
        // Integrate vertical slices of the symmetric lens between unit discs at +/-a.
        // The polynomial/square-root antiderivatives are independent of the boundary algorithm.
        const auto a = half_separation;
        const auto h = std::sqrt(1.0 - a * a);
        const auto angle = std::acos(a);
        const auto integral0 = (angle - a * h) * 0.5;
        const auto integral1 = h * h * h / 3.0;
        const auto integral2 = (angle - a * h * (2.0 * a * a - 1.0)) / 8.0;
        return { 2.0 * (angle - a * h),
            4.0 * ((2.0 / 3.0) * integral2 - 2.0 * a * integral1 + (a * a + 1.0 / 3.0) * integral0) };
    }

    class CustomMassShape final : public Shape
    {
    public:
        ShapeKind category { ShapeKind::circle };
        bool invalid_mass { false };
        ShapeKind kind() const override
        {
            return category;
        }
        std::shared_ptr<Shape> clone() const override
        {
            return std::make_shared<CustomMassShape>(*this);
        }
        rigidbodies::math::Aabb compute_bounds(const rigidbodies::math::Transform2& transform) const override
        {
            return circle_.compute_bounds(transform);
        }
        MassProperties compute_mass_properties(Real density) const override
        {
            return { invalid_mass ? std::numeric_limits<Real>::quiet_NaN() : density * 2.0, { 0.25, -0.5 }, density * 3.0 };
        }
        Vec2 support_point(const Vec2& direction) const override
        {
            return circle_.support_point(direction);
        }
        bool contains_local_point(const Vec2& point) const override
        {
            return circle_.contains_local_point(point);
        }
        Real bounding_radius() const override
        {
            return circle_.bounding_radius();
        }

    private:
        CircleShape circle_ { 1.0 };
    };

    RIGIDBODIES_TEST("custom shape mass dispatch uses the virtual contract rather than the reported shape category")
    {
        for (const auto category : { ShapeKind::circle, ShapeKind::convex_polygon, ShapeKind::segment })
        {
            const auto shape = std::make_shared<CustomMassShape>();
            shape->category = category;
            const auto collider = part(shape, 6.0, 0.5, { 4.0, -2.0 }, pi * 0.5);
            expect_properties(collider.compute_mass_properties(), 6.0, { 4.5, -1.75 }, 9.0);
            BodyDefinition definition;
            definition.colliders.push_back(collider);
            const RigidBody body(definition);
            expect_properties(body.mass_properties(), 6.0, { 4.5, -1.75 }, 9.0);
        }
    }

    RIGIDBODIES_TEST("custom mass rejects unsupported hollow and compound boundaries and invalid virtual results")
    {
        const auto shape = std::make_shared<CustomMassShape>();
        const auto rejects = [](const std::vector<Collider>& colliders)
        {
            bool rejected = false;
            try
            {
                (void)compute_compound_mass_properties(colliders);
            }
            catch (const std::invalid_argument&)
            {
                rejected = true;
            }
            RIGIDBODIES_EXPECT(rejected, "unsupported custom mass must fail explicitly instead of downcasting or approximating");
        };
        auto custom = part(shape);
        rejects({ custom, part(make_circle(1.0)) });
        rejects({ part(make_circle(1.0)), custom });
        custom.shell_thickness_m = 0.1;
        rejects({ custom });
        custom.shell_thickness_m = 0.0;
        expect_properties(custom.compute_mass_properties(), 0.0, { 0.25, -0.5 }, 0.0);
        custom.shell_thickness_m.reset();
        shape->invalid_mass = true;
        rejects({ custom });
    }

    RIGIDBODIES_TEST("per-part density overrides retain other material properties")
    {
        auto collider = part(make_box(2.0, 3.0), 7.0, 0.4, { 5.0, -2.0 });
        collider.material = materials::steel();
        const auto properties = collider.compute_mass_properties();
        const auto mass = 2.0 * 3.0 * 7.0 * 0.4;
        expect_properties(properties, mass, { 5.0, -2.0 }, mass * 13.0 / 12.0);
        RIGIDBODIES_EXPECT_NEAR(collider.effective_density_kg_m3(), 7.0, 0.0, "override changes bulk density");
        RIGIDBODIES_EXPECT_NEAR(collider.effective_drag_coefficient(), collider.material.drag_coefficient, 0.0, "density does not alter contact or drag material");
        collider.density_override_kg_m3.reset();
        RIGIDBODIES_EXPECT_NEAR(collider.effective_density_kg_m3(), collider.material.density_kg_m3, 0.0, "absent override uses material density");
    }

    RIGIDBODIES_TEST("circular tube has exact annular mass and inertia while retaining its collision envelope")
    {
        auto tube = part(make_circle(2.0, { 0.3, 0.2 }), 6.0, 0.7, { 4.0, -3.0 }, pi * 0.5);
        tube.shell_thickness_m = 0.4;
        const auto outer_squared = 4.0;
        const auto inner_squared = 1.6 * 1.6;
        const auto mass = pi * (outer_squared - inner_squared) * 6.0 * 0.7;
        expect_properties(tube.compute_mass_properties(), mass, { 3.8, -2.7 }, mass * (outer_squared + inner_squared) * 0.5);
        RIGIDBODIES_EXPECT(tube.shape->contains_local_point({ 0.3, 0.2 }), "mass cavity keeps the convex collision envelope explicit");
        const auto shell_bounds = tube.compute_bounds({});
        tube.shell_thickness_m.reset();
        const auto solid_bounds = tube.compute_bounds({});
        RIGIDBODIES_EXPECT(shell_bounds.minimum == solid_bounds.minimum && shell_bounds.maximum == solid_bounds.maximum, "hollow mass changes neither collision bounds nor outline");
    }

    RIGIDBODIES_TEST("rectangular hollow profile offsets edges inward and preserves rotated centroidal inertia")
    {
        auto hollow = part(make_box(4.0, 2.0), 3.0, 0.5, { -2.0, 5.0 }, 0.73);
        hollow.shell_thickness_m = 0.25;
        const auto density = 1.5;
        const auto mass = density * (8.0 - 3.5 * 1.5);
        const auto inertia = density * (8.0 * 20.0 - 3.5 * 1.5 * (3.5 * 3.5 + 1.5 * 1.5)) / 12.0;
        expect_properties(hollow.compute_mass_properties(), mass, { -2.0, 5.0 }, inertia);
        hollow.shell_thickness_m = 1.0;
        expect_properties(hollow.compute_mass_properties(), 12.0, { -2.0, 5.0 }, 20.0);
        hollow.shell_thickness_m = 3.0;
        expect_properties(hollow.compute_mass_properties(), 12.0, { -2.0, 5.0 }, 20.0);
    }

    RIGIDBODIES_TEST("very thin circular walls retain their mass without subtractive cancellation")
    {
        auto tube = part(make_circle(1.0));
        tube.shell_thickness_m = 1.0e-10;
        const auto expected_mass = pi * 1.0e-10 * (2.0 - 1.0e-10);
        const auto properties = tube.compute_mass_properties();
        RIGIDBODIES_EXPECT_NEAR(properties.mass_kg, expected_mass, expected_mass * 1.0e-12, "factored annular area preserves relative accuracy for thin walls");
        RIGIDBODIES_EXPECT_NEAR(properties.inertia_kg_m2, expected_mass * (1.0 + (1.0 - 1.0e-10) * (1.0 - 1.0e-10)) * 0.5, expected_mass * 1.0e-12, "thin-wall inertia approaches m R squared accurately");
    }

    RIGIDBODIES_TEST("equal overlapping circles use their analytical union area and second moment")
    {
        for (const auto separation : { 0.2, 0.5, 0.9 })
        {
            const auto lens = unit_circle_lens(separation);
            const auto expected_mass = 2.0 * pi - lens.area;
            const auto expected_inertia = 2.0 * pi * (0.5 + separation * separation) - lens.polar_about_origin;
            const auto properties = compute_compound_mass_properties({ part(make_circle(1.0), 1.0, 1.0, { -separation, 0.0 }),
                part(make_circle(1.0), 1.0, 1.0, { separation, 0.0 }) });
            expect_properties(properties, expected_mass, {}, expected_inertia);
        }
    }

    RIGIDBODIES_TEST("material precedence and centered slab depth count overlapping circle volume once")
    {
        const auto a = 0.5;
        const auto lens = unit_circle_lens(a);
        const auto first_density = 3.0;
        const auto first_depth = 0.4;
        const auto second_density = 7.0;
        const auto second_depth = 0.9;
        const auto first_areal = first_density * first_depth;
        const auto second_areal = second_density * second_depth;
        const auto removed_areal = second_density * first_depth;
        const auto mass = pi * (first_areal + second_areal) - removed_areal * lens.area;
        const auto first_moment = pi * a * (second_areal - first_areal);
        const auto center_x = first_moment / mass;
        const auto inertia = pi * (0.5 + a * a) * (first_areal + second_areal) - removed_areal * lens.polar_about_origin - mass * center_x * center_x;
        const auto first = part(make_circle(1.0), first_density, first_depth, { -a, 0.0 });
        const auto second = part(make_circle(1.0), second_density, second_depth, { a, 0.0 });
        expect_properties(compute_compound_mass_properties({ first, second }), mass, { center_x, 0.0 }, inertia);
        const auto reversed = compute_compound_mass_properties({ second, first });
        RIGIDBODIES_EXPECT(reversed.mass_kg > mass, "first material wins when densities disagree in shared volume");

        const auto shallow = part(make_circle(1.0), 3.0, 2.0);
        const auto deep = part(make_circle(1.0), 5.0, 4.0);
        expect_properties(compute_compound_mass_properties({ shallow, deep }), 16.0 * pi, {}, 8.0 * pi);
        expect_properties(compute_compound_mass_properties({ deep, shallow }), 20.0 * pi, {}, 10.0 * pi);
    }

    RIGIDBODIES_TEST("unequal intersecting circles match independent circular-cap closed forms")
    {
        // R=sqrt(2) at x=0 and r=1 at x=1 intersect at (1,+/-1). Their lens is one
        // large-circle cap and a small-circle semicircle, so all moments reduce to simple forms.
        const auto mass = 2.0 * pi + 1.0;
        const auto first_moment = pi * 0.5;
        const auto center_x = first_moment / mass;
        const auto polar = 9.0 * pi * 0.25 + 2.0;
        expect_properties(compute_compound_mass_properties({ part(make_circle(std::sqrt(2.0))),
                              part(make_circle(1.0), 1.0, 1.0, { 1.0, 0.0 }) }),
            mass,
            { center_x, 0.0 },
            polar - mass * center_x * center_x);
    }

    RIGIDBODIES_TEST("heterogeneous overlapping shells agree with an exact independent rectangular volume partition")
    {
        struct Rectangle
        {
            Real left;
            Real right;
            Real bottom;
            Real top;
            Real wall;
            Real density;
            Real depth;
        };
        const std::vector<Rectangle> rectangles { { -2.0, 1.0, -1.0, 2.0, 0.25, 2.0, 0.8 },
            { -1.0, 2.0, -2.0, 1.0, 0.4, 5.0, 0.3 },
            { -0.5, 1.5, -0.75, 1.75, -1.0, 7.0, 1.2 } };
        std::vector<Collider> colliders;
        std::vector<Real> xs;
        std::vector<Real> ys;
        std::vector<Real> zs;
        for (const auto& rectangle : rectangles)
        {
            auto collider = part(make_box(rectangle.right - rectangle.left, rectangle.top - rectangle.bottom), rectangle.density, rectangle.depth, { (rectangle.left + rectangle.right) * 0.5, (rectangle.bottom + rectangle.top) * 0.5 });
            xs.insert(xs.end(), { rectangle.left, rectangle.right });
            ys.insert(ys.end(), { rectangle.bottom, rectangle.top });
            zs.insert(zs.end(), { -rectangle.depth * 0.5, rectangle.depth * 0.5 });
            if (rectangle.wall >= 0.0)
            {
                collider.shell_thickness_m = rectangle.wall;
                xs.insert(xs.end(), { rectangle.left + rectangle.wall, rectangle.right - rectangle.wall });
                ys.insert(ys.end(), { rectangle.bottom + rectangle.wall, rectangle.top - rectangle.wall });
            }
            colliders.push_back(collider);
        }
        for (auto* coordinates : { &xs, &ys, &zs })
        {
            std::sort(coordinates->begin(), coordinates->end());
            coordinates->erase(std::unique(coordinates->begin(), coordinates->end()), coordinates->end());
        }
        Real mass = 0.0;
        Vec2 first_moment;
        Real polar = 0.0;
        // Partition along every authored x/y/z face. Each resulting rectangular cell is wholly
        // inside one material or empty, so this oracle uses exact cuboid moments, not a grid fit.
        for (std::size_t x = 1; x < xs.size(); ++x)
        {
            for (std::size_t y = 1; y < ys.size(); ++y)
            {
                for (std::size_t z = 1; z < zs.size(); ++z)
                {
                    const Vec2 center { (xs[x - 1] + xs[x]) * 0.5, (ys[y - 1] + ys[y]) * 0.5 };
                    const auto center_z = (zs[z - 1] + zs[z]) * 0.5;
                    for (const auto& rectangle : rectangles)
                    {
                        const auto inside = center.x > rectangle.left && center.x < rectangle.right && center.y > rectangle.bottom && center.y < rectangle.top &&
                            std::abs(center_z) < rectangle.depth * 0.5;
                        const auto in_hole = rectangle.wall >= 0.0 && center.x > rectangle.left + rectangle.wall && center.x < rectangle.right - rectangle.wall &&
                            center.y > rectangle.bottom + rectangle.wall && center.y < rectangle.top - rectangle.wall;
                        if (!inside || in_hole)
                        {
                            continue;
                        }
                        const auto width = xs[x] - xs[x - 1];
                        const auto height = ys[y] - ys[y - 1];
                        const auto cell_mass = rectangle.density * width * height * (zs[z] - zs[z - 1]);
                        mass += cell_mass;
                        first_moment += center * cell_mass;
                        polar += cell_mass * ((width * width + height * height) / 12.0 + rigidbodies::math::length_squared(center));
                        break;
                    }
                }
            }
        }
        const auto center = first_moment / mass;
        expect_properties(compute_compound_mass_properties(colliders), mass, center, polar - mass * rigidbodies::math::length_squared(center));
    }

    RIGIDBODIES_TEST("rotated convex overlaps agree with an octagon intersection closed form")
    {
        const auto leg = 2.0 - std::sqrt(2.0);
        const auto triangle_polar = leg * leg - (2.0 / 3.0) * leg * leg * leg + std::pow(leg, 4) / 6.0;
        const auto overlap_area = 4.0 - 2.0 * leg * leg;
        const auto overlap_polar = 8.0 / 3.0 - 4.0 * triangle_polar;
        const auto first = part(make_box(2.0, 2.0));
        const auto second = part(make_box(2.0, 2.0), 1.0, 1.0, {}, pi * 0.25);
        expect_properties(compute_compound_mass_properties({ first, second }), 8.0 - overlap_area, {}, 16.0 / 3.0 - overlap_polar);
        auto heavier = second;
        heavier.density_override_kg_m3 = 3.0;
        expect_properties(compute_compound_mass_properties({ first, heavier }), 4.0 + 3.0 * (4.0 - overlap_area), {}, 8.0 / 3.0 + 3.0 * (8.0 / 3.0 - overlap_polar));
    }

    RIGIDBODIES_TEST("mixed circle and polygon boundaries retain exact arc mass and centroid")
    {
        const auto circle = part(make_circle(1.0));
        const auto right_box = part(make_box(1.0, 2.0), 1.0, 1.0, { 0.5, 0.0 });
        const auto mass = 2.0 + pi * 0.5;
        const auto center_x = (1.0 / 3.0) / mass;
        const auto inertia = 4.0 / 3.0 + pi * 0.25 - mass * center_x * center_x;
        expect_properties(compute_compound_mass_properties({ circle, right_box }), mass, { center_x, 0.0 }, inertia);
        expect_properties(compute_compound_mass_properties({ right_box, circle }), mass, { center_x, 0.0 }, inertia);
    }

    RIGIDBODIES_TEST("three overlapping collinear polygons remove all repeated volume and duplicate boundaries")
    {
        std::vector<Collider> pieces { part(make_box(2.0, 2.0)), part(make_box(2.0, 2.0), 1.0, 1.0, { 0.5, 0.0 }), part(make_box(2.0, 2.0), 1.0, 1.0, { 1.0, 0.0 }) };
        expect_properties(compute_compound_mass_properties(pieces), 6.0, { 0.5, 0.0 }, 6.5);
        pieces.push_back(pieces[0]);
        expect_properties(compute_compound_mass_properties(pieces), 6.0, { 0.5, 0.0 }, 6.5);
        BodyDefinition definition;
        definition.colliders = pieces;
        expect_properties(RigidBody { definition }.mass_properties(), 6.0, { 0.5, 0.0 }, 6.5);
    }

    RIGIDBODIES_TEST("hollow cavities can be filled and overlapping shell walls obey density and depth precedence")
    {
        auto shell = part(make_circle(2.0), 3.0);
        shell.shell_thickness_m = 1.0;
        auto fill = part(make_circle(1.5), 5.0);
        expect_properties(compute_compound_mass_properties({ shell, fill }), 14.0 * pi, {}, 25.0 * pi);
        fill.depth_m = 2.0;
        expect_properties(compute_compound_mass_properties({ shell, fill }), 25.25 * pi, {}, 37.65625 * pi);
        shell.density_override_kg_m3 = 1.0;
        fill.density_override_kg_m3 = 1.0;
        fill.depth_m = 1.0;
        expect_properties(compute_compound_mass_properties({ shell, fill }), 4.0 * pi, {}, 8.0 * pi);
        expect_properties(compute_compound_mass_properties({ shell, shell }), 3.0 * pi, {}, 7.5 * pi);
    }

    RIGIDBODIES_TEST("overlapping rectangular shells and inserts form a solid rectangle without counting shared walls twice")
    {
        auto shell = part(make_box(4.0, 2.0));
        shell.shell_thickness_m = 0.25;
        const auto insert = part(make_box(3.75, 1.75));
        expect_properties(compute_compound_mass_properties({ shell, insert }), 8.0, {}, 40.0 / 3.0);
        expect_properties(compute_compound_mass_properties({ insert, shell }), 8.0, {}, 40.0 / 3.0);
    }

    RIGIDBODIES_TEST("touching and contained outlines have no fictitious overlap mass")
    {
        expect_properties(compute_compound_mass_properties({ part(make_circle(1.0), 1.0, 1.0, { -1.0, 0.0 }),
                              part(make_circle(1.0), 1.0, 1.0, { 1.0, 0.0 }) }),
            2.0 * pi,
            {},
            3.0 * pi);
        expect_properties(compute_compound_mass_properties({ part(make_box(2.0, 2.0), 1.0, 1.0, { -1.0, 0.0 }),
                              part(make_box(2.0, 2.0), 1.0, 1.0, { 1.0, 0.0 }) }),
            8.0,
            {},
            40.0 / 3.0);
        expect_properties(compute_compound_mass_properties({ part(make_circle(2.0)), part(make_circle(0.5), 20.0, 1.0, { 0.2, 0.0 }) }),
            4.0 * pi,
            {},
            8.0 * pi);
    }

    RIGIDBODIES_TEST("compound mass is invariant under common rigid placement and follows dimensional scaling")
    {
        const auto base = compute_compound_mass_properties({ part(make_circle(1.0)), part(make_box(1.0, 2.0), 1.0, 1.0, { 0.5, 0.0 }) });
        const auto placement = rigidbodies::math::Transform2::from_angle({ 123.0, -456.0 }, 1.13);
        const auto factor = 0.03;
        auto circle = part(make_circle(factor), 1.0, 1.0, placement.translation, 1.13);
        auto box = part(make_box(factor, 2.0 * factor));
        box.local_transform = rigidbodies::math::concatenate(placement, rigidbodies::math::Transform2::from_angle({ factor * 0.5, 0.0 }, 0.0));
        expect_properties(compute_compound_mass_properties({ circle, box }), base.mass_kg * factor * factor, rigidbodies::math::transform_point(placement, base.center_of_mass_m * factor), base.inertia_kg_m2 * std::pow(factor, 4), 1.0e-10);
    }

    RIGIDBODIES_TEST("zero density depth and wall thickness are massless and do not displace other material")
    {
        auto shell = part(make_box(4.0, 4.0));
        shell.shell_thickness_m = 0.0;
        const auto no_density = part(make_circle(4.0), 0.0);
        const auto no_depth = part(make_circle(4.0), 3.0, 0.0);
        const auto line = part(make_segment({ -10.0, 0.0 }, { 10.0, 0.0 }));
        expect_properties(compute_compound_mass_properties({ shell, no_density, no_depth, line, Collider {} }), 0.0, {}, 0.0);
        expect_properties(compute_compound_mass_properties({ shell, no_density, no_depth, line, part(make_circle(1.0)) }), pi, {}, pi * 0.5);
    }

    RIGIDBODIES_TEST("invalid density depth and shell parameters fail explicitly")
    {
        for (const auto value : { -1.0, std::numeric_limits<Real>::infinity(), std::numeric_limits<Real>::quiet_NaN() })
        {
            for (int field = 0; field < 3; ++field)
            {
                auto collider = part(make_circle(1.0));
                if (field == 0)
                {
                    collider.density_override_kg_m3 = value;
                }
                else if (field == 1)
                {
                    collider.depth_m = value;
                }
                else
                {
                    collider.shell_thickness_m = value;
                }
                bool rejected = false;
                try
                {
                    (void)collider.compute_mass_properties();
                }
                catch (const std::invalid_argument&)
                {
                    rejected = true;
                }
                RIGIDBODIES_EXPECT(rejected, "invalid mass parameters are rejected before computation");
            }
        }
    }

    RIGIDBODIES_TEST("mass distribution scenario exposes hollow inertia and unequal compound density")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "mass_distribution"), "mass demonstration is registered");
        const auto find_named = [&](std::string_view name) -> const RigidBody*
        {
            for (const auto id : world.body_ids())
            {
                const auto* body = world.find_body(id);
                if (body->name() == name)
                {
                    return body;
                }
            }
            return nullptr;
        };
        const auto* solid = find_named("solid_disc");
        const auto* hollow = find_named("hollow_tube");
        const auto* uniform = find_named("uniform_density_assembly");
        const auto* unequal = find_named("unequal_density_assembly");
        RIGIDBODIES_EXPECT(solid && hollow && uniform && unequal, "all four named demonstration bodies exist");
        RIGIDBODIES_EXPECT_NEAR(solid->mass_properties().mass_kg, hollow->mass_properties().mass_kg, 1.0e-12, "upper discs have equal mass");
        RIGIDBODIES_EXPECT(hollow->mass_properties().inertia_kg_m2 > solid->mass_properties().inertia_kg_m2, "hollow disc has larger inertia at the same mass");
        RIGIDBODIES_EXPECT(hollow->angular_velocity_rad_s() < solid->angular_velocity_rad_s(), "same spin impulse makes hollow disc rotate more slowly");
        RIGIDBODIES_EXPECT_NEAR(uniform->mass_properties().mass_kg, 0.85 * 0.3 * 0.03 * 350.0, 1.0e-12, "overlap contributes material only once");
        RIGIDBODIES_EXPECT(unequal->mass_properties().center_of_mass_m.x > 0.1, "unequal density visibly displaces the centre of mass");
    }

    RIGIDBODIES_TEST("prescribed motion scenario contains framed periodic paths and replays after restore")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "prescribed_motion"), "prescribed-motion demonstration is registered");
        BodyId slider;
        BodyId paddle;
        for (const auto id : world.body_ids())
        {
            const auto* body = world.find_body(id);
            if (body->name() == "harmonic_platform")
            {
                slider = id;
            }
            if (body->name() == "orbiting_paddle")
            {
                paddle = id;
            }
        }
        RIGIDBODIES_EXPECT(world.kinematic_motion(slider) && world.kinematic_motion(paddle), "both paths are attached");
        const auto bounds = world.compute_bounds();
        RIGIDBODIES_EXPECT(bounds.contains({ -0.9, 0.65 }) && bounds.contains({ 0.9, -0.65 }), "initial bounds frame the complete periodic travel");
        const auto saved = world.snapshot();
        world.step(1.0);
        const auto slider_position = world.find_body(slider)->world_center_of_mass_m();
        const auto paddle_position = world.find_body(paddle)->world_center_of_mass_m();
        RIGIDBODIES_EXPECT_NEAR(slider_position.y, 0.0, 1.0e-12, "harmonic position follows elapsed time");
        RIGIDBODIES_EXPECT_NEAR(paddle_position.x, 0.9 + 0.65 * std::cos(0.8), 1.0e-12, "circular position follows elapsed time");
        world.restore(saved);
        world.step(1.0);
        RIGIDBODIES_EXPECT(world.find_body(slider)->world_center_of_mass_m() == slider_position, "restored harmonic drive replays exactly");
        RIGIDBODIES_EXPECT(world.find_body(paddle)->world_center_of_mass_m() == paddle_position, "restored circular drive replays exactly");
    }

} // namespace

int main()
{
    return rigidbodies::testing::run_all();
}
