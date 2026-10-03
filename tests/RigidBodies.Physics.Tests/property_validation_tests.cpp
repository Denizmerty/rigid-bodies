#include <rigidbodies/physics/world.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>

namespace
{
    using namespace rigidbodies::physics;
    using rigidbodies::math::Vec2;

    // Explicit SplitMix64 mapping avoids implementation-dependent uniform_real_distribution.
    // Each failed property prints its seed and case index, making every generated case replayable.
    class Generator
    {
    public:
        explicit Generator(std::uint64_t seed) : state_(seed)
        {
        }
        std::uint64_t next()
        {
            auto value = (state_ += UINT64_C(0x9e3779b97f4a7c15));
            value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
            value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
            return value ^ (value >> 31U);
        }
        Real between(Real low, Real high)
        {
            return low + (high - low) * static_cast<Real>(next() >> 11U) * (1.0 / 9007199254740992.0);
        }
        Vec2 vector(Real magnitude)
        {
            return { between(-magnitude, magnitude), between(-magnitude, magnitude) };
        }

    private:
        std::uint64_t state_;
    };

    void generated(std::uint64_t seed, int count, const std::function<void(Generator&, int)>& property)
    {
        Generator random(seed);
        for (int index = 0; index < count; ++index)
        {
            try
            {
                property(random, index);
            }
            catch (const std::exception& error)
            {
                throw std::runtime_error("property seed=" + std::to_string(seed) + " case=" + std::to_string(index) + ": " + error.what());
            }
        }
    }

    BodyDefinition definition(Generator& random, Vec2 position)
    {
        BodyDefinition result;
        result.position_m = position;
        result.linear_velocity_m_s = random.vector(2.0);
        result.angular_velocity_rad_s = random.between(-4.0, 4.0);
        result.orientation_rad = random.between(-3.0, 3.0);
        result.sleep_enabled = false;
        Collider collider;
        const auto width = random.between(0.2, 1.0);
        const auto height = random.between(0.2, 1.0);
        collider.shape = make_box(width, height);
        collider.material.restitution = random.between(0.0, 1.0);
        result.colliders.push_back(std::move(collider));
        return result;
    }

    struct Conserved
    {
        Vec2 momentum;
        Real angular {}, energy {};
    };

    Conserved measure(const World& world)
    {
        Conserved result;
        world.for_each_body([&](BodyId, const RigidBody& body)
            {
                const auto momentum = body.linear_momentum_kg_m_s();
                result.momentum += momentum;
                result.angular += rigidbodies::math::cross(body.world_center_of_mass_m(), momentum) + body.angular_momentum_about_center_kg_m2_s();
                result.energy += body.kinetic_energy_j();
            });
        return result;
    }

    void expect_momentum(const Conserved& before, const Conserved& after)
    {
        const auto scale = 1.0 + std::abs(before.momentum.x) + std::abs(before.momentum.y) + std::abs(before.angular) + before.energy;
        RIGIDBODIES_EXPECT_NEAR(after.momentum.x, before.momentum.x, 2.0e-11 * scale, "isolated horizontal momentum is conserved");
        RIGIDBODIES_EXPECT_NEAR(after.momentum.y, before.momentum.y, 2.0e-11 * scale, "isolated vertical momentum is conserved");
        RIGIDBODIES_EXPECT_NEAR(after.angular, before.angular, 2.0e-11 * scale, "spin plus orbital angular momentum is conserved");
    }

    void impulse_case(Generator& random, bool friction)
    {
        World world;
        const auto angle = random.between(-3.0, 3.0);
        const Vec2 normal { std::cos(angle), std::sin(angle) };
        const Vec2 tangent { -normal.y, normal.x };
        const auto origin = random.vector(10.0);
        auto first_definition = definition(random, origin - normal * 0.5);
        auto second_definition = definition(random, origin + normal * 0.5);
        // Sequence draws explicitly: operand/argument evaluation order differs among compilers.
        const auto first_normal_speed = random.between(4.0, 8.0);
        const auto first_tangent_speed = random.between(-3.0, 3.0);
        const auto second_normal_speed = random.between(4.0, 8.0);
        const auto second_tangent_speed = random.between(-3.0, 3.0);
        first_definition.linear_velocity_m_s = normal * first_normal_speed + tangent * first_tangent_speed;
        second_definition.linear_velocity_m_s = -normal * second_normal_speed + tangent * second_tangent_speed;
        const auto first = world.create_body(first_definition);
        const auto second = world.create_body(second_definition);
        world.find_body(first)->override_mass(random.between(0.05, 50.0));
        world.find_body(second)->override_mass(random.between(0.05, 50.0));
        ContactManifold contact;
        contact.first = first;
        contact.second = second;
        contact.normal = normal;
        contact.point_count = 1;
        contact.points[0].world_position_m = origin + tangent * random.between(-0.4, 0.4);
        contact.material.restitution = friction ? 0.0 : random.between(0.0, 1.0);
        if (friction)
        {
            contact.material.static_friction = random.between(0.0, 1.0);
            contact.material.kinetic_friction = contact.material.static_friction * random.between(0.0, 1.0);
            contact.material.rolling_friction_m = random.between(0.0, 0.15);
            contact.material.spinning_friction_m = random.between(0.0, 0.15);
        }
        std::vector<ContactManifold> contacts { contact };
        const auto before = measure(world);
        SequentialImpulseContactSolver solver;
        SolverSettings settings;
        settings.warm_starting = false;
        settings.restitution_threshold_m_s = 0.0;
        solver.prepare(world, contacts, settings, 1.0 / 120.0);
        for (int iteration = 0; iteration < 16; ++iteration)
            solver.solve_velocity(world, contacts, settings, 1.0 / 120.0);
        const auto after = measure(world);
        expect_momentum(before, after);
        RIGIDBODIES_EXPECT(std::isfinite(after.energy) && after.energy <= before.energy + 2.0e-10 * (1.0 + before.energy),
            "passive contact impulses cannot create kinetic energy in an isolated two-body system");
    }

    RIGIDBODIES_TEST("generated isolated impacts conserve linear and orbital plus spin angular momentum")
    {
        generated(UINT64_C(0x11c0111510), 256, [](Generator& random, int)
            {
                impulse_case(random, false);
            });
    }

    RIGIDBODIES_TEST("generated friction rolling and spinning impulses conserve momentum and dissipate energy")
    {
        generated(UINT64_C(0x11d1551a7e), 256, [](Generator& random, int)
            {
                impulse_case(random, true);
            });
    }

    RIGIDBODIES_TEST("generated isolated free flights conserve energy and angular momentum about the origin")
    {
        generated(UINT64_C(0x11f1ee), 96, [](Generator& random, int)
            {
                WorldSettings settings;
                settings.gravity_m_s2 = {};
                settings.sleep.enabled = false;
                World world(settings);
                world.set_parallel_settings({ 1, 64 });
                for (int index = 0; index < 4; ++index)
                {
                    auto body = definition(random, { static_cast<Real>(index) * 8.0 - 12.0, 0.0 });
                    body.colliders[0].filter.mask = 0;
                    const auto id = world.create_body(body);
                    world.find_body(id)->override_mass(random.between(0.1, 30.0));
                }
                const auto before = measure(world);
                for (int step = 0; step < 48; ++step)
                    world.step(1.0 / 120.0);
                const auto after = measure(world);
                expect_momentum(before, after);
                RIGIDBODIES_EXPECT_NEAR(after.energy, before.energy, 2.0e-11 * (1.0 + before.energy), "free flight has no energy source or sink");
                RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "the conservation property never relies on motion clamps");
            });
    }

    RIGIDBODIES_TEST("generated passive body damping never increases kinetic energy step by step")
    {
        generated(UINT64_C(0x11da001a), 96, [](Generator& random, int)
            {
                WorldSettings settings;
                settings.gravity_m_s2 = {};
                settings.sleep.enabled = false;
                World world(settings);
                world.set_parallel_settings({ 1, 64 });
                auto body = definition(random, {});
                body.linear_damping = random.between(0.0, 180.0);
                body.angular_damping = random.between(0.0, 180.0);
                const auto id = world.create_body(body);
                world.find_body(id)->override_mass(random.between(0.1, 30.0));
                auto previous = measure(world).energy;
                for (int step = 0; step < 32; ++step)
                {
                    world.step(random.between(1.0 / 1000.0, 1.0 / 30.0));
                    const auto energy = measure(world).energy;
                    RIGIDBODIES_EXPECT(std::isfinite(energy) && energy <= previous + 1.0e-12 * (1.0 + previous),
                        "damping is passive even when its unconstrained factor would cross zero");
                    previous = energy;
                }
            });
    }

    RIGIDBODIES_TEST("generated bounded stress and storage mutation keep every live body finite")
    {
        generated(UINT64_C(0x11f1017e), 48, [](Generator& random, int trial)
            {
                World world;
                world.set_parallel_settings({ static_cast<std::size_t>(trial % 2 == 0 ? 1 : 4), 1 });
                for (int index = 0; index < 8; ++index)
                    (void)world.create_body(definition(random, random.vector(8.0)));
                for (int step = 0; step < 24; ++step)
                {
                    auto ids = world.body_ids();
                    if (step % 3 == 0)
                    {
                        const auto removed = ids[static_cast<std::size_t>(random.next() % ids.size())];
                        RIGIDBODIES_EXPECT(world.destroy_body(removed), "generated mutation removes a live generation");
                        const auto replacement = world.create_body(definition(random, random.vector(10.0)));
                        RIGIDBODIES_EXPECT(world.find_body(removed) == nullptr && replacement.generation != removed.generation,
                            "generation reuse cannot resurrect an old handle");
                        ids = world.body_ids();
                    }
                    const auto id = ids[static_cast<std::size_t>(random.next() % ids.size())];
                    auto& selected = *world.find_body(id);
                    selected.override_mass(std::pow(10.0, random.between(-4.0, 4.0)));
                    selected.set_linear_velocity(random.vector(1.0e6));
                    selected.set_angular_velocity(random.between(-1.0e6, 1.0e6));
                    selected.apply_force_at_center(random.vector(1.0e12));
                    selected.apply_torque(random.between(-1.0e12, 1.0e12));
                    world.step(random.between(1.0 / 1000.0, 1.0 / 60.0));
                    world.for_each_body([](BodyId, const RigidBody& current)
                        {
                            RIGIDBODIES_EXPECT(rigidbodies::math::is_finite(current.position_m()) &&
                                    rigidbodies::math::is_finite(current.linear_velocity_m_s()) && std::isfinite(current.orientation_rad()) &&
                                    std::isfinite(current.angular_velocity_rad_s()) && std::isfinite(current.kinetic_energy_j()) &&
                                    std::isfinite(current.inverse_mass()) && std::isfinite(current.inverse_inertia()),
                                "stress and mutation leave position, motion, mass response and energy finite");
                        });
                    RIGIDBODIES_EXPECT(std::isfinite(world.statistics().total_kinetic_energy_j), "aggregate reporting remains finite");
                }
            });
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
