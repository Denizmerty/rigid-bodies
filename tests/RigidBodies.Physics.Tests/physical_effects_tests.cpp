#include <rigidbodies/physics/aerodynamic.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <cmath>

namespace
{
    using namespace rigidbodies::physics;
    namespace math = rigidbodies::math;

    BodyId named(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == name)
                return id;
        RIGIDBODIES_FAIL("expected demonstration body exists");
    }

    void advance(World& world, int steps, Real dt = 1.0 / 240.0)
    {
        for (int step = 0; step < steps; ++step)
        {
            world.step(dt);
            RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "physical effect stays within numerical bounds");
            world.for_each_body([](BodyId, const RigidBody& body)
                {
                    RIGIDBODIES_EXPECT(math::is_finite(body.position_m()) && math::is_finite(body.linear_velocity_m_s()) && math::is_finite(body.angular_velocity_rad_s()), "demonstration state remains finite");
                });
        }
    }

    RIGIDBODIES_TEST("gravity demonstration applies independent scales in the same uniform field")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "gravity_scales"), "gravity demonstration loads");
        advance(world, 24);
        const auto reference = world.find_body(named(world, "gravity_sample_2"))->linear_velocity_m_s().y;
        RIGIDBODIES_EXPECT_NEAR(reference, -standard_gravity_m_s2 * 0.1, 1.0e-12, "normal gravity gives the analytical free-fall speed");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(named(world, "gravity_sample_1"))->linear_velocity_m_s().y, reference * 0.25, 1.0e-12, "one body uses quarter gravity");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(named(world, "gravity_sample_3"))->linear_velocity_m_s().y, reference * 2.0, 1.0e-12, "another uses double gravity");
    }

    RIGIDBODIES_TEST("equal mass plates fall differently because their frontal profiles differ")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "aerodynamic_profiles"), "profile demonstration loads");
        advance(world, 60);
        const auto& broadside = *world.find_body(named(world, "broadside_plate"));
        const auto& edge_on = *world.find_body(named(world, "edge_on_plate"));
        RIGIDBODIES_EXPECT_NEAR(broadside.mass_properties().mass_kg, edge_on.mass_properties().mass_kg, 1.0e-15, "mass is controlled in the comparison");
        RIGIDBODIES_EXPECT(broadside.position_m().y > edge_on.position_m().y + 0.05, "broadside plate falls visibly less far");
        RIGIDBODIES_EXPECT(broadside.linear_velocity_m_s().y > edge_on.linear_velocity_m_s().y + 0.4, "broadside drag reduces downward speed");
        ForceContext context;
        const auto broad_terminal = estimate_terminal_speed(broadside, context);
        const auto edge_terminal = estimate_terminal_speed(edge_on, context);
        RIGIDBODIES_EXPECT(broad_terminal && edge_terminal && broad_terminal->speed_m_s < edge_terminal->speed_m_s * 0.5, "terminal estimate reflects the same projected-profile difference");
    }

    RIGIDBODIES_TEST("opposite spinning discs curve in opposite directions while dissipating motion")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "magnus_effect"), "Magnus demonstration loads");
        advance(world, 48);
        const auto& positive = *world.find_body(named(world, "counterclockwise_disc"));
        const auto& negative = *world.find_body(named(world, "clockwise_disc"));
        RIGIDBODIES_EXPECT(positive.linear_velocity_m_s().y > 0.05 && negative.linear_velocity_m_s().y < -0.05, "lift reverses with spin");
        RIGIDBODIES_EXPECT_NEAR(positive.linear_velocity_m_s().y, -negative.linear_velocity_m_s().y, 1.0e-9, "mirrored initial states produce mirrored lift");
        RIGIDBODIES_EXPECT(math::length(positive.linear_velocity_m_s()) < 3.0 && std::abs(positive.angular_velocity_rad_s()) < 20.0, "drag reduces speed and spin");
        bool named_lift = false;
        for (const auto& channel : positive.applied_force_channels())
            named_lift = named_lift || channel.name == "magnus";
        RIGIDBODIES_EXPECT(named_lift, "Magnus contribution is separately inspectable");
    }

    RIGIDBODIES_TEST("asymmetric falling profiles develop mirrored aerodynamic torque")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "asymmetric_aerodynamics"), "tumbling demonstration loads");
        advance(world, 24);
        const auto& first = *world.find_body(named(world, "weighted_profile_left"));
        const auto& second = *world.find_body(named(world, "weighted_profile_right"));
        RIGIDBODIES_EXPECT(std::abs(first.angular_velocity_rad_s()) > 0.01, "off-centre air pressure starts rotation from rest");
        RIGIDBODIES_EXPECT_NEAR(first.angular_velocity_rad_s(), -second.angular_velocity_rad_s(), 1.0e-8, "mirroring the weighted profile reverses the torque");
    }

    RIGIDBODIES_TEST("spring demonstration distinguishes conservative motion from damping and replays exactly")
    {
        World world;
        RIGIDBODIES_EXPECT(load_scenario(world, "spring_damping"), "spring demonstration loads");
        RIGIDBODIES_EXPECT(world.spring_ids().size() == 3, "two linear springs and an angular spring are present");
        const auto spring_for = [&](BodyId body)
        {
            for (const auto id : world.spring_ids())
                if (world.spring_report(id)->second == body)
                    return id;
            RIGIDBODIES_FAIL("spring endpoint belongs to demonstration body");
        };
        const auto elastic = named(world, "elastic_mass");
        const auto damped = named(world, "damped_mass");
        const auto elastic_spring = spring_for(elastic);
        const auto damped_spring = spring_for(damped);
        const auto initial = world.spring_report(elastic_spring)->potential_energy_j;
        const auto checkpoint = world.snapshot();
        advance(world, 720);
        const auto elastic_energy = world.find_body(elastic)->kinetic_energy_j() + world.spring_report(elastic_spring)->potential_energy_j;
        const auto damped_energy = world.find_body(damped)->kinetic_energy_j() + world.spring_report(damped_spring)->potential_energy_j;
        RIGIDBODIES_EXPECT_NEAR(elastic_energy, initial, initial * 0.04, "undamped spring retains energy within Euler discretization error");
        RIGIDBODIES_EXPECT(damped_energy < initial * 0.01, "damper removes almost all initial oscillation energy");
        const auto final_position = world.find_body(elastic)->position_m();
        const auto final_velocity = world.find_body(elastic)->linear_velocity_m_s();
        world.restore(checkpoint);
        advance(world, 720);
        RIGIDBODIES_EXPECT(world.find_body(elastic)->position_m() == final_position && world.find_body(elastic)->linear_velocity_m_s() == final_velocity, "spring snapshots replay the same demonstration bit for bit");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
