#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <array>
#include <algorithm>

namespace
{
    using namespace rigidbodies;
    using namespace rigidbodies::physics;

    BodyId named_body(const World& world, std::string_view name)
    {
        for (const auto id : world.body_ids())
            if (world.find_body(id)->name() == name)
                return id;
        RIGIDBODIES_FAIL("mechanism has its named body");
    }

    std::shared_ptr<JointConstraint> named_joint(const World& world, std::string_view key)
    {
        auto result = std::dynamic_pointer_cast<JointConstraint>(world.constraint_by_key(key));
        RIGIDBODIES_EXPECT(result != nullptr, "mechanism has its stable named joint");
        return result;
    }

    void step(World& world)
    {
        world.step(1.0 / 120.0);
        RIGIDBODIES_EXPECT(world.statistics().limit_event_count == 0, "mechanism remains inside numerical motion limits");
        RIGIDBODIES_EXPECT(world.statistics().sweep_iteration_limit_count == 0, "mechanism does not exhaust continuous collision work");
        for (const auto id : world.body_ids())
        {
            const auto* body = world.find_body(id);
            RIGIDBODIES_EXPECT(math::is_finite(body->position_m()) && math::is_finite(body->linear_velocity_m_s()) && math::is_finite(body->orientation_rad()), "mechanism state remains finite");
        }
    }

    RIGIDBODIES_TEST("articulation catalogue exposes all five mechanisms with keyed joints and framing bounds")
    {
        const std::array<std::pair<const char*, std::size_t>, 5> scenes {
            std::pair { "distance_chain", 5u }, std::pair { "revolute_drive", 1u }, std::pair { "prismatic_drive", 1u }, std::pair { "welded_assembly", 1u }, std::pair { "breakable_joint", 2u }
        };
        for (const auto& [id, joints] : scenes)
        {
            World world;
            RIGIDBODIES_EXPECT(load_scenario(world, id), "mechanism scene is registered");
            RIGIDBODIES_EXPECT(!world.compute_bounds().is_empty(), "initial geometry supplies camera framing bounds");
            RIGIDBODIES_EXPECT(world.constraints().size() == joints, "mechanism has the expected number of joints");
            for (const auto& constraint : world.constraints())
                RIGIDBODIES_EXPECT(!world.constraint_key(constraint).empty(), "every scenario connection has a stable key for interface control");
        }
    }

    RIGIDBODIES_TEST("distance chain swings while every rod stays close to its prescribed length")
    {
        World world;
        load_scenario(world, "distance_chain");
        const auto last = named_body(world, "chain_mass_5");
        const auto initial = world.find_body(last)->position_m();
        for (int frame = 0; frame < 360; ++frame)
        {
            step(world);
            for (const auto& constraint : world.constraints())
            {
                const auto joint = std::dynamic_pointer_cast<JointConstraint>(constraint);
                const auto report = joint->report(world);
                RIGIDBODIES_EXPECT(report.enabled && !report.broken, "ordinary chain loads do not disable a link");
                RIGIDBODIES_EXPECT_NEAR(report.distance_m, std::get<DistanceJointDefinition>(joint->definition()).length_m, 0.025, "each rod holds length throughout the swing");
            }
        }
        RIGIDBODIES_EXPECT(math::distance(world.find_body(last)->position_m(), initial) > 0.2, "chain visibly responds to gravity");
        RIGIDBODIES_EXPECT(world.statistics().active_constraint_count == 5 && world.statistics().constraint_row_count >= 5, "graph solves all five load-carrying links");
    }

    RIGIDBODIES_TEST("driven hinge reaches its angular stop while the pin remains attached")
    {
        World world;
        load_scenario(world, "revolute_drive");
        const auto hinge = named_joint(world, "driven_hinge");
        const auto definition = std::get<RevoluteJointDefinition>(hinge->definition());
        Real largest_angle = 0.0;
        Real largest_motor_torque = 0.0;
        for (int frame = 0; frame < 360; ++frame)
        {
            step(world);
            const auto report = hinge->report(world);
            largest_angle = std::max(largest_angle, report.angle_rad);
            largest_motor_torque = std::max(largest_motor_torque, std::abs(report.motor_torque_n_m));
            RIGIDBODIES_EXPECT(report.position_error_m < 0.025, "hinge anchor stays fixed while the bar turns");
            RIGIDBODIES_EXPECT(report.angle_rad >= definition.lower_angle_rad - 0.03 && report.angle_rad <= definition.upper_angle_rad + 0.03, "hinge remains within its physical angular stops");
        }
        RIGIDBODIES_EXPECT(largest_angle > 0.6, "motor visibly drives the hanging bar");
        RIGIDBODIES_EXPECT_NEAR(hinge->report(world).angle_rad, definition.upper_angle_rad, 0.035, "motor settles against the positive stop");
        RIGIDBODIES_EXPECT(largest_motor_torque > 0.1 && largest_motor_torque <= definition.maximum_motor_torque_n_m + 1.0e-9, "motor supplies bounded torque rather than prescribing body motion");
    }

    RIGIDBODIES_TEST("driven slider traverses its rail without side drift or rotation and stops at its travel limit")
    {
        World world;
        load_scenario(world, "prismatic_drive");
        const auto slider = named_joint(world, "driven_slider");
        const auto definition = std::get<PrismaticJointDefinition>(slider->definition());
        const auto carriage = named_body(world, "driven_carriage");
        Real maximum_speed = 0.0;
        for (int frame = 0; frame < 420; ++frame)
        {
            step(world);
            const auto report = slider->report(world);
            maximum_speed = std::max(maximum_speed, world.find_body(carriage)->linear_velocity_m_s().x);
            RIGIDBODIES_EXPECT(report.position_error_m < 0.025 && std::abs(report.angular_error_rad) < 0.025, "slider rail blocks transverse motion and rotation");
            RIGIDBODIES_EXPECT(report.translation_m >= definition.lower_translation_m - 0.03 && report.translation_m <= definition.upper_translation_m + 0.03, "carriage stays within travel stops");
            RIGIDBODIES_EXPECT(std::abs(report.motor_force_n) <= definition.maximum_motor_force_n + 1.0e-9, "slider motor remains force-limited");
        }
        RIGIDBODIES_EXPECT(maximum_speed > 0.6, "motor propels the carriage at an observable speed");
        RIGIDBODIES_EXPECT_NEAR(slider->report(world).translation_m, definition.upper_translation_m, 0.03, "carriage reaches and holds its end stop");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(carriage)->position_m().y, 0.14, 0.015, "rail supplies the vertical support reaction");
    }

    RIGIDBODIES_TEST("welded bodies rotate together while preserving their relative placement")
    {
        World world;
        load_scenario(world, "welded_assembly");
        const auto weld = named_joint(world, "welded_pair");
        const auto first_id = named_body(world, "welded_bar");
        const auto second_id = named_body(world, "welded_weight");
        const auto original = math::inverse_transform_point(world.find_body(first_id)->transform(), world.find_body(second_id)->world_center_of_mass_m());
        for (int frame = 0; frame < 480; ++frame)
        {
            step(world);
            const auto report = weld->report(world);
            RIGIDBODIES_EXPECT(report.position_error_m < 0.025 && std::abs(report.angular_error_rad) < 0.025, "weld keeps both anchor and angular relationships");
        }
        const auto* first = world.find_body(first_id);
        const auto* second = world.find_body(second_id);
        const auto final = math::inverse_transform_point(first->transform(), second->world_center_of_mass_m());
        RIGIDBODIES_EXPECT_NEAR(final.x, original.x, 0.025, "weld preserves local horizontal separation");
        RIGIDBODIES_EXPECT_NEAR(final.y, original.y, 0.025, "weld preserves local vertical separation");
        RIGIDBODIES_EXPECT(std::abs(first->orientation_rad()) > 1.0 && std::abs(second->orientation_rad()) > 1.0, "both separate bodies participate in the visible rotation");
    }

    RIGIDBODIES_TEST("load threshold breaks the weak link while its stronger comparison remains suspended")
    {
        World world;
        load_scenario(world, "breakable_joint");
        const auto weak = named_joint(world, "weak_link");
        const auto strong = named_joint(world, "strong_link");
        const auto weak_body = named_body(world, "weak_joint_load");
        const auto strong_body = named_body(world, "strong_joint_load");
        bool observed_break = false;
        for (int frame = 0; frame < 240; ++frame)
        {
            step(world);
            observed_break = observed_break || !world.constraint_break_events().empty();
        }
        RIGIDBODIES_EXPECT(observed_break && weak->is_broken() && !strong->is_broken(), "only the weaker threshold creates a reported break event");
        RIGIDBODIES_EXPECT(weak->broken_force_n() > 3.0 && math::is_finite(weak->broken_force_n()), "break retains its actual overloaded reaction for explanation");
        RIGIDBODIES_EXPECT(world.find_body(weak_body)->position_m().y < -1.0, "broken connection releases its mass to fall onto the floor");
        RIGIDBODIES_EXPECT_NEAR(world.find_body(strong_body)->position_m().y, 0.25, 0.02, "strong connection continues to support the equal load");
        RIGIDBODIES_EXPECT(world.statistics().broken_constraint_count == 1, "broken connection remains inspectable in the world");
    }

    RIGIDBODIES_TEST("mechanism snapshots restore keyed joints and replay motor and break states")
    {
        for (const auto* scene : { "revolute_drive", "prismatic_drive", "breakable_joint" })
        {
            World world;
            load_scenario(world, scene);
            const auto checkpoint = world.snapshot();
            const auto ids = world.body_ids();
            for (int frame = 0; frame < 60; ++frame)
                step(world);
            std::vector<math::Vec2> positions;
            std::vector<Real> angles;
            for (const auto id : ids)
            {
                positions.push_back(world.find_body(id)->position_m());
                angles.push_back(world.find_body(id)->orientation_rad());
            }
            const auto broken_count = world.statistics().broken_constraint_count;
            world.restore(checkpoint);
            for (int frame = 0; frame < 60; ++frame)
                step(world);
            for (std::size_t index = 0; index < ids.size(); ++index)
                RIGIDBODIES_EXPECT(world.find_body(ids[index])->position_m() == positions[index] && world.find_body(ids[index])->orientation_rad() == angles[index], "snapshot replays mechanism motion exactly");
            RIGIDBODIES_EXPECT(world.statistics().broken_constraint_count == broken_count, "break state replays with the original force threshold");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
