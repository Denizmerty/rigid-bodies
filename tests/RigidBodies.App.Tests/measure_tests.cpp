#include <rigidbodies/app/simulation_session.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies;

    void send(app::SimulationSession& session, ui::UiCommandKind kind)
    {
        ui::UiCommand command;
        command.kind = kind;
        session.apply(command);
    }

    RIGIDBODIES_TEST("theory checks are separate tests that leave the scene untouched")
    {
        app::SimulationSession session;
        session.load_scenario("collision_comparison");
        const auto ids = session.world().body_ids();
        std::vector<math::Transform2> transforms;
        transforms.reserve(ids.size());
        for (const auto id : ids)
            transforms.push_back(session.world().find_body(id)->transform());
        const auto elapsed = session.world().statistics().elapsed_time_s;
        send(session, ui::UiCommandKind::compare_collisions);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(!model.collision_comparison.empty(), "collision theory results remain in the model");
        RIGIDBODIES_EXPECT(session.world().body_ids() == ids && session.world().statistics().elapsed_time_s == elapsed, "the synthetic check never changes the learner's scene or clock");
        for (std::size_t index = 0; index < ids.size(); ++index)
            RIGIDBODIES_EXPECT(session.world().find_body(ids[index])->transform().translation == transforms[index].translation && session.world().find_body(ids[index])->transform().rotation.angle() == transforms[index].rotation.angle(), "theory checks preserve every pose");
    }

    RIGIDBODIES_TEST("Measure publishes scene energy and a bounded impact list")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        auto model = session.build_model();
        RIGIDBODIES_EXPECT(model.world != nullptr, "Measure receives live scene accounting");
        RIGIDBODIES_EXPECT(model.impacts.size() <= 64, "the impact evidence list is bounded");
    }

    RIGIDBODIES_TEST("impacts carry the names the stage shows and whether each body moves freely")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        session.stepper().set_paused(false);
        for (int frame = 0; frame < 90 && session.build_model().impacts.empty(); ++frame)
            session.advance(1.0 / 60.0);
        const auto model = session.build_model();
        RIGIDBODIES_EXPECT(!model.impacts.empty(), "the balls reach the floor within a second and a half");
        if (model.impacts.empty())
            return;
        const auto& impact = model.impacts.front();
        const auto shown = [&](physics::BodyId id)
        {
            for (const auto& object : model.objects)
                if (object.id == id)
                    return object.display_name;
            return std::string {};
        };
        RIGIDBODIES_EXPECT(impact.first_name == shown(impact.first) && impact.second_name == shown(impact.second), "the Collisions list uses the Inspector's names, not internal identifiers");
        RIGIDBODIES_EXPECT(impact.first_name.find('_') == std::string::npos && impact.second_name.find('_') == std::string::npos, "no snake_case identifier reaches the learner");
        RIGIDBODIES_EXPECT(!impact.between_free_bodies(), "a ball meeting the fixed floor is not a moving pair");
        RIGIDBODIES_EXPECT(impact.first_type == physics::BodyType::static_body || impact.second_type == physics::BodyType::static_body, "the floor's body type travels with the impact");
    }

    RIGIDBODIES_TEST("run and pinned commands do not enter undo history")
    {
        app::SimulationSession session;
        session.load_scenario("free_fall");
        const auto before = session.build_model().undo_history.size();
        ui::UiCommand pin;
        pin.kind = ui::UiCommandKind::pin_run_value;
        pin.id = "mechanical";
        pin.detail = "at_end";
        session.apply(pin);
        ui::UiCommand clear;
        clear.kind = ui::UiCommandKind::clear_runs;
        session.apply(clear);
        RIGIDBODIES_EXPECT(session.build_model().undo_history.size() == before, "measurement organization is view/run state, not a physics edit");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
