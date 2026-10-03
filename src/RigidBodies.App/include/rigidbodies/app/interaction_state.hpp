#pragma once

#include <rigidbodies/physics/force_generator.hpp>

#include <deque>
#include <string>
#include <vector>

namespace rigidbodies::app
{
    enum class InteractionMode
    {
        select,
        move,
        throw_body,
        pull
    };

    struct InteractionBody
    {
        physics::BodyId id;
        math::Vec2 original_position_m;
    };

    struct PointerMotionSample
    {
        math::Vec2 position_m;
        double time_s {};
    };

    // One captured pointer gesture. Selection and undo state belong to the session; no platform
    // clock or mouse state is needed to replay a gesture in a window-free test.
    struct InteractionState
    {
        InteractionMode mode { InteractionMode::select };
        bool active { false };
        bool moved { false };
        bool marquee { false };
        physics::BodyId anchor_body;
        std::vector<InteractionBody> bodies;
        math::Vec2 press_world_m, target_world_m, local_anchor_m, press_screen_px;
        physics::BodyId hover_body;
        std::string hover_connection_key, hover_connection_kind;
        double hover_started_s {};
        std::deque<PointerMotionSample> samples;
        physics::ForceGeneratorPtr pull_generator;
    };

    struct HandleDragState
    {
        std::string id;
        physics::BodyId body;
        math::Vec2 press_screen_px {};
        // Device pixels per metre per second of the velocity arrow when the drag began.
        double velocity_scale {};
        bool active {}, moved {};
    };
}
