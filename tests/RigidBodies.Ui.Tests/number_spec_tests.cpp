#include <rigidbodies/ui/control_spec.hpp>

#include "test_framework.hpp"

#include <cmath>

namespace
{
    using namespace rigidbodies;

    RIGIDBODIES_TEST("number specifications are ordered valid and internally consistent")
    {
        std::string_view previous;
        for (const auto& spec : ui::control_specs())
        {
            RIGIDBODIES_EXPECT(ui::valid_control_key(spec.key), "registry key follows the grammar");
            RIGIDBODIES_EXPECT(previous.empty() || previous < spec.key, "registry keys are sorted and unique");
            previous = spec.key;
            if (spec.kind != ui::ControlKind::number && spec.kind != ui::ControlKind::stepper)
                continue;
            const auto& number = spec.number;
            RIGIDBODIES_EXPECT(number.minimum < number.maximum, "field minimum is below maximum: " + std::string(spec.key));
            RIGIDBODIES_EXPECT(number.step > 0.0 && std::isfinite(number.step), "step is positive: " + std::string(spec.key));
            RIGIDBODIES_EXPECT(number.soft_minimum >= number.minimum && number.soft_maximum <= number.maximum && number.soft_minimum < number.soft_maximum, "slider range lies inside field range");
            for (const auto detent : number.detents)
                RIGIDBODIES_EXPECT(detent >= number.soft_minimum && detent <= number.soft_maximum, "detent lies in slider range");
            if (number.scale == ui::NumberScale::logarithmic)
                RIGIDBODIES_EXPECT(number.soft_minimum > 0.0 && number.soft_maximum / number.soft_minimum >= 100.0, "logarithmic sliders span at least two decades");
        }
    }

    RIGIDBODIES_TEST("household session bounds are marked explicitly")
    {
        const auto* velocity = ui::find_control_spec("object.motion.velocity_x");
        const auto* text_size = ui::find_control_spec("prefs.appearance.text_size");
        const auto* time_step = ui::find_control_spec("world.advanced.time_step_custom");
        RIGIDBODIES_EXPECT(velocity && velocity->number.session_minimum && velocity->number.session_maximum, "velocity matches both session limits");
        RIGIDBODIES_EXPECT(text_size && text_size->number.session_minimum && text_size->number.session_maximum, "text size matches both session limits");
        RIGIDBODIES_EXPECT(time_step && time_step->number.session_minimum && time_step->number.session_maximum, "custom time step matches both session limits");
    }

    RIGIDBODIES_TEST("the convergence controls all have canonical registry entries")
    {
        constexpr std::string_view required[] {
            "bar.speed.choice", "camera.scale.height", "transport.play", "transport.step", "transport.back_to_start", "object.properties.mass", "object.properties.material", "object.motion.velocity_x", "object.motion.spin", "joint.motor.enabled", "object.shape.parts", "world.gravity.strength", "world.gravity.tilt", "world.air.wind_x", "world.air.density", "world.collisions.bounce_rule", "world.advanced.integration_method", "measure.graph.quantities", "measure.graph.clear", "measure.collisions.list", "measure.theory.integration_run", "show.presets.preset", "show.arrows.scale", "show.arrows.split", "prefs.units.system", "prefs.appearance.theme", "prefs.appearance.text_size", "prefs.effects.material_shading", "prefs.limits.spark_budget", "draw.bar.material", "draw.node.edge", "draw.precision.vertex_budget", "draw.actions.apply", "library.cards.card", "library.top.open", "menu.capture.save_image", "tools.mode", "tools.undo", "tools.quit"
        };
        for (const auto key : required)
            RIGIDBODIES_EXPECT(ui::find_control_spec(key) != nullptr, "control is registered under its canonical key: " + std::string(key));
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
