#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/physics/special_relativity.hpp>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <vector>

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
            if (number.scale == ui::NumberScale::rapidity)
                RIGIDBODIES_EXPECT(number.minimum >= 0.0 && number.maximum < 1.0 && number.quantity == core::DisplayQuantity::speed_fraction && number.decimals < 0, "a rapidity spec holds speed fractions below c without rounding: " + std::string(spec.key));
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

    // The probe speed control as the registry will hold it: the field reaches the maximum, the
    // slider stops at 0.99999 c.
    ui::NumberSpec rapidity_spec()
    {
        static constexpr double detents[] { 0.5, 0.9, 0.99, 0.999, 0.9999, 0.99999 };
        ui::NumberSpec spec;
        spec.minimum = 0.0;
        spec.maximum = physics::maximum_speed_fraction;
        spec.soft_minimum = 0.0;
        spec.soft_maximum = 0.99999;
        spec.step = physics::speed_rapidity_step;
        spec.scale = ui::NumberScale::rapidity;
        spec.quantity = core::DisplayQuantity::speed_fraction;
        spec.detents = detents;
        spec.slider = true;
        return spec;
    }

    std::string text_of(double fraction)
    {
        return core::format_value(fraction, core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
    }

    // The stored speed is exactly the speed its own text reads.
    bool reads_as_itself(double fraction)
    {
        const auto parsed = core::parse_quantity(text_of(fraction), core::DisplayQuantity::speed_fraction, core::DisplayUnits::si);
        return parsed && *parsed == fraction;
    }

    std::string described(double fraction)
    {
        std::ostringstream text;
        text.precision(17);
        text << fraction << " (" << text_of(fraction) << ")";
        return text.str();
    }

    RIGIDBODIES_TEST("rapidity sliders are monotonic, never reach c and land on their detents")
    {
        const auto spec = rapidity_spec();
        RIGIDBODIES_EXPECT(ui::slider_value(spec, 0.0) == 0.0 && ui::slider_value(spec, 1.0) == 0.99999, "the slider's ends are rest and exactly 0.99999 c");
        RIGIDBODIES_EXPECT(ui::slider_position(spec, 0.0) == 0.0 && ui::slider_position(spec, 0.99999) == 1.0 && ui::slider_position(spec, physics::maximum_speed_fraction) == 1.0, "speeds past the soft maximum pin the thumb at its end");
        double previous_value = -1.0, previous_position = -1.0;
        for (int index = 0; index <= 1000; ++index)
        {
            const auto value = ui::slider_value(spec, index / 1000.0);
            if (!(value >= previous_value && value < 1.0 && value <= spec.soft_maximum && reads_as_itself(value)))
                RIGIDBODIES_FAIL("slider position " + std::to_string(index) + " gives a non-decreasing speed below c that reads as itself: " + described(value));
            previous_value = value;
            const auto position = ui::slider_position(spec, value);
            if (!(position >= previous_position && position >= 0.0 && position <= 1.0))
                RIGIDBODIES_FAIL("slider positions rise with the speed: " + described(value));
            previous_position = position;
        }
        RIGIDBODIES_EXPECT(ui::slider_value(spec, 999.0 / 1000.0) < 0.99999, "the last step before the end is still below the soft maximum");

        // Every speed of the soft range, spaced evenly in rapidity, comes back to within 2e-3 of itself.
        const auto low = physics::rapidity_from_speed_fraction(spec.soft_minimum);
        const auto high = physics::rapidity_from_speed_fraction(spec.soft_maximum);
        for (int index = 0; index <= 4000; ++index)
        {
            const auto fraction = physics::speed_fraction_from_rapidity(low + (high - low) * index / 4000.0);
            const auto back = ui::slider_value(spec, ui::slider_position(spec, fraction));
            if (!(std::abs(back - fraction) <= 2.0e-3))
                RIGIDBODIES_FAIL("a speed survives the slider round trip to within 2e-3: " + described(fraction) + " came back as " + described(back));
        }

        double previous_detent_position = 0.0;
        for (const auto detent : spec.detents)
        {
            const auto position = ui::slider_position(spec, detent);
            RIGIDBODIES_EXPECT(position > previous_detent_position && position <= 1.0, "detents sit in order along the slider: " + described(detent));
            RIGIDBODIES_EXPECT(ui::slider_value(spec, position) == detent, "a detent's own position maps back to the detent exactly: " + described(detent));
            previous_detent_position = position;
        }
        // Rapidity gives each nine about the same room: 0.9 c sits a quarter of the way along.
        RIGIDBODIES_EXPECT_NEAR(ui::slider_position(spec, 0.9), 0.2412, 1.0e-3, "0.9 c leaves three quarters of the slider for the nines");
    }

    RIGIDBODIES_TEST("rapidity steps are exact and never swallowed")
    {
        const auto spec = rapidity_spec();
        using Size = ui::StepSize;
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.0, 1, Size::normal) == 0.114, "one step from rest is 0.114 c");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.0, -1, Size::normal) == 0.0 && ui::keyboard_step(spec, 0.0, -1, Size::coarse) == 0.0 && ui::keyboard_step(spec, 0.0, -1, Size::fine) == 0.0, "rest is the bottom for every step");

        // Normal steps from every rung of the ladder move the speed and keep it reading as itself.
        for (const auto rung : physics::speed_fraction_ladder)
        {
            const auto up = ui::keyboard_step(spec, rung, 1, Size::normal);
            const auto down = ui::keyboard_step(spec, rung, -1, Size::normal);
            if (rung < physics::maximum_speed_fraction)
                RIGIDBODIES_EXPECT(up > rung && up < 1.0 && reads_as_itself(up), "a normal step up from a rung moves: " + described(rung) + " to " + described(up));
            else
                RIGIDBODIES_EXPECT(up == rung, "the maximum is the top of the normal step");
            if (rung > 0.0)
                RIGIDBODIES_EXPECT(down < rung && down >= 0.0 && reads_as_itself(down), "a normal step down from a rung moves: " + described(rung) + " to " + described(down));
            if (rung >= 0.9 && rung < physics::maximum_speed_fraction)
            {
                // Near c a step is a tenth of a nine: the gap to c shrinks by about a fifth.
                const auto shrink = (1.0 - up) / (1.0 - rung);
                RIGIDBODIES_EXPECT(shrink > 0.7 && shrink < 0.9, "a step up near c takes about a fifth off the gap: " + described(rung));
            }
        }

        // Seeded speeds across the whole range, each first read as its own text.
        std::mt19937_64 engine { 20261005 };
        std::uniform_real_distribution<double> rapidity(0.0, physics::maximum_rapidity());
        for (int sample = 0; sample < 10000; ++sample)
        {
            const auto fraction = physics::speed_fraction_from_rapidity(rapidity(engine));
            const auto start = core::parse_quantity(text_of(fraction), core::DisplayQuantity::speed_fraction, core::DisplayUnits::si).value_or(fraction);
            if (!(start > 0.0))
                continue;
            const auto up = ui::keyboard_step(spec, start, 1, Size::normal);
            const auto down = ui::keyboard_step(spec, start, -1, Size::normal);
            const auto fine_up = ui::keyboard_step(spec, start, 1, Size::fine);
            const auto fine_down = ui::keyboard_step(spec, start, -1, Size::fine);
            const auto top = start >= physics::maximum_speed_fraction;
            const auto ok = (top ? up == start : up > start && text_of(up) != text_of(start)) && down < start && text_of(down) != text_of(start) &&
                (top ? fine_up == start : fine_up > start && text_of(fine_up) != text_of(start)) && fine_down < start && fine_down >= down &&
                reads_as_itself(up) && reads_as_itself(down) && reads_as_itself(fine_up) && reads_as_itself(fine_down) && up <= physics::maximum_speed_fraction && fine_up <= up;
            if (!ok)
                RIGIDBODIES_FAIL("normal and fine steps move a speed to new text, in order, and stay below c: " + described(start) + " up " + described(up) + " down " + described(down) + " fine up " + described(fine_up) + " fine down " + described(fine_down));
        }

        // Coarse steps are the ladder, so a focused field's Shift+arrow is the global Shift+Up.
        for (const auto rung : physics::speed_fraction_ladder)
            for (const auto direction : { -1, 1 })
                RIGIDBODIES_EXPECT(ui::keyboard_step(spec, rung, direction, Size::coarse) == physics::adjacent_speed_preset(rung, direction), "the coarse step is the next speed on the ladder: " + described(rung));
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.0, 1, Size::coarse) == 0.5 && ui::keyboard_step(spec, 0.95, 1, Size::coarse) == 0.99 && ui::keyboard_step(spec, 0.95, -1, Size::coarse) == 0.9, "between rungs the ladder goes to the neighbouring rung");

        // Fine steps move one unit in the last digit the text is worked to.
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.99999, 1, Size::fine) == 0.999991 && ui::keyboard_step(spec, 0.99999, -1, Size::fine) == 0.999989, "0.99999 steps to 0.999991 and 0.999989");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.123, 1, Size::fine) == 0.124 && ui::keyboard_step(spec, 0.123, -1, Size::fine) == 0.122, "0.123 steps to 0.124 and 0.122");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.5, 1, Size::fine) == 0.501 && ui::keyboard_step(spec, 0.5, -1, Size::fine) == 0.499, "0.5 steps to 0.501 and 0.499");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.1234567, 1, Size::fine) == 0.124 && ui::keyboard_step(spec, 0.1234567, -1, Size::fine) == 0.122, "a typed speed steps from the digits it shows");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.09999999, 1, Size::fine) == 0.1 && ui::keyboard_step(spec, 0.09999999, -1, Size::fine) == 0.0998, "a speed just below a power of ten steps from its four shown decimals");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.0, 1, Size::fine) == 0.001, "the fine step from rest is a thousandth of c");

        // The step back from finer digits keeps the coarser unit, so it undoes the step that came in.
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.999991, -1, Size::fine) == 0.99999 && ui::keyboard_step(spec, 0.9999911, -1, Size::fine) == 0.999991, "0.999991 steps back to 0.99999, and 0.9999911 to 0.999991");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.991, -1, Size::fine) == 0.99 && ui::keyboard_step(spec, 0.099, 1, Size::fine) == 0.1 && ui::keyboard_step(spec, 0.0099, 1, Size::fine) == 0.01, "steps back across a power of ten land on it");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.001, -1, Size::fine) == 0.0, "the fine step down from a thousandth of c is rest");
        RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 0.9999909, -1, Size::fine) == 0.9999908 && ui::keyboard_step(spec, 0.0991, -1, Size::fine) == 0.099, "a speed no step came from moves by its own last digit");

        // Alt+Up then Alt+Down, or the other way round, come back to where they started: from every
        // rung, from the powers of ten below ½ c, and after a walk of many digits.
        std::vector<double> starts(physics::speed_fraction_ladder.begin(), physics::speed_fraction_ladder.end());
        for (const auto edge : { 0.1, 0.01, 0.001, 0.0001 })
            starts.push_back(edge);
        for (const auto start : starts)
            for (const auto direction : { 1, -1 })
                for (const auto count : { 1, 2, 10, 150 })
                {
                    auto value = start;
                    int moved = 0;
                    for (; moved < count; ++moved)
                    {
                        const auto next = ui::keyboard_step(spec, value, direction, Size::fine);
                        if (next == value)
                            break;
                        value = next;
                    }
                    const auto turned = value;
                    for (int back = 0; back < moved; ++back)
                        value = ui::keyboard_step(spec, value, -direction, Size::fine);
                    if (value != start)
                        RIGIDBODIES_FAIL("fine steps retrace their way: " + described(start) + " went " + std::to_string(moved) + " steps to " + described(turned) + " and came back to " + described(value));
                }

        // Every step is clamped at the maximum, including from beyond it.
        for (const auto size : { Size::normal, Size::coarse, Size::fine })
        {
            RIGIDBODIES_EXPECT(ui::keyboard_step(spec, physics::maximum_speed_fraction, 1, size) == physics::maximum_speed_fraction, "no step passes the maximum");
            RIGIDBODIES_EXPECT(ui::keyboard_step(spec, 1.0, 1, size) == physics::maximum_speed_fraction && ui::keyboard_step(spec, -0.5, -1, size) == 0.0, "a value outside the range is clamped first");
            RIGIDBODIES_EXPECT(ui::scrub_value(spec, 0.99999, 400, size) <= physics::maximum_speed_fraction, "a long scrub stops at the maximum");
        }

        // A scrub is that many keyboard steps.
        RIGIDBODIES_EXPECT(ui::scrub_value(spec, 0.0, 3, Size::coarse) == 0.99 && ui::scrub_value(spec, 0.9, -2, Size::coarse) == 0.0, "a coarse scrub climbs the ladder one rung per step");
        auto stepped = 0.3;
        for (int step = 0; step < 7; ++step)
            stepped = ui::keyboard_step(spec, stepped, 1, Size::normal);
        RIGIDBODIES_EXPECT(ui::scrub_value(spec, 0.3, 7, Size::normal) == stepped, "a normal scrub of seven steps equals seven arrow presses");
        RIGIDBODIES_EXPECT(ui::scrub_value(spec, 0.99999, -2, Size::fine) == ui::keyboard_step(spec, 0.999989, -1, Size::fine), "a fine scrub repeats the fine step");
        RIGIDBODIES_EXPECT(ui::scrub_value(spec, 0.42, 0, Size::normal) == 0.42 && ui::rounded_value(spec, 0.42) == 0.42, "no scrub and no rounding leave the speed alone");
    }

    RIGIDBODIES_TEST("a speed is rest or at least the slowest moving speed")
    {
        const auto& registered = ui::find_control_spec("world.relativity.speed")->number;
        RIGIDBODIES_EXPECT(registered.smallest_nonzero == physics::minimum_moving_speed_fraction, "Probe speed has the slowest moving speed as its floor");
        for (const auto accepted : { 0.0, physics::minimum_moving_speed_fraction, 1.0e-9, 0.5, physics::maximum_speed_fraction })
            RIGIDBODIES_EXPECT(ui::accepts_value(registered, accepted), "rest, the floor and the range above it are accepted: " + described(accepted));
        for (const auto refused : { 1.0e-13, 1.0e-300, std::numeric_limits<double>::denorm_min(), -1.0e-12, 1.0, std::numeric_limits<double>::quiet_NaN() })
            RIGIDBODIES_EXPECT(!ui::accepts_value(registered, refused), "a speed below the floor or out of range is refused: " + described(refused));
        // From rest, the floor is a change, and so is rest from the floor, though they are closer
        // than a millionth of a millionth; a rounding error is not.
        RIGIDBODIES_EXPECT(ui::differs_from_shown(registered, physics::minimum_moving_speed_fraction, 0.0) && ui::differs_from_shown(registered, 0.0, physics::minimum_moving_speed_fraction), "rest and the floor differ");
        RIGIDBODIES_EXPECT(!ui::differs_from_shown(registered, 99.999 / 100.0, 0.99999) && ui::differs_from_shown(registered, 0.999991, 0.99999), "a rounding error is no change, a digit is");
        const auto& gravity = ui::find_control_spec("world.gravity.strength")->number;
        RIGIDBODIES_EXPECT(gravity.smallest_nonzero == 0.0 && ui::accepts_value(gravity, 1.0e-13) && !ui::differs_from_shown(gravity, 1.0e-13, 0.0) && ui::differs_from_shown(gravity, 0.01, 0.0), "other numbers keep their range and their absolute rounding allowance");
        // The fine step below the floor goes to rest, so Alt+Down never lands on a refused speed.
        using Size = ui::StepSize;
        RIGIDBODIES_EXPECT(ui::keyboard_step(registered, physics::minimum_moving_speed_fraction, -1, Size::fine) == 0.0, "the fine step down from the floor is rest");
        const auto up = ui::keyboard_step(registered, physics::minimum_moving_speed_fraction, 1, Size::fine);
        RIGIDBODIES_EXPECT(up > physics::minimum_moving_speed_fraction && ui::accepts_value(registered, up), "the fine step up from the floor moves above it");
    }

    RIGIDBODIES_TEST("linear and logarithmic helpers equal the old formulas bit for bit")
    {
        std::size_t checked = 0;
        for (const auto& spec : ui::control_specs())
        {
            if (spec.kind != ui::ControlKind::number || spec.number.scale == ui::NumberScale::rapidity)
                continue;
            const auto& number = spec.number;
            const auto logarithmic = number.scale == ui::NumberScale::logarithmic;
            const std::string key(spec.key);
            ++checked;

            // The slider input, its detents and the slider sync before the helpers.
            for (int index = 0; index <= 1000; ++index)
            {
                const auto position = std::clamp(static_cast<double>(index), 0.0, 1000.0) / 1000.0;
                const auto old_value = logarithmic
                    ? number.soft_minimum * std::pow(number.soft_maximum / number.soft_minimum, position)
                    : number.soft_minimum + (number.soft_maximum - number.soft_minimum) * position;
                if (ui::slider_value(number, position) != old_value)
                    RIGIDBODIES_FAIL("slider value matches the old mapping: " + key);
            }
            for (const auto detent : number.detents)
            {
                const auto old_position = logarithmic
                    ? std::log(detent / number.soft_minimum) / std::log(number.soft_maximum / number.soft_minimum)
                    : (detent - number.soft_minimum) / (number.soft_maximum - number.soft_minimum);
                RIGIDBODIES_EXPECT(ui::slider_position(number, detent) == old_position, "detent position matches the old mapping: " + key);
            }
            std::vector<double> values { number.minimum, number.maximum, number.soft_minimum, number.soft_maximum };
            for (int index = 1; index < 32; ++index)
                values.push_back(number.minimum + (number.maximum - number.minimum) * index / 32.0);
            for (int index = 1; index < 32; ++index)
                values.push_back(number.soft_minimum + (number.soft_maximum - number.soft_minimum) * index / 32.0);
            for (const auto value : values)
            {
                const auto clamped = std::clamp(value, number.soft_minimum, number.soft_maximum);
                const auto raw_position = logarithmic ? std::log(clamped / number.soft_minimum) / std::log(number.soft_maximum / number.soft_minimum) : (clamped - number.soft_minimum) / (number.soft_maximum - number.soft_minimum);
                if (ui::slider_position(number, value) != std::clamp(raw_position, 0.0, 1.0))
                    RIGIDBODIES_FAIL("slider sync matches the old mapping: " + key);

                // The field arrows before the helpers.
                for (const auto shift : { false, true })
                    for (const auto alt : { false, true })
                        for (const auto up : { false, true })
                        {
                            const auto size = shift ? ui::StepSize::coarse : alt ? ui::StepSize::fine
                                                                                 : ui::StepSize::normal;
                            const auto direction = up ? 1.0 : -1.0;
                            const auto factor = shift ? number.coarse : alt ? number.fine
                                                                            : number.step;
                            auto old_value = logarithmic ? value * (direction > 0 ? factor : 1.0 / factor) : value + direction * number.step * (shift ? number.coarse : alt ? number.fine
                                                                                                                                                                            : 1.0);
                            old_value = std::clamp(old_value, number.minimum, number.maximum);
                            if (ui::keyboard_step(number, value, up ? 1 : -1, size) != old_value)
                                RIGIDBODIES_FAIL("arrow step matches the old formula: " + key);
                        }

                // The label scrub before the helpers, whose rounding never left the range here.
                for (const auto steps : { -40, -7, -1, 0, 1, 3, 25 })
                    for (const auto size : { ui::StepSize::normal, ui::StepSize::coarse, ui::StepSize::fine })
                    {
                        double old_value = value;
                        if (logarithmic)
                        {
                            const auto factor = size == ui::StepSize::coarse ? number.coarse : size == ui::StepSize::fine ? number.fine
                                                                                                                          : number.step;
                            old_value *= std::pow(factor, steps);
                        }
                        else
                        {
                            const auto multiplier = size == ui::StepSize::coarse ? number.coarse : size == ui::StepSize::fine ? number.fine
                                                                                                                              : 1.0;
                            old_value += steps * number.step * multiplier;
                        }
                        old_value = std::clamp(old_value, number.minimum, number.maximum);
                        if (number.decimals >= 0)
                        {
                            const auto precision = std::pow(10.0, number.decimals);
                            old_value = std::round(old_value * precision) / precision;
                        }
                        const auto scrubbed = ui::rounded_value(number, ui::scrub_value(number, value, steps, size));
                        if (scrubbed != old_value && old_value >= number.minimum && old_value <= number.maximum)
                            RIGIDBODIES_FAIL("scrub matches the old formula: " + key);
                        if (scrubbed < number.minimum || scrubbed > number.maximum)
                            RIGIDBODIES_FAIL("a rounded scrub stays inside the range: " + key);
                    }
            }
        }
        RIGIDBODIES_EXPECT(checked > 20, "the registry's number controls were all compared");
    }

    RIGIDBODIES_TEST("rounding a slider or scrub value never leaves the range")
    {
        ui::NumberSpec spec;
        spec.minimum = 0.05;
        spec.maximum = 0.95;
        spec.soft_minimum = 0.05;
        spec.soft_maximum = 0.95;
        spec.step = 0.01;
        spec.decimals = 0;
        RIGIDBODIES_EXPECT(ui::rounded_value(spec, 0.94) == 0.95 && ui::rounded_value(spec, 0.06) == 0.05, "a value rounded past a limit is clamped back to it");
        RIGIDBODIES_EXPECT(ui::rounded_value(spec, ui::slider_value(spec, 1.0)) == 0.95, "the slider's end stays inside the field range");
        spec.decimals = 1;
        RIGIDBODIES_EXPECT(ui::rounded_value(spec, 0.5) == 0.5 && ui::rounded_value(spec, 0.44) == 0.4, "values inside the range round as before");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
