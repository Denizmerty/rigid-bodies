#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/special_relativity.hpp>
#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/icons.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace rigidbodies::ui
{
    namespace
    {
        using Q = core::DisplayQuantity;

        // U+00A0, which keeps a number with the unit or symbol after it.
        constexpr std::string_view nbsp = "\xC2\xA0";

        const ControlSpec& spec(std::string_view key)
        {
            return *find_control_spec(key);
        }

        UiCommand state_choice(std::string_view key)
        {
            UiCommand result;
            result.detail = "state-id:" + std::string(key);
            return result;
        }

        bool starts_with(std::string_view text, std::string_view prefix)
        {
            return text.substr(0, prefix.size()) == prefix;
        }

        // One curve of the Relativity plot: the relativistic value (solid) against what Newton's
        // mechanics predicts (dashed), in units of the probe's own mc² or mc. Each keeps a fixed
        // range, ticked every `full_step` up to c, so a value that leaves it is shown off the chart
        // rather than rescaling the axis.
        struct RelativityCurve
        {
            std::string_view id, label, unit;
            std::uint8_t slot;
            double full_maximum, full_step, near_minimum, near_maximum;
        };

        constexpr RelativityCurve relativity_curves[] {
            { "energy", "Kinetic energy", "mc\xC2\xB2", 1, 5.0, 1.0, 1.0e-3, 1.0e4 },
            { "momentum", "Momentum", "mc", 2, 10.0, 2.0, 1.0e-2, 1.0e4 },
            { "gamma", "Lorentz factor \xCE\xB3", "", 0, 10.0, 2.0, 0.5, 1.0e4 },
            { "clock_rate", "Probe clock rate", "", 0, 1.05, 0.25, 1.0e-4, 2.0 },
        };

        // A tick value with no trailing zeros: "0", "0.25", "2".
        std::string tick_text(double value)
        {
            auto text = core::fixed(value, 2);
            while (text.back() == '0')
                text.pop_back();
            if (text.back() == '.')
                text.pop_back();
            return text;
        }

        const RelativityCurve& relativity_curve(std::string_view id)
        {
            for (const auto& curve : relativity_curves)
                if (curve.id == id)
                    return curve;
            return relativity_curves[0];
        }

        double solid_value(std::string_view curve, const physics::LorentzFactors& factors)
        {
            if (curve == "momentum")
                return factors.speed_fraction_times_lorentz_factor;
            if (curve == "gamma")
                return factors.lorentz_factor;
            if (curve == "clock_rate")
                return factors.inverse_lorentz_factor;
            return factors.lorentz_factor_minus_one;
        }

        // Newton's mechanics: ½mv², mv, and clocks that never slow (γ = 1).
        double dashed_value(std::string_view curve, const physics::LorentzFactors& factors)
        {
            if (curve == "momentum")
                return factors.speed_fraction;
            if (curve == "gamma" || curve == "clock_rate")
                return 1.0;
            return 0.5 * factors.speed_fraction * factors.speed_fraction;
        }

        std::string speed_text(double speed_fraction, core::DisplayUnits units)
        {
            return core::format_quantity(speed_fraction, Q::speed_fraction, units);
        }

        // The muted remark above the plot: what the two curves are, or how far off the chart the
        // probe has gone, and near c why the point only looks like c.
        std::string relativity_note(const RelativityModel& relativity, const RelativityCurve& curve, bool near, core::DisplayUnits units)
        {
            if (near)
                return curve.id == "clock_rate" ? "Each step right adds a 9 to the speed, and the probe's clock slows about 3 times per step without ever stopping."
                                                : "Each step right adds a 9 to the speed. \xCE\xB3, energy and momentum grow about 3 times per step, and c would need infinitely many steps.";
            const auto factors = physics::lorentz_factors(relativity.speed_fraction);
            const auto value = factors ? solid_value(curve.id, *factors) : 0.0;
            std::string note;
            if (curve.id != "clock_rate" && value > curve.full_maximum)
            {
                const auto reading = curve.id == "energy" ? "K = " + core::format_significant(value) + std::string(nbsp) + "mc\xC2\xB2"
                    : curve.id == "momentum"              ? "p = " + core::format_significant(value) + std::string(nbsp) + "mc"
                                                          : "\xCE\xB3 = " + core::format_value(relativity.lorentz_factor_minus_one, Q::lorentz_factor_excess, units);
                note = "Off the chart: " + reading + ". Choose Near c to follow it.";
            }
            else if (curve.id == "energy")
                note = "Dashed: Newton's \xC2\xBDmv\xC2\xB2, which never passes \xC2\xBDmc\xC2\xB2. Solid: (\xCE\xB3 \xE2\x88\x92 1)mc\xC2\xB2, which grows without limit as v approaches c. For this probe, mc\xC2\xB2 is " +
                    core::format_quantity(relativity.rest_energy_j, Q::relativistic_energy, units) + ".";
            else if (curve.id == "momentum")
                note = "Dashed: Newton's mv, which would reach mc at c. Solid: \xCE\xB3mv, which grows without limit. For this probe, mc is " +
                    core::format_quantity(relativity.rest_mass_kg * core::speed_of_light_m_s, Q::relativistic_momentum, units) + ".";
            else if (curve.id == "gamma")
                note = "\xCE\xB3 stays close to 1 until about half the speed of light, then rises without limit.";
            else
                note = "How far the probe's clock moves for each unit of lab time. It falls towards zero but never reaches it.";
            // On a scale that ends at c, the last few nines are less than a pixel apart.
            if (relativity.speed_fraction >= 0.999)
                note += " At this scale " + speed_text(relativity.speed_fraction, units) + " looks like c, but it is still " + core::format_quantity(relativity.below_light_m_s, Q::speed_gap, units) + " below c.";
            return note;
        }
    }

    std::string now_text(const UiModel& model)
    {
        if (model.relativity)
            return core::format_quantity(model.relativity->lab_time_s, Q::fine_time, model.display_units);
        return core::format_quantity(model.elapsed_time_s, Q::time, model.display_units);
    }

    std::string elapsed_text(const UiModel& model, double world_seconds)
    {
        // A world second is a lab nanosecond, so the digits stay the same and only the unit changes.
        if (model.relativity)
            return core::format_quantity(world_seconds * model.relativity->lab_seconds_per_world_second, Q::fine_time, model.display_units);
        return core::format_quantity(world_seconds, Q::time, model.display_units);
    }

    std::string_view relativity_preset_id(double speed_fraction)
    {
        const auto* spec = find_control_spec("world.relativity.preset");
        if (!spec)
            return {};
        // The ids are the presets' own decimal text, so reading one back gives the value its chip
        // sends. A value a rounding error from a preset, as a hand-written file may hold, is that
        // preset's rung; one merely close to it selects nothing.
        const auto rung = physics::speed_fraction_on_ladder(speed_fraction);
        for (const auto& option : spec->options)
            if (std::strtod(std::string(option.id).c_str(), nullptr) == rung)
                return option.id;
        return {};
    }

    void relativity_speed_rows(const UiModel& model, PanelBuilder& builder, std::string_view instance)
    {
        if (!model.relativity)
            return;
        const auto speed_fraction = model.relativity->speed_fraction;
        UiCommand speed;
        speed.kind = UiCommandKind::set_relativity_speed;
        builder.number_row(spec("world.relativity.speed"), speed_fraction, speed, instance);
        // The chips' template carries the current speed, so activating the row itself changes nothing.
        speed.value = speed_fraction;
        builder.segmented_row(spec("world.relativity.preset"), relativity_preset_id(speed_fraction), speed);
        if (!instance.empty())
            builder.instance_last(instance);
    }

    void relativity_world_section(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.relativity || !builder.section("world.relativity", "Relativity", true))
            return;
        const auto& relativity = *model.relativity;
        relativity_speed_rows(model, builder, {});
        builder.readout("world.relativity.rest_mass", "Rest mass m", core::format_quantity(relativity.rest_mass_kg, Q::mass, model.display_units));
        builder.readout("world.relativity.light_speed", "Speed of light c", core::format_quantity(core::speed_of_light_m_s, Q::relativistic_speed, model.display_units));
        if (physics::speed_fraction_on_ladder(relativity.speed_fraction) >= relativity.maximum_speed_fraction)
            builder.notice("world.relativity.limit", Severity::info, "This is the fastest speed here. Going faster would need ever more energy, and c itself is out of reach.");
        builder.paragraph("Equal moves of the slider are equal boosts. Up and Down nudge the speed; Shift+Up and Shift+Down jump between the presets. The probe can get as close to c as you like but never reach it: that would take infinite energy.");
    }

    bool control_available(const UiModel& model, std::string_view key)
    {
        const auto relativity = model.relativity.has_value();
        if (starts_with(key, "world.relativity.") || starts_with(key, "measure.relativity.") || key == "measure.graph.clocks")
            return relativity;
        if (!relativity)
            return true;
        // Settings of Newtonian objects and of the World they move in have nothing to act on here.
        static constexpr std::string_view newtonian_prefixes[] { "object.", "joint.", "spring.", "draw.", "world.gravity.", "world.air.", "world.collisions.", "world.advanced.", "world.objects.", "world.markers.", "world.statistics.", "measure.energy.", "measure.collisions.", "measure.theory.", "show.arrows." };
        static constexpr std::string_view newtonian_keys[] { "measure.graph.scope", "measure.graph.quantities", "measure.runs.add_object", "tools.mode", "library.top.import", "camera.frame.selection" };
        if (std::any_of(std::begin(newtonian_prefixes), std::end(newtonian_prefixes), [&](std::string_view prefix)
                {
                    return starts_with(key, prefix);
                }))
            return false;
        UiCommand import;
        import.kind = UiCommandKind::import_shape;
        return std::find(std::begin(newtonian_keys), std::end(newtonian_keys), key) == std::end(newtonian_keys) && key != control_key(import);
    }

    void MeasurePanel::build_relativity_tab(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.relativity)
            return;
        const auto& relativity = *model.relativity;
        const auto units = model.display_units;
        const auto curve_id = std::string(builder.view_value("measure.relativity.curve", "energy"));
        const auto range_id = std::string(builder.view_value("measure.relativity.range", "full"));
        const auto& curve = relativity_curve(curve_id);
        const auto near = range_id == "near";

        // The curves depend only on the curve and the range, so they are sampled once per choice:
        // a drag moves the operating point and the note, never the samples.
        const auto samples_key = std::string(curve.id) + (near ? "|near" : "|full");
        if (samples_key != relativity_samples_key_)
        {
            relativity_samples_key_ = samples_key;
            relativity_x_.clear();
            relativity_solid_.clear();
            relativity_dashed_.clear();
            const auto add = [&](float x, const physics::LorentzFactors& factors)
            {
                // Speeds closer to c than a float can tell apart share one sample: the closest.
                if (!relativity_x_.empty() && !(x > relativity_x_.back()))
                {
                    relativity_x_.pop_back();
                    relativity_solid_.pop_back();
                    relativity_dashed_.pop_back();
                }
                relativity_x_.push_back(x);
                relativity_solid_.push_back(static_cast<float>(solid_value(curve.id, factors)));
                relativity_dashed_.push_back(static_cast<float>(dashed_value(curve.id, factors)));
            };
            if (near)
            {
                // Rapidity is the axis: equal steps right are equal boosts, and each nine of the
                // speed is the same distance further on.
                constexpr int count = 400;
                const auto maximum = physics::maximum_rapidity();
                for (int index = 0; index <= count; ++index)
                {
                    const auto rapidity = maximum * index / count;
                    if (const auto factors = physics::lorentz_factors_from_rapidity(rapidity))
                        add(static_cast<float>(rapidity), *factors);
                }
            }
            else
            {
                // Evenly to 0.9 c, then evenly in the number of nines up to seven, from the gap
                // 1 − β so the values keep their digits however close the speed is to c. Each
                // sample is worked out at the speed its axis value stores, which near c is a float
                // up to 6 × 10⁻⁸ from the gap asked for, so a read-out pairs a speed with its own γ.
                for (int index = 0; index <= 180; ++index)
                    if (const auto factors = physics::lorentz_factors(index / 200.0))
                        add(static_cast<float>(factors->speed_fraction), *factors);
                for (int index = 1; index <= 240; ++index)
                {
                    const auto x = static_cast<float>(1.0 - std::pow(10.0, -(1.0 + index / 40.0)));
                    if (const auto factors = physics::lorentz_factors_from_gap(1.0 - static_cast<double>(x)))
                        add(x, *factors);
                }
            }
        }

        relativity_plot_ = {};
        relativity_plot_.series.push_back({ std::string(curve.label), relativity_solid_.data(), relativity_x_.data(), relativity_x_.size(), SeriesStyle::solid, curve.slot, "Relativity", std::string(curve.unit) });
        relativity_plot_.series.push_back({ std::string(curve.label), relativity_dashed_.data(), relativity_x_.data(), relativity_x_.size(), SeriesStyle::dashed, curve.slot, "Newton", std::string(curve.unit) });
        relativity_plot_.y = { std::string(curve.label), std::string(curve.unit), near ? curve.near_minimum : 0.0, near ? curve.near_maximum : curve.full_maximum, near ? AxisScale::log10 : AxisScale::linear };
        // Named ticks keep the linear range as given at every plot size; a log range keeps its decades.
        if (!near)
            for (int index = 0; index * curve.full_step <= curve.full_maximum + 1.0e-9; ++index)
                relativity_plot_.y.ticks.push_back({ index * curve.full_step, tick_text(index * curve.full_step) });
        std::vector<AxisTick> ticks;
        if (near)
        {
            // c is not on this axis: there is always another nine to the right.
            for (const auto tick : { 0.0, 0.9, 0.99, 0.999, 0.9999, 0.99999, 0.999999, 0.9999999 })
                ticks.push_back({ physics::rapidity_from_speed_fraction(tick), tick == 0.0 ? std::string("0") : speed_text(tick, units) });
            relativity_plot_.x = { "Speed (each step adds a 9)", "", 0.0, physics::maximum_rapidity(), AxisScale::linear, std::move(ticks), "v/c", AxisReadout::rapidity_speed_fraction };
            relativity_plot_.operating_x = relativity.rapidity;
        }
        else
        {
            for (const auto tick : { 0.0, 0.2, 0.4, 0.6, 0.8 })
                ticks.push_back({ tick, tick == 0.0 ? std::string("0") : speed_text(tick, units) });
            relativity_plot_.x = { "Speed", "", 0.0, 1.0, AxisScale::linear, std::move(ticks), "v/c", AxisReadout::speed_fraction };
            relativity_plot_.markers.push_back({ 1.0, "c", PlotMarkerKind::limit });
            relativity_plot_.operating_x = relativity.speed_fraction;
        }
        relativity_plot_.subject = "Probe, rest mass " + core::format_quantity(relativity.rest_mass_kg, Q::mass, units);
        const auto note = relativity_note(relativity, curve, near, units);

        // As on the Graph tab: a wide drawer sets the controls and readings in a column beside the
        // plot; otherwise the plot leads and they follow below it.
        const auto beside = builder.view_region_width(RegionId::measure_drawer) >= 720.0;
        const auto show_plot = [&]
        {
            builder.begin_group(beside ? "graph_main" : "");
            builder.label(note);
            builder.plot("measure.relativity.plot", relativity_plot_);
            builder.end_group();
        };
        if (!beside)
            show_plot();
        builder.begin_group(beside ? "graph_side" : "");
        relativity_speed_rows(model, builder, "measure");
        builder.segmented_row(spec("measure.relativity.curve"), curve.id, state_choice("measure.relativity.curve"));
        builder.segmented_row(spec("measure.relativity.range"), near ? "near" : "full", state_choice("measure.relativity.range"));

        // Readings that follow the speed change with an edit, in the same frame as the plot's
        // point and the stage, so they are not live: they flash when the speed changes.
        builder.heading("At this speed");
        const auto reading = [&](std::string_view key, std::string_view label, const std::string& text)
        {
            builder.readout(key, label, text);
            builder.present_last(presentation(icons::measure));
        };
        reading("measure.relativity.fraction", "Speed v/c", core::format_value(relativity.speed_fraction, Q::speed_fraction, units));
        reading("measure.relativity.speed", "Speed v", core::format_quantity(relativity.speed_m_s, Q::relativistic_speed, units));
        reading("measure.relativity.below_c", "Below c by", core::format_quantity(relativity.below_light_m_s, Q::speed_gap, units));
        reading("measure.relativity.gamma", "Lorentz factor \xCE\xB3", core::format_value(relativity.lorentz_factor_minus_one, Q::lorentz_factor_excess, units));
        reading("measure.relativity.energy", "Kinetic energy (\xCE\xB3 \xE2\x88\x92 1)mc\xC2\xB2", core::format_quantity(relativity.kinetic_energy_j, Q::relativistic_energy, units));
        reading("measure.relativity.newton_energy", "Newton's \xC2\xBDmv\xC2\xB2", core::format_quantity(relativity.newtonian_kinetic_energy_j, Q::relativistic_energy, units));
        reading("measure.relativity.energy_ratio", "Kinetic energy \xC3\xB7 \xC2\xBDmv\xC2\xB2", relativity.newtonian_kinetic_energy_j > 0.0 ? core::format_significant(relativity.kinetic_energy_j / relativity.newtonian_kinetic_energy_j) + "\xC3\x97" : std::string("\xE2\x80\x94"));
        reading("measure.relativity.momentum", "Momentum \xCE\xB3mv", core::format_quantity(relativity.momentum_kg_m_s, Q::relativistic_momentum, units));
        reading("measure.relativity.newton_momentum", "Newton's mv", core::format_quantity(relativity.newtonian_momentum_kg_m_s, Q::relativistic_momentum, units));
        reading("measure.relativity.rest_mass", "Rest mass m", core::format_quantity(relativity.rest_mass_kg, Q::mass, units));
        reading("measure.relativity.rest_energy", "Rest energy mc\xC2\xB2", core::format_quantity(relativity.rest_energy_j, Q::relativistic_energy, units));

        // The clocks run on their own, so they refresh with the other live readings.
        builder.heading("Clocks");
        builder.readout("measure.relativity.lab_clock", "Lab clock t", core::format_quantity(relativity.lab_time_s, Q::fine_time, units), RowTone::normal, true);
        builder.present_last(presentation(icons::timer));
        builder.readout("measure.relativity.probe_clock", "Probe clock \xCF\x84", core::format_quantity(relativity.proper_time_s, Q::fine_time, units), RowTone::normal, true);
        builder.present_last(presentation(icons::timer));
        builder.readout("measure.relativity.clock_gap", "Lab \xE2\x88\x92 probe t \xE2\x88\x92 \xCF\x84", core::format_quantity(relativity.clock_lag_s, Q::duration, units), RowTone::normal, true);
        builder.present_last(presentation(icons::timer).primary());
        reading("measure.relativity.clock_rate", "Probe clock rate 1/\xCE\xB3", core::format_significant(relativity.inverse_lorentz_factor));
        builder.label("\xCE\xB3 = 1/\xE2\x88\x9A(1 \xE2\x88\x92 v\xC2\xB2/c\xC2\xB2) \xC2\xB7 K = (\xCE\xB3 \xE2\x88\x92 1)mc\xC2\xB2 \xC2\xB7 p = \xCE\xB3mv. Newton's \xC2\xBDmv\xC2\xB2 is within 1" + std::string(nbsp) + "% of the true kinetic energy only below 0.115" + std::string(nbsp) +
            "c.");
        builder.end_group();
        if (beside)
            show_plot();
    }
}
