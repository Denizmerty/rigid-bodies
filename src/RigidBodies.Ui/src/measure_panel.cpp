#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/physics/education_accounting.hpp>
#include <rigidbodies/ui/run_compare.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>
#include <optional>

namespace rigidbodies::ui
{
    namespace
    {
        constexpr std::size_t scene_stride = 8;
        constexpr std::size_t ledger_stride = 6;
        constexpr std::size_t object_stride = 14;

        UiCommand action(UiCommandKind kind)
        {
            UiCommand result;
            result.kind = kind;
            return result;
        }
        UiCommand toggle(UiCommandKind kind, bool value)
        {
            auto result = action(kind);
            result.flag = value;
            return result;
        }
        const ControlSpec& spec(std::string_view key)
        {
            return *find_control_spec(key);
        }
        UiCommand state_choice(std::string_view key)
        {
            auto result = action(UiCommandKind::none);
            result.detail = "state-id:" + std::string(key);
            return result;
        }
        UiCommand state_value(std::string_view key, std::string_view value)
        {
            auto result = action(UiCommandKind::none);
            result.detail = "state:" + std::string(key) + "=" + std::string(value);
            return result;
        }
        const RunRecord* run_number(math::Span<const RunRecord> runs, std::string_view text)
        {
            int number {};
            const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), number);
            if (error != std::errc {} || end != text.data() + text.size())
                return nullptr;
            const auto found = std::find_if(runs.begin(), runs.end(), [&](const auto& run)
                {
                    return run.number == number;
                });
            return found == runs.end() ? nullptr : &*found;
        }
        std::string body_key(physics::BodyId id)
        {
            return std::to_string(id.index) + ":" + std::to_string(id.generation);
        }

        std::optional<std::size_t> scene_channel(std::string_view key)
        {
            if (key == "kinetic_moving")
                return 0;
            if (key == "kinetic_spinning")
                return 1;
            if (key == "potential_height")
                return 2;
            if (key == "potential_springs")
                return 3;
            if (key == "lost_impacts")
                return 4;
            if (key == "lost_friction")
                return 5;
            if (key == "momentum_x")
                return 6;
            if (key == "momentum_y")
                return 7;
            return {};
        }
        // The rest of the energy budget, recorded for the whole scene as RunSeries::ledger.
        struct LedgerQuantity
        {
            std::string_view key, label;
        };
        constexpr LedgerQuantity ledger_quantities[] { { "lost_air", "Lost to air" }, { "lost_dampers", "Lost in dampers" }, { "lost_joints", "Lost in joints" }, { "added_drives", "Added by motors and drives" }, { "added_forces", "Added by applied forces" }, { "added_changes", "Added by your changes" } };
        std::optional<std::size_t> ledger_channel(std::string_view key)
        {
            for (std::size_t index = 0; index < std::size(ledger_quantities); ++index)
                if (ledger_quantities[index].key == key)
                    return index;
            return {};
        }
        std::optional<std::size_t> object_channel(std::string_view key)
        {
            if (key == "kinetic_moving")
                return 0;
            if (key == "kinetic_spinning")
                return 1;
            if (key == "potential_height")
                return 2;
            if (key == "potential_springs")
                return 3;
            if (key == "mechanical")
                return 4;
            if (key == "momentum_x")
                return 5;
            if (key == "momentum_y")
                return 6;
            if (key == "speed")
                return 7;
            if (key == "velocity_x")
                return 8;
            if (key == "velocity_y")
                return 9;
            if (key == "height")
                return 10;
            if (key == "spin")
                return 11;
            if (key == "position_x")
                return 12;
            if (key == "rotation")
                return 13;
            return {};
        }
        std::string quantity_label(std::string_view key)
        {
            if (key == "kinetic_moving")
                return "Kinetic (moving)";
            if (key == "kinetic_spinning")
                return "Kinetic (spinning)";
            if (key == "potential_height")
                return "Potential (height)";
            if (key == "potential_springs")
                return "Potential (springs)";
            if (key == "lost_impacts")
                return "Lost in impacts";
            if (key == "lost_friction")
                return "Lost to friction";
            if (key == "momentum_x")
                return "Momentum x";
            if (key == "momentum_y")
                return "Momentum y";
            if (key == "speed")
                return "Speed";
            if (const auto channel = ledger_channel(key))
                return std::string(ledger_quantities[*channel].label);
            return "Mechanical energy";
        }
        std::string quantity_unit(std::string_view key)
        {
            if (key == "momentum_x" || key == "momentum_y")
                return "kg·m/s";
            if (key == "speed")
                return "m/s";
            return "J";
        }
        // Energy kinds keep one colour everywhere: kinetic is series 1, lost series 2 and potential
        // series 3, as on the Energy tab's share bars. Other quantities take the colours left.
        std::optional<std::uint8_t> energy_kind_slot(std::string_view key)
        {
            if (key.rfind("kinetic", 0) == 0)
                return std::uint8_t { 0 };
            if (key.rfind("lost", 0) == 0)
                return std::uint8_t { 1 };
            if (key.rfind("potential", 0) == 0)
                return std::uint8_t { 2 };
            return {};
        }
        std::vector<std::uint8_t> series_slots(const std::vector<std::string>& keys)
        {
            constexpr std::uint8_t unassigned = 0xff;
            std::vector<std::uint8_t> slots(keys.size(), unassigned);
            bool used[3] {};
            for (std::size_t index = 0; index < keys.size(); ++index)
                if (const auto slot = energy_kind_slot(keys[index]); slot && !used[*slot])
                {
                    slots[index] = *slot;
                    used[*slot] = true;
                }
            for (auto& slot : slots)
                if (slot == unassigned)
                    for (std::uint8_t candidate = 0; candidate < 3 && slot == unassigned; ++candidate)
                        if (!used[candidate])
                        {
                            slot = candidate;
                            used[candidate] = true;
                        }
            for (std::size_t index = 0; index < slots.size(); ++index)
                if (slots[index] == unassigned)
                    slots[index] = static_cast<std::uint8_t>(index % 3);
            return slots;
        }

        // Fixed decimals with a true minus sign and no negative zero.
        std::string signed_fixed(double value, int decimals)
        {
            auto text = core::fixed(value, decimals);
            if (!text.empty() && text.front() == '-')
            {
                if (text.find_first_not_of("0.", 1) == std::string::npos)
                    text.erase(0, 1);
                else
                    text.replace(0, 1, "\xE2\x88\x92");
            }
            return text;
        }

        // Rows that are read together, such as parts that add up to a total, share one unit and
        // one number of decimals so they can be compared and summed by eye. The decimals change
        // only at wide steps (two below 100, one below 1000), so figures keep their precision as
        // a run goes on and zero always reads the same. Momentum stays in kg·m/s, where solver
        // noise rounds to zero. Units other than SI already use one unit per quantity.
        struct ColumnFormat
        {
            core::DisplayQuantity quantity {};
            core::DisplayUnits units {};
            double factor { 1.0 };
            int decimals { -1 };
            std::string_view unit;
        };
        template <typename Values>
        ColumnFormat column_format(const Values& values, core::DisplayQuantity quantity, core::DisplayUnits units)
        {
            ColumnFormat result { quantity, units };
            const auto energy = quantity == core::DisplayQuantity::energy;
            if (units != core::DisplayUnits::si || (!energy && quantity != core::DisplayQuantity::momentum))
                return result;
            double largest = 0.0;
            for (const auto value : values)
                if (std::isfinite(value))
                    largest = std::max(largest, std::abs(value));
            result.unit = energy ? "J" : "kg\xC2\xB7m/s";
            if (energy && largest >= 10000.0)
            {
                result.factor = 1.0e-3;
                result.unit = "kJ";
            }
            else if (energy && largest >= 1.0e-6 && largest < 0.01)
            {
                result.factor = 1.0e3;
                result.unit = "mJ";
            }
            const auto scaled = largest * result.factor;
            if (scaled >= 1000.0)
                result.decimals = 0;
            else if (scaled >= 100.0)
                result.decimals = 1;
            // Below what three decimals can show, a column reads as zero in the usual form.
            else if (scaled >= 0.1 || scaled < (energy ? 1.0e-6 : 5.0e-4))
                result.decimals = 2;
            else
                result.decimals = 3;
            return result;
        }
        ColumnFormat column_format(std::initializer_list<double> values, core::DisplayQuantity quantity, core::DisplayUnits units)
        {
            return column_format<std::initializer_list<double>>(values, quantity, units);
        }
        std::string in_column(const ColumnFormat& format, double value)
        {
            if (!std::isfinite(value))
                return "\xE2\x80\x94";
            if (format.decimals < 0)
                return core::format_quantity(value, format.quantity, format.units);
            return signed_fixed(value * format.factor, format.decimals) + "\xC2\xA0" + std::string(format.unit);
        }

        // Whole percentages that add up to exactly 100, by largest remainder.
        template <std::size_t Count>
        std::array<int, Count> whole_percentages(const std::array<double, Count>& parts, std::size_t count)
        {
            std::array<int, Count> result {};
            double total = 0.0;
            for (std::size_t index = 0; index < count; ++index)
                total += parts[index];
            if (!(total > 0.0))
                return result;
            std::array<double, Count> remainders {};
            remainders.fill(-1.0);
            int assigned = 0;
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto exact = parts[index] / total * 100.0;
                result[index] = static_cast<int>(std::floor(exact));
                remainders[index] = exact - result[index];
                assigned += result[index];
            }
            for (; assigned < 100; ++assigned)
            {
                const auto largest = static_cast<std::size_t>(std::distance(remainders.begin(), std::max_element(remainders.begin(), remainders.end())));
                ++result[largest];
                remainders[largest] = -1.0;
            }
            return result;
        }

        // Height energy the measured objects would keep resting on the first solid static surface
        // beneath them, along each object's own gravity. This is the floor a fall converts its
        // potential energy down to, known before anything has moved. An object with nothing
        // solid beneath it (a pendulum, a free flight) keeps its present height here, so its floor
        // comes from the lowest point a run has recorded.
        double resting_height_energy(const physics::World& world, const std::vector<physics::BodyId>* only)
        {
            const auto solid = [](const physics::RigidBody& body, const physics::RigidBody& other)
            {
                for (const auto& surface : body.colliders())
                    for (const auto& part : other.colliders())
                        if (!surface.is_sensor && !part.is_sensor && physics::should_collide(surface.filter, part.filter))
                            return true;
                return false;
            };
            double total = 0.0;
            for (const auto id : world.body_ids())
            {
                const auto* body = world.find_body(id);
                if (!body || body->type() != physics::BodyType::dynamic_body || (only && std::find(only->begin(), only->end(), id) == only->end()))
                    continue;
                const auto gravity = physics::effective_uniform_gravity_m_s2(world, id);
                const auto strength = math::length(gravity);
                const auto now = physics::measure_body_energy(world, id).value_or(physics::EnergyBreakdown {}).gravitational_potential_j;
                total += now;
                if (!(strength > 1.0e-9) || !std::isfinite(now))
                    continue;
                const auto down = gravity / strength;
                const auto side = math::perpendicular(down);
                const auto extent = [&](const math::Aabb& box, const math::Vec2& axis)
                {
                    const std::array<math::Vec2, 4> corners { box.minimum, math::Vec2 { box.maximum.x, box.minimum.y }, box.maximum, math::Vec2 { box.minimum.x, box.maximum.y } };
                    auto low = math::dot(corners[0], axis), high = low;
                    for (const auto& corner : corners)
                    {
                        low = std::min(low, math::dot(corner, axis));
                        high = std::max(high, math::dot(corner, axis));
                    }
                    return std::pair { low, high };
                };
                const auto bounds = body->compute_bounds();
                if (bounds.is_empty())
                    continue;
                const auto [left, right] = extent(bounds, side);
                const auto lowest_point = extent(bounds, down).second;
                std::optional<double> drop;
                for (const auto other_id : world.body_ids())
                {
                    const auto* other = world.find_body(other_id);
                    if (!other || other->type() != physics::BodyType::static_body || !solid(*other, *body))
                        continue;
                    const auto other_bounds = other->compute_bounds();
                    if (other_bounds.is_empty())
                        continue;
                    const auto [other_left, other_right] = extent(other_bounds, side);
                    const auto top = extent(other_bounds, down).first;
                    if (other_right <= left || other_left >= right || top < lowest_point - 1.0e-6)
                        continue;
                    drop = std::min(drop.value_or(top - lowest_point), top - lowest_point);
                }
                if (drop)
                    total -= body->mass_properties().mass_kg * strength * std::max(0.0, *drop);
            }
            return total;
        }

        // Whether gravity pulls any measured object, the only way height can hold energy.
        bool under_gravity(const physics::World& world, const std::vector<physics::BodyId>* only)
        {
            for (const auto id : world.body_ids())
                if (!only || std::find(only->begin(), only->end(), id) != only->end())
                    if (math::length_squared(physics::effective_uniform_gravity_m_s2(world, id)) > 0.0)
                        return true;
            return false;
        }

        // Where each measured object sits among a run's recorded objects; empty when none was
        // recorded.
        std::vector<std::size_t> recorded_objects(const RunRecord& run, const std::vector<physics::BodyId>& bodies)
        {
            std::vector<std::size_t> result;
            for (const auto id : bodies)
            {
                const auto object = std::find(run.series.object_ids.begin(), run.series.object_ids.end(), id);
                if (object != run.series.object_ids.end())
                    result.push_back(static_cast<std::size_t>(std::distance(run.series.object_ids.begin(), object)));
            }
            return result;
        }

        // The lowest height energy recorded in a run, for the whole scene or a set of objects: the
        // reference that keeps potential energy a non-negative share.
        std::optional<double> lowest_height_energy(const RunRecord* run, const std::vector<physics::BodyId>* bodies)
        {
            if (!run)
                return {};
            const auto& series = run->series;
            const auto samples = series.time_s.size();
            std::optional<double> lowest;
            const auto consider = [&](double value)
            {
                if (std::isfinite(value))
                    lowest = std::min(lowest.value_or(value), value);
            };
            if (!bodies)
            {
                for (std::size_t sample = 0; sample < samples && (sample + 1) * scene_stride <= series.scene.size(); ++sample)
                    consider(series.scene[sample * scene_stride + 2]);
                return lowest;
            }
            const auto objects = recorded_objects(*run, *bodies);
            if (objects.empty())
                return {};
            const auto stride = series.object_ids.size() * object_stride;
            for (std::size_t sample = 0; sample < samples && (sample + 1) * stride <= series.objects.size(); ++sample)
            {
                double total = 0.0;
                for (const auto object : objects)
                    total += series.objects[sample * stride + object * object_stride + 2];
                consider(total);
            }
            return lowest;
        }

        // The objects the Selected scopes measure: the primary selection first, then the rest.
        std::vector<physics::BodyId> selected_objects(const UiModel& model)
        {
            std::vector<physics::BodyId> result;
            if (model.selection.is_valid())
                result.push_back(model.selection);
            for (const auto id : model.selected_bodies)
                if (id.is_valid() && std::find(result.begin(), result.end(), id) == result.end())
                    result.push_back(id);
            return result;
        }

        // Quantities that add up across objects, so a selection of several can be plotted as one.
        bool additive(std::string_view key)
        {
            return key != "speed" && key != "velocity_x" && key != "velocity_y" && key != "height" && key != "spin" && key != "position_x" && key != "rotation";
        }

        std::string display_name(const UiModel& model, physics::BodyId id)
        {
            for (const auto& object : model.objects)
                if (object.id == id)
                    return object.display_name;
            return "Object " + std::to_string(id.index + 1);
        }

        bool listed(const ImpactItem& impact, std::string_view filter)
        {
            return filter != "moving" || impact.between_free_bodies();
        }
        // "Ball ↔ Floor": the body that moves is named first.
        std::string impact_pair(const ImpactItem& impact)
        {
            const auto fixed_first = impact.first_type == physics::BodyType::static_body && impact.second_type != physics::BodyType::static_body;
            return fixed_first ? impact.second_name + " ↔ " + impact.first_name : impact.first_name + " ↔ " + impact.second_name;
        }

        std::string aggregator_label(RunAggregator value)
        {
            switch (value)
            {
            case RunAggregator::maximum:
                return "Maximum";
            case RunAggregator::minimum:
                return "Minimum";
            case RunAggregator::at_first_impact:
                return "At first impact";
            case RunAggregator::at_time:
                return "At time";
            default:
                return "At end";
            }
        }
    }

    std::size_t listed_impact_count(const UiModel& model, std::string_view filter)
    {
        return static_cast<std::size_t>(std::count_if(model.impacts.begin(), model.impacts.end(), [&](const auto& impact)
            {
                return listed(impact, filter);
            }));
    }

    std::string_view MeasurePanel::id() const
    {
        return "measure";
    }
    std::string_view MeasurePanel::title() const
    {
        return "Measure";
    }
    RegionId MeasurePanel::region() const
    {
        return RegionId::measure_drawer;
    }

    void MeasurePanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto impact_filter = std::string(builder.view_value("measure.collisions.filter", "all"));
        const auto run_count = model.runs.size() + (model.current_run ? 1 : 0);
        tab_labels_ = { "Energy", "Graph", "Collisions (" + std::to_string(listed_impact_count(model, impact_filter)) + ")", "Runs (" + std::to_string(run_count) + ")", "Theory checks" };
        static constexpr std::string_view tab_ids[] { "energy", "graph", "collisions", "runs", "theory" };
        tab_options_.clear();
        for (std::size_t index = 0; index < std::size(tab_ids); ++index)
            tab_options_.push_back({ tab_ids[index], tab_labels_[index], {}, {} });
        const auto tab = builder.tabs("measure.header.tabs", tab_options_, "energy");
        {
            auto close = action(UiCommandKind::none);
            close.detail = "view:measure";
            builder.action_row("view.measure_collapse", "Close Measure", close);
            builder.present_last(presentation(icons::caret_down, "M").icon_label_only().in_header());
        }
        if (tab == "energy")
        {
            const auto measured = selected_objects(model);
            static constexpr OptionSpec scopes[] { { "scene", "Whole scene", {}, {} }, { "selected", "Selected object", {}, {} } };
            static constexpr OptionSpec several_scopes[] { { "scene", "Whole scene", {}, {} }, { "selected", "Selected objects", {}, {} } };
            ControlSpec scope = spec("measure.energy.scope");
            scope.options = measured.size() > 1 ? several_scopes : scopes;
            // A wide drawer sets the shares, headed by the scope, beside the figures, so the parts
            // of the budget and where it went are in view together.
            const auto columns = builder.view_region_width(RegionId::measure_drawer) >= 640.0;
            const auto show_scope = [&](std::string_view group)
            {
                builder.begin_group(group);
                builder.segmented_row(scope, builder.view_value("measure.energy.scope", "scene"), state_choice("measure.energy.scope"));
                builder.end_group();
            };
            if (!model.world)
            {
                show_scope("inline_control");
                return;
            }
            const auto& world = *model.world;
            const auto selected_scope = builder.view_value("measure.energy.scope", "scene") == "selected";
            physics::EnergyBreakdown energy;
            math::Vec2 momentum;
            if (selected_scope)
            {
                bool measured_any = false;
                for (const auto id : measured)
                    if (const auto part = physics::measure_body_energy(world, id))
                    {
                        measured_any = true;
                        energy.translational_kinetic_j += part->translational_kinetic_j;
                        energy.rotational_kinetic_j += part->rotational_kinetic_j;
                        energy.gravitational_potential_j += part->gravitational_potential_j;
                        energy.spring_potential_j += part->spring_potential_j;
                        if (const auto* body = world.find_body(id); body && body->type() == physics::BodyType::dynamic_body)
                            momentum += body->linear_momentum_kg_m_s();
                    }
                if (!measured_any)
                {
                    show_scope("inline_control");
                    builder.paragraph("Select an object on the stage to see its energy.");
                    return;
                }
            }
            else
            {
                energy = physics::measure_world_energy(world);
                momentum = physics::measure_world_momentum_kg_m_s(world);
            }

            // Every way the scene can take energy away or put it in, so the rows add up: mechanical
            // energy now plus everything lost is the starting energy plus everything added. Rows
            // for interactions the scene does not contain appear only once they have done work.
            const auto budget = physics::measure_energy_budget(world);
            const auto nan = std::numeric_limits<double>::quiet_NaN();
            struct EnergyRow
            {
                std::string_view label;
                double value;
                bool optional_row;
                bool present;
            };
            constexpr std::size_t mechanical_row = 4, air_row = 7, damper_row = 8, joint_row = 9, first_added_row = 10;
            std::array<EnergyRow, 13> rows { {
                { "Kinetic (moving)", energy.translational_kinetic_j, false, false },
                { "Kinetic (spinning)", energy.rotational_kinetic_j, false, false },
                { "Potential (height)", energy.gravitational_potential_j, false, false },
                { "Potential (springs)", energy.spring_potential_j, false, false },
                { "Mechanical energy", energy.mechanical_j(), false, false },
                { "Lost in impacts", selected_scope ? nan : budget.lost_in_impacts_j, false, false },
                { "Lost to friction", selected_scope ? nan : budget.lost_to_friction_j, false, false },
                { "Lost to air", budget.lost_to_air_j, true, budget.has_air },
                { "Lost in dampers", budget.lost_in_dampers_j, true, budget.has_dampers },
                { "Lost in joints", budget.lost_in_joints_j, true, budget.has_joints },
                { "Added by motors and drives", budget.added_by_drives_j, true, budget.has_drives },
                { "Added by applied forces", budget.added_by_forces_j, true, budget.has_applied_forces },
                { "Added by your changes", budget.added_by_changes_j, true, false },
            } };
            // The balance: mechanical energy now with everything lost and less everything added
            // comes back to the energy at the start, which the scene shows when it balances.
            const auto balance_j = energy.mechanical_j() + budget.lost_j() - budget.added_j();
            std::array<double, rows.size() + 2> values {};
            for (std::size_t index = 0; index < rows.size(); ++index)
                values[index] = rows[index].optional_row && selected_scope ? 0.0 : rows[index].value;
            values[rows.size()] = selected_scope ? 0.0 : balance_j;
            values[rows.size() + 1] = selected_scope ? 0.0 : budget.start_energy_j;
            const auto energy_column = column_format(values, core::DisplayQuantity::energy, model.display_units);
            const auto momentum_column = column_format({ momentum.x, momentum.y }, core::DisplayQuantity::momentum, model.display_units);
            const auto readable = [&](double value)
            {
                return std::isfinite(value) && in_column(energy_column, value) != in_column(energy_column, 0.0);
            };
            for (auto& row : rows)
                row.present = !row.optional_row || (!selected_scope && (row.present || readable(row.value)));
            bool any_added = false;
            for (std::size_t index = first_added_row; index < rows.size(); ++index)
                any_added = any_added || rows[index].present;

            // Shares of one whole: potential counts down to the surface beneath each object, or to
            // the lowest point reached with this setup (this run, and the previous run when nothing
            // was changed in between) where that is lower, so every part is non-negative and they
            // add up to 100 %.
            const auto height_now = energy.gravitational_potential_j;
            const auto* measured_bodies = selected_scope ? &measured : nullptr;
            auto height_floor = std::min(height_now, resting_height_energy(world, measured_bodies));
            height_floor = std::min(height_floor, lowest_height_energy(model.current_run, measured_bodies).value_or(height_floor));
            if (model.current_run && model.current_run->changes_from_previous.empty() && model.previous_run)
                height_floor = std::min(height_floor, lowest_height_energy(model.previous_run, measured_bodies).value_or(height_floor));
            struct Share
            {
                std::string_view name;
                double part;
                std::uint8_t series;
            };
            // The shares carry the table's own names, so each one can be found among the rows.
            constexpr std::size_t contact_share = 2;
            std::array<Share, 7> shares {};
            std::size_t share_count = 0;
            shares[share_count++] = { "Kinetic", std::max(0.0, energy.kinetic_j()), 1 };
            shares[share_count++] = { "Potential", std::max(0.0, height_now - height_floor) + std::max(0.0, energy.spring_potential_j), 3 };
            shares[share_count++] = { "Lost in impacts", selected_scope ? 0.0 : std::max(0.0, budget.lost_in_impacts_j), 2 };
            shares[share_count++] = { "Lost to friction", selected_scope ? 0.0 : std::max(0.0, budget.lost_to_friction_j), 2 };
            for (const auto row : { air_row, damper_row, joint_row })
                if (rows[row].present)
                    shares[share_count++] = { rows[row].label, std::max(0.0, rows[row].value), 2 };
            std::array<double, shares.size()> parts {};
            double total = 0.0;
            for (std::size_t index = 0; index < share_count; ++index)
            {
                parts[index] = shares[index].part;
                total += parts[index];
            }
            const auto shared = total > 1.0e-6;
            const auto percentages = whole_percentages(parts, share_count);
            // Only energy actually put in is shared out; a joint's negative work is a loss.
            bool added = false;
            for (std::size_t index = first_added_row; index < rows.size(); ++index)
                added = added || (!selected_scope && readable(std::max(0.0, rows[index].value)));

            const auto show_figures = [&]
            {
                builder.begin_group(columns ? "energy_main" : "");
                for (std::size_t index = 0; index < rows.size(); ++index)
                    if (rows[index].present)
                    {
                        builder.live_value_row(rows[index].label, in_column(energy_column, rows[index].value));
                        // Mechanical energy sums the rows above it, so it reads as a subtotal.
                        builder.present_last(index == mechanical_row ? presentation(icons::measure).primary() : presentation(icons::measure));
                    }
                if (!selected_scope)
                {
                    builder.live_value_row(any_added ? "Mechanical + lost \xE2\x88\x92 added" : "Mechanical + lost", in_column(energy_column, balance_j));
                    builder.present_last(presentation(icons::measure).primary());
                    builder.live_value_row("Energy at start", in_column(energy_column, budget.start_energy_j));
                    builder.present_last(presentation(icons::measure));
                }
                for (const auto& [label, value] : { std::pair { "Momentum x", momentum.x }, std::pair { "Momentum y", momentum.y } })
                {
                    builder.live_value_row(label, in_column(momentum_column, value));
                    builder.present_last(presentation(icons::measure));
                }
                builder.end_group();
            };
            const auto show_shares = [&]
            {
                builder.begin_group(columns ? "energy_side" : "");
                builder.heading("Energy shares");
                for (std::size_t index = 0; index < share_count; ++index)
                {
                    const auto available = shared && !(selected_scope && (index == contact_share || index == contact_share + 1));
                    const auto text = available ? core::format_quantity(percentages[index], core::DisplayQuantity::percentage, core::DisplayUnits::si) : std::string { "\xE2\x80\x94" };
                    builder.share_row(shares[index].name, available ? parts[index] / total : 0.0, text, shares[index].series);
                }
                // Say only what applies: where height is counted from matters only under gravity.
                const auto gravity = under_gravity(world, measured_bodies);
                std::string note;
                if (!shared)
                    note = model.elapsed_time_s > 0.0 ? "Nothing is moving yet, and no energy has been lost." : "Press Play to see how the energy changes.";
                else if (selected_scope)
                    note = std::string(gravity ? "Potential energy uses the fixed surface below the object, or its lowest recorded position, as zero height. " : "") + "Losses are measured for the whole scene.";
                else
                {
                    if (gravity)
                        note = "Potential energy uses the fixed surface below each object, or its lowest recorded position, as zero height.";
                    if (added)
                        note += std::string(note.empty() ? "" : " ") + "The chart also includes energy added during the run.";
                }
                if (!note.empty())
                    builder.paragraph(note);
                builder.end_group();
            };
            if (columns)
            {
                show_scope("energy_side");
                show_shares();
                show_figures();
            }
            else
            {
                show_scope("inline_control");
                show_figures();
                show_shares();
            }
        }
        else if (tab == "graph")
        {
            static constexpr OptionSpec energy_quantities[] { { "mechanical", "Mechanical", {}, {} }, { "kinetic_moving", "Kinetic (moving)", {}, {} }, { "kinetic_spinning", "Kinetic (spinning)", {}, {} }, { "potential_height", "Potential (height)", {}, {} }, { "potential_springs", "Potential (springs)", {}, {} }, { "lost_impacts", "Lost in impacts", {}, {} }, { "lost_friction", "Lost to friction", {}, {} } };
            static constexpr OptionSpec motion_quantities[] { { "momentum_x", "Momentum x", {}, {} }, { "momentum_y", "Momentum y", {}, {} }, { "speed", "Speed", {}, {} } };
            // The other rows of the energy budget can be plotted wherever the Energy tab lists them.
            std::array<bool, std::size(ledger_quantities)> ledger_offered {};
            if (model.world)
            {
                const auto budget = physics::measure_energy_budget(*model.world);
                const std::array<double, std::size(ledger_quantities)> ledger_values { budget.lost_to_air_j, budget.lost_in_dampers_j, budget.lost_in_joints_j, budget.added_by_drives_j, budget.added_by_forces_j, budget.added_by_changes_j };
                const std::array<bool, std::size(ledger_quantities)> ledger_kinds { budget.has_air, budget.has_dampers, budget.has_joints, budget.has_drives, budget.has_applied_forces, false };
                for (std::size_t index = 0; index < ledger_offered.size(); ++index)
                    ledger_offered[index] = ledger_kinds[index] || std::abs(ledger_values[index]) >= 0.005;
            }
            graph_quantity_options_.assign(std::begin(energy_quantities), std::end(energy_quantities));
            for (std::size_t index = 0; index < ledger_offered.size(); ++index)
                if (ledger_offered[index])
                    graph_quantity_options_.push_back({ ledger_quantities[index].key, ledger_quantities[index].label, {}, {} });
            graph_quantity_options_.insert(graph_quantity_options_.end(), std::begin(motion_quantities), std::end(motion_quantities));
            ControlSpec choices = spec("measure.graph.quantities");
            choices.options = graph_quantity_options_;
            static constexpr std::string_view defaults[] { "mechanical" };
            // The controls' values are read before the graph is built; their rows are recorded
            // around it once it is.
            auto selected = builder.view_checklist(choices.key, defaults);
            selected.erase(std::remove_if(selected.begin(), selected.end(), [&](const auto& key)
                               {
                                   return std::none_of(graph_quantity_options_.begin(), graph_quantity_options_.end(), [&](const auto& option)
                                       {
                                           return option.id == key;
                                       });
                               }),
                selected.end());
            if (selected.size() > 3)
                selected.resize(3);
            const auto window_text = builder.view_value("measure.graph.window", "10");
            auto compare = std::string(builder.view_value("measure.graph.compare_with", "none"));
            if (compare != "none" && !run_number(model.runs, compare))
                compare = "none";
            const auto graph_scope = std::string(builder.view_value("measure.graph.scope", "selection"));
            const auto previous_visible = builder.view_value("measure.graph.previous_run", "on") != "off";

            plot_ = {};
            plot_values_.clear();
            plot_values_.reserve(selected.size() * 3);
            // Follow selection plots every selected object; several are plotted as their total.
            std::vector<physics::BodyId> scoped_bodies;
            if (graph_scope == "selection")
                scoped_bodies = selected_objects(model);
            else if (model.current_run)
                for (const auto id : model.current_run->series.object_ids)
                    if (body_key(id) == graph_scope)
                        scoped_bodies.push_back(id);
            const auto slots = series_slots(selected);
            const auto append = [&](const RunRecord* run, SeriesStyle style, const std::string& source)
            {
                if (!run)
                    return;
                const auto objects = recorded_objects(*run, scoped_bodies);
                const auto stride = run->series.object_ids.size() * object_stride;
                for (std::size_t slot = 0; slot < selected.size(); ++slot)
                {
                    const auto& key = selected[slot];
                    plot_values_.emplace_back();
                    auto& values = plot_values_.back();
                    values.reserve(run->series.time_s.size());
                    for (std::size_t sample = 0; sample < run->series.time_s.size(); ++sample)
                    {
                        if (!objects.empty())
                        {
                            const auto channel = object_channel(key);
                            if (!channel || (objects.size() > 1 && !additive(key)))
                            {
                                values.push_back(std::numeric_limits<float>::quiet_NaN());
                                continue;
                            }
                            float total = 0.0f;
                            for (const auto object : objects)
                                total += run->series.objects[sample * stride + object * object_stride + *channel];
                            values.push_back(total);
                        }
                        else if (key == "mechanical")
                        {
                            const auto base = sample * scene_stride;
                            values.push_back(run->series.scene[base] + run->series.scene[base + 1] + run->series.scene[base + 2] + run->series.scene[base + 3]);
                        }
                        else if (const auto channel = scene_channel(key))
                            values.push_back(run->series.scene[sample * scene_stride + *channel]);
                        else if (const auto ledger = ledger_channel(key); ledger && (sample + 1) * ledger_stride <= run->series.ledger.size())
                            values.push_back(run->series.ledger[sample * ledger_stride + *ledger]);
                        else
                            values.push_back(std::numeric_limits<float>::quiet_NaN());
                    }
                    plot_.series.push_back({ quantity_label(key), values.data(), run->series.time_s.data(), values.size(), style, slots[slot], source, quantity_unit(key) });
                }
            };
            append(model.current_run, SeriesStyle::solid, "This run");
            if (previous_visible)
                append(model.previous_run, SeriesStyle::dashed, "Previous run");
            if (compare != "none")
                for (const auto& run : model.runs)
                    if (compare == std::to_string(run.number))
                        append(&run, SeriesStyle::dotted, "Run " + std::to_string(run.number));
            for (const auto& marker : model.graph_markers)
                plot_.markers.push_back({ marker.time_s, marker.label, PlotMarkerKind::intervention });
            double window_s = 10.0;
            if (window_text == "30")
                window_s = 30.0;
            else if (window_text == "60")
                window_s = 60.0;
            double x_max = model.current_run ? model.current_run->duration_s : model.previous_run ? model.previous_run->duration_s
                                                                                                  : window_s;
            double x_min = std::max(0.0, x_max - window_s);
            if (model.current_run)
                x_min = std::max(x_min, model.current_run->plot_start_s);
            plot_.x = { "Time", "s", x_min, std::max(x_min + 1.0e-3, x_max) };
            double y_min = 0.0, y_max = 0.0;
            bool finite_value = false;
            std::vector<bool> key_has_data(selected.size(), false);
            for (std::size_t series_index = 0; series_index < plot_.series.size(); ++series_index)
            {
                const auto& series = plot_.series[series_index];
                std::size_t finite = 0;
                for (std::size_t index = 0; index < series.count; ++index)
                    if (series.times[index] >= x_min && series.times[index] <= x_max && std::isfinite(series.values[index]))
                    {
                        y_min = finite_value ? std::min(y_min, static_cast<double>(series.values[index])) : std::min(0.0, static_cast<double>(series.values[index]));
                        y_max = finite_value ? std::max(y_max, static_cast<double>(series.values[index])) : std::max(0.0, static_cast<double>(series.values[index]));
                        finite_value = true;
                        ++finite;
                    }
                if (finite >= 2 && !selected.empty())
                    key_has_data[series_index % selected.size()] = true;
            }

            // The axis is named after the quantities actually drawn, so a chosen series without
            // data never mislabels it; the plot rounds this extent out to whole tick steps.
            std::vector<std::string> drawn;
            for (std::size_t index = 0; index < selected.size(); ++index)
                if (key_has_data[index])
                    drawn.push_back(selected[index]);
            const auto& axis_keys = drawn.empty() ? selected : drawn;
            const auto y_key = axis_keys.empty() ? std::string { "mechanical" } : axis_keys.front();
            const auto y_unit = quantity_unit(y_key);
            const auto shared_unit = std::all_of(axis_keys.begin(), axis_keys.end(), [&](const auto& key)
                {
                    return quantity_unit(key) == y_unit;
                });
            auto y_label = quantity_label(y_key);
            if (axis_keys.size() > 1)
                y_label = !shared_unit ? "Value" : y_unit == "J" ? "Energy"
                    : y_unit == "m/s"                            ? "Speed"
                                                                 : "Momentum";
            plot_.y = { y_label, shared_unit ? y_unit : std::string {}, y_min, y_max };

            // Say why a chosen quantity has nothing to draw, rather than implying nothing ran.
            const auto* reference_run = model.current_run ? model.current_run : model.previous_run;
            const auto resolved_objects = reference_run ? recorded_objects(*reference_run, scoped_bodies) : std::vector<std::size_t> {};
            const auto object_resolved = !resolved_objects.empty();
            const auto several_objects = resolved_objects.size() > 1;
            // Impacts are marked when they involve what is plotted: any of them for the whole
            // scene, otherwise only those of the plotted objects.
            if (model.current_run)
                for (const auto& impact : model.impacts)
                {
                    const auto involves = [&](physics::BodyId id)
                    {
                        return std::any_of(resolved_objects.begin(), resolved_objects.end(), [&](std::size_t object)
                            {
                                return reference_run->series.object_ids[object] == id;
                            });
                    };
                    if (impact.time_s <= model.current_run->duration_s + 1.0e-9 && (!object_resolved || involves(impact.first) || involves(impact.second)))
                        plot_.markers.push_back({ impact.time_s, core::substitute("Impact {}", impact.index + 1), PlotMarkerKind::impact });
                }
            // Name what is plotted, and why Follow selection fell back to the whole scene.
            if (several_objects)
                plot_.subject = core::substitute("{} selected objects", resolved_objects.size());
            else if (object_resolved)
                plot_.subject = display_name(model, reference_run->series.object_ids[resolved_objects.front()]);
            else if (graph_scope != "selection" || !reference_run)
                plot_.subject = "Whole scene";
            else if (!model.selection.is_valid())
                plot_.subject = "Whole scene (nothing selected)";
            else
            {
                const auto object = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& item)
                    {
                        return item.id == model.selection;
                    });
                plot_.subject = object != model.objects.end() && object->kind == "fixed" ? "Whole scene (selection is fixed)"
                    : object != model.objects.end() && object->kind == "driven"          ? "Whole scene (selection is driven)"
                                                                                         : "Whole scene (selection not recorded)";
            }
            const auto lowercase = [](std::string text)
            {
                if (!text.empty() && text.front() >= 'A' && text.front() <= 'Z')
                    text.front() = static_cast<char>(text.front() - 'A' + 'a');
                return text;
            };
            const auto missing_reason = [&](const std::string& key) -> std::string
            {
                if (object_resolved && !object_channel(key))
                    return quantity_label(key) + " is measured for the whole scene only.";
                if (several_objects && !additive(key))
                    return "Select one object to plot " + lowercase(quantity_label(key)) + ".";
                if (!object_resolved && !scene_channel(key) && !ledger_channel(key) && key != "mechanical")
                    return "Select an object to plot " + lowercase(quantity_label(key)) + ".";
                return {};
            };
            const auto window_seconds = static_cast<int>(window_s);
            if (selected.empty())
            {
                plot_.empty_title = "Choose a quantity to plot";
                plot_.empty_detail = "Pick one or more under Quantities.";
            }
            else if (!model.current_run && !model.previous_run && compare == "none")
            {
                plot_.empty_title = "Run the experiment to record a graph";
                plot_.empty_detail = core::substitute("Press Play. The graph follows the last {} s.", window_seconds);
            }
            else if (drawn.empty() && !missing_reason(selected.front()).empty())
            {
                const auto reason = missing_reason(selected.front());
                plot_.empty_title = reason.substr(0, reason.size() - 1);
                plot_.empty_detail = several_objects && !additive(selected.front()) ? "Choose one object under Scope."
                    : object_resolved                                               ? "Set Scope to Whole scene."
                                                                                    : "Select it on the stage, or choose an object under Scope.";
            }
            else if (drawn.empty())
            {
                plot_.empty_title = "No samples in this window yet";
                plot_.empty_detail = model.paused ? core::substitute("Press Play to keep recording. The graph follows the last {} s.", window_seconds) : "Recording as the experiment runs.";
            }
            else
                for (std::size_t index = 0; index < selected.size(); ++index)
                    if (!key_has_data[index])
                    {
                        const auto reason = missing_reason(selected[index]);
                        plot_.note += (plot_.note.empty() ? "" : " ") + (reason.empty() ? "No data yet for " + lowercase(quantity_label(selected[index])) + "." : reason);
                    }

            // In a wide drawer the controls stand beside the graph, so what is plotted can be
            // changed without scrolling; otherwise the graph leads and they follow below it.
            const auto controls_beside = builder.view_region_width(RegionId::measure_drawer) >= 720.0;
            if (!controls_beside)
                builder.plot("measure.graph.plot", plot_);
            builder.begin_group(controls_beside ? "graph_side" : "");
            option_ids_.clear();
            option_labels_.clear();
            graph_compare_options_.clear();
            graph_scope_options_.clear();
            option_ids_.reserve(model.runs.size() * 2 + 64);
            option_labels_.reserve(model.runs.size() * 2 + 64);
            const auto add_option = [&](std::vector<OptionSpec>& options, std::string id, std::string label)
            {
                option_ids_.push_back(std::move(id));
                option_labels_.push_back(std::move(label));
                options.push_back({ option_ids_.back(), option_labels_.back(), {}, {} });
            };
            add_option(graph_scope_options_, "scene", "Whole scene");
            add_option(graph_scope_options_, "selection", "Follow selection");
            if (model.current_run)
                for (const auto id : model.current_run->series.object_ids)
                    add_option(graph_scope_options_, body_key(id), display_name(model, id));
            ControlSpec scope_spec = spec("measure.graph.scope");
            scope_spec.options = graph_scope_options_;
            builder.select_row(scope_spec, graph_scope, state_choice("measure.graph.scope"));

            (void)builder.checklist(choices, defaults);
            static constexpr OptionSpec windows[] { { "10", "10 s", {}, {} }, { "30", "30 s", {}, {} }, { "60", "60 s", {}, {} } };
            ControlSpec window = spec("measure.graph.window");
            window.options = windows;
            builder.segmented_row(window, window_text, state_choice("measure.graph.window"));

            add_option(graph_compare_options_, "none", "None");
            for (const auto& run : model.runs)
                add_option(graph_compare_options_, std::to_string(run.number), "Run " + std::to_string(run.number));
            ControlSpec compare_spec = spec("measure.graph.compare_with");
            compare_spec.options = graph_compare_options_;
            builder.select_row(compare_spec, compare, state_choice("measure.graph.compare_with"));

            builder.checkbox_row(spec("measure.graph.previous_run"), previous_visible, state_value("measure.graph.previous_run", previous_visible ? "off" : "on"));
            builder.action_row("measure.graph.clear", "Clear graph", action(UiCommandKind::clear_energy_history));
            builder.end_group();
            if (controls_beside)
            {
                builder.begin_group("graph_main");
                builder.plot("measure.graph.plot", plot_);
                builder.end_group();
            }
        }
        else if (tab == "collisions")
        {
            // The controls share one line, so the list starts near the top of the drawer.
            builder.begin_group("collision_controls");
            builder.switch_row(spec("measure.collisions.pause_each"), model.pause_on_impact, toggle(UiCommandKind::set_pause_on_impact, !model.pause_on_impact));
            builder.action_row(model.next_impact_armed ? "Cancel pause at next impact" : "Pause at next impact", toggle(UiCommandKind::pause_at_next_impact, !model.next_impact_armed));
            static constexpr OptionSpec filters[] { { "all", "All contacts", {}, {} }, { "moving", "Moving pairs", {}, {} } };
            ControlSpec filter = spec("measure.collisions.filter");
            filter.options = filters;
            builder.segmented_row(filter, impact_filter, state_choice("measure.collisions.filter"));
            builder.end_group();
            std::vector<double> listed_losses;
            for (const auto& impact : model.impacts)
                if (listed(impact, impact_filter))
                    listed_losses.push_back(impact.restitution_loss_j + impact.friction_loss_j);
            const auto listed_loss_column = column_format(listed_losses, core::DisplayQuantity::energy, model.display_units);
            std::size_t listed_count = 0;
            for (const auto& impact : model.impacts)
            {
                if (!listed(impact, impact_filter))
                    continue;
                ++listed_count;
                // When, how fast the two met, and what the impact took away.
                const auto closing_speed = math::length(impact.second_velocity_before_m_s - impact.first_velocity_before_m_s);
                const auto loss = impact.restitution_loss_j + impact.friction_loss_j;
                const auto summary = core::substitute(loss < 0.0 && in_column(listed_loss_column, loss) != in_column(listed_loss_column, 0.0) ? "{} · {} · {} gained" : "{} · {} · {} lost",
                    core::format_quantity(impact.time_s, core::DisplayQuantity::time, model.display_units),
                    core::format_quantity(closing_speed, core::DisplayQuantity::velocity, model.display_units),
                    in_column(listed_loss_column, std::abs(loss)));
                ListItemContent item { core::substitute("Impact {} · {}", impact.index + 1, impact_pair(impact)), summary, {}, {} };
                auto select = action(UiCommandKind::select_impact);
                select.value = static_cast<double>(impact.index);
                builder.list_item("measure.collisions.list", std::to_string(impact.index), item, select);
                builder.select_last(impact.selected);
            }
            if (listed_count == 0)
            {
                const auto hidden = model.impacts.size();
                builder.paragraph(hidden == 0 ? "No impacts yet. Press Play to record them, then select an impact to compare values before and after contact."
                                              : core::substitute("No impacts between two moving objects yet. Choose All contacts to show the {} {} with fixed or driven objects.", hidden, hidden == 1 ? "impact" : "impacts"));
            }
            // The chosen impact stays explained even when the filter hides it from the list, as
            // when it was picked on the stage.
            const auto selected_impact = std::find_if(model.impacts.begin(), model.impacts.end(), [](const auto& value)
                {
                    return value.selected;
                });
            if (selected_impact != model.impacts.end())
            {
                const auto& impact = *selected_impact;
                const auto velocity = [&](double value)
                {
                    return core::format_quantity(value, core::DisplayQuantity::velocity, model.display_units);
                };
                builder.heading(core::substitute("Impact {}: before → after", impact.index + 1));
                builder.label(core::substitute("{} at {}", impact_pair(impact), core::format_quantity(impact.time_s, core::DisplayQuantity::time, model.display_units)));
                // A fixed body never moves, so its velocity rows would only ever read zero.
                const auto body_rows = [&](const std::string& name, physics::BodyType type, math::Vec2 before, math::Vec2 after)
                {
                    if (type == physics::BodyType::static_body)
                        return;
                    builder.value_row(name + " velocity x", core::substitute("{} → {}", velocity(before.x), velocity(after.x)));
                    builder.value_row(name + " velocity y", core::substitute("{} → {}", velocity(before.y), velocity(after.y)));
                    builder.value_row(name + " speed", core::substitute("{} → {}", velocity(math::length(before)), velocity(math::length(after))));
                };
                body_rows(impact.first_name, impact.first_type, impact.first_velocity_before_m_s, impact.first_velocity_after_m_s);
                body_rows(impact.second_name, impact.second_type, impact.second_velocity_before_m_s, impact.second_velocity_after_m_s);
                // Each body receives the same size of impulse from the contact; name one that moves
                // freely, whose momentum change the impulse explains.
                const auto second_named = impact.second_type == physics::BodyType::dynamic_body || impact.first_type != physics::BodyType::dynamic_body;
                const auto& impulse_body = second_named ? impact.second_name : impact.first_name;
                const auto contact_impulse = second_named ? impact.impulse_on_second_n_s : -impact.impulse_on_second_n_s;
                const auto momentum_change = second_named ? impact.second_momentum_after_kg_m_s - impact.second_momentum_before_kg_m_s : impact.first_momentum_after_kg_m_s - impact.first_momentum_before_kg_m_s;
                const auto impulse = math::length(contact_impulse);
                // While the objects touch, their weight (and any other contact or joint) pushes
                // too. Along the contact's impulse the parts add up to the change in momentum.
                const auto along = impulse > 0.0 ? contact_impulse / impulse : math::Vec2 {};
                const auto other_impulse = math::dot(momentum_change - contact_impulse, along);
                const auto momentum_column = column_format({ impulse, other_impulse, impact.total_momentum_before_kg_m_s.x, impact.total_momentum_after_kg_m_s.x, impact.total_momentum_before_kg_m_s.y, impact.total_momentum_after_kg_m_s.y }, core::DisplayQuantity::momentum, model.display_units);
                builder.value_row("Total momentum x", core::substitute("{} → {}", in_column(momentum_column, impact.total_momentum_before_kg_m_s.x), in_column(momentum_column, impact.total_momentum_after_kg_m_s.x)));
                builder.value_row("Total momentum y", core::substitute("{} → {}", in_column(momentum_column, impact.total_momentum_before_kg_m_s.y), in_column(momentum_column, impact.total_momentum_after_kg_m_s.y)));
                const auto free_body = impact.first_type == physics::BodyType::dynamic_body || impact.second_type == physics::BodyType::dynamic_body;
                const auto other_name = impact.coupled ? "Other contacts and joints" : impact.weighted ? "Weight during contact"
                                                                                                       : "Other forces during contact";
                if (!free_body || in_column(momentum_column, other_impulse) == in_column(momentum_column, 0.0))
                    builder.value_row("Impulse on " + impulse_body, in_column(momentum_column, impulse));
                else
                {
                    builder.value_row("Contact impulse on " + impulse_body, in_column(momentum_column, impulse));
                    builder.value_row(other_name, in_column(momentum_column, other_impulse));
                    builder.value_row("Change in momentum of " + impulse_body, in_column(momentum_column, impulse + other_impulse));
                }
                // The loss is the kinetic energy the objects gave up, plus any work done on them
                // while they touched: by their weight, or by a driven object pushing.
                const auto driven = impact.first_type == physics::BodyType::kinematic_body || impact.second_type == physics::BodyType::kinematic_body;
                const auto other_work = impact.restitution_loss_j + impact.friction_loss_j - (impact.kinetic_before_j - impact.kinetic_after_j);
                const auto loss_column = column_format({ impact.restitution_loss_j, impact.friction_loss_j, impact.kinetic_before_j, impact.kinetic_after_j, other_work }, core::DisplayQuantity::energy, model.display_units);
                builder.value_row("Kinetic energy", core::substitute("{} → {}", in_column(loss_column, impact.kinetic_before_j), in_column(loss_column, impact.kinetic_after_j)));
                if (in_column(loss_column, other_work) != in_column(loss_column, 0.0))
                    builder.value_row(impact.coupled ? "Work by other contacts and joints" : impact.weighted && !driven ? "Work by weight during contact"
                                                                                                                        : "Work by other forces during contact",
                        in_column(loss_column, other_work));
                builder.value_row("Lost in this impact", in_column(loss_column, impact.restitution_loss_j));
                builder.value_row("Lost to friction", in_column(loss_column, impact.friction_loss_j));
            }
        }
        else if (tab == "runs")
        {
            option_ids_.clear();
            option_labels_.clear();
            run_options_.clear();
            object_options_.clear();
            option_ids_.reserve(model.runs.size() + 96);
            option_labels_.reserve(model.runs.size() + 96);
            const auto add_option = [&](std::vector<OptionSpec>& options, std::string id, std::string label)
            {
                option_ids_.push_back(std::move(id));
                option_labels_.push_back(std::move(label));
                options.push_back({ option_ids_.back(), option_labels_.back(), {}, {} });
            };
            for (const auto& run : model.runs)
                add_option(run_options_, std::to_string(run.number), "Run " + std::to_string(run.number));
            if (model.current_run)
            {
                // The run reads the stage's clock; its samples fall on a 40 Hz grid that trails it.
                const auto played = core::format_quantity(model.elapsed_time_s, core::DisplayQuantity::time, model.display_units);
                builder.live_value_row(core::substitute("Run {} (this run)", model.current_run->number), model.paused ? "Paused at " + played : "Recording · " + played);
                if (model.runs.empty())
                    builder.paragraph("Press Back to start (R) to keep this run in the table, then change one thing and play again.");
            }
            else if (model.runs.empty())
                builder.paragraph("Press Play to record a run. Back to start (R) keeps it in this table.");
            for (const auto& run : model.runs)
            {
                auto summary = core::format_quantity(run.duration_s, core::DisplayQuantity::time, model.display_units);
                if (!run.changes_from_previous.empty())
                    summary += " · " + run.changes_from_previous.front().label + " " + run.changes_from_previous.front().original_text + " → " + run.changes_from_previous.front().current_text;
                if (run.changes_from_previous.size() > 1)
                    summary += " +" + std::to_string(run.changes_from_previous.size() - 1) + " more";
                if (run.prediction)
                    summary += " · Predicted: " + (!run.prediction->option.empty() ? run.prediction->option : run.prediction->note);
                if (run.changed_during_run)
                    summary += " · changed during run";
                ListItemContent item { "Run " + std::to_string(run.number), summary, run.starred ? "★" : "☆", {} };
                auto star = action(UiCommandKind::star_run);
                star.value = run.number;
                star.flag = !run.starred;
                const auto star_reason = !run.starred && model.starred_run_count >= 8
                    ? "At most 8 runs can be starred. Unstar one first."
                    : "";
                builder.list_item("measure.runs.row", std::to_string(run.number), item, star, {}, star_reason, {}, run.changed_during_run ? "An object was moved, thrown, pulled or edited during this run." : "");
            }
            if (model.runs.size() == 1 && !model.current_run)
                builder.paragraph("Change one thing and play again to compare.");
            builder.heading("Pinned values");
            if (model.pinned_values.empty())
                builder.paragraph("Choose Add value to record a quantity for every run, or right-click any value and choose Add to runs table.");
            for (std::size_t index = 0; index < model.pinned_values.size(); ++index)
            {
                const auto& pinned = model.pinned_values[index];
                builder.value_row(quantity_label(pinned.quantity), aggregator_label(pinned.aggregator));
                auto remove = action(UiCommandKind::unpin_run_value);
                remove.id = pinned.key;
                builder.action_row("Remove " + quantity_label(pinned.quantity), remove);
            }
            const auto add_open = builder.view_value("measure.runs.add_open", "false") == "true";
            builder.action_row("measure.runs.add_value", add_open ? "Close Add value" : "Add value", state_value("measure.runs.add_open", add_open ? "false" : "true"), model.pinned_values.size() >= 12 ? "At most 12 values can be pinned. Remove one first." : "");
            if (add_open && model.pinned_values.size() < 12)
            {
                static constexpr OptionSpec quantities[] {
                    { "mechanical", "Mechanical energy", {}, {} }, { "kinetic_moving", "Kinetic (moving)", {}, {} }, { "kinetic_spinning", "Kinetic (spinning)", {}, {} }, { "potential_height", "Potential (height)", {}, {} }, { "potential_springs", "Potential (springs)", {}, {} }, { "lost_impacts", "Lost in impacts", {}, {} }, { "lost_friction", "Lost to friction", {}, {} }, { "momentum_x", "Momentum x", {}, {} }, { "momentum_y", "Momentum y", {}, {} }, { "speed", "Speed", {}, {} }
                };
                static constexpr OptionSpec aggregators[] {
                    { "at_end", "At end", {}, {} }, { "maximum", "Maximum", {}, {} }, { "minimum", "Minimum", {}, {} }, { "at_first_impact", "At first impact", {}, {} }, { "at_time", "At t = …", {}, {} }
                };
                ControlSpec quantity = spec("measure.runs.add_quantity");
                quantity.options = quantities;
                ControlSpec aggregator = spec("measure.runs.add_aggregator");
                aggregator.options = aggregators;
                const auto quantity_id = std::string(builder.view_value("measure.runs.add_quantity", "mechanical"));
                const auto aggregator_id = std::string(builder.view_value("measure.runs.add_aggregator", "at_end"));
                builder.select_row(quantity, quantity_id, state_choice("measure.runs.add_quantity"));
                add_option(object_options_, "scene", "Whole scene");
                const auto* source = model.current_run ? model.current_run : model.previous_run;
                if (!source && !model.runs.empty())
                    source = &model.runs.back();
                if (source)
                    for (const auto id : source->series.object_ids)
                        add_option(object_options_, body_key(id), display_name(model, id));
                ControlSpec object = spec("measure.runs.add_object");
                object.options = object_options_;
                const auto object_id = std::string(builder.view_value("measure.runs.add_object", "scene"));
                builder.select_row(object, object_id, state_choice("measure.runs.add_object"));
                builder.select_row(aggregator, aggregator_id, state_choice("measure.runs.add_aggregator"));
                double at_time = 0.0;
                if (aggregator_id == "at_time")
                {
                    builder.text_field("measure.runs.add_time", "Time (0–60 s)", builder.view_value("measure.runs.add_time", "0"));
                    try
                    {
                        at_time = std::clamp(std::stod(std::string(builder.view_value("measure.runs.add_time", "0"))), 0.0, 60.0);
                    }
                    catch (...)
                    {
                        at_time = 0.0;
                    }
                }
                auto add = action(UiCommandKind::pin_run_value);
                add.id = quantity_id;
                add.detail = aggregator_id;
                add.value = at_time;
                if (source && object_id != "scene")
                    for (const auto id : source->series.object_ids)
                        if (body_key(id) == object_id)
                            add.body = id;
                const auto scene_only = quantity_id == "lost_impacts" || quantity_id == "lost_friction";
                builder.action_row("measure.runs.add", "Add", add, scene_only && add.body.is_valid() ? "Contact losses are measured for the whole scene only." : "");
            }
            if (model.runs.size() >= 2)
            {
                const auto default_a = std::to_string(model.runs[model.runs.size() - 2].number);
                const auto default_b = std::to_string(model.runs.back().number);
                auto a_choice = std::string(builder.view_value("measure.runs.compare_a", default_a));
                auto b_choice = std::string(builder.view_value("measure.runs.compare_b", default_b));
                const auto* a_ptr = run_number(model.runs, a_choice);
                const auto* b_ptr = run_number(model.runs, b_choice);
                if (!a_ptr)
                {
                    a_choice = default_a;
                    a_ptr = run_number(model.runs, a_choice);
                }
                if (!b_ptr)
                {
                    b_choice = default_b;
                    b_ptr = run_number(model.runs, b_choice);
                }
                ControlSpec compare_a = spec("measure.runs.compare_a");
                compare_a.options = run_options_;
                ControlSpec compare_b = spec("measure.runs.compare_b");
                compare_b.options = run_options_;
                builder.select_row(compare_a, a_choice, state_choice("measure.runs.compare_a"));
                builder.select_row(compare_b, b_choice, state_choice("measure.runs.compare_b"));
                const auto& a = *a_ptr;
                const auto& b = *b_ptr;
                builder.heading(core::substitute("Compare Run {} with Run {}", a.number, b.number));
                builder.value_row("Columns", "A · B · Δ · Δ %");
                for (std::size_t index = 0; index < model.pinned_values.size(); ++index)
                {
                    const auto comparison = compare_run_value(a, b, index);
                    if (!comparison.a || !comparison.b)
                        builder.value_row(quantity_label(model.pinned_values[index].quantity), "—");
                    else
                    {
                        const auto percent = comparison.delta_percent ? core::format_quantity(*comparison.delta_percent, core::DisplayQuantity::percentage, model.display_units) : "—";
                        builder.value_row(quantity_label(model.pinned_values[index].quantity), core::substitute("{} · {} · {} · {}", core::fixed(*comparison.a, 3), core::fixed(*comparison.b, 3), core::fixed(*comparison.delta, 3), percent));
                    }
                }
                const auto differences = setup_difference_count(a, b);
                if (differences > 1)
                    builder.notice("measure.runs.compare_note", Severity::warning, core::substitute("{} things changed between Run {} and Run {}", differences, a.number, b.number));
                auto show = action(UiCommandKind::none);
                show.detail = "show-run-in-graph:" + std::to_string(a.number);
                builder.action_row("measure.runs.show_graph", "Show in graph", show);
            }
            if (!model.runs.empty())
            {
                const auto confirm_clear = builder.view_value("measure.runs.confirm_clear", "false") == "true";
                if (!confirm_clear)
                    builder.action_row("measure.runs.clear", "Clear runs", state_value("measure.runs.confirm_clear", "true"));
                else
                {
                    const auto clear_count = static_cast<std::size_t>(std::count_if(model.runs.begin(), model.runs.end(), [](const auto& run)
                        {
                            return !run.starred;
                        }));
                    builder.notice("measure.runs.clear_confirm", Severity::warning, core::substitute("Clear {} run{}? Starred runs are kept.", clear_count, clear_count == 1 ? "" : "s"));
                    builder.action_row("measure.runs.clear_apply", "Clear", action(UiCommandKind::clear_runs));
                    builder.action_row("measure.runs.clear_cancel", "Cancel", state_value("measure.runs.confirm_clear", "false"));
                }
            }
        }
        else
        {
            builder.paragraph("These checks run separate reference experiments with fixed inputs.");
            builder.heading("Collisions vs theory");
            builder.paragraph("Compare simulated velocities and momentum with the theoretical result.");
            builder.action_row("measure.theory.collisions_run", "Run", action(UiCommandKind::compare_collisions));
            for (const auto& result : model.collision_comparison)
                builder.value_row(result.label, core::substitute("v₁ {} / {} · v₂ {} / {}", core::fixed(result.predicted_first_velocity_m_s, 3), core::fixed(result.measured_first_velocity_m_s, 3), core::fixed(result.predicted_second_velocity_m_s, 3), core::fixed(result.measured_second_velocity_m_s, 3)));
            builder.heading("Bounce rules vs theory");
            builder.paragraph("Measure the first rebound height for each bounce rule.");
            builder.action_row("measure.theory.bounce_run", "Run", action(UiCommandKind::compare_restitution));
            for (const auto& result : model.restitution_comparison)
                builder.value_row(result.mixing_name, core::substitute("height {} / {} m · error {} m", core::fixed(result.theoretical_rebound_height_m, 3), core::fixed(result.measured_rebound_height_m, 3), core::fixed(result.height_error_m, 3)));
            builder.heading("Integration accuracy");
            builder.paragraph("Measure energy drift in a harmonic oscillator over a fixed interval.");
            builder.action_row("measure.theory.integration_run", "Run", action(UiCommandKind::compare_integrators));
            for (const auto& result : model.energy_comparison)
                builder.value_row(result.integrator_name, core::substitute("final drift {} J · max {}%", core::fixed(result.final_drift_j, 5), core::fixed(result.maximum_relative_drift * 100.0, 3)));
        }
    }
}
