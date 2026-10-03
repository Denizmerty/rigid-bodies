#include <rigidbodies/app/simulation_session.hpp>
#include <algorithm>
#include <cmath>
#include <charconv>

namespace rigidbodies::app
{
    bool SimulationSession::apply_developer_command(const ui::UiCommand& command)
    {
        using K = ui::UiCommandKind;
        if (command.kind == K::set_physics_profiling)
        {
            world_.set_profiling_enabled(command.flag);
            return true;
        }
        if (command.kind == K::set_physics_workers)
        {
            if (std::isfinite(command.value) && command.value >= 0 && command.value <= 32 && std::floor(command.value) == command.value)
            {
                auto settings = world_.parallel_settings();
                settings.worker_count = static_cast<std::size_t>(command.value);
                world_.set_parallel_settings(settings);
            }
            return true;
        }
        if (command.kind == K::set_force_generator_enabled)
        {
            std::size_t index {};
            const auto parsed = std::from_chars(command.id.data(), command.id.data() + command.id.size(), index);
            if (parsed.ec != std::errc {} || parsed.ptr != command.id.data() + command.id.size())
                return true;
            if (command.body.is_valid() && !world_.is_valid(command.body))
                return true;
            const auto& generators = command.body.is_valid() ? world_.force_generators(command.body) : world_.force_generators();
            if (index >= generators.size() || !generators[index])
                return true;
            if (generators[index]->is_enabled() == command.flag)
                return true;
            generators[index]->set_enabled(command.flag);
            refresh_force_generators();
            mark_edit_changed();
            return true;
        }
        if (command.kind != K::set_solver_parameter)
            return false;
        const auto v = command.value;
        if (!std::isfinite(v))
            return true;
        auto settings = world_.settings();
        bool changed = false;
        const auto assign = [&](auto& target, auto value)
        {
            changed = changed || target != value;
            target = value;
        };
        if (command.id == "velocity_iterations" && v >= 1 && v <= 128 && std::floor(v) == v)
            assign(settings.solver.velocity_iterations, static_cast<int>(v));
        else if (command.id == "position_iterations" && v >= 0 && v <= 64 && std::floor(v) == v)
            assign(settings.solver.position_iterations, static_cast<int>(v));
        else if (command.id == "linear_slop_m" && v >= 0 && v <= 1)
            assign(settings.solver.linear_slop_m, v);
        else if (command.id == "position_correction_fraction" && v >= 0 && v <= 1)
            assign(settings.solver.position_correction_fraction, v);
        else if (command.id == "restitution_threshold_m_s" && v >= 0 && v <= 100)
            assign(settings.solver.restitution_threshold_m_s, v);
        else if (command.id == "maximum_position_correction_m" && v >= 0 && v <= 10)
            assign(settings.solver.maximum_position_correction_m, v);
        else
            return true;
        if (changed)
        {
            world_.set_settings(settings);
            mark_edit_changed();
        }
        return true;
    }
}
