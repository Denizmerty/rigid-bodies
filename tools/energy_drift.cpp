#include <rigidbodies/physics/energy_drift.hpp>

#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    double read_number(const char* argument)
    {
        const std::string text { argument };
        std::size_t consumed = 0;
        const auto value = std::stod(text, &consumed);
        if (consumed != text.size())
        {
            throw std::invalid_argument("Expected a numeric duration or time step");
        }
        return value;
    }
}

int main(int argc, char** argv)
{
    if (argc > 3 || (argc == 2 && std::string { argv[1] } == "--help"))
    {
        std::cout << "Usage: rigid_bodies_energy_drift [duration_seconds] [step_seconds]\n"
                     "Compares the same undamped harmonic oscillator using Euler, Verlet and RK4.\n"
                     "Defaults: duration 60 s, step 1/120 s. Output is CSV; no window is created.\n";
        return argc > 3 ? 1 : 0;
    }
    try
    {
        rigidbodies::physics::EnergyDriftSettings settings;
        if (argc >= 2)
        {
            settings.duration_s = read_number(argv[1]);
        }
        if (argc >= 3)
        {
            settings.time_step_s = read_number(argv[2]);
        }
        const auto reports = rigidbodies::physics::compare_harmonic_energy_drift(settings);
        std::cout << "integrator,steps,elapsed_s,initial_energy_j,final_energy_j,final_drift_j,max_abs_drift_j,max_relative_drift\n"
                  << std::setprecision(12);
        for (const auto& report : reports)
        {
            std::cout << report.integrator_name << ',' << report.step_count << ',' << report.elapsed_time_s << ','
                      << report.initial_energy_j << ',' << report.final_energy_j << ',' << report.final_drift_j << ','
                      << report.maximum_absolute_drift_j << ',' << report.maximum_relative_drift << '\n';
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
