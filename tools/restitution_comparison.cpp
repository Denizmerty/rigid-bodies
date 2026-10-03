#include <rigidbodies/physics/restitution_comparison.hpp>

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
            throw std::invalid_argument("Expected a numeric time step or drop height");
        }
        return value;
    }
}

int main(int argc, char** argv)
{
    if (argc > 3 || (argc == 2 && std::string { argv[1] } == "--help"))
    {
        std::cout << "Usage: rigid_bodies_restitution_comparison [step_seconds] [drop_height_metres]\n"
                     "Compares four restitution mixing policies using identical frictionless drops.\n"
                     "Defaults: step 1/240 s, height 1 m, ball restitution 0.8, floor 0.2, CCD on.\n"
                     "Output is CSV; measured restitution is sqrt(rebound_height / drop_height).\n";
        return argc > 3 ? 1 : 0;
    }
    try
    {
        rigidbodies::physics::RestitutionComparisonSettings settings;
        if (argc >= 2)
        {
            settings.time_step_s = read_number(argv[1]);
        }
        if (argc >= 3)
        {
            settings.drop_height_m = read_number(argv[2]);
        }
        const auto reports = rigidbodies::physics::compare_restitution_drops(settings);
        std::cout << "mixing,steps,mixed_restitution,measured_restitution,drop_height_m,theoretical_rebound_m,measured_rebound_m,height_error_m,impact_speed_m_s,rebound_speed_m_s,impact_time_s,apex_time_s\n"
                  << std::setprecision(12);
        for (const auto& report : reports)
        {
            std::cout << report.mixing_name << ',' << report.step_count << ',' << report.mixed_restitution << ',' << report.measured_restitution << ','
                      << report.drop_height_m << ',' << report.theoretical_rebound_height_m << ',' << report.measured_rebound_height_m << ','
                      << report.height_error_m << ',' << report.impact_speed_m_s << ',' << report.rebound_speed_m_s << ','
                      << report.impact_time_s << ',' << report.apex_time_s << '\n';
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
