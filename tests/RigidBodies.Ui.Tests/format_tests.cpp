#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>

#include "test_framework.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace
{
    using namespace rigidbodies::core;
    const std::string nbsp = "\xC2\xA0";
    const std::string thin = "\xE2\x80\x89";
    const std::string minus = "\xE2\x88\x92";
    const std::string times_ten = " \xC3\x97 10";
    const std::string dash = "\xE2\x80\x94";
    const std::string middle_dot = "\xC2\xB7";

    // Superscript exponent text as the formatter writes it, so goldens stay readable.
    std::string power(int exponent)
    {
        static const std::array<std::string, 10> digits { "\xE2\x81\xB0", "\xC2\xB9", "\xC2\xB2", "\xC2\xB3", "\xE2\x81\xB4", "\xE2\x81\xB5", "\xE2\x81\xB6", "\xE2\x81\xB7", "\xE2\x81\xB8", "\xE2\x81\xB9" };
        std::string result = times_ten + (exponent < 0 ? "\xE2\x81\xBB" : "");
        for (const auto character : std::to_string(std::abs(exponent)))
            result += digits[static_cast<std::size_t>(character - '0')];
        return result;
    }

    // A minus sign in front of a figure that is all zeros, such as "−0.00".
    bool negative_zero(const std::string& text)
    {
        if (text.rfind(minus, 0) != 0)
            return false;
        for (std::size_t index = minus.size(); index < text.size(); ++index)
        {
            const auto character = text[index];
            if (character >= '1' && character <= '9')
                return false;
            if (character != '0' && character != '.')
                break;
        }
        return true;
    }

    // "1e-05", "inf" or "nan" as the C library would print them.
    bool machine_notation(const std::string& text)
    {
        for (std::size_t index = 1; index < text.size(); ++index)
        {
            if ((text[index] == 'e' || text[index] == 'E') && text[index - 1] >= '0' && text[index - 1] <= '9')
                return true;
        }
        return text.find("inf") != std::string::npos || text.find("nan") != std::string::npos;
    }

    // Uniform in [0, 1) from the portable 64-bit engine, so every standard library draws the same values.
    double unit_draw(std::mt19937_64& engine)
    {
        return std::ldexp(static_cast<double>(engine() >> 11), -53);
    }

    RIGIDBODIES_TEST("quantity input accepts either unit system and either minus sign")
    {
        const auto expect = [](std::string_view text, DisplayQuantity quantity, DisplayUnits units, double expected)
        {
            const auto parsed = parse_quantity(text, quantity, units);
            RIGIDBODIES_EXPECT(parsed.has_value(), std::string("parsed: ") + std::string(text));
            RIGIDBODIES_EXPECT_NEAR(*parsed, expected, 1.0e-12, std::string("SI value: ") + std::string(text));
        };
        expect("2 kg", DisplayQuantity::mass, DisplayUnits::si, 2.0);
        expect("200 g", DisplayQuantity::mass, DisplayUnits::si, 0.2);
        expect("200", DisplayQuantity::mass, DisplayUnits::centimetre_gram, 0.2);
        expect("30\xC2\xB0", DisplayQuantity::angle, DisplayUnits::si, 30.0);
        expect("-1.5", DisplayQuantity::coefficient, DisplayUnits::si, -1.5);
        expect("\xE2\x88\x92"
               "1.5",
            DisplayQuantity::coefficient,
            DisplayUnits::si,
            -1.5);
        expect("0.5 m/s", DisplayQuantity::velocity, DisplayUnits::centimetre_gram, 0.5);
        expect("50 cm/s", DisplayQuantity::velocity, DisplayUnits::si, 0.5);
        expect("1,25 m", DisplayQuantity::length, DisplayUnits::si, 1.25);
        expect("40 N/m", DisplayQuantity::stiffness, DisplayUnits::centimetre_gram, 40.0);
        expect("40000 dyn/cm", DisplayQuantity::stiffness, DisplayUnits::si, 40.0);
        expect("500 dyn\xC2\xB7s/cm", DisplayQuantity::damping, DisplayUnits::si, 0.5);
        expect("1.81e-5 Pa s", DisplayQuantity::viscosity, DisplayUnits::centimetre_gram, 1.81e-5);
        expect("8.33 ms", DisplayQuantity::time, DisplayUnits::si, 0.00833);
        RIGIDBODIES_EXPECT(!parse_quantity("abc", DisplayQuantity::mass, DisplayUnits::si), "words are rejected");
        RIGIDBODIES_EXPECT(!parse_quantity("2 m", DisplayQuantity::mass, DisplayUnits::si), "a mismatched unit is rejected");
        RIGIDBODIES_EXPECT(!parse_quantity("1+2", DisplayQuantity::count, DisplayUnits::si), "formulas are rejected");
    }

    RIGIDBODIES_TEST("the quantity table has stable SI and CGS golden text")
    {
        const auto si = DisplayUnits::si;
        const auto cgs = DisplayUnits::centimetre_gram;
        RIGIDBODIES_EXPECT(format_quantity(16.2, DisplayQuantity::time, si) == "16.20" + nbsp + "s", "time");
        RIGIDBODIES_EXPECT(format_quantity(16.2, DisplayQuantity::time, cgs) == "16.20" + nbsp + "s", "CGS time is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(0.302, DisplayQuantity::mass, si) == "0.302" + nbsp + "kg", "SI mass");
        RIGIDBODIES_EXPECT(format_quantity(0.302, DisplayQuantity::mass, cgs) == "302" + nbsp + "g", "CGS mass");
        RIGIDBODIES_EXPECT(format_quantity(0.008, DisplayQuantity::mass, si) == "8.0" + nbsp + "g", "small SI mass");
        RIGIDBODIES_EXPECT(format_quantity(0.004, DisplayQuantity::length, si) == "4.0" + nbsp + "mm", "small SI length");
        RIGIDBODIES_EXPECT(format_quantity(0.0, DisplayQuantity::length, si) == "0.00" + nbsp + "m", "zero length keeps the scene's metres");
        RIGIDBODIES_EXPECT(format_quantity(1.5, DisplayQuantity::length, cgs) == "150.0" + nbsp + "cm", "CGS length");
        RIGIDBODIES_EXPECT(format_quantity(-1.41, DisplayQuantity::velocity, si) == "\xE2\x88\x92"
                                                                                    "1.41" +
                    nbsp + "m/s",
            "velocity minus");
        RIGIDBODIES_EXPECT(format_quantity(-1.41, DisplayQuantity::velocity, cgs) == "\xE2\x88\x92"
                                                                                     "141.0" +
                    nbsp + "cm/s",
            "CGS velocity");
        RIGIDBODIES_EXPECT(format_quantity(9.80665, DisplayQuantity::acceleration, si) == "9.81" + nbsp + "m/s\xC2\xB2", "acceleration");
        RIGIDBODIES_EXPECT(format_quantity(9.80665, DisplayQuantity::acceleration, cgs) == "981" + nbsp + "cm/s\xC2\xB2", "CGS acceleration");
        RIGIDBODIES_EXPECT(format_quantity(-60.0, DisplayQuantity::angle, si, 1) == "\xE2\x88\x92"
                                                                                    "60.0\xC2\xB0",
            "angle field");
        RIGIDBODIES_EXPECT(format_quantity(-60.0, DisplayQuantity::angle, cgs, 1) == "\xE2\x88\x92"
                                                                                     "60.0\xC2\xB0",
            "CGS angle is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(20.0, DisplayQuantity::angular_velocity, si) == "20.00" + nbsp + "rad/s", "spin");
        RIGIDBODIES_EXPECT(format_quantity(20.0, DisplayQuantity::angular_velocity, cgs) == "20.00" + nbsp + "rad/s", "CGS spin is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(106.0, DisplayQuantity::energy, si) == "106" + nbsp + "J", "energy");
        RIGIDBODIES_EXPECT(format_quantity(106.0, DisplayQuantity::energy, cgs) == "1.06 \xC3\x97 10\xE2\x81\xB9" + nbsp + "erg", "CGS energy");
        RIGIDBODIES_EXPECT(format_quantity(4.9, DisplayQuantity::force, si) == "4.90" + nbsp + "N", "force");
        RIGIDBODIES_EXPECT(format_quantity(4.9, DisplayQuantity::force, cgs) == "490" + thin + "000" + nbsp + "dyn", "CGS force");
        RIGIDBODIES_EXPECT(format_quantity(4.1, DisplayQuantity::torque, si) == "4.10" + nbsp + "N\xC2\xB7m", "torque");
        RIGIDBODIES_EXPECT(format_quantity(4.1, DisplayQuantity::torque, cgs) == "4.10 \xC3\x97 10\xE2\x81\xB7" + nbsp + "dyn\xC2\xB7"
                                                                                                                         "cm",
            "CGS torque");
        RIGIDBODIES_EXPECT(format_quantity(2.0, DisplayQuantity::momentum, si) == "2.00" + nbsp + "kg\xC2\xB7m/s", "momentum");
        RIGIDBODIES_EXPECT(format_quantity(2.0, DisplayQuantity::momentum, cgs) == "200" + thin + "000" + nbsp + "g\xC2\xB7"
                                                                                                                 "cm/s",
            "CGS momentum");
        RIGIDBODIES_EXPECT(format_quantity(0.015, DisplayQuantity::inertia, si) == "0.0150" + nbsp + "kg\xC2\xB7m\xC2\xB2", "inertia");
        RIGIDBODIES_EXPECT(format_quantity(2.41e-4, DisplayQuantity::inertia, cgs) == "2410" + nbsp + "g\xC2\xB7"
                                                                                                      "cm\xC2\xB2",
            "CGS inertia");
        RIGIDBODIES_EXPECT(format_quantity(7850.0, DisplayQuantity::density, si) == "7850" + nbsp + "kg/m\xC2\xB3", "density");
        RIGIDBODIES_EXPECT(format_quantity(7850.0, DisplayQuantity::density, cgs) == "7.85" + nbsp + "g/cm\xC2\xB3", "CGS density");
        RIGIDBODIES_EXPECT(format_quantity(1.225, DisplayQuantity::air_density, si) == "1.225" + nbsp + "kg/m\xC2\xB3", "air density");
        RIGIDBODIES_EXPECT(format_quantity(1.225, DisplayQuantity::air_density, cgs) == "1.23 \xC3\x97 10\xE2\x81\xBB\xC2\xB3" + nbsp + "g/cm\xC2\xB3", "CGS air density");
        RIGIDBODIES_EXPECT(format_quantity(1.81e-5, DisplayQuantity::viscosity, si) == "1.81 \xC3\x97 10\xE2\x81\xBB\xE2\x81\xB5" + nbsp + "Pa\xC2\xB7s", "viscosity");
        RIGIDBODIES_EXPECT(format_quantity(1.81e-5, DisplayQuantity::viscosity, cgs) == "1.81 \xC3\x97 10\xE2\x81\xBB\xE2\x81\xB4" + nbsp + "g/(cm\xC2\xB7s)", "CGS viscosity");
        RIGIDBODIES_EXPECT(format_quantity(12000.0, DisplayQuantity::stiffness, si) == "12\xE2\x80\x89"
                                                                                       "000" +
                    nbsp + "N/m",
            "stiffness");
        RIGIDBODIES_EXPECT(format_quantity(40.0, DisplayQuantity::stiffness, cgs) == "40" + thin + "000" + nbsp + "dyn/cm", "CGS stiffness");
        RIGIDBODIES_EXPECT(format_quantity(0.5, DisplayQuantity::damping, si) == "0.500" + nbsp + "N\xC2\xB7s/m", "damping");
        RIGIDBODIES_EXPECT(format_quantity(0.5, DisplayQuantity::damping, cgs) == "500" + nbsp + "dyn\xC2\xB7s/cm", "CGS damping");
        RIGIDBODIES_EXPECT(format_quantity(1.0, DisplayQuantity::twist_stiffness, si) == "1.00" + nbsp + "N\xC2\xB7m/rad", "twist stiffness");
        RIGIDBODIES_EXPECT(format_quantity(1.0, DisplayQuantity::twist_stiffness, cgs) == "1.00 \xC3\x97 10\xE2\x81\xB7" + nbsp + "dyn\xC2\xB7"
                                                                                                                                  "cm/rad",
            "CGS twist stiffness");
        RIGIDBODIES_EXPECT(format_quantity(0.1, DisplayQuantity::twist_damping, si) == "0.100" + nbsp + "N\xC2\xB7m\xC2\xB7s/rad", "twist damping");
        RIGIDBODIES_EXPECT(format_quantity(0.1, DisplayQuantity::twist_damping, cgs) == "1.00 \xC3\x97 10\xE2\x81\xB6" + nbsp + "dyn\xC2\xB7"
                                                                                                                                "cm\xC2\xB7s/rad",
            "CGS twist damping");
        RIGIDBODIES_EXPECT(format_quantity(0.25, DisplayQuantity::power, si) == "0.250" + nbsp + "W", "power");
        RIGIDBODIES_EXPECT(format_quantity(0.25, DisplayQuantity::power, cgs) == "2.50 \xC3\x97 10\xE2\x81\xB6" + nbsp + "erg/s", "CGS power");
        RIGIDBODIES_EXPECT(format_quantity(0.3, DisplayQuantity::coefficient, si) == "0.30", "coefficient");
        RIGIDBODIES_EXPECT(format_quantity(0.3, DisplayQuantity::coefficient, cgs) == "0.30", "CGS coefficient is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(1.0, DisplayQuantity::multiplier, si) == "1\xC3\x97", "multiplier");
        RIGIDBODIES_EXPECT(format_quantity(0.875, DisplayQuantity::multiplier, si, 6) == "0.875\xC3\x97", "exact multipliers preserve requested precision without trailing zeroes");
        RIGIDBODIES_EXPECT(format_quantity(0.25, DisplayQuantity::multiplier, cgs) == "0.25\xC3\x97", "CGS multiplier is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(30.0, DisplayQuantity::percentage, si) == "30" + nbsp + "%", "percentage");
        RIGIDBODIES_EXPECT(format_quantity(0.012, DisplayQuantity::percentage, cgs) == "0.012" + nbsp + "%", "CGS percentage is unchanged");
        RIGIDBODIES_EXPECT(format_quantity(12400.0, DisplayQuantity::count, si) == "12\xE2\x80\x89"
                                                                                   "400",
            "count");
        RIGIDBODIES_EXPECT(format_quantity(12400.0, DisplayQuantity::count, cgs) == "12" + thin + "400", "CGS count is unchanged");
        RIGIDBODIES_EXPECT_NEAR(display_value(1.81e-5, DisplayQuantity::viscosity, cgs), 1.81e-4, 1.0e-15, "CGS viscosity conversion remains available to numeric consumers");
        RIGIDBODIES_EXPECT(display_value(40.0, DisplayQuantity::stiffness, cgs) == 40000.0, "CGS stiffness conversion remains available to numeric consumers");
        RIGIDBODIES_EXPECT(display_value(0.5, DisplayQuantity::damping, cgs) == 500.0, "CGS damping conversion remains available to numeric consumers");
    }

    RIGIDBODIES_TEST("formatter snaps zero and never leaks machine notation")
    {
        const auto si = DisplayUnits::si;
        RIGIDBODIES_EXPECT(format_quantity(1.05149e-28, DisplayQuantity::energy, si) == "0" + nbsp + "J", "tiny energy snaps to zero");
        RIGIDBODIES_EXPECT(format_quantity(0.0032, DisplayQuantity::energy, si) == "3.20" + nbsp + "mJ", "milli range");
        RIGIDBODIES_EXPECT(format_quantity(-0.0001, DisplayQuantity::velocity, si) == "0.00" + nbsp + "m/s", "fixed zero has no negative sign");
        RIGIDBODIES_EXPECT(format_quantity(2.41e-4, DisplayQuantity::inertia, si) == "2.41 \xC3\x97 10\xE2\x81\xBB\xE2\x81\xB4" + nbsp + "kg\xC2\xB7m\xC2\xB2", "inertia uses raised exponent");
        RIGIDBODIES_EXPECT(format_quantity(std::numeric_limits<double>::infinity(), DisplayQuantity::velocity, si) == "\xE2\x80\x94", "non-finite is unavailable dash only");
        for (int exponent = -30; exponent <= 30; ++exponent)
        {
            const auto text = format_quantity(std::pow(10.0, exponent), DisplayQuantity::energy, si);
            RIGIDBODIES_EXPECT(text.find("e+") == std::string::npos && text.find("e-") == std::string::npos, "magnitude sweep has no e notation");
            RIGIDBODIES_EXPECT(text.find("-0") == std::string::npos, "magnitude sweep has no negative zero");
        }
    }

    RIGIDBODIES_TEST("speed fractions keep their nines and truncate towards rest")
    {
        const auto expect = [](double fraction, const std::string& expected)
        {
            for (const auto units : { DisplayUnits::si, DisplayUnits::centimetre_gram })
            {
                const auto text = format_quantity(fraction, DisplayQuantity::speed_fraction, units);
                RIGIDBODIES_EXPECT(text == expected, "speed fraction " + expected + " reads " + text);
            }
        };
        expect(0.0, "0" + nbsp + "c");
        expect(1.0e-9, "0.000000001" + nbsp + "c");
        expect(0.0123, "0.0123" + nbsp + "c");
        expect(0.1234, "0.123" + nbsp + "c");
        expect(0.4999, "0.499" + nbsp + "c");
        expect(0.5, "0.5" + nbsp + "c");
        expect(0.6234567, "0.623" + nbsp + "c");
        expect(0.99, "0.99" + nbsp + "c");
        expect(0.99999, "0.99999" + nbsp + "c");
        expect(0.999995, "0.999995" + nbsp + "c");
        expect(0.9999949, "0.9999949" + nbsp + "c");
        expect(0.9999999, "0.9999999" + nbsp + "c");
        expect(std::nextafter(0.9999999, 0.0), "0.9999999" + nbsp + "c");
        expect(1.0 - 1.0e-12, "0.999999999999" + nbsp + "c");
        expect(-0.2, minus + "0.2" + nbsp + "c");
        expect(-0.0, "0" + nbsp + "c");
        expect(1.0, dash);
        expect(1.5, dash);
        expect(-1.0, dash);
        expect(std::numeric_limits<double>::quiet_NaN(), dash);
        expect(std::numeric_limits<double>::infinity(), dash);
        RIGIDBODIES_EXPECT(format_value(0.99999, DisplayQuantity::speed_fraction, DisplayUnits::si, 2) == "0.99999", "the precision argument is ignored");
        RIGIDBODIES_EXPECT(format_unit(0.5, DisplayQuantity::speed_fraction, DisplayUnits::si) == "c", "the unit is c");
        RIGIDBODIES_EXPECT(display_unit(DisplayQuantity::speed_fraction, DisplayUnits::si) == "c", "the displayed unit is c");
        RIGIDBODIES_EXPECT(display_unit(DisplayQuantity::speed_fraction, DisplayUnits::centimetre_gram) == "c", "CGS keeps c");
    }

    RIGIDBODIES_TEST("relativity quantities have stable SI and CGS golden text")
    {
        const auto si = DisplayUnits::si;
        const auto cgs = DisplayUnits::centimetre_gram;
        const auto both = [&](double value, DisplayQuantity quantity, const std::string& expected_si, const std::string& expected_cgs)
        {
            const auto text_si = format_quantity(value, quantity, si);
            const auto text_cgs = format_quantity(value, quantity, cgs);
            RIGIDBODIES_EXPECT(text_si == expected_si, "SI " + expected_si + " reads " + text_si);
            RIGIDBODIES_EXPECT(text_cgs == expected_cgs, "CGS " + expected_cgs + " reads " + text_cgs);
        };
        const auto same = [&](double value, DisplayQuantity quantity, const std::string& expected)
        {
            both(value, quantity, expected, expected);
        };

        same(0.0, DisplayQuantity::lorentz_factor_excess, "1");
        same(5.0e-19, DisplayQuantity::lorentz_factor_excess, "1 + 5.00" + power(-19));
        same(7.5654e-5, DisplayQuantity::lorentz_factor_excess, "1.000076");
        same(0.0101, DisplayQuantity::lorentz_factor_excess, "1.010");
        same(0.25, DisplayQuantity::lorentz_factor_excess, "1.25");
        same(0.1547, DisplayQuantity::lorentz_factor_excess, "1.155");
        same(1.2942, DisplayQuantity::lorentz_factor_excess, "2.294");
        same(6.0888, DisplayQuantity::lorentz_factor_excess, "7.089");
        same(222.607, DisplayQuantity::lorentz_factor_excess, "223.6");
        same(2235.07, DisplayQuantity::lorentz_factor_excess, "2236");
        same(-0.5, DisplayQuantity::lorentz_factor_excess, dash);

        both(0.99999 * speed_of_light_m_s, DisplayQuantity::relativistic_speed, "299" + thin + "789" + thin + "460" + nbsp + "m/s", "29" + thin + "978" + thin + "946" + thin + "007" + nbsp + "cm/s");
        both(0.9999999 * speed_of_light_m_s, DisplayQuantity::relativistic_speed, "299" + thin + "792" + thin + "428" + nbsp + "m/s", "29" + thin + "979" + thin + "242" + thin + "802" + nbsp + "cm/s");
        both(speed_of_light_m_s, DisplayQuantity::relativistic_speed, "299" + thin + "792" + thin + "458" + nbsp + "m/s", "29" + thin + "979" + thin + "245" + thin + "800" + nbsp + "cm/s");
        both(0.2998, DisplayQuantity::relativistic_speed, "0.300" + nbsp + "m/s", "30.0" + nbsp + "cm/s");
        both(1999.9, DisplayQuantity::relativistic_speed, "1" + thin + "999" + nbsp + "m/s", "199" + thin + "990" + nbsp + "cm/s");
        both(999.4, DisplayQuantity::relativistic_speed, "999" + nbsp + "m/s", "99" + thin + "940" + nbsp + "cm/s");
        both(999.7, DisplayQuantity::relativistic_speed, "999" + nbsp + "m/s", "99" + thin + "970" + nbsp + "cm/s");
        both(999.7, DisplayQuantity::speed_gap, "1" + thin + "000" + nbsp + "m/s", "99" + thin + "970" + nbsp + "cm/s");
        both(9.997, DisplayQuantity::relativistic_speed, "10.0" + nbsp + "m/s", "999" + nbsp + "cm/s");
        both(9.997, DisplayQuantity::speed_gap, "10.0" + nbsp + "m/s", "1" + thin + "000" + nbsp + "cm/s");
        both(0.0, DisplayQuantity::relativistic_speed, "0" + nbsp + "m/s", "0" + nbsp + "cm/s");
        both(0.0, DisplayQuantity::speed_gap, "0" + nbsp + "m/s", "0" + nbsp + "cm/s");

        RIGIDBODIES_EXPECT(format_quantity(2997.92, DisplayQuantity::speed_gap, si) == "2" + thin + "998" + nbsp + "m/s", "a gap is rounded up to the next whole metre per second");
        both(2997.9246, DisplayQuantity::speed_gap, "2" + thin + "998" + nbsp + "m/s", "299" + thin + "793" + nbsp + "cm/s");
        both(29.979, DisplayQuantity::speed_gap, "30.0" + nbsp + "m/s", "2" + thin + "998" + nbsp + "cm/s");

        both(2.000695e19, DisplayQuantity::relativistic_energy, "2.00" + power(19) + nbsp + "J", "2.00" + power(26) + nbsp + "erg");
        both(8.987552e16, DisplayQuantity::relativistic_energy, "8.99" + power(16) + nbsp + "J", "8.99" + power(23) + nbsp + "erg");
        both(0.0449, DisplayQuantity::relativistic_energy, "0.0449" + nbsp + "J", "4.49" + power(5) + nbsp + "erg");
        both(0.0, DisplayQuantity::relativistic_energy, "0" + nbsp + "J", "0" + nbsp + "erg");
        both(1.234e-4, DisplayQuantity::relativistic_energy, "1.23" + power(-4) + nbsp + "J", "1234" + nbsp + "erg");
        both(9999.7, DisplayQuantity::relativistic_energy, "1.00" + power(4) + nbsp + "J", "1.00" + power(11) + nbsp + "erg");

        both(6.703513e10, DisplayQuantity::relativistic_momentum, "6.70" + power(10) + nbsp + "kg" + middle_dot + "m/s", "6.70" + power(15) + nbsp + "g" + middle_dot + "cm/s");
        both(0.2998, DisplayQuantity::relativistic_momentum, "0.300" + nbsp + "kg" + middle_dot + "m/s", "3.00" + power(4) + nbsp + "g" + middle_dot + "cm/s");

        same(0.0, DisplayQuantity::fine_time, "0.00" + nbsp + "ns");
        same(7.3149e-9, DisplayQuantity::fine_time, "7.31" + nbsp + "ns");
        same(1.08e-5, DisplayQuantity::fine_time, "10" + thin + "800.00" + nbsp + "ns");
        same(3.6e-6, DisplayQuantity::fine_time, "3" + thin + "600.00" + nbsp + "ns");
        same(-3.6e-6, DisplayQuantity::fine_time, minus + "3" + thin + "600.00" + nbsp + "ns");
        same(9.9999e-7, DisplayQuantity::fine_time, "999.99" + nbsp + "ns");
        same(4.4721e-11, DisplayQuantity::fine_time, "0.0447" + nbsp + "ns");
        same(5.568e-11, DisplayQuantity::fine_time, "0.0557" + nbsp + "ns");
        same(4.47e-13, DisplayQuantity::fine_time, "0.000447" + nbsp + "ns");
        same(9.996e-10, DisplayQuantity::fine_time, "1.00" + nbsp + "ns");
        same(-7.3149e-9, DisplayQuantity::fine_time, minus + "7.31" + nbsp + "ns");
        same(12.45 * 1.0e-9, DisplayQuantity::fine_time, "12.45" + nbsp + "ns");

        same(0.0, DisplayQuantity::duration, "0" + nbsp + "ns");
        same(1.1111e-9, DisplayQuantity::duration, "1.11" + nbsp + "ns");
        same(1.0e-8, DisplayQuantity::duration, "10.00" + nbsp + "ns");
        same(1.0101e-10, DisplayQuantity::duration, "101" + nbsp + "ps");
        same(1.0e-13, DisplayQuantity::duration, "100" + nbsp + "fs");
        same(1.0e-15, DisplayQuantity::duration, "1.00" + nbsp + "fs");
        same(5.0e-23, DisplayQuantity::duration, "5.00" + power(-23) + nbsp + "s");
        same(9.996e-10, DisplayQuantity::duration, "1.00" + nbsp + "ns");
        same(9.996e-13, DisplayQuantity::duration, "1.00" + nbsp + "ps");
        same(9.9996e-16, DisplayQuantity::duration, "1.00" + nbsp + "fs");
        // Grouped from four digits, as the clock readings it is listed under are.
        same(3.59946e-6, DisplayQuantity::duration, "3" + thin + "599.46" + nbsp + "ns");
        same(3.6e-6, DisplayQuantity::fine_time, "3" + thin + "600.00" + nbsp + "ns");
        same(1.2345678e-5, DisplayQuantity::duration, "12" + thin + "345.68" + nbsp + "ns");

        both(0.0, DisplayQuantity::small_length, "0" + nbsp + "m", "0" + nbsp + "cm");
        both(1.499, DisplayQuantity::small_length, "1.50" + nbsp + "m", "150" + nbsp + "cm");
        both(1.499e-5, DisplayQuantity::small_length, "15.0" + nbsp + "\xC2\xB5m", "1.50" + power(-3) + nbsp + "cm");
        both(1.5e-7, DisplayQuantity::small_length, "150" + nbsp + "nm", "1.50" + power(-5) + nbsp + "cm");
        both(2.5e-3, DisplayQuantity::small_length, "2.50" + nbsp + "mm", "2.50" + power(-1) + nbsp + "cm");
        both(4.0e-11, DisplayQuantity::small_length, "4.00" + power(-11) + nbsp + "m", "4.00" + power(-9) + nbsp + "cm");
        both(9.9996e-4, DisplayQuantity::small_length, "1.00" + nbsp + "mm", "1.00" + power(-1) + nbsp + "cm");

        RIGIDBODIES_EXPECT(display_unit(DisplayQuantity::fine_time, si) == "ns" && display_unit(DisplayQuantity::fine_time, cgs) == "ns", "clock readings are in nanoseconds in both systems");
        RIGIDBODIES_EXPECT(display_unit(DisplayQuantity::lorentz_factor_excess, si).empty(), "the Lorentz factor has no unit");
    }

    RIGIDBODIES_TEST("a speed and its gap below c add up to c in whole units")
    {
        const auto whole = [](double value_si, DisplayQuantity quantity, DisplayUnits units)
        {
            auto text = format_value(value_si, quantity, units);
            for (auto at = text.find(thin); at != std::string::npos; at = text.find(thin))
                text.erase(at, thin.size());
            RIGIDBODIES_EXPECT(text.find('.') == std::string::npos, "a whole number: " + text);
            return std::stoll(text);
        };
        const auto add_up = [&](double fraction)
        {
            for (const auto units : { DisplayUnits::si, DisplayUnits::centimetre_gram })
            {
                const auto scale = units == DisplayUnits::si ? 1LL : 100LL;
                const auto gap_si = (1.0 - fraction) * speed_of_light_m_s;
                if (gap_si * static_cast<double>(scale) < 1000.0)
                    continue;
                const auto speed = whole(fraction * speed_of_light_m_s, DisplayQuantity::relativistic_speed, units);
                const auto gap = whole(gap_si, DisplayQuantity::speed_gap, units);
                RIGIDBODIES_EXPECT(speed + gap == 299792458LL * scale, "speed and gap add up to c at " + format_quantity(fraction, DisplayQuantity::speed_fraction, DisplayUnits::si));
            }
        };
        for (const auto fraction : { 0.5, 0.6, 0.9, 0.99, 0.999, 0.9999, 0.99999, 0.999999, 0.9999999 })
            add_up(fraction);
        // A typed whole speed comes back through β = v/c with an error of about c·2⁻⁵³, so a small
        // gap must allow for c's rounding, not its own.
        for (const auto typed : { "299792158 m/s", "299787978 m/s", "299789120 m/s", "299789 km/s", "29979223300 cm/s" })
        {
            const auto fraction = parse_quantity(typed, DisplayQuantity::speed_fraction, DisplayUnits::si);
            RIGIDBODIES_EXPECT(fraction.has_value(), std::string("parsed: ") + typed);
            add_up(*fraction);
        }
        std::mt19937_64 engine { 299792458 };
        for (int sample = 0; sample < 10000; ++sample)
        {
            const auto typed = 150000000.0 + static_cast<double>(engine() % 149792458ULL);
            add_up(typed / speed_of_light_m_s);
        }
        RIGIDBODIES_EXPECT(format_quantity((1.0 - 0.99) * speed_of_light_m_s, DisplayQuantity::speed_gap, DisplayUnits::centimetre_gram) == "299" + thin + "792" + thin + "458" + nbsp + "cm/s", "rounding noise does not add a centimetre per second");
        RIGIDBODIES_EXPECT(format_quantity(1000.5, DisplayQuantity::speed_gap, DisplayUnits::si) == "1" + thin + "001" + nbsp + "m/s", "a real fraction is still rounded up");
        RIGIDBODIES_EXPECT(format_quantity(1000.5, DisplayQuantity::relativistic_speed, DisplayUnits::si) == "1" + thin + "000" + nbsp + "m/s", "a real fraction is still cut");
    }

    RIGIDBODIES_TEST("a gap below 1 000 m/s is rounded up in its last figure and still adds up to c")
    {
        // The figure the formatter shows, its thin spaces removed.
        const auto shown = [](double value_si, DisplayQuantity quantity)
        {
            auto text = format_value(value_si, quantity, DisplayUnits::si);
            for (auto at = text.find(thin); at != std::string::npos; at = text.find(thin))
                text.erase(at, thin.size());
            return std::stod(text);
        };
        // Every speed the keyboard and slider reach from 0.9999966 c to the maximum: seven
        // decimals up to 0.999999, then eight. The whole speed and the gap add up to c exactly.
        const auto two_digits = [](int value)
        {
            return std::string(value < 10 ? "0" : "") + std::to_string(value);
        };
        std::vector<std::string> reachable;
        for (int step = 1; step <= 34; ++step)
            reachable.push_back("0.99999" + two_digits(100 - step));
        for (int step = 10; step <= 99; ++step)
            reachable.push_back("0.999999" + two_digits(100 - step));
        for (const auto& text : reachable)
        {
            const auto fraction = std::stod(text);
            const auto gap_si = (1.0 - fraction) * speed_of_light_m_s;
            const auto speed = shown(fraction * speed_of_light_m_s, DisplayQuantity::relativistic_speed);
            const auto gap = shown(gap_si, DisplayQuantity::speed_gap);
            RIGIDBODIES_EXPECT(gap >= gap_si - 1.0e-6, "the gap is never understated at " + text + " c: " + std::to_string(gap) + " for " + std::to_string(gap_si));
            RIGIDBODIES_EXPECT(std::abs(speed + gap - speed_of_light_m_s) < 1.0e-6, "speed and gap add up to c at " + text + " c: " + std::to_string(speed) + " + " + std::to_string(gap));
        }
        RIGIDBODIES_EXPECT(format_quantity((1.0 - 0.9999971) * speed_of_light_m_s, DisplayQuantity::speed_gap, DisplayUnits::si) == "870" + nbsp + "m/s", "869.4 m/s reads 870 beside the whole speed 299 791 588");
        RIGIDBODIES_EXPECT(format_quantity((1.0 - 0.9999999) * speed_of_light_m_s, DisplayQuantity::speed_gap, DisplayUnits::si) == "30.0" + nbsp + "m/s", "the gap at the maximum reads 30.0 m/s");
        RIGIDBODIES_EXPECT(format_quantity(999.4, DisplayQuantity::speed_gap, DisplayUnits::si) == "1" + thin + "000" + nbsp + "m/s", "a gap rounded up to 1000 is grouped as a whole number");
        RIGIDBODIES_EXPECT(format_quantity(87.0, DisplayQuantity::speed_gap, DisplayUnits::si) == "87.0" + nbsp + "m/s" && format_quantity(87.0 + 1.0e-5, DisplayQuantity::speed_gap, DisplayUnits::si) == "87.0" + nbsp + "m/s", "rounding noise in a gap is not rounded up");
        RIGIDBODIES_EXPECT(format_quantity(869.4, DisplayQuantity::relativistic_speed, DisplayUnits::si) == "869" + nbsp + "m/s", "a slow speed is still rounded to three figures");
        // Typed speeds keep the guarantee that matters: the gap is never understated.
        std::mt19937_64 engine { 869 };
        for (int sample = 0; sample < 10000; ++sample)
        {
            const auto gap_si = 29.98 + static_cast<double>(engine() % 970000ULL) / 1000.0;
            RIGIDBODIES_EXPECT(shown(gap_si, DisplayQuantity::speed_gap) >= gap_si - 1.0e-3, "a gap is never understated: " + std::to_string(gap_si));
        }
    }

    RIGIDBODIES_TEST("the decimals of a speed fraction follow its text")
    {
        RIGIDBODIES_EXPECT(speed_fraction_decimals(0.99999) == 6 && speed_fraction_decimals(0.9999) == 5 && speed_fraction_decimals(0.9999999) == 8, "near c two figures of the gap");
        RIGIDBODIES_EXPECT(speed_fraction_decimals(0.123) == 3 && speed_fraction_decimals(0.0123) == 4 && speed_fraction_decimals(1.0e-9) == 11 && speed_fraction_decimals(0.001) == 5, "below half c three significant figures");
        RIGIDBODIES_EXPECT(speed_fraction_decimals(0.5) == 3 && speed_fraction_decimals(0.0) == 3 && speed_fraction_decimals(1.0) == 3, "at least three, and three outside the open interval");
        for (const auto fraction : { 0.5, 0.6, 0.99, 0.999, 0.99999, 0.9999949, 0.9999999, 0.1234, 0.0123, 1.0e-9 })
        {
            const auto text = format_value(fraction, DisplayQuantity::speed_fraction, DisplayUnits::si);
            const auto decimals = static_cast<int>(text.size() - text.find('.') - 1);
            RIGIDBODIES_EXPECT(decimals <= speed_fraction_decimals(fraction), "the text is worked to its decimals before zeros are trimmed: " + text);
        }
    }

    RIGIDBODIES_TEST("relativity display factors read bare numbers in the displayed unit")
    {
        const auto si = DisplayUnits::si;
        const auto cgs = DisplayUnits::centimetre_gram;
        for (const auto units : { si, cgs })
        {
            RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::fine_time, units) == 1.0e9, "a bare clock reading is in nanoseconds");
            RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::duration, units) == 1.0e9, "a bare interval is in nanoseconds");
            RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::speed_fraction, units) == 1.0, "a speed fraction is a pure number");
            RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::lorentz_factor_excess, units) == 1.0, "a Lorentz factor is a pure number");
        }
        for (const auto quantity : { DisplayQuantity::relativistic_speed, DisplayQuantity::speed_gap, DisplayQuantity::relativistic_energy, DisplayQuantity::relativistic_momentum, DisplayQuantity::small_length })
            RIGIDBODIES_EXPECT(display_factor(quantity, si) == 1.0, "SI relativity quantities are in base units");
        RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::relativistic_speed, cgs) == 100.0, "CGS speed");
        RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::speed_gap, cgs) == 100.0, "CGS speed gap");
        RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::relativistic_energy, cgs) == 1.0e7, "CGS energy");
        RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::relativistic_momentum, cgs) == 1.0e5, "CGS momentum");
        RIGIDBODIES_EXPECT(display_factor(DisplayQuantity::small_length, cgs) == 100.0, "CGS small length");
        RIGIDBODIES_EXPECT(speed_of_light_m_s == 299792458.0, "Core's c is the defined SI value");
    }

    RIGIDBODIES_TEST("relativity input accepts c, percentages, speeds and short times")
    {
        const auto si = DisplayUnits::si;
        const auto cgs = DisplayUnits::centimetre_gram;
        const auto expect = [](std::string_view text, DisplayQuantity quantity, DisplayUnits units, double expected, double tolerance)
        {
            const auto parsed = parse_quantity(text, quantity, units);
            RIGIDBODIES_EXPECT(parsed.has_value(), std::string("parsed: ") + std::string(text));
            RIGIDBODIES_EXPECT_NEAR(*parsed, expected, tolerance, std::string("SI value: ") + std::string(text));
        };
        const auto fraction = DisplayQuantity::speed_fraction;
        expect("0.6", fraction, si, 0.6, 1.0e-15);
        expect("0.6", fraction, cgs, 0.6, 1.0e-15);
        expect("60 %", fraction, si, 0.6, 1.0e-15);
        expect("60 % c", fraction, si, 0.6, 1.0e-15);
        expect("60%c", fraction, si, 0.6, 1.0e-15);
        expect("0.5 c", fraction, si, 0.5, 0.0);
        expect("0.5" + nbsp + "c", fraction, si, 0.5, 0.0);
        expect("0.5 C", fraction, si, 0.5, 0.0);
        expect("99.999%", fraction, si, 0.99999, 1.0e-15);
        expect("150000 km/s", fraction, si, 1.5e8 / speed_of_light_m_s, 1.0e-15);
        expect("150000 km/s", fraction, si, 0.500346, 1.0e-6);
        expect("299792158 m/s", fraction, si, 1.0 - 300.0 / speed_of_light_m_s, 1.0e-15);
        expect("29979245800 cm/s", fraction, si, 1.0, 1.0e-15);
        expect("1 c", fraction, si, 1.0, 0.0);
        expect(minus + "0.2", fraction, si, -0.2, 0.0);
        RIGIDBODIES_EXPECT(!parse_quantity("c", fraction, si), "c alone is not a number");
        RIGIDBODIES_EXPECT(!parse_quantity("0.5 kg", fraction, si), "a mismatched unit is rejected");

        for (const auto quantity : { DisplayQuantity::fine_time, DisplayQuantity::duration })
        {
            for (const auto units : { si, cgs })
            {
                expect("5", quantity, units, 5.0e-9, 1.0e-24);
                expect("5 \xC2\xB5s", quantity, units, 5.0e-6, 1.0e-21);
                expect("5 us", quantity, units, 5.0e-6, 1.0e-21);
                expect("2 ms", quantity, units, 2.0e-3, 1.0e-18);
                expect("1.5 s", quantity, units, 1.5, 0.0);
                expect("3 ns", quantity, units, 3.0e-9, 1.0e-24);
                expect("40 ps", quantity, units, 4.0e-11, 1.0e-26);
                expect("7 fs", quantity, units, 7.0e-15, 1.0e-30);
            }
            RIGIDBODIES_EXPECT(!parse_quantity("5 m", quantity, si), "a length is not a time");
        }

        const auto small = DisplayQuantity::small_length;
        expect("15 \xC2\xB5m", small, si, 1.5e-5, 1.0e-20);
        expect("15 um", small, si, 1.5e-5, 1.0e-20);
        expect("2 mm", small, si, 2.0e-3, 1.0e-18);
        expect("3 nm", small, si, 3.0e-9, 1.0e-24);
        expect("4 cm", small, si, 0.04, 1.0e-17);
        expect("1 m", small, cgs, 1.0, 0.0);
        expect("15", small, si, 15.0, 0.0);
        expect("15", small, cgs, 0.15, 1.0e-16);
        RIGIDBODIES_EXPECT(!parse_quantity("15 ns", small, si), "a time is not a length");

        expect("300 m/s", DisplayQuantity::relativistic_speed, cgs, 300.0, 0.0);
        expect("30000 cm/s", DisplayQuantity::relativistic_speed, si, 300.0, 1.0e-12);
        expect("30000", DisplayQuantity::speed_gap, cgs, 300.0, 1.0e-12);
        expect("2 erg", DisplayQuantity::relativistic_energy, si, 2.0e-7, 1.0e-22);
        expect("5 J", DisplayQuantity::relativistic_energy, cgs, 5.0, 0.0);
        expect("2 kg" + middle_dot + "m/s", DisplayQuantity::relativistic_momentum, cgs, 2.0, 0.0);
        expect("2 g" + middle_dot + "cm/s", DisplayQuantity::relativistic_momentum, si, 2.0e-5, 1.0e-20);
        RIGIDBODIES_EXPECT(!parse_quantity("5 MJ", DisplayQuantity::relativistic_energy, si), "no mega prefix is accepted");
    }

    RIGIDBODIES_TEST("speed fractions never read as light speed")
    {
        const auto check = [](double fraction)
        {
            const auto text = format_value(fraction, DisplayQuantity::speed_fraction, DisplayUnits::si);
            RIGIDBODIES_EXPECT(!text.empty() && text.front() != '1', "never 1: " + text);
            RIGIDBODIES_EXPECT(!machine_notation(text), "no machine notation: " + text);
            const auto parsed = parse_quantity(format_quantity(fraction, DisplayQuantity::speed_fraction, DisplayUnits::si), DisplayQuantity::speed_fraction, DisplayUnits::si);
            RIGIDBODIES_EXPECT(parsed.has_value() && *parsed < 1.0, "the text parses below c: " + text);
            RIGIDBODIES_EXPECT(*parsed <= fraction + 1.0e-12, "the text never overstates the speed: " + text);
            // Up to ten decimals the double nearest the printed text lies well inside its last
            // digit, so formatting the text again gives the same text.
            if (1.0 - fraction >= 1.0e-9)
            {
                const auto again = format_value(*parsed, DisplayQuantity::speed_fraction, DisplayUnits::si);
                RIGIDBODIES_EXPECT(again == text, "formatting is idempotent: " + text + " then " + again);
            }
        };
        for (int nines = 1; nines <= 15; ++nines)
        {
            const auto fraction = 1.0 - std::pow(10.0, -nines);
            check(fraction);
            check(std::nextafter(fraction, 0.0));
            if (std::nextafter(fraction, 2.0) < 1.0)
                check(std::nextafter(fraction, 2.0));
        }
        check(std::nextafter(1.0, 0.0));
        check(0.9999999);
        check(std::nextafter(0.9999999, 0.0));
        std::mt19937_64 engine { 20261005 };
        for (int sample = 0; sample < 10000; ++sample)
        {
            const auto uniform = unit_draw(engine);
            check(sample % 2 == 0 ? uniform : 1.0 - std::pow(10.0, -7.0 * uniform));
        }
    }

    RIGIDBODIES_TEST("a speed below c never reads as c")
    {
        for (const auto units : { DisplayUnits::si, DisplayUnits::centimetre_gram })
        {
            const auto light = format_quantity(speed_of_light_m_s, DisplayQuantity::relativistic_speed, units);
            const auto check = [&](double fraction)
            {
                const auto text = format_quantity(fraction * speed_of_light_m_s, DisplayQuantity::relativistic_speed, units);
                RIGIDBODIES_EXPECT(text != light, "a speed below c reads as c: " + text);
                const auto parsed = parse_quantity(text, DisplayQuantity::relativistic_speed, units);
                RIGIDBODIES_EXPECT(parsed.has_value() && *parsed < speed_of_light_m_s, "the text parses below c: " + text);
            };
            for (int nines = 1; nines <= 15; ++nines)
            {
                const auto fraction = 1.0 - std::pow(10.0, -nines);
                check(fraction);
                check(std::nextafter(fraction, 0.0));
                if (std::nextafter(fraction, 2.0) < 1.0)
                    check(std::nextafter(fraction, 2.0));
            }
            check(std::nextafter(1.0, 0.0));
        }
        const auto fraction = 1.0 - 1.0e-12;
        RIGIDBODIES_EXPECT(format_quantity(fraction * speed_of_light_m_s, DisplayQuantity::relativistic_speed, DisplayUnits::si) == "299" + thin + "792" + thin + "457" + nbsp + "m/s", "a hair below c is cut to the metre per second below");
        RIGIDBODIES_EXPECT(format_quantity(fraction * speed_of_light_m_s, DisplayQuantity::relativistic_speed, DisplayUnits::centimetre_gram) == "29" + thin + "979" + thin + "245" + thin + "799" + nbsp + "cm/s", "and to the centimetre per second below in CGS");
    }

    RIGIDBODIES_TEST("mantissa rounding carries into the next power of ten")
    {
        const auto si = DisplayUnits::si;
        const auto cgs = DisplayUnits::centimetre_gram;
        RIGIDBODIES_EXPECT(format_quantity(9.9996e-4, DisplayQuantity::inertia, si) == "1.00" + power(-3) + nbsp + "kg\xC2\xB7m\xC2\xB2", "inertia carries to 10⁻³");
        RIGIDBODIES_EXPECT(format_quantity(-9.9996e-4, DisplayQuantity::inertia, si) == minus + "1.00" + power(-3) + nbsp + "kg\xC2\xB7m\xC2\xB2", "a negative mantissa carries alike");
        RIGIDBODIES_EXPECT(format_quantity(0.99996, DisplayQuantity::energy, cgs) == "1.00" + power(7) + nbsp + "erg", "CGS energy carries to 10⁷");
        RIGIDBODIES_EXPECT(format_quantity(9.9996e18, DisplayQuantity::relativistic_energy, si) == "1.00" + power(19) + nbsp + "J", "relativistic energy carries to 10¹⁹");
        RIGIDBODIES_EXPECT(format_quantity(9.99996e-17, DisplayQuantity::duration, si) == "1.00" + power(-16) + nbsp + "s", "a tiny duration carries to 10⁻¹⁶");
        RIGIDBODIES_EXPECT(format_significant(9.9996e-4) == "1.00" + power(-3), "format_significant carries to 10⁻³");
        RIGIDBODIES_EXPECT(format_quantity(2.41e-4, DisplayQuantity::inertia, si) == "2.41" + power(-4) + nbsp + "kg\xC2\xB7m\xC2\xB2", "a mantissa that does not carry is unchanged");
    }

    RIGIDBODIES_TEST("format_significant keeps three figures and raises the rest")
    {
        RIGIDBODIES_EXPECT(format_significant(0.43589) == "0.436", "0.436");
        RIGIDBODIES_EXPECT(format_significant(0.0044721) == "0.00447", "0.00447");
        RIGIDBODIES_EXPECT(format_significant(4.47e-4) == "4.47" + power(-4), "below a thousandth");
        RIGIDBODIES_EXPECT(format_significant(445.22) == "445", "445");
        RIGIDBODIES_EXPECT(format_significant(1.0) == "1.00", "one keeps three figures");
        RIGIDBODIES_EXPECT(format_significant(0.0) == "0", "zero");
        RIGIDBODIES_EXPECT(format_significant(-0.0) == "0", "no negative zero");
        RIGIDBODIES_EXPECT(format_significant(-0.43589) == minus + "0.436", "U+2212 minus");
        RIGIDBODIES_EXPECT(format_significant(2.5e6) == "2.50" + power(6), "a million and above");
        RIGIDBODIES_EXPECT(format_significant(999999.7) == "1.00" + power(6), "rounding up to a million is raised");
        RIGIDBODIES_EXPECT(format_significant(12345.6) == "12" + thin + "346", "large figures are grouped");
        RIGIDBODIES_EXPECT(format_significant(0.99996) == "1.00", "rounding that carries keeps three figures");
        RIGIDBODIES_EXPECT(format_significant(0.123456, 5) == "0.12346", "five figures");
        RIGIDBODIES_EXPECT(format_significant(std::numeric_limits<double>::quiet_NaN()) == dash, "non-finite is a dash");
    }

    RIGIDBODIES_TEST("relativity quantities never leak machine notation")
    {
        const std::array<DisplayQuantity, 9> quantities { DisplayQuantity::speed_fraction, DisplayQuantity::lorentz_factor_excess, DisplayQuantity::relativistic_speed, DisplayQuantity::speed_gap, DisplayQuantity::relativistic_energy, DisplayQuantity::relativistic_momentum, DisplayQuantity::fine_time, DisplayQuantity::duration, DisplayQuantity::small_length };
        std::vector<double> values;
        for (int exponent = -30; exponent <= 30; ++exponent)
        {
            for (const auto mantissa : { 1.0, 0.99996, 1.2345 })
                values.push_back(mantissa * std::pow(10.0, exponent));
        }
        // Below the smallest normal double a power of ten underflows to zero, which must not
        // leave "inf" in a mantissa.
        for (const auto tiny : { std::numeric_limits<double>::denorm_min(), 1.0e-323, 1.0e-310, std::numeric_limits<double>::min() })
            values.push_back(tiny);
        const auto count = values.size();
        for (std::size_t index = 0; index < count; ++index)
            values.push_back(-values[index]);
        const auto clean = [](const std::string& text)
        {
            RIGIDBODIES_EXPECT(!text.empty(), "every value has text");
            RIGIDBODIES_EXPECT(!machine_notation(text), "no machine notation: " + text);
            RIGIDBODIES_EXPECT(text.find('-') == std::string::npos, "no ASCII minus: " + text);
            RIGIDBODIES_EXPECT(!negative_zero(text), "no negative zero: " + text);
        };
        for (const auto value : values)
        {
            for (const auto quantity : quantities)
            {
                for (const auto units : { DisplayUnits::si, DisplayUnits::centimetre_gram })
                    clean(format_quantity(value, quantity, units));
            }
            clean(format_significant(value));
        }
        RIGIDBODIES_EXPECT(format_significant(std::numeric_limits<double>::denorm_min()) == "4.94" + power(-324), "the smallest double keeps its figures");
        RIGIDBODIES_EXPECT(format_quantity(1.0e-323, DisplayQuantity::lorentz_factor_excess, DisplayUnits::si) == "1 + 9.88" + power(-324), "a Lorentz factor a hair above one keeps its excess");
        RIGIDBODIES_EXPECT(format_quantity(std::numeric_limits<double>::min(), DisplayQuantity::duration, DisplayUnits::si) == "2.23" + power(-308) + nbsp + "s", "the smallest normal duration");
    }

    RIGIDBODIES_TEST("identifiers become sentence-case fallback names")
    {
        RIGIDBODIES_EXPECT(humanise_identifier("wooden_ball") == "Wooden ball", "underscores become spaces and the first letter is capitalised");
        RIGIDBODIES_EXPECT(humanise_identifier("").empty(), "empty identifier stays available for Object n fallback");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
