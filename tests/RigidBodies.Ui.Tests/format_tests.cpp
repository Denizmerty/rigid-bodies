#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/text_format.hpp>

#include "test_framework.hpp"

#include <limits>
#include <string>

namespace
{
    using namespace rigidbodies::core;
    const std::string nbsp = "\xC2\xA0";
    const std::string thin = "\xE2\x80\x89";

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
