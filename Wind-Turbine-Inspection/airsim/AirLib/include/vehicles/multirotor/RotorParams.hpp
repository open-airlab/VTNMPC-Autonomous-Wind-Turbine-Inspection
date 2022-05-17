// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_RotorParams_hpp
#define msr_airlib_RotorParams_hpp

#include "common/Common.hpp"

namespace msr
{
namespace airlib
{

    //In NED system, +ve torque would generate clockwise rotation
    enum class RotorTurningDirection : int
    {
        RotorTurningDirectionCCW = -1,
        RotorTurningDirectionCW = 1
    };

    struct RotorParams
    {
        /*
            Ref: http://physics.stackexchange.com/a/32013/14061
            force in Newton = C_T * \rho * n^2 * D^4
            torque in N.m = C_P * \rho * n^2 * D^5 / (2*pi)
            where,
            \rho = air density (1.225 kg/m^3)
            n = revolutions per sec
            D = propeller diameter in meters
            C_T, C_P = dimensionless constants available at
            propeller performance database http://m-selig.ae.illinois.edu/props/propDB.html

            We use values for GWS 9X5 propeller for which,
            C_T = 0.109919, C_P = 0.040164 @ 6396.667 RPM

            Motor: https://www.dji.com/dk/e1200-standard/info#specs
            C_T = 21.00 / (1.225 * (8060/60)² * 0.5334⁴)

            Propeller spec:
            Diameter / Thread Pitch	13×4.5 inch
            Weight	19 g
            */
        real_T C_T = 0.073321f; // the thrust co-efficient @ 6396.667 RPM, measured by UIUC.0.02137320084
        real_T C_P = 0.021201f; // the torque co-efficient at @ 6396.667 RPM, measured by UIUC.
        real_T air_density = 1.225f; //  kg/m^3
        real_T max_rpm = 7770.0f; // revolutions per minute (Volt from battery [22.2V] * [350] RPM/V  from motor)
        real_T propeller_diameter = 0.3302f; //diameter in meters, default is for DJI Phantom 2
        real_T propeller_height = 1 / 100.0f; //height of cylindrical area when propeller rotates, 1 cm
        real_T control_signal_filter_tc = 0.005f; //time constant for low pass filter

        real_T revolutions_per_second;
        real_T max_speed; // in radians per second
        real_T max_speed_square;
        real_T max_thrust = 17.906564886255776f; //computed from above formula for the given constants
        real_T max_torque = 0.2721055800047333f; //computed from above formula

        // call this method to recalculate thrust if you want to use different numbers for C_T, C_P, max_rpm, etc.
        void calculateMaxThrust()
        {
            revolutions_per_second = max_rpm / 60;
            max_speed = revolutions_per_second * 2 * M_PIf; // radians / sec
            max_speed_square = pow(max_speed, 2.0f);

            real_T nsquared = revolutions_per_second * revolutions_per_second;
            max_thrust = C_T * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 4));
            max_torque = C_P * air_density * nsquared * static_cast<real_T>(pow(propeller_diameter, 5)) / (2 * M_PIf);
        }
    };
}
} //namespace
#endif
