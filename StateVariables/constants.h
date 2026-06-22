#pragma once
#include <cmath>
// gravity never changes, so stays a constant variable
namespace Constants {
    constexpr double gravity = -9.81;

    // International Standard Atmosphere air density [kg/m^3] vs geometric
    // altitude [m]. Troposphere (<= 11 km) uses the standard 6.5 K/km lapse
    // rate; the lower stratosphere (11-20 km) is isothermal. Below sea level we
    // clamp to the sea-level value; above 20 km we hold the 20 km value so a
    // high flight still returns a small positive density instead of garbage.
    inline double airDensity(double altitude) {
        if (altitude < 0.0)     altitude = 0.0;
        if (altitude > 20000.0) altitude = 20000.0;
        const double R = 287.05, T0 = 288.15, p0 = 101325.0, g0 = 9.80665;
        double T, p;
        if (altitude <= 11000.0) {
            const double L = 0.0065;                 // lapse rate [K/m]
            T = T0 - L * altitude;
            p = p0 * std::pow(T / T0, g0 / (R * L));
        } else {
            const double T11 = 216.65, p11 = 22632.0; // conditions at 11 km
            T = T11;
            p = p11 * std::exp(-g0 * (altitude - 11000.0) / (R * T11));
        }
        return p / (R * T);
    }
}
