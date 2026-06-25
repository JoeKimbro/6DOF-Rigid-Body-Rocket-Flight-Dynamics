#pragma once
#include <vector>

// ---------------------------------------------------------------------------
// Clean simulation core: the 6DOF physics with ZERO console I/O.
//
// 6DOF/6DOF.cpp drives the same integrator but is welded to std::cin/std::cout
// (it prompts for every parameter and prints a line per step). That makes it
// impossible to reuse from a GUI, a test, or a browser. Here the parameters
// arrive as a plain struct and the whole flight comes back as a vector of
// samples. The caller decides what to do with them -- print, plot, or marshal
// across the WASM boundary to JavaScript.
// ---------------------------------------------------------------------------

// Everything the user would normally be prompted for in DOF6::run().
struct SimParams {
    double dryMass        = 5.0;    // kg, vehicle mass with no fuel
    double fuelMass       = 5.0;    // kg, propellant at launch
    double mass_flow_rate = 2.5;    // kg/s, engine burn rate
    double Ve             = 2500.0; // m/s, exhaust velocity
    double Cd             = 0.5;    // drag coefficient
    double A              = 0.0079; // m^2, reference area
    double startAltitude  = 0.0;    // m
    double total_time     = 60.0;   // s, sim duration
    double theta_deg      = 5.0;    // launch angle from vertical
    double phi_deg        = 90.0;   // launch azimuth (90 = East)
    double CP             = 0.6;    // center of pressure (fraction of L_ref)
    double CG             = 0.5;    // center of gravity (fraction of L_ref)
    double L_ref          = 2.0;    // m, reference length
    double C_mq           = 5.0;    // aerodynamic damping coefficient
    double Cn_alpha       = 10.0;   // normal-force slope (per rad)
    double radius         = 0.05;   // m, body radius (sets axial inertia)
    double fin_cant_deg   = 0.0;    // canted-fin roll angle (0 = no roll)
};

// One snapshot of the flight. Everything a plot or 3D view might want.
struct Sample {
    double t      = 0.0;  // s
    double mass   = 0.0;  // kg
    double x      = 0.0;  // world horizontal position (m)
    double y      = 0.0;  // world vertical position / altitude (m)
    double z      = 0.0;  // world depth position (m)
    double vx     = 0.0;  // m/s
    double vy     = 0.0;  // m/s
    double vz     = 0.0;  // m/s
    double speed  = 0.0;  // m/s, |velocity|
    double AoA    = 0.0;  // deg, angle of attack
    double sideslip = 0.0; // deg
    double theta  = 0.0;  // deg, angle from vertical (derived from quaternion)
    double phi    = 0.0;  // deg, azimuth (derived from quaternion)
    double p      = 0.0;  // rad/s, roll rate
    double q      = 0.0;  // rad/s, pitch rate
    double r      = 0.0;  // rad/s, yaw rate
    double qw     = 1.0;  // attitude quaternion (body -> world)
    double qx     = 0.0;
    double qy     = 0.0;
    double qz     = 0.0;
    double thrust = 0.0;  // N
};

// Run a full flight and return the sampled trajectory.
std::vector<Sample> runSim(const SimParams& in);
