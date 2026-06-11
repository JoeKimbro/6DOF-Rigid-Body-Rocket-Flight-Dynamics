#pragma once
#include "StateVariables/RigidBody.h"

// The time-derivative of every variable in the 4DOF state vector.
// RK4 evaluates these at four trial states per step, so they are kept
// separate from the RigidBody (the "truth") and never committed directly.
struct Deriv {
    double dy     = 0.0; // d(vertical.position)/dt   = vertical.velocity
    double dvy    = 0.0; // d(vertical.velocity)/dt   = Fy / mass
    double dx     = 0.0; // d(horizontal.position)/dt = horizontal.velocity
    double dvx    = 0.0; // d(horizontal.velocity)/dt = Fx / mass
    double dz = 0.0;
    double dvz = 0.0;
    double dtheta = 0.0; // d(theta)/dt               = omega
    double domega = 0.0; // d(omega)/dt               = M / I_yy
    double dfuel  = 0.0; // d(fuelMass)/dt            = -mass_flow_rate (while burning)
    double dphi = 0.0; // yaw angle
    double domega_phi = 0.0; // yaw rate 
    // add azimuth?
};

class DOF5Integrator {
    private:
        double rho = 1.225; // air density (static, as in earlier stages)

        // Core physics for one state. Computes all forces/moments, writes the derived
        // & diagnostic fields (netForce, accel, AoA, N, M, I_yy, v_total, ...) into
        // `s`, and returns the derivatives of the integrated state vector. It never
        // touches the integrated variables themselves (position/velocity/theta/
        // omega/fuelMass), only quantities derived from them.
        Deriv physics(RigidBody& s);

        // Pure wrapper used for RK4 trial sampling: `s` is a throwaway copy, so the
        // diagnostic writes inside physics() are discarded and the master is untouched.
        Deriv evaluate(RigidBody s) { return physics(s); }

        // Build a trial state: base state advanced by (h * k) along each axis.
        RigidBody advance(const RigidBody& base, const Deriv& k, double h);

    public:
        double dt = 0.01;
        void stepDOF5(RigidBody& body);

        // Recompute the force/diagnostic fields on the master state so run() can
        // print live AoA/N/M/etc. after a step. Discards the returned derivatives.
        void refresh(RigidBody& body) { physics(body); }
};
