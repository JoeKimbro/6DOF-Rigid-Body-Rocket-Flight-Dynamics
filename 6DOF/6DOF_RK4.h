#pragma once
#include "StateVariables/RigidBody.h"

// The time-derivative of every variable in the 4DOF state vector.
// RK4 evaluates these at four trial states per step, so they are kept
// separate from the RigidBody (the "truth") and never committed directly.
struct Deriv6DOF {
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
    double quatW = 0.0; 
    double quatX = 0.0; 
    double quatZ = 0.0; 
    double quatY = 0.0; 
    double angularR = 0.0; 
    double angularQ = 0.0; 
    double angularP = 0.0; 
};

class DOF6Integrator {
    private:
        // air density is now computed per-step from altitude (ISA model in
        // Constants::airDensity), so there is no static rho member.

        // Core physics for one state. Computes all forces/moments, writes the derived
        // & diagnostic fields (netForce, accel, AoA, N, M, I_yy, v_total, ...) into
        // `s`, and returns the derivatives of the integrated state vector. It never
        // touches the integrated variables themselves (position/velocity/theta/
        // omega/fuelMass), only quantities derived from them.
        Deriv6DOF physics(RigidBody& s);

        // Pure wrapper used for RK4 trial sampling: `s` is a throwaway copy, so the
        // diagnostic writes inside physics() are discarded and the master is untouched.
        Deriv6DOF evaluate(RigidBody s) { return physics(s); }

        // Build a trial state: base state advanced by (h * k) along each axis.
        RigidBody advance(const RigidBody& base, const Deriv6DOF& k, double h);

    public:
        // 0.002 s: small enough to resolve the gyroscopic nutation of a
        // fast-spinning body. At 0.01 s a rolling rocket's nutation outruns the
        // step and RK4 diverges to NaN.
        double dt = 0.002;
        void stepDOF6(RigidBody& body);

        // Recompute the force/diagnostic fields on the master state so run() can
        // print live AoA/N/M/etc. after a step. Discards the returned derivatives.
        void refresh(RigidBody& body) { physics(body); }
};
