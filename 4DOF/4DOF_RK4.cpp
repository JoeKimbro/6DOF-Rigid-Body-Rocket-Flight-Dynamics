#define _USE_MATH_DEFINES
#include <cmath>
#include "StateVariables/constants.h"
#include "4DOF_RK4.h"

// Same 3DOF physics, reorganized into one routine that computes forces/moments
// and returns the state derivatives. Called by reference here; RK4 reaches it
// through evaluate() (by value) so trial samples never corrupt the real rocket,
// while run() reaches it through refresh() to repopulate the master's printables.
Deriv DOF4Integrator::physics(RigidBody& s) {
    if (s.props.fuelMass < 0.0) s.props.fuelMass = 0.0;
    s.props.mass = s.props.dryMass + s.props.fuelMass;
    s.v_total = std::sqrt(s.vertical.velocity * s.vertical.velocity +
                          s.horizontal.velocity * s.horizontal.velocity);

    // Thrust is only produced while there is fuel left to burn.
    double thrust = 0.0;
    double dfuel  = 0.0;
    if (s.props.fuelMass > 0.0) {
        thrust = s.props.mass_flow_rate * s.propul.Ve;
        dfuel  = -s.props.mass_flow_rate;
    }
    s.propul.thrust = thrust;

    // --- Force summation (identical to 3DOF) ---
    double Fy = 0.0, Fx = 0.0;
    Fy += s.props.mass * Constants::gravity;                       // gravity (y only)

    double drag_coeff = -0.5 * rho * s.v_total * s.propul.Cd * s.propul.A;
    Fy += drag_coeff * s.vertical.velocity;                        // drag opposes velocity
    Fx += drag_coeff * s.horizontal.velocity;

    Fy += thrust * std::cos(s.theta);                              // thrust along body axis
    Fx += thrust * std::sin(s.theta);

    double q_bar = 0.5 * rho * s.v_total * s.v_total;              // dynamic pressure
    s.rotation.AoA = s.theta - std::atan2(s.horizontal.velocity, s.vertical.velocity);
    s.props.N = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.AoA; // normal force
    Fx += s.props.N * std::cos(s.theta);
    Fy -= s.props.N * std::sin(s.theta);

    s.vertical.netForce      = Fy;
    s.horizontal.netForce    = Fx;
    s.vertical.acceleration  = Fy / s.props.mass;
    s.horizontal.acceleration = Fx / s.props.mass;

    // --- Pitching moment + aerodynamic damping ---
    s.rotation.M = s.propul.Cn_alpha * q_bar * s.propul.A * s.propul.L_ref *
                   s.rotation.AoA * (s.propul.CP - s.propul.CG);
    s.rotation.M += 0.25 * rho * s.v_total * s.propul.A *
                    (s.propul.L_ref * s.propul.L_ref) * s.propul.C_mq * s.rotation.omega;
    s.props.I_yy = (1.0 / 12.0) * s.props.mass * (s.propul.L_ref * s.propul.L_ref);
    s.rotation.theta_ddot = s.rotation.M / s.props.I_yy;

    // --- Package the derivatives of the state vector ---
    Deriv d;
    d.dy     = s.vertical.velocity;
    d.dvy    = s.vertical.acceleration;
    d.dx     = s.horizontal.velocity;
    d.dvx    = s.horizontal.acceleration;
    d.dtheta = s.rotation.omega;
    d.domega = s.rotation.theta_ddot;
    d.dfuel  = dfuel;
    return d;
}

// Returns a copy of `base` nudged forward by (h * k) along every state variable.
RigidBody DOF4Integrator::advance(const RigidBody& base, const Deriv& k, double h) {
    RigidBody s = base;                       // copies ALL parameters unchanged
    s.vertical.position   += h * k.dy;
    s.vertical.velocity   += h * k.dvy;
    s.horizontal.position += h * k.dx;
    s.horizontal.velocity += h * k.dvx;
    s.theta               += h * k.dtheta;
    s.rotation.omega      += h * k.domega;
    s.props.fuelMass      += h * k.dfuel;
    return s;
}

void DOF4Integrator::stepDOF4(RigidBody& body) {
    // Four slope samples: start, two midpoints, and the endpoint.
    Deriv k1 = evaluate(body);
    Deriv k2 = evaluate(advance(body, k1, dt * 0.5));
    Deriv k3 = evaluate(advance(body, k2, dt * 0.5));
    Deriv k4 = evaluate(advance(body, k3, dt));

    // Weighted average of the four slopes: (k1 + 2*k2 + 2*k3 + k4) / 6.
    const double w = dt / 6.0;
    body.vertical.position   += w * (k1.dy     + 2.0 * k2.dy     + 2.0 * k3.dy     + k4.dy);
    body.vertical.velocity   += w * (k1.dvy    + 2.0 * k2.dvy    + 2.0 * k3.dvy    + k4.dvy);
    body.horizontal.position += w * (k1.dx     + 2.0 * k2.dx     + 2.0 * k3.dx     + k4.dx);
    body.horizontal.velocity += w * (k1.dvx    + 2.0 * k2.dvx    + 2.0 * k3.dvx    + k4.dvx);
    body.theta               += w * (k1.dtheta + 2.0 * k2.dtheta + 2.0 * k3.dtheta + k4.dtheta);
    body.rotation.omega      += w * (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega);
    body.props.fuelMass      += w * (k1.dfuel  + 2.0 * k2.dfuel  + 2.0 * k3.dfuel  + k4.dfuel);

    if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    body.props.mass = body.props.dryMass + body.props.fuelMass;

    // NOTE: the integrated state (position/velocity/theta/omega/fuelMass/mass) is now
    // correct. The diagnostic fields (netForce, accel, AoA, N, M, I_yy, v_total) are
    // still from the last trial sample because evaluate() is pure-by-value. They don't
    // affect the trajectory; refresh them for printing when run() is written.
}
