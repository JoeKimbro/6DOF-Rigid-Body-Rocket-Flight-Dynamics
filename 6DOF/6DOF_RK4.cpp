#define _USE_MATH_DEFINES
#include <cmath>
#include "StateVariables/constants.h"
#include "6DOF_RK4.h"

Deriv6DOF DOF6Integrator::physics(RigidBody& s) {
    if (s.props.fuelMass < 0.0) s.props.fuelMass = 0.0;
    s.props.mass = s.props.dryMass + s.props.fuelMass;
    s.v_total = std::sqrt(s.vertical.velocity * s.vertical.velocity +
                          s.horizontal.velocity * s.horizontal.velocity + s.depth.velocity * s.depth.velocity); // add Z velocity sqrd

    // Thrust is only produced while there is fuel left to burn.
    double thrust = 0.0;
    double dfuel  = 0.0;
    if (s.props.fuelMass > 0.0) {
        thrust = s.props.mass_flow_rate * s.propul.Ve;
        dfuel  = -s.props.mass_flow_rate;
    }
    s.propul.thrust = thrust;

    // --- Force summation (identical to 3DOF) ---
    double Fy = 0.0, Fx = 0.0, Fz = 0.0;
    Fy += s.props.mass * Constants::gravity;                       // gravity (y only)

    double drag_coeff = -0.5 * rho * s.v_total * s.propul.Cd * s.propul.A;
    Fy += drag_coeff * s.vertical.velocity;                        // drag opposes velocity
    Fx += drag_coeff * s.horizontal.velocity;
    Fz += drag_coeff * s.depth.velocity;
    double Ty = thrust * std::cos(s.theta);   // vertical part
    double Th = thrust * std::sin(s.theta);   // horizontal magnitude
    Fy += Ty;
    Fx += Th * std::cos(s.phi);
    Fz += Th * std::sin(s.phi);

    double q_bar = 0.5 * rho * s.v_total * s.v_total;              // dynamic pressure
    // Aerodynamic angles: each body angle vs velocity angle in the same plane
    s.rotation.AoA = s.theta - std::atan2(s.horizontal.velocity, s.vertical.velocity);
    const double v_h = std::sqrt(s.horizontal.velocity * s.horizontal.velocity +
                                 s.depth.velocity * s.depth.velocity);
    if (v_h > 1e-6) {
        s.rotation.Sideslip = s.phi - std::atan2(s.depth.velocity, s.horizontal.velocity);
    } else {
        s.rotation.Sideslip = 0.0;
    }

    double N_pitch = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.AoA;
    double N_yaw   = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.Sideslip;

    // Pitch normal: vertical + horizontal, then rotate horizontal by azimuth (same as thrust)
    const double N_pitch_h = N_pitch * std::cos(s.theta);
    Fy -= N_pitch * std::sin(s.theta);
    Fx += N_pitch_h * std::cos(s.phi);
    Fz += N_pitch_h * std::sin(s.phi);

    // Yaw normal: horizontal side force, rotated by azimuth
    Fx += -N_yaw * std::sin(s.phi);
    Fz +=  N_yaw * std::cos(s.phi);

    s.vertical.netForce      = Fy;
    s.horizontal.netForce    = Fx;
    s.vertical.acceleration  = Fy / s.props.mass;
    s.depth.netForce     = Fz;      
    s.depth.acceleration = Fz / s.props.mass;
    s.horizontal.acceleration = Fx / s.props.mass;

    // --- Pitching moment + aerodynamic damping ---
    s.rotation.M = s.propul.Cn_alpha * q_bar * s.propul.A * s.propul.L_ref *
                   s.rotation.AoA * (s.propul.CG - s.propul.CP);
    s.rotation.M -= 0.25 * rho * s.v_total * s.propul.A * (s.propul.L_ref * s.propul.L_ref) * s.propul.C_mq * s.rotation.omega;
    s.props.I_yy = (1.0 / 12.0) * s.props.mass * (s.propul.L_ref * s.propul.L_ref);
    s.rotation.theta_ddot = s.rotation.M / s.props.I_yy;

    // Aerodynamic restoring yawing moment
    s.rotation.M_yaw = s.propul.Cn_alpha * q_bar * s.propul.A * s.propul.L_ref * s.rotation.Sideslip * (s.propul.CG - s.propul.CP); 
    // Aerodynamic yaw damping (opposes yaw rotation rate)
    s.rotation.M_yaw -= 0.25 * rho * s.v_total * s.propul.A * (s.propul.L_ref * s.propul.L_ref) * s.propul.C_mq * s.rotation.omega_phi;
    // Assuming a symmetric cylindrical rocket body where I_zz around the yaw axis equals I_yy
    s.rotation.phi_ddot = s.rotation.M_yaw  / s.props.I_yy; 
    s.rotation.N_pitch = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.AoA;
    s.rotation.N_yaw   = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.Sideslip;

    const double I_t = s.props.I_yy;                    
    const double I_x = 0.5 * s.props.mass * s.props.radius*s.props.radius;  // axial — needs a radius

    const double M_x = 0.0;              // no roll forcing yet
    const double M_y = s.rotation.M;     // pitch moment 
    const double M_z = s.rotation.M_yaw; // yaw moment   
    const double p = s.sixStates.p;
    const double q = s.sixStates.q;
    const double r = s.sixStates.r;
    

    // --- Package the derivatives of the state vector ---
    Deriv6DOF d;
    d.dy     = s.vertical.velocity;
    d.dvy    = s.vertical.acceleration;
    d.dx     = s.horizontal.velocity;
    d.dvx    = s.horizontal.acceleration;
    d.dz  = s.depth.velocity;       
    d.dvz = s.depth.acceleration; 
    d.dtheta = s.rotation.omega;
    d.domega = s.rotation.theta_ddot;
    d.dfuel  = dfuel;
    d.dphi = s.rotation.omega_phi;    
    d.domega_phi = s.rotation.phi_ddot;
    // Quaternion Evolutions
    d.quatW = 0.0;
    d.quatX = 0.0;
    d.quatZ = 0.0;
    d.quatY = 0.0;
    // New Body Angular Rates
    d.angularR =  (M_z + (I_x - I_t) * p * q) / I_t;
    d.angularQ = (M_y + (I_t - I_x) * r * p) / I_t;
    d.angularP = M_x / I_x;
    
    // add azimuth?
    return d;
}

// Returns a copy of `base` nudged forward by (h * k) along every state variable.
RigidBody DOF6Integrator::advance(const RigidBody& base, const Deriv6DOF& k, double h) {
    RigidBody s = base;                       // copies ALL parameters unchanged
    s.vertical.position   += h * k.dy;
    s.vertical.velocity   += h * k.dvy;
    s.horizontal.position += h * k.dx;
    s.horizontal.velocity += h * k.dvx;
    s.theta               += h * k.dtheta;
    s.rotation.omega      += h * k.domega;
    s.props.fuelMass      += h * k.dfuel;
    s.depth.position += h * k.dz;   
    s.depth.velocity += h * k.dvz;
    s.phi += h * k.dphi; 
    s.rotation.omega_phi += h * k.domega_phi;
    s.sixStates.qw += h * k.quatW;
    s.sixStates.qx += h * k.quatX;
    s.sixStates.qy += h * k.quatY;
    s.sixStates.qz += h * k.quatZ;
    s.sixStates.p  += h * k.angularP;
    s.sixStates.q  += h * k.angularQ;
    s.sixStates.r  += h * k.angularR;
    return s;
}

void DOF6Integrator::stepDOF6(RigidBody& body) {
    // Four slope samples: start, two midpoints, and the endpoint.
    Deriv6DOF k1 = evaluate(body);
    Deriv6DOF k2 = evaluate(advance(body, k1, dt * 0.5));
    Deriv6DOF k3 = evaluate(advance(body, k2, dt * 0.5));
    Deriv6DOF k4 = evaluate(advance(body, k3, dt));

    // Weighted average of the four slopes: (k1 + 2*k2 + 2*k3 + k4) / 6.
    const double w = dt / 6.0;
    body.vertical.position   += w * (k1.dy     + 2.0 * k2.dy     + 2.0 * k3.dy     + k4.dy);
    body.vertical.velocity   += w * (k1.dvy    + 2.0 * k2.dvy    + 2.0 * k3.dvy    + k4.dvy);
    body.horizontal.position += w * (k1.dx     + 2.0 * k2.dx     + 2.0 * k3.dx     + k4.dx);
    body.horizontal.velocity += w * (k1.dvx    + 2.0 * k2.dvx    + 2.0 * k3.dvx    + k4.dvx);
    body.sixStates.qw += w * (k1.quatW + 2.0*k2.quatW + 2.0*k3.quatW + k4.quatW);
    body.sixStates.qx += w * (k1.quatX + 2.0*k2.quatX + 2.0*k3.quatX + k4.quatX);
    body.sixStates.qy += w * (k1.quatY + 2.0*k2.quatY + 2.0*k3.quatY + k4.quatY);
    body.sixStates.qz += w * (k1.quatZ + 2.0*k2.quatZ + 2.0*k3.quatZ + k4.quatZ);
    body.sixStates.p  += w * (k1.angularP + 2.0*k2.angularP + 2.0*k3.angularP + k4.angularP);
    body.sixStates.q  += w * (k1.angularQ + 2.0*k2.angularQ + 2.0*k3.angularQ + k4.angularQ);
    body.sixStates.r  += w * (k1.angularR + 2.0*k2.angularR + 2.0*k3.angularR + k4.angularR);
    body.theta               += w * (k1.dtheta + 2.0 * k2.dtheta + 2.0 * k3.dtheta + k4.dtheta);
    body.rotation.omega      += w * (k1.domega + 2.0 * k2.domega + 2.0 * k3.domega + k4.domega);
    body.props.fuelMass      += w * (k1.dfuel  + 2.0 * k2.dfuel  + 2.0 * k3.dfuel  + k4.dfuel);
    body.depth.position += w * (k1.dz  + 2.0*k2.dz  + 2.0*k3.dz + k4.dz);
    body.depth.velocity += w * (k1.dvz + 2.0*k2.dvz + 2.0*k3.dvz + k4.dvz);
    body.phi += w * (k1.dphi + 2.0 * k2.dphi + 2.0 * k3.dphi + k4.dphi);
    body.rotation.omega_phi += w * (k1.domega_phi + 2.0 * k2.domega_phi + 2.0 * k3.domega_phi + k4.domega_phi);

    if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    body.props.mass = body.props.dryMass + body.props.fuelMass;
}
