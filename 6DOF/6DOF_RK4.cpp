#define _USE_MATH_DEFINES
#include <cmath>
#include "StateVariables/constants.h"
#include "6DOF_RK4.h"

Deriv6DOF DOF6Integrator::physics(RigidBody& s) {
    if (s.props.fuelMass < 0.0) s.props.fuelMass = 0.0;
    s.props.mass = s.props.dryMass + s.props.fuelMass;
    s.v_total = std::sqrt(s.vertical.velocity * s.vertical.velocity +
                          s.horizontal.velocity * s.horizontal.velocity + s.depth.velocity * s.depth.velocity); // add Z velocity sqrd

    // Air density now varies with altitude (ISA model) instead of a fixed 1.225.
    const double rho = Constants::airDensity(s.vertical.position);

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
    // Thrust acts along the body nose (body +x). Rotate that body-axis vector
    // into the world frame with the attitude quaternion instead of theta/phi.
    Vec3 nose = bodyToWorld(s.sixStates, {1.0, 0.0, 0.0});
    Fx += thrust * nose.x;
    Fy += thrust * nose.y;
    Fz += thrust * nose.z;

    double q_bar = 0.5 * rho * s.v_total * s.v_total;              // dynamic pressure

    // --- Aerodynamic angles from the BODY-frame velocity (Step 4) ---
    // Rotate world velocity into the body frame: u along nose (+x), v (+y), w (+z).
    Vec3 v_body = worldToBody(s.sixStates,
                              {s.horizontal.velocity, s.vertical.velocity, s.depth.velocity});
    const double u = v_body.x, v = v_body.y, w = v_body.z;
    if (s.v_total > 1e-6) {
        s.rotation.AoA      = std::atan2(w, u);   // angle of attack  (body x-z plane)
        s.rotation.Sideslip = std::atan2(v, u);   // sideslip         (body x-y plane)
    } else {
        s.rotation.AoA = 0.0;
        s.rotation.Sideslip = 0.0;
    }

    // Normal/side force magnitudes (linear in the aero angle).
    const double N_pitch = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.AoA;
    const double N_yaw   = s.propul.Cn_alpha * q_bar * s.propul.A * s.rotation.Sideslip;
    s.rotation.N_pitch = N_pitch;
    s.rotation.N_yaw   = N_yaw;

    // Aero force lives in the body transverse plane, opposing the cross-velocity,
    // then rotates into the world frame to join gravity/drag/thrust (Step 3).
    Vec3 F_aero = bodyToWorld(s.sixStates, {0.0, -N_yaw, -N_pitch});
    Fx += F_aero.x;
    Fy += F_aero.y;
    Fz += F_aero.z;

    s.vertical.netForce      = Fy;
    s.horizontal.netForce    = Fx;
    s.depth.netForce         = Fz;
    s.vertical.acceleration   = Fy / s.props.mass;
    s.horizontal.acceleration = Fx / s.props.mass;
    s.depth.acceleration      = Fz / s.props.mass;

    // --- Moments about the body axes ---
    // Solid-cylinder transverse inertia: (1/12) m L^2 (slender term) + (1/4) m r^2.
    s.props.I_yy = s.props.mass * ((1.0 / 12.0) * (s.propul.L_ref * s.propul.L_ref) +
                                   0.25 * (s.props.radius * s.props.radius));
    const double I_t = s.props.I_yy;                                        // transverse (pitch = yaw)
    const double I_x = 0.5 * s.props.mass * s.props.radius * s.props.radius; // axial (roll)

    const double p = s.sixStates.p, q = s.sixStates.q, r = s.sixStates.r;
    const double damp = 0.25 * rho * s.v_total * s.propul.A *
                        (s.propul.L_ref * s.propul.L_ref) * s.propul.C_mq;

    // Static restoring moment from the aero force acting at the CP: M = d x F,
    // with arm d = ((CG-CP)*L_ref, 0, 0) and F_aero = (0,-N_yaw,-N_pitch). This
    // yields OPPOSITE signs for pitch vs yaw (the classic Cm_alpha<0 / Cn_beta>0
    // asymmetry, because d(AoA)/dt=+q but d(beta)/dt=-r). Plus body-rate damping.
    const double arm = s.propul.L_ref * (s.propul.CG - s.propul.CP);
    s.rotation.M     =  arm * N_pitch - damp * q;   // pitch moment about body y
    s.rotation.M_yaw = -arm * N_yaw   - damp * r;   // yaw moment about body z

    // Roll (Step 6): canted-fin forcing (fin_cant=0 => none) opposed by aero roll
    // damping. Uses the same L_ref reference as pitch/yaw (not the tiny radius) so
    // the steady roll rate stays physical and within the integrator's dt stability
    // limit -- a radius arm makes p spin to hundreds of rad/s and the gyroscopic
    // nutation then outruns dt=0.01 and blows up.
    const double roll_force = s.propul.Cn_alpha * q_bar * s.propul.A *
                              s.propul.L_ref * s.propul.fin_cant;
    const double M_x = roll_force - damp * p;
    const double M_y = s.rotation.M;
    const double M_z = s.rotation.M_yaw;

    s.rotation.theta_ddot = M_y / I_t;   // diagnostics only
    s.rotation.phi_ddot   = M_z / I_t;

    // theta/phi are now DERIVED from the quaternion for display, not integrated (Step 5).
    // (reuses `nose`, the body +x axis in world coords, computed above for thrust)
    s.theta = std::atan2(std::sqrt(nose.x * nose.x + nose.z * nose.z), nose.y); // from vertical
    s.phi   = std::atan2(nose.z, nose.x);                                       // azimuth
    s.rotation.omega     = q;   // display the body rates in the old fields
    s.rotation.omega_phi = r;


    // --- Package the derivatives of the state vector ---
    Deriv6DOF d;
    d.dy     = s.vertical.velocity;
    d.dvy    = s.vertical.acceleration;
    d.dx     = s.horizontal.velocity;
    d.dvx    = s.horizontal.acceleration;
    d.dz  = s.depth.velocity;       
    d.dvz = s.depth.acceleration; 
    d.dfuel  = dfuel;
    // theta/phi/omega/omega_phi are no longer integrated -- attitude lives in the
    // quaternion + body rates (p,q,r). These stay zero (Step 5).
    d.dtheta = 0.0;
    d.domega = 0.0;
    d.dphi = 0.0;
    d.domega_phi = 0.0;
    // Quaternion Evolutions
    d.quatW = -0.5 * ((s.sixStates.qx * p) + (s.sixStates.qy * q) + (s.sixStates.qz * r));
    d.quatX = 0.5 * ((s.sixStates.qw * p) + (s.sixStates.qy * r) - (s.sixStates.qz * q));
    d.quatZ =  0.5 * ((s.sixStates.qw * r) + (s.sixStates.qx * q) - (s.sixStates.qy * p));
    d.quatY =  0.5 * ((s.sixStates.qw * q) + (s.sixStates.qz * p) - (s.sixStates.qx * r));
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
    double n = std::sqrt(body.sixStates.qw*body.sixStates.qw +
        body.sixStates.qx * body.sixStates.qx + body.sixStates.qy * body.sixStates.qy + body.sixStates.qz*body.sixStates.qz);
        body.sixStates.qw /= n;
        body.sixStates.qx /= n;
        body.sixStates.qy /= n;
        body.sixStates.qz /= n;

    if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    body.props.mass = body.props.dryMass + body.props.fuelMass;
}
