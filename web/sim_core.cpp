#define _USE_MATH_DEFINES
#include <cmath>
#include "sim_core.h"
#include "6DOF/6DOF_RK4.h"   // DOF6Integrator (-I<repo root> on the build line)

static constexpr double RAD2DEG = 180.0 / M_PI;

std::vector<Sample> runSim(const SimParams& in) {
    RigidBody body;
    DOF6Integrator integrator;   // dt = 0.002 s baked into the class

    // --- Load parameters (mirrors the std::cin block in 6DOF/6DOF.cpp) ---
    body.props.dryMass        = in.dryMass;
    body.props.fuelMass       = in.fuelMass;
    body.props.mass_flow_rate = in.mass_flow_rate;
    body.propul.Ve            = in.Ve;
    body.propul.Cd            = in.Cd;
    body.propul.A             = in.A;
    body.vertical.position    = in.startAltitude;
    body.theta_deg            = in.theta_deg;
    body.theta                = in.theta_deg * M_PI / 180.0;
    body.phi_deg              = in.phi_deg;
    body.phi                  = in.phi_deg * M_PI / 180.0;
    body.propul.CP            = in.CP;
    body.propul.CG            = in.CG;
    body.propul.L_ref         = in.L_ref;
    body.propul.C_mq          = in.C_mq;
    body.propul.Cn_alpha      = in.Cn_alpha;
    body.props.radius         = (in.radius > 0.0) ? in.radius : 1e-3;
    body.propul.fin_cant      = in.fin_cant_deg * M_PI / 180.0;

    // --- Initialize the attitude quaternion from the launch angles ---
    // Identical to DOF6::run(): the nose (+x) starts pointing along
    // (sinθ·cosφ, cosθ, sinθ·sinφ). q0 = q_yaw(φ) ⊗ q_pitch(π/2 − θ).
    {
        const double a  = M_PI / 2.0 - body.theta;
        const double cp = std::cos(a * 0.5), sp = std::sin(a * 0.5);
        const double cy = std::cos(body.phi * 0.5), sy = std::sin(body.phi * 0.5);
        body.sixStates.qw =  cy * cp;
        body.sixStates.qx = -sy * sp;
        body.sixStates.qy = -sy * cp;
        body.sixStates.qz =  cy * sp;
        const double n = std::sqrt(body.sixStates.qw * body.sixStates.qw +
                                   body.sixStates.qx * body.sixStates.qx +
                                   body.sixStates.qy * body.sixStates.qy +
                                   body.sixStates.qz * body.sixStates.qz);
        body.sixStates.qw /= n; body.sixStates.qx /= n;
        body.sixStates.qy /= n; body.sixStates.qz /= n;
    }

    std::vector<Sample> out;
    const double dt = integrator.dt;
    const int steps = static_cast<int>(in.total_time / dt);
    out.reserve(static_cast<size_t>(steps / 5) + 2);

    auto capture = [&](double t) {
        Sample s;
        s.t        = t;
        s.mass     = body.props.mass;
        s.x        = body.horizontal.position;
        s.y        = body.vertical.position;
        s.z        = body.depth.position;
        s.vx       = body.horizontal.velocity;
        s.vy       = body.vertical.velocity;
        s.vz       = body.depth.velocity;
        s.speed    = body.v_total;
        s.AoA      = body.rotation.AoA * RAD2DEG;
        s.sideslip = body.rotation.Sideslip * RAD2DEG;
        s.theta    = body.theta * RAD2DEG;
        s.phi      = body.phi * RAD2DEG;
        s.p        = body.sixStates.p;
        s.q        = body.sixStates.q;
        s.r        = body.sixStates.r;
        s.qw       = body.sixStates.qw;
        s.qx       = body.sixStates.qx;
        s.qy       = body.sixStates.qy;
        s.qz       = body.sixStates.qz;
        s.thrust   = body.propul.thrust;
        out.push_back(s);
    };

    // A diverged step produces NaN/Inf. The roll channel is stiff (small axial
    // inertia I_x = 0.5·m·r²), so an aggressive fin cant can outrun the explicit
    // RK4 step and blow up -- exactly the timestep limit the README documents.
    // We stop the moment the state stops being finite so the caller always gets
    // a clean, finite trajectory instead of a crash or NaN-poisoned JSON.
    // "Sane" = finite AND within a generous physical bound (1e7 m ~ far beyond
    // any real sub-orbital flight). The magnitude cap catches the huge-but-still-
    // finite step that a divergence produces just before it overflows to Inf,
    // so the captured trajectory never contains a garbage sample.
    constexpr double LIMIT = 1e7;
    auto stateSane = [&]() {
        return std::isfinite(body.vertical.position)   && std::fabs(body.vertical.position)   < LIMIT &&
               std::isfinite(body.horizontal.position) && std::fabs(body.horizontal.position) < LIMIT &&
               std::isfinite(body.depth.position)      && std::fabs(body.depth.position)      < LIMIT &&
               std::isfinite(body.sixStates.qw) &&
               std::isfinite(body.sixStates.p);
    };

    integrator.refresh(body);   // populate diagnostics for the t=0 sample
    capture(0.0);

    for (int i = 0; i < steps; ++i) {
        integrator.stepDOF6(body);
        integrator.refresh(body);
        if (!stateSane()) break;               // diverged -> keep the last good samples
        if (body.vertical.position <= 0.0) {   // impact -> stop
            capture(i * dt);
            break;
        }
        if (i % 5 == 0) capture(i * dt);   // ~0.01 s cadence, same as the CLI
    }

    return out;
}
