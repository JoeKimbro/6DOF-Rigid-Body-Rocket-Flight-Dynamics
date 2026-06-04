#define _USE_MATH_DEFINES
#include <cmath>
#include "StateVariables/constants.h"
#include "Integrator3DOF.h"
#include "StateVariables/RigidBody.h"

void DOF3Integrator::stepDOF3(RigidBody& body) {
    // for this we need to serpate a new variable for X's position, gravity, thrust, drag
    if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    body.props.mass = body.props.dryMass + body.props.fuelMass;
    body.propul.thrust = body.props.mass_flow_rate * body.propul.Ve;
    body.vertical.netForce = 0.0;
    body.horizontal.netForce = 0.0;
    // total velocity 
    body.vertical.netForce += body.props.mass * Constants::gravity; // gravity only affects y axis
    double drag_coeff = -0.5 * rho * body.v_total * body.propul.Cd * body.propul.A;
    body.vertical.netForce   += drag_coeff * body.vertical.velocity; // made drag_coeff cause redundant multiplacation
    body.horizontal.netForce += drag_coeff * body.horizontal.velocity;    // drag formula above seperated now x and y
    if (body.props.fuelMass > 0) {
        body.vertical.netForce += body.propul.thrust * std::cos(body.theta);
        body.horizontal.netForce += body.propul.thrust * std::sin(body.theta);
        body.props.fuelMass -= body.props.mass_flow_rate * dt;
        if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    }
      // Dynamic pressure q = 1/2 rhov^2
    double q_bar = 0.5 * rho * body.v_total * body.v_total;
    //   aoa = theta - atan2(v_z, v_x)\): Angle of attack
    body.rotation.AoA = body.theta - atan2(body.horizontal.velocity, body.vertical.velocity);
    body.props.N = body.propul.Cn_alpha * q_bar * body.propul.A * body.rotation.AoA;
    body.horizontal.netForce += body.props.N * std::cos(body.theta);
    body.vertical.netForce -= body.props.N * std::sin(body.theta);
    body.vertical.acceleration = body.vertical.netForce / body.props.mass;
    body.horizontal.acceleration = body.horizontal.netForce / body.props.mass; 
    body.vertical.velocity += body.vertical.acceleration * dt;
    body.horizontal.velocity += body.horizontal.acceleration * dt;
    body.v_total = std::sqrt(body.vertical.velocity * body.vertical.velocity + body.horizontal.velocity * body.horizontal.velocity);
    body.vertical.position += body.vertical.velocity * dt;
    body.horizontal.position += body.horizontal.velocity * dt;
    // Pitching moment
    body.rotation.M = body.propul.Cn_alpha * q_bar * body.propul.A * body.propul.L_ref * body.rotation.AoA * (body.propul.CP - body.propul.CG); 
    body.props.I_yy = (1.0/12.0) * body.props.mass * (body.propul.L_ref * body.propul.L_ref);
        // Angular acceleration
    body.rotation.theta_ddot = body.rotation.M / body.props.I_yy; 
    // Integrate rotation
    body.rotation.omega += body.rotation.theta_ddot * dt;
    body.theta += body.rotation.omega * dt;

}  

/*
  ┌──────┬──────────────────────────────────────────┬───────────────────────────────────────────────┐
  │ Step │                 Formula                  │                   Have it?                    │
  ├──────┼──────────────────────────────────────────┼───────────────────────────────────────────────┤
  │ 1    │ α = θ − atan2(v_h, v_v)                  │ Yes                                            │
  ├──────┼──────────────────────────────────────────┼───────────────────────────────────────────────┤
  │ 2    │ q̄ = ½ρv²                                 │ Implicitly — embedded in drag, not standalone │
  ├──────┼──────────────────────────────────────────┼───────────────────────────────────────────────┤
  │ 3    │ M = Cn_α · q̄ · A · L_ref · α · (CP − CG) │ Yes                                           │
  ├──────┼──────────────────────────────────────────┼───────────────────────────────────────────────┤
  │ 4    │ θ̈ = M / I_yy                             │ Yes                                           │
  ├──────┼──────────────────────────────────────────┼───────────────────────────────────────────────┤
  │ 5    │ ω += θ̈·dt, θ += ω·dt                     │ Yes                                             │
  └──────┴──────────────────────────────────────────┴───────────────────────────────────────────────┘
*/