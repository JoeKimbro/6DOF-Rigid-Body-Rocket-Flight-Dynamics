#define _USE_MATH_DEFINES
#include <cmath>
#include "StateVariables/constants.h"
#include "Integrator2DOF.h"
#include "StateVariables/RigidBody.h"

void DOF2Integrator::stepDOF2(RigidBody& body) {
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
    body.vertical.acceleration = body.vertical.netForce / body.props.mass;
    body.horizontal.acceleration = body.horizontal.netForce / body.props.mass; 
    body.vertical.velocity += body.vertical.acceleration * dt;
    body.horizontal.velocity += body.horizontal.acceleration * dt;
    body.v_total = std::sqrt(body.vertical.velocity * body.vertical.velocity + body.horizontal.velocity * body.horizontal.velocity);
    body.vertical.position += body.vertical.velocity * dt;
    body.horizontal.position += body.horizontal.velocity * dt;
}