#include <cmath>
#include "StateVariables/constants.h"
#include "Integrator1DOF.h"
#include "StateVariables/RigidBody.h"

// Integrator applies Eulars and Second Law calculations to find variables 
void Integrator::step(RigidBody& body) {

    if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    body.props.mass = body.props.dryMass + body.props.fuelMass;
    body.propul.thrust = body.props.mass_flow_rate * body.propul.Ve;
    body.vertical.netForce = 0.0;
    body.vertical.netForce += body.props.mass * Constants::gravity;
    body.vertical.netForce += -0.5 * rho * std::abs(body.vertical.velocity) * body.vertical.velocity * body.propul.Cd * body.propul.A;
    if (body.props.fuelMass > 0) {
        body.vertical.netForce += body.propul.thrust;
        body.props.fuelMass -= body.props.mass_flow_rate * dt;
        if (body.props.fuelMass < 0.0) body.props.fuelMass = 0.0;
    }
    body.vertical.acceleration = body.vertical.netForce / body.props.mass;
    body.vertical.velocity += body.vertical.acceleration * dt;
    body.vertical.position += body.vertical.velocity * dt;
}