#include <cmath>
#include "constants.h"
#include "Integrator1DOF.h"
#include "RigidBody1DOF.h"

// Integrator applies Eulars and Second Law calculations to find variables 
void Integrator::step(RigidBody& body) {
            
    if (body.fuelMass < 0.0) {body.fuelMass = 0.0;} //fixing float point bug, goes past mass without fuel by 0.01
    body.mass = body.dryMass + body.fuelMass;
    body.thrust = body.mass_flow_rate * body.Ve;
    body.netForce = 0.0;
    body.netForce += body.mass * Constants::gravity; 
    body.netForce += -0.5 * rho * abs(body.velocity) * body.velocity * body.Cd * body.A; // This is drag we can just add it without needing variable.
    if (body.fuelMass > 0) {
        body.netForce += body.thrust;
        body.fuelMass -= body.mass_flow_rate * dt;
    }
    body.acceleration = body.netForce / body.mass; 
    body.velocity += body.acceleration * dt; 
    body.position += body.velocity * dt;
}