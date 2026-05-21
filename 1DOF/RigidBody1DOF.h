#pragma once

// seperation of class variables for cleaner code, as these will changes frequently. Applied Struct
struct RigidBody {
    double mass = 0.0;
    double netForce = 0.0;
    double acceleration = 0.0;
    double velocity = 0.0;
    double position = 0.0;
    double Cd = 0.0; // Drag coefficient 
    double A = 0.0; // Reference area 
    double dryMass = 0.0; // Vehicle with no fuel (kg)
    double fuelMass = 0.0; // Propellant loaded at launch (kg)
    double mass_flow_rate = 0.0; // How fast fuel burns (kg/s)    
    double thrust = 0.0; // Deciding if i want to add more in depth propulsion analysis, but for now simple thurst equation
    double Ve = 0.0; // User will set theirs depending on the engine to make 1DOF easier 
};