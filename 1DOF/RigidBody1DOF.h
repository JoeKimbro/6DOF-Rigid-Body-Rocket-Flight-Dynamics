#pragma once
#include "StateVariables/LinearState.h"
#include "StateVariables/MassProperties.h"
// seperation of class variables for cleaner code, as these will changes frequently. Applied Struct
struct RigidBody {
    double Cd = 0.0; // Drag coefficient 
    double A = 0.0; // Reference area 
    double thrust = 0.0; // Deciding if i want to add more in depth propulsion analysis, but for now simple thurst equation
    double Ve = 0.0; // User will set theirs depending on the engine to make 1DOF easier 
    MassProperties props;
    LinearState    vertical;
};