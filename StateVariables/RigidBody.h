#pragma once
#include "StateVariables/LinearState.h"
#include "StateVariables/MassProperties.h"
#include "StateVariables/Propulsion.h"
#include "StateVariables/AngularState.h"

// seperation of class variables for cleaner code, as these will changes frequently. Applied Struct
struct RigidBody { 
    double theta = 0.0; 
    double theta_deg = 0.0;
    double v_total = 0.0; 
    PropulsionProps propul;
    MassProperties props;
    LinearState    vertical;
    LinearState horizontal; 
    AngularState rotation;
};