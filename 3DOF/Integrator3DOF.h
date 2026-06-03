#pragma once 
#include "StateVariables/RigidBody.h"

class DOF3Integrator{
    private: 
    double rho = 1.225; // air density (set static for 1DOF)
    public: 
        double dt = 0.01;
        void stepDOF3(RigidBody& body);  
};