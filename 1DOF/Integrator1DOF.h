#pragma once 
#include "RigidBody1DOF.h"

class Integrator {
    private: 
    double rho = 1.225; // air density (set static for 1DOF)
    public: 
        double dt = 0.01;
        void step(RigidBody& body); 
};
