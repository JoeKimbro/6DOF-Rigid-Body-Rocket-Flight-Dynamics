#pragma once
#include "StateVariables/RigidBody.h"
#include "Integrator1DOF.h"

class DOF1 {
    public:
        double total_time = 0.0; 
        void run(RigidBody& body, Integrator& integrator);
};  