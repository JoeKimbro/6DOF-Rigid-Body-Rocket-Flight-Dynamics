#pragma once 
#include "StateVariables/RigidBody.h"
#include "StateVariables/FlightSim.h"
#include "Integrator3DOF.h"

class DOF3 : public FlightSim {
    private: DOF3Integrator integrator;
    public: 
        double total_time = 0.0; 
        void run(RigidBody& body);
};