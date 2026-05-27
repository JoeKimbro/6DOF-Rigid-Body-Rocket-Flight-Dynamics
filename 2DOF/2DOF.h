#pragma once 
#include "StateVariables/RigidBody.h"
#include "StateVariables/FlightSim.h"
#include "Integrator2DOF.h"

class DOF2 : public FlightSim {
    private: DOF2Integrator integrator;
    public: 
        double total_time = 0.0; 
        void run(RigidBody& body);
};