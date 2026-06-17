#pragma once
#include "StateVariables/RigidBody.h"
#include "StateVariables/FlightSim.h"
#include "4DOF_RK4.h"

class DOF4 : public FlightSim {
    private: DOF4Integrator integrator;
    public:
        double total_time = 0.0;
        void run(RigidBody& body);
};
