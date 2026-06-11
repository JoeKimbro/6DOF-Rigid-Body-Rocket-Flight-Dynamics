#pragma once
#include "StateVariables/RigidBody.h"
#include "StateVariables/FlightSim.h"
#include "5DOF_RK4.h"

class DOF5 : public FlightSim {
    private: DOF5Integrator integrator;
    public:
        double total_time = 0.0;
        void run(RigidBody& body);
};
