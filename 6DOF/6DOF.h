#pragma once
#include "StateVariables/RigidBody.h"
#include "StateVariables/FlightSim.h"
#include "6DOF_RK4.h"

class DOF6 : public FlightSim {
    private: DOF6Integrator integrator;
    public:
        double total_time = 0.0;
        void run(RigidBody& body);
};
