#pragma once
#include "StateVariables/RigidBody.h"
#include "Integrator1DOF.h"
#include "StateVariables/FlightSim.h"

class DOF1 : public FlightSim {
    private: Integrator integrator;
    public:
        double total_time = 0.0;
        void run(RigidBody& body);
};  