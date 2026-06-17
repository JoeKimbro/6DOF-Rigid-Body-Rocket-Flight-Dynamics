#pragma once
#include "RigidBody.h"
// so this will call any class the user selects to avoid repetative code.
class FlightSim {
    public: 
        virtual void run(RigidBody& body) = 0;
        virtual ~FlightSim() = default;
};