#pragma once
#include "RigidBody1DOF.h"

class DOF1 {
    public: 
    double total_time; 
    void run(RigidBody& body);
};