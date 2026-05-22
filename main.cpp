#include "1DOF/1DOF.h"
#include "1DOF/Integrator1DOF.h"

int main() {
    RigidBody body;
    Integrator integrator; 
    DOF1 sim; 

    sim.run(body, integrator);
    return 0;
}
