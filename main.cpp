#include "1DOF/1DOF.h"
#include "2DOF/2DOF.h"
#include <memory>
#include <iostream>

int main() {
    int DOF;
do {
    RigidBody body;
    std::cout << "1. 1DOF SIM \n2. 2DOF SIM\n0. Exit\n";
    std::cin >> DOF; 
    std::unique_ptr<FlightSim> sim;
    switch(DOF){
            case 1: 
            sim = std::make_unique<DOF1>();
            break;
            case 2: 
            sim = std::make_unique<DOF2>();
            break;

            case 0: 
            std::cout << "Exiting..."; 
            break;
        }
        if (DOF != 0){sim->run(body);}
    } while(DOF != 0);
}
