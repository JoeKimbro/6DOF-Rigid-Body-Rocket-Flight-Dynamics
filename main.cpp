#include "1DOF/1DOF.h"
#include "2DOF/2DOF.h"  
#include "3DOF/3DOF.h"
#include "4DOF/4DOF.h"
#include "5DOF/5DOF.h"
#include "6DOF/6DOF.h"
#include <memory>
#include <iostream>

int main() {
    int DOF;
do {
    RigidBody body;
    std::cout << "1. 1DOF SIM \n2. 2DOF SIM\n3. 3DOF SIM\n4. 4DOF SIM (RK4)\n 5. 5DOF SIM (RK4)\n 6. 6DOF SIM (RK4)\n0. Exit\n";
    std::cin >> DOF; 
    std::unique_ptr<FlightSim> sim;
    switch(DOF){
            case 1: 
            sim = std::make_unique<DOF1>();
            break;
            case 2: 
            sim = std::make_unique<DOF2>();
            break;
            case 3:
            sim = std::make_unique<DOF3>();
            break;
            case 4:
            sim = std::make_unique<DOF4>();
            break;
            case 5:
            sim = std::make_unique<DOF5>();
            break;
            case 6:
            sim = std::make_unique<DOF6>();
            break;
            case 0:
            std::cout << "Exiting..."; 
            break;
        }
        if (DOF != 0){sim->run(body);}
    } while(DOF != 0);
}
