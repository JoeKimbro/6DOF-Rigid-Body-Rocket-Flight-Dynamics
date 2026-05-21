#include <iostream>
#include "1DOF.h"
// Now we calculate the first degree of freedom ( 'merica )

void DOF1::run(RigidBody& body) {
    std::cout << "First Degree of Freedom Simulation \n"; 
    std::cout << "Please input Dry Mass: \n";
    std::cin >> body.dryMass;
    std::cout << "Please input Fuel Mass: \n";
    std::cin >> body.fuelMass;
    std::cout << "How fast does the engine burn fuel?: \n";
    std::cin >> body.mass_flow_rate;
    int engineType; 
    std::cout << "Please Select Engine Type: \n";
    std::cout << "1. Solid rocket (APCP)\n";
    std::cout <<"2. Kerosene + LOX (like Falcon 9)\n";
    std::cout <<"3. Hydrogen + LOX (like Space Shuttle main engine)\n";
    std::cout <<"4. Hydrazine (monopropellant)\n";
    std::cout<< "Any key to select your own measure of Exhaust Velocity (The top are estimates based of engine)\n";
    std::cin >> engineType; 
    switch (engineType) {
        case 1:
        body.Ve = 2500; break;
        case 2:
        body.Ve = 3050; break;
        case 3: 
        body.Ve = 4400; break;
        case 4: 
        body.Ve = 2200;
            break;
        case 27: 
            std::cout << "Exiting...\n";
            break;
        default: 
            std::cout << "Please provide your measure of Ve: \n";
            std::cin >> body.Ve; 
            break;
    }
    std::cout << "Provide Drag Coefficient: \n";
    std::cin >> body.Cd;
    std::cout << "Provide A (Cross Sectional reference Area) Coefficient: \n";
    std::cin >> body.A; 
    std::cout << "Provide Position / Starting Altitude (If not provided starting altitude will be at sea level): \n";
    std::cin >> body.position; 
    std::cout << "Time steps will be 0.01 seconds (dt), provide total time: \n";
    std::cin >> total_time; 
    
}


/*
  ├─────────────────────────────────────────────────┼──────────┤
  │ Solid rocket (APCP)                             │ ~2,500   │
  ├─────────────────────────────────────────────────┼──────────┤
  │ Kerosene + LOX (like Falcon 9)                  │ ~3,050   │
  ├─────────────────────────────────────────────────┼──────────┤
  │ Hydrogen + LOX (like Space Shuttle main engine) │ ~4,400   │
  ├─────────────────────────────────────────────────┼──────────┤
  │ Hydrazine (monopropellant)                      │ ~2,200   |
  */