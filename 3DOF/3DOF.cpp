#define _USE_MATH_DEFINES
#include <cmath>
// body.theta = theta_deg * M_PI / 180.0;'
#include <iostream>
#include "3DOF.h"


void DOF3::run(RigidBody& body) {
    std::cout << "Third Degree of Freedom Simulation \n"; 
    std::cout << "Please input Dry Mass: \n";
    std::cin >> body.props.dryMass;
    std::cout << "Please input Fuel Mass: \n";
    std::cin >> body.props.fuelMass;
    std::cout << "How fast does the engine burn fuel?: \n";
    std::cin >> body.props.mass_flow_rate;
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
        body.propul.Ve = 2500; break;
        case 2:
        body.propul.Ve = 3050; break;
        case 3: 
        body.propul.Ve = 4400; break;
        case 4: 
        body.propul.Ve = 2200;
            break;
        case 27: 
            std::cout << "Exiting...\n";
            break;
        default: 
            std::cout << "Please provide your measure of Ve: \n";
            std::cin >> body.propul.Ve; 
            break;
    }
    std::cout << "Provide Drag Coefficient: \n";
    std::cin >> body.propul.Cd;
    std::cout << "Provide A (Cross Sectional reference Area) Coefficient: \n";
    std::cin >> body.propul.A; 
    std::cout << "Provide Position / Starting Altitude (If not provided starting altitude will be at sea level): \n";
    std::cin >> body.vertical.position;
    std::cout << "Time steps will be 0.01 seconds (dt), provide total time: \n";
    std::cin >> total_time;
    std::cout << "Input launch angle: (angle from vertical, in degrees, 0 = straight up.)\n";
    std::cin >> body.theta_deg;
    body.theta = body.theta_deg * M_PI / 180.0;
    std::cout << "Provide CP (Center of Pressure) Coefficient: \n";
    std::cin >> body.propul.CP;
    std::cout << "Provide CG (Center of Gravity) Coefficient: \n";
    std::cin >> body.propul.CG;
    std::cout << "Provide L_ref (Reference Length) Coefficient: \n";
    std::cin >> body.propul.L_ref;
    std::cout << "Provide C_mq (Coefficient of Damping Moment) Coefficient: \n";
    std::cin >> body.propul.C_mq;
    std::cout << "Provide Cn_alpha (Coefficient of Normal Force) Coefficient: \n";
    std::cin >> body.propul.Cn_alpha;

    int steps = static_cast<int>(total_time / integrator.dt);
    for (int i = 0; i < steps; i++) {
        integrator.stepDOF3(body);
        if (body.vertical.position <= 0.0) break;
        std::cout << "t="    << i * integrator.dt
                  << " mass=" << body.props.mass
                  << " horizontal position=" << body.horizontal.position
                  << " vertical position=" << body.vertical.position
                  << " horizontal velocity=" << body.horizontal.velocity
                  << " vertical velocity=" << body.vertical.velocity
                  << " total speed=" << body.v_total
                  << " AoA=" << body.rotation.AoA
                  << " N=" << body.props.N
                  << " M=" << body.rotation.M
                  << " theta=" << body.theta
                  << " omega=" << body.rotation.omega
                  << " theta_ddot=" << body.rotation.theta_ddot
                  << "\n";
    }             
}