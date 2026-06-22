#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "6DOF.h"

void DOF6::run(RigidBody& body) {
    std::cout << "Fourth Degree of Freedom Simulation (RK4 integration) \n";
    std::cout << "Please input Dry Mass: \n";
    std::cin >> body.props.dryMass;
    std::cout << "Please input Fuel Mass: \n";
    std::cin >> body.props.fuelMass;
    std::cout << "How fast does the engine burn fuel?: \n";
    std::cin >> body.props.mass_flow_rate;
    int engineType;
    std::cout << "Please Select Engine Type: \n";
    std::cout << "1. Solid rocket (APCP)\n";
    std::cout << "2. Kerosene + LOX (like Falcon 9)\n";
    std::cout << "3. Hydrogen + LOX (like Space Shuttle main engine)\n";
    std::cout << "4. Hydrazine (monopropellant)\n";
    std::cout << "Any key to select your own measure of Exhaust Velocity (The top are estimates based of engine)\n";
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
    std::cout << "Input launch compass heading (Azimuth, in degrees. 90 = East, 180 = South, 360/0 = North):\n";
    std::cin >> body.phi_deg;
    body.phi = body.phi_deg * M_PI / 180.0;

    // Initialize the attitude quaternion from the launch angles so the body
    // nose (+x) starts pointing along (sinθ·cosφ, cosθ, sinθ·sinφ) -- i.e. it
    // matches the old theta/phi thrust direction at t=0.
    // q0 = q_yaw(φ about world -y) ⊗ q_pitch((π/2 - θ) about world +z).
    {
        const double a  = M_PI / 2.0 - body.theta;          // pitch up from horizontal
        const double cp = std::cos(a * 0.5), sp = std::sin(a * 0.5);
        const double cy = std::cos(body.phi * 0.5), sy = std::sin(body.phi * 0.5);
        body.sixStates.qw =  cy * cp;
        body.sixStates.qx = -sy * sp;
        body.sixStates.qy = -sy * cp;
        body.sixStates.qz =  cy * sp;
        const double n = std::sqrt(body.sixStates.qw * body.sixStates.qw +
                                   body.sixStates.qx * body.sixStates.qx +
                                   body.sixStates.qy * body.sixStates.qy +
                                   body.sixStates.qz * body.sixStates.qz);
        body.sixStates.qw /= n; body.sixStates.qx /= n;
        body.sixStates.qy /= n; body.sixStates.qz /= n;
    }
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
// add radius
    std::cout << "Provide Radius: \n";
    std::cin >> body.props.radius;
    while(body.props.radius <= 0) {
        std::cout << "Please provide another value for Radius";
        std::cin >> body.props.radius;
    }
    int steps = static_cast<int>(total_time / integrator.dt);
    for (int i = 0; i < steps; i++) {
        integrator.stepDOF6(body);   // RK4 advances the integrated state vector
        integrator.refresh(body);    // recompute forces/diagnostics at the new state
        if (body.vertical.position <= 0.0) break;
        std::cout << "t="    << i * integrator.dt
                  << " mass=" << body.props.mass
                  << " horizontal position=" << body.horizontal.position
                  << " vertical position=" << body.vertical.position
                  << " horizontal velocity=" << body.horizontal.velocity
                  << " vertical velocity=" << body.vertical.velocity
                  << " total speed=" << body.v_total
                  << " AoA=" << body.rotation.AoA
                  << " N_Pitch=" << body.rotation.N_pitch << " N_Yaw=" << body.rotation.N_yaw
                  << " M=" << body.rotation.M
                  << " theta=" << body.theta
                  << " omega=" << body.rotation.omega
                  << " theta_ddot=" << body.rotation.theta_ddot
                  << " Yaw Angle: " << body.phi
                  << " Yaw Rate (r): " << body.rotation.omega_phi
                  << " Roll Rate (p): " << body.sixStates.p
                  << " Sideslip: " << body.rotation.Sideslip
                  << " M_yaw: " << body.rotation.M_yaw
                  << " |q|: " << std::sqrt(body.sixStates.qw*body.sixStates.qw +
                                           body.sixStates.qx*body.sixStates.qx +
                                           body.sixStates.qy*body.sixStates.qy +
                                           body.sixStates.qz*body.sixStates.qz)
                  << "\n";
    }
}
