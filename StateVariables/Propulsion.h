#pragma once 

struct PropulsionProps {
    double Cd = 0.0; // Drag coefficient 
    double A = 0.0; // Reference area 
    double thrust = 0.0; // Deciding if i want to add more in depth propulsion analysis, but for now simple thurst equation
    double Ve = 0.0; // User will set theirs depending on the engine to make 1DOF easier 
    double Cn_alpha = 0.0; 
    double CP = 0.0;
    double C_mq = 0.0;
    double CG = 0.0;
    double L_ref = 0.0; 
    double alpha = 0.0; 
};

//  The chain: AoA → normal force (via Cn_alpha) → pitching moment (via CP−CG arm) → angular acceleration (via I_yy) → omega → theta.