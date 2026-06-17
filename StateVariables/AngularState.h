#pragma once 


struct AngularState {
    double omega = 0.0; 
    double pitch_acceleration = 0.0;
    double AoA = 0.0; // Angle of attack, α = theta - atan2(vx, vy)
    double Sideslip = 0.0; // β = phi - atan2(vz, vx) in the horizontal plane
    double M = 0.0; 
    double M_yaw = 0.0;
    double N_pitch = 0.0;
    double N_yaw = 0.0;
    double theta_ddot = 0.0;  // angular acceleration 
    double omega_phi = 0.0; 
    double phi_ddot = 0.0; 
};

/*
theta_ddot = M / I_yy
  omega     += theta_ddot * dt
  theta     += omega * dt
M = Cn_alpha * q_bar * A * L_ref * alpha * (CP - CG)
θ̈ = M / I_yy
*/
