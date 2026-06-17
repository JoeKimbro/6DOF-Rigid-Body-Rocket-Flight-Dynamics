#pragma once 

struct DOF6states {
    //Quaternion
    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    // Body Angular Rates 
    double r  = 0.0; // yaw
    double q  = 0.0; // pitch
    double p  = 0.0; // roll
};