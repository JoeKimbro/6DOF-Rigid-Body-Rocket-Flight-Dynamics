#pragma once

struct DOF6states {
    //Quaternion (body -> world). Identity (qw=1) means "no rotation".
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    // Body Angular Rates
    double r  = 0.0; // yaw
    double q  = 0.0; // pitch
    double p  = 0.0; // roll
};

// A 3D vector. The caller tracks whether it is in body or world coordinates.
// Plain aggregate (no default initializers) so brace-init like {1,0,0} works.
struct Vec3 { double x, y, z; };

// Rotate a BODY-frame vector into the WORLD frame using the body->world unit
// quaternion. The columns of this matrix are the body axes written in world
// coordinates, so bodyToWorld(Q, {1,0,0}) is the nose direction.
inline Vec3 bodyToWorld(const DOF6states& Q, Vec3 v) {
    const double w = Q.qw, x = Q.qx, y = Q.qy, z = Q.qz;
    return {
        (1 - 2*(y*y + z*z)) * v.x + 2*(x*y - w*z) * v.y + 2*(x*z + w*y) * v.z,
        2*(x*y + w*z) * v.x + (1 - 2*(x*x + z*z)) * v.y + 2*(y*z - w*x) * v.z,
        2*(x*z - w*y) * v.x + 2*(y*z + w*x) * v.y + (1 - 2*(x*x + y*y)) * v.z
    };
}

// Rotate a WORLD-frame vector into the BODY frame (the transpose of the above).
// Used next to read AoA/sideslip from the body-frame velocity.
inline Vec3 worldToBody(const DOF6states& Q, Vec3 v) {
    const double w = Q.qw, x = Q.qx, y = Q.qy, z = Q.qz;
    return {
        (1 - 2*(y*y + z*z)) * v.x + 2*(x*y + w*z) * v.y + 2*(x*z - w*y) * v.z,
        2*(x*y - w*z) * v.x + (1 - 2*(x*x + z*z)) * v.y + 2*(y*z + w*x) * v.z,
        2*(x*z + w*y) * v.x + 2*(y*z - w*x) * v.y + (1 - 2*(x*x + y*y)) * v.z
    };
}