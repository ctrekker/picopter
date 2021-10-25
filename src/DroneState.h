#pragma once

#include <Eigen/Geometry>
// #include <Eigen/Dense>

using namespace Eigen;

class DroneState3d {
    public:
        Vector3d p;
        Quaternion<float> q;
        Vector3d v;
        float w;     // scalar angular velocity
        Vector3d a;  // axis of rotation

        DroneState3d();
        DroneState3d(Vector3d p, Quaternion<float> q, Vector3d v, float w, Vector3d a);
};

std::ostream& operator<<(std::ostream& os, const DroneState3d& s);
