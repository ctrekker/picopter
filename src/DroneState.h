#pragma once

#include <vector>
#include <Eigen/Geometry>

using namespace Eigen;

class DroneState3d {
    public:
        Vector3f p;
        Quaternion<float> q;
        Vector3f v;
        float w;     // scalar angular velocity
        Vector3f a;  // axis of rotation

        DroneState3d();
        DroneState3d(Vector3f p, Quaternion<float> q, Vector3f v, float w, Vector3f a);
};

std::ostream& operator<<(std::ostream& os, const DroneState3d& s);
std::ostream& operator<<(std::ostream& os, const std::vector<DroneState3d>& s);
