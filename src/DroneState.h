#pragma once

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class DroneState3d {
    public:
        Eigen::Vector3f p;
        Eigen::Quaternionf q;
        Eigen::Vector3f v;
        float w;            // scalar angular velocity
        Eigen::Vector3f a;  // axis of rotation

        DroneState3d();
        DroneState3d(Eigen::Vector3f p, Eigen::Quaternionf q, Eigen::Vector3f v, float w, Eigen::Vector3f a);
};

class RealtimeDroneState {
    public:
        Eigen::Vector2f q;  // RELATIVE orientation (that we care about (xy)) (it's super janky (see my math in notebook))
        Eigen::Vector3f w;  // angular velocity

        RealtimeDroneState();
        RealtimeDroneState(Eigen::Vector2f q, Eigen::Vector3f w);
};

std::ostream& operator<<(std::ostream& os, const DroneState3d& s);
std::ostream& operator<<(std::ostream& os, const std::vector<DroneState3d>& s);

std::ostream& operator<<(std::ostream& os, const RealtimeDroneState& s);
std::ostream& operator<<(std::ostream& os, const std::vector<RealtimeDroneState>& s);
