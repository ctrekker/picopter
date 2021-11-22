#pragma once

#include <Eigen/Dense>

struct CraftProperties {
    Eigen::Vector3f g = Eigen::Vector3f(0, 0, -9.8);
    float mass;
    float radius;
    float rotationalInertia;
    float maxThrust;
};
