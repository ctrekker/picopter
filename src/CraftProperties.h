#pragma once

#include <Eigen/Dense>

struct CraftProperties {
    Vector3f g = Vector3f(0, 0, -9.8);
    float mass;
    float radius;
    float rotationalInertia;
    float maxThrust;
};
