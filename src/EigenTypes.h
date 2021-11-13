#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 4, 1> Vector4f;

template <typename T>
Eigen::Quaternion<T> realImaginaryQuaternion(T real, Eigen::Vector3<T> imaginary) {
    return Eigen::Quaternion<T>(real, imaginary.x(), imaginary.y(), imaginary.z());
}
template <typename T>
Eigen::Quaternion<T> angleAxisQuaternion(float angle, Eigen::Vector3f axis) {
    Eigen::Vector3<T> im = sin(angle/2) * axis;
    return realImaginaryQuaternion<T>(cos(angle/2), im);
}
