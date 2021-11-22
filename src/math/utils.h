#pragma once

#include <vector>
#include <Eigen/Dense>


Eigen::Vector3f _sum(std::vector<Eigen::Vector3f> vecs);
Eigen::Vector3f _mean(std::vector<Eigen::Vector3f> vecs);
float _dot(Eigen::Vector3f u, Eigen::Vector3f v);
float _angleBetween(Eigen::Vector3f vec1, Eigen::Vector3f vec2);
// project u onto v
Eigen::Vector3f _proj(Eigen::Vector3f u, Eigen::Vector3f v);

template <typename T> int _sign(T val) {
    return (T(0) < val) - (val < T(0));
}
