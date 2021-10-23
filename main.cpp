#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Geometry>
using namespace Eigen;
// #include "src/Quaternion.h"

int main()
{
    Quaternion<float> q1(1., 0., 0., 0.);
    Quaternion<float> q2(0., 1., 0., 0.);

    // std::cout << (q1 + q2) << std::endl;
    std::cout << (q1 * q2) << std::endl;
}
