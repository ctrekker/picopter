#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Geometry>
using namespace Eigen;
#include "src/DroneState.h"

int main()
{
    DroneState3d s;

    std::cout << s << std::endl;

    return 0;
}
