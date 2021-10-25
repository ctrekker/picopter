#include <iostream>
#include <vector>
#include <string>

#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;

#include "src/DroneState.h"
#include "src/Simulation.h"
#include "src/CraftProperties.h"
#include "src/Timer.h"

Vector4f noThrust(DroneState3d s, float t) {
    return Vector4f(0, 0, 0, 0);
}

int main()
{
    DroneState3d initialState(
        Vector3f(0, 0, 0),
        // angleAxisQuaternion<float>(M_PI / 2, Vector3f(1., 0, 0)),
        Quaternionf(1, 0, 0, 0),
        Vector3f(1, 0, 0),
        M_PI / 4,
        Vector3f(1, 0, 0)
    );

    std::cout << "Initial state: ";
    std::cout << initialState << std::endl;

    CraftProperties properties = {
        Vector3f(0, 0, -9.8), // g
        1,   // mass
        0.5, // radius
        1,   // rotational inertia
        100  // max thrust
    };

    Timer t;
    t.start();
    std::vector<DroneState3d> states = simulate(properties, initialState, noThrust, 0.001, 60);
    std::cout << "Simulation took " << t.stop() << "ms" << std::endl;

    // std::cout << "===== Simulation Data =====" << std::endl;
    // std::cout << states << std::endl;

    return 0;
}
