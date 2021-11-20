#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pigpio.h>

using namespace Eigen;

#include "src/DroneState.h"
#include "src/Simulation.h"
#include "src/CraftProperties.h"
#include "src/Timer.h"

// #include "src/sensors/ADXL345.h"
// #include "src/sensors/ITG3200.h"
// #include "src/sensors/BMP280.h"
#include "src/sensors/HMC5883.h"

// #define SEA_LEVEL_PRESSURE    1015.0f   // sea level pressure

Vector4f noThrust(DroneState3d s, float t) {
    return Vector4f(0, 0, 0, 0);
}

void simulate() {
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
}

int main()
{
    if(gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }

    // ADXL345 acc;
    // ITG3200 gyro;
    // BMP280 bar;
    HMC5883 mag;
    // acc.init();
    // gyro.init();
    // bar.init();
    mag.init();

    for(int i=0 ;; i++) {
        Vector3f w = mag.readXYZ();
        std::cout << w.x() << "\t" << w.y() << "\t" << w.z() << std::endl;
        time_sleep(1./10);
    }

    return 0;
}
