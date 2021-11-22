#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "EventLoop.h"
#include "DroneState.h"
#include "Simulation.h"
#include "EigenTypes.h"
#include "Motors.h"
#include "dronecontrol.h"
#include "sensors/ADXL345.h"
#include "sensors/ITG3200.h"
#include "math/PIDController.h"
#include "math/utils.h"


ADXL345 acc;
ITG3200 gyro;

RealtimeDroneState state;
std::vector<RealtimeDroneState> stateHistory;
std::vector<Vector3f> accelerationHistory;

const float DEG_TO_RAD = M_PI / 180.;

/* calibration variables */
const Vector3f expectedAcceleration(0., 0., G_ACC);
Vector3f accelerationBias(0., 0., 0.);
float accelerationGain = 1.;
Vector3f angularVelocityBias(0., 0., 0.);

std::vector<Vector3f> calibrationAccelerations;
std::vector<Vector3f> calibrationAngVelocities;

/* control things */
int motorPins[MOTOR_COUNT] = {6, 13, 19, 26};
Motors motors(motorPins);
ThrustController thrustController(0.3, 0.2, 0, 10, 0, 5, 0.01);

int init() {
    if(gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }

    acc.init(ADXL345_BW_200, ADXL345_RANGE_4G);
    gyro.init();

    return 0;
}


void calibrateMotors() {
    motors.calibrate();
}

void setMotorThrottle(int throttle) {
    motors.setThrottle(throttle);
}



void calibrateSensors() {
    Vector3f avgAcc = _mean(calibrationAccelerations);
    Vector3f avgAngVel = _mean(calibrationAngVelocities);

    accelerationBias = -avgAcc;
    accelerationBias += Vector3f(0., 0., 1.); // this is the expected value

    // acceleration calibration
    accelerationGain = G_ACC * 1. / (avgAcc + accelerationBias).norm();

    // angular velocities calibration
    angularVelocityBias = -avgAngVel;
}


void readCalibrationSensors() {
    Vector3f a = acc.readXYZ();
    Vector3f w = gyro.readXYZ();

    calibrationAccelerations.push_back(a);
    calibrationAngVelocities.push_back(w);
}
void readOrientationSensors() {
    Vector3f a = acc.readXYZ();
    Vector3f w = gyro.readXYZ();

    a += accelerationBias;
    a *= accelerationGain;

    Vector3f smoothedAcc(a);
    int r = 10;
    int actualR = 0;
    for(int i=0; i<r; i++) {
        int idx = accelerationHistory.size()-1-i;
        if(idx >= 0) {
            smoothedAcc += accelerationHistory[idx];
            actualR++;
        }
    }
    smoothedAcc /= (actualR + 1);


    accelerationHistory.push_back(a);


    w += angularVelocityBias;
    w *= DEG_TO_RAD;

    state = realtimeTransition(state, a, w, 0.01);
    stateHistory.push_back(state);

    std::cout << state.q.x() << "\t" << state.q.y() << std::endl;


    // PID CONTROL LOOPS
    Vector4f thrusts = thrustController.step(state);
    // std::cout << thrusts(0) << ", " << thrusts(1) << ", " << thrusts(2) << ", " << thrusts(3) << std::endl;

}


void runEventLoop() {
    std::cout << "Calibrating..." << std::endl;
    EventLoop calibrationLoop(1000 * 10, 300); // calibrate for 3 seconds
    calibrationLoop.registerEvent(Event(readCalibrationSensors, 5));
    calibrationLoop.run();

    calibrateSensors();
    std::cout << "Calibration complete" << std::endl;


    stateHistory.push_back(state);

    EventLoop controlLoop(1000 * 10);
    controlLoop.registerEvent(Event(readOrientationSensors, 1));
    controlLoop.run();
    std::cout << stateHistory << std::endl;
}
