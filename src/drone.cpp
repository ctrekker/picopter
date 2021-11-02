#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "EventLoop.h"
#include "DroneState.h"
#include "EigenTypes.h"
#include "sensors/ADXL345.h"
#include "sensors/ITG3200.h"

#define G_ACC 9.80665


ADXL345 acc;
ITG3200 gyro;

DroneState3d state;

/* calibration variables */
const Vector3f expectedAcceleration(0., 0., G_ACC);
float accelerationGain = 1.;
Quaternionf accelerationRotation(0., 0., 0., 0.);

Vector3f angularVelocityBias(0., 0., 0.);
std::vector<Vector3f> calibrationAccelerations;
std::vector<Vector3f> calibrationAngVelocities;

int init() {
    if(gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }

    acc.init(ADXL345_BW_200, ADXL345_RANGE_4G);
    gyro.init();

    return 0;
}


Vector3f _sum(std::vector<Vector3f> vecs) {
    Vector3f agg(0., 0., 0.);

    for(int i=0; i<vecs.size(); i++) {
        agg += vecs[i];
    }

    return agg;
}
Vector3f _mean(std::vector<Vector3f> vecs) {
    return _sum(vecs) / vecs.size();
}
float _angleBetween(Vector3f vec1, Vector3f vec2) {
    float dotProduct = vec1.transpose() * vec2;
    float normProduct = vec1.norm() * vec2.norm();
    float in = dotProduct / normProduct;
    return acos(in);
}


void calibrateSensors() {
    Vector3f avgAcc = _mean(calibrationAccelerations);
    Vector3f avgAngVel = _mean(calibrationAngVelocities);

    accelerationGain = G_ACC * 1. / avgAcc.norm();

    Vector3f normAcc = avgAcc * accelerationGain;
    Vector3f rotAxis = normAcc.cross(expectedAcceleration);
    rotAxis /= rotAxis.norm();

    accelerationRotation = angleAxisQuaternion<float>(_angleBetween(normAcc, expectedAcceleration), rotAxis);
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

    a *= accelerationGain;
}


void runEventLoop() {
    std::cout << "Calibrating..." << std::endl;
    EventLoop calibrationLoop(1000 * 10, 300); // calibrate for 3 seconds
    calibrationLoop.registerEvent(Event(readCalibrationSensors, 5));
    calibrationLoop.run();

    calibrateSensors();
    std::cout << "Calibration complete" << std::endl;
}
