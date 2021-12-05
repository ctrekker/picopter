#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "dronecontrol.h"
#include "EigenTypes.h"
#include "DroneState.h"
#include "math/PIDController.h"
#include "math/utils.h"

#include <iostream>

using namespace Eigen;

ThrustController::ThrustController(float Kpq1, float Kiq1, float Kdq1, float Kpq2, float Kiq2, float Kdq2, float dt) {
    this->Kpq1 = Kpq1;
    this->Kiq1 = Kiq1;
    this->Kdq1 = Kdq1;
    this->Kpq2 = Kpq2;
    this->Kiq2 = Kiq2;
    this->Kdq2 = Kdq2;
    this->dt = dt;
    
    this->target = Vector2f(0., 0.);

    // instantiate controllers
    rotationXController = new PIDController(Kpq1, Kiq1, Kdq1, dt);
    rotationYController = new PIDController(Kpq2, Kiq2, Kdq2, dt);
}
ThrustController::~ThrustController() {
    delete rotationXController;
    delete rotationYController;
}

// this returns adjustments for now
Vector4f ThrustController::step(RealtimeDroneState s) {
    float xAdjustment = rotationXController->step(target.x() - s.q.x());
    float yAdjustment = rotationYController->step(target.y() - s.q.y());

    return Vector4f(
        -yAdjustment,
         xAdjustment,
         yAdjustment,
        -xAdjustment
    );
}

void ThrustController::setTarget(Vector2f target) {
    this->target = target;
}

void ThrustController::setParameters(float p1, float i1, float d1, float p2, float i2, float d2) {
    rotationXController->clearState();
    rotationYController->clearState();
    rotationXController->setParameters(p1, i1, d1);
    rotationYController->setParameters(p2, i2, d2);
}
