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

ThrustController::ThrustController(float Kpq, float Kiq, float Kdq, float dt) {
    this->Kpq = Kpq;
    this->Kiq = Kiq;
    this->Kdq = Kdq;
    this->dt = dt;
    
    this->target = Vector2f(0., 0.);

    // instantiate controllers
    rotationXController = new PIDController(Kpq, Kiq, Kdq, dt);
    rotationYController = new PIDController(Kpq, Kiq, Kdq, dt);
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
        -xAdjustment - yAdjustment,
         xAdjustment - yAdjustment,
         xAdjustment + yAdjustment,
        -xAdjustment + yAdjustment
    );
}

void ThrustController::setTarget(Vector2f target) {
    this->target = target;
}
