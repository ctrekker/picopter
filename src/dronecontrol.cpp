#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "dronecontrol.h"
#include "EigenTypes.h"
#include "DroneState.h"
#include "math/PIDController.h"
#include "math/utils.h"

ThrustController::ThrustController(float Kpp, float Kip, float Kdp, float Kpq, float Kiq, float Kdq, float dt) {
    this->Kpp = Kpp;
    this->Kip = Kip;
    this->Kdp = Kdp;
    this->Kpq = Kpq;
    this->Kiq = Kiq;
    this->Kdq = Kdq;
    this->dt = dt;

    // instantiate controllers
    translationalController = new PIDController(Kpp, Kip, Kdp, dt);
    rotationalController = new PIDController(Kpq, Kiq, Kdq, dt);
}
ThrustController::~ThrustController() {
    delete translationalController;
    delete rotationalController;
}

Vector4f ThrustController::step(DroneState3d s) {
    const float expected_z = 0.5;
    const Vector3f z_axis(0., 0., 1.);


    // compute translational thrust
    float error_z = expected_z - s.p.z();
    float translationalThrust = translationalController->step(error_z);


    // compute rotational thrust
    // NOTE: the projected axis math assumes the target is the XY-PLANE!!!
    //       this will need to be changed for arbitrary rotation targets

    // scalar error in q
    Vector3f rotated_z = (s.q * realImaginaryQuaternion<float>(0., z_axis) * s.q.inverse()).vec();
    float error_q = -_angleBetween(rotated_z, z_axis);
    float rotationalThrust = rotationalController->step(error_q);
    // directional error in q
    Vector3f projectedAxis = rotated_z - _proj(rotated_z, z_axis);
    projectedAxis /= projectedAxis.norm();

    if(isnan(projectedAxis.x()) || isnan(projectedAxis.y()) || isnan(projectedAxis.z())) {
        projectedAxis = Vector3f(0., 0., 0.);
    }

    float wx = projectedAxis.x();
    float wy = projectedAxis.y();


    // combine PIDs additively with approprate weighting of rotational PID
    return Vector4f(
        translationalThrust + wy * rotationalThrust,
        translationalThrust + wx * rotationalThrust,
        translationalThrust + wy * -rotationalThrust,
        translationalThrust + wx * -rotationalThrust
    );
}
