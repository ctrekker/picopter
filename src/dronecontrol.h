#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "EigenTypes.h"
#include "DroneState.h"
#include "math/PIDController.h"

class ThrustController {
    private:
        float Kpp, Kip, Kdp;
        float Kpq, Kiq, Kdq;
        float dt;

        PIDController* translationalController;
        PIDController* rotationalController;
    public:
        ThrustController(float Kpp, float Kip, float Kdp, float Kpq, float Kiq, float Kdq, float dt);
        ~ThrustController();

        Vector4f step(RealtimeDroneState s);
};
