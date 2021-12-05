#pragma once

#include <Eigen/Dense>
#include "EigenTypes.h"
#include "DroneState.h"
#include "math/PIDController.h"

class ThrustController {
    private:
        float Kpq1, Kiq1, Kdq1;
        float Kpq2, Kiq2, Kdq2;
        float dt;

        Eigen::Vector2f target;

        PIDController* rotationXController;
        PIDController* rotationYController;
    public:
        ThrustController(float Kpq1, float Kiq1, float Kdq1, float Kpq2, float Kiq2, float Kdq2, float dt);
        ~ThrustController();

        Vector4f step(RealtimeDroneState s);
        void setTarget(Eigen::Vector2f target);
        void setParameters(float p1, float i1, float d1, float p2, float i2, float d2);
};
