#pragma once

#include <Eigen/Dense>
#include "EigenTypes.h"
#include "DroneState.h"
#include "math/PIDController.h"

class ThrustController {
    private:
        float Kpq, Kiq, Kdq;
        float dt;

        Eigen::Vector2f target;

        PIDController* rotationXController;
        PIDController* rotationYController;
    public:
        ThrustController(float Kpq, float Kiq, float Kdq, float dt);
        ~ThrustController();

        Vector4f step(RealtimeDroneState s);
        void setTarget(Eigen::Vector2f target);
};
