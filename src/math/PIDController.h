# pragma once

#include "LowPassFilter.h"

class PIDController {
    private:
        float integralE;
        float lastError;
        LowPassFilter *derivativeFilter;

    public:
        float kp;
        float ki;
        float kd;
        float dt;

        PIDController(float kp, float ki, float kd, float dt);
        ~PIDController();
        
        float step(float error);
        void setParameters(float p, float i, float d);
        void clearState();
};
