# pragma once

class PIDController {
    private:
        float integralE;
        float lastError;

    public:
        float kp;
        float ki;
        float kd;
        float dt;

        PIDController(float kp, float ki, float kd, float dt);
        
        float step(float error);
};
