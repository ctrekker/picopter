#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;

    this->integralE = 0.;
    this->lastError = 0.;
}

float PIDController::step(float error) {
    // update integral
    this->integralE += error * this->dt;

    float dEdt = (error - this->lastError) / this->dt;
    float u = this->kp * error + this->ki * this->integralE + this->kd * dEdt;
    this->lastError = error;

    return u;
}
