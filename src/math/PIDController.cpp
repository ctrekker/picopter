#include "PIDController.h"
#include "utils.h"
#include <math.h>

float rect_fn(float x) {
    return _sign(x) * sqrt(abs(x));
}

PIDController::PIDController(float kp, float ki, float kd, float dt) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;

    this->derivativeFilter = new LowPassFilter(25);

    this->integralE = 0.;
    this->lastError = 0.;
}

PIDController::~PIDController() {
    delete derivativeFilter;
}

float PIDController::step(float error) {
    // update integral
    this->integralE += error * this->dt;

    float filteredError = derivativeFilter->step(error);
    float dEdt = (filteredError - this->lastError) / this->dt;

    float u = this->kp * error + this->ki * this->integralE + this->kd * dEdt;
    this->lastError = filteredError;

    return u;
}

void PIDController::setParameters(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PIDController::clearState() {
    integralE = 0.;
    lastError = 0.;
}
