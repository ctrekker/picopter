#pragma once

void init();
void calibrateMotors();
void setMotorThrottle(int throttle);
void setControllerParameters(float p1, float i1, float d1, float p2, float i2, float d2);
void setTargetAngle(float componentX, float componentY);
void runEventLoop();
