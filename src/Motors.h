#pragma once

#include "EigenTypes.h"

#define MOTOR_COUNT 4
#define BASE_PWM 990

class Motors {
    private:
        int *servo_pins;
        int throttle;
        Vector4f adjustments;
    public:
        Motors(int *servo_pins);
        void gpioServoSafe(int pin, int dur);
        void setServosRaw(int dur);
        void setThrottle(int throttle);
        void setAdjustments(Vector4f adjustments);
        void writeSpeeds();
        void calibrate();
};
