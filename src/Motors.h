#pragma once

#define MOTOR_COUNT 4

class Motors {
    private:
        int *servo_pins;
    public:
        Motors(int *servo_pins);
        void setServosRaw(int dur);
        void setThrottle(int throttle);
        void calibrate();
};
