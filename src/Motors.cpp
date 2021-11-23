#include <pigpio.h>
#include <unistd.h>
#include <iostream>
#include "Motors.h"

Motors::Motors(int servo_pins[MOTOR_COUNT]) {
    this->servo_pins = servo_pins;
    this->throttle = 0;
    this->adjustments = Vector4f(0., 0., 0., 0.);
}

void Motors::gpioServoSafe(int pin, int dur) {
    if(dur > 2000) {
        dur = 2000;
    }
    gpioServo(pin, dur);
}
void Motors::setServosRaw(int dur) {
	for(int i=0; i<MOTOR_COUNT; i++) {
		gpioServo(servo_pins[i], dur);
	}
}

// takes a value 0 to 1000
void Motors::setThrottle(int throttle) {
    if(throttle < 0 || throttle > 1000) {
        std::cout << "ERROR: throttle is capped at 1000 (provided " << throttle << ")" << std::endl;
        return;
    }
    this->throttle = throttle;
}

void Motors::setAdjustments(Vector4f adjustments) {
    this->adjustments = adjustments;
}

void Motors::writeSpeeds() {
    gpioServoSafe(servo_pins[0], BASE_PWM + throttle + adjustments[0]);
    gpioServoSafe(servo_pins[1], BASE_PWM + throttle + adjustments[1]);
    gpioServoSafe(servo_pins[2], BASE_PWM + throttle + adjustments[2]);
    gpioServoSafe(servo_pins[3], BASE_PWM + throttle + adjustments[3]);
}

void Motors::calibrate() {
    setServosRaw(1100);
	usleep(1000 * 2000);
	setServosRaw(500);
	usleep(1000 * 1500);
}
