#include <pigpio.h>
#include <unistd.h>
#include <iostream>
#include "Motors.h"

Motors::Motors(int servo_pins[MOTOR_COUNT]) {
    this->servo_pins = servo_pins;
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
    setServosRaw(990 + throttle);
}

void Motors::calibrate() {
    setServosRaw(1100);
	usleep(1000 * 2000);
	setServosRaw(500);
	usleep(1000 * 1500);
}
