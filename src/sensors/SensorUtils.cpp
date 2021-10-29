#include "SensorUtils.h"
#include <pigpio.h>

void writeTo(int handle, uint8_t address, uint8_t val) {
    uint8_t command[2];
    command[0] = address;
    command[1] = val;

    i2cWriteDevice(handle, reinterpret_cast<char *>(command), 2);
}
