#include <pigpio.h>
#include <Eigen/Dense>
#include "ITG3200.h"
#include "SensorUtils.h"

void ITG3200::init() {
    handle = i2cOpen(1, ITG3200_I2C_ADDR, 0);
    writeTo(handle, ITG3200_DLPF, 3 << 3 | ITG3200_DLPF_256HZ);
    writeTo(handle, ITG3200_POWER_MGMT, ITG3200_CLK_GYROZ);  // all zeros is active, no standby, clocked from gyro z
}

Eigen::Vector3f ITG3200::readXYZ() {
    uint8_t buff[6];
    i2cReadI2CBlockData(handle, ITG3200_GYROX_H, reinterpret_cast<char *>(buff), sizeof(buff));

    int16_t x = buff[0] << 8 | buff[1];
    int16_t y = buff[2] << 8 | buff[3];
    int16_t z = buff[4] << 8 | buff[5];

    return Eigen::Vector3f((float)x / ITG3200_SENSITIVITY, (float)y / ITG3200_SENSITIVITY, (float)z / ITG3200_SENSITIVITY);
}
