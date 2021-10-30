#include <pigpio.h>
#include <Eigen/Dense>
#include "HMC5883.h"
#include "SensorUtils.h"

#include <iostream>

void HMC5883::init() {
    handle = i2cOpen(1, HMC5883_I2C_ADDR, 0);
    std::cout << handle << std::endl;
    // writeTo(handle, HMC5883_CFA, HMC5883_SAMPLES_8 << 5 | HMC5883_DO_75 << 2 | HMC5883_MS_NORMAL);
    // writeTo(handle, HMC5883_MODE, HMC5883_MD_CONTINUOUS);
    writeTo(handle, 0x09, 1);
    writeTo(handle, 0x0b, 1);

    std::cout << (int)readFrom(handle, 0x06) << std::endl;
}

Eigen::Vector3f HMC5883::readXYZ() {
    uint8_t buff[6];
    std::cout << i2cReadI2CBlockData(handle, 0x1, reinterpret_cast<char *>(buff), sizeof(buff)) << std::endl;

    int16_t x = buff[0] << 8 | buff[1];
    int16_t y = buff[4] << 8 | buff[5];
    int16_t z = buff[2] << 8 | buff[3];

    return Eigen::Vector3f(x, y, z);
}
