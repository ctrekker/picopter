#include <Eigen/Dense>
#include <pigpio.h>
#include <math.h>
#include "ADXL345.h"
#include "SensorUtils.h"

using namespace Eigen;

void ADXL345::init(uint8_t bandwidth, uint8_t range) {
    handle = i2cOpen(1, ADXL345_I2C_ADDR, 0);
    writeTo(handle, ADXL345_POWER_CTL, 8);  // set measure mode = 1
    writeTo(handle, ADXL345_DATA_FORMAT, range);  // set range to +-4g
    writeTo(handle, ADXL345_BW_RATE, bandwidth);

    scale = exp2(range + 1);
}

Vector3f ADXL345::readXYZ() {
    uint8_t buff[ADXL345_READ_LENGTH];
    i2cReadI2CBlockData(handle, ADXL345_DATAX0, reinterpret_cast<char *>(buff), ADXL345_READ_LENGTH);

    int16_t x = (((int)buff[1]) << 8) | buff[0];  
    int16_t y = (((int)buff[3]) << 8) | buff[2];
    int16_t z = (((int)buff[5]) << 8) | buff[4];

    return Vector3f((float)scale * x / 512., (float)scale * y / 512., (float)scale * z / 512.);
}
