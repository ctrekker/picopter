#include <pigpio.h>
#include <iostream>
#include "BMP280.h"
#include "SensorUtils.h"

void BMP280::init() {
    handle = i2cOpen(1, BMP280_I2C_ADDR, 0);
    setMeasurementCtrl(BMP280_SAMPLE_X2, BMP280_SAMPLE_X16, BMP280_MODE_NORMAL);
    setConfig(BMP280_STANDBY_0_5, BMP280_FILTER_16);
    readCalibration();
}

void BMP280::setMeasurementCtrl(uint8_t temperatureSample, uint8_t pressureSample, uint8_t deviceMode) {
    writeTo(handle, BMP280_CTRL_MEAS, temperatureSample << 5 | pressureSample << 2 | deviceMode);
}

void BMP280::setConfig(uint8_t tStandby, uint8_t filter) {
    writeTo(handle, BMP280_CONFIG, tStandby << 5 | filter << 2);
}

float BMP280::getTemperature() {
    int32_t raw = getTemperatureRaw();
    float rslt = 0;
    int32_t v1, v2;
    v1 = ((((raw >> 3) - ((int32_t) cal.t1 << 1))) * ((int32_t) cal.t2)) >> 11;
    v2 = (((((raw >> 4) - ((int32_t) cal.t1)) * ((raw >> 4) - ((int32_t) cal.t1))) >> 12) * ((int32_t) cal.t3)) >> 14;
    _tFine = v1 + v2;
    rslt = (_tFine * 5 + 128) >> 8;
    return (rslt / 100);
}

float BMP280::getPressure() {
    getTemperature();
    int32_t raw = getPressureRaw();
    int64_t rslt = 0;
    int64_t v1, v2;
    v1 = ((int64_t) _tFine) - 128000;
    v2 = v1 * v1 * (int64_t) cal.p6;
    v2 = v2 + ((v1 * (int64_t) cal.p5) << 17);
    v2 = v2 + (((int64_t) cal.p4) << 35);
    v1 = ((v1 * v1 * (int64_t) cal.p3) >> 8) + ((v1 * (int64_t) cal.p2) << 12);
    v1 = (((((int64_t) 1) << 47) + v1)) * ((int64_t) cal.p1) >> 33;
    if(v1 == 0)
        return 0;
    rslt = 1048576 - raw;
    rslt = (((rslt << 31) - v2) * 3125) / v1;
    v1 = (((int64_t) cal.p9) * (rslt >> 13) * (rslt >> 13)) >> 25;
    v2 = (((int64_t) cal.p8) * rslt) >> 19;
    rslt = ((rslt + v1 + v2) >> 8) + (((int64_t) cal.p7) << 4);
    return (rslt / 256.);
}

uint32_t BMP280::getTemperatureRaw() {
    uint8_t tempBuff[3];
    i2cReadI2CBlockData(handle, BMP280_TEMP_MSB, reinterpret_cast<char *>(&tempBuff), sizeof(tempBuff));
    return ((uint32_t)tempBuff[0] << 12) | ((uint32_t)tempBuff[1] << 4) | ((uint32_t)tempBuff[0]);
}
uint32_t BMP280::getPressureRaw() {
    uint8_t presBuff[3];
    i2cReadI2CBlockData(handle, BMP280_TEMP_MSB, reinterpret_cast<char *>(&presBuff), sizeof(presBuff));
    return ((uint32_t)presBuff[0] << 12) | ((uint32_t)presBuff[1] << 4) | ((uint32_t)presBuff[0]);
}

void BMP280::readCalibration() {
    i2cReadI2CBlockData(handle, BMP280_CALIB, reinterpret_cast<char *>(&cal), sizeof(cal));
}


std::ostream& operator<<(std::ostream& os, const BMP280Calibration& c) {
    os << "BMP280Calibration {" << std::endl;
    os << "\t" << "t1 = " << c.t1 << std::endl;
    os << "\t" << "t2 = " << c.t2 << std::endl;
    os << "\t" << "t3 = " << c.t3 << std::endl;
    os << std::endl;
    os << "\t" << "p1 = " << c.p1 << std::endl;
    os << "\t" << "p2 = " << c.p2 << std::endl;
    os << "\t" << "p3 = " << c.p3 << std::endl;
    os << "\t" << "p4 = " << c.p4 << std::endl;
    os << "\t" << "p5 = " << c.p5 << std::endl;
    os << "\t" << "p6 = " << c.p6 << std::endl;
    os << "\t" << "p7 = " << c.p7 << std::endl;
    os << "\t" << "p8 = " << c.p8 << std::endl;
    os << "\t" << "p9 = " << c.p9 << std::endl;
    os << std::endl;
    os << "\t" << "reserved0 = " << c.reserved0 << std::endl;
    os << "}" << std::endl;
    return os;
}
