#pragma once

#define BMP280_I2C_ADDR 0x77

// Registers
#define BMP280_CALIB      0x88
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0
#define BMP280_STATUS     0xF3
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_CONFIG     0xF5
#define BMP280_PRESS_MSB  0xF7
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_TEMP_MSB   0xFA
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_XLSB  0xFC

// Measurement modes
#define BMP280_MODE_SLEEP  0x0
#define BMP280_MODE_FORCED 0x1 // or 0x2
#define BMP280_MODE_NORMAL 0x3

// Sampling modes
#define BMP280_SAMPLE_X0  0x0
#define BMP280_SAMPLE_X1  0x1
#define BMP280_SAMPLE_X2  0x2
#define BMP280_SAMPLE_X8  0x4
#define BMP280_SAMPLE_X16 0x5 // or 0x6 or 0x7

// Standby modes
#define BMP280_STANDBY_0_5  0x0
#define BMP280_STANDBY_62_5 0x1
#define BMP280_STANDBY_125  0x2
#define BMP280_STANDBY_250  0x3
#define BMP280_STANDBY_500  0x4
#define BMP280_STANDBY_1000 0x5
#define BMP280_STANDBY_2000 0x6
#define BMP280_STANDBY_4000 0x7

// Filter modes
#define BMP280_FILTER_OFF 0x0
#define BMP280_FILTER_2   0x1
#define BMP280_FILTER_4   0x2
#define BMP280_FILTER_8   0x3
#define BMP280_FILTER_16  0x4


typedef struct {
    uint16_t    t1;
    int16_t     t2, t3;
    uint16_t    p1;
    int16_t     p2, p3, p4, p5, p6, p7, p8, p9;
    uint16_t    reserved0;
} BMP280Calibration;

std::ostream& operator<<(std::ostream& os, const BMP280Calibration& s);

class Pressure {
    private:
        float pa;
    public:
        Pressure(float pa);

        float pascals();
        float altitude();  // convert pressure to altitude (on earth)
};

class BMP280 {
    private:
        int handle;
        BMP280Calibration cal;
        int32_t _tFine;
    public:
        void init();
        void readXYZ();
        void readCalibration();

        void setMeasurementCtrl(uint8_t temperatureSample, uint8_t pressureSample, uint8_t deviceMode);
        void setConfig(uint8_t tStandby, uint8_t filter);

        float getTemperature();
        Pressure getPressure();
        uint32_t getTemperatureRaw();
        uint32_t getPressureRaw();
};
