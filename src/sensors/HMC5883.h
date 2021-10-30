#pragma once

#include <Eigen/Dense>

#define HMC5883_I2C_ADDR 0x0c

// Registers
#define HMC5883_CFA      0x00
#define HMC5883_CFB      0x01
#define HMC5883_MODE     0x02
#define HMC5883_OUTX_MSB 0x03
#define HMC5883_OUTX_LSB 0x04
#define HMC5883_OUTZ_MSB 0x05
#define HMC5883_OUTZ_LSB 0x06
#define HMC5883_OUTY_MSB 0x07
#define HMC5883_OUTY_LSB 0x08
#define HMC5883_STATUS   0x09
#define HMC5883_IDA      0x0a
#define HMC5883_IDB      0x0b
#define HMC5883_IDC      0x0c

// Samples per measurement
#define HMC5883_SAMPLES_1 0x0
#define HMC5883_SAMPLES_2 0x1
#define HMC5883_SAMPLES_4 0x2
#define HMC5883_SAMPLES_8 0x3

// Data output (Hz)
#define HMC5883_DO_0_75     0x0
#define HMC5883_DO_1_5      0x1
#define HMC5883_DO_3        0x2
#define HMC5883_DO_7_5      0x3
#define HMC5883_DO_15       0x4
#define HMC5883_DO_30       0x5
#define HMC5883_DO_75       0x6
#define HMC5883_DO_RESERVED 0x7

// Measurement modes
#define HMC5883_MS_NORMAL   0x0
#define HMC5883_MS_BIAS_POS 0x1
#define HMC5883_MS_BIAS_NEG 0x2
#define HMC5883_MS_RESERVED 0x3

// Device modes
#define HMC5883_MD_CONTINUOUS 0x0
#define HMC5883_MD_SINGLE     0x1
#define HMC5883_MD_IDLE       0x2 // or 0x3


class HMC5883 {
    private:
        int handle;
    public:
        void init();
        Eigen::Vector3f readXYZ();
};
