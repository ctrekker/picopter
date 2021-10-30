#pragma once

#include <Eigen/Dense>

#define ITG3200_I2C_ADDR 0x68

// Registers
#define ITG3200_WHO_AM_I   0x00
#define ITG3200_SMPLRT_DIV 0x15
#define ITG3200_DLPF       0x16
#define ITG3200_INT_CONFIG 0x17
#define ITG3200_INT_STATUS 0x1A
#define ITG3200_TEMP_H     0x1B
#define ITG3200_TEMP_L     0x1C
#define ITG3200_GYROX_H    0x1D
#define ITG3200_GYROX_L    0x1E
#define ITG3200_GYROY_H    0x1F
#define ITG3200_GYROY_L    0x20
#define ITG3200_GYROZ_H    0x21
#define ITG3200_GYROZ_L    0x22
#define ITG3200_POWER_MGMT 0x3E

// DLPF (Digital Low Pass Filter) modes
#define ITG3200_DLPF_256HZ 0x0  // internal sample rate 8kHz
#define ITG3200_DLPF_188HZ 0x1  // internal sample rate 1kHz
#define ITG3200_DLPF_98HZ  0x2  // |
#define ITG3200_DLPF_42HZ  0x3  // v
#define ITG3200_DLPF_20HZ  0x4
#define ITG3200_DLPF_10HZ  0x5
#define ITG3200_DLPF_5HZ   0x6

// Clock selection
#define ITG3200_CLK_INTERNAL     0x0
#define ITG3200_CLK_GYROX        0x1
#define ITG3200_CLK_GYROY        0x2
#define ITG3200_CLK_GYROZ        0x3
#define ITG3200_CLK_EXT_32768KHZ 0x4
#define ITG3200_CLK_EXT_192MHZ   0x5

// Calibration constants
#define ITG3200_SENSITIVITY 14.375


class ITG3200 {
    private:
        int handle;
    public:
        void init();
        Eigen::Vector3f readXYZ();
};
