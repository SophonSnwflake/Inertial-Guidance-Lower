#pragma once

#include "bmi088_mm.h"

class IMU
{
public:
    IMU() = default;
    ~IMU() = default;

public:
    bmi08_sensor_data m_accelData;
    bmi08_sensor_data m_gyroData;
    bmi08_dev m_bmi08;
    int8_t m_rslt;
    float m_gyrox;
    float m_gyroy;
    float m_gyroz;
    float m_accelx;
    float m_accely;
    float m_accelz;

public:
    void imuInit();
    static void init_bmi08(struct bmi08_dev *bmi08dev);
    static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
    static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
    void imuLoop();

private:
};
