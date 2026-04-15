#include "crt_imu.h"
#include "common.h"
#include "para_imu.h"
#include <cstdio>
#include <cmath>

void IMU::imuInit()
{
    bmi08_interface_init(&m_bmi08, BMI08_I2C_INTF);
    init_bmi08(&m_bmi08);
}

void IMU::init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    int8_t gyro_rslt = bmi08g_init(bmi08dev);
    printf("gyro_init=%d chip_g=0x%x\r\n", gyro_rslt, bmi08dev->gyro_chip_id);

    if (gyro_rslt != BMI08_OK)
    {
        printf("Gyro initialization failure!\r\n");
        return;
    }
    printf("Gyro initialization success!\r\n");

    rslt = BMI08_OK;

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        printf("set_power_mode...\r\n");
        rslt = bmi08g_set_power_mode(bmi08dev);
        bmi08_check_rslt("bmi08g_set_power_mode", rslt);
        printf("set_power_mode done rslt=%d\r\n", rslt);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.odr   = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
        bmi08dev->gyro_cfg.bw    = BMI08_GYRO_BW_32_ODR_100_HZ;
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        printf("set_meas_conf...\r\n");
        rslt = bmi08g_set_meas_conf(bmi08dev);
        bmi08_check_rslt("bmi08g_set_meas_conf", rslt);
        printf("set_meas_conf done rslt=%d\r\n", rslt);
    }
}

float IMU::lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

float IMU::lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

void IMU::imuLoop()
{
    m_rslt = bmi08g_get_data(&m_gyroData, &m_bmi08);
    if (m_rslt != BMI08_OK)
    {
        printf("gyro read fail rslt=%d\r\n", m_rslt);
        return;
    }
    m_gyrox = lsb_to_dps(m_gyroData.x, 1000.0f, 16);
    m_gyroy = lsb_to_dps(m_gyroData.y, 1000.0f, 16);
    m_gyroz = lsb_to_dps(m_gyroData.z, 1000.0f, 16);
    printf("gyro x=%.2f y=%.2f z=%.2f dps raw=%d,%d,%d\r\n",
           m_gyrox, m_gyroy, m_gyroz,
           m_gyroData.x, m_gyroData.y, m_gyroData.z);
}
