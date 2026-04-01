#include "crt_imu.h"
#include "common.h"
#include "para_imu.h"
#include <cstdio>
#include <cmath>


void IMU::imuInit()
{
    init_bmi08(&m_bmi08);
}

void IMU::init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel & gyro) */
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
    {
        printf("BMI08 initialization success!\n");
        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);
    }
    else
    {
        printf("BMI08 initialization failure!\n");
        return;
    }

    if (rslt == BMI08_OK)
    {
        /* Set accel power mode */
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;

        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(bmi08dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi088_mma_set_meas_conf(bmi08dev);
            bmi08_check_rslt("bmi088_mma_set_meas_conf", rslt);

            bmi08dev->gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
            bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
            bmi08dev->gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
            bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

            rslt = bmi08g_set_power_mode(bmi08dev);

            if (rslt == BMI08_OK)
            {
                rslt = bmi08g_set_meas_conf(bmi08dev);
                bmi08_check_rslt("bmi08g_set_meas_conf", rslt);
            }
        }
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
    m_rslt = bmi088_mma_get_data(&m_accelData, &m_bmi08);
    m_accelx = lsb_to_mps2(m_accelData.x, (float)6, 16);
    m_accely = lsb_to_mps2(m_accelData.y, (float)6, 16);
    m_accelz = lsb_to_mps2(m_accelData.z, (float)6, 16);
    printf("accel x=%.3f y=%.3f z=%.3f m/s2\r\n", m_accelx, m_accely, m_accelz);
}