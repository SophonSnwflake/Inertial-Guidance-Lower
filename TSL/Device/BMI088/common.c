/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 * Adapted for STM32F411 HAL by Wang.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "main.h"
#include "spi.h"
#include "i2c.h"

/*
 * CS pin assignment — must match CubeMX GPIO config:
 *   PA4  →  BMI088_ACCEL_CS  (accelerometer)
 *   PB0  →  BMI088_GYRO_CS   (gyroscope)
 */
#define BMI088_ACCEL_CS_PORT  GPIOA
#define BMI088_ACCEL_CS_PIN   GPIO_PIN_4
#define BMI088_GYRO_CS_PORT   GPIOB
#define BMI088_GYRO_CS_PIN    GPIO_PIN_0

/* Device index stored in intf_ptr: 0 = accel, 1 = gyro */
static uint8_t acc_dev_idx  = 0;
static uint8_t gyro_dev_idx = 1;

/******************************************************************************/
/*!                Static helpers                                              */

static inline void cs_low(uint8_t dev_idx)
{
    if (dev_idx == 0)
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_high(uint8_t dev_idx)
{
    if (dev_idx == 0)
        HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
}

/******************************************************************************/
/*!                I2C read / write                                            */

BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1,
                                                 (uint16_t)(device_addr << 1),
                                                 reg_addr,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 reg_data,
                                                 (uint16_t)len,
                                                 HAL_MAX_DELAY);
    return (status == HAL_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}

BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1,
                                                  (uint16_t)(device_addr << 1),
                                                  reg_addr,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  (uint8_t *)reg_data,
                                                  (uint16_t)len,
                                                  HAL_MAX_DELAY);
    return (status == HAL_OK) ? BMI08_OK : BMI08_E_COM_FAIL;
}

/******************************************************************************/
/*!                SPI read / write                                            */

BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_idx = *(uint8_t *)intf_ptr;
    uint8_t tx_addr = reg_addr | 0x80;

    cs_low(dev_idx);
    HAL_SPI_Transmit(&hspi1, &tx_addr, 1, HAL_MAX_DELAY);

    /* Dummy clocks during data phase — HAL needs a TX buffer of equal length */
    uint8_t dummy[64];
    uint32_t remaining = len;
    while (remaining > 0)
    {
        uint16_t chunk = (remaining > sizeof(dummy)) ? (uint16_t)sizeof(dummy) : (uint16_t)remaining;
        memset(dummy, 0x00, chunk);
        HAL_SPI_TransmitReceive(&hspi1, dummy, reg_data, chunk, HAL_MAX_DELAY);
        reg_data  += chunk;
        remaining -= chunk;
    }
    cs_high(dev_idx);

    return BMI08_OK;
}

BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_idx = *(uint8_t *)intf_ptr;

    cs_low(dev_idx);
    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)reg_data, (uint16_t)len, HAL_MAX_DELAY);
    cs_high(dev_idx);

    return BMI08_OK;
}

/******************************************************************************/
/*!                Microsecond delay (DWT cycle counter)                      */

void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    /* Enable DWT cycle counter on first call */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks)
        ;
}

/******************************************************************************/
/*!                Interface init                                              */

int8_t bmi08_interface_init(struct bmi08_dev *bmi08dev, uint8_t intf)
{
    if (bmi08dev == NULL)
        return BMI08_E_NULL_PTR;

    if (intf == BMI08_I2C_INTF)
    {
        /* hi2c1 is already initialised by MX_I2C1_Init() in main.c */
        bmi08dev->read  = bmi08_i2c_read;
        bmi08dev->write = bmi08_i2c_write;
        bmi08dev->intf  = BMI08_I2C_INTF;

        acc_dev_idx  = (uint8_t)BMI08_ACCEL_I2C_ADDR_SECONDARY;
        gyro_dev_idx = (uint8_t)BMI08_GYRO_I2C_ADDR_SECONDARY;
    }
    else if (intf == BMI08_SPI_INTF)
    {
        /*
         * hspi1 is already initialised by MX_SPI1_Init() in main.c.
         * CS GPIO pins (PA4 / PB0) must be configured in CubeMX as
         * GPIO Output Push-Pull, default HIGH.
         */
        bmi08dev->read  = bmi08_spi_read;
        bmi08dev->write = bmi08_spi_write;
        bmi08dev->intf  = BMI08_SPI_INTF;

        /* Pull both CS lines high before starting */
        cs_high(0);
        cs_high(1);

        acc_dev_idx  = 0;
        gyro_dev_idx = 1;
    }

    bmi08dev->intf_ptr_accel  = &acc_dev_idx;
    bmi08dev->intf_ptr_gyro   = &gyro_dev_idx;
    bmi08dev->delay_us        = bmi08_delay_us;
    bmi08dev->read_write_len  = 32;

    /* Allow sensors to power up */
    HAL_Delay(200);

    return BMI08_OK;
}

/******************************************************************************/
/*!                Error print                                                 */

void bmi08_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI08_OK:
            break;
        case BMI08_E_NULL_PTR:
            printf("[%s] E: Null pointer\r\n", api_name);
            break;
        case BMI08_E_COM_FAIL:
            printf("[%s] E: Communication failure\r\n", api_name);
            break;
        case BMI08_E_DEV_NOT_FOUND:
            printf("[%s] E: Device not found\r\n", api_name);
            break;
        case BMI08_E_OUT_OF_RANGE:
            printf("[%s] E: Out of range\r\n", api_name);
            break;
        case BMI08_E_INVALID_INPUT:
            printf("[%s] E: Invalid input\r\n", api_name);
            break;
        case BMI08_E_CONFIG_STREAM_ERROR:
            printf("[%s] E: Config stream error\r\n", api_name);
            break;
        case BMI08_E_RD_WR_LENGTH_INVALID:
            printf("[%s] E: Invalid read/write length\r\n", api_name);
            break;
        case BMI08_E_INVALID_CONFIG:
            printf("[%s] E: Invalid config\r\n", api_name);
            break;
        case BMI08_E_FEATURE_NOT_SUPPORTED:
            printf("[%s] E: Feature not supported\r\n", api_name);
            break;
        case BMI08_W_FIFO_EMPTY:
            printf("[%s] W: FIFO empty\r\n", api_name);
            break;
        default:
            printf("[%s] E: Unknown error %d\r\n", api_name, rslt);
            break;
    }
}

void bmi08_coines_deinit(void)
{
    /* Nothing to do on STM32 */
}
