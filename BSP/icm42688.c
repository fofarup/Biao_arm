#include "icm42688.h"

#include <stddef.h>

I2C_HandleTypeDef *icm42688_i2c;
static uint8_t icm42688_ready;

static int16_t ICM42688_MakeInt16(uint8_t high, uint8_t low)
{
    uint16_t value;

    value = (uint16_t)(((uint16_t)high << 8U) | (uint16_t)low);
    return (int16_t)value;
}

HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value)
{
    if ((icm42688_i2c == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        icm42688_i2c,
        (uint16_t)ICM42688_I2C_ADDR_HAL,
        (uint16_t)reg,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1U,
        ICM42688_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value)
{
    if (icm42688_i2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(
        icm42688_i2c,
        (uint16_t)ICM42688_I2C_ADDR_HAL,
        (uint16_t)reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1U,
        ICM42688_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef ICM42688_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i;

    icm42688_ready = 0U;
    icm42688_i2c = hi2c;

    if (icm42688_i2c == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_IsDeviceReady(
            icm42688_i2c,
            (uint16_t)ICM42688_I2C_ADDR_HAL,
            3U,
            ICM42688_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Ensure the device is in bank 0 before issuing the soft reset. */
    if (ICM42688_WriteRegister(ICM42688_REG_BANK_SEL, 0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (ICM42688_WriteRegister(ICM42688_REG_DEVICE_CONFIG, 0x01U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* The reset bit is self-clearing; allow the sensor to restart. */
    HAL_Delay(2U);

    if (ICM42688_ReadRegister(ICM42688_REG_WHO_AM_I, &who_am_i) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (who_am_i != ICM42688_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    if (ICM42688_WriteRegister(
            ICM42688_REG_BANK_SEL,
            0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (ICM42688_WriteRegister(
            ICM42688_REG_ACCEL_CONFIG0,
            ICM42688_ACCEL_CONFIG0_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (ICM42688_WriteRegister(
            ICM42688_REG_GYRO_CONFIG0,
            ICM42688_GYRO_CONFIG0_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Enable accelerometer and gyroscope in low-noise mode. */
    if (ICM42688_WriteRegister(
            ICM42688_REG_PWR_MGMT0,
            ICM42688_PWR_MGMT0_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* The gyro requires a settling interval after being enabled. */
    HAL_Delay(50U);
    icm42688_ready = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadRawData(ICM42688_RawData_t *data)
{
    uint8_t buffer[14];

    if ((data == NULL) || (icm42688_i2c == NULL) || (icm42688_ready == 0U))
    {
        return HAL_ERROR;
    }

    /* Temperature (2 bytes) followed by accel and gyro (12 bytes). */
    if (HAL_I2C_Mem_Read(
            icm42688_i2c,
            (uint16_t)ICM42688_I2C_ADDR_HAL,
            (uint16_t)ICM42688_REG_TEMP_DATA1,
            I2C_MEMADD_SIZE_8BIT,
            buffer,
            (uint16_t)sizeof(buffer),
            ICM42688_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    data->temperature = ICM42688_MakeInt16(buffer[0], buffer[1]);
    data->accel_x = ICM42688_MakeInt16(buffer[2], buffer[3]);
    data->accel_y = ICM42688_MakeInt16(buffer[4], buffer[5]);
    data->accel_z = ICM42688_MakeInt16(buffer[6], buffer[7]);
    data->gyro_x = ICM42688_MakeInt16(buffer[8], buffer[9]);
    data->gyro_y = ICM42688_MakeInt16(buffer[10], buffer[11]);
    data->gyro_z = ICM42688_MakeInt16(buffer[12], buffer[13]);
    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadData(ICM42688_Data_t *data)
{
    ICM42688_RawData_t raw;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    if (ICM42688_ReadRawData(&raw) != HAL_OK)
    {
        return HAL_ERROR;
    }

    data->accel_x_g = (float)raw.accel_x * ICM42688_ACCEL_G_PER_LSB;
    data->accel_y_g = (float)raw.accel_y * ICM42688_ACCEL_G_PER_LSB;
    data->accel_z_g = (float)raw.accel_z * ICM42688_ACCEL_G_PER_LSB;
    data->gyro_x_dps = (float)raw.gyro_x * ICM42688_GYRO_DPS_PER_LSB;
    data->gyro_y_dps = (float)raw.gyro_y * ICM42688_GYRO_DPS_PER_LSB;
    data->gyro_z_dps = (float)raw.gyro_z * ICM42688_GYRO_DPS_PER_LSB;
    data->temperature_c = ((float)raw.temperature / 132.48f) + 25.0f;
    return HAL_OK;
}

uint8_t ICM42688_IsReady(void)
{
    return icm42688_ready;
}
