#ifndef BSP_ICM42688_H
#define BSP_ICM42688_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ICM42688P is strapped for I2C by tying CS to VDDIO.
 * AD0 = 0 (GND) selects the 7-bit address 0x68.
 * HAL_I2C_* APIs expect the address shifted left by one bit.
 */
#define ICM42688_I2C_ADDR_7BIT       (0x68U)
#define ICM42688_I2C_ADDR_HAL        (ICM42688_I2C_ADDR_7BIT << 1U)
#define ICM42688_WHO_AM_I_VALUE      (0x47U)

/* Bank 0 register map used by this driver. */
#define ICM42688_REG_DEVICE_CONFIG   (0x11U)
#define ICM42688_REG_TEMP_DATA1      (0x1DU)
#define ICM42688_REG_ACCEL_DATA_X1   (0x1FU)
#define ICM42688_REG_PWR_MGMT0       (0x4EU)
#define ICM42688_REG_GYRO_CONFIG0    (0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0   (0x50U)
#define ICM42688_REG_INT_CONFIG1     (0x64U)
#define ICM42688_REG_INT_SOURCE0     (0x65U)
#define ICM42688_REG_WHO_AM_I        (0x75U)
#define ICM42688_REG_BANK_SEL        (0x76U)

/*
 * Reference configuration: +/-4 g, 100 Hz accelerometer and
 * +/-1000 dps, 100 Hz gyroscope.  The 100 Hz ODR matches the
 * 10 ms end-level task period and leaves margin for the 40 cm I2C cable.
 */
#define ICM42688_ACCEL_CONFIG0_VALUE \
    ((uint8_t)((2U << 5U) | 0x08U))
#define ICM42688_GYRO_CONFIG0_VALUE  \
    ((uint8_t)((1U << 5U) | 0x08U))
#define ICM42688_PWR_MGMT0_VALUE     (0x0FU)

#define ICM42688_I2C_TIMEOUT_MS      (100U)
#define ICM42688_ACCEL_G_PER_LSB     (4.0f / 32768.0f)
#define ICM42688_GYRO_DPS_PER_LSB    (1000.0f / 32768.0f)

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} ICM42688_RawData_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} ICM42688_Data_t;

HAL_StatusTypeDef ICM42688_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM42688_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef ICM42688_WriteRegister(uint8_t reg, uint8_t value);
HAL_StatusTypeDef ICM42688_ReadRawData(ICM42688_RawData_t *data);
HAL_StatusTypeDef ICM42688_ReadData(ICM42688_Data_t *data);
uint8_t ICM42688_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ICM42688_H */
