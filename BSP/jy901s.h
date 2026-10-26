/*
  程序说明: JY901S 姿态、角速度和加速度解析，供末端调平使用。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef BSP_JY901S_H
#define BSP_JY901S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint32_t accel_tick_ms;
    uint32_t gyro_tick_ms;
    uint32_t angle_tick_ms;
    uint32_t angle_count;
    uint32_t checksum_error_count;
} JY901S_Data_t;

/*
  函数说明: 启动 USART1 单字节接收中断；已启动时直接返回当前状态。
  调用示例: JY901S_StartReceive();
  参数意义: 无。
*/
HAL_StatusTypeDef JY901S_StartReceive(void);
/*
  函数说明: 读取完整且新鲜的 IMU 数据；缺帧、过期或参数错误返回 HAL_ERROR。
  调用示例: JY901S_ReadData(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
HAL_StatusTypeDef JY901S_ReadData(JY901S_Data_t *data);
/*
  函数说明: 处理 USART1 接收完成事件，解析一个字节并续接下一字节。
  调用示例: JY901S_UART_RxCpltCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void JY901S_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/*
  函数说明: 处理 USART1 接收错误，清理状态并尝试恢复接收。
  调用示例: JY901S_UART_ErrorCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void JY901S_UART_ErrorCallback(UART_HandleTypeDef *huart);
/*
  函数说明: 复制最近一次 IMU 缓存，供调试查看；该接口不保证数据新鲜。
  调用示例: JY901S_GetLatest(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
void JY901S_GetLatest(JY901S_Data_t *data);

#endif
