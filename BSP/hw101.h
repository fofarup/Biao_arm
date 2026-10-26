/*
  程序说明: HW101 航向传感器串口接收、校验和缓存读取。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef HW101_H
#define HW101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* HW101 角度帧长度 */
#define HW101_FRAME_LENGTH    (11U)

/* HW101 最近一次角度数据 */
typedef struct
{
    float yaw_deg;              /* 航向角，单位：度 */
    float temperature_c;        /* 温度，单位：摄氏度 */

    uint32_t last_update_ms;    /* 最近一次有效数据时间 */
    uint32_t frame_count;       /* 有效角度帧数量 */
    uint32_t bad_frame_count;   /* 校验错误帧数量 */

    uint8_t valid;              /* 1：已经收到有效角度帧 */
} HW101_Data_t;

/*
  函数说明: 启动 USART2 的单字节接收中断
  调用示例: HW101_StartReceive();
  参数意义: 无。
*/
HAL_StatusTypeDef HW101_StartReceive(void);

/*
  函数说明: 获取最近一次有效数据
  调用示例: HW101_GetData(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
uint8_t HW101_GetData(HW101_Data_t *data);

/*
  函数说明: USART2 接收完成后的处理
  调用示例: HW101_UART_RxCpltCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void HW101_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/*
  函数说明: USART2 出错后的处理
  调用示例: HW101_UART_ErrorCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void HW101_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* HW101_H */

