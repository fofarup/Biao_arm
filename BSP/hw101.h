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

/* 启动 USART2 单字节接收中断 */
HAL_StatusTypeDef HW101_StartReceive(void);

/* 读取最近一次有效数据 */
uint8_t HW101_GetData(HW101_Data_t *data);

/*
 * 这两个函数由工程中的统一 HAL UART 回调调用。
 * 不要在 hw101.c 中再次定义 HAL_UART_RxCpltCallback。
 */
void HW101_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HW101_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* HW101_H */

