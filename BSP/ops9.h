/*
  程序说明: OPS9 定位模块的字节流解析、异常统计和快照读取。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef OPS9_H
#define OPS9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* OPS9 固定数据帧：2 字节帧头 + 6 个 float + 2 字节帧尾。 */
#define OPS9_DATA_LENGTH    (24U)
#define OPS9_FRAME_LENGTH   (28U)

/* 一帧已经解析完成的 OPS9 测量结果。 */
typedef struct
{
    float heading_deg;       /* 航向角，单位：度 */
    float pitch_deg;         /* 俯仰角，单位：度 */
    float roll_deg;          /* 横滚角，单位：度 */
    float x_mm;              /* X 坐标，单位：毫米 */
    float y_mm;              /* Y 坐标，单位：毫米 */
    float heading_rate_dps;  /* 航向角速度，单位：度/秒 */

    uint32_t last_update_ms; /* 最近一次合法帧的系统时间 */
    uint32_t frame_count;    /* 合法帧计数 */
    uint32_t bad_frame_count;/* 帧头、帧尾或数据异常计数 */
    uint8_t valid;           /* 1：已经收到过合法帧；0：尚未收到 */
} OPS9_Data_t;

/*
  函数说明: 启动单字节 UART 接收。任务启动后调用一次即可。
  调用示例: OPS9_StartReceive();
  参数意义: 无。
*/
HAL_StatusTypeDef OPS9_StartReceive(void);

/*
  函数说明: 在关中断保护下复制共享数据，避免任务读到更新一半的结构体。
  调用示例: OPS9_GetLatest(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
uint8_t OPS9_GetLatest(OPS9_Data_t *data);

/*
  函数说明: 根据最近合法帧时间判断 OPS9 链路是否仍然在线。
  调用示例: OPS9_IsFresh(timeout_ms);
  参数意义: timeout_ms 为等待或心跳超时，毫秒。
*/
uint8_t OPS9_IsFresh(uint32_t timeout_ms);

/*
  函数说明: UART 出错后停止当前接收，清空解析状态，再重新接收。
  调用示例: OPS9_ResetReceive();
  参数意义: 无。
*/
void OPS9_ResetReceive(void);
/*
  函数说明: 处理 USART3 接收完成事件，将字节交给 OPS9 帧解析器并续接收。
  调用示例: OPS9_UART_RxCpltCallback(huart);
  参数意义: huart 为HAL UART 句柄。
*/
void OPS9_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* OPS9_H */
