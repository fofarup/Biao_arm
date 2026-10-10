#ifndef __ZDT_CAN_H
#define __ZDT_CAN_H

/*
 * ZDT CAN 公共收发接口。
 * 地址 1~4 等帧进入机械臂队列，地址 5~8 的扩展帧被丢弃；
 * 底盘只使用 OPS9/HW101 外部闭环，不抢机械臂的位置反馈。
 */

#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>

typedef struct
{
    uint32_t id;           /* 标准 ID 或扩展 ID。 */
    uint32_t ide;          /* CAN_ID_STD 或 CAN_ID_EXT。 */
    uint32_t rtr;          /* 数据帧/远程帧标志。 */
    uint32_t timestamp_ms; /* CAN 中断取帧时间，ms。 */
    uint8_t  dlc;          /* 有效数据长度。 */
    uint8_t  data[8];      /* CAN 数据区。 */
} ZDT_CanRxFrame_t;

HAL_StatusTypeDef ZDT_CAN_Init(void);

HAL_StatusTypeDef can_SendCmd(const uint8_t *cmd, uint8_t len);

osStatus_t ZDT_CAN_GetRx(ZDT_CanRxFrame_t *frame,
                        uint32_t timeout_ticks);

/* 丢弃当前队列中尚未处理的旧帧，返回丢弃帧数。 */
uint32_t ZDT_CAN_FlushRx(void);

extern volatile uint32_t zdt_can_tx_count;
extern volatile uint32_t zdt_can_tx_error_count;
extern volatile uint32_t zdt_can_rx_count;
extern volatile uint32_t zdt_can_rx_drop_count;
extern volatile uint32_t zdt_can_error_count;
extern volatile ZDT_CanRxFrame_t zdt_can_last_rx;

#endif
