/*
  程序说明: CAN 初始化、完整命令互斥发送、接收队列及错误统计。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

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

/*
  函数说明: 配置 CAN 过滤器、发送互斥锁与接收队列，并启动 CAN 通知。
  调用示例: ZDT_CAN_Init();
  参数意义: 无。
*/
HAL_StatusTypeDef ZDT_CAN_Init(void);

/*
  函数说明: 用互斥锁保护一整条驱动器命令，避免不同任务的 CAN 分帧交错。
  调用示例: can_SendCmd(cmd, len);
  参数意义: cmd 为驱动器协议字节数组。
            len 为输入字节数。
*/
HAL_StatusTypeDef can_SendCmd(const uint8_t *cmd, uint8_t len);

/*
  函数说明: 从 CAN 接收队列取一帧；按指定 RTOS tick 时限等待。
  调用示例: ZDT_CAN_GetRx(frame, timeout_ticks);
  参数意义: frame 为CAN 接收帧结构。
            timeout_ticks 为队列等待时间，RTOS tick。
*/
osStatus_t ZDT_CAN_GetRx(ZDT_CanRxFrame_t *frame,
                        uint32_t timeout_ticks);

/*
  函数说明: 丢弃接收队列中的旧帧并返回丢弃数量。
  调用示例: ZDT_CAN_FlushRx();
  参数意义: 无。
*/
uint32_t ZDT_CAN_FlushRx(void);

extern volatile uint32_t zdt_can_tx_count;
extern volatile uint32_t zdt_can_tx_error_count;
extern volatile uint32_t zdt_can_rx_count;
extern volatile uint32_t zdt_can_rx_drop_count;
extern volatile uint32_t zdt_can_error_count;
extern volatile ZDT_CanRxFrame_t zdt_can_last_rx;

#endif
