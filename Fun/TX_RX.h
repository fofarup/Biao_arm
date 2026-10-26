/*
  程序说明: USART6 上位机 12 字节协议、CRC16/MODBUS、命令队列与完成/错误应答。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#ifndef TX_RX_H
#define TX_RX_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#define TX_RX_FRAME_SIZE (12U)
#define TX_RX_MODE_IDLE (0x0000U)
#define TX_RX_MODE_CHASSIS (0x0001U)
#define TX_RX_MODE_ARM (0x0002U)
#define TX_RX_MODE_CLAW (0x0003U)
#define TX_RX_MODE_CHASSIS_DIAGONAL (0x0004U)
#define TX_RX_ACK_CHASSIS (0x0011U)
#define TX_RX_ACK_ARM (0x0012U)
#define TX_RX_ACK_CLAW TX_RX_ACK_ARM
#define TX_RX_ERROR_ARM (0x10FFU)
#define TX_RX_EVENT_START (0x0020U)
#define TX_RX_START_AREA_1 (1U)
#define TX_RX_START_AREA_2 (2U)
/* FF MODE_H MODE_L X_H X_L Y_H Y_L Z_H Z_L CRC_H CRC_L FE。
 * CRC16/MODBUS 覆盖下标 0～8，包含帧头；所有 16 位字段高字节在前。
 * XYZ 为有符号整数：机械臂坐标单位 mm，夹爪 X 为 0～180 度、YZ 为 0。
 * 0001 的 X/Y 沿发令时车右/车前；0004 的 X/Y 轴相对发令时车身向右偏 45 度。
 * 0004 的 +X 向右后、+Y 向右前；位移单位 mm，从发令时当前位置累计。
 * 两种底盘模式的 Z 都是逆时针相对转角，单位度，Z=0 保持当前航向；到位后回 0011。
 * 0012 为机械臂和夹爪共用完成应答。
 * 10FF 为机械臂错误：X=动作状态，Y=恢复状态，Z=是否可接收机械臂动作。
 * 0020 为启动事件：X=启停区 1 或 2，Y/Z=0；仅由下位机上报。 */
typedef struct
{
    uint16_t mode;
    int16_t x;
    int16_t y;
    int16_t z;
} TX_RX_Command_t;
/* Keil Watch 直接展开 tx_rx_diagnostics；计数是累计值，不是队列深度。 */
typedef struct
{
    uint32_t rx_events;
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t queued_frames;
    uint32_t popped_frames;
    uint32_t last_rx_ms;
    uint32_t last_valid_ms;
    uint32_t last_pop_ms;
    uint32_t rx_start_count;
    uint32_t rx_restart_errors;
    uint32_t rx_health_faults;
    uint32_t last_uart_error;
    uint32_t last_dma_error;
    uint32_t tx_attempts;
    uint32_t tx_frames;
    uint32_t tx_failures;
    uint32_t tx_retries;
    uint32_t tx_uncertain_frames; /* 整帧已写入但未确认发完，避免重复应答而不重发。 */
    uint32_t last_tx_ms;
    HAL_StatusTypeDef last_rx_status;
    HAL_StatusTypeDef last_tx_status;
    uint16_t last_tx_mode;
    uint8_t rx_active;
} TX_RX_Diagnostics_t;
extern volatile TX_RX_Diagnostics_t tx_rx_diagnostics;

/*
  函数说明: 创建发送互斥锁、接收维护定时器并启动 USART6 DMA；初始化时不接受业务动作。
  调用示例: TX_RX_Init();
  参数意义: 无。
*/
uint8_t TX_RX_Init(void);
/*
  函数说明: 切换业务命令接收开关并清空旧队列，防止初始化期间积压动作。
  调用示例: TX_RX_SetAccepting(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
void TX_RX_SetAccepting(uint8_t enabled);
/*
  函数说明: 在 main_task 中调用；每次最多从队列取一条完整帧。
  调用示例: if (TX_RX_PopCommand(&command)) { 执行业务函数(); }
  参数意义: command 为单条命令结构。
*/
uint8_t TX_RX_PopCommand(TX_RX_Command_t *command);
/*
  函数说明: 复制最近一次合法接收命令，供调试观察。
  调用示例: TX_RX_GetLastCommand(command);
  参数意义: command 为单条命令结构。
*/
uint8_t TX_RX_GetLastCommand(TX_RX_Command_t *command);
/*
  函数说明: 发送底盘或机械臂/夹爪完成帧；成功返回 1，最多尝试 3 次。
  已写出完整帧但结果不确定时不重发；失败保存在 tx_rx_diagnostics。
  调用示例: TX_RX_SendCompletion(TX_RX_ACK_ARM); 由主任务在机械臂到位或夹爪发送成功后等待 300 ms 调用。
  参数意义: ack_mode 为应答功能码，仅允许 0011、0012；XYZ 全填 0。
*/
uint8_t TX_RX_SendCompletion(uint16_t ack_mode);
/*
  函数说明: 发送桌面版 10FF 机械臂错误帧，成功返回 1；采用与完成帧相同的有限重试。
  调用示例: TX_RX_SendArmError(status, recovery_status, ready);
  参数意义: status 为动作错误码，与 ArmControlStatus 的数值一致。
            recovery_status 为最近一次动作的位置恢复状态，未尝试恢复时为 0。
            ready 为非零表示机械臂仍可接受动作，零表示未就绪。
*/
uint8_t TX_RX_SendArmError(uint16_t status, uint16_t recovery_status, uint8_t ready);
/*
  函数说明: 上报 0020 启动事件；非阻塞获取串口锁，避免与动作应答交叉发送。
  调用示例: status = TX_RX_SendStart(TX_RX_START_AREA_1);
  参数意义: area 为启停区 1 或 2，写入 X，Y/Z 为 0。
            返回 HAL_OK 表示已发送；HAL_BUSY 表示尚未开始，可稍后重试；其他值表示发送失败。
            仅在任务环境调用，不能在中断中调用。
*/
HAL_StatusTypeDef TX_RX_SendStart(uint8_t area);
/*
  函数说明: 计算 CRC16/MODBUS：初值 FFFF，反射多项式 A001，最终异或 0000。
  调用示例: crc = TX_RX_Crc16(frame, 9U); 自定义协议发送时 CRC 高字节在前。
  参数意义: data 为数据缓冲区或传感器快照。
            length 为参与 CRC 计算的字节数。
*/
uint16_t TX_RX_Crc16(const uint8_t *data, uint16_t length);
/*
  函数说明: HAL 在 USART6 空闲、DMA 半满或全满时回调。
  循环 DMA 模式下 size 表示 DMA 当前写入位置，不是本次新字节数；
  因此只能处理 upper_dma_position 到 size 之间尚未读过的字节。
  size 变小时说明缓冲区回卷，先处理末尾，再从下标 0 处理到 size。
  调用示例: TX_RX_OnRxEvent(huart, size);
  参数意义: huart 为HAL UART 句柄。
            size 为DMA 当前写入位置，不是本次新增字节数。
*/
void TX_RX_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/*
  函数说明: 标记 USART6 接收异常并清理半帧状态，等待维护定时器恢复。
  调用示例: TX_RX_OnError();
  参数意义: 无。
*/
void TX_RX_OnError(void);
/*
  函数说明: 记录异步接收终止完成，使维护定时器能够重新启动 DMA。
  调用示例: TX_RX_OnAbortReceiveComplete();
  参数意义: 无。
*/
void TX_RX_OnAbortReceiveComplete(void);
extern volatile uint32_t tx_rx_bad_frames;
extern volatile uint32_t tx_rx_dropped_frames;
extern volatile uint32_t tx_rx_uart_errors;
#endif
