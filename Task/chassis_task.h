/*
  程序说明: 每 20 ms 更新底盘闭环和四轮输出；初始化由主任务统一调用。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* 底盘主任务：周期执行控制器，并发送四电机 F6 速度命令。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"

/*
  函数说明: 初始化底盘控制器、OPS9/HW101 接收及四个电机；由 main_task 调用。
  调用示例: ChassisTask_Init();
  参数意义: 无。
*/
uint8_t ChassisTask_Init(void);

/*
  函数说明: 等待主任务初始化完成后，每 20 ms 执行底盘闭环和四轮发令。
  调用示例: 由 CubeMX 创建的 StartChassisTask 调用 ChassisTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void ChassisTask_Run(void *argument);

/* 底盘驱动器配置完成后才为 READY；失败时保持 FAULT。 */
typedef enum
{
    CHASSIS_TASK_STARTING = 0U,
    CHASSIS_TASK_READY,
    CHASSIS_TASK_FAULT
} ChassisTaskStartupState_t;

/*
  函数说明: 读取底盘驱动器初始化状态，供主任务决定是否接受动作。
  调用示例: ChassisTask_GetStartupState();
  参数意义: 无。
*/
ChassisTaskStartupState_t ChassisTask_GetStartupState(void);

/* 便于 Keil Watch 观察底盘当前控制状态。 */
extern volatile ChassisControlState_t chassis_debug_state;

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
