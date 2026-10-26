/*
  程序说明: 每 50 ms 推进探头允许间隔内的测距，并提供结果快照。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef ULTRASONIC_TASK_H
#define ULTRASONIC_TASK_H
#include "hcsr04.h"
#define ULTRASONIC_TASK_PERIOD_MS (50U)
/*
  函数说明: 初始化测距驱动，由 main_task 在业务启动阶段调用。
  调用示例: UltrasonicTask_Init();
  参数意义: 无。
*/
uint8_t UltrasonicTask_Init(void);
/*
  函数说明: 每 50 ms 推进非阻塞测距，按探头允许间隔更新数据，不执行避障决策。
  调用示例: 由 CubeMX 创建的 StartUltrasonicTask 调用 UltrasonicTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void UltrasonicTask_Run(void *argument);
/*
  函数说明: 查询超声波驱动是否已完成初始化。
  调用示例: UltrasonicTask_Ready();
  参数意义: 无。
*/
uint8_t UltrasonicTask_Ready(void);
/*
  函数说明: 在临界区复制指定探头最近的测距状态、时间戳和序号。
  调用示例: UltrasonicTask_GetReading(HCSR04_FRONT, &reading);
  参数意义: channel 为前方或右侧探头枚举。
            reading 为测距结果、状态、采样时刻与序号。
*/
void UltrasonicTask_GetReading(HCSR04_Channel_t channel, HCSR04_Reading_t *reading);
/*
  函数说明: 切换前方探头优先模式，供 Route 行走时缩短前方检测间隔。
  调用示例: UltrasonicTask_SetFrontPriority(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
void UltrasonicTask_SetFrontPriority(uint8_t enabled);
#endif
