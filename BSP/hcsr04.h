/*
  程序说明: 前方和右侧超声波测距驱动，使用 TIM2 与 ECHO 双边沿中断。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#ifndef HCSR04_H
#define HCSR04_H
#include <stdint.h>
typedef enum { HCSR04_FRONT = 0U, HCSR04_RIGHT = 1U } HCSR04_Channel_t;
typedef enum
{
    HCSR04_VALID = 0U, HCSR04_NO_ECHO, HCSR04_STUCK_HIGH, HCSR04_NOT_READY
} HCSR04_Status_t;
typedef struct
{
    HCSR04_Status_t status;
    uint16_t distance_mm;
    uint32_t timestamp_ms;
    uint32_t sequence;
} HCSR04_Reading_t;
/*
  函数说明: 启动 TIM2 微秒计时并初始化测距状态；成功返回 1。
  调用示例: HCSR04_Init();
  参数意义: 无。
*/
uint8_t HCSR04_Init(void);
/*
  函数说明: 按公共发射间隔启动一次非阻塞测距；已占用或间隔不足返回 0。
  调用示例: HCSR04_Start(channel);
  参数意义: channel 为前方或右侧探头枚举。
*/
uint8_t HCSR04_Start(HCSR04_Channel_t channel);
/*
  函数说明: 取走一次已完成或超时的测距结果；结果尚未就绪返回 0。
  调用示例: HCSR04_Poll(reading);
  参数意义: reading 为测距结果、状态、采样时刻与序号。
*/
uint8_t HCSR04_Poll(HCSR04_Reading_t *reading);
/*
  函数说明: 处理 ECHO 双边沿中断，记录脉宽和完成时刻，不执行距离滤波。
  调用示例: HCSR04_OnEcho(pin);
  参数意义: pin 为触发 EXTI 的 GPIO 引脚位掩码。
*/
void HCSR04_OnEcho(uint16_t pin);
#endif
