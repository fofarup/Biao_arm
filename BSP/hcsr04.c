/*
  程序说明: 前方和右侧超声波测距驱动，使用 TIM2 与 ECHO 双边沿中断。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "hcsr04.h"
#include "main.h"
#include "tim.h"
#include <stddef.h>

#define HCSR04_QUIET_US (60000U)
#define HCSR04_ECHO_TIMEOUT_MS (27U)
static volatile uint8_t active_channel = 0xFFU;
static volatile uint8_t pulse_state;
static volatile uint32_t rise_us;
static volatile uint32_t pulse_us;
static volatile uint32_t complete_ms;
static uint32_t trigger_ms;
static uint32_t last_trigger_us;
static uint8_t initialized;
static uint8_t stuck_high;


/*
  函数说明: 启动 TIM2 微秒计时并初始化测距状态；成功返回 1。
  调用示例: HCSR04_Init();
  参数意义: 无。
*/
uint8_t HCSR04_Init(void)
{
    if (initialized != 0U) return 1U;
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) return 0U;
    last_trigger_us = TIM2->CNT - HCSR04_QUIET_US;
    initialized = 1U;
    return 1U;
}

/*
  函数说明: 按公共发射间隔启动一次非阻塞测距；已占用或间隔不足返回 0。
  调用示例: HCSR04_Start(channel);
  参数意义: channel 为前方或右侧探头枚举。
*/
uint8_t HCSR04_Start(HCSR04_Channel_t channel)
{
    uint16_t echo_pin;
    uint16_t trig_pin;
    uint32_t trigger_us;
    if (!initialized || (uint32_t)channel > (uint32_t)HCSR04_RIGHT || active_channel != 0xFFU ||
        (uint32_t)(TIM2->CNT - last_trigger_us) < HCSR04_QUIET_US) return 0U;
    echo_pin = channel == HCSR04_FRONT ? HC_FRONT_ECHO_Pin : HC_RIGHT_ECHO_Pin;
    trig_pin = channel == HCSR04_FRONT ? HC_FRONT_TRIG_Pin : HC_RIGHT_TRIG_Pin;
    trigger_ms = HAL_GetTick();
    pulse_us = 0U;
    stuck_high = HAL_GPIO_ReadPin(GPIOF, echo_pin) == GPIO_PIN_SET;
    pulse_state = stuck_high ? 3U : 1U;
    complete_ms = trigger_ms;
    active_channel = (uint8_t)channel;
    if (!stuck_high)
    {
        HAL_GPIO_WritePin(GPIOE, trig_pin, GPIO_PIN_SET);
        trigger_us = TIM2->CNT;
        /* 仅触发脉冲占用约 12 us，回波等待完全由中断和 Poll 推进。 */
        while ((uint32_t)(TIM2->CNT - trigger_us) < 12U) { }
        HAL_GPIO_WritePin(GPIOE, trig_pin, GPIO_PIN_RESET);
    }
    last_trigger_us = TIM2->CNT;
    return 1U;
}

/*
  函数说明: 取走一次已完成或超时的测距结果；结果尚未就绪返回 0。
  调用示例: HCSR04_Poll(reading);
  参数意义: reading 为测距结果、状态、采样时刻与序号。
*/
uint8_t HCSR04_Poll(HCSR04_Reading_t *reading)
{
    if (reading == NULL || active_channel == 0xFFU) return 0U;
    if (pulse_state != 3U && (uint32_t)(HAL_GetTick() - trigger_ms) < HCSR04_ECHO_TIMEOUT_MS)
        return 0U;
    active_channel = 0xFFU;
    __DMB();
    reading->distance_mm = 0U;
    reading->sequence = 0U;
    reading->timestamp_ms = pulse_state == 3U ? complete_ms : trigger_ms + HCSR04_ECHO_TIMEOUT_MS;
    reading->status = stuck_high ? HCSR04_STUCK_HIGH : HCSR04_NO_ECHO;
    if (!stuck_high && pulse_state == 3U && pulse_us > 0U && pulse_us < 25000U)
    {
        reading->status = HCSR04_VALID;
        reading->distance_mm = (uint16_t)((pulse_us * 343U + 1000U) / 2000U);
    }
    pulse_state = 0U;
    return 1U;
}

/*
  函数说明: 处理 ECHO 双边沿中断，记录脉宽和完成时刻，不执行距离滤波。
  调用示例: HCSR04_OnEcho(pin);
  参数意义: pin 为触发 EXTI 的 GPIO 引脚位掩码。
*/
void HCSR04_OnEcho(uint16_t pin)
{
    uint8_t channel;
    GPIO_PinState level;
    if (pin == HC_FRONT_ECHO_Pin) channel = HCSR04_FRONT;
    else if (pin == HC_RIGHT_ECHO_Pin) channel = HCSR04_RIGHT;
    else return;
    if (channel != active_channel ||
        (uint32_t)(HAL_GetTick() - trigger_ms) > HCSR04_ECHO_TIMEOUT_MS) return;
    level = HAL_GPIO_ReadPin(GPIOF, pin);
    if (level == GPIO_PIN_SET && pulse_state == 1U)
    {
        rise_us = TIM2->CNT;
        pulse_state = 2U;
    }
    else if (level == GPIO_PIN_RESET && pulse_state == 2U)
    {
        pulse_us = (uint32_t)(TIM2->CNT - rise_us);
        complete_ms = HAL_GetTick();
        __DMB();
        pulse_state = 3U;
    }
}
