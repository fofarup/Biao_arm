#ifndef CLAW_H
#define CLAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * 控制单个 270° 总线舵机夹爪，角度单位为度，运动时间单位为毫秒。
 * 目前只接 UART4 TX：HAL_OK 仅表示指令已从串口发出，不表示舵机到位。
 * 本函数发送后立即返回；连续动作由调用任务自行等待运动时间。
 * 请只在任务中调用，不要由多个任务同时使用 UART4。
 */
HAL_StatusTypeDef Claw_SetAngle(float angle_deg, uint16_t move_time_ms);

#ifdef __cplusplus
}
#endif

#endif /* CLAW_H */
