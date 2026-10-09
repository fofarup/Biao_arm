#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* 底盘主任务：周期执行控制器，并发送四电机 F6 速度命令。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_control.h"

void ChassisTask_Run(void *argument);

/* 便于 Keil Watch 观察底盘当前控制状态。 */
extern volatile ChassisControlState_t chassis_debug_state;

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
