#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arm_motion.h"

/*
 * 系统级流程任务入口。
 * 本任务负责机械臂一次性初始化和高层动作流程；传感器采样与底盘实时控制仍由各自任务完成。
 */
void MainTask_Run(void *argument);

/* 底盘动作前按需检查 OPS9/HW101；机械臂动作不依赖这两路传感器。 */
uint8_t MainTask_ChassisSensorsReady(void);

/* 末端 XYZ 动作，时间单位为秒；详细错误可在 Watch 查看 main_task_arm_status。 */
void ArmTask_MoveToXYZOrFault(float x, float y, float z, float time_s);

/* Watch 可直接查看：1 表示机械臂初始化完成；状态码见 arm_motion.h。 */
extern volatile ArmMotionStatus main_task_arm_status;
/* 到位超时后的停稳/位置恢复结果；OK 表示下一条 XYZ 可正常下发。 */
extern volatile ArmMotionStatus main_task_arm_recovery_status;
extern volatile uint8_t main_task_arm_ready;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_TASK_H */
