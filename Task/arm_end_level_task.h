/*
  程序说明: 每 20 ms 读取姿态，控制末端到达并持续保持指定俯仰角。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#ifndef ARM_END_LEVEL_TASK_H
#define ARM_END_LEVEL_TASK_H
#include <stdint.h>
#define ARM_END_LEVEL_TASK_PERIOD_MS (20U)
#define ARM_LEVEL_READY_TIMEOUT_MS (3000U)
/* ID4 pitch holding controller: output is motor speed in RPM. */
#define ARM_PID_MOTOR_KP_RPM_PER_DEG        (3.0f)
#define ARM_PID_MOTOR_KI_RPM_PER_DEG_S      (0.08f)
#define ARM_PID_MOTOR_KD_RPM_PER_DPS        (0.15f)
#define ARM_PID_MOTOR_INTEGRAL_LIMIT_DEG_S (10.0f)

/* Sensor mounting and motor direction must be checked on the actual arm. */
#define ARM_LEVEL_PITCH_ZERO_OFFSET_DEG    (0.0f)
#define ARM_LEVEL_MOTOR_POSITIVE_IS_CCW     (0U)
#define ARM_LEVEL_MOTOR_ID                  (4U)
#define ARM_LEVEL_MOTOR_ACC_RPM_PER_S       (500U)
#define ARM_LEVEL_MOTOR_MAX_SPEED_RPM       (60.0f)
#define ARM_LEVEL_MOTOR_MIN_SPEED_RPM       (0.1f)
#define ARM_LEVEL_PITCH_DEADBAND_DEG        (0.1f)
#define ARM_LEVEL_GYRO_DEADBAND_DPS         (0.5f)
/* 目标角相对水平面定义，正负方向沿用 JY901S 的 pitch，边界值允许调用。 */
#define ARM_LEVEL_TARGET_MIN_DEG            (-90.0f)
#define ARM_LEVEL_TARGET_MAX_DEG            (10.0f)
#define ARM_LEVEL_TARGET_SETTLE_MS          (100U)  /* 连续新姿态帧满足到位条件的时长。 */
#define ARM_LEVEL_MOVE_TIMEOUT_MS           (5000U) /* 故障等待上限，不是固定运动时间。 */
typedef struct
{
    float pitch_deg;             /* JY901S 原始俯仰角，单位度。 */
    float target_pitch_deg;      /* 当前生效的目标角，0 度为校准后的水平。 */
    float pitch_error_deg;       /* 当前角减去水平零偏和目标角。 */
    float command_rpm;
    uint32_t target_sequence;   /* 当前控制周期使用的请求序号，防止旧到位状态误判。 */
    uint32_t sample_tick_ms;    /* 最近一次新姿态帧的采样时刻。 */
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t cycle_overrun_count;
    uint8_t ready;
    uint8_t control_enabled;
    uint8_t motor_running;
    uint8_t motor_fault;
    uint8_t target_reached;     /* 已连续稳定到位；偏离目标或数据失效后清零。 */
} ArmEndLevelState_t;
/*
  函数说明: 启动 JY901S、等待连续新姿态帧并初始化末端电机；由 main_task 调用。
  调用示例: ArmEndLevelTask_Init();
  参数意义: 无。
*/
uint8_t ArmEndLevelTask_Init(void);
/*
  函数说明: 每 20 ms 检查姿态新鲜度并保持目标角；失效时停止末端电机。
  调用示例: 由 CubeMX 创建的 StartArmEndLevelTask 调用 ArmEndLevelTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void ArmEndLevelTask_Run(void *argument);
/*
  函数说明: 开关目标角保持；关闭会取消正在等待的角度动作，重新开启沿用原目标。
  调用示例: ArmEndLevelTask_SetControlEnabled(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
void ArmEndLevelTask_SetControlEnabled(uint8_t enabled);
/*
  函数说明: 转到指定俯仰角，实际稳定到位后返回，后台继续保持该角度。
  调用示例: if (!ArmEndLevelTask_MoveToAngle(-30.0f)) ++main_task_command_errors;
  参数意义: angle_deg 为相对水平面的绝对角，范围 -90～+10 度，0 度恢复水平。
            返回 1 表示新姿态帧连续 100 ms 满足角度和角速度死区；返回 0 表示失败。
            在初始化并启用保持后，由 main_task 等普通任务调用，不能在中断中调用。
            参数非法、未就绪或其他调用进行中时拒绝请求，不改变当前目标。
            已接受的动作遇到数据失效、控制关闭、CAN 故障或超时会失败；故障退出关闭保持。
*/
uint8_t ArmEndLevelTask_MoveToAngle(float angle_deg);
/*
  函数说明: 在指定时限内等待连续有效姿态帧；它只等待数据，不执行机械臂动作。
  调用示例: ArmEndLevelTask_WaitReady(timeout_ms);
  参数意义: timeout_ms 为等待或心跳超时，毫秒。
*/
uint8_t ArmEndLevelTask_WaitReady(uint32_t timeout_ms);
/*
  函数说明: 在临界区复制上一个完整控制周期的调平状态。
  调用示例: ArmEndLevelTask_GetState(&state);
  参数意义: state 为状态值或状态快照。
*/
void ArmEndLevelTask_GetState(ArmEndLevelState_t *state);
#endif
