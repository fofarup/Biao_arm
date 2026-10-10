#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

/*
 * 文件作用：底盘控制器。
 * 速度模式直接跟踪车体速度；位置模式使用 OPS9/HW101 做外部闭环，
 * 再通过麦克纳姆轮解算生成四个电机的 F6 速度指令。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "X_V2.h"
#include "chassis_mecanum.h"

typedef enum
{
    CHASSIS_CONTROL_STOP = 0U, /* 受控停止。 */
    CHASSIS_CONTROL_VELOCITY,  /* 速度指令模式。 */
    CHASSIS_CONTROL_POSE,      /* OPS9/HW101 位置闭环模式。 */
    CHASSIS_CONTROL_FAULT      /* 传感器或 CAN 发送故障，强制停机。 */
} ChassisControlMode_t;

/* 相对位移动作的阶段：先平移，再旋转。 */
typedef enum
{
    CHASSIS_SEQUENCE_IDLE = 0U,
    CHASSIS_SEQUENCE_TRANSLATE,
    CHASSIS_SEQUENCE_ROTATE,
    CHASSIS_SEQUENCE_DONE
} ChassisSequencePhase_t;

typedef struct
{
    float x_mm;     /* 相对上电原点的 X 坐标，mm。 */
    float y_mm;     /* 相对上电原点的 Y 坐标，mm。 */
    float yaw_deg;  /* 相对上电航向的角度，deg。 */
} ChassisPose_t;

typedef struct
{
    ChassisMecanumConfig_t mecanum;
    uint8_t motor_addr[CHASSIS_MOTOR_COUNT];
    X_V2_Direction_t motor_positive_dir[CHASSIS_MOTOR_COUNT];

    /* 外环允许的电机速度变化上限；单位 RPM/s。 */
    float motor_acc_rpm_per_s[CHASSIS_MOTOR_COUNT];
    float motor_dec_rpm_per_s[CHASSIS_MOTOR_COUNT];

    float motor_max_rpm[CHASSIS_MOTOR_COUNT];

    float max_x_right_mm_s;    /* 车体 +X 最大速度，mm/s。 */
    float max_y_forward_mm_s;  /* 车体 +Y 最大速度，mm/s。 */
    float max_yaw_deg_s;       /* 车体最大角速度，deg/s。 */
    float max_x_acc_mm_s2;     /* 车体 X 加速度上限，mm/s²。 */
    float max_y_acc_mm_s2;     /* 车体 Y 加速度上限，mm/s²。 */
    float max_yaw_acc_deg_s2;  /* 车体角加速度上限，deg/s²。 */
    float max_x_dec_mm_s2;     /* 车体 X 减速度上限，mm/s²。 */
    float max_y_dec_mm_s2;     /* 车体 Y 减速度上限，mm/s²。 */
    float max_yaw_dec_deg_s2;  /* 车体角减速度上限，deg/s²。 */

    /* 位置环：误差单位 mm，输出单位 mm/s。 */
    float position_kp;
    float position_kd;

    /* 航向环：误差单位 deg，输出单位 deg/s。 */
    float yaw_kp;
    float yaw_kd;

    uint32_t sensor_timeout_ms; /* OPS9/HW101 数据超时判定，ms。 */
} ChassisControlConfig_t;

typedef struct
{
    float signed_rpm[CHASSIS_MOTOR_COUNT];
} ChassisWheelSetpoint_t;

typedef struct
{
    ChassisControlMode_t mode; /* 当前控制模式。 */
    ChassisPose_t target_pose; /* 当前闭环目标。 */
    ChassisPose_t current_pose; /* 传感器测得的当前位置。 */
    ChassisPose_t pose_error; /* 目标减当前位置。 */
    ChassisBodyVelocity_t body_velocity_command; /* 当前输出车体速度。 */
    uint8_t origin_valid; /* 是否已经锁定传感器原点。 */
    uint8_t sensor_fault; /* OPS9/HW101 超时。 */
    ChassisSequencePhase_t sequence_phase; /* 相对动作的当前阶段。 */
    uint8_t sequence_done; /* 相对动作是否完成。 */
} ChassisControlState_t;

void ChassisControl_LoadDefaultConfig(ChassisControlConfig_t *config);

void ChassisControl_Init(const ChassisControlConfig_t *config);

uint8_t ChassisControl_SetVelocity(float x_right_mm_s,
                                   float y_forward_mm_s,
                                   float yaw_ccw_deg_s);

uint8_t ChassisControl_SetPoseTarget(float x_mm,
                                     float y_mm,
                                     float yaw_deg);

/*
 * 相对当前位置执行一次“先平移 XY、再旋转 yaw”的动作。
 * dx/dy 是调用瞬间车体系中的位移（+X 车右、+Y 车前），单位为 mm；
 * 内部会按当前航向转换到 OPS9 固定坐标系。dyaw 为相对转角，单位为 deg；
 * acc/dec 单位为 RPM/s，max_rpm 单位为 RPM，四轮共用这一组限制。
 * acc/dec 限制主动平移和转弯；行走中的航向/横向纠偏及到位保持
 * 使用 chassis_control.c 顶部各自的速度与斜率宏。所有分量仍受 max_rpm 限制。
 * 函数只能从运行中的 RTOS 任务调用，并阻塞到 XY 合成误差不超过 2 mm、
 * 航向误差不超过 1° 后返回 1；启动失败、故障、中断或 30 秒超时返回 0。
 * 返回后底盘任务继续闭环保持目标位姿，调用者可以直接下发下一条动作。
 */
uint8_t ChassisControl_MoveRelativeSequential(
    float dx_mm,
    float dy_mm,
    float dyaw_deg,
    float acc_rpm_per_s,
    float dec_rpm_per_s,
    float max_rpm);

uint8_t ChassisControl_IsMoveComplete(void);

void ChassisControl_Stop(void);
void ChassisControl_ClearFault(void);
void ChassisControl_FaultStop(void);

/* 由 ChassisTask 每个周期调用，输出已完成加减速限制的轮速。 */
uint8_t ChassisControl_Update(float dt_s,
                              ChassisWheelSetpoint_t *setpoint);

/* 将带符号逻辑转速转换为 X 固件 F6 命令；底盘 F6 加速度固定为 0。 */
void ChassisControl_BuildMotorCommands(
    const ChassisWheelSetpoint_t *setpoint,
    X_V2_Vel_Command_t commands[CHASSIS_MOTOR_COUNT]);

void ChassisControl_GetState(ChassisControlState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONTROL_H */
