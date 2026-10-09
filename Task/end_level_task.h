#ifndef TASK_END_LEVEL_TASK_H
#define TASK_END_LEVEL_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 末端水平任务的采样周期。ICM42688 的 ODR 同样配置为 100 Hz。 */
#define END_LEVEL_TASK_PERIOD_MS              (10U)
#define END_LEVEL_TASK_CALIBRATION_SAMPLES   (100U)
#define END_LEVEL_TASK_CALIBRATION_READ_RETRIES (3U)
#define END_LEVEL_TASK_CALIBRATION_MAX_CONSECUTIVE_ERRORS (5U)
#define END_LEVEL_TASK_READY_TIMEOUT_MS      (5000U)

/*
 * 末端轴的电机参数。电机方向和传感器安装轴必须实测后再打开控制。
 * 0：只采集姿态；1：ID4 完成初始化后再启用速度模式 roll 闭环。
 * 闭环目标为固定 0°，控制器为角度 PI 加陀螺仪角速度阻尼。
 */
#define END_LEVEL_CONTROL_DEFAULT            (1U)
#define END_LEVEL_MOTOR_ID                   (4U)
#define END_LEVEL_MOTOR_ACC_RPM_PER_S       (100U)
#define END_LEVEL_MOTOR_MAX_SPEED_RPM        (20.0f)
#define END_LEVEL_MOTOR_KP_RPM_PER_DEG       (1.0f)
#define END_LEVEL_MOTOR_KI_RPM_PER_DEG_S     (0.08f)
#define END_LEVEL_MOTOR_KD_RPM_PER_DPS       (0.15f)
#define END_LEVEL_MOTOR_INTEGRAL_LIMIT_DEG_S (10.0f)
#define END_LEVEL_MOTOR_MIN_COMMAND_RPM      (0.10f)
#define END_LEVEL_MOTOR_DEADBAND_DEG         (0.10f)

/* Roll 经过安装零偏修正后，以 0° 作为固定水平目标。 */
#define END_LEVEL_ROLL_TARGET_DEG            (0.0f)
#define END_LEVEL_ROLL_ZERO_OFFSET_DEG      (0.0f)

/* 传感器正方向到 ID4 正方向的映射，0 表示电机 CW 为正。 */
#define END_LEVEL_MOTOR_POSITIVE_IS_CCW      (0U)

typedef struct
{
    float roll_deg;
    float pitch_deg;
    /* Yaw 仅由陀螺仪积分得到，是相对角度，长期会漂移。 */
    float yaw_deg;
    float gyro_bias_x_dps;
    float gyro_bias_y_dps;
    float gyro_bias_z_dps;
    float target_roll_deg;
    float target_pitch_deg;
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t int1_count;
    uint8_t ready;
    uint8_t fault;
    uint8_t control_enabled;
} EndLevelTelemetry_t;

/* FreeRTOS 线程入口，由 Core/Src/freertos.c 的 wrapper 调用。 */
void EndLevelTask_Run(void *argument);

/* 供机械臂任务等待传感器初始化和静止零偏校准。 */
bool EndLevelTask_IsReady(void);
bool EndLevelTask_HasFault(void);
bool EndLevelTask_WaitReady(uint32_t timeout_ms);

/* 运行时打开/关闭 ID4 水平补偿；默认值由 END_LEVEL_CONTROL_DEFAULT 决定。 */
void EndLevelTask_SetControlEnabled(bool enabled);
bool EndLevelTask_IsControlEnabled(void);

/* 兼容接口：恢复固定 0° 目标；安装零偏请修改 END_LEVEL_ROLL_ZERO_OFFSET_DEG。 */
void EndLevelTask_CaptureLevel(void);

/* 读取一份一致的任务遥测快照。 */
void EndLevelTask_GetTelemetry(EndLevelTelemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* TASK_END_LEVEL_TASK_H */
