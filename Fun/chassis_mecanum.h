#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

/*
 * 文件作用：四个 45° 全向轮的运动学解算。
 * 坐标约定：+X 向右、+Y 向前，俯视逆时针为正旋转。
 * 轮子数组顺序固定为：ID5、ID6、ID7、ID8。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CHASSIS_MOTOR_COUNT    (4U)

/* 底盘坐标：+X 右移，+Y 前进，Yaw 逆时针为正。 */
typedef struct
{
    float wheel_radius_mm; /* 轮半径，mm。 */
    float wheelbase_mm;    /* 前后轮旋转中心距离，mm。 */
    float track_width_mm;  /* 左右轮旋转中心距离，mm。 */
    float motor_revolutions_per_wheel_revolution; /* 电机/车轮转速比。 */
} ChassisMecanumConfig_t;

typedef struct
{
    float x_right_mm_s;    /* +X 右移速度，mm/s。 */
    float y_forward_mm_s;  /* +Y 前进速度，mm/s。 */
    float yaw_ccw_deg_s;   /* 逆时针角速度，deg/s。 */
} ChassisBodyVelocity_t;

/* rpm 为带符号的逻辑转速，正负号表示轮子逻辑正方向。 */
typedef struct
{
    float rpm[CHASSIS_MOTOR_COUNT]; /* 带符号的逻辑轮速，RPM。 */
} ChassisWheelVelocity_t;

void ChassisMecanum_LoadDefaultConfig(ChassisMecanumConfig_t *config);

uint8_t ChassisMecanum_ConfigIsValid(
    const ChassisMecanumConfig_t *config);

uint8_t ChassisMecanum_SolveVelocity(
    const ChassisMecanumConfig_t *config,
    const ChassisBodyVelocity_t *body_velocity,
    ChassisWheelVelocity_t *wheel_velocity);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MECANUM_H */
