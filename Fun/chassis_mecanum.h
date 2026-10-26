/*
  程序说明: 麦克纳姆底盘机械尺寸配置及车体速度到四轮 RPM 的解算。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

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

/*
  函数说明: 装载轮径、轴距、轮距和传动比等原底盘机械参数。
  调用示例: ChassisMecanum_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ChassisMecanum_LoadDefaultConfig(ChassisMecanumConfig_t *config);

/*
  函数说明: 检查麦克纳姆底盘几何参数是否为合法正值。
  调用示例: ChassisMecanum_ConfigIsValid(config);
  参数意义: config 为对应模块的配置结构。
*/
uint8_t ChassisMecanum_ConfigIsValid(
    const ChassisMecanumConfig_t *config);

/*
  函数说明: 把车体横向、纵向和旋转速度换算为四轮带符号逻辑 RPM。
  调用示例: ChassisMecanum_SolveVelocity(config, body_velocity, wheel_velocity);
  参数意义: config 为对应模块的配置结构。
            body_velocity 为输入车体速度，平移 mm/s、转动 deg/s。
            wheel_velocity 为输出四轮带符号转速，RPM。
*/
uint8_t ChassisMecanum_SolveVelocity(
    const ChassisMecanumConfig_t *config,
    const ChassisBodyVelocity_t *body_velocity,
    ChassisWheelVelocity_t *wheel_velocity);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MECANUM_H */
