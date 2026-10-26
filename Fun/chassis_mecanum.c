/*
  程序说明: 麦克纳姆底盘机械尺寸配置及车体速度到四轮 RPM 的解算。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "chassis_mecanum.h"

/*
 * 轮速解算只负责数学换算，不负责方向位转换和 CAN 发送。
 * 逻辑轮速的正负会在 chassis_control.c 中映射为各电机的 CW/CCW。
 */

#include <math.h>
#include <string.h>

#define CHASSIS_PI                 (3.14159265358979323846f)
#define CHASSIS_SQRT_HALF          (0.70710678118654752440f)
#define CHASSIS_TWO_PI             (6.28318530717958647692f)

/*
  函数说明: 判断参与麦克纳姆解算的浮点参数是否有限。
  调用示例: ChassisMecanum_Finite(value);
  参数意义: value 为输入数值。
*/
static uint8_t ChassisMecanum_Finite(float value)
{
    return (value == value) ? 1U : 0U;
}

/*
  函数说明: 装载轮径、轴距、轮距和传动比等原底盘机械参数。
  调用示例: ChassisMecanum_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ChassisMecanum_LoadDefaultConfig(ChassisMecanumConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    /* 轴距、轮距均为轮子旋转中心之间的距离。 */
    config->wheel_radius_mm = 37.5f;
    config->wheelbase_mm = 230.0f;
    config->track_width_mm = 230.0f;
    config->motor_revolutions_per_wheel_revolution = 1.0f;
}

/*
  函数说明: 检查麦克纳姆底盘几何参数是否为合法正值。
  调用示例: ChassisMecanum_ConfigIsValid(config);
  参数意义: config 为对应模块的配置结构。
*/
uint8_t ChassisMecanum_ConfigIsValid(
    const ChassisMecanumConfig_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }

    return (ChassisMecanum_Finite(config->wheel_radius_mm) != 0U) &&
           (ChassisMecanum_Finite(config->wheelbase_mm) != 0U) &&
           (ChassisMecanum_Finite(config->track_width_mm) != 0U) &&
           (ChassisMecanum_Finite(
                config->motor_revolutions_per_wheel_revolution) != 0U) &&
           (config->wheel_radius_mm > 0.0f) &&
           (config->wheelbase_mm >= 0.0f) &&
           (config->track_width_mm >= 0.0f) &&
           (config->motor_revolutions_per_wheel_revolution > 0.0f);
}

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
    ChassisWheelVelocity_t *wheel_velocity)
{
    float rotation_mm_s;
    float wheel_mm_s[CHASSIS_MOTOR_COUNT];
    float rpm_per_mm_s;
    uint8_t index;

    if ((body_velocity == NULL) ||
        (wheel_velocity == NULL) ||
        (ChassisMecanum_ConfigIsValid(config) == 0U))
    {
        return 0U;
    }

    if ((ChassisMecanum_Finite(body_velocity->x_right_mm_s) == 0U) ||
        (ChassisMecanum_Finite(body_velocity->y_forward_mm_s) == 0U) ||
        (ChassisMecanum_Finite(body_velocity->yaw_ccw_deg_s) == 0U))
    {
        return 0U;
    }

    /* 旋转轮边速度：((轴距+轮距)/2) * cos45° * yaw_rad_s。 */
    rotation_mm_s =
        ((config->wheelbase_mm + config->track_width_mm) * 0.5f) *
        CHASSIS_SQRT_HALF *
        (body_velocity->yaw_ccw_deg_s * CHASSIS_PI / 180.0f);

    /*
     * 保持原工程已经验证过的轮子符号：
     *   +X 右移：[-, +, +, -]
     *   +Y 前进：[+, +, -, -]
     *   +Yaw CCW：[-, +, -, +]
     * [0..3] 依次为 ID5、ID6、ID7、ID8。
     */
    wheel_mm_s[0] = CHASSIS_SQRT_HALF *
                    (-body_velocity->x_right_mm_s +
                     body_velocity->y_forward_mm_s) - rotation_mm_s;
    wheel_mm_s[1] = CHASSIS_SQRT_HALF *
                    ( body_velocity->x_right_mm_s +
                     body_velocity->y_forward_mm_s) + rotation_mm_s;
    wheel_mm_s[2] = CHASSIS_SQRT_HALF *
                    ( body_velocity->x_right_mm_s -
                     body_velocity->y_forward_mm_s) - rotation_mm_s;
    wheel_mm_s[3] = CHASSIS_SQRT_HALF *
                    (-body_velocity->x_right_mm_s -
                     body_velocity->y_forward_mm_s) + rotation_mm_s;

    rpm_per_mm_s = (60.0f *
                    config->motor_revolutions_per_wheel_revolution) /
                   (CHASSIS_TWO_PI * config->wheel_radius_mm);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        wheel_velocity->rpm[index] = wheel_mm_s[index] * rpm_per_mm_s;
    }

    return 1U;
}
