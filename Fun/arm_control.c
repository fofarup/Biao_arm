/*
  程序说明: 机械臂电机映射、同步点位规划及微动处理、曲线控制和真实到位处理。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#include "arm_control.h"
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "X_V2.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#define ARM_CONTROL_DEG_PER_RPM_S   (6.0f)
#define ARM_CONTROL_MIN_SPEED_RPM   (0.1f)
#define ARM_CONTROL_SPEED_STEP_RPM  (0.1)
#define ARM_CONTROL_TIME_EPS_S     (1.0e-3)
#define ARM_CONTROL_TIME_TOLERANCE_S (0.01)
#define ARM_CONTROL_ACCEL_CORRECTION_STEPS (64U)
#define ARM_CONTROL_SYNC_ATTEMPTS (64U)


/*
  函数说明: 把机械安装方向转换为方向偏置表下标；非法方向返回 -1。
  调用示例: ArmMotorMap_DirectionIndex(direction);
  参数意义: direction 为机械安装方向枚举。
*/
static int ArmMotorMap_DirectionIndex(ArmInstallationDirection direction)
{
    int value = (int)direction;

    if ((value < (int)ARM_INSTALLATION_DIRECTION_1) ||
        (value > (int)ARM_INSTALLATION_DIRECTION_4))
    {
        return -1;
    }
    return value - 1;
}

/*
  函数说明: 检查电机映射中的减速比、脉冲数、耦合系数和方向配置。
  调用示例: ArmMotorMap_ConfigIsValid(config);
  参数意义: config 为对应模块的配置结构。
*/
static int ArmMotorMap_ConfigIsValid(const ArmMotorMapConfig *config)
{
    if ((config == NULL) ||
        !isfinite(config->motor1_ratio) ||
        !isfinite(config->motor2_ratio) ||
        !isfinite(config->motor3_gamma_ratio) ||
        !isfinite(config->gamma_zero_deg) ||
        (config->motor1_ratio <= 0.0f) ||
        (config->motor2_ratio <= 0.0f) ||
        (config->motor3_gamma_ratio <= 0.0f) ||
        (config->pulses_per_revolution == 0U) ||
        (config->motor1_positive_direction > ARM_MOTOR_DIR_CCW) ||
        (config->motor2_positive_direction > ARM_MOTOR_DIR_CCW) ||
        (config->motor3_positive_direction > ARM_MOTOR_DIR_CCW))
    {
        return 0;
    }

    return 1;
}

/*
  函数说明: 根据脉冲符号及正向安装定义确定电机转向。
  调用示例: ArmMotorMap_DirectionForPulse(positive_direction, pulse);
  参数意义: positive_direction 为电机逻辑正向对应的物理转向。
            pulse 为带符号电机脉冲。
*/
static ArmMotorDirection ArmMotorMap_DirectionForPulse(
    ArmMotorDirection positive_direction,
    int32_t pulse)
{
    if (pulse >= 0)
    {
        return positive_direction;
    }

    return (positive_direction == ARM_MOTOR_DIR_CW) ?
           ARM_MOTOR_DIR_CCW : ARM_MOTOR_DIR_CW;
}

/*
  函数说明: 把双精度脉冲值四舍五入为 int32_t，同时检查范围。
  调用示例: ArmMotorMap_RoundPulse(value, result);
  参数意义: value 为输入数值。
            result 为输出计算结果。
*/
static ArmMotorMapStatus ArmMotorMap_RoundPulse(double value,
                                                int32_t *result)
{
    double rounded;

    if (!isfinite(value))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }

    rounded = (value >= 0.0) ? floor(value + 0.5) : ceil(value - 0.5);
    if ((rounded > (double)INT32_MAX) || (rounded < (double)INT32_MIN))
    {
        return ARM_MOTOR_MAP_OVERFLOW;
    }

    *result = (int32_t)rounded;
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 装载原机械臂的减速比、脉冲数、耦合关系及方向偏置。
  调用示例: ArmMotorMap_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmMotorMap_LoadDefaultConfig(ArmMotorMapConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->motor1_ratio = 26.25f;
    config->motor2_ratio = 4.5f;
    config->motor3_gamma_ratio = 13.5f;
    config->gamma_zero_deg = 12.02f;
    config->pulses_per_revolution = 3200U;

    /* 实测正方向：底座 CCW、大臂 CCW；地址3正脉冲坐标为 CCW。 */
    config->motor1_positive_direction = ARM_MOTOR_DIR_CCW;
    config->motor2_positive_direction = ARM_MOTOR_DIR_CCW;
    config->motor3_positive_direction = ARM_MOTOR_DIR_CCW;

    config->direction_offset_pulse[0] = 0;
    config->direction_offset_pulse[1] = -42000;
    config->direction_offset_pulse[2] = -21000;
    config->direction_offset_pulse[3] = 0;
    config->direction_offset_valid[0] = 1U;
    config->direction_offset_valid[1] = 1U;
    config->direction_offset_valid[2] = 1U;
    config->direction_offset_valid[3] = 0U;
}

/*
  函数说明: 把相对机械零位的关节偏移换算为几何关节角。
  调用示例: ArmMotorMap_OffsetsToJoints(config, base_deg, large_arm_deg, small_arm_deg, result);
  参数意义: config 为对应模块的配置结构。
            base_deg 为底座相对机械零位角度，度。
            large_arm_deg 为大臂相对机械零位角度，度。
            small_arm_deg 为小臂相对机械零位角度，度。
            result 为输出计算结果。
*/
ArmMotorMapStatus ArmMotorMap_OffsetsToJoints(
    const ArmMotorMapConfig *config,
    float base_deg,
    float large_arm_deg,
    float small_arm_deg,
    ArmJointAngles *result)
{
    if ((config == NULL) || (result == NULL))
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    if (!ArmMotorMap_ConfigIsValid(config))
    {
        return ARM_MOTOR_MAP_INVALID_CONFIG;
    }
    if (!isfinite(base_deg) || !isfinite(large_arm_deg) ||
        !isfinite(small_arm_deg))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }

    result->alpha_deg = base_deg;
    result->beta_deg = large_arm_deg;
    result->gamma_deg = config->gamma_zero_deg +
        ((config->motor2_ratio / config->motor3_gamma_ratio) *
         large_arm_deg) + small_arm_deg;

    if (!isfinite(result->gamma_deg))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 记录某一安装方向对应的底座脉冲偏置并标记有效。
  调用示例: ArmMotorMap_SetDirectionOffset(config, direction, offset_pulse);
  参数意义: config 为对应模块的配置结构。
            direction 为机械安装方向枚举。
            offset_pulse 为底座方向偏置，脉冲。
*/
ArmMotorMapStatus ArmMotorMap_SetDirectionOffset(
    ArmMotorMapConfig *config,
    ArmInstallationDirection direction,
    int32_t offset_pulse)
{
    int index;

    if (config == NULL)
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    index = ArmMotorMap_DirectionIndex(direction);
    if (index < 0)
    {
        return ARM_MOTOR_MAP_INVALID_DIRECTION;
    }

    config->direction_offset_pulse[index] = offset_pulse;
    config->direction_offset_valid[index] = 1U;
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 清除指定安装方向的底座偏置有效标志。
  调用示例: ArmMotorMap_ClearDirectionOffset(config, direction);
  参数意义: config 为对应模块的配置结构。
            direction 为机械安装方向枚举。
*/
ArmMotorMapStatus ArmMotorMap_ClearDirectionOffset(
    ArmMotorMapConfig *config,
    ArmInstallationDirection direction)
{
    int index;

    if (config == NULL)
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    index = ArmMotorMap_DirectionIndex(direction);
    if (index < 0)
    {
        return ARM_MOTOR_MAP_INVALID_DIRECTION;
    }

    config->direction_offset_valid[index] = 0U;
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 两种正向映射共用输入检查：关节有限值、安装方向及方向偏置。
  角度计算与脉冲舍入仍由各自接口完成，避免改变原有量化结果。
  调用示例: ArmMotorMap_ValidateJointInput(config, installation_direction, joints, direction_index);
  参数意义: config 为对应模块的配置结构。
            installation_direction 为机械臂安装方向。
            joints 为三个几何关节角，度。
            direction_index 为输出安装方向表下标。
*/
static ArmMotorMapStatus ArmMotorMap_ValidateJointInput(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const ArmJointAngles *joints,
    int *direction_index)
{
    if ((config == NULL) || (joints == NULL))
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    if (!ArmMotorMap_ConfigIsValid(config))
    {
        return ARM_MOTOR_MAP_INVALID_CONFIG;
    }
    if (!isfinite(joints->alpha_deg) ||
        !isfinite(joints->beta_deg) ||
        !isfinite(joints->gamma_deg))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }
    *direction_index = ArmMotorMap_DirectionIndex(installation_direction);
    if (*direction_index < 0)
    {
        return ARM_MOTOR_MAP_INVALID_DIRECTION;
    }
    if (config->direction_offset_valid[*direction_index] == 0U)
    {
        return ARM_MOTOR_MAP_DIRECTION_NOT_CONFIGURED;
    }
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 把几何关节角换算为驱动器的方向、绝对脉冲和角度目标。
  调用示例: ArmMotorMap_JointsToTargets(config, installation_direction, joints, result);
  参数意义: config 为对应模块的配置结构。
            installation_direction 为机械臂安装方向。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
*/
ArmMotorMapStatus ArmMotorMap_JointsToTargets(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const ArmJointAngles *joints,
    ArmMotorTargets *result)
{
    ArmMotorMapStatus status;
    double pulse_scale;
    double motor1_pulse;
    double motor2_pulse;
    double motor3_pulse;
    int direction_index;
    int64_t motor1_with_offset;

    if (result == NULL)
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    status = ArmMotorMap_ValidateJointInput(
        config, installation_direction, joints, &direction_index);
    if (status != ARM_MOTOR_MAP_OK)
    {
        return status;
    }

    pulse_scale = (double)config->pulses_per_revolution / 360.0;
    motor1_pulse = (double)config->motor1_ratio *
                   (double)joints->alpha_deg * pulse_scale;
    motor2_pulse = (double)config->motor2_ratio *
                   (double)joints->beta_deg * pulse_scale;
    motor3_pulse = (((double)config->motor3_gamma_ratio *
                    ((double)config->gamma_zero_deg -
                     (double)joints->gamma_deg)) +
                   ((double)config->motor2_ratio *
                    (double)joints->beta_deg)) * pulse_scale;

    status = ArmMotorMap_RoundPulse(motor1_pulse, &result->pulse[0]);
    if (status != ARM_MOTOR_MAP_OK) return status;
    status = ArmMotorMap_RoundPulse(motor2_pulse, &result->pulse[1]);
    if (status != ARM_MOTOR_MAP_OK) return status;
    status = ArmMotorMap_RoundPulse(motor3_pulse, &result->pulse[2]);
    if (status != ARM_MOTOR_MAP_OK) return status;

    motor1_with_offset = (int64_t)result->pulse[0] +
                         (int64_t)config->direction_offset_pulse[direction_index];
    if ((motor1_with_offset > (int64_t)INT32_MAX) ||
        (motor1_with_offset < (int64_t)INT32_MIN))
    {
        return ARM_MOTOR_MAP_OVERFLOW;
    }
    result->pulse[0] = (int32_t)motor1_with_offset;

    result->direction[0] = ArmMotorMap_DirectionForPulse(
        config->motor1_positive_direction, result->pulse[0]);
    result->direction[1] = ArmMotorMap_DirectionForPulse(
        config->motor2_positive_direction, result->pulse[1]);
    result->direction[2] = ArmMotorMap_DirectionForPulse(
        config->motor3_positive_direction, result->pulse[2]);

    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 把关节角映射为连续的电机轴角度，用于曲线速度计算。
  调用示例: ArmMotorMap_JointsToMotorCoordinates(config, installation_direction, joints, result);
  参数意义: config 为对应模块的配置结构。
            installation_direction 为机械臂安装方向。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
*/
ArmMotorMapStatus ArmMotorMap_JointsToMotorCoordinates(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const ArmJointAngles *joints,
    ArmMotorCoordinates *result)
{
    ArmMotorMapStatus status;
    int direction_index;
    double offset_degrees;
    double motor1_degrees;
    double motor2_degrees;
    double motor3_degrees;

    if (result == NULL)
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    status = ArmMotorMap_ValidateJointInput(
        config, installation_direction, joints, &direction_index);
    if (status != ARM_MOTOR_MAP_OK)
    {
        return status;
    }

    offset_degrees =
        ((double)config->direction_offset_pulse[direction_index] * 360.0) /
        (double)config->pulses_per_revolution;
    motor1_degrees = ((double)config->motor1_ratio *
                      (double)joints->alpha_deg) + offset_degrees;
    motor2_degrees = (double)config->motor2_ratio *
                     (double)joints->beta_deg;
    motor3_degrees = ((double)config->motor3_gamma_ratio *
                      ((double)config->gamma_zero_deg -
                       (double)joints->gamma_deg)) +
                     ((double)config->motor2_ratio *
                      (double)joints->beta_deg);

    if (!isfinite(motor1_degrees) ||
        !isfinite(motor2_degrees) ||
        !isfinite(motor3_degrees))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }

    result->degree[0] = (float)motor1_degrees;
    result->degree[1] = (float)motor2_degrees;
    result->degree[2] = (float)motor3_degrees;
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 从三轴带符号脉冲反算几何关节角，包含减速比和机构耦合。
  调用示例: ArmMotorMap_PulsesToJoints(config, installation_direction, pulse, result);
  参数意义: config 为对应模块的配置结构。
            installation_direction 为机械臂安装方向。
            pulse 为带符号电机脉冲。
            result 为输出计算结果。
*/
ArmMotorMapStatus ArmMotorMap_PulsesToJoints(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const int32_t pulse[3],
    ArmJointAngles *result)
{
    int direction_index;
    double pulse_scale;
    double motor1_pulse;
    double motor2_degrees;
    double motor3_degrees;
    double alpha;
    double beta;
    double gamma;

    if ((config == NULL) || (pulse == NULL) || (result == NULL))
    {
        return ARM_MOTOR_MAP_NULL_POINTER;
    }
    if (!ArmMotorMap_ConfigIsValid(config))
    {
        return ARM_MOTOR_MAP_INVALID_CONFIG;
    }

    direction_index = ArmMotorMap_DirectionIndex(installation_direction);
    if (direction_index < 0)
    {
        return ARM_MOTOR_MAP_INVALID_DIRECTION;
    }
    if (config->direction_offset_valid[direction_index] == 0U)
    {
        return ARM_MOTOR_MAP_DIRECTION_NOT_CONFIGURED;
    }

    pulse_scale = (double)config->pulses_per_revolution / 360.0;
    motor1_pulse = (double)pulse[0] -
                   (double)config->direction_offset_pulse[direction_index];
    motor2_degrees = (double)pulse[1] / pulse_scale;
    motor3_degrees = (double)pulse[2] / pulse_scale;

    alpha = (motor1_pulse / pulse_scale) /
            (double)config->motor1_ratio;
    beta = motor2_degrees / (double)config->motor2_ratio;
    gamma = (double)config->gamma_zero_deg +
            (((double)config->motor2_ratio * beta) - motor3_degrees) /
            (double)config->motor3_gamma_ratio;

    if (!isfinite(alpha) || !isfinite(beta) || !isfinite(gamma))
    {
        return ARM_MOTOR_MAP_NUMERIC_ERROR;
    }

    result->alpha_deg = (float)alpha;
    result->beta_deg = (float)beta;
    result->gamma_deg = (float)gamma;
    return ARM_MOTOR_MAP_OK;
}

/*
  函数说明: 把电机映射状态码转换为只读诊断字符串。
  调用示例: ArmMotorMap_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmMotorMap_StatusString(ArmMotorMapStatus status)
{
    switch (status)
    {
        case ARM_MOTOR_MAP_OK: return "OK";
        case ARM_MOTOR_MAP_NULL_POINTER: return "null pointer";
        case ARM_MOTOR_MAP_INVALID_CONFIG: return "invalid configuration";
        case ARM_MOTOR_MAP_INVALID_DIRECTION: return "invalid direction";
        case ARM_MOTOR_MAP_DIRECTION_NOT_CONFIGURED:
            return "direction not configured";
        case ARM_MOTOR_MAP_NUMERIC_ERROR: return "numeric error";
        case ARM_MOTOR_MAP_OVERFLOW: return "target overflow";
        default: return "unknown status";
    }
}


/*
  函数说明: 检查三维坐标的三个分量是否均为有限值。
  调用示例: ArmTrajectory_IsFinitePosition(position);
  参数意义: position 为三维位置或单轴位置，mm。
*/
static int ArmTrajectory_IsFinitePosition(
    const ArmCartesianPosition *position)
{
    return (position != NULL) &&
           isfinite(position->x_mm) &&
           isfinite(position->y_mm) &&
           isfinite(position->z_mm);
}

/*
  函数说明: 求三次贝塞尔曲线在单个坐标轴上的位置。
  调用示例: ArmTrajectory_BezierValue(p0, p1, p2, p3, s);
  参数意义: p0 为曲线起点的单轴坐标，mm。
            p1 为第一个控制点的单轴坐标，mm。
            p2 为第二个控制点的单轴坐标，mm。
            p3 为曲线终点的单轴坐标，mm。
            s 为归一化路径进度，0～1。
*/
static float ArmTrajectory_BezierValue(float p0,
                                       float p1,
                                       float p2,
                                       float p3,
                                       float s)
{
    float one_minus_s = 1.0f - s;
    float one_minus_s2 = one_minus_s * one_minus_s;
    float s2 = s * s;

    return (one_minus_s2 * one_minus_s * p0) +
           (3.0f * one_minus_s2 * s * p1) +
           (3.0f * one_minus_s * s2 * p2) +
           (s2 * s * p3);
}

/*
  函数说明: 求三次贝塞尔曲线对路径进度的一阶导数。
  调用示例: ArmTrajectory_BezierFirstDerivative(p0, p1, p2, p3, s);
  参数意义: p0 为曲线起点的单轴坐标，mm。
            p1 为第一个控制点的单轴坐标，mm。
            p2 为第二个控制点的单轴坐标，mm。
            p3 为曲线终点的单轴坐标，mm。
            s 为归一化路径进度，0～1。
*/
static float ArmTrajectory_BezierFirstDerivative(float p0,
                                                 float p1,
                                                 float p2,
                                                 float p3,
                                                 float s)
{
    float one_minus_s = 1.0f - s;

    return (3.0f * one_minus_s * one_minus_s * (p1 - p0)) +
           (6.0f * one_minus_s * s * (p2 - p1)) +
           (3.0f * s * s * (p3 - p2));
}

/*
  函数说明: 求三次贝塞尔曲线对路径进度的二阶导数。
  调用示例: ArmTrajectory_BezierSecondDerivative(p0, p1, p2, p3, s);
  参数意义: p0 为曲线起点的单轴坐标，mm。
            p1 为第一个控制点的单轴坐标，mm。
            p2 为第二个控制点的单轴坐标，mm。
            p3 为曲线终点的单轴坐标，mm。
            s 为归一化路径进度，0～1。
*/
static float ArmTrajectory_BezierSecondDerivative(float p0,
                                                  float p1,
                                                  float p2,
                                                  float p3,
                                                  float s)
{
    return (6.0f * (1.0f - s) * (p2 - (2.0f * p1) + p0)) +
           (6.0f * s * (p3 - (2.0f * p2) + p1));
}

/*
  函数说明: 将曲线对进度的导数换算为单轴位置、速度和加速度。
  调用示例: ArmTrajectory_EvaluateAxis(p0, p1, p2, p3, s, ds_dt, d2s_dt2, position, velocity, acceleration);
  参数意义: p0 为曲线起点的单轴坐标，mm。
            p1 为第一个控制点的单轴坐标，mm。
            p2 为第二个控制点的单轴坐标，mm。
            p3 为曲线终点的单轴坐标，mm。
            s 为归一化路径进度，0～1。
            ds_dt 为路径进度一阶时间导数，1/s。
            d2s_dt2 为路径进度二阶时间导数，1/s²。
            position 为三维位置或单轴位置，mm。
            velocity 为单轴或车体速度，mm/s；车体转动分量为 deg/s。
            acceleration 为加速度；与输入速度对应的每秒变化量。
*/
static void ArmTrajectory_EvaluateAxis(float p0,
                                       float p1,
                                       float p2,
                                       float p3,
                                       float s,
                                       float ds_dt,
                                       float d2s_dt2,
                                       float *position,
                                       float *velocity,
                                       float *acceleration)
{
    float first = ArmTrajectory_BezierFirstDerivative(
        p0, p1, p2, p3, s);
    float second = ArmTrajectory_BezierSecondDerivative(
        p0, p1, p2, p3, s);

    *position = ArmTrajectory_BezierValue(p0, p1, p2, p3, s);
    *velocity = first * ds_dt;
    *acceleration = (second * ds_dt * ds_dt) +
                    (first * d2s_dt2);
}

/*
  函数说明: 校验四个控制点和采样周期，建立三次贝塞尔轨迹。
  调用示例: ArmTrajectory_InitCubicBezier(trajectory, start, control1, control2, end, duration_s, sample_period_ms);
  参数意义: trajectory 为三次贝塞尔曲线结构。
            start 为起点位置或起始运动状态。
            control1 为第一个贝塞尔控制点，mm。
            control2 为第二个贝塞尔控制点，mm。
            end 为曲线终点坐标，mm。
            duration_s 为曲线参考时长，秒。
            sample_period_ms 为曲线采样周期，毫秒。
*/
ArmTrajectoryStatus ArmTrajectory_InitCubicBezier(
    ArmCubicBezierTrajectory *trajectory,
    const ArmCartesianPosition *start,
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *end,
    float duration_s,
    uint16_t sample_period_ms)
{
    double duration_ms_value;
    uint32_t duration_ms;
    uint32_t segment_count;

    if ((trajectory == NULL) || (start == NULL) ||
        (control1 == NULL) || (control2 == NULL) || (end == NULL))
    {
        return ARM_TRAJECTORY_NULL_POINTER;
    }

    memset(trajectory, 0, sizeof(*trajectory));
    if (!ArmTrajectory_IsFinitePosition(start) ||
        !ArmTrajectory_IsFinitePosition(control1) ||
        !ArmTrajectory_IsFinitePosition(control2) ||
        !ArmTrajectory_IsFinitePosition(end) ||
        !isfinite(duration_s) || (duration_s <= 0.0f) ||
        (sample_period_ms == 0U))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    /*
     * 执行时基为整数毫秒。先量化再用整数除法分段，避免 3.0f
     * 被表示成略大于 3000 ms 后错误地多生成一个零时长末段。
     */
    duration_ms_value = floor(((double)duration_s * 1000.0) + 0.5);
    if ((duration_ms_value < 1.0) || !isfinite(duration_ms_value) ||
        (duration_ms_value > (double)UINT_MAX))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }
    duration_ms = (uint32_t)duration_ms_value;
    segment_count = duration_ms / (uint32_t)sample_period_ms;
    if ((duration_ms % (uint32_t)sample_period_ms) != 0U)
    {
        ++segment_count;
    }
    if (segment_count == 0U)
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    trajectory->point[0] = *start;
    trajectory->point[1] = *control1;
    trajectory->point[2] = *control2;
    trajectory->point[3] = *end;
    trajectory->duration_ms = duration_ms;
    trajectory->duration_s = (float)duration_ms * 0.001f;
    trajectory->sample_period_ms = sample_period_ms;
    trajectory->segment_count = segment_count;
    trajectory->initialized = 1U;
    return ARM_TRAJECTORY_OK;
}

/*
  函数说明: 按给定运行时间采样曲线的位置、速度和加速度。
  调用示例: ArmTrajectory_Evaluate(trajectory, elapsed_s, sample);
  参数意义: trajectory 为三次贝塞尔曲线结构。
            elapsed_s 为曲线已经运行的时间，秒。
            sample 为曲线采样或 IMU 姿态数据。
*/
ArmTrajectoryStatus ArmTrajectory_Evaluate(
    const ArmCubicBezierTrajectory *trajectory,
    float elapsed_s,
    ArmTrajectorySample *sample)
{
    float u;
    float u2;
    float u3;
    float u4;
    float s;
    float ds_du;
    float d2s_du2;
    float ds_dt;
    float d2s_dt2;
    float duration_s;

    if ((trajectory == NULL) || (sample == NULL))
    {
        return ARM_TRAJECTORY_NULL_POINTER;
    }
    if ((trajectory->initialized == 0U) ||
        (trajectory->duration_ms == 0U) ||
        !isfinite(trajectory->duration_s) ||
        (trajectory->duration_s <= 0.0f) ||
        !isfinite(elapsed_s))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    if (elapsed_s < 0.0f)
    {
        elapsed_s = 0.0f;
    }
    if (elapsed_s > trajectory->duration_s)
    {
        elapsed_s = trajectory->duration_s;
    }

    duration_s = trajectory->duration_s;
    u = elapsed_s / duration_s;
    u2 = u * u;
    u3 = u2 * u;
    u4 = u3 * u;

    s = (10.0f * u3) - (15.0f * u4) + (6.0f * u4 * u);
    ds_du = (30.0f * u2) - (60.0f * u3) + (30.0f * u4);
    d2s_du2 = (60.0f * u) - (180.0f * u2) + (120.0f * u3);
    ds_dt = ds_du / duration_s;
    d2s_dt2 = d2s_du2 / (duration_s * duration_s);

    memset(sample, 0, sizeof(*sample));
    sample->path_parameter = s;
    sample->elapsed_s = elapsed_s;

    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].x_mm,
        trajectory->point[1].x_mm,
        trajectory->point[2].x_mm,
        trajectory->point[3].x_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.x_mm,
        &sample->velocity_mm_s.x_mm,
        &sample->acceleration_mm_s2.x_mm);
    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].y_mm,
        trajectory->point[1].y_mm,
        trajectory->point[2].y_mm,
        trajectory->point[3].y_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.y_mm,
        &sample->velocity_mm_s.y_mm,
        &sample->acceleration_mm_s2.y_mm);
    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].z_mm,
        trajectory->point[1].z_mm,
        trajectory->point[2].z_mm,
        trajectory->point[3].z_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.z_mm,
        &sample->velocity_mm_s.z_mm,
        &sample->acceleration_mm_s2.z_mm);

    if (!ArmTrajectory_IsFinitePosition(&sample->position) ||
        !ArmTrajectory_IsFinitePosition(&sample->velocity_mm_s) ||
        !ArmTrajectory_IsFinitePosition(&sample->acceleration_mm_s2) ||
        !isfinite(sample->path_parameter))
    {
        memset(sample, 0, sizeof(*sample));
        return ARM_TRAJECTORY_NUMERIC_ERROR;
    }

    return ARM_TRAJECTORY_OK;
}

/*
  函数说明: 把曲线计算状态码转换为只读诊断字符串。
  调用示例: ArmTrajectory_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmTrajectory_StatusString(ArmTrajectoryStatus status)
{
    switch (status)
    {
        case ARM_TRAJECTORY_OK: return "OK";
        case ARM_TRAJECTORY_NULL_POINTER: return "null pointer";
        case ARM_TRAJECTORY_INVALID_ARGUMENT: return "invalid argument";
        case ARM_TRAJECTORY_NUMERIC_ERROR: return "numeric error";
        default: return "unknown status";
    }
}


/*
  函数说明: 将带符号脉冲量化为驱动器的 0.1 度位置单位。
  调用示例: ArmControlProfile_PulseToProtocolTenths(pulse, pulses_per_revolution);
  参数意义: pulse 为带符号电机脉冲。
            pulses_per_revolution 为电机每转脉冲数。
*/
int64_t ArmControlProfile_PulseToProtocolTenths(
    int32_t pulse,
    uint32_t pulses_per_revolution)
{
    double tenths = (double)pulse * 3600.0 /
                    (double)pulses_per_revolution;

    if (tenths >= 0.0)
    {
        return (int64_t)floor(tenths + 0.5);
    }
    return (int64_t)ceil(tenths - 0.5);
}

/*
  函数说明: 按驱动器实际量化规则计算起点到目标的电机轴位移绝对值。
  调用示例: ArmControlProfile_DistanceDegrees(target_pulse, current_pulse, pulses_per_revolution);
  参数意义: target_pulse 为目标带符号电机脉冲。
            current_pulse 为当前带符号电机脉冲。
            pulses_per_revolution 为电机每转脉冲数。
*/
double ArmControlProfile_DistanceDegrees(
    int32_t target_pulse,
    int32_t current_pulse,
    uint32_t pulses_per_revolution)
{
    int64_t target_tenths = ArmControlProfile_PulseToProtocolTenths(
        target_pulse, pulses_per_revolution);
    int64_t current_tenths = ArmControlProfile_PulseToProtocolTenths(
        current_pulse, pulses_per_revolution);
    int64_t delta_tenths = target_tenths - current_tenths;

    if (delta_tenths < 0)
    {
        delta_tenths = -delta_tenths;
    }
    return (double)delta_tenths * 0.1;
}

/*
  函数说明: 按速度、加速度和减速度计算三角形或梯形运动时间，单位秒。
  调用示例: ArmControlProfile_TimeSeconds(distance_deg, speed_rpm, acc_rpm_per_s, dec_rpm_per_s);
  参数意义: distance_deg 为电机轴位移绝对值，度。
            speed_rpm 为电机转速，RPM。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
*/
double ArmControlProfile_TimeSeconds(double distance_deg,
                                           double speed_rpm,
                                           uint16_t acc_rpm_per_s,
                                           uint16_t dec_rpm_per_s)
{
    double speed_deg_s;
    double acc_deg_s2;
    double dec_deg_s2;
    double acc_distance;
    double dec_distance;
    double cruise_distance;

    if (distance_deg <= 0.0)
    {
        return 0.0;
    }

    speed_deg_s = speed_rpm * ARM_CONTROL_DEG_PER_RPM_S;
    acc_deg_s2 = (double)acc_rpm_per_s * ARM_CONTROL_DEG_PER_RPM_S;
    dec_deg_s2 = (double)dec_rpm_per_s * ARM_CONTROL_DEG_PER_RPM_S;
    if ((speed_deg_s <= 0.0) || (acc_deg_s2 <= 0.0) ||
        (dec_deg_s2 <= 0.0))
    {
        return INFINITY;
    }

    acc_distance = (speed_deg_s * speed_deg_s) /
                   (2.0 * acc_deg_s2);
    dec_distance = (speed_deg_s * speed_deg_s) /
                   (2.0 * dec_deg_s2);

    if (distance_deg <= (acc_distance + dec_distance))
    {
        /* 小位移时驱动器会自动使用三角形速度曲线。 */
        double peak_speed_deg_s = sqrt(
            (2.0 * distance_deg) /
            (1.0 / acc_deg_s2 + 1.0 / dec_deg_s2));
        return peak_speed_deg_s / acc_deg_s2 +
               peak_speed_deg_s / dec_deg_s2;
    }

    cruise_distance = distance_deg - acc_distance - dec_distance;
    return speed_deg_s / acc_deg_s2 +
           speed_deg_s / dec_deg_s2 +
           cruise_distance / speed_deg_s;
}

/*
  函数说明: 速度量化后保持加速度，反求减速度的相邻整数编码以缩小到位误差。
  调用示例: ArmControlProfile_MatchDeceleration(distance_deg, speed_rpm, time_s, acc, dec_cap, dec, predicted_s, next_s);
  参数意义: distance_deg 为电机转角绝对值，度；speed_rpm 为量化速度。
            time_s 为共同时间，秒；acc、dec_cap 为加速度及减速度上限，RPM/s。
            dec、predicted_s 为输入候选值，找到更接近共同时间的合法参数时更新。
            next_s 为比共同时间更晚的最近候选时间，供自动模式继续同步搜索。
*/
static void ArmControlProfile_MatchDeceleration(
    double distance_deg, double speed_rpm, double time_s,
    uint16_t acc, uint16_t dec_cap, uint16_t *dec, double *predicted_s, double *next_s)
{
    double inverse_sum;
    double inverse_dec;
    double ideal_dec;
    double candidate_dec;
    double candidate_time;
    uint8_t index;

    /* 根据三角形或梯形时间方程计算 1/A + 1/D；A/D 单位均为 RPM/s。 */
    if (speed_rpm * 6.0 >= 2.0 * distance_deg / time_s)
        inverse_sum = 3.0 * time_s * time_s / distance_deg;
    else
        inverse_sum = 2.0 * (time_s - distance_deg / (speed_rpm * 6.0)) / speed_rpm;
    inverse_dec = inverse_sum - 1.0 / (double)acc;
    if (inverse_dec <= 0.0) return;
    ideal_dec = 1.0 / inverse_dec;
    if (!isfinite(ideal_dec)) return;
    for (index = 0U; index < 2U; ++index)
    {
        candidate_dec = floor(ideal_dec) + index;
        if (candidate_dec < 1.0 || candidate_dec > (double)dec_cap) continue;
        candidate_time = ArmControlProfile_TimeSeconds(distance_deg, speed_rpm,
            acc, (uint16_t)candidate_dec);
        if (candidate_time > time_s && candidate_time < *next_s)
            *next_s = candidate_time;
        if (candidate_time <= time_s + ARM_CONTROL_TIME_EPS_S &&
            (*predicted_s > time_s + ARM_CONTROL_TIME_EPS_S ||
             fabs(candidate_time - time_s) < fabs(*predicted_s - time_s)))
        {
            *dec = (uint16_t)candidate_dec;
            *predicted_s = candidate_time;
        }
    }
}

/*
  函数说明: 为单轴求接近同步时刻的合法速度和加减速度；优先使用配置上限。
  调用示例: ArmControlProfile_CalculateTimed(config, index, distance_deg, time_s, acc_rpm_per_s, dec_rpm_per_s, speed_rpm, predicted_time_s, next_time_s);
  参数意义: config 为对应模块的配置结构。
            index 为数组或电机下标，从 0 开始。
            distance_deg 为电机轴位移绝对值，度。
            time_s 为内部规划或曲线参考时长，秒。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            speed_rpm 为电机转速，RPM。
            predicted_time_s 为输出量化后预计动作时间，秒。
            next_time_s 可为 NULL；本次无法同步时输出较晚的合法候选时间，秒。
*/
ArmControlStatus ArmControlProfile_CalculateTimed(
    const ArmControlConfig *config,
    uint8_t index,
    double distance_deg,
    float time_s,
    uint16_t *acc_rpm_per_s,
    uint16_t *dec_rpm_per_s,
    float *speed_rpm,
    float *predicted_time_s,
    float *next_time_s)
{
    double next_candidate_s = INFINITY;
    double time_limit_s;
    double acc_cap_deg_s2;
    double dec_cap_deg_s2;
    double speed_cap_deg_s;
    double ramp_distance_deg;
    double peak_speed_deg_s;
    double minimum_time_s;
    double candidate_acc_deg_s2;
    double candidate_dec_deg_s2;
    double fastest_candidate_time_s;
    double coefficient;
    double discriminant;
    double ideal_speed_rpm;
    double candidate_speed_rpm;
    uint16_t best_acc;
    uint16_t best_dec;
    double best_rpm;
    double best_error;
    double candidate_time_s;
    double speed_candidates[2];
    uint16_t candidate_acc;
    uint16_t candidate_dec;
    uint16_t matched_dec;
    uint16_t effort_low;
    uint16_t effort_high;
    uint16_t effort_mid;
    uint16_t effort_limit;
    uint32_t effort_value;
    uint32_t correction_step;
    uint8_t speed_index;
    double error_s;

    if (next_time_s != NULL) *next_time_s = INFINITY;
    if ((config == NULL) || (acc_rpm_per_s == NULL) ||
        (dec_rpm_per_s == NULL) || (speed_rpm == NULL) ||
        (predicted_time_s == NULL))
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        return ARM_CONTROL_TIME_INVALID;
    }
    if (distance_deg <= 0.0)
    {
        *acc_rpm_per_s = config->acc_rpm_per_s[index];
        *dec_rpm_per_s = config->dec_rpm_per_s[index];
        *speed_rpm = 0.0f;
        *predicted_time_s = 0.0f;
        return ARM_CONTROL_OK;
    }

    time_limit_s = (double)time_s;
    acc_cap_deg_s2 = (double)config->acc_rpm_per_s[index] *
                     ARM_CONTROL_DEG_PER_RPM_S;
    dec_cap_deg_s2 = (double)config->dec_rpm_per_s[index] *
                     ARM_CONTROL_DEG_PER_RPM_S;
    speed_cap_deg_s = (double)config->vel_rpm[index] *
                      ARM_CONTROL_DEG_PER_RPM_S;
    if ((acc_cap_deg_s2 <= 0.0) || (dec_cap_deg_s2 <= 0.0) ||
        (speed_cap_deg_s <
         ((double)ARM_CONTROL_MIN_SPEED_RPM *
          (double)ARM_CONTROL_DEG_PER_RPM_S)))
    {
        return ARM_CONTROL_TIME_UNACHIEVABLE;
    }

    /* 先使用三个上限计算该电机的最快梯形/三角形轨迹。 */
    ramp_distance_deg =
        (speed_cap_deg_s * speed_cap_deg_s) /
        (2.0 * acc_cap_deg_s2) +
        (speed_cap_deg_s * speed_cap_deg_s) /
        (2.0 * dec_cap_deg_s2);
    if (distance_deg <= ramp_distance_deg)
    {
        peak_speed_deg_s = sqrt(
            (2.0 * distance_deg) /
            (1.0 / acc_cap_deg_s2 + 1.0 / dec_cap_deg_s2));
        minimum_time_s = peak_speed_deg_s / acc_cap_deg_s2 +
                         peak_speed_deg_s / dec_cap_deg_s2;
    }
    else
    {
        peak_speed_deg_s = speed_cap_deg_s;
        minimum_time_s = speed_cap_deg_s / acc_cap_deg_s2 +
                         speed_cap_deg_s / dec_cap_deg_s2 +
                         (distance_deg - ramp_distance_deg) /
                         speed_cap_deg_s;
    }

    if (time_limit_s < (minimum_time_s - ARM_CONTROL_TIME_EPS_S))
    {
        if (next_time_s != NULL) *next_time_s = (float)minimum_time_s;
        return ARM_CONTROL_TIME_UNACHIEVABLE;
    }

    /* 最大加减速度下的稳定二次方程解，避免 T 很大时相减损失精度。 */
    coefficient = 0.5 * (1.0 / acc_cap_deg_s2 + 1.0 / dec_cap_deg_s2);
    discriminant = time_limit_s * time_limit_s - 4.0 * coefficient * distance_deg;
    if (discriminant < 0.0 && discriminant > -1.0e-6) discriminant = 0.0;
    if (discriminant >= 0.0)
    {
        ideal_speed_rpm = (2.0 * distance_deg) /
            (time_limit_s + sqrt(discriminant)) / ARM_CONTROL_DEG_PER_RPM_S;
        best_error = 1.0e30;
        best_rpm = 0.0;
        for (speed_index = 0U; speed_index < 2U; ++speed_index)
        {
            candidate_speed_rpm = (floor(ideal_speed_rpm * 10.0) + speed_index) * 0.1;
            if (candidate_speed_rpm < ARM_CONTROL_MIN_SPEED_RPM ||
                candidate_speed_rpm > config->vel_rpm[index]) continue;
            candidate_time_s = ArmControlProfile_TimeSeconds(distance_deg,
                candidate_speed_rpm, config->acc_rpm_per_s[index], config->dec_rpm_per_s[index]);
            if (candidate_time_s > time_limit_s && candidate_time_s < next_candidate_s)
                next_candidate_s = candidate_time_s;
            error_s = fabs(candidate_time_s - time_limit_s);
            if (error_s < best_error)
            {
                best_error = error_s;
                best_rpm = candidate_speed_rpm;
            }
        }
        if (best_rpm > 0.0 && best_error <= ARM_CONTROL_TIME_TOLERANCE_S)
        {
            *acc_rpm_per_s = config->acc_rpm_per_s[index];
            *dec_rpm_per_s = config->dec_rpm_per_s[index];
            *speed_rpm = (float)best_rpm;
            *predicted_time_s = (float)ArmControlProfile_TimeSeconds(distance_deg,
                best_rpm, *acc_rpm_per_s, *dec_rpm_per_s);
            return ARM_CONTROL_OK;
        }
    }

    /*
     * 把 max(A,D) 定义为加速度等级。对某个等级，A/D 都取不超过
     * 各自上限的相同最小值。最快完成时间随等级单调下降，因此先
     * 用二分法找到满足指定时间的最低等级，再只沿这个一维等级做
     * 少量协议量化修正；速度取整后反求 D 的相邻编码，不遍历 A/D 二维组合。
     */
    effort_low = 1U;
    effort_limit = (config->acc_rpm_per_s[index] >
                    config->dec_rpm_per_s[index]) ?
                   config->acc_rpm_per_s[index] :
                   config->dec_rpm_per_s[index];
    effort_high = effort_limit;
    while (effort_low < effort_high)
    {
        effort_mid = (uint16_t)(
            (uint32_t)effort_low +
            (((uint32_t)effort_high - (uint32_t)effort_low) / 2U));
        candidate_acc = (effort_mid < config->acc_rpm_per_s[index]) ?
                        effort_mid : config->acc_rpm_per_s[index];
        candidate_dec = (effort_mid < config->dec_rpm_per_s[index]) ?
                        effort_mid : config->dec_rpm_per_s[index];
        fastest_candidate_time_s = ArmControlProfile_TimeSeconds(
            distance_deg,
            (double)config->vel_rpm[index],
            candidate_acc,
            candidate_dec);
        if (fastest_candidate_time_s <=
            (time_limit_s + ARM_CONTROL_TIME_EPS_S))
        {
            effort_high = effort_mid;
        }
        else
        {
            effort_low = (uint16_t)(effort_mid + 1U);
        }
    }

    for (correction_step = 0U;
         correction_step <= ARM_CONTROL_ACCEL_CORRECTION_STEPS;
         ++correction_step)
    {
        effort_value = (uint32_t)effort_low + correction_step;
        if (effort_value > (uint32_t)effort_limit)
        {
            break;
        }

        candidate_acc = (effort_value <
                         (uint32_t)config->acc_rpm_per_s[index]) ?
                        (uint16_t)effort_value :
                        config->acc_rpm_per_s[index];
        candidate_dec = (effort_value <
                         (uint32_t)config->dec_rpm_per_s[index]) ?
                        (uint16_t)effort_value :
                        config->dec_rpm_per_s[index];
        candidate_acc_deg_s2 = (double)candidate_acc *
                               ARM_CONTROL_DEG_PER_RPM_S;
        candidate_dec_deg_s2 = (double)candidate_dec *
                               ARM_CONTROL_DEG_PER_RPM_S;
        coefficient = 0.5 *
            (1.0 / candidate_acc_deg_s2 +
             1.0 / candidate_dec_deg_s2);
        discriminant = (time_limit_s * time_limit_s) -
                       (4.0 * coefficient * distance_deg);
        if (discriminant < 0.0)
        {
            continue;
        }

        ideal_speed_rpm =
            ((time_limit_s - sqrt(discriminant)) /
             (2.0 * coefficient)) /
            ARM_CONTROL_DEG_PER_RPM_S;
        speed_candidates[0] = floor(
            ideal_speed_rpm / ARM_CONTROL_SPEED_STEP_RPM) *
            ARM_CONTROL_SPEED_STEP_RPM;
        speed_candidates[1] = ceil(
            ideal_speed_rpm / ARM_CONTROL_SPEED_STEP_RPM) *
            ARM_CONTROL_SPEED_STEP_RPM;

        best_acc = 0U;
        best_dec = 0U;
        best_rpm = 0.0;
        best_error = INFINITY;
        for (speed_index = 0U; speed_index < 2U; ++speed_index)
        {
            candidate_speed_rpm = speed_candidates[speed_index];
            if (candidate_speed_rpm < ARM_CONTROL_MIN_SPEED_RPM)
            {
                candidate_speed_rpm = ARM_CONTROL_MIN_SPEED_RPM;
            }
            if (candidate_speed_rpm >
                (double)config->vel_rpm[index])
            {
                continue;
            }

            candidate_time_s = ArmControlProfile_TimeSeconds(
                distance_deg,
                candidate_speed_rpm,
                candidate_acc,
                candidate_dec);
            if (candidate_time_s > time_limit_s && candidate_time_s < next_candidate_s)
                next_candidate_s = candidate_time_s;
            matched_dec = candidate_dec;
            ArmControlProfile_MatchDeceleration(distance_deg, candidate_speed_rpm,
                time_limit_s, candidate_acc, config->dec_rpm_per_s[index],
                &matched_dec, &candidate_time_s, &next_candidate_s);
            if (candidate_time_s >
                (time_limit_s + ARM_CONTROL_TIME_EPS_S))
            {
                continue;
            }

            error_s = fabs(time_limit_s - candidate_time_s);
            if (error_s < best_error)
            {
                best_error = error_s;
                best_acc = candidate_acc;
                best_dec = matched_dec;
                best_rpm = candidate_speed_rpm;
                *predicted_time_s = (float)candidate_time_s;
            }
        }

        /* 找到满足时间误差的合法 A/D 与速度组合即可结束。 */
        if ((best_acc != 0U) && (best_dec != 0U) &&
            (best_rpm > 0.0) &&
            (best_error <= ARM_CONTROL_TIME_TOLERANCE_S))
        {
            *acc_rpm_per_s = best_acc;
            *dec_rpm_per_s = best_dec;
            *speed_rpm = (float)best_rpm;
            return ARM_CONTROL_OK;
        }
    }

    if (next_time_s != NULL) *next_time_s = (float)next_candidate_s;
    return ARM_CONTROL_TIME_UNACHIEVABLE;
}


#define ARM_CONTROL_MOTOR_COUNT     (3U)
#define ARM_CONTROL_SYNC_ADDRESS    (0U)
#define ARM_CONTROL_ZERO_EPS_DEG    (1.0e-3f)
#define ARM_CONTROL_REACHED_TIMEOUT_MARGIN_MS (1000U)
#define ARM_CONTROL_TIMEOUT_RECOVERY_MS (1500U)
#define ARM_CONTROL_TIMEOUT_RECOVERY_POLL_MS (100U)
#define ARM_CONTROL_TIMEOUT_RECOVERY_STABLE_MS (50U)
#define ARM_CONTROL_DIAG_STATUS_PERIOD_MS (100U)
#define ARM_CONTROL_DIAG_POSITION_PERIOD_MS (200U)
#define ARM_CONTROL_DIAG_ERROR_PERIOD_MS (200U)
#define ARM_CONTROL_HEARTBEAT_READ_PERIOD_MS (1000U)
#define ARM_CONTROL_RX_DRAIN_LIMIT (16U)
#define ARM_CONTROL_DRIVER_FAULT_MASK (0x0CU)
#define ARM_CONTROL_MAX_OS_DELAY_TICKS (0xFFFFFFFEUL)
#define ARM_CONTROL_ALL_MOTORS_MASK (0x07U)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_PERIOD_MS (20U)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_FEEDBACK_TIMEOUT_MS (100U)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_DEADLINE_TOLERANCE_MS (5U)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_HEARTBEAT_MS (200U)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_LIMIT_SCALE (0.85f)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_SETTLE_TIME_S (0.30f)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_MIN_RADIAL_MM (20.0f)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_KP (0.5f)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_CORRECTION_RPM (10.0f)
#define ARM_CONTROL_TRAJECTORY_DEFAULT_ERROR_DEG (30.0f)
#define ARM_CONTROL_TRAJECTORY_START_MATCH_DEG (1.0f)
#define ARM_CONTROL_TRAJECTORY_MAX_SEGMENTS (10000UL)
#define ARM_CONTROL_TRAJECTORY_PRECHECK_INTERVALS (200U)
#define ARM_CONTROL_TRAJECTORY_DIRECTION_EPS_DEG_PER_U (0.1f)
#define ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM (1.0e-4f)
#define ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM (0.5f)
#define ARM_CONTROL_TRAJECTORY_POSITION_PERIOD_MS (20U)
#define ARM_CONTROL_TRAJECTORY_SPEED_PERIOD_MS (40U)
#define ARM_CONTROL_TRAJECTORY_STATUS_PERIOD_MS (100U)
#define ARM_CONTROL_TRAJECTORY_ERROR_PERIOD_MS (100U)
#define ARM_CONTROL_TIMED_STOP_ACKS_PER_MOTOR (4U)
#define ARM_CONTROL_TIMED_STOP_ACK_TIMEOUT_MS (100U)
#define ARM_CONTROL_TRAJECTORY_SETTLE_MARGIN_MS (1000U)
#define ARM_CONTROL_MINIMUM_VERIFY_ITERATIONS (3U)
#define ARM_CONTROL_MINIMUM_VERIFY_MARGIN (1.02)

/* 根据当前机构强度先采用的三轴安全上限，不是驱动器手册的最大值。 */
static const uint16_t arm_control_default_acc[ARM_CONTROL_MOTOR_COUNT] =
    {600U, 300U, 400U};
static const uint16_t arm_control_default_dec[ARM_CONTROL_MOTOR_COUNT] =
    {700U, 400U, 500U};
static const float arm_control_default_speed[ARM_CONTROL_MOTOR_COUNT] =
    {2500.0f, 1800.0f, 2500.0f};

static ArmControlConfig *arm_control_active_config;
static ArmControlResult arm_control_last_result;
static ArmIkStatus arm_control_last_ik_status = ARM_IK_OK;
static int32_t arm_control_current_pulse[ARM_CONTROL_MOTOR_COUNT];
static bool arm_control_current_valid;

typedef struct
{
    bool active;
    uint8_t pending_mask;
    uint8_t reached_mask;
    uint8_t error_mask;
    uint8_t last_error_code;
    int32_t target_pulse[ARM_CONTROL_MOTOR_COUNT];
} ArmControlMoveTracker;

typedef struct
{
    ArmCubicBezierTrajectory trajectory;
    ArmControlTrajectoryResult result;
    ArmJointAngles final_joints;
    ArmMotorCoordinates final_coordinates;
    ArmMotorCoordinates reference_coordinates;
    float previous_command_rpm[ARM_CONTROL_MOTOR_COUNT];
    int8_t planned_direction[ARM_CONTROL_MOTOR_COUNT];
    uint32_t start_ms;
    uint32_t feedback_request_ms;
    uint32_t next_update_ms;
    uint32_t feedback_deadline_ms;
    uint32_t settle_deadline_ms;
    uint32_t stop_command_ms;
    uint32_t stop_deadline_ms;
    uint32_t stop_position_barrier_ms;
    uint32_t heartbeat_request_ms;
    uint32_t rx_drop_baseline;
    uint32_t can_error_baseline;
    uint32_t generation;
    uint32_t segment_index;
    uint8_t driver_error_mask;
    uint8_t heartbeat_ack_mask;
    uint8_t timed_stop_ack_count[ARM_CONTROL_MOTOR_COUNT];
    uint32_t timed_stop_request_ms;
    bool feedback_stream_enabled;
    bool timed_stop_waiting;
    bool trajectory_preflight_ready;
    bool heartbeat_enabled;
    bool heartbeat_disable_pending;
    bool stop_speed_confirmed;
} ArmControlTrajectoryContext;

static ArmControlMoveTracker arm_control_move_tracker;
ArmControlDriverStatus arm_control_driver_status[
    ARM_CONTROL_MOTOR_COUNT];
static uint32_t arm_control_diag_next_status_ms;
static uint32_t arm_control_diag_next_position_ms;
static uint32_t arm_control_diag_next_error_ms;
static ArmControlTrajectoryContext arm_control_trajectory_context;
static volatile uint32_t arm_control_trajectory_generation;
static volatile uint32_t arm_control_stop_depth;
static osMutexId_t arm_control_command_mutex;
static osMutexId_t arm_control_feedback_config_mutex;
static osMutexId_t arm_control_trajectory_api_mutex;
volatile uint32_t arm_control_idle_count;
volatile uint32_t arm_control_idle_last_ms;
volatile uint32_t arm_control_diagnostic_tx_errors;
static uint32_t arm_control_heartbeat_read_ms;

/*
  函数说明: 尝试关闭曲线的周期回传和心跳配置，并保留清理错误。
  调用示例: ArmControl_TrajectoryBestEffortCleanup(config);
  参数意义: config 为对应模块的配置结构。
*/
static ArmControlStatus ArmControl_TrajectoryBestEffortCleanup(
    const ArmControlConfig *config);
/*
  函数说明: 判断曲线状态是否属于预检、运行或停止收尾阶段。
  调用示例: ArmControl_TrajectoryStateIsBusy(state);
  参数意义: state 为状态值或状态快照。
*/
static bool ArmControl_TrajectoryStateIsBusy(
    ArmControlTrajectoryState state);
/*
  函数说明: 在调用者已持有配置锁时设置三台驱动器的周期回传。
  调用示例: ArmControl_SetTimedReturnInfoUnlocked(config, info_function, period_ms);
  参数意义: config 为对应模块的配置结构。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
static ArmControlStatus ArmControl_SetTimedReturnInfoUnlocked(
    const ArmControlConfig *config,
    uint8_t info_function,
    uint16_t period_ms);

/*
  函数说明: 使用带符号差值判断毫秒截止时间是否到达，支持计时回卷。
  调用示例: ArmControl_TimeReached(now_ms, due_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
            due_ms 为计划到达时刻，毫秒。
*/
static bool ArmControl_TimeReached(uint32_t now_ms, uint32_t due_ms)
{
    return ((int32_t)(now_ms - due_ms) >= 0);
}

/*
  函数说明: 判断一个毫秒时间戳是否严格晚于指定屏障时刻。
  调用示例: ArmControl_TimeAfter(timestamp_ms, barrier_ms);
  参数意义: timestamp_ms 为待比较的毫秒时间戳。
            barrier_ms 为时间屏障，毫秒。
*/
static bool ArmControl_TimeAfter(uint32_t timestamp_ms, uint32_t barrier_ms)
{
    return ((int32_t)(timestamp_ms - barrier_ms) > 0);
}

/*
  函数说明: 尝试取得电机命令互斥标志；忙时立即返回 false。
  调用示例: ArmControl_CommandLock();
  参数意义: 无。
*/
static bool ArmControl_CommandLock(void)
{
    if (arm_control_command_mutex == NULL)
    {
        arm_control_command_mutex = osMutexNew(NULL);
        if (arm_control_command_mutex == NULL)
        {
            return false;
        }
    }
    return (osMutexAcquire(arm_control_command_mutex, osWaitForever) == osOK);
}

/*
  函数说明: 释放电机命令互斥标志。
  调用示例: ArmControl_CommandUnlock();
  参数意义: 无。
*/
static void ArmControl_CommandUnlock(void)
{
    if (arm_control_command_mutex != NULL)
    {
        (void)osMutexRelease(arm_control_command_mutex);
    }
}

/*
  函数说明: 尝试取得驱动器反馈配置互斥标志，防止周期回传配置交错。
  调用示例: ArmControl_FeedbackConfigLock();
  参数意义: 无。
*/
static bool ArmControl_FeedbackConfigLock(void)
{
    if (arm_control_feedback_config_mutex == NULL)
    {
        arm_control_feedback_config_mutex = osMutexNew(NULL);
        if (arm_control_feedback_config_mutex == NULL)
        {
            return false;
        }
    }
    return (osMutexAcquire(arm_control_feedback_config_mutex,
                           osWaitForever) == osOK);
}

/*
  函数说明: 释放驱动器反馈配置互斥标志。
  调用示例: ArmControl_FeedbackConfigUnlock();
  参数意义: 无。
*/
static void ArmControl_FeedbackConfigUnlock(void)
{
    if (arm_control_feedback_config_mutex != NULL)
    {
        (void)osMutexRelease(arm_control_feedback_config_mutex);
    }
}

/*
  函数说明: 尝试取得机械臂运动 API 互斥标志，防止并发运动请求覆盖状态。
  调用示例: ArmControl_TrajectoryApiLock();
  参数意义: 无。
*/
static bool ArmControl_TrajectoryApiLock(void)
{
    if (arm_control_trajectory_api_mutex == NULL)
    {
        arm_control_trajectory_api_mutex = osMutexNew(NULL);
        if (arm_control_trajectory_api_mutex == NULL)
        {
            return false;
        }
    }
    /* 控制入口不排队：忙时立即返回，避免 Stop 后执行先前排队的旧请求。 */
    return (osMutexAcquire(arm_control_trajectory_api_mutex, 0U) == osOK);
}

/*
  函数说明: 释放机械臂运动 API 互斥标志。
  调用示例: ArmControl_TrajectoryApiUnlock();
  参数意义: 无。
*/
static void ArmControl_TrajectoryApiUnlock(void)
{
    if (arm_control_trajectory_api_mutex != NULL)
    {
        (void)osMutexRelease(arm_control_trajectory_api_mutex);
    }
}

/*
  函数说明: 读取当前停止代次，用于识别已取消的旧运动请求。
  调用示例: ArmControl_ReadGeneration();
  参数意义: 无。
*/
static uint32_t ArmControl_ReadGeneration(void)
{
    uint32_t generation;

    taskENTER_CRITICAL();
    generation = arm_control_trajectory_generation;
    taskEXIT_CRITICAL();
    return generation;
}

/*
  函数说明: 在没有停止操作进行时取得当前命令代次；成功返回 true。
  调用示例: ArmControl_TryCaptureCommandToken(generation);
  参数意义: generation 为停止操作代次。
*/
static bool ArmControl_TryCaptureCommandToken(uint32_t *generation)
{
    bool captured = false;

    if (generation == NULL)
    {
        return false;
    }
    taskENTER_CRITICAL();
    if (arm_control_stop_depth == 0U)
    {
        *generation = arm_control_trajectory_generation;
        captured = true;
    }
    taskEXIT_CRITICAL();
    return captured;
}

/*
  函数说明: 检查请求代次是否仍有效，防止停止后旧流程继续发令。
  调用示例: ArmControl_CommandTokenIsCurrent(generation);
  参数意义: generation 为停止操作代次。
*/
static bool ArmControl_CommandTokenIsCurrent(uint32_t generation)
{
    bool current;

    taskENTER_CRITICAL();
    current = (arm_control_stop_depth == 0U) &&
              (arm_control_trajectory_generation == generation);
    taskEXIT_CRITICAL();
    return current;
}

/*
  函数说明: 在代次仍有效时原子发布曲线执行状态和结果。
  调用示例: ArmControl_TryPublishTrajectoryState(generation, state, status);
  参数意义: generation 为停止操作代次。
            state 为状态值或状态快照。
            status 为状态码或驱动器诊断快照。
*/
static bool ArmControl_TryPublishTrajectoryState(
    uint32_t generation,
    ArmControlTrajectoryState state,
    ArmControlStatus status)
{
    bool published = false;

    taskENTER_CRITICAL();
    if ((arm_control_stop_depth == 0U) &&
        (arm_control_trajectory_generation == generation))
    {
        arm_control_trajectory_context.result.state = state;
        arm_control_trajectory_context.result.status = status;
        published = true;
    }
    taskEXIT_CRITICAL();
    return published;
}

/*
  函数说明: 退出停止保护区，允许后续新命令获取代次。
  调用示例: ArmControl_StopGateExit();
  参数意义: 无。
*/
static void ArmControl_StopGateExit(void)
{
    taskENTER_CRITICAL();
    if (arm_control_stop_depth > 0U)
    {
        --arm_control_stop_depth;
    }
    taskEXIT_CRITICAL();
}

/*
  函数说明: 按 CAN 地址查找三轴配置下标；不属于机械臂时返回 -1。
  调用示例: ArmControl_FindMotorIndex(config, address);
  参数意义: config 为对应模块的配置结构。
            address 为待匹配的电机 CAN 地址。
*/
static int8_t ArmControl_FindMotorIndex(const ArmControlConfig *config,
                                       uint8_t address)
{
    uint8_t index;

    if (config == NULL)
    {
        return -1;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (config->motor_addr[index] == address)
        {
            return (int8_t)index;
        }
    }
    return -1;
}

/*
  函数说明: 清空待到位目标和驱动器到位位图。
  调用示例: ArmControl_ClearMoveTracker();
  参数意义: 无。
*/
static void ArmControl_ClearMoveTracker(void)
{
    taskENTER_CRITICAL();
    memset(&arm_control_move_tracker, 0, sizeof(arm_control_move_tracker));
    taskEXIT_CRITICAL();
}

/*
  函数说明: 在真实到位反馈齐全后提交目标脉冲，作为下一次动作起点。
  调用示例: ArmControl_CommitPendingMove();
  参数意义: 无。
*/
static void ArmControl_CommitPendingMove(void)
{
    uint8_t index;

    taskENTER_CRITICAL();
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        arm_control_current_pulse[index] =
            arm_control_move_tracker.target_pulse[index];
    }
    arm_control_current_valid = true;
    memset(&arm_control_move_tracker, 0, sizeof(arm_control_move_tracker));
    taskEXIT_CRITICAL();
}

/*
  函数说明: token 校验、9F 完成判定和 current_pulse 提交必须处于同一个临界区，
  否则 StopAll 清 tracker 的瞬间可能被误判成 pending==reached==0。
  调用示例: ArmControl_CheckMoveCompletion(request_generation, publish_trajectory_done);
  参数意义: request_generation 为本次运动请求的停止代次。
            publish_trajectory_done 为是否同时发布曲线完成状态。
*/
static ArmControlStatus ArmControl_CheckMoveCompletion(
    uint32_t request_generation,
    bool publish_trajectory_done)
{
    ArmControlStatus status = ARM_CONTROL_BUSY;

    taskENTER_CRITICAL();
    if ((arm_control_stop_depth != 0U) ||
        (arm_control_trajectory_generation != request_generation))
    {
        status = ARM_CONTROL_CANCELLED;
    }
    else if (arm_control_move_tracker.error_mask != 0U)
    {
        status = ARM_CONTROL_DRIVER_ERROR;
    }
    else if (!arm_control_move_tracker.active ||
             ((arm_control_move_tracker.reached_mask &
               arm_control_move_tracker.pending_mask) ==
              arm_control_move_tracker.pending_mask))
    {
        if (arm_control_move_tracker.active)
        {
            ArmControl_CommitPendingMove();
        }
        if (publish_trajectory_done)
        {
            arm_control_trajectory_context.result.state =
                ARM_CONTROL_TRAJECTORY_DONE;
            arm_control_trajectory_context.result.status = ARM_CONTROL_OK;
        }
        status = ARM_CONTROL_OK;
    }
    taskEXIT_CRITICAL();
    return status;
}

/*
  函数说明: 从四个高字节在前的字节读取无符号 32 位数。
  调用示例: ArmControl_ReadU32BE(data);
  参数意义: data 为数据缓冲区或传感器快照。
*/
static uint32_t ArmControl_ReadU32BE(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           ((uint32_t)data[3] << 0);
}

/*
  函数说明: 解析机械臂驱动器应答，更新到位标志和位置、速度、故障诊断。
  调用示例: ArmControl_ProcessRxFrame(config, frame);
  参数意义: config 为对应模块的配置结构。
            frame 为CAN 接收帧结构。
*/
static void ArmControl_ProcessRxFrame(const ArmControlConfig *config,
                                     const ZDT_CanRxFrame_t *frame)
{
    uint8_t address;
    uint8_t function_code;
    uint8_t response_code;
    uint8_t status_flags;
    uint32_t raw_value;
    int8_t motor_index;
    float value;
    uint32_t now_ms;

    if ((config == NULL) || (frame == NULL) || (frame->dlc < 3U))
    {
        return;
    }

    if (frame->data[frame->dlc - 1U] != 0x6BU)
    {
        return;
    }

    address = (uint8_t)((frame->id >> 8) & 0xFFU);
    motor_index = ArmControl_FindMotorIndex(config, address);
    if (motor_index < 0)
    {
        return;
    }

    function_code = frame->data[0];
    response_code = frame->data[1];
    /* 使用 CAN ISR 真正收到该帧的时刻，不用任务延后解析时刻。 */
    now_ms = frame->timestamp_ms;

    arm_control_driver_status[(uint8_t)motor_index].last_update_ms = now_ms;

    if (((function_code == 0xFDU) || (function_code == 0xF6U) ||
         (function_code == 0x68U) || (function_code == 0x11U)) &&
        (response_code == 0x02U))
    {
        arm_control_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        if ((function_code == 0x68U) &&
            (arm_control_trajectory_context.result.state !=
             ARM_CONTROL_TRAJECTORY_IDLE) &&
            ArmControl_TimeReached(
                now_ms,
                arm_control_trajectory_context.heartbeat_request_ms))
        {
            arm_control_trajectory_context.heartbeat_ack_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        if ((function_code == 0x11U) &&
            arm_control_trajectory_context.timed_stop_waiting &&
            ArmControl_TimeAfter(
                now_ms,
                arm_control_trajectory_context.timed_stop_request_ms) &&
            (arm_control_trajectory_context.
             timed_stop_ack_count[(uint8_t)motor_index] <
             ARM_CONTROL_TIMED_STOP_ACKS_PER_MOTOR))
        {
            ++arm_control_trajectory_context.
                timed_stop_ack_count[(uint8_t)motor_index];
        }
        return;
    }

    /* FD 9F 是位置模式动作执行完成帧，只作为正常完成判据。 */
    if ((function_code == 0xFDU) && (response_code == 0x9FU))
    {
        arm_control_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        arm_control_driver_status[(uint8_t)motor_index].position_reached_flag =
            true;
        taskENTER_CRITICAL();
        if (arm_control_move_tracker.active)
        {
            arm_control_move_tracker.reached_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        taskEXIT_CRITICAL();
        return;
    }

    /*
     * FD/F6 的 12、22、E2、EE 都表示本次命令没有正常执行：
     * 零位/限位输入已触发、参数/保护错误或命令格式错误。
     */
    if ((((function_code == 0xFDU) || (function_code == 0xF6U)) &&
         ((response_code == 0x12U) || (response_code == 0x22U) ||
          (response_code == 0xE2U) || (response_code == 0xEEU))) ||
        ((function_code == 0x68U) &&
         ((response_code == 0xE2U) || (response_code == 0xEEU))))
    {
        arm_control_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        arm_control_driver_status[(uint8_t)motor_index].last_error_code =
            response_code;
        taskENTER_CRITICAL();
        if (arm_control_move_tracker.active && (function_code == 0xFDU))
        {
            arm_control_move_tracker.error_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
            arm_control_move_tracker.last_error_code = response_code;
        }
        taskEXIT_CRITICAL();
        if ((arm_control_trajectory_context.result.state !=
             ARM_CONTROL_TRAJECTORY_IDLE) &&
            (arm_control_trajectory_context.result.state !=
             ARM_CONTROL_TRAJECTORY_DONE) &&
            (arm_control_trajectory_context.result.state !=
             ARM_CONTROL_TRAJECTORY_FAULT))
        {
            arm_control_trajectory_context.driver_error_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        return;
    }

    /* CAN 数据不带地址：16 + 4 字节大端毫秒值 + 6B，共 6 字节。 */
    if ((function_code == 0x16U) && (frame->dlc == 6U))
    {
        arm_control_driver_status[(uint8_t)motor_index].heartbeat_timeout_ms =
            ArmControl_ReadU32BE(&frame->data[1]);
        arm_control_driver_status[(uint8_t)motor_index].heartbeat_update_ms = now_ms;
        arm_control_driver_status[(uint8_t)motor_index].heartbeat_valid = true;
        return;
    }

    /* 35：X 固件实时速度，单位 0.1 RPM，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_REALTIME_SPEED) &&
        (frame->dlc >= 5U))
    {
        raw_value = ((uint32_t)frame->data[2] << 8) |
                    (uint32_t)frame->data[3];
        value = (float)raw_value * 0.1f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_control_driver_status[(uint8_t)motor_index].realtime_speed_rpm =
            value;
        arm_control_driver_status[(uint8_t)motor_index].realtime_speed_valid =
            true;
        arm_control_driver_status[(uint8_t)motor_index].
            realtime_speed_update_ms = now_ms;
        return;
    }

    /* 36：X 固件实时位置，单位 0.1 度，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_REALTIME_POSITION) &&
        (frame->dlc >= 7U))
    {
        raw_value = ArmControl_ReadU32BE(&frame->data[2]);
        value = (float)raw_value * 0.1f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_control_driver_status[(uint8_t)motor_index].realtime_position_deg =
            value;
        arm_control_driver_status[(uint8_t)motor_index].realtime_position_valid =
            true;
        arm_control_driver_status[(uint8_t)motor_index].
            realtime_position_update_ms = now_ms;
        return;
    }

    /* 37：X 固件位置误差，单位 0.01 度，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_POSITION_ERROR) &&
        (frame->dlc >= 7U))
    {
        raw_value = ArmControl_ReadU32BE(&frame->data[2]);
        value = (float)raw_value * 0.01f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_control_driver_status[(uint8_t)motor_index].position_error_deg =
            value;
        arm_control_driver_status[(uint8_t)motor_index].position_error_valid =
            true;
        arm_control_driver_status[(uint8_t)motor_index].
            position_error_update_ms = now_ms;
        return;
    }

    /*
     * 3A：bit0 为使能，bit2/3 为堵转相关标志；
     * bit4/5 仅记录左右限位输入，不在未标定时自动停机。
     */
    if ((function_code == X_V2_INFO_STATUS) && (frame->dlc >= 3U))
    {
        status_flags = response_code;
        arm_control_driver_status[(uint8_t)motor_index].status_flags =
            status_flags;
        arm_control_driver_status[(uint8_t)motor_index].status_valid = true;
        arm_control_driver_status[(uint8_t)motor_index].status_update_ms = now_ms;
        arm_control_driver_status[(uint8_t)motor_index].position_reached_flag =
            ((status_flags & 0x02U) != 0U);

        if (((status_flags & ARM_CONTROL_DRIVER_FAULT_MASK) != 0U) ||
            (((status_flags & 0x01U) == 0U) &&
             (arm_control_move_tracker.active ||
              ((arm_control_trajectory_context.result.state !=
                ARM_CONTROL_TRAJECTORY_IDLE) &&
               (arm_control_trajectory_context.result.state !=
                ARM_CONTROL_TRAJECTORY_DONE) &&
               (arm_control_trajectory_context.result.state !=
                ARM_CONTROL_TRAJECTORY_FAULT)))))
        {
            if (arm_control_move_tracker.active)
            {
                arm_control_move_tracker.error_mask |=
                    (uint8_t)(1U << (uint8_t)motor_index);
                arm_control_move_tracker.last_error_code = 0x3AU;
            }
            if ((arm_control_trajectory_context.result.state !=
                 ARM_CONTROL_TRAJECTORY_IDLE) &&
                (arm_control_trajectory_context.result.state !=
                 ARM_CONTROL_TRAJECTORY_DONE) &&
                (arm_control_trajectory_context.result.state !=
                 ARM_CONTROL_TRAJECTORY_FAULT))
            {
                arm_control_trajectory_context.driver_error_mask |=
                    (uint8_t)(1U << (uint8_t)motor_index);
            }
        }
    }
}

/*
  函数说明: 限量取出 CAN 接收队列中的帧并交给机械臂解析器。
  调用示例: ArmControl_DrainRx(config, first_timeout_ticks);
  参数意义: config 为对应模块的配置结构。
            first_timeout_ticks 为首次取帧等待时间，RTOS tick。
*/
static void ArmControl_DrainRx(const ArmControlConfig *config,
                              uint32_t first_timeout_ticks)
{
    ZDT_CanRxFrame_t frame;
    uint32_t drain_count;

    for (drain_count = 0U;
         drain_count < ARM_CONTROL_RX_DRAIN_LIMIT;
         ++drain_count)
    {
        if (ZDT_CAN_GetRx(&frame,
                          (drain_count == 0U) ? first_timeout_ticks : 0U) !=
            osOK)
        {
            break;
        }
        ArmControl_ProcessRxFrame(config, &frame);
    }
}

/*
  函数说明: 设置低频诊断查询的起始调度时间。
  调用示例: ArmControl_BeginDiagnostics(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static void ArmControl_BeginDiagnostics(uint32_t now_ms)
{
    arm_control_diag_next_status_ms = now_ms;
    arm_control_diag_next_position_ms = now_ms;
    arm_control_diag_next_error_ms = now_ms;
}

/*
  函数说明: 按周期查询驱动器状态、位置和误差，避免每周期重复查询。
  调用示例: ArmControl_ServiceDiagnostics(config, now_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
*/
static void ArmControl_ServiceDiagnostics(const ArmControlConfig *config,
                                         uint32_t now_ms)
{
    uint8_t index;

    if (config == NULL)
    {
        return;
    }
    if (ArmControl_TimeReached(now_ms, arm_control_diag_next_status_ms))
    {
        for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
        {
            if (X_V2_Read_Info(config->motor_addr[index],
                               X_V2_INFO_STATUS) != HAL_OK)
                ++arm_control_diagnostic_tx_errors;
        }
        arm_control_diag_next_status_ms =
            now_ms + ARM_CONTROL_DIAG_STATUS_PERIOD_MS;
    }

    if (ArmControl_TimeReached(now_ms, arm_control_diag_next_position_ms))
    {
        for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
        {
            if (X_V2_Read_Info(config->motor_addr[index],
                               X_V2_INFO_REALTIME_POSITION) != HAL_OK)
                ++arm_control_diagnostic_tx_errors;
        }
        arm_control_diag_next_position_ms =
            now_ms + ARM_CONTROL_DIAG_POSITION_PERIOD_MS;
    }

    if (ArmControl_TimeReached(now_ms, arm_control_diag_next_error_ms))
    {
        for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
        {
            if (X_V2_Read_Info(config->motor_addr[index],
                               X_V2_INFO_POSITION_ERROR) != HAL_OK)
                ++arm_control_diagnostic_tx_errors;
        }
        arm_control_diag_next_error_ms =
            now_ms + ARM_CONTROL_DIAG_ERROR_PERIOD_MS;
    }
}

/*
  函数说明: 补充无主机指令期间的三轴通信，反馈仍由运动所属任务统一消费。
  调用示例: if (ArmControl_IsReady()) ArmControl_ServiceIdle();
  参数意义: 无；只查询状态/位置/误差/心跳，最多读取 16 帧，不改变运动起点。
            API 锁和命令锁均尝试获取，运动/停止持锁时留到下一轮维护。
*/
void ArmControl_ServiceIdle(void)
{
    uint32_t now_ms;
    uint8_t index;
    if (!ArmControl_IsReady() || !ArmControl_TrajectoryApiLock()) return;
    if (arm_control_command_mutex == NULL ||
        osMutexAcquire(arm_control_command_mutex, 0U) != osOK)
    {
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    if (ArmControl_IsReady() && arm_control_active_config != NULL &&
        arm_control_stop_depth == 0U && !arm_control_move_tracker.active &&
        !ArmControl_TrajectoryIsBusy())
    {
        now_ms = HAL_GetTick();
        ArmControl_DrainRx(arm_control_active_config, 0U);
        ArmControl_ServiceDiagnostics(arm_control_active_config, now_ms);
        /* 首次维护即读心跳；后续每秒更新，缺回复时 valid 仍为 false。 */
        if (arm_control_idle_count == 0U ||
            (uint32_t)(now_ms - arm_control_heartbeat_read_ms) >= ARM_CONTROL_HEARTBEAT_READ_PERIOD_MS)
        {
            arm_control_heartbeat_read_ms = now_ms;
            for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
            {
                if (X_V2_Read_Heartbeat(arm_control_active_config->motor_addr[index]) != HAL_OK)
                    ++arm_control_diagnostic_tx_errors;
            }
        }
        arm_control_idle_last_ms = HAL_GetTick();
        ++arm_control_idle_count;
    }
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
}

/*
  函数说明: 等待所有参与运动轴的 FD 9F 反馈；取消、故障或超时立即返回错误。
  调用示例: ArmControl_WaitForCompletion(config, requested_time_s, profile, request_generation);
  参数意义: config 为对应模块的配置结构。
            requested_time_s 为用于到位等待的预计运动时长，秒。
            profile 为各轴运动的速度、加减速度和预测时长。
            request_generation 为本次运动请求的停止代次。
*/
static ArmControlStatus ArmControl_WaitForCompletion(
    const ArmControlConfig *config,
    float requested_time_s,
    const ArmControlProfile *profile,
    uint32_t request_generation)
{
    uint32_t now_ms;
    uint32_t deadline_ms;
    uint32_t timeout_ms;
    uint32_t timeout_ticks;
    uint32_t drain_count;
    double timeout_s;
    double timeout_ms_double;
    ArmControlStatus completion_status;
    ArmControlStatus stop_status;

    if ((config == NULL) || (profile == NULL))
    {
        return ARM_CONTROL_NULL_POINTER;
    }

    completion_status = ArmControl_CheckMoveCompletion(
        request_generation, false);
    if ((completion_status == ARM_CONTROL_OK) ||
        (completion_status == ARM_CONTROL_CANCELLED))
    {
        return completion_status;
    }
    if (completion_status == ARM_CONTROL_DRIVER_ERROR)
    {
        stop_status = ArmControl_StopAll(config);
        return (stop_status == ARM_CONTROL_OK) ?
               ARM_CONTROL_DRIVER_ERROR : ARM_CONTROL_CAN_ERROR;
    }

    timeout_s = (double)requested_time_s;
    for (drain_count = 0U; drain_count < ARM_CONTROL_MOTOR_COUNT;
         ++drain_count)
    {
        if ((double)profile->predicted_time_s[drain_count] > timeout_s)
        {
            timeout_s = (double)profile->predicted_time_s[drain_count];
        }
    }
    timeout_ms_double = ceil(timeout_s * 1000.0) +
                        (double)ARM_CONTROL_REACHED_TIMEOUT_MARGIN_MS;
    if ((timeout_ms_double <= 0.0) ||
        (timeout_ms_double > (double)ARM_CONTROL_MAX_OS_DELAY_TICKS))
    {
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        return ARM_CONTROL_TIME_INVALID;
    }

    timeout_ms = (uint32_t)timeout_ms_double;
    now_ms = HAL_GetTick();
    deadline_ms = now_ms + timeout_ms;
    ArmControl_BeginDiagnostics(now_ms);

    for (;;)
    {
        now_ms = HAL_GetTick();

        completion_status = ArmControl_CheckMoveCompletion(
            request_generation, false);
        if ((completion_status == ARM_CONTROL_OK) ||
            (completion_status == ARM_CONTROL_CANCELLED))
        {
            return completion_status;
        }
        if (completion_status == ARM_CONTROL_DRIVER_ERROR)
        {
            stop_status = ArmControl_StopAll(config);
            if (stop_status != ARM_CONTROL_OK)
            {
                return ARM_CONTROL_CAN_ERROR;
            }
            return ARM_CONTROL_DRIVER_ERROR;
        }

        if ((int32_t)(now_ms - deadline_ms) >= 0)
        {
            stop_status = ArmControl_StopAll(config);
            if (stop_status != ARM_CONTROL_OK)
            {
                return ARM_CONTROL_CAN_ERROR;
            }
            return ARM_CONTROL_REACHED_TIMEOUT;
        }

        ArmControl_ServiceDiagnostics(config, now_ms);

        timeout_ticks = 10U;
        ArmControl_DrainRx(config, timeout_ticks);

        completion_status = ArmControl_CheckMoveCompletion(
            request_generation, false);
        if ((completion_status == ARM_CONTROL_OK) ||
            (completion_status == ARM_CONTROL_CANCELLED))
        {
            return completion_status;
        }
        if (completion_status == ARM_CONTROL_DRIVER_ERROR)
        {
            continue;
        }

        osDelay(1U);
    }
}

/*
  函数说明: 校验机械臂几何、电机映射、地址、速度上限和曲线运行参数。
  调用示例: ArmControl_ConfigIsValid(config);
  参数意义: config 为对应模块的配置结构。
*/
static int ArmControl_ConfigIsValid(const ArmControlConfig *config)
{
    uint8_t index;

    if ((config == NULL) ||
        (config->motor_map.pulses_per_revolution == 0U) ||
        (config->installation_direction < ARM_INSTALLATION_DIRECTION_1) ||
        (config->installation_direction > ARM_INSTALLATION_DIRECTION_4) ||
        ((config->ik_branch != ARM_IK_RADIAL_POSITIVE) &&
         (config->ik_branch != ARM_IK_RADIAL_NEGATIVE)))
    {
        return 0;
    }

    if ((config->trajectory_period_ms == 0U) ||
        (config->trajectory_feedback_timeout_ms == 0U) ||
        (config->trajectory_deadline_tolerance_ms == 0U) ||
        ((config->trajectory_heartbeat_ms != 0U) &&
         ((config->trajectory_heartbeat_ms <=
           config->trajectory_feedback_timeout_ms) ||
          ((uint32_t)config->trajectory_heartbeat_ms <=
           ((uint32_t)config->trajectory_period_ms +
            (uint32_t)config->trajectory_deadline_tolerance_ms)))) ||
        !isfinite(config->trajectory_limit_scale) ||
        (config->trajectory_limit_scale <= 0.0f) ||
        (config->trajectory_limit_scale > 1.0f) ||
        !isfinite(config->trajectory_settle_time_s) ||
        (config->trajectory_settle_time_s <= 0.0f) ||
        !isfinite(config->trajectory_min_radial_mm) ||
        (config->trajectory_min_radial_mm < 0.0f))
    {
        return 0;
    }

    /* 本机构把 Gamma 机械下限作为地址3的零位，两份分层配置必须一致。 */
    if ((config->kinematics.gamma_limit.min_enabled == 0U) ||
        !isfinite(config->motor_map.gamma_zero_deg) ||
        (fabsf(config->motor_map.gamma_zero_deg -
               config->kinematics.gamma_limit.min_deg) >
         ARM_CONTROL_ZERO_EPS_DEG))
    {
        return 0;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        uint8_t previous;

        if (config->motor_addr[index] == 0U)
        {
            return 0;
        }
        for (previous = 0U; previous < index; ++previous)
        {
            if (config->motor_addr[previous] == config->motor_addr[index])
            {
                return 0;
            }
        }

        if (!isfinite(config->vel_rpm[index]) ||
            (config->vel_rpm[index] <= 0.0f) ||
            (config->vel_rpm[index] > 3000.0f) ||
            (config->acc_rpm_per_s[index] == 0U) ||
            (config->dec_rpm_per_s[index] == 0U) ||
            !isfinite(config->trajectory_kp_rpm_per_deg[index]) ||
            (config->trajectory_kp_rpm_per_deg[index] < 0.0f) ||
            !isfinite(config->trajectory_correction_limit_rpm[index]) ||
            (config->trajectory_correction_limit_rpm[index] < 0.0f) ||
            !isfinite(config->trajectory_feedback_error_limit_deg[index]) ||
            (config->trajectory_feedback_error_limit_deg[index] <= 0.0f))
        {
            return 0;
        }
    }

    return 1;
}

/*
  函数说明: 自动求较短共同时间并同步正常轴；微动允许提前到位，正时长请求保持严格同步。
  调用示例: ArmControl_PlanTargets(config, current_pulse, targets, requested_s, profile);
  参数意义: config 为对应模块的配置结构。
            current_pulse 为当前带符号电机脉冲。
            targets 为三轴驱动器绝对位置目标。
            requested_s 为内部时间请求，秒；0 表示自动同步且微动可提前到位，正值表示指定同步时间。
            profile 为各轴速度、加减速度及预计到位时间，planned_time_s 为最慢轴时间。
*/
static ArmControlStatus ArmControl_PlanTargets(
    const ArmControlConfig *config, const int32_t current_pulse[3],
    const ArmMotorTargets *targets, float requested_s, ArmControlProfile *profile)
{
    double distance[3];
    double common_s;
    double next_s;
    float next_axis_s;
    float earliest;
    float latest = 0.0f;
    uint16_t speed_tenths;
    uint8_t index;
    uint8_t attempt;
    bool all_valid;
    ArmControlStatus status;
    ArmControlProfile fastest;

    if (config == NULL || current_pulse == NULL || targets == NULL || profile == NULL)
        return ARM_CONTROL_NULL_POINTER;
    if (!isfinite(requested_s) || requested_s < 0.0f || config->motor_map.pulses_per_revolution == 0U)
        return ARM_CONTROL_INVALID_CONFIG;
    memset(profile, 0, sizeof(*profile));
    for (index = 0U; index < 3U; ++index)
    {
        if (!isfinite(config->vel_rpm[index]) || config->vel_rpm[index] < 0.1f ||
            config->vel_rpm[index] > 3000.0f || config->acc_rpm_per_s[index] == 0U ||
            config->dec_rpm_per_s[index] == 0U) return ARM_CONTROL_INVALID_CONFIG;
    }

    earliest = INFINITY;
    for (index = 0U; index < 3U; ++index)
    {
        distance[index] = ArmControlProfile_DistanceDegrees(targets->pulse[index],
            current_pulse[index], config->motor_map.pulses_per_revolution);
        /* 仅跳过量化后确实没有变化的轴；非零小位移仍然下发并等待到位。 */
        if (distance[index] <= 0.0) continue;
        if (requested_s > 0.0f)
        {
            status = ArmControlProfile_CalculateTimed(config, index, distance[index], requested_s,
                &profile->acc_rpm_per_s[index], &profile->dec_rpm_per_s[index],
                &profile->vel_rpm[index], &profile->predicted_time_s[index], NULL);
            if (status != ARM_CONTROL_OK) return status;
        }
        else
        {
            /* 取不超过配置上限的最大 0.1 RPM 编码，修正浮点边界。 */
            speed_tenths = (uint16_t)(config->vel_rpm[index] * 10.0f);
            if ((float)speed_tenths / 10.0f > config->vel_rpm[index]) --speed_tenths;
            if ((float)(speed_tenths + 1U) / 10.0f <= config->vel_rpm[index]) ++speed_tenths;
            profile->vel_rpm[index] = (float)speed_tenths / 10.0f;
            profile->acc_rpm_per_s[index] = config->acc_rpm_per_s[index];
            profile->dec_rpm_per_s[index] = config->dec_rpm_per_s[index];
            profile->predicted_time_s[index] = (float)ArmControlProfile_TimeSeconds(
                distance[index], (double)speed_tenths / 10.0,
                profile->acc_rpm_per_s[index], profile->dec_rpm_per_s[index]);
        }
        if (profile->predicted_time_s[index] < earliest) earliest = profile->predicted_time_s[index];
        if (profile->predicted_time_s[index] > latest) latest = profile->predicted_time_s[index];
    }
    if (requested_s > 0.0f || latest == 0.0f)
    {
        /* 显式定时入口维持严格同步语义，不使用自动模式的小位移例外。 */
        if (latest - earliest > (float)ARM_CONTROL_TIME_TOLERANCE_S)
            return ARM_CONTROL_TIME_UNACHIEVABLE;
        profile->planned_time_s = latest;
        return ARM_CONTROL_OK;
    }

    common_s = (double)latest;
    for (index = 0U; index < 3U; ++index)
    {
        /*
         * 按最快共同时间固定微动分类，避免后续量化修正把正常轴变成微动轴。
         * 以配置加减速降至最低 0.1 RPM 仍会提前的轴，允许保持最快合法参数。
         */
        if (distance[index] > 0.0 &&
            ArmControlProfile_TimeSeconds(distance[index], ARM_CONTROL_SPEED_STEP_RPM,
                config->acc_rpm_per_s[index], config->dec_rpm_per_s[index]) <
                common_s - ARM_CONTROL_TIME_TOLERANCE_S)
            profile->small_move_mask |= (uint8_t)(1U << index);
    }
    fastest = *profile;
    for (attempt = 0U; attempt < ARM_CONTROL_SYNC_ATTEMPTS; ++attempt)
    {
        *profile = fastest;
        earliest = INFINITY;
        latest = 0.0f;
        all_valid = true;
        next_s = common_s + ARM_CONTROL_TIME_EPS_S;
        for (index = 0U; index < 3U; ++index)
        {
            if (distance[index] <= 0.0 ||
                (profile->small_move_mask & (uint8_t)(1U << index)) != 0U) continue;
            if ((double)fastest.predicted_time_s[index] <
                common_s - ARM_CONTROL_TIME_TOLERANCE_S)
            {
                status = ArmControlProfile_CalculateTimed(config, index, distance[index], (float)common_s,
                    &profile->acc_rpm_per_s[index], &profile->dec_rpm_per_s[index],
                    &profile->vel_rpm[index], &profile->predicted_time_s[index], &next_axis_s);
                if (status != ARM_CONTROL_OK)
                {
                    all_valid = false;
                    /* 只推进到已找到的较晚候选，避免直接降一档速度导致大幅延时。 */
                    if (isfinite(next_axis_s) && (double)next_axis_s > next_s)
                        next_s = ceil((double)next_axis_s * 1000.0) * 0.001;
                    continue;
                }
            }
            if (profile->predicted_time_s[index] < earliest) earliest = profile->predicted_time_s[index];
            if (profile->predicted_time_s[index] > latest) latest = profile->predicted_time_s[index];
        }
        if (all_valid && latest - earliest <= (float)ARM_CONTROL_TIME_TOLERANCE_S)
        {
            profile->planned_time_s = latest;
            return ARM_CONTROL_OK;
        }
        common_s = next_s;
    }
    return ARM_CONTROL_TIME_UNACHIEVABLE;
}



/*
  函数说明: 规划并打包三轴绝对位置指令，登记待到位目标；只在真实到位后更新起点。
  调用示例: ArmControl_SendTargets(config, targets, time_s, profile);
  参数意义: config 为对应模块的配置结构。
            targets 为三轴驱动器绝对位置目标。
            time_s 为内部时间请求，秒；0 表示自动同步且微动可提前到位，正值表示指定同步时间。
            profile 为各轴运动的速度、加减速度和预测时长。
*/
static ArmControlStatus ArmControl_SendTargets(
    const ArmControlConfig *config, ArmMotorTargets *targets,
    float time_s, ArmControlProfile *profile)
{
    X_V2_Traj_Pos_Command_t commands[3];
    ArmControlStatus status;
    uint8_t count = 0U;
    uint8_t pending = 0U;
    uint8_t index;
    if (config == NULL || targets == NULL || profile == NULL) return ARM_CONTROL_NULL_POINTER;
    if (!config->use_multi_command) return ARM_CONTROL_INVALID_CONFIG;
    if (arm_control_active_config != config || !arm_control_current_valid)
        return ARM_CONTROL_POSITION_UNKNOWN;
    status = ArmControl_PlanTargets(config, arm_control_current_pulse, targets, time_s, profile);
    if (status != ARM_CONTROL_OK) return status;
    for (index = 0U; index < 3U; ++index)
    {
        if (profile->vel_rpm[index] == 0.0f) continue;
        pending |= (uint8_t)(1U << index);
        commands[count].addr = config->motor_addr[index];
        /* 绝对位置协议的方向表示目标坐标符号，不能改成运动增量的方向。 */
        commands[count].dir = (X_V2_Direction_t)targets->direction[index];
        commands[count].acc_rpm_per_s = profile->acc_rpm_per_s[index];
        commands[count].dec_rpm_per_s = profile->dec_rpm_per_s[index];
        commands[count].vel_rpm = profile->vel_rpm[index];
        commands[count].pos_deg = (float)(fabs((double)ArmControlProfile_PulseToProtocolTenths(
            targets->pulse[index], config->motor_map.pulses_per_revolution)) * 0.1);
        commands[count].mode = X_V2_POS_ABSOLUTE;
        commands[count].snF = true;
        ++count;
    }
    if (!count)
    {
        memcpy(arm_control_current_pulse, targets->pulse, sizeof(arm_control_current_pulse));
        return ARM_CONTROL_OK;
    }
    (void)ZDT_CAN_FlushRx();
    if (X_V2_MultiMotor_Traj_Pos_Control(commands, count) != HAL_OK)
    {
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        return ARM_CONTROL_CAN_ERROR;
    }
    taskENTER_CRITICAL();
    memcpy(arm_control_move_tracker.target_pulse, targets->pulse, sizeof(targets->pulse));
    arm_control_move_tracker.pending_mask = pending;
    arm_control_move_tracker.reached_mask = 0U;
    arm_control_move_tracker.error_mask = 0U;
    arm_control_move_tracker.last_error_code = 0U;
    arm_control_move_tracker.active = true;
    for (index = 0U; index < 3U; ++index)
    {
        if (pending & (1U << index))
        {
            arm_control_driver_status[index].position_reached_flag = false;
            arm_control_driver_status[index].last_error_code = 0U;
        }
    }
    taskEXIT_CRITICAL();
    return ARM_CONTROL_OK;
}


/*
  函数说明: 装载原机械参数、方向、三轴速度和加减速上限及诊断配置。
  调用示例: ArmControl_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmControl_LoadDefaultConfig(ArmControlConfig *config)
{
    uint8_t index;
    uint32_t generation;

    if (config == NULL)
    {
        return;
    }
    if (!ArmControl_TrajectoryApiLock())
    {
        return;
    }
    if (!ArmControl_CommandLock())
    {
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    if (!ArmControl_FeedbackConfigLock())
    {
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    generation = ArmControl_ReadGeneration();
    if (!ArmControl_CommandTokenIsCurrent(generation) ||
        ((arm_control_active_config == config) &&
         (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)))
    {
        /* 运动中不允许清空正在使用的配置和状态机。 */
        ArmControl_FeedbackConfigUnlock();
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return;
    }

    memset(config, 0, sizeof(*config));
    ArmIk_LoadDefaultConfig(&config->kinematics);
    ArmMotorMap_LoadDefaultConfig(&config->motor_map);
    config->installation_direction = ARM_INSTALLATION_DIRECTION_1;
    config->ik_branch = ARM_IK_RADIAL_POSITIVE;

    config->motor_addr[0] = 1U;
    config->motor_addr[1] = 2U;
    config->motor_addr[2] = 3U;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        config->acc_rpm_per_s[index] = arm_control_default_acc[index];
        config->dec_rpm_per_s[index] = arm_control_default_dec[index];
        config->vel_rpm[index] = arm_control_default_speed[index];
        config->report_reached[index] = true;
        config->trajectory_kp_rpm_per_deg[index] =
            ARM_CONTROL_TRAJECTORY_DEFAULT_KP;
        config->trajectory_correction_limit_rpm[index] =
            ARM_CONTROL_TRAJECTORY_DEFAULT_CORRECTION_RPM;
        config->trajectory_feedback_error_limit_deg[index] =
            ARM_CONTROL_TRAJECTORY_DEFAULT_ERROR_DEG;
    }

    config->use_multi_command = true;
    config->use_sync = true;
    config->trajectory_period_ms =
        ARM_CONTROL_TRAJECTORY_DEFAULT_PERIOD_MS;
    config->trajectory_feedback_timeout_ms =
        ARM_CONTROL_TRAJECTORY_DEFAULT_FEEDBACK_TIMEOUT_MS;
    config->trajectory_deadline_tolerance_ms =
        ARM_CONTROL_TRAJECTORY_DEFAULT_DEADLINE_TOLERANCE_MS;
    config->trajectory_heartbeat_ms =
        ARM_CONTROL_TRAJECTORY_DEFAULT_HEARTBEAT_MS;
    config->trajectory_limit_scale =
        ARM_CONTROL_TRAJECTORY_DEFAULT_LIMIT_SCALE;
    config->trajectory_settle_time_s =
        ARM_CONTROL_TRAJECTORY_DEFAULT_SETTLE_TIME_S;
    config->trajectory_min_radial_mm =
        ARM_CONTROL_TRAJECTORY_DEFAULT_MIN_RADIAL_MM;
    config->zero_confirmed = false;

    if (arm_control_active_config == config)
    {
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        memset(&arm_control_trajectory_context,
               0,
               sizeof(arm_control_trajectory_context));
    }
    ArmControl_FeedbackConfigUnlock();
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
}

/*
  函数说明: 绑定单机械臂活动配置；正在执行运动时拒绝替换。
  调用示例: ArmControl_SetActiveConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_SetActiveConfig(ArmControlConfig *config)
{
    uint32_t generation;

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_CommandLock())
    {
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_NOT_INITIALIZED;
    }
    if (!ArmControl_FeedbackConfigLock())
    {
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_NOT_INITIALIZED;
    }
    generation = ArmControl_ReadGeneration();
    if (!ArmControl_CommandTokenIsCurrent(generation) ||
        ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        ArmControl_FeedbackConfigUnlock();
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_BUSY;
    }

    arm_control_active_config = config;
    ArmControl_ClearMoveTracker();
    arm_control_current_valid = false;
    arm_control_last_ik_status = ARM_IK_OK;
    memset(&arm_control_last_result, 0, sizeof(arm_control_last_result));
    memset(arm_control_driver_status,
           0,
           sizeof(arm_control_driver_status));
    memset(&arm_control_trajectory_context,
           0,
           sizeof(arm_control_trajectory_context));
    arm_control_trajectory_context.result.state =
        ARM_CONTROL_TRAJECTORY_IDLE;
    arm_control_trajectory_context.result.status = ARM_CONTROL_OK;
    ArmControl_FeedbackConfigUnlock();
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
    return ARM_CONTROL_OK;
}

/*
  函数说明: 记录操作者已确认的机械零位状态；不发送驱动器清零命令。
  调用示例: ArmControl_ConfirmZero(config, confirmed);
  参数意义: config 为对应模块的配置结构。
            confirmed 为是否已人工或由外部流程确认机械零位。
*/
void ArmControl_ConfirmZero(ArmControlConfig *config, bool confirmed)
{
    uint32_t generation;

    if (config == NULL)
    {
        return;
    }
    if (!ArmControl_TrajectoryApiLock())
    {
        return;
    }
    if (!ArmControl_CommandLock())
    {
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    generation = ArmControl_ReadGeneration();
    if (!ArmControl_CommandTokenIsCurrent(generation) ||
        ((arm_control_active_config == config) &&
         (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)))
    {
        /* 零位只能在所有运动结束后由归零流程确认或撤销。 */
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    config->zero_confirmed = confirmed;

    if (arm_control_active_config != config)
    {
        /* 非活动配置不得清除当前机械臂的全局运动跟踪状态。 */
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return;
    }
    if (!confirmed)
    {
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return;
    }

    {
        ArmJointAngles zero_joints;
        ArmMotorTargets zero_targets;

        zero_joints.alpha_deg = 0.0f;
        zero_joints.beta_deg = 0.0f;
        zero_joints.gamma_deg = config->motor_map.gamma_zero_deg;
        if (ArmMotorMap_JointsToTargets(
                &config->motor_map,
                config->installation_direction,
                &zero_joints,
                &zero_targets) != ARM_MOTOR_MAP_OK)
        {
            arm_control_current_valid = false;
            ArmControl_CommandUnlock();
            ArmControl_TrajectoryApiUnlock();
            return;
        }

        arm_control_current_pulse[0] = zero_targets.pulse[0];
        arm_control_current_pulse[1] = zero_targets.pulse[1];
        arm_control_current_pulse[2] = zero_targets.pulse[2];
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = true;
    }
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
}

/*
  函数说明: 设置后续 XYZ 运动使用的径向逆解分支。
  调用示例: ArmControl_SetIkBranch(config, branch);
  参数意义: config 为对应模块的配置结构。
            branch 为逆解径向分支。
*/
ArmControlStatus ArmControl_SetIkBranch(ArmControlConfig *config,
                                      ArmIkRadialBranch branch)
{
    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if ((branch != ARM_IK_RADIAL_POSITIVE) &&
        (branch != ARM_IK_RADIAL_NEGATIVE))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_CommandLock())
    {
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_CAN_ERROR;
    }
    if (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_BUSY;
    }

    config->ik_branch = branch;
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
    return ARM_CONTROL_OK;
}

/*
  函数说明: 依次使能或失能三个机械臂位置电机，并维护位置有效状态。
  调用示例: ArmControl_EnableAll(config, state);
  参数意义: config 为对应模块的配置结构。
            state 为true 使能，false 失能。
*/
ArmControlStatus ArmControl_EnableAll(const ArmControlConfig *config,
                                    bool state)
{
    uint8_t index;
    uint8_t offset;
    uint8_t enable_commands[ARM_CONTROL_MOTOR_COUNT * 6U];

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }

    if (config->use_multi_command)
    {
        offset = 0U;
        for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
        {
            enable_commands[offset++] = config->motor_addr[index];
            enable_commands[offset++] = 0xF3U;
            enable_commands[offset++] = 0xABU;
            enable_commands[offset++] = (uint8_t)state;
            enable_commands[offset++] = 0U;
            enable_commands[offset++] = 0x6BU;
        }

        return (X_V2_MultiMotor_Command(enable_commands, offset) == HAL_OK) ?
               ARM_CONTROL_OK : ARM_CONTROL_CAN_ERROR;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_En_Control(config->motor_addr[index], state, false) != HAL_OK)
        {
            return ARM_CONTROL_CAN_ERROR;
        }
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 使旧运动代次失效，停止三轴并撤销未完成目标。
  调用示例: ArmControl_StopAll(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_StopAll(const ArmControlConfig *config)
{
    ArmControlStatus cleanup_status = ARM_CONTROL_OK;
    bool trajectory_was_busy;
    bool stop_failed = false;
    uint8_t index;

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }

    /*
     * 先原子关闭新运动入口并使所有在途计算代次失效；Stop 完成前，
     * 即使别的任务读到了新代次，也会被 stop_depth 门闩拒绝。
     */
    taskENTER_CRITICAL();
    ++arm_control_stop_depth;
    ++arm_control_trajectory_generation;
    trajectory_was_busy = ArmControl_TrajectoryStateIsBusy(
        arm_control_trajectory_context.result.state);
    taskEXIT_CRITICAL();

    if (!ArmControl_CommandLock())
    {
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        if (trajectory_was_busy)
        {
            arm_control_trajectory_context.result.state =
                ARM_CONTROL_TRAJECTORY_FAULT;
            arm_control_trajectory_context.result.status =
                ARM_CONTROL_CAN_ERROR;
        }
        ArmControl_StopGateExit();
        return ARM_CONTROL_CAN_ERROR;
    }
    /* 地址 0 会广播到同一 CAN 总线的 ID4 和底盘；这里只停止 XYZ 三轴。 */
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Stop_Now(config->motor_addr[index], false) != HAL_OK)
        {
            stop_failed = true;
        }
    }
    if (stop_failed)
    {
        ArmControl_CommandUnlock();
        ArmControl_ClearMoveTracker();
        arm_control_current_valid = false;
        if (trajectory_was_busy)
        {
            /*
             * 停止帧未发出时必须保留驱动器心跳。否则“停止失败、
             * 关心跳成功”会让电机持续执行最后一条 F6 而失去超时急停。
             */
            arm_control_trajectory_context.result.state =
                ARM_CONTROL_TRAJECTORY_FAULT;
            arm_control_trajectory_context.result.status =
                ARM_CONTROL_CAN_ERROR;
        }
        ArmControl_StopGateExit();
        return ARM_CONTROL_CAN_ERROR;
    }
    ArmControl_CommandUnlock();

    ArmControl_ClearMoveTracker();
    arm_control_current_valid = false;
    /* cleanup 内部会读取最终标志；无条件调用可覆盖并发 enable 的尾部。 */
    cleanup_status = ArmControl_TrajectoryBestEffortCleanup(config);
    if (trajectory_was_busy)
    {
        arm_control_trajectory_context.result.state =
            ARM_CONTROL_TRAJECTORY_FAULT;
        arm_control_trajectory_context.result.status =
            (cleanup_status == ARM_CONTROL_OK) ?
            ARM_CONTROL_CANCELLED : cleanup_status;
    }
    ArmControl_StopGateExit();
    return cleanup_status;
}

/*
  函数说明: 完成关节限位校验、电机映射与指令下发；由已加锁的入口调用。
  调用示例: ArmControl_MoveToJointsInternal(config, joints, result, ik_status, time_s, request_generation, command_lock_held);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
            ik_status 为输出逆运动学状态码。
            time_s 为内部同步时间请求，秒；0 表示自动规划。
            request_generation 为本次运动请求的停止代次。
            command_lock_held 为调用者是否已经持有命令锁。
*/
static ArmControlStatus ArmControl_MoveToJointsInternal(
    const ArmControlConfig *config,
    const ArmJointAngles *joints,
    ArmControlResult *result,
    ArmIkStatus *ik_status,
    float time_s,
    uint32_t request_generation,
    bool command_lock_held)
{
    ArmIkStatus joint_status;
    ArmMotorMapStatus map_status;
    ArmControlStatus motion_status;
    bool acquired_here = false;

    if (ik_status != NULL)
    {
        *ik_status = ARM_IK_OK;
    }
    if ((config == NULL) || (joints == NULL) || (result == NULL))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_NULL_POINTER;
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_INVALID_CONFIG;
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        return ARM_CONTROL_NOT_ZEROED;
    }

    joint_status = ArmIk_ValidateJoints(&config->kinematics, joints);
    if (ik_status != NULL)
    {
        *ik_status = joint_status;
    }
    if (joint_status != ARM_IK_OK)
    {
        return ARM_CONTROL_IK_ERROR;
    }

    result->joints = *joints;
    joint_status = ArmIk_Forward(&config->kinematics,
                                 joints,
                                 &result->position);
    if (joint_status != ARM_IK_OK)
    {
        if (ik_status != NULL) *ik_status = joint_status;
        return ARM_CONTROL_IK_ERROR;
    }

    map_status = ArmMotorMap_JointsToTargets(
        &config->motor_map,
        config->installation_direction,
        joints,
        &result->motors);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        return ARM_CONTROL_MAPPING_ERROR;
    }

    if (!command_lock_held)
    {
        if (!ArmControl_CommandLock())
        {
            return ARM_CONTROL_CAN_ERROR;
        }
        acquired_here = true;

        /* 锁内重查，避免 Stop 或另一条运动在计算期间抢先提交。 */
        if (!ArmControl_CommandTokenIsCurrent(request_generation))
        {
            ArmControl_CommandUnlock();
            return ARM_CONTROL_CANCELLED;
        }
        if (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
        {
            ArmControl_CommandUnlock();
            return ARM_CONTROL_BUSY;
        }
    }
    else if (!ArmControl_CommandTokenIsCurrent(request_generation))
    {
        return ARM_CONTROL_CANCELLED;
    }

    motion_status = ArmControl_SendTargets(
        config, &result->motors, time_s, &result->profile);
    if (!ArmControl_CommandTokenIsCurrent(request_generation))
    {
        motion_status = ARM_CONTROL_CANCELLED;
    }
    if (acquired_here)
    {
        ArmControl_CommandUnlock();
    }
    return motion_status;
}

/*
  函数说明: 关节角与关节偏移入口共用的命令保护。
  成功时持有 API 锁和命令锁；失败时释放已取得的锁，调用者可直接返回。
  两把锁的取得/释放顺序及 Stop 代次检查与原入口一致。
  调用示例: ArmControl_BeginJointCommand(request_generation);
  参数意义: request_generation 为本次运动请求的停止代次。
*/
static ArmControlStatus ArmControl_BeginJointCommand(
    uint32_t *request_generation)
{
    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_TryCaptureCommandToken(request_generation))
    {
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_CANCELLED;
    }

    if (!ArmControl_CommandLock())
    {
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_CAN_ERROR;
    }
    if (!ArmControl_CommandTokenIsCurrent(*request_generation))
    {
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_CANCELLED;
    }
    if (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        ArmControl_CommandUnlock();
        ArmControl_TrajectoryApiUnlock();
        return ARM_CONTROL_BUSY;
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 按顺序释放关节命令锁和运动 API 锁。
  调用示例: ArmControl_EndJointCommand();
  参数意义: 无。
*/
static void ArmControl_EndJointCommand(void)
{
    ArmControl_CommandUnlock();
    ArmControl_TrajectoryApiUnlock();
}

/*
  函数说明: 按几何关节角自动规划同步运动，微动可提前到位，等待全部实际到位。
  调用示例: ArmControl_MoveToJoints(config, joints, result, ik_status);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
            ik_status 为输出逆运动学状态码。
*/
ArmControlStatus ArmControl_MoveToJoints(const ArmControlConfig *config,
                                       const ArmJointAngles *joints,
                                       ArmControlResult *result,
                                       ArmIkStatus *ik_status)
{
    ArmControlStatus motion_status;
    uint32_t request_generation;

    motion_status = ArmControl_BeginJointCommand(&request_generation);
    if (motion_status != ARM_CONTROL_OK)
    {
        return motion_status;
    }
    motion_status = ArmControl_MoveToJointsInternal(
        config,
        joints,
        result,
        ik_status,
        0.0f,
        request_generation,
        true);
    ArmControl_CommandUnlock();
    if (motion_status == ARM_CONTROL_OK)
        motion_status = ArmControl_WaitForCompletion(config,
            result->profile.planned_time_s, &result->profile, request_generation);
    ArmControl_TrajectoryApiUnlock();
    return motion_status;
}


/*
  函数说明: 执行 XYZ 逆解、时间规划、发令和实际到位等待；自动模式正常轴同步、微动可提前到位。
  调用示例: ArmControl_MoveToXYZInternal(x_mm, y_mm, z_mm, requested_time_s);
  参数意义: x_mm 为机械臂基坐标系 X 坐标，mm。
            y_mm 为机械臂基坐标系 Y 坐标，mm。
            z_mm 为机械臂基坐标系 Z 坐标，mm。
            requested_time_s 为请求时长，秒；0 表示自动规划。
*/
static ArmControlStatus ArmControl_MoveToXYZInternal(
    float x_mm, float y_mm, float z_mm, float requested_time_s)
{
    ArmJointAngles joints;
    ArmIkStatus solve_status;
    ArmControlStatus motion_status;
    ArmControlConfig *config;
    ArmControlResult *result = &arm_control_last_result;
    ArmIkStatus *ik_status = &arm_control_last_ik_status;
    uint32_t request_generation;

    if (!ArmControl_TryCaptureCommandToken(&request_generation))
    {
        return ARM_CONTROL_CANCELLED;
    }
    if (!ArmControl_CommandLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    if (!ArmControl_CommandTokenIsCurrent(request_generation))
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_CANCELLED;
    }

    config = arm_control_active_config;

    *ik_status = ARM_IK_OK;
    memset(result, 0, sizeof(*result));

    if (config == NULL)
    {
        *ik_status = ARM_IK_NULL_POINTER;
        ArmControl_CommandUnlock();
        return ARM_CONTROL_NOT_INITIALIZED;
    }
    if (ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        *ik_status = ARM_IK_INVALID_CONFIG;
        ArmControl_CommandUnlock();
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_NOT_ZEROED;
    }

    /* 逆解或关节限位失败后立即返回；下方发送函数不会被调用。 */
    solve_status = ArmIk_SolveBranch(&config->kinematics,
                                     x_mm,
                                     y_mm,
                                     z_mm,
                                     config->ik_branch,
                                     &joints);
    *ik_status = solve_status;
    if (solve_status != ARM_IK_OK)
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_IK_ERROR;
    }

    motion_status = ArmControl_MoveToJointsInternal(
        config,
        &joints,
        result,
        ik_status,
        requested_time_s,
        request_generation,
        true);
    ArmControl_CommandUnlock();
    if (motion_status == ARM_CONTROL_OK)
    {
        /* 等待所有有效轴返回 FD 9F；收到最后一帧后立即返回。 */
        motion_status = ArmControl_WaitForCompletion(
            config,
            result->profile.planned_time_s,
            &result->profile,
            request_generation);
    }
    return motion_status;
}



/*
  函数说明: 为 XYZ 内部运动流程提供 API 互斥保护。
  调用示例: ArmControl_MoveXYZ(x_mm, y_mm, z_mm, requested_time_s);
  参数意义: x_mm 为机械臂基坐标系 X 坐标，mm。
            y_mm 为机械臂基坐标系 Y 坐标，mm。
            z_mm 为机械臂基坐标系 Z 坐标，mm。
            requested_time_s 为请求时长，秒；0 表示自动规划。
*/
static ArmControlStatus ArmControl_MoveXYZ(
    float x_mm, float y_mm, float z_mm, float requested_time_s)
{
    ArmControlStatus status;

    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    status = ArmControl_MoveToXYZInternal(x_mm, y_mm, z_mm, requested_time_s);
    ArmControl_TrajectoryApiUnlock();
    return status;
}



/*
  函数说明: 返回最近一次 XYZ 逆解、目标脉冲和规划结果的只读指针。
  调用示例: ArmControl_GetLastResult();
  参数意义: 无。
*/
const ArmControlResult *ArmControl_GetLastResult(void)
{
    return &arm_control_last_result;
}

/*
  函数说明: 将机械零位相对偏移转换为关节角，自动规划并等待到位。
  调用示例: ArmControl_MoveJointOffsets(config, base_deg, large_arm_deg, small_arm_deg, result, ik_status);
  参数意义: config 为对应模块的配置结构。
            base_deg 为底座相对机械零位角度，度。
            large_arm_deg 为大臂相对机械零位角度，度。
            small_arm_deg 为小臂相对机械零位角度，度。
            result 为输出计算结果。
            ik_status 为输出逆运动学状态码。
*/
ArmControlStatus ArmControl_MoveJointOffsets(const ArmControlConfig *config,
                                            float base_deg,
                                            float large_arm_deg,
                                            float small_arm_deg,
                                            ArmControlResult *result,
                                            ArmIkStatus *ik_status)
{
    ArmJointAngles joints;
    ArmMotorMapStatus map_status;
    ArmControlStatus motion_status;
    uint32_t request_generation;

    motion_status = ArmControl_BeginJointCommand(&request_generation);
    if (motion_status != ARM_CONTROL_OK)
    {
        return motion_status;
    }

    if (ik_status != NULL)
    {
        *ik_status = ARM_IK_OK;
    }
    if ((config == NULL) || (result == NULL))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_NULL_POINTER;
        ArmControl_EndJointCommand();
        return ARM_CONTROL_NULL_POINTER;
    }

    map_status = ArmMotorMap_OffsetsToJoints(&config->motor_map,
                                             base_deg,
                                             large_arm_deg,
                                             small_arm_deg,
                                             &joints);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        ArmControl_EndJointCommand();
        return ARM_CONTROL_MAPPING_ERROR;
    }

    motion_status = ArmControl_MoveToJointsInternal(
        config,
        &joints,
        result,
        ik_status,
        0.0f,
        request_generation,
        true);
    ArmControl_CommandUnlock();
    if (motion_status == ARM_CONTROL_OK)
        motion_status = ArmControl_WaitForCompletion(config,
            result->profile.planned_time_s, &result->profile, request_generation);
    ArmControl_TrajectoryApiUnlock();
    return motion_status;
}


/*
  函数说明: 判断曲线状态是否属于预检、运行或停止收尾阶段。
  调用示例: ArmControl_TrajectoryStateIsBusy(state);
  参数意义: state 为状态值或状态快照。
*/
static bool ArmControl_TrajectoryStateIsBusy(
    ArmControlTrajectoryState state)
{
    return (state == ARM_CONTROL_TRAJECTORY_PRECHECKING) ||
           (state == ARM_CONTROL_TRAJECTORY_WAITING_FEEDBACK) ||
           (state == ARM_CONTROL_TRAJECTORY_RUNNING_SPEED) ||
           (state == ARM_CONTROL_TRAJECTORY_STOPPING_SPEED) ||
           (state == ARM_CONTROL_TRAJECTORY_SETTLING_POSITION);
}

/*
  函数说明: 取得指定电机的逻辑正方向定义。
  调用示例: ArmControl_PositiveDirection(config, index);
  参数意义: config 为对应模块的配置结构。
            index 为数组或电机下标，从 0 开始。
*/
static ArmMotorDirection ArmControl_PositiveDirection(
    const ArmControlConfig *config,
    uint8_t index)
{
    if (index == 0U)
    {
        return config->motor_map.motor1_positive_direction;
    }
    if (index == 1U)
    {
        return config->motor_map.motor2_positive_direction;
    }
    return config->motor_map.motor3_positive_direction;
}

/*
  函数说明: 把带符号逻辑 RPM 转换为驱动器方向位。
  调用示例: ArmControl_VelocityDirection(config, index, signed_rpm);
  参数意义: config 为对应模块的配置结构。
            index 为数组或电机下标，从 0 开始。
            signed_rpm 为带逻辑方向符号的转速，RPM。
*/
static X_V2_Direction_t ArmControl_VelocityDirection(
    const ArmControlConfig *config,
    uint8_t index,
    float signed_rpm)
{
    ArmMotorDirection direction = ArmControl_PositiveDirection(config, index);

    if (signed_rpm < 0.0f)
    {
        direction = (direction == ARM_MOTOR_DIR_CW) ?
                    ARM_MOTOR_DIR_CCW : ARM_MOTOR_DIR_CW;
    }
    return (X_V2_Direction_t)direction;
}

/*
  函数说明: 把驱动器位置反馈转换为带符号逻辑电机轴角度。
  调用示例: ArmControl_LogicalPositionDegrees(config, index, driver_position_deg);
  参数意义: config 为对应模块的配置结构。
            index 为数组或电机下标，从 0 开始。
            driver_position_deg 为驱动器实际位置反馈，度。
*/
static float ArmControl_LogicalPositionDegrees(
    const ArmControlConfig *config,
    uint8_t index,
    float driver_position_deg)
{
    /* X 固件 36 的正号是 CW；映射层的正号由各轴实测方向定义。 */
    return (ArmControl_PositiveDirection(config, index) ==
            ARM_MOTOR_DIR_CW) ?
           driver_position_deg : -driver_position_deg;
}

/*
  函数说明: 检查驱动器实际位置是否与曲线起点一致。
  调用示例: ArmControl_TrajectoryStartMatchesFeedback(config);
  参数意义: config 为对应模块的配置结构。
*/
static bool ArmControl_TrajectoryStartMatchesFeedback(
    const ArmControlConfig *config)
{
    float actual_degrees;
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].realtime_position_valid)
        {
            return false;
        }
        actual_degrees = ArmControl_LogicalPositionDegrees(
            config,
            index,
            arm_control_driver_status[index].realtime_position_deg);
        if (!isfinite(actual_degrees) ||
            (fabsf(actual_degrees - arm_control_trajectory_context.
                   reference_coordinates.degree[index]) >
             ARM_CONTROL_TRAJECTORY_START_MATCH_DEG))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 用新鲜的三轴位置反馈重建当前脉冲缓存。
  调用示例: ArmControl_SyncCurrentFromFeedback(config, now_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
*/
static ArmControlStatus ArmControl_SyncCurrentFromFeedback(
    const ArmControlConfig *config,
    uint32_t now_ms)
{
    int32_t pulse[ARM_CONTROL_MOTOR_COUNT];
    uint8_t index;
    double pulse_value;
    double rounded;
    float logical_degrees;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].realtime_position_valid ||
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        realtime_position_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return ARM_CONTROL_FEEDBACK_TIMEOUT;
        }

        logical_degrees = ArmControl_LogicalPositionDegrees(
            config,
            index,
            arm_control_driver_status[index].realtime_position_deg);
        pulse_value = (double)logical_degrees *
                      (double)config->motor_map.pulses_per_revolution /
                      360.0;
        rounded = (pulse_value >= 0.0) ?
                  floor(pulse_value + 0.5) : ceil(pulse_value - 0.5);
        if (!isfinite(rounded) ||
            (rounded > (double)INT32_MAX) ||
            (rounded < (double)INT32_MIN))
        {
            return ARM_CONTROL_MAPPING_ERROR;
        }
        pulse[index] = (int32_t)rounded;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        arm_control_current_pulse[index] = pulse[index];
    }
    arm_control_current_valid = true;
    return ARM_CONTROL_OK;
}

/*
  函数说明: 由当前三轴脉冲反算机械臂末端坐标。
  调用示例: ArmControl_CurrentEndpoint(config, position);
  参数意义: config 为对应模块的配置结构。
            position 为三维位置或单轴位置，mm。
*/
static ArmControlStatus ArmControl_CurrentEndpoint(
    const ArmControlConfig *config,
    ArmCartesianPosition *position)
{
    ArmJointAngles joints;
    ArmMotorMapStatus map_status;
    ArmIkStatus ik_status;

    if ((config == NULL) || (position == NULL))
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!arm_control_current_valid || (arm_control_active_config != config))
    {
        return ARM_CONTROL_POSITION_UNKNOWN;
    }

    map_status = ArmMotorMap_PulsesToJoints(
        &config->motor_map,
        config->installation_direction,
        arm_control_current_pulse,
        &joints);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        return ARM_CONTROL_MAPPING_ERROR;
    }

    ik_status = ArmIk_ValidateJoints(&config->kinematics, &joints);
    if (ik_status != ARM_IK_OK)
    {
        return ARM_CONTROL_IK_ERROR;
    }
    ik_status = ArmIk_Forward(&config->kinematics, &joints, position);
    return (ik_status == ARM_IK_OK) ?
           ARM_CONTROL_OK : ARM_CONTROL_IK_ERROR;
}

/*
  函数说明: 对一个曲线采样点依次执行采样、逆解和连续电机坐标映射。
  调用示例: ArmControl_SolveTrajectorySample(config, trajectory, elapsed_s, joints, coordinates, trajectory_status, ik_status);
  参数意义: config 为对应模块的配置结构。
            trajectory 为三次贝塞尔曲线结构。
            elapsed_s 为曲线已经运行的时间，秒。
            joints 为三个几何关节角，度。
            coordinates 为输出连续电机坐标，度。
            trajectory_status 为输出曲线采样状态码。
            ik_status 为输出逆运动学状态码。
*/
static ArmControlStatus ArmControl_SolveTrajectorySample(
    const ArmControlConfig *config,
    const ArmCubicBezierTrajectory *trajectory,
    float elapsed_s,
    ArmJointAngles *joints,
    ArmMotorCoordinates *coordinates,
    ArmTrajectoryStatus *trajectory_status,
    ArmIkStatus *ik_status)
{
    ArmTrajectorySample sample;
    ArmMotorMapStatus map_status;
    float radial_mm;

    *trajectory_status = ArmTrajectory_Evaluate(
        trajectory, elapsed_s, &sample);
    if (*trajectory_status != ARM_TRAJECTORY_OK)
    {
        return ARM_CONTROL_TRAJECTORY_ERROR;
    }

    radial_mm = sqrtf((sample.position.x_mm * sample.position.x_mm) +
                      (sample.position.y_mm * sample.position.y_mm));
    if (!isfinite(radial_mm) ||
        (radial_mm < config->trajectory_min_radial_mm))
    {
        return ARM_CONTROL_TRAJECTORY_ERROR;
    }

    *ik_status = ArmIk_SolveBranch(
        &config->kinematics,
        sample.position.x_mm,
        sample.position.y_mm,
        sample.position.z_mm,
        config->ik_branch,
        joints);
    if (*ik_status != ARM_IK_OK)
    {
        return ARM_CONTROL_IK_ERROR;
    }

    map_status = ArmMotorMap_JointsToMotorCoordinates(
        &config->motor_map,
        config->installation_direction,
        joints,
        coordinates);
    return (map_status == ARM_MOTOR_MAP_OK) ?
           ARM_CONTROL_OK : ARM_CONTROL_MAPPING_ERROR;
}

/*
  函数说明: 预检曲线几何可达性、方向和速度加减速要求。
  调用示例: ArmControl_PreflightTrajectory(config, trajectory);
  参数意义: config 为对应模块的配置结构。
            trajectory 为三次贝塞尔曲线结构。
*/
static ArmControlStatus ArmControl_PreflightTrajectory(
    const ArmControlConfig *config,
    const ArmCubicBezierTrajectory *trajectory)
{
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmControlTrajectoryResult *result = &context->result;
    ArmJointAngles previous_joints;
    ArmJointAngles joints;
    ArmMotorCoordinates previous_coordinates;
    ArmMotorCoordinates coordinates;
    ArmTrajectoryStatus trajectory_status = ARM_TRAJECTORY_OK;
    ArmIkStatus ik_status = ARM_IK_OK;
    ArmControlStatus motion_status;
    float previous_derivative[ARM_CONTROL_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_derivative[ARM_CONTROL_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_acceleration[ARM_CONTROL_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_deceleration[ARM_CONTROL_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float derivative;
    float second_derivative;
    float absolute_value;
    float u;
    float du;
    float elapsed_s;
    float safe_limit;
    float required_time_s;
    float candidate_time_s;
    uint32_t sample_index;
    uint32_t precheck_intervals;
    uint8_t motor_index;
    int8_t derivative_direction;

    if ((config == NULL) || (trajectory == NULL))
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if ((trajectory->initialized == 0U) ||
        (trajectory->segment_count == 0U) ||
        (trajectory->segment_count > ARM_CONTROL_TRAJECTORY_MAX_SEGMENTS))
    {
        result->trajectory_status = ARM_TRAJECTORY_INVALID_ARGUMENT;
        return ARM_CONTROL_TIME_INVALID;
    }

    memset(result->max_speed_rpm, 0, sizeof(result->max_speed_rpm));
    memset(result->max_acc_rpm_per_s,
           0,
           sizeof(result->max_acc_rpm_per_s));
    memset(result->max_dec_rpm_per_s,
           0,
           sizeof(result->max_dec_rpm_per_s));
    memset(result->final_speed_rpm,
           0,
           sizeof(result->final_speed_rpm));
    memset(context->planned_direction,
           0,
           sizeof(context->planned_direction));
    result->minimum_time_s = 0.0f;
    result->failed_sample = 0U;
    result->trajectory_status = ARM_TRAJECTORY_OK;
    result->ik_status = ARM_IK_OK;

    motion_status = ArmControl_SolveTrajectorySample(
        config,
        trajectory,
        0.0f,
        &previous_joints,
        &previous_coordinates,
        &trajectory_status,
        &ik_status);
    result->trajectory_status = trajectory_status;
    result->ik_status = ik_status;
    if (motion_status != ARM_CONTROL_OK)
    {
        return motion_status;
    }

    precheck_intervals = trajectory->segment_count;
    if (precheck_intervals < ARM_CONTROL_TRAJECTORY_PRECHECK_INTERVALS)
    {
        precheck_intervals = ARM_CONTROL_TRAJECTORY_PRECHECK_INTERVALS;
    }
    du = 1.0f / (float)precheck_intervals;
    for (sample_index = 1U;
         sample_index <= precheck_intervals;
         ++sample_index)
    {
        u = (float)sample_index * du;
        elapsed_s = trajectory->duration_s * u;
        motion_status = ArmControl_SolveTrajectorySample(
            config,
            trajectory,
            elapsed_s,
            &joints,
            &coordinates,
            &trajectory_status,
            &ik_status);
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        result->failed_sample = sample_index;
        if (motion_status != ARM_CONTROL_OK)
        {
            return motion_status;
        }

        /* Alpha 在 +/-180 度接缝处跳变会让底座产生约一整圈误动作。 */
        if (fabsf(joints.alpha_deg - previous_joints.alpha_deg) >=
            (180.0f - ARM_CONTROL_ZERO_EPS_DEG))
        {
            return ARM_CONTROL_DIRECTION_REVERSAL;
        }

        for (motor_index = 0U;
             motor_index < ARM_CONTROL_MOTOR_COUNT;
             ++motor_index)
        {
            derivative = (coordinates.degree[motor_index] -
                          previous_coordinates.degree[motor_index]) / du;
            if (!isfinite(derivative))
            {
                return ARM_CONTROL_TRAJECTORY_ERROR;
            }

            absolute_value = fabsf(derivative);
            if (absolute_value > maximum_derivative[motor_index])
            {
                maximum_derivative[motor_index] = absolute_value;
            }

            derivative_direction = 0;
            if (derivative >
                ARM_CONTROL_TRAJECTORY_DIRECTION_EPS_DEG_PER_U)
            {
                derivative_direction = 1;
            }
            else if (derivative <
                     -ARM_CONTROL_TRAJECTORY_DIRECTION_EPS_DEG_PER_U)
            {
                derivative_direction = -1;
            }

            if (derivative_direction != 0)
            {
                if (context->planned_direction[motor_index] == 0)
                {
                    context->planned_direction[motor_index] =
                        derivative_direction;
                }
                else if (context->planned_direction[motor_index] !=
                         derivative_direction)
                {
                    return ARM_CONTROL_DIRECTION_REVERSAL;
                }
            }

            second_derivative =
                (derivative - previous_derivative[motor_index]) / du;
            if (!isfinite(second_derivative))
            {
                return ARM_CONTROL_TRAJECTORY_ERROR;
            }
            absolute_value = fabsf(second_derivative);
            if ((derivative * second_derivative) >= 0.0f)
            {
                if (absolute_value > maximum_acceleration[motor_index])
                {
                    maximum_acceleration[motor_index] = absolute_value;
                }
            }
            else if (absolute_value > maximum_deceleration[motor_index])
            {
                maximum_deceleration[motor_index] = absolute_value;
            }

            previous_derivative[motor_index] = derivative;
        }

        previous_joints = joints;
        previous_coordinates = coordinates;
    }

    /* 五次时间律终点速度为零，把最后一个离散区间也计入减速约束。 */
    for (motor_index = 0U;
         motor_index < ARM_CONTROL_MOTOR_COUNT;
         ++motor_index)
    {
        second_derivative = -previous_derivative[motor_index] / du;
        absolute_value = fabsf(second_derivative);
        if (absolute_value > maximum_deceleration[motor_index])
        {
            maximum_deceleration[motor_index] = absolute_value;
        }

        result->max_speed_rpm[motor_index] =
            maximum_derivative[motor_index] /
            (ARM_CONTROL_DEG_PER_RPM_S * trajectory->duration_s);
        result->max_acc_rpm_per_s[motor_index] =
            maximum_acceleration[motor_index] /
            (ARM_CONTROL_DEG_PER_RPM_S * trajectory->duration_s *
             trajectory->duration_s);
        result->max_dec_rpm_per_s[motor_index] =
            maximum_deceleration[motor_index] /
            (ARM_CONTROL_DEG_PER_RPM_S * trajectory->duration_s *
             trajectory->duration_s);
    }

    /*
     * 速度按 1/T、加减速度按 1/T^2 缩放，直接反算最短时间；
     * 不枚举 A/D/V 组合。0.85 等安全系数给反馈修正和协议量化留余量。
     */
    required_time_s = 0.0f;
    for (motor_index = 0U;
         motor_index < ARM_CONTROL_MOTOR_COUNT;
         ++motor_index)
    {
        safe_limit = config->vel_rpm[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = maximum_derivative[motor_index] /
                           (ARM_CONTROL_DEG_PER_RPM_S * safe_limit);
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }

        safe_limit = (float)config->acc_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = sqrtf(maximum_acceleration[motor_index] /
                                 (ARM_CONTROL_DEG_PER_RPM_S * safe_limit));
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }

        safe_limit = (float)config->dec_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = sqrtf(maximum_deceleration[motor_index] /
                                 (ARM_CONTROL_DEG_PER_RPM_S * safe_limit));
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }
    }

    result->minimum_time_s = required_time_s;
    result->failed_sample = 0U;
    context->final_joints = joints;
    context->final_coordinates = coordinates;
    if (trajectory->duration_s <
        (required_time_s - (float)ARM_CONTROL_TIME_EPS_S))
    {
        return ARM_CONTROL_TIME_UNACHIEVABLE;
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 按实际执行周期检查离散曲线指令，拒绝超限、反向或不安全的末速。
  调用示例: ArmControl_ValidateDiscreteTrajectory(config, trajectory);
  参数意义: config 为对应模块的配置结构。
            trajectory 为三次贝塞尔曲线结构。
*/
static ArmControlStatus ArmControl_ValidateDiscreteTrajectory(
    const ArmControlConfig *config,
    const ArmCubicBezierTrajectory *trajectory)
{
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmControlTrajectoryResult *result = &context->result;
    ArmJointAngles joints;
    ArmMotorCoordinates previous_coordinates;
    ArmMotorCoordinates coordinates;
    ArmTrajectoryStatus trajectory_status = ARM_TRAJECTORY_OK;
    ArmIkStatus ik_status = ARM_IK_OK;
    ArmControlStatus motion_status;
    float previous_command_rpm[ARM_CONTROL_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float maximum_speed[ARM_CONTROL_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_acceleration[ARM_CONTROL_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float maximum_deceleration[ARM_CONTROL_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float previous_elapsed_s;
    float elapsed_s;
    float interval_s;
    float average_rpm;
    float endpoint_rpm;
    float change_rpm_per_s;
    float previous_magnitude;
    float endpoint_magnitude;
    float safe_limit;
    float ratio;
    float candidate_time_s;
    uint32_t elapsed_ms;
    uint32_t sample_index;
    uint8_t motor_index;
    bool discrete_direction_invalid = false;
    bool discrete_limits_invalid = false;

    motion_status = ArmControl_SolveTrajectorySample(
        config,
        trajectory,
        0.0f,
        &joints,
        &previous_coordinates,
        &trajectory_status,
        &ik_status);
    if (motion_status != ARM_CONTROL_OK)
    {
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        return motion_status;
    }

    previous_elapsed_s = 0.0f;
    for (sample_index = 1U;
         sample_index <= trajectory->segment_count;
         ++sample_index)
    {
        elapsed_ms = sample_index *
                     (uint32_t)trajectory->sample_period_ms;
        if (elapsed_ms > trajectory->duration_ms)
        {
            elapsed_ms = trajectory->duration_ms;
        }
        elapsed_s = (float)elapsed_ms * 0.001f;
        interval_s = elapsed_s - previous_elapsed_s;
        if (!isfinite(interval_s) || (interval_s <= 0.0f))
        {
            result->failed_sample = sample_index;
            return ARM_CONTROL_TIME_INVALID;
        }

        motion_status = ArmControl_SolveTrajectorySample(
            config,
            trajectory,
            elapsed_s,
            &joints,
            &coordinates,
            &trajectory_status,
            &ik_status);
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        result->failed_sample = sample_index;
        if (motion_status != ARM_CONTROL_OK)
        {
            return motion_status;
        }

        for (motor_index = 0U;
             motor_index < ARM_CONTROL_MOTOR_COUNT;
             ++motor_index)
        {
            average_rpm =
                (coordinates.degree[motor_index] -
                 previous_coordinates.degree[motor_index]) /
                ((float)ARM_CONTROL_DEG_PER_RPM_S * interval_s);
            endpoint_rpm = (2.0f * average_rpm) -
                           previous_command_rpm[motor_index];
            if (!isfinite(endpoint_rpm))
            {
                return ARM_CONTROL_TRAJECTORY_ERROR;
            }

            if (((context->planned_direction[motor_index] > 0) &&
                 (endpoint_rpm < -ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM)) ||
                ((context->planned_direction[motor_index] < 0) &&
                 (endpoint_rpm > ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM)))
            {
                /*
                 * 真实路径换向已由连续预检拒绝。此处的反号是离散
                 * 末速递推在当前时间网格上不可实现，可通过增加 T 解决。
                 */
                discrete_direction_invalid = true;
            }

            previous_magnitude =
                fabsf(previous_command_rpm[motor_index]);
            endpoint_magnitude = fabsf(endpoint_rpm);
            if (endpoint_magnitude > maximum_speed[motor_index])
            {
                maximum_speed[motor_index] = endpoint_magnitude;
            }
            change_rpm_per_s = fabsf(
                endpoint_rpm - previous_command_rpm[motor_index]) /
                interval_s;
            if (endpoint_magnitude >= previous_magnitude)
            {
                if (change_rpm_per_s >
                    maximum_acceleration[motor_index])
                {
                    maximum_acceleration[motor_index] =
                        change_rpm_per_s;
                }
            }
            else if (change_rpm_per_s >
                     maximum_deceleration[motor_index])
            {
                maximum_deceleration[motor_index] = change_rpm_per_s;
            }
            previous_command_rpm[motor_index] = endpoint_rpm;
        }

        previous_coordinates = coordinates;
        previous_elapsed_s = elapsed_s;
    }

    for (motor_index = 0U;
         motor_index < ARM_CONTROL_MOTOR_COUNT;
         ++motor_index)
    {
        result->final_speed_rpm[motor_index] =
            previous_command_rpm[motor_index];
        if (maximum_speed[motor_index] >
            result->max_speed_rpm[motor_index])
        {
            result->max_speed_rpm[motor_index] =
                maximum_speed[motor_index];
        }
        if (maximum_acceleration[motor_index] >
            result->max_acc_rpm_per_s[motor_index])
        {
            result->max_acc_rpm_per_s[motor_index] =
                maximum_acceleration[motor_index];
        }
        if (maximum_deceleration[motor_index] >
            result->max_dec_rpm_per_s[motor_index])
        {
            result->max_dec_rpm_per_s[motor_index] =
                maximum_deceleration[motor_index];
        }

        safe_limit = config->vel_rpm[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_speed[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * ratio;
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        safe_limit = (float)config->acc_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_acceleration[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * sqrtf(ratio);
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        safe_limit = (float)config->dec_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_deceleration[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * sqrtf(ratio);
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        if ((maximum_speed[motor_index] >
             (config->vel_rpm[motor_index] *
               config->trajectory_limit_scale)) ||
            (maximum_acceleration[motor_index] >
             ((float)config->acc_rpm_per_s[motor_index] *
              config->trajectory_limit_scale)) ||
            (maximum_deceleration[motor_index] >
             ((float)config->dec_rpm_per_s[motor_index] *
              config->trajectory_limit_scale)) ||
            (fabsf(previous_command_rpm[motor_index]) >
             ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM))
        {
            discrete_limits_invalid = true;
        }
    }

    if (discrete_direction_invalid || discrete_limits_invalid)
    {
        return ARM_CONTROL_TIME_UNACHIEVABLE;
    }

    result->failed_sample = 0U;
    return ARM_CONTROL_OK;
}

/*
  函数说明: 把候选轨迹时长向上量化为合法采样周期的整数倍。
  调用示例: ArmControl_CandidateDurationMs(time_s, period_ms, maximum_ms, duration_ms);
  参数意义: time_s 为内部规划或曲线参考时长，秒。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
            maximum_ms 为允许的最大时长，毫秒。
            duration_ms 为输出量化后的时长，毫秒。
*/
static int ArmControl_CandidateDurationMs(double time_s,
                                         uint16_t period_ms,
                                         uint32_t maximum_ms,
                                         uint32_t *duration_ms)
{
    double value_ms;
    uint32_t rounded_ms;
    uint32_t remainder;
    uint32_t increment;

    if ((duration_ms == NULL) || !isfinite(time_s) || (time_s <= 0.0))
    {
        return 0;
    }

    value_ms = ceil(time_s * 1000.0 * ARM_CONTROL_MINIMUM_VERIFY_MARGIN);
    if ((value_ms < 1.0) || (value_ms > (double)maximum_ms))
    {
        return 0;
    }
    rounded_ms = (uint32_t)value_ms;
    remainder = rounded_ms % (uint32_t)period_ms;
    if (remainder != 0U)
    {
        increment = (uint32_t)period_ms - remainder;
        if (rounded_ms > (maximum_ms - increment))
        {
            return 0;
        }
        rounded_ms += increment;
    }
    *duration_ms = rounded_ms;
    return 1;
}

/*
  函数说明: 不枚举 A/D/V，也不逐毫秒搜索。由解析缩放给出候选，再最多复核三次；
  minimum_time_s 只记录已经在相同离散执行网格上实际验证通过的时间。
  调用示例: ArmControl_CertifyMinimumTime(config, requested_trajectory, requested_time_passed);
  参数意义: config 为对应模块的配置结构。
            requested_trajectory 为待验证的曲线参数。
            requested_time_passed 为原始曲线时长是否已经通过离散校验。
*/
static ArmControlStatus ArmControl_CertifyMinimumTime(
    const ArmControlConfig *config,
    const ArmCubicBezierTrajectory *requested_trajectory,
    bool requested_time_passed)
{
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmControlTrajectoryResult requested_result = context->result;
    ArmControlTrajectoryResult verification_result;
    ArmCubicBezierTrajectory candidate_trajectory;
    ArmTrajectoryStatus trajectory_status;
    ArmControlStatus motion_status;
    double candidate_time_s;
    double factor;
    double ratio;
    double safe_limit;
    uint32_t candidate_ms;
    uint32_t maximum_ms;
    uint8_t iteration;
    uint8_t motor_index;

    maximum_ms = (uint32_t)requested_trajectory->sample_period_ms *
                 (uint32_t)ARM_CONTROL_TRAJECTORY_MAX_SEGMENTS;
    candidate_time_s = (double)requested_result.minimum_time_s;
    if (!requested_time_passed &&
        (candidate_time_s < (double)requested_trajectory->duration_s))
    {
        candidate_time_s = (double)requested_trajectory->duration_s;
    }
    if (candidate_time_s <= 0.0)
    {
        candidate_time_s =
            (double)requested_trajectory->sample_period_ms * 0.001;
    }

    for (iteration = 0U;
         iteration < ARM_CONTROL_MINIMUM_VERIFY_ITERATIONS;
         ++iteration)
    {
        if (!ArmControl_CandidateDurationMs(
                candidate_time_s,
                requested_trajectory->sample_period_ms,
                maximum_ms,
                &candidate_ms))
        {
            break;
        }
        if (requested_time_passed &&
            (candidate_ms > requested_trajectory->duration_ms))
        {
            candidate_ms = requested_trajectory->duration_ms;
        }

        trajectory_status = ArmTrajectory_InitCubicBezier(
            &candidate_trajectory,
            &requested_trajectory->point[0],
            &requested_trajectory->point[1],
            &requested_trajectory->point[2],
            &requested_trajectory->point[3],
            (float)candidate_ms * 0.001f,
            requested_trajectory->sample_period_ms);
        if (trajectory_status != ARM_TRAJECTORY_OK)
        {
            context->result = requested_result;
            if (requested_time_passed)
            {
                context->result.minimum_time_s =
                    requested_trajectory->duration_s;
                return ARM_CONTROL_OK;
            }
            context->result.minimum_time_s = 0.0f;
            return ARM_CONTROL_TIME_INVALID;
        }

        verification_result = requested_result;
        memset(verification_result.max_speed_rpm,
               0,
               sizeof(verification_result.max_speed_rpm));
        memset(verification_result.max_acc_rpm_per_s,
               0,
               sizeof(verification_result.max_acc_rpm_per_s));
        memset(verification_result.max_dec_rpm_per_s,
               0,
               sizeof(verification_result.max_dec_rpm_per_s));
        memset(verification_result.final_speed_rpm,
               0,
               sizeof(verification_result.final_speed_rpm));
        verification_result.minimum_time_s = requested_result.minimum_time_s;
        context->result = verification_result;
        motion_status = ArmControl_ValidateDiscreteTrajectory(
            config, &candidate_trajectory);
        verification_result = context->result;
        if (motion_status == ARM_CONTROL_OK)
        {
            context->result = requested_result;
            context->result.minimum_time_s =
                candidate_trajectory.duration_s;
            return ARM_CONTROL_OK;
        }
        if (motion_status != ARM_CONTROL_TIME_UNACHIEVABLE)
        {
            context->result = requested_result;
            if (requested_time_passed)
            {
                context->result.minimum_time_s =
                    requested_trajectory->duration_s;
                return ARM_CONTROL_OK;
            }
            context->result.trajectory_status =
                verification_result.trajectory_status;
            context->result.ik_status = verification_result.ik_status;
            context->result.failed_sample = verification_result.failed_sample;
            context->result.minimum_time_s = 0.0f;
            return motion_status;
        }

        factor = 1.0;
        for (motor_index = 0U;
             motor_index < ARM_CONTROL_MOTOR_COUNT;
             ++motor_index)
        {
            safe_limit = (double)config->vel_rpm[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = (double)verification_result.max_speed_rpm[motor_index] /
                    safe_limit;
            if (ratio > factor) factor = ratio;

            safe_limit = (double)config->acc_rpm_per_s[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = sqrt((double)verification_result.
                         max_acc_rpm_per_s[motor_index] / safe_limit);
            if (ratio > factor) factor = ratio;

            safe_limit = (double)config->dec_rpm_per_s[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = sqrt((double)verification_result.
                         max_dec_rpm_per_s[motor_index] / safe_limit);
            if (ratio > factor) factor = ratio;

            ratio = fabs((double)verification_result.
                         final_speed_rpm[motor_index]) /
                    (double)ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM;
            if (ratio > factor) factor = ratio;
        }
        candidate_time_s = (double)candidate_trajectory.duration_s * factor;
    }

    if (requested_time_passed)
    {
        /* 当前用户时间刚刚已由同一离散校验通过，本身就是可靠上界。 */
        context->result = requested_result;
        context->result.minimum_time_s = requested_trajectory->duration_s;
        return ARM_CONTROL_OK;
    }

    /* 三次解析修正仍失败时，仅复核一次执行器允许的最大时间。 */
    trajectory_status = ArmTrajectory_InitCubicBezier(
        &candidate_trajectory,
        &requested_trajectory->point[0],
        &requested_trajectory->point[1],
        &requested_trajectory->point[2],
        &requested_trajectory->point[3],
        (float)maximum_ms * 0.001f,
        requested_trajectory->sample_period_ms);
    context->result = requested_result;
    memset(context->result.max_speed_rpm,
           0,
           sizeof(context->result.max_speed_rpm));
    memset(context->result.max_acc_rpm_per_s,
           0,
           sizeof(context->result.max_acc_rpm_per_s));
    memset(context->result.max_dec_rpm_per_s,
           0,
           sizeof(context->result.max_dec_rpm_per_s));
    memset(context->result.final_speed_rpm,
           0,
           sizeof(context->result.final_speed_rpm));
    if (trajectory_status == ARM_TRAJECTORY_OK)
    {
        motion_status = ArmControl_ValidateDiscreteTrajectory(
            config, &candidate_trajectory);
        verification_result = context->result;
    }
    else
    {
        motion_status = ARM_CONTROL_TIME_INVALID;
        verification_result = requested_result;
        verification_result.trajectory_status = trajectory_status;
        verification_result.failed_sample = 0U;
    }
    context->result = requested_result;
    if (motion_status == ARM_CONTROL_OK)
    {
        context->result.minimum_time_s = candidate_trajectory.duration_s;
        return ARM_CONTROL_OK;
    }
    context->result.minimum_time_s = 0.0f;
    if (motion_status != ARM_CONTROL_TIME_UNACHIEVABLE)
    {
        context->result.trajectory_status =
            verification_result.trajectory_status;
        context->result.ik_status = verification_result.ik_status;
        context->result.failed_sample = verification_result.failed_sample;
    }
    return (motion_status == ARM_CONTROL_TIME_UNACHIEVABLE) ?
           ARM_CONTROL_TIME_UNACHIEVABLE : motion_status;
}

/*
  函数说明: 在调用者已持有配置锁时设置三台驱动器的周期回传。
  调用示例: ArmControl_SetTimedReturnInfoUnlocked(config, info_function, period_ms);
  参数意义: config 为对应模块的配置结构。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
static ArmControlStatus ArmControl_SetTimedReturnInfoUnlocked(
    const ArmControlConfig *config,
    uint8_t info_function,
    uint16_t period_ms)
{
    uint8_t index;

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Timed_Return_Info(config->motor_addr[index],
                                   info_function,
                                   period_ms) != HAL_OK)
        {
            return ARM_CONTROL_CAN_ERROR;
        }
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 配置曲线所需的周期位置、速度、状态反馈并检查停止代次。
  调用示例: ArmControl_SetTrajectoryFeedback(config, enable, expected_generation, enforce_generation);
  参数意义: config 为对应模块的配置结构。
            enable 为是否启用。
            expected_generation 为预期命令代次。
            enforce_generation 为是否强制检查停止代次。
*/
static ArmControlStatus ArmControl_SetTrajectoryFeedback(
    const ArmControlConfig *config,
    bool enable,
    uint32_t expected_generation,
    bool enforce_generation)
{
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmControlStatus result = ARM_CONTROL_OK;
    /* 固定按位置、速度、状态、误差的原顺序配置，不遗漏任何一种反馈。 */
    static const uint8_t info_functions[4] = {
        X_V2_INFO_REALTIME_POSITION, X_V2_INFO_REALTIME_SPEED,
        X_V2_INFO_STATUS, X_V2_INFO_POSITION_ERROR
    };
    static const uint16_t info_periods_ms[4] = {
        ARM_CONTROL_TRAJECTORY_POSITION_PERIOD_MS,
        ARM_CONTROL_TRAJECTORY_SPEED_PERIOD_MS,
        ARM_CONTROL_TRAJECTORY_STATUS_PERIOD_MS,
        ARM_CONTROL_TRAJECTORY_ERROR_PERIOD_MS
    };
    uint32_t deadline_ms;
    uint8_t drain_index;
    uint8_t motor_index;
    uint8_t info_index;
    bool all_acknowledged;

    if (!ArmControl_FeedbackConfigLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    if (enable && enforce_generation &&
        !ArmControl_CommandTokenIsCurrent(expected_generation))
    {
        ArmControl_FeedbackConfigUnlock();
        return ARM_CONTROL_CANCELLED;
    }
    if (!enable && !context->feedback_stream_enabled)
    {
        ArmControl_FeedbackConfigUnlock();
        return ARM_CONTROL_OK;
    }
    if (enable)
    {
        /* 部分发送失败时也保留 true，强制 Stop cleanup 再发 period=0。 */
        context->feedback_stream_enabled = true;
    }

    if (!enable)
    {
        /* 先解析现有队列，不能让 flush 隐藏已经到达的驱动器故障帧。 */
        for (drain_index = 0U; drain_index < 3U; ++drain_index)
        {
            ArmControl_DrainRx(config, 0U);
        }
        (void)ZDT_CAN_FlushRx();
        memset(context->timed_stop_ack_count,
               0,
               sizeof(context->timed_stop_ack_count));
        context->timed_stop_request_ms = HAL_GetTick();
        context->timed_stop_waiting = false;
        osDelay(1U);
        for (drain_index = 0U; drain_index < 3U; ++drain_index)
        {
            ArmControl_DrainRx(config, 0U);
        }
        (void)ZDT_CAN_FlushRx();
        context->timed_stop_waiting = true;
    }

    for (info_index = 0U; info_index < 4U; ++info_index)
    {
        if (ArmControl_SetTimedReturnInfoUnlocked(
                config, info_functions[info_index],
                enable ? info_periods_ms[info_index] : 0U) != ARM_CONTROL_OK)
        {
            context->timed_stop_waiting = false;
            result = ARM_CONTROL_CAN_ERROR;
            goto done;
        }
    }

    if (enable)
    {
        if (enforce_generation &&
            !ArmControl_CommandTokenIsCurrent(expected_generation))
        {
            result = ARM_CONTROL_CANCELLED;
        }
        goto done;
    }

    /*
     * 手册 5.5.1：period=0 时每条停止命令返回 Addr 11 02 6B。
     * 共有四类信息、三台电机；12 个确认全部收到才证明定时流已停。
     */
    deadline_ms = HAL_GetTick() + ARM_CONTROL_TIMED_STOP_ACK_TIMEOUT_MS;
    for (;;)
    {
        ArmControl_DrainRx(config, 5U);
        all_acknowledged = true;
        for (motor_index = 0U;
             motor_index < ARM_CONTROL_MOTOR_COUNT;
             ++motor_index)
        {
            if (context->timed_stop_ack_count[motor_index] <
                ARM_CONTROL_TIMED_STOP_ACKS_PER_MOTOR)
            {
                all_acknowledged = false;
                break;
            }
        }
        if (all_acknowledged)
        {
            /* 继续清空剩余队列，确保停止前已到达的故障帧不会被忽略。 */
            for (drain_index = 0U; drain_index < 3U; ++drain_index)
            {
                ArmControl_DrainRx(config, 0U);
            }
            context->timed_stop_waiting = false;
            context->feedback_stream_enabled = false;
            result = ARM_CONTROL_OK;
            goto done;
        }
        if (ArmControl_TimeReached(HAL_GetTick(), deadline_ms))
        {
            context->timed_stop_waiting = false;
            result = ARM_CONTROL_FEEDBACK_TIMEOUT;
            goto done;
        }
    }

done:
    ArmControl_FeedbackConfigUnlock();
    return result;
}

/*
  函数说明: 设置曲线驱动器心跳超时，配置过程中检查停止代次。
  调用示例: ArmControl_SetTrajectoryHeartbeat(config, timeout_ms, expected_generation, enforce_generation);
  参数意义: config 为对应模块的配置结构。
            timeout_ms 为等待或心跳超时，毫秒。
            expected_generation 为预期命令代次。
            enforce_generation 为是否强制检查停止代次。
*/
static ArmControlStatus ArmControl_SetTrajectoryHeartbeat(
    const ArmControlConfig *config,
    uint32_t timeout_ms,
    uint32_t expected_generation,
    bool enforce_generation)
{
    ArmControlStatus result = ARM_CONTROL_OK;
    uint8_t index;

    if (!ArmControl_FeedbackConfigLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    if ((timeout_ms != 0U) && enforce_generation &&
        !ArmControl_CommandTokenIsCurrent(expected_generation))
    {
        ArmControl_FeedbackConfigUnlock();
        return ARM_CONTROL_CANCELLED;
    }
    if ((timeout_ms == 0U) &&
        !arm_control_trajectory_context.heartbeat_enabled &&
        !arm_control_trajectory_context.heartbeat_disable_pending)
    {
        ArmControl_FeedbackConfigUnlock();
        return ARM_CONTROL_OK;
    }
    if (timeout_ms != 0U)
    {
        /* 部分发送失败时保留 true，Stop cleanup 必须重试 timeout=0。 */
        arm_control_trajectory_context.heartbeat_enabled = true;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Set_Heartbeat(config->motor_addr[index],
                               false,
                               timeout_ms) != HAL_OK)
        {
            result = ARM_CONTROL_CAN_ERROR;
            break;
        }
    }
    if ((result == ARM_CONTROL_OK) && (timeout_ms != 0U) &&
        enforce_generation &&
        !ArmControl_CommandTokenIsCurrent(expected_generation))
    {
        result = ARM_CONTROL_CANCELLED;
    }
    ArmControl_FeedbackConfigUnlock();
    return result;
}

/*
  函数说明: 尝试关闭曲线的周期回传和心跳配置，并保留清理错误。
  调用示例: ArmControl_TrajectoryBestEffortCleanup(config);
  参数意义: config 为对应模块的配置结构。
*/
static ArmControlStatus ArmControl_TrajectoryBestEffortCleanup(
    const ArmControlConfig *config)
{
    ArmControlStatus cleanup_status = ARM_CONTROL_OK;

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }

    if (ArmControl_SetTrajectoryHeartbeat(
            config, 0U, 0U, false) == ARM_CONTROL_OK)
    {
        arm_control_trajectory_context.heartbeat_enabled = false;
        arm_control_trajectory_context.heartbeat_disable_pending = false;
    }
    else
    {
        cleanup_status = ARM_CONTROL_CAN_ERROR;
    }
    if (ArmControl_SetTrajectoryFeedback(
            config, false, 0U, false) == ARM_CONTROL_OK)
    {
        arm_control_trajectory_context.feedback_stream_enabled = false;
    }
    else
    {
        cleanup_status = ARM_CONTROL_CAN_ERROR;
    }
    return cleanup_status;
}

/*
  函数说明: 记录曲线错误并执行停止及资源清理。
  调用示例: ArmControl_TrajectoryFault(cause);
  参数意义: cause 为曲线故障原因。
*/
static ArmControlStatus ArmControl_TrajectoryFault(
    ArmControlStatus cause)
{
    ArmControlStatus stop_status;
    ArmControlConfig *config = arm_control_active_config;

    if (config != NULL)
    {
        stop_status = ArmControl_StopAll(config);
        if ((stop_status != ARM_CONTROL_OK) &&
            (cause != ARM_CONTROL_CAN_ERROR))
        {
            cause = ARM_CONTROL_CAN_ERROR;
        }
    }

    arm_control_trajectory_context.result.state =
        ARM_CONTROL_TRAJECTORY_FAULT;
    arm_control_trajectory_context.result.status = cause;
    arm_control_current_valid = false;
    return cause;
}

/*
  函数说明: 检查三台位置反馈均有效且未超时。
  调用示例: ArmControl_AllPositionFeedbackFresh(config, now_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
*/
static bool ArmControl_AllPositionFeedbackFresh(
    const ArmControlConfig *config,
    uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].realtime_position_valid ||
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        realtime_position_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 要求三台位置反馈均新鲜且产生于指定时刻之后。
  调用示例: ArmControl_AllPositionFeedbackFreshSince(config, now_ms, not_before_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
            not_before_ms 为反馈必须晚于此时刻，毫秒。
*/
static bool ArmControl_AllPositionFeedbackFreshSince(
    const ArmControlConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    if (!ArmControl_AllPositionFeedbackFresh(config, now_ms))
    {
        return false;
    }
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!ArmControl_TimeAfter(
                arm_control_driver_status[index].realtime_position_update_ms,
                not_before_ms))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 依据指定时刻后的新鲜速度反馈确认三轴已停稳。
  调用示例: ArmControl_AllMotorsStopped(config, now_ms, not_before_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
            not_before_ms 为反馈必须晚于此时刻，毫秒。
*/
static bool ArmControl_AllMotorsStopped(
    const ArmControlConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].realtime_speed_valid ||
            !ArmControl_TimeAfter(
                arm_control_driver_status[index].realtime_speed_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        realtime_speed_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms) ||
            (fabsf(arm_control_driver_status[index].realtime_speed_rpm) >
             ARM_CONTROL_TRAJECTORY_SPEED_ZERO_RPM))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 检查三台速度反馈均新鲜且晚于指定时刻。
  调用示例: ArmControl_AllSpeedFeedbackFreshSince(config, now_ms, not_before_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
            not_before_ms 为反馈必须晚于此时刻，毫秒。
*/
static bool ArmControl_AllSpeedFeedbackFreshSince(
    const ArmControlConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].realtime_speed_valid ||
            !ArmControl_TimeAfter(
                arm_control_driver_status[index].realtime_speed_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        realtime_speed_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 检查三台状态反馈均新鲜且晚于指定时刻。
  调用示例: ArmControl_AllStatusFeedbackFreshSince(config, now_ms, not_before_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
            not_before_ms 为反馈必须晚于此时刻，毫秒。
*/
static bool ArmControl_AllStatusFeedbackFreshSince(
    const ArmControlConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (!arm_control_driver_status[index].status_valid ||
            !ArmControl_TimeAfter(
                arm_control_driver_status[index].status_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms -
                        arm_control_driver_status[index].status_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

/*
  函数说明: 停车后限时核验实际速度、故障与位置，恢复下一次运动的可信起点。
  调用示例: ArmControl_RecoverAfterTimeout(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_RecoverAfterTimeout(const ArmControlConfig *config)
{
    ArmControlStatus status = ARM_CONTROL_FEEDBACK_TIMEOUT;
    ArmCartesianPosition endpoint;
    uint32_t request_generation;
    uint32_t start_ms;
    uint32_t barrier_ms;
    uint32_t next_poll_ms;
    uint32_t now_ms;
    uint8_t index;
    uint8_t recovery_phase = 0U;
    bool query_failed = false;
    bool status_fault;

    if (config == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_TryCaptureCommandToken(&request_generation))
    {
        status = ARM_CONTROL_CANCELLED;
        goto done;
    }
    if (!ArmControl_CommandLock())
    {
        status = ARM_CONTROL_CAN_ERROR;
        goto done;
    }
    if ((arm_control_active_config != config) ||
        ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
    {
        ArmControl_CommandUnlock();
        status = ARM_CONTROL_BUSY;
        goto done;
    }
    /* 只有停稳后的实测位置才能重新成为定时绝对位置运动的起点。 */
    arm_control_current_valid = false;
    ArmControl_CommandUnlock();

    /* 清理停机前的旧帧，用接收时间建立反馈屏障。 */
    ArmControl_DrainRx(config, 0U);
    ArmControl_DrainRx(config, 0U);
    start_ms = HAL_GetTick();
    barrier_ms = start_ms;
    next_poll_ms = start_ms + 1U;

    for (;;)
    {
        if (!ArmControl_CommandTokenIsCurrent(request_generation))
        {
            status = ARM_CONTROL_CANCELLED;
            break;
        }

        ArmControl_DrainRx(config, 10U);
        now_ms = HAL_GetTick();
        status_fault = false;
        for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
        {
            const ArmControlDriverStatus *driver =
                &arm_control_driver_status[index];
            if (driver->status_valid &&
                ArmControl_TimeAfter(driver->status_update_ms, start_ms) &&
                (((driver->status_flags & ARM_CONTROL_DRIVER_FAULT_MASK) != 0U) ||
                 ((driver->status_flags & 0x01U) == 0U)))
            {
                status_fault = true;
            }
        }
        if (status_fault)
        {
            status = ARM_CONTROL_DRIVER_ERROR;
            break;
        }

        if ((recovery_phase == 0U) &&
            ArmControl_AllSpeedFeedbackFreshSince(config, now_ms, barrier_ms) &&
            ArmControl_AllStatusFeedbackFreshSince(config, now_ms, barrier_ms) &&
            ArmControl_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            /* 首次读到零速后，再隔一段时间确认一次，防止瞬态零速。 */
            recovery_phase = 1U;
            barrier_ms = now_ms;
            next_poll_ms = now_ms + ARM_CONTROL_TIMEOUT_RECOVERY_STABLE_MS;
        }
        else if ((recovery_phase == 1U) &&
                 ((uint32_t)(now_ms - barrier_ms) >=
                  ARM_CONTROL_TIMEOUT_RECOVERY_STABLE_MS) &&
                 ArmControl_AllSpeedFeedbackFreshSince(config, now_ms,
                                                      barrier_ms) &&
                 ArmControl_AllStatusFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmControl_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            /* 35 两次确认停稳后，才建立 36 的新位置屏障。 */
            recovery_phase = 2U;
            barrier_ms = now_ms;
            next_poll_ms = now_ms + 1U;
        }
        else if ((recovery_phase == 2U) &&
                 ArmControl_AllSpeedFeedbackFreshSince(config, now_ms,
                                                      barrier_ms) &&
                 ArmControl_AllStatusFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmControl_AllPositionFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmControl_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            if (!ArmControl_CommandLock())
            {
                status = ARM_CONTROL_CAN_ERROR;
                break;
            }
            if (!ArmControl_CommandTokenIsCurrent(request_generation) ||
                (arm_control_active_config != config) ||
                ArmControl_TrajectoryIsBusy() || arm_control_move_tracker.active)
            {
                status = ARM_CONTROL_CANCELLED;
            }
            else
            {
                status = ArmControl_SyncCurrentFromFeedback(config, now_ms);
                if (status == ARM_CONTROL_OK)
                {
                    /* 实际关节越界时，不允许从该姿态发起下一条 XYZ。 */
                    status = ArmControl_CurrentEndpoint(config, &endpoint);
                }
            }
            if (status != ARM_CONTROL_OK)
            {
                arm_control_current_valid = false;
            }
            ArmControl_CommandUnlock();
            break;
        }

        if ((uint32_t)(now_ms - start_ms) >=
            ARM_CONTROL_TIMEOUT_RECOVERY_MS)
        {
            status = query_failed ? ARM_CONTROL_CAN_ERROR :
                                    ARM_CONTROL_FEEDBACK_TIMEOUT;
            break;
        }

        if (ArmControl_TimeReached(now_ms, next_poll_ms))
        {
            for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
            {
                if (X_V2_Read_Info(config->motor_addr[index],
                                   X_V2_INFO_REALTIME_SPEED) != HAL_OK)
                {
                    query_failed = true;
                }
                if (X_V2_Read_Info(config->motor_addr[index],
                                   X_V2_INFO_STATUS) != HAL_OK)
                {
                    query_failed = true;
                }
                if ((recovery_phase == 2U) &&
                    (X_V2_Read_Info(config->motor_addr[index],
                                    X_V2_INFO_REALTIME_POSITION) != HAL_OK))
                {
                    query_failed = true;
                }
            }
            next_poll_ms = HAL_GetTick() +
                           ARM_CONTROL_TIMEOUT_RECOVERY_POLL_MS;
        }
        osDelay(5U);
    }

done:
    ArmControl_TrajectoryApiUnlock();
    return status;
}

/*
  函数说明: 结合当前速度和减速上限计算曲线停止等待时限。
  调用示例: ArmControl_TrajectoryStopTimeoutMs(config);
  参数意义: config 为对应模块的配置结构。
*/
static uint32_t ArmControl_TrajectoryStopTimeoutMs(
    const ArmControlConfig *config)
{
    double maximum_time_s = 0.0;
    double axis_time_s;
    uint32_t timeout_ms;
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        axis_time_s = fabs((double)arm_control_trajectory_context.
                           previous_command_rpm[index]) /
                      (double)config->dec_rpm_per_s[index];
        if (axis_time_s > maximum_time_s)
        {
            maximum_time_s = axis_time_s;
        }
    }

    timeout_ms = (uint32_t)ceil(maximum_time_s * 1000.0);
    timeout_ms += (uint32_t)ARM_CONTROL_TRAJECTORY_SPEED_PERIOD_MS;
    timeout_ms += (uint32_t)config->trajectory_feedback_timeout_ms;
    return timeout_ms;
}

/*
  函数说明: 向三台机械臂驱动器请求一次实际位置。
  调用示例: ArmControl_RequestTrajectoryPosition(config);
  参数意义: config 为对应模块的配置结构。
*/
static ArmControlStatus ArmControl_RequestTrajectoryPosition(
    const ArmControlConfig *config)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Read_Info(config->motor_addr[index],
                           X_V2_INFO_REALTIME_POSITION) != HAL_OK)
        {
            return ARM_CONTROL_CAN_ERROR;
        }
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 向三台机械臂驱动器请求一次当前状态。
  调用示例: ArmControl_RequestTrajectoryStatus(config);
  参数意义: config 为对应模块的配置结构。
*/
static ArmControlStatus ArmControl_RequestTrajectoryStatus(
    const ArmControlConfig *config)
{
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Read_Info(config->motor_addr[index],
                           X_V2_INFO_STATUS) != HAL_OK)
        {
            return ARM_CONTROL_CAN_ERROR;
        }
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 由相邻曲线采样点计算带反馈修正的三轴速度，并校验后统一下发。
  调用示例: ArmControl_SendTrajectoryVelocity(config, reference, next_reference, interval_s, now_ms);
  参数意义: config 为对应模块的配置结构。
            reference 为当前曲线参考电机坐标。
            next_reference 为下一曲线采样的连续电机坐标。
            interval_s 为相邻采样的实际间隔，秒。
            now_ms 为当前 HAL 毫秒时刻。
*/
static ArmControlStatus ArmControl_SendTrajectoryVelocity(
    const ArmControlConfig *config,
    const ArmMotorCoordinates *reference,
    const ArmMotorCoordinates *next_reference,
    float interval_s,
    uint32_t now_ms)
{
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    X_V2_Vel_Command_t commands[ARM_CONTROL_MOTOR_COUNT];
    uint8_t index;
    float actual_degrees;
    float estimated_degrees;
    float age_s;
    float extrapolation_rpm;
    float position_error;
    float correction_rpm;
    float feedforward_rpm;
    float desired_rpm;
    float desired_magnitude;
    float previous_magnitude;
    float allowed_lower_rpm;
    float allowed_upper_rpm;
    float quantized_rpm;
    float acceleration_rpm_per_s;
    uint16_t change_limit;

    if ((config == NULL) || (reference == NULL) ||
        (next_reference == NULL) || !isfinite(interval_s) ||
        (interval_s <= 0.0f))
    {
        return ARM_CONTROL_NULL_POINTER;
    }

    if (!ArmControl_AllPositionFeedbackFresh(config, now_ms))
    {
        return ARM_CONTROL_FEEDBACK_TIMEOUT;
    }

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        actual_degrees = ArmControl_LogicalPositionDegrees(
            config,
            index,
            arm_control_driver_status[index].realtime_position_deg);
        age_s = (float)(now_ms - arm_control_driver_status[index].
                        realtime_position_update_ms) * 0.001f;
        extrapolation_rpm = context->previous_command_rpm[index];
        if (arm_control_driver_status[index].realtime_speed_valid &&
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        realtime_speed_update_ms) <=
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            extrapolation_rpm = ArmControl_LogicalPositionDegrees(
                config,
                index,
                arm_control_driver_status[index].realtime_speed_rpm);
        }
        estimated_degrees = actual_degrees +
            (extrapolation_rpm *
             (float)ARM_CONTROL_DEG_PER_RPM_S * age_s);
        position_error = reference->degree[index] - estimated_degrees;
        if (!isfinite(position_error) ||
            (fabsf(position_error) >
             config->trajectory_feedback_error_limit_deg[index]))
        {
            return ARM_CONTROL_DRIVER_ERROR;
        }

        if (arm_control_driver_status[index].position_error_valid &&
            ((uint32_t)(now_ms - arm_control_driver_status[index].
                        position_error_update_ms) <=
             (uint32_t)config->trajectory_feedback_timeout_ms) &&
            (fabsf(arm_control_driver_status[index].position_error_deg) >
             config->trajectory_feedback_error_limit_deg[index]))
        {
            return ARM_CONTROL_DRIVER_ERROR;
        }

        /*
         * F6 会按 acc 从上一速度线性爬升到新速度。令区间末速
         * v1=2*delta/dt-v0，使 (v0+v1)/2 的积分正好等于本段位移。
         */
        feedforward_rpm =
            (2.0f *
             (next_reference->degree[index] - reference->degree[index]) /
             ((float)ARM_CONTROL_DEG_PER_RPM_S * interval_s)) -
            context->previous_command_rpm[index];
        correction_rpm = 2.0f *
                         config->trajectory_kp_rpm_per_deg[index] *
                         position_error;
        if (correction_rpm >
            config->trajectory_correction_limit_rpm[index])
        {
            correction_rpm =
                config->trajectory_correction_limit_rpm[index];
        }
        else if (correction_rpm <
                 -config->trajectory_correction_limit_rpm[index])
        {
            correction_rpm =
                -config->trajectory_correction_limit_rpm[index];
        }
        desired_rpm = feedforward_rpm + correction_rpm;

        /* 第一版轨迹不允许单轴中途换向，反馈修正也不能越过零速。 */
        if ((context->planned_direction[index] > 0) &&
            (desired_rpm < 0.0f))
        {
            desired_rpm = 0.0f;
        }
        else if ((context->planned_direction[index] < 0) &&
                 (desired_rpm > 0.0f))
        {
            desired_rpm = 0.0f;
        }
        else if (context->planned_direction[index] == 0)
        {
            desired_rpm = 0.0f;
        }

        previous_magnitude = fabsf(context->previous_command_rpm[index]);
        allowed_lower_rpm = previous_magnitude -
            ((float)config->dec_rpm_per_s[index] * interval_s);
        if (allowed_lower_rpm < 0.0f)
        {
            allowed_lower_rpm = 0.0f;
        }
        allowed_upper_rpm = previous_magnitude +
            ((float)config->acc_rpm_per_s[index] * interval_s);
        if (allowed_upper_rpm > config->vel_rpm[index])
        {
            allowed_upper_rpm = config->vel_rpm[index];
        }

        desired_magnitude = fabsf(desired_rpm);
        if (desired_magnitude < allowed_lower_rpm)
        {
            desired_magnitude = allowed_lower_rpm;
        }
        if (desired_magnitude > allowed_upper_rpm)
        {
            desired_magnitude = allowed_upper_rpm;
        }

        /*
         * 协议只能表示 0.1 RPM。量化后再次投影到本周期 A/D 可达区间；
         * 如果区间内没有新的 0.1 网格点，就保持上一条速度。
         */
        quantized_rpm = floorf((desired_magnitude * 10.0f) + 0.5f) /
                        10.0f;
        if (quantized_rpm >
            (allowed_upper_rpm +
             ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM))
        {
            quantized_rpm = floorf(
                (allowed_upper_rpm +
                 ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM) * 10.0f) /
                10.0f;
        }
        if (quantized_rpm <
            (allowed_lower_rpm -
             ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM))
        {
            quantized_rpm = ceilf(
                (allowed_lower_rpm -
                 ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM) * 10.0f) /
                10.0f;
        }
        if ((quantized_rpm <
             (allowed_lower_rpm -
              ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM)) ||
            (quantized_rpm >
             (allowed_upper_rpm +
              ARM_CONTROL_TRAJECTORY_QUANTIZATION_EPS_RPM)))
        {
            quantized_rpm = previous_magnitude;
        }
        desired_magnitude = quantized_rpm;
        desired_rpm = (context->planned_direction[index] < 0) ?
                      -desired_magnitude : desired_magnitude;
        change_limit = (desired_magnitude >= previous_magnitude) ?
                       config->acc_rpm_per_s[index] :
                       config->dec_rpm_per_s[index];
        acceleration_rpm_per_s =
            fabsf(desired_magnitude - previous_magnitude) / interval_s;
        if (acceleration_rpm_per_s < 1.0f)
        {
            acceleration_rpm_per_s = 1.0f;
        }
        if (acceleration_rpm_per_s > (float)change_limit)
        {
            acceleration_rpm_per_s = (float)change_limit;
        }

        commands[index].addr = config->motor_addr[index];
        commands[index].dir = ArmControl_VelocityDirection(
            config, index, desired_rpm);
        commands[index].acc_rpm_per_s =
            (uint16_t)ceilf(acceleration_rpm_per_s);
        commands[index].vel_rpm = desired_magnitude;
        commands[index].snF = false;
        context->previous_command_rpm[index] = desired_rpm;
    }

    if (X_V2_MultiMotor_Vel_Control(
            commands, ARM_CONTROL_MOTOR_COUNT) != HAL_OK)
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 向三个位置电机发送零速命令，开始曲线停止收尾。
  调用示例: ArmControl_SendTrajectoryZeroVelocity(config);
  参数意义: config 为对应模块的配置结构。
*/
static ArmControlStatus ArmControl_SendTrajectoryZeroVelocity(
    const ArmControlConfig *config)
{
    X_V2_Vel_Command_t commands[ARM_CONTROL_MOTOR_COUNT];
    uint8_t index;

    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        commands[index].addr = config->motor_addr[index];
        commands[index].dir = ArmControl_VelocityDirection(
            config,
            index,
            arm_control_trajectory_context.previous_command_rpm[index]);
        commands[index].acc_rpm_per_s = config->dec_rpm_per_s[index];
        commands[index].vel_rpm = 0.0f;
        commands[index].snF = false;
    }

    if (X_V2_MultiMotor_Vel_Control(
            commands, ARM_CONTROL_MOTOR_COUNT) != HAL_OK)
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    memset(arm_control_trajectory_context.previous_command_rpm,
           0,
           sizeof(arm_control_trajectory_context.previous_command_rpm));
    return ARM_CONTROL_OK;
}

/*
  函数说明: 用实际停止位置重新规划末端位置收尾，并登记到位反馈。
  调用示例: ArmControl_BeginTrajectoryFinalPosition(config, now_ms);
  参数意义: config 为对应模块的配置结构。
            now_ms 为当前 HAL 毫秒时刻。
*/
static ArmControlStatus ArmControl_BeginTrajectoryFinalPosition(
    ArmControlConfig *config,
    uint32_t now_ms)
{
    ArmControlStatus motion_status;
    double timeout_s;
    double timeout_ms_double;
    uint8_t index;

    motion_status = ArmControl_SyncCurrentFromFeedback(config, now_ms);
    if (motion_status != ARM_CONTROL_OK)
    {
        return motion_status;
    }

    motion_status = ArmControl_SetTrajectoryFeedback(
        config, false, 0U, false);
    if (motion_status != ARM_CONTROL_OK)
    {
        return motion_status;
    }
    arm_control_trajectory_context.feedback_stream_enabled = false;
    if (arm_control_trajectory_context.driver_error_mask != 0U)
    {
        return ARM_CONTROL_DRIVER_ERROR;
    }
    if ((zdt_can_rx_drop_count !=
         arm_control_trajectory_context.rx_drop_baseline) ||
        (zdt_can_error_count !=
         arm_control_trajectory_context.can_error_baseline))
    {
        return ARM_CONTROL_CAN_ERROR;
    }

    memset(&arm_control_last_result, 0, sizeof(arm_control_last_result));
    arm_control_last_ik_status = ARM_IK_OK;
    if (!ArmControl_CommandLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    if (!ArmControl_CommandTokenIsCurrent(
            arm_control_trajectory_context.generation))
    {
        ArmControl_CommandUnlock();
        arm_control_trajectory_context.result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    motion_status = ArmControl_MoveToJointsInternal(
        config,
        &arm_control_trajectory_context.final_joints,
        &arm_control_last_result,
        &arm_control_last_ik_status,
        config->trajectory_settle_time_s,
        arm_control_trajectory_context.generation,
        true);
    if (motion_status != ARM_CONTROL_OK)
    {
        ArmControl_CommandUnlock();
        return motion_status;
    }

    motion_status = ArmControl_CheckMoveCompletion(
        arm_control_trajectory_context.generation, true);
    if (motion_status != ARM_CONTROL_BUSY)
    {
        ArmControl_CommandUnlock();
        if (motion_status == ARM_CONTROL_CANCELLED)
        {
            arm_control_trajectory_context.result.minimum_time_s = 0.0f;
        }
        return motion_status;
    }

    /* 定时反馈关闭和 FD 分包发送可能耗时，超时从发送完成起算。 */
    now_ms = HAL_GetTick();

    timeout_s = (double)config->trajectory_settle_time_s;
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if ((double)arm_control_last_result.profile.predicted_time_s[index] >
            timeout_s)
        {
            timeout_s =
                (double)arm_control_last_result.profile.predicted_time_s[index];
        }
    }
    timeout_ms_double = ceil(timeout_s * 1000.0) +
                        (double)ARM_CONTROL_TRAJECTORY_SETTLE_MARGIN_MS;
    if ((timeout_ms_double <= 0.0) ||
        (timeout_ms_double > (double)ARM_CONTROL_MAX_OS_DELAY_TICKS))
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_TIME_INVALID;
    }

    arm_control_trajectory_context.settle_deadline_ms =
        now_ms + (uint32_t)timeout_ms_double;
    if (!ArmControl_TryPublishTrajectoryState(
            arm_control_trajectory_context.generation,
            ARM_CONTROL_TRAJECTORY_SETTLING_POSITION,
            ARM_CONTROL_BUSY))
    {
        ArmControl_CommandUnlock();
        arm_control_trajectory_context.result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    ArmControl_BeginDiagnostics(now_ms);
    ArmControl_CommandUnlock();
    return ARM_CONTROL_BUSY;
}

/*
  函数说明: 准备实际起点、校验曲线并启动非阻塞曲线状态机。
  调用示例: ArmControl_StartBezierInternal(control1, control2, target, time_s);
  参数意义: control1 为第一个贝塞尔控制点，mm。
            control2 为第二个贝塞尔控制点，mm。
            target 为目标值或目标位置。
            time_s 为内部规划或曲线参考时长，秒。
*/
static ArmControlStatus ArmControl_StartBezierInternal(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmControlConfig *config = arm_control_active_config;
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmCartesianPosition start;
    ArmTrajectoryStatus trajectory_status;
    ArmControlStatus motion_status;
    ArmControlStatus minimum_status;
    uint32_t now_ms;
    uint32_t duration_ms;
    uint32_t start_generation;
    uint8_t index;
    double duration_ms_value;

    if ((control1 == NULL) || (control2 == NULL) || (target == NULL))
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (config == NULL)
    {
        return ARM_CONTROL_NOT_INITIALIZED;
    }
    if (ArmControl_TrajectoryIsBusy())
    {
        return ARM_CONTROL_BUSY;
    }
    if (arm_control_move_tracker.active)
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        return ARM_CONTROL_NOT_ZEROED;
    }
    if (!config->use_multi_command)
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        return ARM_CONTROL_TIME_INVALID;
    }

    /* HAL/RTOS 的执行时基是 1 ms，规划时间统一量化到整数毫秒。 */
    duration_ms_value = floor(((double)time_s * 1000.0) + 0.5);
    if ((duration_ms_value < 1.0) ||
        (duration_ms_value > (double)UINT32_MAX))
    {
        return ARM_CONTROL_TIME_INVALID;
    }
    duration_ms = (uint32_t)duration_ms_value;
    time_s = (float)duration_ms * 0.001f;

    /*
     * 先与普通运动/配置切换串行，再原子认领 PRECHECKING，避免双 Start、
     * Stop 或 SetActiveConfig 在入口检查后穿透。
     */
    if (!ArmControl_CommandLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    if (arm_control_active_config != config)
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_ConfigIsValid(config))
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        ArmControl_CommandUnlock();
        return ARM_CONTROL_NOT_ZEROED;
    }
    taskENTER_CRITICAL();
    if ((arm_control_stop_depth != 0U) ||
        ArmControl_TrajectoryStateIsBusy(context->result.state) ||
        arm_control_move_tracker.active ||
        (arm_control_active_config != config))
    {
        taskEXIT_CRITICAL();
        ArmControl_CommandUnlock();
        return ARM_CONTROL_BUSY;
    }
    start_generation = arm_control_trajectory_generation + 1U;
    arm_control_trajectory_generation = start_generation;
    memset(context, 0, sizeof(*context));
    context->result.state = ARM_CONTROL_TRAJECTORY_PRECHECKING;
    context->result.status = ARM_CONTROL_BUSY;
    context->result.requested_time_s = time_s;
    context->result.target = *target;
    context->generation = start_generation;
    taskEXIT_CRITICAL();
    ArmControl_CommandUnlock();

    motion_status = ArmControl_CurrentEndpoint(config, &start);
    if (!ArmControl_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    if (motion_status != ARM_CONTROL_OK)
    {
        context->result.state = ARM_CONTROL_TRAJECTORY_FAULT;
        context->result.status = motion_status;
        return motion_status;
    }
    context->result.start = start;

    trajectory_status = ArmTrajectory_InitCubicBezier(
        &context->trajectory,
        &start,
        control1,
        control2,
        target,
        time_s,
        config->trajectory_period_ms);
    context->result.trajectory_status = trajectory_status;
    if (!ArmControl_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    if (trajectory_status != ARM_TRAJECTORY_OK)
    {
        context->result.state = ARM_CONTROL_TRAJECTORY_FAULT;
        context->result.status = ARM_CONTROL_TRAJECTORY_ERROR;
        return ARM_CONTROL_TRAJECTORY_ERROR;
    }

    /* 第一次预检使用最近一次已确认到位的位置，失败时不发送任何 CAN。 */
    motion_status = ArmControl_PreflightTrajectory(config,
                                                   &context->trajectory);
    if (!ArmControl_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    if (motion_status == ARM_CONTROL_OK)
    {
        motion_status = ArmControl_ValidateDiscreteTrajectory(
            config, &context->trajectory);
        if (!ArmControl_CommandTokenIsCurrent(start_generation))
        {
            context->result.minimum_time_s = 0.0f;
            return ARM_CONTROL_CANCELLED;
        }
    }
    if ((motion_status == ARM_CONTROL_OK) ||
        (motion_status == ARM_CONTROL_TIME_UNACHIEVABLE))
    {
        minimum_status = ArmControl_CertifyMinimumTime(
            config,
            &context->trajectory,
            motion_status == ARM_CONTROL_OK);
        if (minimum_status != ARM_CONTROL_OK)
        {
            motion_status = minimum_status;
        }
        if (!ArmControl_CommandTokenIsCurrent(start_generation))
        {
            context->result.minimum_time_s = 0.0f;
            return ARM_CONTROL_CANCELLED;
        }
    }
    if ((motion_status != ARM_CONTROL_OK) &&
        (motion_status != ARM_CONTROL_TIME_UNACHIEVABLE))
    {
        context->result.minimum_time_s = 0.0f;
    }
    if (motion_status != ARM_CONTROL_OK)
    {
        context->result.state = ARM_CONTROL_TRAJECTORY_FAULT;
        context->result.status = motion_status;
        return motion_status;
    }
    if (!ArmControl_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }

    (void)ZDT_CAN_FlushRx();
    memset(arm_control_driver_status, 0, sizeof(arm_control_driver_status));
    context->rx_drop_baseline = zdt_can_rx_drop_count;
    context->can_error_baseline = zdt_can_error_count;
    context->feedback_stream_enabled = true;
    motion_status = ArmControl_SetTrajectoryFeedback(
        config, true, start_generation, true);
    if (motion_status != ARM_CONTROL_OK)
    {
        return ArmControl_TrajectoryFault(motion_status);
    }
    /* 丢弃启流发送期间到达的旧/首批帧，再建立严格的新反馈屏障。 */
    (void)ZDT_CAN_FlushRx();
    memset(arm_control_driver_status, 0, sizeof(arm_control_driver_status));
    context->feedback_request_ms = HAL_GetTick();
    osDelay(1U);
    if (ArmControl_RequestTrajectoryStatus(config) != ARM_CONTROL_OK)
    {
        return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
    }
    if (zdt_can_error_count != context->can_error_baseline)
    {
        return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
    }
    context->rx_drop_baseline = zdt_can_rx_drop_count;

    /* 心跳只覆盖 F6 流式阶段，二次预检完成后再启用。 */
    context->heartbeat_ack_mask = ARM_CONTROL_ALL_MOTORS_MASK;
    context->trajectory_preflight_ready = false;

    now_ms = HAL_GetTick();
    context->feedback_deadline_ms =
        now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        context->previous_command_rpm[index] = 0.0f;
    }
    if (!ArmControl_TryPublishTrajectoryState(
            start_generation,
            ARM_CONTROL_TRAJECTORY_WAITING_FEEDBACK,
            ARM_CONTROL_BUSY))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }
    return ARM_CONTROL_OK;
}

/*
  函数说明: 推进一次曲线反馈等待、速度流执行或位置收尾状态。
  调用示例: ArmControl_ServiceTrajectoryInternal();
  参数意义: 无。
*/
static ArmControlStatus ArmControl_ServiceTrajectoryInternal(void)
{
    ArmControlConfig *config = arm_control_active_config;
    ArmControlTrajectoryContext *context = &arm_control_trajectory_context;
    ArmCartesianPosition start;
    ArmCartesianPosition control1;
    ArmCartesianPosition control2;
    ArmCartesianPosition target;
    ArmJointAngles joints;
    ArmMotorCoordinates next_coordinates;
    ArmTrajectoryStatus trajectory_status;
    ArmIkStatus ik_status;
    ArmControlStatus motion_status;
    ArmControlStatus minimum_status;
    uint32_t now_ms;
    uint32_t offset_ms;
    uint32_t elapsed_ms;
    uint32_t next_elapsed_ms;
    uint32_t stop_timeout_ms;
    float next_elapsed_s;
    float interval_s;

    if (config == NULL)
    {
        return ARM_CONTROL_NOT_INITIALIZED;
    }
    if (!ArmControl_TrajectoryStateIsBusy(context->result.state))
    {
        return context->result.status;
    }
    if (context->result.state == ARM_CONTROL_TRAJECTORY_PRECHECKING)
    {
        return ARM_CONTROL_BUSY;
    }
    if (!ArmControl_CommandTokenIsCurrent(context->generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_CONTROL_CANCELLED;
    }

    ArmControl_DrainRx(config, 0U);
    now_ms = HAL_GetTick();
    if ((zdt_can_rx_drop_count != context->rx_drop_baseline) ||
        (zdt_can_error_count != context->can_error_baseline))
    {
        return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
    }
    if ((context->driver_error_mask != 0U) ||
        (arm_control_move_tracker.error_mask != 0U))
    {
        return ArmControl_TrajectoryFault(ARM_CONTROL_DRIVER_ERROR);
    }

    if (context->result.state ==
        ARM_CONTROL_TRAJECTORY_WAITING_FEEDBACK)
    {
        if (!context->trajectory_preflight_ready &&
            ArmControl_AllPositionFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ArmControl_AllMotorsStopped(
                config, now_ms, context->feedback_request_ms) &&
            ArmControl_AllStatusFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms))
        {
            if (!ArmControl_TryPublishTrajectoryState(
                    context->generation,
                    ARM_CONTROL_TRAJECTORY_PRECHECKING,
                    ARM_CONTROL_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            motion_status = ArmControl_SyncCurrentFromFeedback(config, now_ms);
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            motion_status = ArmControl_CurrentEndpoint(config, &start);
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }

            /*
             * 二次完整预检包含较多三角函数。先停止定时返回，避免长轨迹
             * 计算期间 35/36/37/3A 持续入队并挤满 CAN RX 队列。
             * 三轴此时已经由新的 35 确认停稳，因此暂停反馈不会改变 P0。
             */
            motion_status = ArmControl_SetTrajectoryFeedback(
                config, false, 0U, false);
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            context->feedback_stream_enabled = false;
            if (context->driver_error_mask != 0U)
            {
                return ArmControl_TrajectoryFault(
                    ARM_CONTROL_DRIVER_ERROR);
            }
            if (!ArmControl_CommandTokenIsCurrent(context->generation))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }

            control1 = context->trajectory.point[1];
            control2 = context->trajectory.point[2];
            target = context->trajectory.point[3];
            trajectory_status = ArmTrajectory_InitCubicBezier(
                &context->trajectory,
                &start,
                &control1,
                &control2,
                &target,
                context->result.requested_time_s,
                config->trajectory_period_ms);
            context->result.trajectory_status = trajectory_status;
            context->result.start = start;
            if (trajectory_status != ARM_TRAJECTORY_OK)
            {
                return ArmControl_TrajectoryFault(
                    ARM_CONTROL_TRAJECTORY_ERROR);
            }

            /* 用刚收到的真实 36 位置重建 P0，再做一次完整预检。 */
            motion_status = ArmControl_PreflightTrajectory(
                config, &context->trajectory);
            if (!ArmControl_CommandTokenIsCurrent(context->generation))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            if (motion_status == ARM_CONTROL_OK)
            {
                motion_status = ArmControl_ValidateDiscreteTrajectory(
                    config, &context->trajectory);
                if (!ArmControl_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_CONTROL_CANCELLED;
                }
            }
            if ((motion_status == ARM_CONTROL_OK) ||
                (motion_status == ARM_CONTROL_TIME_UNACHIEVABLE))
            {
                minimum_status = ArmControl_CertifyMinimumTime(
                    config,
                    &context->trajectory,
                    motion_status == ARM_CONTROL_OK);
                if (minimum_status != ARM_CONTROL_OK)
                {
                    motion_status = minimum_status;
                }
                if (!ArmControl_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_CONTROL_CANCELLED;
                }
            }
            if ((motion_status != ARM_CONTROL_OK) &&
                (motion_status != ARM_CONTROL_TIME_UNACHIEVABLE))
            {
                context->result.minimum_time_s = 0.0f;
            }
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            motion_status = ArmControl_SolveTrajectorySample(
                config,
                &context->trajectory,
                0.0f,
                &joints,
                &context->reference_coordinates,
                &trajectory_status,
                &ik_status);
                if (!ArmControl_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_CONTROL_CANCELLED;
                }
            if (motion_status != ARM_CONTROL_OK)
            {
                context->result.trajectory_status = trajectory_status;
                context->result.ik_status = ik_status;
                return ArmControl_TrajectoryFault(motion_status);
            }

            /*
             * 预检通过后清除旧帧并重新开启定时反馈。后续必须再
             * 收到这个时刻之后的新 35/36，才允许启动 F6 轨迹。
             */
            context->feedback_stream_enabled = true;
            motion_status = ArmControl_SetTrajectoryFeedback(
                config, true, context->generation, true);
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            if ((zdt_can_rx_drop_count != context->rx_drop_baseline) ||
                (zdt_can_error_count != context->can_error_baseline))
            {
                return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
            }
            (void)ZDT_CAN_FlushRx();
            memset(arm_control_driver_status,
                   0,
                   sizeof(arm_control_driver_status));
            context->feedback_request_ms = HAL_GetTick();
            osDelay(1U);
            if (ArmControl_RequestTrajectoryStatus(config) != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
            }
            context->rx_drop_baseline = zdt_can_rx_drop_count;
            now_ms = HAL_GetTick();
            context->trajectory_preflight_ready = true;
            context->heartbeat_ack_mask = ARM_CONTROL_ALL_MOTORS_MASK;
            if (config->trajectory_heartbeat_ms != 0U)
            {
                context->heartbeat_ack_mask = 0U;
                context->heartbeat_enabled = true;
                context->heartbeat_request_ms = HAL_GetTick();
                motion_status = ArmControl_SetTrajectoryHeartbeat(
                    config,
                    (uint32_t)config->trajectory_heartbeat_ms,
                    context->generation,
                    true);
                if (motion_status != ARM_CONTROL_OK)
                {
                    return ArmControl_TrajectoryFault(motion_status);
                }
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            if (!ArmControl_TryPublishTrajectoryState(
                    context->generation,
                    ARM_CONTROL_TRAJECTORY_WAITING_FEEDBACK,
                    ARM_CONTROL_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            return ARM_CONTROL_BUSY;
        }

        if (context->trajectory_preflight_ready &&
            ArmControl_AllPositionFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ArmControl_AllMotorsStopped(
                config, now_ms, context->feedback_request_ms) &&
            ArmControl_AllStatusFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ((context->heartbeat_ack_mask & ARM_CONTROL_ALL_MOTORS_MASK) ==
             ARM_CONTROL_ALL_MOTORS_MASK))
        {
            /* 预检期间若机械位置发生变化，禁止用旧 P0 直接启动。 */
            if (!ArmControl_TrajectoryStartMatchesFeedback(config))
            {
                return ArmControl_TrajectoryFault(
                    ARM_CONTROL_POSITION_UNKNOWN);
            }
            /* 从心跳已确认且反馈仍新鲜的时刻重新起算绝对节拍。 */
            now_ms = HAL_GetTick();
            memset(context->previous_command_rpm,
                   0,
                   sizeof(context->previous_command_rpm));
            context->segment_index = 0U;
            context->start_ms = now_ms;
            context->next_update_ms = now_ms;
            arm_control_current_valid = false;
            if (!ArmControl_TryPublishTrajectoryState(
                    context->generation,
                    ARM_CONTROL_TRAJECTORY_RUNNING_SPEED,
                    ARM_CONTROL_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            return ARM_CONTROL_BUSY;
        }

        if (ArmControl_TimeReached(now_ms, context->feedback_deadline_ms))
        {
            return ArmControl_TrajectoryFault(ARM_CONTROL_FEEDBACK_TIMEOUT);
        }
        return ARM_CONTROL_BUSY;
    }

    if (context->result.state == ARM_CONTROL_TRAJECTORY_RUNNING_SPEED)
    {
        if (!ArmControl_AllPositionFeedbackFresh(config, now_ms))
        {
            return ArmControl_TrajectoryFault(ARM_CONTROL_FEEDBACK_TIMEOUT);
        }
        if (!ArmControl_TimeReached(now_ms, context->next_update_ms))
        {
            return ARM_CONTROL_BUSY;
        }
        if ((uint32_t)(now_ms - context->next_update_ms) >
            (uint32_t)config->trajectory_deadline_tolerance_ms)
        {
            return ArmControl_TrajectoryFault(ARM_CONTROL_DEADLINE_MISSED);
        }

        if (context->segment_index >= context->trajectory.segment_count)
        {
            stop_timeout_ms = ArmControl_TrajectoryStopTimeoutMs(config);
            if (!ArmControl_CommandLock())
            {
                return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
            }
            if (!ArmControl_CommandTokenIsCurrent(context->generation))
            {
                ArmControl_CommandUnlock();
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            motion_status = ArmControl_SendTrajectoryZeroVelocity(config);
            ArmControl_CommandUnlock();
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            context->stop_command_ms = HAL_GetTick();
            context->heartbeat_ack_mask = ARM_CONTROL_ALL_MOTORS_MASK;
            if (context->heartbeat_enabled)
            {
                context->heartbeat_ack_mask = 0U;
                context->heartbeat_request_ms = HAL_GetTick();
                if (ArmControl_SetTrajectoryHeartbeat(
                        config, 0U, 0U, false) !=
                    ARM_CONTROL_OK)
                {
                    return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
                }
                context->heartbeat_disable_pending = true;
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            context->stop_deadline_ms = now_ms + stop_timeout_ms;
            context->stop_speed_confirmed = false;
            if (!ArmControl_TryPublishTrajectoryState(
                    context->generation,
                    ARM_CONTROL_TRAJECTORY_STOPPING_SPEED,
                    ARM_CONTROL_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_CONTROL_CANCELLED;
            }
            return ARM_CONTROL_BUSY;
        }

        elapsed_ms = context->segment_index *
                     (uint32_t)config->trajectory_period_ms;
        if (elapsed_ms > context->trajectory.duration_ms)
        {
            elapsed_ms = context->trajectory.duration_ms;
        }
        next_elapsed_ms = (context->segment_index + 1U) *
                          (uint32_t)config->trajectory_period_ms;
        if (next_elapsed_ms > context->trajectory.duration_ms)
        {
            next_elapsed_ms = context->trajectory.duration_ms;
        }
        next_elapsed_s = (float)next_elapsed_ms * 0.001f;
        interval_s = (float)(next_elapsed_ms - elapsed_ms) * 0.001f;
        motion_status = ArmControl_SolveTrajectorySample(
            config,
            &context->trajectory,
            next_elapsed_s,
            &joints,
            &next_coordinates,
            &trajectory_status,
            &ik_status);
        if (motion_status != ARM_CONTROL_OK)
        {
            context->result.trajectory_status = trajectory_status;
            context->result.ik_status = ik_status;
            context->result.failed_sample = context->segment_index + 1U;
            return ArmControl_TrajectoryFault(motion_status);
        }
        if (!ArmControl_CommandLock())
        {
            return ArmControl_TrajectoryFault(ARM_CONTROL_CAN_ERROR);
        }
        if (!ArmControl_CommandTokenIsCurrent(context->generation))
        {
            ArmControl_CommandUnlock();
            context->result.minimum_time_s = 0.0f;
            return ARM_CONTROL_CANCELLED;
        }
        motion_status = ArmControl_SendTrajectoryVelocity(
            config,
            &context->reference_coordinates,
            &next_coordinates,
            interval_s,
            now_ms);
        ArmControl_CommandUnlock();
        if (motion_status != ARM_CONTROL_OK)
        {
            return ArmControl_TrajectoryFault(motion_status);
        }
        context->reference_coordinates = next_coordinates;
        ++context->segment_index;
        offset_ms = next_elapsed_ms;
        context->next_update_ms = context->start_ms + offset_ms;
        return ARM_CONTROL_BUSY;
    }

    if (context->result.state == ARM_CONTROL_TRAJECTORY_STOPPING_SPEED)
    {
        if ((context->heartbeat_ack_mask & ARM_CONTROL_ALL_MOTORS_MASK) !=
            ARM_CONTROL_ALL_MOTORS_MASK)
        {
            if (ArmControl_TimeReached(now_ms, context->feedback_deadline_ms))
            {
                return ArmControl_TrajectoryFault(
                    ARM_CONTROL_FEEDBACK_TIMEOUT);
            }
            return ARM_CONTROL_BUSY;
        }

        context->heartbeat_enabled = false;
        context->heartbeat_disable_pending = false;
        if (!context->stop_speed_confirmed)
        {
            if (!ArmControl_AllSpeedFeedbackFreshSince(
                    config, now_ms, context->stop_command_ms))
            {
                if (ArmControl_TimeReached(
                        now_ms, context->feedback_deadline_ms))
                {
                    return ArmControl_TrajectoryFault(
                        ARM_CONTROL_FEEDBACK_TIMEOUT);
                }
                return ARM_CONTROL_BUSY;
            }
            if (!ArmControl_AllMotorsStopped(
                    config, now_ms, context->stop_command_ms))
            {
                if (ArmControl_TimeReached(now_ms, context->stop_deadline_ms))
                {
                    return ArmControl_TrajectoryFault(
                        ARM_CONTROL_REACHED_TIMEOUT);
                }
                return ARM_CONTROL_BUSY;
            }

            /*
             * 35 已确认三轴停稳。丢弃减速阶段的旧 36，建立屏障并
             * 主动读取一次位置，只允许用停稳之后的三轴坐标生成 FD。
             */
            ArmControl_DrainRx(config, 0U);
            ArmControl_DrainRx(config, 0U);
            context->stop_position_barrier_ms = HAL_GetTick();
            for (uint8_t index = 0U;
                 index < ARM_CONTROL_MOTOR_COUNT;
                 ++index)
            {
                arm_control_driver_status[index].realtime_position_valid = false;
            }
            motion_status = ArmControl_RequestTrajectoryPosition(config);
            if (motion_status != ARM_CONTROL_OK)
            {
                return ArmControl_TrajectoryFault(motion_status);
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            context->stop_speed_confirmed = true;
            return ARM_CONTROL_BUSY;
        }

        if (!ArmControl_AllPositionFeedbackFreshSince(
                config, now_ms, context->stop_position_barrier_ms))
        {
            if (ArmControl_TimeReached(now_ms, context->feedback_deadline_ms))
            {
                return ArmControl_TrajectoryFault(
                    ARM_CONTROL_FEEDBACK_TIMEOUT);
            }
            return ARM_CONTROL_BUSY;
        }

        motion_status = ArmControl_BeginTrajectoryFinalPosition(
            config, now_ms);
        if (motion_status == ARM_CONTROL_CANCELLED)
        {
            return ARM_CONTROL_CANCELLED;
        }
        if ((motion_status != ARM_CONTROL_OK) &&
            (motion_status != ARM_CONTROL_BUSY))
        {
            return ArmControl_TrajectoryFault(motion_status);
        }
        return motion_status;
    }

    /* SETTLING_POSITION：token 与参与轴全部 FD 9F 在同一临界区提交。 */
    motion_status = ArmControl_CheckMoveCompletion(
        context->generation, true);
    if (motion_status == ARM_CONTROL_OK)
    {
        return ARM_CONTROL_OK;
    }
    if (motion_status != ARM_CONTROL_BUSY)
    {
        return ArmControl_TrajectoryFault(motion_status);
    }
    if (ArmControl_TimeReached(now_ms, context->settle_deadline_ms))
    {
        return ArmControl_TrajectoryFault(ARM_CONTROL_REACHED_TIMEOUT);
    }

    ArmControl_ServiceDiagnostics(config, now_ms);
    return ARM_CONTROL_BUSY;
}

/*
  函数说明: 查询曲线状态机是否仍在处理运动。
  调用示例: ArmControl_TrajectoryIsBusy();
  参数意义: 无。
*/
bool ArmControl_TrajectoryIsBusy(void)
{
    return ArmControl_TrajectoryStateIsBusy(
        arm_control_trajectory_context.result.state);
}

/*
  函数说明: 加锁后启动一条曲线；返回成功只表示启动成功，需继续调用 ServiceTrajectory。
  调用示例: status = ArmControl_StartBezier(&p1, &p2, &p3, 2.0f);
  参数意义: control1 为第一个贝塞尔控制点，mm。
            control2 为第二个贝塞尔控制点，mm。
            target 为目标值或目标位置。
            time_s 为内部规划或曲线参考时长，秒。
*/
ArmControlStatus ArmControl_StartBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmControlStatus status;

    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    status = ArmControl_StartBezierInternal(
        control1, control2, target, time_s);
    ArmControl_TrajectoryApiUnlock();
    return status;
}

/*
  函数说明: 推进非阻塞曲线状态机；调用者应按配置周期持续调用。
  调用示例: ArmControl_ServiceTrajectory();
  参数意义: 无。
*/
ArmControlStatus ArmControl_ServiceTrajectory(void)
{
    ArmControlStatus status;

    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    status = ArmControl_ServiceTrajectoryInternal();
    ArmControl_TrajectoryApiUnlock();
    return status;
}

/*
  函数说明: 取消当前曲线，停止电机并使旧命令失效。
  调用示例: ArmControl_AbortTrajectory();
  参数意义: 无。
*/
ArmControlStatus ArmControl_AbortTrajectory(void)
{
    if (!ArmControl_TrajectoryIsBusy())
    {
        return ARM_CONTROL_OK;
    }
    return ArmControl_TrajectoryFault(ARM_CONTROL_CANCELLED);
}

/*
  函数说明: 执行一条三次贝塞尔轨迹并阻塞等待到位或错误。
  调用示例: status = ArmControl_MoveBezier(&p1, &p2, &p3, 2.0f);
  参数意义: control1 为第一个贝塞尔控制点，mm。
            control2 为第二个贝塞尔控制点，mm。
            target 为目标值或目标位置。
            time_s 为内部规划或曲线参考时长，秒。
*/
ArmControlStatus ArmControl_MoveBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmControlStatus status;

    if (!ArmControl_TrajectoryApiLock())
    {
        return ARM_CONTROL_BUSY;
    }
    status = ArmControl_StartBezierInternal(
        control1, control2, target, time_s);
    if (status != ARM_CONTROL_OK)
    {
        ArmControl_TrajectoryApiUnlock();
        return status;
    }

    while (ArmControl_TrajectoryIsBusy())
    {
        status = ArmControl_ServiceTrajectoryInternal();
        if ((status != ARM_CONTROL_OK) && (status != ARM_CONTROL_BUSY))
        {
            ArmControl_TrajectoryApiUnlock();
            return status;
        }
        if (ArmControl_TrajectoryIsBusy())
        {
            osDelay(1U);
        }
    }
    status = arm_control_trajectory_context.result.status;
    ArmControl_TrajectoryApiUnlock();
    return status;
}

/*
  函数说明: 返回最近曲线运行状态和预检诊断的只读指针。
  调用示例: ArmControl_GetTrajectoryResult();
  参数意义: 无。
*/
const ArmControlTrajectoryResult *ArmControl_GetTrajectoryResult(void)
{
    return &arm_control_trajectory_context.result;
}

/*
  函数说明: 加锁后设置三轴驱动器的指定信息周期回传。
  调用示例: ArmControl_SetTimedReturnInfo(config, info_function, period_ms);
  参数意义: config 为对应模块的配置结构。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
ArmControlStatus ArmControl_SetTimedReturnInfo(const ArmControlConfig *config,
                                             uint8_t info_function,
                                             uint16_t period_ms)
{
    ArmControlStatus status;

    if (!ArmControl_FeedbackConfigLock())
    {
        return ARM_CONTROL_CAN_ERROR;
    }
    status = ArmControl_SetTimedReturnInfoUnlocked(
        config, info_function, period_ms);
    ArmControl_FeedbackConfigUnlock();
    return status;
}

/*
  函数说明: 在临界区复制一个机械臂电机的驱动器诊断快照。
  调用示例: ArmControl_GetDriverStatus(motor_index, status);
  参数意义: motor_index 为机械臂电机下标，0～2。
            status 为状态码或驱动器诊断快照。
*/
ArmControlStatus ArmControl_GetDriverStatus(uint8_t motor_index,
                                          ArmControlDriverStatus *status)
{
    if (status == NULL)
    {
        return ARM_CONTROL_NULL_POINTER;
    }
    if (motor_index >= ARM_CONTROL_MOTOR_COUNT)
    {
        return ARM_CONTROL_INVALID_CONFIG;
    }

    *status = arm_control_driver_status[motor_index];
    return ARM_CONTROL_OK;
}

/*
  函数说明: 把机械臂控制状态码转换为只读诊断字符串。
  调用示例: ArmControl_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmControl_StatusString(ArmControlStatus status)
{
    switch (status)
    {
        case ARM_CONTROL_OK: return "OK";
        case ARM_CONTROL_NULL_POINTER: return "null pointer";
        case ARM_CONTROL_INVALID_CONFIG: return "invalid configuration";
        case ARM_CONTROL_NOT_ZEROED: return "mechanical zero not confirmed";
        case ARM_CONTROL_IK_ERROR: return "kinematics or joint limit error";
        case ARM_CONTROL_MAPPING_ERROR: return "motor mapping error";
        case ARM_CONTROL_CAN_ERROR: return "CAN command error";
        case ARM_CONTROL_NOT_INITIALIZED: return "active configuration not set";
        case ARM_CONTROL_TIME_INVALID: return "invalid motion time";
        case ARM_CONTROL_TIME_UNACHIEVABLE:
            return "motion time cannot be achieved";
        case ARM_CONTROL_POSITION_UNKNOWN: return "motor position unknown";
        case ARM_CONTROL_REACHED_TIMEOUT:
            return "driver reached feedback timeout";
        case ARM_CONTROL_DRIVER_ERROR:
            return "driver rejected command or reported fault";
        case ARM_CONTROL_BUSY: return "motion is still running";
        case ARM_CONTROL_TRAJECTORY_ERROR:
            return "Bezier trajectory error";
        case ARM_CONTROL_FEEDBACK_TIMEOUT:
            return "trajectory feedback timeout";
        case ARM_CONTROL_DEADLINE_MISSED:
            return "trajectory service deadline missed";
        case ARM_CONTROL_DIRECTION_REVERSAL:
            return "motor direction reversal inside trajectory";
        case ARM_CONTROL_CANCELLED:
            return "motion cancelled by stop command";
        default: return "unknown status";
    }
}


static ArmControlConfig arm_application_config;
static uint8_t arm_application_ready;
static ArmControlStatus arm_application_recovery_status;

/*
  函数说明: 建立默认活动配置并初始化三个位置电机，沿用上电前人工摆放机械零位的约定。
  调用示例: status = ArmControl_Init();
  参数意义: 无。
*/
ArmControlStatus ArmControl_Init(void)
{
    ArmControlStatus status;
    uint8_t index;
    if (arm_application_ready) return ARM_CONTROL_OK;
    arm_application_recovery_status = ARM_CONTROL_OK;
    ArmControl_LoadDefaultConfig(&arm_application_config);
    status = ArmControl_SetActiveConfig(&arm_application_config);
    if (status != ARM_CONTROL_OK) return status;
    osDelay(2000U);
    /* 沿用上电前人工摆放机械零位的约定，此处不修改驱动器实际零点。 */
    ArmControl_ConfirmZero(&arm_application_config, true);
    for (index = 0U; index < ARM_CONTROL_MOTOR_COUNT; ++index)
    {
        if (X_V2_Reset_Clog_Pro(arm_application_config.motor_addr[index]) != HAL_OK)
            return ARM_CONTROL_CAN_ERROR;
        osDelay(10U);
    }
    status = ArmControl_EnableAll(&arm_application_config, true);
    if (status != ARM_CONTROL_OK)
    {
        (void)ArmControl_StopAll(&arm_application_config);
        (void)ArmControl_EnableAll(&arm_application_config, false);
        return status;
    }
    arm_application_ready = 1U;
    return ARM_CONTROL_OK;
}

/*
  函数说明: 查询应用机械臂是否完成初始化且仍具备可信运动状态。
  调用示例: ArmControl_IsReady();
  参数意义: 无。
*/
uint8_t ArmControl_IsReady(void)
{
    return arm_application_ready;
}

/*
  函数说明: 查询最近一次应用 XYZ 动作的超时恢复结果，供主机 10FF 错误帧使用。
  调用示例: recovery_status = ArmControl_GetRecoveryStatus();
  参数意义: 无；未尝试恢复或恢复成功返回 ARM_CONTROL_OK，失败返回实际状态码。
*/
ArmControlStatus ArmControl_GetRecoveryStatus(void)
{
    return arm_application_recovery_status;
}

/*
  函数说明: 共用 XYZ 动作的应用状态检查、到位等待和失败恢复处理。
  调用示例: status = ArmControl_ExecuteXYZ(x_mm, y_mm, z_mm, requested_time_s);
  参数意义: x_mm、y_mm、z_mm 为机械臂基坐标系绝对坐标，mm。
            requested_time_s 为请求时长，秒；0 表示自动规划。
*/
static ArmControlStatus ArmControl_ExecuteXYZ(
    float x_mm, float y_mm, float z_mm, float requested_time_s)
{
    ArmControlStatus status;
    if (!arm_application_ready) return ARM_CONTROL_NOT_INITIALIZED;
    arm_application_recovery_status = ARM_CONTROL_OK;
    status = ArmControl_MoveXYZ(x_mm, y_mm, z_mm, requested_time_s);
    if (status == ARM_CONTROL_REACHED_TIMEOUT)
    {
        arm_application_recovery_status = ArmControl_RecoverAfterTimeout(&arm_application_config);
        if (arm_application_recovery_status != ARM_CONTROL_OK)
            arm_application_ready = 0U;
    }
    else if (status != ARM_CONTROL_OK && status != ARM_CONTROL_IK_ERROR &&
             status != ARM_CONTROL_TIME_UNACHIEVABLE && status != ARM_CONTROL_BUSY)
    {
        arm_application_ready = 0U;
        (void)ArmControl_StopAll(&arm_application_config);
        (void)ArmControl_EnableAll(&arm_application_config, false);
    }
    return status;
}

/*
  函数说明: 以毫米坐标控制机械臂，同时启动、正常轴同步到位、微动可提前到位；失败停止或恢复。
  调用示例: status = ArmControl_MoveToXYZ(150.0f, 0.0f, 150.0f);
  参数意义: x_mm、y_mm、z_mm 为机械臂基坐标系绝对坐标，mm。
*/
ArmControlStatus ArmControl_MoveToXYZ(float x_mm, float y_mm, float z_mm)
{
    return ArmControl_ExecuteXYZ(x_mm, y_mm, z_mm, 0.0f);
}

/*
  函数说明: 按指定同步时长规划 XYZ 动作，并等待驱动器实际到位；时长无法实现时返回错误。
  调用示例: status = ArmControl_MoveToXYZTimed(0.0f, 300.0f, 260.0f, 2.0f);
  参数意义: x_mm、y_mm、z_mm 为机械臂基坐标系绝对坐标，mm。
            time_s 为正的计划时长，秒；仍受速度、加减速及协议量化约束。
*/
ArmControlStatus ArmControl_MoveToXYZTimed(
    float x_mm, float y_mm, float z_mm, float time_s)
{
    if (!isfinite(time_s) || time_s <= 0.0f) return ARM_CONTROL_TIME_INVALID;
    return ArmControl_ExecuteXYZ(x_mm, y_mm, z_mm, time_s);
}

