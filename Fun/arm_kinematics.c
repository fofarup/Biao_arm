/*
  程序说明: 机械臂几何模型、臂长配置、正运动学、逆运动学与限位校验。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_PI_F                 (3.14159265358979323846f)
#define ARM_RAD_TO_DEG_F         (180.0f / ARM_PI_F)
#define ARM_DEG_TO_RAD_F         (ARM_PI_F / 180.0f)
#define ARM_NUMERIC_EPSILON_F    (1.0e-5f)
#define ARM_LIMIT_EPSILON_DEG_F  (1.0e-3f)

/*
  函数说明: 判断浮点值是否有限，拒绝 NaN 和无穷。
  调用示例: ArmIk_IsFinite(value);
  参数意义: value 为输入数值。
*/
static int ArmIk_IsFinite(float value)
{
    return isfinite(value) ? 1 : 0;
}

/*
  函数说明: 校验单个关节的上下限配置。
  调用示例: ArmIk_LimitConfigIsValid(limit);
  参数意义: limit 为限幅值或关节限位配置。
*/
static int ArmIk_LimitConfigIsValid(const ArmJointLimit *limit)
{
    if ((limit->min_enabled > 1U) || (limit->max_enabled > 1U) ||
        !ArmIk_IsFinite(limit->min_deg) ||
        !ArmIk_IsFinite(limit->max_deg))
    {
        return 0;
    }

    if ((limit->min_enabled != 0U) &&
        (limit->max_enabled != 0U) &&
        (limit->min_deg > limit->max_deg))
    {
        return 0;
    }

    return 1;
}

/*
  函数说明: 校验臂长、末端偏置和全部关节限位配置。
  调用示例: ArmIk_ConfigIsValid(config);
  参数意义: config 为对应模块的配置结构。
*/
static int ArmIk_ConfigIsValid(const ArmIkConfig *config)
{
    if ((config == NULL) ||
        !ArmIk_IsFinite(config->oa_mm) ||
        !ArmIk_IsFinite(config->ab_mm) ||
        !ArmIk_IsFinite(config->bc_mm) ||
        !ArmIk_IsFinite(config->cd1_mm) ||
        (config->oa_mm <= 0.0f) ||
        (config->ab_mm <= 0.0f) ||
        (config->bc_mm <= 0.0f) ||
        (config->cd1_mm < 0.0f))
    {
        return 0;
    }

    return ArmIk_LimitConfigIsValid(&config->alpha_limit) &&
           ArmIk_LimitConfigIsValid(&config->beta_limit) &&
           ArmIk_LimitConfigIsValid(&config->gamma_limit);
}

/*
  函数说明: 检查一个关节角是否在启用的机械限位范围内。
  调用示例: ArmIk_JointWithinLimit(value, limit);
  参数意义: value 为输入数值。
            limit 为限幅值或关节限位配置。
*/
static int ArmIk_JointWithinLimit(float value, const ArmJointLimit *limit)
{
    if ((limit->min_enabled != 0U) &&
        (value < (limit->min_deg - ARM_LIMIT_EPSILON_DEG_F)))
    {
        return 0;
    }
    if ((limit->max_enabled != 0U) &&
        (value > (limit->max_deg + ARM_LIMIT_EPSILON_DEG_F)))
    {
        return 0;
    }

    return 1;
}

/*
  函数说明: 将弧度角归一化到主值区间，避免等价角重复。
  调用示例: ArmIk_NormalizeRadians(angle);
  参数意义: angle 为待归一化的弧度角。
*/
static float ArmIk_NormalizeRadians(float angle)
{
    while (angle > ARM_PI_F)
    {
        angle -= 2.0f * ARM_PI_F;
    }
    while (angle <= -ARM_PI_F)
    {
        angle += 2.0f * ARM_PI_F;
    }
    return angle;
}

/*
  函数说明: 装载原机械臂臂长、末端偏置和关节限位。
  调用示例: ArmIk_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmIk_LoadDefaultConfig(ArmIkConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->oa_mm = 180.1f;
    config->ab_mm = 150.0f;
    config->bc_mm = 240.0f;
    config->cd1_mm = 85.0f;

    /* 用户已确认底座安全范围为 -180 到 +180 度。 */
    config->alpha_limit.min_deg = -180.0f;
    config->alpha_limit.max_deg = 180.0f;
    config->alpha_limit.min_enabled = 1U;
    config->alpha_limit.max_enabled = 1U;

    /* 用户已确认大臂安全范围为 0 到 180 度。 */
    config->beta_limit.min_deg = 0.0f;
    config->beta_limit.max_deg = 180.0f;
    config->beta_limit.min_enabled = 1U;
    config->beta_limit.max_enabled = 1U;

    /* 已确认的 Gamma 机械最小内角；不得再用自动钳制掩盖超限。 */
    config->gamma_limit.min_deg = 12.02f;
    config->gamma_limit.max_deg = 180.0f;
    config->gamma_limit.min_enabled = 1U;
    config->gamma_limit.max_enabled = 1U;
}

/*
  函数说明: 检查三个几何关节角是否有限且处于机械限位内。
  调用示例: ArmIk_ValidateJoints(config, joints);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
*/
ArmIkStatus ArmIk_ValidateJoints(const ArmIkConfig *config,
                                 const ArmJointAngles *joints)
{
    if ((config == NULL) || (joints == NULL))
    {
        return ARM_IK_NULL_POINTER;
    }
    if (!ArmIk_ConfigIsValid(config))
    {
        return ARM_IK_INVALID_CONFIG;
    }
    if (!ArmIk_IsFinite(joints->alpha_deg) ||
        !ArmIk_IsFinite(joints->beta_deg) ||
        !ArmIk_IsFinite(joints->gamma_deg))
    {
        return ARM_IK_NUMERIC_ERROR;
    }

    if (!ArmIk_JointWithinLimit(joints->alpha_deg,
                                &config->alpha_limit) ||
        !ArmIk_JointWithinLimit(joints->beta_deg,
                                &config->beta_limit) ||
        !ArmIk_JointWithinLimit(joints->gamma_deg,
                                &config->gamma_limit))
    {
        return ARM_IK_JOINT_LIMIT;
    }

    return ARM_IK_OK;
}

/*
  函数说明: 对给定有符号径向位置求平面两连杆逆解并校验限位。
  调用示例: ArmIk_SolveCandidate(config, signed_radial_mm, wrist_z_mm, alpha_rad, result);
  参数意义: config 为对应模块的配置结构。
            signed_radial_mm 为带分支符号的水平径向距离，mm。
            wrist_z_mm 为扣除末端偏置后的腕部高度，mm。
            alpha_rad 为底座几何角，弧度。
            result 为输出计算结果。
*/
static ArmIkStatus ArmIk_SolveCandidate(const ArmIkConfig *config,
                                        float signed_radial_mm,
                                        float wrist_z_mm,
                                        float alpha_rad,
                                        ArmJointAngles *result)
{
    float wrist_radial_mm;
    float ac_mm;
    float min_reach_mm;
    float max_reach_mm;
    float cos_gamma;
    float cos_b1;
    float gamma;
    float b1;
    float b2;
    float beta;

    wrist_radial_mm = signed_radial_mm - config->cd1_mm;
    ac_mm = hypotf(wrist_radial_mm, wrist_z_mm);
    min_reach_mm = fabsf(config->bc_mm - config->ab_mm);
    max_reach_mm = config->ab_mm + config->bc_mm;

    if (ac_mm < (min_reach_mm - ARM_NUMERIC_EPSILON_F))
    {
        return ARM_IK_TARGET_TOO_CLOSE;
    }
    if (ac_mm > (max_reach_mm + ARM_NUMERIC_EPSILON_F))
    {
        return ARM_IK_TARGET_TOO_FAR;
    }
    if (ac_mm <= ARM_NUMERIC_EPSILON_F)
    {
        return ARM_IK_NUMERIC_ERROR;
    }

    cos_gamma = ((config->ab_mm * config->ab_mm) +
                 (config->bc_mm * config->bc_mm) - (ac_mm * ac_mm)) /
                (2.0f * config->ab_mm * config->bc_mm);
    cos_b1 = ((config->ab_mm * config->ab_mm) + (ac_mm * ac_mm) -
              (config->bc_mm * config->bc_mm)) /
             (2.0f * config->ab_mm * ac_mm);

    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    if (cos_b1 > 1.0f) cos_b1 = 1.0f;
    if (cos_b1 < -1.0f) cos_b1 = -1.0f;

    gamma = acosf(cos_gamma);
    b1 = acosf(cos_b1);
    b2 = (ARM_PI_F * 0.5f) +
         atan2f(wrist_z_mm, wrist_radial_mm);
    beta = (ARM_PI_F * 1.5f) - (b1 + b2);

    result->alpha_deg = ArmIk_NormalizeRadians(alpha_rad) * ARM_RAD_TO_DEG_F;
    result->beta_deg = beta * ARM_RAD_TO_DEG_F;
    result->gamma_deg = gamma * ARM_RAD_TO_DEG_F;

    /* 超限只返回错误，绝不修改或钳制求出的关节角。 */
    return ArmIk_ValidateJoints(config, result);
}

/*
  函数说明: 按默认正径向分支求机械臂基坐标系 XYZ 对应的三个关节角。
  调用示例: ArmIk_Solve(config, x_mm, y_mm, z_mm, result);
  参数意义: config 为对应模块的配置结构。
            x_mm 为机械臂基坐标系 X 坐标，mm。
            y_mm 为机械臂基坐标系 Y 坐标，mm。
            z_mm 为机械臂基坐标系 Z 坐标，mm。
            result 为输出计算结果。
*/
ArmIkStatus ArmIk_Solve(const ArmIkConfig *config,
                        float x_mm,
                        float y_mm,
                        float z_mm,
                        ArmJointAngles *result)
{
    return ArmIk_SolveBranch(config,
                             x_mm,
                             y_mm,
                             z_mm,
                             ARM_IK_RADIAL_POSITIVE,
                             result);
}

/*
  函数说明: 按明确指定的径向分支求逆解；不可达、奇异或超限时返回错误。
  调用示例: ArmIk_SolveBranch(config, x_mm, y_mm, z_mm, branch, result);
  参数意义: config 为对应模块的配置结构。
            x_mm 为机械臂基坐标系 X 坐标，mm。
            y_mm 为机械臂基坐标系 Y 坐标，mm。
            z_mm 为机械臂基坐标系 Z 坐标，mm。
            branch 为逆解径向分支。
            result 为输出计算结果。
*/
ArmIkStatus ArmIk_SolveBranch(const ArmIkConfig *config,
                              float x_mm,
                              float y_mm,
                              float z_mm,
                              ArmIkRadialBranch branch,
                              ArmJointAngles *result)
{
    float radial_mm;
    float alpha_rad;

    if ((config == NULL) || (result == NULL))
    {
        return ARM_IK_NULL_POINTER;
    }
    if (!ArmIk_ConfigIsValid(config))
    {
        return ARM_IK_INVALID_CONFIG;
    }
    if (!ArmIk_IsFinite(x_mm) || !ArmIk_IsFinite(y_mm) ||
        !ArmIk_IsFinite(z_mm))
    {
        return ARM_IK_NUMERIC_ERROR;
    }
    if ((branch != ARM_IK_RADIAL_POSITIVE) &&
        (branch != ARM_IK_RADIAL_NEGATIVE))
    {
        return ARM_IK_INVALID_CONFIG;
    }

    radial_mm = hypotf(x_mm, y_mm);
    alpha_rad = atan2f(x_mm, y_mm);
    if (branch == ARM_IK_RADIAL_NEGATIVE)
    {
        radial_mm = -radial_mm;
        alpha_rad = ArmIk_NormalizeRadians(alpha_rad - ARM_PI_F);
    }
    return ArmIk_SolveCandidate(config,
                                radial_mm,
                                z_mm - config->oa_mm,
                                alpha_rad,
                                result);
}

/*
  函数说明: 由三个几何关节角计算末端 XYZ 坐标。
  调用示例: ArmIk_Forward(config, joints, result);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
*/
ArmIkStatus ArmIk_Forward(const ArmIkConfig *config,
                          const ArmJointAngles *joints,
                          ArmCartesianPosition *result)
{
    ArmIkStatus joint_status;
    float alpha;
    float beta;
    float gamma;
    float wrist_radial_mm;
    float wrist_z_mm;
    float radial_mm;

    if ((config == NULL) || (joints == NULL) || (result == NULL))
    {
        return ARM_IK_NULL_POINTER;
    }

    joint_status = ArmIk_ValidateJoints(config, joints);
    if (joint_status != ARM_IK_OK)
    {
        return joint_status;
    }

    alpha = joints->alpha_deg * ARM_DEG_TO_RAD_F;
    beta = joints->beta_deg * ARM_DEG_TO_RAD_F;
    gamma = joints->gamma_deg * ARM_DEG_TO_RAD_F;

    wrist_radial_mm = (-config->ab_mm * cosf(beta)) +
                      (config->bc_mm * cosf(gamma - beta));
    wrist_z_mm = (config->ab_mm * sinf(beta)) +
                 (config->bc_mm * sinf(gamma - beta));
    radial_mm = wrist_radial_mm + config->cd1_mm;

    result->x_mm = radial_mm * sinf(alpha);
    result->y_mm = radial_mm * cosf(alpha);
    result->z_mm = config->oa_mm + wrist_z_mm;

    if (!ArmIk_IsFinite(result->x_mm) ||
        !ArmIk_IsFinite(result->y_mm) ||
        !ArmIk_IsFinite(result->z_mm))
    {
        return ARM_IK_NUMERIC_ERROR;
    }

    return ARM_IK_OK;
}

/*
  函数说明: 把运动学状态码转换为只读诊断字符串。
  调用示例: ArmIk_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmIk_StatusString(ArmIkStatus status)
{
    switch (status)
    {
        case ARM_IK_OK: return "OK";
        case ARM_IK_NULL_POINTER: return "null pointer";
        case ARM_IK_INVALID_CONFIG: return "invalid configuration";
        case ARM_IK_TARGET_TOO_CLOSE: return "target too close";
        case ARM_IK_TARGET_TOO_FAR: return "target too far";
        case ARM_IK_JOINT_LIMIT: return "joint limit";
        case ARM_IK_NUMERIC_ERROR: return "numeric error";
        default: return "unknown status";
    }
}
