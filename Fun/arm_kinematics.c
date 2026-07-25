#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_PI_F                (3.14159265358979323846f)
#define ARM_RAD_TO_DEG_F        (180.0f / ARM_PI_F)
/* 用于容忍浮点计算在可达边界处产生的微小舍入误差。 */
#define ARM_NUMERIC_EPSILON_F   (1.0e-5f)

static int ArmIk_IsFinite(float value)
{
    return isfinite(value) ? 1 : 0;
}

static int32_t ArmIk_RoundToInt32(float value)
{
    /* 将浮点脉冲四舍五入为驱动器使用的整数脉冲。 */
    return (int32_t)lroundf(value);
}

static int ArmIk_DirectionIndex(ArmDirection direction)
{
    /* 将用户使用的方向编号 1~4 转换为数组下标 0~3。 */
    int value = (int)direction;
    if ((value < (int)ARM_DIRECTION_1) ||
        (value > (int)ARM_DIRECTION_4))
    {
        return -1;
    }
    return value - 1;
}

static int ArmIk_ConfigIsValid(const ArmIkConfig *config)
{
    /* 在进入三角函数前检查配置，避免除零、NaN 或无意义的机械参数。 */
    if ((config->oa_mm <= 0.0f) ||
        (config->ab_mm <= 0.0f) ||
        (config->bc_mm <= 0.0f) ||
        (config->cd1_mm < 0.0f) ||
        (config->gamma_min_deg < 0.0f) ||
        (config->gamma_min_deg >= 180.0f) ||
        (config->motor1_ratio <= 0.0f) ||
        (config->motor2_ratio <= 0.0f) ||
        (config->motor3_gamma_ratio <= 0.0f) ||
        (config->pulses_per_revolution == 0U))
    {
        return 0;
    }

    return ArmIk_IsFinite(config->oa_mm) &&
           ArmIk_IsFinite(config->ab_mm) &&
           ArmIk_IsFinite(config->bc_mm) &&
           ArmIk_IsFinite(config->cd1_mm) &&
           ArmIk_IsFinite(config->gamma_min_deg) &&
           ArmIk_IsFinite(config->motor1_ratio) &&
           ArmIk_IsFinite(config->motor2_ratio) &&
           ArmIk_IsFinite(config->motor3_gamma_ratio);
}

void ArmIk_LoadDefaultConfig(ArmIkConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    /* 连杆尺寸完全取自原 Python 程序，后续实测后可在此统一修正。 */
    memset(config, 0, sizeof(*config));
    config->oa_mm = 150.1f;
    config->ab_mm = 150.0f;
    config->bc_mm = 240.0f;
    config->cd1_mm = 85.0f;
    config->gamma_min_deg = 12.02f;

    /* 传动比和每圈脉冲数共同决定“关节角度 -> 电机脉冲”的换算。 */
    config->motor1_ratio = 26.25f;
    config->motor2_ratio = 4.5f;
    config->motor3_gamma_ratio = 13.5f;
    config->pulses_per_revolution = 3200U;

    /* 原程序仅给出方向 1~3 的底座偏移，方向 4 必须标定后再启用。 */
    config->direction_offset_pulse[0] = 0;
    config->direction_offset_pulse[1] = -42000;
    config->direction_offset_pulse[2] = -21000;
    config->direction_offset_pulse[3] = 0;
    config->direction_offset_valid[0] = 1U;
    config->direction_offset_valid[1] = 1U;
    config->direction_offset_valid[2] = 1U;
    config->direction_offset_valid[3] = 0U;
}

ArmIkStatus ArmIk_SetDirectionOffset(ArmIkConfig *config,
                                     ArmDirection direction,
                                     int32_t offset_pulse)
{
    int index;
    if (config == NULL)
    {
        return ARM_IK_NULL_POINTER;
    }

    index = ArmIk_DirectionIndex(direction);
    if (index < 0)
    {
        return ARM_IK_INVALID_DIRECTION;
    }

    config->direction_offset_pulse[index] = offset_pulse;
    config->direction_offset_valid[index] = 1U;
    return ARM_IK_OK;
}

ArmIkStatus ArmIk_ClearDirectionOffset(ArmIkConfig *config,
                                       ArmDirection direction)
{
    int index;
    if (config == NULL)
    {
        return ARM_IK_NULL_POINTER;
    }

    index = ArmIk_DirectionIndex(direction);
    if (index < 0)
    {
        return ARM_IK_INVALID_DIRECTION;
    }

    config->direction_offset_valid[index] = 0U;
    return ARM_IK_OK;
}

ArmIkStatus ArmIk_Solve(const ArmIkConfig *config,
                        ArmDirection direction,
                        float x_mm,
                        float y_mm,
                        float z_mm,
                        ArmIkResult *result)
{
    int direction_index;
    float alpha;
    float radial_mm;
    float wrist_radial_mm;
    float wrist_z_mm;
    float ac_mm;
    float min_reach_mm;
    float max_reach_mm;
    float cos_gamma;
    float cos_b1;
    float gamma;
    float b1;
    float b2;
    float beta;
    float alpha_deg;
    float beta_deg;
    float gamma_deg;
    float pulse_scale;

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

    direction_index = ArmIk_DirectionIndex(direction);
    if (direction_index < 0)
    {
        return ARM_IK_INVALID_DIRECTION;
    }
    if (config->direction_offset_valid[direction_index] == 0U)
    {
        return ARM_IK_DIRECTION_NOT_CONFIGURED;
    }

    /*
     * Alpha 为底座绕 Z 轴的旋转角。
     * 使用 atan2f(X,Y) 可正确区分象限，也避免原 atan(X/Y) 在 Y=0 时除零。
     */
    alpha = atan2f(x_mm, y_mm);

    /*
     * 将三维目标投影到机械臂所在的竖直平面：
     * radial_mm       为目标到 Z 轴的水平距离；
     * wrist_radial_mm 为扣除末端 CD1 后，腕部相对肩部的水平距离；
     * wrist_z_mm      为腕部相对肩部高度 OA 的竖直距离；
     * ac_mm           为肩部 A 到腕部 C 的直线距离。
     */
    radial_mm = hypotf(x_mm, y_mm);
    wrist_radial_mm = radial_mm - config->cd1_mm;
    wrist_z_mm = z_mm - config->oa_mm;
    ac_mm = hypotf(wrist_radial_mm, wrist_z_mm);

    /* 两连杆机构的可达条件：|BC-AB| <= AC <= AB+BC。 */
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

    /*
     * 根据余弦定理求肘部角 Gamma 和三角形内角 B1：
     * cos(Gamma) = (AB^2 + BC^2 - AC^2) / (2*AB*BC)
     * cos(B1)    = (AB^2 + AC^2 - BC^2) / (2*AB*AC)
     */
    cos_gamma = ((config->ab_mm * config->ab_mm) +
                 (config->bc_mm * config->bc_mm) - (ac_mm * ac_mm)) /
                (2.0f * config->ab_mm * config->bc_mm);
    cos_b1 = ((config->ab_mm * config->ab_mm) + (ac_mm * ac_mm) -
              (config->bc_mm * config->bc_mm)) /
             (2.0f * config->ab_mm * ac_mm);

    /* acosf 只接受 [-1,1]，在可达边界处吸收浮点舍入造成的微小越界。 */
    if (cos_gamma > 1.0f) cos_gamma = 1.0f;
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    if (cos_b1 > 1.0f) cos_b1 = 1.0f;
    if (cos_b1 < -1.0f) cos_b1 = -1.0f;

    gamma = acosf(cos_gamma);
    b1 = acosf(cos_b1);

    /*
     * B2 表示 AC 连线相对机构基准方向的角度；
     * Beta 为第二关节最终目标角，公式保持与 robocal.py 一致。
     */
    b2 = (ARM_PI_F * 0.5f) + atan2f(wrist_z_mm, wrist_radial_mm);
    beta = (ARM_PI_F * 1.5f) - (b1 + b2);

    alpha_deg = alpha * ARM_RAD_TO_DEG_F;
    beta_deg = beta * ARM_RAD_TO_DEG_F;
    gamma_deg = gamma * ARM_RAD_TO_DEG_F;
    if (gamma_deg < config->gamma_min_deg)
    {
        gamma_deg = config->gamma_min_deg;
    }

    if (!ArmIk_IsFinite(alpha_deg) || !ArmIk_IsFinite(beta_deg) ||
        !ArmIk_IsFinite(gamma_deg))
    {
        return ARM_IK_NUMERIC_ERROR;
    }

    /* 每度对应的基础脉冲数，之后再乘各关节传动比。 */
    pulse_scale = (float)config->pulses_per_revolution / 360.0f;

    /* Motor1：底座角 Alpha 换算为脉冲，再叠加当前安装方向的零点偏移。 */
    result->motor1_pulse = ArmIk_RoundToInt32(config->motor1_ratio *
                                               alpha_deg * pulse_scale) +
                           config->direction_offset_pulse[direction_index];
    /* Motor2：由肩关节角 Beta 直接换算。 */
    result->motor2_pulse = ArmIk_RoundToInt32(config->motor2_ratio *
                                               beta_deg * pulse_scale);

    /*
     * Motor3 存在与 Motor2 的机械耦合，因此同时包含 Gamma 项和 Beta 补偿项。
     * 该公式按原 Python 程序保留，输出允许为负数。
     */
    result->motor3_pulse = ArmIk_RoundToInt32(
        ((config->motor3_gamma_ratio *
          (config->gamma_min_deg - gamma_deg)) +
         (config->motor2_ratio * beta_deg)) * pulse_scale);

    result->alpha_deg = alpha_deg;
    result->beta_deg = beta_deg;
    result->gamma_deg = gamma_deg;

    /* 原机构不允许 Motor2 目标脉冲为负，调用方收到错误后不得下发电机命令。 */
    if (result->motor2_pulse < 0)
    {
        return ARM_IK_MOTOR2_NEGATIVE;
    }

    return ARM_IK_OK;
}

const char *ArmIk_StatusString(ArmIkStatus status)
{
    switch (status)
    {
        case ARM_IK_OK: return "OK";
        case ARM_IK_NULL_POINTER: return "null pointer";
        case ARM_IK_INVALID_CONFIG: return "invalid configuration";
        case ARM_IK_INVALID_DIRECTION: return "invalid direction";
        case ARM_IK_DIRECTION_NOT_CONFIGURED: return "direction not configured";
        case ARM_IK_TARGET_TOO_CLOSE: return "target too close";
        case ARM_IK_TARGET_TOO_FAR: return "target too far";
        case ARM_IK_NUMERIC_ERROR: return "numeric error";
        case ARM_IK_MOTOR2_NEGATIVE: return "motor 2 target is negative";
        default: return "unknown status";
    }
}
