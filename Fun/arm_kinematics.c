#include "arm_kinematics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ARM_PI_F                 (3.14159265358979323846f)
#define ARM_RAD_TO_DEG_F         (180.0f / ARM_PI_F)
#define ARM_DEG_TO_RAD_F         (ARM_PI_F / 180.0f)
#define ARM_NUMERIC_EPSILON_F    (1.0e-5f)
#define ARM_LIMIT_EPSILON_DEG_F  (1.0e-3f)

static int ArmIk_IsFinite(float value)
{
    return isfinite(value) ? 1 : 0;
}

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

void ArmIk_LoadDefaultConfig(ArmIkConfig *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->oa_mm = 150.1f;
    config->ab_mm = 150.0f;
    config->bc_mm = 240.0f;
    config->cd1_mm = 85.0f;

    /* Alpha 的真实机械上下限尚未标定，本轮不擅自启用。 */
    config->alpha_limit.min_deg = -180.0f;
    config->alpha_limit.max_deg = 180.0f;

    /* 原机构明确禁止大臂低于零位；上限仍等待实机标定。 */
    config->beta_limit.min_deg = 0.0f;
    config->beta_limit.max_deg = 180.0f;
    config->beta_limit.min_enabled = 1U;

    /* 已确认的 Gamma 机械最小内角；不得再用自动钳制掩盖超限。 */
    config->gamma_limit.min_deg = 12.02;
    config->gamma_limit.max_deg = 180.0f;
    config->gamma_limit.min_enabled = 1U;
    config->gamma_limit.max_enabled = 1U;
}

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
