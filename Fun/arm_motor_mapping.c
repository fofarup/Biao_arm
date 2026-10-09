#include "arm_motor_mapping.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

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

    if ((config == NULL) || (joints == NULL) || (result == NULL))
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

ArmMotorMapStatus ArmMotorMap_JointsToMotorCoordinates(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const ArmJointAngles *joints,
    ArmMotorCoordinates *result)
{
    int direction_index;
    double offset_degrees;
    double motor1_degrees;
    double motor2_degrees;
    double motor3_degrees;

    if ((config == NULL) || (joints == NULL) || (result == NULL))
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

    direction_index = ArmMotorMap_DirectionIndex(installation_direction);
    if (direction_index < 0)
    {
        return ARM_MOTOR_MAP_INVALID_DIRECTION;
    }
    if (config->direction_offset_valid[direction_index] == 0U)
    {
        return ARM_MOTOR_MAP_DIRECTION_NOT_CONFIGURED;
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
