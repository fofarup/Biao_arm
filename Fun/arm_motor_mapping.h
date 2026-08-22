#ifndef ARM_MOTOR_MAPPING_H
#define ARM_MOTOR_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "arm_kinematics.h"

/* 安装方向只影响底座零点偏移，不参与机械臂几何逆解。 */
typedef enum
{
    ARM_INSTALLATION_DIRECTION_1 = 1,
    ARM_INSTALLATION_DIRECTION_2 = 2,
    ARM_INSTALLATION_DIRECTION_3 = 3,
    ARM_INSTALLATION_DIRECTION_4 = 4
} ArmInstallationDirection;

/* 电机正脉冲坐标在 X 固件中的实际旋转方向。 */
typedef enum
{
    ARM_MOTOR_DIR_CW = 0U,
    ARM_MOTOR_DIR_CCW = 1U
} ArmMotorDirection;

typedef enum
{
    ARM_MOTOR_MAP_OK = 0,
    ARM_MOTOR_MAP_NULL_POINTER,
    ARM_MOTOR_MAP_INVALID_CONFIG,
    ARM_MOTOR_MAP_INVALID_DIRECTION,
    ARM_MOTOR_MAP_DIRECTION_NOT_CONFIGURED,
    ARM_MOTOR_MAP_NUMERIC_ERROR,
    ARM_MOTOR_MAP_OVERFLOW
} ArmMotorMapStatus;

/*
 * 这里只存放电机/传动系统参数，不存放关节机械限位。
 * gamma_zero_deg 是地址3电机零位对应的几何 Gamma；当前标定为 12.02 度。
 */
typedef struct
{
    float motor1_ratio;
    float motor2_ratio;
    float motor3_gamma_ratio;
    float gamma_zero_deg;
    uint32_t pulses_per_revolution;

    ArmMotorDirection motor1_positive_direction;
    ArmMotorDirection motor2_positive_direction;
    ArmMotorDirection motor3_positive_direction;

    int32_t direction_offset_pulse[4];
    uint8_t direction_offset_valid[4];
} ArmMotorMapConfig;

typedef struct
{
    int32_t pulse[3];
    ArmMotorDirection direction[3];
} ArmMotorTargets;

void ArmMotorMap_LoadDefaultConfig(ArmMotorMapConfig *config);

/*
 * 用户关节位移 q1/q2/q3 -> 几何 Alpha/Beta/Gamma。
 * q2 单独增加时地址3不补偿，因此 Gamma 按 4.5/13.5 的耦合量自然变化。
 */
ArmMotorMapStatus ArmMotorMap_OffsetsToJoints(
    const ArmMotorMapConfig *config,
    float base_deg,
    float large_arm_deg,
    float small_arm_deg,
    ArmJointAngles *result);

ArmMotorMapStatus ArmMotorMap_SetDirectionOffset(
    ArmMotorMapConfig *config,
    ArmInstallationDirection direction,
    int32_t offset_pulse);

ArmMotorMapStatus ArmMotorMap_ClearDirectionOffset(
    ArmMotorMapConfig *config,
    ArmInstallationDirection direction);

/* 机械关节角 -> 三台电机的有符号绝对目标脉冲和实际方向。 */
ArmMotorMapStatus ArmMotorMap_JointsToTargets(
    const ArmMotorMapConfig *config,
    ArmInstallationDirection installation_direction,
    const ArmJointAngles *joints,
    ArmMotorTargets *result);

const char *ArmMotorMap_StatusString(ArmMotorMapStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_MOTOR_MAPPING_H */
