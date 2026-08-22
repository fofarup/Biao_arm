#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    ARM_IK_OK = 0,                   /* 解算成功 */
    ARM_IK_NULL_POINTER,             /* 传入了空指针 */
    ARM_IK_INVALID_CONFIG,           /* 几何参数或限位配置非法 */
    ARM_IK_TARGET_TOO_CLOSE,         /* 目标点在几何最小可达半径以内 */
    ARM_IK_TARGET_TOO_FAR,           /* 目标点超过几何最大可达半径 */
    ARM_IK_JOINT_LIMIT,              /* 解存在，但至少一个关节超出机械限位 */
    ARM_IK_NUMERIC_ERROR             /* 输入为 NaN/Inf 或发生数值异常 */
} ArmIkStatus;

/* 同一个 XYZ 可能对应底座两侧的不同姿态。 */
typedef enum
{
    ARM_IK_RADIAL_POSITIVE = 0,
    ARM_IK_RADIAL_NEGATIVE = 1
} ArmIkRadialBranch;

/* 机械关节角。Gamma 是几何内角 ABC，不是从零开始的电机角度。 */
typedef struct
{
    float alpha_deg;
    float beta_deg;
    float gamma_deg;
} ArmJointAngles;

typedef struct
{
    float x_mm;
    float y_mm;
    float z_mm;
} ArmCartesianPosition;

/*
 * 单个关节的机械限位。上下限可分别启用，未实测的限位不应擅自启用。
 * 限位只属于关节/机构层，与电机 CW/CCW、减速比及 CAN 地址无关。
 */
typedef struct
{
    float min_deg;
    float max_deg;
    uint8_t min_enabled;
    uint8_t max_enabled;
} ArmJointLimit;

typedef struct
{
    /* 机械尺寸，单位 mm；名称与原 robocal.py 中的连杆名称一致。 */
    float oa_mm;
    float ab_mm;
    float bc_mm;
    float cd1_mm;

    ArmJointLimit alpha_limit;
    ArmJointLimit beta_limit;
    ArmJointLimit gamma_limit;
} ArmIkConfig;

/* 装载当前已知的几何参数及机械限位。 */
void ArmIk_LoadDefaultConfig(ArmIkConfig *config);

/* 只检查关节角和机械限位，不进行电机映射，也不发送 CAN。 */
ArmIkStatus ArmIk_ValidateJoints(const ArmIkConfig *config,
                                 const ArmJointAngles *joints);

/*
 * 末端 XYZ -> 关节角。默认选择正径向分支；任何关节超限均直接返回
 * ARM_IK_JOINT_LIMIT，函数不会钳制关节角。
 */
ArmIkStatus ArmIk_Solve(const ArmIkConfig *config,
                        float x_mm,
                        float y_mm,
                        float z_mm,
                        ArmJointAngles *result);

/* 指定末端位于底座轴线的正径向或负径向分支。 */
ArmIkStatus ArmIk_SolveBranch(const ArmIkConfig *config,
                              float x_mm,
                              float y_mm,
                              float z_mm,
                              ArmIkRadialBranch branch,
                              ArmJointAngles *result);

/* 关节角 -> 末端 XYZ，用于计算关节测试的预期末端位置。 */
ArmIkStatus ArmIk_Forward(const ArmIkConfig *config,
                          const ArmJointAngles *joints,
                          ArmCartesianPosition *result);

const char *ArmIk_StatusString(ArmIkStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */
