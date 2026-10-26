/*
  程序说明: 机械臂几何模型、臂长配置、正运动学、逆运动学与限位校验。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

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

/*
  函数说明: 装载原机械臂臂长、末端偏置和关节限位。
  调用示例: ArmIk_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmIk_LoadDefaultConfig(ArmIkConfig *config);

/*
  函数说明: 检查三个几何关节角是否有限且处于机械限位内。
  调用示例: ArmIk_ValidateJoints(config, joints);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
*/
ArmIkStatus ArmIk_ValidateJoints(const ArmIkConfig *config,
                                 const ArmJointAngles *joints);

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
                        ArmJointAngles *result);

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
                              ArmJointAngles *result);

/*
  函数说明: 由三个几何关节角计算末端 XYZ 坐标。
  调用示例: ArmIk_Forward(config, joints, result);
  参数意义: config 为对应模块的配置结构。
            joints 为三个几何关节角，度。
            result 为输出计算结果。
*/
ArmIkStatus ArmIk_Forward(const ArmIkConfig *config,
                          const ArmJointAngles *joints,
                          ArmCartesianPosition *result);

/*
  函数说明: 把运动学状态码转换为只读诊断字符串。
  调用示例: ArmIk_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmIk_StatusString(ArmIkStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_KINEMATICS_H */
