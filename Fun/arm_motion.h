#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "arm_kinematics.h"
#include "arm_motor_mapping.h"

typedef enum
{
    ARM_MOTION_OK = 0,
    ARM_MOTION_NULL_POINTER,
    ARM_MOTION_INVALID_CONFIG,
    ARM_MOTION_NOT_ZEROED,
    ARM_MOTION_IK_ERROR,
    ARM_MOTION_MAPPING_ERROR,
    ARM_MOTION_CAN_ERROR,
    ARM_MOTION_NOT_INITIALIZED,
    ARM_MOTION_TIME_INVALID,
    ARM_MOTION_TIME_UNACHIEVABLE,
    ARM_MOTION_POSITION_UNKNOWN
} ArmMotionStatus;

/*
 * 三层配置明确分开：
 * kinematics 只含几何和机械限位；motor_map 只含减速比、耦合和方向映射；
 * 本结构剩余字段只负责运动命令与 CAN 下发参数。
 */
typedef struct
{
    ArmIkConfig kinematics;
    ArmMotorMapConfig motor_map;
    ArmInstallationDirection installation_direction;
    ArmIkRadialBranch ik_branch;

    uint8_t motor_addr[3];
    uint16_t acc_rpm_per_s[3];
    uint16_t dec_rpm_per_s[3];
    float vel_rpm[3];

    bool use_multi_command;
    bool use_sync;
    bool report_reached[3];

    /* 绝对位置运动前必须由应用明确确认三台驱动器处于机械零位。 */
    bool zero_confirmed;
} ArmMotionConfig;

typedef struct
{
    uint16_t acc_rpm_per_s[3];
    uint16_t dec_rpm_per_s[3];
    float vel_rpm[3];
    float predicted_time_s[3];
    float requested_time_s;
} ArmMotionProfile;

typedef struct
{
    ArmCartesianPosition position;
    ArmJointAngles joints;
    ArmMotorTargets motors;
    ArmMotionProfile profile;
} ArmMotionResult;

void ArmMotion_LoadDefaultConfig(ArmMotionConfig *config);

/*
 * 绑定单机械臂的活动配置。简化的 MoveToXYZ 接口使用该配置。
 * 当前工程是单机械臂实例，因此一次只绑定一个配置。
 */
ArmMotionStatus ArmMotion_SetActiveConfig(ArmMotionConfig *config);

/* 这里只记录零位已由操作者/外部归零流程确认，不会修改驱动器零点。 */
void ArmMotion_ConfirmZero(ArmMotionConfig *config, bool confirmed);

/* 选择 MoveToXYZ 使用的正/负径向逆解分支。 */
ArmMotionStatus ArmMotion_SetIkBranch(ArmMotionConfig *config,
                                      ArmIkRadialBranch branch);

ArmMotionStatus ArmMotion_EnableAll(const ArmMotionConfig *config,
                                    bool state);

ArmMotionStatus ArmMotion_StopAll(const ArmMotionConfig *config);

/*
 * 末端位置控制：时间单位为秒。
 * 根据三轴位移和外部指定时间，自动计算每轴的加速度、
 * 减速度和最大速度；在满足时间误差和每轴配置上限的前提下，
 * 优先选择最低的 max(加速度, 减速度)，以减小机械冲击。
 * 三轴使用一条多电机命令同时启动，并按同一时间到达。
 * 成功下发后函数阻塞等待“指定时间 + 50 ms”再返回；失败立即返回。
 * 调用前必须先调用 ArmMotion_SetActiveConfig() 并确认机械零位。
 * Gamma 或其他已启用关节限位超限时，不会下发 CAN 运动命令。
 */
ArmMotionStatus ArmMotion_MoveToXYZ(float x_mm,
                                    float y_mm,
                                    float z_mm,
                                    float time_s);

/* 返回最近一次 MoveToXYZ 计算的关节、电机和梯形轨迹参数。 */
const ArmMotionResult *ArmMotion_GetLastResult(void);

/* 几何关节角控制，仍先经过机械限位，再进行电机映射。 */
ArmMotionStatus ArmMotion_MoveToJoints(const ArmMotionConfig *config,
                                       const ArmJointAngles *joints,
                                       ArmMotionResult *result,
                                       ArmIkStatus *ik_status);

/*
 * 用户关节位移控制：q=(0,0,0) 为机械零位。
 * 大臂 q2 单独增加时允许 Gamma 按机构耦合自然变化约 q2/3。
 */
ArmMotionStatus ArmMotion_MoveJointOffsets(const ArmMotionConfig *config,
                                            float base_deg,
                                            float large_arm_deg,
                                            float small_arm_deg,
                                            ArmMotionResult *result,
                                            ArmIkStatus *ik_status);

/* 配置三台电机的 5.5.1 定时返回信息；period_ms=0 表示停止返回。 */
ArmMotionStatus ArmMotion_SetTimedReturnInfo(const ArmMotionConfig *config,
                                             uint8_t info_function,
                                             uint16_t period_ms);

const char *ArmMotion_StatusString(ArmMotionStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_MOTION_H */
