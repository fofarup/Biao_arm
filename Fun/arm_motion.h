#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "arm_kinematics.h"
#include "arm_motor_mapping.h"
#include "arm_trajectory.h"

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
    ARM_MOTION_POSITION_UNKNOWN,
    ARM_MOTION_REACHED_TIMEOUT,
    ARM_MOTION_DRIVER_ERROR,
    ARM_MOTION_BUSY,
    ARM_MOTION_TRAJECTORY_ERROR,
    ARM_MOTION_FEEDBACK_TIMEOUT,
    ARM_MOTION_DEADLINE_MISSED,
    ARM_MOTION_DIRECTION_REVERSAL,
    ARM_MOTION_CANCELLED
} ArmMotionStatus;

typedef enum
{
    ARM_MOTION_TRAJECTORY_IDLE = 0,
    ARM_MOTION_TRAJECTORY_PRECHECKING,
    ARM_MOTION_TRAJECTORY_WAITING_FEEDBACK,
    ARM_MOTION_TRAJECTORY_RUNNING_SPEED,
    ARM_MOTION_TRAJECTORY_STOPPING_SPEED,
    ARM_MOTION_TRAJECTORY_SETTLING_POSITION,
    ARM_MOTION_TRAJECTORY_DONE,
    ARM_MOTION_TRAJECTORY_FAULT
} ArmMotionTrajectoryState;

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

    /* 贝塞尔轨迹流参数。 */
    uint16_t trajectory_period_ms;
    uint16_t trajectory_feedback_timeout_ms;
    uint16_t trajectory_deadline_tolerance_ms;
    uint16_t trajectory_heartbeat_ms;
    float trajectory_kp_rpm_per_deg[3];
    float trajectory_correction_limit_rpm[3];
    float trajectory_feedback_error_limit_deg[3];
    float trajectory_limit_scale;
    float trajectory_settle_time_s;
    float trajectory_min_radial_mm;

    /* 绝对位置运动前必须由应用明确确认三台驱动器处于机械零位。 */
    bool zero_confirmed;
} ArmMotionConfig;

/* 单个驱动器的低频诊断快照。位置单位为电机轴角度。 */
typedef struct
{
    bool status_valid;
    uint8_t status_flags;
    bool position_reached_flag;
    bool realtime_position_valid;
    float realtime_position_deg;
    uint32_t realtime_position_update_ms;
    bool realtime_speed_valid;
    float realtime_speed_rpm;
    uint32_t realtime_speed_update_ms;
    bool position_error_valid;
    float position_error_deg;
    uint32_t position_error_update_ms;
    uint32_t status_update_ms;
    uint32_t last_update_ms;
    uint8_t last_response_code;
    uint8_t last_error_code;
} ArmMotionDriverStatus;

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

typedef struct
{
    ArmMotionTrajectoryState state;
    ArmMotionStatus status;
    ArmTrajectoryStatus trajectory_status;
    ArmIkStatus ik_status;
    float requested_time_s;
    /*
     * 相同 P0/P1/P2/P3 与配置下已通过离散校验的安全时间上界；
     * max_speed/max_acc/max_dec/final_speed 仍是本次
     * requested_time_s 的调试指标。
     */
    float minimum_time_s;
    uint32_t failed_sample;
    float max_speed_rpm[3];
    float max_acc_rpm_per_s[3];
    float max_dec_rpm_per_s[3];
    float final_speed_rpm[3];
    ArmCartesianPosition start;
    ArmCartesianPosition target;
} ArmMotionTrajectoryResult;

/* 运动中不会覆写当前活动配置；请只在 IDLE 阶段调用。 */
void ArmMotion_LoadDefaultConfig(ArmMotionConfig *config);

/*
 * 绑定单机械臂的活动配置。简化的 MoveToXYZ 接口使用该配置。
 * 当前工程是单机械臂实例，因此一次只绑定一个配置。
 */
ArmMotionStatus ArmMotion_SetActiveConfig(ArmMotionConfig *config);

/*
 * 这里只记录零位已由操作者/外部归零流程确认，不会修改驱动器零点。
 * 运动中的修改会被忽略；请只在 IDLE 阶段调用。
 */
void ArmMotion_ConfirmZero(ArmMotionConfig *config, bool confirmed);

/* 选择 MoveToXYZ 使用的正/负径向逆解分支。 */
ArmMotionStatus ArmMotion_SetIkBranch(ArmMotionConfig *config,
                                      ArmIkRadialBranch branch);

ArmMotionStatus ArmMotion_EnableAll(const ArmMotionConfig *config,
                                    bool state);

ArmMotionStatus ArmMotion_StopAll(const ArmMotionConfig *config);

/*
 * MoveToXYZ 到位超时且三轴已停止后调用：限时确认 35 速度为零、
 * 3A 无故障，再用停稳后的 36 实际位置重建下一条运动的起点。
 * 恢复失败时位置保持未知，不允许继续下发定时 XYZ 运动。
 */
ArmMotionStatus ArmMotion_RecoverAfterTimeout(const ArmMotionConfig *config);

/*
 * 末端位置控制：时间单位为秒。
 * 根据三轴位移和外部指定时间，自动计算每轴的加速度、
 * 减速度和最大速度；在满足时间误差和每轴配置上限的前提下，
 * 优先选择最低的 max(加速度, 减速度)，以减小机械冲击。
 * 三轴使用一条多电机命令同时启动，并按同一时间到达。
 * 成功下发后函数阻塞等待所有参与运动的电机返回 FD 9F 到位帧；
 * 36/37/3A 只用于低频位置、误差和状态诊断。全部到位后立即返回，
 * 不再额外等待固定时间；超时或驱动器报警时停止三轴并返回错误。
 * 调用前必须先调用 ArmMotion_SetActiveConfig() 并确认机械零位。
 * Gamma 或其他已启用关节限位超限时，不会下发 CAN 运动命令。
 */
ArmMotionStatus ArmMotion_MoveToXYZ(float x_mm,
                                    float y_mm,
                                    float z_mm,
                                    float time_s);

/* 返回最近一次 MoveToXYZ 计算的关节、电机和梯形轨迹参数。 */
const ArmMotionResult *ArmMotion_GetLastResult(void);

/*
 * 从新的 36 反馈位置 P0 出发，沿 P0/P1/P2/P3 三次贝塞尔曲线运动。
 * P1/P2/P3 是基坐标系绝对坐标；曲线只经过 P0/P3。time_s 按 1 ms
 * 量化，表示速度流参考轨迹时间；真实完成还要等待末尾 FD 9F 收尾。
 * Start/Service 为非阻塞接口；MoveBezier 为阻塞兼容包装。
 */
ArmMotionStatus ArmMotion_StartBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s);

ArmMotionStatus ArmMotion_ServiceTrajectory(void);
bool ArmMotion_TrajectoryIsBusy(void);
ArmMotionStatus ArmMotion_AbortTrajectory(void);

ArmMotionStatus ArmMotion_MoveBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s);

const ArmMotionTrajectoryResult *ArmMotion_GetTrajectoryResult(void);

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

/* 获取某个地址对应的低频诊断快照，motor_index 为 0、1、2。 */
ArmMotionStatus ArmMotion_GetDriverStatus(uint8_t motor_index,
                                          ArmMotionDriverStatus *status);

const char *ArmMotion_StatusString(ArmMotionStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_MOTION_H */
