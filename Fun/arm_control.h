/*
  程序说明: 机械臂电机映射、同步点位规划及微动处理、曲线控制和真实到位处理。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

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

/* 未量化的电机轴绝对坐标，正负号使用本项目的逻辑正方向。 */
typedef struct
{
    float degree[3];
} ArmMotorCoordinates;

/*
  函数说明: 装载原机械臂的减速比、脉冲数、耦合关系及方向偏置。
  调用示例: ArmMotorMap_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmMotorMap_LoadDefaultConfig(ArmMotorMapConfig *config);

/*
  函数说明: 把相对机械零位的关节偏移换算为几何关节角。
  用户关节位移 q1/q2/q3 -> 几何 Alpha/Beta/Gamma。
  q2 单独增加时地址3不补偿，因此 Gamma 按 4.5/13.5 的耦合量自然变化。
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
    ArmJointAngles *result);

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
    int32_t offset_pulse);

/*
  函数说明: 清除指定安装方向的底座偏置有效标志。
  调用示例: ArmMotorMap_ClearDirectionOffset(config, direction);
  参数意义: config 为对应模块的配置结构。
            direction 为机械安装方向枚举。
*/
ArmMotorMapStatus ArmMotorMap_ClearDirectionOffset(
    ArmMotorMapConfig *config,
    ArmInstallationDirection direction);

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
    ArmMotorTargets *result);

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
    ArmMotorCoordinates *result);

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
    ArmJointAngles *result);

/*
  函数说明: 把电机映射状态码转换为只读诊断字符串。
  调用示例: ArmMotorMap_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmMotorMap_StatusString(ArmMotorMapStatus status);



typedef enum
{
    ARM_TRAJECTORY_OK = 0,
    ARM_TRAJECTORY_NULL_POINTER,
    ARM_TRAJECTORY_INVALID_ARGUMENT,
    ARM_TRAJECTORY_NUMERIC_ERROR
} ArmTrajectoryStatus;

/*
 * 三次贝塞尔末端轨迹。P0/P3 是起终点，P1/P2 控制离开和接近方向。
 * duration_s 是规划运动时间；sample_period_ms 是执行器的目标更新周期。
 */
typedef struct
{
    ArmCartesianPosition point[4];
    float duration_s;
    uint32_t duration_ms;
    uint16_t sample_period_ms;
    uint32_t segment_count;
    uint8_t initialized;
} ArmCubicBezierTrajectory;

typedef struct
{
    ArmCartesianPosition position;
    ArmCartesianPosition velocity_mm_s;
    ArmCartesianPosition acceleration_mm_s2;
    float path_parameter;
    float elapsed_s;
} ArmTrajectorySample;

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
    uint16_t sample_period_ms);

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
    ArmTrajectorySample *sample);

/*
  函数说明: 把曲线计算状态码转换为只读诊断字符串。
  调用示例: ArmTrajectory_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmTrajectory_StatusString(ArmTrajectoryStatus status);

#include <stdbool.h>

typedef enum
{
    ARM_CONTROL_OK = 0,
    ARM_CONTROL_NULL_POINTER,
    ARM_CONTROL_INVALID_CONFIG,
    ARM_CONTROL_NOT_ZEROED,
    ARM_CONTROL_IK_ERROR,
    ARM_CONTROL_MAPPING_ERROR,
    ARM_CONTROL_CAN_ERROR,
    ARM_CONTROL_NOT_INITIALIZED,
    ARM_CONTROL_TIME_INVALID,
    ARM_CONTROL_TIME_UNACHIEVABLE,
    ARM_CONTROL_POSITION_UNKNOWN,
    ARM_CONTROL_REACHED_TIMEOUT,
    ARM_CONTROL_DRIVER_ERROR,
    ARM_CONTROL_BUSY,
    ARM_CONTROL_TRAJECTORY_ERROR,
    ARM_CONTROL_FEEDBACK_TIMEOUT,
    ARM_CONTROL_DEADLINE_MISSED,
    ARM_CONTROL_DIRECTION_REVERSAL,
    ARM_CONTROL_CANCELLED
} ArmControlStatus;

typedef enum
{
    ARM_CONTROL_TRAJECTORY_IDLE = 0,
    ARM_CONTROL_TRAJECTORY_PRECHECKING,
    ARM_CONTROL_TRAJECTORY_WAITING_FEEDBACK,
    ARM_CONTROL_TRAJECTORY_RUNNING_SPEED,
    ARM_CONTROL_TRAJECTORY_STOPPING_SPEED,
    ARM_CONTROL_TRAJECTORY_SETTLING_POSITION,
    ARM_CONTROL_TRAJECTORY_DONE,
    ARM_CONTROL_TRAJECTORY_FAULT
} ArmControlTrajectoryState;

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
} ArmControlConfig;

/* 单个驱动器的低频诊断快照。位置单位为电机轴角度。 */
typedef struct
{
    bool heartbeat_valid; /* 非零表示收到过 16 心跳参数回复；不能把未收到误认为 0 ms。 */
    uint32_t heartbeat_timeout_ms;
    uint32_t heartbeat_update_ms;
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
} ArmControlDriverStatus;

typedef struct
{
    uint16_t acc_rpm_per_s[3];
    uint16_t dec_rpm_per_s[3];
    float vel_rpm[3];
    float predicted_time_s[3]; /* 各轴预计到位时间；正常轴同步，微动轴可提前到位。 */
    float planned_time_s;     /* 最慢轴预计时间，用于到位等待超时计算。 */
    uint8_t small_move_mask;  /* 自动模式下豁免同步的微动轴；bit0/1/2 对应电机 1/2/3。 */
} ArmControlProfile;

typedef struct
{
    ArmCartesianPosition position;
    ArmJointAngles joints;
    ArmMotorTargets motors;
    ArmControlProfile profile;
} ArmControlResult;

typedef struct
{
    ArmControlTrajectoryState state;
    ArmControlStatus status;
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
} ArmControlTrajectoryResult;

/*
  函数说明: 装载原机械参数、方向、三轴速度和加减速上限及诊断配置。
  调用示例: ArmControl_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
void ArmControl_LoadDefaultConfig(ArmControlConfig *config);

/*
  函数说明: 绑定单机械臂活动配置；正在执行运动时拒绝替换。
  调用示例: ArmControl_SetActiveConfig(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_SetActiveConfig(ArmControlConfig *config);

/*
  函数说明: 记录操作者已确认的机械零位状态；不发送驱动器清零命令。
  调用示例: ArmControl_ConfirmZero(config, confirmed);
  参数意义: config 为对应模块的配置结构。
            confirmed 为是否已人工或由外部流程确认机械零位。
*/
void ArmControl_ConfirmZero(ArmControlConfig *config, bool confirmed);

/*
  函数说明: 设置后续 XYZ 运动使用的径向逆解分支。
  调用示例: ArmControl_SetIkBranch(config, branch);
  参数意义: config 为对应模块的配置结构。
            branch 为逆解径向分支。
*/
ArmControlStatus ArmControl_SetIkBranch(ArmControlConfig *config,
                                      ArmIkRadialBranch branch);

/*
  函数说明: 依次使能或失能三个机械臂位置电机，并维护位置有效状态。
  调用示例: ArmControl_EnableAll(config, state);
  参数意义: config 为对应模块的配置结构。
            state 为true 使能，false 失能。
*/
ArmControlStatus ArmControl_EnableAll(const ArmControlConfig *config,
                                    bool state);

/*
  函数说明: 使旧运动代次失效，停止三轴并撤销未完成目标。
  调用示例: ArmControl_StopAll(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_StopAll(const ArmControlConfig *config);

/*
  函数说明: 停车后限时核验实际速度、故障与位置，恢复下一次运动的可信起点。
  MoveToXYZ 到位超时且三轴已停止后调用：限时确认 35 速度为零、
  3A 无故障，再用停稳后的 36 实际位置重建下一条运动的起点。
  恢复失败时位置保持未知，不允许继续下发XYZ 运动。
  调用示例: ArmControl_RecoverAfterTimeout(config);
  参数意义: config 为对应模块的配置结构。
*/
ArmControlStatus ArmControl_RecoverAfterTimeout(const ArmControlConfig *config);

/*
  函数说明: 以毫米坐标控制机械臂，同时启动、正常轴同步到位、微动轴可提前到位，等待全部反馈。
  XYZ 点位运动从各轴最快时间出发自动求较短共同时间；调用方只传毫米坐标。
  统一由 ArmControl_Init 初始化。成功返回前等待所有参与轴的 FD 9F。
  速度及加减速度始终不超过配置上限；正常轴预计到位差不超过 10 ms。
  若按加减速上限降至 0.1 RPM 仍会提前到位，视为微动轴并允许提前到位。
  末端途中路径由各轴运动共同决定；没有延迟启动或增加小位移死区。
  到位超时先停止，再尝试反馈恢复；返回错误时不会发送成功应答。
  调用示例: status = ArmControl_MoveToXYZ(150.0f, 0.0f, 150.0f); 坐标须符合实际机构可达范围。
  参数意义: x_mm 为机械臂基坐标系 X 坐标，mm。
            y_mm 为机械臂基坐标系 Y 坐标，mm。
            z_mm 为机械臂基坐标系 Z 坐标，mm。
*/
ArmControlStatus ArmControl_MoveToXYZ(float x_mm,
                                    float y_mm,
                                    float z_mm);

/*
  函数说明: 按指定同步时长规划 XYZ 动作，并等待驱动器实际到位；时长无法实现时返回错误。
  调用示例: status = ArmControl_MoveToXYZTimed(0.0f, 300.0f, 260.0f, 2.0f);
  参数意义: x_mm、y_mm、z_mm 为机械臂基坐标系绝对坐标，mm。
            time_s 为正的计划时长，秒；仍受速度、加减速及协议量化约束。
*/
ArmControlStatus ArmControl_MoveToXYZTimed(
    float x_mm, float y_mm, float z_mm, float time_s);

/*
  函数说明: 返回最近一次 XYZ 逆解、目标脉冲和规划结果的只读指针。
  调用示例: ArmControl_GetLastResult();
  参数意义: 无。
*/
const ArmControlResult *ArmControl_GetLastResult(void);

/*
  函数说明: 加锁后启动一条曲线；返回成功只表示启动成功，需继续调用 ServiceTrajectory。
  从新的 36 反馈位置 P0 出发，沿 P0/P1/P2/P3 三次贝塞尔曲线运动。
  P1/P2/P3 是基坐标系绝对坐标；曲线只经过 P0/P3。time_s 按 1 ms
  量化，表示速度流参考轨迹时间；真实完成还要等待末尾 FD 9F 收尾。
  Start/Service 为非阻塞接口；MoveBezier 为阻塞兼容包装。
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
    float time_s);

/*
  函数说明: 推进非阻塞曲线状态机；调用者应按配置周期持续调用。
  调用示例: ArmControl_ServiceTrajectory();
  参数意义: 无。
*/
ArmControlStatus ArmControl_ServiceTrajectory(void);
/*
  函数说明: 查询曲线状态机是否仍在处理运动。
  调用示例: ArmControl_TrajectoryIsBusy();
  参数意义: 无。
*/
bool ArmControl_TrajectoryIsBusy(void);
/*
  函数说明: 取消当前曲线，停止电机并使旧命令失效。
  调用示例: ArmControl_AbortTrajectory();
  参数意义: 无。
*/
ArmControlStatus ArmControl_AbortTrajectory(void);

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
    float time_s);

/*
  函数说明: 返回最近曲线运行状态和预检诊断的只读指针。
  调用示例: ArmControl_GetTrajectoryResult();
  参数意义: 无。
*/
const ArmControlTrajectoryResult *ArmControl_GetTrajectoryResult(void);

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
                                       ArmIkStatus *ik_status);

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
                                            ArmIkStatus *ik_status);

/*
  函数说明: 加锁后设置三轴驱动器的指定信息周期回传。
  调用示例: ArmControl_SetTimedReturnInfo(config, info_function, period_ms);
  参数意义: config 为对应模块的配置结构。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
ArmControlStatus ArmControl_SetTimedReturnInfo(const ArmControlConfig *config,
                                             uint8_t info_function,
                                             uint16_t period_ms);

/*
  函数说明: 在临界区复制一个机械臂电机的驱动器诊断快照。
  调用示例: ArmControl_GetDriverStatus(motor_index, status);
  参数意义: motor_index 为机械臂电机下标，0～2。
            status 为状态码或驱动器诊断快照。
*/
ArmControlStatus ArmControl_GetDriverStatus(uint8_t motor_index,
                                          ArmControlDriverStatus *status);

/*
  函数说明: 在主任务空闲时处理 CAN 反馈并查询三轴状态，保持空闲通信。
  调用示例: 无上位机命令时调用 ArmControl_ServiceIdle(); 随后 osDelay(5U)。
  参数意义: 无；仅任务上下文调用。未就绪、运动中、停止中或命令锁忙时直接返回。
            本接口不发运动/使能/清保护命令，也不改心跳时间。
*/
void ArmControl_ServiceIdle(void);
/* Keil Watch：维护次数、最后维护时刻及诊断查询发送失败累计数。 */
extern volatile uint32_t arm_control_idle_count;
extern volatile uint32_t arm_control_idle_last_ms;
extern volatile uint32_t arm_control_diagnostic_tx_errors;

/*
  函数说明: 把机械臂控制状态码转换为只读诊断字符串。
  调用示例: ArmControl_StatusString(status);
  参数意义: status 为状态码或驱动器诊断快照。
*/
const char *ArmControl_StatusString(ArmControlStatus status);

/*
  函数说明: 建立默认活动配置并初始化三个位置电机，沿用上电前人工摆放机械零位的约定。
  调用示例: status = ArmControl_Init();
  参数意义: 无。
*/
ArmControlStatus ArmControl_Init(void);
/*
  函数说明: 查询应用机械臂是否完成初始化且仍具备可信运动状态。
  调用示例: ArmControl_IsReady();
  参数意义: 无。
*/
uint8_t ArmControl_IsReady(void);
/*
  函数说明: 查询最近一次应用 XYZ 动作的超时恢复结果，供主机 10FF 错误帧使用。
  调用示例: recovery_status = ArmControl_GetRecoveryStatus();
  参数意义: 无；未尝试恢复或恢复成功返回 ARM_CONTROL_OK，失败返回实际状态码。
*/
ArmControlStatus ArmControl_GetRecoveryStatus(void);

#endif
