#ifndef ARM_TRAJECTORY_H
#define ARM_TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "arm_kinematics.h"

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

/* 使用五次时间函数 s=10u^3-15u^4+6u^5 初始化三次贝塞尔轨迹。 */
ArmTrajectoryStatus ArmTrajectory_InitCubicBezier(
    ArmCubicBezierTrajectory *trajectory,
    const ArmCartesianPosition *start,
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *end,
    float duration_s,
    uint16_t sample_period_ms);

/* 按实际经过时间求末端位置、速度和加速度；时间会钳制到 [0,T]。 */
ArmTrajectoryStatus ArmTrajectory_Evaluate(
    const ArmCubicBezierTrajectory *trajectory,
    float elapsed_s,
    ArmTrajectorySample *sample);

const char *ArmTrajectory_StatusString(ArmTrajectoryStatus status);

#ifdef __cplusplus
}
#endif

#endif /* ARM_TRAJECTORY_H */
