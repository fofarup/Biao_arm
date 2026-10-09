#include "arm_trajectory.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static int ArmTrajectory_IsFinitePosition(
    const ArmCartesianPosition *position)
{
    return (position != NULL) &&
           isfinite(position->x_mm) &&
           isfinite(position->y_mm) &&
           isfinite(position->z_mm);
}

static float ArmTrajectory_BezierValue(float p0,
                                       float p1,
                                       float p2,
                                       float p3,
                                       float s)
{
    float one_minus_s = 1.0f - s;
    float one_minus_s2 = one_minus_s * one_minus_s;
    float s2 = s * s;

    return (one_minus_s2 * one_minus_s * p0) +
           (3.0f * one_minus_s2 * s * p1) +
           (3.0f * one_minus_s * s2 * p2) +
           (s2 * s * p3);
}

static float ArmTrajectory_BezierFirstDerivative(float p0,
                                                 float p1,
                                                 float p2,
                                                 float p3,
                                                 float s)
{
    float one_minus_s = 1.0f - s;

    return (3.0f * one_minus_s * one_minus_s * (p1 - p0)) +
           (6.0f * one_minus_s * s * (p2 - p1)) +
           (3.0f * s * s * (p3 - p2));
}

static float ArmTrajectory_BezierSecondDerivative(float p0,
                                                  float p1,
                                                  float p2,
                                                  float p3,
                                                  float s)
{
    return (6.0f * (1.0f - s) * (p2 - (2.0f * p1) + p0)) +
           (6.0f * s * (p3 - (2.0f * p2) + p1));
}

static void ArmTrajectory_EvaluateAxis(float p0,
                                       float p1,
                                       float p2,
                                       float p3,
                                       float s,
                                       float ds_dt,
                                       float d2s_dt2,
                                       float *position,
                                       float *velocity,
                                       float *acceleration)
{
    float first = ArmTrajectory_BezierFirstDerivative(
        p0, p1, p2, p3, s);
    float second = ArmTrajectory_BezierSecondDerivative(
        p0, p1, p2, p3, s);

    *position = ArmTrajectory_BezierValue(p0, p1, p2, p3, s);
    *velocity = first * ds_dt;
    *acceleration = (second * ds_dt * ds_dt) +
                    (first * d2s_dt2);
}

ArmTrajectoryStatus ArmTrajectory_InitCubicBezier(
    ArmCubicBezierTrajectory *trajectory,
    const ArmCartesianPosition *start,
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *end,
    float duration_s,
    uint16_t sample_period_ms)
{
    double duration_ms_value;
    uint32_t duration_ms;
    uint32_t segment_count;

    if ((trajectory == NULL) || (start == NULL) ||
        (control1 == NULL) || (control2 == NULL) || (end == NULL))
    {
        return ARM_TRAJECTORY_NULL_POINTER;
    }

    memset(trajectory, 0, sizeof(*trajectory));
    if (!ArmTrajectory_IsFinitePosition(start) ||
        !ArmTrajectory_IsFinitePosition(control1) ||
        !ArmTrajectory_IsFinitePosition(control2) ||
        !ArmTrajectory_IsFinitePosition(end) ||
        !isfinite(duration_s) || (duration_s <= 0.0f) ||
        (sample_period_ms == 0U))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    /*
     * 执行时基为整数毫秒。先量化再用整数除法分段，避免 3.0f
     * 被表示成略大于 3000 ms 后错误地多生成一个零时长末段。
     */
    duration_ms_value = floor(((double)duration_s * 1000.0) + 0.5);
    if ((duration_ms_value < 1.0) || !isfinite(duration_ms_value) ||
        (duration_ms_value > (double)UINT_MAX))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }
    duration_ms = (uint32_t)duration_ms_value;
    segment_count = duration_ms / (uint32_t)sample_period_ms;
    if ((duration_ms % (uint32_t)sample_period_ms) != 0U)
    {
        ++segment_count;
    }
    if (segment_count == 0U)
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    trajectory->point[0] = *start;
    trajectory->point[1] = *control1;
    trajectory->point[2] = *control2;
    trajectory->point[3] = *end;
    trajectory->duration_ms = duration_ms;
    trajectory->duration_s = (float)duration_ms * 0.001f;
    trajectory->sample_period_ms = sample_period_ms;
    trajectory->segment_count = segment_count;
    trajectory->initialized = 1U;
    return ARM_TRAJECTORY_OK;
}

ArmTrajectoryStatus ArmTrajectory_Evaluate(
    const ArmCubicBezierTrajectory *trajectory,
    float elapsed_s,
    ArmTrajectorySample *sample)
{
    float u;
    float u2;
    float u3;
    float u4;
    float s;
    float ds_du;
    float d2s_du2;
    float ds_dt;
    float d2s_dt2;
    float duration_s;

    if ((trajectory == NULL) || (sample == NULL))
    {
        return ARM_TRAJECTORY_NULL_POINTER;
    }
    if ((trajectory->initialized == 0U) ||
        (trajectory->duration_ms == 0U) ||
        !isfinite(trajectory->duration_s) ||
        (trajectory->duration_s <= 0.0f) ||
        !isfinite(elapsed_s))
    {
        return ARM_TRAJECTORY_INVALID_ARGUMENT;
    }

    if (elapsed_s < 0.0f)
    {
        elapsed_s = 0.0f;
    }
    if (elapsed_s > trajectory->duration_s)
    {
        elapsed_s = trajectory->duration_s;
    }

    duration_s = trajectory->duration_s;
    u = elapsed_s / duration_s;
    u2 = u * u;
    u3 = u2 * u;
    u4 = u3 * u;

    s = (10.0f * u3) - (15.0f * u4) + (6.0f * u4 * u);
    ds_du = (30.0f * u2) - (60.0f * u3) + (30.0f * u4);
    d2s_du2 = (60.0f * u) - (180.0f * u2) + (120.0f * u3);
    ds_dt = ds_du / duration_s;
    d2s_dt2 = d2s_du2 / (duration_s * duration_s);

    memset(sample, 0, sizeof(*sample));
    sample->path_parameter = s;
    sample->elapsed_s = elapsed_s;

    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].x_mm,
        trajectory->point[1].x_mm,
        trajectory->point[2].x_mm,
        trajectory->point[3].x_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.x_mm,
        &sample->velocity_mm_s.x_mm,
        &sample->acceleration_mm_s2.x_mm);
    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].y_mm,
        trajectory->point[1].y_mm,
        trajectory->point[2].y_mm,
        trajectory->point[3].y_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.y_mm,
        &sample->velocity_mm_s.y_mm,
        &sample->acceleration_mm_s2.y_mm);
    ArmTrajectory_EvaluateAxis(
        trajectory->point[0].z_mm,
        trajectory->point[1].z_mm,
        trajectory->point[2].z_mm,
        trajectory->point[3].z_mm,
        s, ds_dt, d2s_dt2,
        &sample->position.z_mm,
        &sample->velocity_mm_s.z_mm,
        &sample->acceleration_mm_s2.z_mm);

    if (!ArmTrajectory_IsFinitePosition(&sample->position) ||
        !ArmTrajectory_IsFinitePosition(&sample->velocity_mm_s) ||
        !ArmTrajectory_IsFinitePosition(&sample->acceleration_mm_s2) ||
        !isfinite(sample->path_parameter))
    {
        memset(sample, 0, sizeof(*sample));
        return ARM_TRAJECTORY_NUMERIC_ERROR;
    }

    return ARM_TRAJECTORY_OK;
}

const char *ArmTrajectory_StatusString(ArmTrajectoryStatus status)
{
    switch (status)
    {
        case ARM_TRAJECTORY_OK: return "OK";
        case ARM_TRAJECTORY_NULL_POINTER: return "null pointer";
        case ARM_TRAJECTORY_INVALID_ARGUMENT: return "invalid argument";
        case ARM_TRAJECTORY_NUMERIC_ERROR: return "numeric error";
        default: return "unknown status";
    }
}
