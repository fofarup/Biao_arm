#include "chassis_control.h"

#include <math.h>
#include <string.h>

#include "cmsis_os.h"
#include "hw101.h"
#include "ops9.h"

/* 位置环和航向环的 PD 初始参数，实车调试时集中修改这里。 */
#define CHASSIS_KP_POSITION  (2.0f)
#define CHASSIS_KD_POSITION  (0.05f)
#define CHASSIS_KP_YAW       (1.5f)
#define CHASSIS_KD_YAW       (0.015f)

/* 行走中航向纠偏：PD 输出角速度，独立于本次动作传入的轮速加减速。 */
#define CHASSIS_TRACK_YAW_KP              (3.0f)
#define CHASSIS_TRACK_YAW_KD              (0.02f)
#define CHASSIS_TRACK_YAW_MAX_DEG_S       (900.0f)
#define CHASSIS_TRACK_YAW_ACC_DEG_S2      (360.0f)
#define CHASSIS_TRACK_YAW_DEC_DEG_S2      (540.0f)

/* 行走中横向纠偏：沿起点到终点直线的法向修正，单位为 mm/s、mm/s²。 */
#define CHASSIS_TRACK_CROSS_KP            (2.0f)
#define CHASSIS_TRACK_CROSS_KD            (0.15f)
#define CHASSIS_TRACK_CROSS_MAX_MM_S      (3000.0f)
#define CHASSIS_TRACK_CROSS_ACC_MM_S2     (600.0f)
#define CHASSIS_TRACK_CROSS_DEC_MM_S2     (1000.0f)

/* 转弯及到位后的 XY 保持：与行走中横向纠偏分别调参。 */
#define CHASSIS_HOLD_XY_KP                (2.0f)
#define CHASSIS_HOLD_XY_KD                (0.15f)
#define CHASSIS_HOLD_XY_MAX_MM_S          (3000.0f)
#define CHASSIS_HOLD_XY_ACC_MM_S2         (600.0f)
#define CHASSIS_HOLD_XY_DEC_MM_S2         (1000.0f)

/* 最终轮速安全斜率：只约束叠加后的输出，不让动作的低斜率拖慢纠偏。 */
#define CHASSIS_CORRECTION_WHEEL_ACC_RPM_S (2000.0f)
#define CHASSIS_CORRECTION_WHEEL_DEC_RPM_S (3000.0f)
/* OPS 固定坐标转车体坐标：-1 为 R(-yaw)；车体位移转 OPS 坐标用其逆变换。 */
#define CHASSIS_COORD_ROTATION_SIGN (-1.0f)
/*
 * 控制器数据流：
 * OPS9 X/Y + HW101 航向角
 *          ↓
 * 位置/速度控制与加减速限制
 *          ↓
 * 麦克纳姆轮解算
 *          ↓
 * 四个带符号逻辑 RPM，再转换为 X 固件方向位。
 */
#define CHASSIS_PI                 (3.14159265358979323846f) /* 圆周率，用于角度换算。 */
#define CHASSIS_MIN_DT_S           (0.001f)  /* 控制周期下限，单位秒。 */
#define CHASSIS_MAX_DT_S           (0.100f)  /* 控制周期上限，单位秒。 */
#define CHASSIS_DEFAULT_SENSOR_TIMEOUT_MS (200U) /* 传感器数据超时，单位毫秒。 */
#define CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM (1.0f) /* XY 到位误差，单位毫米。 */
#define CHASSIS_SEQUENCE_YAW_TOLERANCE_DEG     (0.3f) /* 航向到位误差，单位度。 */
#define CHASSIS_SEQUENCE_WAIT_PERIOD_MS        (20U) /* 等待动作完成的检查间隔，单位毫秒。 */
#define CHASSIS_SEQUENCE_TIMEOUT_MS         (30000U) /* 单次动作超时，单位毫秒。 */
#define CHASSIS_MAX_SOFTWARE_RAMP_RPM_S        (65535.0f) /* 软件加减速参数上限，单位 RPM/s。 */
#define CHASSIS_MAX_COMMAND_RPM                (3000.0f) /* 电机命令转速上限，单位 RPM。 */

static ChassisControlConfig_t chassis_config;
static ChassisControlMode_t chassis_mode = CHASSIS_CONTROL_STOP;
static ChassisPose_t chassis_target_pose;
static ChassisPose_t chassis_current_pose;
static ChassisPose_t chassis_pose_error;
static ChassisBodyVelocity_t chassis_target_velocity;
static ChassisBodyVelocity_t chassis_current_velocity;
static float chassis_current_wheel_rpm[CHASSIS_MOTOR_COUNT];

/* 相对位移动作状态：平移阶段结束后才允许进入旋转阶段。 */
static ChassisSequencePhase_t chassis_sequence_phase =
    CHASSIS_SEQUENCE_IDLE;
static float chassis_sequence_final_yaw_deg;
static uint8_t chassis_sequence_done;

/* 固定在 OPS 坐标系的直线路径单位方向；法向为 (-unit_y, unit_x)。 */
static float chassis_path_unit_x;
static float chassis_path_unit_y;
static float chassis_cross_speed_mm_s;
static float chassis_hold_x_speed_mm_s;
static float chassis_hold_y_speed_mm_s;
static float chassis_heading_speed_deg_s;
static float chassis_turn_speed_deg_s;
/* 动作本身仍使用调用者给出的 RPM/s；纠偏有单独的斜率。 */
static float chassis_translate_wheel_rpm[CHASSIS_MOTOR_COUNT];
static float chassis_turn_wheel_rpm[CHASSIS_MOTOR_COUNT];

/* 当前动作的统一轮速限制；未启用时使用配置中的逐电机默认值。 */
static float chassis_runtime_acc_rpm_per_s[CHASSIS_MOTOR_COUNT];
static float chassis_runtime_dec_rpm_per_s[CHASSIS_MOTOR_COUNT];
static float chassis_runtime_max_rpm;
static uint8_t chassis_runtime_limits_active;

static float chassis_origin_x_mm;
static float chassis_origin_y_mm;
static float chassis_origin_yaw_deg;
static uint8_t chassis_origin_valid;
static uint8_t chassis_sensor_fault;
static uint8_t chassis_control_initialized;

static float chassis_previous_x_mm;
static float chassis_previous_y_mm;
static float chassis_previous_yaw_deg;
static uint8_t chassis_previous_sample_valid;

/**
 * 功能：判断浮点数是否为可用数值。
 * 使用方法：内部调用，用于拒绝 NaN 输入；返回 1 表示可用，0 表示 NaN。
 */
static uint8_t ChassisControl_Finite(float value)
{
    return (value == value) ? 1U : 0U;
}

/**
 * 功能：返回浮点数的绝对值。
 * 使用方法：内部调用，用于比较误差、速度和限幅，不改变原变量。
 */
static float ChassisControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * 功能：把数值限制在指定的最小值和最大值之间。
 * 使用方法：内部调用，用于限制车体速度、航向速度和控制周期；返回限幅后的值。
 */
static float ChassisControl_Clamp(float value,
                                  float minimum,
                                  float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/**
 * 功能：撤销一次相对运动临时设置的统一轮速限制，恢复配置中的逐电机限制。
 * 使用方法：内部调用；初始化、普通速度/位置控制、停止或清故障时调用。
 */
static void ChassisControl_ResetRuntimeLimits(void)
{
    uint8_t index;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_runtime_acc_rpm_per_s[index] =
            chassis_config.motor_acc_rpm_per_s[index];
        chassis_runtime_dec_rpm_per_s[index] =
            chassis_config.motor_dec_rpm_per_s[index];
    }

    chassis_runtime_max_rpm = 0.0f;
    chassis_runtime_limits_active = 0U;
}

/**
 * 功能：为当前相对运动设置四个电机共用的加速度、减速度和最大 RPM。
 * 使用方法：内部调用；参数单位分别为 RPM/s、RPM/s、RPM，成功返回 1，非法参数返回 0。
 */
static uint8_t ChassisControl_SetRuntimeLimits(
    float acc_rpm_per_s,
    float dec_rpm_per_s,
    float max_rpm)
{
    uint8_t index;

    /* 运行时统一限制不能突破 X 固件允许的 3000 RPM 速度范围。 */
    if ((ChassisControl_Finite(acc_rpm_per_s) == 0U) ||
        (ChassisControl_Finite(dec_rpm_per_s) == 0U) ||
        (ChassisControl_Finite(max_rpm) == 0U) ||
        (acc_rpm_per_s <= 0.0f) ||
        (dec_rpm_per_s <= 0.0f) ||
        (max_rpm <= 0.0f) ||
        (max_rpm > CHASSIS_MAX_COMMAND_RPM) ||
        (acc_rpm_per_s > CHASSIS_MAX_SOFTWARE_RAMP_RPM_S) ||
        (dec_rpm_per_s > CHASSIS_MAX_SOFTWARE_RAMP_RPM_S))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_runtime_acc_rpm_per_s[index] = acc_rpm_per_s;
        chassis_runtime_dec_rpm_per_s[index] = dec_rpm_per_s;
    }

    chassis_runtime_max_rpm = max_rpm;
    chassis_runtime_limits_active = 1U;
    return 1U;
}

/**
 * 功能：把角度归一化到 [-180, 180] 度，避免跨越 0/360 度时产生大误差。
 * 使用方法：内部调用，尤其用于航向角误差和传感器角度差计算。
 */
static float ChassisControl_WrapAngle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**
 * 功能：将 OPS 固定坐标系中的平移速度转换为当前车体坐标系速度。
 * 使用方法：仅对位置闭环算出的 X/Y 速度调用；直接设定的车体速度不转换。
 */
static void ChassisControl_WorldVelocityToBody(
    ChassisBodyVelocity_t *velocity,
    float yaw_deg)
{
    float angle_rad;
    float cosine;
    float sine;
    float world_x;
    float world_y;

    if (velocity == NULL)
    {
        return;
    }

    angle_rad = CHASSIS_COORD_ROTATION_SIGN * yaw_deg *
                CHASSIS_PI / 180.0f;
    cosine = cosf(angle_rad);
    sine = sinf(angle_rad);
    world_x = velocity->x_right_mm_s;
    world_y = velocity->y_forward_mm_s;
    velocity->x_right_mm_s = cosine * world_x - sine * world_y;
    velocity->y_forward_mm_s = sine * world_x + cosine * world_y;
}

/**
 * 功能：按照加速度和减速度上限限制一个标量在单个控制周期内的变化量。
 * 使用方法：内部调用；按速度绝对值区分加减速，换向时先减到零。
 */
static float ChassisControl_RateLimit(float current,
                                      float target,
                                      float acceleration,
                                      float deceleration,
                                      float dt_s)
{
    float difference;
    float limit;

    /* 换向时先刹到零，下一周期再按加速度向反方向起步。 */
    if (((current > 0.0f) && (target < 0.0f)) ||
        ((current < 0.0f) && (target > 0.0f)))
    {
        target = 0.0f;
    }

    /* 正负速度使用同一套加减速判据，避免负方向把两者用反。 */
    difference = target - current;
    if (difference == 0.0f)
    {
        return target;
    }

    limit = (ChassisControl_Abs(target) > ChassisControl_Abs(current)) ?
            acceleration : deceleration;
    if (limit <= 0.0f)
    {
        return current;
    }

    limit *= dt_s;
    if (difference > limit)
    {
        return current + limit;
    }
    if (difference < -limit)
    {
        return current - limit;
    }
    return target;
}

/**
 * 功能：限制单个轮子的 RPM 变化，并在换向时先减速到零。
 * 使用方法：内部调用；用于避免轮子直接从正转跳到反转造成冲击。
 */
static float ChassisControl_RateLimitWheel(float current,
                                           float target,
                                           float acceleration,
                                           float deceleration,
                                           float dt_s)
{
    /* 换向时先减到零，再在下一周期向反方向加速。 */
    if ((current != 0.0f) &&
        (target != 0.0f) &&
        (((current > 0.0f) && (target < 0.0f)) ||
         ((current < 0.0f) && (target > 0.0f))))
    {
        target = 0.0f;
    }

    return ChassisControl_RateLimit(current,
                                    target,
                                    acceleration,
                                    deceleration,
                                    dt_s);
}

/**
 * 功能：清除上一次直线跟踪、转弯和保持环的中间速度。
 * 使用方法：切换控制方式或开始新的相对动作时调用；不直接改动已下发的最终轮速。
 */
static void ChassisControl_ResetSequenceControl(void)
{
    chassis_path_unit_x = 0.0f;
    chassis_path_unit_y = 0.0f;
    chassis_cross_speed_mm_s = 0.0f;
    chassis_hold_x_speed_mm_s = 0.0f;
    chassis_hold_y_speed_mm_s = 0.0f;
    chassis_heading_speed_deg_s = 0.0f;
    chassis_turn_speed_deg_s = 0.0f;
    memset(chassis_translate_wheel_rpm,
           0,
           sizeof(chassis_translate_wheel_rpm));
    memset(chassis_turn_wheel_rpm,
           0,
           sizeof(chassis_turn_wheel_rpm));
}

/**
 * 功能：生成四轮零速目标。
 * 使用方法：内部调用；停止、故障或运动解算失败时调用，setpoint 不能为 NULL。
 */
static void ChassisControl_ZeroSetpoint(
    ChassisWheelSetpoint_t *setpoint)
{
    if (setpoint == NULL)
    {
        return;
    }

    /* STOP、故障或计算失败时，默认输出四个零速目标。 */
    memset(setpoint, 0, sizeof(*setpoint));
}

/**
 * 功能：检查 OPS9 和 HW101 是否有效且未超过传感器超时。
 * 使用方法：内部调用；主动闭环控制前调用，返回 1 才允许继续输出运动指令。
 */
static uint8_t ChassisControl_SensorsFresh(
    const OPS9_Data_t *ops9,
    const HW101_Data_t *hw101)
{
    uint32_t now_ms;

    /* 进入主动控制前，两路外部传感器必须都在线。 */
    if ((ops9 == NULL) || (hw101 == NULL) ||
        (ops9->valid == 0U) || (hw101->valid == 0U))
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - ops9->last_update_ms) >
        chassis_config.sensor_timeout_ms)
    {
        return 0U;
    }
    if ((now_ms - hw101->last_update_ms) >
        chassis_config.sensor_timeout_ms)
    {
        return 0U;
    }

    return 1U;
}

/**
 * 功能：按比例缩放四个轮子的目标 RPM，使所有轮子不超过当前最大转速。
 * 使用方法：内部调用；输入数组为麦克纳姆解算结果，函数原地修改数组并保持轮速比例。
 */
static void ChassisControl_ClampWheelTarget(float wheel_rpm[CHASSIS_MOTOR_COUNT])
{
    float motor_limit;
    float scale;
    float candidate;
    uint8_t index;

    /* 若某个轮超速，按比例缩放四轮，保持运动方向和比例不变。 */
    scale = 1.0f;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        motor_limit = chassis_config.motor_max_rpm[index];
        if ((chassis_runtime_limits_active != 0U) &&
            (chassis_runtime_max_rpm < motor_limit))
        {
            motor_limit = chassis_runtime_max_rpm;
        }

        if (ChassisControl_Abs(wheel_rpm[index]) >
            motor_limit)
        {
            if (ChassisControl_Abs(wheel_rpm[index]) > 0.0f)
            {
                candidate = motor_limit /
                            ChassisControl_Abs(wheel_rpm[index]);
                if (candidate < scale)
                {
                    scale = candidate;
                }
            }
        }
    }

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        wheel_rpm[index] *= scale;
    }
}

/**
 * 功能：按轮速余量加入一组轮速，不缩小此前已分配的高优先级分量。
 * 使用方法：依次加入航向、横向、沿路径分量；四轮共用同一缩放比例以保持该分量方向。
 */
static void ChassisControl_AddWheelComponent(
    float result[CHASSIS_MOTOR_COUNT],
    const float component[CHASSIS_MOTOR_COUNT])
{
    float limit;
    float candidate;
    float scale = 1.0f;
    uint8_t index;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        limit = chassis_config.motor_max_rpm[index];
        if ((chassis_runtime_limits_active != 0U) &&
            (chassis_runtime_max_rpm < limit))
        {
            limit = chassis_runtime_max_rpm;
        }

        if (component[index] > 0.0f)
        {
            candidate = (limit - result[index]) / component[index];
        }
        else if (component[index] < 0.0f)
        {
            candidate = (-limit - result[index]) / component[index];
        }
        else
        {
            continue;
        }

        if (candidate < scale)
        {
            scale = candidate;
        }
    }

    scale = ChassisControl_Clamp(scale, 0.0f, 1.0f);
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        result[index] += scale * component[index];
    }
}

/**
 * 功能：相对动作专用控制。行走时沿直线前进，同时独立修正横向漂移和航向。
 * 使用方法：Update() 已更新 OPS/HW101 位姿与动作阶段后调用；轮速分配顺序为航向、横向、进度。
 */
static uint8_t ChassisControl_UpdateSequence(
    float dt,
    float measured_x_mm_s,
    float measured_y_mm_s,
    float measured_yaw_deg_s,
    ChassisWheelSetpoint_t *setpoint)
{
    ChassisBodyVelocity_t heading_body = {0};
    ChassisBodyVelocity_t correction_body = {0};
    ChassisBodyVelocity_t translation_body = {0};
    ChassisBodyVelocity_t turn_body = {0};
    ChassisWheelVelocity_t heading_wheels;
    ChassisWheelVelocity_t correction_wheels;
    ChassisWheelVelocity_t translation_wheels;
    ChassisWheelVelocity_t turn_wheels;
    float low_wheels[CHASSIS_MOTOR_COUNT];
    float along_error;
    float along_speed;
    float cross_error;
    float cross_speed;
    float target_cross_speed = 0.0f;
    float target_heading_speed = 0.0f;
    float target_hold_x_speed = 0.0f;
    float target_hold_y_speed = 0.0f;
    float target_turn_speed = 0.0f;
    float acceleration;
    float deceleration;
    uint8_t index;

    if (chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE)
    {
        /* 用目标到当前位置的误差投影：切向负责进度，法向负责走直线。 */
        along_error =
            (chassis_pose_error.x_mm * chassis_path_unit_x) +
            (chassis_pose_error.y_mm * chassis_path_unit_y);
        along_speed =
            (measured_x_mm_s * chassis_path_unit_x) +
            (measured_y_mm_s * chassis_path_unit_y);
        cross_error =
            -(chassis_pose_error.x_mm * chassis_path_unit_y) +
             (chassis_pose_error.y_mm * chassis_path_unit_x);
        cross_speed =
            -(measured_x_mm_s * chassis_path_unit_y) +
             (measured_y_mm_s * chassis_path_unit_x);

        translation_body.x_right_mm_s = chassis_path_unit_x *
            ((chassis_config.position_kp * along_error) -
             (chassis_config.position_kd * along_speed));
        translation_body.y_forward_mm_s = chassis_path_unit_y *
            ((chassis_config.position_kp * along_error) -
             (chassis_config.position_kd * along_speed));

        target_cross_speed = ChassisControl_Clamp(
            (CHASSIS_TRACK_CROSS_KP * cross_error) -
            (CHASSIS_TRACK_CROSS_KD * cross_speed),
            -CHASSIS_TRACK_CROSS_MAX_MM_S,
            CHASSIS_TRACK_CROSS_MAX_MM_S);
    }
    else
    {
        /* 转弯和到位保持都以终点为目标，用独立的 XY 保持参数。 */
        target_hold_x_speed = ChassisControl_Clamp(
            (CHASSIS_HOLD_XY_KP * chassis_pose_error.x_mm) -
            (CHASSIS_HOLD_XY_KD * measured_x_mm_s),
            -CHASSIS_HOLD_XY_MAX_MM_S,
            CHASSIS_HOLD_XY_MAX_MM_S);
        target_hold_y_speed = ChassisControl_Clamp(
            (CHASSIS_HOLD_XY_KP * chassis_pose_error.y_mm) -
            (CHASSIS_HOLD_XY_KD * measured_y_mm_s),
            -CHASSIS_HOLD_XY_MAX_MM_S,
            CHASSIS_HOLD_XY_MAX_MM_S);
    }

    if (chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE)
    {
        /* 主动转弯仍受动作参数与常规航向加减速限制。 */
        target_turn_speed = ChassisControl_Clamp(
            (chassis_config.yaw_kp * chassis_pose_error.yaw_deg) -
            (chassis_config.yaw_kd * measured_yaw_deg_s),
            -chassis_config.max_yaw_deg_s,
            chassis_config.max_yaw_deg_s);
    }
    else
    {
        /* 行走时锁定起始航向；首次到位后继续以同一套参数保持终点航向。 */
        target_heading_speed = ChassisControl_Clamp(
            (CHASSIS_TRACK_YAW_KP * chassis_pose_error.yaw_deg) -
            (CHASSIS_TRACK_YAW_KD * measured_yaw_deg_s),
            -CHASSIS_TRACK_YAW_MAX_DEG_S,
            CHASSIS_TRACK_YAW_MAX_DEG_S);
    }

    chassis_cross_speed_mm_s = ChassisControl_RateLimit(
        chassis_cross_speed_mm_s,
        target_cross_speed,
        CHASSIS_TRACK_CROSS_ACC_MM_S2,
        CHASSIS_TRACK_CROSS_DEC_MM_S2,
        dt);
    chassis_hold_x_speed_mm_s = ChassisControl_RateLimit(
        chassis_hold_x_speed_mm_s,
        target_hold_x_speed,
        CHASSIS_HOLD_XY_ACC_MM_S2,
        CHASSIS_HOLD_XY_DEC_MM_S2,
        dt);
    chassis_hold_y_speed_mm_s = ChassisControl_RateLimit(
        chassis_hold_y_speed_mm_s,
        target_hold_y_speed,
        CHASSIS_HOLD_XY_ACC_MM_S2,
        CHASSIS_HOLD_XY_DEC_MM_S2,
        dt);
    chassis_heading_speed_deg_s = ChassisControl_RateLimit(
        chassis_heading_speed_deg_s,
        target_heading_speed,
        CHASSIS_TRACK_YAW_ACC_DEG_S2,
        CHASSIS_TRACK_YAW_DEC_DEG_S2,
        dt);
    chassis_turn_speed_deg_s = ChassisControl_RateLimit(
        chassis_turn_speed_deg_s,
        target_turn_speed,
        chassis_config.max_yaw_acc_deg_s2,
        chassis_config.max_yaw_dec_deg_s2,
        dt);

    if (chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE)
    {
        correction_body.x_right_mm_s =
            -chassis_path_unit_y * chassis_cross_speed_mm_s;
        correction_body.y_forward_mm_s =
             chassis_path_unit_x * chassis_cross_speed_mm_s;
    }
    else
    {
        correction_body.x_right_mm_s = chassis_hold_x_speed_mm_s;
        correction_body.y_forward_mm_s = chassis_hold_y_speed_mm_s;
    }

    ChassisControl_WorldVelocityToBody(
        &translation_body, chassis_current_pose.yaw_deg);
    ChassisControl_WorldVelocityToBody(
        &correction_body, chassis_current_pose.yaw_deg);
    translation_body.x_right_mm_s = ChassisControl_Clamp(
        translation_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    translation_body.y_forward_mm_s = ChassisControl_Clamp(
        translation_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    correction_body.x_right_mm_s = ChassisControl_Clamp(
        correction_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    correction_body.y_forward_mm_s = ChassisControl_Clamp(
        correction_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    heading_body.yaw_ccw_deg_s = ChassisControl_Clamp(
        chassis_heading_speed_deg_s,
        -chassis_config.max_yaw_deg_s,
        chassis_config.max_yaw_deg_s);
    turn_body.yaw_ccw_deg_s = chassis_turn_speed_deg_s;

    if ((ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &heading_body,
            &heading_wheels) == 0U) ||
        (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &correction_body,
            &correction_wheels) == 0U) ||
        (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &translation_body,
            &translation_wheels) == 0U) ||
        (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &turn_body,
            &turn_wheels) == 0U))
    {
        return 0U;
    }

    memset(low_wheels, 0, sizeof(low_wheels));
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        acceleration = chassis_runtime_acc_rpm_per_s[index];
        deceleration = chassis_runtime_dec_rpm_per_s[index];
        chassis_translate_wheel_rpm[index] =
            ChassisControl_RateLimitWheel(
                chassis_translate_wheel_rpm[index],
                translation_wheels.rpm[index],
                acceleration,
                deceleration,
                dt);
        chassis_turn_wheel_rpm[index] =
            ChassisControl_RateLimitWheel(
                chassis_turn_wheel_rpm[index],
                turn_wheels.rpm[index],
                acceleration,
                deceleration,
                dt);

        if (chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE)
        {
            heading_wheels.rpm[index] = chassis_turn_wheel_rpm[index];
            low_wheels[index] = chassis_translate_wheel_rpm[index];
        }
        else if (chassis_sequence_phase == CHASSIS_SEQUENCE_DONE)
        {
            low_wheels[index] = chassis_translate_wheel_rpm[index] +
                                chassis_turn_wheel_rpm[index];
        }
        else
        {
            low_wheels[index] = chassis_translate_wheel_rpm[index];
        }
    }

    /* 饱和时先保航向，再保横向误差，最后才降低沿路径的行走进度。 */
    memset(setpoint->signed_rpm, 0, sizeof(setpoint->signed_rpm));
    ChassisControl_AddWheelComponent(
        setpoint->signed_rpm, heading_wheels.rpm);
    ChassisControl_AddWheelComponent(
        setpoint->signed_rpm, correction_wheels.rpm);
    ChassisControl_AddWheelComponent(
        setpoint->signed_rpm, low_wheels);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_current_wheel_rpm[index] =
            ChassisControl_RateLimitWheel(
                chassis_current_wheel_rpm[index],
                setpoint->signed_rpm[index],
                CHASSIS_CORRECTION_WHEEL_ACC_RPM_S,
                CHASSIS_CORRECTION_WHEEL_DEC_RPM_S,
                dt);
        setpoint->signed_rpm[index] = chassis_current_wheel_rpm[index];
    }

    chassis_current_velocity.x_right_mm_s =
        translation_body.x_right_mm_s + correction_body.x_right_mm_s;
    chassis_current_velocity.y_forward_mm_s =
        translation_body.y_forward_mm_s + correction_body.y_forward_mm_s;
    chassis_current_velocity.yaw_ccw_deg_s =
        heading_body.yaw_ccw_deg_s + turn_body.yaw_ccw_deg_s;
    return 1U;
}

/**
 * 功能：装载当前底盘的几何尺寸、电机地址/方向、速度限制和 PD 默认参数。
 * 使用方法：系统启动时先声明 ChassisControlConfig_t，再调用本函数，随后传给 ChassisControl_Init()。
 * 返回：无；config 为 NULL 时不执行任何操作。
 */
void ChassisControl_LoadDefaultConfig(ChassisControlConfig_t *config)
{
    uint8_t index;

    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    ChassisMecanum_LoadDefaultConfig(&config->mecanum);

    config->motor_addr[0] = 5U;
    config->motor_addr[1] = 6U;
    config->motor_addr[2] = 7U;
    config->motor_addr[3] = 8U;

    /* 正逻辑方向：ID5 CW、ID6 CCW、ID7 CW、ID8 CCW。 */
    config->motor_positive_dir[0] = X_V2_DIR_CW;
    config->motor_positive_dir[1] = X_V2_DIR_CCW;
    config->motor_positive_dir[2] = X_V2_DIR_CW;
    config->motor_positive_dir[3] = X_V2_DIR_CCW;

    /* 先采用低速、可调的底盘安全默认值。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        config->motor_acc_rpm_per_s[index] = 1000.0f;
        config->motor_dec_rpm_per_s[index] = 1500.0f;
        config->motor_max_rpm[index] = 3000.0f;
    }

    config->max_x_right_mm_s = 3000.0f;
    config->max_y_forward_mm_s = 3000.0f;
    config->max_yaw_deg_s = 600.0f;
    config->max_x_acc_mm_s2 = 2000.0f;
    config->max_y_acc_mm_s2 = 2000.0f;
    config->max_yaw_acc_deg_s2 = 2000.0f;
    config->max_x_dec_mm_s2 = 2000.0f;
    config->max_y_dec_mm_s2 = 2000.0f;
    config->max_yaw_dec_deg_s2 = 2000.0f;

    /* 这是未标定前的起始值，后续应在实车上调参。 */
    config->position_kp = CHASSIS_KP_POSITION;
    config->position_kd = CHASSIS_KD_POSITION;
    config->yaw_kp = CHASSIS_KP_YAW;
    config->yaw_kd = CHASSIS_KD_YAW;
    config->sensor_timeout_ms = CHASSIS_DEFAULT_SENSOR_TIMEOUT_MS;
}

/**
 * 功能：初始化底盘控制器及其运行状态。
 * 使用方法：加载默认或自定义配置后调用一次；初始化完成后保持 STOP，需再调用速度/位置接口。
 * 返回：无；config 为 NULL 时进入未初始化故障状态。
 */
void ChassisControl_Init(const ChassisControlConfig_t *config)
{
    if (config == NULL)
    {
        chassis_control_initialized = 0U;
        chassis_mode = CHASSIS_CONTROL_FAULT;
        return;
    }

    chassis_config = *config;
    chassis_mode = CHASSIS_CONTROL_STOP;
    memset(&chassis_target_pose, 0, sizeof(chassis_target_pose));
    memset(&chassis_current_pose, 0, sizeof(chassis_current_pose));
    memset(&chassis_pose_error, 0, sizeof(chassis_pose_error));
    memset(&chassis_target_velocity, 0, sizeof(chassis_target_velocity));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    memset(chassis_current_wheel_rpm,
           0,
           sizeof(chassis_current_wheel_rpm));
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_final_yaw_deg = 0.0f;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    chassis_origin_x_mm = 0.0f;
    chassis_origin_y_mm = 0.0f;
    chassis_origin_yaw_deg = 0.0f;
    chassis_origin_valid = 0U;
    chassis_sensor_fault = 0U;
    chassis_previous_sample_valid = 0U;
    chassis_control_initialized = 1U;
    ChassisControl_ResetRuntimeLimits();
}

/**
 * 功能：设置车体速度目标，采用速度闭环并由 Update() 执行加减速限制和四轮解算。
 * 使用方法：周期任务或上层控制器调用；x/y 单位为 mm/s，yaw 单位为 deg/s，调用不阻塞。
 * 返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
 */
uint8_t ChassisControl_SetVelocity(float x_right_mm_s,
                                   float y_forward_mm_s,
                                   float yaw_ccw_deg_s)
{
    /* 速度接口只改变目标速度，实际输出仍由 Update() 做加减速限制。 */
    if ((chassis_control_initialized == 0U) ||
        (ChassisControl_Finite(x_right_mm_s) == 0U) ||
        (ChassisControl_Finite(y_forward_mm_s) == 0U) ||
        (ChassisControl_Finite(yaw_ccw_deg_s) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT))
    {
        return 0U;
    }

    chassis_target_velocity.x_right_mm_s = x_right_mm_s;
    chassis_target_velocity.y_forward_mm_s = y_forward_mm_s;
    chassis_target_velocity.yaw_ccw_deg_s = yaw_ccw_deg_s;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_VELOCITY;
    return 1U;
}

/**
 * 功能：设置相对上电原点的绝对车体目标位置和航向。
 * 使用方法：传入 x/y(mm)、yaw(deg) 后由 OPS9/HW101 位置环和航向环闭环逼近，调用不阻塞。
 * 返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
 */
uint8_t ChassisControl_SetPoseTarget(float x_mm,
                                     float y_mm,
                                     float yaw_deg)
{
    /* 绝对位置接口由 OPS9/HW101 闭环，不直接发送 FD 位置命令。 */
    if ((chassis_control_initialized == 0U) ||
        (ChassisControl_Finite(x_mm) == 0U) ||
        (ChassisControl_Finite(y_mm) == 0U) ||
        (ChassisControl_Finite(yaw_deg) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT))
    {
        return 0U;
    }

    chassis_target_pose.x_mm = x_mm;
    chassis_target_pose.y_mm = y_mm;
    chassis_target_pose.yaw_deg = yaw_deg;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_POSE;
    return 1U;
}

/**
 * 功能：执行一次“先平移 XY、再旋转航向”的相对运动，并等待到位。
 * 使用方法：只能在 RTOS 任务中调用；dx/dy 是调用瞬间车体系中的位移，单位 mm；
 *        +X 指车右，+Y 指车前；dyaw 是相对转角，单位 deg。
 *        acc/dec/max 单位分别为 RPM/s、RPM/s、RPM。底盘任务仍每 20 ms 实时控制。
 * 返回：1 表示动作到位；0 表示启动失败、故障、中断或 30 秒超时。
 */
uint8_t ChassisControl_MoveRelativeSequential(
    float dx_mm,
    float dy_mm,
    float dyaw_deg,
    float acc_rpm_per_s,
    float dec_rpm_per_s,
    float max_rpm)
{
    OPS9_Data_t ops9;
    HW101_Data_t hw101;
    ChassisControlState_t state;
    float current_x_mm;
    float current_y_mm;
    float current_yaw_deg;
    float body_to_world_rad;
    float cosine;
    float sine;
    float path_length_mm;
    uint32_t start_ms;
    uint8_t index;

    /*
     * 调用时锁定当前位置和最终目标，底盘任务在后台推进
     * TRANSLATE→ROTATE→DONE；本任务等待动作结束。
     */
    if ((chassis_control_initialized == 0U) ||
        (__get_IPSR() != 0U) ||
        (osKernelGetState() != osKernelRunning) ||
        (ChassisControl_Finite(dx_mm) == 0U) ||
        (ChassisControl_Finite(dy_mm) == 0U) ||
        (ChassisControl_Finite(dyaw_deg) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT) ||
        (chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE) ||
        (chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE))
    {
        return 0U;
    }

    (void)OPS9_GetLatest(&ops9);
    (void)HW101_GetData(&hw101);
    if (ChassisControl_SensorsFresh(&ops9, &hw101) == 0U)
    {
        return 0U;
    }

    /* 首次调用时，将第一帧有效传感器数据作为相对坐标原点。 */
    if (chassis_origin_valid == 0U)
    {
        chassis_origin_x_mm = ops9.x_mm;
        chassis_origin_y_mm = ops9.y_mm;
        chassis_origin_yaw_deg = hw101.yaw_deg;
        chassis_origin_valid = 1U;
        chassis_previous_x_mm = ops9.x_mm;
        chassis_previous_y_mm = ops9.y_mm;
        chassis_previous_yaw_deg = hw101.yaw_deg;
        chassis_previous_sample_valid = 1U;
    }

    current_x_mm = ops9.x_mm - chassis_origin_x_mm;
    current_y_mm = ops9.y_mm - chassis_origin_y_mm;
    current_yaw_deg = ChassisControl_WrapAngle(
        hw101.yaw_deg - chassis_origin_yaw_deg);

    if (ChassisControl_SetRuntimeLimits(
            acc_rpm_per_s,
            dec_rpm_per_s,
            max_rpm) == 0U)
    {
        return 0U;
    }

    chassis_current_pose.x_mm = current_x_mm;
    chassis_current_pose.y_mm = current_y_mm;
    chassis_current_pose.yaw_deg = current_yaw_deg;

    /* 将调用瞬间的车体系位移转到 OPS 固定坐标；它与位置环的坐标变换互为逆变换。 */
    body_to_world_rad = -CHASSIS_COORD_ROTATION_SIGN *
                        current_yaw_deg * CHASSIS_PI / 180.0f;
    cosine = cosf(body_to_world_rad);
    sine = sinf(body_to_world_rad);
    chassis_target_pose.x_mm = current_x_mm +
                               (cosine * dx_mm) - (sine * dy_mm);
    chassis_target_pose.y_mm = current_y_mm +
                               (sine * dx_mm) + (cosine * dy_mm);
    ChassisControl_ResetSequenceControl();
    /* 下一动作从当前轮速受控衔接，避免重置动作分量造成瞬间急停。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_translate_wheel_rpm[index] =
            chassis_current_wheel_rpm[index];
    }
    path_length_mm = sqrtf(
        (dx_mm * dx_mm) + (dy_mm * dy_mm));
    if (path_length_mm > 0.001f)
    {
        chassis_path_unit_x =
            (chassis_target_pose.x_mm - current_x_mm) / path_length_mm;
        chassis_path_unit_y =
            (chassis_target_pose.y_mm - current_y_mm) / path_length_mm;
    }
    /* 平移阶段保持调用时的航向，旋转阶段再切换到最终航向。 */
    chassis_target_pose.yaw_deg = current_yaw_deg;
    chassis_sequence_final_yaw_deg = ChassisControl_WrapAngle(
        current_yaw_deg + dyaw_deg);
    chassis_sequence_phase = CHASSIS_SEQUENCE_TRANSLATE;
    chassis_sequence_done = 0U;
    chassis_mode = CHASSIS_CONTROL_POSE;

    start_ms = HAL_GetTick();
    for (;;)
    {
        ChassisControl_GetState(&state);
        if (state.mode == CHASSIS_CONTROL_FAULT)
        {
            return 0U;
        }
        if (state.mode != CHASSIS_CONTROL_POSE)
        {
            return 0U;
        }
        if ((state.sequence_phase == CHASSIS_SEQUENCE_DONE) &&
            (state.sequence_done != 0U))
        {
            return 1U;
        }
        if ((HAL_GetTick() - start_ms) >= CHASSIS_SEQUENCE_TIMEOUT_MS)
        {
            ChassisControl_Stop();
            return 0U;
        }
        osDelay(CHASSIS_SEQUENCE_WAIT_PERIOD_MS);
    }
}

/**
 * 功能：查询最近一次“先平移、后旋转”相对动作是否首次到位。
 * 使用方法：供其他任务或调试代码查询；返回 1 后仍持续保持目标位姿，直到停止、改目标或故障。
 */
uint8_t ChassisControl_IsMoveComplete(void)
{
    return ((chassis_sequence_phase == CHASSIS_SEQUENCE_DONE) &&
            (chassis_sequence_done != 0U)) ? 1U : 0U;
}

/**
 * 功能：取消当前速度/位置动作，并将目标速度设为零。
 * 使用方法：上层需要正常停车时调用；后续控制周期仍会按减速度把实际轮速降到零。
 */
void ChassisControl_Stop(void)
{
    if (chassis_control_initialized == 0U)
    {
        return;
    }

    chassis_target_velocity.x_right_mm_s = 0.0f;
    chassis_target_velocity.y_forward_mm_s = 0.0f;
    chassis_target_velocity.yaw_ccw_deg_s = 0.0f;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_STOP;
}

/**
 * 功能：清除控制器记录的传感器故障标志并回到 STOP 状态。
 * 使用方法：确认外部故障已排除后调用；它会重新等待有效传感器建立原点。
 */
void ChassisControl_ClearFault(void)
{
    if (chassis_control_initialized == 0U)
    {
        return;
    }

    chassis_sensor_fault = 0U;
    chassis_origin_valid = 0U;
    chassis_previous_sample_valid = 0U;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    memset(chassis_current_wheel_rpm,
           0,
           sizeof(chassis_current_wheel_rpm));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_STOP;
}

/**
 * 功能：锁存底盘故障并立即把控制目标清零。
 * 使用方法：检测到传感器、反馈或解算异常时调用；底盘任务随后应发送零速/停止命令。
 */
void ChassisControl_FaultStop(void)
{
    if (chassis_control_initialized == 0U)
    {
        return;
    }

    chassis_sensor_fault = 1U;
    chassis_mode = CHASSIS_CONTROL_FAULT;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    memset(&chassis_target_velocity, 0, sizeof(chassis_target_velocity));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    memset(chassis_current_wheel_rpm,
           0,
           sizeof(chassis_current_wheel_rpm));
}

/**
 * 功能：执行一次底盘控制周期，完成传感器采样、位置/航向 PD、速度加减速、麦克纳姆解算和轮速限幅。
 * 使用方法：由 chassisTask 以固定周期调用（当前为 20 ms），把输出 setpoint 交给电机命令构造函数。
 * 返回：1 表示本周期输出有效，0 表示参数/传感器/反馈/解算异常；setpoint 不能为 NULL。
 */
uint8_t ChassisControl_Update(float dt_s,
                              ChassisWheelSetpoint_t *setpoint)
{
    OPS9_Data_t ops9;
    HW101_Data_t hw101;
    ChassisBodyVelocity_t desired_body;
    ChassisWheelVelocity_t target_wheels;
    float measured_x_mm_s;
    float measured_y_mm_s;
    float measured_yaw_deg_s;
    float position_error_sq_mm2;
    float dt;
    uint8_t index;

    if ((setpoint == NULL) || (chassis_control_initialized == 0U))
    {
        return 0U;
    }

    ChassisControl_ZeroSetpoint(setpoint);

    dt = ChassisControl_Clamp(dt_s,
                              CHASSIS_MIN_DT_S,
                              CHASSIS_MAX_DT_S);

    (void)OPS9_GetLatest(&ops9);
    (void)HW101_GetData(&hw101);
    if (ChassisControl_SensorsFresh(&ops9, &hw101) != 0U)
    {
        if (chassis_origin_valid == 0U)
        {
            chassis_origin_x_mm = ops9.x_mm;
            chassis_origin_y_mm = ops9.y_mm;
            chassis_origin_yaw_deg = hw101.yaw_deg;
            chassis_origin_valid = 1U;
            chassis_previous_x_mm = ops9.x_mm;
            chassis_previous_y_mm = ops9.y_mm;
            chassis_previous_yaw_deg = hw101.yaw_deg;
            chassis_previous_sample_valid = 1U;
        }

        chassis_current_pose.x_mm = ops9.x_mm - chassis_origin_x_mm;
        chassis_current_pose.y_mm = ops9.y_mm - chassis_origin_y_mm;
        chassis_current_pose.yaw_deg = ChassisControl_WrapAngle(
            hw101.yaw_deg - chassis_origin_yaw_deg);

        measured_x_mm_s = 0.0f;
        measured_y_mm_s = 0.0f;
        measured_yaw_deg_s = 0.0f;
        if (chassis_previous_sample_valid != 0U)
        {
            measured_x_mm_s =
                (ops9.x_mm - chassis_previous_x_mm) / dt;
            measured_y_mm_s =
                (ops9.y_mm - chassis_previous_y_mm) / dt;
            measured_yaw_deg_s =
                ChassisControl_WrapAngle(
                    hw101.yaw_deg - chassis_previous_yaw_deg) / dt;
        }
        chassis_previous_x_mm = ops9.x_mm;
        chassis_previous_y_mm = ops9.y_mm;
        chassis_previous_yaw_deg = hw101.yaw_deg;
        chassis_previous_sample_valid = 1U;
        chassis_sensor_fault = 0U;
    }
    else if (chassis_mode != CHASSIS_CONTROL_STOP)
    {
        chassis_sensor_fault = 1U;
        chassis_mode = CHASSIS_CONTROL_FAULT;
        chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
        chassis_sequence_done = 0U;
    }

    memset(&desired_body, 0, sizeof(desired_body));

    if (chassis_mode == CHASSIS_CONTROL_VELOCITY)
    {
        desired_body = chassis_target_velocity;
    }
    else if (chassis_mode == CHASSIS_CONTROL_POSE)
    {
        chassis_pose_error.x_mm =
            chassis_target_pose.x_mm - chassis_current_pose.x_mm;
        chassis_pose_error.y_mm =
            chassis_target_pose.y_mm - chassis_current_pose.y_mm;
        chassis_pose_error.yaw_deg = ChassisControl_WrapAngle(
            chassis_target_pose.yaw_deg - chassis_current_pose.yaw_deg);
        position_error_sq_mm2 =
            (chassis_pose_error.x_mm * chassis_pose_error.x_mm) +
            (chassis_pose_error.y_mm * chassis_pose_error.y_mm);

        if ((chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE) &&
            (position_error_sq_mm2 <=
             (CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM *
              CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM)))
        {
            /* XY 合成误差达标后转弯；转弯期间仍纠正位置漂移。 */
            chassis_sequence_phase = CHASSIS_SEQUENCE_ROTATE;
            chassis_target_pose.yaw_deg = chassis_sequence_final_yaw_deg;
            chassis_pose_error.yaw_deg = ChassisControl_WrapAngle(
                chassis_target_pose.yaw_deg -
                chassis_current_pose.yaw_deg);
        }

        if ((chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE) &&
            (position_error_sq_mm2 <=
             (CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM *
              CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM)) &&
            (ChassisControl_Abs(chassis_pose_error.yaw_deg) <=
             CHASSIS_SEQUENCE_YAW_TOLERANCE_DEG))
        {
            chassis_sequence_phase = CHASSIS_SEQUENCE_DONE;
            chassis_sequence_done = 1U;
        }

        if (chassis_sequence_phase != CHASSIS_SEQUENCE_IDLE)
        {
            /* 相对动作独立计算航向、横向和行走速度，避免低速动作拖慢纠偏。 */
            if (ChassisControl_UpdateSequence(
                    dt,
                    measured_x_mm_s,
                    measured_y_mm_s,
                    measured_yaw_deg_s,
                    setpoint) == 0U)
            {
                ChassisControl_FaultStop();
                ChassisControl_ZeroSetpoint(setpoint);
                return 0U;
            }
            return 1U;
        }

        /* 非相对动作的普通绝对位置接口仍使用原来的 XY/航向闭环。 */
        desired_body.x_right_mm_s =
            (chassis_config.position_kp * chassis_pose_error.x_mm) -
            (chassis_config.position_kd * measured_x_mm_s);
        desired_body.y_forward_mm_s =
            (chassis_config.position_kp * chassis_pose_error.y_mm) -
            (chassis_config.position_kd * measured_y_mm_s);
        desired_body.yaw_ccw_deg_s =
            (chassis_config.yaw_kp * chassis_pose_error.yaw_deg) -
            (chassis_config.yaw_kd * measured_yaw_deg_s);
    }

    if (chassis_mode == CHASSIS_CONTROL_POSE)
    {
        /* 位置误差来自 OPS 固定坐标，麦克纳姆解算需要车体坐标。 */
        ChassisControl_WorldVelocityToBody(
            &desired_body, chassis_current_pose.yaw_deg);
    }

    desired_body.x_right_mm_s = ChassisControl_Clamp(
        desired_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    desired_body.y_forward_mm_s = ChassisControl_Clamp(
        desired_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    desired_body.yaw_ccw_deg_s = ChassisControl_Clamp(
        desired_body.yaw_ccw_deg_s,
        -chassis_config.max_yaw_deg_s,
        chassis_config.max_yaw_deg_s);

    if (chassis_mode == CHASSIS_CONTROL_FAULT)
    {
        memset(&desired_body, 0, sizeof(desired_body));
    }

    chassis_current_velocity.x_right_mm_s = ChassisControl_RateLimit(
        chassis_current_velocity.x_right_mm_s,
        desired_body.x_right_mm_s,
        chassis_config.max_x_acc_mm_s2,
        chassis_config.max_x_dec_mm_s2,
        dt);
    chassis_current_velocity.y_forward_mm_s = ChassisControl_RateLimit(
        chassis_current_velocity.y_forward_mm_s,
        desired_body.y_forward_mm_s,
        chassis_config.max_y_acc_mm_s2,
        chassis_config.max_y_dec_mm_s2,
        dt);
    chassis_current_velocity.yaw_ccw_deg_s = ChassisControl_RateLimit(
        chassis_current_velocity.yaw_ccw_deg_s,
        desired_body.yaw_ccw_deg_s,
        chassis_config.max_yaw_acc_deg_s2,
        chassis_config.max_yaw_dec_deg_s2,
        dt);

    if (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &chassis_current_velocity,
            &target_wheels) == 0U)
    {
        ChassisControl_FaultStop();
        ChassisControl_ZeroSetpoint(setpoint);
        return 0U;
    }

    ChassisControl_ClampWheelTarget(target_wheels.rpm);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        float acceleration = chassis_config.motor_acc_rpm_per_s[index];
        float deceleration = chassis_config.motor_dec_rpm_per_s[index];

        if (chassis_runtime_limits_active != 0U)
        {
            acceleration = chassis_runtime_acc_rpm_per_s[index];
            deceleration = chassis_runtime_dec_rpm_per_s[index];
        }

        chassis_current_wheel_rpm[index] =
            ChassisControl_RateLimitWheel(
                chassis_current_wheel_rpm[index],
                target_wheels.rpm[index],
                acceleration,
                deceleration,
                dt);
        setpoint->signed_rpm[index] = chassis_current_wheel_rpm[index];
    }

    return (chassis_mode == CHASSIS_CONTROL_FAULT) ? 0U : 1U;
}

/**
 * 功能：把四轮带符号逻辑 RPM 转换为每个 X 电机的地址、CW/CCW 方向、速度和 F6 加速度字段。
 * 使用方法：先调用 ChassisControl_Update() 得到 setpoint，再调用本函数；本函数只组包，不发送 CAN。
 */
void ChassisControl_BuildMotorCommands(
    const ChassisWheelSetpoint_t *setpoint,
    X_V2_Vel_Command_t commands[CHASSIS_MOTOR_COUNT])
{
    uint8_t index;
    X_V2_Direction_t direction;

    if ((setpoint == NULL) || (commands == NULL))
    {
        return;
    }

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        direction = chassis_config.motor_positive_dir[index];
        if (setpoint->signed_rpm[index] < 0.0f)
        {
            direction = (direction == X_V2_DIR_CW) ?
                        X_V2_DIR_CCW : X_V2_DIR_CW;
        }

        commands[index].addr = chassis_config.motor_addr[index];
        commands[index].dir = direction;
        /* 底盘 ID5~ID8 的 F6 加速度字段固定为 0；软件轮速加减速限制仍生效。 */
        commands[index].acc_rpm_per_s = 0U;
        commands[index].vel_rpm =
            ChassisControl_Abs(setpoint->signed_rpm[index]);
        commands[index].snF = false;
    }
}

/**
 * 功能：复制当前控制器状态、目标/实测位姿、误差、故障和动作阶段。
 * 使用方法：调试 Watch 或上层任务传入有效 state 指针读取；函数在临界区内复制，返回后可安全查看。
 */
void ChassisControl_GetState(ChassisControlState_t *state)
{
    uint32_t primask;

    if (state == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    state->mode = chassis_mode;
    state->target_pose = chassis_target_pose;
    state->current_pose = chassis_current_pose;
    state->pose_error = chassis_pose_error;
    state->body_velocity_command = chassis_current_velocity;
    state->origin_valid = chassis_origin_valid;
    state->sensor_fault = chassis_sensor_fault;
    state->sequence_phase = chassis_sequence_phase;
    state->sequence_done = chassis_sequence_done;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
