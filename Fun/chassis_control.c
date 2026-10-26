/*
  程序说明: 底盘直线纠偏、相对/绝对动作、静止保持及定位异常恢复。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-26
*/

#include "chassis_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hw101.h"
#include "ops9.h"

/* X 固件的协议速度范围，属于硬件约束。 */
#define CHASSIS_MAX_COMMAND_RPM (3000.0f)

typedef enum
{
    CHASSIS_PID_CROSS = 0U,
    CHASSIS_PID_X,
    CHASSIS_PID_Y,
    CHASSIS_PID_HEADING,
    CHASSIS_PID_TURN,
    CHASSIS_PID_ALONG,
    CHASSIS_PID_COUNT
} ChassisPidIndex_t;

/* 积分输出及上周期已接受值，单位随各环为 mm/s 或 deg/s。 */
static float chassis_pid_integral[CHASSIS_PID_COUNT];
static float chassis_pid_previous[CHASSIS_PID_COUNT];
static uint8_t chassis_pid_output_limited;
static uint8_t chassis_pid_sensor_fresh;

static ChassisControlConfig_t chassis_config;
static ChassisControlMode_t chassis_mode = CHASSIS_CONTROL_STOP;
static ChassisPose_t chassis_target_pose;
static uint8_t stop_hold_valid;
static uint8_t stop_stable_frames;
static ChassisPose_t chassis_current_pose;
static ChassisPose_t chassis_pose_error;
static ChassisBodyVelocity_t chassis_target_velocity;
static ChassisBodyVelocity_t chassis_current_velocity;
static float chassis_current_wheel_rpm[CHASSIS_MOTOR_COUNT];

/* 相对位移动作状态：顺序模式进入过渡距离后允许同时平移、旋转。 */
static ChassisSequencePhase_t chassis_sequence_phase =
    CHASSIS_SEQUENCE_IDLE;
static float chassis_sequence_final_yaw_deg;
static uint8_t chassis_sequence_done;

/*
 * 同步直线转向专用状态。路径固定在 OPS9 坐标系，yaw 使用连续角度：
 * 即使 HW101 在 +/-180° 处回卷，+180° 指令也不会误走反方向。
 */
static float chassis_straight_start_x_mm;
static float chassis_straight_start_y_mm;
static float chassis_straight_start_yaw_deg;
static float chassis_straight_delta_x_mm;
static float chassis_straight_delta_y_mm;
static float chassis_straight_delta_yaw_deg;
static float chassis_straight_path_length_mm;
static float chassis_straight_path_unit_x;
static float chassis_straight_path_unit_y;
static float chassis_straight_progress;
static float chassis_straight_progress_rate;
static float chassis_straight_filtered_lead;
static float chassis_straight_yaw_unwrapped_deg;
static float chassis_straight_previous_raw_yaw_deg;
static uint8_t chassis_straight_settle_cycles;
/* 只供同步直线转向使用；按传感器实际帧间隔求速度，没有新帧时保持上次值。 */
static ChassisBodyVelocity_t chassis_straight_measured_velocity;
static float chassis_straight_previous_ops_x_mm;
static float chassis_straight_previous_ops_y_mm;
static float chassis_straight_previous_hw_yaw_deg;
static uint32_t chassis_straight_ops_frame_count;
static uint32_t chassis_straight_hw_frame_count;
static uint32_t chassis_straight_ops_sample_ms;
static uint32_t chassis_straight_hw_sample_ms;
static uint8_t chassis_straight_ops_velocity_valid;
static uint8_t chassis_straight_hw_velocity_valid;

/* Keil Watch 可直接查看：参考/实测进度、规划/实发进度速度和最终轮速缩放。 */
volatile float chassis_straight_debug_reference_progress;
volatile float chassis_straight_debug_actual_progress;
volatile float chassis_straight_debug_requested_rate;
volatile float chassis_straight_debug_command_rate;
volatile float chassis_straight_debug_wheel_scale;
volatile uint32_t chassis_straight_debug_no_feasible_rate_count;

/* 固定在 OPS 坐标系的直线路径单位方向；法向为 (-unit_y, unit_x)。 */
static float chassis_path_unit_x;
static float chassis_path_unit_y;
static float chassis_cross_speed_mm_s;
static float chassis_hold_x_speed_mm_s;
static float chassis_hold_y_speed_mm_s;
static float chassis_heading_speed_deg_s;
static float chassis_turn_speed_deg_s;
/* 行走分量使用本次动作的 RPM/s，纠偏与最终输出受公共限制约束。 */
static float chassis_translate_wheel_rpm[CHASSIS_MOTOR_COUNT];
static float chassis_turn_wheel_rpm[CHASSIS_MOTOR_COUNT];
static uint8_t chassis_rotate_settle_cycles;

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

/*
 * 可信采样与暂停状态由底盘任务独占更新。暂停只改变输出和 mode，
 * 原终点、坐标原点、执行阶段、直线路径及运行限速仍保留在原变量中。
 * 因而恢复不是重新调用相对位移接口，不会把整段距离再加一次。
 */
static OPS9_Data_t chassis_sensor_ops9;
static HW101_Data_t chassis_sensor_hw101;
static ChassisBodyVelocity_t chassis_measured_velocity;
static uint8_t chassis_ops9_sample_valid;
static uint8_t chassis_hw101_sample_valid;
static uint8_t chassis_ops9_new_sample;
static uint8_t chassis_hw101_new_sample;
static uint8_t chassis_recovery_active;
static ChassisControlMode_t chassis_resume_mode;
static uint32_t chassis_pause_started_ms;
static uint32_t chassis_pause_total_ms;
static uint32_t chassis_recovery_verify_ms;
static uint8_t chassis_recovery_verify_started;
static uint8_t chassis_recovery_ops9_frames;
static uint8_t chassis_recovery_hw101_frames;
static uint32_t chassis_recovery_count;
static uint32_t chassis_last_sensor_gap_ms;
static uint32_t chassis_motion_id;
static uint32_t chassis_settle_ops9_frame;
static uint32_t chassis_settle_hw101_frame;
static uint8_t chassis_arrival_new_pair;
static ChassisFaultReason_t chassis_fault_reason;
static ChassisFaultReason_t chassis_last_fault_reason;
static uint32_t chassis_last_fault_ms;

/*
  函数说明: 判断浮点数是否为可用数值。
  使用方法：同时拒绝 NaN 和无穷，避免超大坏数据进入坐标与角度运算。
  调用示例: ChassisControl_Finite(value);
  参数意义: value 为输入数值。
*/
static uint8_t ChassisControl_Finite(float value)
{
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}

/*
  函数说明: 返回浮点数的绝对值。
  使用方法：内部调用，用于比较误差、速度和限幅，不改变原变量。
  调用示例: ChassisControl_Abs(value);
  参数意义: value 为输入数值。
*/
static float ChassisControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/*
  函数说明: 把数值限制在指定的最小值和最大值之间。
  使用方法：内部调用，用于限制车体速度、航向速度和控制周期；返回限幅后的值。
  调用示例: ChassisControl_Clamp(value, minimum, maximum);
  参数意义: value 为输入数值。
            minimum 为允许的下限。
            maximum 为允许的上限。
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

/*
  函数说明: 清空各环积分；新动作、模式切换和传感器暂停时调用。
  调用示例: ChassisControl_ResetPid();
  参数意义: 无。
*/
static void ChassisControl_ResetPid(void)
{
    memset(chassis_pid_integral, 0, sizeof(chassis_pid_integral));
    memset(chassis_pid_previous, 0, sizeof(chassis_pid_previous));
}

/*
  函数说明: 限制实际控制输出，记录饱和状态以抑制积分继续增长。
  调用示例: output = ChassisControl_LimitOutput(output, -maximum, maximum);
  参数意义: value 为速度，minimum、maximum 为相同单位的上下限。
*/
static float ChassisControl_LimitOutput(float value, float minimum, float maximum)
{
    if ((value < minimum) || (value > maximum)) chassis_pid_output_limited = 1U;
    return ChassisControl_Clamp(value, minimum, maximum);
}

/*
  函数说明: 用公共 PID 计算速度；D 对实测速度提供阻尼，各轴积分分别保存。
  调用示例: speed = ChassisControl_Pid(CHASSIS_PID_X, error, measured_speed, maximum, dt);
  参数意义: index 为积分状态编号；error 为误差，mm 或 deg。
            measured_speed、maximum 为实测速度及输出上限，mm/s 或 deg/s。
            dt 为周期，秒。KI=0 时清空积分，传感器不新鲜时暂停积分。
*/
static float ChassisControl_Pid(ChassisPidIndex_t index, float error,
    float measured_speed, float maximum, float dt)
{
    float pd = CHASSIS_KP * error - CHASSIS_KD * measured_speed;
    float candidate, output;

    if (CHASSIS_KI <= 0.0f)
    {
        chassis_pid_integral[index] = 0.0f;
        chassis_pid_previous[index] = 0.0f;
    }
    else if (chassis_pid_sensor_fresh != 0U)
    {
        candidate = ChassisControl_Clamp(
            chassis_pid_integral[index] + CHASSIS_KI * error * dt, -maximum, maximum);
        output = pd + candidate;
        /* 仅接受未加重饱和的积分；反向误差仍允许卸掉已有积分。 */
        if (!(((output > maximum) && (error > 0.0f)) ||
              ((output < -maximum) && (error < 0.0f))))
        {
            chassis_pid_integral[index] = candidate;
        }
    }
    return ChassisControl_LimitOutput(pd + chassis_pid_integral[index], -maximum, maximum);
}

/*
  函数说明: 最终轮速生成后提交积分；限速/限加速度时撤销本周期增大的积分。
  调用示例: ChassisControl_FinishPidCycle();
  参数意义: 无。输出受限时仍允许减小已有积分。
*/
static void ChassisControl_FinishPidCycle(void)
{
    uint8_t index;
    for (index = 0U; index < CHASSIS_PID_COUNT; ++index)
    {
        if ((chassis_pid_output_limited != 0U) &&
            (fabsf(chassis_pid_integral[index]) > fabsf(chassis_pid_previous[index])))
        {
            chassis_pid_integral[index] = chassis_pid_previous[index];
        }
        chassis_pid_previous[index] = chassis_pid_integral[index];
    }
}

/*
  函数说明: 应用公共电机限制，通过单位 X/Y/yaw 的四轮解算换算车体侧限制。
  调用示例: valid = ChassisControl_ApplyCommonLimits(config);
  参数意义: config 为配置；保留更低的车体速度及逐电机限制。
            返回 1 表示有效，0 表示参数或几何尺寸无效。
*/
static uint8_t ChassisControl_ApplyCommonLimits(ChassisControlConfig_t *config)
{
    ChassisBodyVelocity_t unit = {0};
    ChassisWheelVelocity_t wheels;
    float speed[3], acceleration[3], deceleration[3], component;
    uint8_t axis, index;

    if ((config == NULL) || !ChassisControl_Finite(CHASSIS_KP) ||
        !ChassisControl_Finite(CHASSIS_KI) || !ChassisControl_Finite(CHASSIS_KD) ||
        (CHASSIS_KP < 0.0f) || (CHASSIS_KI < 0.0f) || (CHASSIS_KD < 0.0f) ||
        !ChassisControl_Finite(CHASSIS_ACC_MAX_RPM_S) || (CHASSIS_ACC_MAX_RPM_S <= 0.0f) ||
        !ChassisControl_Finite(CHASSIS_DEC_MAX_RPM_S) || (CHASSIS_DEC_MAX_RPM_S <= 0.0f) ||
        !ChassisControl_Finite(CHASSIS_SPEED_MAX_RPM) || (CHASSIS_SPEED_MAX_RPM <= 0.0f) ||
        (CHASSIS_SPEED_MAX_RPM > CHASSIS_MAX_COMMAND_RPM)) return 0U;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (!ChassisControl_Finite(config->motor_acc_rpm_per_s[index]) ||
            !ChassisControl_Finite(config->motor_dec_rpm_per_s[index]) ||
            !ChassisControl_Finite(config->motor_max_rpm[index]) ||
            (config->motor_acc_rpm_per_s[index] <= 0.0f) ||
            (config->motor_dec_rpm_per_s[index] <= 0.0f) ||
            (config->motor_max_rpm[index] <= 0.0f)) return 0U;
        config->motor_acc_rpm_per_s[index] = fminf(config->motor_acc_rpm_per_s[index], CHASSIS_ACC_MAX_RPM_S);
        config->motor_dec_rpm_per_s[index] = fminf(config->motor_dec_rpm_per_s[index], CHASSIS_DEC_MAX_RPM_S);
        config->motor_max_rpm[index] = fminf(config->motor_max_rpm[index], CHASSIS_SPEED_MAX_RPM);
    }
    speed[0] = config->max_x_right_mm_s;
    speed[1] = config->max_y_forward_mm_s;
    speed[2] = config->max_yaw_deg_s;
    for (axis = 0U; axis < 3U; ++axis)
    {
        if (!ChassisControl_Finite(speed[axis]) || (speed[axis] <= 0.0f)) return 0U;
        memset(&unit, 0, sizeof(unit));
        if (axis == 0U) unit.x_right_mm_s = 1.0f;
        else if (axis == 1U) unit.y_forward_mm_s = 1.0f;
        else unit.yaw_ccw_deg_s = 1.0f;
        if (!ChassisMecanum_SolveVelocity(&config->mecanum, &unit, &wheels)) return 0U;
        acceleration[axis] = FLT_MAX;
        deceleration[axis] = FLT_MAX;
        for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
        {
            component = fabsf(wheels.rpm[index]);
            if (component > 0.0f)
            {
                speed[axis] = fminf(speed[axis], config->motor_max_rpm[index] / component);
                acceleration[axis] = fminf(acceleration[axis], config->motor_acc_rpm_per_s[index] / component);
                deceleration[axis] = fminf(deceleration[axis], config->motor_dec_rpm_per_s[index] / component);
            }
        }
        if (!ChassisControl_Finite(acceleration[axis]) || (acceleration[axis] == FLT_MAX) ||
            !ChassisControl_Finite(deceleration[axis]) || (deceleration[axis] == FLT_MAX)) return 0U;
    }
    config->max_x_right_mm_s = speed[0];
    config->max_y_forward_mm_s = speed[1];
    config->max_yaw_deg_s = speed[2];
    config->max_x_acc_mm_s2 = acceleration[0];
    config->max_y_acc_mm_s2 = acceleration[1];
    config->max_yaw_acc_deg_s2 = acceleration[2];
    config->max_x_dec_mm_s2 = deceleration[0];
    config->max_y_dec_mm_s2 = deceleration[1];
    config->max_yaw_dec_deg_s2 = deceleration[2];
    return 1U;
}


/*
  函数说明: 撤销一次相对运动临时设置的统一轮速限制，恢复配置中的逐电机限制。
  使用方法：内部调用；初始化、普通速度/位置控制、停止或清故障时调用。
  调用示例: ChassisControl_ResetRuntimeLimits();
  参数意义: 无。
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

/*
  函数说明: 为当前相对运动设置四个电机共用的加速度、减速度和最大 RPM。
  使用方法：内部调用；参数单位分别为 RPM/s、RPM/s、RPM，成功返回 1，非法参数返回 0。
  调用示例: ChassisControl_SetRuntimeLimits(acc_rpm_per_s, dec_rpm_per_s, max_rpm);
  参数意义: acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
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
        (max_rpm > CHASSIS_MAX_COMMAND_RPM))
    {
        return 0U;
    }

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_runtime_acc_rpm_per_s[index] = fminf(acc_rpm_per_s, chassis_config.motor_acc_rpm_per_s[index]);
        chassis_runtime_dec_rpm_per_s[index] = fminf(dec_rpm_per_s, chassis_config.motor_dec_rpm_per_s[index]);
    }

    chassis_runtime_max_rpm = fminf(max_rpm, CHASSIS_SPEED_MAX_RPM);
    chassis_runtime_limits_active = 1U;
    return 1U;
}

/*
  函数说明: 把角度归一化到 [-180, 180] 度，避免跨越 0/360 度时产生大误差。
  使用方法：内部调用，尤其用于航向角误差和传感器角度差计算。
  调用示例: ChassisControl_WrapAngle(angle_deg);
  参数意义: angle_deg 为角度，度。
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

/*
  函数说明: 将 OPS 固定坐标系中的平移速度转换为当前车体坐标系速度。
  使用方法：仅对位置闭环算出的 X/Y 速度调用；直接设定的车体速度不转换。
  调用示例: ChassisControl_WorldVelocityToBody(velocity, yaw_deg);
  参数意义: velocity 为单轴或车体速度，mm/s；车体转动分量为 deg/s。
            yaw_deg 为航向或转角，度，参考系见函数说明。
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

/*
  函数说明: 按照加速度和减速度上限限制一个标量在单个控制周期内的变化量。
  使用方法：内部调用；按速度绝对值区分加减速，换向时先减到零。
  调用示例: ChassisControl_RateLimit(current, target, acceleration, deceleration, dt_s);
  参数意义: current 为当前数值。
            target 为目标值或目标位置。
            acceleration 为加速度；与输入速度对应的每秒变化量。
            deceleration 为减速度；与输入速度对应的每秒变化量。
            dt_s 为控制周期，秒。
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
        chassis_pid_output_limited = 1U;
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
        chassis_pid_output_limited = 1U;
        return current;
    }

    limit *= dt_s;
    if (difference > limit)
    {
        chassis_pid_output_limited = 1U;
        return current + limit;
    }
    if (difference < -limit)
    {
        chassis_pid_output_limited = 1U;
        return current - limit;
    }
    return target;
}

/*
  函数说明: 清除上一次直线跟踪、转弯和保持环的中间速度。
  使用方法：切换控制方式或开始新的相对动作时调用；不直接改动已下发的最终轮速。
  调用示例: ChassisControl_ResetSequenceControl();
  参数意义: 无。
*/
static void ChassisControl_ResetSequenceControl(void)
{
    ChassisControl_ResetPid();
    chassis_path_unit_x = 0.0f;
    chassis_path_unit_y = 0.0f;
    chassis_cross_speed_mm_s = 0.0f;
    chassis_straight_filtered_lead = 0.0f;
    chassis_hold_x_speed_mm_s = 0.0f;
    chassis_hold_y_speed_mm_s = 0.0f;
    chassis_heading_speed_deg_s = 0.0f;
    chassis_turn_speed_deg_s = 0.0f;
    chassis_rotate_settle_cycles = 0U;
    memset(chassis_translate_wheel_rpm,
           0,
           sizeof(chassis_translate_wheel_rpm));
    memset(chassis_turn_wheel_rpm,
           0,
           sizeof(chassis_turn_wheel_rpm));
}

/*
  函数说明: 生成四轮零速目标。
  使用方法：内部调用；停止、故障或运动解算失败时调用，setpoint 不能为 NULL。
  调用示例: ChassisControl_ZeroSetpoint(setpoint);
  参数意义: setpoint 为四轮带符号目标转速，RPM。
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

/*
  函数说明: 检查 OPS9 和 HW101 是否有效且未超过传感器超时。
  使用方法：内部调用；主动闭环控制前调用，返回 1 才允许继续输出运动指令。
  调用示例: ChassisControl_SensorsFresh(ops9, hw101);
  参数意义: ops9 为输入 OPS9 定位快照。
            hw101 为输入 HW101 航向快照。
*/
static uint8_t ChassisControl_SensorsFresh(
    const OPS9_Data_t *ops9,
    const HW101_Data_t *hw101)
{
    uint32_t now_ms;

    /* 进入主动控制前，两路外部传感器必须都在线。 */
    if ((ops9 == NULL) || (hw101 == NULL) ||
        (ops9->valid == 0U) || (hw101->valid == 0U) ||
        (ChassisControl_Finite(ops9->x_mm) == 0U) ||
        (ChassisControl_Finite(ops9->y_mm) == 0U) ||
        (ChassisControl_Finite(hw101->yaw_deg) == 0U))
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - ops9->last_update_ms) >
        CHASSIS_SENSOR_FRESH_MS)
    {
        return 0U;
    }
    if ((now_ms - hw101->last_update_ms) >
        CHASSIS_SENSOR_FRESH_MS)
    {
        return 0U;
    }

    return 1U;
}

/*
  函数说明: main_task 与位移接口共用此检查；3000 ms 旧数据不能用于开始新动作。
  调用示例: ChassisControl_SensorsReady();
  参数意义: 无。
*/
uint8_t ChassisControl_SensorsReady(void)
{
    OPS9_Data_t ops9;
    HW101_Data_t hw101;

    if ((chassis_control_initialized == 0U) ||
        (chassis_recovery_active != 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT))
    {
        return 0U;
    }
    (void)OPS9_GetLatest(&ops9);
    (void)HW101_GetData(&hw101);
    return ChassisControl_SensorsFresh(&ops9, &hw101);
}

/*
  函数说明: 返回最近可信 OPS9 帧的年龄；尚无有效帧时返回 UINT32_MAX。
  调用示例: ChassisControl_Ops9Age(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static uint32_t ChassisControl_Ops9Age(uint32_t now_ms)
{
    return (chassis_ops9_sample_valid != 0U) ?
        now_ms - chassis_sensor_ops9.last_update_ms : 0xFFFFFFFFUL;
}

/*
  函数说明: 返回最近可信 HW101 帧的年龄；尚无有效帧时返回 UINT32_MAX。
  调用示例: ChassisControl_Hw101Age(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static uint32_t ChassisControl_Hw101Age(uint32_t now_ms)
{
    return (chassis_hw101_sample_valid != 0U) ?
        now_ms - chassis_sensor_hw101.last_update_ms : 0xFFFFFFFFUL;
}

/*
  函数说明: 连续角度只对同步旋转接口有意义，普通顺序转向仍使用原航向定义。
  调用示例: ChassisControl_IsStraightSequence();
  参数意义: 无。
*/
static uint8_t ChassisControl_IsStraightSequence(void)
{
    return ((chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN) ||
            (chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE)) ?
        1U : 0U;
}

/*
 * 独立接收两路新样本，用传感器时间戳差分，避免掉帧后把整段位移
 * 除以固定 20 ms 而产生虚假的高速/D 冲击。没有新帧时不重复差分。
 * 可达范围只用于排除明显跳变，不能证明定位模块没有重置坐标原点。
 */
/*
  函数说明: 共用实际帧间隔差分。filter_valid=0 时直接取原始速度；
  非零时沿用同步直线控制的低通系数，不重复读取传感器缓存。
  调用示例: ChassisControl_MeasuredSpeed(delta, interval_s, previous_speed, filter_valid);
  参数意义: delta 为相邻帧的位置或角度变化量。
            interval_s 为相邻采样的实际间隔，秒。
            previous_speed 为上一次有效的实测速度。
            filter_valid 为非零表示已有差分历史并启用速度滤波。
*/
static float ChassisControl_MeasuredSpeed(
    float delta, float interval_s, float previous_speed, uint8_t filter_valid)
{
    float raw_speed = delta / interval_s;

    if (filter_valid != 0U)
    {
        return previous_speed + CHASSIS_STRAIGHT_TURN_VELOCITY_ALPHA *
               (raw_speed - previous_speed);
    }
    return raw_speed;
}

/*
  函数说明: 验证两路新帧的有限性及跳变范围，更新可信缓存和实测速度。
  调用示例: ChassisControl_ObserveSensors(ops9, hw101);
  参数意义: ops9 为输入 OPS9 定位快照。
            hw101 为输入 HW101 航向快照。
*/
static ChassisFaultReason_t ChassisControl_ObserveSensors(
    const OPS9_Data_t *ops9, const HW101_Data_t *hw101)
{
    uint32_t interval_ms;
    float interval_s;
    float limit;
    float dx;
    float dy;
    float yaw_delta;

    chassis_ops9_new_sample = 0U;
    chassis_hw101_new_sample = 0U;
    if ((ops9->valid != 0U) &&
        ((chassis_ops9_sample_valid == 0U) ||
         (ops9->frame_count != chassis_sensor_ops9.frame_count)))
    {
        if ((ChassisControl_Finite(ops9->x_mm) == 0U) ||
            (ChassisControl_Finite(ops9->y_mm) == 0U))
        {
            return CHASSIS_FAULT_INVALID_SAMPLE;
        }
        if (chassis_ops9_sample_valid != 0U)
        {
            interval_ms = ops9->last_update_ms - chassis_sensor_ops9.last_update_ms;
            interval_s = (float)interval_ms * 0.001f;
            dx = ops9->x_mm - chassis_sensor_ops9.x_mm;
            dy = ops9->y_mm - chassis_sensor_ops9.y_mm;
            limit = 1.5f * sqrtf(
                chassis_config.max_x_right_mm_s * chassis_config.max_x_right_mm_s +
                chassis_config.max_y_forward_mm_s * chassis_config.max_y_forward_mm_s) *
                interval_s + CHASSIS_SENSOR_POSITION_MARGIN_MM;
            if ((ChassisControl_Finite(dx) == 0U) ||
                (ChassisControl_Finite(dy) == 0U) ||
                (ChassisControl_Abs(dx) > limit) ||
                (ChassisControl_Abs(dy) > limit) ||
                (dx * dx + dy * dy > limit * limit))
            {
                return CHASSIS_FAULT_POSITION_JUMP;
            }
            if (interval_ms != 0U)
            {
                chassis_measured_velocity.x_right_mm_s =
                    ChassisControl_MeasuredSpeed(dx, interval_s, 0.0f, 0U);
                chassis_measured_velocity.y_forward_mm_s =
                    ChassisControl_MeasuredSpeed(dy, interval_s, 0.0f, 0U);
            }
        }
        chassis_sensor_ops9 = *ops9;
        chassis_ops9_sample_valid = 1U;
        chassis_ops9_new_sample = 1U;
    }
    if ((hw101->valid != 0U) &&
        ((chassis_hw101_sample_valid == 0U) ||
         (hw101->frame_count != chassis_sensor_hw101.frame_count)))
    {
        if ((ChassisControl_Finite(hw101->yaw_deg) == 0U) ||
            (ChassisControl_Abs(hw101->yaw_deg) > 180.1f))
        {
            return CHASSIS_FAULT_INVALID_SAMPLE;
        }
        if (chassis_hw101_sample_valid != 0U)
        {
            interval_ms = hw101->last_update_ms - chassis_sensor_hw101.last_update_ms;
            interval_s = (float)interval_ms * 0.001f;
            yaw_delta = ChassisControl_WrapAngle(
                hw101->yaw_deg - chassis_sensor_hw101.yaw_deg);
            limit = chassis_config.max_yaw_deg_s * interval_s +
                    CHASSIS_SENSOR_YAW_MARGIN_DEG;
            /* 超过半圈的未知角度无法用单圈 yaw 唯一还原，禁止猜圈数续跑。 */
            if ((ChassisControl_IsStraightSequence() != 0U) && (limit >= 180.0f))
            {
                return CHASSIS_FAULT_YAW_UNCERTAIN;
            }
            if (ChassisControl_Abs(yaw_delta) > limit)
            {
                return CHASSIS_FAULT_YAW_UNCERTAIN;
            }
            if (interval_ms != 0U)
            {
                chassis_measured_velocity.yaw_ccw_deg_s =
                    ChassisControl_MeasuredSpeed(yaw_delta, interval_s, 0.0f, 0U);
            }
        }
        chassis_sensor_hw101 = *hw101;
        chassis_hw101_sample_valid = 1U;
        chassis_hw101_new_sample = 1U;
        /* OPS9 掉线时，只要 HW101 仍正常，就继续记录停车滑行中的真实转角。 */
        if (ChassisControl_IsStraightSequence() != 0U)
        {
            chassis_straight_yaw_unwrapped_deg += ChassisControl_WrapAngle(
                hw101->yaw_deg - chassis_straight_previous_raw_yaw_deg);
            chassis_straight_previous_raw_yaw_deg = hw101->yaw_deg;
        }
    }
    return CHASSIS_FAULT_NONE;
}

/*
  函数说明: 不清原点/终点，只结束本次暂停的计时；Stop 和 ClearFault 用它取消续跑。
  调用示例: ChassisControl_EndSensorHold(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static void ChassisControl_EndSensorHold(uint32_t now_ms)
{
    if (chassis_recovery_active != 0U)
    {
        chassis_pause_total_ms += now_ms - chassis_pause_started_ms;
    }
    chassis_recovery_active = 0U;
    chassis_recovery_verify_started = 0U;
    chassis_recovery_ops9_frames = 0U;
    chassis_recovery_hw101_frames = 0U;
}

/*
  函数说明: 返回累计传感器暂停时长，供动作等待函数扣除暂停时间。
  调用示例: ChassisControl_PauseClock(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static uint32_t ChassisControl_PauseClock(uint32_t now_ms)
{
    return chassis_pause_total_ms + ((chassis_recovery_active != 0U) ?
        now_ms - chassis_pause_started_ms : 0U);
}

/*
  函数说明: 记录待恢复动作并进入减速等待状态，保留原点与原目标。
  调用示例: ChassisControl_BeginSensorHold(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static void ChassisControl_BeginSensorHold(uint32_t now_ms)
{
    ChassisControl_ResetPid();
    chassis_resume_mode = (chassis_mode == CHASSIS_CONTROL_POSE) ?
        CHASSIS_CONTROL_POSE : CHASSIS_CONTROL_STOP;
    chassis_pause_started_ms = now_ms;
    chassis_recovery_active = 1U;
    chassis_recovery_verify_started = 0U;
    chassis_recovery_ops9_frames = 0U;
    chassis_recovery_hw101_frames = 0U;
    chassis_last_sensor_gap_ms = 0U;
    chassis_mode = CHASSIS_CONTROL_SENSOR_HOLD;
    chassis_rotate_settle_cycles = 0U;
    chassis_straight_settle_cycles = 0U;
    /* 速度模式没有固定终点，掉线后只回 STOP，不能自动恢复无限运动。 */
    memset(&chassis_target_velocity, 0, sizeof(chassis_target_velocity));
}

/*
  函数说明: 四轮按同一比例减速，停车过程不分别改变各轮比例。
  调用示例: ChassisControl_BrakeForSensorHold(dt, setpoint);
  参数意义: dt 为控制周期，秒。
            setpoint 为四轮带符号目标转速，RPM。
*/
static uint8_t ChassisControl_BrakeForSensorHold(
    float dt, ChassisWheelSetpoint_t *setpoint)
{
    float fraction = 1.0f;
    float deceleration;
    float wheel_speed;
    uint8_t index;
    uint8_t stopped = 1U;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        wheel_speed = ChassisControl_Abs(chassis_current_wheel_rpm[index]);
        deceleration = (chassis_runtime_limits_active != 0U) ?
            chassis_runtime_dec_rpm_per_s[index] :
            chassis_config.motor_dec_rpm_per_s[index];
        if (wheel_speed > 0.05f)
        {
            fraction = fminf(fraction, deceleration * dt / wheel_speed);
        }
    }
    fraction = ChassisControl_Clamp(fraction, 0.0f, 1.0f);
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_current_wheel_rpm[index] *= 1.0f - fraction;
        if (ChassisControl_Abs(chassis_current_wheel_rpm[index]) <= 0.05f)
        {
            chassis_current_wheel_rpm[index] = 0.0f;
        }
        else
        {
            stopped = 0U;
        }
        setpoint->signed_rpm[index] = chassis_current_wheel_rpm[index];
    }
    chassis_current_velocity.x_right_mm_s *= 1.0f - fraction;
    chassis_current_velocity.y_forward_mm_s *= 1.0f - fraction;
    chassis_current_velocity.yaw_ccw_deg_s *= 1.0f - fraction;
    return stopped;
}

/*
  函数说明: 在原路径上重建参考进度，不从恢复位置再加一遍原相对指令。
  调用示例: ChassisControl_ResumeSensorMotion(now_ms);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
*/
static void ChassisControl_ResumeSensorMotion(uint32_t now_ms)
{
    float path_x = chassis_path_unit_x;
    float path_y = chassis_path_unit_y;
    float progress = 1.0f;
    float axis_progress;

    ChassisControl_ResetSequenceControl();
    chassis_path_unit_x = path_x;
    chassis_path_unit_y = path_y;
    chassis_straight_progress_rate = 0.0f;
    chassis_straight_settle_cycles = 0U;
    chassis_settle_ops9_frame = chassis_sensor_ops9.frame_count;
    chassis_settle_hw101_frame = chassis_sensor_hw101.frame_count;
    chassis_arrival_new_pair = 0U;
    if (ChassisControl_IsStraightSequence() != 0U)
    {
        if (chassis_straight_path_length_mm > CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            progress = ((chassis_current_pose.x_mm - chassis_straight_start_x_mm) *
                            chassis_straight_path_unit_x +
                        (chassis_current_pose.y_mm - chassis_straight_start_y_mm) *
                            chassis_straight_path_unit_y) / chassis_straight_path_length_mm;
        }
        if (ChassisControl_Abs(chassis_straight_delta_yaw_deg) > CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            axis_progress = (chassis_straight_yaw_unwrapped_deg -
                             chassis_straight_start_yaw_deg) / chassis_straight_delta_yaw_deg;
            progress = fminf(progress, axis_progress);
        }
        chassis_straight_progress = ChassisControl_Clamp(progress, 0.0f, 1.0f);
    }
    /* 重新建立差分历史；首次恢复控制不能拿失联前位置除以 20 ms。 */
    chassis_straight_previous_ops_x_mm = chassis_sensor_ops9.x_mm;
    chassis_straight_previous_ops_y_mm = chassis_sensor_ops9.y_mm;
    chassis_straight_previous_hw_yaw_deg = chassis_sensor_hw101.yaw_deg;
    chassis_straight_ops_sample_ms = chassis_sensor_ops9.last_update_ms;
    chassis_straight_hw_sample_ms = chassis_sensor_hw101.last_update_ms;
    chassis_straight_ops_frame_count = chassis_sensor_ops9.frame_count;
    chassis_straight_hw_frame_count = chassis_sensor_hw101.frame_count;
    chassis_straight_ops_velocity_valid = 0U;
    chassis_straight_hw_velocity_valid = 0U;
    memset(&chassis_straight_measured_velocity, 0, sizeof(chassis_straight_measured_velocity));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    chassis_sensor_fault = 0U;
    chassis_fault_reason = CHASSIS_FAULT_NONE;
    chassis_mode = chassis_resume_mode;
    ChassisControl_EndSensorHold(now_ms);
    ++chassis_recovery_count;
}

/*
  函数说明: 返回 1 表示已恢复，0 表示本周期仍需停车等待。
  调用示例: ChassisControl_UpdateSensorHold(now_ms, dt, setpoint);
  参数意义: now_ms 为当前 HAL 毫秒时刻。
            dt 为控制周期，秒。
            setpoint 为四轮带符号目标转速，RPM。
*/
static uint8_t ChassisControl_UpdateSensorHold(
    uint32_t now_ms, float dt, ChassisWheelSetpoint_t *setpoint)
{
    uint32_t ops_age = ChassisControl_Ops9Age(now_ms);
    uint32_t hw_age = ChassisControl_Hw101Age(now_ms);
    uint32_t longest_age = (ops_age > hw_age) ? ops_age : hw_age;
    uint32_t reason = CHASSIS_FAULT_NONE;
    uint8_t stopped;

    if (longest_age > chassis_last_sensor_gap_ms)
    {
        chassis_last_sensor_gap_ms = longest_age;
    }
    if (ops_age > chassis_config.sensor_timeout_ms)
    {
        reason |= CHASSIS_FAULT_OPS9_TIMEOUT;
    }
    if (hw_age > chassis_config.sensor_timeout_ms)
    {
        reason |= CHASSIS_FAULT_HW101_TIMEOUT;
    }
    if (reason != CHASSIS_FAULT_NONE)
    {
        if (chassis_fault_reason != (ChassisFaultReason_t)reason)
        {
            chassis_last_fault_ms = now_ms;
            chassis_last_fault_reason = (ChassisFaultReason_t)reason;
        }
        chassis_fault_reason = (ChassisFaultReason_t)reason;
        chassis_sensor_fault = 1U;
        chassis_mode = CHASSIS_CONTROL_FAULT;
        /* 底盘任务接下来发送立即停止，原终点与阶段仍然保留。 */
        memset(chassis_current_wheel_rpm, 0, sizeof(chassis_current_wheel_rpm));
        memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    }
    stopped = ChassisControl_BrakeForSensorHold(dt, setpoint);
    if ((ops_age > CHASSIS_SENSOR_FRESH_MS) || (hw_age > CHASSIS_SENSOR_FRESH_MS))
    {
        chassis_recovery_verify_started = 0U;
        chassis_recovery_ops9_frames = 0U;
        chassis_recovery_hw101_frames = 0U;
        return 0U;
    }
    if (chassis_recovery_verify_started == 0U)
    {
        chassis_recovery_verify_started = 1U;
        chassis_recovery_verify_ms = now_ms;
    }
    if ((chassis_ops9_new_sample != 0U) &&
        (chassis_recovery_ops9_frames < CHASSIS_SENSOR_RECOVERY_MIN_FRAMES))
    {
        ++chassis_recovery_ops9_frames;
    }
    if ((chassis_hw101_new_sample != 0U) &&
        (chassis_recovery_hw101_frames < CHASSIS_SENSOR_RECOVERY_MIN_FRAMES))
    {
        ++chassis_recovery_hw101_frames;
    }
    if ((now_ms - chassis_recovery_verify_ms >= CHASSIS_SENSOR_RECOVERY_STABLE_MS) &&
        (chassis_recovery_ops9_frames >= CHASSIS_SENSOR_RECOVERY_MIN_FRAMES) &&
        (chassis_recovery_hw101_frames >= CHASSIS_SENSOR_RECOVERY_MIN_FRAMES) &&
        (stopped != 0U) &&
        (ChassisControl_Abs(chassis_measured_velocity.x_right_mm_s) <=
         CHASSIS_SEQUENCE_TRANSLATE_SPEED_TOLERANCE_MM_S) &&
        (ChassisControl_Abs(chassis_measured_velocity.y_forward_mm_s) <=
         CHASSIS_SEQUENCE_TRANSLATE_SPEED_TOLERANCE_MM_S) &&
        (ChassisControl_Abs(chassis_measured_velocity.yaw_ccw_deg_s) <=
         CHASSIS_STRAIGHT_TURN_YAW_RATE_TOLERANCE_DEG_S))
    {
        ChassisControl_ResumeSensorMotion(now_ms);
        return 1U;
    }
    return 0U;
}

/*
  函数说明: 按 OPS9/HW101 的真实帧间隔更新同步直线转向专用速度。
  说明：控制周期可能读到同一帧；此时保持上次速度，不把重复帧误判成停车。
        新帧只做轻度低通，减少速度差分噪声放大末段的 PID 输出。
  调用示例: ChassisControl_UpdateStraightMeasuredVelocity(ops9, hw101);
  参数意义: ops9 为输入 OPS9 定位快照。
            hw101 为输入 HW101 航向快照。
*/
static void ChassisControl_UpdateStraightMeasuredVelocity(
    const OPS9_Data_t *ops9,
    const HW101_Data_t *hw101)
{
    uint32_t interval_ms;
    float interval_s;

    if ((ops9 == NULL) || (hw101 == NULL))
    {
        return;
    }

    if (ops9->frame_count != chassis_straight_ops_frame_count)
    {
        interval_ms = ops9->last_update_ms - chassis_straight_ops_sample_ms;
        if (interval_ms > 0U)
        {
            interval_s = (float)interval_ms * 0.001f;
            chassis_straight_measured_velocity.x_right_mm_s =
                ChassisControl_MeasuredSpeed(
                    ops9->x_mm - chassis_straight_previous_ops_x_mm,
                    interval_s,
                    chassis_straight_measured_velocity.x_right_mm_s,
                    chassis_straight_ops_velocity_valid);

            chassis_straight_measured_velocity.y_forward_mm_s =
                ChassisControl_MeasuredSpeed(
                    ops9->y_mm - chassis_straight_previous_ops_y_mm,
                    interval_s,
                    chassis_straight_measured_velocity.y_forward_mm_s,
                    chassis_straight_ops_velocity_valid);

            chassis_straight_previous_ops_x_mm = ops9->x_mm;
            chassis_straight_previous_ops_y_mm = ops9->y_mm;
            chassis_straight_ops_sample_ms = ops9->last_update_ms;
            chassis_straight_ops_frame_count = ops9->frame_count;
            chassis_straight_ops_velocity_valid = 1U;
        }
    }

    if (hw101->frame_count != chassis_straight_hw_frame_count)
    {
        interval_ms = hw101->last_update_ms - chassis_straight_hw_sample_ms;
        if (interval_ms > 0U)
        {
            interval_s = (float)interval_ms * 0.001f;
            chassis_straight_measured_velocity.yaw_ccw_deg_s =
                ChassisControl_MeasuredSpeed(
                    ChassisControl_WrapAngle(
                    hw101->yaw_deg - chassis_straight_previous_hw_yaw_deg),
                    interval_s,
                    chassis_straight_measured_velocity.yaw_ccw_deg_s,
                    chassis_straight_hw_velocity_valid);

            chassis_straight_previous_hw_yaw_deg = hw101->yaw_deg;
            chassis_straight_hw_sample_ms = hw101->last_update_ms;
            chassis_straight_hw_frame_count = hw101->frame_count;
            chassis_straight_hw_velocity_valid = 1U;
        }
    }
}

/*
  函数说明: 按比例缩放四个轮子的目标 RPM，使所有轮子不超过当前最大转速。
  使用方法：内部调用；输入数组为麦克纳姆解算结果，函数原地修改数组并保持轮速比例。
  调用示例: ChassisControl_ClampWheelTarget(wheel_rpm);
  参数意义: wheel_rpm 为四轮转速数组，RPM。
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

    if (scale < 1.0f) chassis_pid_output_limited = 1U;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        wheel_rpm[index] *= scale;
    }
}

/*
  函数说明: 按轮速余量加入一组轮速，不缩小此前已分配的高优先级分量。
  使用方法：依次加入航向、横向、沿路径分量；四轮共用同一缩放比例以保持该分量方向。
  调用示例: ChassisControl_AddWheelComponent(result, component);
  参数意义: result 为输出计算结果。
            component 为待叠加的运动分量。
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
    if (scale < 1.0f) chassis_pid_output_limited = 1U;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        result[index] += scale * component[index];
    }
}

/*
  函数说明: 一个进度单位对应的速度越大，允许的进度速度/加速度就越小。
  调用示例: ChassisControl_LimitStraightProgress(component, component_limit, progress_limit);
  参数意义: component 为待叠加的运动分量。
            component_limit 为该分量允许的最大绝对值。
            progress_limit 为输入输出的路径进度限制。
*/
static void ChassisControl_LimitStraightProgress(float component,
                                                 float component_limit,
                                                 float *progress_limit)
{
    float candidate;

    if ((progress_limit == NULL) ||
        (ChassisControl_Abs(component) <= CHASSIS_STRAIGHT_TURN_EPSILON))
    {
        return;
    }

    candidate = component_limit / ChassisControl_Abs(component);
    if (candidate < *progress_limit)
    {
        *progress_limit = candidate;
    }
}

/*
  函数说明: 把机体速度、角速度和四轮 RPM 限制统一换算为进度 s 的限速、加速、减速。
  横向纠偏已占用的轮速先留出来，平移与旋转再共用剩余轮速余量。
  调用示例: ChassisControl_GetStraightProgressLimits(reserved_wheel_rpm, max_rate, acc_rate, dec_rate);
  参数意义: reserved_wheel_rpm 为已分配给纠偏的四轮转速，RPM。
            max_rate 为输出最大路径进度速度，1/s。
            acc_rate 为输出允许的路径进度加速度，1/s²。
            dec_rate 为输出允许的路径进度减速度，1/s²。
*/
static uint8_t ChassisControl_GetStraightProgressLimits(
    const float reserved_wheel_rpm[CHASSIS_MOTOR_COUNT],
    float *max_rate,
    float *acc_rate,
    float *dec_rate)
{
    ChassisBodyVelocity_t unit_body = {0};
    ChassisBodyVelocity_t derivative_body = {0};
    ChassisWheelVelocity_t unit_wheels;
    ChassisWheelVelocity_t derivative_wheels;
    float wheel_limit;
    float candidate;
    float yaw_rad_per_progress;
    float acceleration_fraction = 1.0f;
    uint8_t curve_active = 0U;
    uint8_t index;

    if ((reserved_wheel_rpm == NULL) || (max_rate == NULL) ||
        (acc_rate == NULL) || (dec_rate == NULL))
    {
        return 0U;
    }

    /* 假设 s 每秒增加 1，求对应的车体速度和四轮转速系数。 */
    unit_body.x_right_mm_s = chassis_straight_delta_x_mm;
    unit_body.y_forward_mm_s = chassis_straight_delta_y_mm;
    unit_body.yaw_ccw_deg_s = chassis_straight_delta_yaw_deg;
    ChassisControl_WorldVelocityToBody(
        &unit_body, chassis_current_pose.yaw_deg);
    if (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &unit_body,
            &unit_wheels) == 0U)
    {
        return 0U;
    }

    memset(&derivative_wheels, 0, sizeof(derivative_wheels));
    if ((chassis_straight_path_length_mm >
         CHASSIS_STRAIGHT_TURN_EPSILON) &&
        (ChassisControl_Abs(chassis_straight_delta_yaw_deg) >
         CHASSIS_STRAIGHT_TURN_EPSILON))
    {
        /*
         * 四轮前馈为 W(s)=C(s)*s_dot。车身旋转时 C 随路径进度变化，
         * 因而轮速变化还含 C'(s)*s_dot²；只按 C 约束 s_ddot 会漏掉它。
         */
        yaw_rad_per_progress = CHASSIS_COORD_ROTATION_SIGN *
            chassis_straight_delta_yaw_deg * CHASSIS_PI / 180.0f;
        derivative_body.x_right_mm_s =
            -yaw_rad_per_progress * unit_body.y_forward_mm_s;
        derivative_body.y_forward_mm_s =
             yaw_rad_per_progress * unit_body.x_right_mm_s;
        if (ChassisMecanum_SolveVelocity(
                &chassis_config.mecanum,
                &derivative_body,
                &derivative_wheels) == 0U)
        {
            return 0U;
        }
        acceleration_fraction =
            1.0f - CHASSIS_STRAIGHT_TURN_CURVE_RESERVE;
        curve_active = 1U;
    }

    *max_rate = FLT_MAX;
    *acc_rate = FLT_MAX;
    *dec_rate = FLT_MAX;
    ChassisControl_LimitStraightProgress(
        unit_body.x_right_mm_s,
        chassis_config.max_x_right_mm_s,
        max_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.y_forward_mm_s,
        chassis_config.max_y_forward_mm_s,
        max_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.yaw_ccw_deg_s,
        chassis_config.max_yaw_deg_s,
        max_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.x_right_mm_s,
        chassis_config.max_x_acc_mm_s2,
        acc_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.y_forward_mm_s,
        chassis_config.max_y_acc_mm_s2,
        acc_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.yaw_ccw_deg_s,
        chassis_config.max_yaw_acc_deg_s2,
        acc_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.x_right_mm_s,
        chassis_config.max_x_dec_mm_s2,
        dec_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.y_forward_mm_s,
        chassis_config.max_y_dec_mm_s2,
        dec_rate);
    ChassisControl_LimitStraightProgress(
        unit_body.yaw_ccw_deg_s,
        chassis_config.max_yaw_dec_deg_s2,
        dec_rate);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        wheel_limit = chassis_config.motor_max_rpm[index];
        if ((chassis_runtime_limits_active != 0U) &&
            (chassis_runtime_max_rpm < wheel_limit))
        {
            wheel_limit = chassis_runtime_max_rpm;
        }
        /* 先扣除横向纠偏的轮速，再决定公共进度还能占用多少轮速。 */
        if (unit_wheels.rpm[index] > CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            candidate = (wheel_limit - reserved_wheel_rpm[index]) /
                        unit_wheels.rpm[index];
            if (candidate < *max_rate)
            {
                *max_rate = candidate;
            }
        }
        else if (unit_wheels.rpm[index] < -CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            candidate = (-wheel_limit - reserved_wheel_rpm[index]) /
                        unit_wheels.rpm[index];
            if (candidate < *max_rate)
            {
                *max_rate = candidate;
            }
        }
        ChassisControl_LimitStraightProgress(
            unit_wheels.rpm[index],
            chassis_runtime_acc_rpm_per_s[index] *
                acceleration_fraction,
            acc_rate);
        ChassisControl_LimitStraightProgress(
            unit_wheels.rpm[index],
            chassis_runtime_dec_rpm_per_s[index] *
                acceleration_fraction,
            dec_rate);
        if ((curve_active != 0U) &&
            (ChassisControl_Abs(derivative_wheels.rpm[index]) >
             CHASSIS_STRAIGHT_TURN_EPSILON))
        {
            /* 为转弯造成的轮速变化预留额度，避免最终四轮限速突然缩小整组。 */
            candidate = sqrtf(
                CHASSIS_STRAIGHT_TURN_CURVE_RESERVE *
                fminf(chassis_runtime_acc_rpm_per_s[index],
                      chassis_runtime_dec_rpm_per_s[index]) /
                ChassisControl_Abs(derivative_wheels.rpm[index]));
            if (candidate < *max_rate)
            {
                *max_rate = candidate;
            }
        }
    }

    /* 纠偏占满轮速时允许进度限速为零；这不是控制器故障。 */
    if (*max_rate < 0.0f)
    {
        *max_rate = 0.0f;
    }

    return ((*max_rate < FLT_MAX) &&
            (*acc_rate > 0.0f) && (*acc_rate < FLT_MAX) &&
            (*dec_rate > 0.0f) && (*dec_rate < FLT_MAX)) ? 1U : 0U;
}

/*
  函数说明: 求单个轮子本周期能到达的带符号 RPM 区间。换向时先减速到零，
  再把本周期剩余时间用于反向加速；不会额外停留一个控制周期。
  调用示例: ChassisControl_GetStraightWheelStepRange(index, dt, minimum, maximum);
  参数意义: index 为数组或电机下标，从 0 开始。
            dt 为控制周期，秒。
            minimum 为允许的下限。
            maximum 为允许的上限。
*/
static void ChassisControl_GetStraightWheelStepRange(
    uint8_t index, float dt, float *minimum, float *maximum)
{
    float current = chassis_current_wheel_rpm[index];
    float acceleration = chassis_runtime_acc_rpm_per_s[index];
    float deceleration = chassis_runtime_dec_rpm_per_s[index];
    float wheel_limit = chassis_config.motor_max_rpm[index];

    if ((chassis_runtime_limits_active != 0U) &&
        (chassis_runtime_max_rpm < wheel_limit))
    {
        wheel_limit = chassis_runtime_max_rpm;
    }

    if (current > 0.0f)
    {
        *maximum = current + acceleration * dt;
        *minimum = (current >= deceleration * dt) ?
            current - deceleration * dt :
            -acceleration * (dt - current / deceleration);
    }
    else if (current < 0.0f)
    {
        *minimum = current - acceleration * dt;
        *maximum = (-current >= deceleration * dt) ?
            current + deceleration * dt :
            acceleration * (dt + current / deceleration);
    }
    else
    {
        *minimum = -acceleration * dt;
        *maximum = acceleration * dt;
    }

    *minimum = ChassisControl_Clamp(*minimum, -wheel_limit, wheel_limit);
    *maximum = ChassisControl_Clamp(*maximum, -wheel_limit, wheel_limit);
}

/*
  函数说明: 把“基值 + 系数 * 进度速度”限制在指定区间，逐轴求公共可行速度范围。
  调用示例: ChassisControl_RestrictStraightRate(base, coefficient, minimum, maximum, minimum_rate, maximum_rate);
  参数意义: base 为轮速线性关系中的基值，RPM。
            coefficient 为进度速度到轮速的线性系数。
            minimum 为允许的下限。
            maximum 为允许的上限。
            minimum_rate 为输出允许的最小进度速度，1/s。
            maximum_rate 为输出允许的最大进度速度，1/s。
*/
static uint8_t ChassisControl_RestrictStraightRate(
    float base, float coefficient, float minimum, float maximum,
    float *minimum_rate, float *maximum_rate)
{
    float first;
    float second;
    float temporary;

    if (ChassisControl_Abs(coefficient) <=
        CHASSIS_STRAIGHT_TURN_EPSILON)
    {
        return ((base >= minimum) && (base <= maximum)) ? 1U : 0U;
    }

    first = (minimum - base) / coefficient;
    second = (maximum - base) / coefficient;
    if (first > second)
    {
        temporary = first;
        first = second;
        second = temporary;
    }
    if (first > *minimum_rate)
    {
        *minimum_rate = first;
    }
    if (second < *maximum_rate)
    {
        *maximum_rate = second;
    }
    return (*minimum_rate <= *maximum_rate) ? 1U : 0U;
}

/*
  函数说明: 对四轮使用同一个过渡比例，避免单独改某一轮目标而破坏合成运动。
  正常情况下，上层可行速度计算已满足轮速斜率，这里只做最后保护。
  调用示例: ChassisControl_RampStraightWheels(target_rpm, dt, setpoint);
  参数意义: target_rpm 为四轮目标转速，RPM。
            dt 为控制周期，秒。
            setpoint 为四轮带符号目标转速，RPM。
*/
static float ChassisControl_RampStraightWheels(
    const float target_rpm[CHASSIS_MOTOR_COUNT],
    float dt,
    ChassisWheelSetpoint_t *setpoint)
{
    float scale = 1.0f;
    float current;
    float difference;
    float minimum;
    float maximum;
    float candidate;
    uint8_t index;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        current = chassis_current_wheel_rpm[index];
        difference = target_rpm[index] - current;
        ChassisControl_GetStraightWheelStepRange(
            index, dt, &minimum, &maximum);
        if ((difference > 0.0f) && (target_rpm[index] > maximum))
        {
            candidate = (maximum - current) / difference;
        }
        else if ((difference < 0.0f) && (target_rpm[index] < minimum))
        {
            candidate = (minimum - current) / difference;
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
    if (scale < 1.0f) chassis_pid_output_limited = 1U;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_current_wheel_rpm[index] +=
            scale * (target_rpm[index] - chassis_current_wheel_rpm[index]);
        setpoint->signed_rpm[index] = chassis_current_wheel_rpm[index];
    }
    return scale;
}

/*
  函数说明: 同步直线转向闭环：目标点始终在起点到终点的固定直线上；
  yaw 目标使用同一个进度 s。实际转向或平移落后时减慢 s，
  横向误差单独纠正，最终同时检查 XY 和 yaw 是否稳定到位。
  调用示例: ChassisControl_UpdateStraightTurn(dt, raw_yaw_deg, measured_x_mm_s, measured_y_mm_s, measured_yaw_deg_s, setpoint);
  参数意义: dt 为控制周期，秒。
            raw_yaw_deg 为传感器原始航向角，度。
            measured_x_mm_s 为固定坐标系 X 实测速度，mm/s。
            measured_y_mm_s 为固定坐标系 Y 实测速度，mm/s。
            measured_yaw_deg_s 为实测航向角速度，deg/s。
            setpoint 为四轮带符号目标转速，RPM。
*/
static uint8_t ChassisControl_UpdateStraightTurn(
    float dt,
    float raw_yaw_deg,
    float measured_x_mm_s,
    float measured_y_mm_s,
    float measured_yaw_deg_s,
    ChassisWheelSetpoint_t *setpoint)
{
    ChassisBodyVelocity_t desired_body = {0};
    ChassisBodyVelocity_t base_body = {0};
    ChassisBodyVelocity_t rate_body = {0};
    ChassisBodyVelocity_t correction_body = {0};
    ChassisWheelVelocity_t correction_wheels;
    ChassisWheelVelocity_t motion_wheels;
    ChassisWheelVelocity_t base_wheels;
    ChassisWheelVelocity_t rate_wheels;
    ChassisWheelVelocity_t target_wheels;
    float actual_progress = 1.0f;
    float brake_progress = 0.0f;
    float measured_progress_rate = 0.0f;
    float slow_progress_rate = 0.0f;
    float axis_progress;
    float axis_rate;
    float max_rate;
    float acc_rate;
    float dec_rate;
    float safe_dec_rate;
    float target_rate;
    float lead_rate;
    float lead_gap;
    float cross_step;
    float brake_rate;
    float brake_remaining;
    float minimum_brake_remaining;
    float next_progress;
    float progress_limit = 0.0f;
    float requested_rate = 0.0f;
    float commanded_rate = 0.0f;
    float feasible_min_rate = 0.0f;
    float feasible_max_rate = 0.0f;
    float wheel_minimum;
    float wheel_maximum;
    float wheel_scale;
    float projection_numerator;
    float projection_denominator;
    float along_error;
    float cross_error;
    float along_speed = 0.0f;
    float along_command;
    float cross_speed;
    float final_yaw_error_deg;
    uint8_t rate_feasible = 1U;
    uint8_t wheel_commands_slow = 1U;
    uint8_t index;

    /* 连续角度在新 HW101 采样处累加，OPS9 暂停期间也持续记录。 */
    (void)raw_yaw_deg;
    final_yaw_error_deg = chassis_straight_start_yaw_deg +
                          chassis_straight_delta_yaw_deg -
                          chassis_straight_yaw_unwrapped_deg;
    chassis_pose_error.yaw_deg = final_yaw_error_deg;

    /* 法向误差与参考进度无关，可先为纠偏预留四轮转速。 */
    if (chassis_straight_path_length_mm > CHASSIS_STRAIGHT_TURN_EPSILON)
    {
        cross_error =
            (chassis_straight_start_x_mm - chassis_current_pose.x_mm) *
                -chassis_straight_path_unit_y +
            (chassis_straight_start_y_mm - chassis_current_pose.y_mm) *
                chassis_straight_path_unit_x;
        cross_speed =
            -measured_x_mm_s * chassis_straight_path_unit_y +
             measured_y_mm_s * chassis_straight_path_unit_x;
        cross_speed = ChassisControl_Pid(CHASSIS_PID_CROSS, cross_error,
            cross_speed, fminf(chassis_config.max_x_right_mm_s, chassis_config.max_y_forward_mm_s), dt);
        /* 传感器法向速度抖动不能直接让四轮目标骤变。跨零连续过渡。 */
        if (ChassisControl_Abs(cross_speed) >
            ChassisControl_Abs(chassis_cross_speed_mm_s))
        {
            cross_step = fminf(chassis_config.max_x_acc_mm_s2, chassis_config.max_y_acc_mm_s2) * dt;
        }
        else
        {
            cross_step = fminf(chassis_config.max_x_dec_mm_s2, chassis_config.max_y_dec_mm_s2) * dt;
        }
        if (fabsf(cross_speed - chassis_cross_speed_mm_s) > cross_step)
            chassis_pid_output_limited = 1U;
        chassis_cross_speed_mm_s += ChassisControl_Clamp(
            cross_speed - chassis_cross_speed_mm_s,
            -cross_step, cross_step);
        cross_speed = chassis_cross_speed_mm_s;
        correction_body.x_right_mm_s =
            -chassis_straight_path_unit_y * cross_speed;
        correction_body.y_forward_mm_s =
             chassis_straight_path_unit_x * cross_speed;
    }
    else
    {
        /* 纯旋转时把起点作为固定 XY，使用公共 PID 保持位置。 */
        correction_body.x_right_mm_s = ChassisControl_Pid(CHASSIS_PID_X,
            chassis_straight_start_x_mm - chassis_current_pose.x_mm,
            measured_x_mm_s, chassis_config.max_x_right_mm_s, dt);
        correction_body.y_forward_mm_s = ChassisControl_Pid(CHASSIS_PID_Y,
            chassis_straight_start_y_mm - chassis_current_pose.y_mm,
            measured_y_mm_s, chassis_config.max_y_forward_mm_s, dt);
    }
    ChassisControl_WorldVelocityToBody(
        &correction_body, chassis_current_pose.yaw_deg);
    correction_body.x_right_mm_s = ChassisControl_LimitOutput(
        correction_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    correction_body.y_forward_mm_s = ChassisControl_LimitOutput(
        correction_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    if (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &correction_body,
            &correction_wheels) == 0U)
    {
        return 0U;
    }
    ChassisControl_ClampWheelTarget(correction_wheels.rpm);

    if (chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN)
    {
        /* 较慢轴用于防止参考跑远，较快轴用于提前判断刹车时机。 */
        if (chassis_straight_path_length_mm >
            CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            axis_progress =
                ((chassis_current_pose.x_mm - chassis_straight_start_x_mm) *
                     chassis_straight_path_unit_x +
                 (chassis_current_pose.y_mm - chassis_straight_start_y_mm) *
                     chassis_straight_path_unit_y) /
                chassis_straight_path_length_mm;
            axis_progress = ChassisControl_Clamp(axis_progress, 0.0f, 1.0f);
            actual_progress = axis_progress;
            brake_progress = axis_progress;
            along_speed =
                measured_x_mm_s * chassis_straight_path_unit_x +
                measured_y_mm_s * chassis_straight_path_unit_y;
            axis_rate = along_speed / chassis_straight_path_length_mm;
            slow_progress_rate = axis_rate;
            if (axis_rate > measured_progress_rate)
            {
                measured_progress_rate = axis_rate;
            }
        }
        if (ChassisControl_Abs(chassis_straight_delta_yaw_deg) >
            CHASSIS_STRAIGHT_TURN_EPSILON)
        {
            axis_progress =
                (chassis_straight_yaw_unwrapped_deg -
                 chassis_straight_start_yaw_deg) /
                chassis_straight_delta_yaw_deg;
            axis_progress = ChassisControl_Clamp(axis_progress, 0.0f, 1.0f);
            axis_rate = measured_yaw_deg_s /
                        chassis_straight_delta_yaw_deg;
            if ((chassis_straight_path_length_mm <=
                 CHASSIS_STRAIGHT_TURN_EPSILON) ||
                (axis_progress < actual_progress) ||
                ((axis_progress == actual_progress) &&
                 (axis_rate < slow_progress_rate)))
            {
                slow_progress_rate = axis_rate;
            }
            if (axis_progress < actual_progress)
            {
                actual_progress = axis_progress;
            }
            if (axis_progress > brake_progress)
            {
                brake_progress = axis_progress;
            }
            if (axis_rate > measured_progress_rate)
            {
                measured_progress_rate = axis_rate;
            }
        }

        if ((chassis_straight_path_length_mm <=
             CHASSIS_STRAIGHT_TURN_EPSILON) &&
            (ChassisControl_Abs(chassis_straight_delta_yaw_deg) <=
             CHASSIS_STRAIGHT_TURN_EPSILON))
        {
            chassis_straight_progress = 1.0f;
        }
        else
        {
            if (ChassisControl_GetStraightProgressLimits(
                    correction_wheels.rpm,
                    &max_rate, &acc_rate, &dec_rate) == 0U)
            {
                return 0U;
            }

            /* 软件减速能力不是实测电机制动能力，留余量后再规划终点。 */
            safe_dec_rate = dec_rate *
                            CHASSIS_STRAIGHT_TURN_BRAKE_MARGIN;
            target_rate = sqrtf(2.0f * safe_dec_rate *
                                (1.0f - chassis_straight_progress));
            if (target_rate > max_rate)
            {
                target_rate = max_rate;
            }
            /* 只过滤参考与实测的进度差，避免传感器单帧跳动引起反复急减速。 */
            lead_gap = chassis_straight_progress - actual_progress;
            if (lead_gap < 0.0f)
            {
                lead_gap = 0.0f;
            }
            chassis_straight_filtered_lead +=
                dt / (CHASSIS_STRAIGHT_TURN_LEAD_FILTER_S + dt) *
                (lead_gap - chassis_straight_filtered_lead);
            /*
             * 较慢轴还在前进时，参考跟随它的实测进度速度，而不是在进度差
             * 达到阈值时把目标直接压为零；真正停住时才随误差逐步减速。
             */
            lead_rate = fmaxf(0.0f, slow_progress_rate) +
                CHASSIS_KP *
                (CHASSIS_STRAIGHT_TURN_PROGRESS_LEAD -
                 chassis_straight_filtered_lead);
            if (lead_rate < 0.0f)
            {
                lead_rate = 0.0f;
            }
            if (target_rate > lead_rate)
            {
                target_rate = lead_rate;
            }
            /*
             * 任一实测轴已领先时，按它的剩余路程及当前速度提前制动。
             * 保留一小段随参考终点缩小的进度空间，让落后轴继续追上，
             * 避免领先轴一到终点就把整个动作永久锁住。
             */
            brake_remaining = 1.0f - brake_progress -
                CHASSIS_STRAIGHT_TURN_BRAKE_LAG_S *
                    measured_progress_rate;
            minimum_brake_remaining =
                CHASSIS_STRAIGHT_TURN_BRAKE_FLOOR *
                (1.0f - chassis_straight_progress);
            if (brake_remaining < minimum_brake_remaining)
            {
                brake_remaining = minimum_brake_remaining;
            }
            brake_rate = sqrtf(2.0f * safe_dec_rate *
                               brake_remaining);
            if (target_rate > brake_rate)
            {
                target_rate = brake_rate;
            }
            requested_rate = ChassisControl_RateLimit(
                chassis_straight_progress_rate,
                target_rate,
                acc_rate,
                dec_rate,
                dt);
            progress_limit = fminf(max_rate,
                (1.0f - chassis_straight_progress) / dt);
            requested_rate = ChassisControl_Clamp(
                requested_rate, 0.0f, progress_limit);
        }
    }
    else
    {
        /* 首次到位后继续闭环保持终点，而不是停止位置控制。 */
        chassis_straight_progress = 1.0f;
    }

    /*
     * 下一周期的轮速可写成 W = W0 + Wr * s_dot。
     * 参考进度增量 s_dot*dt 已计入公共 P/D 的系数；I 只累计本次实测误差。
     * X、Y、yaw 三个输入都参与同一个可行进度速度的计算。
     */
    if (chassis_straight_path_length_mm > CHASSIS_STRAIGHT_TURN_EPSILON)
    {
        along_error = chassis_straight_progress *
                      chassis_straight_path_length_mm -
            (chassis_current_pose.x_mm - chassis_straight_start_x_mm) *
                chassis_straight_path_unit_x -
            (chassis_current_pose.y_mm - chassis_straight_start_y_mm) *
                chassis_straight_path_unit_y;
        along_speed =
            measured_x_mm_s * chassis_straight_path_unit_x +
            measured_y_mm_s * chassis_straight_path_unit_y;
        along_command = ChassisControl_Pid(CHASSIS_PID_ALONG, along_error,
            along_speed, fminf(chassis_config.max_x_right_mm_s, chassis_config.max_y_forward_mm_s), dt);
        base_body.x_right_mm_s = chassis_straight_path_unit_x * along_command;
        base_body.y_forward_mm_s = chassis_straight_path_unit_y * along_command;
        rate_body.x_right_mm_s = chassis_straight_path_unit_x *
            chassis_straight_path_length_mm *
            (1.0f + CHASSIS_KD +
             CHASSIS_KP * dt);
        rate_body.y_forward_mm_s = chassis_straight_path_unit_y *
            chassis_straight_path_length_mm *
            (1.0f + CHASSIS_KD +
             CHASSIS_KP * dt);
    }

    base_body.yaw_ccw_deg_s = ChassisControl_Pid(CHASSIS_PID_TURN,
        chassis_straight_start_yaw_deg +
        chassis_straight_progress * chassis_straight_delta_yaw_deg -
        chassis_straight_yaw_unwrapped_deg,
        measured_yaw_deg_s, chassis_config.max_yaw_deg_s, dt);
    rate_body.yaw_ccw_deg_s = chassis_straight_delta_yaw_deg *
        (1.0f + CHASSIS_KD +
         CHASSIS_KP * dt);

    /* 世界直线速度转换到当前车体，车身边转向也能沿原直线移动。 */
    ChassisControl_WorldVelocityToBody(
        &base_body, chassis_current_pose.yaw_deg);
    ChassisControl_WorldVelocityToBody(
        &rate_body, chassis_current_pose.yaw_deg);
    if ((ChassisMecanum_SolveVelocity(
             &chassis_config.mecanum,
             &base_body, &base_wheels) == 0U) ||
        (ChassisMecanum_SolveVelocity(
             &chassis_config.mecanum,
             &rate_body, &rate_wheels) == 0U))
    {
        return 0U;
    }

    /* 同时检查车体速度上限和四轮本周期的加减速/最大 RPM。 */
    feasible_max_rate = progress_limit;
    rate_feasible = ChassisControl_RestrictStraightRate(
        base_body.x_right_mm_s, rate_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s,
        &feasible_min_rate, &feasible_max_rate);
    if (rate_feasible != 0U)
    {
        rate_feasible = ChassisControl_RestrictStraightRate(
            base_body.y_forward_mm_s, rate_body.y_forward_mm_s,
            -chassis_config.max_y_forward_mm_s,
            chassis_config.max_y_forward_mm_s,
            &feasible_min_rate, &feasible_max_rate);
    }
    if (rate_feasible != 0U)
    {
        rate_feasible = ChassisControl_RestrictStraightRate(
            base_body.yaw_ccw_deg_s, rate_body.yaw_ccw_deg_s,
            -chassis_config.max_yaw_deg_s,
            chassis_config.max_yaw_deg_s,
            &feasible_min_rate, &feasible_max_rate);
    }
    for (index = 0U; (index < CHASSIS_MOTOR_COUNT) &&
                     (rate_feasible != 0U); ++index)
    {
        ChassisControl_GetStraightWheelStepRange(
            index, dt, &wheel_minimum, &wheel_maximum);
        rate_feasible = ChassisControl_RestrictStraightRate(
            base_wheels.rpm[index] + correction_wheels.rpm[index],
            rate_wheels.rpm[index],
            wheel_minimum, wheel_maximum,
            &feasible_min_rate, &feasible_max_rate);
    }

    if (rate_feasible != 0U)
    {
        commanded_rate = ChassisControl_Clamp(
            requested_rate, feasible_min_rate, feasible_max_rate);
    }
    else
    {
        /* 纠偏或传感器突变使公共区间暂时为空时，保留四轮同比例过渡。 */
        commanded_rate = requested_rate;
        ++chassis_straight_debug_no_feasible_rate_count;
    }

    desired_body.x_right_mm_s = base_body.x_right_mm_s +
                                rate_body.x_right_mm_s * commanded_rate;
    desired_body.y_forward_mm_s = base_body.y_forward_mm_s +
                                  rate_body.y_forward_mm_s * commanded_rate;
    desired_body.yaw_ccw_deg_s = base_body.yaw_ccw_deg_s +
                                 rate_body.yaw_ccw_deg_s * commanded_rate;
    desired_body.x_right_mm_s = ChassisControl_LimitOutput(
        desired_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    desired_body.y_forward_mm_s = ChassisControl_LimitOutput(
        desired_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    desired_body.yaw_ccw_deg_s = ChassisControl_LimitOutput(
        desired_body.yaw_ccw_deg_s,
        -chassis_config.max_yaw_deg_s,
        chassis_config.max_yaw_deg_s);

    if (ChassisMecanum_SolveVelocity(
            &chassis_config.mecanum,
            &desired_body,
            &motion_wheels) == 0U)
    {
        return 0U;
    }

    /* 先分配横向纠偏，再把平移与旋转作为一组按剩余轮速同比例加入。 */
    memcpy(target_wheels.rpm,
           correction_wheels.rpm,
           sizeof(target_wheels.rpm));
    ChassisControl_AddWheelComponent(
        target_wheels.rpm, motion_wheels.rpm);
    wheel_scale = ChassisControl_RampStraightWheels(
        target_wheels.rpm, dt, setpoint);

    if ((rate_feasible == 0U) || (wheel_scale < 0.999f))
    {
        /* 最终保护确实缩小轮速时，用实发四轮投影回公共进度，避免参考继续空跑。 */
        projection_numerator = 0.0f;
        projection_denominator = 0.0f;
        for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
        {
            projection_numerator +=
                (setpoint->signed_rpm[index] -
                 base_wheels.rpm[index] -
                 correction_wheels.rpm[index]) *
                rate_wheels.rpm[index];
            projection_denominator +=
                rate_wheels.rpm[index] * rate_wheels.rpm[index];
        }
        commanded_rate = (projection_denominator >
                          CHASSIS_STRAIGHT_TURN_EPSILON) ?
            ChassisControl_Clamp(
                projection_numerator / projection_denominator,
                0.0f, progress_limit) : 0.0f;
    }

    next_progress = chassis_straight_progress + commanded_rate * dt;
    chassis_straight_progress = ChassisControl_Clamp(
        next_progress, 0.0f, 1.0f);
    chassis_straight_progress_rate = commanded_rate;
    chassis_straight_debug_reference_progress = chassis_straight_progress;
    chassis_straight_debug_actual_progress = actual_progress;
    chassis_straight_debug_requested_rate = requested_rate;
    chassis_straight_debug_command_rate = commanded_rate;
    chassis_straight_debug_wheel_scale = wheel_scale;
    chassis_current_velocity = desired_body;
    chassis_current_velocity.x_right_mm_s += correction_body.x_right_mm_s;
    chassis_current_velocity.y_forward_mm_s += correction_body.y_forward_mm_s;

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (ChassisControl_Abs(chassis_current_wheel_rpm[index]) >
            CHASSIS_STRAIGHT_TURN_WHEEL_TOLERANCE_RPM)
        {
            wheel_commands_slow = 0U;
            break;
        }
    }

    /* 缓存被重复读取时只做控制，不累计到位次数。 */
    if (chassis_arrival_new_pair == 0U)
    {
        return 1U;
    }
    /* 经过目标点时还在运动不能报告完成；到位新鲜度独立于 3000 ms 故障门限。 */
    if ((chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN) &&
        (chassis_straight_progress >= 1.0f) &&
        ((chassis_pose_error.x_mm * chassis_pose_error.x_mm +
          chassis_pose_error.y_mm * chassis_pose_error.y_mm) <=
         (CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM *
          CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM)) &&
        (ChassisControl_Abs(final_yaw_error_deg) <=
         CHASSIS_SEQUENCE_YAW_TOLERANCE_DEG) &&
        (((measured_x_mm_s * measured_x_mm_s) +
          (measured_y_mm_s * measured_y_mm_s)) <=
         (CHASSIS_STRAIGHT_TURN_SPEED_TOLERANCE_MM_S *
          CHASSIS_STRAIGHT_TURN_SPEED_TOLERANCE_MM_S)) &&
        (ChassisControl_Abs(measured_yaw_deg_s) <=
         CHASSIS_STRAIGHT_TURN_YAW_RATE_TOLERANCE_DEG_S) &&
        (wheel_commands_slow != 0U) &&
        (chassis_straight_ops_velocity_valid != 0U) &&
        (chassis_straight_hw_velocity_valid != 0U) &&
        ((HAL_GetTick() - chassis_straight_ops_sample_ms) <=
          CHASSIS_SENSOR_FRESH_MS) &&
        ((HAL_GetTick() - chassis_straight_hw_sample_ms) <=
          CHASSIS_SENSOR_FRESH_MS))
    {
        ++chassis_straight_settle_cycles;
        if (chassis_straight_settle_cycles >=
            CHASSIS_STRAIGHT_TURN_SETTLE_CYCLES)
        {
            chassis_sequence_phase = CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE;
            chassis_sequence_done = 1U;
        }
    }
    else if (chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN)
    {
        chassis_straight_settle_cycles = 0U;
    }

    return 1U;
}

/*
  函数说明: 进入边平移边旋转阶段，把上一周期的轮速指令交给 XY 和转向控制。
  使用方法：新鲜定位进入过渡距离后调用一次；保持原 XY 终点，不等待停车。
  调用示例: valid = ChassisControl_BeginSequenceTurn();
  参数意义: 无。返回 1 表示衔接完成，0 表示运动学换算失败。
*/
static uint8_t ChassisControl_BeginSequenceTurn(void)
{
    ChassisBodyVelocity_t unit = {0};
    ChassisBodyVelocity_t velocity = {0};
    ChassisWheelVelocity_t unit_wheels;
    float command[3];
    float numerator;
    float denominator;
    float cross_integral = chassis_pid_integral[CHASSIS_PID_CROSS];
    float heading_integral = chassis_pid_integral[CHASSIS_PID_HEADING];
    uint8_t axis;
    uint8_t index;

    /* X/Y/yaw 的轮速基向量相互正交；投影最终轮速，避免使用尚未发出的目标速度。
     * 此处只恢复指令状态用于衔接，实测速度仍由 OPS9/HW101 提供。 */
    for (axis = 0U; axis < 3U; ++axis)
    {
        memset(&unit, 0, sizeof(unit));
        if (axis == 0U) unit.x_right_mm_s = 1.0f;
        else if (axis == 1U) unit.y_forward_mm_s = 1.0f;
        else unit.yaw_ccw_deg_s = 1.0f;
        if (!ChassisMecanum_SolveVelocity(&chassis_config.mecanum, &unit, &unit_wheels))
            return 0U;
        numerator = 0.0f;
        denominator = 0.0f;
        for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
        {
            numerator += chassis_current_wheel_rpm[index] * unit_wheels.rpm[index];
            denominator += unit_wheels.rpm[index] * unit_wheels.rpm[index];
        }
        if (denominator <= 0.0f) return 0U;
        command[axis] = numerator / denominator;
    }

    velocity.x_right_mm_s = command[0];
    velocity.y_forward_mm_s = command[1];
    /* 反向旋转把车体速度还原到固定坐标系，供 XY 控制继续使用。 */
    ChassisControl_WorldVelocityToBody(&velocity, -chassis_current_pose.yaw_deg);
    chassis_hold_x_speed_mm_s = velocity.x_right_mm_s;
    chassis_hold_y_speed_mm_s = velocity.y_forward_mm_s;
    chassis_turn_speed_deg_s = command[2];
    chassis_cross_speed_mm_s = 0.0f;
    chassis_heading_speed_deg_s = 0.0f;
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        /* 平移已交给 XY 环，清除旧分量，防止同一平移速度叠加两次。 */
        chassis_translate_wheel_rpm[index] = 0.0f;
        chassis_turn_wheel_rpm[index] = unit_wheels.rpm[index] * command[2];
    }

    /* 横向积分按原路径法向交给 XY，航向积分交给主动转向环。 */
    ChassisControl_ResetPid();
    chassis_pid_integral[CHASSIS_PID_X] = -chassis_path_unit_y * cross_integral;
    chassis_pid_integral[CHASSIS_PID_Y] = chassis_path_unit_x * cross_integral;
    chassis_pid_integral[CHASSIS_PID_TURN] = heading_integral;
    memcpy(chassis_pid_previous, chassis_pid_integral, sizeof(chassis_pid_previous));
    chassis_sequence_phase = CHASSIS_SEQUENCE_ROTATE;
    chassis_target_pose.yaw_deg = chassis_sequence_final_yaw_deg;
    chassis_pose_error.yaw_deg = ChassisControl_WrapAngle(
        chassis_target_pose.yaw_deg - chassis_current_pose.yaw_deg);
    chassis_rotate_settle_cycles = 0U;
    return 1U;
}

/*
  函数说明: 相对动作专用控制。行走时沿直线前进，同时独立修正横向漂移和航向。
  使用方法：Update() 已更新 OPS/HW101 位姿与动作阶段后调用；轮速分配顺序为航向、横向、进度。
  调用示例: ChassisControl_UpdateSequence(dt, measured_x_mm_s, measured_y_mm_s, measured_yaw_deg_s, setpoint);
  参数意义: dt 为控制周期，秒。
            measured_x_mm_s 为固定坐标系 X 实测速度，mm/s。
            measured_y_mm_s 为固定坐标系 Y 实测速度，mm/s。
            measured_yaw_deg_s 为实测航向角速度，deg/s。
            setpoint 为四轮带符号目标转速，RPM。
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
    float target_along_speed;
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
        cross_error =
            -(chassis_pose_error.x_mm * chassis_path_unit_y) +
             (chassis_pose_error.y_mm * chassis_path_unit_x);
        cross_speed =
            -(measured_x_mm_s * chassis_path_unit_y) +
             (measured_y_mm_s * chassis_path_unit_x);

        /* 沿路径目标速度由公共 P 生成，随后执行车体限速和本次动作的轮速加减速。 */
        target_along_speed = CHASSIS_KP * along_error;
        translation_body.x_right_mm_s =
            chassis_path_unit_x * target_along_speed;
        translation_body.y_forward_mm_s =
            chassis_path_unit_y * target_along_speed;

        target_cross_speed = ChassisControl_Pid(CHASSIS_PID_CROSS,
            cross_error, cross_speed,
            fminf(chassis_config.max_x_right_mm_s, chassis_config.max_y_forward_mm_s), dt);
    }
    else
    {
        /* 转弯和到位保持都以终点为目标，使用公共 PID 保持 XY。 */
        target_hold_x_speed = ChassisControl_Pid(CHASSIS_PID_X,
            chassis_pose_error.x_mm, measured_x_mm_s, chassis_config.max_x_right_mm_s, dt);
        target_hold_y_speed = ChassisControl_Pid(CHASSIS_PID_Y,
            chassis_pose_error.y_mm, measured_y_mm_s, chassis_config.max_y_forward_mm_s, dt);
    }

    if (chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE)
    {
        /* 主动转弯使用公共 PID 和电机侧加减速限制。 */
        target_turn_speed = ChassisControl_Pid(CHASSIS_PID_TURN,
            chassis_pose_error.yaw_deg, measured_yaw_deg_s, chassis_config.max_yaw_deg_s, dt);
    }
    else
    {
        /* 行走时锁定起始航向；首次到位后继续以同一套参数保持终点航向。 */
        target_heading_speed = ChassisControl_Pid(CHASSIS_PID_HEADING,
            chassis_pose_error.yaw_deg, measured_yaw_deg_s, chassis_config.max_yaw_deg_s, dt);
    }

    chassis_cross_speed_mm_s = ChassisControl_RateLimit(
        chassis_cross_speed_mm_s,
        target_cross_speed,
        fminf(chassis_config.max_x_acc_mm_s2, chassis_config.max_y_acc_mm_s2),
        fminf(chassis_config.max_x_dec_mm_s2, chassis_config.max_y_dec_mm_s2),
        dt);
    chassis_hold_x_speed_mm_s = ChassisControl_RateLimit(
        chassis_hold_x_speed_mm_s,
        target_hold_x_speed,
        fminf(chassis_config.max_x_acc_mm_s2, chassis_config.max_y_acc_mm_s2),
        fminf(chassis_config.max_x_dec_mm_s2, chassis_config.max_y_dec_mm_s2),
        dt);
    chassis_hold_y_speed_mm_s = ChassisControl_RateLimit(
        chassis_hold_y_speed_mm_s,
        target_hold_y_speed,
        fminf(chassis_config.max_x_acc_mm_s2, chassis_config.max_y_acc_mm_s2),
        fminf(chassis_config.max_x_dec_mm_s2, chassis_config.max_y_dec_mm_s2),
        dt);
    chassis_heading_speed_deg_s = ChassisControl_RateLimit(
        chassis_heading_speed_deg_s,
        target_heading_speed,
        chassis_config.max_yaw_acc_deg_s2,
        chassis_config.max_yaw_dec_deg_s2,
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
    translation_body.x_right_mm_s = ChassisControl_LimitOutput(
        translation_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    translation_body.y_forward_mm_s = ChassisControl_LimitOutput(
        translation_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    correction_body.x_right_mm_s = ChassisControl_LimitOutput(
        correction_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    correction_body.y_forward_mm_s = ChassisControl_LimitOutput(
        correction_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    heading_body.yaw_ccw_deg_s = ChassisControl_LimitOutput(
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
            ChassisControl_RateLimit(
                chassis_translate_wheel_rpm[index],
                translation_wheels.rpm[index],
                acceleration,
                deceleration,
                dt);
        chassis_turn_wheel_rpm[index] =
            ChassisControl_RateLimit(
                chassis_turn_wheel_rpm[index],
                turn_wheels.rpm[index],
                chassis_config.motor_acc_rpm_per_s[index],
                chassis_config.motor_dec_rpm_per_s[index],
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
            ChassisControl_RateLimit(
                chassis_current_wheel_rpm[index],
                setpoint->signed_rpm[index],
                chassis_config.motor_acc_rpm_per_s[index],
                chassis_config.motor_dec_rpm_per_s[index],
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


/*
  函数说明: 装载底盘几何尺寸、电机地址/方向及公共速度限制。
  使用方法：系统启动时先声明 ChassisControlConfig_t，再调用本函数，随后传给 ChassisControl_Init()。
  返回：无；config 为 NULL 时不执行任何操作。
  调用示例: ChassisControl_LoadDefaultConfig(config);
  参数意义: config 为对应模块的配置结构。
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

    /* 公共电机侧参数；最终输出及每个反馈环均受这些限制约束。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        config->motor_acc_rpm_per_s[index] = CHASSIS_ACC_MAX_RPM_S;
        config->motor_dec_rpm_per_s[index] = CHASSIS_DEC_MAX_RPM_S;
        config->motor_max_rpm[index] = CHASSIS_SPEED_MAX_RPM;
    }

    config->max_x_right_mm_s = 3000.0f;
    config->max_y_forward_mm_s = 3000.0f;
    config->max_yaw_deg_s = 600.0f;
    (void)ChassisControl_ApplyCommonLimits(config);
    config->sensor_timeout_ms = CHASSIS_DEFAULT_SENSOR_TIMEOUT_MS;
}

/*
  函数说明: 初始化底盘控制器及其运行状态。
  使用方法：加载默认或自定义配置后调用一次；初始化完成后保持 STOP，需再调用速度/位置接口。
  返回：无；config 为 NULL 时进入未初始化故障状态。
  调用示例: ChassisControl_Init(config);
  参数意义: config 为对应模块的配置结构。
*/
void ChassisControl_Init(const ChassisControlConfig_t *config)
{
    ChassisControl_ResetPid();
    stop_hold_valid = 0U;
    stop_stable_frames = 0U;

    if (config == NULL)
    {
        chassis_control_initialized = 0U;
        chassis_mode = CHASSIS_CONTROL_FAULT;
        chassis_fault_reason = CHASSIS_FAULT_INITIALIZATION;
        return;
    }

    chassis_config = *config;
    if (!ChassisControl_ApplyCommonLimits(&chassis_config))
    {
        ChassisControl_ResetPid();
        chassis_control_initialized = 0U;
        chassis_mode = CHASSIS_CONTROL_FAULT;
        chassis_fault_reason = CHASSIS_FAULT_INITIALIZATION;
        return;
    }
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
    memset(&chassis_sensor_ops9, 0, sizeof(chassis_sensor_ops9));
    memset(&chassis_sensor_hw101, 0, sizeof(chassis_sensor_hw101));
    memset(&chassis_measured_velocity, 0, sizeof(chassis_measured_velocity));
    chassis_ops9_sample_valid = 0U;
    chassis_hw101_sample_valid = 0U;
    chassis_ops9_new_sample = 0U;
    chassis_hw101_new_sample = 0U;
    chassis_recovery_active = 0U;
    chassis_pause_total_ms = 0U;
    chassis_recovery_count = 0U;
    chassis_last_sensor_gap_ms = 0U;
    chassis_fault_reason = CHASSIS_FAULT_NONE;
    chassis_last_fault_reason = CHASSIS_FAULT_NONE;
    chassis_last_fault_ms = 0U;
    chassis_settle_ops9_frame = 0U;
    chassis_settle_hw101_frame = 0U;
    chassis_arrival_new_pair = 0U;
    ++chassis_motion_id;
    chassis_control_initialized = 1U;
    ChassisControl_ResetRuntimeLimits();
}


/*
  函数说明: 设置车体速度目标，由 Update() 执行加减速限制和四轮解算。
  本模式未使用实测车速构成整车速度误差闭环。
  使用方法：周期任务或上层控制器调用；x/y 单位为 mm/s，yaw 单位为 deg/s，调用不阻塞。
  返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
  调用示例: ChassisControl_SetVelocity(x_right_mm_s, y_forward_mm_s, yaw_ccw_deg_s);
  参数意义: x_right_mm_s 为车体向右速度，mm/s。
            y_forward_mm_s 为车体向前速度，mm/s。
            yaw_ccw_deg_s 为逆时针角速度，deg/s。
*/
uint8_t ChassisControl_SetVelocity(float x_right_mm_s,
                                   float y_forward_mm_s,
                                   float yaw_ccw_deg_s)
{
    taskENTER_CRITICAL();
    /* 速度接口只改变目标速度，实际输出仍由 Update() 做加减速限制。 */
    if ((chassis_control_initialized == 0U) ||
        (ChassisControl_Finite(x_right_mm_s) == 0U) ||
        (ChassisControl_Finite(y_forward_mm_s) == 0U) ||
        (ChassisControl_Finite(yaw_ccw_deg_s) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT) ||
        (chassis_recovery_active != 0U))
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    chassis_target_velocity.x_right_mm_s = x_right_mm_s;
    chassis_target_velocity.y_forward_mm_s = y_forward_mm_s;
    chassis_target_velocity.yaw_ccw_deg_s = yaw_ccw_deg_s;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_VELOCITY;
    ++chassis_motion_id;
    taskEXIT_CRITICAL();
    return 1U;
}



/*
  函数说明: 设置相对上电原点的绝对车体目标位置和航向。
  使用方法：传入 x/y(mm)、yaw(deg) 后由 OPS9/HW101 位置环和航向环闭环逼近，调用不阻塞。
  返回：1 表示目标已接受，0 表示未初始化、参数无效或当前处于故障状态。
  调用示例: ChassisControl_SetPoseTarget(x_mm, y_mm, yaw_deg);
  参数意义: x_mm 为X 坐标或相对位移，mm，坐标系见函数说明。
            y_mm 为Y 坐标或相对位移，mm，坐标系见函数说明。
            yaw_deg 为航向或转角，度，参考系见函数说明。
*/
uint8_t ChassisControl_SetPoseTarget(float x_mm,
                                     float y_mm,
                                     float yaw_deg)
{
    taskENTER_CRITICAL();
    /* 绝对位置接口由 OPS9/HW101 闭环，不直接发送 FD 位置命令。 */
    if ((chassis_control_initialized == 0U) ||
        (ChassisControl_Finite(x_mm) == 0U) ||
        (ChassisControl_Finite(y_mm) == 0U) ||
        (ChassisControl_Finite(yaw_deg) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT) ||
        (chassis_recovery_active != 0U))
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    chassis_target_pose.x_mm = x_mm;
    chassis_target_pose.y_mm = y_mm;
    chassis_target_pose.yaw_deg = yaw_deg;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_POSE;
    ++chassis_motion_id;
    taskEXIT_CRITICAL();
    return 1U;
}



/*
  函数说明: 相对位移接口共用等待逻辑：传感器暂停期间不返回、不切下一条。
  运动代号变化表示主动取消或被新接口覆盖，旧函数不能复活旧目标。
  调用示例: ChassisControl_WaitForMotion(motion_id, straight);
  参数意义: motion_id 为需要等待的底盘动作代号。
            straight 为非零等待同步直线转向，零等待顺序动作。
*/
static uint8_t ChassisControl_WaitForMotion(uint32_t motion_id, uint8_t straight)
{
    ChassisControlState_t state;
    uint32_t start_ms = HAL_GetTick();
    uint32_t pause_base;
    uint32_t elapsed_ms;

    ChassisControl_GetState(&state);
    pause_base = state.paused_ms;
    for (;;)
    {
        ChassisControl_GetState(&state);
        if (state.motion_id != motion_id)
        {
            return 0U;
        }
        if (state.recovery_active != 0U)
        {
            osDelay(CHASSIS_SEQUENCE_WAIT_PERIOD_MS);
            continue;
        }
        if (state.mode != CHASSIS_CONTROL_POSE)
        {
            return 0U;
        }
        if ((state.sequence_done != 0U) &&
            (((straight == 0U) && (state.sequence_phase == CHASSIS_SEQUENCE_DONE)) ||
             ((straight != 0U) &&
              (state.sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE))))
        {
            return 1U;
        }
        elapsed_ms = HAL_GetTick() - start_ms;
        /* paused_ms 与 HAL 时钟均用无符号差值，允许毫秒计数正常回绕。 */
        if ((elapsed_ms >= state.paused_ms - pause_base) &&
            (elapsed_ms - (state.paused_ms - pause_base) >= CHASSIS_SEQUENCE_TIMEOUT_MS))
        {
            ChassisControl_Stop();
            return 0U;
        }
        osDelay(CHASSIS_SEQUENCE_WAIT_PERIOD_MS);
    }
}

typedef struct
{
    OPS9_Data_t ops9;
    HW101_Data_t hw101;
    ChassisPose_t pose;
    float cosine;
    float sine;
} ChassisRelativeStart_t;

/*
  函数说明: 相对运动共用的启动准备。只检查并取得起点，不设置执行阶段或最终目标。
  cosine/sine 对应发令瞬间的车体到 OPS 固定坐标变换，后续动作不能重取原点。
  调用示例: ChassisControl_PrepareRelativeMove(x_mm, y_mm, yaw_deg, acc_rpm_per_s, dec_rpm_per_s, max_rpm, start);
  参数意义: x_mm 为X 坐标或相对位移，mm，坐标系见函数说明。
            y_mm 为Y 坐标或相对位移，mm，坐标系见函数说明。
            yaw_deg 为航向或转角，度，参考系见函数说明。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
            start 为起点位置或起始运动状态。
*/
static uint8_t ChassisControl_PrepareRelativeMove(
    float x_mm, float y_mm, float yaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm,
    ChassisRelativeStart_t *start)
{
    float body_to_world_rad;

    if ((chassis_control_initialized == 0U) ||
        (__get_IPSR() != 0U) ||
        (osKernelGetState() != osKernelRunning) ||
        (ChassisControl_Finite(x_mm) == 0U) ||
        (ChassisControl_Finite(y_mm) == 0U) ||
        (ChassisControl_Finite(yaw_deg) == 0U) ||
        (chassis_mode == CHASSIS_CONTROL_FAULT) ||
        (chassis_recovery_active != 0U) ||
        (chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE) ||
        (chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE) ||
        (chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN))
    {
        return 0U;
    }

    (void)OPS9_GetLatest(&start->ops9);
    (void)HW101_GetData(&start->hw101);
    if (ChassisControl_SensorsFresh(&start->ops9, &start->hw101) == 0U)
    {
        return 0U;
    }
    if (chassis_origin_valid == 0U)
    {
        chassis_origin_x_mm = start->ops9.x_mm;
        chassis_origin_y_mm = start->ops9.y_mm;
        chassis_origin_yaw_deg = start->hw101.yaw_deg;
        chassis_origin_valid = 1U;
    }
    start->pose.x_mm = start->ops9.x_mm - chassis_origin_x_mm;
    start->pose.y_mm = start->ops9.y_mm - chassis_origin_y_mm;
    start->pose.yaw_deg = ChassisControl_WrapAngle(
        start->hw101.yaw_deg - chassis_origin_yaw_deg);

    if (ChassisControl_SetRuntimeLimits(
            acc_rpm_per_s, dec_rpm_per_s, max_rpm) == 0U)
    {
        return 0U;
    }
    body_to_world_rad = -CHASSIS_COORD_ROTATION_SIGN *
                        start->pose.yaw_deg * CHASSIS_PI / 180.0f;
    start->cosine = cosf(body_to_world_rad);
    start->sine = sinf(body_to_world_rad);
    return 1U;
}

/*
  函数说明: 在调用者临界区内提交相对动作换算后的固定坐标终点和最终航向。
  调用示例: ChassisControl_BeginSequential(&start.pose, target_x, target_y, final_yaw);
  参数意义: start 为发令瞬间的新鲜定位快照。
            x_mm、y_mm 为 OPS 固定坐标终点，mm。
            yaw_deg 为最终绝对航向，度，正值逆时针。
*/
static void ChassisControl_BeginSequential(
    const ChassisPose_t *start, float x_mm, float y_mm, float yaw_deg)
{
    float dx_mm = x_mm - start->x_mm;
    float dy_mm = y_mm - start->y_mm;
    float path_length_mm = sqrtf((dx_mm * dx_mm) + (dy_mm * dy_mm));
    uint8_t index;

    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    chassis_current_pose = *start;
    chassis_target_pose.x_mm = x_mm;
    chassis_target_pose.y_mm = y_mm;
    ChassisControl_ResetSequenceControl();
    /* 下一动作从当前轮速受控衔接，避免重置动作分量造成瞬间急停。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
        chassis_translate_wheel_rpm[index] = chassis_current_wheel_rpm[index];
    if (path_length_mm > 0.001f)
    {
        chassis_path_unit_x = dx_mm / path_length_mm;
        chassis_path_unit_y = dy_mm / path_length_mm;
    }
    /* 平移先保持起始航向；进入过渡距离后边平移边转向最终航向。 */
    chassis_target_pose.yaw_deg = start->yaw_deg;
    chassis_sequence_final_yaw_deg = ChassisControl_WrapAngle(yaw_deg);
    chassis_sequence_phase = CHASSIS_SEQUENCE_TRANSLATE;
    chassis_sequence_done = 0U;
    chassis_mode = CHASSIS_CONTROL_POSE;
    ++chassis_motion_id;
}

/*
  函数说明: 非阻塞启动相对动作，先平移，进入过渡距离后边平移边旋转；成功返回 1。
  调用示例: ChassisControl_StartRelativeSequential(100, 0, 0, 1000, 1500, 3000);
  参数意义: dx_mm 为发令瞬间车体向右的相对位移，mm。
            dy_mm 为发令瞬间车体向前的相对位移，mm。
            dyaw_deg 为相对转角，度，正值逆时针。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_StartRelativeSequential(
    float dx_mm, float dy_mm, float dyaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm)
{
    ChassisRelativeStart_t start;

    taskENTER_CRITICAL();
    if (ChassisControl_PrepareRelativeMove(
            dx_mm, dy_mm, dyaw_deg, acc_rpm_per_s,
            dec_rpm_per_s, max_rpm, &start) == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    ChassisControl_BeginSequential(&start.pose,
        start.pose.x_mm + start.cosine * dx_mm - start.sine * dy_mm,
        start.pose.y_mm + start.sine * dx_mm + start.cosine * dy_mm,
        start.pose.yaw_deg + dyaw_deg);
    taskEXIT_CRITICAL();
    return 1U;
}



/*
  函数说明: 执行相对动作，先平移，进入过渡距离后边平移边旋转；阻塞等待实际到位，失败返回 0。
  调用示例: ChassisControl_MoveRelativeSequential(100, 0, 90, 1000, 1500, 3000);
  参数意义: dx_mm 为发令瞬间车体向右的相对位移，mm。
            dy_mm 为发令瞬间车体向前的相对位移，mm。
            dyaw_deg 为相对转角，度，正值逆时针。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveRelativeSequential(
    float dx_mm, float dy_mm, float dyaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm)
{
    if (!ChassisControl_StartRelativeSequential(dx_mm, dy_mm, dyaw_deg,
            acc_rpm_per_s, dec_rpm_per_s, max_rpm)) return 0U;
    return ChassisControl_WaitForMotion(chassis_motion_id, 0U);
}

/*
  函数说明: 使用相对发令时车身向右偏 45 度的坐标系移动，阻塞等待实际到位。
  使用方法：从 RTOS 任务调用；先将斜向 X/Y 换算为车体位移，再复用原顺序动作。
            每次发令重新采用当前车身方向；本次终点确定后，不随运动中的转向重算。
            新 +Y 指向车右前方，新 +X 指向车右后方；本函数不额外转动车身。
  返回：1 表示实际到位，0 表示参数无效、未就绪、忙、故障、取消或超时。
  调用示例: ChassisControl_MoveDiagonalRelative(0, 100, 0, 100, 150, 800);
  参数意义: dx_mm 为沿新 X 轴的相对位移，mm，正值向车右后方。
            dy_mm 为沿新 Y 轴的相对位移，mm，正值向车右前方。
            dyaw_deg 为相对当前航向的转角，度，正值逆时针；0 保持当前航向。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            max_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveDiagonalRelative(
    float dx_mm, float dy_mm, float dyaw_deg,
    float acc_rpm_per_s, float dec_rpm_per_s, float max_rpm)
{
    const float diagonal_scale = 0.70710678118654752440f;
    /* 坐标轴顺时针转 45 度，长度保持不变；角度参数仍直接控制真实车身转角。 */
    float body_x_mm = diagonal_scale * dx_mm + diagonal_scale * dy_mm;
    float body_y_mm = diagonal_scale * dy_mm - diagonal_scale * dx_mm;

    return ChassisControl_MoveRelativeSequential(
        body_x_mm, body_y_mm, dyaw_deg,
        acc_rpm_per_s, dec_rpm_per_s, max_rpm);
}

/*
  函数说明: 在固定地面直线上平移，同时按相同进度旋转 yaw，并阻塞等待双轴到位。
  使用方法：从 main_task 等 RTOS 任务调用；x/y 是调用瞬间的车体相对位移，
         +X 为车右、+Y 为车前，单位 mm；yaw 为相对角度，正值逆时针。
         acc/dcc 单位 RPM/s，max_speed 单位 RPM，四轮共用这些限制。
         本函数不发送完成帧，也不读取串口；底盘任务仍每 20 ms 做闭环。
  返回：1 为 XY 和 yaw 连续到位；0 为参数、传感器、故障、中断或超时。
  调用示例: ChassisControl_MoveRelativeStraightTurn(100, 0, 90, 1000, 1500, 3000);
  参数意义: x_mm 为X 坐标或相对位移，mm，坐标系见函数说明。
            y_mm 为Y 坐标或相对位移，mm，坐标系见函数说明。
            yaw_deg 为航向或转角，度，参考系见函数说明。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dcc_rpm_per_s 为电机减速度，RPM/s。
            max_speed_rpm 为本次动作最大轮速，RPM。
*/
uint8_t ChassisControl_MoveRelativeStraightTurn(
    float x_mm,
    float y_mm,
    float yaw_deg,
    float acc_rpm_per_s,
    float dcc_rpm_per_s,
    float max_speed_rpm)
{
    ChassisRelativeStart_t start;
    uint32_t motion_id;

    taskENTER_CRITICAL();
    if (ChassisControl_PrepareRelativeMove(
            x_mm, y_mm, yaw_deg, acc_rpm_per_s,
            dcc_rpm_per_s, max_speed_rpm, &start) == 0U)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }

    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    chassis_straight_delta_x_mm = start.cosine * x_mm - start.sine * y_mm;
    chassis_straight_delta_y_mm = start.sine * x_mm + start.cosine * y_mm;
    chassis_straight_path_length_mm = sqrtf(
        chassis_straight_delta_x_mm * chassis_straight_delta_x_mm +
        chassis_straight_delta_y_mm * chassis_straight_delta_y_mm);
    chassis_straight_path_unit_x = 0.0f;
    chassis_straight_path_unit_y = 0.0f;
    if (chassis_straight_path_length_mm > CHASSIS_STRAIGHT_TURN_EPSILON)
    {
        chassis_straight_path_unit_x = chassis_straight_delta_x_mm /
                                       chassis_straight_path_length_mm;
        chassis_straight_path_unit_y = chassis_straight_delta_y_mm /
                                       chassis_straight_path_length_mm;
    }

    chassis_straight_start_x_mm = start.pose.x_mm;
    chassis_straight_start_y_mm = start.pose.y_mm;
    chassis_straight_start_yaw_deg = start.pose.yaw_deg;
    chassis_straight_delta_yaw_deg = yaw_deg;
    chassis_straight_yaw_unwrapped_deg = start.pose.yaw_deg;
    chassis_straight_previous_raw_yaw_deg = start.hw101.yaw_deg;
    chassis_straight_progress = 0.0f;
    chassis_straight_progress_rate = 0.0f;
    chassis_straight_settle_cycles = 0U;
    chassis_straight_debug_reference_progress = 0.0f;
    chassis_straight_debug_actual_progress = 0.0f;
    chassis_straight_debug_requested_rate = 0.0f;
    chassis_straight_debug_command_rate = 0.0f;
    chassis_straight_debug_wheel_scale = 1.0f;
    chassis_straight_debug_no_feasible_rate_count = 0U;
    memset(&chassis_straight_measured_velocity,
           0,
           sizeof(chassis_straight_measured_velocity));
    chassis_straight_previous_ops_x_mm = start.ops9.x_mm;
    chassis_straight_previous_ops_y_mm = start.ops9.y_mm;
    chassis_straight_previous_hw_yaw_deg = start.hw101.yaw_deg;
    chassis_straight_ops_frame_count = start.ops9.frame_count;
    chassis_straight_hw_frame_count = start.hw101.frame_count;
    chassis_straight_ops_sample_ms = start.ops9.last_update_ms;
    chassis_straight_hw_sample_ms = start.hw101.last_update_ms;
    chassis_straight_ops_velocity_valid = 0U;
    chassis_straight_hw_velocity_valid = 0U;

    chassis_current_pose.x_mm = start.pose.x_mm;
    chassis_current_pose.y_mm = start.pose.y_mm;
    chassis_current_pose.yaw_deg = start.pose.yaw_deg;
    chassis_target_pose.x_mm =
        start.pose.x_mm + chassis_straight_delta_x_mm;
    chassis_target_pose.y_mm =
        start.pose.y_mm + chassis_straight_delta_y_mm;
    chassis_target_pose.yaw_deg = ChassisControl_WrapAngle(
        start.pose.yaw_deg + yaw_deg);
    ChassisControl_ResetSequenceControl();
    chassis_sequence_done = 0U;
    chassis_sequence_phase = CHASSIS_SEQUENCE_STRAIGHT_TURN;
    chassis_mode = CHASSIS_CONTROL_POSE;
    motion_id = ++chassis_motion_id;
    taskEXIT_CRITICAL();
    return ChassisControl_WaitForMotion(motion_id, 1U);
}



/*
  函数说明: 查询最近一次相对动作是否首次到位，支持顺序和同步直线转向两种模式。
  使用方法：供其他任务或调试代码查询；返回 1 后仍持续保持目标位姿，直到停止、改目标或故障。
  调用示例: ChassisControl_IsMoveComplete();
  参数意义: 无。
*/
uint8_t ChassisControl_IsMoveComplete(void)
{
    return (((chassis_sequence_phase == CHASSIS_SEQUENCE_DONE) ||
             (chassis_sequence_phase ==
              CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE)) &&
            (chassis_sequence_done != 0U)) ? 1U : 0U;
}

/*
  函数说明: 取消当前速度/位置动作，并将目标速度设为零。
  使用方法：上层需要正常停车时调用；后续控制周期仍会按减速度把实际轮速降到零。
  调用示例: ChassisControl_Stop();
  参数意义: 无。
*/
void ChassisControl_Stop(void)
{
    uint8_t fatal_fault;

    if (chassis_control_initialized == 0U)
    {
        return;
    }

    taskENTER_CRITICAL();
    fatal_fault = ((chassis_mode == CHASSIS_CONTROL_FAULT) &&
                   (chassis_recovery_active == 0U)) ? 1U : 0U;

    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    ChassisControl_EndSensorHold(HAL_GetTick());
    ++chassis_motion_id;
    chassis_target_velocity.x_right_mm_s = 0.0f;
    chassis_target_velocity.y_forward_mm_s = 0.0f;
    chassis_target_velocity.yaw_ccw_deg_s = 0.0f;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    ChassisControl_ResetRuntimeLimits();
    if (fatal_fault == 0U)
    {
        chassis_fault_reason = CHASSIS_FAULT_NONE;
        chassis_sensor_fault = 0U;
        chassis_mode = CHASSIS_CONTROL_STOP;
    }
    taskEXIT_CRITICAL();
}


/*
  函数说明: 清除控制器记录的传感器故障标志并回到 STOP 状态。
  使用方法：确认外部故障已排除后调用；它会重新等待有效传感器建立原点。
  调用示例: ChassisControl_ClearFault();
  参数意义: 无。
*/
void ChassisControl_ClearFault(void)
{
    if (chassis_control_initialized == 0U)
    {
        return;
    }


    taskENTER_CRITICAL();
    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    ChassisControl_EndSensorHold(HAL_GetTick());
    ++chassis_motion_id;
    chassis_fault_reason = CHASSIS_FAULT_NONE;
    chassis_ops9_sample_valid = 0U;
    chassis_hw101_sample_valid = 0U;
    memset(&chassis_measured_velocity, 0, sizeof(chassis_measured_velocity));
    chassis_sensor_fault = 0U;
    chassis_origin_valid = 0U;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    memset(chassis_current_wheel_rpm,
           0,
           sizeof(chassis_current_wheel_rpm));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    ChassisControl_ResetRuntimeLimits();
    chassis_mode = CHASSIS_CONTROL_STOP;
    taskEXIT_CRITICAL();
}


/*
  函数说明: 锁存底盘故障并立即把控制目标清零。
  使用方法：检测到传感器、反馈或解算异常时调用；底盘任务随后应发送零速/停止命令。
  调用示例: ChassisControl_FaultStop();
  参数意义: 无。
*/
void ChassisControl_FaultStop(void)
{
    ChassisControl_FaultStopWithReason(CHASSIS_FAULT_CONTROL);
}

/*
  函数说明: 锁存具体故障原因、清空运动目标并取消自动续跑。
  调用示例: ChassisControl_FaultStopWithReason(reason);
  参数意义: reason 为底盘故障原因位图。
*/
void ChassisControl_FaultStopWithReason(ChassisFaultReason_t reason)
{
    if (chassis_control_initialized == 0U)
    {
        return;
    }


    taskENTER_CRITICAL();
    stop_hold_valid = 0U;
    stop_stable_frames = 0U;
    ChassisControl_EndSensorHold(HAL_GetTick());
    chassis_fault_reason = reason;
    chassis_last_fault_reason = reason;
    chassis_last_fault_ms = HAL_GetTick();
    chassis_sensor_fault = (((uint32_t)reason &
        (CHASSIS_FAULT_OPS9_TIMEOUT | CHASSIS_FAULT_HW101_TIMEOUT |
         CHASSIS_FAULT_POSITION_JUMP | CHASSIS_FAULT_YAW_UNCERTAIN |
         CHASSIS_FAULT_INVALID_SAMPLE)) != 0U) ? 1U : 0U;
    chassis_mode = CHASSIS_CONTROL_FAULT;
    chassis_sequence_phase = CHASSIS_SEQUENCE_IDLE;
    chassis_sequence_done = 0U;
    ChassisControl_ResetSequenceControl();
    memset(&chassis_target_velocity, 0, sizeof(chassis_target_velocity));
    memset(&chassis_current_velocity, 0, sizeof(chassis_current_velocity));
    memset(chassis_current_wheel_rpm,
           0,
           sizeof(chassis_current_wheel_rpm));
    taskEXIT_CRITICAL();
}


/*
  函数说明: 执行一次底盘控制周期，完成传感器采样、位置/航向 PID、速度加减速、麦克纳姆解算和轮速限幅。
  使用方法：由 chassisTask 以固定周期调用（当前为 20 ms），把输出 setpoint 交给电机命令构造函数。
  返回：1 表示本周期输出有效，0 表示参数/传感器/反馈/解算异常；setpoint 不能为 NULL。
  调用示例: ChassisControl_Update(dt_s, setpoint);
  参数意义: dt_s 为控制周期，秒。
            setpoint 为四轮带符号目标转速，RPM。
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
    uint32_t now_ms;
    ChassisFaultReason_t sample_fault;
    uint8_t index;

    if ((setpoint == NULL) || (chassis_control_initialized == 0U))
    {
        return 0U;
    }

    ChassisControl_ZeroSetpoint(setpoint);
    chassis_pid_output_limited = 0U;

    dt = ChassisControl_Clamp(dt_s,
                              CHASSIS_MIN_DT_S,
                              CHASSIS_MAX_DT_S);

    (void)OPS9_GetLatest(&ops9);
    (void)HW101_GetData(&hw101);
    now_ms = HAL_GetTick();
    sample_fault = ChassisControl_ObserveSensors(&ops9, &hw101);
    if ((sample_fault != CHASSIS_FAULT_NONE) &&
        ((chassis_mode != CHASSIS_CONTROL_FAULT) || (chassis_recovery_active != 0U)))
    {
        ChassisControl_FaultStopWithReason(sample_fault);
        return 0U;
    }

    if ((chassis_origin_valid == 0U) &&
        (ChassisControl_Ops9Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS) &&
        (ChassisControl_Hw101Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS))
    {
        chassis_origin_x_mm = chassis_sensor_ops9.x_mm;
        chassis_origin_y_mm = chassis_sensor_ops9.y_mm;
        chassis_origin_yaw_deg = chassis_sensor_hw101.yaw_deg;
        chassis_origin_valid = 1U;
    }
    if (chassis_origin_valid != 0U)
    {
        /* 两路独立更新：OPS9 掉线不会阻止记录 HW101 的停车转角。 */
        if (chassis_ops9_sample_valid != 0U)
        {
            chassis_current_pose.x_mm = chassis_sensor_ops9.x_mm - chassis_origin_x_mm;
            chassis_current_pose.y_mm = chassis_sensor_ops9.y_mm - chassis_origin_y_mm;
        }
        if (chassis_hw101_sample_valid != 0U)
        {
            chassis_current_pose.yaw_deg = ChassisControl_WrapAngle(
                chassis_sensor_hw101.yaw_deg - chassis_origin_yaw_deg);
        }
    }

    chassis_arrival_new_pair = 0U;
    if ((ChassisControl_Ops9Age(now_ms) > CHASSIS_SENSOR_FRESH_MS) ||
        (ChassisControl_Hw101Age(now_ms) > CHASSIS_SENSOR_FRESH_MS))
    {
        chassis_rotate_settle_cycles = 0U;
        chassis_straight_settle_cycles = 0U;
    }
    if ((ChassisControl_Ops9Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS) &&
        (ChassisControl_Hw101Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS) &&
        (chassis_sensor_ops9.frame_count != chassis_settle_ops9_frame) &&
        (chassis_sensor_hw101.frame_count != chassis_settle_hw101_frame))
    {
        chassis_arrival_new_pair = 1U;
        chassis_settle_ops9_frame = chassis_sensor_ops9.frame_count;
        chassis_settle_hw101_frame = chassis_sensor_hw101.frame_count;
    }
    if ((chassis_mode == CHASSIS_CONTROL_FAULT) && (chassis_recovery_active == 0U))
    {
        return 0U; /* CAN、数据跳变、解算错误等不得被新传感器帧自动解锁。 */
    }
    if (((chassis_mode != CHASSIS_CONTROL_STOP) || stop_hold_valid) && (chassis_recovery_active == 0U) &&
        ((ChassisControl_Ops9Age(now_ms) > CHASSIS_SENSOR_HOLD_MS) ||
         (ChassisControl_Hw101Age(now_ms) > CHASSIS_SENSOR_HOLD_MS)))
    {
        ChassisControl_BeginSensorHold(now_ms);
    }
    if (chassis_recovery_active != 0U)
    {
        if (ChassisControl_UpdateSensorHold(now_ms, dt, setpoint) == 0U)
        {
            return (chassis_mode == CHASSIS_CONTROL_FAULT) ? 0U : 1U;
        }
    }
    chassis_pid_sensor_fresh =
        (ChassisControl_Ops9Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS) &&
        (ChassisControl_Hw101Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS);
    measured_x_mm_s = chassis_measured_velocity.x_right_mm_s;
    measured_y_mm_s = chassis_measured_velocity.y_forward_mm_s;
    measured_yaw_deg_s = chassis_measured_velocity.yaw_ccw_deg_s;

    memset(&desired_body, 0, sizeof(desired_body));

    if (chassis_mode == CHASSIS_CONTROL_STOP && chassis_origin_valid &&
        ChassisControl_Ops9Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS &&
        ChassisControl_Hw101Age(now_ms) <= CHASSIS_SENSOR_FRESH_MS)
    {
        if (!stop_hold_valid)
        {
            if (fabsf(chassis_current_velocity.x_right_mm_s) < 1.0f &&
                fabsf(chassis_current_velocity.y_forward_mm_s) < 1.0f &&
                fabsf(chassis_current_velocity.yaw_ccw_deg_s) < 0.5f &&
                fabsf(measured_x_mm_s) < 20.0f && fabsf(measured_y_mm_s) < 20.0f &&
                fabsf(measured_yaw_deg_s) < 2.0f)
            {
                if (chassis_arrival_new_pair && ++stop_stable_frames >= 3U)
                {
                    chassis_target_pose = chassis_current_pose;
                    stop_hold_valid = 1U;
                }
            }
            else stop_stable_frames = 0U;
        }
        if (stop_hold_valid)
        {
            chassis_pose_error.x_mm = chassis_target_pose.x_mm - chassis_current_pose.x_mm;
            chassis_pose_error.y_mm = chassis_target_pose.y_mm - chassis_current_pose.y_mm;
            chassis_pose_error.yaw_deg = ChassisControl_WrapAngle(chassis_target_pose.yaw_deg - chassis_current_pose.yaw_deg);
            desired_body.x_right_mm_s = ChassisControl_Pid(CHASSIS_PID_X,
                chassis_pose_error.x_mm, measured_x_mm_s, chassis_config.max_x_right_mm_s, dt);
            desired_body.y_forward_mm_s = ChassisControl_Pid(CHASSIS_PID_Y,
                chassis_pose_error.y_mm, measured_y_mm_s, chassis_config.max_y_forward_mm_s, dt);
            desired_body.yaw_ccw_deg_s = ChassisControl_Pid(CHASSIS_PID_HEADING,
                chassis_pose_error.yaw_deg, measured_yaw_deg_s, chassis_config.max_yaw_deg_s, dt);
            ChassisControl_WorldVelocityToBody(&desired_body, chassis_current_pose.yaw_deg);
        }
    }

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

        if ((chassis_sequence_phase == CHASSIS_SEQUENCE_STRAIGHT_TURN) ||
            (chassis_sequence_phase ==
             CHASSIS_SEQUENCE_STRAIGHT_TURN_DONE))
        {
            /* 新模式直接生成同时平移与旋转的轮速，不进入旧模式的阶段切换。 */
            ChassisControl_UpdateStraightMeasuredVelocity(&ops9, &hw101);
            if (ChassisControl_UpdateStraightTurn(
                    dt,
                    hw101.yaw_deg,
                    chassis_straight_measured_velocity.x_right_mm_s,
                    chassis_straight_measured_velocity.y_forward_mm_s,
                    chassis_straight_measured_velocity.yaw_ccw_deg_s,
                    setpoint) == 0U)
            {
                ChassisControl_FaultStop();
                ChassisControl_ZeroSetpoint(setpoint);
                return 0U;
            }
            ChassisControl_FinishPidCycle();
            return 1U;
        }

        /* 新鲜定位进入过渡范围即开始转向，不要求先减速或停车。 */
        if ((chassis_sequence_phase == CHASSIS_SEQUENCE_TRANSLATE) &&
            (chassis_arrival_new_pair != 0U) &&
            (position_error_sq_mm2 <=
             (CHASSIS_SEQUENCE_TRANSLATE_TOLERANCE_MM *
              CHASSIS_SEQUENCE_TRANSLATE_TOLERANCE_MM)))
        {
            if (ChassisControl_BeginSequenceTurn() == 0U)
            {
                ChassisControl_FaultStop();
                ChassisControl_ZeroSetpoint(setpoint);
                return 0U;
            }
        }

        if ((chassis_sequence_phase == CHASSIS_SEQUENCE_ROTATE) &&
            (chassis_arrival_new_pair != 0U))
        {
            uint8_t rotation_wheels_slow = 1U;
            for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
            {
                if (ChassisControl_Abs(chassis_current_wheel_rpm[index]) >
                    CHASSIS_STRAIGHT_TURN_WHEEL_TOLERANCE_RPM)
                {
                    rotation_wheels_slow = 0U;
                }
            }
            /* 最终完成仍使用严格的位置、角度和低速条件，不使用平移过渡阈值。 */
            if ((position_error_sq_mm2 <=
                 (CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM * CHASSIS_SEQUENCE_POSITION_TOLERANCE_MM)) &&
                (ChassisControl_Abs(chassis_pose_error.yaw_deg) <= CHASSIS_SEQUENCE_YAW_TOLERANCE_DEG) &&
                (ChassisControl_Abs(measured_x_mm_s) <= CHASSIS_SEQUENCE_TRANSLATE_SPEED_TOLERANCE_MM_S) &&
                (ChassisControl_Abs(measured_y_mm_s) <= CHASSIS_SEQUENCE_TRANSLATE_SPEED_TOLERANCE_MM_S) &&
                (ChassisControl_Abs(measured_yaw_deg_s) <= CHASSIS_STRAIGHT_TURN_YAW_RATE_TOLERANCE_DEG_S) &&
                (rotation_wheels_slow != 0U))
            {
                ++chassis_rotate_settle_cycles;
                if (chassis_rotate_settle_cycles >= CHASSIS_SEQUENCE_TRANSLATE_SETTLE_CYCLES)
                {
                    /* 航向目标未变，把转向积分平滑交给到位保持环。XY 积分继续保留。 */
                    chassis_pid_integral[CHASSIS_PID_HEADING] = chassis_pid_integral[CHASSIS_PID_TURN];
                    chassis_pid_previous[CHASSIS_PID_HEADING] = chassis_pid_integral[CHASSIS_PID_TURN];
                    chassis_pid_integral[CHASSIS_PID_TURN] = 0.0f;
                    chassis_pid_previous[CHASSIS_PID_TURN] = 0.0f;
                    chassis_sequence_phase = CHASSIS_SEQUENCE_DONE;
                    chassis_sequence_done = 1U;
                }
            }
            else
            {
                chassis_rotate_settle_cycles = 0U;
            }
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
            ChassisControl_FinishPidCycle();
            return 1U;
        }

        /* 非相对动作的普通绝对位置接口使用公共 PID 的 XY/航向闭环。 */
        desired_body.x_right_mm_s = ChassisControl_Pid(CHASSIS_PID_X,
            chassis_pose_error.x_mm, measured_x_mm_s, chassis_config.max_x_right_mm_s, dt);
        desired_body.y_forward_mm_s = ChassisControl_Pid(CHASSIS_PID_Y,
            chassis_pose_error.y_mm, measured_y_mm_s, chassis_config.max_y_forward_mm_s, dt);
        desired_body.yaw_ccw_deg_s = ChassisControl_Pid(CHASSIS_PID_HEADING,
            chassis_pose_error.yaw_deg, measured_yaw_deg_s, chassis_config.max_yaw_deg_s, dt);
    }

    if (chassis_mode == CHASSIS_CONTROL_POSE)
    {
        /* 位置误差来自 OPS 固定坐标，麦克纳姆解算需要车体坐标。 */
        ChassisControl_WorldVelocityToBody(
            &desired_body, chassis_current_pose.yaw_deg);
    }

    desired_body.x_right_mm_s = ChassisControl_LimitOutput(
        desired_body.x_right_mm_s,
        -chassis_config.max_x_right_mm_s,
        chassis_config.max_x_right_mm_s);
    desired_body.y_forward_mm_s = ChassisControl_LimitOutput(
        desired_body.y_forward_mm_s,
        -chassis_config.max_y_forward_mm_s,
        chassis_config.max_y_forward_mm_s);
    desired_body.yaw_ccw_deg_s = ChassisControl_LimitOutput(
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
            ChassisControl_RateLimit(
                chassis_current_wheel_rpm[index],
                target_wheels.rpm[index],
                acceleration,
                deceleration,
                dt);
        setpoint->signed_rpm[index] = chassis_current_wheel_rpm[index];
    }

    ChassisControl_FinishPidCycle();
    return (chassis_mode == CHASSIS_CONTROL_FAULT) ? 0U : 1U;
}



/*
  函数说明: 把四轮带符号逻辑 RPM 转换为每个 X 电机的地址、CW/CCW 方向、速度和 F6 加速度字段。
  使用方法：先调用 ChassisControl_Update() 得到 setpoint，再调用本函数；本函数只组包，不发送 CAN。
  调用示例: ChassisControl_BuildMotorCommands(setpoint, commands);
  参数意义: setpoint 为四轮带符号目标转速，RPM。
            commands 为多电机命令数组。
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

/*
  函数说明: 复制当前控制器状态、目标/实测位姿、误差、故障和动作阶段。
  使用方法：调试 Watch 或上层任务传入有效 state 指针读取；函数在临界区内复制，返回后可安全查看。
  调用示例: ChassisControl_GetState(&state);
  参数意义: state 为状态值或状态快照。
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
    state->fault_reason = chassis_fault_reason;
    state->last_fault_reason = chassis_last_fault_reason;
    state->last_fault_ms = chassis_last_fault_ms;
    state->ops9_age_ms = ChassisControl_Ops9Age(HAL_GetTick());
    state->hw101_age_ms = ChassisControl_Hw101Age(HAL_GetTick());
    state->last_sensor_gap_ms = chassis_last_sensor_gap_ms;
    state->recovery_count = chassis_recovery_count;
    state->recovery_active = chassis_recovery_active;
    state->paused_ms = ChassisControl_PauseClock(HAL_GetTick());
    state->motion_id = chassis_motion_id;
    state->remaining_pose.x_mm = chassis_target_pose.x_mm - chassis_current_pose.x_mm;
    state->remaining_pose.y_mm = chassis_target_pose.y_mm - chassis_current_pose.y_mm;
    if (ChassisControl_IsStraightSequence() != 0U)
    {
        state->remaining_pose.yaw_deg = chassis_straight_start_yaw_deg +
            chassis_straight_delta_yaw_deg - chassis_straight_yaw_unwrapped_deg;
    }
    else
    {
        state->remaining_pose.yaw_deg = ChassisControl_WrapAngle(
            ((chassis_sequence_phase != CHASSIS_SEQUENCE_IDLE) ?
                chassis_sequence_final_yaw_deg : chassis_target_pose.yaw_deg) -
            chassis_current_pose.yaw_deg);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
}
