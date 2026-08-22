#include "arm_motion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "X_V2.h"
#include "cmsis_os.h"

#define ARM_MOTION_MOTOR_COUNT     (3U)
#define ARM_MOTION_SYNC_ADDRESS    (0U)
#define ARM_MOTION_ZERO_EPS_DEG    (1.0e-3f)
#define ARM_MOTION_DEG_PER_RPM_S   (6.0)
#define ARM_MOTION_MIN_SPEED_RPM   (0.1)
#define ARM_MOTION_SPEED_STEP_RPM  (0.1)
#define ARM_MOTION_TIME_EPS_S     (1.0e-3)
#define ARM_MOTION_TIME_TOLERANCE_S (0.01)
#define ARM_MOTION_ACCEL_CORRECTION_STEPS (64U)
#define ARM_MOTION_COMPLETION_MARGIN_MS (50U)
#define ARM_MOTION_MAX_OS_DELAY_TICKS (0xFFFFFFFEUL)

/* 根据当前机构强度先采用的三轴安全上限，不是驱动器手册的最大值。 */
static const uint16_t arm_motion_default_acc[ARM_MOTION_MOTOR_COUNT] =
    {600U, 300U, 400U};
static const uint16_t arm_motion_default_dec[ARM_MOTION_MOTOR_COUNT] =
    {700U, 400U, 500U};
static const float arm_motion_default_speed[ARM_MOTION_MOTOR_COUNT] =
    {2500.0f, 1800.0f, 2500.0f};

static ArmMotionConfig *arm_motion_active_config;
static ArmMotionResult arm_motion_last_result;
static ArmIkStatus arm_motion_last_ik_status = ARM_IK_OK;
static int32_t arm_motion_current_pulse[ARM_MOTION_MOTOR_COUNT];
static bool arm_motion_current_valid;

static int ArmMotion_ConfigIsValid(const ArmMotionConfig *config)
{
    uint8_t index;

    if ((config == NULL) ||
        (config->motor_map.pulses_per_revolution == 0U) ||
        (config->installation_direction < ARM_INSTALLATION_DIRECTION_1) ||
        (config->installation_direction > ARM_INSTALLATION_DIRECTION_4) ||
        ((config->ik_branch != ARM_IK_RADIAL_POSITIVE) &&
         (config->ik_branch != ARM_IK_RADIAL_NEGATIVE)))
    {
        return 0;
    }

    /* 本机构把 Gamma 机械下限作为地址3的零位，两份分层配置必须一致。 */
    if ((config->kinematics.gamma_limit.min_enabled == 0U) ||
        !isfinite(config->motor_map.gamma_zero_deg) ||
        (fabsf(config->motor_map.gamma_zero_deg -
               config->kinematics.gamma_limit.min_deg) >
         ARM_MOTION_ZERO_EPS_DEG))
    {
        return 0;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        uint8_t previous;

        if (config->motor_addr[index] == 0U)
        {
            return 0;
        }
        for (previous = 0U; previous < index; ++previous)
        {
            if (config->motor_addr[previous] == config->motor_addr[index])
            {
                return 0;
            }
        }

        if (!isfinite(config->vel_rpm[index]) ||
            (config->vel_rpm[index] < 0.0f) ||
            (config->vel_rpm[index] > 3000.0f) ||
            (config->acc_rpm_per_s[index] == 0U) ||
            (config->dec_rpm_per_s[index] == 0U))
        {
            return 0;
        }
    }

    return 1;
}

static int64_t ArmMotion_PulseToProtocolTenths(
    int32_t pulse,
    uint32_t pulses_per_revolution)
{
    double tenths = (double)pulse * 3600.0 /
                    (double)pulses_per_revolution;

    if (tenths >= 0.0)
    {
        return (int64_t)floor(tenths + 0.5);
    }
    return (int64_t)ceil(tenths - 0.5);
}

static double ArmMotion_ProtocolDistanceDegrees(
    int32_t target_pulse,
    int32_t current_pulse,
    uint32_t pulses_per_revolution)
{
    int64_t target_tenths = ArmMotion_PulseToProtocolTenths(
        target_pulse, pulses_per_revolution);
    int64_t current_tenths = ArmMotion_PulseToProtocolTenths(
        current_pulse, pulses_per_revolution);
    int64_t delta_tenths = target_tenths - current_tenths;

    if (delta_tenths < 0)
    {
        delta_tenths = -delta_tenths;
    }
    return (double)delta_tenths * 0.1;
}

static double ArmMotion_ProfileTimeSeconds(double distance_deg,
                                           double speed_rpm,
                                           uint16_t acc_rpm_per_s,
                                           uint16_t dec_rpm_per_s)
{
    double speed_deg_s;
    double acc_deg_s2;
    double dec_deg_s2;
    double acc_distance;
    double dec_distance;
    double cruise_distance;

    if (distance_deg <= 0.0)
    {
        return 0.0;
    }

    speed_deg_s = speed_rpm * ARM_MOTION_DEG_PER_RPM_S;
    acc_deg_s2 = (double)acc_rpm_per_s * ARM_MOTION_DEG_PER_RPM_S;
    dec_deg_s2 = (double)dec_rpm_per_s * ARM_MOTION_DEG_PER_RPM_S;
    if ((speed_deg_s <= 0.0) || (acc_deg_s2 <= 0.0) ||
        (dec_deg_s2 <= 0.0))
    {
        return INFINITY;
    }

    acc_distance = (speed_deg_s * speed_deg_s) /
                   (2.0 * acc_deg_s2);
    dec_distance = (speed_deg_s * speed_deg_s) /
                   (2.0 * dec_deg_s2);

    if (distance_deg <= (acc_distance + dec_distance))
    {
        /* 小位移时驱动器会自动使用三角形速度曲线。 */
        double peak_speed_deg_s = sqrt(
            (2.0 * distance_deg) /
            (1.0 / acc_deg_s2 + 1.0 / dec_deg_s2));
        return peak_speed_deg_s / acc_deg_s2 +
               peak_speed_deg_s / dec_deg_s2;
    }

    cruise_distance = distance_deg - acc_distance - dec_distance;
    return speed_deg_s / acc_deg_s2 +
           speed_deg_s / dec_deg_s2 +
           cruise_distance / speed_deg_s;
}

static ArmMotionStatus ArmMotion_CalculateTimedProfile(
    const ArmMotionConfig *config,
    uint8_t index,
    double distance_deg,
    float time_s,
    uint16_t *acc_rpm_per_s,
    uint16_t *dec_rpm_per_s,
    float *speed_rpm,
    float *predicted_time_s)
{
    double time_limit_s;
    double acc_cap_deg_s2;
    double dec_cap_deg_s2;
    double speed_cap_deg_s;
    double ramp_distance_deg;
    double peak_speed_deg_s;
    double minimum_time_s;
    double candidate_acc_deg_s2;
    double candidate_dec_deg_s2;
    double fastest_candidate_time_s;
    double coefficient;
    double discriminant;
    double ideal_speed_rpm;
    double candidate_speed_rpm;
    uint16_t best_acc;
    uint16_t best_dec;
    double best_rpm;
    double best_error;
    double candidate_time_s;
    double speed_candidates[2];
    uint16_t candidate_acc;
    uint16_t candidate_dec;
    uint16_t effort_low;
    uint16_t effort_high;
    uint16_t effort_mid;
    uint16_t effort_limit;
    uint32_t effort_value;
    uint32_t correction_step;
    uint8_t speed_index;
    double error_s;

    if ((config == NULL) || (acc_rpm_per_s == NULL) ||
        (dec_rpm_per_s == NULL) || (speed_rpm == NULL) ||
        (predicted_time_s == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        return ARM_MOTION_TIME_INVALID;
    }
    if (distance_deg <= 0.0)
    {
        *acc_rpm_per_s = config->acc_rpm_per_s[index];
        *dec_rpm_per_s = config->dec_rpm_per_s[index];
        *speed_rpm = 0.0f;
        *predicted_time_s = 0.0f;
        return ARM_MOTION_OK;
    }

    time_limit_s = (double)time_s;
    acc_cap_deg_s2 = (double)config->acc_rpm_per_s[index] *
                     ARM_MOTION_DEG_PER_RPM_S;
    dec_cap_deg_s2 = (double)config->dec_rpm_per_s[index] *
                     ARM_MOTION_DEG_PER_RPM_S;
    speed_cap_deg_s = (double)config->vel_rpm[index] *
                      ARM_MOTION_DEG_PER_RPM_S;
    if ((acc_cap_deg_s2 <= 0.0) || (dec_cap_deg_s2 <= 0.0) ||
        (speed_cap_deg_s <
         (ARM_MOTION_MIN_SPEED_RPM * ARM_MOTION_DEG_PER_RPM_S)))
    {
        return ARM_MOTION_TIME_UNACHIEVABLE;
    }

    /* 先使用三个上限计算该电机的最快梯形/三角形轨迹。 */
    ramp_distance_deg =
        (speed_cap_deg_s * speed_cap_deg_s) /
        (2.0 * acc_cap_deg_s2) +
        (speed_cap_deg_s * speed_cap_deg_s) /
        (2.0 * dec_cap_deg_s2);
    if (distance_deg <= ramp_distance_deg)
    {
        peak_speed_deg_s = sqrt(
            (2.0 * distance_deg) /
            (1.0 / acc_cap_deg_s2 + 1.0 / dec_cap_deg_s2));
        minimum_time_s = peak_speed_deg_s / acc_cap_deg_s2 +
                         peak_speed_deg_s / dec_cap_deg_s2;
    }
    else
    {
        peak_speed_deg_s = speed_cap_deg_s;
        minimum_time_s = speed_cap_deg_s / acc_cap_deg_s2 +
                         speed_cap_deg_s / dec_cap_deg_s2 +
                         (distance_deg - ramp_distance_deg) /
                         speed_cap_deg_s;
    }

    if (time_limit_s < (minimum_time_s - ARM_MOTION_TIME_EPS_S))
    {
        return ARM_MOTION_TIME_UNACHIEVABLE;
    }

    /*
     * 把 max(A,D) 定义为加速度等级。对某个等级，A/D 都取不超过
     * 各自上限的相同最小值。最快完成时间随等级单调下降，因此先
     * 用二分法找到满足指定时间的最低等级，再只沿这个一维等级做
     * 少量协议量化修正；不再进行 A/D 二维组合搜索。
     */
    effort_low = 1U;
    effort_limit = (config->acc_rpm_per_s[index] >
                    config->dec_rpm_per_s[index]) ?
                   config->acc_rpm_per_s[index] :
                   config->dec_rpm_per_s[index];
    effort_high = effort_limit;
    while (effort_low < effort_high)
    {
        effort_mid = (uint16_t)(
            (uint32_t)effort_low +
            (((uint32_t)effort_high - (uint32_t)effort_low) / 2U));
        candidate_acc = (effort_mid < config->acc_rpm_per_s[index]) ?
                        effort_mid : config->acc_rpm_per_s[index];
        candidate_dec = (effort_mid < config->dec_rpm_per_s[index]) ?
                        effort_mid : config->dec_rpm_per_s[index];
        fastest_candidate_time_s = ArmMotion_ProfileTimeSeconds(
            distance_deg,
            (double)config->vel_rpm[index],
            candidate_acc,
            candidate_dec);
        if (fastest_candidate_time_s <=
            (time_limit_s + ARM_MOTION_TIME_EPS_S))
        {
            effort_high = effort_mid;
        }
        else
        {
            effort_low = (uint16_t)(effort_mid + 1U);
        }
    }

    for (correction_step = 0U;
         correction_step <= ARM_MOTION_ACCEL_CORRECTION_STEPS;
         ++correction_step)
    {
        effort_value = (uint32_t)effort_low + correction_step;
        if (effort_value > (uint32_t)effort_limit)
        {
            break;
        }

        candidate_acc = (effort_value <
                         (uint32_t)config->acc_rpm_per_s[index]) ?
                        (uint16_t)effort_value :
                        config->acc_rpm_per_s[index];
        candidate_dec = (effort_value <
                         (uint32_t)config->dec_rpm_per_s[index]) ?
                        (uint16_t)effort_value :
                        config->dec_rpm_per_s[index];
        candidate_acc_deg_s2 = (double)candidate_acc *
                               ARM_MOTION_DEG_PER_RPM_S;
        candidate_dec_deg_s2 = (double)candidate_dec *
                               ARM_MOTION_DEG_PER_RPM_S;
        coefficient = 0.5 *
            (1.0 / candidate_acc_deg_s2 +
             1.0 / candidate_dec_deg_s2);
        discriminant = (time_limit_s * time_limit_s) -
                       (4.0 * coefficient * distance_deg);
        if (discriminant < 0.0)
        {
            continue;
        }

        ideal_speed_rpm =
            ((time_limit_s - sqrt(discriminant)) /
             (2.0 * coefficient)) /
            ARM_MOTION_DEG_PER_RPM_S;
        speed_candidates[0] = floor(
            ideal_speed_rpm / ARM_MOTION_SPEED_STEP_RPM) *
            ARM_MOTION_SPEED_STEP_RPM;
        speed_candidates[1] = ceil(
            ideal_speed_rpm / ARM_MOTION_SPEED_STEP_RPM) *
            ARM_MOTION_SPEED_STEP_RPM;

        best_acc = 0U;
        best_dec = 0U;
        best_rpm = 0.0;
        best_error = INFINITY;
        for (speed_index = 0U; speed_index < 2U; ++speed_index)
        {
            candidate_speed_rpm = speed_candidates[speed_index];
            if (candidate_speed_rpm < ARM_MOTION_MIN_SPEED_RPM)
            {
                candidate_speed_rpm = ARM_MOTION_MIN_SPEED_RPM;
            }
            if (candidate_speed_rpm >
                (double)config->vel_rpm[index])
            {
                continue;
            }

            candidate_time_s = ArmMotion_ProfileTimeSeconds(
                distance_deg,
                candidate_speed_rpm,
                candidate_acc,
                candidate_dec);
            if (candidate_time_s >
                (time_limit_s + ARM_MOTION_TIME_EPS_S))
            {
                continue;
            }

            error_s = fabs(time_limit_s - candidate_time_s);
            if (error_s < best_error)
            {
                best_error = error_s;
                best_acc = candidate_acc;
                best_dec = candidate_dec;
                best_rpm = candidate_speed_rpm;
                *predicted_time_s = (float)candidate_time_s;
            }
        }

        /* 先满足时间误差的等级就是最小 max(A,D)，立即结束。 */
        if ((best_acc != 0U) && (best_dec != 0U) &&
            (best_rpm > 0.0) &&
            (best_error <= ARM_MOTION_TIME_TOLERANCE_S))
        {
            *acc_rpm_per_s = best_acc;
            *dec_rpm_per_s = best_dec;
            *speed_rpm = (float)best_rpm;
            return ARM_MOTION_OK;
        }
    }

    return ARM_MOTION_TIME_UNACHIEVABLE;
}

static ArmMotionStatus ArmMotion_SendTargets(
    const ArmMotionConfig *config,
    ArmMotorTargets *targets,
    float time_s,
    ArmMotionProfile *profile)
{
    uint8_t index;
    int64_t pulse_delta;
    double distance_deg;
    float target_degrees[ARM_MOTION_MOTOR_COUNT];
    X_V2_Traj_Pos_Command_t position_commands[ARM_MOTION_MOTOR_COUNT];
    HAL_StatusTypeDef can_status;
    ArmMotionStatus motion_status;

    if ((config == NULL) || (targets == NULL) || (profile == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }

    profile->requested_time_s = time_s;

    if (time_s > 0.0f)
    {
        /* 只有 00 AA 多电机命令才能保证三轴同时启动。 */
        if (!config->use_multi_command)
        {
            return ARM_MOTION_INVALID_CONFIG;
        }
        if ((arm_motion_active_config != config) ||
            !arm_motion_current_valid)
        {
            return ARM_MOTION_POSITION_UNKNOWN;
        }
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        pulse_delta = 0;
        profile->acc_rpm_per_s[index] = config->acc_rpm_per_s[index];
        profile->dec_rpm_per_s[index] = config->dec_rpm_per_s[index];
        profile->vel_rpm[index] = config->vel_rpm[index];
        profile->predicted_time_s[index] = 0.0f;
        if (arm_motion_current_valid && (arm_motion_active_config == config))
        {
            pulse_delta = (int64_t)targets->pulse[index] -
                          (int64_t)arm_motion_current_pulse[index];
            distance_deg = ArmMotion_ProtocolDistanceDegrees(
                targets->pulse[index],
                arm_motion_current_pulse[index],
                config->motor_map.pulses_per_revolution);
            /*
             * X 绝对位置命令的方向字段表示目标坐标的正负，
             * 必须保留电机映射结果；pulse_delta 只用于距离和定时。
             */
        }
        else
        {
            distance_deg = 0.0;
        }

        if (time_s > 0.0f)
        {
            motion_status = ArmMotion_CalculateTimedProfile(
                config,
                index,
                distance_deg,
                time_s,
                &profile->acc_rpm_per_s[index],
                &profile->dec_rpm_per_s[index],
                &profile->vel_rpm[index],
                &profile->predicted_time_s[index]);
            if (motion_status != ARM_MOTION_OK)
            {
                return motion_status;
            }
        }
        else if (pulse_delta != 0)
        {
            profile->predicted_time_s[index] =
                (float)ArmMotion_ProfileTimeSeconds(
                    distance_deg,
                    profile->vel_rpm[index],
                    profile->acc_rpm_per_s[index],
                    profile->dec_rpm_per_s[index]);
        }
    }

    if (config->use_multi_command)
    {
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            target_degrees[index] = (float)(fabs((double)
                ArmMotion_PulseToProtocolTenths(
                    targets->pulse[index],
                    config->motor_map.pulses_per_revolution)) * 0.1);

            position_commands[index].addr = config->motor_addr[index];
            position_commands[index].dir =
                (X_V2_Direction_t)targets->direction[index];
            position_commands[index].acc_rpm_per_s =
                profile->acc_rpm_per_s[index];
            position_commands[index].dec_rpm_per_s =
                profile->dec_rpm_per_s[index];
            position_commands[index].vel_rpm = profile->vel_rpm[index];
            position_commands[index].pos_deg = target_degrees[index];
            position_commands[index].mode = X_V2_POS_ABSOLUTE;
            position_commands[index].snF = config->report_reached[index];
        }

        can_status = X_V2_MultiMotor_Traj_Pos_Control(
            position_commands, ARM_MOTION_MOTOR_COUNT);
        if (can_status != HAL_OK)
        {
            arm_motion_current_valid = false;
            return ARM_MOTION_CAN_ERROR;
        }

        if (arm_motion_active_config == config)
        {
            for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
            {
                arm_motion_current_pulse[index] = targets->pulse[index];
            }
            arm_motion_current_valid = true;
        }
        return ARM_MOTION_OK;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        target_degrees[index] = (float)(fabs((double)
            ArmMotion_PulseToProtocolTenths(
                targets->pulse[index],
                config->motor_map.pulses_per_revolution)) * 0.1);
        can_status = X_V2_Traj_Pos_Control(
            config->motor_addr[index],
            (X_V2_Direction_t)targets->direction[index],
            profile->acc_rpm_per_s[index],
            profile->dec_rpm_per_s[index],
            profile->vel_rpm[index],
            target_degrees[index],
            X_V2_POS_ABSOLUTE,
            config->use_sync);
        if (can_status != HAL_OK)
        {
            arm_motion_current_valid = false;
            return ARM_MOTION_CAN_ERROR;
        }
    }

    if (config->use_sync &&
        (X_V2_Synchronous_motion(ARM_MOTION_SYNC_ADDRESS) != HAL_OK))
    {
        arm_motion_current_valid = false;
        return ARM_MOTION_CAN_ERROR;
    }

    if (arm_motion_active_config == config)
    {
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            arm_motion_current_pulse[index] = targets->pulse[index];
        }
        arm_motion_current_valid = true;
    }
    return ARM_MOTION_OK;
}

void ArmMotion_LoadDefaultConfig(ArmMotionConfig *config)
{
    uint8_t index;

    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    ArmIk_LoadDefaultConfig(&config->kinematics);
    ArmMotorMap_LoadDefaultConfig(&config->motor_map);
    config->installation_direction = ARM_INSTALLATION_DIRECTION_1;
    config->ik_branch = ARM_IK_RADIAL_POSITIVE;

    config->motor_addr[0] = 1U;
    config->motor_addr[1] = 2U;
    config->motor_addr[2] = 3U;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        config->acc_rpm_per_s[index] = arm_motion_default_acc[index];
        config->dec_rpm_per_s[index] = arm_motion_default_dec[index];
        config->vel_rpm[index] = arm_motion_default_speed[index];
        config->report_reached[index] = false;
    }

    config->use_multi_command = true;
    config->use_sync = true;
    config->zero_confirmed = false;

    if (arm_motion_active_config == config)
    {
        arm_motion_current_valid = false;
    }
}

ArmMotionStatus ArmMotion_SetActiveConfig(ArmMotionConfig *config)
{
    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    arm_motion_active_config = config;
    arm_motion_current_valid = false;
    arm_motion_last_ik_status = ARM_IK_OK;
    memset(&arm_motion_last_result, 0, sizeof(arm_motion_last_result));
    return ARM_MOTION_OK;
}

void ArmMotion_ConfirmZero(ArmMotionConfig *config, bool confirmed)
{
    if (config != NULL)
    {
        config->zero_confirmed = confirmed;

        if (!confirmed || (arm_motion_active_config != config))
        {
            arm_motion_current_valid = false;
            return;
        }

        {
            ArmJointAngles zero_joints;
            ArmMotorTargets zero_targets;

            zero_joints.alpha_deg = 0.0f;
            zero_joints.beta_deg = 0.0f;
            zero_joints.gamma_deg = config->motor_map.gamma_zero_deg;
            if (ArmMotorMap_JointsToTargets(
                    &config->motor_map,
                    config->installation_direction,
                    &zero_joints,
                    &zero_targets) != ARM_MOTOR_MAP_OK)
            {
                arm_motion_current_valid = false;
                return;
            }

            arm_motion_current_pulse[0] = zero_targets.pulse[0];
            arm_motion_current_pulse[1] = zero_targets.pulse[1];
            arm_motion_current_pulse[2] = zero_targets.pulse[2];
            arm_motion_current_valid = true;
        }
    }
}

ArmMotionStatus ArmMotion_SetIkBranch(ArmMotionConfig *config,
                                      ArmIkRadialBranch branch)
{
    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if ((branch != ARM_IK_RADIAL_POSITIVE) &&
        (branch != ARM_IK_RADIAL_NEGATIVE))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    config->ik_branch = branch;
    return ARM_MOTION_OK;
}

ArmMotionStatus ArmMotion_EnableAll(const ArmMotionConfig *config,
                                    bool state)
{
    uint8_t index;
    uint8_t offset;
    uint8_t enable_commands[ARM_MOTION_MOTOR_COUNT * 6U];

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    if (config->use_multi_command)
    {
        offset = 0U;
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            enable_commands[offset++] = config->motor_addr[index];
            enable_commands[offset++] = 0xF3U;
            enable_commands[offset++] = 0xABU;
            enable_commands[offset++] = (uint8_t)state;
            enable_commands[offset++] = 0U;
            enable_commands[offset++] = 0x6BU;
        }

        return (X_V2_MultiMotor_Command(enable_commands, offset) == HAL_OK) ?
               ARM_MOTION_OK : ARM_MOTION_CAN_ERROR;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_En_Control(config->motor_addr[index], state, false) != HAL_OK)
        {
            return ARM_MOTION_CAN_ERROR;
        }
    }
    return ARM_MOTION_OK;
}

ArmMotionStatus ArmMotion_StopAll(const ArmMotionConfig *config)
{
    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    if (X_V2_Stop_Now(ARM_MOTION_SYNC_ADDRESS, false) != HAL_OK)
    {
        arm_motion_current_valid = false;
        return ARM_MOTION_CAN_ERROR;
    }

    arm_motion_current_valid = false;
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_MoveToJointsInternal(
    const ArmMotionConfig *config,
    const ArmJointAngles *joints,
    ArmMotionResult *result,
    ArmIkStatus *ik_status,
    float time_s)
{
    ArmIkStatus joint_status;
    ArmMotorMapStatus map_status;

    if (ik_status != NULL)
    {
        *ik_status = ARM_IK_OK;
    }
    if ((config == NULL) || (joints == NULL) || (result == NULL))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_NULL_POINTER;
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_INVALID_CONFIG;
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        return ARM_MOTION_NOT_ZEROED;
    }

    joint_status = ArmIk_ValidateJoints(&config->kinematics, joints);
    if (ik_status != NULL)
    {
        *ik_status = joint_status;
    }
    if (joint_status != ARM_IK_OK)
    {
        return ARM_MOTION_IK_ERROR;
    }

    result->joints = *joints;
    joint_status = ArmIk_Forward(&config->kinematics,
                                 joints,
                                 &result->position);
    if (joint_status != ARM_IK_OK)
    {
        if (ik_status != NULL) *ik_status = joint_status;
        return ARM_MOTION_IK_ERROR;
    }

    map_status = ArmMotorMap_JointsToTargets(
        &config->motor_map,
        config->installation_direction,
        joints,
        &result->motors);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        return ARM_MOTION_MAPPING_ERROR;
    }

    return ArmMotion_SendTargets(
        config, &result->motors, time_s, &result->profile);
}

ArmMotionStatus ArmMotion_MoveToJoints(const ArmMotionConfig *config,
                                       const ArmJointAngles *joints,
                                       ArmMotionResult *result,
                                       ArmIkStatus *ik_status)
{
    return ArmMotion_MoveToJointsInternal(
        config, joints, result, ik_status, 0.0f);
}

ArmMotionStatus ArmMotion_MoveToXYZ(float x_mm,
                                    float y_mm,
                                    float z_mm,
                                    float time_s)
{
    double wait_ticks_double;
    uint32_t wait_ticks;
    ArmJointAngles joints;
    ArmIkStatus solve_status;
    ArmMotionStatus motion_status;
    ArmMotionConfig *config = arm_motion_active_config;
    ArmMotionResult *result = &arm_motion_last_result;
    ArmIkStatus *ik_status = &arm_motion_last_ik_status;

    *ik_status = ARM_IK_OK;
    memset(result, 0, sizeof(*result));

    if (config == NULL)
    {
        *ik_status = ARM_IK_NULL_POINTER;
        return ARM_MOTION_NOT_INITIALIZED;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        return ARM_MOTION_TIME_INVALID;
    }
    /* 本工程FreeRTOS节拍为1 kHz，向上取整避免比设定时间提前返回。 */
    wait_ticks_double = ceil((double)time_s * 1000.0) +
                        (double)ARM_MOTION_COMPLETION_MARGIN_MS;
    if (wait_ticks_double > (double)ARM_MOTION_MAX_OS_DELAY_TICKS)
    {
        return ARM_MOTION_TIME_INVALID;
    }
    wait_ticks = (uint32_t)wait_ticks_double;
    if (!ArmMotion_ConfigIsValid(config))
    {
        *ik_status = ARM_IK_INVALID_CONFIG;
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        return ARM_MOTION_NOT_ZEROED;
    }

    /* 逆解或关节限位失败后立即返回；下方发送函数不会被调用。 */
    solve_status = ArmIk_SolveBranch(&config->kinematics,
                                     x_mm,
                                     y_mm,
                                     z_mm,
                                     config->ik_branch,
                                     &joints);
    *ik_status = solve_status;
    if (solve_status != ARM_IK_OK)
    {
        return ARM_MOTION_IK_ERROR;
    }

    motion_status = ArmMotion_MoveToJointsInternal(
        config, &joints, result, ik_status, time_s);
    if (motion_status == ARM_MOTION_OK)
    {
        /* 仅成功下发后等待；解算、限位或CAN失败时立即返回。 */
        osDelay(wait_ticks);
    }
    return motion_status;
}

const ArmMotionResult *ArmMotion_GetLastResult(void)
{
    return &arm_motion_last_result;
}

ArmMotionStatus ArmMotion_MoveJointOffsets(const ArmMotionConfig *config,
                                            float base_deg,
                                            float large_arm_deg,
                                            float small_arm_deg,
                                            ArmMotionResult *result,
                                            ArmIkStatus *ik_status)
{
    ArmJointAngles joints;
    ArmMotorMapStatus map_status;

    if (ik_status != NULL)
    {
        *ik_status = ARM_IK_OK;
    }
    if ((config == NULL) || (result == NULL))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_NULL_POINTER;
        return ARM_MOTION_NULL_POINTER;
    }

    map_status = ArmMotorMap_OffsetsToJoints(&config->motor_map,
                                             base_deg,
                                             large_arm_deg,
                                             small_arm_deg,
                                             &joints);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        return ARM_MOTION_MAPPING_ERROR;
    }

    return ArmMotion_MoveToJoints(config, &joints, result, ik_status);
}

ArmMotionStatus ArmMotion_SetTimedReturnInfo(const ArmMotionConfig *config,
                                             uint8_t info_function,
                                             uint16_t period_ms)
{
    uint8_t index;

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_Timed_Return_Info(config->motor_addr[index],
                                   info_function,
                                   period_ms) != HAL_OK)
        {
            return ARM_MOTION_CAN_ERROR;
        }
    }
    return ARM_MOTION_OK;
}

const char *ArmMotion_StatusString(ArmMotionStatus status)
{
    switch (status)
    {
        case ARM_MOTION_OK: return "OK";
        case ARM_MOTION_NULL_POINTER: return "null pointer";
        case ARM_MOTION_INVALID_CONFIG: return "invalid configuration";
        case ARM_MOTION_NOT_ZEROED: return "mechanical zero not confirmed";
        case ARM_MOTION_IK_ERROR: return "kinematics or joint limit error";
        case ARM_MOTION_MAPPING_ERROR: return "motor mapping error";
        case ARM_MOTION_CAN_ERROR: return "CAN command error";
        case ARM_MOTION_NOT_INITIALIZED: return "active configuration not set";
        case ARM_MOTION_TIME_INVALID: return "invalid motion time";
        case ARM_MOTION_TIME_UNACHIEVABLE:
            return "motion time cannot be achieved";
        case ARM_MOTION_POSITION_UNKNOWN: return "motor position unknown";
        default: return "unknown status";
    }
}
