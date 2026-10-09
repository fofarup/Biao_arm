#include "arm_motion.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "X_V2.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#define ARM_MOTION_MOTOR_COUNT     (3U)
#define ARM_MOTION_SYNC_ADDRESS    (0U)
#define ARM_MOTION_ZERO_EPS_DEG    (1.0e-3f)
#define ARM_MOTION_DEG_PER_RPM_S   (6.0f)
#define ARM_MOTION_MIN_SPEED_RPM   (0.1f)
#define ARM_MOTION_SPEED_STEP_RPM  (0.1)
#define ARM_MOTION_TIME_EPS_S     (1.0e-3)
#define ARM_MOTION_TIME_TOLERANCE_S (0.01)
#define ARM_MOTION_ACCEL_CORRECTION_STEPS (64U)
#define ARM_MOTION_REACHED_TIMEOUT_MARGIN_MS (1000U)
#define ARM_MOTION_TIMEOUT_RECOVERY_MS (1500U)
#define ARM_MOTION_TIMEOUT_RECOVERY_POLL_MS (100U)
#define ARM_MOTION_TIMEOUT_RECOVERY_STABLE_MS (50U)
#define ARM_MOTION_DIAG_STATUS_PERIOD_MS (100U)
#define ARM_MOTION_DIAG_POSITION_PERIOD_MS (200U)
#define ARM_MOTION_DIAG_ERROR_PERIOD_MS (200U)
#define ARM_MOTION_RX_DRAIN_LIMIT (16U)
#define ARM_MOTION_DRIVER_FAULT_MASK (0x0CU)
#define ARM_MOTION_MAX_OS_DELAY_TICKS (0xFFFFFFFEUL)
#define ARM_MOTION_ALL_MOTORS_MASK (0x07U)
#define ARM_MOTION_TRAJECTORY_DEFAULT_PERIOD_MS (20U)
#define ARM_MOTION_TRAJECTORY_DEFAULT_FEEDBACK_TIMEOUT_MS (100U)
#define ARM_MOTION_TRAJECTORY_DEFAULT_DEADLINE_TOLERANCE_MS (5U)
#define ARM_MOTION_TRAJECTORY_DEFAULT_HEARTBEAT_MS (200U)
#define ARM_MOTION_TRAJECTORY_DEFAULT_LIMIT_SCALE (0.85f)
#define ARM_MOTION_TRAJECTORY_DEFAULT_SETTLE_TIME_S (0.30f)
#define ARM_MOTION_TRAJECTORY_DEFAULT_MIN_RADIAL_MM (20.0f)
#define ARM_MOTION_TRAJECTORY_DEFAULT_KP (0.5f)
#define ARM_MOTION_TRAJECTORY_DEFAULT_CORRECTION_RPM (10.0f)
#define ARM_MOTION_TRAJECTORY_DEFAULT_ERROR_DEG (30.0f)
#define ARM_MOTION_TRAJECTORY_START_MATCH_DEG (1.0f)
#define ARM_MOTION_TRAJECTORY_MAX_SEGMENTS (10000UL)
#define ARM_MOTION_TRAJECTORY_PRECHECK_INTERVALS (200U)
#define ARM_MOTION_TRAJECTORY_DIRECTION_EPS_DEG_PER_U (0.1f)
#define ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM (1.0e-4f)
#define ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM (0.5f)
#define ARM_MOTION_TRAJECTORY_POSITION_PERIOD_MS (20U)
#define ARM_MOTION_TRAJECTORY_SPEED_PERIOD_MS (40U)
#define ARM_MOTION_TRAJECTORY_STATUS_PERIOD_MS (100U)
#define ARM_MOTION_TRAJECTORY_ERROR_PERIOD_MS (100U)
#define ARM_MOTION_TIMED_STOP_ACKS_PER_MOTOR (4U)
#define ARM_MOTION_TIMED_STOP_ACK_TIMEOUT_MS (100U)
#define ARM_MOTION_TRAJECTORY_SETTLE_MARGIN_MS (1000U)
#define ARM_MOTION_MINIMUM_VERIFY_ITERATIONS (3U)
#define ARM_MOTION_MINIMUM_VERIFY_MARGIN (1.02)

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

typedef struct
{
    bool active;
    uint8_t pending_mask;
    uint8_t reached_mask;
    uint8_t error_mask;
    uint8_t last_error_code;
    int32_t target_pulse[ARM_MOTION_MOTOR_COUNT];
} ArmMotionMoveTracker;

typedef struct
{
    ArmCubicBezierTrajectory trajectory;
    ArmMotionTrajectoryResult result;
    ArmJointAngles final_joints;
    ArmMotorCoordinates final_coordinates;
    ArmMotorCoordinates reference_coordinates;
    float previous_command_rpm[ARM_MOTION_MOTOR_COUNT];
    int8_t planned_direction[ARM_MOTION_MOTOR_COUNT];
    uint32_t start_ms;
    uint32_t feedback_request_ms;
    uint32_t next_update_ms;
    uint32_t feedback_deadline_ms;
    uint32_t settle_deadline_ms;
    uint32_t stop_command_ms;
    uint32_t stop_deadline_ms;
    uint32_t stop_position_barrier_ms;
    uint32_t heartbeat_request_ms;
    uint32_t rx_drop_baseline;
    uint32_t can_error_baseline;
    uint32_t generation;
    uint32_t segment_index;
    uint8_t driver_error_mask;
    uint8_t heartbeat_ack_mask;
    uint8_t timed_stop_ack_count[ARM_MOTION_MOTOR_COUNT];
    uint32_t timed_stop_request_ms;
    bool feedback_stream_enabled;
    bool timed_stop_waiting;
    bool trajectory_preflight_ready;
    bool heartbeat_enabled;
    bool heartbeat_disable_pending;
    bool stop_speed_confirmed;
} ArmMotionTrajectoryContext;

static ArmMotionMoveTracker arm_motion_move_tracker;
static ArmMotionDriverStatus arm_motion_driver_status[
    ARM_MOTION_MOTOR_COUNT];
static uint32_t arm_motion_diag_next_status_ms;
static uint32_t arm_motion_diag_next_position_ms;
static uint32_t arm_motion_diag_next_error_ms;
static ArmMotionTrajectoryContext arm_motion_trajectory_context;
static volatile uint32_t arm_motion_trajectory_generation;
static volatile uint32_t arm_motion_stop_depth;
static osMutexId_t arm_motion_command_mutex;
static osMutexId_t arm_motion_feedback_config_mutex;
static osMutexId_t arm_motion_trajectory_api_mutex;

static ArmMotionStatus ArmMotion_TrajectoryBestEffortCleanup(
    const ArmMotionConfig *config);
static bool ArmMotion_TrajectoryStateIsBusy(
    ArmMotionTrajectoryState state);
static ArmMotionStatus ArmMotion_SetTimedReturnInfoUnlocked(
    const ArmMotionConfig *config,
    uint8_t info_function,
    uint16_t period_ms);

static bool ArmMotion_TimeReached(uint32_t now_ms, uint32_t due_ms)
{
    return ((int32_t)(now_ms - due_ms) >= 0);
}

static bool ArmMotion_TimeAfter(uint32_t timestamp_ms, uint32_t barrier_ms)
{
    return ((int32_t)(timestamp_ms - barrier_ms) > 0);
}

static bool ArmMotion_CommandLock(void)
{
    if (arm_motion_command_mutex == NULL)
    {
        arm_motion_command_mutex = osMutexNew(NULL);
        if (arm_motion_command_mutex == NULL)
        {
            return false;
        }
    }
    return (osMutexAcquire(arm_motion_command_mutex, osWaitForever) == osOK);
}

static void ArmMotion_CommandUnlock(void)
{
    if (arm_motion_command_mutex != NULL)
    {
        (void)osMutexRelease(arm_motion_command_mutex);
    }
}

static bool ArmMotion_FeedbackConfigLock(void)
{
    if (arm_motion_feedback_config_mutex == NULL)
    {
        arm_motion_feedback_config_mutex = osMutexNew(NULL);
        if (arm_motion_feedback_config_mutex == NULL)
        {
            return false;
        }
    }
    return (osMutexAcquire(arm_motion_feedback_config_mutex,
                           osWaitForever) == osOK);
}

static void ArmMotion_FeedbackConfigUnlock(void)
{
    if (arm_motion_feedback_config_mutex != NULL)
    {
        (void)osMutexRelease(arm_motion_feedback_config_mutex);
    }
}

static bool ArmMotion_TrajectoryApiLock(void)
{
    if (arm_motion_trajectory_api_mutex == NULL)
    {
        arm_motion_trajectory_api_mutex = osMutexNew(NULL);
        if (arm_motion_trajectory_api_mutex == NULL)
        {
            return false;
        }
    }
    /* 控制入口不排队：忙时立即返回，避免 Stop 后执行先前排队的旧请求。 */
    return (osMutexAcquire(arm_motion_trajectory_api_mutex, 0U) == osOK);
}

static void ArmMotion_TrajectoryApiUnlock(void)
{
    if (arm_motion_trajectory_api_mutex != NULL)
    {
        (void)osMutexRelease(arm_motion_trajectory_api_mutex);
    }
}

static uint32_t ArmMotion_ReadGeneration(void)
{
    uint32_t generation;

    taskENTER_CRITICAL();
    generation = arm_motion_trajectory_generation;
    taskEXIT_CRITICAL();
    return generation;
}

static bool ArmMotion_TryCaptureCommandToken(uint32_t *generation)
{
    bool captured = false;

    if (generation == NULL)
    {
        return false;
    }
    taskENTER_CRITICAL();
    if (arm_motion_stop_depth == 0U)
    {
        *generation = arm_motion_trajectory_generation;
        captured = true;
    }
    taskEXIT_CRITICAL();
    return captured;
}

static bool ArmMotion_CommandTokenIsCurrent(uint32_t generation)
{
    bool current;

    taskENTER_CRITICAL();
    current = (arm_motion_stop_depth == 0U) &&
              (arm_motion_trajectory_generation == generation);
    taskEXIT_CRITICAL();
    return current;
}

static bool ArmMotion_TryPublishTrajectoryState(
    uint32_t generation,
    ArmMotionTrajectoryState state,
    ArmMotionStatus status)
{
    bool published = false;

    taskENTER_CRITICAL();
    if ((arm_motion_stop_depth == 0U) &&
        (arm_motion_trajectory_generation == generation))
    {
        arm_motion_trajectory_context.result.state = state;
        arm_motion_trajectory_context.result.status = status;
        published = true;
    }
    taskEXIT_CRITICAL();
    return published;
}

static void ArmMotion_StopGateExit(void)
{
    taskENTER_CRITICAL();
    if (arm_motion_stop_depth > 0U)
    {
        --arm_motion_stop_depth;
    }
    taskEXIT_CRITICAL();
}

static int8_t ArmMotion_FindMotorIndex(const ArmMotionConfig *config,
                                       uint8_t address)
{
    uint8_t index;

    if (config == NULL)
    {
        return -1;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (config->motor_addr[index] == address)
        {
            return (int8_t)index;
        }
    }
    return -1;
}

static void ArmMotion_ClearMoveTracker(void)
{
    taskENTER_CRITICAL();
    memset(&arm_motion_move_tracker, 0, sizeof(arm_motion_move_tracker));
    taskEXIT_CRITICAL();
}

static void ArmMotion_CommitPendingMove(void)
{
    uint8_t index;

    taskENTER_CRITICAL();
    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        arm_motion_current_pulse[index] =
            arm_motion_move_tracker.target_pulse[index];
    }
    arm_motion_current_valid = true;
    memset(&arm_motion_move_tracker, 0, sizeof(arm_motion_move_tracker));
    taskEXIT_CRITICAL();
}

/*
 * token 校验、9F 完成判定和 current_pulse 提交必须处于同一个临界区，
 * 否则 StopAll 清 tracker 的瞬间可能被误判成 pending==reached==0。
 */
static ArmMotionStatus ArmMotion_CheckMoveCompletion(
    uint32_t request_generation,
    bool publish_trajectory_done)
{
    ArmMotionStatus status = ARM_MOTION_BUSY;

    taskENTER_CRITICAL();
    if ((arm_motion_stop_depth != 0U) ||
        (arm_motion_trajectory_generation != request_generation))
    {
        status = ARM_MOTION_CANCELLED;
    }
    else if (arm_motion_move_tracker.error_mask != 0U)
    {
        status = ARM_MOTION_DRIVER_ERROR;
    }
    else if (!arm_motion_move_tracker.active ||
             ((arm_motion_move_tracker.reached_mask &
               arm_motion_move_tracker.pending_mask) ==
              arm_motion_move_tracker.pending_mask))
    {
        if (arm_motion_move_tracker.active)
        {
            ArmMotion_CommitPendingMove();
        }
        if (publish_trajectory_done)
        {
            arm_motion_trajectory_context.result.state =
                ARM_MOTION_TRAJECTORY_DONE;
            arm_motion_trajectory_context.result.status = ARM_MOTION_OK;
        }
        status = ARM_MOTION_OK;
    }
    taskEXIT_CRITICAL();
    return status;
}

static uint32_t ArmMotion_ReadU32BE(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           ((uint32_t)data[3] << 0);
}

static void ArmMotion_ProcessRxFrame(const ArmMotionConfig *config,
                                     const ZDT_CanRxFrame_t *frame)
{
    uint8_t address;
    uint8_t function_code;
    uint8_t response_code;
    uint8_t status_flags;
    uint32_t raw_value;
    int8_t motor_index;
    float value;
    uint32_t now_ms;

    if ((config == NULL) || (frame == NULL) || (frame->dlc < 3U))
    {
        return;
    }

    if (frame->data[frame->dlc - 1U] != 0x6BU)
    {
        return;
    }

    address = (uint8_t)((frame->id >> 8) & 0xFFU);
    motor_index = ArmMotion_FindMotorIndex(config, address);
    if (motor_index < 0)
    {
        return;
    }

    function_code = frame->data[0];
    response_code = frame->data[1];
    /* 使用 CAN ISR 真正收到该帧的时刻，不用任务延后解析时刻。 */
    now_ms = frame->timestamp_ms;

    arm_motion_driver_status[(uint8_t)motor_index].last_update_ms = now_ms;

    if (((function_code == 0xFDU) || (function_code == 0xF6U) ||
         (function_code == 0x68U) || (function_code == 0x11U)) &&
        (response_code == 0x02U))
    {
        arm_motion_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        if ((function_code == 0x68U) &&
            (arm_motion_trajectory_context.result.state !=
             ARM_MOTION_TRAJECTORY_IDLE) &&
            ArmMotion_TimeReached(
                now_ms,
                arm_motion_trajectory_context.heartbeat_request_ms))
        {
            arm_motion_trajectory_context.heartbeat_ack_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        if ((function_code == 0x11U) &&
            arm_motion_trajectory_context.timed_stop_waiting &&
            ArmMotion_TimeAfter(
                now_ms,
                arm_motion_trajectory_context.timed_stop_request_ms) &&
            (arm_motion_trajectory_context.
             timed_stop_ack_count[(uint8_t)motor_index] <
             ARM_MOTION_TIMED_STOP_ACKS_PER_MOTOR))
        {
            ++arm_motion_trajectory_context.
                timed_stop_ack_count[(uint8_t)motor_index];
        }
        return;
    }

    /* FD 9F 是位置模式动作执行完成帧，只作为正常完成判据。 */
    if ((function_code == 0xFDU) && (response_code == 0x9FU))
    {
        arm_motion_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        arm_motion_driver_status[(uint8_t)motor_index].position_reached_flag =
            true;
        taskENTER_CRITICAL();
        if (arm_motion_move_tracker.active)
        {
            arm_motion_move_tracker.reached_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        taskEXIT_CRITICAL();
        return;
    }

    /*
     * FD/F6 的 12、22、E2、EE 都表示本次命令没有正常执行：
     * 零位/限位输入已触发、参数/保护错误或命令格式错误。
     */
    if ((((function_code == 0xFDU) || (function_code == 0xF6U)) &&
         ((response_code == 0x12U) || (response_code == 0x22U) ||
          (response_code == 0xE2U) || (response_code == 0xEEU))) ||
        ((function_code == 0x68U) &&
         ((response_code == 0xE2U) || (response_code == 0xEEU))))
    {
        arm_motion_driver_status[(uint8_t)motor_index].last_response_code =
            response_code;
        arm_motion_driver_status[(uint8_t)motor_index].last_error_code =
            response_code;
        taskENTER_CRITICAL();
        if (arm_motion_move_tracker.active && (function_code == 0xFDU))
        {
            arm_motion_move_tracker.error_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
            arm_motion_move_tracker.last_error_code = response_code;
        }
        taskEXIT_CRITICAL();
        if ((arm_motion_trajectory_context.result.state !=
             ARM_MOTION_TRAJECTORY_IDLE) &&
            (arm_motion_trajectory_context.result.state !=
             ARM_MOTION_TRAJECTORY_DONE) &&
            (arm_motion_trajectory_context.result.state !=
             ARM_MOTION_TRAJECTORY_FAULT))
        {
            arm_motion_trajectory_context.driver_error_mask |=
                (uint8_t)(1U << (uint8_t)motor_index);
        }
        return;
    }

    /* 35：X 固件实时速度，单位 0.1 RPM，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_REALTIME_SPEED) &&
        (frame->dlc >= 5U))
    {
        raw_value = ((uint32_t)frame->data[2] << 8) |
                    (uint32_t)frame->data[3];
        value = (float)raw_value * 0.1f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_motion_driver_status[(uint8_t)motor_index].realtime_speed_rpm =
            value;
        arm_motion_driver_status[(uint8_t)motor_index].realtime_speed_valid =
            true;
        arm_motion_driver_status[(uint8_t)motor_index].
            realtime_speed_update_ms = now_ms;
        return;
    }

    /* 36：X 固件实时位置，单位 0.1 度，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_REALTIME_POSITION) &&
        (frame->dlc >= 7U))
    {
        raw_value = ArmMotion_ReadU32BE(&frame->data[2]);
        value = (float)raw_value * 0.1f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_motion_driver_status[(uint8_t)motor_index].realtime_position_deg =
            value;
        arm_motion_driver_status[(uint8_t)motor_index].realtime_position_valid =
            true;
        arm_motion_driver_status[(uint8_t)motor_index].
            realtime_position_update_ms = now_ms;
        return;
    }

    /* 37：X 固件位置误差，单位 0.01 度，data[1] 为符号。 */
    if ((function_code == X_V2_INFO_POSITION_ERROR) &&
        (frame->dlc >= 7U))
    {
        raw_value = ArmMotion_ReadU32BE(&frame->data[2]);
        value = (float)raw_value * 0.01f;
        if (response_code != 0U)
        {
            value = -value;
        }
        arm_motion_driver_status[(uint8_t)motor_index].position_error_deg =
            value;
        arm_motion_driver_status[(uint8_t)motor_index].position_error_valid =
            true;
        arm_motion_driver_status[(uint8_t)motor_index].
            position_error_update_ms = now_ms;
        return;
    }

    /*
     * 3A：bit0 为使能，bit2/3 为堵转相关标志；
     * bit4/5 仅记录左右限位输入，不在未标定时自动停机。
     */
    if ((function_code == X_V2_INFO_STATUS) && (frame->dlc >= 3U))
    {
        status_flags = response_code;
        arm_motion_driver_status[(uint8_t)motor_index].status_flags =
            status_flags;
        arm_motion_driver_status[(uint8_t)motor_index].status_valid = true;
        arm_motion_driver_status[(uint8_t)motor_index].status_update_ms = now_ms;
        arm_motion_driver_status[(uint8_t)motor_index].position_reached_flag =
            ((status_flags & 0x02U) != 0U);

        if (((status_flags & ARM_MOTION_DRIVER_FAULT_MASK) != 0U) ||
            (((status_flags & 0x01U) == 0U) &&
             (arm_motion_move_tracker.active ||
              ((arm_motion_trajectory_context.result.state !=
                ARM_MOTION_TRAJECTORY_IDLE) &&
               (arm_motion_trajectory_context.result.state !=
                ARM_MOTION_TRAJECTORY_DONE) &&
               (arm_motion_trajectory_context.result.state !=
                ARM_MOTION_TRAJECTORY_FAULT)))))
        {
            if (arm_motion_move_tracker.active)
            {
                arm_motion_move_tracker.error_mask |=
                    (uint8_t)(1U << (uint8_t)motor_index);
                arm_motion_move_tracker.last_error_code = 0x3AU;
            }
            if ((arm_motion_trajectory_context.result.state !=
                 ARM_MOTION_TRAJECTORY_IDLE) &&
                (arm_motion_trajectory_context.result.state !=
                 ARM_MOTION_TRAJECTORY_DONE) &&
                (arm_motion_trajectory_context.result.state !=
                 ARM_MOTION_TRAJECTORY_FAULT))
            {
                arm_motion_trajectory_context.driver_error_mask |=
                    (uint8_t)(1U << (uint8_t)motor_index);
            }
        }
    }
}

static void ArmMotion_DrainRx(const ArmMotionConfig *config,
                              uint32_t first_timeout_ticks)
{
    ZDT_CanRxFrame_t frame;
    uint32_t drain_count;

    for (drain_count = 0U;
         drain_count < ARM_MOTION_RX_DRAIN_LIMIT;
         ++drain_count)
    {
        if (ZDT_CAN_GetRx(&frame,
                          (drain_count == 0U) ? first_timeout_ticks : 0U) !=
            osOK)
        {
            break;
        }
        ArmMotion_ProcessRxFrame(config, &frame);
    }
}

static void ArmMotion_BeginDiagnostics(uint32_t now_ms)
{
    arm_motion_diag_next_status_ms = now_ms;
    arm_motion_diag_next_position_ms = now_ms;
    arm_motion_diag_next_error_ms = now_ms;
}

static void ArmMotion_ServiceDiagnostics(const ArmMotionConfig *config,
                                         uint32_t now_ms)
{
    uint8_t index;

    if (config == NULL)
    {
        return;
    }
    if (ArmMotion_TimeReached(now_ms, arm_motion_diag_next_status_ms))
    {
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            (void)X_V2_Read_Info(config->motor_addr[index],
                                 X_V2_INFO_STATUS);
        }
        arm_motion_diag_next_status_ms =
            now_ms + ARM_MOTION_DIAG_STATUS_PERIOD_MS;
    }

    if (ArmMotion_TimeReached(now_ms, arm_motion_diag_next_position_ms))
    {
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            (void)X_V2_Read_Info(config->motor_addr[index],
                                 X_V2_INFO_REALTIME_POSITION);
        }
        arm_motion_diag_next_position_ms =
            now_ms + ARM_MOTION_DIAG_POSITION_PERIOD_MS;
    }

    if (ArmMotion_TimeReached(now_ms, arm_motion_diag_next_error_ms))
    {
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            (void)X_V2_Read_Info(config->motor_addr[index],
                                 X_V2_INFO_POSITION_ERROR);
        }
        arm_motion_diag_next_error_ms =
            now_ms + ARM_MOTION_DIAG_ERROR_PERIOD_MS;
    }
}

static ArmMotionStatus ArmMotion_WaitForCompletion(
    const ArmMotionConfig *config,
    float requested_time_s,
    const ArmMotionProfile *profile,
    uint32_t request_generation)
{
    uint32_t now_ms;
    uint32_t deadline_ms;
    uint32_t timeout_ms;
    uint32_t timeout_ticks;
    uint32_t drain_count;
    double timeout_s;
    double timeout_ms_double;
    ArmMotionStatus completion_status;
    ArmMotionStatus stop_status;

    if ((config == NULL) || (profile == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }

    completion_status = ArmMotion_CheckMoveCompletion(
        request_generation, false);
    if ((completion_status == ARM_MOTION_OK) ||
        (completion_status == ARM_MOTION_CANCELLED))
    {
        return completion_status;
    }
    if (completion_status == ARM_MOTION_DRIVER_ERROR)
    {
        stop_status = ArmMotion_StopAll(config);
        return (stop_status == ARM_MOTION_OK) ?
               ARM_MOTION_DRIVER_ERROR : ARM_MOTION_CAN_ERROR;
    }

    timeout_s = (double)requested_time_s;
    for (drain_count = 0U; drain_count < ARM_MOTION_MOTOR_COUNT;
         ++drain_count)
    {
        if ((double)profile->predicted_time_s[drain_count] > timeout_s)
        {
            timeout_s = (double)profile->predicted_time_s[drain_count];
        }
    }
    timeout_ms_double = ceil(timeout_s * 1000.0) +
                        (double)ARM_MOTION_REACHED_TIMEOUT_MARGIN_MS;
    if ((timeout_ms_double <= 0.0) ||
        (timeout_ms_double > (double)ARM_MOTION_MAX_OS_DELAY_TICKS))
    {
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = false;
        return ARM_MOTION_TIME_INVALID;
    }

    timeout_ms = (uint32_t)timeout_ms_double;
    now_ms = HAL_GetTick();
    deadline_ms = now_ms + timeout_ms;
    ArmMotion_BeginDiagnostics(now_ms);

    for (;;)
    {
        now_ms = HAL_GetTick();

        completion_status = ArmMotion_CheckMoveCompletion(
            request_generation, false);
        if ((completion_status == ARM_MOTION_OK) ||
            (completion_status == ARM_MOTION_CANCELLED))
        {
            return completion_status;
        }
        if (completion_status == ARM_MOTION_DRIVER_ERROR)
        {
            stop_status = ArmMotion_StopAll(config);
            if (stop_status != ARM_MOTION_OK)
            {
                return ARM_MOTION_CAN_ERROR;
            }
            return ARM_MOTION_DRIVER_ERROR;
        }

        if ((int32_t)(now_ms - deadline_ms) >= 0)
        {
            stop_status = ArmMotion_StopAll(config);
            if (stop_status != ARM_MOTION_OK)
            {
                return ARM_MOTION_CAN_ERROR;
            }
            return ARM_MOTION_REACHED_TIMEOUT;
        }

        ArmMotion_ServiceDiagnostics(config, now_ms);

        timeout_ticks = 10U;
        ArmMotion_DrainRx(config, timeout_ticks);

        completion_status = ArmMotion_CheckMoveCompletion(
            request_generation, false);
        if ((completion_status == ARM_MOTION_OK) ||
            (completion_status == ARM_MOTION_CANCELLED))
        {
            return completion_status;
        }
        if (completion_status == ARM_MOTION_DRIVER_ERROR)
        {
            continue;
        }

        osDelay(1U);
    }
}

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

    if ((config->trajectory_period_ms == 0U) ||
        (config->trajectory_feedback_timeout_ms == 0U) ||
        (config->trajectory_deadline_tolerance_ms == 0U) ||
        ((config->trajectory_heartbeat_ms != 0U) &&
         ((config->trajectory_heartbeat_ms <=
           config->trajectory_feedback_timeout_ms) ||
          ((uint32_t)config->trajectory_heartbeat_ms <=
           ((uint32_t)config->trajectory_period_ms +
            (uint32_t)config->trajectory_deadline_tolerance_ms)))) ||
        !isfinite(config->trajectory_limit_scale) ||
        (config->trajectory_limit_scale <= 0.0f) ||
        (config->trajectory_limit_scale > 1.0f) ||
        !isfinite(config->trajectory_settle_time_s) ||
        (config->trajectory_settle_time_s <= 0.0f) ||
        !isfinite(config->trajectory_min_radial_mm) ||
        (config->trajectory_min_radial_mm < 0.0f))
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
            (config->vel_rpm[index] <= 0.0f) ||
            (config->vel_rpm[index] > 3000.0f) ||
            (config->acc_rpm_per_s[index] == 0U) ||
            (config->dec_rpm_per_s[index] == 0U) ||
            !isfinite(config->trajectory_kp_rpm_per_deg[index]) ||
            (config->trajectory_kp_rpm_per_deg[index] < 0.0f) ||
            !isfinite(config->trajectory_correction_limit_rpm[index]) ||
            (config->trajectory_correction_limit_rpm[index] < 0.0f) ||
            !isfinite(config->trajectory_feedback_error_limit_deg[index]) ||
            (config->trajectory_feedback_error_limit_deg[index] <= 0.0f))
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
         ((double)ARM_MOTION_MIN_SPEED_RPM *
          (double)ARM_MOTION_DEG_PER_RPM_S)))
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
    uint8_t pending_mask;
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
    pending_mask = 0U;

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
            if (distance_deg > 0.0)
            {
                pending_mask |= (uint8_t)(1U << index);
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
            /* MoveToXYZ 必须依靠 FD 9F 完成判定，强制打开到位返回。 */
            position_commands[index].snF =
                (time_s > 0.0f) ? true : config->report_reached[index];
        }

        if (time_s > 0.0f)
        {
            /* 丢弃上一动作残留的确认/诊断帧，避免旧 9F 误判本动作。 */
            (void)ZDT_CAN_FlushRx();
        }

        can_status = X_V2_MultiMotor_Traj_Pos_Control(
            position_commands, ARM_MOTION_MOTOR_COUNT);
        if (can_status != HAL_OK)
        {
            ArmMotion_ClearMoveTracker();
            arm_motion_current_valid = false;
            return ARM_MOTION_CAN_ERROR;
        }

        if (time_s > 0.0f)
        {
            taskENTER_CRITICAL();
            for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
            {
                arm_motion_move_tracker.target_pulse[index] =
                    targets->pulse[index];
            }
            arm_motion_move_tracker.pending_mask = pending_mask;
            arm_motion_move_tracker.reached_mask = 0U;
            arm_motion_move_tracker.error_mask = 0U;
            arm_motion_move_tracker.last_error_code = 0U;
            arm_motion_move_tracker.active = (pending_mask != 0U);
            for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
            {
                if ((pending_mask & (uint8_t)(1U << index)) != 0U)
                {
                    arm_motion_driver_status[index].position_reached_flag =
                        false;
                    arm_motion_driver_status[index].last_error_code = 0U;
                }
            }
            if (!arm_motion_move_tracker.active)
            {
                ArmMotion_CommitPendingMove();
            }
            taskEXIT_CRITICAL();
        }
        else if (arm_motion_active_config == config)
        {
            taskENTER_CRITICAL();
            for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
            {
                arm_motion_current_pulse[index] = targets->pulse[index];
            }
            arm_motion_current_valid = true;
            taskEXIT_CRITICAL();
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
            ArmMotion_ClearMoveTracker();
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
        taskENTER_CRITICAL();
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            arm_motion_current_pulse[index] = targets->pulse[index];
        }
        arm_motion_current_valid = true;
        taskEXIT_CRITICAL();
    }
    return ARM_MOTION_OK;
}

void ArmMotion_LoadDefaultConfig(ArmMotionConfig *config)
{
    uint8_t index;
    uint32_t generation;

    if (config == NULL)
    {
        return;
    }
    if (!ArmMotion_TrajectoryApiLock())
    {
        return;
    }
    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return;
    }
    if (!ArmMotion_FeedbackConfigLock())
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return;
    }
    generation = ArmMotion_ReadGeneration();
    if (!ArmMotion_CommandTokenIsCurrent(generation) ||
        ((arm_motion_active_config == config) &&
         (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)))
    {
        /* 运动中不允许清空正在使用的配置和状态机。 */
        ArmMotion_FeedbackConfigUnlock();
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
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
        config->report_reached[index] = true;
        config->trajectory_kp_rpm_per_deg[index] =
            ARM_MOTION_TRAJECTORY_DEFAULT_KP;
        config->trajectory_correction_limit_rpm[index] =
            ARM_MOTION_TRAJECTORY_DEFAULT_CORRECTION_RPM;
        config->trajectory_feedback_error_limit_deg[index] =
            ARM_MOTION_TRAJECTORY_DEFAULT_ERROR_DEG;
    }

    config->use_multi_command = true;
    config->use_sync = true;
    config->trajectory_period_ms =
        ARM_MOTION_TRAJECTORY_DEFAULT_PERIOD_MS;
    config->trajectory_feedback_timeout_ms =
        ARM_MOTION_TRAJECTORY_DEFAULT_FEEDBACK_TIMEOUT_MS;
    config->trajectory_deadline_tolerance_ms =
        ARM_MOTION_TRAJECTORY_DEFAULT_DEADLINE_TOLERANCE_MS;
    config->trajectory_heartbeat_ms =
        ARM_MOTION_TRAJECTORY_DEFAULT_HEARTBEAT_MS;
    config->trajectory_limit_scale =
        ARM_MOTION_TRAJECTORY_DEFAULT_LIMIT_SCALE;
    config->trajectory_settle_time_s =
        ARM_MOTION_TRAJECTORY_DEFAULT_SETTLE_TIME_S;
    config->trajectory_min_radial_mm =
        ARM_MOTION_TRAJECTORY_DEFAULT_MIN_RADIAL_MM;
    config->zero_confirmed = false;

    if (arm_motion_active_config == config)
    {
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = false;
        memset(&arm_motion_trajectory_context,
               0,
               sizeof(arm_motion_trajectory_context));
    }
    ArmMotion_FeedbackConfigUnlock();
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
}

ArmMotionStatus ArmMotion_SetActiveConfig(ArmMotionConfig *config)
{
    uint32_t generation;

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_NOT_INITIALIZED;
    }
    if (!ArmMotion_FeedbackConfigLock())
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_NOT_INITIALIZED;
    }
    generation = ArmMotion_ReadGeneration();
    if (!ArmMotion_CommandTokenIsCurrent(generation) ||
        ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_FeedbackConfigUnlock();
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_BUSY;
    }

    arm_motion_active_config = config;
    ArmMotion_ClearMoveTracker();
    arm_motion_current_valid = false;
    arm_motion_last_ik_status = ARM_IK_OK;
    memset(&arm_motion_last_result, 0, sizeof(arm_motion_last_result));
    memset(arm_motion_driver_status,
           0,
           sizeof(arm_motion_driver_status));
    memset(&arm_motion_trajectory_context,
           0,
           sizeof(arm_motion_trajectory_context));
    arm_motion_trajectory_context.result.state =
        ARM_MOTION_TRAJECTORY_IDLE;
    arm_motion_trajectory_context.result.status = ARM_MOTION_OK;
    ArmMotion_FeedbackConfigUnlock();
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
    return ARM_MOTION_OK;
}

void ArmMotion_ConfirmZero(ArmMotionConfig *config, bool confirmed)
{
    uint32_t generation;

    if (config == NULL)
    {
        return;
    }
    if (!ArmMotion_TrajectoryApiLock())
    {
        return;
    }
    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return;
    }
    generation = ArmMotion_ReadGeneration();
    if (!ArmMotion_CommandTokenIsCurrent(generation) ||
        ((arm_motion_active_config == config) &&
         (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)))
    {
        /* 零位只能在所有运动结束后由归零流程确认或撤销。 */
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return;
    }
    config->zero_confirmed = confirmed;

    if (arm_motion_active_config != config)
    {
        /* 非活动配置不得清除当前机械臂的全局运动跟踪状态。 */
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return;
    }
    if (!confirmed)
    {
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = false;
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
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
            ArmMotion_CommandUnlock();
            ArmMotion_TrajectoryApiUnlock();
            return;
        }

        arm_motion_current_pulse[0] = zero_targets.pulse[0];
        arm_motion_current_pulse[1] = zero_targets.pulse[1];
        arm_motion_current_pulse[2] = zero_targets.pulse[2];
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = true;
    }
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
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
    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CAN_ERROR;
    }
    if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_BUSY;
    }

    config->ik_branch = branch;
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
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
    ArmMotionStatus cleanup_status = ARM_MOTION_OK;
    bool trajectory_was_busy;
    bool stop_failed = false;
    uint8_t index;

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    /*
     * 先原子关闭新运动入口并使所有在途计算代次失效；Stop 完成前，
     * 即使别的任务读到了新代次，也会被 stop_depth 门闩拒绝。
     */
    taskENTER_CRITICAL();
    ++arm_motion_stop_depth;
    ++arm_motion_trajectory_generation;
    trajectory_was_busy = ArmMotion_TrajectoryStateIsBusy(
        arm_motion_trajectory_context.result.state);
    taskEXIT_CRITICAL();

    if (!ArmMotion_CommandLock())
    {
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = false;
        if (trajectory_was_busy)
        {
            arm_motion_trajectory_context.result.state =
                ARM_MOTION_TRAJECTORY_FAULT;
            arm_motion_trajectory_context.result.status =
                ARM_MOTION_CAN_ERROR;
        }
        ArmMotion_StopGateExit();
        return ARM_MOTION_CAN_ERROR;
    }
    /* 地址 0 会广播到同一 CAN 总线的 ID4 和底盘；这里只停止 XYZ 三轴。 */
    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_Stop_Now(config->motor_addr[index], false) != HAL_OK)
        {
            stop_failed = true;
        }
    }
    if (stop_failed)
    {
        ArmMotion_CommandUnlock();
        ArmMotion_ClearMoveTracker();
        arm_motion_current_valid = false;
        if (trajectory_was_busy)
        {
            /*
             * 停止帧未发出时必须保留驱动器心跳。否则“停止失败、
             * 关心跳成功”会让电机持续执行最后一条 F6 而失去超时急停。
             */
            arm_motion_trajectory_context.result.state =
                ARM_MOTION_TRAJECTORY_FAULT;
            arm_motion_trajectory_context.result.status =
                ARM_MOTION_CAN_ERROR;
        }
        ArmMotion_StopGateExit();
        return ARM_MOTION_CAN_ERROR;
    }
    ArmMotion_CommandUnlock();

    ArmMotion_ClearMoveTracker();
    arm_motion_current_valid = false;
    /* cleanup 内部会读取最终标志；无条件调用可覆盖并发 enable 的尾部。 */
    cleanup_status = ArmMotion_TrajectoryBestEffortCleanup(config);
    if (trajectory_was_busy)
    {
        arm_motion_trajectory_context.result.state =
            ARM_MOTION_TRAJECTORY_FAULT;
        arm_motion_trajectory_context.result.status =
            (cleanup_status == ARM_MOTION_OK) ?
            ARM_MOTION_CANCELLED : cleanup_status;
    }
    ArmMotion_StopGateExit();
    return cleanup_status;
}

static ArmMotionStatus ArmMotion_MoveToJointsInternal(
    const ArmMotionConfig *config,
    const ArmJointAngles *joints,
    ArmMotionResult *result,
    ArmIkStatus *ik_status,
    float time_s,
    uint32_t request_generation,
    bool command_lock_held)
{
    ArmIkStatus joint_status;
    ArmMotorMapStatus map_status;
    ArmMotionStatus motion_status;
    bool acquired_here = false;

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

    if (!command_lock_held)
    {
        if (!ArmMotion_CommandLock())
        {
            return ARM_MOTION_CAN_ERROR;
        }
        acquired_here = true;

        /* 锁内重查，避免 Stop 或另一条运动在计算期间抢先提交。 */
        if (!ArmMotion_CommandTokenIsCurrent(request_generation))
        {
            ArmMotion_CommandUnlock();
            return ARM_MOTION_CANCELLED;
        }
        if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
        {
            ArmMotion_CommandUnlock();
            return ARM_MOTION_BUSY;
        }
    }
    else if (!ArmMotion_CommandTokenIsCurrent(request_generation))
    {
        return ARM_MOTION_CANCELLED;
    }

    motion_status = ArmMotion_SendTargets(
        config, &result->motors, time_s, &result->profile);
    if (!ArmMotion_CommandTokenIsCurrent(request_generation))
    {
        motion_status = ARM_MOTION_CANCELLED;
    }
    if (acquired_here)
    {
        ArmMotion_CommandUnlock();
    }
    return motion_status;
}

ArmMotionStatus ArmMotion_MoveToJoints(const ArmMotionConfig *config,
                                       const ArmJointAngles *joints,
                                       ArmMotionResult *result,
                                       ArmIkStatus *ik_status)
{
    ArmMotionStatus motion_status;
    uint32_t request_generation;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_TryCaptureCommandToken(&request_generation))
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CANCELLED;
    }

    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CAN_ERROR;
    }
    if (!ArmMotion_CommandTokenIsCurrent(request_generation))
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CANCELLED;
    }
    if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_BUSY;
    }
    motion_status = ArmMotion_MoveToJointsInternal(
        config,
        joints,
        result,
        ik_status,
        0.0f,
        request_generation,
        true);
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
    return motion_status;
}

static ArmMotionStatus ArmMotion_MoveToXYZInternal(float x_mm,
                                                   float y_mm,
                                                   float z_mm,
                                                   float time_s)
{
    ArmJointAngles joints;
    ArmIkStatus solve_status;
    ArmMotionStatus motion_status;
    ArmMotionConfig *config;
    ArmMotionResult *result = &arm_motion_last_result;
    ArmIkStatus *ik_status = &arm_motion_last_ik_status;
    uint32_t request_generation;

    if (!ArmMotion_TryCaptureCommandToken(&request_generation))
    {
        return ARM_MOTION_CANCELLED;
    }
    if (!ArmMotion_CommandLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    if (!ArmMotion_CommandTokenIsCurrent(request_generation))
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_CANCELLED;
    }

    config = arm_motion_active_config;

    *ik_status = ARM_IK_OK;
    memset(result, 0, sizeof(*result));

    if (config == NULL)
    {
        *ik_status = ARM_IK_NULL_POINTER;
        ArmMotion_CommandUnlock();
        return ARM_MOTION_NOT_INITIALIZED;
    }
    if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_BUSY;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_TIME_INVALID;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        *ik_status = ARM_IK_INVALID_CONFIG;
        ArmMotion_CommandUnlock();
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        ArmMotion_CommandUnlock();
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
        ArmMotion_CommandUnlock();
        return ARM_MOTION_IK_ERROR;
    }

    motion_status = ArmMotion_MoveToJointsInternal(
        config,
        &joints,
        result,
        ik_status,
        time_s,
        request_generation,
        true);
    ArmMotion_CommandUnlock();
    if (motion_status == ARM_MOTION_OK)
    {
        /* 等待所有有效轴返回 FD 9F；收到最后一帧后立即返回。 */
        motion_status = ArmMotion_WaitForCompletion(
            config,
            time_s,
            &result->profile,
            request_generation);
    }
    return motion_status;
}

ArmMotionStatus ArmMotion_MoveToXYZ(float x_mm,
                                    float y_mm,
                                    float z_mm,
                                    float time_s)
{
    ArmMotionStatus status;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    status = ArmMotion_MoveToXYZInternal(x_mm, y_mm, z_mm, time_s);
    ArmMotion_TrajectoryApiUnlock();
    return status;
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
    ArmMotionStatus motion_status;
    uint32_t request_generation;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_TryCaptureCommandToken(&request_generation))
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CANCELLED;
    }
    if (!ArmMotion_CommandLock())
    {
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CAN_ERROR;
    }
    if (!ArmMotion_CommandTokenIsCurrent(request_generation))
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_CANCELLED;
    }
    if (ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_BUSY;
    }

    if (ik_status != NULL)
    {
        *ik_status = ARM_IK_OK;
    }
    if ((config == NULL) || (result == NULL))
    {
        if (ik_status != NULL) *ik_status = ARM_IK_NULL_POINTER;
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_NULL_POINTER;
    }

    map_status = ArmMotorMap_OffsetsToJoints(&config->motor_map,
                                             base_deg,
                                             large_arm_deg,
                                             small_arm_deg,
                                             &joints);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        ArmMotion_CommandUnlock();
        ArmMotion_TrajectoryApiUnlock();
        return ARM_MOTION_MAPPING_ERROR;
    }

    motion_status = ArmMotion_MoveToJointsInternal(
        config,
        &joints,
        result,
        ik_status,
        0.0f,
        request_generation,
        true);
    ArmMotion_CommandUnlock();
    ArmMotion_TrajectoryApiUnlock();
    return motion_status;
}

static bool ArmMotion_TrajectoryStateIsBusy(
    ArmMotionTrajectoryState state)
{
    return (state == ARM_MOTION_TRAJECTORY_PRECHECKING) ||
           (state == ARM_MOTION_TRAJECTORY_WAITING_FEEDBACK) ||
           (state == ARM_MOTION_TRAJECTORY_RUNNING_SPEED) ||
           (state == ARM_MOTION_TRAJECTORY_STOPPING_SPEED) ||
           (state == ARM_MOTION_TRAJECTORY_SETTLING_POSITION);
}

static ArmMotorDirection ArmMotion_PositiveDirection(
    const ArmMotionConfig *config,
    uint8_t index)
{
    if (index == 0U)
    {
        return config->motor_map.motor1_positive_direction;
    }
    if (index == 1U)
    {
        return config->motor_map.motor2_positive_direction;
    }
    return config->motor_map.motor3_positive_direction;
}

static X_V2_Direction_t ArmMotion_VelocityDirection(
    const ArmMotionConfig *config,
    uint8_t index,
    float signed_rpm)
{
    ArmMotorDirection direction = ArmMotion_PositiveDirection(config, index);

    if (signed_rpm < 0.0f)
    {
        direction = (direction == ARM_MOTOR_DIR_CW) ?
                    ARM_MOTOR_DIR_CCW : ARM_MOTOR_DIR_CW;
    }
    return (X_V2_Direction_t)direction;
}

static float ArmMotion_LogicalPositionDegrees(
    const ArmMotionConfig *config,
    uint8_t index,
    float driver_position_deg)
{
    /* X 固件 36 的正号是 CW；映射层的正号由各轴实测方向定义。 */
    return (ArmMotion_PositiveDirection(config, index) ==
            ARM_MOTOR_DIR_CW) ?
           driver_position_deg : -driver_position_deg;
}

static bool ArmMotion_TrajectoryStartMatchesFeedback(
    const ArmMotionConfig *config)
{
    float actual_degrees;
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].realtime_position_valid)
        {
            return false;
        }
        actual_degrees = ArmMotion_LogicalPositionDegrees(
            config,
            index,
            arm_motion_driver_status[index].realtime_position_deg);
        if (!isfinite(actual_degrees) ||
            (fabsf(actual_degrees - arm_motion_trajectory_context.
                   reference_coordinates.degree[index]) >
             ARM_MOTION_TRAJECTORY_START_MATCH_DEG))
        {
            return false;
        }
    }
    return true;
}

static ArmMotionStatus ArmMotion_SyncCurrentFromFeedback(
    const ArmMotionConfig *config,
    uint32_t now_ms)
{
    int32_t pulse[ARM_MOTION_MOTOR_COUNT];
    uint8_t index;
    double pulse_value;
    double rounded;
    float logical_degrees;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].realtime_position_valid ||
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        realtime_position_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return ARM_MOTION_FEEDBACK_TIMEOUT;
        }

        logical_degrees = ArmMotion_LogicalPositionDegrees(
            config,
            index,
            arm_motion_driver_status[index].realtime_position_deg);
        pulse_value = (double)logical_degrees *
                      (double)config->motor_map.pulses_per_revolution /
                      360.0;
        rounded = (pulse_value >= 0.0) ?
                  floor(pulse_value + 0.5) : ceil(pulse_value - 0.5);
        if (!isfinite(rounded) ||
            (rounded > (double)INT32_MAX) ||
            (rounded < (double)INT32_MIN))
        {
            return ARM_MOTION_MAPPING_ERROR;
        }
        pulse[index] = (int32_t)rounded;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        arm_motion_current_pulse[index] = pulse[index];
    }
    arm_motion_current_valid = true;
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_CurrentEndpoint(
    const ArmMotionConfig *config,
    ArmCartesianPosition *position)
{
    ArmJointAngles joints;
    ArmMotorMapStatus map_status;
    ArmIkStatus ik_status;

    if ((config == NULL) || (position == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!arm_motion_current_valid || (arm_motion_active_config != config))
    {
        return ARM_MOTION_POSITION_UNKNOWN;
    }

    map_status = ArmMotorMap_PulsesToJoints(
        &config->motor_map,
        config->installation_direction,
        arm_motion_current_pulse,
        &joints);
    if (map_status != ARM_MOTOR_MAP_OK)
    {
        return ARM_MOTION_MAPPING_ERROR;
    }

    ik_status = ArmIk_ValidateJoints(&config->kinematics, &joints);
    if (ik_status != ARM_IK_OK)
    {
        return ARM_MOTION_IK_ERROR;
    }
    ik_status = ArmIk_Forward(&config->kinematics, &joints, position);
    return (ik_status == ARM_IK_OK) ?
           ARM_MOTION_OK : ARM_MOTION_IK_ERROR;
}

static ArmMotionStatus ArmMotion_SolveTrajectorySample(
    const ArmMotionConfig *config,
    const ArmCubicBezierTrajectory *trajectory,
    float elapsed_s,
    ArmJointAngles *joints,
    ArmMotorCoordinates *coordinates,
    ArmTrajectoryStatus *trajectory_status,
    ArmIkStatus *ik_status)
{
    ArmTrajectorySample sample;
    ArmMotorMapStatus map_status;
    float radial_mm;

    *trajectory_status = ArmTrajectory_Evaluate(
        trajectory, elapsed_s, &sample);
    if (*trajectory_status != ARM_TRAJECTORY_OK)
    {
        return ARM_MOTION_TRAJECTORY_ERROR;
    }

    radial_mm = sqrtf((sample.position.x_mm * sample.position.x_mm) +
                      (sample.position.y_mm * sample.position.y_mm));
    if (!isfinite(radial_mm) ||
        (radial_mm < config->trajectory_min_radial_mm))
    {
        return ARM_MOTION_TRAJECTORY_ERROR;
    }

    *ik_status = ArmIk_SolveBranch(
        &config->kinematics,
        sample.position.x_mm,
        sample.position.y_mm,
        sample.position.z_mm,
        config->ik_branch,
        joints);
    if (*ik_status != ARM_IK_OK)
    {
        return ARM_MOTION_IK_ERROR;
    }

    map_status = ArmMotorMap_JointsToMotorCoordinates(
        &config->motor_map,
        config->installation_direction,
        joints,
        coordinates);
    return (map_status == ARM_MOTOR_MAP_OK) ?
           ARM_MOTION_OK : ARM_MOTION_MAPPING_ERROR;
}

static ArmMotionStatus ArmMotion_PreflightTrajectory(
    const ArmMotionConfig *config,
    const ArmCubicBezierTrajectory *trajectory)
{
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmMotionTrajectoryResult *result = &context->result;
    ArmJointAngles previous_joints;
    ArmJointAngles joints;
    ArmMotorCoordinates previous_coordinates;
    ArmMotorCoordinates coordinates;
    ArmTrajectoryStatus trajectory_status = ARM_TRAJECTORY_OK;
    ArmIkStatus ik_status = ARM_IK_OK;
    ArmMotionStatus motion_status;
    float previous_derivative[ARM_MOTION_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_derivative[ARM_MOTION_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_acceleration[ARM_MOTION_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_deceleration[ARM_MOTION_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float derivative;
    float second_derivative;
    float absolute_value;
    float u;
    float du;
    float elapsed_s;
    float safe_limit;
    float required_time_s;
    float candidate_time_s;
    uint32_t sample_index;
    uint32_t precheck_intervals;
    uint8_t motor_index;
    int8_t derivative_direction;

    if ((config == NULL) || (trajectory == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if ((trajectory->initialized == 0U) ||
        (trajectory->segment_count == 0U) ||
        (trajectory->segment_count > ARM_MOTION_TRAJECTORY_MAX_SEGMENTS))
    {
        result->trajectory_status = ARM_TRAJECTORY_INVALID_ARGUMENT;
        return ARM_MOTION_TIME_INVALID;
    }

    memset(result->max_speed_rpm, 0, sizeof(result->max_speed_rpm));
    memset(result->max_acc_rpm_per_s,
           0,
           sizeof(result->max_acc_rpm_per_s));
    memset(result->max_dec_rpm_per_s,
           0,
           sizeof(result->max_dec_rpm_per_s));
    memset(result->final_speed_rpm,
           0,
           sizeof(result->final_speed_rpm));
    memset(context->planned_direction,
           0,
           sizeof(context->planned_direction));
    result->minimum_time_s = 0.0f;
    result->failed_sample = 0U;
    result->trajectory_status = ARM_TRAJECTORY_OK;
    result->ik_status = ARM_IK_OK;

    motion_status = ArmMotion_SolveTrajectorySample(
        config,
        trajectory,
        0.0f,
        &previous_joints,
        &previous_coordinates,
        &trajectory_status,
        &ik_status);
    result->trajectory_status = trajectory_status;
    result->ik_status = ik_status;
    if (motion_status != ARM_MOTION_OK)
    {
        return motion_status;
    }

    precheck_intervals = trajectory->segment_count;
    if (precheck_intervals < ARM_MOTION_TRAJECTORY_PRECHECK_INTERVALS)
    {
        precheck_intervals = ARM_MOTION_TRAJECTORY_PRECHECK_INTERVALS;
    }
    du = 1.0f / (float)precheck_intervals;
    for (sample_index = 1U;
         sample_index <= precheck_intervals;
         ++sample_index)
    {
        u = (float)sample_index * du;
        elapsed_s = trajectory->duration_s * u;
        motion_status = ArmMotion_SolveTrajectorySample(
            config,
            trajectory,
            elapsed_s,
            &joints,
            &coordinates,
            &trajectory_status,
            &ik_status);
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        result->failed_sample = sample_index;
        if (motion_status != ARM_MOTION_OK)
        {
            return motion_status;
        }

        /* Alpha 在 +/-180 度接缝处跳变会让底座产生约一整圈误动作。 */
        if (fabsf(joints.alpha_deg - previous_joints.alpha_deg) >=
            (180.0f - ARM_MOTION_ZERO_EPS_DEG))
        {
            return ARM_MOTION_DIRECTION_REVERSAL;
        }

        for (motor_index = 0U;
             motor_index < ARM_MOTION_MOTOR_COUNT;
             ++motor_index)
        {
            derivative = (coordinates.degree[motor_index] -
                          previous_coordinates.degree[motor_index]) / du;
            if (!isfinite(derivative))
            {
                return ARM_MOTION_TRAJECTORY_ERROR;
            }

            absolute_value = fabsf(derivative);
            if (absolute_value > maximum_derivative[motor_index])
            {
                maximum_derivative[motor_index] = absolute_value;
            }

            derivative_direction = 0;
            if (derivative >
                ARM_MOTION_TRAJECTORY_DIRECTION_EPS_DEG_PER_U)
            {
                derivative_direction = 1;
            }
            else if (derivative <
                     -ARM_MOTION_TRAJECTORY_DIRECTION_EPS_DEG_PER_U)
            {
                derivative_direction = -1;
            }

            if (derivative_direction != 0)
            {
                if (context->planned_direction[motor_index] == 0)
                {
                    context->planned_direction[motor_index] =
                        derivative_direction;
                }
                else if (context->planned_direction[motor_index] !=
                         derivative_direction)
                {
                    return ARM_MOTION_DIRECTION_REVERSAL;
                }
            }

            second_derivative =
                (derivative - previous_derivative[motor_index]) / du;
            if (!isfinite(second_derivative))
            {
                return ARM_MOTION_TRAJECTORY_ERROR;
            }
            absolute_value = fabsf(second_derivative);
            if ((derivative * second_derivative) >= 0.0f)
            {
                if (absolute_value > maximum_acceleration[motor_index])
                {
                    maximum_acceleration[motor_index] = absolute_value;
                }
            }
            else if (absolute_value > maximum_deceleration[motor_index])
            {
                maximum_deceleration[motor_index] = absolute_value;
            }

            previous_derivative[motor_index] = derivative;
        }

        previous_joints = joints;
        previous_coordinates = coordinates;
    }

    /* 五次时间律终点速度为零，把最后一个离散区间也计入减速约束。 */
    for (motor_index = 0U;
         motor_index < ARM_MOTION_MOTOR_COUNT;
         ++motor_index)
    {
        second_derivative = -previous_derivative[motor_index] / du;
        absolute_value = fabsf(second_derivative);
        if (absolute_value > maximum_deceleration[motor_index])
        {
            maximum_deceleration[motor_index] = absolute_value;
        }

        result->max_speed_rpm[motor_index] =
            maximum_derivative[motor_index] /
            (ARM_MOTION_DEG_PER_RPM_S * trajectory->duration_s);
        result->max_acc_rpm_per_s[motor_index] =
            maximum_acceleration[motor_index] /
            (ARM_MOTION_DEG_PER_RPM_S * trajectory->duration_s *
             trajectory->duration_s);
        result->max_dec_rpm_per_s[motor_index] =
            maximum_deceleration[motor_index] /
            (ARM_MOTION_DEG_PER_RPM_S * trajectory->duration_s *
             trajectory->duration_s);
    }

    /*
     * 速度按 1/T、加减速度按 1/T^2 缩放，直接反算最短时间；
     * 不枚举 A/D/V 组合。0.85 等安全系数给反馈修正和协议量化留余量。
     */
    required_time_s = 0.0f;
    for (motor_index = 0U;
         motor_index < ARM_MOTION_MOTOR_COUNT;
         ++motor_index)
    {
        safe_limit = config->vel_rpm[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = maximum_derivative[motor_index] /
                           (ARM_MOTION_DEG_PER_RPM_S * safe_limit);
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }

        safe_limit = (float)config->acc_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = sqrtf(maximum_acceleration[motor_index] /
                                 (ARM_MOTION_DEG_PER_RPM_S * safe_limit));
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }

        safe_limit = (float)config->dec_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        candidate_time_s = sqrtf(maximum_deceleration[motor_index] /
                                 (ARM_MOTION_DEG_PER_RPM_S * safe_limit));
        if (candidate_time_s > required_time_s)
        {
            required_time_s = candidate_time_s;
        }
    }

    result->minimum_time_s = required_time_s;
    result->failed_sample = 0U;
    context->final_joints = joints;
    context->final_coordinates = coordinates;
    if (trajectory->duration_s <
        (required_time_s - (float)ARM_MOTION_TIME_EPS_S))
    {
        return ARM_MOTION_TIME_UNACHIEVABLE;
    }
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_ValidateDiscreteTrajectory(
    const ArmMotionConfig *config,
    const ArmCubicBezierTrajectory *trajectory)
{
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmMotionTrajectoryResult *result = &context->result;
    ArmJointAngles joints;
    ArmMotorCoordinates previous_coordinates;
    ArmMotorCoordinates coordinates;
    ArmTrajectoryStatus trajectory_status = ARM_TRAJECTORY_OK;
    ArmIkStatus ik_status = ARM_IK_OK;
    ArmMotionStatus motion_status;
    float previous_command_rpm[ARM_MOTION_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float maximum_speed[ARM_MOTION_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    float maximum_acceleration[ARM_MOTION_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float maximum_deceleration[ARM_MOTION_MOTOR_COUNT] =
        {0.0f, 0.0f, 0.0f};
    float previous_elapsed_s;
    float elapsed_s;
    float interval_s;
    float average_rpm;
    float endpoint_rpm;
    float change_rpm_per_s;
    float previous_magnitude;
    float endpoint_magnitude;
    float safe_limit;
    float ratio;
    float candidate_time_s;
    uint32_t elapsed_ms;
    uint32_t sample_index;
    uint8_t motor_index;
    bool discrete_direction_invalid = false;
    bool discrete_limits_invalid = false;

    motion_status = ArmMotion_SolveTrajectorySample(
        config,
        trajectory,
        0.0f,
        &joints,
        &previous_coordinates,
        &trajectory_status,
        &ik_status);
    if (motion_status != ARM_MOTION_OK)
    {
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        return motion_status;
    }

    previous_elapsed_s = 0.0f;
    for (sample_index = 1U;
         sample_index <= trajectory->segment_count;
         ++sample_index)
    {
        elapsed_ms = sample_index *
                     (uint32_t)trajectory->sample_period_ms;
        if (elapsed_ms > trajectory->duration_ms)
        {
            elapsed_ms = trajectory->duration_ms;
        }
        elapsed_s = (float)elapsed_ms * 0.001f;
        interval_s = elapsed_s - previous_elapsed_s;
        if (!isfinite(interval_s) || (interval_s <= 0.0f))
        {
            result->failed_sample = sample_index;
            return ARM_MOTION_TIME_INVALID;
        }

        motion_status = ArmMotion_SolveTrajectorySample(
            config,
            trajectory,
            elapsed_s,
            &joints,
            &coordinates,
            &trajectory_status,
            &ik_status);
        result->trajectory_status = trajectory_status;
        result->ik_status = ik_status;
        result->failed_sample = sample_index;
        if (motion_status != ARM_MOTION_OK)
        {
            return motion_status;
        }

        for (motor_index = 0U;
             motor_index < ARM_MOTION_MOTOR_COUNT;
             ++motor_index)
        {
            average_rpm =
                (coordinates.degree[motor_index] -
                 previous_coordinates.degree[motor_index]) /
                ((float)ARM_MOTION_DEG_PER_RPM_S * interval_s);
            endpoint_rpm = (2.0f * average_rpm) -
                           previous_command_rpm[motor_index];
            if (!isfinite(endpoint_rpm))
            {
                return ARM_MOTION_TRAJECTORY_ERROR;
            }

            if (((context->planned_direction[motor_index] > 0) &&
                 (endpoint_rpm < -ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM)) ||
                ((context->planned_direction[motor_index] < 0) &&
                 (endpoint_rpm > ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM)))
            {
                /*
                 * 真实路径换向已由连续预检拒绝。此处的反号是离散
                 * 末速递推在当前时间网格上不可实现，可通过增加 T 解决。
                 */
                discrete_direction_invalid = true;
            }

            previous_magnitude =
                fabsf(previous_command_rpm[motor_index]);
            endpoint_magnitude = fabsf(endpoint_rpm);
            if (endpoint_magnitude > maximum_speed[motor_index])
            {
                maximum_speed[motor_index] = endpoint_magnitude;
            }
            change_rpm_per_s = fabsf(
                endpoint_rpm - previous_command_rpm[motor_index]) /
                interval_s;
            if (endpoint_magnitude >= previous_magnitude)
            {
                if (change_rpm_per_s >
                    maximum_acceleration[motor_index])
                {
                    maximum_acceleration[motor_index] =
                        change_rpm_per_s;
                }
            }
            else if (change_rpm_per_s >
                     maximum_deceleration[motor_index])
            {
                maximum_deceleration[motor_index] = change_rpm_per_s;
            }
            previous_command_rpm[motor_index] = endpoint_rpm;
        }

        previous_coordinates = coordinates;
        previous_elapsed_s = elapsed_s;
    }

    for (motor_index = 0U;
         motor_index < ARM_MOTION_MOTOR_COUNT;
         ++motor_index)
    {
        result->final_speed_rpm[motor_index] =
            previous_command_rpm[motor_index];
        if (maximum_speed[motor_index] >
            result->max_speed_rpm[motor_index])
        {
            result->max_speed_rpm[motor_index] =
                maximum_speed[motor_index];
        }
        if (maximum_acceleration[motor_index] >
            result->max_acc_rpm_per_s[motor_index])
        {
            result->max_acc_rpm_per_s[motor_index] =
                maximum_acceleration[motor_index];
        }
        if (maximum_deceleration[motor_index] >
            result->max_dec_rpm_per_s[motor_index])
        {
            result->max_dec_rpm_per_s[motor_index] =
                maximum_deceleration[motor_index];
        }

        safe_limit = config->vel_rpm[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_speed[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * ratio;
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        safe_limit = (float)config->acc_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_acceleration[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * sqrtf(ratio);
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        safe_limit = (float)config->dec_rpm_per_s[motor_index] *
                     config->trajectory_limit_scale;
        ratio = maximum_deceleration[motor_index] / safe_limit;
        candidate_time_s = trajectory->duration_s * sqrtf(ratio);
        if (candidate_time_s > result->minimum_time_s)
        {
            result->minimum_time_s = candidate_time_s;
        }

        if ((maximum_speed[motor_index] >
             (config->vel_rpm[motor_index] *
               config->trajectory_limit_scale)) ||
            (maximum_acceleration[motor_index] >
             ((float)config->acc_rpm_per_s[motor_index] *
              config->trajectory_limit_scale)) ||
            (maximum_deceleration[motor_index] >
             ((float)config->dec_rpm_per_s[motor_index] *
              config->trajectory_limit_scale)) ||
            (fabsf(previous_command_rpm[motor_index]) >
             ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM))
        {
            discrete_limits_invalid = true;
        }
    }

    if (discrete_direction_invalid || discrete_limits_invalid)
    {
        return ARM_MOTION_TIME_UNACHIEVABLE;
    }

    result->failed_sample = 0U;
    return ARM_MOTION_OK;
}

static int ArmMotion_CandidateDurationMs(double time_s,
                                         uint16_t period_ms,
                                         uint32_t maximum_ms,
                                         uint32_t *duration_ms)
{
    double value_ms;
    uint32_t rounded_ms;
    uint32_t remainder;
    uint32_t increment;

    if ((duration_ms == NULL) || !isfinite(time_s) || (time_s <= 0.0))
    {
        return 0;
    }

    value_ms = ceil(time_s * 1000.0 * ARM_MOTION_MINIMUM_VERIFY_MARGIN);
    if ((value_ms < 1.0) || (value_ms > (double)maximum_ms))
    {
        return 0;
    }
    rounded_ms = (uint32_t)value_ms;
    remainder = rounded_ms % (uint32_t)period_ms;
    if (remainder != 0U)
    {
        increment = (uint32_t)period_ms - remainder;
        if (rounded_ms > (maximum_ms - increment))
        {
            return 0;
        }
        rounded_ms += increment;
    }
    *duration_ms = rounded_ms;
    return 1;
}

/*
 * 不枚举 A/D/V，也不逐毫秒搜索。由解析缩放给出候选，再最多复核三次；
 * minimum_time_s 只记录已经在相同离散执行网格上实际验证通过的时间。
 */
static ArmMotionStatus ArmMotion_CertifyMinimumTime(
    const ArmMotionConfig *config,
    const ArmCubicBezierTrajectory *requested_trajectory,
    bool requested_time_passed)
{
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmMotionTrajectoryResult requested_result = context->result;
    ArmMotionTrajectoryResult verification_result;
    ArmCubicBezierTrajectory candidate_trajectory;
    ArmTrajectoryStatus trajectory_status;
    ArmMotionStatus motion_status;
    double candidate_time_s;
    double factor;
    double ratio;
    double safe_limit;
    uint32_t candidate_ms;
    uint32_t maximum_ms;
    uint8_t iteration;
    uint8_t motor_index;

    maximum_ms = (uint32_t)requested_trajectory->sample_period_ms *
                 (uint32_t)ARM_MOTION_TRAJECTORY_MAX_SEGMENTS;
    candidate_time_s = (double)requested_result.minimum_time_s;
    if (!requested_time_passed &&
        (candidate_time_s < (double)requested_trajectory->duration_s))
    {
        candidate_time_s = (double)requested_trajectory->duration_s;
    }
    if (candidate_time_s <= 0.0)
    {
        candidate_time_s =
            (double)requested_trajectory->sample_period_ms * 0.001;
    }

    for (iteration = 0U;
         iteration < ARM_MOTION_MINIMUM_VERIFY_ITERATIONS;
         ++iteration)
    {
        if (!ArmMotion_CandidateDurationMs(
                candidate_time_s,
                requested_trajectory->sample_period_ms,
                maximum_ms,
                &candidate_ms))
        {
            break;
        }
        if (requested_time_passed &&
            (candidate_ms > requested_trajectory->duration_ms))
        {
            candidate_ms = requested_trajectory->duration_ms;
        }

        trajectory_status = ArmTrajectory_InitCubicBezier(
            &candidate_trajectory,
            &requested_trajectory->point[0],
            &requested_trajectory->point[1],
            &requested_trajectory->point[2],
            &requested_trajectory->point[3],
            (float)candidate_ms * 0.001f,
            requested_trajectory->sample_period_ms);
        if (trajectory_status != ARM_TRAJECTORY_OK)
        {
            context->result = requested_result;
            if (requested_time_passed)
            {
                context->result.minimum_time_s =
                    requested_trajectory->duration_s;
                return ARM_MOTION_OK;
            }
            context->result.minimum_time_s = 0.0f;
            return ARM_MOTION_TIME_INVALID;
        }

        verification_result = requested_result;
        memset(verification_result.max_speed_rpm,
               0,
               sizeof(verification_result.max_speed_rpm));
        memset(verification_result.max_acc_rpm_per_s,
               0,
               sizeof(verification_result.max_acc_rpm_per_s));
        memset(verification_result.max_dec_rpm_per_s,
               0,
               sizeof(verification_result.max_dec_rpm_per_s));
        memset(verification_result.final_speed_rpm,
               0,
               sizeof(verification_result.final_speed_rpm));
        verification_result.minimum_time_s = requested_result.minimum_time_s;
        context->result = verification_result;
        motion_status = ArmMotion_ValidateDiscreteTrajectory(
            config, &candidate_trajectory);
        verification_result = context->result;
        if (motion_status == ARM_MOTION_OK)
        {
            context->result = requested_result;
            context->result.minimum_time_s =
                candidate_trajectory.duration_s;
            return ARM_MOTION_OK;
        }
        if (motion_status != ARM_MOTION_TIME_UNACHIEVABLE)
        {
            context->result = requested_result;
            if (requested_time_passed)
            {
                context->result.minimum_time_s =
                    requested_trajectory->duration_s;
                return ARM_MOTION_OK;
            }
            context->result.trajectory_status =
                verification_result.trajectory_status;
            context->result.ik_status = verification_result.ik_status;
            context->result.failed_sample = verification_result.failed_sample;
            context->result.minimum_time_s = 0.0f;
            return motion_status;
        }

        factor = 1.0;
        for (motor_index = 0U;
             motor_index < ARM_MOTION_MOTOR_COUNT;
             ++motor_index)
        {
            safe_limit = (double)config->vel_rpm[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = (double)verification_result.max_speed_rpm[motor_index] /
                    safe_limit;
            if (ratio > factor) factor = ratio;

            safe_limit = (double)config->acc_rpm_per_s[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = sqrt((double)verification_result.
                         max_acc_rpm_per_s[motor_index] / safe_limit);
            if (ratio > factor) factor = ratio;

            safe_limit = (double)config->dec_rpm_per_s[motor_index] *
                         (double)config->trajectory_limit_scale;
            ratio = sqrt((double)verification_result.
                         max_dec_rpm_per_s[motor_index] / safe_limit);
            if (ratio > factor) factor = ratio;

            ratio = fabs((double)verification_result.
                         final_speed_rpm[motor_index]) /
                    (double)ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM;
            if (ratio > factor) factor = ratio;
        }
        candidate_time_s = (double)candidate_trajectory.duration_s * factor;
    }

    if (requested_time_passed)
    {
        /* 当前用户时间刚刚已由同一离散校验通过，本身就是可靠上界。 */
        context->result = requested_result;
        context->result.minimum_time_s = requested_trajectory->duration_s;
        return ARM_MOTION_OK;
    }

    /* 三次解析修正仍失败时，仅复核一次执行器允许的最大时间。 */
    trajectory_status = ArmTrajectory_InitCubicBezier(
        &candidate_trajectory,
        &requested_trajectory->point[0],
        &requested_trajectory->point[1],
        &requested_trajectory->point[2],
        &requested_trajectory->point[3],
        (float)maximum_ms * 0.001f,
        requested_trajectory->sample_period_ms);
    context->result = requested_result;
    memset(context->result.max_speed_rpm,
           0,
           sizeof(context->result.max_speed_rpm));
    memset(context->result.max_acc_rpm_per_s,
           0,
           sizeof(context->result.max_acc_rpm_per_s));
    memset(context->result.max_dec_rpm_per_s,
           0,
           sizeof(context->result.max_dec_rpm_per_s));
    memset(context->result.final_speed_rpm,
           0,
           sizeof(context->result.final_speed_rpm));
    if (trajectory_status == ARM_TRAJECTORY_OK)
    {
        motion_status = ArmMotion_ValidateDiscreteTrajectory(
            config, &candidate_trajectory);
        verification_result = context->result;
    }
    else
    {
        motion_status = ARM_MOTION_TIME_INVALID;
        verification_result = requested_result;
        verification_result.trajectory_status = trajectory_status;
        verification_result.failed_sample = 0U;
    }
    context->result = requested_result;
    if (motion_status == ARM_MOTION_OK)
    {
        context->result.minimum_time_s = candidate_trajectory.duration_s;
        return ARM_MOTION_OK;
    }
    context->result.minimum_time_s = 0.0f;
    if (motion_status != ARM_MOTION_TIME_UNACHIEVABLE)
    {
        context->result.trajectory_status =
            verification_result.trajectory_status;
        context->result.ik_status = verification_result.ik_status;
        context->result.failed_sample = verification_result.failed_sample;
    }
    return (motion_status == ARM_MOTION_TIME_UNACHIEVABLE) ?
           ARM_MOTION_TIME_UNACHIEVABLE : motion_status;
}

static ArmMotionStatus ArmMotion_SetTimedReturnInfoUnlocked(
    const ArmMotionConfig *config,
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

static ArmMotionStatus ArmMotion_SetTrajectoryFeedback(
    const ArmMotionConfig *config,
    bool enable,
    uint32_t expected_generation,
    bool enforce_generation)
{
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmMotionStatus result = ARM_MOTION_OK;
    uint16_t position_period = enable ?
        ARM_MOTION_TRAJECTORY_POSITION_PERIOD_MS : 0U;
    uint16_t speed_period = enable ?
        ARM_MOTION_TRAJECTORY_SPEED_PERIOD_MS : 0U;
    uint16_t status_period = enable ?
        ARM_MOTION_TRAJECTORY_STATUS_PERIOD_MS : 0U;
    uint16_t error_period = enable ?
        ARM_MOTION_TRAJECTORY_ERROR_PERIOD_MS : 0U;
    uint32_t deadline_ms;
    uint8_t drain_index;
    uint8_t motor_index;
    bool all_acknowledged;

    if (!ArmMotion_FeedbackConfigLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    if (enable && enforce_generation &&
        !ArmMotion_CommandTokenIsCurrent(expected_generation))
    {
        ArmMotion_FeedbackConfigUnlock();
        return ARM_MOTION_CANCELLED;
    }
    if (!enable && !context->feedback_stream_enabled)
    {
        ArmMotion_FeedbackConfigUnlock();
        return ARM_MOTION_OK;
    }
    if (enable)
    {
        /* 部分发送失败时也保留 true，强制 Stop cleanup 再发 period=0。 */
        context->feedback_stream_enabled = true;
    }

    if (!enable)
    {
        /* 先解析现有队列，不能让 flush 隐藏已经到达的驱动器故障帧。 */
        for (drain_index = 0U; drain_index < 3U; ++drain_index)
        {
            ArmMotion_DrainRx(config, 0U);
        }
        (void)ZDT_CAN_FlushRx();
        memset(context->timed_stop_ack_count,
               0,
               sizeof(context->timed_stop_ack_count));
        context->timed_stop_request_ms = HAL_GetTick();
        context->timed_stop_waiting = false;
        osDelay(1U);
        for (drain_index = 0U; drain_index < 3U; ++drain_index)
        {
            ArmMotion_DrainRx(config, 0U);
        }
        (void)ZDT_CAN_FlushRx();
        context->timed_stop_waiting = true;
    }

    if (ArmMotion_SetTimedReturnInfoUnlocked(
            config,
            X_V2_INFO_REALTIME_POSITION,
            position_period) != ARM_MOTION_OK)
    {
        context->timed_stop_waiting = false;
        result = ARM_MOTION_CAN_ERROR;
        goto done;
    }
    if (ArmMotion_SetTimedReturnInfoUnlocked(
            config,
            X_V2_INFO_REALTIME_SPEED,
            speed_period) != ARM_MOTION_OK)
    {
        context->timed_stop_waiting = false;
        result = ARM_MOTION_CAN_ERROR;
        goto done;
    }
    if (ArmMotion_SetTimedReturnInfoUnlocked(
            config,
            X_V2_INFO_STATUS,
            status_period) != ARM_MOTION_OK)
    {
        context->timed_stop_waiting = false;
        result = ARM_MOTION_CAN_ERROR;
        goto done;
    }
    if (ArmMotion_SetTimedReturnInfoUnlocked(
            config,
            X_V2_INFO_POSITION_ERROR,
            error_period) != ARM_MOTION_OK)
    {
        context->timed_stop_waiting = false;
        result = ARM_MOTION_CAN_ERROR;
        goto done;
    }

    if (enable)
    {
        if (enforce_generation &&
            !ArmMotion_CommandTokenIsCurrent(expected_generation))
        {
            result = ARM_MOTION_CANCELLED;
        }
        goto done;
    }

    /*
     * 手册 5.5.1：period=0 时每条停止命令返回 Addr 11 02 6B。
     * 共有四类信息、三台电机；12 个确认全部收到才证明定时流已停。
     */
    deadline_ms = HAL_GetTick() + ARM_MOTION_TIMED_STOP_ACK_TIMEOUT_MS;
    for (;;)
    {
        ArmMotion_DrainRx(config, 5U);
        all_acknowledged = true;
        for (motor_index = 0U;
             motor_index < ARM_MOTION_MOTOR_COUNT;
             ++motor_index)
        {
            if (context->timed_stop_ack_count[motor_index] <
                ARM_MOTION_TIMED_STOP_ACKS_PER_MOTOR)
            {
                all_acknowledged = false;
                break;
            }
        }
        if (all_acknowledged)
        {
            /* 继续清空剩余队列，确保停止前已到达的故障帧不会被忽略。 */
            for (drain_index = 0U; drain_index < 3U; ++drain_index)
            {
                ArmMotion_DrainRx(config, 0U);
            }
            context->timed_stop_waiting = false;
            context->feedback_stream_enabled = false;
            result = ARM_MOTION_OK;
            goto done;
        }
        if (ArmMotion_TimeReached(HAL_GetTick(), deadline_ms))
        {
            context->timed_stop_waiting = false;
            result = ARM_MOTION_FEEDBACK_TIMEOUT;
            goto done;
        }
    }

done:
    ArmMotion_FeedbackConfigUnlock();
    return result;
}

static ArmMotionStatus ArmMotion_SetTrajectoryHeartbeat(
    const ArmMotionConfig *config,
    uint32_t timeout_ms,
    uint32_t expected_generation,
    bool enforce_generation)
{
    ArmMotionStatus result = ARM_MOTION_OK;
    uint8_t index;

    if (!ArmMotion_FeedbackConfigLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    if ((timeout_ms != 0U) && enforce_generation &&
        !ArmMotion_CommandTokenIsCurrent(expected_generation))
    {
        ArmMotion_FeedbackConfigUnlock();
        return ARM_MOTION_CANCELLED;
    }
    if ((timeout_ms == 0U) &&
        !arm_motion_trajectory_context.heartbeat_enabled &&
        !arm_motion_trajectory_context.heartbeat_disable_pending)
    {
        ArmMotion_FeedbackConfigUnlock();
        return ARM_MOTION_OK;
    }
    if (timeout_ms != 0U)
    {
        /* 部分发送失败时保留 true，Stop cleanup 必须重试 timeout=0。 */
        arm_motion_trajectory_context.heartbeat_enabled = true;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_Set_Heartbeat(config->motor_addr[index],
                               false,
                               timeout_ms) != HAL_OK)
        {
            result = ARM_MOTION_CAN_ERROR;
            break;
        }
    }
    if ((result == ARM_MOTION_OK) && (timeout_ms != 0U) &&
        enforce_generation &&
        !ArmMotion_CommandTokenIsCurrent(expected_generation))
    {
        result = ARM_MOTION_CANCELLED;
    }
    ArmMotion_FeedbackConfigUnlock();
    return result;
}

static ArmMotionStatus ArmMotion_TrajectoryBestEffortCleanup(
    const ArmMotionConfig *config)
{
    ArmMotionStatus cleanup_status = ARM_MOTION_OK;

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }

    if (ArmMotion_SetTrajectoryHeartbeat(
            config, 0U, 0U, false) == ARM_MOTION_OK)
    {
        arm_motion_trajectory_context.heartbeat_enabled = false;
        arm_motion_trajectory_context.heartbeat_disable_pending = false;
    }
    else
    {
        cleanup_status = ARM_MOTION_CAN_ERROR;
    }
    if (ArmMotion_SetTrajectoryFeedback(
            config, false, 0U, false) == ARM_MOTION_OK)
    {
        arm_motion_trajectory_context.feedback_stream_enabled = false;
    }
    else
    {
        cleanup_status = ARM_MOTION_CAN_ERROR;
    }
    return cleanup_status;
}

static ArmMotionStatus ArmMotion_TrajectoryFault(
    ArmMotionStatus cause)
{
    ArmMotionStatus stop_status;
    ArmMotionConfig *config = arm_motion_active_config;

    if (config != NULL)
    {
        stop_status = ArmMotion_StopAll(config);
        if ((stop_status != ARM_MOTION_OK) &&
            (cause != ARM_MOTION_CAN_ERROR))
        {
            cause = ARM_MOTION_CAN_ERROR;
        }
    }

    arm_motion_trajectory_context.result.state =
        ARM_MOTION_TRAJECTORY_FAULT;
    arm_motion_trajectory_context.result.status = cause;
    arm_motion_current_valid = false;
    return cause;
}

static bool ArmMotion_AllPositionFeedbackFresh(
    const ArmMotionConfig *config,
    uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].realtime_position_valid ||
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        realtime_position_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

static bool ArmMotion_AllPositionFeedbackFreshSince(
    const ArmMotionConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    if (!ArmMotion_AllPositionFeedbackFresh(config, now_ms))
    {
        return false;
    }
    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!ArmMotion_TimeAfter(
                arm_motion_driver_status[index].realtime_position_update_ms,
                not_before_ms))
        {
            return false;
        }
    }
    return true;
}

static bool ArmMotion_AllMotorsStopped(
    const ArmMotionConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].realtime_speed_valid ||
            !ArmMotion_TimeAfter(
                arm_motion_driver_status[index].realtime_speed_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        realtime_speed_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms) ||
            (fabsf(arm_motion_driver_status[index].realtime_speed_rpm) >
             ARM_MOTION_TRAJECTORY_SPEED_ZERO_RPM))
        {
            return false;
        }
    }
    return true;
}

static bool ArmMotion_AllSpeedFeedbackFreshSince(
    const ArmMotionConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].realtime_speed_valid ||
            !ArmMotion_TimeAfter(
                arm_motion_driver_status[index].realtime_speed_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        realtime_speed_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

static bool ArmMotion_AllStatusFeedbackFreshSince(
    const ArmMotionConfig *config,
    uint32_t now_ms,
    uint32_t not_before_ms)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (!arm_motion_driver_status[index].status_valid ||
            !ArmMotion_TimeAfter(
                arm_motion_driver_status[index].status_update_ms,
                not_before_ms) ||
            ((uint32_t)(now_ms -
                        arm_motion_driver_status[index].status_update_ms) >
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            return false;
        }
    }
    return true;
}

ArmMotionStatus ArmMotion_RecoverAfterTimeout(const ArmMotionConfig *config)
{
    ArmMotionStatus status = ARM_MOTION_FEEDBACK_TIMEOUT;
    ArmCartesianPosition endpoint;
    uint32_t request_generation;
    uint32_t start_ms;
    uint32_t barrier_ms;
    uint32_t next_poll_ms;
    uint32_t now_ms;
    uint8_t index;
    uint8_t recovery_phase = 0U;
    bool query_failed = false;
    bool status_fault;

    if (config == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_TryCaptureCommandToken(&request_generation))
    {
        status = ARM_MOTION_CANCELLED;
        goto done;
    }
    if (!ArmMotion_CommandLock())
    {
        status = ARM_MOTION_CAN_ERROR;
        goto done;
    }
    if ((arm_motion_active_config != config) ||
        ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
    {
        ArmMotion_CommandUnlock();
        status = ARM_MOTION_BUSY;
        goto done;
    }
    /* 只有停稳后的实测位置才能重新成为定时绝对位置运动的起点。 */
    arm_motion_current_valid = false;
    ArmMotion_CommandUnlock();

    /* 清理停机前的旧帧，用接收时间建立反馈屏障。 */
    ArmMotion_DrainRx(config, 0U);
    ArmMotion_DrainRx(config, 0U);
    start_ms = HAL_GetTick();
    barrier_ms = start_ms;
    next_poll_ms = start_ms + 1U;

    for (;;)
    {
        if (!ArmMotion_CommandTokenIsCurrent(request_generation))
        {
            status = ARM_MOTION_CANCELLED;
            break;
        }

        ArmMotion_DrainRx(config, 10U);
        now_ms = HAL_GetTick();
        status_fault = false;
        for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
        {
            const ArmMotionDriverStatus *driver =
                &arm_motion_driver_status[index];
            if (driver->status_valid &&
                ArmMotion_TimeAfter(driver->status_update_ms, start_ms) &&
                (((driver->status_flags & ARM_MOTION_DRIVER_FAULT_MASK) != 0U) ||
                 ((driver->status_flags & 0x01U) == 0U)))
            {
                status_fault = true;
            }
        }
        if (status_fault)
        {
            status = ARM_MOTION_DRIVER_ERROR;
            break;
        }

        if ((recovery_phase == 0U) &&
            ArmMotion_AllSpeedFeedbackFreshSince(config, now_ms, barrier_ms) &&
            ArmMotion_AllStatusFeedbackFreshSince(config, now_ms, barrier_ms) &&
            ArmMotion_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            /* 首次读到零速后，再隔一段时间确认一次，防止瞬态零速。 */
            recovery_phase = 1U;
            barrier_ms = now_ms;
            next_poll_ms = now_ms + ARM_MOTION_TIMEOUT_RECOVERY_STABLE_MS;
        }
        else if ((recovery_phase == 1U) &&
                 ((uint32_t)(now_ms - barrier_ms) >=
                  ARM_MOTION_TIMEOUT_RECOVERY_STABLE_MS) &&
                 ArmMotion_AllSpeedFeedbackFreshSince(config, now_ms,
                                                      barrier_ms) &&
                 ArmMotion_AllStatusFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmMotion_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            /* 35 两次确认停稳后，才建立 36 的新位置屏障。 */
            recovery_phase = 2U;
            barrier_ms = now_ms;
            next_poll_ms = now_ms + 1U;
        }
        else if ((recovery_phase == 2U) &&
                 ArmMotion_AllSpeedFeedbackFreshSince(config, now_ms,
                                                      barrier_ms) &&
                 ArmMotion_AllStatusFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmMotion_AllPositionFeedbackFreshSince(
                     config, now_ms, barrier_ms) &&
                 ArmMotion_AllMotorsStopped(config, now_ms, barrier_ms))
        {
            if (!ArmMotion_CommandLock())
            {
                status = ARM_MOTION_CAN_ERROR;
                break;
            }
            if (!ArmMotion_CommandTokenIsCurrent(request_generation) ||
                (arm_motion_active_config != config) ||
                ArmMotion_TrajectoryIsBusy() || arm_motion_move_tracker.active)
            {
                status = ARM_MOTION_CANCELLED;
            }
            else
            {
                status = ArmMotion_SyncCurrentFromFeedback(config, now_ms);
                if (status == ARM_MOTION_OK)
                {
                    /* 实际关节越界时，不允许从该姿态发起下一条 XYZ。 */
                    status = ArmMotion_CurrentEndpoint(config, &endpoint);
                }
            }
            if (status != ARM_MOTION_OK)
            {
                arm_motion_current_valid = false;
            }
            ArmMotion_CommandUnlock();
            break;
        }

        if ((uint32_t)(now_ms - start_ms) >=
            ARM_MOTION_TIMEOUT_RECOVERY_MS)
        {
            status = query_failed ? ARM_MOTION_CAN_ERROR :
                                    ARM_MOTION_FEEDBACK_TIMEOUT;
            break;
        }

        if (ArmMotion_TimeReached(now_ms, next_poll_ms))
        {
            for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
            {
                if (X_V2_Read_Info(config->motor_addr[index],
                                   X_V2_INFO_REALTIME_SPEED) != HAL_OK)
                {
                    query_failed = true;
                }
                if (X_V2_Read_Info(config->motor_addr[index],
                                   X_V2_INFO_STATUS) != HAL_OK)
                {
                    query_failed = true;
                }
                if ((recovery_phase == 2U) &&
                    (X_V2_Read_Info(config->motor_addr[index],
                                    X_V2_INFO_REALTIME_POSITION) != HAL_OK))
                {
                    query_failed = true;
                }
            }
            next_poll_ms = HAL_GetTick() +
                           ARM_MOTION_TIMEOUT_RECOVERY_POLL_MS;
        }
        osDelay(5U);
    }

done:
    ArmMotion_TrajectoryApiUnlock();
    return status;
}

static uint32_t ArmMotion_TrajectoryStopTimeoutMs(
    const ArmMotionConfig *config)
{
    double maximum_time_s = 0.0;
    double axis_time_s;
    uint32_t timeout_ms;
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        axis_time_s = fabs((double)arm_motion_trajectory_context.
                           previous_command_rpm[index]) /
                      (double)config->dec_rpm_per_s[index];
        if (axis_time_s > maximum_time_s)
        {
            maximum_time_s = axis_time_s;
        }
    }

    timeout_ms = (uint32_t)ceil(maximum_time_s * 1000.0);
    timeout_ms += (uint32_t)ARM_MOTION_TRAJECTORY_SPEED_PERIOD_MS;
    timeout_ms += (uint32_t)config->trajectory_feedback_timeout_ms;
    return timeout_ms;
}

static ArmMotionStatus ArmMotion_RequestTrajectoryPosition(
    const ArmMotionConfig *config)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_Read_Info(config->motor_addr[index],
                           X_V2_INFO_REALTIME_POSITION) != HAL_OK)
        {
            return ARM_MOTION_CAN_ERROR;
        }
    }
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_RequestTrajectoryStatus(
    const ArmMotionConfig *config)
{
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if (X_V2_Read_Info(config->motor_addr[index],
                           X_V2_INFO_STATUS) != HAL_OK)
        {
            return ARM_MOTION_CAN_ERROR;
        }
    }
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_SendTrajectoryVelocity(
    const ArmMotionConfig *config,
    const ArmMotorCoordinates *reference,
    const ArmMotorCoordinates *next_reference,
    float interval_s,
    uint32_t now_ms)
{
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    X_V2_Vel_Command_t commands[ARM_MOTION_MOTOR_COUNT];
    uint8_t index;
    float actual_degrees;
    float estimated_degrees;
    float age_s;
    float extrapolation_rpm;
    float position_error;
    float correction_rpm;
    float feedforward_rpm;
    float desired_rpm;
    float desired_magnitude;
    float previous_magnitude;
    float allowed_lower_rpm;
    float allowed_upper_rpm;
    float quantized_rpm;
    float acceleration_rpm_per_s;
    uint16_t change_limit;

    if ((config == NULL) || (reference == NULL) ||
        (next_reference == NULL) || !isfinite(interval_s) ||
        (interval_s <= 0.0f))
    {
        return ARM_MOTION_NULL_POINTER;
    }

    if (!ArmMotion_AllPositionFeedbackFresh(config, now_ms))
    {
        return ARM_MOTION_FEEDBACK_TIMEOUT;
    }

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        actual_degrees = ArmMotion_LogicalPositionDegrees(
            config,
            index,
            arm_motion_driver_status[index].realtime_position_deg);
        age_s = (float)(now_ms - arm_motion_driver_status[index].
                        realtime_position_update_ms) * 0.001f;
        extrapolation_rpm = context->previous_command_rpm[index];
        if (arm_motion_driver_status[index].realtime_speed_valid &&
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        realtime_speed_update_ms) <=
             (uint32_t)config->trajectory_feedback_timeout_ms))
        {
            extrapolation_rpm = ArmMotion_LogicalPositionDegrees(
                config,
                index,
                arm_motion_driver_status[index].realtime_speed_rpm);
        }
        estimated_degrees = actual_degrees +
            (extrapolation_rpm *
             (float)ARM_MOTION_DEG_PER_RPM_S * age_s);
        position_error = reference->degree[index] - estimated_degrees;
        if (!isfinite(position_error) ||
            (fabsf(position_error) >
             config->trajectory_feedback_error_limit_deg[index]))
        {
            return ARM_MOTION_DRIVER_ERROR;
        }

        if (arm_motion_driver_status[index].position_error_valid &&
            ((uint32_t)(now_ms - arm_motion_driver_status[index].
                        position_error_update_ms) <=
             (uint32_t)config->trajectory_feedback_timeout_ms) &&
            (fabsf(arm_motion_driver_status[index].position_error_deg) >
             config->trajectory_feedback_error_limit_deg[index]))
        {
            return ARM_MOTION_DRIVER_ERROR;
        }

        /*
         * F6 会按 acc 从上一速度线性爬升到新速度。令区间末速
         * v1=2*delta/dt-v0，使 (v0+v1)/2 的积分正好等于本段位移。
         */
        feedforward_rpm =
            (2.0f *
             (next_reference->degree[index] - reference->degree[index]) /
             ((float)ARM_MOTION_DEG_PER_RPM_S * interval_s)) -
            context->previous_command_rpm[index];
        correction_rpm = 2.0f *
                         config->trajectory_kp_rpm_per_deg[index] *
                         position_error;
        if (correction_rpm >
            config->trajectory_correction_limit_rpm[index])
        {
            correction_rpm =
                config->trajectory_correction_limit_rpm[index];
        }
        else if (correction_rpm <
                 -config->trajectory_correction_limit_rpm[index])
        {
            correction_rpm =
                -config->trajectory_correction_limit_rpm[index];
        }
        desired_rpm = feedforward_rpm + correction_rpm;

        /* 第一版轨迹不允许单轴中途换向，反馈修正也不能越过零速。 */
        if ((context->planned_direction[index] > 0) &&
            (desired_rpm < 0.0f))
        {
            desired_rpm = 0.0f;
        }
        else if ((context->planned_direction[index] < 0) &&
                 (desired_rpm > 0.0f))
        {
            desired_rpm = 0.0f;
        }
        else if (context->planned_direction[index] == 0)
        {
            desired_rpm = 0.0f;
        }

        previous_magnitude = fabsf(context->previous_command_rpm[index]);
        allowed_lower_rpm = previous_magnitude -
            ((float)config->dec_rpm_per_s[index] * interval_s);
        if (allowed_lower_rpm < 0.0f)
        {
            allowed_lower_rpm = 0.0f;
        }
        allowed_upper_rpm = previous_magnitude +
            ((float)config->acc_rpm_per_s[index] * interval_s);
        if (allowed_upper_rpm > config->vel_rpm[index])
        {
            allowed_upper_rpm = config->vel_rpm[index];
        }

        desired_magnitude = fabsf(desired_rpm);
        if (desired_magnitude < allowed_lower_rpm)
        {
            desired_magnitude = allowed_lower_rpm;
        }
        if (desired_magnitude > allowed_upper_rpm)
        {
            desired_magnitude = allowed_upper_rpm;
        }

        /*
         * 协议只能表示 0.1 RPM。量化后再次投影到本周期 A/D 可达区间；
         * 如果区间内没有新的 0.1 网格点，就保持上一条速度。
         */
        quantized_rpm = floorf((desired_magnitude * 10.0f) + 0.5f) /
                        10.0f;
        if (quantized_rpm >
            (allowed_upper_rpm +
             ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM))
        {
            quantized_rpm = floorf(
                (allowed_upper_rpm +
                 ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM) * 10.0f) /
                10.0f;
        }
        if (quantized_rpm <
            (allowed_lower_rpm -
             ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM))
        {
            quantized_rpm = ceilf(
                (allowed_lower_rpm -
                 ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM) * 10.0f) /
                10.0f;
        }
        if ((quantized_rpm <
             (allowed_lower_rpm -
              ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM)) ||
            (quantized_rpm >
             (allowed_upper_rpm +
              ARM_MOTION_TRAJECTORY_QUANTIZATION_EPS_RPM)))
        {
            quantized_rpm = previous_magnitude;
        }
        desired_magnitude = quantized_rpm;
        desired_rpm = (context->planned_direction[index] < 0) ?
                      -desired_magnitude : desired_magnitude;
        change_limit = (desired_magnitude >= previous_magnitude) ?
                       config->acc_rpm_per_s[index] :
                       config->dec_rpm_per_s[index];
        acceleration_rpm_per_s =
            fabsf(desired_magnitude - previous_magnitude) / interval_s;
        if (acceleration_rpm_per_s < 1.0f)
        {
            acceleration_rpm_per_s = 1.0f;
        }
        if (acceleration_rpm_per_s > (float)change_limit)
        {
            acceleration_rpm_per_s = (float)change_limit;
        }

        commands[index].addr = config->motor_addr[index];
        commands[index].dir = ArmMotion_VelocityDirection(
            config, index, desired_rpm);
        commands[index].acc_rpm_per_s =
            (uint16_t)ceilf(acceleration_rpm_per_s);
        commands[index].vel_rpm = desired_magnitude;
        commands[index].snF = false;
        context->previous_command_rpm[index] = desired_rpm;
    }

    if (X_V2_MultiMotor_Vel_Control(
            commands, ARM_MOTION_MOTOR_COUNT) != HAL_OK)
    {
        return ARM_MOTION_CAN_ERROR;
    }
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_SendTrajectoryZeroVelocity(
    const ArmMotionConfig *config)
{
    X_V2_Vel_Command_t commands[ARM_MOTION_MOTOR_COUNT];
    uint8_t index;

    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        commands[index].addr = config->motor_addr[index];
        commands[index].dir = ArmMotion_VelocityDirection(
            config,
            index,
            arm_motion_trajectory_context.previous_command_rpm[index]);
        commands[index].acc_rpm_per_s = config->dec_rpm_per_s[index];
        commands[index].vel_rpm = 0.0f;
        commands[index].snF = false;
    }

    if (X_V2_MultiMotor_Vel_Control(
            commands, ARM_MOTION_MOTOR_COUNT) != HAL_OK)
    {
        return ARM_MOTION_CAN_ERROR;
    }
    memset(arm_motion_trajectory_context.previous_command_rpm,
           0,
           sizeof(arm_motion_trajectory_context.previous_command_rpm));
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_BeginTrajectoryFinalPosition(
    ArmMotionConfig *config,
    uint32_t now_ms)
{
    ArmMotionStatus motion_status;
    double timeout_s;
    double timeout_ms_double;
    uint8_t index;

    motion_status = ArmMotion_SyncCurrentFromFeedback(config, now_ms);
    if (motion_status != ARM_MOTION_OK)
    {
        return motion_status;
    }

    motion_status = ArmMotion_SetTrajectoryFeedback(
        config, false, 0U, false);
    if (motion_status != ARM_MOTION_OK)
    {
        return motion_status;
    }
    arm_motion_trajectory_context.feedback_stream_enabled = false;
    if (arm_motion_trajectory_context.driver_error_mask != 0U)
    {
        return ARM_MOTION_DRIVER_ERROR;
    }
    if ((zdt_can_rx_drop_count !=
         arm_motion_trajectory_context.rx_drop_baseline) ||
        (zdt_can_error_count !=
         arm_motion_trajectory_context.can_error_baseline))
    {
        return ARM_MOTION_CAN_ERROR;
    }

    memset(&arm_motion_last_result, 0, sizeof(arm_motion_last_result));
    arm_motion_last_ik_status = ARM_IK_OK;
    if (!ArmMotion_CommandLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    if (!ArmMotion_CommandTokenIsCurrent(
            arm_motion_trajectory_context.generation))
    {
        ArmMotion_CommandUnlock();
        arm_motion_trajectory_context.result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    motion_status = ArmMotion_MoveToJointsInternal(
        config,
        &arm_motion_trajectory_context.final_joints,
        &arm_motion_last_result,
        &arm_motion_last_ik_status,
        config->trajectory_settle_time_s,
        arm_motion_trajectory_context.generation,
        true);
    if (motion_status != ARM_MOTION_OK)
    {
        ArmMotion_CommandUnlock();
        return motion_status;
    }

    motion_status = ArmMotion_CheckMoveCompletion(
        arm_motion_trajectory_context.generation, true);
    if (motion_status != ARM_MOTION_BUSY)
    {
        ArmMotion_CommandUnlock();
        if (motion_status == ARM_MOTION_CANCELLED)
        {
            arm_motion_trajectory_context.result.minimum_time_s = 0.0f;
        }
        return motion_status;
    }

    /* 定时反馈关闭和 FD 分包发送可能耗时，超时从发送完成起算。 */
    now_ms = HAL_GetTick();

    timeout_s = (double)config->trajectory_settle_time_s;
    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        if ((double)arm_motion_last_result.profile.predicted_time_s[index] >
            timeout_s)
        {
            timeout_s =
                (double)arm_motion_last_result.profile.predicted_time_s[index];
        }
    }
    timeout_ms_double = ceil(timeout_s * 1000.0) +
                        (double)ARM_MOTION_TRAJECTORY_SETTLE_MARGIN_MS;
    if ((timeout_ms_double <= 0.0) ||
        (timeout_ms_double > (double)ARM_MOTION_MAX_OS_DELAY_TICKS))
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_TIME_INVALID;
    }

    arm_motion_trajectory_context.settle_deadline_ms =
        now_ms + (uint32_t)timeout_ms_double;
    if (!ArmMotion_TryPublishTrajectoryState(
            arm_motion_trajectory_context.generation,
            ARM_MOTION_TRAJECTORY_SETTLING_POSITION,
            ARM_MOTION_BUSY))
    {
        ArmMotion_CommandUnlock();
        arm_motion_trajectory_context.result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    ArmMotion_BeginDiagnostics(now_ms);
    ArmMotion_CommandUnlock();
    return ARM_MOTION_BUSY;
}

static ArmMotionStatus ArmMotion_StartBezierInternal(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmMotionConfig *config = arm_motion_active_config;
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmCartesianPosition start;
    ArmTrajectoryStatus trajectory_status;
    ArmMotionStatus motion_status;
    ArmMotionStatus minimum_status;
    uint32_t now_ms;
    uint32_t duration_ms;
    uint32_t start_generation;
    uint8_t index;
    double duration_ms_value;

    if ((control1 == NULL) || (control2 == NULL) || (target == NULL))
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (config == NULL)
    {
        return ARM_MOTION_NOT_INITIALIZED;
    }
    if (ArmMotion_TrajectoryIsBusy())
    {
        return ARM_MOTION_BUSY;
    }
    if (arm_motion_move_tracker.active)
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        return ARM_MOTION_NOT_ZEROED;
    }
    if (!config->use_multi_command)
    {
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!isfinite(time_s) || (time_s <= 0.0f))
    {
        return ARM_MOTION_TIME_INVALID;
    }

    /* HAL/RTOS 的执行时基是 1 ms，规划时间统一量化到整数毫秒。 */
    duration_ms_value = floor(((double)time_s * 1000.0) + 0.5);
    if ((duration_ms_value < 1.0) ||
        (duration_ms_value > (double)UINT32_MAX))
    {
        return ARM_MOTION_TIME_INVALID;
    }
    duration_ms = (uint32_t)duration_ms_value;
    time_s = (float)duration_ms * 0.001f;

    /*
     * 先与普通运动/配置切换串行，再原子认领 PRECHECKING，避免双 Start、
     * Stop 或 SetActiveConfig 在入口检查后穿透。
     */
    if (!ArmMotion_CommandLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    if (arm_motion_active_config != config)
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_ConfigIsValid(config))
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_INVALID_CONFIG;
    }
    if (!config->zero_confirmed)
    {
        ArmMotion_CommandUnlock();
        return ARM_MOTION_NOT_ZEROED;
    }
    taskENTER_CRITICAL();
    if ((arm_motion_stop_depth != 0U) ||
        ArmMotion_TrajectoryStateIsBusy(context->result.state) ||
        arm_motion_move_tracker.active ||
        (arm_motion_active_config != config))
    {
        taskEXIT_CRITICAL();
        ArmMotion_CommandUnlock();
        return ARM_MOTION_BUSY;
    }
    start_generation = arm_motion_trajectory_generation + 1U;
    arm_motion_trajectory_generation = start_generation;
    memset(context, 0, sizeof(*context));
    context->result.state = ARM_MOTION_TRAJECTORY_PRECHECKING;
    context->result.status = ARM_MOTION_BUSY;
    context->result.requested_time_s = time_s;
    context->result.target = *target;
    context->generation = start_generation;
    taskEXIT_CRITICAL();
    ArmMotion_CommandUnlock();

    motion_status = ArmMotion_CurrentEndpoint(config, &start);
    if (!ArmMotion_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    if (motion_status != ARM_MOTION_OK)
    {
        context->result.state = ARM_MOTION_TRAJECTORY_FAULT;
        context->result.status = motion_status;
        return motion_status;
    }
    context->result.start = start;

    trajectory_status = ArmTrajectory_InitCubicBezier(
        &context->trajectory,
        &start,
        control1,
        control2,
        target,
        time_s,
        config->trajectory_period_ms);
    context->result.trajectory_status = trajectory_status;
    if (!ArmMotion_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    if (trajectory_status != ARM_TRAJECTORY_OK)
    {
        context->result.state = ARM_MOTION_TRAJECTORY_FAULT;
        context->result.status = ARM_MOTION_TRAJECTORY_ERROR;
        return ARM_MOTION_TRAJECTORY_ERROR;
    }

    /* 第一次预检使用最近一次已确认到位的位置，失败时不发送任何 CAN。 */
    motion_status = ArmMotion_PreflightTrajectory(config,
                                                   &context->trajectory);
    if (!ArmMotion_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    if (motion_status == ARM_MOTION_OK)
    {
        motion_status = ArmMotion_ValidateDiscreteTrajectory(
            config, &context->trajectory);
        if (!ArmMotion_CommandTokenIsCurrent(start_generation))
        {
            context->result.minimum_time_s = 0.0f;
            return ARM_MOTION_CANCELLED;
        }
    }
    if ((motion_status == ARM_MOTION_OK) ||
        (motion_status == ARM_MOTION_TIME_UNACHIEVABLE))
    {
        minimum_status = ArmMotion_CertifyMinimumTime(
            config,
            &context->trajectory,
            motion_status == ARM_MOTION_OK);
        if (minimum_status != ARM_MOTION_OK)
        {
            motion_status = minimum_status;
        }
        if (!ArmMotion_CommandTokenIsCurrent(start_generation))
        {
            context->result.minimum_time_s = 0.0f;
            return ARM_MOTION_CANCELLED;
        }
    }
    if ((motion_status != ARM_MOTION_OK) &&
        (motion_status != ARM_MOTION_TIME_UNACHIEVABLE))
    {
        context->result.minimum_time_s = 0.0f;
    }
    if (motion_status != ARM_MOTION_OK)
    {
        context->result.state = ARM_MOTION_TRAJECTORY_FAULT;
        context->result.status = motion_status;
        return motion_status;
    }
    if (!ArmMotion_CommandTokenIsCurrent(start_generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }

    (void)ZDT_CAN_FlushRx();
    memset(arm_motion_driver_status, 0, sizeof(arm_motion_driver_status));
    context->rx_drop_baseline = zdt_can_rx_drop_count;
    context->can_error_baseline = zdt_can_error_count;
    context->feedback_stream_enabled = true;
    motion_status = ArmMotion_SetTrajectoryFeedback(
        config, true, start_generation, true);
    if (motion_status != ARM_MOTION_OK)
    {
        return ArmMotion_TrajectoryFault(motion_status);
    }
    /* 丢弃启流发送期间到达的旧/首批帧，再建立严格的新反馈屏障。 */
    (void)ZDT_CAN_FlushRx();
    memset(arm_motion_driver_status, 0, sizeof(arm_motion_driver_status));
    context->feedback_request_ms = HAL_GetTick();
    osDelay(1U);
    if (ArmMotion_RequestTrajectoryStatus(config) != ARM_MOTION_OK)
    {
        return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
    }
    if (zdt_can_error_count != context->can_error_baseline)
    {
        return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
    }
    context->rx_drop_baseline = zdt_can_rx_drop_count;

    /* 心跳只覆盖 F6 流式阶段，二次预检完成后再启用。 */
    context->heartbeat_ack_mask = ARM_MOTION_ALL_MOTORS_MASK;
    context->trajectory_preflight_ready = false;

    now_ms = HAL_GetTick();
    context->feedback_deadline_ms =
        now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
    for (index = 0U; index < ARM_MOTION_MOTOR_COUNT; ++index)
    {
        context->previous_command_rpm[index] = 0.0f;
    }
    if (!ArmMotion_TryPublishTrajectoryState(
            start_generation,
            ARM_MOTION_TRAJECTORY_WAITING_FEEDBACK,
            ARM_MOTION_BUSY))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }
    return ARM_MOTION_OK;
}

static ArmMotionStatus ArmMotion_ServiceTrajectoryInternal(void)
{
    ArmMotionConfig *config = arm_motion_active_config;
    ArmMotionTrajectoryContext *context = &arm_motion_trajectory_context;
    ArmCartesianPosition start;
    ArmCartesianPosition control1;
    ArmCartesianPosition control2;
    ArmCartesianPosition target;
    ArmJointAngles joints;
    ArmMotorCoordinates next_coordinates;
    ArmTrajectoryStatus trajectory_status;
    ArmIkStatus ik_status;
    ArmMotionStatus motion_status;
    ArmMotionStatus minimum_status;
    uint32_t now_ms;
    uint32_t offset_ms;
    uint32_t elapsed_ms;
    uint32_t next_elapsed_ms;
    uint32_t stop_timeout_ms;
    float next_elapsed_s;
    float interval_s;

    if (config == NULL)
    {
        return ARM_MOTION_NOT_INITIALIZED;
    }
    if (!ArmMotion_TrajectoryStateIsBusy(context->result.state))
    {
        return context->result.status;
    }
    if (context->result.state == ARM_MOTION_TRAJECTORY_PRECHECKING)
    {
        return ARM_MOTION_BUSY;
    }
    if (!ArmMotion_CommandTokenIsCurrent(context->generation))
    {
        context->result.minimum_time_s = 0.0f;
        return ARM_MOTION_CANCELLED;
    }

    ArmMotion_DrainRx(config, 0U);
    now_ms = HAL_GetTick();
    if ((zdt_can_rx_drop_count != context->rx_drop_baseline) ||
        (zdt_can_error_count != context->can_error_baseline))
    {
        return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
    }
    if ((context->driver_error_mask != 0U) ||
        (arm_motion_move_tracker.error_mask != 0U))
    {
        return ArmMotion_TrajectoryFault(ARM_MOTION_DRIVER_ERROR);
    }

    if (context->result.state ==
        ARM_MOTION_TRAJECTORY_WAITING_FEEDBACK)
    {
        if (!context->trajectory_preflight_ready &&
            ArmMotion_AllPositionFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ArmMotion_AllMotorsStopped(
                config, now_ms, context->feedback_request_ms) &&
            ArmMotion_AllStatusFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms))
        {
            if (!ArmMotion_TryPublishTrajectoryState(
                    context->generation,
                    ARM_MOTION_TRAJECTORY_PRECHECKING,
                    ARM_MOTION_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            motion_status = ArmMotion_SyncCurrentFromFeedback(config, now_ms);
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            motion_status = ArmMotion_CurrentEndpoint(config, &start);
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }

            /*
             * 二次完整预检包含较多三角函数。先停止定时返回，避免长轨迹
             * 计算期间 35/36/37/3A 持续入队并挤满 CAN RX 队列。
             * 三轴此时已经由新的 35 确认停稳，因此暂停反馈不会改变 P0。
             */
            motion_status = ArmMotion_SetTrajectoryFeedback(
                config, false, 0U, false);
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            context->feedback_stream_enabled = false;
            if (context->driver_error_mask != 0U)
            {
                return ArmMotion_TrajectoryFault(
                    ARM_MOTION_DRIVER_ERROR);
            }
            if (!ArmMotion_CommandTokenIsCurrent(context->generation))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }

            control1 = context->trajectory.point[1];
            control2 = context->trajectory.point[2];
            target = context->trajectory.point[3];
            trajectory_status = ArmTrajectory_InitCubicBezier(
                &context->trajectory,
                &start,
                &control1,
                &control2,
                &target,
                context->result.requested_time_s,
                config->trajectory_period_ms);
            context->result.trajectory_status = trajectory_status;
            context->result.start = start;
            if (trajectory_status != ARM_TRAJECTORY_OK)
            {
                return ArmMotion_TrajectoryFault(
                    ARM_MOTION_TRAJECTORY_ERROR);
            }

            /* 用刚收到的真实 36 位置重建 P0，再做一次完整预检。 */
            motion_status = ArmMotion_PreflightTrajectory(
                config, &context->trajectory);
            if (!ArmMotion_CommandTokenIsCurrent(context->generation))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            if (motion_status == ARM_MOTION_OK)
            {
                motion_status = ArmMotion_ValidateDiscreteTrajectory(
                    config, &context->trajectory);
                if (!ArmMotion_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_MOTION_CANCELLED;
                }
            }
            if ((motion_status == ARM_MOTION_OK) ||
                (motion_status == ARM_MOTION_TIME_UNACHIEVABLE))
            {
                minimum_status = ArmMotion_CertifyMinimumTime(
                    config,
                    &context->trajectory,
                    motion_status == ARM_MOTION_OK);
                if (minimum_status != ARM_MOTION_OK)
                {
                    motion_status = minimum_status;
                }
                if (!ArmMotion_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_MOTION_CANCELLED;
                }
            }
            if ((motion_status != ARM_MOTION_OK) &&
                (motion_status != ARM_MOTION_TIME_UNACHIEVABLE))
            {
                context->result.minimum_time_s = 0.0f;
            }
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            motion_status = ArmMotion_SolveTrajectorySample(
                config,
                &context->trajectory,
                0.0f,
                &joints,
                &context->reference_coordinates,
                &trajectory_status,
                &ik_status);
                if (!ArmMotion_CommandTokenIsCurrent(context->generation))
                {
                    context->result.minimum_time_s = 0.0f;
                    return ARM_MOTION_CANCELLED;
                }
            if (motion_status != ARM_MOTION_OK)
            {
                context->result.trajectory_status = trajectory_status;
                context->result.ik_status = ik_status;
                return ArmMotion_TrajectoryFault(motion_status);
            }

            /*
             * 预检通过后清除旧帧并重新开启定时反馈。后续必须再
             * 收到这个时刻之后的新 35/36，才允许启动 F6 轨迹。
             */
            context->feedback_stream_enabled = true;
            motion_status = ArmMotion_SetTrajectoryFeedback(
                config, true, context->generation, true);
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            if ((zdt_can_rx_drop_count != context->rx_drop_baseline) ||
                (zdt_can_error_count != context->can_error_baseline))
            {
                return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
            }
            (void)ZDT_CAN_FlushRx();
            memset(arm_motion_driver_status,
                   0,
                   sizeof(arm_motion_driver_status));
            context->feedback_request_ms = HAL_GetTick();
            osDelay(1U);
            if (ArmMotion_RequestTrajectoryStatus(config) != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
            }
            context->rx_drop_baseline = zdt_can_rx_drop_count;
            now_ms = HAL_GetTick();
            context->trajectory_preflight_ready = true;
            context->heartbeat_ack_mask = ARM_MOTION_ALL_MOTORS_MASK;
            if (config->trajectory_heartbeat_ms != 0U)
            {
                context->heartbeat_ack_mask = 0U;
                context->heartbeat_enabled = true;
                context->heartbeat_request_ms = HAL_GetTick();
                motion_status = ArmMotion_SetTrajectoryHeartbeat(
                    config,
                    (uint32_t)config->trajectory_heartbeat_ms,
                    context->generation,
                    true);
                if (motion_status != ARM_MOTION_OK)
                {
                    return ArmMotion_TrajectoryFault(motion_status);
                }
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            if (!ArmMotion_TryPublishTrajectoryState(
                    context->generation,
                    ARM_MOTION_TRAJECTORY_WAITING_FEEDBACK,
                    ARM_MOTION_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            return ARM_MOTION_BUSY;
        }

        if (context->trajectory_preflight_ready &&
            ArmMotion_AllPositionFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ArmMotion_AllMotorsStopped(
                config, now_ms, context->feedback_request_ms) &&
            ArmMotion_AllStatusFeedbackFreshSince(
                config, now_ms, context->feedback_request_ms) &&
            ((context->heartbeat_ack_mask & ARM_MOTION_ALL_MOTORS_MASK) ==
             ARM_MOTION_ALL_MOTORS_MASK))
        {
            /* 预检期间若机械位置发生变化，禁止用旧 P0 直接启动。 */
            if (!ArmMotion_TrajectoryStartMatchesFeedback(config))
            {
                return ArmMotion_TrajectoryFault(
                    ARM_MOTION_POSITION_UNKNOWN);
            }
            /* 从心跳已确认且反馈仍新鲜的时刻重新起算绝对节拍。 */
            now_ms = HAL_GetTick();
            memset(context->previous_command_rpm,
                   0,
                   sizeof(context->previous_command_rpm));
            context->segment_index = 0U;
            context->start_ms = now_ms;
            context->next_update_ms = now_ms;
            arm_motion_current_valid = false;
            if (!ArmMotion_TryPublishTrajectoryState(
                    context->generation,
                    ARM_MOTION_TRAJECTORY_RUNNING_SPEED,
                    ARM_MOTION_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            return ARM_MOTION_BUSY;
        }

        if (ArmMotion_TimeReached(now_ms, context->feedback_deadline_ms))
        {
            return ArmMotion_TrajectoryFault(ARM_MOTION_FEEDBACK_TIMEOUT);
        }
        return ARM_MOTION_BUSY;
    }

    if (context->result.state == ARM_MOTION_TRAJECTORY_RUNNING_SPEED)
    {
        if (!ArmMotion_AllPositionFeedbackFresh(config, now_ms))
        {
            return ArmMotion_TrajectoryFault(ARM_MOTION_FEEDBACK_TIMEOUT);
        }
        if (!ArmMotion_TimeReached(now_ms, context->next_update_ms))
        {
            return ARM_MOTION_BUSY;
        }
        if ((uint32_t)(now_ms - context->next_update_ms) >
            (uint32_t)config->trajectory_deadline_tolerance_ms)
        {
            return ArmMotion_TrajectoryFault(ARM_MOTION_DEADLINE_MISSED);
        }

        if (context->segment_index >= context->trajectory.segment_count)
        {
            stop_timeout_ms = ArmMotion_TrajectoryStopTimeoutMs(config);
            if (!ArmMotion_CommandLock())
            {
                return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
            }
            if (!ArmMotion_CommandTokenIsCurrent(context->generation))
            {
                ArmMotion_CommandUnlock();
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            motion_status = ArmMotion_SendTrajectoryZeroVelocity(config);
            ArmMotion_CommandUnlock();
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            context->stop_command_ms = HAL_GetTick();
            context->heartbeat_ack_mask = ARM_MOTION_ALL_MOTORS_MASK;
            if (context->heartbeat_enabled)
            {
                context->heartbeat_ack_mask = 0U;
                context->heartbeat_request_ms = HAL_GetTick();
                if (ArmMotion_SetTrajectoryHeartbeat(
                        config, 0U, 0U, false) !=
                    ARM_MOTION_OK)
                {
                    return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
                }
                context->heartbeat_disable_pending = true;
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            context->stop_deadline_ms = now_ms + stop_timeout_ms;
            context->stop_speed_confirmed = false;
            if (!ArmMotion_TryPublishTrajectoryState(
                    context->generation,
                    ARM_MOTION_TRAJECTORY_STOPPING_SPEED,
                    ARM_MOTION_BUSY))
            {
                context->result.minimum_time_s = 0.0f;
                return ARM_MOTION_CANCELLED;
            }
            return ARM_MOTION_BUSY;
        }

        elapsed_ms = context->segment_index *
                     (uint32_t)config->trajectory_period_ms;
        if (elapsed_ms > context->trajectory.duration_ms)
        {
            elapsed_ms = context->trajectory.duration_ms;
        }
        next_elapsed_ms = (context->segment_index + 1U) *
                          (uint32_t)config->trajectory_period_ms;
        if (next_elapsed_ms > context->trajectory.duration_ms)
        {
            next_elapsed_ms = context->trajectory.duration_ms;
        }
        next_elapsed_s = (float)next_elapsed_ms * 0.001f;
        interval_s = (float)(next_elapsed_ms - elapsed_ms) * 0.001f;
        motion_status = ArmMotion_SolveTrajectorySample(
            config,
            &context->trajectory,
            next_elapsed_s,
            &joints,
            &next_coordinates,
            &trajectory_status,
            &ik_status);
        if (motion_status != ARM_MOTION_OK)
        {
            context->result.trajectory_status = trajectory_status;
            context->result.ik_status = ik_status;
            context->result.failed_sample = context->segment_index + 1U;
            return ArmMotion_TrajectoryFault(motion_status);
        }
        if (!ArmMotion_CommandLock())
        {
            return ArmMotion_TrajectoryFault(ARM_MOTION_CAN_ERROR);
        }
        if (!ArmMotion_CommandTokenIsCurrent(context->generation))
        {
            ArmMotion_CommandUnlock();
            context->result.minimum_time_s = 0.0f;
            return ARM_MOTION_CANCELLED;
        }
        motion_status = ArmMotion_SendTrajectoryVelocity(
            config,
            &context->reference_coordinates,
            &next_coordinates,
            interval_s,
            now_ms);
        ArmMotion_CommandUnlock();
        if (motion_status != ARM_MOTION_OK)
        {
            return ArmMotion_TrajectoryFault(motion_status);
        }
        context->reference_coordinates = next_coordinates;
        ++context->segment_index;
        offset_ms = next_elapsed_ms;
        context->next_update_ms = context->start_ms + offset_ms;
        return ARM_MOTION_BUSY;
    }

    if (context->result.state == ARM_MOTION_TRAJECTORY_STOPPING_SPEED)
    {
        if ((context->heartbeat_ack_mask & ARM_MOTION_ALL_MOTORS_MASK) !=
            ARM_MOTION_ALL_MOTORS_MASK)
        {
            if (ArmMotion_TimeReached(now_ms, context->feedback_deadline_ms))
            {
                return ArmMotion_TrajectoryFault(
                    ARM_MOTION_FEEDBACK_TIMEOUT);
            }
            return ARM_MOTION_BUSY;
        }

        context->heartbeat_enabled = false;
        context->heartbeat_disable_pending = false;
        if (!context->stop_speed_confirmed)
        {
            if (!ArmMotion_AllSpeedFeedbackFreshSince(
                    config, now_ms, context->stop_command_ms))
            {
                if (ArmMotion_TimeReached(
                        now_ms, context->feedback_deadline_ms))
                {
                    return ArmMotion_TrajectoryFault(
                        ARM_MOTION_FEEDBACK_TIMEOUT);
                }
                return ARM_MOTION_BUSY;
            }
            if (!ArmMotion_AllMotorsStopped(
                    config, now_ms, context->stop_command_ms))
            {
                if (ArmMotion_TimeReached(now_ms, context->stop_deadline_ms))
                {
                    return ArmMotion_TrajectoryFault(
                        ARM_MOTION_REACHED_TIMEOUT);
                }
                return ARM_MOTION_BUSY;
            }

            /*
             * 35 已确认三轴停稳。丢弃减速阶段的旧 36，建立屏障并
             * 主动读取一次位置，只允许用停稳之后的三轴坐标生成 FD。
             */
            ArmMotion_DrainRx(config, 0U);
            ArmMotion_DrainRx(config, 0U);
            context->stop_position_barrier_ms = HAL_GetTick();
            for (uint8_t index = 0U;
                 index < ARM_MOTION_MOTOR_COUNT;
                 ++index)
            {
                arm_motion_driver_status[index].realtime_position_valid = false;
            }
            motion_status = ArmMotion_RequestTrajectoryPosition(config);
            if (motion_status != ARM_MOTION_OK)
            {
                return ArmMotion_TrajectoryFault(motion_status);
            }
            now_ms = HAL_GetTick();
            context->feedback_deadline_ms =
                now_ms + (uint32_t)config->trajectory_feedback_timeout_ms;
            context->stop_speed_confirmed = true;
            return ARM_MOTION_BUSY;
        }

        if (!ArmMotion_AllPositionFeedbackFreshSince(
                config, now_ms, context->stop_position_barrier_ms))
        {
            if (ArmMotion_TimeReached(now_ms, context->feedback_deadline_ms))
            {
                return ArmMotion_TrajectoryFault(
                    ARM_MOTION_FEEDBACK_TIMEOUT);
            }
            return ARM_MOTION_BUSY;
        }

        motion_status = ArmMotion_BeginTrajectoryFinalPosition(
            config, now_ms);
        if (motion_status == ARM_MOTION_CANCELLED)
        {
            return ARM_MOTION_CANCELLED;
        }
        if ((motion_status != ARM_MOTION_OK) &&
            (motion_status != ARM_MOTION_BUSY))
        {
            return ArmMotion_TrajectoryFault(motion_status);
        }
        return motion_status;
    }

    /* SETTLING_POSITION：token 与参与轴全部 FD 9F 在同一临界区提交。 */
    motion_status = ArmMotion_CheckMoveCompletion(
        context->generation, true);
    if (motion_status == ARM_MOTION_OK)
    {
        return ARM_MOTION_OK;
    }
    if (motion_status != ARM_MOTION_BUSY)
    {
        return ArmMotion_TrajectoryFault(motion_status);
    }
    if (ArmMotion_TimeReached(now_ms, context->settle_deadline_ms))
    {
        return ArmMotion_TrajectoryFault(ARM_MOTION_REACHED_TIMEOUT);
    }

    ArmMotion_ServiceDiagnostics(config, now_ms);
    return ARM_MOTION_BUSY;
}

bool ArmMotion_TrajectoryIsBusy(void)
{
    return ArmMotion_TrajectoryStateIsBusy(
        arm_motion_trajectory_context.result.state);
}

ArmMotionStatus ArmMotion_StartBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmMotionStatus status;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    status = ArmMotion_StartBezierInternal(
        control1, control2, target, time_s);
    ArmMotion_TrajectoryApiUnlock();
    return status;
}

ArmMotionStatus ArmMotion_ServiceTrajectory(void)
{
    ArmMotionStatus status;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    status = ArmMotion_ServiceTrajectoryInternal();
    ArmMotion_TrajectoryApiUnlock();
    return status;
}

ArmMotionStatus ArmMotion_AbortTrajectory(void)
{
    if (!ArmMotion_TrajectoryIsBusy())
    {
        return ARM_MOTION_OK;
    }
    return ArmMotion_TrajectoryFault(ARM_MOTION_CANCELLED);
}

ArmMotionStatus ArmMotion_MoveBezier(
    const ArmCartesianPosition *control1,
    const ArmCartesianPosition *control2,
    const ArmCartesianPosition *target,
    float time_s)
{
    ArmMotionStatus status;

    if (!ArmMotion_TrajectoryApiLock())
    {
        return ARM_MOTION_BUSY;
    }
    status = ArmMotion_StartBezierInternal(
        control1, control2, target, time_s);
    if (status != ARM_MOTION_OK)
    {
        ArmMotion_TrajectoryApiUnlock();
        return status;
    }

    while (ArmMotion_TrajectoryIsBusy())
    {
        status = ArmMotion_ServiceTrajectoryInternal();
        if ((status != ARM_MOTION_OK) && (status != ARM_MOTION_BUSY))
        {
            ArmMotion_TrajectoryApiUnlock();
            return status;
        }
        if (ArmMotion_TrajectoryIsBusy())
        {
            osDelay(1U);
        }
    }
    status = arm_motion_trajectory_context.result.status;
    ArmMotion_TrajectoryApiUnlock();
    return status;
}

const ArmMotionTrajectoryResult *ArmMotion_GetTrajectoryResult(void)
{
    return &arm_motion_trajectory_context.result;
}

ArmMotionStatus ArmMotion_SetTimedReturnInfo(const ArmMotionConfig *config,
                                             uint8_t info_function,
                                             uint16_t period_ms)
{
    ArmMotionStatus status;

    if (!ArmMotion_FeedbackConfigLock())
    {
        return ARM_MOTION_CAN_ERROR;
    }
    status = ArmMotion_SetTimedReturnInfoUnlocked(
        config, info_function, period_ms);
    ArmMotion_FeedbackConfigUnlock();
    return status;
}

ArmMotionStatus ArmMotion_GetDriverStatus(uint8_t motor_index,
                                          ArmMotionDriverStatus *status)
{
    if (status == NULL)
    {
        return ARM_MOTION_NULL_POINTER;
    }
    if (motor_index >= ARM_MOTION_MOTOR_COUNT)
    {
        return ARM_MOTION_INVALID_CONFIG;
    }

    *status = arm_motion_driver_status[motor_index];
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
        case ARM_MOTION_REACHED_TIMEOUT:
            return "driver reached feedback timeout";
        case ARM_MOTION_DRIVER_ERROR:
            return "driver rejected command or reported fault";
        case ARM_MOTION_BUSY: return "motion is still running";
        case ARM_MOTION_TRAJECTORY_ERROR:
            return "Bezier trajectory error";
        case ARM_MOTION_FEEDBACK_TIMEOUT:
            return "trajectory feedback timeout";
        case ARM_MOTION_DEADLINE_MISSED:
            return "trajectory service deadline missed";
        case ARM_MOTION_DIRECTION_REVERSAL:
            return "motor direction reversal inside trajectory";
        case ARM_MOTION_CANCELLED:
            return "motion cancelled by stop command";
        default: return "unknown status";
    }
}
