/*
  程序说明: 每 20 ms 读取姿态，控制末端到达并持续保持指定俯仰角。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#include "arm_end_level_task.h"
#include "jy901s.h"
#include "X_V2.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

#define ARM_LEVEL_ANGLE_FRESH_MS (40U)
#define ARM_LEVEL_HEARTBEAT_MS (200U)
static ArmEndLevelState_t level_state;
static ArmEndLevelState_t published_state;
static volatile uint8_t initialized;
static volatile uint8_t requested;
static volatile uint8_t driver_ready;
/* 请求由普通任务提交，目标角、序号和占用状态统一在临界区内读写。 */
static volatile float requested_pitch_deg;
static volatile uint32_t request_sequence;
static volatile uint8_t move_busy;
/* 以下状态仅由末端控制任务更新。 */
static float integral_deg_s;
static uint32_t last_control_ms;
static uint32_t previous_angle_count;
static uint32_t last_restart_ms;
static uint32_t settle_start_ms;
static uint8_t ready_frames;
static uint8_t settling;

/*
  函数说明: 对调平控制量执行正负对称限幅。
  调用示例: ArmEndLevel_Clamp(value, limit);
  参数意义: value 为输入数值。
            limit 为限幅值或关节限位配置。
*/
static float ArmEndLevel_Clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

/*
  函数说明: 将俯仰角误差折回到正负 180 度以内。
  调用示例: ArmEndLevel_Wrap(value);
  参数意义: value 为输入数值。
*/
static float ArmEndLevel_Wrap(float value)
{
    if (value > 180.0f) value -= 360.0f;
    if (value < -180.0f) value += 360.0f;
    return value;
}

/*
  函数说明: 清除积分并在电机可能运动时发送停止命令；失败锁存电机故障。
  调用示例: ArmEndLevel_Stop();
  参数意义: 无。
*/
static void ArmEndLevel_Stop(void)
{
    integral_deg_s = 0.0f;
    level_state.command_rpm = 0.0f;
    if (level_state.motor_running)
    {
        if (X_V2_Stop_Now(ARM_LEVEL_MOTOR_ID, false) == HAL_OK)
            level_state.motor_running = 0U;
        else
            level_state.motor_fault = 1U;
    }
}

/*
  函数说明: 启动 JY901S、等待连续新姿态帧并初始化末端电机；由 main_task 调用。
  调用示例: ArmEndLevelTask_Init();
  参数意义: 无。
*/
uint8_t ArmEndLevelTask_Init(void)
{
    taskENTER_CRITICAL();
    requested = 0U;
    driver_ready = 0U;
    requested_pitch_deg = 0.0f;
    ++request_sequence;
    taskEXIT_CRITICAL();
    if (JY901S_StartReceive() != HAL_OK) return 0U;
    initialized = 1U;
    if (!ArmEndLevelTask_WaitReady(ARM_LEVEL_READY_TIMEOUT_MS)) return 0U;
    if (X_V2_Reset_Clog_Pro(ARM_LEVEL_MOTOR_ID) != HAL_OK) return 0U;
    osDelay(10U);
    if (X_V2_Set_Heartbeat(ARM_LEVEL_MOTOR_ID, false, ARM_LEVEL_HEARTBEAT_MS) != HAL_OK)
        return 0U;
    if (X_V2_En_Control(ARM_LEVEL_MOTOR_ID, true, false) != HAL_OK) return 0U;
    driver_ready = 1U;
    return 1U;
}

/*
  函数说明: 开关目标角保持；关闭会取消正在等待的角度动作，重新开启沿用原目标。
  调用示例: ArmEndLevelTask_SetControlEnabled(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
void ArmEndLevelTask_SetControlEnabled(uint8_t enabled)
{
    taskENTER_CRITICAL();
    enabled = enabled && driver_ready;
    if (requested != enabled)
    {
        requested = enabled;
        ++request_sequence;
    }
    taskEXIT_CRITICAL();
}

/*
  函数说明: 提交目标角并等待实际到位；电机发令和到位判定统一由末端任务完成。
  调用示例: success = ArmEndLevelTask_MoveToAngle(-30.0f);
  参数意义: angle_deg 为相对水平面的绝对俯仰角，允许 -90～+10 度。
            成功返回 1 并持续保持；失败返回 0，已受理的失败动作关闭保持。
            仅普通任务调用，等待期间让出 CPU；同一时刻只接受一次角度动作。
*/
uint8_t ArmEndLevelTask_MoveToAngle(float angle_deg)
{
    ArmEndLevelState_t state;
    uint32_t sequence, start_ms, now_ms;
    uint8_t cancelled, success = 0U;

    if (!isfinite(angle_deg) || angle_deg < ARM_LEVEL_TARGET_MIN_DEG ||
        angle_deg > ARM_LEVEL_TARGET_MAX_DEG || __get_IPSR() != 0U ||
        osKernelGetState() != osKernelRunning) return 0U;
    start_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    if (!initialized || !driver_ready || !requested || move_busy ||
        !published_state.ready || published_state.motor_fault ||
        (uint32_t)(start_ms - published_state.sample_tick_ms) > ARM_LEVEL_ANGLE_FRESH_MS)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    move_busy = 1U;
    requested_pitch_deg = angle_deg;
    sequence = ++request_sequence;
    taskEXIT_CRITICAL();

    for (;;)
    {
        ArmEndLevelTask_GetState(&state);
        now_ms = HAL_GetTick();
        taskENTER_CRITICAL();
        cancelled = !requested || request_sequence != sequence;
        taskEXIT_CRITICAL();
        if (cancelled || !state.ready || state.motor_fault ||
            (uint32_t)(now_ms - state.sample_tick_ms) > ARM_LEVEL_ANGLE_FRESH_MS ||
            (uint32_t)(now_ms - start_ms) >= ARM_LEVEL_MOVE_TIMEOUT_MS) break;
        if (state.target_sequence == sequence && state.control_enabled && state.target_reached)
        {
            success = 1U;
            break;
        }
        osDelay(ARM_END_LEVEL_TASK_PERIOD_MS);
    }

    taskENTER_CRITICAL();
    cancelled = 0U;
    if (!success && request_sequence == sequence)
    {
        requested = 0U;
        ++request_sequence;
        cancelled = 1U;
    }
    taskEXIT_CRITICAL();
    /* 交给控制任务停止电机；主任务不直接发送 ID4 指令，避免两个控制来源竞争。 */
    if (cancelled) osDelay(ARM_END_LEVEL_TASK_PERIOD_MS);
    taskENTER_CRITICAL();
    move_busy = 0U;
    taskEXIT_CRITICAL();
    return success;
}

/*
  函数说明: 在指定时限内等待连续有效姿态帧；它只等待数据，不执行机械臂动作。
  调用示例: ArmEndLevelTask_WaitReady(timeout_ms);
  参数意义: timeout_ms 为等待或心跳超时，毫秒。
*/
uint8_t ArmEndLevelTask_WaitReady(uint32_t timeout_ms)
{
    ArmEndLevelState_t state;
    uint32_t start = HAL_GetTick();
    do
    {
        ArmEndLevelTask_GetState(&state);
        if (state.ready) return 1U;
        osDelay(ARM_END_LEVEL_TASK_PERIOD_MS);
    } while ((uint32_t)(HAL_GetTick() - start) < timeout_ms);
    return 0U;
}

/*
  函数说明: 在临界区复制上一个完整控制周期的调平状态。
  调用示例: ArmEndLevelTask_GetState(&state);
  参数意义: state 为状态值或状态快照。
*/
void ArmEndLevelTask_GetState(ArmEndLevelState_t *state)
{
    if (state == NULL) return;
    taskENTER_CRITICAL();
    *state = published_state;
    taskEXIT_CRITICAL();
}

/*
  函数说明: 对当前生效目标执行俯仰 PID，包含积分限幅、死区、速度限制和 CAN 故障处理。
  调用示例: ArmEndLevel_Update(sample, now);
  参数意义: sample 为曲线采样或 IMU 姿态数据。
            now 为当前 HAL 毫秒时刻。
*/
static void ArmEndLevel_Update(const JY901S_Data_t *sample, uint32_t now)
{
    float error = ArmEndLevel_Wrap(sample->pitch_deg - ARM_LEVEL_PITCH_ZERO_OFFSET_DEG -
                                   level_state.target_pitch_deg);
    float rate = -sample->gyro_y_dps;
    float dt = (float)(now - last_control_ms) * 0.001f;
    float candidate;
    float output;
    X_V2_Direction_t direction;
    last_control_ms = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.020f;
    level_state.pitch_error_deg = error;
    if (fabsf(error) <= ARM_LEVEL_PITCH_DEADBAND_DEG && fabsf(rate) <= ARM_LEVEL_GYRO_DEADBAND_DPS)
    {
        ArmEndLevel_Stop();
        return;
    }
    candidate = ArmEndLevel_Clamp(integral_deg_s + error * dt, ARM_PID_MOTOR_INTEGRAL_LIMIT_DEG_S);
    output = ARM_PID_MOTOR_KP_RPM_PER_DEG * error +
        ARM_PID_MOTOR_KI_RPM_PER_DEG_S * candidate - ARM_PID_MOTOR_KD_RPM_PER_DPS * rate;
    if (fabsf(output) < ARM_LEVEL_MOTOR_MAX_SPEED_RPM || output * error < 0.0f)
        integral_deg_s = candidate;
    output = ArmEndLevel_Clamp(output, ARM_LEVEL_MOTOR_MAX_SPEED_RPM);
    if (fabsf(output) < ARM_LEVEL_MOTOR_MIN_SPEED_RPM)
    {
        ArmEndLevel_Stop();
        return;
    }
    direction = ((output > 0.0f) == (ARM_LEVEL_MOTOR_POSITIVE_IS_CCW != 0U)) ? X_V2_DIR_CCW : X_V2_DIR_CW;
    /* 先标记可能已运动，CAN 分帧发送失败也必须尝试停止。 */
    level_state.motor_running = 1U;
    if (X_V2_Vel_Control(ARM_LEVEL_MOTOR_ID, direction, ARM_LEVEL_MOTOR_ACC_RPM_PER_S,
                         fabsf(output), false) != HAL_OK)
    {
        level_state.motor_fault = 1U;
        ArmEndLevelTask_SetControlEnabled(0U);
        ArmEndLevel_Stop();
        return;
    }
    level_state.command_rpm = output;
}

/*
  函数说明: 完成一次目标接收、姿态检查、角度保持和实际到位判定，最后发布完整状态。
  调用示例: ArmEndLevel_Process();
  参数意义: 无；只由末端控制任务调用，不与主任务共同修改 PID 和电机状态。
*/
static void ArmEndLevel_Process(void)
{
    JY901S_Data_t sample;
    uint32_t now, previous_sample_ms;
    uint8_t fresh, new_frame;
    if (initialized)
    {
        fresh = JY901S_ReadData(&sample) == HAL_OK;
        now = HAL_GetTick();
        fresh = fresh && (uint32_t)(now - sample.angle_tick_ms) <= ARM_LEVEL_ANGLE_FRESH_MS &&
            isfinite(sample.pitch_deg) && isfinite(sample.gyro_y_dps);
        taskENTER_CRITICAL();
        if (level_state.target_sequence != request_sequence)
        {
            level_state.target_sequence = request_sequence;
            level_state.target_pitch_deg = requested_pitch_deg;
            integral_deg_s = 0.0f;
            last_control_ms = now;
            settling = 0U;
            level_state.target_reached = 0U;
        }
        level_state.control_enabled = requested;
        taskEXIT_CRITICAL();
        previous_sample_ms = level_state.sample_tick_ms;
        new_frame = fresh && sample.angle_count != previous_angle_count;
        if (new_frame)
        {
            previous_angle_count = sample.angle_count;
            if (ready_frames < 3U) ++ready_frames;
            ++level_state.sample_count;
            level_state.pitch_deg = sample.pitch_deg;
            level_state.sample_tick_ms = sample.angle_tick_ms;
        }
        if (!fresh)
        {
            ready_frames = 0U;
            ++level_state.read_error_count;
            if ((uint32_t)(now - last_restart_ms) >= 500U)
            {
                (void)JY901S_StartReceive();
                last_restart_ms = now;
            }
        }
        level_state.ready = ready_frames >= 3U;
        if (level_state.control_enabled && level_state.ready && !level_state.motor_fault)
            ArmEndLevel_Update(&sample, now);
        else
            ArmEndLevel_Stop();

        if (level_state.motor_fault)
        {
            ArmEndLevelTask_SetControlEnabled(0U);
            level_state.control_enabled = 0U;
        }
        if (!level_state.control_enabled || !level_state.ready || level_state.motor_fault ||
            fabsf(level_state.pitch_error_deg) > ARM_LEVEL_PITCH_DEADBAND_DEG ||
            fabsf(sample.gyro_y_dps) > ARM_LEVEL_GYRO_DEADBAND_DPS)
        {
            settling = 0U;
            level_state.target_reached = 0U;
        }
        else if (new_frame)
        {
            /* 只用新姿态帧确认到位；中途超过新鲜度间隔则重新累计 100 ms。 */
            if (!settling || (uint32_t)(sample.angle_tick_ms - previous_sample_ms) > ARM_LEVEL_ANGLE_FRESH_MS)
            {
                settling = 1U;
                settle_start_ms = now;
                level_state.target_reached = 0U;
            }
            else if ((uint32_t)(now - settle_start_ms) >= ARM_LEVEL_TARGET_SETTLE_MS)
                level_state.target_reached = 1U;
        }
    }
    taskENTER_CRITICAL();
    if (!requested || level_state.target_sequence != request_sequence)
        level_state.target_reached = 0U;
    published_state = level_state;
    taskEXIT_CRITICAL();
}

/*
  函数说明: 每 20 ms 检查姿态新鲜度并保持目标角；失效时停止末端电机。
  调用示例: 由 CubeMX 创建的 StartArmEndLevelTask 调用 ArmEndLevelTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void ArmEndLevelTask_Run(void *argument)
{
    uint32_t wake = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        ArmEndLevel_Process();
        wake += ARM_END_LEVEL_TASK_PERIOD_MS;
        if ((int32_t)(wake - osKernelGetTickCount()) <= 0)
        {
            ++level_state.cycle_overrun_count;
            wake = osKernelGetTickCount() + ARM_END_LEVEL_TASK_PERIOD_MS;
        }
        osDelayUntil(wake);
    }
}
