#include "end_level_task.h"

#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"

#include "X_V2.h"
#include "icm42688.h"

#include <math.h>
#include <stddef.h>

#define END_LEVEL_RAD_TO_DEG                   (57.2957795130823208768f)
#define END_LEVEL_COMPLEMENT_GYRO_WEIGHT       (0.98f)
#define END_LEVEL_COMPLEMENT_ACCEL_WEIGHT      (0.02f)
#define END_LEVEL_ACCEL_NORM_MIN_G             (0.75f)
#define END_LEVEL_ACCEL_NORM_MAX_G             (1.25f)
#define END_LEVEL_CONTROL_PERIOD_MS           (20U)
#define END_LEVEL_MAX_DT_S                    (0.100f)

static volatile EndLevelTelemetry_t end_level_telemetry;
static volatile uint8_t end_level_ready;
static volatile uint8_t end_level_fault;
/* 上电先只采集姿态；ID4 初始化完成后由 main_task 打开自动调平。 */
static volatile uint8_t end_level_control_enabled = 0U;

float end_level_roll_deg;
float end_level_pitch_deg;
float end_level_yaw_deg;
static float end_level_gyro_bias_x_dps;
static float end_level_gyro_bias_y_dps;
static float end_level_gyro_bias_z_dps;
static float end_level_target_roll_deg;
static float end_level_target_pitch_deg;
static float end_level_roll_integral_deg_s;
static float end_level_gyro_x_dps;
static uint8_t end_level_motor_running;

static float EndLevelTask_WrapAngle(float angle_deg)
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

static void EndLevelTask_ResetController(void)
{
    end_level_roll_integral_deg_s = 0.0f;
    end_level_gyro_x_dps = 0.0f;
}

static void EndLevelTask_SetFault(void)
{
    end_level_fault = 1U;
    end_level_ready = 0U;
    end_level_control_enabled = 0U;
    EndLevelTask_ResetController();
    end_level_telemetry.fault = 1U;
    end_level_telemetry.ready = 0U;
    end_level_telemetry.control_enabled = 0U;
}

static void EndLevelTask_UpdateTelemetry(void)
{
    end_level_telemetry.roll_deg = end_level_roll_deg;
    end_level_telemetry.pitch_deg = end_level_pitch_deg;
    end_level_telemetry.yaw_deg = end_level_yaw_deg;
    end_level_telemetry.gyro_bias_x_dps = end_level_gyro_bias_x_dps;
    end_level_telemetry.gyro_bias_y_dps = end_level_gyro_bias_y_dps;
    end_level_telemetry.gyro_bias_z_dps = end_level_gyro_bias_z_dps;
    end_level_telemetry.target_roll_deg = end_level_target_roll_deg;
    end_level_telemetry.target_pitch_deg = end_level_target_pitch_deg;
    end_level_telemetry.ready = end_level_ready;
    end_level_telemetry.fault = end_level_fault;
    end_level_telemetry.control_enabled = end_level_control_enabled;
}

static void EndLevelTask_AccelAngles(const ICM42688_Data_t *data,
                                     float *roll_deg,
                                     float *pitch_deg,
                                     float *accel_norm_g)
{
    float horizontal;

    horizontal = sqrtf((data->accel_y_g * data->accel_y_g) +
                       (data->accel_z_g * data->accel_z_g));
    *roll_deg = EndLevelTask_WrapAngle(
        (atan2f(data->accel_y_g, data->accel_z_g) *
         END_LEVEL_RAD_TO_DEG) - END_LEVEL_ROLL_ZERO_OFFSET_DEG);
    *pitch_deg = atan2f(-data->accel_x_g, horizontal) *
                 END_LEVEL_RAD_TO_DEG;
    *accel_norm_g = sqrtf((data->accel_x_g * data->accel_x_g) +
                          (data->accel_y_g * data->accel_y_g) +
                          (data->accel_z_g * data->accel_z_g));
}
 ICM42688_Data_t data;
bool EndLevelTask_CalibrateGyro(void)
{
   
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint32_t valid_samples = 0U;
    uint32_t consecutive_errors = 0U;
    uint32_t retry;

    while (valid_samples < END_LEVEL_TASK_CALIBRATION_SAMPLES)
    {
        retry = 0U;
        while (retry < END_LEVEL_TASK_CALIBRATION_READ_RETRIES)
        {
            if (ICM42688_ReadData(&data) == HAL_OK)
            {
                break;
            }

            ++retry;
            ++end_level_telemetry.read_error_count;
            osDelay(2U);
        }

        if (retry >= END_LEVEL_TASK_CALIBRATION_READ_RETRIES)
        {
            ++consecutive_errors;
            if (consecutive_errors >=
                END_LEVEL_TASK_CALIBRATION_MAX_CONSECUTIVE_ERRORS)
            {
                return false;
            }
            continue;
        }

        consecutive_errors = 0U;
        sum_x += data.gyro_x_dps;
        sum_y += data.gyro_y_dps;
        sum_z += data.gyro_z_dps;
        ++valid_samples;

        if (valid_samples < END_LEVEL_TASK_CALIBRATION_SAMPLES)
        {
            osDelay(END_LEVEL_TASK_PERIOD_MS);
        }
    }

    end_level_gyro_bias_x_dps =
        sum_x / (float)END_LEVEL_TASK_CALIBRATION_SAMPLES;
    end_level_gyro_bias_y_dps =
        sum_y / (float)END_LEVEL_TASK_CALIBRATION_SAMPLES;
    end_level_gyro_bias_z_dps =
        sum_z / (float)END_LEVEL_TASK_CALIBRATION_SAMPLES;

    return true;
}

static bool EndLevelTask_InitializeAttitude(void)
{
    ICM42688_Data_t data;
    float accel_roll_deg;
    float accel_pitch_deg;
    float accel_norm_g;

    if (ICM42688_ReadData(&data) != HAL_OK)
    {
        ++end_level_telemetry.read_error_count;
        return false;
    }

    EndLevelTask_AccelAngles(&data,
                             &accel_roll_deg,
                             &accel_pitch_deg,
                             &accel_norm_g);
    if ((accel_norm_g < END_LEVEL_ACCEL_NORM_MIN_G) ||
        (accel_norm_g > END_LEVEL_ACCEL_NORM_MAX_G))
    {
        /* 设备应在静止状态校准，避免把搬动时的线加速度当成重力。 */
        return false;
    }

    end_level_roll_deg = accel_roll_deg;
    end_level_pitch_deg = accel_pitch_deg;
    end_level_gyro_x_dps = 0.0f;
    end_level_roll_integral_deg_s = 0.0f;
    /* 没有磁力计，Yaw 以启动时姿态为 0°，只做相对角度诊断。 */
    end_level_yaw_deg = 0.0f;
    /* Roll 使用固定 0° 目标，不再把上电姿态当成目标。 */
    end_level_target_roll_deg = END_LEVEL_ROLL_TARGET_DEG;
    end_level_target_pitch_deg = 0.0f;
    return true;
}

static void EndLevelTask_UpdateAttitude(const ICM42688_Data_t *data,
                                        float dt_s)
{
    float accel_roll_deg;
    float accel_pitch_deg;
    float accel_norm_g;

    EndLevelTask_AccelAngles(data,
                             &accel_roll_deg,
                             &accel_pitch_deg,
                             &accel_norm_g);

    end_level_gyro_x_dps =
        data->gyro_x_dps - end_level_gyro_bias_x_dps;
    end_level_roll_deg += end_level_gyro_x_dps * dt_s;
    end_level_pitch_deg +=
        (data->gyro_y_dps - end_level_gyro_bias_y_dps) * dt_s;
    end_level_yaw_deg +=
        (data->gyro_z_dps - end_level_gyro_bias_z_dps) * dt_s;

    if ((accel_norm_g >= END_LEVEL_ACCEL_NORM_MIN_G) &&
        (accel_norm_g <= END_LEVEL_ACCEL_NORM_MAX_G))
    {
        end_level_roll_deg =
            (END_LEVEL_COMPLEMENT_GYRO_WEIGHT * end_level_roll_deg) +
            (END_LEVEL_COMPLEMENT_ACCEL_WEIGHT * accel_roll_deg);
        end_level_pitch_deg =
            (END_LEVEL_COMPLEMENT_GYRO_WEIGHT * end_level_pitch_deg) +
            (END_LEVEL_COMPLEMENT_ACCEL_WEIGHT * accel_pitch_deg);
    }

    end_level_roll_deg = EndLevelTask_WrapAngle(end_level_roll_deg);
    end_level_pitch_deg = EndLevelTask_WrapAngle(end_level_pitch_deg);
    end_level_yaw_deg = EndLevelTask_WrapAngle(end_level_yaw_deg);
}

static void EndLevelTask_StopMotor(void)
{
    if (end_level_motor_running != 0U)
    {
        (void)X_V2_Stop_Now(END_LEVEL_MOTOR_ID, false);
        end_level_motor_running = 0U;
    }
}

static void EndLevelTask_UpdateMotor(void)
{
    float error_deg;
    float control_rpm;
    float speed_rpm;
    X_V2_Direction_t direction;
    HAL_StatusTypeDef status;
    const float control_dt_s =
        (float)END_LEVEL_CONTROL_PERIOD_MS / 1000.0f;

    if ((end_level_control_enabled == 0U) ||
        (end_level_ready == 0U) ||
        (end_level_fault != 0U))
    {
        EndLevelTask_ResetController();
        EndLevelTask_StopMotor();
        return;
    }

    error_deg = EndLevelTask_WrapAngle(
        end_level_target_roll_deg - end_level_roll_deg);

    /* 只对死区外的误差积分，避免传感器噪声使积分项来回累积。 */
    if (fabsf(error_deg) > END_LEVEL_MOTOR_DEADBAND_DEG)
    {
        end_level_roll_integral_deg_s += error_deg * control_dt_s;
        if (end_level_roll_integral_deg_s >
            END_LEVEL_MOTOR_INTEGRAL_LIMIT_DEG_S)
        {
            end_level_roll_integral_deg_s =
                END_LEVEL_MOTOR_INTEGRAL_LIMIT_DEG_S;
        }
        else if (end_level_roll_integral_deg_s <
                 -END_LEVEL_MOTOR_INTEGRAL_LIMIT_DEG_S)
        {
            end_level_roll_integral_deg_s =
                -END_LEVEL_MOTOR_INTEGRAL_LIMIT_DEG_S;
        }
    }
    else
    {
        /* 接近水平时缓慢释放积分，避免停机后保留过大的历史指令。 */
        end_level_roll_integral_deg_s *= 0.98f;
    }

    /*
     * 速度模式下的外环：角度 P + 小积分 I + 陀螺仪角速度阻尼。
     * 角速度项直接使用去零偏的 gyro X，避免对角度数值再次微分放大噪声。
     */
    control_rpm =
        (END_LEVEL_MOTOR_KP_RPM_PER_DEG * error_deg) +
        (END_LEVEL_MOTOR_KI_RPM_PER_DEG_S *
         end_level_roll_integral_deg_s) -
        (END_LEVEL_MOTOR_KD_RPM_PER_DPS * end_level_gyro_x_dps);

    if (control_rpm > END_LEVEL_MOTOR_MAX_SPEED_RPM)
    {
        control_rpm = END_LEVEL_MOTOR_MAX_SPEED_RPM;
    }
    else if (control_rpm < -END_LEVEL_MOTOR_MAX_SPEED_RPM)
    {
        control_rpm = -END_LEVEL_MOTOR_MAX_SPEED_RPM;
    }

    if (fabsf(control_rpm) < END_LEVEL_MOTOR_MIN_COMMAND_RPM)
    {
        EndLevelTask_StopMotor();
        return;
    }

    speed_rpm = fabsf(control_rpm);

    if (control_rpm > 0.0f)
    {
        direction = (END_LEVEL_MOTOR_POSITIVE_IS_CCW != 0U) ?
                    X_V2_DIR_CCW : X_V2_DIR_CW;
    }
    else
    {
        direction = (END_LEVEL_MOTOR_POSITIVE_IS_CCW != 0U) ?
                    X_V2_DIR_CW : X_V2_DIR_CCW;
    }

    status = X_V2_Vel_Control(END_LEVEL_MOTOR_ID,
                              direction,
                              END_LEVEL_MOTOR_ACC_RPM_PER_S,
                              speed_rpm,
                              false);
    if (status == HAL_OK)
    {
        end_level_motor_running = 1U;
    }
    else
    {
        /* CAN 发送失败时立即清除运行标志，下一周期继续保护性停止。 */
        end_level_motor_running = 0U;
    }
}

void EndLevelTask_Run(void *argument)
{
    ICM42688_Data_t data;
    uint32_t next_wake;
    uint32_t previous_tick;
    uint32_t now_tick;
    uint32_t control_elapsed_ms = END_LEVEL_CONTROL_PERIOD_MS;
    uint8_t sample_valid;
    float dt_s;

    (void)argument;
    end_level_control_enabled = 0U;
    end_level_ready = 0U;
    end_level_fault = 0U;
    end_level_motor_running = 0U;
    EndLevelTask_ResetController();
    end_level_telemetry.sample_count = 0U;
    end_level_telemetry.read_error_count = 0U;
    end_level_telemetry.int1_count = 0U;
    end_level_telemetry.fault = 0U;
    end_level_telemetry.ready = 0U;
    end_level_telemetry.control_enabled = end_level_control_enabled;

    if (ICM42688_Init(&hi2c1) != HAL_OK)
    {
        EndLevelTask_SetFault();
    }
    else if (!EndLevelTask_CalibrateGyro())
    {
        EndLevelTask_SetFault();
    }
    else if (!EndLevelTask_InitializeAttitude())
    {
        EndLevelTask_SetFault();
    }
    else
    {
        end_level_ready = 1U;
        end_level_telemetry.ready = 1U;
    }

    EndLevelTask_UpdateTelemetry();

    if (end_level_fault != 0U)
    {
        /* 失败后保持任务存活，便于调试遥测；main_task 会拒绝机械臂动作。 */
        for (;;)
        {
            osDelay(1000U);
        }
    }

    previous_tick = HAL_GetTick();
    next_wake = osKernelGetTickCount() + END_LEVEL_TASK_PERIOD_MS;

    for (;;)
    {
        now_tick = HAL_GetTick();
        dt_s = (float)(now_tick - previous_tick) / 1000.0f;
        if ((dt_s <= 0.0f) || (dt_s > END_LEVEL_MAX_DT_S))
        {
            dt_s = (float)END_LEVEL_TASK_PERIOD_MS / 1000.0f;
        }
        previous_tick = now_tick;

        if (ICM42688_ReadData(&data) == HAL_OK)
        {
            EndLevelTask_UpdateAttitude(&data, dt_s);
            ++end_level_telemetry.sample_count;
            sample_valid = 1U;
        }
        else
        {
            ++end_level_telemetry.read_error_count;
            EndLevelTask_ResetController();
            EndLevelTask_StopMotor();
            sample_valid = 0U;
        }

        control_elapsed_ms += END_LEVEL_TASK_PERIOD_MS;
        if (control_elapsed_ms >= END_LEVEL_CONTROL_PERIOD_MS)
        {
            if (sample_valid != 0U)
            {
                EndLevelTask_UpdateMotor();
            }
            else
            {
                /* 传感器本周期无有效数据时，不允许用旧角度继续驱动。 */
                EndLevelTask_ResetController();
                EndLevelTask_StopMotor();
            }
            control_elapsed_ms = 0U;
        }

        EndLevelTask_UpdateTelemetry();
        (void)osDelayUntil(next_wake);
        next_wake += END_LEVEL_TASK_PERIOD_MS;
    }
}

bool EndLevelTask_IsReady(void)
{
    return (end_level_ready != 0U);
}

bool EndLevelTask_HasFault(void)
{
    return (end_level_fault != 0U);
}

bool EndLevelTask_WaitReady(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    for (;;)
    {
        if (end_level_ready != 0U)
        {
            return true;
        }
        if (end_level_fault != 0U)
        {
            return false;
        }
        if ((timeout_ms != 0xFFFFFFFFUL) &&
            ((HAL_GetTick() - start_tick) >= timeout_ms))
        {
            return false;
        }
        osDelay(10U);
    }
}

void EndLevelTask_SetControlEnabled(bool enabled)
{
    end_level_control_enabled = enabled ? 1U : 0U;
    end_level_telemetry.control_enabled = end_level_control_enabled;
}

bool EndLevelTask_IsControlEnabled(void)
{
    return (end_level_control_enabled != 0U);
}

void EndLevelTask_CaptureLevel(void)
{
    if (end_level_ready != 0U)
    {
        /* 固定水平模式下不再把当前姿态覆盖成目标。 */
        end_level_target_roll_deg = END_LEVEL_ROLL_TARGET_DEG;
        end_level_target_pitch_deg = 0.0f;
        end_level_roll_integral_deg_s = 0.0f;
        EndLevelTask_UpdateTelemetry();
    }
}

void EndLevelTask_GetTelemetry(EndLevelTelemetry_t *telemetry)
{
    if (telemetry == NULL)
    {
        return;
    }

    telemetry->roll_deg = end_level_telemetry.roll_deg;
    telemetry->pitch_deg = end_level_telemetry.pitch_deg;
    telemetry->yaw_deg = end_level_telemetry.yaw_deg;
    telemetry->gyro_bias_x_dps = end_level_telemetry.gyro_bias_x_dps;
    telemetry->gyro_bias_y_dps = end_level_telemetry.gyro_bias_y_dps;
    telemetry->gyro_bias_z_dps = end_level_telemetry.gyro_bias_z_dps;
    telemetry->target_roll_deg = end_level_telemetry.target_roll_deg;
    telemetry->target_pitch_deg = end_level_telemetry.target_pitch_deg;
    telemetry->sample_count = end_level_telemetry.sample_count;
    telemetry->read_error_count = end_level_telemetry.read_error_count;
    telemetry->int1_count = end_level_telemetry.int1_count;
    telemetry->ready = end_level_telemetry.ready;
    telemetry->fault = end_level_telemetry.fault;
    telemetry->control_enabled = end_level_telemetry.control_enabled;
}

/* PB5 已由 CubeMX 配置为 ICM42688 INT1 的下降沿 EXTI。ISR 中只记数。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        ++end_level_telemetry.int1_count;
    }
}
