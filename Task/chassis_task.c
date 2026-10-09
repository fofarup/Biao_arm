#include "chassis_task.h"

#include <string.h>

#include "cmsis_os.h"

#include "X_V2.h"
#include "chassis_feedback.h"
#include "zdt_can.h"

/*
 * 底盘主循环采用 20 ms 周期：读取控制器输出、生成四轮 F6 指令，
 * 再通过一条多电机命令发送。实际位置真值来自 OPS9/HW101，
 * 电机 36/37/3A 反馈由 CanRxTask 异步解析。
 */
#define CHASSIS_TASK_PERIOD_MS             (20U)
#define CHASSIS_STARTUP_DELAY_MS          (100U)
#define CHASSIS_HEARTBEAT_TIMEOUT_MS      (300U)
#define CHASSIS_POSITION_INFO_PERIOD_MS   (50U)
#define CHASSIS_STATUS_INFO_PERIOD_MS     (50U)
#define CHASSIS_ERROR_INFO_PERIOD_MS      (100U)

volatile ChassisControlState_t chassis_debug_state;

static HAL_StatusTypeDef ChassisTask_ConfigureDrivers(
    const ChassisControlConfig_t *config)
{
    uint8_t index;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    /* 清除驱动器堵转保护状态，避免上一次故障影响本次启动。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (X_V2_Reset_Clog_Pro(config->motor_addr[index]) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(5U);
    }

    /* 使能四个底盘电机，但不使用同步缓存启动。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (X_V2_En_Control(config->motor_addr[index],
                            true,
                            false) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(5U);
    }

    /* 速度模式持续运行时启用心跳保护，通信中断后由驱动器停机。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (X_V2_Set_Heartbeat(config->motor_addr[index],
                               false,
                               CHASSIS_HEARTBEAT_TIMEOUT_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(2U);
    }

    /*
     * 低频获取电机反馈：
     * 36 观察实时位置，37 观察位置误差，3A 监测状态和故障。
     * 这些帧进入底盘专用队列，不会抢占机械臂的原有队列。
     */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        if (X_V2_Timed_Return_Info(
                config->motor_addr[index],
                X_V2_INFO_REALTIME_POSITION,
                CHASSIS_POSITION_INFO_PERIOD_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(2U);

        if (X_V2_Timed_Return_Info(
                config->motor_addr[index],
                X_V2_INFO_POSITION_ERROR,
                CHASSIS_ERROR_INFO_PERIOD_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(2U);

        if (X_V2_Timed_Return_Info(
                config->motor_addr[index],
                X_V2_INFO_STATUS,
                CHASSIS_STATUS_INFO_PERIOD_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }
        osDelay(2U);
    }

    return HAL_OK;
}

static void ChassisTask_StopDrivers(
    const ChassisControlConfig_t *config,
    uint8_t disable)
{
    uint8_t index;

    if (config == NULL)
    {
        return;
    }

    /* 先发送立即停止；初始化失败时再关闭驱动器使能。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        (void)X_V2_Stop_Now(config->motor_addr[index], false);
        osDelay(2U);
        if (disable != 0U)
        {
            (void)X_V2_En_Control(config->motor_addr[index],
                                  false,
                                  false);
            osDelay(2U);
        }
    }
}

static HAL_StatusTypeDef ChassisTask_SendVelocity(
    const ChassisWheelSetpoint_t *setpoint)
{
    X_V2_Vel_Command_t commands[CHASSIS_MOTOR_COUNT];

    if (setpoint == NULL)
    {
        return HAL_ERROR;
    }

    /* 一个多电机 F6 命令中包含四条子命令，保证同一控制周期下发。 */
    ChassisControl_BuildMotorCommands(setpoint, commands);
    return X_V2_MultiMotor_Vel_Control(
        commands,
        CHASSIS_MOTOR_COUNT);
}

void ChassisTask_Run(void *argument)
{
    ChassisControlConfig_t config;
    ChassisWheelSetpoint_t setpoint;
    ChassisControlState_t state;
    uint32_t next_wake;
    uint8_t fault_handled;

    (void)argument;

    /* 默认上电为 STOP，不会自动让底盘运动。 */
    ChassisControl_LoadDefaultConfig(&config);
    ChassisControl_Init(&config);
    memset((void *)&chassis_debug_state,
           0,
           sizeof(chassis_debug_state));
    fault_handled = 0U;

    /* 允许 CanRxTask 或机械臂任务先后调用，ZDT_CAN_Init 已做幂等保护。 */
    if (ZDT_CAN_Init() != HAL_OK)
    {
        ChassisControl_FaultStop();
        for (;;)
        {
            osDelay(1000U);
        }
    }

    osDelay(CHASSIS_STARTUP_DELAY_MS);
    if (ChassisTask_ConfigureDrivers(&config) != HAL_OK)
    {
        ChassisControl_FaultStop();
        ChassisTask_StopDrivers(&config, 1U);
        for (;;)
        {
            osDelay(1000U);
        }
    }

    next_wake = osKernelGetTickCount();

    for (;;)
    {
        /* 控制器返回 0 时通常表示故障，立即发送停止指令。 */
        if (ChassisControl_Update(
                (float)CHASSIS_TASK_PERIOD_MS * 0.001f,
                &setpoint) == 0U)
        {
            ChassisControl_GetState(&state);
            chassis_debug_state = state;

            if ((state.mode == CHASSIS_CONTROL_FAULT) &&
                (fault_handled == 0U))
            {
                (void)ChassisTask_StopDrivers(&config, 0U);
                fault_handled = 1U;
            }
        }
        else
        {
            ChassisControl_GetState(&state);
            chassis_debug_state = state;

            if (state.mode == CHASSIS_CONTROL_FAULT)
            {
                if (fault_handled == 0U)
                {
                    (void)ChassisTask_StopDrivers(&config, 0U);
                    fault_handled = 1U;
                }
            }
            else
            {
                fault_handled = 0U;
                if (ChassisTask_SendVelocity(&setpoint) != HAL_OK)
                {
                    ChassisControl_FaultStop();
                    (void)ChassisTask_StopDrivers(&config, 0U);
                    fault_handled = 1U;
                }
            }
        }

        /* 使用绝对唤醒时间，避免单次 CAN 发送耗时造成周期漂移。 */
        next_wake += CHASSIS_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}
