/*
  程序说明: 每 20 ms 更新底盘闭环和四轮输出；初始化由主任务统一调用。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "chassis_task.h"

#include <string.h>

#include "cmsis_os.h"

#include "X_V2.h"
#include "zdt_can.h"
#include "ops9.h"
#include "hw101.h"

/*
 * 底盘主循环采用 20 ms 周期：读取控制器输出、生成四轮 F6 指令，
 * 再通过一条多电机命令发送。实际位置真值来自 OPS9/HW101，
 * 不使用 ID5~8 的电机状态反馈参与底盘控制。
 */
#define CHASSIS_TASK_PERIOD_MS             (20U)
#define CHASSIS_STARTUP_DELAY_MS          (100U)
#define CHASSIS_HEARTBEAT_TIMEOUT_MS      (300U)

volatile ChassisControlState_t chassis_debug_state;
static ChassisControlConfig_t task_config;
static volatile ChassisTaskStartupState_t chassis_startup_state =
    CHASSIS_TASK_STARTING;
/* Keil Watch：用于判断 CAN 发送或任务调度是否把 20 ms 控制周期拉长。 */
volatile uint32_t chassis_debug_cycle_ms;
volatile uint32_t chassis_debug_cycle_max_ms;
volatile uint32_t chassis_debug_cycle_overrun_count;
volatile uint32_t chassis_debug_send_ms;
volatile uint32_t chassis_debug_send_max_ms;

/*
  函数说明: 清除四个底盘电机的旧状态，配置回传、使能与心跳。
  调用示例: ChassisTask_ConfigureDrivers(config);
  参数意义: config 为对应模块的配置结构。
*/
static HAL_StatusTypeDef ChassisTask_ConfigureDrivers(
    const ChassisControlConfig_t *config)
{
    static const uint8_t feedback_codes[4] = {
        X_V2_INFO_REALTIME_SPEED,
        X_V2_INFO_REALTIME_POSITION,
        X_V2_INFO_POSITION_ERROR,
        X_V2_INFO_STATUS
    };
    uint8_t index;
    uint8_t info_index;

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

    /* 关闭旧配置可能仍在输出的 35/36/37/3A 定时反馈；0 ms 表示停止。 */
    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        for (info_index = 0U; info_index < 4U; ++info_index)
        {
            if (X_V2_Timed_Return_Info(config->motor_addr[index],
                                       feedback_codes[info_index],
                                       0U) != HAL_OK)
            {
                return HAL_ERROR;
            }
            osDelay(2U);
        }
    }

    return HAL_OK;
}

/*
  函数说明: 依次停止四个底盘电机，可选择同时失能。
  调用示例: ChassisTask_StopDrivers(config, disable);
  参数意义: config 为对应模块的配置结构。
            disable 为非零表示停止后同时失能。
*/
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

/*
  函数说明: 将控制器四轮目标打包为一组速度命令并发送。
  调用示例: ChassisTask_SendVelocity(setpoint);
  参数意义: setpoint 为四轮带符号目标转速，RPM。
*/
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

/*
  函数说明: 读取底盘驱动器初始化状态，供主任务决定是否接受动作。
  调用示例: ChassisTask_GetStartupState();
  参数意义: 无。
*/
ChassisTaskStartupState_t ChassisTask_GetStartupState(void)
{
    return chassis_startup_state;
}

/*
  函数说明: 等待主任务初始化完成后，每 20 ms 执行底盘闭环和四轮发令。
  调用示例: 由 CubeMX 创建的 StartChassisTask 调用 ChassisTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void ChassisTask_Run(void *argument)
{
    ChassisControlConfig_t config;
    ChassisWheelSetpoint_t setpoint;
    ChassisControlState_t state;
    uint32_t next_wake;
    uint32_t previous_cycle_ms = 0U;
    uint32_t cycle_now_ms;
    uint32_t send_start_ms;
    uint8_t fault_handled;
    uint8_t update_ok;
    uint32_t receive_retry_ms = 0U;

    (void)argument;
    while (chassis_startup_state != CHASSIS_TASK_READY) osDelay(20U);
    config = task_config;
    fault_handled = 0U;
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        cycle_now_ms = HAL_GetTick();
        if ((uint32_t)(cycle_now_ms - receive_retry_ms) >= 500U)
        {
            OPS9_Data_t ops;
            HW101_Data_t hw;
            (void)OPS9_GetLatest(&ops);
            (void)HW101_GetData(&hw);
            if (!ops.valid || (uint32_t)(cycle_now_ms - ops.last_update_ms) > 500U)
                (void)OPS9_StartReceive();
            if (!hw.valid || (uint32_t)(cycle_now_ms - hw.last_update_ms) > 500U)
                (void)HW101_StartReceive();
            receive_retry_ms = cycle_now_ms;
        }
        if (previous_cycle_ms != 0U)
        {
            chassis_debug_cycle_ms = cycle_now_ms - previous_cycle_ms;
            if (chassis_debug_cycle_ms > chassis_debug_cycle_max_ms)
            {
                chassis_debug_cycle_max_ms = chassis_debug_cycle_ms;
            }
            if (chassis_debug_cycle_ms > CHASSIS_TASK_PERIOD_MS)
            {
                ++chassis_debug_cycle_overrun_count;
            }
        }
        previous_cycle_ms = cycle_now_ms;

        /* 控制器返回 0 时通常表示故障，立即发送停止指令。 */
        update_ok = ChassisControl_Update(
            (float)CHASSIS_TASK_PERIOD_MS * 0.001f, &setpoint);
        /* 无论本次更新是否成功，状态快照和故障停车统一在这里处理。 */
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
        else if (update_ok != 0U)
        {
            fault_handled = 0U;
            send_start_ms = HAL_GetTick();
            if (ChassisTask_SendVelocity(&setpoint) != HAL_OK)
            {
                ChassisControl_FaultStopWithReason(CHASSIS_FAULT_CAN_TX);
                (void)ChassisTask_StopDrivers(&config, 0U);
                fault_handled = 1U;
            }
            chassis_debug_send_ms = HAL_GetTick() - send_start_ms;
            if (chassis_debug_send_ms > chassis_debug_send_max_ms)
            {
                chassis_debug_send_max_ms = chassis_debug_send_ms;
            }
        }

        /* 使用绝对唤醒时间，避免单次 CAN 发送耗时造成周期漂移。 */
        next_wake += CHASSIS_TASK_PERIOD_MS;
        if ((int32_t)(next_wake - osKernelGetTickCount()) <= 0)
            next_wake = osKernelGetTickCount() + CHASSIS_TASK_PERIOD_MS;
        osDelayUntil(next_wake);
    }
}


/*
  函数说明: 初始化底盘控制器、OPS9/HW101 接收及四个电机；由 main_task 调用。
  调用示例: ChassisTask_Init();
  参数意义: 无。
*/
uint8_t ChassisTask_Init(void)
{
    if (chassis_startup_state == CHASSIS_TASK_READY) return 1U;
    ChassisControl_LoadDefaultConfig(&task_config);
    ChassisControl_Init(&task_config);
    (void)OPS9_StartReceive();
    (void)HW101_StartReceive();
    osDelay(CHASSIS_STARTUP_DELAY_MS);
    if (ChassisTask_ConfigureDrivers(&task_config) != HAL_OK)
    {
        ChassisControl_FaultStopWithReason(CHASSIS_FAULT_INITIALIZATION);
        ChassisTask_StopDrivers(&task_config, 1U);
        chassis_startup_state = CHASSIS_TASK_FAULT;
        return 0U;
    }
    chassis_startup_state = CHASSIS_TASK_READY;
    return 1U;
}
