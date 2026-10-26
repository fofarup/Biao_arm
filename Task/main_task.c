/*
  程序说明: 统一业务初始化、主机命令执行及可选跑图测试入口，支持仅底盘模式。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#include "main_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "zdt_can.h"
#include "arm_control.h"
#include "arm_end_level_task.h"
#include "chassis_task.h"
#include "ultrasonic_task.h"
#include "claw.h"
#include "TX_RX.h"
#include "Route.h"
#include "ops9.h"
#include "hw101.h"
#include <math.h>

/* 1 为仅底盘模式，0 为整机模式；修改后重新编译下载。 */
#define MAIN_TASK_CHASSIS_ONLY (0U)
/* 上电定位稳定后才初始化底盘电机；未稳定时持续等待，不设置放行超时。 */
#define MAIN_CHASSIS_READY_POLL_MS (20U)           /* 启动检查周期，等待时让出 CPU。 */
#define MAIN_CHASSIS_STABLE_TIME_MS (500U)         /* 连续稳定窗口，单位毫秒。 */
#define MAIN_CHASSIS_STABLE_XY_MM (5.0f)           /* 窗口内 X/Y 极差合成距离上限，毫米。 */
#define MAIN_CHASSIS_STABLE_YAW_DEG (1.0f)         /* HW101 航向相对窗口首帧的偏移上限，度。 */
#define MAIN_CHASSIS_RX_RETRY_MS (500U)            /* 无新鲜数据时的接收重试间隔。 */

volatile uint32_t main_task_init_faults;
volatile uint32_t main_task_command_errors;
volatile ArmControlStatus main_task_arm_status;
volatile MainTaskCommandState_t main_task_command_state;

#define MAIN_START_QUEUE_SLOTS (8U)
static uint8_t main_start_queue[MAIN_START_QUEUE_SLOTS];
static volatile uint8_t main_start_head;
static volatile uint8_t main_start_tail;
static volatile uint8_t main_start_enabled;
/* EXTI4 记录最新边沿；TIM7 在很短的关中断区间内取走快照。 */
static volatile uint8_t main_start_edge_pending;
static volatile uint8_t main_start_edge_pressed;
static volatile uint32_t main_start_edge_ms;
/* 以下消抖状态仅由 TIM7 中断使用，初始化在 enabled 置 1 前完成。 */
static uint8_t main_start_candidate;
static uint8_t main_start_stable;
static uint8_t main_start_armed;
static uint8_t main_start_pressed;
static uint8_t main_start_long_reported;
static uint32_t main_start_sample_ms;
static uint32_t main_start_candidate_ms;
static uint32_t main_start_press_ms;
volatile uint32_t main_start_button_dropped_events;
volatile uint32_t main_start_button_tx_errors;

/*
  函数说明: 在 TIM7 中断中缓存一个启动区域事件，满队列丢弃新事件并计数。
  调用示例: MainTask_QueueStart(TX_RX_START_AREA_1);
  参数意义: area 为启停区 1 或 2；中断是唯一生产者，软件定时器是唯一消费者。
*/
static void MainTask_QueueStart(uint8_t area)
{
    uint8_t next = (uint8_t)((main_start_head + 1U) % MAIN_START_QUEUE_SLOTS);
    if (next == main_start_tail)
    {
        ++main_start_button_dropped_events;
        return;
    }
    main_start_queue[main_start_head] = area;
    __DMB();
    main_start_head = next;
}

/*
  函数说明: 在 EXTI4 双边沿中断中记录 PD4 电平和时刻，不等待、不发送串口。
  调用示例: MainTask_StartButtonOnEdge();
  参数意义: 无；初始化完成前直接返回。
*/
void MainTask_StartButtonOnEdge(void)
{
    if (!main_start_enabled) return;
    main_start_edge_pressed = HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_RESET;
    main_start_edge_ms = HAL_GetTick();
    __DMB();
    main_start_edge_pending = 1U;
}

/*
  函数说明: 在 TIM7 中断内推进 PD4 消抖和长短按判定，只缓存事件，不发送串口。
  调用示例: MainTask_StartButtonTick(HAL_GetTick());
  参数意义: now_ms 为 HAL 毫秒时钟；初始化完成前直接返回。
*/
void MainTask_StartButtonTick(uint32_t now_ms)
{
    uint32_t irq_state;
    uint32_t held_ms;
    if (!main_start_enabled ||
        (uint32_t)(now_ms - main_start_sample_ms) < MAIN_START_SAMPLE_PERIOD_MS) return;
    main_start_sample_ms = now_ms;
    /* 只保护电平和时间戳的复制，防止优先级更高的 EXTI4 在两次读取之间改写。 */
    irq_state = __get_PRIMASK();
    __disable_irq();
    if (main_start_edge_pending)
    {
        main_start_candidate = main_start_edge_pressed;
        main_start_candidate_ms = main_start_edge_ms;
        main_start_edge_pending = 0U;
    }
    __set_PRIMASK(irq_state);
    if (main_start_candidate != main_start_stable &&
        (uint32_t)(now_ms - main_start_candidate_ms) >= MAIN_START_DEBOUNCE_MS)
    {
        main_start_stable = main_start_candidate;
        if (main_start_stable)
        {
            if (main_start_armed)
            {
                main_start_pressed = 1U;
                main_start_long_reported = 0U;
                main_start_press_ms = main_start_candidate_ms;
            }
        }
        else
        {
            if (main_start_pressed && !main_start_long_reported)
            {
                /* 用释放边沿时刻计时，避免释放消抖使不足 1 s 的短按变成长按。 */
                held_ms = (uint32_t)(main_start_candidate_ms - main_start_press_ms);
                MainTask_QueueStart(held_ms <= MAIN_START_LONG_PRESS_MS ?
                    TX_RX_START_AREA_1 : TX_RX_START_AREA_2);
            }
            main_start_pressed = 0U;
            main_start_armed = 1U;
        }
    }
    /* 超过 1 s 且仍为低电平时上报一次；松开边沿等待消抖期间不触发长按。 */
    if (main_start_pressed && main_start_candidate && !main_start_long_reported &&
        (uint32_t)(now_ms - main_start_press_ms) > MAIN_START_LONG_PRESS_MS)
    {
        MainTask_QueueStart(TX_RX_START_AREA_2);
        main_start_long_reported = 1U;
    }
}

/*
  函数说明: 在软件定时器任务中发送启动事件，主任务执行动作时仍可响应按键。
  调用示例: MainTask_ReportStart(argument);
  参数意义: argument 未使用；串口忙时保留事件，发送失败时计数，不自动重复启动。
*/
static void MainTask_ReportStart(void *argument)
{
    uint8_t tail = main_start_tail;
    HAL_StatusTypeDef status;
    (void)argument;
    if (tail == main_start_head) return;
    __DMB();
    status = TX_RX_SendStart(main_start_queue[tail]);
    if (status == HAL_BUSY) return;
    if (status != HAL_OK) ++main_start_button_tx_errors;
    __DMB();
    main_start_tail = (uint8_t)((tail + 1U) % MAIN_START_QUEUE_SLOTS);
}

/*
  函数说明: 创建启动事件发送定时器，再开放按键中断处理；必须先稳定松开后才能触发。
  调用示例: ready = MainTask_StartButtonInit();
  参数意义: 无；成功返回 1，定时器资源不足返回 0。
*/

static uint8_t MainTask_StartButtonInit(void)
{
    static osTimerId_t timer;
    if (timer != NULL) return 1U;
    timer = osTimerNew(MainTask_ReportStart, osTimerPeriodic, NULL, NULL);
    if (timer == NULL) return 0U;
    if (osTimerStart(timer, MAIN_START_REPORT_PERIOD_MS) != osOK)
    {
        (void)osTimerDelete(timer);
        timer = NULL;
        return 0U;
    }
    main_start_candidate = HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) == GPIO_PIN_RESET;
    main_start_stable = 1U;
    main_start_sample_ms = HAL_GetTick();
    main_start_candidate_ms = main_start_sample_ms;
    __DMB();
    main_start_enabled = 1U;
    return 1U;
}

/*
  函数说明: 持续等待 OPS9/HW101 连续稳定更新；只在底盘电机初始化前调用。
  调用示例: MainTask_WaitPositionStable();
  参数意义: 无；未收到数据、断流、异常帧或坐标/航向波动过大时重新计时，不超时放行。
*/
static void MainTask_WaitPositionStable(void)
{
    OPS9_Data_t ops, baseline_ops;
    HW101_Data_t hw, baseline_hw;
    uint32_t now_ms, stable_start_ms = 0U;
    uint32_t retry_ms = HAL_GetTick();
    uint32_t ops_frame = 0U, hw_frame = 0U;
    uint32_t ops_update_ms = 0U, hw_update_ms = 0U;
    float min_x = 0.0f, max_x = 0.0f, min_y = 0.0f, max_y = 0.0f;
    float span_x, span_y, yaw_delta;
    uint8_t ops_fresh, hw_fresh, tracking = 0U;

    /* 此时底盘任务尚未就绪，只启动定位接收，不建立底盘停车保持目标。 */
    (void)OPS9_StartReceive();
    (void)HW101_StartReceive();
    for (;;)
    {
        ops_fresh = OPS9_GetLatest(&ops);
        hw_fresh = HW101_GetData(&hw);
        now_ms = HAL_GetTick();
        ops_fresh = ops_fresh && ops.valid &&
            (uint32_t)(now_ms - ops.last_update_ms) <= CHASSIS_SENSOR_FRESH_MS;
        hw_fresh = hw_fresh && hw.valid &&
            (uint32_t)(now_ms - hw.last_update_ms) <= CHASSIS_SENSOR_FRESH_MS;
        if (!ops_fresh || !hw_fresh || !isfinite(ops.x_mm) || !isfinite(ops.y_mm) ||
            !isfinite(hw.yaw_deg) || fabsf(hw.yaw_deg) > 180.1f)
        {
            tracking = 0U;
            /* 底盘任务尚未启动，由这里在无数据时重试接收；不反复重启正常的数据流。 */
            if ((uint32_t)(now_ms - retry_ms) >= MAIN_CHASSIS_RX_RETRY_MS)
            {
                if (!ops_fresh) (void)OPS9_StartReceive();
                if (!hw_fresh) (void)HW101_StartReceive();
                retry_ms = now_ms;
            }
        }
        else
        {
            if (tracking)
            {
                if (ops.x_mm < min_x) min_x = ops.x_mm;
                if (ops.x_mm > max_x) max_x = ops.x_mm;
                if (ops.y_mm < min_y) min_y = ops.y_mm;
                if (ops.y_mm > max_y) max_y = ops.y_mm;
                span_x = max_x - min_x;
                span_y = max_y - min_y;
                /* 航向跨越 +180/-180 度时按实际小角度差判断。 */
                yaw_delta = hw.yaw_deg - baseline_hw.yaw_deg;
                if (yaw_delta > 180.0f) yaw_delta -= 360.0f;
                if (yaw_delta < -180.0f) yaw_delta += 360.0f;
                if (ops.bad_frame_count != baseline_ops.bad_frame_count ||
                    hw.bad_frame_count != baseline_hw.bad_frame_count ||
                    (uint32_t)(ops.last_update_ms - ops_update_ms) > CHASSIS_SENSOR_FRESH_MS ||
                    (uint32_t)(hw.last_update_ms - hw_update_ms) > CHASSIS_SENSOR_FRESH_MS ||
                    span_x * span_x + span_y * span_y >
                        MAIN_CHASSIS_STABLE_XY_MM * MAIN_CHASSIS_STABLE_XY_MM ||
                    fabsf(yaw_delta) > MAIN_CHASSIS_STABLE_YAW_DEG)
                    tracking = 0U;
            }
            if (!tracking)
            {
                /* 用当前合法快照建立新窗口，坐标不要求为零。 */
                baseline_ops = ops;
                baseline_hw = hw;
                min_x = max_x = ops.x_mm;
                min_y = max_y = ops.y_mm;
                ops_frame = ops.frame_count;
                hw_frame = hw.frame_count;
                stable_start_ms = now_ms;
                tracking = 1U;
            }
            else if (ops.frame_count != ops_frame && hw.frame_count != hw_frame)
            {
                /* 两路都收到新帧才推进确认；重复读取缓存不能使等待结束。 */
                ops_frame = ops.frame_count;
                hw_frame = hw.frame_count;
                if ((uint32_t)(now_ms - stable_start_ms) >= MAIN_CHASSIS_STABLE_TIME_MS)
                    return;
            }
            ops_update_ms = ops.last_update_ms;
            hw_update_ms = hw.last_update_ms;
        }
        osDelay(MAIN_CHASSIS_READY_POLL_MS);
    }
}

/*
  函数说明: 定位稳定后初始化底盘，并持续等待现有就绪条件；重复调用不重新初始化。
  调用示例: ready = MainTask_WaitChassisReady();
  参数意义: 无；就绪返回 1，CAN 或底盘初始化故障返回 0；等待传感器不设置超时。
*/
static uint8_t MainTask_WaitChassisReady(void)
{
    if ((main_task_init_faults & (MAIN_INIT_FAULT_CAN | MAIN_INIT_FAULT_CHASSIS)) != 0U ||
        ChassisTask_GetStartupState() == CHASSIS_TASK_FAULT) return 0U;
    if (ChassisTask_GetStartupState() != CHASSIS_TASK_READY)
    {
        MainTask_WaitPositionStable();
        if (!ChassisTask_Init()) return 0U;
    }
    /* 底盘初始化会重启定位接收；确认重新收到新鲜数据后才允许本地动作。 */
    while (!ChassisControl_SensorsReady()) osDelay(MAIN_CHASSIS_READY_POLL_MS);
    return 1U;
}

/*
  函数说明: 按 MODE 执行 0001 普通或 0004 右偏 45 度的相对移动，共用主机速度参数。
  调用示例: success = MainTask_HandleChassisCommand(command);
  参数意义: command 为 0001/0004 底盘命令；成功表示实际到位。
*/
static uint8_t MainTask_HandleChassisCommand(const TX_RX_Command_t *command)
{
    if (ChassisTask_GetStartupState() != CHASSIS_TASK_READY ||
        !ChassisControl_SensorsReady()) return 0U;
    if (command->mode == TX_RX_MODE_CHASSIS_DIAGONAL)
    {
        /* 本条指令的坐标轴相对当前车身向右偏 45 度，Z 仍为真实相对转角。 */
        return ChassisControl_MoveDiagonalRelative(
            (float)command->x, (float)command->y, (float)command->z,
            MAIN_HOST_ACC_RPM_S, MAIN_HOST_DEC_RPM_S, MAIN_HOST_MAX_RPM);
    }
    /* 普通模式沿发令时车右/车前移动；不保留上一条指令的斜向模式。 */
    return ChassisControl_MoveRelativeSequential(
        (float)command->x, (float)command->y, (float)command->z,
        MAIN_HOST_ACC_RPM_S, MAIN_HOST_DEC_RPM_S, MAIN_HOST_MAX_RPM);
}


/*
  函数说明: 记录当前命令处理阶段和切换时刻，便于区分动作等待与回包失败。
  调用示例: MainTask_SetCommandStage(MAIN_COMMAND_EXECUTING);
  参数意义: stage 为 main_task_command_state.stage 的可读枚举值。
*/
static void MainTask_SetCommandStage(MainTaskCommandStage_t stage)
{
    main_task_command_state.stage_ms = HAL_GetTick();
    main_task_command_state.stage = stage;
}

/*
  函数说明: 执行桌面版主机命令；仅底盘模式拒绝机械臂命令并忽略夹爪命令。
  底盘及机械臂实际到位后回包，夹爪指令发送成功后等待 300 ms 回包，机械臂失败回 10FF。
  调用示例: MainTask_HandleCommand(command);
  参数意义: command 为单条命令结构。
*/
static void MainTask_HandleCommand(const TX_RX_Command_t *command)
{
    uint8_t success = 0U;
    uint8_t reply_ok = 0U;
    uint8_t arm_ready = 0U;
    uint16_t reply_mode = 0U;
    ArmEndLevelState_t level;
    if (command == NULL) return;
    ++main_task_command_state.command_count;
    main_task_command_state.mode = command->mode;
    main_task_command_state.x = command->x;
    main_task_command_state.y = command->y;
    main_task_command_state.z = command->z;
    main_task_command_state.action_start_ms = 0U;
    main_task_command_state.action_end_ms = 0U;
    main_task_command_state.action_success = 0U;
    main_task_command_state.reply_success = 0U;
    main_task_command_state.arm_status = (uint16_t)main_task_arm_status;
    main_task_command_state.recovery_status = 0U;
    main_task_command_state.arm_ready = 0U;
    MainTask_SetCommandStage(MAIN_COMMAND_TAKEN);
    /* 仅底盘模式继续拒绝机械臂，夹爪保持忽略；不触发任何机械臂初始化。 */
    if (MAIN_TASK_CHASSIS_ONLY &&
        (command->mode == TX_RX_MODE_ARM || command->mode == TX_RX_MODE_CLAW))
    {
        if (command->mode == TX_RX_MODE_ARM)
        {
            main_task_arm_status = ARM_CONTROL_NOT_INITIALIZED;
            reply_mode = TX_RX_ERROR_ARM;
        }
    }
    else switch (command->mode)
    {
    case TX_RX_MODE_IDLE:
        MainTask_SetCommandStage(MAIN_COMMAND_IGNORED);
        return;
    case TX_RX_MODE_CHASSIS:
    case TX_RX_MODE_CHASSIS_DIAGONAL:
        main_task_command_state.action_start_ms = HAL_GetTick();
        MainTask_SetCommandStage(MAIN_COMMAND_EXECUTING);
        success = MainTask_HandleChassisCommand(command);
        main_task_command_state.action_end_ms = HAL_GetTick();
        MainTask_SetCommandStage(MAIN_COMMAND_RETURNED);
        if (success) reply_mode = TX_RX_ACK_CHASSIS;
        break;
    case TX_RX_MODE_ARM:
        ArmEndLevelTask_GetState(&level);
        arm_ready = (main_task_init_faults & MAIN_INIT_FAULT_LEVEL) == 0U &&
            level.ready && level.control_enabled && !level.motor_fault && ArmControl_IsReady();
        main_task_command_state.arm_ready = arm_ready;
        if (arm_ready)
        {
            main_task_command_state.action_start_ms = HAL_GetTick();
            MainTask_SetCommandStage(MAIN_COMMAND_EXECUTING);
            /* 保持自动规划：同时启动，正常轴同步到位，小位移可提前到位。 */
            main_task_arm_status = ArmControl_MoveToXYZ(command->x, command->y, command->z);
            main_task_command_state.action_end_ms = HAL_GetTick();
            MainTask_SetCommandStage(MAIN_COMMAND_RETURNED);
            arm_ready = ArmControl_IsReady();
            success = arm_ready && main_task_arm_status == ARM_CONTROL_OK;
            if (!arm_ready) ArmEndLevelTask_SetControlEnabled(0U);
        }
        else if (main_task_arm_status == ARM_CONTROL_OK)
        {
            main_task_arm_status = level.motor_fault ? ARM_CONTROL_DRIVER_ERROR :
                ARM_CONTROL_NOT_INITIALIZED;
        }
        main_task_command_state.recovery_status = (uint16_t)ArmControl_GetRecoveryStatus();
        reply_mode = success ? TX_RX_ACK_ARM : TX_RX_ERROR_ARM;
        break;
    case TX_RX_MODE_CLAW:
        if (command->x >= MAIN_HOST_CLAW_MIN_DEG && command->x <= MAIN_HOST_CLAW_MAX_DEG &&
            command->y == 0 && command->z == 0)
        {
            main_task_command_state.action_start_ms = HAL_GetTick();
            MainTask_SetCommandStage(MAIN_COMMAND_EXECUTING);
            success = Claw_SetAngle((float)command->x) == HAL_OK;
            if (success)
            {
                /* 从完整指令发出后计时；只等待回包间隔，不控制舵机运动时长。 */
                uint32_t start_ms = HAL_GetTick();
                while ((uint32_t)(HAL_GetTick() - start_ms) < MAIN_HOST_CLAW_REPLY_DELAY_MS)
                    osDelay(1U);
                reply_mode = TX_RX_ACK_CLAW;
            }
            main_task_command_state.action_end_ms = HAL_GetTick();
            MainTask_SetCommandStage(MAIN_COMMAND_RETURNED);
        }
        break;
    default:
        MainTask_SetCommandStage(MAIN_COMMAND_IGNORED);
        ++main_task_command_errors;
        return;
    }
    main_task_command_state.action_success = success;
    main_task_command_state.arm_status = (uint16_t)main_task_arm_status;
    main_task_command_state.arm_ready = arm_ready;
    if (reply_mode != 0U)
    {
        MainTask_SetCommandStage(MAIN_COMMAND_REPLYING);
        if (reply_mode == TX_RX_ERROR_ARM)
            reply_ok = TX_RX_SendArmError((uint16_t)main_task_arm_status,
                main_task_command_state.recovery_status, arm_ready);
        else
            reply_ok = TX_RX_SendCompletion(reply_mode);
        main_task_command_state.reply_success = reply_ok;
        MainTask_SetCommandStage(reply_ok ? MAIN_COMMAND_REPLIED : MAIN_COMMAND_REPLY_FAILED);
    }
    else MainTask_SetCommandStage(MAIN_COMMAND_REJECTED);
    if (!success || !reply_ok) ++main_task_command_errors;
}


/*
  函数说明: 按所选模式初始化模块，等待底盘就绪后执行本地动作，再处理主机命令。
  调用示例: 由 CubeMX 创建的 StartMainTask 调用 MainTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void MainTask_Run(void *argument)
{
    TX_RX_Command_t command; /* 保存从上位机命令队列取出的模式及 X/Y/Z 参数。 */
    uint8_t can_ready;        /* CAN 接口初始化是否成功：1 为成功，0 为失败。 */
    (void)argument;           /* 当前不使用任务入口参数，显式忽略以避免编译警告。 */

    /* 初始化失败时，用 |= 将对应故障位置 1，同时保留其他模块已有的故障记录。 */
    /* 启动 USART6 上位机接收；此时暂不接受业务动作，后面再统一开放。 */
    if (!TX_RX_Init()) main_task_init_faults |= MAIN_INIT_FAULT_COMM;

    /* 初始化共用的 CAN 接口；返回 HAL_OK 时，比较结果为 1，否则为 0。 */
    can_ready = ZDT_CAN_Init() == HAL_OK;
    if (!can_ready) main_task_init_faults |= MAIN_INIT_FAULT_CAN;

    /* 初始化超声波驱动，后续由超声波任务周期测距。 */
    if (!UltrasonicTask_Init()) main_task_init_faults |= MAIN_INIT_FAULT_ULTRASONIC;

    /* 先将机械臂标记为未初始化；仅底盘模式下保持该状态。 */
    main_task_arm_status = ARM_CONTROL_NOT_INITIALIZED;

    /* CAN 失败时，|| 短路跳过底盘准备并记录故障。
     * CAN 成功时先持续等待定位稳定，再初始化四个底盘电机；等待多久都不提前放行。 */
    if (!can_ready || !MainTask_WaitChassisReady()) main_task_init_faults |= MAIN_INIT_FAULT_CHASSIS;

    /* 开关为 0 才初始化整机；为 1 时跳过机械臂、末端调平和夹爪。
     * 主动跳过的模块不记初始化故障，调平任务也因未初始化而不执行控制。 */
    if (!MAIN_TASK_CHASSIS_ONLY)
    {
        /* CAN 成功才初始化机械臂三个位置电机；否则直接记录 CAN 错误状态。 */
        main_task_arm_status = can_ready ? ArmControl_Init() : ARM_CONTROL_CAN_ERROR;
        /* 保留具体错误码，同时设置机械臂初始化故障位，供后续诊断和命令处理使用。 */
        if (main_task_arm_status != ARM_CONTROL_OK) main_task_init_faults |= MAIN_INIT_FAULT_ARM;

        /* CAN 成功才启动 JY901S、等待连续有效姿态帧并初始化 ID4 末端电机。
         * CAN 失败或调平初始化失败时，记录调平故障；CAN 失败时同样利用 || 跳过调用。 */
        if (!can_ready || !ArmEndLevelTask_Init()) main_task_init_faults |= MAIN_INIT_FAULT_LEVEL;
        /* 调平初始化成功后，仅在机械臂也已就绪时启用实时调平，否则保持关闭。 */
        else ArmEndLevelTask_SetControlEnabled(ArmControl_IsReady());

        /* 发出夹爪 0 度极速指令；发送失败时记录夹爪故障，本地调用不等待到位。 */
        if (Claw_SetAngle(0.0f) != HAL_OK) main_task_init_faults |= MAIN_INIT_FAULT_CLAW;
    }
    //osDelay(2000);
    /* 本地动作示例，按实际目标替换坐标后放入业务流程：
     * 末端角度功能需 MAIN_TASK_CHASSIS_ONLY 为 0，且末端初始化并启用保持成功。
     * if (!ArmEndLevelTask_MoveToAngle(-30.0f)) ++main_task_command_errors;
     * 上句实际到位后返回，后台持续保持 -30 度；允许范围 -90～+10 度。
     * if (!ArmEndLevelTask_MoveToAngle(0.0f)) ++main_task_command_errors; 恢复水平。
     * ArmControl_MoveToXYZ(x_mm, y_mm, z_mm);
     * ChassisControl_MoveRelativeSequential(100, 0, 0, 1000, 1500, 3000);
     * ChassisControl_MoveRelativeStraightTurn(100, 0, 90, 1000, 1500, 3000);
     * 行驶段沿当前车身右前方走 100 mm；到作业区后用普通车体坐标微调：
     * if (!ChassisControl_MoveDiagonalRelative(0, 100, 0, 100, 150, 800) ||
     *     !ChassisControl_MoveRelativeSequential(0, 50, 0, 100, 150, 800))
     *     ++main_task_command_errors;
     * Claw_SetAngle(90.0f);
     * Route_Run();
     * Route_RunFixedSequence(); 原 paotu_alone 的 14 段固定动作。
     */
//    if (MAIN_TASK_RUN_ROUTE_TEST)
//    {
//        if ((main_task_init_faults & (MAIN_INIT_FAULT_CAN | MAIN_INIT_FAULT_ULTRASONIC | MAIN_INIT_FAULT_CHASSIS)) == 0U)
//        {
//            if (MainTask_WaitChassisReady()) (void)Route_Run();
//        }
//        for (;;) osDelay(1000U);
//    }
    /* 定位未就绪时持续等待；初始化故障或动作失败时记录本次失败，后续命令仍检查就绪状态。 */
//    while(1){
//        ChassisControl_MoveRelativeSequential(-10, 0, 90, 1000, 1500, 3000);
//        ChassisControl_MoveRelativeSequential(0, 10, -90, 1000, 1500, 3000);
//				ChassisControl_MoveRelativeSequential(10, 0, 90, 1000, 1500, 3000);
//				ChassisControl_MoveRelativeSequential(0, -10, -90, 1000, 1500, 3000);
//		}
//  while(1)
//		{
//		 ArmControl_MoveToXYZ(0,300,250);
//			osDelay(1000);
//		 ArmControl_MoveToXYZ(130,130,220);//物料盘1
//			osDelay(1000);
//		 ArmControl_MoveToXYZ(-130,130,220);//物料盘2
//			osDelay(1000);
//		 ArmControl_MoveToXYZ(-130,-130,220);//物料盘3
//			osDelay(1000);
//		}			
   // ArmControl_MoveToXYZ(80,450,190);
		//osDelay(1000);
		//ArmControl_MoveToXYZ(170,300,100);//初加工右边
//		ArmControl_MoveToXYZ(-150,300,100);
//		ArmControl_MoveToXYZ(0,300,250);
//		ArmControl_MoveToXYZ(150,300,100);
//		Claw_SetAngle(0.0f);
    TX_RX_SetAccepting(1U);
//    if ((main_task_init_faults & MAIN_INIT_FAULT_COMM) != 0U || !MainTask_StartButtonInit())
//        main_task_init_faults |= MAIN_INIT_FAULT_START_BUTTON;
    //Route_RunFixedSequence();

		MainTask_SetCommandStage(MAIN_COMMAND_WAITING);
    for (;;)
    {
        main_task_command_state.loop_ms = HAL_GetTick();
        if (TX_RX_PopCommand(&command)) MainTask_HandleCommand(&command);
        else
        {
            /* 空闲期间仍维护 ID1～3 的 CAN 通信；故障或仅底盘模式不查询。 */
            if (!MAIN_TASK_CHASSIS_ONLY && ArmControl_IsReady()) ArmControl_ServiceIdle();
            osDelay(5U);
        }
    }
}

/*融和跑图*/
//	ChassisControl_MoveRelativeSequential(190.0f, 190.0f, 45.0f, 100, 150, 800);//开到路中间
//		ChassisControl_MoveDiagonalRelative(850.0f,   0.0f, 45.0f, 100, 150, 800); //扫码
//		
//		ChassisControl_MoveRelativeSequential(850.0f, 0.0f, -135.0f, 100, 150, 800);//十字路口
//		ChassisControl_MoveDiagonalRelative(0,950,-45,1000,1500,3000);//物料盘
//		ChassisControl_MoveRelativeSequential(0.0f,   0.0f, 135.0f, 100, 150, 800); 
//		ChassisControl_MoveDiagonalRelative(-1800.0f,0.0f, 45.0f, 100, 150, 800);//粗加工 
//		
//		ChassisControl_MoveRelativeSequential(0,-850,-90,100,150,800);
//		ChassisControl_MoveRelativeSequential(0,850,0,100,150,800);
//		ChassisControl_MoveRelativeSequential(0,-850,-45,100,150,800);
//		ChassisControl_MoveDiagonalRelative(0,950,-45,100,150,800);
//		
//		ChassisControl_MoveRelativeSequential(0.0f,   0.0f, 135.0f, 100, 150, 800);
//		ChassisControl_MoveDiagonalRelative(-1800.0f,0.0f, 45.0f, 100, 150, 800);
//		ChassisControl_MoveRelativeSequential(0,-850,-90,100,150,800);
//		ChassisControl_MoveRelativeSequential(0,850,0,100,150,800);
//		ChassisControl_MoveRelativeSequential(0,0,45,100,150,800);
//		ChassisControl_MoveDiagonalRelative(0,-1700,0,100,150,800);
//		ChassisControl_MoveDiagonalRelative(-850,0,45,100,150,800);
//		 
//    ChassisControl_MoveDiagonalRelative(-190.0f,  -190.0f,   0.0f, 100, 150, 800); 