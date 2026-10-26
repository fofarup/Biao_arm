/*
  程序说明: 统一业务初始化、主机命令执行及可选跑图测试入口。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#ifndef MAIN_TASK_H
#define MAIN_TASK_H
#include <stdint.h>
/* 0 为上位机命令入口；1 为初始化后执行一次 Route_Run 跑图测试。 */
#define MAIN_TASK_RUN_ROUTE_TEST (0U)
/* 0001：X 向车右、Y 向车前；0004：两轴相对发令时车身向右偏 45 度。
 * X/Y 单位 mm，Z 均为逆时针相对转角，度；下列电机速度参数供两种命令共用。 */
#define MAIN_HOST_ACC_RPM_S (100.0f)
#define MAIN_HOST_DEC_RPM_S (150.0f)
#define MAIN_HOST_MAX_RPM (800.0f)
/* 夹爪采用极速模式；此处只控制发送成功到 0012 回包的等待，不限制运动速度。 */
#define MAIN_HOST_CLAW_REPLY_DELAY_MS (300U)
#define MAIN_HOST_CLAW_MIN_DEG (0)
#define MAIN_HOST_CLAW_MAX_DEG (180)
/* PD4 低电平有效；EXTI4 记录边沿，TIM7 每 5 ms 检查稳定 20 ms 的消抖结果。 */
#define MAIN_START_SAMPLE_PERIOD_MS (5U)
#define MAIN_START_DEBOUNCE_MS (20U)
#define MAIN_START_LONG_PRESS_MS (1000U)
#define MAIN_START_REPORT_PERIOD_MS (10U)
/* main_task_init_faults 中每一位对应一个初始化失败模块。 */
#define MAIN_INIT_FAULT_COMM (1U)
#define MAIN_INIT_FAULT_CAN (2U)
#define MAIN_INIT_FAULT_ULTRASONIC (4U)
#define MAIN_INIT_FAULT_CHASSIS (8U)
#define MAIN_INIT_FAULT_LEVEL (16U)
#define MAIN_INIT_FAULT_ARM (32U)
#define MAIN_INIT_FAULT_CLAW (64U)
#define MAIN_INIT_FAULT_START_BUTTON (128U)

typedef enum
{
    MAIN_COMMAND_WAITING = 0U,
    MAIN_COMMAND_TAKEN,
    MAIN_COMMAND_EXECUTING,
    MAIN_COMMAND_RETURNED,
    MAIN_COMMAND_REPLYING,
    MAIN_COMMAND_REPLIED,
    MAIN_COMMAND_REPLY_FAILED,
    MAIN_COMMAND_REJECTED,
    MAIN_COMMAND_IGNORED
} MainTaskCommandStage_t;

/* Watch 保留最后一条命令的阶段；空闲时不覆盖最后的回包结果。 */
typedef struct
{
    MainTaskCommandStage_t stage;
    uint32_t command_count;
    uint32_t loop_ms;
    uint32_t stage_ms;
    uint32_t action_start_ms;
    uint32_t action_end_ms;
    uint16_t mode;
    int16_t x, y, z;
    uint16_t arm_status;
    uint16_t recovery_status;
    uint8_t arm_ready;
    uint8_t action_success;
    uint8_t reply_success;
} MainTaskCommandState_t;
extern volatile MainTaskCommandState_t main_task_command_state;

/*
  函数说明: 按依赖顺序初始化模块，然后执行主机命令循环或显式启用的跑图测试。
  调用示例: 由 CubeMX 创建的 StartMainTask 调用 MainTask_Run(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
void MainTask_Run(void *argument);
/*
  函数说明: 在 EXTI4 双边沿中断中记录 PD4 电平和时刻，不等待、不发送串口。
  调用示例: MainTask_StartButtonOnEdge();
  参数意义: 无；初始化完成前直接返回。
*/
void MainTask_StartButtonOnEdge(void);
/*
  函数说明: 在 TIM7 中断内推进 PD4 消抖和长短按判定，只缓存事件，不发送串口。
  调用示例: MainTask_StartButtonTick(HAL_GetTick());
  参数意义: now_ms 为 HAL 毫秒时钟；初始化完成前直接返回。
*/
void MainTask_StartButtonTick(uint32_t now_ms);
extern volatile uint32_t main_task_init_faults;
extern volatile uint32_t main_task_command_errors;
extern volatile uint32_t main_start_button_dropped_events;
extern volatile uint32_t main_start_button_tx_errors;
#endif
