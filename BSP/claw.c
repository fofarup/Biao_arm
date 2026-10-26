/*
  程序说明: FashionStar 单总线夹爪舵机，发送单圈绝对角度并使用极速模式，不读取反馈。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6；PB10/USART3_TX 接舵机总线，PD9/USART3_RX 接 OPS9。
  日    期: 2026-10-25
*/

#include "claw.h"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"

#define CLAW_COMMAND_SET_ANGLE       (0x08U)
#define CLAW_COMMAND_CONTENT_SIZE    (7U)
#define CLAW_COMMAND_FRAME_SIZE      (12U)
#define CLAW_UART_TX_TIMEOUT_MS      (50U)
#define CLAW_BUS_GAP_MS              (5U)

/* 仅保护本驱动的发送过程；其他模块不能直接向 USART3 写入调试文本等数据。 */
static uint8_t claw_tx_busy;
static uint8_t claw_last_tx_valid;
static uint32_t claw_last_tx_end_ms;

/*
  函数说明: 仅切换 PB10 的驱动状态；不重新初始化 USART3，不影响 PD9 接收 OPS9。
  参数意义: transmitting 非零时使用 USART3 复用推挽发送，零时切为输入高阻。
  单总线舵机可能在执行指令后自行应答，即使不读取应答，也要释放总线。
*/
static void Claw_SetTxPin(uint8_t transmitting)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = (transmitting != 0U) ? GPIO_MODE_AF_PP : GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/* 协议的两字节数据均为小端，与上位机 USART6 的大端字段分别处理。 */
static void Claw_WriteU16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

/*
  函数说明: 发出 0x08 单圈角度指令，时间字段固定为 0，使用舵机极速模式。
            发送完成后立即返回，不等待舵机运动结束。
  参数意义: angle_deg 为相对舵机内部零位的绝对角度，范围 -180～180 度。
  调用示例: Claw_SetAngle(90.0f);
  对应报文: 12 4C 08 07 00 84 03 00 00 00 00 F4（ID 0，90 度，极速模式）。
*/
HAL_StatusTypeDef Claw_SetAngle(float angle_deg)
{
    uint8_t frame[CLAW_COMMAND_FRAME_SIZE];
    uint8_t index;
    uint8_t checksum = 0U;
    float raw_angle;
    int16_t position;
    HAL_StatusTypeDef status;

    /* 参数错误时拒绝发令，不静默夹紧到另一角度；中断中也不允许等待串口。 */
    if (!isfinite(angle_deg) ||
        (angle_deg < CLAW_MIN_ANGLE_DEG) ||
        (angle_deg > CLAW_MAX_ANGLE_DEG) ||
        (__get_IPSR() != 0U))
    {
        return HAL_ERROR;
    }
    if ((huart3.Instance != USART3) ||
        (huart3.Init.BaudRate != CLAW_UART_BAUDRATE))
    {
        return HAL_ERROR;
    }

    /* 短临界区只领取发送权，不在关中断期间发送整帧或等待运动。 */
    taskENTER_CRITICAL();
    if (claw_tx_busy != 0U)
    {
        taskEXIT_CRITICAL();
        return HAL_BUSY;
    }
    claw_tx_busy = 1U;
    taskEXIT_CRITICAL();

    /* 舵机可能回显上一条写命令，帧间先让出总线；毫秒计数回绕仍可正常相减。 */
    while ((claw_last_tx_valid != 0U) &&
           ((uint32_t)(HAL_GetTick() - claw_last_tx_end_ms) < CLAW_BUS_GAP_MS))
    {
        if (osKernelGetState() == osKernelRunning)
        {
            (void)osDelay(1U);
        }
        else
        {
            HAL_Delay(1U);
        }
    }

    /* 角度直接乘 10，按最近的 0.1 度编码；负角度使用 int16 二进制补码。 */
    raw_angle = angle_deg * 10.0f;
    position = (int16_t)((raw_angle >= 0.0f) ?
                        (raw_angle + 0.5f) : (raw_angle - 0.5f));
    frame[0] = 0x12U;
    frame[1] = 0x4CU;
    frame[2] = CLAW_COMMAND_SET_ANGLE;
    frame[3] = CLAW_COMMAND_CONTENT_SIZE;
    frame[4] = CLAW_SERVO_ID;
    Claw_WriteU16(&frame[5], (uint16_t)position);
    /* FashionStar 官方 SDK 的 0x08 指令以 interval=0 请求极速运动。 */
    Claw_WriteU16(&frame[7], 0U);
    /* 功率字段为 0：按舵机已保存的功率保护阈值执行，不改写舵机参数。 */
    Claw_WriteU16(&frame[9], 0U);
    for (index = 0U; index < CLAW_COMMAND_FRAME_SIZE - 1U; ++index)
    {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    frame[CLAW_COMMAND_FRAME_SIZE - 1U] = checksum;

    /* USART3 的接收和发送状态独立；不终止 OPS9 接收，也不切换硬件半双工模式。 */
    if (huart3.gState != HAL_UART_STATE_READY)
    {
        status = HAL_BUSY;
    }
    else
    {
        Claw_SetTxPin(1U);
        status = HAL_UART_Transmit(&huart3, frame, (uint16_t)sizeof(frame),
                                   CLAW_UART_TX_TIMEOUT_MS);
        /* HAL_OK 已等到 TC，即最后停止位发完；错误/超时也释放 PB10，避免占线。 */
        Claw_SetTxPin(0U);
        claw_last_tx_end_ms = HAL_GetTick();
        claw_last_tx_valid = 1U;
    }

    taskENTER_CRITICAL();
    claw_tx_busy = 0U;
    taskEXIT_CRITICAL();
    return status;
}
