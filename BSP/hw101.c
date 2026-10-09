#include "hw101.h"
#include "usart.h"

#include <string.h>

/* WIT/HW101 角度数据帧类型 */
#define HW101_ANGLE_FRAME_ID    (0x53U)

/* USART2 单字节接收缓存 */
static uint8_t hw101_rx_byte;

/* 当前正在接收的数据帧 */
static uint8_t hw101_frame[HW101_FRAME_LENGTH];
static uint8_t hw101_frame_index;

/* 最近一次有效数据 */
static volatile HW101_Data_t hw101_latest;

/* 清空当前帧接收状态 */
static void HW101_ResetParser(void)
{
    hw101_frame_index = 0U;
}

/* 检查一帧数据的校验和 */
static uint8_t HW101_ChecksumOK(void)
{
    uint8_t index;
    uint8_t sum;

    sum = 0U;

    /*
     * 第 0～9 字节累加，
     * 第 10 字节是校验和。
     */
    for (index = 0U; index < 10U; ++index)
    {
        sum = (uint8_t)(sum + hw101_frame[index]);
    }

    return (sum == hw101_frame[10]) ? 1U : 0U;
}

/* 处理一帧完整数据 */
static void HW101_ProcessFrame(void)
{
    HW101_Data_t decoded;
    int16_t raw_yaw;
    uint16_t raw_temperature;

    /* 校验失败，丢弃当前帧 */
    if (HW101_ChecksumOK() == 0U)
    {
        hw101_latest.bad_frame_count++;
        return;
    }

    /*
     * 0x53 表示角度帧。
     * 其他帧，例如 0x51、0x52，暂时忽略。
     */
    if (hw101_frame[1] != HW101_ANGLE_FRAME_ID)
    {
        return;
    }

    /*
     * WIT 协议：
     * Yaw 数据位于第 6、7 字节，低字节在前。
     */
    raw_yaw = (int16_t)(((uint16_t)hw101_frame[7] << 8U) |
                         (uint16_t)hw101_frame[6]);

    raw_temperature = (uint16_t)(((uint16_t)hw101_frame[9] << 8U) |
                                  (uint16_t)hw101_frame[8]);

    decoded = hw101_latest;

    /*
     * 原始角度范围约为 -32768～32767，
     * 对应 -180～180 度。
     */
    decoded.yaw_deg =
        ((float)raw_yaw * 180.0f) / 32768.0f;

    /* 温度原始值单位为 0.01 摄氏度 */
    decoded.temperature_c =
        (float)raw_temperature / 100.0f;

    decoded.last_update_ms = HAL_GetTick();
    decoded.frame_count++;
    decoded.valid = 1U;

    /*
     * 一次性提交完整数据，避免任务读取到半帧。
     */
    hw101_latest = decoded;
}

/* 收到一个字节时调用 */
static void HW101_ParseByte(uint8_t byte)
{
    /*
     * 第一个字节必须是 0x55。
     * 如果不是，就继续等待下一字节。
     */
    if (hw101_frame_index == 0U)
    {
        if (byte != 0x55U)
        {
            return;
        }
    }

    hw101_frame[hw101_frame_index] = byte;
    hw101_frame_index++;

    /*
     * 收满 11 字节后处理。
     */
    if (hw101_frame_index >= HW101_FRAME_LENGTH)
    {
        HW101_ProcessFrame();
        HW101_ResetParser();
    }
}

/* 启动 USART2 的单字节接收中断 */
HAL_StatusTypeDef HW101_StartReceive(void)
{
    memset((void *)&hw101_latest, 0, sizeof(hw101_latest));
    HW101_ResetParser();

    return HAL_UART_Receive_IT(
        &huart2,
        &hw101_rx_byte,
        1U);
}

/* 获取最近一次有效数据 */
uint8_t HW101_GetData(HW101_Data_t *data)
{
    uint32_t primask;
    uint8_t valid;

    if (data == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();

    __disable_irq();

    *data = hw101_latest;
    valid = hw101_latest.valid;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return valid;
}

/* USART2 接收完成后的处理 */
void HW101_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    HW101_ParseByte(hw101_rx_byte);

    /*
     * 必须重新开启下一字节接收。
     */
    (void)HAL_UART_Receive_IT(
        &huart2,
        &hw101_rx_byte,
        1U);
}

/* USART2 出错后的处理 */
void HW101_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(&huart2);

    HW101_ResetParser();

    (void)HAL_UART_Receive_IT(
        &huart2,
        &hw101_rx_byte,
        1U);
}
