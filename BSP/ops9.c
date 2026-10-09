#include "ops9.h"
#include "usart.h"
#include "hw101.h"

/*
 * OPS9 每约 5 ms 输出一帧，共 28 字节。
 * 这里使用单字节中断接收：中断只负责收字节和解析状态机，
 * 不在中断中打印串口或执行底盘控制。
 */
static uint8_t ops9_rx_byte;
static uint8_t ops9_payload[OPS9_DATA_LENGTH];
static uint8_t ops9_payload_index;
static uint8_t ops9_parser_state;
static volatile OPS9_Data_t ops9_latest;

/* 帧解析状态：等待帧头、接收数据、等待帧尾。 */
enum
{
    OPS9_WAIT_HEAD_0 = 0U,
    OPS9_WAIT_HEAD_1,
    OPS9_READ_PAYLOAD,
    OPS9_WAIT_TAIL_0,
    OPS9_WAIT_TAIL_1
};

/* 将 OPS9 的低字节优先数据还原为 STM32 的 float。 */
static float OPS9_DecodeFloatLE(const uint8_t *bytes)
{
    union
    {
        uint32_t u32;
        float f32;
    } value;

    value.u32 = ((uint32_t)bytes[0]) |
                ((uint32_t)bytes[1] << 8U) |
                ((uint32_t)bytes[2] << 16U) |
                ((uint32_t)bytes[3] << 24U);

    return value.f32;
}

/* 判断 float 是否为有限数，过滤 NaN 和正负无穷。 */
static uint8_t OPS9_IsFinite(float value)
{
    union
    {
        uint32_t u32;
        float f32;
    } bits;

    bits.f32 = value;

    /* IEEE754 指数全为 1 时表示 NaN 或无穷大。 */
    return ((bits.u32 & 0x7F800000UL) != 0x7F800000UL) ? 1U : 0U;
}

/* 记录一帧异常，但不修改上一帧合法测量值。 */
static void OPS9_RecordBadFrame(void)
{
    ops9_latest.bad_frame_count++;
}

/* 解析 24 字节数据区，并在全部字段合法后一次性提交。 */
static void OPS9_CommitPayload(void)
{
    OPS9_Data_t decoded;

    /* 数据顺序来自 OPS9 手册：航向、俯仰、横滚、X、Y、航向角速度。 */
    decoded.heading_deg =
        OPS9_DecodeFloatLE(&ops9_payload[0]);
    decoded.pitch_deg =
        OPS9_DecodeFloatLE(&ops9_payload[4]);
    decoded.roll_deg =
        OPS9_DecodeFloatLE(&ops9_payload[8]);
    decoded.x_mm =
        OPS9_DecodeFloatLE(&ops9_payload[12]);
    decoded.y_mm =
        OPS9_DecodeFloatLE(&ops9_payload[16]);
    decoded.heading_rate_dps =
        OPS9_DecodeFloatLE(&ops9_payload[20]);

    /* 任一 float 异常时丢弃整帧，保留上一帧数据。 */
    if ((OPS9_IsFinite(decoded.heading_deg) == 0U) ||
        (OPS9_IsFinite(decoded.pitch_deg) == 0U) ||
        (OPS9_IsFinite(decoded.roll_deg) == 0U) ||
        (OPS9_IsFinite(decoded.x_mm) == 0U) ||
        (OPS9_IsFinite(decoded.y_mm) == 0U) ||
        (OPS9_IsFinite(decoded.heading_rate_dps) == 0U))
    {
        OPS9_RecordBadFrame();
        return;
    }

    decoded.last_update_ms = HAL_GetTick();
    decoded.frame_count = ops9_latest.frame_count + 1U;
    decoded.bad_frame_count = ops9_latest.bad_frame_count;
    decoded.valid = 1U;

    /* 只在合法帧完整解析后更新共享数据，避免任务读取半帧。 */
    ops9_latest = decoded;
}

/* 每收到一个字节就调用一次的帧同步状态机。 */
static void OPS9_ParseByte(uint8_t byte)
{
    switch (ops9_parser_state)
    {
    case OPS9_WAIT_HEAD_0:
        /* 手册帧头为 0D 0A。 */
        if (byte == 0x0DU)
        {
            ops9_parser_state = OPS9_WAIT_HEAD_1;
        }
        break;

    case OPS9_WAIT_HEAD_1:
        if (byte == 0x0AU)
        {
            ops9_payload_index = 0U;
            ops9_parser_state = OPS9_READ_PAYLOAD;
        }
        else if (byte != 0x0DU)
        {
            /* 当前字节不是帧头第二字节，重新寻找帧头。 */
            ops9_parser_state = OPS9_WAIT_HEAD_0;
        }
        break;

    case OPS9_READ_PAYLOAD:
        ops9_payload[ops9_payload_index++] = byte;

        if (ops9_payload_index >= OPS9_DATA_LENGTH)
        {
            ops9_parser_state = OPS9_WAIT_TAIL_0;
        }
        break;

    case OPS9_WAIT_TAIL_0:
        /* 手册帧尾为 0A 0D。 */
        if (byte == 0x0AU)
        {
            ops9_parser_state = OPS9_WAIT_TAIL_1;
        }
        else
        {
            OPS9_RecordBadFrame();
            ops9_parser_state = OPS9_WAIT_HEAD_0;
        }
        break;

    case OPS9_WAIT_TAIL_1:
        if (byte == 0x0DU)
        {
            OPS9_CommitPayload();
        }
        else
        {
            OPS9_RecordBadFrame();
        }

        /* 一帧结束后重新寻找下一帧帧头。 */
        ops9_parser_state = OPS9_WAIT_HEAD_0;
        break;

    default:
        /* 状态异常时回到空闲状态，等待下一帧。 */
        ops9_parser_state = OPS9_WAIT_HEAD_0;
        break;
    }
}

/* 启动单字节 UART 接收。任务启动后调用一次即可。 */
HAL_StatusTypeDef OPS9_StartReceive(void)
{
    ops9_parser_state = OPS9_WAIT_HEAD_0;
    ops9_payload_index = 0U;
    ops9_latest.valid = 0U;

    return HAL_UART_Receive_IT(
        &huart3,
        &ops9_rx_byte,
        1U);
}

/* UART 出错后停止当前接收，清空解析状态，再重新接收。 */
void OPS9_ResetReceive(void)
{
    (void)HAL_UART_AbortReceive(&huart3);

    ops9_parser_state = OPS9_WAIT_HEAD_0;
    ops9_payload_index = 0U;

    (void)HAL_UART_Receive_IT(
        &huart3,
        &ops9_rx_byte,
        1U);
}

/* 在关中断保护下复制共享数据，避免任务读到更新一半的结构体。 */
uint8_t OPS9_GetLatest(OPS9_Data_t *data)
{
    uint32_t primask;
    uint8_t valid;

    if (data == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();

    __disable_irq();
    *data = ops9_latest;
    valid = ops9_latest.valid;

    if (primask == 0U)
    {
        __enable_irq();
    }

    return valid;
}

/* 根据最近合法帧时间判断 OPS9 链路是否仍然在线。 */
uint8_t OPS9_IsFresh(uint32_t timeout_ms)
{
    OPS9_Data_t data;
    uint32_t now;

    if (OPS9_GetLatest(&data) == 0U)
    {
        return 0U;
    }

    now = HAL_GetTick();

    return ((now - data.last_update_ms) <= timeout_ms)
               ? 1U
               : 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        /* OPS9 使用 USART3 */
        OPS9_ParseByte(ops9_rx_byte);

        (void)HAL_UART_Receive_IT(
            &huart3,
            &ops9_rx_byte,
            1U);
    }
    else if (huart == &huart2)
    {
        /* HW101 使用 USART2 */
        HW101_UART_RxCpltCallback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        OPS9_ResetReceive();
    }
    else if (huart == &huart2)
    {
        HW101_UART_ErrorCallback(huart);
    }
}
