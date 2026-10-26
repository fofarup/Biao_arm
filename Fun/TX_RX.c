/*
  程序说明: USART6 上位机 12 字节协议、CRC16/MODBUS、命令队列与完成/错误应答。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#include "TX_RX.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>


#define TX_RX_DMA_SIZE       (256U)
#define TX_RX_QUEUE_SLOTS    (5U)
#define TX_RX_TX_TIMEOUT_MS  (30U)
#define TX_RX_REPLY_ATTEMPTS (3U)
#define TX_RX_REPLY_RETRY_MS (5U)

/* DMA 持续写入此数组；upper_dma_position 是上次已经处理到的位置。 */
static uint8_t upper_dma_buffer[TX_RX_DMA_SIZE];
/* 逐字节组帧的临时空间，帧尾和 CRC 均正确才入队。 */
static uint8_t upper_frame_buffer[TX_RX_FRAME_SIZE];
static uint8_t upper_frame_queue[TX_RX_QUEUE_SLOTS]
                                [TX_RX_FRAME_SIZE];
static uint16_t upper_dma_position;
static uint8_t upper_frame_length;
/* head 由 USART6 中断写、main_task 读；tail 由 main_task 写、中断读。 */
static volatile uint8_t upper_queue_head;
static volatile uint8_t upper_queue_tail;
/* 出错时先清 started，由独立软件定时器重启 DMA 接收。 */
static volatile uint8_t upper_receive_started;
/* main_task 完成初始化前保持 0，防止上电期间收到的动作立即执行。 */
static volatile uint8_t upper_accept_commands;
static TX_RX_Command_t upper_last_command;
static uint8_t upper_last_command_valid;
/* 主任务动作应答和按键定时器共用 USART6；按整帧串行发送。 */
static osMutexId_t upper_tx_mutex;

volatile uint32_t tx_rx_bad_frames;
volatile uint32_t tx_rx_dropped_frames;
volatile uint32_t tx_rx_uart_errors;
volatile TX_RX_Diagnostics_t tx_rx_diagnostics;


/*
  函数说明: 读取高字节在前的无符号 16 位字段。
  调用示例: TX_RX_ReadU16(bytes);
  参数意义: bytes 为输入字节数组。
*/
static uint16_t TX_RX_ReadU16(const uint8_t *bytes)
{
    /* 协议采用大端序：例如 00 64 解为十进制 100。 */
    return (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
}

/*
  函数说明: 读取高字节在前的有符号 16 位字段，保留负坐标。
  调用示例: TX_RX_ReadI16(bytes);
  参数意义: bytes 为输入字节数组。
*/
static int16_t TX_RX_ReadI16(const uint8_t *bytes)
{
    uint16_t encoded = TX_RX_ReadU16(bytes);
    /* 手工符号扩展：例如 FF 9C 解为 -100。 */
    int32_t signed_value = (encoded >= 0x8000U)
                               ? ((int32_t)encoded - 65536L)
                               : (int32_t)encoded;

    return (int16_t)signed_value;
}


/*
  函数说明: 从字节流恢复 12 字节帧，验证 CRC 后写入命令队列；满队列丢弃新帧。
  调用示例: TX_RX_AcceptByte(byte);
  参数意义: byte 为新收到的一个字节。
*/
static void TX_RX_AcceptByte(uint8_t byte)
{
    uint8_t next_head;
    uint8_t next_start;

    if (upper_accept_commands == 0U)
    {
        /* 初始化期间直接丢弃字节和半帧，避免启用时拼出旧指令。 */
        upper_frame_length = 0U;
        return;
    }

    if (upper_frame_length == 0U)
    {
        /* 尚未进入帧时，仅 FF 可以开始一条新指令。 */
        if (byte != 0xFFU)
        {
            return;
        }
    }

    upper_frame_buffer[upper_frame_length++] = byte;
    if (upper_frame_length < TX_RX_FRAME_SIZE)
    {
        return;
    }

    if ((upper_frame_buffer[TX_RX_FRAME_SIZE - 1U] == 0xFEU) &&
        (TX_RX_ReadU16(&upper_frame_buffer[9]) == TX_RX_Crc16(upper_frame_buffer, 9U)))
    {
        ++tx_rx_diagnostics.valid_frames;
        tx_rx_diagnostics.last_valid_ms = HAL_GetTick();
        /* 先检查剩余槽位；满队列只丢新帧，不覆盖尚未执行的旧帧。 */
        next_head = (uint8_t)((upper_queue_head + 1U) %
                              TX_RX_QUEUE_SLOTS);
        if (next_head == upper_queue_tail)
        {
            ++tx_rx_dropped_frames;
        }
        else
        {
            memcpy(upper_frame_queue[upper_queue_head],
                   upper_frame_buffer,
                   TX_RX_FRAME_SIZE);
            /* 数据全部复制完成后，再公布新的 head 给任务读取。 */
            __DMB();
            upper_queue_head = next_head;
            ++tx_rx_diagnostics.queued_frames;
        }
        upper_frame_length = 0U;
        return;
    }

    /* 帧尾或 CRC 错误：在收到的 12 字节中寻找下一个 FF，重新对齐。 */
    ++tx_rx_bad_frames;
    for (next_start = 1U; next_start < TX_RX_FRAME_SIZE;
         ++next_start)
    {
        if (upper_frame_buffer[next_start] == 0xFFU)
        {
            break;
        }
    }
    if (next_start < TX_RX_FRAME_SIZE)
    {
        upper_frame_length =
            (uint8_t)(TX_RX_FRAME_SIZE - next_start);
        memmove(upper_frame_buffer,
                &upper_frame_buffer[next_start],
                upper_frame_length);
    }
    else
    {
        upper_frame_length = 0U;
    }
}

/*
  函数说明: 解析循环 DMA 缓冲区中指定的未读字节区间。
  调用示例: TX_RX_ReadDmaRange(begin, end);
  参数意义: begin 为DMA 读取起始下标，包含该字节。
            end 为DMA 读取结束下标，不包含该字节。
*/
static void TX_RX_ReadDmaRange(uint16_t begin, uint16_t end)
{
    uint16_t index;
    if (end > begin)
    {
        tx_rx_diagnostics.rx_bytes += (uint32_t)(end - begin);
        tx_rx_diagnostics.last_rx_ms = HAL_GetTick();
    }

    for (index = begin; index < end; ++index)
    {
        TX_RX_AcceptByte(upper_dma_buffer[index]);
    }
}

/*
  函数说明: HAL 在 USART6 空闲、DMA 半满或全满时回调。
  循环 DMA 模式下 size 表示 DMA 当前写入位置，不是本次新字节数；
  因此只能处理 upper_dma_position 到 size 之间尚未读过的字节。
  size 变小时说明缓冲区回卷，先处理末尾，再从下标 0 处理到 size。
  调用示例: TX_RX_OnRxEvent(huart, size);
  参数意义: huart 为HAL UART 句柄。
            size 为DMA 当前写入位置，不是本次新增字节数。
*/
void TX_RX_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart != &huart6) || (upper_receive_started == 0U))
    {
        return;
    }

    ++tx_rx_diagnostics.rx_events;
    if (size > TX_RX_DMA_SIZE)
    {
        /* 异常位置不再访问缓冲区，交给任务周期性重启接收。 */
        ++tx_rx_bad_frames;
        upper_receive_started = 0U;
        tx_rx_diagnostics.rx_active = 0U;
        return;
    }

    if (size > upper_dma_position)
    {
        TX_RX_ReadDmaRange(upper_dma_position, size);
    }
    else if (size < upper_dma_position)
    {
        TX_RX_ReadDmaRange(upper_dma_position,
                                    TX_RX_DMA_SIZE);
        TX_RX_ReadDmaRange(0U, size);
    }
    upper_dma_position = size;
}

/*
  函数说明: 标记 USART6 接收异常并清理半帧状态，等待维护定时器恢复。
  调用示例: TX_RX_OnError();
  参数意义: 无。
*/
void TX_RX_OnError(void)
{
    ++tx_rx_uart_errors;
    tx_rx_diagnostics.last_rx_status = HAL_ERROR;
    tx_rx_diagnostics.last_uart_error = huart6.ErrorCode;
    if (huart6.hdmarx != NULL) tx_rx_diagnostics.last_dma_error = huart6.hdmarx->ErrorCode;
    upper_receive_started = 0U;
    tx_rx_diagnostics.rx_active = 0U;
    upper_frame_length = 0U;
}


/*
  函数说明: 在任务中清理异常接收并重建循环 DMA，避开异步终止回调缺失造成的反复启动失败。
  只暂时屏蔽 USART6/RX DMA 中断；HAL 的 DMA 停止/初始化各有 5 ms 超时。
  调用示例: TX_RX_StartReceive();
  参数意义: 无。
*/
static void TX_RX_StartReceive(void)
{
    HAL_StatusTypeDef status;
    DMA_HandleTypeDef *dma = huart6.hdmarx;
    upper_receive_started = 0U;
    tx_rx_diagnostics.rx_active = 0U;
    if (dma == NULL)
    {
        tx_rx_diagnostics.last_rx_status = HAL_ERROR;
        ++tx_rx_diagnostics.rx_restart_errors;
        return;
    }
    /* 避免恢复过程中旧 RX/终止回调又改写 HAL 状态；TX 发送状态不做重置。 */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    status = HAL_UART_AbortReceive(&huart6);
    /* DMAR 已清除而 DMA 仍忙时，UART 的 AbortReceive 不会处理这个残留状态。 */
    if (status == HAL_OK && dma->State == HAL_DMA_STATE_BUSY)
        status = HAL_DMA_Abort(dma);
    if (status == HAL_OK) status = HAL_DMA_DeInit(dma);
    if (status == HAL_OK) status = HAL_DMA_Init(dma);
    upper_dma_position = 0U;
    upper_frame_length = 0U;
    HAL_NVIC_ClearPendingIRQ(USART6_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    if (status == HAL_OK)
    {
        /* 恢复接收使能；不改波特率、引脚、发送状态或 DMA 的 CubeMX 配置。 */
        ATOMIC_SET_BIT(huart6.Instance->CR1, USART_CR1_UE | USART_CR1_RE);
        status = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, upper_dma_buffer, TX_RX_DMA_SIZE);
        if (status == HAL_OK)
        {
            upper_receive_started = 1U;
            tx_rx_diagnostics.rx_active = 1U;
            ++tx_rx_diagnostics.rx_start_count;
        }
    }
    tx_rx_diagnostics.last_rx_status = status;
    if (status != HAL_OK)
    {
        ++tx_rx_diagnostics.rx_restart_errors;
        tx_rx_diagnostics.last_uart_error = huart6.ErrorCode;
        tx_rx_diagnostics.last_dma_error = dma->ErrorCode;
    }
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}



/*
  函数说明: 切换业务命令接收开关并清空旧队列，防止初始化期间积压动作。
  调用示例: TX_RX_SetAccepting(enabled);
  参数意义: enabled 为非零表示启用，零表示关闭。
*/
void TX_RX_SetAccepting(uint8_t enabled)
{
    taskENTER_CRITICAL();
    upper_accept_commands = enabled != 0U;
    upper_frame_length = 0U;
    upper_queue_tail = upper_queue_head;
    taskEXIT_CRITICAL();
}


/*
  函数说明: 在 main_task 中调用；每次最多从队列取一条完整帧。
  调用示例: if (TX_RX_PopCommand(&command)) { 执行业务函数(); }
  参数意义: command 为单条命令结构。
*/
uint8_t TX_RX_PopCommand(TX_RX_Command_t *command)
{
    const uint8_t *frame;
    uint8_t tail;

    if (command == NULL)
    {
        return 0U;
    }

    tail = upper_queue_tail;
    if (tail == upper_queue_head)
    {
        return 0U;
    }

    /* 确认中断已经复制完当前槽位，再按协议偏移解码。 */
    __DMB();
    frame = upper_frame_queue[tail];
    command->mode = TX_RX_ReadU16(&frame[1]);
    command->x = TX_RX_ReadI16(&frame[3]);
    command->y = TX_RX_ReadI16(&frame[5]);
    command->z = TX_RX_ReadI16(&frame[7]);
    /* 槽位读完后才推进 tail，允许中断复用这个槽位。 */
    __DMB();
    upper_queue_tail =
        (uint8_t)((tail + 1U) % TX_RX_QUEUE_SLOTS);

    upper_last_command = *command;
    upper_last_command_valid = 1U;
    ++tx_rx_diagnostics.popped_frames;
    tx_rx_diagnostics.last_pop_ms = HAL_GetTick();
    return 1U;
}

/*
  函数说明: 复制最近一次合法接收命令，供调试观察。
  调用示例: TX_RX_GetLastCommand(command);
  参数意义: command 为单条命令结构。
*/
uint8_t TX_RX_GetLastCommand(TX_RX_Command_t *command)
{
    /* 方便在任务上下文查看最近取走的命令；不代表动作执行成功。 */
    if ((command == NULL) || (upper_last_command_valid == 0U))
    {
        return 0U;
    }
    *command = upper_last_command;
    return 1U;
}

/*
  函数说明: 按桌面版协议补齐帧尾与 CRC，CRC 覆盖下标 0～8，结果高字节在前。
  调用示例: TX_RX_FinishFrame(frame);
  参数意义: frame 为已填写帧头、功能码和 XYZ 的 12 字节缓冲区。
*/
static void TX_RX_FinishFrame(uint8_t *frame)
{
    uint16_t crc = TX_RX_Crc16(frame, 9U);
    frame[9] = (uint8_t)(crc >> 8U);
    frame[10] = (uint8_t)crc;
    frame[11] = 0xFEU;
}

/*
  函数说明: 按整帧持锁发送；动作应答最多尝试 3 次，启动事件只尝试一次。
  只在尚未写出完整帧时重试，防止重复完成帧被上位机当成下一条动作完成。
  调用示例: status = TX_RX_TransmitFrame(frame, 0U);
  参数意义: frame 为待发送的 12 字节完整帧；返回 HAL 发送状态。
            wait_ticks 为等待互斥锁的 RTOS tick 数，0 表示不等待。
*/
static HAL_StatusTypeDef TX_RX_TransmitFrame(uint8_t *frame, uint32_t wait_ticks)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t attempt;
    uint8_t locked = 0U;
    uint8_t limit = wait_ticks == 0U ? 1U : TX_RX_REPLY_ATTEMPTS;
    if (upper_tx_mutex != NULL)
    {
        for (attempt = 0U; attempt < limit; ++attempt)
        {
            if (!locked)
                locked = osMutexAcquire(upper_tx_mutex, wait_ticks) == osOK;
            status = HAL_BUSY;
            if (locked)
            {
                ++tx_rx_diagnostics.tx_attempts;
                status = HAL_UART_Transmit(&huart6, frame, TX_RX_FRAME_SIZE, TX_RX_TX_TIMEOUT_MS);
                if (status == HAL_OK) break;
                if (status != HAL_BUSY &&
                    (huart6.TxXferCount == 0U || huart6.TxXferCount > TX_RX_FRAME_SIZE))
                {
                    ++tx_rx_diagnostics.tx_uncertain_frames;
                    break;
                }
            }
            if (attempt + 1U >= limit) break;
            /* 只终止发送状态，USART6 的循环接收继续运行。 */
            if (locked && HAL_UART_AbortTransmit(&huart6) != HAL_OK) break;
            ++tx_rx_diagnostics.tx_retries;
            osDelay(TX_RX_REPLY_RETRY_MS);
        }
    }
    /* 与按键定时器的失败统计串行，Watch 留下本次逻辑帧的最终发送结果。 */
    taskENTER_CRITICAL();
    tx_rx_diagnostics.last_tx_status = status;
    tx_rx_diagnostics.last_tx_mode = TX_RX_ReadU16(&frame[1]);
    tx_rx_diagnostics.last_tx_ms = HAL_GetTick();
    if (status == HAL_OK) ++tx_rx_diagnostics.tx_frames;
    else ++tx_rx_diagnostics.tx_failures;
    taskEXIT_CRITICAL();
    if (locked) (void)osMutexRelease(upper_tx_mutex);
    return status;
}


/*
  函数说明: 上报 0020 启动事件；非阻塞获取串口锁，避免与动作应答交叉发送。
  调用示例: status = TX_RX_SendStart(TX_RX_START_AREA_1);
  参数意义: area 为启停区 1 或 2，写入 X，Y/Z 为 0。
            返回 HAL_OK 表示已发送；HAL_BUSY 表示尚未开始，可稍后重试；其他值表示发送失败。
            仅在任务环境调用，不能在中断中调用。
*/
HAL_StatusTypeDef TX_RX_SendStart(uint8_t area)
{
    uint8_t frame[TX_RX_FRAME_SIZE] = {0U};
    if (area != TX_RX_START_AREA_1 && area != TX_RX_START_AREA_2) return HAL_ERROR;
    frame[0] = 0xFFU;
    frame[1] = (uint8_t)(TX_RX_EVENT_START >> 8U);
    frame[2] = (uint8_t)TX_RX_EVENT_START;
    frame[4] = area;
    TX_RX_FinishFrame(frame);
    return TX_RX_TransmitFrame(frame, 0U);
}

/*
  函数说明: 发送桌面版 10FF 机械臂错误帧，串口发送成功返回 1；仅供任务上下文调用。
  调用示例: TX_RX_SendArmError(status, recovery_status, ready);
  参数意义: status 为动作错误码，与 ArmControlStatus 的数值一致。
            recovery_status 为最近一次动作的位置恢复状态，未尝试恢复时为 0。
            ready 为非零表示机械臂仍可接受动作，零表示未就绪。
*/
uint8_t TX_RX_SendArmError(uint16_t status, uint16_t recovery_status, uint8_t ready)
{
    uint8_t frame[TX_RX_FRAME_SIZE] = {0U};
    frame[0] = 0xFFU;
    frame[1] = (uint8_t)(TX_RX_ERROR_ARM >> 8U);
    frame[2] = (uint8_t)TX_RX_ERROR_ARM;
    frame[3] = (uint8_t)(status >> 8U);
    frame[4] = (uint8_t)status;
    frame[5] = (uint8_t)(recovery_status >> 8U);
    frame[6] = (uint8_t)recovery_status;
    frame[8] = ready != 0U ? 1U : 0U;
    TX_RX_FinishFrame(frame);
    return TX_RX_TransmitFrame(frame, TX_RX_TX_TIMEOUT_MS) == HAL_OK ? 1U : 0U;
}

/*
  函数说明: 发送底盘或机械臂/夹爪完成帧；串口发送成功返回 1，未定义的应答码返回 0。
  调用示例: TX_RX_SendCompletion(TX_RX_ACK_ARM); 由主任务在机械臂到位或夹爪发送成功后等待 300 ms 调用。
  参数意义: ack_mode 为应答功能码，仅允许 0011、0012；XYZ 全填 0。
*/
uint8_t TX_RX_SendCompletion(uint16_t ack_mode)
{
    uint8_t frame[TX_RX_FRAME_SIZE] = {0U};
    if (ack_mode != TX_RX_ACK_CHASSIS && ack_mode != TX_RX_ACK_ARM) return 0U;
    frame[0] = 0xFFU;
    frame[1] = (uint8_t)(ack_mode >> 8U);
    frame[2] = (uint8_t)ack_mode;
    /* 完成应答的 XYZ 均为 0；CRC 包含 FF，不包含 CRC 自身和 FE。 */
    TX_RX_FinishFrame(frame);
    return TX_RX_TransmitFrame(frame, TX_RX_TX_TIMEOUT_MS) == HAL_OK ? 1U : 0U;
}



/*
  函数说明: 计算 CRC16/MODBUS：初值 FFFF，反射多项式 A001，最终异或 0000。
  调用示例: crc = TX_RX_Crc16(frame, 9U); 自定义协议发送时 CRC 高字节在前。
  参数意义: data 为数据缓冲区或传感器快照。
            length 为参与 CRC 计算的字节数。
*/
uint16_t TX_RX_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;
    if (data == NULL) return crc;
    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}

/*
  函数说明: 每 50 ms 核对软件标志、UART 接收状态、DMA 状态及使能位，异常时重建接收。
  长时间没有主机数据属于正常空闲，不按无数据时长重置或禁止命令。
  调用示例: TX_RX_Maintain(argument);
  参数意义: argument 为任务或定时器参数，当前未使用。
*/
static void TX_RX_Maintain(void *argument)
{
    DMA_HandleTypeDef *dma = huart6.hdmarx;
    uint8_t active;
    (void)argument;
    active = upper_receive_started && dma != NULL &&
        huart6.RxState == HAL_UART_STATE_BUSY_RX &&
        huart6.ReceptionType == HAL_UART_RECEPTION_TOIDLE &&
        dma->State == HAL_DMA_STATE_BUSY &&
        (dma->Instance->CR & DMA_SxCR_EN) != 0U &&
        (dma->Instance->CR & (DMA_SxCR_HTIE | DMA_SxCR_TCIE)) == (DMA_SxCR_HTIE | DMA_SxCR_TCIE) &&
        (huart6.Instance->CR1 & (USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE)) ==
            (USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE) &&
        (huart6.Instance->CR3 & (USART_CR3_DMAR | USART_CR3_EIE)) == (USART_CR3_DMAR | USART_CR3_EIE);
    if (!active)
    {
        if (upper_receive_started) ++tx_rx_diagnostics.rx_health_faults;
        TX_RX_StartReceive();
    }
}


/*
  函数说明: 创建发送互斥锁、接收维护定时器并启动 USART6 DMA；初始化时不接受业务动作。
  调用示例: TX_RX_Init();
  参数意义: 无。
*/
uint8_t TX_RX_Init(void)
{
    static osTimerId_t timer;
    if (timer != NULL) return 1U;
    if (upper_tx_mutex == NULL) upper_tx_mutex = osMutexNew(NULL);
    if (upper_tx_mutex == NULL) return 0U;
    timer = osTimerNew(TX_RX_Maintain, osTimerPeriodic, NULL, NULL);
    if (timer == NULL) return 0U;
    if (osTimerStart(timer, 50U) != osOK)
    {
        osTimerDelete(timer);
        timer = NULL;
        return 0U;
    }
    TX_RX_StartReceive();
    return 1U;
}

/*
  函数说明: 记录异步接收终止完成，使维护定时器能够重新启动 DMA。
  调用示例: TX_RX_OnAbortReceiveComplete();
  参数意义: 无。
*/
void TX_RX_OnAbortReceiveComplete(void)
{
    upper_receive_started = 0U;
    tx_rx_diagnostics.rx_active = 0U;
    upper_frame_length = 0U;
}

