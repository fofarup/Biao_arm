/*
  程序说明: CAN 初始化、完整命令互斥发送、接收队列及错误统计。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-23
*/

#include "zdt_can.h"
#include "can.h"
#include <string.h>

#define ZDT_CAN_TX_TIMEOUT_MS    20U
#define ZDT_CAN_RX_QUEUE_DEPTH   32U

/* 机械臂反馈使用接收队列；底盘不消费轮电机反馈，发送端共用互斥锁。 */
static osMessageQueueId_t zdt_can_rx_queue = NULL;
static osMutexId_t zdt_can_tx_mutex = NULL;
static volatile uint8_t zdt_can_initialized = 0U;

volatile uint32_t zdt_can_tx_count = 0;
volatile uint32_t zdt_can_tx_error_count = 0;
volatile uint32_t zdt_can_rx_count = 0;
volatile uint32_t zdt_can_rx_drop_count = 0;
volatile uint32_t zdt_can_error_count = 0;
volatile ZDT_CanRxFrame_t zdt_can_last_rx = {0};

/*
  函数说明: 在调度器运行时让出 1 ms，启动阶段使用 HAL 延时。
  调用示例: ZDT_CAN_Delay1ms();
  参数意义: 无。
*/
static void ZDT_CAN_Delay1ms(void)
{
    if (osKernelGetState() == osKernelRunning)
    {
        osDelay(1);
    }
    else
    {
        HAL_Delay(1);
    }
}

/*
  函数说明: 配置 CAN 过滤器、发送互斥锁与接收队列，并启动 CAN 通知。
  调用示例: ZDT_CAN_Init();
  参数意义: 无。
*/
HAL_StatusTypeDef ZDT_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    /* CAN 由 main_task、底盘任务及接收任务共用，初始化必须幂等。 */
    if (zdt_can_initialized != 0U)
    {
        return HAL_OK;
    }

    if (zdt_can_rx_queue == NULL)
    {
        zdt_can_rx_queue =
            osMessageQueueNew(ZDT_CAN_RX_QUEUE_DEPTH,
                              sizeof(ZDT_CanRxFrame_t),
                              NULL);

        if (zdt_can_rx_queue == NULL)
        {
            return HAL_ERROR;
        }
    }

    if (zdt_can_tx_mutex == NULL)
    {
        zdt_can_tx_mutex = osMutexNew(NULL);
        if (zdt_can_tx_mutex == NULL)
        {
            return HAL_ERROR;
        }
    }

    /*
     * 接收所有扩展数据帧：
     * FilterIdLow和MaskIdLow只检查IDE位。
     */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = CAN_ID_EXT;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = CAN_ID_EXT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING |
            CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE |
            CAN_IT_ERROR) != HAL_OK)
    {
        return HAL_ERROR;
    }

    zdt_can_initialized = 1U;
    return HAL_OK;
}

/*
  函数说明: 在有限等待内提交一帧 CAN 扩展帧，并检查发送结果。
  调用示例: ZDT_CAN_SendFrame(ext_id, data, dlc);
  参数意义: ext_id 为CAN 扩展标识符。
            data 为数据缓冲区或传感器快照。
            dlc 为CAN 数据区字节数。
*/
static HAL_StatusTypeDef ZDT_CAN_SendFrame(
    uint32_t ext_id,
    const uint8_t *data,
    uint8_t dlc)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    uint32_t start_tick;
    HAL_StatusTypeDef status;

    header.StdId = 0;
    header.ExtId = ext_id;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    start_tick = HAL_GetTick();

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= ZDT_CAN_TX_TIMEOUT_MS)
        {
            ++zdt_can_tx_error_count;
            return HAL_TIMEOUT;
        }

        ZDT_CAN_Delay1ms();
    }

    status = HAL_CAN_AddTxMessage(
        &hcan1,
        &header,
        (uint8_t *)data,
        &mailbox);

    if (status != HAL_OK)
    {
        ++zdt_can_tx_error_count;
        return status;
    }

    start_tick = HAL_GetTick();

    while (HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U)
    {
        if ((HAL_GetTick() - start_tick) >= ZDT_CAN_TX_TIMEOUT_MS)
        {
            HAL_CAN_AbortTxRequest(&hcan1, mailbox);
            ++zdt_can_tx_error_count;
            return HAL_TIMEOUT;
        }

        ZDT_CAN_Delay1ms();
    }

    ++zdt_can_tx_count;
    return HAL_OK;
}

/*
  函数说明: 按驱动器扩展帧格式拆包发送；调用者须已持有发送互斥锁。
  调用示例: ZDT_CAN_SendCmdUnlocked(cmd, len);
  参数意义: cmd 为驱动器协议字节数组。
            len 为输入字节数。
*/
static HAL_StatusTypeDef ZDT_CAN_SendCmdUnlocked(
    const uint8_t *cmd,
    uint8_t len)
{
    uint8_t offset = 0;
    uint8_t remaining;
    uint8_t packet_number = 0;
    uint8_t chunk;
    uint8_t tx_data[8];
    uint32_t ext_id;
    HAL_StatusTypeDef status;

    if ((cmd == NULL) || (len < 3U))
    {
        return HAL_ERROR;
    }

    /*
     * cmd[0]：电机地址
     * cmd[1]：功能码
     * cmd[2...]：命令参数
     */
    remaining = len - 2U;

    while (offset < remaining)
    {
        chunk = remaining - offset;

        if (chunk > 7U)
        {
            chunk = 7U;
        }

        memset(tx_data, 0, sizeof(tx_data));

        tx_data[0] = cmd[1];
        memcpy(&tx_data[1], &cmd[offset + 2U], chunk);

        ext_id = ((uint32_t)cmd[0] << 8) |
                 (uint32_t)packet_number;

        status = ZDT_CAN_SendFrame(
            ext_id,
            tx_data,
            chunk + 1U);

        if (status != HAL_OK)
        {
            return status;
        }

        offset += chunk;
        ++packet_number;
    }

    return HAL_OK;
}

/*
  函数说明: 用互斥锁保护一整条驱动器命令，避免不同任务的 CAN 分帧交错。
  调用示例: can_SendCmd(cmd, len);
  参数意义: cmd 为驱动器协议字节数组。
            len 为输入字节数。
*/
HAL_StatusTypeDef can_SendCmd(const uint8_t *cmd, uint8_t len)
{
    HAL_StatusTypeDef status;

    if ((cmd == NULL) || (len < 3U))
    {
        return HAL_ERROR;
    }

    /* 防止多任务同时发送时，Packet 0/1/2... 被不同命令交错。 */
    if (zdt_can_tx_mutex != NULL)
    {
        if (osMutexAcquire(zdt_can_tx_mutex, osWaitForever) != osOK)
        {
            return HAL_BUSY;
        }
    }

    status = ZDT_CAN_SendCmdUnlocked(cmd, len);

    if (zdt_can_tx_mutex != NULL)
    {
        (void)osMutexRelease(zdt_can_tx_mutex);
    }

    return status;
}

/*
  函数说明: 在 CAN 接收中断中取帧、核验格式并投递接收队列。
  调用示例: HAL_CAN_RxFifo0MsgPendingCallback(hcan);
  参数意义: hcan 为HAL CAN 句柄。
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    ZDT_CanRxFrame_t frame;
    uint8_t data[8];

    if (hcan->Instance != CAN1)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(
               hcan,
               CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &header,
                data) != HAL_OK)
        {
            ++zdt_can_error_count;
            break;
        }

        frame.ide = header.IDE;
        frame.rtr = header.RTR;
        frame.timestamp_ms = HAL_GetTick();
        frame.dlc = (uint8_t)header.DLC;

        if (header.IDE == CAN_ID_EXT)
        {
            frame.id = header.ExtId;
        }
        else
        {
            frame.id = header.StdId;
        }

        memset(frame.data, 0, sizeof(frame.data));
        memcpy(frame.data, data, frame.dlc);

        zdt_can_last_rx = frame;
        ++zdt_can_rx_count;

        if (frame.ide == CAN_ID_EXT)
        {
            uint8_t address = (uint8_t)((frame.id >> 8U) & 0xFFU);

            if ((address >= 5U) && (address <= 8U))
            {
                /* 底盘闭环用 OPS9/HW101；这些回复不占机械臂队列。 */
                continue;
            }
            else if (zdt_can_rx_queue != NULL)
            {
                /* 地址 1~4 及其他扩展帧仍交给机械臂原有队列。 */
                if (osMessageQueuePut(
                        zdt_can_rx_queue,
                        &frame,
                        0,
                        0) != osOK)
                {
                    ++zdt_can_rx_drop_count;
                }
            }
        }
        else if (zdt_can_rx_queue != NULL)
        {
            if (osMessageQueuePut(
                    zdt_can_rx_queue,
                    &frame,
                    0,
                    0) != osOK)
            {
                ++zdt_can_rx_drop_count;
            }
        }
    }
}

/*
  函数说明: 记录 CAN 错误中断计数与最近错误码。
  调用示例: HAL_CAN_ErrorCallback(hcan);
  参数意义: hcan 为HAL CAN 句柄。
*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        ++zdt_can_error_count;
    }
}

/*
  函数说明: 从 CAN 接收队列取一帧；按指定 RTOS tick 时限等待。
  调用示例: ZDT_CAN_GetRx(frame, timeout_ticks);
  参数意义: frame 为CAN 接收帧结构。
            timeout_ticks 为队列等待时间，RTOS tick。
*/
osStatus_t ZDT_CAN_GetRx(
    ZDT_CanRxFrame_t *frame,
    uint32_t timeout_ticks)
{
    if ((frame == NULL) || (zdt_can_rx_queue == NULL))
    {
        return osErrorParameter;
    }

    return osMessageQueueGet(
        zdt_can_rx_queue,
        frame,
        NULL,
        timeout_ticks);
}

/*
  函数说明: 丢弃接收队列中的旧帧并返回丢弃数量。
  调用示例: ZDT_CAN_FlushRx();
  参数意义: 无。
*/
uint32_t ZDT_CAN_FlushRx(void)
{
    ZDT_CanRxFrame_t frame;
    uint32_t count = 0U;

    if (zdt_can_rx_queue == NULL)
    {
        return 0U;
    }

    while (osMessageQueueGet(
               zdt_can_rx_queue,
               &frame,
               NULL,
               0U) == osOK)
    {
        ++count;
    }

    return count;
}
