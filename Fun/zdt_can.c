#include "zdt_can.h"
#include "can.h"
#include <string.h>

#define ZDT_CAN_TX_TIMEOUT_MS    20U
#define ZDT_CAN_RX_QUEUE_DEPTH   16U

static osMessageQueueId_t zdt_can_rx_queue = NULL;

volatile uint32_t zdt_can_tx_count = 0;
volatile uint32_t zdt_can_tx_error_count = 0;
volatile uint32_t zdt_can_rx_count = 0;
volatile uint32_t zdt_can_rx_drop_count = 0;
volatile uint32_t zdt_can_error_count = 0;
volatile ZDT_CanRxFrame_t zdt_can_last_rx = {0};

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

HAL_StatusTypeDef ZDT_CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};

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

    return HAL_OK;
}

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

HAL_StatusTypeDef can_SendCmd(const uint8_t *cmd, uint8_t len)
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

        if (zdt_can_rx_queue != NULL)
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

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        ++zdt_can_error_count;
    }
}

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
