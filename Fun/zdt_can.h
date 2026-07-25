#ifndef __ZDT_CAN_H
#define __ZDT_CAN_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>

typedef struct
{
    uint32_t id;
    uint32_t ide;
    uint32_t rtr;
    uint8_t  dlc;
    uint8_t  data[8];
} ZDT_CanRxFrame_t;

HAL_StatusTypeDef ZDT_CAN_Init(void);

HAL_StatusTypeDef can_SendCmd(const uint8_t *cmd, uint8_t len);

osStatus_t ZDT_CAN_GetRx(ZDT_CanRxFrame_t *frame,
                        uint32_t timeout_ticks);

extern volatile uint32_t zdt_can_tx_count;
extern volatile uint32_t zdt_can_tx_error_count;
extern volatile uint32_t zdt_can_rx_count;
extern volatile uint32_t zdt_can_rx_drop_count;
extern volatile uint32_t zdt_can_error_count;
extern volatile ZDT_CanRxFrame_t zdt_can_last_rx;

#endif
