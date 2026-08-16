#ifndef __X_V2_H
#define __X_V2_H

#include <stdbool.h>
#include <stdint.h>

#include "zdt_can.h"

/* ZDT X42S second-generation X-firmware protocol. */
typedef enum
{
    X_V2_DIR_CW = 0U,
    X_V2_DIR_CCW = 1U
} X_V2_Direction_t;

typedef enum
{
    X_V2_POS_RELATIVE_TO_TARGET = 0U,
    X_V2_POS_ABSOLUTE = 1U,
    X_V2_POS_RELATIVE_TO_CURRENT = 2U
} X_V2_PositionMode_t;

HAL_StatusTypeDef X_V2_Reset_CurPos_To_Zero(uint8_t addr);
HAL_StatusTypeDef X_V2_Reset_Clog_Pro(uint8_t addr);

HAL_StatusTypeDef X_V2_En_Control(uint8_t addr, bool state, bool snF);
HAL_StatusTypeDef X_V2_Vel_Control(uint8_t addr,
                                   X_V2_Direction_t dir,
                                   uint16_t acc,
                                   float vel_rpm,
                                   bool snF);
HAL_StatusTypeDef X_V2_Traj_Pos_Control(uint8_t addr,
                                        X_V2_Direction_t dir,
                                        uint16_t acc_rpm_per_s,
                                        uint16_t dec_rpm_per_s,
                                        float vel_rpm,
                                        float pos_deg,
                                        X_V2_PositionMode_t mode,
                                        bool snF);
HAL_StatusTypeDef X_V2_Stop_Now(uint8_t addr, bool snF);
HAL_StatusTypeDef X_V2_Synchronous_motion(uint8_t addr);

#endif /* __X_V2_H */
