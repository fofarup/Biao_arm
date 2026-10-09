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

/* 5.5.1 定时返回信息命令中常用的信息功能码。 */
typedef enum
{
    X_V2_INFO_REALTIME_SPEED = 0x35U,
    X_V2_INFO_REALTIME_POSITION = 0x36U,
    X_V2_INFO_POSITION_ERROR = 0x37U,
    X_V2_INFO_STATUS = 0x3AU
} X_V2_InfoFunction_t;

/* 5.3.1 多电机命令中的一条 X 固件梯形位置命令。 */
typedef struct
{
    uint8_t addr;
    X_V2_Direction_t dir;
    uint16_t acc_rpm_per_s;
    uint16_t dec_rpm_per_s;
    float vel_rpm;
    float pos_deg;
    X_V2_PositionMode_t mode;

    /* 单机命令中为同步缓存标志；多电机命令中为到位返回标志。 */
    bool snF;
} X_V2_Traj_Pos_Command_t;

/* 5.3.5 多电机命令中的一条 X 固件速度命令。 */
typedef struct
{
    uint8_t addr;
    X_V2_Direction_t dir;
    uint16_t acc_rpm_per_s;
    float vel_rpm;

    /* 多电机命令立即执行时应为 false。 */
    bool snF;
} X_V2_Vel_Command_t;

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

/*
 * 发送 5.3.1 的多电机命令。
 * commands 为多条完整子命令的连续字节流，每条子命令必须包含自己的
 * 地址、功能码和结尾 0x6B，但不包含外层 00 AA 和外层结尾 0x6B。
 */
HAL_StatusTypeDef X_V2_MultiMotor_Command(const uint8_t *commands,
                                           uint8_t commands_len);

/* 将多条 X 梯形位置命令打包为一条 00 AA 多电机命令并发送。 */
HAL_StatusTypeDef X_V2_MultiMotor_Traj_Pos_Control(
    const X_V2_Traj_Pos_Command_t *commands,
    uint8_t command_count);

/* 将多条 X 速度命令打包为一条 00 AA 多电机命令并发送。 */
HAL_StatusTypeDef X_V2_MultiMotor_Vel_Control(
    const X_V2_Vel_Command_t *commands,
    uint8_t command_count);

/* 5.5.1：周期读取实时位置、状态等信息；period_ms=0 表示停止返回。 */
HAL_StatusTypeDef X_V2_Timed_Return_Info(uint8_t addr,
                                         uint8_t info_function,
                                         uint16_t period_ms);

/* 读取一次 5.5 章节中的实时信息，返回帧进入 ZDT_CAN 接收队列。 */
HAL_StatusTypeDef X_V2_Read_Info(uint8_t addr,
                                 uint8_t info_function);

/* 3.8/5.6.25 心跳保护；timeout_ms=0 关闭，save=false 不写入掉电存储。 */
HAL_StatusTypeDef X_V2_Set_Heartbeat(uint8_t addr,
                                     bool save,
                                     uint32_t timeout_ms);

HAL_StatusTypeDef X_V2_Stop_Now(uint8_t addr, bool snF);
HAL_StatusTypeDef X_V2_Synchronous_motion(uint8_t addr);

#endif /* __X_V2_H */
