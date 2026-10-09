#ifndef CHASSIS_FEEDBACK_H
#define CHASSIS_FEEDBACK_H

/*
 * 文件作用：保存和解析 5~8 号底盘电机的反馈帧。
 * 该模块只处理接收数据，不发送控制指令；控制真值来自 OPS9/HW101。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "chassis_mecanum.h"
#include "zdt_can.h"

typedef struct
{
    uint8_t address; /* 电机地址，固定为 5~8。 */

    float realtime_position_deg; /* 36：实时位置，deg。 */
    float position_error_deg;    /* 37：位置误差，deg。 */
    uint8_t status_flags;        /* 3A：状态位。 */

    uint8_t realtime_position_valid; /* 是否收到过有效 36。 */
    uint8_t position_error_valid;    /* 是否收到过有效 37。 */
    uint8_t status_valid;            /* 是否收到过有效 3A。 */
    uint8_t reached;                 /* 位置模式是否收到 9F。 */
    uint8_t command_error;           /* 是否收到 E2/EE。 */

    uint32_t realtime_position_update_ms; /* 36 到达时间，ms。 */
    uint32_t position_error_update_ms;    /* 37 到达时间，ms。 */
    uint32_t status_update_ms;            /* 3A 到达时间，ms。 */
    uint32_t last_frame_ms;               /* 任意反馈帧到达时间，ms。 */
} ChassisMotorFeedback_t;

typedef struct
{
    ChassisMotorFeedback_t motor[CHASSIS_MOTOR_COUNT];
} ChassisFeedbackSnapshot_t;

/* 清空反馈缓存，并建立地址 5~8 到数组下标的对应关系。 */
void ChassisFeedback_Init(void);

/* 由底盘专用 CAN 接收任务调用。 */
void ChassisFeedback_ProcessFrame(const ZDT_CanRxFrame_t *frame);

/* 复制四个底盘电机的最近反馈。 */
void ChassisFeedback_GetSnapshot(ChassisFeedbackSnapshot_t *snapshot);

/* 3A 中的堵转/堵转保护标志或命令错误。 */
uint8_t ChassisFeedback_HasFault(void);

/* 判断某个底盘电机的反馈是否在 timeout_ms 内更新。 */
uint8_t ChassisFeedback_IsFresh(uint8_t address,
                                uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_FEEDBACK_H */
