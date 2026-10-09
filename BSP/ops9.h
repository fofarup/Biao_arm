#ifndef OPS9_H
#define OPS9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* OPS9 固定数据帧：2 字节帧头 + 6 个 float + 2 字节帧尾。 */
#define OPS9_DATA_LENGTH    (24U)
#define OPS9_FRAME_LENGTH   (28U)

/* 一帧已经解析完成的 OPS9 测量结果。 */
typedef struct
{
    float heading_deg;       /* 航向角，单位：度 */
    float pitch_deg;         /* 俯仰角，单位：度 */
    float roll_deg;          /* 横滚角，单位：度 */
    float x_mm;              /* X 坐标，单位：毫米 */
    float y_mm;              /* Y 坐标，单位：毫米 */
    float heading_rate_dps;  /* 航向角速度，单位：度/秒 */

    uint32_t last_update_ms; /* 最近一次合法帧的系统时间 */
    uint32_t frame_count;    /* 合法帧计数 */
    uint32_t bad_frame_count;/* 帧头、帧尾或数据异常计数 */
    uint8_t valid;           /* 1：已经收到过合法帧；0：尚未收到 */
} OPS9_Data_t;

/* 启动 USART3 的单字节接收中断。 */
HAL_StatusTypeDef OPS9_StartReceive(void);

/* 复制最近一帧数据；返回 1 表示曾收到过合法帧。 */
uint8_t OPS9_GetLatest(OPS9_Data_t *data);

/* 判断最近一帧是否在 timeout_ms 内更新。 */
uint8_t OPS9_IsFresh(uint32_t timeout_ms);

/* UART 出错后清理解析状态并重新开始接收。 */
void OPS9_ResetReceive(void);

#ifdef __cplusplus
}
#endif

#endif /* OPS9_H */
