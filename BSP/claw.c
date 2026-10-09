#include "claw.h"

#include <math.h>

#include "usart.h"

/* 默认使用示例中的 ID 000；这里不发送修改 ID 或舵机模式的指令。 */
#define CLAW_SERVO_ID                 (0U)

/* 270° 模式的完整角度与协议位置映射，修改夹爪限位时不要动这组映射。 */
#define CLAW_FULL_MIN_DEG            (0.0f)
#define CLAW_FULL_MAX_DEG            (270.0f)
#define CLAW_POSITION_MIN            (500U)
#define CLAW_POSITION_MAX            (2500U)

/* 夹爪软件允许范围；当前仅按协议全范围填写，实机标定后应收窄。 */
#define CLAW_ALLOWED_MIN_DEG         (0.0f)
#define CLAW_ALLOWED_MAX_DEG         (270.0f)

/* 装配方向相反时改为 1；不会改变舵机固件中的旋转模式。 */
#define CLAW_REVERSE_ANGLE           (0U)

/* T 字段为四位十进制毫秒数；禁止 0，避免未确认的瞬时动作。 */
#define CLAW_TIME_MIN_MS             (1U)
#define CLAW_TIME_MAX_MS             (9999U)
#define CLAW_UART_TX_TIMEOUT_MS      (50U)

/* 把 0～9999 写入四位 ASCII 字段；报文不需要字符串结尾的 \0。 */
static void Claw_WriteFourDigits(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)('0' + ((value / 1000U) % 10U));
    destination[1] = (uint8_t)('0' + ((value / 100U) % 10U));
    destination[2] = (uint8_t)('0' + ((value / 10U) % 10U));
    destination[3] = (uint8_t)('0' + (value % 10U));
}

HAL_StatusTypeDef Claw_SetAngle(float angle_deg, uint16_t move_time_ms)
{
    uint8_t frame[15];
    float position_float;
    uint16_t position;

    /* 参数不合法时不发指令，不能把角度自动钳到未知的机械端点。 */
    if (!isfinite(angle_deg) ||
        (angle_deg < CLAW_FULL_MIN_DEG) ||
        (angle_deg > CLAW_FULL_MAX_DEG) ||
        (angle_deg < CLAW_ALLOWED_MIN_DEG) ||
        (angle_deg > CLAW_ALLOWED_MAX_DEG) ||
        (move_time_ms < CLAW_TIME_MIN_MS) ||
        (move_time_ms > CLAW_TIME_MAX_MS))
    {
        return HAL_ERROR;
    }

    position_float = (float)CLAW_POSITION_MIN +
        (angle_deg - CLAW_FULL_MIN_DEG) *
        ((float)(CLAW_POSITION_MAX - CLAW_POSITION_MIN) /
         (CLAW_FULL_MAX_DEG - CLAW_FULL_MIN_DEG));
    if (CLAW_REVERSE_ANGLE != 0U)
    {
        position_float = (float)(CLAW_POSITION_MIN + CLAW_POSITION_MAX) -
                         position_float;
    }
    position = (uint16_t)(position_float + 0.5f);

    /* 舵机协议：#000P1500T1000!，ID/P/T 均为固定宽度十进制字段。 */
    frame[0] = (uint8_t)'#';
    frame[1] = (uint8_t)('0' + ((CLAW_SERVO_ID / 100U) % 10U));
    frame[2] = (uint8_t)('0' + ((CLAW_SERVO_ID / 10U) % 10U));
    frame[3] = (uint8_t)('0' + (CLAW_SERVO_ID % 10U));
    frame[4] = (uint8_t)'P';
    Claw_WriteFourDigits(&frame[5], position);
    frame[9] = (uint8_t)'T';
    Claw_WriteFourDigits(&frame[10], move_time_ms);
    frame[14] = (uint8_t)'!';

    /* 只有 TX 线：不能根据返回值判断夹爪真实角度或是否夹住物体。 */
    return HAL_UART_Transmit(&huart4,
                             frame,
                             (uint16_t)sizeof(frame),
                             CLAW_UART_TX_TIMEOUT_MS);
}
