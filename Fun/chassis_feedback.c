#include "chassis_feedback.h"

#include <string.h>

/*
 * X 固件反馈帧的功能码：
 * 36=实时位置，37=位置误差，3A=状态；9F 只用于记录位置模式完成。
 * 速度模式下底盘不依靠 9F 判断到位，而是依靠 OPS9/HW101 做外部闭环。
 */

#define CHASSIS_INFO_REALTIME_POSITION   (0x36U)
#define CHASSIS_INFO_POSITION_ERROR      (0x37U)
#define CHASSIS_INFO_STATUS              (0x3AU)
#define CHASSIS_POSITION_COMMAND_FB      (0xFBU)
#define CHASSIS_TRAJ_POSITION_COMMAND_FB (0xFDU)
#define CHASSIS_HOME_COMMAND_FB          (0x9AU)
#define CHASSIS_FRAME_DONE               (0x9FU)
#define CHASSIS_ERROR_PARAMETER          (0xE2U)
#define CHASSIS_ERROR_FORMAT             (0xEEU)

static volatile ChassisMotorFeedback_t
    chassis_feedback[CHASSIS_MOTOR_COUNT];

static int8_t ChassisFeedback_AddressToIndex(uint8_t address)
{
    if ((address < 5U) || (address > 8U))
    {
        return -1;
    }

    return (int8_t)(address - 5U);
}

static float ChassisFeedback_DecodeSignedAngle(
    const uint8_t *data,
    float divisor)
{
    uint32_t raw;
    float angle;

    raw = ((uint32_t)data[2] << 24U) |
          ((uint32_t)data[3] << 16U) |
          ((uint32_t)data[4] << 8U) |
          ((uint32_t)data[5] << 0U);
    angle = (float)raw / divisor;

    /* 36/37 的 data[1] 为符号：0 表示正，非 0 表示负。 */
    return (data[1] == 0U) ? angle : -angle;
}

void ChassisFeedback_Init(void)
{
    uint8_t index;

    memset((void *)chassis_feedback,
           0,
           sizeof(chassis_feedback));

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        chassis_feedback[index].address = (uint8_t)(5U + index);
    }
}

void ChassisFeedback_ProcessFrame(const ZDT_CanRxFrame_t *frame)
{
    uint8_t address;
    uint8_t function_code;
    uint8_t response_code;
    uint32_t now_ms;
    int8_t index;
    ChassisMotorFeedback_t *feedback;

    if (frame == NULL)
    {
        return;
    }

    /*
     * 底盘专用队列只应包含扩展帧：
     * 扩展 ID 高 8 位为电机地址，低 8 位为分包序号。
     * 反馈数据都在第 0 包，因此忽略其他分包。
     */
    if ((frame->ide != CAN_ID_EXT) ||
        ((frame->id & 0xFFU) != 0U))
    {
        return;
    }

    address = (uint8_t)((frame->id >> 8U) & 0xFFU);
    index = ChassisFeedback_AddressToIndex(address);
    if ((index < 0) || (frame->dlc == 0U))
    {
        return;
    }

    feedback = (ChassisMotorFeedback_t *)&chassis_feedback[index];
    function_code = frame->data[0];
    response_code = (frame->dlc >= 2U) ? frame->data[1] : 0U;
    now_ms = frame->timestamp_ms;
    feedback->address = address;
    feedback->last_frame_ms = now_ms;

    if ((function_code == CHASSIS_INFO_REALTIME_POSITION) &&
        (frame->dlc >= 7U))
    {
        feedback->realtime_position_deg =
            ChassisFeedback_DecodeSignedAngle(frame->data, 10.0f);
        feedback->realtime_position_valid = 1U;
        feedback->realtime_position_update_ms = now_ms;
        return;
    }

    if ((function_code == CHASSIS_INFO_POSITION_ERROR) &&
        (frame->dlc >= 7U))
    {
        feedback->position_error_deg =
            ChassisFeedback_DecodeSignedAngle(frame->data, 100.0f);
        feedback->position_error_valid = 1U;
        feedback->position_error_update_ms = now_ms;
        return;
    }

    if ((function_code == CHASSIS_INFO_STATUS) &&
        (frame->dlc >= 3U))
    {
        feedback->status_flags = response_code;
        feedback->status_valid = 1U;
        feedback->status_update_ms = now_ms;
        return;
    }

    /* 位置/回零动作完成帧：速度模式不会主动返回 9F。 */
    if (((function_code == CHASSIS_POSITION_COMMAND_FB) ||
         (function_code == CHASSIS_TRAJ_POSITION_COMMAND_FB) ||
         (function_code == CHASSIS_HOME_COMMAND_FB)) &&
        (response_code == CHASSIS_FRAME_DONE))
    {
        feedback->reached = 1U;
        return;
    }

    if ((response_code == CHASSIS_ERROR_PARAMETER) ||
        (response_code == CHASSIS_ERROR_FORMAT))
    {
        feedback->command_error = 1U;
    }
}

void ChassisFeedback_GetSnapshot(ChassisFeedbackSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return;
    }

    /* 反馈可能在 CAN 中断中更新，复制时暂时屏蔽中断避免读到半帧。 */
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(snapshot,
           (const void *)chassis_feedback,
           sizeof(*snapshot));
    if (primask == 0U)
    {
        __enable_irq();
    }
}

uint8_t ChassisFeedback_HasFault(void)
{
    ChassisFeedbackSnapshot_t snapshot;
    uint8_t index;

    ChassisFeedback_GetSnapshot(&snapshot);

    for (index = 0U; index < CHASSIS_MOTOR_COUNT; ++index)
    {
        /* 3A bit2=Cgi_TF、bit3=Cgp_TF，另检查命令错误响应。 */
        if ((snapshot.motor[index].command_error != 0U) ||
            ((snapshot.motor[index].status_flags & 0x0CU) != 0U))
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t ChassisFeedback_IsFresh(uint8_t address,
                                uint32_t timeout_ms)
{
    ChassisFeedbackSnapshot_t snapshot;
    int8_t index;
    uint32_t now_ms;

    index = ChassisFeedback_AddressToIndex(address);
    if (index < 0)
    {
        return 0U;
    }

    ChassisFeedback_GetSnapshot(&snapshot);
    if (snapshot.motor[index].last_frame_ms == 0U)
    {
        return 0U;
    }

    now_ms = HAL_GetTick();
    return ((now_ms - snapshot.motor[index].last_frame_ms) <= timeout_ms)
               ? 1U
               : 0U;
}
