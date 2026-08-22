#include "X_V2.h"

#include <stddef.h>
#include <string.h>

#define X_V2_MAX_SPEED_RPM       3000.0f
#define X_V2_MAX_POSITION_DEG    429496729.5
#define X_V2_FRAME_END           0x6BU
#define X_V2_MULTI_COMMAND_CODE  0xAAU
#define X_V2_MULTI_MAX_MOTORS    8U
#define X_V2_MULTI_MAX_DATA      240U

static bool X_V2_DirectionIsValid(X_V2_Direction_t dir)
{
    return (dir == X_V2_DIR_CW) || (dir == X_V2_DIR_CCW);
}

static bool X_V2_PositionModeIsValid(X_V2_PositionMode_t mode)
{
    return (mode == X_V2_POS_RELATIVE_TO_TARGET) ||
           (mode == X_V2_POS_ABSOLUTE) ||
           (mode == X_V2_POS_RELATIVE_TO_CURRENT);
}

static bool X_V2_SpeedIsValid(float vel_rpm)
{
    return (vel_rpm >= 0.0f) &&
           (vel_rpm <= X_V2_MAX_SPEED_RPM);
}

static bool X_V2_PositionIsValid(float pos_deg)
{
    return (pos_deg >= 0.0f) &&
           ((double)pos_deg <= X_V2_MAX_POSITION_DEG);
}

static bool X_V2_Traj_Pos_CommandIsValid(
    const X_V2_Traj_Pos_Command_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    return X_V2_DirectionIsValid(command->dir) &&
           X_V2_PositionModeIsValid(command->mode) &&
           X_V2_SpeedIsValid(command->vel_rpm) &&
           X_V2_PositionIsValid(command->pos_deg);
}

static bool X_V2_Build_Traj_Pos_Command(
    const X_V2_Traj_Pos_Command_t *command,
    uint8_t cmd[16])
{
    uint16_t vel_x10;
    uint32_t pos_x10;

    if ((cmd == NULL) || !X_V2_Traj_Pos_CommandIsValid(command))
    {
        return false;
    }

    /* 协议分辨率均为 0.1，必须四舍五入，不能因 float 误差向下截断。 */
    vel_x10 = (uint16_t)(((double)command->vel_rpm * 10.0) + 0.5);
    pos_x10 = (uint32_t)(((double)command->pos_deg * 10.0) + 0.5);

    cmd[0] = command->addr;
    cmd[1] = 0xFDU;
    cmd[2] = (uint8_t)command->dir;
    cmd[3] = (uint8_t)(command->acc_rpm_per_s >> 8);
    cmd[4] = (uint8_t)(command->acc_rpm_per_s >> 0);
    cmd[5] = (uint8_t)(command->dec_rpm_per_s >> 8);
    cmd[6] = (uint8_t)(command->dec_rpm_per_s >> 0);
    cmd[7] = (uint8_t)(vel_x10 >> 8);
    cmd[8] = (uint8_t)(vel_x10 >> 0);
    cmd[9] = (uint8_t)(pos_x10 >> 24);
    cmd[10] = (uint8_t)(pos_x10 >> 16);
    cmd[11] = (uint8_t)(pos_x10 >> 8);
    cmd[12] = (uint8_t)(pos_x10 >> 0);
    cmd[13] = (uint8_t)command->mode;
    cmd[14] = (uint8_t)command->snF;
    cmd[15] = X_V2_FRAME_END;

    return true;
}

HAL_StatusTypeDef X_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0x0AU, 0x6DU, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_Reset_Clog_Pro(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0x0EU, 0x52U, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_En_Control(uint8_t addr, bool state, bool snF)
{
    const uint8_t cmd[] = {
        addr, 0xF3U, 0xABU, (uint8_t)state, (uint8_t)snF, X_V2_FRAME_END
    };
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_Vel_Control(uint8_t addr,
                                   X_V2_Direction_t dir,
                                   uint16_t acc,
                                   float vel_rpm,
                                   bool snF)
{
    uint16_t vel_x10;
    uint8_t cmd[9];

    if (!X_V2_DirectionIsValid(dir) || !X_V2_SpeedIsValid(vel_rpm))
    {
        return HAL_ERROR;
    }

    vel_x10 = (uint16_t)(vel_rpm * 10.0f);

    cmd[0] = addr;
    cmd[1] = 0xF6U;
    cmd[2] = (uint8_t)dir;
    cmd[3] = (uint8_t)(acc >> 8);
    cmd[4] = (uint8_t)(acc >> 0);
    cmd[5] = (uint8_t)(vel_x10 >> 8);
    cmd[6] = (uint8_t)(vel_x10 >> 0);
    cmd[7] = (uint8_t)snF;
    cmd[8] = X_V2_FRAME_END;

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_Traj_Pos_Control(uint8_t addr,
                                        X_V2_Direction_t dir,
                                        uint16_t acc_rpm_per_s,
                                        uint16_t dec_rpm_per_s,
                                        float vel_rpm,
                                        float pos_deg,
                                        X_V2_PositionMode_t mode,
                                        bool snF)
{
    X_V2_Traj_Pos_Command_t command;
    uint8_t cmd[16];

    command.addr = addr;
    command.dir = dir;
    command.acc_rpm_per_s = acc_rpm_per_s;
    command.dec_rpm_per_s = dec_rpm_per_s;
    command.vel_rpm = vel_rpm;
    command.pos_deg = pos_deg;
    command.mode = mode;
    command.snF = snF;

    if (!X_V2_Build_Traj_Pos_Command(&command, cmd))
    {
        return HAL_ERROR;
    }

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_MultiMotor_Command(const uint8_t *commands,
                                           uint8_t commands_len)
{
    uint8_t multi_cmd[5U + X_V2_MULTI_MAX_DATA];
    uint16_t total_len;

    if ((commands == NULL) ||
        (commands_len == 0U) ||
        (commands_len > X_V2_MULTI_MAX_DATA))
    {
        return HAL_ERROR;
    }

    /* 手册 5.3.1 的总字节数包含 00 AA、长度字段和最后的 6B。 */
    total_len = (uint16_t)commands_len + 5U;
    if (total_len > 0xFFU)
    {
        /* 当前 CAN 打包接口的长度参数为 uint8_t。 */
        return HAL_ERROR;
    }

    multi_cmd[0] = 0U;
    multi_cmd[1] = X_V2_MULTI_COMMAND_CODE;
    multi_cmd[2] = (uint8_t)(total_len >> 8);
    multi_cmd[3] = (uint8_t)(total_len >> 0);
    memcpy(&multi_cmd[4], commands, commands_len);
    multi_cmd[4U + commands_len] = X_V2_FRAME_END;

    /* can_SendCmd() 会按 4.2.1 拆为 ID 0000、0001、0002... 的 CAN 帧。 */
    return can_SendCmd(multi_cmd, (uint8_t)total_len);
}

HAL_StatusTypeDef X_V2_MultiMotor_Traj_Pos_Control(
    const X_V2_Traj_Pos_Command_t *commands,
    uint8_t command_count)
{
    uint8_t command_stream[X_V2_MULTI_MAX_MOTORS * 16U];
    uint8_t one_command[16];
    uint8_t index;
    uint8_t stream_len;
    uint8_t previous;

    if ((commands == NULL) ||
        (command_count == 0U) ||
        (command_count > X_V2_MULTI_MAX_MOTORS))
    {
        return HAL_ERROR;
    }

    stream_len = 0U;
    for (index = 0U; index < command_count; ++index)
    {
        /* 多电机子命令不能使用广播地址，也不能重复控制同一地址。 */
        if (commands[index].addr == 0U)
        {
            return HAL_ERROR;
        }
        for (previous = 0U; previous < index; ++previous)
        {
            if (commands[previous].addr == commands[index].addr)
            {
                return HAL_ERROR;
            }
        }

        if (!X_V2_Build_Traj_Pos_Command(&commands[index], one_command))
        {
            return HAL_ERROR;
        }

        memcpy(&command_stream[stream_len], one_command, sizeof(one_command));
        stream_len = (uint8_t)(stream_len + sizeof(one_command));
    }

    return X_V2_MultiMotor_Command(command_stream, stream_len);
}

HAL_StatusTypeDef X_V2_Timed_Return_Info(uint8_t addr,
                                         uint8_t info_function,
                                         uint16_t period_ms)
{
    const uint8_t cmd[] = {
        addr,
        0x11U,
        0x18U,
        info_function,
        (uint8_t)(period_ms >> 8),
        (uint8_t)(period_ms >> 0),
        X_V2_FRAME_END
    };

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_Stop_Now(uint8_t addr, bool snF)
{
    const uint8_t cmd[] = {addr, 0xFEU, 0x98U, (uint8_t)snF, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

HAL_StatusTypeDef X_V2_Synchronous_motion(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0xFFU, 0x66U, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}
