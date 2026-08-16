#include "X_V2.h"

#define X_V2_MAX_SPEED_RPM       3000.0f
#define X_V2_MAX_POSITION_DEG    429496729.5f
#define X_V2_FRAME_END           0x6BU

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
           (pos_deg <= X_V2_MAX_POSITION_DEG);
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
    uint16_t vel_x10;
    uint32_t pos_x10;
    uint8_t cmd[16];

    if (!X_V2_DirectionIsValid(dir) ||
        !X_V2_PositionModeIsValid(mode) ||
        !X_V2_SpeedIsValid(vel_rpm) ||
        !X_V2_PositionIsValid(pos_deg))
    {
        return HAL_ERROR;
    }

    vel_x10 = (uint16_t)(vel_rpm * 10.0f);
    pos_x10 = (uint32_t)(pos_deg * 10.0f);

    cmd[0] = addr;
    cmd[1] = 0xFDU;
    cmd[2] = (uint8_t)dir;
    cmd[3] = (uint8_t)(acc_rpm_per_s >> 8);
    cmd[4] = (uint8_t)(acc_rpm_per_s >> 0);
    cmd[5] = (uint8_t)(dec_rpm_per_s >> 8);
    cmd[6] = (uint8_t)(dec_rpm_per_s >> 0);
    cmd[7] = (uint8_t)(vel_x10 >> 8);
    cmd[8] = (uint8_t)(vel_x10 >> 0);
    cmd[9] = (uint8_t)(pos_x10 >> 24);
    cmd[10] = (uint8_t)(pos_x10 >> 16);
    cmd[11] = (uint8_t)(pos_x10 >> 8);
    cmd[12] = (uint8_t)(pos_x10 >> 0);
    cmd[13] = (uint8_t)mode;
    cmd[14] = (uint8_t)snF;
    cmd[15] = X_V2_FRAME_END;

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
