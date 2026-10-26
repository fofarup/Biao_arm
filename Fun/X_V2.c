/*
  程序说明: 张大头第二代 X 固件电机控制协议的检查、组包与发送。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#include "X_V2.h"

#include <stddef.h>
#include <string.h>

#define X_V2_MAX_SPEED_RPM       3000.0f
#define X_V2_MAX_POSITION_DEG    429496729.5
#define X_V2_FRAME_END           0x6BU
#define X_V2_MULTI_COMMAND_CODE  0xAAU
#define X_V2_MULTI_MAX_MOTORS    8U
#define X_V2_MULTI_MAX_DATA      240U

/*
  函数说明: 检查电机方向是否为 CW 或 CCW。
  调用示例: X_V2_DirectionIsValid(dir);
  参数意义: dir 为电机 CW/CCW 转向。
*/
static bool X_V2_DirectionIsValid(X_V2_Direction_t dir)
{
    return (dir == X_V2_DIR_CW) || (dir == X_V2_DIR_CCW);
}

/*
  函数说明: 检查位置控制模式是否属于驱动器支持的三种模式。
  调用示例: X_V2_PositionModeIsValid(mode);
  参数意义: mode 为驱动器位置控制模式。
*/
static bool X_V2_PositionModeIsValid(X_V2_PositionMode_t mode)
{
    return (mode == X_V2_POS_RELATIVE_TO_TARGET) ||
           (mode == X_V2_POS_ABSOLUTE) ||
           (mode == X_V2_POS_RELATIVE_TO_CURRENT);
}

/*
  函数说明: 检查查询或周期回传的信息功能码。
  调用示例: X_V2_InfoFunctionIsValid(info_function);
  参数意义: info_function 为驱动器信息功能码。
*/
static bool X_V2_InfoFunctionIsValid(uint8_t info_function)
{
    return (info_function == X_V2_INFO_REALTIME_SPEED) ||
           (info_function == X_V2_INFO_REALTIME_POSITION) ||
           (info_function == X_V2_INFO_POSITION_ERROR) ||
           (info_function == X_V2_INFO_STATUS);
}

/*
  函数说明: 检查转速数值是否有限且处于驱动器支持范围内。
  调用示例: X_V2_SpeedIsValid(vel_rpm);
  参数意义: vel_rpm 为电机转速幅值，RPM。
*/
static bool X_V2_SpeedIsValid(float vel_rpm)
{
    return (vel_rpm >= 0.0f) &&
           (vel_rpm <= X_V2_MAX_SPEED_RPM);
}

/*
  函数说明: 检查位置角度是否有限、非负且可用协议字段表示。
  调用示例: X_V2_PositionIsValid(pos_deg);
  参数意义: pos_deg 为电机位置指令的非负角度幅值，度。
*/
static bool X_V2_PositionIsValid(float pos_deg)
{
    return (pos_deg >= 0.0f) &&
           ((double)pos_deg <= X_V2_MAX_POSITION_DEG);
}

/*
  函数说明: 检查一条梯形位置指令的方向、速度、位置和模式字段。
  调用示例: X_V2_Traj_Pos_CommandIsValid(command);
  参数意义: command 为单条命令结构。
*/
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

/*
  函数说明: 检查一条速度指令的方向和转速字段。
  调用示例: X_V2_Vel_CommandIsValid(command);
  参数意义: command 为单条命令结构。
*/
static bool X_V2_Vel_CommandIsValid(const X_V2_Vel_Command_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    return X_V2_DirectionIsValid(command->dir) &&
           X_V2_SpeedIsValid(command->vel_rpm);
}

/*
  函数说明: 把一条速度命令编码为 9 字节 X 固件报文。
  调用示例: X_V2_Build_Vel_Command(command, cmd);
  参数意义: command 为单条命令结构。
            cmd 为驱动器协议字节数组。
*/
static bool X_V2_Build_Vel_Command(const X_V2_Vel_Command_t *command,
                                   uint8_t cmd[9])
{
    uint16_t vel_x10;

    if ((cmd == NULL) || !X_V2_Vel_CommandIsValid(command))
    {
        return false;
    }

    vel_x10 = (uint16_t)(((double)command->vel_rpm * 10.0) + 0.5);
    cmd[0] = command->addr;
    cmd[1] = 0xF6U;
    cmd[2] = (uint8_t)command->dir;
    cmd[3] = (uint8_t)(command->acc_rpm_per_s >> 8);
    cmd[4] = (uint8_t)(command->acc_rpm_per_s >> 0);
    cmd[5] = (uint8_t)(vel_x10 >> 8);
    cmd[6] = (uint8_t)(vel_x10 >> 0);
    cmd[7] = (uint8_t)command->snF;
    cmd[8] = X_V2_FRAME_END;
    return true;
}

/*
  函数说明: 把一条梯形位置命令编码为 16 字节 X 固件报文。
  调用示例: X_V2_Build_Traj_Pos_Command(command, cmd);
  参数意义: command 为单条命令结构。
            cmd 为驱动器协议字节数组。
*/
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

/*
  函数说明: 发送驱动器当前位置清零命令；只在明确执行归零流程时使用。
  调用示例: X_V2_Reset_CurPos_To_Zero(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0x0AU, 0x6DU, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 发送清除驱动器堵转保护状态的命令。
  调用示例: X_V2_Reset_Clog_Pro(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Reset_Clog_Pro(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0x0EU, 0x52U, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 发送指定电机的使能或失能命令。
  调用示例: X_V2_En_Control(addr, state, snF);
  参数意义: addr 为电机 CAN 地址。
            state 为true 使能，false 失能。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
HAL_StatusTypeDef X_V2_En_Control(uint8_t addr, bool state, bool snF)
{
    const uint8_t cmd[] = {
        addr, 0xF3U, 0xABU, (uint8_t)state, (uint8_t)snF, X_V2_FRAME_END
    };
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 编码并发送单电机速度控制指令。
  调用示例: X_V2_Vel_Control(addr, dir, acc, vel_rpm, snF);
  参数意义: addr 为电机 CAN 地址。
            dir 为电机 CW/CCW 转向。
            acc 为电机加速度，RPM/s。
            vel_rpm 为电机转速幅值，RPM。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
HAL_StatusTypeDef X_V2_Vel_Control(uint8_t addr,
                                   X_V2_Direction_t dir,
                                   uint16_t acc,
                                   float vel_rpm,
                                   bool snF)
{
    X_V2_Vel_Command_t command;
    uint8_t cmd[9];

    command.addr = addr;
    command.dir = dir;
    command.acc_rpm_per_s = acc;
    command.vel_rpm = vel_rpm;
    command.snF = snF;

    if (!X_V2_Build_Vel_Command(&command, cmd))
    {
        return HAL_ERROR;
    }

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 编码并发送单电机三角形或梯形位置控制指令。
  调用示例: X_V2_Traj_Pos_Control(addr, dir, acc_rpm_per_s, dec_rpm_per_s, vel_rpm, pos_deg, mode, snF);
  参数意义: addr 为电机 CAN 地址。
            dir 为电机 CW/CCW 转向。
            acc_rpm_per_s 为电机加速度，RPM/s。
            dec_rpm_per_s 为电机减速度，RPM/s。
            vel_rpm 为电机转速幅值，RPM。
            pos_deg 为电机位置指令的非负角度幅值，度。
            mode 为驱动器位置控制模式。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
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

/*
  函数说明: 封装并发送一组原始多电机命令数据。
  调用示例: X_V2_MultiMotor_Command(commands, commands_len);
  参数意义: commands 为多电机命令数组。
            commands_len 为命令数据长度，字节。
*/
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

/*
  函数说明: 校验并打包多台电机的位置指令，一次提交到 CAN 发送层。
  调用示例: X_V2_MultiMotor_Traj_Pos_Control(commands, command_count);
  参数意义: commands 为多电机命令数组。
            command_count 为多电机命令条数。
*/
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

/*
  函数说明: 校验并打包多台电机的速度指令，一次提交到 CAN 发送层。
  调用示例: X_V2_MultiMotor_Vel_Control(commands, command_count);
  参数意义: commands 为多电机命令数组。
            command_count 为多电机命令条数。
*/
HAL_StatusTypeDef X_V2_MultiMotor_Vel_Control(
    const X_V2_Vel_Command_t *commands,
    uint8_t command_count)
{
    uint8_t command_stream[X_V2_MULTI_MAX_MOTORS * 9U];
    uint8_t one_command[9];
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

        if (!X_V2_Build_Vel_Command(&commands[index], one_command))
        {
            return HAL_ERROR;
        }

        memcpy(&command_stream[stream_len], one_command, sizeof(one_command));
        stream_len = (uint8_t)(stream_len + sizeof(one_command));
    }

    return X_V2_MultiMotor_Command(command_stream, stream_len);
}

/*
  函数说明: 设置驱动器某类信息的自动回传周期；周期为 0 时关闭。
  调用示例: X_V2_Timed_Return_Info(addr, info_function, period_ms);
  参数意义: addr 为电机 CAN 地址。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
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

    if ((addr == 0U) || !X_V2_InfoFunctionIsValid(info_function))
    {
        return HAL_ERROR;
    }

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 向指定驱动器请求一次指定信息。
  调用示例: X_V2_Read_Info(addr, info_function);
  参数意义: addr 为电机 CAN 地址。
            info_function 为驱动器信息功能码。
*/
HAL_StatusTypeDef X_V2_Read_Info(uint8_t addr,
                                 uint8_t info_function)
{
    const uint8_t cmd[] = {addr, info_function, X_V2_FRAME_END};

    if ((addr == 0U) || !X_V2_InfoFunctionIsValid(info_function))
    {
        return HAL_ERROR;
    }

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 设置驱动器通信心跳超时和是否保存到掉电参数。
  调用示例: X_V2_Set_Heartbeat(addr, save, timeout_ms);
  参数意义: addr 为电机 CAN 地址。
            save 为是否写入驱动器掉电保存参数。
            timeout_ms 为等待或心跳超时，毫秒。
*/
HAL_StatusTypeDef X_V2_Set_Heartbeat(uint8_t addr,
                                     bool save,
                                     uint32_t timeout_ms)
{
    const uint8_t cmd[] = {
        addr,
        0x68U,
        0x38U,
        (uint8_t)save,
        (uint8_t)(timeout_ms >> 24),
        (uint8_t)(timeout_ms >> 16),
        (uint8_t)(timeout_ms >> 8),
        (uint8_t)(timeout_ms >> 0),
        X_V2_FRAME_END
    };

    if (addr == 0U)
    {
        return HAL_ERROR;
    }

    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 发送立即停止指令。
  调用示例: X_V2_Stop_Now(addr, snF);
  参数意义: addr 为电机 CAN 地址。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
HAL_StatusTypeDef X_V2_Stop_Now(uint8_t addr, bool snF)
{
    const uint8_t cmd[] = {addr, 0xFEU, 0x98U, (uint8_t)snF, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 发送同步触发命令，使缓存中的同步指令开始执行。
  调用示例: X_V2_Synchronous_motion(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Synchronous_motion(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0xFFU, 0x66U, X_V2_FRAME_END};
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}

/*
  函数说明: 按手册 5.6.24 查询心跳时间，使用 Addr 16 6B 命令。
  调用示例: X_V2_Read_Heartbeat(1U);
  参数意义: addr 为非零电机地址；查询结果以毫秒为单位，不写入或更改参数。
*/
HAL_StatusTypeDef X_V2_Read_Heartbeat(uint8_t addr)
{
    const uint8_t cmd[] = {addr, 0x16U, X_V2_FRAME_END};
    if (addr == 0U) return HAL_ERROR;
    return can_SendCmd(cmd, (uint8_t)sizeof(cmd));
}
