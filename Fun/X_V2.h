/*
  程序说明: 张大头第二代 X 固件电机控制协议的检查、组包与发送。 本文件声明接口与数据类型。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

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

/*
  函数说明: 发送驱动器当前位置清零命令；只在明确执行归零流程时使用。
  调用示例: X_V2_Reset_CurPos_To_Zero(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Reset_CurPos_To_Zero(uint8_t addr);
/*
  函数说明: 发送清除驱动器堵转保护状态的命令。
  调用示例: X_V2_Reset_Clog_Pro(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Reset_Clog_Pro(uint8_t addr);

/*
  函数说明: 发送指定电机的使能或失能命令。
  调用示例: X_V2_En_Control(addr, state, snF);
  参数意义: addr 为电机 CAN 地址。
            state 为true 使能，false 失能。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
HAL_StatusTypeDef X_V2_En_Control(uint8_t addr, bool state, bool snF);
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
                                   bool snF);
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
                                        bool snF);

/*
  函数说明: 封装并发送一组原始多电机命令数据。
  发送 5.3.1 的多电机命令。
  commands 为多条完整子命令的连续字节流，每条子命令必须包含自己的
  地址、功能码和结尾 0x6B，但不包含外层 00 AA 和外层结尾 0x6B。
  调用示例: X_V2_MultiMotor_Command(commands, commands_len);
  参数意义: commands 为多电机命令数组。
            commands_len 为命令数据长度，字节。
*/
HAL_StatusTypeDef X_V2_MultiMotor_Command(const uint8_t *commands,
                                           uint8_t commands_len);

/*
  函数说明: 校验并打包多台电机的位置指令，一次提交到 CAN 发送层。
  调用示例: X_V2_MultiMotor_Traj_Pos_Control(commands, command_count);
  参数意义: commands 为多电机命令数组。
            command_count 为多电机命令条数。
*/
HAL_StatusTypeDef X_V2_MultiMotor_Traj_Pos_Control(
    const X_V2_Traj_Pos_Command_t *commands,
    uint8_t command_count);

/*
  函数说明: 校验并打包多台电机的速度指令，一次提交到 CAN 发送层。
  调用示例: X_V2_MultiMotor_Vel_Control(commands, command_count);
  参数意义: commands 为多电机命令数组。
            command_count 为多电机命令条数。
*/
HAL_StatusTypeDef X_V2_MultiMotor_Vel_Control(
    const X_V2_Vel_Command_t *commands,
    uint8_t command_count);

/*
  函数说明: 设置驱动器某类信息的自动回传周期；周期为 0 时关闭。
  调用示例: X_V2_Timed_Return_Info(addr, info_function, period_ms);
  参数意义: addr 为电机 CAN 地址。
            info_function 为驱动器信息功能码。
            period_ms 为周期，毫秒；回传配置中 0 表示关闭。
*/
HAL_StatusTypeDef X_V2_Timed_Return_Info(uint8_t addr,
                                         uint8_t info_function,
                                         uint16_t period_ms);

/*
  函数说明: 向指定驱动器请求一次指定信息。
  调用示例: X_V2_Read_Info(addr, info_function);
  参数意义: addr 为电机 CAN 地址。
            info_function 为驱动器信息功能码。
*/
HAL_StatusTypeDef X_V2_Read_Info(uint8_t addr,
                                 uint8_t info_function);

/*
  函数说明: 设置驱动器通信心跳超时和是否保存到掉电参数。
  调用示例: X_V2_Set_Heartbeat(addr, save, timeout_ms);
  参数意义: addr 为电机 CAN 地址。
            save 为是否写入驱动器掉电保存参数。
            timeout_ms 为等待或心跳超时，毫秒。
*/
HAL_StatusTypeDef X_V2_Set_Heartbeat(uint8_t addr,
                                     bool save,
                                     uint32_t timeout_ms);

/*
  函数说明: 查询单台驱动器实际心跳保护时间，结果由 CAN 接收解析器保存。
  调用示例: X_V2_Read_Heartbeat(1U);
  参数意义: addr 为非零电机地址；返回值只表示查询发送结果，不修改驱动器配置。
*/
HAL_StatusTypeDef X_V2_Read_Heartbeat(uint8_t addr);

/*
  函数说明: 发送立即停止指令。
  调用示例: X_V2_Stop_Now(addr, snF);
  参数意义: addr 为电机 CAN 地址。
            snF 为单机同步缓存标志；多电机位置命令中用于到位返回。
*/
HAL_StatusTypeDef X_V2_Stop_Now(uint8_t addr, bool snF);
/*
  函数说明: 发送同步触发命令，使缓存中的同步指令开始执行。
  调用示例: X_V2_Synchronous_motion(addr);
  参数意义: addr 为电机 CAN 地址。
*/
HAL_StatusTypeDef X_V2_Synchronous_motion(uint8_t addr);

#endif /* __X_V2_H */
