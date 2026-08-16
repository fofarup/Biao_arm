# Biao_arm

基于 STM32F407ZET6、FreeRTOS 和 CAN 总线的三轴机械臂控制工程。

当前电机控制代码按 **ZDT X42S 第二代闭环步进电机的 X 固件协议**实现。工程启动后会自动初始化 CAN，并对地址为 `1` 的电机执行一次位置测试。

> 上电后约 2 秒会自动使能电机并运动。首次联调请卸下机械负载，确认急停、限位和供电保护有效。

## 硬件与工具

- MCU：STM32F407ZET6
- CAN1：`PA11/CAN1_RX`、`PA12/CAN1_TX`
- 系统时钟：168 MHz；APB1 外设时钟：42 MHz
- CAN 收发器：外接 CAN 收发器后连接电机驱动器
- RTOS：FreeRTOS（CMSIS-RTOS2 接口）
- 构建工具：Keil MDK-ARM V5；工程文件为 `MDK-ARM/Biao_arm.uvprojx`
- 配置工具：STM32CubeMX；配置文件为 `Biao_arm.ioc`

## 电机端配置

电机端参数必须与代码一致，否则电机可能完全不响应：

| 参数 | 当前工程要求 |
| --- | --- |
| 固件协议 | X（不是 Emm） |
| 通信端口 | CAN |
| CAN 波特率 | 500 kbit/s |
| 校验方式 | 固定校验字节 `0x6B` |
| 应答模式 | `Receive` 或 `Both` |
| 电机地址 | `1`（启动测试使用） |

CAN 总线还需要正确连接 `CANH`、`CANL` 和公共地，并按总线拓扑安装终端电阻。若电机仍处于 UART 端口、Emm 固件或其他波特率，X 协议指令不会生效。电机固件切换需要使用厂家上位机/配置工具完成，本工程不会自动升级电机固件。

## CAN 帧格式

`Fun/zdt_can.c` 将一条电机命令拆分成一个或多个 CAN 扩展帧：

- 使用 29 位扩展帧；扩展 ID 为 `((地址 << 8) | 分包序号)`。
- 分包序号从 `0` 开始递增。
- 每帧第 1 个数据字节是功能码，后面最多携带 7 个命令数据字节。
- 命令末尾固定使用 `0x6B`。

当前 CAN1 配置为普通模式，分频系数 `6`、时间段 `BS1=11TQ`、`BS2=2TQ`，对应 500 kbit/s。

## X 协议接口

`Fun/X_V2.c` 和 `Fun/X_V2.h` 提供以下接口：

| 接口 | 用途 |
| --- | --- |
| `X_V2_Reset_Clog_Pro` | 清除堵转/过流保护状态 |
| `X_V2_En_Control` | 使能或关闭电机 |
| `X_V2_Vel_Control` | 速度控制 |
| `X_V2_Traj_Pos_Control` | 梯形速度位置控制 |
| `X_V2_Stop_Now` | 立即停止 |
| `X_V2_Synchronous_motion` | 执行同步运动 |
| `X_V2_Reset_CurPos_To_Zero` | 将当前位置清零 |

常用功能码如下：

- 清除保护：`0x0E 0x52 0x6B`
- 使能：`0xF3 0xAB state snF 0x6B`
- 速度：`0xF6 direction acceleration speed×10 snF 0x6B`
- 位置：`0xFD direction acceleration deceleration speed×10 position×10 mode snF 0x6B`
- 立即停止：`0xFE 0x98 snF 0x6B`

位置命令中的速度单位为 RPM，位置单位为角度；代码会分别按 `×10` 转换为协议要求的整数格式。

## 上电测试流程

`Core/Src/freertos.c` 中的 `StartDefaultTask` 当前执行一次以下流程：

1. 初始化 CAN 接收队列、过滤器和中断。
2. 等待约 2 秒，让电机驱动器完成上电初始化。
3. 清除地址 `1` 电机的堵转/过流保护状态。
4. 使能地址 `1` 电机。
5. 发送一次 X 协议位置命令：顺时针、速度 `200 RPM`、加速度 `200 RPM/s`、减速度 `100 RPM/s`，相对运动 `360°`。
6. 进入接收循环，保存最近收到的 CAN 帧，便于调试。

这只是单电机联调示例，不是连续速度控制。车辆或机械臂正式控制时，应在任务中按需要调用 `X_V2_Vel_Control` 或 `X_V2_Traj_Pos_Control`，并增加限位、急停和状态机保护。

## 工程结构

- `Core/Src/main.c`：HAL、时钟、GPIO、CAN 和 RTOS 启动
- `Core/Src/can.c`：CAN1 外设和引脚配置
- `Core/Src/freertos.c`：电机测试任务和 CAN 接收观察
- `Fun/zdt_can.c/.h`：CAN 扩展帧分包、发送、接收队列和统计量
- `Fun/X_V2.c/.h`：ZDT X42S X 固件协议命令
- `Fun/arm_kinematics.c/.h`：机械臂运动学
- `Fun/Emm_V5.c/.h`：旧 Emm 协议代码，仅作为历史参考，当前 Keil 工程未编译

## 构建与下载

1. 使用 Keil MDK-ARM 打开 `MDK-ARM/Biao_arm.uvprojx`。
2. 确认 `Fun/X_V2.c` 已加入工程，且 `Core/Src/freertos.c` 引用 `X_V2.h`。
3. 执行 Build/Rebuild，确认无编译错误后下载到 STM32F407ZET6。
4. 连接电机和 CAN 总线后复位开发板，观察电机动作及 CAN 应答。

当前开发环境未安装 Keil/ARMCC，因此无法在此处替代本地完成最终的 ARM 工具链构建；下载前请在 Keil 中实际编译一次。

## 无动作排查

优先检查以下项目：

1. 电机协议是否为 X，通信端口是否选择 CAN。
2. 电机地址、500 kbit/s 波特率和固定 `0x6B` 校验是否一致。
3. CANH/CANL 是否接反，CAN 收发器是否供电，控制板与驱动器是否共地。
4. 总线两端是否有终端电阻，驱动器是否有报警或急停锁定。
5. 观察 `zdt_can_tx_error_count`、`zdt_can_error_count` 和 `zdt_can_rx_count`：发送错误通常表示总线没有收到 ACK，接收计数为零则应继续检查应答模式和物理链路。

## 安全提示

电机上电后会自动执行测试动作。调试时请架空或卸载机械臂，限制运动范围，确认急停和断电措施可用。正式运行前应删除或禁用自动测试代码，并加入限位、故障复位和通信超时处理。

