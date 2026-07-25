# Biao_arm

基于 STM32F407ZET6、FreeRTOS 和 CAN 总线的三轴机械臂控制工程，使用 Keil
MDK-ARM V5 构建。

## 硬件与通信配置

- MCU：STM32F407ZET6，系统时钟 168 MHz
- CAN1：PA11/RX、PA12/TX
- CAN 速率：500 Kbps
- 电机：ZDT_X42S，Emm 固件
- 通信协议：扩展帧自由协议，固定校验字节 `0x6B`
- FreeRTOS：CMSIS-RTOS2

CAN 速率按《ZDT_X42S 第二代闭环步进电机用户手册 V1.0.5》的出厂默认值配置。
如果电机已通过上位机改为其他速率，需要同步修改 `Core/Src/can.c` 和
`Biao_arm.ioc`。

电机端还需要确认以下配置：

- 通信端口复用模式为 `CAN`
- CAN 通信速率为 `500K`
- 通信校验方式为自由协议固定 `0x6B`
- 控制命令应答方式建议为 `Receive` 或 `Both`

## 代码结构

- `Core/Src/main.c`：MCU、时钟、GPIO、CAN 和 FreeRTOS 启动
- `Core/Src/freertos.c`：电机控制任务
- `Core/Src/can.c`：CAN1 外设配置
- `Fun/zdt_can.c`：ZDT CAN 扩展帧分包、发送和接收队列
- `Fun/Emm_V5.c`：Emm 固件控制指令
- `Fun/arm_kinematics.c`：机械臂逆运动学

## 当前演示流程

`motorTask` 启动后：

1. 初始化 CAN 接收队列和中断；
2. 等待电机上电初始化；
3. 使能地址 1 的电机；
4. 以 200 RPM、加速度档位 10，相对运动 3200 脉冲；
5. 等待并保存电机返回的 CAN 帧。

默认 1.8° 电机、16 细分时，3200 脉冲对应一圈。

## CAN 分包格式

工程按手册第 4.2 节实现：

- 使用 29 位扩展帧；
- 扩展帧 ID 为 `(Addr << 8) | Packet`；
- `Packet` 从 0 开始；
- 每个分包的第一个数据字节均为功能码；
- 每帧最多携带功能码加 7 字节命令数据。

## 构建

使用 Keil MDK-ARM 打开：

```text
MDK-ARM/Biao_arm.uvprojx
```

最近一次验证使用 ARMCC V5.06 update 7，构建结果为 0 errors、0 warnings。

## 安全提示

上电测试前请架空或卸载机械负载，确认 CAN 收发器、电机供电、终端电阻和急停
措施正确。当前示例任务上电后会主动使能并转动地址 1 的电机。

