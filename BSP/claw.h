/*
  程序说明: FashionStar 单总线夹爪舵机的角度接口，使用舵机极速模式。
  软件环境: Keil MDK / ARM Compiler 5.06u7，STM32Cube HAL，FreeRTOS CMSIS-RTOS2。
  硬件环境: STM32F407ZET6 机械臂与麦克纳姆移动平台。
  日    期: 2026-10-25
*/

#ifndef CLAW_H
#define CLAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 必须与实际舵机的 ID、EEPROM 波特率一致；这些宏不会修改舵机内部设置。 */
#define CLAW_SERVO_ID                 (0U)
#define CLAW_UART_BAUDRATE            (115200U)

/* 新协议使用舵机自身的单圈角度，首次换装应先确认零位及夹爪机械活动范围。 */
#define CLAW_MIN_ANGLE_DEG            (-180.0f)
#define CLAW_MAX_ANGLE_DEG            (180.0f)

/*
  函数说明: 经 PB10/USART3_TX 发送 FashionStar 0x08 单圈绝对角度指令，只发不收。
  接线要求: PB10 接与参考工程兼容的舵机单总线接口，控制板与舵机共地。
            USART3 保持 115200、8N1；PD9/USART3_RX 继续接 OPS9，不接舵机回包。
            若原接线使用了单总线转接板，沿用转接板，不能把 TTL 引脚直连 RS-485 A/B。
  发送行为: 完整帧发完后 PB10 切输入高阻；连续发令至少间隔 5 ms，不解析应答。
  返回说明: HAL_OK 仅表示完整指令已发出，不能证明舵机到位或夹持成功。
            参数无效返回 HAL_ERROR，发送占用返回 HAL_BUSY，串口超时返回 HAL_TIMEOUT。
  使用方法: 在任务中调用，只传目标角度；运动由舵机以极速模式执行，函数不等待到位。
            其他模块不能同时直接向 USART3 发送数据；无需新增 DMA、接收任务或定时器。
  调用示例: Claw_SetAngle(40.0f); 本地顺序动作需要的等待由业务流程安排。
            上位机 0003 的 X 仍是角度（0～180 度），发送成功后等待 300 ms 再回 0012。
  参数意义: angle_deg 为舵机内部零位下的绝对角度，-180～180 度，编码分辨率 0.1 度。
  协议说明: 0x08 的时间字段固定为 0，表示极速模式；实际速度取决于舵机及负载。
            300 ms 回包等待位于 main_task.h，与运动速度无关，也不代表实际到位。
*/
HAL_StatusTypeDef Claw_SetAngle(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* CLAW_H */
