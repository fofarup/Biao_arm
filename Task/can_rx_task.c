#include "can_rx_task.h"

#include "cmsis_os.h"

#include "chassis_feedback.h"
#include "zdt_can.h"

/*
 * CAN 中断只负责把帧放入队列；本任务在普通任务上下文中解析协议，
 * 避免在中断里做浮点换算和较长的状态处理。
 */
void CanRxTask_Run(void *argument)
{
    ZDT_CanRxFrame_t frame;

    (void)argument;

    /*
     * 底盘任务使用地址 5~8 的独立接收队列；机械臂原有队列仍由
     * arm_motion.c 使用，避免破坏机械臂的到位/故障等待逻辑。
     */
    if (ZDT_CAN_Init() != HAL_OK)
    {
        Error_Handler();
        for (;;)
        {
            osDelay(1000U);
        }
    }

    ChassisFeedback_Init();

    for (;;)
    {
        /* 阻塞等待，队列为空时不会占用 CPU。 */
        if (ZDT_CAN_GetChassisRx(&frame, osWaitForever) == osOK)
        {
            ChassisFeedback_ProcessFrame(&frame);
        }
    }
}
