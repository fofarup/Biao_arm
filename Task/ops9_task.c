#include "ops9_task.h"

#include "cmsis_os.h"
#include "ops9.h"

/* 任务只负责读取缓存和链路监测，不承担底盘控制。 */
#define OPS9_TASK_PERIOD_MS    (10U)

void Ops9Task_Run(void *argument)
{
    OPS9_Data_t data;

    (void)argument;

    /* 任务启动时开启 USART3 的单字节接收中断。 */
    if (OPS9_StartReceive() != HAL_OK)
    {
        /* 初始化失败时保留任务，便于在线调试。 */
        for (;;)
        {
            osDelay(1000U);
        }
    }

    for (;;)
    {
        /* 当前阶段只读取并缓存 OPS9 数据，不控制底盘或机械臂。 */
        (void)OPS9_GetLatest(&data);

        /* 以后可以在这里增加超时报警或串口调试输出。 */
        osDelay(OPS9_TASK_PERIOD_MS);
    }
}
