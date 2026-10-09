#include "hw101_task.h"

#include "cmsis_os.h"

/* 方便在 Keil Watch 窗口中直接查看 */
volatile HW101_Data_t hw101_debug_data;

void Hw101Task_Run(void *argument)
{
    HW101_Data_t data;

    (void)argument;

    /*
     * 只启动一次串口接收。
     * 后续每个字节都由中断自动接收。
     */
    if (HW101_StartReceive() != HAL_OK)
    {
        /*
         * 启动失败时保留任务，便于调试。
         */
        for (;;)
        {
            osDelay(1000U);
        }
    }

    for (;;)
    {
        /*
         * 读取最近一次有效角度。
         * 接收工作由 USART2 中断完成。
         */
        if (HW101_GetData(&data) != 0U)
        {
            hw101_debug_data = data;
        }

        osDelay(10U);
    }
}
