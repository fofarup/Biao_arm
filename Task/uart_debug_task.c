#include "uart_debug_task.h"

#include "cmsis_os.h"
#include "end_level_task.h"
#include "hw101.h"
#include "ops9.h"
#include "usart.h"

#include <stddef.h>
#include <stdio.h>

/* USART1 每 50 ms 发送一行数据。 */
#define UART_DEBUG_TASK_PERIOD_MS       (500U)
#define UART_DEBUG_TX_TIMEOUT_MS        (20U)

OPS9_Data_t ops9_data;
HW101_Data_t hw101_data;
EndLevelTelemetry_t end_level_data;


/*
 * 按固定小数位输出浮点数，避免依赖 ARMCC printf 的 %f 支持，
 * 也可以减少串口调试代码的运行开销。
 */
static long UartDebug_RoundScaled(float value, float scale)
{
    float scaled;

    scaled = value * scale;

    if (scaled >= 0.0f)
    {
        return (long)(scaled + 0.5f);
    }

    return (long)(scaled - 0.5f);
}

static void UartDebug_FormatFixed(char *text,
                                  size_t text_size,
                                  long scaled,
                                  unsigned long scale,
                                  unsigned int decimals)
{
    unsigned long magnitude;
    unsigned long whole;
    unsigned long fraction;
    const char *sign;

    if ((text == NULL) || (text_size == 0U) || (scale == 0UL))
    {
        return;
    }

    if (scaled < 0L)
    {
        sign = "-";
        magnitude = (unsigned long)(-(scaled + 1L)) + 1UL;
    }
    else
    {
        sign = "";
        magnitude = (unsigned long)scaled;
    }

    whole = magnitude / scale;
    fraction = magnitude % scale;

    (void)snprintf(text,
                   text_size,
                   "%s%lu.%0*lu",
                   sign,
                   whole,
                   (int)decimals,
                   fraction);
}

void UartDebugTask_Run(void *argument)
{


	
    char x_text[20];
    char y_text[20];
    char yaw_text[20];
    char roll_text[20];
    char line[96];
    int length;
    uint32_t next_wake;

    (void)argument;

    /* 以绝对唤醒时间运行，避免发送耗时造成周期逐渐漂移。 */
    next_wake = osKernelGetTickCount() + UART_DEBUG_TASK_PERIOD_MS;

    for (;;)
    {
        /* 读取三个模块已经缓存好的最新数据，不在此处直接访问传感器。 */
        (void)OPS9_GetLatest(&ops9_data);
        (void)HW101_GetData(&hw101_data);
        EndLevelTask_GetTelemetry(&end_level_data);

        UartDebug_FormatFixed(x_text,
                              sizeof(x_text),
                              UartDebug_RoundScaled(ops9_data.x_mm, 100.0f),
                              100UL,
                              2U);
        UartDebug_FormatFixed(y_text,
                              sizeof(y_text),
                              UartDebug_RoundScaled(ops9_data.y_mm, 100.0f),
                              100UL,
                              2U);
        UartDebug_FormatFixed(yaw_text,
                              sizeof(yaw_text),
                              UartDebug_RoundScaled(hw101_data.yaw_deg, 100.0f),
                              100UL,
                              2U);
        UartDebug_FormatFixed(roll_text,
                              sizeof(roll_text),
                              UartDebug_RoundScaled(end_level_data.roll_deg,
                                                    100.0f),
                              100UL,
                              2U);

        /* 只发送四个数据，不附带状态字段。 */
        length = snprintf(line,
                          sizeof(line),
                          "X=%s,Y=%s,YAW=%s,ROLL=%s\r\n",
                          x_text,
                          y_text,
                          yaw_text,
                          roll_text);

        if ((length > 0) && ((size_t)length < sizeof(line)))
        {
            /* USART1 只由本任务发送，避免多个任务同时占用串口。 */
            (void)HAL_UART_Transmit(&huart1,
                                    (uint8_t *)line,
                                    (uint16_t)length,
                                    UART_DEBUG_TX_TIMEOUT_MS);
        }

        (void)osDelayUntil(next_wake);
        next_wake += UART_DEBUG_TASK_PERIOD_MS;
    }
}
