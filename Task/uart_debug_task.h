#ifndef TASK_UART_DEBUG_TASK_H
#define TASK_UART_DEBUG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif




/* USART1 数据发送任务入口，由 freertos.c 的 CubeMX wrapper 调用。 */
void UartDebugTask_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_UART_DEBUG_TASK_H */
