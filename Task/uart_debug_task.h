#ifndef TASK_UART_DEBUG_TASK_H
#define TASK_UART_DEBUG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 OPS9/HW101 接收，并通过 USART1 定期发送调试数据。 */
void UartDebugTask_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_UART_DEBUG_TASK_H */
