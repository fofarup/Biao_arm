#ifndef CAN_RX_TASK_H
#define CAN_RX_TASK_H

/* 底盘 CAN 接收任务：从专用队列取原始帧，再交给反馈解析模块。 */

#ifdef __cplusplus
extern "C" {
#endif

void CanRxTask_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RX_TASK_H */
