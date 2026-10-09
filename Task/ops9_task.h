#ifndef OPS9_TASK_H
#define OPS9_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS 中由 CubeMX 创建的 OPS9 任务入口。 */
void Ops9Task_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* OPS9_TASK_H */
