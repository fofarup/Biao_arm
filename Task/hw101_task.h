#ifndef HW101_TASK_H
#define HW101_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hw101.h"

/* 用于 Keil 调试窗口观察数据 */
extern volatile HW101_Data_t hw101_debug_data;

/* HW101 任务入口 */
void Hw101Task_Run(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* HW101_TASK_H */
