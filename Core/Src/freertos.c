/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main_task.h"
#include "arm_end_level_task.h"
#include "chassis_task.h"
#include "ultrasonic_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for main_task */
osThreadId_t main_taskHandle;
const osThreadAttr_t main_task_attributes = {
  .name = "main_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for armEndLevelTask */
osThreadId_t armEndLevelTaskHandle;
const osThreadAttr_t armEndLevelTask_attributes = {
  .name = "armEndLevelTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ultrasonicTask */
osThreadId_t ultrasonicTaskHandle;
const osThreadAttr_t ultrasonicTask_attributes = {
  .name = "ultrasonicTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartArmEndLevelTask(void *argument);
void StartChassisTask(void *argument);
void StartUltrasonicTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(StartMainTask, NULL, &main_task_attributes);

  /* creation of armEndLevelTask */
  armEndLevelTaskHandle = osThreadNew(StartArmEndLevelTask, NULL, &armEndLevelTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);

  /* creation of ultrasonicTask */
  ultrasonicTaskHandle = osThreadNew(StartUltrasonicTask, NULL, &ultrasonicTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (main_taskHandle == NULL || armEndLevelTaskHandle == NULL || chassisTaskHandle == NULL || ultrasonicTaskHandle == NULL) Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/*
  函数说明: 将 CubeMX 任务入口连接到业务任务。
  调用示例: 由 osThreadNew 自动启动。
  参数意义: argument 为任务参数，当前未使用。
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  MainTask_Run(argument);
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartArmEndLevelTask */
/*
  函数说明: 将 CubeMX 任务入口连接到业务任务。
  调用示例: 由 osThreadNew 自动启动。
  参数意义: argument 为任务参数，当前未使用。
*/
/* USER CODE END Header_StartArmEndLevelTask */
void StartArmEndLevelTask(void *argument)
{
  /* USER CODE BEGIN StartArmEndLevelTask */
  ArmEndLevelTask_Run(argument);
  /* USER CODE END StartArmEndLevelTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/*
  函数说明: 将 CubeMX 任务入口连接到业务任务。
  调用示例: 由 osThreadNew 自动启动。
  参数意义: argument 为任务参数，当前未使用。
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  ChassisTask_Run(argument);
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartUltrasonicTask */
/*
  函数说明: 将 CubeMX 任务入口连接到业务任务。
  调用示例: 由 osThreadNew 自动启动。
  参数意义: argument 为任务参数，当前未使用。
*/
/* USER CODE END Header_StartUltrasonicTask */
void StartUltrasonicTask(void *argument)
{
  /* USER CODE BEGIN StartUltrasonicTask */
  UltrasonicTask_Run(argument);
  /* USER CODE END StartUltrasonicTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

