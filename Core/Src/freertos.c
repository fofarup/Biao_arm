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
#include "end_level_task.h"
#include "uart_debug_task.h"
#include "chassis_task.h"
#include "main_task.h"
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
/* Definitions for endLevelTask */
osThreadId_t endLevelTaskHandle;
const osThreadAttr_t endLevelTask_attributes = {
  .name = "endLevelTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for uartDebugTask */
osThreadId_t uartDebugTaskHandle;
const osThreadAttr_t uartDebugTask_attributes = {
  .name = "uartDebugTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartEndLevelTask(void *argument);
void StartUartDebugTask(void *argument);
void StartChassisTask(void *argument);

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
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main_task */
  main_taskHandle = osThreadNew(StartMainTask, NULL, &main_task_attributes);

  /* creation of endLevelTask */
 // endLevelTaskHandle = osThreadNew(StartEndLevelTask, NULL, &endLevelTask_attributes);

  /* creation of uartDebugTask */
  uartDebugTaskHandle = osThreadNew(StartUartDebugTask, NULL, &uartDebugTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the main_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
  MainTask_Run(argument);
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartEndLevelTask */
/**
* @brief Function implementing the endLevelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEndLevelTask */
void StartEndLevelTask(void *argument)
{
  /* USER CODE BEGIN StartEndLevelTask */
  EndLevelTask_Run(argument);
  /* USER CODE END StartEndLevelTask */
}

/* USER CODE BEGIN Header_StartUartDebugTask */
/**
* @brief Function implementing the uartDebugTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartDebugTask */
void StartUartDebugTask(void *argument)
{
  /* USER CODE BEGIN StartUartDebugTask */
  UartDebugTask_Run(argument);
  /* USER CODE END StartUartDebugTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  ChassisTask_Run(argument);
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

