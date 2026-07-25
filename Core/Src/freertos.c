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
#include "Emm_V5.h"
#include "zdt_can.h"
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
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* creation of motorTask */
  motorTaskHandle = osThreadNew(StartDefaultTask, NULL, &motorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the motorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
		ZDT_CanRxFrame_t rx_frame;

    if (ZDT_CAN_Init() != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 等待电机驱动器上电初始化。
     */
    osDelay(2000);

    /*
     * 地址1，使能，不使用同步缓存。
     */
    if (Emm_V5_En_Control(1, true, false) != HAL_OK)
    {
        Error_Handler();
    }

    osDelay(100);

    /*
     * 地址1
     * CW
     * 200 RPM
     * 加速度10
     * 3200脉冲
     * 相对位置
     * 不使用同步缓存
     */
    if (Emm_V5_Pos_Control(
            1,
            EMM_V5_DIR_CW,
            200,
            10,
            3200,
            EMM_V5_POS_RELATIVE_TO_TARGET,
            false) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        /*
         * 接收电机返回帧。
         * 这里先保存和观察，不立即假设返回格式。
         */
        if (ZDT_CAN_GetRx(
                &rx_frame,
                osWaitForever) == osOK)
        {
            /*
             * 在这里给rx_frame打断点，
             * 观察ID、DLC和Data。
             */
        }
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

