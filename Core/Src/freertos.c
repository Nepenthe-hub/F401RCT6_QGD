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
#include "AD7705.h"
#include "TMC5160.h"
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
/* USER CODE BEGIN Variables */
#define EVT_TENSION_OVER   (1UL << 0)
#define EVT_TENSION_READY  (1UL << 1)  // 新增：去皮完成才允许检测
#define TENSION_LIMIT_KG   3.5f
/* USER CODE END Variables */
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TensionTask */
osThreadId_t TensionTaskHandle;
const osThreadAttr_t TensionTask_attributes = {
  .name = "TensionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for tensionQueue */
osMessageQueueId_t tensionQueueHandle;
const osMessageQueueAttr_t tensionQueue_attributes = {
  .name = "tensionQueue"
};
/* Definitions for motorEvent */
osEventFlagsId_t motorEventHandle;
const osEventFlagsAttr_t motorEvent_attributes = {
  .name = "motorEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void *argument);
void StartTensionTask(void *argument);

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

  /* Create the queue(s) */
  /* creation of tensionQueue */
  tensionQueueHandle = osMessageQueueNew (4, sizeof(float), &tensionQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of TensionTask */
  TensionTaskHandle = osThreadNew(StartTensionTask, NULL, &TensionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of motorEvent */
  motorEventHandle = osEventFlagsNew(&motorEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  float tension = 0.0f;

  AD7705_Init();
  AD7705_Tare();

  // 通知 TensionTask 去皮完成，可以开始检测
  osEventFlagsSet(motorEventHandle, EVT_TENSION_READY);
  printf("[MOTOR] 去皮完成，准备启动\r\n");

  osDelay(500);
  TMC_Run_Velocity(30000);
  printf("[MOTOR] 电机启动\r\n");

  for (;;)
  {
    if (osEventFlagsGet(motorEventHandle) & EVT_TENSION_OVER) {
      TMC_Stop();
      osEventFlagsClear(motorEventHandle, EVT_TENSION_OVER);
      printf("[MOTOR] 拉力到达限制，电机已停止\r\n");
      osThreadSuspend(NULL);
    }

    if (osMessageQueueGet(tensionQueueHandle, &tension, NULL, 0) == osOK) {
      printf("[MON] v=%ld  tension=%.3f kg\r\n",
             (long)TMC_Get_Velocity(), tension);
    }

    osDelay(50);
  }
  /* USER CODE END StartMotorTask */
}
/* USER CODE BEGIN Header_StartTensionTask */
/**
* @brief Function implementing the TensionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTensionTask */
__weak void StartTensionTask(void *argument)
{
  /* USER CODE BEGIN StartTensionTask */
  float   tension  = 0.0f;
  float   last_val = 0.0f;
  uint8_t over_cnt = 0;

  // 等待 MotorTask 去皮完成信号，永久等待
  osEventFlagsWait(motorEventHandle, EVT_TENSION_READY,
                   osFlagsWaitAny, osWaitForever);
  printf("[TENSION] 开始拉力监测\r\n");

  for (;;)
  {
    tension = AD7705_TryReadWeightKg();

    if (tension >= 0.0f) {
      last_val = tension;
      osMessageQueuePut(tensionQueueHandle, &last_val, 0, 0);

      if (last_val > TENSION_LIMIT_KG) {
        over_cnt++;
        if (over_cnt >= 3) {
          osEventFlagsSet(motorEventHandle, EVT_TENSION_OVER);
          over_cnt = 0;
        }
      } else {
        over_cnt = 0;
      }
    }

    osDelay(20);
  }
  /* USER CODE END StartTensionTask */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

