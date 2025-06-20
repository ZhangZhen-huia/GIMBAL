/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
osThreadId INS_TASKHandle;
osThreadId GIMBAL_TASKHandle;
osThreadId DETECT_TASKHandle;
osThreadId SHOOT_TASKHandle;
osThreadId VOFA_TASKHandle;
osThreadId COMMUNICATE_TASHandle;
osThreadId KEY_TASKHandle;
osThreadId BUZZER_TASKHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_task(void const * argument);
void gimbal_task(void const * argument);
void detect_task(void const * argument);
void shoot_task(void const * argument);
void vofa_task(void const * argument);
void communicate_task(void const * argument);
void key_task(void const * argument);
void buzzer_effects_task(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_task, osPriorityRealtime, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of GIMBAL_TASK */
  osThreadDef(GIMBAL_TASK, gimbal_task, osPriorityNormal, 0, 256);
  GIMBAL_TASKHandle = osThreadCreate(osThread(GIMBAL_TASK), NULL);

  /* definition and creation of DETECT_TASK */
  osThreadDef(DETECT_TASK, detect_task, osPriorityBelowNormal, 0, 128);
  DETECT_TASKHandle = osThreadCreate(osThread(DETECT_TASK), NULL);

  /* definition and creation of SHOOT_TASK */
  osThreadDef(SHOOT_TASK, shoot_task, osPriorityNormal, 0, 128);
  SHOOT_TASKHandle = osThreadCreate(osThread(SHOOT_TASK), NULL);

  /* definition and creation of VOFA_TASK */
  osThreadDef(VOFA_TASK, vofa_task, osPriorityIdle, 0, 128);
  VOFA_TASKHandle = osThreadCreate(osThread(VOFA_TASK), NULL);

  /* definition and creation of COMMUNICATE_TAS */
  osThreadDef(COMMUNICATE_TAS, communicate_task, osPriorityAboveNormal, 0, 256);
  COMMUNICATE_TASHandle = osThreadCreate(osThread(COMMUNICATE_TAS), NULL);

  /* definition and creation of KEY_TASK */
  osThreadDef(KEY_TASK, key_task, osPriorityLow, 0, 128);
  KEY_TASKHandle = osThreadCreate(osThread(KEY_TASK), NULL);

  osThreadDef(BUZZER_EFFECTS_TASK, buzzer_effects_task, osPriorityBelowNormal, 0, 128);
  BUZZER_TASKHandle = osThreadCreate(osThread(BUZZER_EFFECTS_TASK), NULL);
	
	
	/* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the GIMBAL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void const * argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_detect_task */
/**
* @brief Function implementing the DETECT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_detect_task */
__weak void detect_task(void const * argument)
{
  /* USER CODE BEGIN detect_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END detect_task */
}

/* USER CODE BEGIN Header_shoot_task */
/**
* @brief Function implementing the SHOOT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_task */
__weak void shoot_task(void const * argument)
{
  /* USER CODE BEGIN shoot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END shoot_task */
}

/* USER CODE BEGIN Header_vofa_task */
/**
* @brief Function implementing the VOFA_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vofa_task */
__weak void vofa_task(void const * argument)
{
  /* USER CODE BEGIN vofa_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vofa_task */
}

/* USER CODE BEGIN Header_communicate_task */
/**
* @brief Function implementing the COMMUNICATE_TAS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_communicate_task */
__weak void communicate_task(void const * argument)
{
  /* USER CODE BEGIN communicate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END communicate_task */
}

/* USER CODE BEGIN Header_key_task */
/**
* @brief Function implementing the KEY_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_key_task */
__weak void key_task(void const * argument)
{
  /* USER CODE BEGIN key_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END key_task */
}


__weak void buzzer_effects_task(void const * argument)
{
	for(;;)
	osDelay(1);
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
