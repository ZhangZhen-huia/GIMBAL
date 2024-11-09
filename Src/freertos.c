/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "shoot_task.h"
#include "shoot_behaviour.h"
#include "event_groups.h"
#include "tim.h"
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

EventGroupHandle_t my_shootEventGroupHandle;

/* USER CODE END Variables */
osThreadId TEST_TASKHandle;
osThreadId GIMBAL_TASKHandle;
osThreadId INS_TASKHandle;
osThreadId DETECT_TASKHandle;
osThreadId SHOOT_TASKHandle;
osThreadId VOFA_TASKHandle;
osThreadId AIMBOTS_TASKHandle;
osTimerId ShootTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void gimbal_task(void const * argument);
void INS_task(void const * argument);
void detect_task(void const * argument);
void shoot_task(void const * argument);
void vofa_task(void const * argument);
void aimbots_task(void const * argument);
void ShootTimer_Callback(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	my_shootEventGroupHandle = xEventGroupCreate();
	if(my_shootEventGroupHandle == NULL)
	{
		return;
	}
	HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of ShootTimer */
  osTimerDef(ShootTimer, ShootTimer_Callback);
  ShootTimerHandle = osTimerCreate(osTimer(ShootTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TEST_TASK */
  osThreadDef(TEST_TASK, test_task, osPriorityBelowNormal, 0, 128);
  TEST_TASKHandle = osThreadCreate(osThread(TEST_TASK), NULL);

  /* definition and creation of GIMBAL_TASK */
  osThreadDef(GIMBAL_TASK, gimbal_task, osPriorityNormal, 0, 256);
  GIMBAL_TASKHandle = osThreadCreate(osThread(GIMBAL_TASK), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_task, osPriorityRealtime, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of DETECT_TASK */
  osThreadDef(DETECT_TASK, detect_task, osPriorityBelowNormal, 0, 256);
  DETECT_TASKHandle = osThreadCreate(osThread(DETECT_TASK), NULL);

  /* definition and creation of SHOOT_TASK */
  osThreadDef(SHOOT_TASK, shoot_task, osPriorityNormal, 0, 256);
  SHOOT_TASKHandle = osThreadCreate(osThread(SHOOT_TASK), NULL);

  /* definition and creation of VOFA_TASK */
  osThreadDef(VOFA_TASK, vofa_task, osPriorityBelowNormal, 0, 128);
  VOFA_TASKHandle = osThreadCreate(osThread(VOFA_TASK), NULL);

  /* definition and creation of AIMBOTS_TASK */
  osThreadDef(AIMBOTS_TASK, aimbots_task, osPriorityNormal, 0, 128);
  AIMBOTS_TASKHandle = osThreadCreate(osThread(AIMBOTS_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the TEST_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
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

/* USER CODE BEGIN Header_INS_task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_task */
__weak void INS_task(void const * argument)
{
  /* USER CODE BEGIN INS_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_task */
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

/* USER CODE BEGIN Header_aimbots_task */
/**
* @brief Function implementing the AIMBOTS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_aimbots_task */
__weak void aimbots_task(void const * argument)
{
  /* USER CODE BEGIN aimbots_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END aimbots_task */
}

/* ShootTimer_Callback function */
void ShootTimer_Callback(void const * argument)
{
  /* USER CODE BEGIN ShootTimer_Callback */

	if(switch_is_up(shoot_control.shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
	{
		//xEventGroupSetBits(my_shootEventGroupHandle,ShootEvent_1);
		shoot_control.trig_fire_mode = Serial_fire;
	}


  /* USER CODE END ShootTimer_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
