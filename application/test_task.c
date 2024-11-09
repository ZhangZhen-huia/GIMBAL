/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "tim.h"
#include "bsp_buzzer.h"
#include "shoot_task.h"
uint8_t Bee_flag=1;
uint16_t psc = 1;
uint16_t pwm = 5000;



/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void test_task(void const * argument)
{

    while(1)
    {

			if(Bee_flag==1)
			{	
					Bee_flag=0;
					buzzer_on(psc, pwm);
					osDelay(500);
					buzzer_off();

			}

        osDelay(10);
    }
}

