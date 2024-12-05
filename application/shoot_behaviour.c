#include "shoot_behaviour.h"
#include "shoot_task.h"
#include "tim.h"

//extern osTimerId ShootTimerHandle;


static void shoot_trig_motor_behaviour_set(shoot_control_t *shoot_behaviour);


static void shoot_trig_motor_behaviour_set(shoot_control_t *shoot_behaviour)
{
	if(switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[0]))//右中
	{
		//左中，下
		if(switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]) || switch_is_down(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
		{
			shoot_behaviour->trig_fire_mode = Serial_fire;
		}
		else
		{
			shoot_behaviour->trig_fire_mode = Cease_fire;
		}
//		else if(switch_is_down(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
//		{
//			shoot_behaviour->trig_fire_mode = Single_fire;
//		}
//		else if(switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
//		{
//			shoot_behaviour->trig_fire_mode = Cease_fire;
//		}
	}
	else
	{
		shoot_behaviour->trig_fire_mode = Cease_fire;
	}
	if (toe_is_error(DBUS_TOE))
	{
		shoot_behaviour->trig_fire_mode = Cease_fire;
	}

	shoot_behaviour->last_trig_fire_mode = shoot_behaviour->trig_fire_mode;
}

void shoot_trig_motor_mode_set(shoot_control_t *shoot_mode)
{
	shoot_trig_motor_behaviour_set(shoot_mode);
}
//static void shoot_trig_motor_behaviour_set(shoot_control_t *shoot_behaviour)
//{
//	static uint8_t first=1;
//	if(switch_is_up(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
//	{
//		if(first)
//		{
//			osTimerStart(ShootTimerHandle,500);
//			first=0;
//		}
//		if(shoot_behaviour->last_trig_fire_mode == Cease_fire)
//		shoot_control.trig_fire_mode = Single_fire;

//	}
//	else if(switch_is_down(shoot_behaviour->shoot_rc_ctrl->rc.s[TIRG_MODE_CHANNEL]))
//	{
//		first=1;
//		shoot_behaviour->trig_fire_mode = Cease_fire;
//	}
//	else
//	{
//		first=1;
//		shoot_behaviour->trig_fire_mode = Cease_fire;
//	}
//	shoot_behaviour->last_trig_fire_mode = shoot_behaviour->trig_fire_mode;
//}



////2500单点,385连发
////宏定义用来给定时器arr寄存器赋值
////__HAL_TIM_SET_AUTORELOAD(&htim5,2500);
////__HAL_TIM_SET_AUTORELOAD(&htim5,385);
//void shoot_trig_motor_mode_set(shoot_control_t *shoot_mode)
//{
//	shoot_trig_motor_behaviour_set(shoot_mode);
//	
//	if(shoot_mode->trig_fire_mode ==Serial_fire)
//	{
//		HAL_TIM_Base_Stop_IT(&htim5);
//		__HAL_TIM_SET_AUTORELOAD(&htim5,Serial_ARR);
//		HAL_TIM_Base_Start_IT(&htim5);

//	}
//	else if(shoot_mode->trig_fire_mode ==Single_fire)
//	{
//		HAL_TIM_Base_Stop_IT(&htim5);
//		__HAL_TIM_SET_AUTORELOAD(&htim5,Single_ARR);
//		HAL_TIM_Base_Start_IT(&htim5);

//	}
//	else if(shoot_mode->trig_fire_mode == Cease_fire)
//	{
//			HAL_TIM_Base_Stop_IT(&htim5);
//	}
//}


