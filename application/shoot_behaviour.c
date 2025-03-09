#include "shoot_behaviour.h"
#include "referee.h"
#include "shoot_task.h"
#include "tim.h"
#include "communicate_task.h"
#include "gimbal_task.h"
#include "key_task.h"


static void shoot_motor_behaviour_set(shoot_control_t *shoot_behaviour)
{
	static uint16_t shoot_flag = 0;
	static uint8_t shoot_force = 0;
	
	if(shoot_behaviour == NULL)
	{
		return;
	}
	
	//遥控器掉线就关闭发射结构
	if (POWER_OFF)
	{
		//关闭发射结构总开关
		shoot_behaviour->shoot_agency_state = SHOOT_OFF;
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;

	}
	
	//强制开火信号
	if(shoot_behaviour->fric_mode == START && Mouse_Data.mouse_l)//shoot_behaviour->shoot_rc_ctrl->rc.ch[0] == -660 && shoot_behaviour->shoot_rc_ctrl->rc.ch[1] == 660)
	{
		shoot_force = 1;
	}
	else
	{
		shoot_force = 0;
	}
	
	//强制开火：强制给拨弹盘开火信号，主要用于自瞄模式下
	if( shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START && shoot_force)
	{
		shoot_behaviour->trig_mode = Start_fire;
	}
	else
	{
		shoot_behaviour->trig_mode = Cease_fire;
	}
	
	//如果发射机构总开关关了，那么所有的都关闭
	if(shoot_behaviour->shoot_agency_state == SHOOT_OFF)
	{
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;
		shoot_flag = 0;
		
	}
	
	
	//发射机构开关判断，在这里多加了一级发射机构的状态是为了防止遥控器误触导致摩擦轮开启
	if(((shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000)) || (Key_ScanValue.Key_Value.B && !Key_ScanValue.Key_Value_Last.B))
	{
		if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000)
		{
			shoot_flag++;
			if(shoot_flag >= 80)
			{
				shoot_behaviour->shoot_agency_state = SHOOT_OFF;//发射机构关
				shoot_flag=0;
			}
		}
		else if((Key_ScanValue.Key_Value.B && !Key_ScanValue.Key_Value_Last.B))
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
		}

	}
	else if((shoot_behaviour->shoot_rc_ctrl->rc.ch[4] ==660 && (!switch_is_up(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]))) || (Key_ScanValue.Key_Value.G && !Key_ScanValue.Key_Value_Last.G))//右中
	{
		shoot_behaviour->shoot_agency_state = SHOOT_ON;//发射结构开
		shoot_flag = 0;
	}
	else
	{
		shoot_flag=0;
	}
	
	//摩擦轮开启判断
		if(shoot_behaviour->shoot_agency_state == SHOOT_ON)	
		{	
			if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000 || (Key_ScanValue.Key_Value.G && !Key_ScanValue.Key_Value_Last.G))	
					shoot_behaviour->fric_mode = START;	
		}	
		
		else
		{
			shoot_behaviour->fric_mode = STOP;	
		}


	/*-- 模式选择 --*/
	//不是自瞄模式
	if(gimbal_control.gimbal_behaviour != GIMBAL_AUTO_ANGLE)
	{				
				//拨弹盘
			if(shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START)
			{
				if(shoot_behaviour->shoot_agency_state == SHOOT_OFF || shoot_behaviour->fric_mode == STOP)
						shoot_behaviour->trig_mode = Cease_fire;
				
				else if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] == 660 || Mouse_Data.mouse_l)
				{
					shoot_behaviour->trig_mode = Start_fire;
				}
				
				else 
				{
					shoot_behaviour->trig_mode = Cease_fire;
				}
			}
			else
			{
				shoot_behaviour->trig_mode = Cease_fire;
			}
	}	
	

	//自瞄模式
	else if(gimbal_control.gimbal_behaviour == GIMBAL_AUTO_ANGLE)
	{
		if(POWER_OFF)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_behaviour->fric_mode = STOP;
			shoot_behaviour->trig_mode = Cease_fire;
		}

					//拨弹盘开启判断
				if((shoot_behaviour->fric_mode == START &&  auto_data.auto_fireFlag) || shoot_force)//shoot_behaviour->auto_fireFlag[0] == fire)
				{
					
					shoot_behaviour->trig_mode = Start_fire;
				}
				else
				{
					shoot_behaviour->trig_mode = Cease_fire;
				}			

		
	}

	//状态赋值
	shoot_behaviour->trig_mode_last = shoot_behaviour->trig_mode;
	shoot_behaviour->fric_mode_last = shoot_behaviour->fric_mode;
	shoot_behaviour->shoot_agency_state_last = shoot_behaviour->shoot_agency_state;
}

void shoot_motor_mode_set(shoot_control_t *shoot_mode)
{
	shoot_motor_behaviour_set(shoot_mode);
}

