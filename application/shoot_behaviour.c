#include "shoot_behaviour.h"
#include "shoot_task.h"
#include "tim.h"
#include "aimbots_task.h"
#include "gimbal_task.h"
#include "key_task.h"


static void shoot_motor_behaviour_set(shoot_control_t *shoot_behaviour)
{
	static uint8_t firc_step = 0;
	static uint16_t shoot_flag = 0;
	static uint8_t shoot_force = 0;
	if(shoot_behaviour == NULL)
	{
		return;
	}
	
	//遥控器掉线就关闭发射结构
	if (toe_is_error(DBUS_TOE))
	{
		//关闭发射结构总开关
		shoot_behaviour->shoot_agency_state = SHOOT_OFF;
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;

	}
	
	//强制开火信号
	if(shoot_behaviour->shoot_rc_ctrl->mouse.press_l == 1)//shoot_behaviour->shoot_rc_ctrl->rc.ch[0] == -660 && shoot_behaviour->shoot_rc_ctrl->rc.ch[1] == 660)
	{
		shoot_force = 1;
	}
	else
	{
		shoot_force = 0;
	}
	
	
	if( shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START && shoot_force)
	{
		shoot_behaviour->trig_mode = Start_fire;
	}
	else
	{
		shoot_behaviour->trig_mode = Cease_fire;
	}
	
	
	if(shoot_behaviour->shoot_agency_state == SHOOT_OFF)
	{
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;
		shoot_flag = 0;
		firc_step = 0;
		
	}
		//发射机构开关
	if(((shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000)) || Key_Value.Z_B)
	{
		shoot_flag++;
		if(shoot_flag >= 400)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_flag=0;
		}
	}
	else if((shoot_behaviour->shoot_rc_ctrl->rc.ch[4] ==660 && (switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]) || switch_is_down(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]))) || Key_Value.Z_F)//右中
	{
		shoot_behaviour->shoot_agency_state = SHOOT_ON;//发射结构开
		shoot_flag = 0;
	}
	else
	{
		shoot_flag=0;
	}
	
	//不是自瞄模式
	if(gimbal_control.gimbal_behaviour != GIMBAL_AUTO_ANGLE)
	{		
		if(Key_Value.Z_F)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_ON;
			shoot_behaviour->fric_mode = START;
		}
		else if(Key_Value.Z_B)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_behaviour->fric_mode = STOP;
		}

	
	
		//摩擦轮开启判断
		if(shoot_behaviour->shoot_agency_state == SHOOT_ON)	
		{	
			if(switch_is_up(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]))	
			{	
				shoot_behaviour->fric_mode = STOP;	
			}	
			else if(switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]))	
			{	
						switch(firc_step)	
						{	
							case 0:if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000 || Key_Value.Z_F)	
											firc_step = 1;	
											break;	
							case 1:if(switch_is_mid(shoot_behaviour->shoot_rc_ctrl->rc.s[SHOOT_MODE_CHANNEL]))//右中	
											{	
												shoot_behaviour->fric_mode = START;	
												firc_step = 0;	
											}	
											else 	
											{	
												firc_step = 0;	
												shoot_behaviour->fric_mode = STOP;	
											}	
											break;								
						}			
			}	
		}
		else
		{
			shoot_behaviour->fric_mode = STOP;	
		}
				//拨弹盘
			if(shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START)
			{
				if(shoot_behaviour->shoot_agency_state == SHOOT_OFF || shoot_behaviour->fric_mode == STOP)
						shoot_behaviour->trig_mode = Cease_fire;
				
				else if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] == 660 || shoot_behaviour->shoot_rc_ctrl->mouse.press_l)
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
		if(toe_is_error(DBUS_TOE))
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_behaviour->fric_mode = STOP;
			shoot_behaviour->trig_mode = Cease_fire;
		}

		
			//摩擦轮开启判断
			if(shoot_behaviour->shoot_agency_state == SHOOT_ON)
			{

				if((shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000) || Key_Value.Z_F)
						shoot_behaviour->fric_mode = START;
			
				else if((shoot_behaviour->shoot_rc_ctrl->rc.ch[0] == 660 && shoot_behaviour->shoot_rc_ctrl->rc.ch[1] == -660 && shoot_behaviour->shoot_rc_ctrl->rc.ch[4] ==660) || Key_Value.Z_B)
				{
					shoot_behaviour->fric_mode = STOP;
				}
	
			}
			else
			{
				shoot_behaviour->fric_mode = STOP;
			}
					//拨弹盘开启判断
				if(shoot_behaviour->fric_mode == START &&  shoot_behaviour->auto_fireFlag[0] == fire)
				{
					
					shoot_behaviour->trig_mode = Start_fire;
				}
				else
				{
					shoot_behaviour->trig_mode = Cease_fire;
				}			
				if(shoot_force && shoot_behaviour->fric_mode == START)
				{
					shoot_behaviour->trig_mode = Start_fire;
				}
				else
				{
					shoot_behaviour->trig_mode = Cease_fire;
				}

		
	}


	shoot_behaviour->trig_mode_last = shoot_behaviour->trig_mode;
	shoot_behaviour->fric_mode_last = shoot_behaviour->fric_mode;
	shoot_behaviour->shoot_agency_state_last = shoot_behaviour->shoot_agency_state;
}

void shoot_motor_mode_set(shoot_control_t *shoot_mode)
{
	shoot_motor_behaviour_set(shoot_mode);
}

