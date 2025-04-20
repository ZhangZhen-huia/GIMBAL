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
	
	//ң�������߾͹رշ���ṹ
	if (POWER_OFF)
	{
		//�رշ���ṹ�ܿ���
		shoot_behaviour->shoot_agency_state = SHOOT_OFF;
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;

	}
	
	//ǿ�ƿ����ź�
	if(shoot_behaviour->fric_mode == START && Mouse_Data.mouse_l)//shoot_behaviour->shoot_rc_ctrl->rc.ch[0] == -660 && shoot_behaviour->shoot_rc_ctrl->rc.ch[1] == 660)
	{
		shoot_force = 1;
	}
	else
	{
		shoot_force = 0;
	}
	
	//ǿ�ƿ���ǿ�Ƹ������̿����źţ���Ҫ��������ģʽ��
	if( shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START && shoot_force)
	{
		shoot_behaviour->trig_mode = Start_fire;
	}
	else
	{
		shoot_behaviour->trig_mode = Cease_fire;
	}

	//�����������ܿ��ع��ˣ���ô���еĶ��ر�
	if(shoot_behaviour->shoot_agency_state == SHOOT_OFF)
	{
		shoot_behaviour->fric_mode = STOP;
		shoot_behaviour->trig_mode = Cease_fire;
		shoot_flag = 0;
		
	}

	//������������жϣ�����������һ�����������״̬��Ϊ�˷�ֹң�����󴥵���Ħ���ֿ���
	if(((shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->shoot_rc_ctrl->rc.ch[4] <=-600)))
	{
		if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] <= -600)
		{
			shoot_flag++;
			if(shoot_flag >= 50)
			{
				shoot_behaviour->shoot_agency_state = SHOOT_OFF;//���������
				shoot_flag=0;
			}
		}
	}
	else if((shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 600))
	{
		shoot_behaviour->shoot_agency_state = SHOOT_ON;//����ṹ��
		shoot_flag = 0;
	}
	else
	{
		shoot_flag=0;
	}
	
	//Ħ���ֿ����ж�
		if(shoot_behaviour->shoot_agency_state == SHOOT_ON)	
		{	
			if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] <= -600)
					shoot_behaviour->fric_mode = START;	
		}	
		
		else
		{
			shoot_behaviour->fric_mode = STOP;	
		}

		if(Key_ScanValue.Key_Value.FN_2)
		{
			static uint8_t flag;
			flag++;
			flag%=2;
			if(flag)
			{
				shoot_behaviour->shoot_agency_state = SHOOT_ON;
				shoot_behaviour->fric_mode = START;
			}
			else
			{
				shoot_behaviour->shoot_agency_state = SHOOT_OFF;
				shoot_behaviour->fric_mode = STOP;
			}
		}

		if(Mouse_Data.mouse_z != 0)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_ON;
			shoot_behaviour->fric_mode = START;
		}
		if(Key_ScanValue.Key_Value.G)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_behaviour->fric_mode = STOP;
		}
	
	/*-- ģʽѡ�� --*/
	//��������ģʽ
	if(gimbal_control.gimbal_behaviour != GIMBAL_AUTO_ANGLE)
	{				
				//������
			if(shoot_behaviour->shoot_agency_state == SHOOT_ON && shoot_behaviour->fric_mode == START)
			{
				if(shoot_behaviour->shoot_agency_state == SHOOT_OFF || shoot_behaviour->fric_mode == STOP)
						shoot_behaviour->trig_mode = Cease_fire;
				
				else if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >=600 || Mouse_Data.mouse_l || Referee_System.new_remote_data.wheel==364)
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
	

	//����ģʽ
	else if(gimbal_control.gimbal_behaviour == GIMBAL_AUTO_ANGLE)
	{
		if(POWER_OFF)
		{
			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
			shoot_behaviour->fric_mode = STOP;
			shoot_behaviour->trig_mode = Cease_fire;
		}

					//�����̿����ж�
				if((shoot_behaviour->fric_mode == START &&  auto_data.auto_fireFlag) || shoot_force)
				{
					
					shoot_behaviour->trig_mode = Start_fire;
				}
				else
				{
					shoot_behaviour->trig_mode = Cease_fire;
				}			

		
	}

	//״̬��ֵ
	shoot_behaviour->trig_mode_last = shoot_behaviour->trig_mode;
	shoot_behaviour->fric_mode_last = shoot_behaviour->fric_mode;
	shoot_behaviour->shoot_agency_state_last = shoot_behaviour->shoot_agency_state;
}

void shoot_motor_mode_set(shoot_control_t *shoot_mode)
{
	shoot_motor_behaviour_set(shoot_mode);
}

//		if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000)
//		{
//			shoot_flag++;
//			if(shoot_flag >= 50)
//			{
//				shoot_behaviour->shoot_agency_state = SHOOT_OFF;//���������
//				shoot_flag=0;
//			}
//		}
//		else if((Key_ScanValue.Key_Value.B && !Key_ScanValue.Key_Value_Last.B))
//		{
//			shoot_behaviour->shoot_agency_state = SHOOT_OFF;
//		}

//	}
//	else if((shoot_behaviour->shoot_rc_ctrl->rc.ch[4] ==660 ) || (Key_ScanValue.Key_Value.G && !Key_ScanValue.Key_Value_Last.G))//����
//	{
//		shoot_behaviour->shoot_agency_state = SHOOT_ON;//����ṹ��
//		shoot_flag = 0;
//	}
//	else
//	{
//		shoot_flag=0;
//	}
//	
//	//Ħ���ֿ����ж�
//		if(shoot_behaviour->shoot_agency_state == SHOOT_ON)	
//		{	
//			if(shoot_behaviour->shoot_rc_ctrl->rc.ch[4] >= 5000 || (Key_ScanValue.Key_Value.G && !Key_ScanValue.Key_Value_Last.G))	
//					shoot_behaviour->fric_mode = START;	
//		}	
//		
//		else
//		{
//			shoot_behaviour->fric_mode = STOP;	
//		}
//	}
//	else
//	{
//		/*����ֱ�ӿ��ƿ���Ħ����*/
//		if(Key_ScanValue.Key_Value.G)
//		{
//			shoot_behaviour->shoot_agency_state++;
//			shoot_behaviour->shoot_agency_state%=2;
//			shoot_behaviour->fric_mode++;
//			shoot_behaviour->fric_mode%=2;
//		}
