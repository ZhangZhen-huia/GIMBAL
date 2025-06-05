#include "communicate_task.h"
#include "INS_task.h"
#include "can.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "user_lib.h"
#include "shoot_task.h"
#include "usbd_cdc_if.h"
#include "referee.h"
#include "key_task.h"
static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,uint8_t *buf,uint8_t num);
static void Gimbal_data_transfer(void);
void USB_CMD_PC(void);
static void AIMBOT_MODE(uint8_t* mode);

chassis_data_t chassis_data;
mini_data_t auto_data;
EnemyColor_e EnemyColor = BLUE;
Aimbot_Mode_e Aimbot_Mode;

void CAN_BusOff_Recovery(void); 
//1m 0.0
//2m	0.21
//3m 0.27
//4m	0.1	低了
//5m 0
//6m  0.2
//7m	0.17
fp32 yaw_err;
fp32 pitch_err;
void communicate_task(void const * argument)
{
	
	while(1)
	{
		USB_CMD_PC();//自瞄
		Gimbal_data_transfer();//双板通信
//	 yaw_err = auto_data.auto_yaw_set - gimbal_control.gimbal_yaw_motor.absolute_angle;
//		pitch_err = auto_data.auto_pitch_set - gimbal_control.gimbal_pitch_motor.relative_angle;
		osDelay(2);
	}
}
 


extern fp32 aim_pitch_err;
fp32 pitch_To_Pc;
void USB_CMD_PC(void)
{
	
		static uint8_t Send_to_minpc[16];
		//模式，roll,pitch,yaw,弹速
//		Send_to_minpc[0]=Heading;//0xFF;
//		//AIMBOT_MODE(&Send_to_minpc[1]);
//		Send_to_minpc[1] = EnemyColor;//赛场上修改颜色直接在定义处赋值，平常线下调试切换颜色把key_task.c中的函数打开就行
//		//memcpy(&Send_to_minpc[2],get_INS_angle(2),4);
//		Send_to_minpc[2]=0;
//		Send_to_minpc[3]=0;
//		Send_to_minpc[4]=0;
//		Send_to_minpc[5]=0;
//		memcpy(&Send_to_minpc[6],&gimbal_control.gimbal_pitch_motor.relative_angle,4);
//		memcpy(&Send_to_minpc[10],get_INS_angle(0),4);
//		//memcpy(&Send_to_minpc[14],a,1);
//		Send_to_minpc[14]=22;
//		Send_to_minpc[15]=Tail;//0x0D;
//		CDC_Transmit_FS(Send_to_minpc,16);
	        Send_to_minpc[0]=Heading;
        Send_to_minpc[1]=EnemyColor; //red
        
		Send_to_minpc[2]=0;
		Send_to_minpc[3]=0;
		Send_to_minpc[4]=0;
		Send_to_minpc[5]=0;
		//pitch_To_Pc = gimbal_control.gimbal_pitch_motor.relative_angle - aim_pitch_err;
		memcpy(&Send_to_minpc[6],&gimbal_control.gimbal_pitch_motor.relative_angle,4);
		memcpy(&Send_to_minpc[10],get_INS_angle(0),4);
        
        Send_to_minpc[15]=Tail;
        CDC_Transmit_FS(Send_to_minpc,16);
}

const mini_data_t* get_mini_data_point(void)
{
	return &auto_data;
}


static void Gimbal_data_transfer(void)
{
	static uint8_t gimbal_mode = 0x00;
	uint8_t buf[8];
	uint8_t vx_set;
	uint8_t vy_set;
	uint16_t rc_key_v;
	uint8_t rc_err;
	uint8_t rc_sl; 
	uint8_t rc_sr; 
	uint16_t ImghandleKey = ImgTransferKey>>16;
	
	if(ControlMode == Rc)
	{
		rc_key_v = rc_ctrl.key.v;		
		buf[3] = rc_sl;
		buf[4] = rc_sr;
	}
	else
	{
		rc_key_v = Referee_System.new_remote_data.key;
		memcpy(&buf[3],&ImghandleKey,2);
	}
	
	vx_set = (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]+660)/20.0f+(Referee_System.new_remote_data.ch_3-1024+660)/20.0f;//(0-66)
	vy_set = (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]+660)/20.0f+(Referee_System.new_remote_data.ch_2-1024+660)/20.0f;//(0-66)


	
	if(ControlMode == Rc)
		rc_err |= 0x01;
	else
		rc_err &= 0xFE;
	
	if(ControlMode == ImageTransfer)
		rc_err |= 0x02;
	else
		rc_err &= 0xFD;
	
	if(toe_is_error(DBUS_TOE))
		rc_err |= 0x04;
	else
		rc_err &= 0xFB;
	
	if(toe_is_error(REFEREE_TOE))
		rc_err |= 0x08;
	else
		rc_err &= 0xF7;
	
	rc_sl = rc_ctrl.rc.s[RC_sl_channel];
	rc_sr = rc_ctrl.rc.s[RC_sr_channel];

	/*-- 传输开火 --*/
	if(shoot_control.trig_mode == Start_fire)
		gimbal_mode |= 0x01; 
	else
		gimbal_mode &= 0xFE; 
	
	if(gimbal_control.gimbal_behaviour != GIMBAL_ZERO_FORCE)
		gimbal_mode |= 0x02;
	else
		gimbal_mode &= 0xFD;
	
	if(shoot_control.fric_mode == START)
		gimbal_mode |= 0x04;
	else
		gimbal_mode &= 0xFB;	
	
	if(EnemyColor == RED)
		gimbal_mode |= 0x08;
	else
		gimbal_mode &= 0xF7;
	
	if(EnemyColor == BLUE)
		gimbal_mode |= 0x10;
	else
		gimbal_mode &= 0xEF;
	
	//如果收到了自瞄的消息
	if(!toe_is_error(AIMBOT_TOE))
		gimbal_mode |= 0x20;
	else
		gimbal_mode &= 0xDF;
	
	
	buf[0] = vx_set;
	buf[1] = vy_set;
	buf[2] = rc_err;
	buf[5] = gimbal_mode;
	memcpy(&buf[6],&rc_key_v,2);
	
	can_cmd_to_chassis(&hcan1,GIMBAL_ID,buf,8);
	
}

static void AIMBOT_MODE(uint8_t* mode)
{
		switch(Referee_System.new_remote_data.mode_sw)
		{
			case 0:Aimbot_Mode = NORMAL;break;
			case 1:Aimbot_Mode = BIG;break;
			case 2:Aimbot_Mode = SMALL;break;
		}
		

	if(Aimbot_Mode == NORMAL)
	{
		*mode = EnemyColor;
	}
	else if(Aimbot_Mode == BIG)
	{

			*mode = BIG+EnemyColor;

	}
	else if(Aimbot_Mode == SMALL)
	{
			*mode = SMALL+EnemyColor;
	}
}
static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,uint8_t *buf,uint8_t num)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef gimbal_tx_message;
	uint8_t    chassis_can_send_data[num];
	
	gimbal_tx_message.StdId = can_id;
	gimbal_tx_message.DLC = num;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	for(uint8_t i=0;i<num;i++)
	chassis_can_send_data[i] = buf[i];


	HAL_CAN_AddTxMessage(hcan,&gimbal_tx_message,chassis_can_send_data, &send_mail_box);

}




//void CAN_BusOff_Recovery(void) 
//{
//    if (__HAL_CAN_GET_FLAG(&hcan2, CAN_FLAG_BOF) != RESET)
//		{
//        // CAN2进入Bus-Off状态
//			 __HAL_CAN_CLEAR_FLAG(&hcan2, CAN_FLAG_BOF); // 清除Bus-Off标志
//			HAL_CAN_Stop(&hcan2);
//			MX_CAN2_Init();
//			canfilter_init_start();
//    }
//		
//		 if (__HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) != RESET)
//		{
//        // CAN1进入Bus-Off状态
//			 __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_BOF); // 清除Bus-Off标志
//			HAL_CAN_Stop(&hcan1);
//			MX_CAN1_Init();
//			canfilter_init_start();
//    }
//}

