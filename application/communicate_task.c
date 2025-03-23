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

chassis_data_t chassis_data;
mini_data_t auto_data;
EnemyColor_e EnemyColor;




void communicate_task(void const * argument)
{
	
	while(1)
	{
		USB_CMD_PC();//自瞄
		Gimbal_data_transfer();//双板通信

		osDelay(2);
	}
}




void USB_CMD_PC(void)
{
	
		static uint8_t Send_to_minpc[16];
	
		Send_to_minpc[0]=0xFF;
		Send_to_minpc[1]=EnemyColor;
		//memcpy(&Send_to_minpc[2],get_INS_angle(2),4);
		Send_to_minpc[2]=0;
		Send_to_minpc[3]=0;
		Send_to_minpc[4]=0;
		Send_to_minpc[5]=0;
		memcpy(&Send_to_minpc[6],&gimbal_control.gimbal_pitch_motor.relative_angle,4);
		memcpy(&Send_to_minpc[10],get_INS_angle(0),4);
		//memcpy(&Send_to_minpc[14],a,1);
		Send_to_minpc[14]=22;
		Send_to_minpc[15]=0x0D;
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
	
	if(ControlMode == Rc)
	rc_key_v = rc_ctrl.key.v;		
	else
	rc_key_v = Referee_System.Image_trans_remote.keyboard_value;
	
	vx_set = (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]+660)/20.0f;//(0-66)
	vy_set = (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]+660)/20.0f;//(0-66)
	

	
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
	
	if(gimbal_control.gimbal_behaviour == GIMBAL_INIT)
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
	buf[0] = vx_set;
	buf[1] = vy_set;
	buf[2] = rc_err;
	buf[3] = rc_sl;
	buf[4] = rc_sr;
	buf[5] = gimbal_mode;
	memcpy(&buf[6],&rc_key_v,2);
	
	can_cmd_to_chassis(&hcan1,GIMBAL_ID,buf,8);
	
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





