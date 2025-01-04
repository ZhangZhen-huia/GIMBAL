#include "communicate_task.h"
#include "INS_task.h"
#include "can.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "user_lib.h"
#include "shoot_task.h"
#include "aimbots_task.h"
#include "usbd_cdc_if.h"


static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,uint8_t *buf,uint8_t num);
static void Gimbal_data_transfer(void);
chassis_data_t chassis_data;





void communicate_task(void const * argument)
{
	
	while(1)
	{

		Gimbal_data_transfer();

		osDelay(2);
	}
}







static void Gimbal_data_transfer(void)
{
	static uint8_t gimbal_mode = 0x00;
	uint8_t buf[8];
	uint8_t vx_set;
	uint8_t vy_set;
//	uint8_t wz_set;
	uint16_t rc_key_v;
	uint8_t rc_err;
	uint8_t rc_sl; 
	uint8_t rc_sr; 
	
	
	rc_key_v = rc_ctrl.key.v;		
	vx_set = (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]+660)/20.0f;//(0-66)
	vy_set = (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]+660)/20.0f;//(0-66)
//	wz_set = (rc_ctrl.rc.ch[CHASSIS_W_CHANNEL]+660)/20.0f;//0-66
	
	rc_err = (uint8_t)toe_is_error(DBUS_TOE);
	rc_sl = rc_ctrl.rc.s[RC_sl_channel];
	rc_sr = rc_ctrl.rc.s[RC_sr_channel];
	
	if(shoot_control.trig_mode == Start_fire)
		gimbal_mode |= 0x01; 
	else
		gimbal_mode &= 0xFE; 
	
	if(gimbal_control.gimbal_behaviour == GIMBAL_INIT)
		gimbal_mode |= 0x02;
	else
		gimbal_mode &= 0xFD;
	

	buf[0] = vx_set;
	buf[1] = vy_set;
//	buf[2] = wz_set;
	buf[2] = rc_err;
	buf[3] = rc_sl;
	buf[4] = rc_sr;
	buf[5] = gimbal_mode;
	memcpy(&buf[6],&rc_key_v,2);
//	buf[7] = ;


	
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





