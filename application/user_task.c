#include "user_task.h"
#include "INS_task.h"
#include "can.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "gimbal_task.h"


static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,int16_t buf1,int16_t buf2 ,int16_t buf3 ,int16_t buf4);
static void Rc_data_transfer(void);

static void Gimbal_data_transfer(void);




void user_task(void const * argument)
{
	
	while(1)
	{
		Rc_data_transfer();
		Gimbal_data_transfer();
		osDelay(10);
	}
}



static void Gimbal_data_transfer(void)
{
	fp32 yaw_data = bmi088_real_data.INS_angle[INS_YAW_ADDRESS_OFFSET];
	int16_t gimbal_mode = 0;
	if(gimbal_control.gimbal_behaviour == GIMBAL_INIT)
		gimbal_mode |= 1<<0;
	if(gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
		gimbal_mode |= 1<<1;
	
	can_cmd_to_chassis(&hcan1,GIMBAL_ID,(int16_t)yaw_data,gimbal_mode,0,0);
	
	
}



static void Rc_data_transfer(void)
{
	int16_t vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL];
	int16_t vy_set = rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL];
	int16_t wz_set = rc_ctrl.rc.ch[CHASSIS_W_CHANNEL];
	uint8_t rc_s[2] = {rc_ctrl.rc.s[RC_sl_channel], rc_ctrl.rc.s[RC_sr_channel]};
	int16_t S = *((int16_t* )(rc_s)); // Ò£¿ØÆ÷¿ª¹Ø

	can_cmd_to_chassis(&hcan1,RC_ID,vx_set,vy_set,wz_set,S);
	
}





static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,int16_t buf1,int16_t buf2 ,int16_t buf3 ,int16_t buf4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef chassis_tx_message;
	uint16_t    chassis_can_send_data[4];
	
	chassis_tx_message.StdId = can_id;
	chassis_tx_message.DLC = 8;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	
	chassis_can_send_data[0] = buf1;
	chassis_can_send_data[1] = buf2;
	chassis_can_send_data[2] = buf3;
	chassis_can_send_data[3] = buf4;

	HAL_CAN_AddTxMessage(hcan,&chassis_tx_message,(uint8_t*)chassis_can_send_data, &send_mail_box);

}

