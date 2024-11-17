#include "user_task.h"
#include "INS_task.h"
#include "can.h"
#include "Can_receive.h"
#include "remote_control.h"

static void CAN_cmd_to_chassis_rc(int16_t buf1,int16_t buf2 ,int16_t buf3 ,int16_t buf4);
static void Rc_data_transfer(void);

static void Gimbal_data_transfer(void);




void user_task(void const * argument)
{
	
	while(1)
	{
		Rc_data_transfer();
		osDelay(1);
	}
}



static void Gimbal_data_transfer(void)
{
	int16_t yaw_data = bmi088_real_data.INS_angle[INS_YAW_ADDRESS_OFFSET];

	//CAN_cmd_to_chassis(yaw_data,0,0,0);
	
	
}



static void Rc_data_transfer(void)
{
	int16_t vx_set = rc_ctrl.rc.ch[CHASSIS_X_CHANNEL];
	int16_t vy_set = rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL];
	int16_t wz_set = rc_ctrl.rc.ch[CHASSIS_W_CHANNEL];
	uint8_t rc_s[2] = {rc_ctrl.rc.s[RC_sl_channel], rc_ctrl.rc.s[RC_sr_channel]};
	int16_t S = *((int16_t* )(rc_s)); // Ò£¿ØÆ÷¿ª¹Ø

	CAN_cmd_to_chassis_rc(vx_set,vy_set,wz_set,S);
	
}





static void CAN_cmd_to_chassis_rc(int16_t buf1,int16_t buf2 ,int16_t buf3 ,int16_t buf4)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef chassis_tx_message;
	static uint16_t    chassis_can_send_data[4];
	
	chassis_tx_message.StdId = RC_ID;
	chassis_tx_message.DLC = 8;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	
	chassis_can_send_data[0] = buf1;
	chassis_can_send_data[1] = buf2;
	chassis_can_send_data[2] = buf3;
	chassis_can_send_data[3] = buf4;

	HAL_CAN_AddTxMessage(&hcan1,&chassis_tx_message,(uint8_t*)chassis_can_send_data, &send_mail_box);


}
