#include "user_task.h"
#include "INS_task.h"
#include "can.h"
#include "Can_receive.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "detect_task.h"

static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,uint8_t *buf);
static void Rc_data_transfer(void);





void user_task(void const * argument)
{
	
	while(1)
	{
		Rc_data_transfer();
		osDelay(10);
	}
}








uint8_t buf[8];
static void Rc_data_transfer(void)
{

	
	
	uint8_t vx_set = (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]+660)/10.0f;
	uint8_t vy_set = (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]+660)/10.0f;
	uint8_t wz_set = (rc_ctrl.rc.ch[CHASSIS_W_CHANNEL]+660)/10.0f;
	uint8_t rc_err = (uint8_t)toe_is_error(DBUS_TOE);
	uint8_t rc_sl = rc_ctrl.rc.s[RC_sl_channel];
	uint8_t rc_sr = rc_ctrl.rc.s[RC_sr_channel];
	fp32 yaw_data = (200*bmi088_real_data.INS_angle[INS_YAW_ADDRESS_OFFSET]/57.2957795f/4.0f);	
		
	uint8_t gimbal_mode = 0;
	if(gimbal_control.gimbal_behaviour == GIMBAL_INIT)
		gimbal_mode |= 1<<0;
	if(gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
		gimbal_mode |= 1<<1;

	buf[0] = vx_set;
	buf[1] = vy_set;
	buf[2] = wz_set;
	buf[3] = rc_err;
	buf[4] = rc_sl;
	buf[5] = rc_sr;
	buf[6] = (uint8_t)yaw_data;
	buf[7] = gimbal_mode;

	
	can_cmd_to_chassis(&hcan1,GIMBAL_ID,buf);
	
}


static void can_cmd_to_chassis(CAN_HandleTypeDef*hcan,int16_t can_id,uint8_t *buf)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef chassis_tx_message;
	uint8_t    chassis_can_send_data[8];
	
	chassis_tx_message.StdId = can_id;
	chassis_tx_message.DLC = 8;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	for(uint8_t i=0;i<8;i++)
	chassis_can_send_data[i] = buf[i];
//	for(uint8_t i=0;*(++buf) !=NULL;i++)
//	{
//		chassis_can_send_data[i] = *buf;
//	}


	HAL_CAN_AddTxMessage(hcan,&chassis_tx_message,chassis_can_send_data, &send_mail_box);

}

