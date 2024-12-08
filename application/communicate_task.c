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
static void Get_Radar_Data(Radar_data_t *Radar_data);


chassis_data_t chassis_data;
Radar_data_t Radar_data;


void communicate_task(void const * argument)
{
	
	while(1)
	{
		Gimbal_data_transfer();
		#ifdef RADAR
		Get_Radar_Data(&Radar_data);
		#endif
		osDelay(2);
	}
}







static void Gimbal_data_transfer(void)
{
	static uint8_t gimbal_mode = 0x00;
	uint8_t buf[8];
	uint8_t vx_set;
	uint8_t vy_set;
	uint8_t wz_set;
	
	
	if(gimbal_control.gimbal_behaviour == GIMBAL_FOLLOW_RADAR)
	{
		#ifdef RADAR
		vx_set = Radar_data.vx > 0 ? Radar_data.vx*100.0f:Radar_data.vx*(-100.0f);

		vy_set = Radar_data.vy > 0 ? Radar_data.vy*100.0f:Radar_data.vy*(-100.0f);
		wz_set = Radar_data.wz > 0 ? Radar_data.wz*100.0f:Radar_data.wz*(-100.0f);
		
		#else
		vx_set = 0;
		vy_set = 0;
		wz_set = 0;
		#endif
	}
	else
	{
		vx_set = (rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]+660)/20.0f;//0-66
		vy_set = (rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]+660)/20.0f;//0-66
		wz_set = (rc_ctrl.rc.ch[CHASSIS_W_CHANNEL]+660)/20.0f;//0-66
	}
	uint8_t rc_err = (uint8_t)toe_is_error(DBUS_TOE);
	uint8_t rc_sl = rc_ctrl.rc.s[RC_sl_channel];
	uint8_t rc_sr = rc_ctrl.rc.s[RC_sr_channel];
	
	if(shoot_control.trig_mode == Start_fire)
		gimbal_mode |= 0x01; 
	else
		gimbal_mode &= 0xFE; 
	
	if(gimbal_control.gimbal_behaviour == GIMBAL_INIT)
		gimbal_mode |= 0x02;
	else
		gimbal_mode &= 0xFD;	
	
	if(gimbal_control.gimbal_behaviour == GIMBAL_FOLLOW_RADAR)
		gimbal_mode |= 0x04;
	else
		gimbal_mode &= 0xFB;	
	

	buf[0] = vx_set;
	buf[1] = vy_set;
	buf[2] = wz_set;
	buf[3] = rc_err;
	buf[4] = rc_sl;
	buf[5] = rc_sr;
	buf[6] = gimbal_mode;
//	buf[7] = gimbal_mode;

	
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


static void Get_Radar_Data(Radar_data_t *Radar_data)
{
	memcpy(&Radar_data->heading,&usb_recive_buffer[0],3);
	memcpy(&Radar_data->tailing,&usb_recive_buffer[15],3);
	if (memcmp(Radar_data->heading, "IST", 3) == 0 && memcmp(Radar_data->tailing, "AAA", 3) == 0)
	{
		memcpy(&Radar_data->vx,&usb_recive_buffer[3],4);
		memcpy(&Radar_data->vy,&usb_recive_buffer[7],4);
		memcpy(&Radar_data->wz,&usb_recive_buffer[11],4);
	}
	

}



//void get_chassis_data(chassis_data_t *chassis_data,uint8_t *buf)
//{
//	chassis_data->pitch = (buf[0]/40.0f-3.1415926f)*57.2957795f;

////chassis_data->pitch = buf[0]-3.1415926f;	
//}


