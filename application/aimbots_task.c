#include "aimbots_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "usbd_cdc_if.h"




mini_data_t auto_data;

static void dispose_usb_data(mini_data_t *auto_data);

int8_t fire_flag;

uint8_t Send_to_minpc[18];
uint8_t a[18]={1,2,3,4,1,6,7,0,1};
fp32 buffet_speed;


fp32 relative_angle;
void aimbots_task(void const * argument)
{
	


	while(1)
	{
//		relative_angle = -gimbal_control.gimbal_pitch_motor.relative_angle;
//		buffet_speed = 20;//shoot_control.shoot_fric_L_motor.motor_speed;
//		Send_to_minpc[0]='I';
//		Send_to_minpc[1]='R';
//		memcpy(&Send_to_minpc[2],a,1);
//		memcpy(&Send_to_minpc[3],&a[1],1);
//		memcpy(&Send_to_minpc[4],&buffet_speed,4);
//		memcpy(&Send_to_minpc[8],&relative_angle,4);
//		memcpy(&Send_to_minpc[12],get_INS_angle(0),4);
//		Send_to_minpc[16]='O';
//		Send_to_minpc[17]='N';
//		CDC_Transmit_FS(Send_to_minpc,18);

		Send_to_minpc[0]=0xFF;
		Send_to_minpc[1]=0;
		memcpy(&Send_to_minpc[2],get_INS_angle(2),4);
		//memcpy(&Send_to_minpc[6],get_INS_pitch_to_minpc(),4);
		memcpy(&Send_to_minpc[6],&gimbal_control.gimbal_pitch_motor.relative_angle,4);
		memcpy(&Send_to_minpc[10],get_INS_angle(0),4);
		memcpy(&Send_to_minpc[14],a,1);
		Send_to_minpc[15]=0x0D;
		CDC_Transmit_FS(Send_to_minpc,16);

		dispose_usb_data(&auto_data);
		osDelay(5);
	}
}


//static void dispose_usb_data(mini_data_t *auto_data)
//{
//	/*-- ³Â£¬Ö¡Í·Ö¡Î² --*/
//	memcpy(&auto_data->heading,&usb_recive_buffer[0],2);
//	memcpy(&auto_data->tailing,&usb_recive_buffer[11],2);
//	
//	/*-- pitch yaw ¿ª»ð±êÖ¾ --*/
//	if (memcmp(auto_data->heading, "IR", 2) == 0 && memcmp(auto_data->tailing, "ON", 2) == 0)
//	{
//		memcpy(&auto_data->auto_pitch_set,&usb_recive_buffer[2],4);
//		memcpy(&auto_data->auto_yaw_set,&usb_recive_buffer[6],4);
//		memcpy(&auto_data->auto_fireFlag,&usb_recive_buffer[10],1);
//	}
//	
//}

static void dispose_usb_data(mini_data_t *auto_data)
{

	/*-- ¹¨£¬Ö¡Í·Ö¡Î² --*/
	memcpy(&auto_data->heading,&usb_recive_buffer[0],1);
	memcpy(&auto_data->tailing,&usb_recive_buffer[15],1);
	
	/*-- ¿ª»ð£¬pitch£¬yaw --*/
	if (auto_data->heading == 0xFF && auto_data->tailing == 0x0D)
	{

			memcpy(&auto_data->auto_fireFlag,&usb_recive_buffer[1],1);

		memcpy(&auto_data->auto_pitch_set,&usb_recive_buffer[2],4);
		memcpy(&auto_data->auto_yaw_set,&usb_recive_buffer[6],4);
	}
	
	
}


const mini_data_t* get_mini_data_point(void)
{
	return &auto_data;
}

const uint8_t *get_autofire_flag_point(void)
{
		return &auto_data.auto_fireFlag;
}



