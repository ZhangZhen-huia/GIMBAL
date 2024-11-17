#include "aimbots_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "usbd_cdc_if.h"

extern uint8_t usb_recive_buffer[100];


mini_data_t auto_data;

static void dispose_usb_data(mini_data_t *auto_data);

int8_t fire_flag;

uint8_t Send_to_minpc[18];
uint8_t a[18]={1,2,3,4,1,6,7,0,1};
fp32 buffet_speed;



void aimbots_task(void const * argument)
{
	osDelay(1000);


	while(1)
	{
		buffet_speed = shoot_control.shoot_fric_L_motor.motor_speed;
		Send_to_minpc[0]='I';
		Send_to_minpc[1]='R';
		memcpy(&Send_to_minpc[2],a,1);
		memcpy(&Send_to_minpc[3],&a[1],1);
		memcpy(&Send_to_minpc[4],&buffet_speed,4);
		memcpy(&Send_to_minpc[8],get_INS_angle(1),4);
		memcpy(&Send_to_minpc[12],get_INS_angle(0),4);
		Send_to_minpc[16]='O';
		Send_to_minpc[17]='N';
		CDC_Transmit_FS(Send_to_minpc,18);


		dispose_usb_data(&auto_data);
		osDelay(10);
	}
}



static void dispose_usb_data(mini_data_t *auto_data)
{
	memcpy(&auto_data->heading,&usb_recive_buffer[0],2);
	memcpy(&auto_data->auto_pitch_set,&usb_recive_buffer[2],4);
	memcpy(&auto_data->auto_yaw_set,&usb_recive_buffer[6],4);
	memcpy(&auto_data->heading,&usb_recive_buffer[10],2);
	//fire_flag = usb_recive_buffer[8];	

	
}


const mini_data_t* get_mini_data_point(void)
{
	return &auto_data;
}
