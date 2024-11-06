#include "aimbots_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
extern uint8_t usb_recive_buffer[100];


mini_data_t auto_data;

static void dispose_usb_data(mini_data_t *auto_data);




void aimbots_task(void const * argument)
{
	
	while(1)
	{
		dispose_usb_data(&auto_data);
	}
}


fp32 pitch_receive,yaw_receive;
uint8_t angle_receive[8];
//void get_mini_data(gimbal_control_t *gimbal_control_mini)
//{
////	pitch_set.bytes[0] = usb_recive_buffer[0];
////	pitch_set.bytes[1] = usb_recive_buffer[1];
////	pitch_set.bytes[2] = usb_recive_buffer[2];
////	pitch_set.bytes[3] = usb_recive_buffer[3];
////	
////	yaw_set.bytes[0] = usb_recive_buffer[4];
////	yaw_set.bytes[1] = usb_recive_buffer[5];
////	yaw_set.bytes[2] = usb_recive_buffer[6];
////	yaw_set.bytes[3] = usb_recive_buffer[7];
//	angle_receive[0] = usb_recive_buffer[0];//符号位
//	angle_receive[1] = usb_recive_buffer[1];//pitch绝对值
//	angle_receive[2] = usb_recive_buffer[2];
//	angle_receive[3] = usb_recive_buffer[3];
//	angle_receive[4] = usb_recive_buffer[4];//符号位
//	angle_receive[5] = usb_recive_buffer[5];//yaw绝对值
//	angle_receive[6] = usb_recive_buffer[6];
//	angle_receive[7] = usb_recive_buffer[7];
//	
//	if(usb_recive_buffer[0] == 1)
//	{
//		pitch_receive = (float)usb_recive_buffer[1];
//	}
//	else if(usb_recive_buffer[0] == 0)
//	{
//		pitch_receive = -(float)usb_recive_buffer[1];
//	}
//	if(usb_recive_buffer[4] == 1)
//	{
//		yaw_receive = (float)usb_recive_buffer[5];
//	}
//	else if(usb_recive_buffer[4] == 0)
//	{
//		yaw_receive = -(float)usb_recive_buffer[5];
//	}
////	
////	pitch_receive = pitch_set.value;
////	yaw_receive = yaw_set.value;

//}


static void dispose_usb_data(mini_data_t *auto_data)
{
	
	auto_data->pitch_PNflag = usb_recive_buffer[0]?1:-1;
	auto_data->rec_mini_pitch.bytes[0] = usb_recive_buffer[1];
	auto_data->rec_mini_pitch.bytes[1] = usb_recive_buffer[2];
	auto_data->rec_mini_pitch.bytes[2] = usb_recive_buffer[3];
	auto_data->rec_mini_pitch.bytes[3] = 0;
	auto_data->auto_pitch_set=auto_data->rec_mini_pitch.value * auto_data->pitch_PNflag;
	
	auto_data->yaw_PNflag = usb_recive_buffer[4]?1:-1;
	auto_data->rec_mini_yaw.bytes[0] = usb_recive_buffer[5];
	auto_data->rec_mini_yaw.bytes[1] = usb_recive_buffer[6];
	auto_data->rec_mini_yaw.bytes[2] = usb_recive_buffer[7];
	auto_data->rec_mini_yaw.bytes[3] = 0;
	auto_data->auto_yaw_set=auto_data->rec_mini_yaw.value * auto_data->yaw_PNflag;
}

const mini_data_t* get_mini_data_point(void)
{
	return &auto_data;
}
