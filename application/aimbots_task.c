#include "aimbots_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
extern uint8_t usb_recive_buffer[100];

union
{
    uint8_t bytes[4];
    float value;
} pitch_set,yaw_set;


fp32 pitch_receive,yaw_receive;
fp32 angle_receive[8];
void get_mini_data(gimbal_control_t *gimbal_control_mini)
{
//	pitch_set.bytes[0] = usb_recive_buffer[0];
//	pitch_set.bytes[1] = usb_recive_buffer[1];
//	pitch_set.bytes[2] = usb_recive_buffer[2];
//	pitch_set.bytes[3] = usb_recive_buffer[3];
//	
//	yaw_set.bytes[0] = usb_recive_buffer[4];
//	yaw_set.bytes[1] = usb_recive_buffer[5];
//	yaw_set.bytes[2] = usb_recive_buffer[6];
//	yaw_set.bytes[3] = usb_recive_buffer[7];
	angle_receive[0] = usb_recive_buffer[0];//符号位
	angle_receive[1] = usb_recive_buffer[1];//pitch绝对值
	angle_receive[2] = usb_recive_buffer[2];//符号位
	angle_receive[3] = usb_recive_buffer[3];//yaw绝对值
	angle_receive[4] = usb_recive_buffer[4];
	angle_receive[5] = usb_recive_buffer[5];
	angle_receive[6] = usb_recive_buffer[6];
	angle_receive[7] = usb_recive_buffer[7];
	
	if(angle_receive[0] == 1)
	{
		pitch_receive = usb_recive_buffer[1];
	}
	else if(angle_receive[0] == 0)
	{
		pitch_receive = -usb_recive_buffer[1];
	}
	if(angle_receive[2] == 1)
	{
		yaw_receive = usb_recive_buffer[3];
	}
	else if(angle_receive[2] == 0)
	{
		yaw_receive = -usb_recive_buffer[3];
	}
//	
//	pitch_receive = pitch_set.value;
//	yaw_receive = yaw_set.value;

}



