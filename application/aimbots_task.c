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
		osDelay(1);
	}
}




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
