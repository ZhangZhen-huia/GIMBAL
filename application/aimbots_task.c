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
uint16_t ba[2]={1,1};
fp32 jjj[2] = {1.1,2.2};

union {
	float prior_data;
	uint8_t data[4];
}pitch_union,yaw_union;

union {
	float bmi_data;
	uint8_t data[4];
}pitch,yaw;



//float yaw;
void aimbots_task(void const * argument)
{
	osDelay(1000);


	while(1)
	{
		
			Send_to_minpc[0]='I';
		Send_to_minpc[1]='R';
		
		memcpy(&Send_to_minpc[2],ba,2);
		memcpy(&Send_to_minpc[4],&ba[1],2);
		
		pitch.bmi_data = bmi088_real_data.INS_angle[1];
		yaw.bmi_data = bmi088_real_data.INS_angle[0];
		
//		Send_to_minpc[6]=pitch.data[0];
//		Send_to_minpc[7]=pitch.data[1];
//		Send_to_minpc[8]=pitch.data[2];
//		Send_to_minpc[9]=pitch.data[3];
//		
//		Send_to_minpc[10]=yaw.data[0];
//		Send_to_minpc[11]=yaw.data[1];
//		Send_to_minpc[12]=yaw.data[2];
//		Send_to_minpc[13]=yaw.data[3];

		
		memcpy(&Send_to_minpc[6],get_INS_angle(1),4);
		memcpy(&Send_to_minpc[10],get_INS_angle(0),4);
		memcpy(&Send_to_minpc[14],&a[4],1);
		memcpy(&Send_to_minpc[15],&a[5],1);
		Send_to_minpc[16]='O';
		Send_to_minpc[17]='N';
				CDC_Transmit_FS(Send_to_minpc,18);
	memcpy(&pitch,&Send_to_minpc[6],4);
		memcpy(&yaw,&Send_to_minpc[10],4);

		dispose_usb_data(&auto_data);
		osDelay(10);
	}
}



/*小电脑发送的时候直接发浮点数，小端开头，使用移位操作来把浮点数分解为4个8位的数*/
static void dispose_usb_data(mini_data_t *auto_data)
{
	
	
//	auto_data->rec_mini_pitch.bytes[0] = usb_recive_buffer[0];
//	auto_data->rec_mini_pitch.bytes[1] = usb_recive_buffer[1];
//	auto_data->rec_mini_pitch.bytes[2] = usb_recive_buffer[2];
//	auto_data->rec_mini_pitch.bytes[3] = usb_recive_buffer[3];
//	auto_data->auto_pitch_set=auto_data->rec_mini_pitch.value;
	
	memcpy(&auto_data->auto_pitch_set,&usb_recive_buffer[0],4);
	memcpy(&auto_data->auto_yaw_set,&usb_recive_buffer[4],4);
	fire_flag = usb_recive_buffer[8];	
//	auto_data->rec_mini_yaw.bytes[0] = usb_recive_buffer[4];
//	auto_data->rec_mini_yaw.bytes[1] = usb_recive_buffer[5];
//	auto_data->rec_mini_yaw.bytes[2] = usb_recive_buffer[6];
//	auto_data->rec_mini_yaw.bytes[3] = usb_recive_buffer[7];
	//auto_data->auto_yaw_set=auto_data->rec_mini_yaw.value;
	
}

//static void dispose_usb_data(mini_data_t *auto_data)
//{
//	
//	auto_data->pitch_PNflag = usb_recive_buffer[0]?1:-1;
//	auto_data->rec_mini_pitch.bytes[0] = usb_recive_buffer[1];
//	auto_data->rec_mini_pitch.bytes[1] = usb_recive_buffer[2];
//	auto_data->rec_mini_pitch.bytes[2] = usb_recive_buffer[3];
//	auto_data->rec_mini_pitch.bytes[3] = 0;
//	auto_data->auto_pitch_set=auto_data->rec_mini_pitch.value * auto_data->pitch_PNflag;
//	
//	auto_data->yaw_PNflag = usb_recive_buffer[4]?1:-1;
//	auto_data->rec_mini_yaw.bytes[0] = usb_recive_buffer[5];
//	auto_data->rec_mini_yaw.bytes[1] = usb_recive_buffer[6];
//	auto_data->rec_mini_yaw.bytes[2] = usb_recive_buffer[7];
//	auto_data->rec_mini_yaw.bytes[3] = 0;
//	auto_data->auto_yaw_set=auto_data->rec_mini_yaw.value * auto_data->yaw_PNflag;
//}
//static void dispose_usb_data(mini_data_t *auto_data)
//{
//	auto_data->auto_pitch_set = 0;//usb_recive_buffer[0];
//	auto_data->auto_yaw_set = (usb_recive_buffer[1]-40)*5;
//}

const mini_data_t* get_mini_data_point(void)
{
	return &auto_data;
}
