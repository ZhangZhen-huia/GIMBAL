#include "vofa_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "vofa_task.h"
#include "shoot_task.h"



const DebugData *shoot_debug;
float Vofa_Data[6];

void vofa_justfloat(UART_HandleTypeDef* huart, float* fdata, uint16_t count);
void VofaInit(void);


void vofa_task(void const * argument)
{
	#ifndef Vofa_Debug
	vTaskDelete(NULL);
	#endif
	VofaInit();
	
	while(1)
	{
		Vofa_Send(shoot_debug->data1, shoot_debug->data2, shoot_debug->data3,shoot_debug->data4, shoot_debug->data5, shoot_debug->data6);
		
		vTaskDelay(5);
	}
	

}



const DebugData* get_shoot_PID_Debug(void);

void VofaInit(void)
{
	shoot_debug = get_shoot_PID_Debug();

}









void Vofa_Send(float data1, float data2, float data3, float data4, float data5, float data6)
{
  //数据绑定	
	Vofa_Data[0] = data1;
	Vofa_Data[1] = data2;
	Vofa_Data[2] = data3;
	Vofa_Data[3] = data4;	
	Vofa_Data[4] = data5;
	Vofa_Data[5] = data6;
	//数据发送
	vofa_justfloat(&huart1,Vofa_Data,6);
}


void vofa_justfloat(UART_HandleTypeDef* huart, float* fdata, uint16_t count)
{
	// 最多支持长度为6的float数组
	uint8_t data[40];
	int i = 0;
	for (i=0;i<count*4;i++)
	data[i] = ((uint8_t*)fdata)[i];
	data[i++] = 0;
	data[i++] = 0;
	data[i++] = 0x80;
	data[i++] = 0x7f;
	HAL_UART_Transmit(huart, data, count*4+4, 1000);
}






