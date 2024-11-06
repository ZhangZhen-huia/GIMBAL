#ifndef _VOFA_TASK_H_
#define _VOFA_TASK_H_

#include "main.h"
#include "usart.h"


typedef struct
{
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
	float data6;
}DebugData;


/*----发射pid调试使能----*/
#define SHOOT_DEBUG

/*----Vofa任务使能----*/
#define Vofa_Debug


void Vofa_Send(float data1, float data2, float data3, float data4, float data5, float data6);

#endif

