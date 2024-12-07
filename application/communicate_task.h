#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "main.h"





#define CHASSIS_X_CHANNEL	2
#define CHASSIS_Y_CHANNEL 3
#define CHASSIS_W_CHANNEL 0

typedef struct
{
	uint8_t heading[3];
	uint8_t tailing[3];
	fp32 vx;
	fp32 vy;
	fp32 wz;
}Radar_data_t;

typedef struct
{
	fp32 pitch;
}chassis_data_t;

extern chassis_data_t chassis_data;

void get_chassis_data(chassis_data_t *chassis_data,uint8_t *buf);
#endif
