#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "main.h"





#define CHASSIS_X_CHANNEL	2
#define CHASSIS_Y_CHANNEL 3
#define CHASSIS_W_CHANNEL 0

typedef struct
{
	fp32 bullet_speed;
	fp32 shoot_cooling_heat;
}chassis_data_t;

extern chassis_data_t chassis_data;
#endif
