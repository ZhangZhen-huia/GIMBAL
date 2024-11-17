#ifndef AIMBOT_TASK_H
#define AIMBOT_TASK_H


#include "main.h"
typedef enum
{
	ceasefire = 0,
	fire = 1,
}Fire_e;



typedef struct
{
	uint8_t heading[2];
	uint8_t tailing[2];
	Fire_e Auto_fire;
	fp32 auto_pitch_set;
	fp32 auto_yaw_set;
}mini_data_t;


const mini_data_t* get_mini_data_point(void);



#endif
