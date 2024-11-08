#ifndef AIMBOT_TASK_H
#define AIMBOT_TASK_H


#include "main.h"
typedef enum
{
	ceasefire = 0,
	fire = 1,
}Fire_e;

typedef union
{
    uint8_t bytes[4];
    float value;
}angle_rec;

typedef struct
{
	angle_rec rec_mini_yaw;
	angle_rec rec_mini_pitch;
	Fire_e Auto_fire;
	int8_t yaw_PNflag;
	int8_t pitch_PNflag;
	fp32 auto_pitch_set;
	fp32 auto_yaw_set;
}mini_data_t;


const mini_data_t* get_mini_data_point(void);



#endif
