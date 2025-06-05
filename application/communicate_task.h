#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#include "main.h"

#define Heading	0xFF//'I'
#define Tail		0x0D//'O'

typedef struct
{
	uint8_t heading;
	uint8_t tailing;
	uint8_t auto_fireFlag;
	fp32 auto_pitch_set;
	fp32 auto_yaw_set;
	fp32 distance;
}mini_data_t;

typedef enum
{
	NORMAL = 0,
	BIG = 4,
	SMALL = 2,
}Aimbot_Mode_e;

//小符红是2
//大符红是3
//小符蓝是4
//大符蓝是5
typedef enum
{
	BLUE = 1,
	RED	 = 0,
}EnemyColor_e;

const mini_data_t* get_mini_data_point(void);
extern mini_data_t auto_data;


#define CHASSIS_X_CHANNEL	2
#define CHASSIS_Y_CHANNEL 3
#define CHASSIS_W_CHANNEL 0


typedef struct
{
	fp32 bullet_speed;
	fp32 shoot_cooling_heat;
}chassis_data_t;

extern EnemyColor_e EnemyColor;
extern chassis_data_t chassis_data;

#endif
