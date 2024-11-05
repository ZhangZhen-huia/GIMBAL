#ifndef __CHASSIS_BEHAVIOR__H
#define __CHASSIS_BEHAVIOR__H

#include "main.h"

#include "struct_typedef.h"

#include "gimbal_task.h"

#define MAX_SPEED   			 1.84f
#define HALF_MAX_SPEED   	 MAX_SPEED*0.5f
#define QUTR_MAX_SPEED    	HALF_MAX_SPEED*0.5f



void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);







#endif
