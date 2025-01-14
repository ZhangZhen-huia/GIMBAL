#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H



#include "main.h"
#include "pid.h"
#include "Can_receive.h"
#include "detect_task.h"
#include "remote_control.h"
#include "vofa_task.h"
#include "shoot_behaviour.h"
#include "aimbots_task.h"

//定义摩擦轮转速（rpm）转化为摩擦轮线速（m/s）
//rpm/60*60*3.1415926*10^-3//摩擦轮受损，直径可能会变小
#define FRIC_RPM_TO_SPEED_SEN   0.0031416


//任务初始化时间
#define SHOOT_TASK_INIT_TIME    500




//左摩擦轮速度pid
#define FRIC_L_SPEED_PID_KP         6000.0f
#define FRIC_L_SPEED_PID_KI         300.0f
#define FRIC_L_SPEED_PID_KD         750.0f
#define FRIC_L_SPEED_PID_MAX_OUT   16384.0f
#define FRIC_L_SPEED_PID_MAX_IOUT  2000.0f

#define FRIC_L_SPEED_KF_STATIC      20.0f
#define FRIC_L_SPEED_KF_DYNAMIC     0.0f


//右摩擦轮速度pid
#define FRIC_R_SPEED_PID_KP         6000.0f
#define FRIC_R_SPEED_PID_KI         500.0f
#define FRIC_R_SPEED_PID_KD         750.0f
#define FRIC_R_SPEED_PID_MAX_OUT   16384.0f
#define FRIC_R_SPEED_PID_MAX_IOUT  2000.0f

#define FRIC_R_SPEED_KF_STATIC      20.0f
#define FRIC_R_SPEED_KF_DYNAMIC     0.0f

#define MAX_SPEED   25



//shoot相关电机信息数据包
typedef struct
{
	const motor_measure_t  *shoot_motor_measure;
	fp32 motor_speed;
  fp32 motor_speed_set;
  int16_t current_set;
	pid_type_def shoot_speed_pid;

} shoot_motor_t;


//发射机构总开关
typedef enum
{
	SHOOT_ON  = 0,
	SHOOT_OFF = 1,
	
}shoot_agency_e;


//拨弹盘模式枚举
typedef enum
{
	Single_fire=0,
	Serial_fire,
	Cease_fire,
	Start_fire,

}trig_mode_e;


//摩擦轮开关
typedef enum
{
	STOP  = 0,
	START = 1,
}fric_mode_e;


//发射总结构体
typedef struct 
{
    const RC_ctrl_t *shoot_rc_ctrl;														//获取遥控器指针
		const uint8_t * auto_fireFlag;
    shoot_motor_t shoot_fric_L_motor;
    shoot_motor_t shoot_fric_R_motor;
		pid_type_def shoot_speed_compensate_pid;
		
		DebugData shoot_debug1;
	
		//发射机构总开关
		shoot_agency_e shoot_agency_state;
		shoot_agency_e shoot_agency_state_last;

		//拨弹盘模式开关
		trig_mode_e trig_mode;
		trig_mode_e trig_mode_last;
		
		//摩擦轮开关
		fric_mode_e fric_mode;
		fric_mode_e fric_mode_last;
		
		fp32 bullet_speed;
		fp32 shoot_cooling_heat;
		fp32 shoot_cooling_heat_last;
}shoot_control_t;


const DebugData* get_shoot_PID_Debug(void);
extern shoot_control_t shoot_control;

extern void shoot_motor_mode_set(shoot_control_t *shoot_mode);

#endif
