#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H



#include "main.h"
#include "pid.h"
#include "Can_receive.h"
#include "detect_task.h"
#include "remote_control.h"
#include "vofa_task.h"

//rpm转换到线速度的比率，还未给
#define FRIC_RPM_TO_SPEED_SEN   0.0031416
#define TRIG_RPM_TO_SPEED_SEN   0.0031416

//任务初始化时间
#define SHOOT_TASK_INIT_TIME    1000

//拨弹盘速度pid
#define TRIG_SPEED_PID_KP           15.0f
#define TRIG_SPEED_PID_KI           0.0f
#define TRIG_SPEED_PID_KD           0.0f
#define TRIG_SPEED_PID_MAX_OUT     10000.0f
#define TRIG_SPEED_PID_MAX_IOUT    0.0f

#define TRIG_SPEED_KF_STATIC        0.0f
#define TRIG_SPEED_KF_DYNAMIC       0.0f


//拨弹盘角度pid
#define TRIG_ANGLE_PID_KP           0.30f
#define TRIG_ANGLE_PID_KI           0.0f
#define TRIG_ANGLE_PID_KD           0.0f
#define TRIG_ANGLE_PID_MAX_OUT     30000.0f
#define TRIG_ANGLE_PID_MAX_IOUT    0.0f

#define TRIG_ANGLE_KF_STATIC        0.0f
#define TRIG_ANGLE_KF_DYNAMIC       0.0f


//左摩擦轮速度pid
#define FRIC_L_SPEED_PID_KP         9000.0f
#define FRIC_L_SPEED_PID_KI         500.0f
#define FRIC_L_SPEED_PID_KD         0.0f
#define FRIC_L_SPEED_PID_MAX_OUT   16384.0f
#define FRIC_L_SPEED_PID_MAX_IOUT  10000.0f

#define FRIC_L_SPEED_KF_STATIC      10000.0f
#define FRIC_L_SPEED_KF_DYNAMIC     0.0f


//右摩擦轮速度pid
#define FRIC_R_SPEED_PID_KP         9000.0f
#define FRIC_R_SPEED_PID_KI         500.0f
#define FRIC_R_SPEED_PID_KD         0.0f
#define FRIC_R_SPEED_PID_MAX_OUT   16384.0f
#define FRIC_R_SPEED_PID_MAX_IOUT  10000.0f

#define FRIC_R_SPEED_KF_STATIC      10000.0f
#define FRIC_R_SPEED_KF_DYNAMIC     0.0f


#define TRIG_CHANNEL   1

//shoot相关电机信息数据包
typedef struct
{
	const motor_measure_t  *shoot_motor_measure;
	fp32 motor_speed;
  fp32 motor_speed_set;
  int16_t current_set;
	pid_type_def shoot_speed_pid;
	pid_type_def shoot_angle_pid;

} shoot_motor_t;



//发射总结构体
typedef struct 
{
    const RC_ctrl_t *shoot_rc_ctrl;																			//获取遥控器指针
		shoot_motor_t shoot_trig_motor;		
    shoot_motor_t shoot_fric_L_motor;
    shoot_motor_t shoot_fric_R_motor;
		DebugData shoot_debug1;
}shoot_control_t;


const DebugData* get_shoot_PID_Debug(void);

#endif
