#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

#include "main.h"
#include "Can_receive.h"
#include "pid.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"
#include "math.h"
#include "INS_task.h"
#include "communicate_task.h"

///*--------------------------------------Pitch--------------------------------------*/

//陀螺仪
#define PITCH_GYRO_SPEED_PID_KP        	0.0f//12000.0f
#define PITCH_GYRO_SPEED_PID_KI        	0.0f//0.0f
#define PITCH_GYRO_SPEED_PID_KD        	0.0f//0.0f
#define PITCH_GYRO_SPEED_PID_MAX_OUT   	0.0f//30000.0f
#define PITCH_GYRO_SPEED_PID_MAX_IOUT  	0.0f//0.0f

#define PITCH_GYRO_ANGLE_PID_KP 				0.0f//-0.8f
#define PITCH_GYRO_ANGLE_PID_KI 				0.0f//-0.1f
#define PITCH_GYRO_ANGLE_PID_KD 				0.0f//0.0f
#define PITCH_GYRO_ANGLE_PID_MAX_OUT  	0.0f//40.0f
#define PITCH_GYRO_ANGLE_PID_MAX_IOUT 	0.0f//0.3f

//编码器
#define PITCH_ENCODE_SPEED_PID_KP        12000.f
#define PITCH_ENCODE_SPEED_PID_KI        0.0f
#define PITCH_ENCODE_SPEED_PID_KD        0.0f
#define PITCH_ENCODE_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_ENCODE_SPEED_PID_MAX_IOUT  0.0f

#define PITCH_ENCODE_ANGLE_PID_KP 				-0.5f
#define PITCH_ENCODE_ANGLE_PID_KI 				-0.1f
#define PITCH_ENCODE_ANGLE_PID_KD 				 0.0f
#define PITCH_ENCODE_ANGLE_PID_SKP			 	 0.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_OUT 		 40.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_IOUT 	 0.1f

//自瞄
#define PITCH_AUTO_SPEED_PID_KP        12000.0f
#define PITCH_AUTO_SPEED_PID_KI        0.0f
#define PITCH_AUTO_SPEED_PID_KD        0.0f
#define PITCH_AUTO_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_AUTO_SPEED_PID_MAX_IOUT  0.0f


#define PITCH_AUTO_ANGLE_PID_KP 				-0.5f
#define PITCH_AUTO_ANGLE_PID_KI 				-0.1f
#define PITCH_AUTO_ANGLE_PID_KD 				0.0f
#define PITCH_AUTO_ANGLE_PID_SKP				0.0f
#define PITCH_AUTO_ANGLE_PID_MAX_OUT 		40.0f
#define PITCH_AUTO_ANGLE_PID_MAX_IOUT 	0.1f



/*--------------------------------------Yaw--------------------------------------*/




//陀螺仪
#define YAW_GYRO_SPEED_PID_KP        12500.0f//20000.0f //35000
#define YAW_GYRO_SPEED_PID_KI        0.0f
#define YAW_GYRO_SPEED_PID_KD        1.0f
#define YAW_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT  0.0f

#define YAW_GYRO_ANGLE_PID_KP        0.35f//1.0f //0.4
#define YAW_GYRO_ANGLE_PID_KI        0.01f
#define YAW_GYRO_ANGLE_PID_KD        0.0f
#define YAW_GYRO_ANGLE_PID_MAX_OUT   360.0f
#define YAW_GYRO_ANGLE_PID_MAX_IOUT  0.2f


//自瞄
#define YAW_AUTO_SPEED_PID_KP        12500.0f//20000.0f //35000
#define YAW_AUTO_SPEED_PID_KI        0.0f
#define YAW_AUTO_SPEED_PID_KD        1.0f
#define YAW_AUTO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_AUTO_SPEED_PID_MAX_IOUT  0.0f
                                     
#define YAW_AUTO_ANGLE_PID_KP        0.35f//1.0f //0.4
#define YAW_AUTO_ANGLE_PID_KI        0.01f
#define YAW_AUTO_ANGLE_PID_KD        0.0f
#define YAW_AUTO_ANGLE_PID_MAX_OUT   360.0f
#define YAW_AUTO_ANGLE_PID_MAX_IOUT  0.2f//0.001f






//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191



typedef enum
{
    GIMBAL_MOTOR_GYRO=0,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
		GIMBAL_MOTOR_AUTO,

} gimbal_motor_mode_e;


typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_GYRO_ANGLE, 
  GIMBAL_ENCODE_ANGLE, 
  GIMBAL_MOTIONLESS,    
	GIMBAL_AUTO_ANGLE,

} gimbal_behaviour_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_gyro_angle_pid; //陀螺仪角度
		pid_type_def gimbal_motor_gyro_speed_pid;
	
    pid_type_def gimbal_motor_encode_angle_pid; //ecd转的角度制
		pid_type_def gimbal_motor_encode_speed_pid; 
	
    pid_type_def gimbal_motor_auto_angle_pid; //自瞄
		pid_type_def gimbal_motor_auto_speed_pid; 
//	
//		pid_type_def gimbal_motor_radar_speed_pid;//云台导航速度环
	
    gimbal_motor_mode_e gimbal_motor_mode;						//电机控制状态
    gimbal_motor_mode_e last_gimbal_motor_mode;				//上次电机控制状态
    uint16_t offset_ecd;													
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad
	
    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
	
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
		
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 current_set;
    int16_t given_current;

} gimbal_motor_t;


typedef struct
{
		const mini_data_t	*gimbal_mini_data;																	//获取小电脑数据指针
    const RC_ctrl_t *gimbal_rc_ctrl;																			//获取遥控器指针
		const bmi088_real_data_t *gimbal_bmi088_data;     										//获取陀螺仪解算出的欧拉角指针
    gimbal_motor_t gimbal_yaw_motor;																			//yaw轴电机数据
    gimbal_motor_t gimbal_pitch_motor;																		//pitch轴电机数据
   //gimbal_step_cali_t gimbal_cali;
		gimbal_behaviour_e  gimbal_behaviour;																	//云台状态控制
		gimbal_behaviour_e  last_gimbal_behaviour;														//云台状态控制
} gimbal_control_t;


//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

extern gimbal_control_t gimbal_control;

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357





#define YAW_RC_SEN    -0.002f
#define PITCH_RC_SEN  -0.0005f //0.005

#define YAW_MOUSE_SEN   0.005f
#define PITCH_MOUSE_SEN 0.005f


#define GIMBAL_CONTROL_TIME 1

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.0439453125f //      2*  PI  /8192
#endif

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 2000

//摇杆死区
#define RC_DEADBAND 10 
extern gimbal_control_t gimbal_control;	
extern fp32 yaw_relative_init;
void gimbal_task(void const * argument);

#endif



