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
#include "aimbots_task.h"

///*--------------------------------------Pitch--------------------------------------*/

//������
#define PITCH_GYRO_SPEED_PID_KP        15000.0f
#define PITCH_GYRO_SPEED_PID_KI        0.0f
#define PITCH_GYRO_SPEED_PID_KD        0.0f
#define PITCH_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_GYRO_SPEED_PID_MAX_IOUT  0.0f

#define PITCH_GYRO_ANGLE_PID_KP -0.8f
#define PITCH_GYRO_ANGLE_PID_KI -0.01f
#define PITCH_GYRO_ANGLE_PID_KD 0.0f
#define PITCH_GYRO_ANGLE_PID_MAX_OUT 40.0f
#define PITCH_GYRO_ANGLE_PID_MAX_IOUT 0.5f

//������
#define PITCH_ENCODE_SPEED_PID_KP        10000.0f
#define PITCH_ENCODE_SPEED_PID_KI        0.0f
#define PITCH_ENCODE_SPEED_PID_KD        0.0f
#define PITCH_ENCODE_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_ENCODE_SPEED_PID_MAX_IOUT  0.0f

#define PITCH_ENCODE_ANGLE_PID_KP -0.7f
#define PITCH_ENCODE_ANGLE_PID_KI -0.05f
#define PITCH_ENCODE_ANGLE_PID_KD 0.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_OUT 40.0f
#define PITCH_ENCODE_ANGLE_PID_MAX_IOUT 0.5f

//����
#define PITCH_AUTO_SPEED_PID_KP        10000.0f//15000.0f
#define PITCH_AUTO_SPEED_PID_KI        0.0f
#define PITCH_AUTO_SPEED_PID_KD        0.0f
#define PITCH_AUTO_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_AUTO_SPEED_PID_MAX_IOUT  0.0f


#define PITCH_AUTO_ANGLE_PID_KP -0.7f
#define PITCH_AUTO_ANGLE_PID_KI -0.05f
#define PITCH_AUTO_ANGLE_PID_KD 0.0f
#define PITCH_AUTO_ANGLE_PID_MAX_OUT 40.0f
#define PITCH_AUTO_ANGLE_PID_MAX_IOUT 0.5f



/*--------------------------------------Yaw--------------------------------------*/

//������
#define YAW_GYRO_SPEED_PID_KP        15000.0f //35000
#define YAW_GYRO_SPEED_PID_KI        0.0f
#define YAW_GYRO_SPEED_PID_KD        0.0f
#define YAW_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT  0.0f

#define YAW_GYRO_ANGLE_PID_KP        0.8f //0.4
#define YAW_GYRO_ANGLE_PID_KI        0.0f
#define YAW_GYRO_ANGLE_PID_KD        0.0f
#define YAW_GYRO_ANGLE_PID_MAX_OUT   360.0f
#define YAW_GYRO_ANGLE_PID_MAX_IOUT  0.0f


//����
#define YAW_AUTO_SPEED_PID_KP        10000.0f//15000.0f //35000
#define YAW_AUTO_SPEED_PID_KI        0.0f
#define YAW_AUTO_SPEED_PID_KD        0.0f
#define YAW_AUTO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_AUTO_SPEED_PID_MAX_IOUT  0.0f


#define YAW_AUTO_ANGLE_PID_KP        0.5f //0.4
#define YAW_AUTO_ANGLE_PID_KI        0.0f
#define YAW_AUTO_ANGLE_PID_KD        0.0f
#define YAW_AUTO_ANGLE_PID_MAX_OUT   360.0f
#define YAW_AUTO_ANGLE_PID_MAX_IOUT  0.0f






//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191



typedef enum
{
    GIMBAL_MOTOR_GYRO=0,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
		GIMBAL_MOTOR_AUTO,
		GIMBAL_MOTOR_RADAR,
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
	GIMBAL_FOLLOW_RADAR,
} gimbal_behaviour_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_gyro_angle_pid; //�����ǽǶ�
		pid_type_def gimbal_motor_gyro_speed_pid;
	
    pid_type_def gimbal_motor_encode_angle_pid; //ecdת�ĽǶ���
		pid_type_def gimbal_motor_encode_speed_pid; 
	
    pid_type_def gimbal_motor_auto_angle_pid; //����
		pid_type_def gimbal_motor_auto_speed_pid; 
	
    gimbal_motor_mode_e gimbal_motor_mode;						//�������״̬
    gimbal_motor_mode_e last_gimbal_motor_mode;				//�ϴε������״̬
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
		const mini_data_t	*gimbal_mini_data;																	//��ȡС��������ָ��
    const RC_ctrl_t *gimbal_rc_ctrl;																			//��ȡң����ָ��
		const bmi088_real_data_t *gimbal_bmi088_data;     										//��ȡ�����ǽ������ŷ����ָ��
    gimbal_motor_t gimbal_yaw_motor;																			//yaw��������
    gimbal_motor_t gimbal_pitch_motor;																		//pitch��������
   //gimbal_step_cali_t gimbal_cali;
		gimbal_behaviour_e  gimbal_behaviour;																	//��̨״̬����
		gimbal_behaviour_e  last_gimbal_behaviour;														//��̨״̬����
} gimbal_control_t;


//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   0
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

extern gimbal_control_t gimbal_control;


//#define CHASSIS_ACCEL_Z_NUM 0.995f
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357



//С���ݰ뾶 m 
#define MOTOR_DISTANCE_TO_CENTER 0.235619445f

//С�����ܳ�
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//���Ӱ뾶  m
#define WHEEL_HALF_SIZE 	0.0375f

//�����ܳ�	m
#define WHEEL_CIRCUMFERENCE				0.235619445f          //WHEEL_HALF_SIZE*2*3.1415926f  	





#define YAW_RC_SEN    -0.001f
#define PITCH_RC_SEN  -0.0005f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.0005f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.0439453125f //      2*  PI  /8192
#endif

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 2000

//ҡ������
#define RC_DEADBAND 10 
extern gimbal_control_t gimbal_control;	
extern fp32 yaw_relative_init;
void gimbal_task(void const * argument);

#endif



