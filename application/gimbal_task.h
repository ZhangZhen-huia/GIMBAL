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

//ƽ�� pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        5000.0f
#define PITCH_SPEED_PID_KI        0.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f	//30000
#define PITCH_SPEED_PID_MAX_IOUT  0.0f	//5000
#define PITCH_SPEED_KF_STATIC     0.0f
#define PITCH_SPEED_KF_DYNAMIC     0.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP -0.1f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f			//60
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f
#define PITCH_GYRO_ABSOLUTE_KF_STATIC     0.000f		//-0.005
#define PITCH_GYRO_ABSOLUTE_KF_DYNAMIC     0.0f


///*--------------------------------------Pitch--------------------------------------*/

////pitch �ٶȻ� PID�����Լ� PID���������������
//#define PITCH_SPEED_PID_KP        65000.0f
//#define PITCH_SPEED_PID_KI        20.0f
//#define PITCH_SPEED_PID_KD        0.0f
//#define PITCH_SPEED_PID_MAX_OUT   30000.0f
//#define PITCH_SPEED_PID_MAX_IOUT  3000.0f
//#define PITCH_SPEED_KF_STATIC     500.0f
//#define PITCH_SPEED_KF_DYNAMIC     500.0f

////pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
//#define PITCH_GYRO_ABSOLUTE_PID_KP 0.5f
//#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 5.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f
//#define PITCH_GYRO_ABSOLUTE_KF_STATIC     0.03f
//#define PITCH_GYRO_ABSOLUTE_KF_DYNAMIC     5.0f


////pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
//#define PITCH_ENCODE_RELATIVE_PID_KP 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
//#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 50.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//#define PITCH_ENCODE_RELATIVE_KF_STATIC     0.0f
//#define PITCH_ENCODE_RELATIVE_KF_DYNAMIC     0.0f


/*--------------------------------------Yaw--------------------------------------*/

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        10000.0f //35000
#define YAW_SPEED_PID_KI        0.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  0.0f
#define YAW_SPEED_KF_STATIC     0.0f
#define YAW_SPEED_KF_DYNAMIC     0.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP        0.1f //0.4
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
#define YAW_GYRO_ABSOLUTE_KF_STATIC     0.0f
#define YAW_GYRO_ABSOLUTE_KF_DYNAMIC     0.0f


//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        0.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

#define YAW_ENCODE_RELATIVE_KF_STATIC     0.0f
#define YAW_ENCODE_RELATIVE_KF_DYNAMIC     0.0f

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191



typedef enum
{
    GIMBAL_MOTOR_GYRO=0,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
		GIMBAL_MOTOR_AIMBOT,
		
} gimbal_motor_mode_e;


typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,    
	GIMBAL_AIMBOT_ANGLE,
} gimbal_behaviour_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_absolute_angle_pid; //�����ǽǶ�
    pid_type_def gimbal_motor_relative_angle_pid; //ecdת�ĽǶ���
	
    pid_type_def gimbal_motor_gyro_pid; 					//���ٶ�
	
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
    fp32 motor_speed;
    fp32 raw_cmd_current;
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
		fp32 final_yaw;																												//������������yaw������
} gimbal_control_t;


//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

extern gimbal_control_t gimbal_control;

#define CHASSIS_CONTROL_TIME 0.01f   //x��y�������β���
#define CHASSIS_CONTROL_TIME_Z 0.005f  //z�������β���

//������һ�β���ռ��
#define CHASSIS_ACCEL_X_NUM 0.99f
#define CHASSIS_ACCEL_Y_NUM 0.99f
#define CHASSIS_ACCEL_Z_NUM 0.995f
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//��û�õ�
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


//С���ݰ뾶 m 
#define MOTOR_DISTANCE_TO_CENTER 0.235619445f

//С�����ܳ�
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//���Ӱ뾶  m
#define WHEEL_HALF_SIZE 	0.0375f

//�����ܳ�	m
#define WHEEL_CIRCUMFERENCE				0.235619445f          //WHEEL_HALF_SIZE*2*3.1415926f  	


//ǰ������ٶ�  1.84175866175m/s   --8911
//									0						0

//���ٱ�19��rpm: Ȧ/min
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���	
#define CHASSIS_VX_RC_SEN            19.0f/60.0f*WHEEL_CIRCUMFERENCE
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE / SMALL_TOP_CIRCUMFERENCE/2.0f/PI			


#define YAW_RC_SEN    -0.0005f
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

void gimbal_task(void const * argument);

#endif



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

//ƽ�� pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP        5000.0f
#define PITCH_SPEED_PID_KI        0.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f	//30000
#define PITCH_SPEED_PID_MAX_IOUT  0.0f	//5000
#define PITCH_SPEED_KF_STATIC     0.0f
#define PITCH_SPEED_KF_DYNAMIC     0.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define PITCH_GYRO_ABSOLUTE_PID_KP -0.1f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f			//60
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f
#define PITCH_GYRO_ABSOLUTE_KF_STATIC     0.000f		//-0.005
#define PITCH_GYRO_ABSOLUTE_KF_DYNAMIC     0.0f


///*--------------------------------------Pitch--------------------------------------*/

////pitch �ٶȻ� PID�����Լ� PID���������������
//#define PITCH_SPEED_PID_KP        65000.0f
//#define PITCH_SPEED_PID_KI        20.0f
//#define PITCH_SPEED_PID_KD        0.0f
//#define PITCH_SPEED_PID_MAX_OUT   30000.0f
//#define PITCH_SPEED_PID_MAX_IOUT  3000.0f
//#define PITCH_SPEED_KF_STATIC     500.0f
//#define PITCH_SPEED_KF_DYNAMIC     500.0f

////pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
//#define PITCH_GYRO_ABSOLUTE_PID_KP 0.5f
//#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 5.0f
//#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f
//#define PITCH_GYRO_ABSOLUTE_KF_STATIC     0.03f
//#define PITCH_GYRO_ABSOLUTE_KF_DYNAMIC     5.0f


////pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
//#define PITCH_ENCODE_RELATIVE_PID_KP 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
//#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 50.0f
//#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//#define PITCH_ENCODE_RELATIVE_KF_STATIC     0.0f
//#define PITCH_ENCODE_RELATIVE_KF_DYNAMIC     0.0f


/*--------------------------------------Yaw--------------------------------------*/

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        10000.0f //35000
#define YAW_SPEED_PID_KI        0.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  0.0f
#define YAW_SPEED_KF_STATIC     0.0f
#define YAW_SPEED_KF_DYNAMIC     0.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP        0.1f //0.4
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
#define YAW_GYRO_ABSOLUTE_KF_STATIC     0.0f
#define YAW_GYRO_ABSOLUTE_KF_DYNAMIC     0.0f


//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        0.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

#define YAW_ENCODE_RELATIVE_KF_STATIC     0.0f
#define YAW_ENCODE_RELATIVE_KF_DYNAMIC     0.0f

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191



typedef enum
{
    GIMBAL_MOTOR_GYRO=0,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
		GIMBAL_MOTOR_AIMBOT,
		
} gimbal_motor_mode_e;


typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,    
	GIMBAL_AIMBOT_ANGLE,
} gimbal_behaviour_e;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_absolute_angle_pid; //�����ǽǶ�
    pid_type_def gimbal_motor_relative_angle_pid; //ecdת�ĽǶ���
	
    pid_type_def gimbal_motor_gyro_pid; 					//���ٶ�
	
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
    fp32 motor_speed;
    fp32 raw_cmd_current;
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
   // gimbal_step_cali_t gimbal_cali;
		gimbal_behaviour_e  gimbal_behaviour;																	//��̨״̬����
		gimbal_behaviour_e  last_gimbal_behaviour;														//��̨״̬����
		
} gimbal_control_t;


//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 1
#define GIMBAL_MODE_CHANNEL 0

extern gimbal_control_t gimbal_control;

#define CHASSIS_CONTROL_TIME 0.01f   //x��y�������β���
#define CHASSIS_CONTROL_TIME_Z 0.005f  //z�������β���

//������һ�β���ռ��
#define CHASSIS_ACCEL_X_NUM 0.99f
#define CHASSIS_ACCEL_Y_NUM 0.99f
#define CHASSIS_ACCEL_Z_NUM 0.995f
#define R       MOTOR_DISTANCE_TO_CENTER
#define PI       3.1415926f

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//��û�õ�
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


//С���ݰ뾶 m 
#define MOTOR_DISTANCE_TO_CENTER 0.235619445f

//С�����ܳ�
#define SMALL_TOP_CIRCUMFERENCE	 	MOTOR_DISTANCE_TO_CENTER*2*3.1415926f			//1.480440609656214f

//���Ӱ뾶  m
#define WHEEL_HALF_SIZE 	0.0375f

//�����ܳ�	m
#define WHEEL_CIRCUMFERENCE				0.235619445f          //WHEEL_HALF_SIZE*2*3.1415926f  	


//ǰ������ٶ�  1.84175866175m/s   --8911
//									0						0

//���ٱ�19��rpm: Ȧ/min
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���	
#define CHASSIS_VX_RC_SEN            19.0f/60.0f*WHEEL_CIRCUMFERENCE
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 					 19.0f/60.0f*WHEEL_CIRCUMFERENCE / SMALL_TOP_CIRCUMFERENCE/2.0f/PI			


#define YAW_RC_SEN    -0.0005f
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

void gimbal_task(void const * argument);

#endif



