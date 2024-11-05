#ifndef __DETECT_TASK
#define __DETECT_TASK

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "remote_control.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10
enum errorList
{
    DBUS_TOE = 0,
//    CHASSIS_MOTOR1_TOE,
//    CHASSIS_MOTOR2_TOE,
//    CHASSIS_MOTOR3_TOE,
//    CHASSIS_MOTOR4_TOE,
//    YAW_GIMBAL_MOTOR_TOE,
//    PITCH_GIMBAL_MOTOR_TOE,
//    TRIGGER_MOTOR_TOE,
//    BOARD_GYRO_TOE,
//    BOARD_ACCEL_TOE,
//    BOARD_MAG_TOE,
//    REFEREE_TOE,
//    RM_IMU_TOE,
//    OLED_TOE,
    ERROR_LIST_LENGHT,
};


/**
	*C����������һ���ṹ������λΪ��λ��ָ�����Ա��ռ�ڴ泤�ȣ�
	*������λΪ��λ�ĳ�Ա��Ϊ��λ�Ρ���ơ�λ��( bit field) ��
	*����λ���ܹ��ý��ٵ�λ���洢���ݡ�
	* uint16_t set_offline_time : 12;		��ʾ�������ռ��12λ
**/
typedef __packed struct
{
    uint32_t new_time;
    uint32_t last_time;
    uint32_t lost_time;
    uint32_t work_time;
    uint16_t set_offline_time : 12;
    uint16_t set_online_time : 12;
    uint8_t enable : 1;
    uint8_t priority : 4;
    uint8_t error_exist : 1;
    uint8_t is_lost : 1;
    uint8_t data_is_error : 1;

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} error_t;

static void detect_init(uint32_t time);
unsigned char toe_is_error(uint8_t toe);
void detect_hook(uint8_t toe);

void detect_task(void const * argument);
#endif
