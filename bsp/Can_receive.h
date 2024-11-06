#ifndef _Can_Receice_H
#define _Can_Receice_H

#include "stm32f4xx_hal.h"
#include "can.h"

/*****************������ݻ��սṹ��********************/ 
typedef struct 
{
		uint16_t ecd; //ת�ӻ�е�Ƕ� 0-8191
    int16_t rpm; // ת��ת��ֵ r/min
    int16_t given_current; //��ǰ����ֵ -16384-16384
    uint8_t temperate; //Temp/��
    int16_t last_ecd; //��һ�εĻ�е�Ƕ�
}motor_measure_t;


#define Fric_L 0
#define Fric_R 1
typedef enum
{
/*----------can1����id----------*/
	
	/*---��̨---*/
	CAN_GIMBAL_ALL_ID = 0x1FF,
	
	/*---������+Ħ����---*/
	CAN_Fric_Trg_ALL_ID = 0x200,
	
	
/*----------can1����id----------*/


	
	/*---������---*/
	CAN_TRIGGER_MOTOR_ID = 0x201, //1
	/*---Ħ����---*/
  CAN_Fric_L_ID = 0x202,			//2
  CAN_Fric_R_ID = 0x203,			//3
	
	
	
	
		
//	/*---������---*/
//	CAN_TRIGGER_MOTOR_ID = 0x203, //3
//	/*---Ħ����---*/
//  CAN_Fric_L_ID = 0x201,			//1
//  CAN_Fric_R_ID = 0x202,			//2
	
	
	/*---��̨---*/
	CAN_YAW_MOTOR_ID = 0x205,		//1
  CAN_PIT_MOTOR_ID = 0x206,		//2

	

/*------------------------------*/

} can_msg_id_e;




void canfilter_init_start(void);
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch);
void CAN_cmd_trigger_firc(int16_t trigger_current,int16_t L_fric_current,int16_t R_fric_current);

const motor_measure_t *get_gimbal_friction_motor_measure_point(uint8_t id);
const motor_measure_t *get_gimbal_trigger_motor_measure_point(void);
const motor_measure_t *get_gimbal_pitch_motor_measure_point(void);
const motor_measure_t *get_gimbal_yaw_motor_measure_point(void);

#endif
