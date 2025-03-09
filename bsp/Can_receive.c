#include "stm32f4xx_hal.h"
#include "main.h"
#include "Can_receive.h"
#include "communicate_task.h"
#include "detect_task.h"



//������ݶ�ȡ
#define get_motor_measure(ptr, data)                                 \
{                                                                    \
	(ptr)->last_ecd = (ptr)->ecd;                                      \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);               \
	(ptr)->rpm = (uint16_t)((data)[2] << 8 | (data)[3]);         			 \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
	(ptr)->temperate = (data)[6];                                      \
}

#define get_shootdata_measure(ptr, data)                                  			  \
    {                                                                   					\
        (ptr)->bullet_speed = ((data)[0]<<8 | (data)[1]);					              \
			  (ptr)->shoot_cooling_heat = (uint16_t) ((data)[2]<<8 | (data)[3]);     		\
    }


//����yaw��pitch,Ħ���֣������̵�����ݽṹ��
motor_measure_t yaw_motor,pitch_motor,friction_motor[2],trigger_motor;

uint8_t rx_data1[7],rx_data2[7];


void canfilter_init_start(void)
{
		CAN_FilterTypeDef can_filter_st;	                //����������ṹ��
    can_filter_st.FilterActivation = ENABLE;			//ENABLEʹ�ܹ�����
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;	//���ù�����ģʽ--��ʶ������λģʽ
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;	//��������λ�� 32 λ
    can_filter_st.FilterIdHigh = 0x0000;				//ID��λ
    can_filter_st.FilterIdLow = 0x0000;					//ID��λ
    can_filter_st.FilterMaskIdHigh = 0x0000;			//�����������λ
    can_filter_st.FilterMaskIdLow = 0x0000;				//�����������λ
    
    can_filter_st.FilterBank = 0;						//��������-˫CAN��ָ��0~27
    can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;	//������������� FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);		//HAL�����ù���������
	
    HAL_CAN_Start(&hcan1);								//ʹ��CAN1������
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ��CAN�ĸ����ж�
	
	
    can_filter_st.SlaveStartFilterBank = 14;   //˫CANģʽ�¹涨CAN������ģʽ�Ĺ��������䣬�ӹ�����Ϊ14
    can_filter_st.FilterBank = 14;						//��������-˫CAN��ָ��0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);		//HAL�����ù���������
    HAL_CAN_Start(&hcan2);								//ʹ��CAN2������
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ��CAN�ĸ����ж�
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)//  CAN FIFO0���жϻص�������������������ݵĽ���
{
	CAN_RxHeaderTypeDef rx_header2,rx_header1;
	if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&rx_header2,rx_data2);
		switch(rx_header2.StdId)
		{
			case CAN_PIT_MOTOR_ID:
				get_motor_measure(&pitch_motor,rx_data2);
				detect_hook(PITCH_TOE);
				break;
			
			case CAN_Fric_L_ID:
				get_motor_measure(&friction_motor[Fric_L],rx_data2);
				detect_hook(FRIC_L_TOE);
				break;
			
			case CAN_Fric_R_ID:
				get_motor_measure(&friction_motor[Fric_R],rx_data2);
				detect_hook(FRIC_R_TOE);	
				break;




		}
	}
	else if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rx_header1,rx_data1);
		switch(rx_header1.StdId)
		{
			case CAN_YAW_MOTOR_ID:
				get_motor_measure(&yaw_motor,rx_data1);
				detect_hook(YAW_TOE);	
				break;



				
		}
		

	}
	
}




//������̨ pitch
void CAN_cmd_gimbal_pitch(int16_t pitch)
{
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef gimbal_tx_message;
	static uint8_t    gimbal_can_send_data[2];

	//0x1ff
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x02;
  gimbal_can_send_data[0] = (pitch >> 8);
  gimbal_can_send_data[1] = pitch;


  HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	

}

//������̨ yaw
void CAN_cmd_gimbal_yaw(int16_t yaw)
{
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef gimbal_tx_message;
	static uint8_t    gimbal_can_send_data[2];

	//0x1ff
  gimbal_tx_message.StdId = 0x2ff;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x02;
	
	gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

}

//����Ħ���ֵ������
void CAN_cmd_firc(int16_t L_fric_current,int16_t R_fric_current)
{
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef trigger_shoot_tx_message;
	static uint8_t    trigger_shoot_can_send_data[4];

	//0x200
  trigger_shoot_tx_message.StdId = CAN_Fric_ALL_ID;
  trigger_shoot_tx_message.IDE = CAN_ID_STD;
  trigger_shoot_tx_message.RTR = CAN_RTR_DATA;
  trigger_shoot_tx_message.DLC = 0x04;

	trigger_shoot_can_send_data[0] = (L_fric_current >> 8);
  trigger_shoot_can_send_data[1] = L_fric_current;
  trigger_shoot_can_send_data[2] = (R_fric_current >> 8);
  trigger_shoot_can_send_data[3] = R_fric_current;

  HAL_CAN_AddTxMessage(&hcan2, &trigger_shoot_tx_message, trigger_shoot_can_send_data, &send_mail_box);
}



//yaw
const motor_measure_t *get_gimbal_yaw_motor_measure_point(void)
{
    return &yaw_motor;
}


//pitch
const motor_measure_t *get_gimbal_pitch_motor_measure_point(void)
{
    return &pitch_motor;
}



//friction1��
const motor_measure_t *get_gimbal_friction_motor_measure_point(uint8_t id)
{
    return &friction_motor[id];
}

