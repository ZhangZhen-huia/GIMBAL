#include "stm32f4xx_hal.h"
#include "main.h"
#include "Can_receive.h"
#include "user_task.h"




//电机数据读取
#define get_motor_measure(ptr, data)                                 \
{                                                                    \
	(ptr)->last_ecd = (ptr)->ecd;                                      \
	(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);               \
	(ptr)->rpm = (uint16_t)((data)[2] << 8 | (data)[3]);         \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);     \
	(ptr)->temperate = (data)[6];                                      \
}




//定义yaw，pitch,摩擦轮，拨弹盘电机数据结构体
motor_measure_t yaw_motor,pitch_motor,friction_motor[2],trigger_motor;

uint8_t rx_data1[7],rx_data2[7];
int16_t rec_rc[4];

void canfilter_init_start(void)
{
	CAN_FilterTypeDef can_filter_st;	                //定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;			//ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;	//设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;	//过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;				//ID高位
    can_filter_st.FilterIdLow = 0x0000;					//ID低位
	
	//1表示该位要与上面的ID相关位一莫一样
    can_filter_st.FilterMaskIdHigh = 0x0000;			//过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;				//过滤器掩码低位
    
    can_filter_st.FilterBank = 0;						//过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;	//与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);		//HAL库配置过滤器函数
	
    HAL_CAN_Start(&hcan1);								//使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN的各种中断
	
	
    can_filter_st.SlaveStartFilterBank = 14;   //双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;						//过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);		//HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);								//使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能CAN的各种中断
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)//  CAN FIFO0的中断回调函数，在里面完成数据的接收
{
	CAN_RxHeaderTypeDef rx_header1;
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rx_header1,rx_data1);
		switch(rx_header1.StdId)
		{
			case CAN_YAW_MOTOR_ID:get_motor_measure(&yaw_motor,rx_data1);
				break;
			case CAN_PIT_MOTOR_ID:get_motor_measure(&pitch_motor,rx_data1);
				break;
			case CAN_Fric_L_ID:get_motor_measure(&friction_motor[Fric_L],rx_data1);
				break;
			case CAN_Fric_R_ID:get_motor_measure(&friction_motor[Fric_R],rx_data1);
				break;
			case CAN_TRIGGER_MOTOR_ID:get_motor_measure(&trigger_motor,rx_data1);
				break;




		}
	}

//	else if(hcan->Instance==CAN2)
//	{
//		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&rx_header2,rx_data2);
//		switch(rx_header2.StdId)
//		{
//			case GIM_M1_ID:
//				Gim_Get_Motor_Data(MOTOR1,rx_data2,Revice_GimbalData);
//				break;
//			case GIM_M2_ID:
//				Gim_Get_Motor_Data(MOTOR2,rx_data2,Revice_GimbalData);
//				break;
//			case GIM_M3_ID:
//				Gim_Get_Motor_Data(MOTOR3,rx_data2,Revice_GimbalData);
//				break;
//			case GIM_M4_ID:
//				Gim_Get_Motor_Data(MOTOR4,rx_data2,Revice_GimbalData);
//				break;
//		}
//	}
	
}




//发送云台 pitch 和 yaw
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef gimbal_tx_message;
	static uint8_t    gimbal_can_send_data[4];

	//0x1ff
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;

  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


//发送拨弹盘和摩擦轮电机电流
void CAN_cmd_trigger_firc(int16_t trigger_current,int16_t L_fric_current,int16_t R_fric_current)
{
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef trigger_shoot_tx_message;
	static uint8_t    trigger_shoot_can_send_data[6];

	//0x200
  trigger_shoot_tx_message.StdId = CAN_Fric_Trg_ALL_ID;
  trigger_shoot_tx_message.IDE = CAN_ID_STD;
  trigger_shoot_tx_message.RTR = CAN_RTR_DATA;
  trigger_shoot_tx_message.DLC = 0x08;
	
  trigger_shoot_can_send_data[0] = (trigger_current >> 8);
  trigger_shoot_can_send_data[1] = trigger_current;
	trigger_shoot_can_send_data[2] = (L_fric_current >> 8);
  trigger_shoot_can_send_data[3] = L_fric_current;
  trigger_shoot_can_send_data[4] = (R_fric_current >> 8);
  trigger_shoot_can_send_data[5] = R_fric_current;

  HAL_CAN_AddTxMessage(&hcan1, &trigger_shoot_tx_message, trigger_shoot_can_send_data, &send_mail_box);
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

//trigger
const motor_measure_t *get_gimbal_trigger_motor_measure_point(void)
{
    return &trigger_motor;
}


//friction1右
const motor_measure_t *get_gimbal_friction_motor_measure_point(uint8_t id)
{
    return &friction_motor[id];
}

