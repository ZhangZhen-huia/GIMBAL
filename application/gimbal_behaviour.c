#include "gimbal_behaviour.h"
#include "referee.h"
#include "detect_task.h"
#include "key_task.h"
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
//static void gimbal_follow_radar_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);


/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
  * @param          �����ң����ֵ
  * @param          ��������������ң����ֵ
  * @param          ����ֵ
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_gyro_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);


    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = -(pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);
//    {
//        static uint16_t last_turn_keyboard = 0;
//        static uint8_t gimbal_turn_flag = 0;
//        static fp32 gimbal_end_angle = 0.0f;

//        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
//        {
//            if (gimbal_turn_flag == 0)
//            {
//                gimbal_turn_flag = 1;
//                //�����ͷ��Ŀ��ֵ
//                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
//            }
//        }
//        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

//        if (gimbal_turn_flag)
//        {
//            //���Ͽ��Ƶ���ͷ��Ŀ��ֵ����ת����װ�����
//            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
//            {
//                *yaw += TURN_SPEED;
//            }
//            else
//            {
//                *yaw -= TURN_SPEED;
//            }
//        }
//        //����pi ��180�㣩��ֹͣ
//        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
//        {
//            gimbal_turn_flag = 0;
//        }
//    }
}


/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);


    *yaw = yaw_channel * YAW_RC_SEN - Mouse_Data.mouse_x * YAW_MOUSE_SEN;
    *pitch = -(pitch_channel * PITCH_RC_SEN + Mouse_Data.mouse_y * PITCH_MOUSE_SEN);
	
		/*-- һ����ͷ --*/
		if(Key_ScanValue.Key_Value.r)
		{
			 *yaw = 180;
		}

}

/*-- ������ --*/
//fp32 aim_yaw_set;
//fp32 aim_pitch_set;
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_auto_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = auto_data.auto_yaw_set;//aim_yaw_set;//gimbal_control_set->gimbal_mini_data->auto_yaw_set;
    *pitch =auto_data.auto_pitch_set;//aim_pitch_set;//gimbal_control_set->gimbal_mini_data->auto_pitch_set;


}


/*-- ��һ�汾���е��״���ƶ�ȡ���� --*/
///**
//  * @brief          �״���ƣ����Զ�ģʽ
//  * @param[in]      yaw: yaw��Ƕȿ��ƣ������ǿ���Ϊ�Ƕȵ����� ��λ rad
//  * @param[in]      pitch: pitch��Ƕȿ��ƣ�����ֵ���ƣ�Ϊ�Ƕȵ����� ��λ rad
//  * @param[in]      gimbal_control_set: ��̨����ָ��
//  * @retval         none
//  */
//static void gimbal_follow_radar_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
//{

//	static int8_t turn_flag = 1;
//	    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }


//		//*yaw = 0.3;
//		
//		if(turn_flag == 1)
//		{
//			*pitch = 0.1;
//			if(gimbal_control_set->gimbal_pitch_motor.relative_angle_set == gimbal_control_set->gimbal_pitch_motor.max_relative_angle)
//				turn_flag = -1;
//		}
//		
//		else if(turn_flag == -1)
//		{
//			*pitch = -0.1;
//			if(gimbal_control_set->gimbal_pitch_motor.relative_angle_set == gimbal_control_set->gimbal_pitch_motor.min_relative_angle)
//				turn_flag = 1;

//		}


//}








/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      gimbal_mode_set:��̨����ָ��
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

		 /*-- ���ݲ�ͬģʽ����pitch��yaw������ȡ --*/
    if (gimbal_control_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE )
    {
        gimbal_gyro_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
    {
        gimbal_encode_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
		 else if (gimbal_control_set->gimbal_behaviour == GIMBAL_AUTO_ANGLE)
    {
        gimbal_auto_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }

	
			
			
}




/**
  * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
  * @param[out]     gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //��̨��Ϊ״̬�����ã�6��ģʽ������ֻ�õ��������Ǿ��ԽǶȿ��ƺͱ�������ԽǶȿ��ƣ�
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    //������̨��Ϊ״̬�����õ��״̬��
    if (gimbal_mode_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_mode_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		else if (gimbal_mode_set->gimbal_behaviour == GIMBAL_AUTO_ANGLE)
		{
			  gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_AUTO;

		}

}

/**
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
		
    if (gimbal_mode_set == NULL)
    {
        return;
    }

////		//�Ҽ����½�������
//	if(Mouse_Data.mouse_r == 1)
//	{
//		gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
//	}
//	else
//	{
//		gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
//	}	
//	
	
//	
//		if(Key_ScanValue.Key_Value.Q)// && Key_ScanValue.Key_Value_Last.Q !=1)
//		{
//			if( gimbal_mode_set->last_gimbal_behaviour == GIMBAL_AUTO_ANGLE)
//			{
//					gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
//			}
//			else if(gimbal_mode_set->last_gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
//			{
//				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
//			}
//		}
		
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
    }
		
		else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				//gimbal_mode_set->gimbal_behaviour = GIMBAL_GYRO_ANGLE;
			
				//		//�Ҽ����½�������
					if(Mouse_Data.mouse_r == 1)
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
	
		}
		else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
		}
}


