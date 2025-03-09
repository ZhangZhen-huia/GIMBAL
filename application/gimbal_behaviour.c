#include "gimbal_behaviour.h"
#include "referee.h"
#include "detect_task.h"
#include "key_task.h"
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);


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



///**
//  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
//  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
//  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
//  * @param[in]      gimbal_control_set:��̨����ָ��
//  * @retval         none
//  */
//static void gimbal_gyro_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
//{
//    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }

//    static int16_t yaw_channel = 0, pitch_channel = 0;

//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);


//    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
//    *pitch = -(pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN);
//}


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

//		 /*-- ���ݲ�ͬģʽ����pitch��yaw������ȡ --*/
//    if (gimbal_control_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE )
//    {
//        gimbal_gyro_angle_control(add_yaw, add_pitch, gimbal_control_set);
//    }
    if (gimbal_control_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
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
//    //������̨��Ϊ״̬�����õ��״̬��
//    if (gimbal_mode_set->gimbal_behaviour == GIMBAL_GYRO_ANGLE)
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
//        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
//    }
    if (gimbal_mode_set->gimbal_behaviour == GIMBAL_ENCODE_ANGLE)
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

		//�м��Ǳ���ֵ���ƣ�yaw�����ǣ�pitch����ֵ��
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
    }
		
		//ң������ʧ������ͼ��������
		if(toe_is_error(DBUS_TOE) && !toe_is_error(REFEREE_TOE))
		{
					//�Ҽ����½������飬�����鵽�ˣ��ͽ�������ģʽ
					if(Mouse_Data.mouse_r == 1 && !toe_is_error(AIMBOT_TOE))
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					//������
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
		}
		//ң����û�ж�ʧ���ұ���������
		else if ( !toe_is_error(DBUS_TOE) && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{			
					//�Ҽ����½������飬�����鵽�ˣ��ͽ�������ģʽ
					if(Mouse_Data.mouse_r == 1 && !toe_is_error(AIMBOT_TOE))
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					//û���鵽�Ͳ�����
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
	
		}
		//ң����ֱ���ұ߲��������棬ǿ�����飬�����Ƿ��յ����Ӿ���Ϣ
		else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
		}
}


