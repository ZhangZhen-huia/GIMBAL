#include "gimbal_behaviour.h"
#include "referee.h"
#include "detect_task.h"
#include "key_task.h"
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_encode_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);


/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
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
//  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
//  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
//  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
//  * @param[in]      gimbal_control_set:云台数据指针
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
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
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
	
		/*-- 一键掉头 --*/
		if(Key_ScanValue.Key_Value.r)
		{
			 *yaw = 180;
		}

}

/*-- 调试用 --*/
//fp32 aim_yaw_set;
//fp32 aim_pitch_set;
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
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
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

//		 /*-- 根据不同模式进行pitch和yaw增量获取 --*/
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
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //云台行为状态机设置（6种模式，但是只用的上陀螺仪绝对角度控制和编码器相对角度控制）
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
//    //根据云台行为状态机设置电机状态机
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
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
		
    if (gimbal_mode_set == NULL)
    {
        return;
    }

		//中间是编码值控制（yaw陀螺仪，pitch编码值）
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
    }
		
		//遥控器丢失，但是图传连接了
		if(toe_is_error(DBUS_TOE) && !toe_is_error(REFEREE_TOE))
		{
					//右键按下进入自瞄，并且瞄到了，就进入自瞄模式
					if(Mouse_Data.mouse_r == 1 && !toe_is_error(AIMBOT_TOE))
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					//不自喵
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
		}
		//遥控器没有丢失，右边再最上面
		else if ( !toe_is_error(DBUS_TOE) && switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{			
					//右键按下进入自瞄，并且瞄到了，就进入自瞄模式
					if(Mouse_Data.mouse_r == 1 && !toe_is_error(AIMBOT_TOE))
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
					}
					//没有瞄到就不自瞄
					else
					{
						gimbal_mode_set->gimbal_behaviour = GIMBAL_ENCODE_ANGLE;
					}	
	
		}
		//遥控器直接右边拨到最下面，强制自瞄，不管是否收到了视觉消息
		else if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
		{
				gimbal_mode_set->gimbal_behaviour = GIMBAL_AUTO_ANGLE;
		}
}


